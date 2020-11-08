#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

extern crate embedded_hal as hal;

//#[macro_use]
extern crate cortex_m_rt;

extern crate nb;

extern crate cortex_m_semihosting;
extern crate ds323x;
extern crate embedded_graphics;
extern crate panic_semihosting;
extern crate smart_leds;
extern crate tm4c123x_hal;
extern crate tm4c_hal;
extern crate ws2812_spi;

use cortex_m_semihosting::{debug, hprintln};
use panic_semihosting as _;

use tm4c_hal::delay::Delay;

use embedded_graphics::{
    fonts::{Font24x32, Text},
    pixelcolor::BinaryColor::Off as White,
    pixelcolor::BinaryColor::On as Black,
    prelude::*,
    primitive_style,
    primitives::Rectangle,
    text_style,
};

use epd_waveshare::{
    epd2in13_v2::{Display2in13, EPD2in13},
    graphics::{Display, DisplayRotation},
    prelude::*,
};

use cortex_m::interrupt::{free, Mutex};
use cortex_m_rt::{entry, exception};
use cortex_m_semihosting::hio;

use ds323x::{Alarm2Matching, DayAlarm2, Ds323x, Hours, NaiveTime, Rtcc, Timelike};

use core::{cell::RefCell, fmt::Write, ops::DerefMut};

use smart_leds::{colors, SmartLedsWrite, RGB8};

use ws2812_spi as ws2812;

macro_rules! debug_only {
    ($statement:stmt) => {
        if cfg!(debug_assertions) {
            $statement
        }
    };
    ($code:block) => {
        if cfg!(debug_assertions) {
            $code
        }
    };
}

// cheap/dumb way to convert time fields to strings.
const NUM_CONV: [&str; 60] = [
    "00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12", "13", "14", "15",
    "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31",
    "32", "33", "34", "35", "36", "37", "38", "39", "40", "41", "42", "43", "44", "45", "46", "47",
    "48", "49", "50", "51", "52", "53", "54", "55", "56", "57", "58", "59",
];

use tm4c123x_hal::{
    gpio::gpioa::{PA2, PA4, PA5},
    gpio::gpiob::{PB0, PB2, PB3},
    gpio::gpioc::PC6,
    gpio::gpiod::{PD1, PD6},
    gpio::gpioe::PE4,
    gpio::{
        AlternateFunction, Floating, GpioExt, Input, InterruptMode, OpenDrain, Output, PullDown,
        PullUp, PushPull, AF2, AF3,
    },
    i2c::I2c,
    spi::{Spi, MODE_0},
    sysctl::{self, SysctlExt},
    time::U32Ext,
    tm4c123x::{I2C0, SSI0},
};

type I2cT = I2c<
    I2C0,
    (
        PB2<AlternateFunction<AF3, PushPull>>,
        PB3<AlternateFunction<AF3, OpenDrain<Floating>>>,
    ),
>;

type RtcT = Ds323x<ds323x::interface::I2cInterface<I2cT>, ds323x::ic::DS3231>;

type SpiT = Spi<
    SSI0,
    (
        PA2<AlternateFunction<AF2, PushPull>>,
        PA4<AlternateFunction<AF2, PushPull>>,
        PA5<AlternateFunction<AF2, PullDown>>,
    ),
>;

type EpdT = EPD2in13<
    SpiT,
    PC6<Output<PushPull>>,
    PB0<Input<Floating>>,
    PE4<Output<PushPull>>,
    PD1<Output<PushPull>>,
>;

#[rtic::app(device = tm4c123x, peripherals = true)]
const APP: () = {
    struct Resources {
        // The RTC periph
        rtc: RtcT,

        // e-ink
        epd: EpdT,
        display: Display2in13,
        spi: SpiT,

        rtc_int_pin: PD6<Input<PullUp>>,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        // Alias peripherals
        let p: tm4c123x_hal::Peripherals = cx.device;

        //        let p = tm4c123x_hal::Peripherals::take().unwrap();
        let mut sc = p.SYSCTL.constrain();

        sc.clock_setup.oscillator = sysctl::Oscillator::Main(
            sysctl::CrystalFrequency::_16mhz,
            sysctl::SystemClock::UsePll(sysctl::PllOutputFrequency::_80_00mhz),
        );
        let clocks = sc.clock_setup.freeze();

        let mut porta = p.GPIO_PORTA.split(&sc.power_control);
        let mut portb = p.GPIO_PORTB.split(&sc.power_control);
        let portc = p.GPIO_PORTC.split(&sc.power_control);
        let portd = p.GPIO_PORTD.split(&sc.power_control);
        let porte = p.GPIO_PORTE.split(&sc.power_control);

        let i2c_dev = I2c::i2c0(
            p.I2C0,
            (
                portb.pb2.into_af_push_pull::<AF3>(&mut portb.control), // SCL
                portb
                    .pb3
                    .into_af_open_drain::<AF3, Floating>(&mut portb.control),
            ), // SDA
            100.khz(),
            &clocks,
            &sc.power_control,
        );
        let mut rtc = Ds323x::new_ds3231(i2c_dev);

        let mut spi0 = Spi::spi0(
            p.SSI0,
            (
                porta.pa2.into_af_push_pull::<AF2>(&mut porta.control), // SCK
                porta.pa4.into_af_push_pull::<AF2>(&mut porta.control), // Miso
                porta.pa5.into_af_pull_down::<AF2>(&mut porta.control),
            ), // Mosi
            MODE_0,
            2_000_000.hz(),
            &clocks,
            &sc.power_control,
        );
        let cs_pin = portc.pc6.into_push_pull_output(); // cs
        let rst_pin = portd.pd1.into_push_pull_output();
        let dc_pin = porte.pe4.into_push_pull_output(); // dc
        let busy_pin = portb.pb0.into_floating_input();

        // Cortex-M peripherals
        let cp: cortex_m::Peripherals = cx.core;

        let mut delay = Delay::new(cp.SYST, &clocks);

        let mut epd =
            EPD2in13::new(&mut spi0, cs_pin, busy_pin, dc_pin, rst_pin, &mut delay).unwrap();

        let mut display = Display2in13::default();
        display.set_rotation(DisplayRotation::Rotate90);

        display.clear(White).unwrap();
        epd.update_and_display_frame(&mut spi0, &display.buffer())
            .unwrap();

        // GPIO for interrupt
        // SQW/INT pin wired to PD6
        let mut rtc_int_pin = portd.pd6.into_pull_up_input();
        rtc_int_pin.set_interrupt_mode(InterruptMode::EdgeFalling);
        rtc_int_pin.clear_interrupt();

        rtc.clear_alarm1_matched_flag().unwrap();
        rtc.clear_alarm2_matched_flag().unwrap();

        rtc.use_int_sqw_output_as_interrupt().unwrap();

        // alarm1 used to track events
        // alarm2 used to refresh clock display every minute.
        rtc.enable_alarm1_interrupts().unwrap();
        rtc.enable_alarm2_interrupts().unwrap();

        rtc.set_alarm2_day(
            DayAlarm2 {
                day: 1,
                hour: Hours::H24(1),
                minute: 0,
            },
            Alarm2Matching::OncePerMinute,
        )
        .unwrap();

        hprintln!("init").unwrap();

        init::LateResources {
            rtc,
            epd,
            display,
            spi: spi0,
            rtc_int_pin,
        }
    }

    #[task(priority = 3, resources = [rtc, epd])]
    fn handle_event_alarm(cx: handle_event_alarm::Context, time: NaiveTime) {
        hprintln!("handle event alarm! {} {}", time.hour(), time.minute()).unwrap();
    }

    #[task(priority = 3, resources = [rtc, epd, display, spi])]
    fn display_time(mut cx: display_time::Context, time: NaiveTime) {
        draw_hour(&mut cx.resources.display, &time);

        cx.resources
            .epd
            .update_and_display_frame(&mut cx.resources.spi, &cx.resources.display.buffer())
            .unwrap();
        cx.resources.epd.sleep(&mut cx.resources.spi).unwrap();

        hprintln!("minute is changing {} {}", time.hour(), time.minute()).unwrap();
    }

    #[task(binds = GPIOD, resources = [rtc, rtc_int_pin], spawn = [display_time, handle_event_alarm] )]
    fn gpiod(mut cx: gpiod::Context) {
        let mut a1 = false;
        let mut a2 = false;

        let mut time: Option<NaiveTime> = None;

        cx.resources.rtc.lock(|rtc| {
            time = Some(rtc.get_time().unwrap());

            if rtc.has_alarm1_matched().unwrap() {
                rtc.clear_alarm1_matched_flag().unwrap();
                a1 = true;
            }
            if rtc.has_alarm2_matched().unwrap() {
                rtc.clear_alarm2_matched_flag().unwrap();
                a2 = true;
            }
        });

        if a1 {
            cx.spawn.handle_event_alarm(time.unwrap()).unwrap();
        }
        if a2 {
            cx.spawn.display_time(time.unwrap()).unwrap();
        }

        cx.resources.rtc_int_pin.clear_interrupt();

        hprintln!("hep").unwrap();
    }

    // Use GPIOA as dispatch interrupt.
    extern "C" {
        fn GPIOA();
    }
};

const FONT: embedded_graphics::fonts::Font24x32 = Font24x32;
const FONT_SZ: Size = embedded_graphics::fonts::Font24x32::CHARACTER_SIZE;

fn draw_text(display: &mut Display2in13, text: &str, line: u8, x: u8) {
    let y = line as i32 * FONT_SZ.height as i32;
    let x = x as i32 * FONT_SZ.width as i32;

    let _ = Text::new(text, Point::new(x as i32, y))
        .into_styled(text_style!(
            font = FONT,
            text_color = Black,
            background_color = White
        ))
        .draw(display);
}

fn draw_hour(display: &mut Display2in13, time: &NaiveTime) {
    draw_text(display, NUM_CONV[time.hour() as usize], 0, 0);
    draw_text(display, NUM_CONV[time.minute() as usize], 0, 3);
}
