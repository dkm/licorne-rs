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

#[cfg(feature = "usesemihosting")]
macro_rules! debug_only {
    ($statement:stmt) => {
        $statement
    };
    ($code:block) => {
        $code
    };
}

#[cfg(not(feature = "usesemihosting"))]
macro_rules! debug_only {
    ($statement:stmt) => {};
    ($code:block) => {};
}

const NUM_LEDS: usize = 1;

// cheap/dumb way to convert time fields to strings.
const NUM_CONV: [&str; 60] = [
    "00", "01", "02", "03", "04", "05", "06", "07", "08", "09", "10", "11", "12", "13", "14", "15",
    "16", "17", "18", "19", "20", "21", "22", "23", "24", "25", "26", "27", "28", "29", "30", "31",
    "32", "33", "34", "35", "36", "37", "38", "39", "40", "41", "42", "43", "44", "45", "46", "47",
    "48", "49", "50", "51", "52", "53", "54", "55", "56", "57", "58", "59",
];

// PA2: SSI0
// PA4: SSI0
// PA5: SSI0
//
// PB0: EPD
// PB1: Rotary Encoder (pinA)
// PB2: I2C0
// PB3: I2C0
// PB4: Rotary Encoder (pinB)
//
// PC6: EPD
//
// PD0: SSI1
// PD1: EPD
// PD2: SSI1
// PD3: SSI1
// PD6: RTC / INT
//
// PE4: EPD

use tm4c123x_hal::{
    gpio::gpioa::{PA2, PA4, PA5},
    gpio::gpiob::{PB0, PB1, PB2, PB3, PB4},
    gpio::gpioc::PC6,
    gpio::gpiod::{PD0, PD1, PD2, PD3, PD6},
    gpio::gpioe::PE4,
    gpio::{
        AlternateFunction, Floating, GpioExt, Input, InterruptMode, OpenDrain, Output, PullDown,
        PullUp, PushPull, AF2, AF3,
    },
    i2c::I2c,
    spi::{Spi, MODE_0},
    sysctl::{self, SysctlExt},
    time::U32Ext,
    tm4c123x::{I2C0, SSI0, SSI1},
};

use rotary_encoder_hal::{Direction, Rotary};
// Specialized types for our particular setup.

// i2c for DS3231 RTC
type I2cT = I2c<
    I2C0,
    (
        PB2<AlternateFunction<AF3, PushPull>>,
        PB3<AlternateFunction<AF3, OpenDrain<Floating>>>,
    ),
>;

type RtcT = Ds323x<ds323x::interface::I2cInterface<I2cT>, ds323x::ic::DS3231>;

// spi for WS2812
type Spi0T = Spi<
    SSI0,
    (
        PA2<AlternateFunction<AF2, PushPull>>,
        PA4<AlternateFunction<AF2, PushPull>>,
        PA5<AlternateFunction<AF2, PullDown>>,
    ),
>;

// spi for E-ink screen
type Spi1T = Spi<
    SSI1,
    (
        PD0<AlternateFunction<AF2, PushPull>>,
        PD2<AlternateFunction<AF2, PushPull>>,
        PD3<AlternateFunction<AF2, PullDown>>,
    ),
>;

type EpdT = EPD2in13<
    Spi0T,
    PC6<Output<PushPull>>,
    PB0<Input<Floating>>,
    PE4<Output<PushPull>>,
    PD1<Output<PushPull>>,
>;

type RotaryT = Rotary<PB1<Input<PullUp>>, PB4<Input<PullUp>>>;

#[derive(PartialEq)]
pub struct TimeRange {
    start: NaiveTime,
    end: NaiveTime,

    in_color: RGB8,
    name: &'static str,
}

pub enum FSMState {
    Idle,
    InRange(usize),
    WaitNextRange(usize),
}

#[rtic::app(device = tm4c123x, peripherals = true)]
const APP: () = {
    struct Resources {
        // The RTC periph
        rtc: RtcT,

        // e-ink
        screen: (EpdT, Display2in13, Spi0T),

        rtc_int_pin: PD6<Input<PullUp>>,
        // ws2812
        leds: ws2812::Ws2812<Spi1T>,

        rotary: RotaryT,

        next_or_current_range: usize,
        ranges: [TimeRange; 3],
        state: FSMState,
    }

    #[init(spawn = [display_time])]
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
        let mut portd = p.GPIO_PORTD.split(&sc.power_control);
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

        let spi_ws2812 = Spi::spi1(
            p.SSI1,
            (
                portd.pd0.into_af_push_pull::<AF2>(&mut portd.control), // SCK
                portd.pd2.into_af_push_pull::<AF2>(&mut portd.control), // Miso
                portd.pd3.into_af_pull_down::<AF2>(&mut portd.control),
            ), // Mosi
            ws2812::MODE,
            3_000_000.hz(),
            &clocks,
            &sc.power_control,
        );

        let mut leds = ws2812::Ws2812::new(spi_ws2812);

        let mut rot_pinA = portb.pb1.into_pull_up_input();
        rot_pinA.set_interrupt_mode(InterruptMode::EdgeBoth);
        rot_pinA.clear_interrupt();

        let mut rot_pinB = portb.pb4.into_pull_up_input();
        rot_pinB.set_interrupt_mode(InterruptMode::EdgeBoth);
        rot_pinB.clear_interrupt();

        let rotary = Rotary::new(rot_pinA, rot_pinB);

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

        let time = NaiveTime::from_hms(23, 00, 3);
        //        rtc.set_time(&time).unwrap();

        let time = rtc.get_time().unwrap();
        cx.spawn.display_time(time);

        let ranges = [
            TimeRange {
                start: NaiveTime::from_hms(0, 0, 0),
                end: NaiveTime::from_hms(5, 0, 0),
                in_color: colors::RED,
                name: "0:0->5:0",
            },
            TimeRange {
                start: NaiveTime::from_hms(5, 1, 0),
                end: NaiveTime::from_hms(7, 0, 0),
                in_color: colors::GREEN,
                name: "5:1->7:0",
            },
            TimeRange {
                start: NaiveTime::from_hms(7, 1, 0),
                end: NaiveTime::from_hms(23, 59, 0),
                in_color: colors::BLUE,
                name: "7:1->23:59",
            },
        ];

        let mut state = FSMState::Idle;
        let mut found = false;
        let mut next_or_current_range = 0;

        for (i, range) in ranges.iter().rev().enumerate() {
            if time < range.start {
                state = FSMState::WaitNextRange(i);
                debug_only! {
                    hprintln!(
                        "Waiting for next range to activate: {}:{}",
                        range.start,
                        range.end
                    ).unwrap()
                }

                draw_text(&mut display, "Wait: ", 1, 0);
                draw_text(&mut display, range.name, 1, 6);

                rtc.set_alarm1_hms(range.start).unwrap();
                found = true;
                break;
            }

            if time >= range.start && time < range.end {
                state = FSMState::InRange(i);
                debug_only! {hprintln!("In range: {}:{}", range.start, range.end).unwrap()}

                let data = [range.in_color; NUM_LEDS];
                leds.write(data.iter().cloned()).unwrap();

                draw_text(&mut display, "In: ", 1, 0);

                draw_text(&mut display, range.name, 1, 4);

                rtc.set_alarm1_hms(range.end).unwrap();
                found = true;
                break;
            }
            next_or_current_range += 1;
        }

        if !found {
            next_or_current_range = 0;

            debug_only! {hprintln!(
                "Next alarm is tomorrow: {}:{}",
                ranges[0].start,
                ranges[0].end
            )
                         .unwrap()}
            rtc.set_alarm1_hms(ranges[0].start).unwrap();
        }

        epd.update_and_display_frame(&mut spi0, &display.buffer())
            .unwrap();

        debug_only! {hprintln!("init").unwrap()}

        init::LateResources {
            rtc,
            screen: (epd, display, spi0),
            rtc_int_pin,
            leds,
            ranges,
            next_or_current_range,
            state,
            rotary,
        }
    }

    #[task(priority = 3, resources = [rtc, screen, leds, ranges, state, next_or_current_range])]
    fn handle_event_alarm(mut cx: handle_event_alarm::Context, time: NaiveTime) {
        debug_only! {hprintln!("handle event alarm! {} {}", time.hour(), time.minute()).unwrap()}

        clear_line(&mut cx.resources.screen.1, 1);

        if let Some(new_state) = match cx.resources.state {
            FSMState::Idle => None,
            FSMState::WaitNextRange(ref range) => {
                debug_only! {hprintln!("Enter").unwrap()}
                clear_line(&mut cx.resources.screen.1, 1);

                draw_text(&mut cx.resources.screen.1, "In: ", 1, 0);

                draw_text(
                    &mut cx.resources.screen.1,
                    cx.resources.ranges[*range].name,
                    1,
                    4,
                );

                let data = [cx.resources.ranges[*range].in_color; NUM_LEDS];

                cx.resources.leds.write(data.iter().cloned()).unwrap();

                debug_only! {hprintln!("Time: {}, Alarm :{}", time, cx.resources.ranges[*range].end).unwrap()}
                cx.resources
                    .rtc
                    .set_alarm1_hms(cx.resources.ranges[*range].end)
                    .unwrap();

                Some(FSMState::InRange(*range))
            }

            FSMState::InRange(ref range) => {
                debug_only! {hprintln!("Exit").unwrap()}

                let data = [colors::GREEN; NUM_LEDS];

                cx.resources.leds.write(data.iter().cloned()).unwrap();
                draw_text(&mut cx.resources.screen.1, "Wait: ", 1, 0);

                if *range == cx.resources.ranges.len() - 1 {
                    draw_text(
                        &mut cx.resources.screen.1,
                        cx.resources.ranges[0].name,
                        1,
                        6,
                    );

                    cx.resources
                        .rtc
                        .set_alarm1_hms(cx.resources.ranges[0].start)
                        .unwrap();
                    Some(FSMState::WaitNextRange(0))
                } else {
                    draw_text(
                        &mut cx.resources.screen.1,
                        cx.resources.ranges[*range + 1].name,
                        1,
                        6,
                    );

                    cx.resources
                        .rtc
                        .set_alarm1_hms(cx.resources.ranges[*range + 1].start)
                        .unwrap();
                    Some(FSMState::WaitNextRange(*range + 1))
                }
            }
        } {
            *cx.resources.state = new_state;
        }

        // refresh display and make it go ZZZzzzZZzzz
        // cx.resources
        //     .epd
        //     .update_and_display_frame(&mut cx.resources.spi, &cx.resources.display.buffer())
        //     .unwrap();
        // cx.resources.epd.sleep(&mut cx.resources.spi).unwrap();
    }

    #[task(priority = 1, resources = [leds, rtc, screen])]
    fn display_time(mut cx: display_time::Context, time: NaiveTime) {
        cx.resources.screen.lock(|screen| {
            draw_hour(&mut screen.1, &time);
            screen
                .0
                .update_and_display_frame(&mut screen.2, &screen.1.buffer())
                .unwrap();
            screen.0.sleep(&mut screen.2).unwrap();
        });

        debug_only! {hprintln!("refresh time with: {} {}", time.hour(), time.minute()).unwrap()}
    }

    #[task(priority = 1)]
    fn knob_turned(cx: knob_turned::Context, dir: Direction) {
        match dir {
            Direction::Clockwise => {
                debug_only! {hprintln!("clockwise").unwrap()}
            }
            Direction::CounterClockwise => {
                debug_only! {hprintln!("counter clockwise").unwrap()}
            }
            _ => (),
        }
    }

    #[task(binds = GPIOB, resources = [rotary], spawn = [ knob_turned ])]
    fn gpiob_rotary(cx: gpiob_rotary::Context) {
        cx.resources.rotary.pin_a().clear_interrupt();
        cx.resources.rotary.pin_b().clear_interrupt();

        match cx.resources.rotary.update().unwrap() {
            Direction::Clockwise => {
                cx.spawn.knob_turned(Direction::Clockwise);
            }
            Direction::CounterClockwise => {
                cx.spawn.knob_turned(Direction::CounterClockwise);
            }
            Direction::None => {}
        }
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

        debug_only! {hprintln!("hep").unwrap()}
    }

    // Use some UARTs as dispatch interrupt.
    extern "C" {
        fn UART3();
    }
    extern "C" {
        fn UART4();
    }
};

const FONT: embedded_graphics::fonts::Font24x32 = Font24x32;
const FONT_SZ: Size = embedded_graphics::fonts::Font24x32::CHARACTER_SIZE;

fn fill_line(
    display: &mut Display2in13,
    line: u8,
    color: embedded_graphics::pixelcolor::BinaryColor,
) {
    let style = primitive_style!(fill_color = color);
    let line = line as i32;

    let h = FONT_SZ.height as i32;

    Rectangle::new(
        Point::new(0, line * h),
        Point::new(display.size().width as i32, (line + 1) * h),
    )
    .into_styled(style)
    .draw(display)
    .unwrap();
}

fn clear_line(display: &mut Display2in13, line: u8) {
    fill_line(display, line, White);
}

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
    draw_text(display, ":", 0, 2);
    draw_text(display, NUM_CONV[time.minute() as usize], 0, 3);
}
