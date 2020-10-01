#![no_main]
#![no_std]

extern crate embedded_hal as hal;

//#[macro_use]
extern crate cortex_m_rt;

#[macro_use(block)]
extern crate nb;

extern crate cortex_m_semihosting;
extern crate ds323x;
extern crate panic_semihosting;
extern crate smart_leds;
extern crate tm4c123x_hal;
extern crate ws2812_spi;
extern crate embedded_graphics;
extern crate tm4c_hal;

const num_conv : [&str; 60] = [
    "0",
    "1",
    "2",
    "3",
    "4",
    "5",
    "6",
    "7",
    "8",
    "9",
    "10",
    "11",
    "12",
    "13",
    "14",
    "15",
    "16",
    "17",
    "18",
    "19",
    "20",
    "21",
    "22",
    "23",
    "24",
    "25",
    "26",
    "27",
    "28",
    "29",
    "30",
    "31",
    "32",
    "33",
    "34",
    "35",
    "36",
    "37",
    "38",
    "39",
    "40",
    "41",
    "42",
    "43",
    "44",
    "45",
    "46",
    "47",
    "48",
    "49",
    "50",
    "51",
    "52",
    "53",
    "54",
    "55",
    "56",
    "57",
    "58",
    "59",
];

use tm4c_hal::delay::Delay;

use embedded_graphics::{
    fonts::{Font24x32, Font12x16, Font6x8, Text},
    pixelcolor::BinaryColor,
    pixelcolor::BinaryColor::On as Black,
    pixelcolor::BinaryColor::Off as White,
    prelude::*,
    primitives::{Circle, Line, Rectangle, Triangle},
    style::{PrimitiveStyle, TextStyle},
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

use ds323x::{Ds323x, NaiveTime, Timelike, Rtcc, DayAlarm2, Alarm2Matching, Hours};
use tm4c123x_hal::spi::MODE_0;

use tm4c123x_hal::{
    delay,
    gpio::{gpiod::PD6, Floating, GpioExt, Input, InterruptMode, PullUp, AF2, AF3},
    i2c::I2c,
    interrupt,
    spi::Spi,
    sysctl::{self, SysctlExt},
    time::U32Ext,
    timer::Timer,
    tm4c123x,
};

use core::{cell::RefCell, fmt::Write, ops::DerefMut};

use hal::prelude::*;
use hal::spi::{FullDuplex, Phase, Polarity};

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

#[derive(PartialEq)]
struct TimeRange<'a> {
    start: NaiveTime,
    end: NaiveTime,

    next: Option<&'a TimeRange<'a>>,
    in_color: RGB8,
}

enum FSMState<'a> {
    Idle,
    InRange { range: &'a TimeRange<'a> },
    WaitNextRange { range: &'a TimeRange<'a> },
}

// PIN used for RTC signaling. Shared between main app and IRQ handler
static GPIO_PD6: Mutex<RefCell<Option<PD6<Input<PullUp>>>>> = Mutex::new(RefCell::new(None));

fn draw_text(display: &mut Display2in13, text: &str, line: u8, x: u8) {
    let y = line as i32 * 32;
    let x = x as i32 * 24;

    let _ = Text::new(text, Point::new(x as i32, y))
        .into_styled(text_style!(
            font = Font24x32,
            text_color = Black,
            background_color = White
        ))
        .draw(display);
}

fn draw_hour(display: &mut Display2in13, time: &NaiveTime) {
    draw_text(display, num_conv[time.hour() as usize], 0, 0);
    draw_text(display, num_conv[time.minute() as usize], 0, 3);
}

#[entry]
fn main() -> ! {
    let mut stdout = hio::hstdout().unwrap();
    debug_only! {writeln!(stdout, "Hello, world!").unwrap()}

    let p = tm4c123x_hal::Peripherals::take().unwrap();
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

    writeln!(stdout, "conf spi").unwrap();

    let spi = Spi::spi1(
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
    writeln!(stdout, "SPI configured").unwrap();

    writeln!(stdout, "SPI for EPD setup").unwrap();
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
    writeln!(stdout, "SPI for EPD setup done").unwrap();

    let cs_pin = portc.pc6.into_push_pull_output(); // cs
    let rst_pin = portd.pd1.into_push_pull_output();
    let dc_pin = porte.pe4.into_push_pull_output(); // dc
    let busy_pin = portb.pb0.into_floating_input();

    // Setup EPD
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut delay = Delay::new(cp.SYST, &clocks);

    writeln!(stdout, "EPD setup").unwrap();
    let mut epd = EPD2in13::new(&mut spi0, cs_pin, busy_pin, dc_pin, rst_pin, &mut delay).unwrap();
    writeln!(stdout, "EPD setup done").unwrap();

    let text_style = TextStyle::new(Font24x32, BinaryColor::On);
    let mut display = Display2in13::default();
    display.set_rotation(DisplayRotation::Rotate90);

    display.clear(White).unwrap();
    epd.update_and_display_frame(&mut spi0, &display.buffer()).unwrap();

    writeln!(stdout, "EPD cleared").unwrap();

    writeln!(stdout, "conf i2c").unwrap();
    let i2c_dev = I2c::i2c0(
        p.I2C0,
        (
            portb.pb2.into_af_push_pull::<AF3>(&mut portb.control), // SCL
            portb.pb3
                .into_af_open_drain::<AF3, Floating>(&mut portb.control),
        ), // SDA
        100.khz(),
        &clocks,
        &sc.power_control,
    );
    writeln!(stdout, "I2C configured").unwrap();

    writeln!(stdout, "conf ds3231").unwrap();
    let mut rtc = Ds323x::new_ds3231(i2c_dev);

    // we'll use interrupt
    rtc.clear_alarm1_matched_flag().unwrap();
    rtc.clear_alarm2_matched_flag().unwrap();

    rtc.use_int_sqw_output_as_interrupt().unwrap();
    rtc.enable_alarm1_interrupts().unwrap();
    rtc.enable_alarm2_interrupts().unwrap();

    // GPIO for interrupt
    // SQW/INT pin wired to PD6
    let mut pd6 = portd.pd6.into_pull_up_input();
    pd6.set_interrupt_mode(InterruptMode::EdgeFalling);

    free(|cs| {
        GPIO_PD6.borrow(cs).replace(Some(pd6));
    });

    let time = NaiveTime::from_hms(07, 00, 45);
    rtc.set_time(&time).unwrap();

    let time = rtc.get_time().unwrap();
    writeln!(stdout, "Time: {} ", time).unwrap();

    let text_time = "00:00";
    let text_time_width = text_time.len() as i32 * 6;

    draw_text(&mut display, "00:00", 0, 0);
    draw_text(&mut display, "XXXX", 1, 0);

    epd.update_and_display_frame(&mut spi0, &display.buffer()).unwrap();

    let time = rtc.get_time().unwrap();

    writeln!(stdout, "Time: {} ", time).unwrap();
    writeln!(stdout, "ds3231 configured").unwrap();

    // writeln!(stdout, "Creating Timer").unwrap();
    // let mut timer0 = Timer::wtimer0(p.WTIMER0, 1.hz(), &sc.power_control, &clocks);
    // writeln!(stdout, "Timer created").unwrap();
    // writeln!(stdout, "Will block until timeout").unwrap();
    // let _ret = block!(timer0.wait());
    // writeln!(stdout, "Timeout").unwrap();

    // ws2812
    let mut ws = ws2812::Ws2812::new(spi);

    let mut data = [colors::GREEN];

    // let m0 = TimeRange {
    //     start: NaiveTime::from_hms(00, 01, 00),
    //     end: NaiveTime::from_hms(00, 05, 00),
    //     next: None,
    //     in_color: colors::RED,
    // };

    // let m1 = TimeRange {
    //     start: NaiveTime::from_hms(00, 06, 00),
    //     end: NaiveTime::from_hms(00, 07, 00),
    //     next: Some(&m0),
    //     in_color: colors::PINK,
    // };

    // let first = TimeRange {
    //     start: NaiveTime::from_hms(00, 10, 00),
    //     end: NaiveTime::from_hms(00, 01, 00),
    //     next: Some(&m1),
    //     in_color: colors::BLUE,
    // };

    let m0 = TimeRange {
        start: NaiveTime::from_hms(07, 09, 05),
        end: NaiveTime::from_hms(07, 10, 00),
        next: None,
        in_color: colors::RED,
    };

    let m1 = TimeRange {
        start: NaiveTime::from_hms(07, 06, 00),
        end: NaiveTime::from_hms(07, 08, 00),
        next: Some(&m0),
        in_color: colors::PINK,
    };

    let first = TimeRange {
        start: NaiveTime::from_hms(07, 01, 00),
        end: NaiveTime::from_hms(07, 02, 00),
        next: Some(&m1),
        in_color: colors::BLUE,
    };


    //
    rtc.set_alarm2_day(
        DayAlarm2 {
            day: 1,
            hour: Hours::H24(2),
            minute: 3,
        },
        Alarm2Matching::OncePerMinute).unwrap();

    let all_ranges = [&first, &m1, &m0];

    let time = rtc.get_time().unwrap();
    let mut state = FSMState::Idle;
    writeln!(stdout, "Will check for init state ?").unwrap();
    let mut found = false;

    for range in all_ranges.iter() {
        if time < range.start {
            state = FSMState::WaitNextRange { range };
            writeln!(
                stdout,
                "Waiting for next range to activate: {}:{}",
                range.start, range.end
            )
            .unwrap();
            rtc.set_alarm1_hms(range.start).unwrap();
            found = true;
            break;
        }

        if time >= range.start && time < range.end {
            state = FSMState::InRange { range };
            writeln!(stdout, "In range: {}:{}", range.start, range.end).unwrap();
            rtc.set_alarm1_hms(range.end).unwrap();
            found = true;
            break;
        }
    }

    if !found {
        let range = all_ranges[all_ranges.len() - 1];

        writeln!(
            stdout,
            "Next alarm is tomorrow: {}:{}",
            range.start, range.end
        )
            .unwrap();
        rtc.set_alarm1_hms(range.start).unwrap();
    }

    let init_data = [colors::GREEN, colors::BLUE, colors::AQUAMARINE];
    ws.write(init_data.iter().cloned()).unwrap();

    // unmask GPIOD Interrupt
    tm4c123x::NVIC::unpend(tm4c123x_hal::tm4c123x::Interrupt::GPIOD);
    unsafe {
        tm4c123x::NVIC::unmask(tm4c123x_hal::tm4c123x::Interrupt::GPIOD);
    };

    loop {
        if !rtc.has_alarm1_matched().unwrap() || !rtc.has_alarm2_matched().unwrap() {
            // check if an alarm has been triggered. If not, then make the core
            // go to sleep until the next IRQ is raised.

            cortex_m::asm::wfi();
            continue;
        }
        let time = rtc.get_time().unwrap();

        // refresh the displayed time
        if rtc.has_alarm2_matched().unwrap() {
            writeln!(stdout, "minute is changing {} {}", time.hour(), time.minute()).unwrap();
            rtc.clear_alarm2_matched_flag().unwrap();

            draw_hour(&mut display, &time);
        }

        // something happenned
        if rtc.has_alarm1_matched().unwrap() {
            rtc.clear_alarm1_matched_flag().unwrap();

            writeln!(stdout, "Something is happening").unwrap();

            match state {
                FSMState::Idle => (),
                FSMState::WaitNextRange { range } => {
                    writeln!(stdout, "Enter").unwrap();
                    state = FSMState::InRange { range };
                    data = [range.in_color; 1];
                    writeln!(stdout, "Time: {}, Alarm :{}", time, range.end).unwrap();
                    rtc.set_alarm1_hms(range.end).unwrap();
                }

                FSMState::InRange { range } => {
                    writeln!(stdout, "Exit").unwrap();
                    data = [colors::GREEN; 1];
                    if let Some(x) = range.next {
                        state = FSMState::WaitNextRange { range: x };
                        rtc.set_alarm1_hms(x.start).unwrap();
                    } else {
                        state = FSMState::WaitNextRange { range: &first };
                        rtc.set_alarm1_hms(first.start).unwrap();
                    }
                }
            }

            ws.write(data.iter().cloned()).unwrap();
        }
        epd.update_and_display_frame(&mut spi0, &display.buffer()).unwrap();
    }
}

#[exception]
/// The hard fault handler
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[interrupt]
/// The GPIOD handler (in our case, PD6 from RTC on falling edge)
fn GPIOD() {
    free(|cs| {
        let mut pd6_ref = GPIO_PD6.borrow(cs).borrow_mut();
        if let Some(ref mut pd6) = pd6_ref.deref_mut() {
            pd6.clear_interrupt()
        }
    });
}

#[exception]
/// The default exception handler
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
