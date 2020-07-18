#![no_main]
#![no_std]

extern crate embedded_hal as hal;

//#[macro_use]
extern crate cortex_m_rt;

#[macro_use(block)]
extern crate nb;

use cortex_m_rt::{entry, exception};

extern crate cortex_m_semihosting as sh;
extern crate panic_semihosting;

extern crate tm4c123x_hal;

extern crate ds323x;
use ds323x::{Ds323x, NaiveTime, Rtcc};

use tm4c123x_hal::gpio::GpioExt;
use tm4c123x_hal::gpio::{InterruptMode, Floating, AF2, AF3};
use tm4c123x_hal::i2c::I2c;
use tm4c123x_hal::sysctl::{self, SysctlExt};
use tm4c123x_hal::time::U32Ext;
use tm4c123x_hal::timer::*;

use core::fmt::Write;

use hal::prelude::*;

extern crate smart_leds;
use smart_leds::RGB8;
extern crate ws2812_spi;

use ws2812_spi as ws2812;

//use crate::ws2812::Ws2812;
//use crate::ws2812::prerendered::Timing;

use smart_leds::{colors, SmartLedsWrite};

pub use crate::hal::spi::{MODE_0, MODE_1, MODE_2, MODE_3};
use tm4c123x_hal::spi::Spi;

use sh::hio;

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

#[entry]
fn main() -> ! {
    let mut stdout = hio::hstdout().unwrap();
    writeln!(stdout, "Hello, world!").unwrap();

    let p = tm4c123x_hal::Peripherals::take().unwrap();
    let mut sc = p.SYSCTL.constrain();

    sc.clock_setup.oscillator = sysctl::Oscillator::Main(
        sysctl::CrystalFrequency::_16mhz,
        sysctl::SystemClock::UsePll(sysctl::PllOutputFrequency::_80_00mhz),
    );
    let clocks = sc.clock_setup.freeze();

    writeln!(stdout, "conf spi").unwrap();
    let mut portd = p.GPIO_PORTD.split(&sc.power_control);
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

    writeln!(stdout, "conf i2c").unwrap();
    let mut portb = p.GPIO_PORTB.split(&sc.power_control);
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
    writeln!(stdout, "I2C configured").unwrap();

    writeln!(stdout, "conf ds3231").unwrap();
    let mut rtc = Ds323x::new_ds3231(i2c_dev);

    // we'll use interrupt
    rtc.clear_alarm1_matched_flag().unwrap();
    rtc.use_int_sqw_output_as_interrupt().unwrap();
    rtc.enable_alarm1_interrupts().unwrap();

    // GPIO for interrupt
    // SQW pin wired to PD6
    let mut pd6 = portd.pd6.into_pull_up_input();
    pd6.set_interrupt_mode(InterruptMode::EdgeFalling);

    let time = NaiveTime::from_hms(23, 58, 45);
    rtc.set_time(&time).unwrap();

    let time = rtc.get_time().unwrap();
    writeln!(stdout, "Time: {} ", time).unwrap();

    let time = rtc.get_time().unwrap();
    writeln!(stdout, "Time: {} ", time).unwrap();

    writeln!(stdout, "ds3231 configured").unwrap();

    writeln!(stdout, "Creating Timer").unwrap();

    let mut timer0 = Timer::wtimer0(p.WTIMER0, 1.hz(), &sc.power_control, &clocks);

    writeln!(stdout, "Timer created").unwrap();
    writeln!(stdout, "Will block until timeout").unwrap();
    let _ret = block!(timer0.wait());
    writeln!(stdout, "Timeout").unwrap();

    // ws2812
    let mut ws = ws2812::Ws2812::new(spi);

    let mut data = [colors::GREEN];

    let m1 = TimeRange {
        start: NaiveTime::from_hms(23, 59, 20),
        end: NaiveTime::from_hms(00, 00, 5),
        next: None,
        in_color: colors::PINK,
    };

    let first = TimeRange {
        start: NaiveTime::from_hms(23, 58, 55),
        end: NaiveTime::from_hms(23, 59, 10),
        next: Some(&m1),
        in_color: colors::BLUE,
    };

    let all_ranges = [&first, &m1];

    let time = rtc.get_time().unwrap();
    let mut state = FSMState::Idle;

    for range in all_ranges.iter() {
        if time < range.start {
            state = FSMState::WaitNextRange { range };
            writeln!(
                stdout,
                "Waiting for next range to activate: {}:{}",
                range.start, range.end
            ).unwrap();
            rtc.set_alarm1_hms(range.start).unwrap();
            break;
        }

        if time >= range.start && time < range.end {
            state = FSMState::InRange { range };
            writeln!(stdout, "In range: {}:{}", range.start, range.end).unwrap();
            rtc.set_alarm1_hms(range.end).unwrap();
            break;
        }
    }

    loop {
        let time = rtc.get_time().unwrap();
        writeln!(stdout, "Delay").unwrap();

        match state {
            FSMState::Idle => (),
            FSMState::WaitNextRange { range } => {
                if time >= range.start {
                    writeln!(stdout, "Enter").unwrap();
                    state = FSMState::InRange { range };
                    data = [range.in_color; 1];
                }
            }

            FSMState::InRange { range } => {
                if time > range.end && (range.start < range.end || time < range.start) {
                    writeln!(stdout, "Exit").unwrap();
                    data = [colors::GREEN; 1];
                    if let Some(x) = range.next {
                        state = FSMState::WaitNextRange { range: x };
                    } else {
                        state = FSMState::WaitNextRange { range: &first };
                    }
                }
            }
        }

        ws.write(data.iter().cloned()).unwrap();
    }
}

#[exception]
/// The hard fault handler
fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    panic!("HardFault at {:#?}", ef);
}

#[exception]
/// The default exception handler
fn DefaultHandler(irqn: i16) {
    panic!("Unhandled exception (IRQn = {})", irqn);
}
