#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

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

use heapless::{consts::U5, String};

use ufmt::uwrite;

use rtic::cyccnt::U32Ext as _;

use bme680::{Bme680, I2CAddress, IIRFilterSize, OversamplingSetting, PowerMode, SettingsBuilder};

use core::time::Duration;

use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    digital::v2::InputPin,
};

use cortex_m_semihosting::hprintln;

use debouncr::{debounce_stateful_12, DebouncerStateful, Edge, Repeat12};

use embedded_graphics::{
    fonts::Text, pixelcolor::BinaryColor::Off as White, pixelcolor::BinaryColor::On as Black,
    prelude::*, text_style,
};

use cortex_m::peripheral::DWT;
use epd_waveshare::{
    epd2in13_v2::{Display2in13, EPD2in13},
    graphics::{Display, DisplayRotation},
    prelude::*,
};

use ds323x::{Alarm2Matching, DayAlarm2, Ds323x, Hours, NaiveTime, Rtcc, Timelike};

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

const NUM_LEDS: usize = 8;

const FONT_TIME: embedded_junkyardfont::FontJunkyard = embedded_junkyardfont::FontJunkyard;
//const FONT_TIME: embedded_graphics::fonts::Font24x32 = Font24x32;
const FONT_RIGHT_BAR: embedded_graphics::fonts::Font12x16 = embedded_graphics::fonts::Font12x16;

const SCR_HOUR_X_OFF: i32 = 1;
const SCR_HOUR_Y_OFF: i32 = 20;

const SCR_RIGHT_BAR_TEMP_X_OFF: i32 = 200;
const SCR_RIGHT_BAR_TEMP_Y_OFF: i32 = 5;

const SCR_RIGHT_BAR_HUMID_X_OFF: i32 = 200;
const SCR_RIGHT_BAR_HUMID_Y_OFF: i32 = 18 + SCR_RIGHT_BAR_TEMP_Y_OFF;

const SCR_RIGHT_BAR_ERRCNT_X_OFF: i32 = 200;
const SCR_RIGHT_BAR_ERRCNT_Y_OFF: i32 = 18 + SCR_RIGHT_BAR_HUMID_Y_OFF;

const FONT_CONFIG: embedded_graphics::fonts::Font12x16 = embedded_graphics::fonts::Font12x16;
const SCR_CONFIG_X_OFF: i32 = 200;
const SCR_CONFIG_Y_OFF: i32 = 100;

pub struct SimpleAsmDelay {
    freq: u32,
}

impl DelayUs<u32> for SimpleAsmDelay {
    fn delay_us(&mut self, us: u32) {
        cortex_m::asm::delay(self.freq * us / 1_000_000)
    }
}
impl DelayMs<u16> for SimpleAsmDelay {
    fn delay_ms(&mut self, ms: u16) {
        self.delay_ms(cast::u32(ms));
    }
}
impl DelayMs<u8> for SimpleAsmDelay {
    fn delay_ms(&mut self, ms: u8) {
        self.delay_ms(cast::u32(ms));
    }
}

impl DelayMs<u32> for SimpleAsmDelay {
    fn delay_ms(&mut self, ms: u32) {
        self.delay_us(ms * 1_000);
    }
}
impl DelayUs<u16> for SimpleAsmDelay {
    fn delay_us(&mut self, us: u16) {
        self.delay_us(cast::u32(us))
    }
}

impl DelayUs<u8> for SimpleAsmDelay {
    fn delay_us(&mut self, us: u8) {
        self.delay_us(cast::u32(us))
    }
}
// PA2: SSI0
// PA4: SSI0
// PA5: SSI0
// PA6: I2C1 (scl)
// PA7: I2C1 (sca)
//
// PB0: EPD
// PB1: Rotary Encoder (pinA)
// PB2: I2C0 (SCL)
// PB3: I2C0 (SDA)
// PB4: Rotary Encoder (pinB)
// PB5: Toggle Switch (config mode)
// PB6:
// PB7: Rotary switch
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
    gpio::gpioa::{PA2, PA4, PA5, PA6, PA7},
    gpio::gpiob::{PB0, PB1, PB2, PB3, PB4, PB5, PB7},
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
    tm4c123x::{I2C0, I2C1, SSI0, SSI1},
};

use rotary_encoder_hal::{Direction, Rotary};
// Specialized types for our particular setup.

// i2c for DS3231 RTC
type I2c0T = I2c<
    I2C0,
    (
        PB2<AlternateFunction<AF3, PushPull>>,
        PB3<AlternateFunction<AF3, OpenDrain<Floating>>>,
    ),
>;

type RtcT = Ds323x<ds323x::interface::I2cInterface<I2c0T>, ds323x::ic::DS3231>;

// i2c for BME680 RTC
type I2c1T = I2c<
    I2C1,
    (
        PA6<AlternateFunction<AF3, PushPull>>,
        PA7<AlternateFunction<AF3, OpenDrain<Floating>>>,
    ),
>;
type Bme680T = Bme680<I2c1T, SimpleAsmDelay>;

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

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SubConfig {
    Hour,
    Minute,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum OperatingMode {
    Normal,
    Configuration(SubConfig),
}

pub struct Screen {
    epd: EpdT,
    display: Display2in13,
    spi: Spi0T,
    delay: SimpleAsmDelay,
}

pub struct Switch<P>
where
    P: InputPin,
{
    pin: P,
    debouncer: DebouncerStateful<u16, Repeat12>,
    sample_count: u8,
}

impl<P> Switch<P>
where
    P: InputPin,
{
    pub fn new(pin: P, state: bool) -> Switch<P> {
        Switch {
            pin,
            debouncer: debounce_stateful_12(state),
            sample_count: 0,
        }
    }
}

type ToggleSwitchPinT = PB5<Input<PullUp>>;
type ToggleSwitchT = Switch<ToggleSwitchPinT>;

type RotarySwitchPinT = PB7<Input<PullUp>>;
type RotarySwitchT = Switch<RotarySwitchPinT>;

// At 80Mhz, this is ~2ms
const POLL_SWITCH_PERIOD: u32 = 80_000;
const DEBOUNCE_SAMPLE_CNT: u8 = 12;

#[rtic::app(device = tm4c123x_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        mode: OperatingMode,

        // The RTC periph
        rtc: RtcT,

        // e-ink
        screen: Screen,

        rtc_int_pin: PD6<Input<PullUp>>,

        // ws2812
        leds: ws2812::Ws2812<Spi1T>,
        leds_data: [RGB8; NUM_LEDS],

        // temp & cov
        bme680: Bme680T,

        toggle_switch: ToggleSwitchT,
        rotary_switch: RotarySwitchT,
        new_time: (u32, u32),

        rotary: RotaryT,

        i2c_error: u32,

        next_or_current_range: usize,
        ranges: [TimeRange; 3],
        state: FSMState,
    }

    #[init(spawn = [refresh_bme680, refresh_time, change_mode, set_leds])]
    fn init(mut cx: init::Context) -> init::LateResources {
        // Initialize (enable) the monotonic timer (CYCCNT)
        cx.core.DCB.enable_trace();
        // required on Cortex-M7 devices that software lock the DWT (e.g. STM32F7)
        DWT::unlock();
        cx.core.DWT.enable_cycle_counter();

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

        let i2c_rtc_dev = I2c::i2c0(
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
        let mut rtc = Ds323x::new_ds3231(i2c_rtc_dev);

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
        // : cortex_m::Peripherals
        let i2c_bme680 = I2c::i2c1(
            p.I2C1,
            (
                porta.pa6.into_af_push_pull::<AF3>(&mut porta.control), // SCL
                porta
                    .pa7
                    .into_af_open_drain::<AF3, Floating>(&mut porta.control),
            ), // SDA
            100.khz(),
            &clocks,
            &sc.power_control,
        );

        let mut bme680 = Bme680::init(
            i2c_bme680,
            SimpleAsmDelay { freq: 80_000_000 },
            I2CAddress::Primary,
        )
        .unwrap();

        let settings = SettingsBuilder::new()
            .with_humidity_oversampling(OversamplingSetting::OS2x)
            .with_pressure_oversampling(OversamplingSetting::OS4x)
            .with_temperature_oversampling(OversamplingSetting::OS8x)
            .with_temperature_filter(IIRFilterSize::Size3)
            .with_gas_measurement(Duration::from_millis(1500), 320, 25)
            .with_run_gas(true)
            .build();
        bme680.set_sensor_settings(settings).unwrap();

        let mut delay = SimpleAsmDelay { freq: 80_000_000 };
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

        let leds = ws2812::Ws2812::new(spi_ws2812);

        let rot_pinA = portb.pb1.into_pull_up_input();
        let rot_pinB = portb.pb4.into_pull_up_input();

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

        let time = rtc.get_time().unwrap();
        cx.spawn.refresh_time(time).unwrap();
        cx.spawn.refresh_bme680().unwrap();

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

                //                draw_text(&mut display, "Wait: ", 1, 0);
                //                draw_text(&mut display, range.name, 1, 6);

                rtc.set_alarm1_hms(range.start).unwrap();
                found = true;
                break;
            }

            if time >= range.start && time < range.end {
                state = FSMState::InRange(i);
                debug_only! {hprintln!("In range: {}:{}", range.start, range.end).unwrap()}

                cx.spawn.set_leds(0, 1, range.in_color).unwrap();

                // draw_text(&mut display, "In: ", 1, 0);
                // draw_text(&mut display, range.name, 1, 4);

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

        let rotary_pin = portb.pb7.into_pull_up_input();
        let init_state = rotary_pin.is_high().unwrap();
        let mut rotary_switch = RotarySwitchT::new(rotary_pin, init_state);
        rotary_switch
            .pin
            .set_interrupt_mode(InterruptMode::EdgeBoth);
        rotary_switch.pin.clear_interrupt();

        // switch is using PUR, is_low() <=> pressed <=> 'true' state
        let mut toggle_switch = ToggleSwitchT::new(portb.pb5.into_pull_up_input(), false);
        toggle_switch
            .pin
            .set_interrupt_mode(InterruptMode::EdgeBoth);
        toggle_switch.pin.clear_interrupt();

        let switch_state = toggle_switch.pin.is_high().unwrap();

        let mode = if switch_state {
            OperatingMode::Configuration(SubConfig::Hour)
        } else {
            OperatingMode::Normal
        };

        cx.spawn.change_mode(mode).unwrap();

        debug_only! {
            hprintln!("init done, mode will be {:?}, switch is {:?} debouncer is {:?}",
                      mode,
                      switch_state,
                      toggle_switch.debouncer.is_high())
                .unwrap()
        }

        init::LateResources {
            mode: OperatingMode::Normal,

            rtc,
            screen: Screen {
                epd,
                display,
                spi: spi0,
                delay,
            },
            rtc_int_pin,
            leds,
            leds_data: [colors::BLACK; NUM_LEDS],

            bme680,

            ranges,
            next_or_current_range,
            state,
            rotary,
            new_time: (0, 0),

            i2c_error: 0,

            toggle_switch,
            rotary_switch,
        }
    }

    #[task(
        resources = [mode, rotary, screen, new_time, rtc],
        spawn = [refresh_epd]
    )]
    fn change_mode(mut ctx: change_mode::Context, mode: OperatingMode) {
        debug_only! {hprintln!("change mode from {:?} to {:?}", *ctx.resources.mode, mode).unwrap()}
        if *ctx.resources.mode == mode {
            return;
        }

        *ctx.resources.mode = mode;

        match *ctx.resources.mode {
            OperatingMode::Normal => {
                ctx.resources.screen.lock(|screen| {
                    screen
                        .epd
                        .set_refresh(&mut screen.spi, &mut screen.delay, RefreshLUT::FULL)
                        .unwrap();
                    draw_config_hint(&mut screen.display, false);
                });
                ctx.resources.rotary.pin_a().clear_interrupt();
                ctx.resources
                    .rotary
                    .pin_a()
                    .set_interrupt_mode(InterruptMode::Disabled);
                ctx.resources.rotary.pin_b().clear_interrupt();
                ctx.resources
                    .rotary
                    .pin_b()
                    .set_interrupt_mode(InterruptMode::Disabled);
                let new_time =
                    NaiveTime::from_hms(ctx.resources.new_time.0, ctx.resources.new_time.1, 0);
                ctx.resources.rtc.lock(|rtc| {
                    rtc.set_time(&new_time).unwrap();
                });
            }
            OperatingMode::Configuration(_) => {
                let new_time = ctx.resources.rtc.lock(|rtc| {
                    let t = rtc.get_time().unwrap();
                    (t.hour(), t.minute())
                });

                ctx.resources.screen.lock(|screen| {
                    draw_config_hint(&mut screen.display, true);
                });
                *ctx.resources.new_time = new_time;

                ctx.resources.rotary.pin_a().clear_interrupt();
                ctx.resources
                    .rotary
                    .pin_a()
                    .set_interrupt_mode(InterruptMode::EdgeBoth);
                ctx.resources.rotary.pin_b().clear_interrupt();
                ctx.resources
                    .rotary
                    .pin_b()
                    .set_interrupt_mode(InterruptMode::EdgeBoth);
            }
        }
        ctx.spawn.refresh_epd().unwrap();
    }

    #[task(
        resources = [mode, toggle_switch],
        schedule = [poll_toggle_switch],
        spawn = [change_mode]
    )]
    fn poll_toggle_switch(ctx: poll_toggle_switch::Context) {
        // Poll button
        let pressed: bool = ctx.resources.toggle_switch.pin.is_high().unwrap();

        // Update state
        let edge = ctx.resources.toggle_switch.debouncer.update(pressed);
        ctx.resources.toggle_switch.sample_count += 1;

        // Dispatch event
        if edge == Some(Edge::Rising) {
            ctx.resources.toggle_switch.sample_count = 0;
            ctx.spawn
                .change_mode(OperatingMode::Configuration(SubConfig::Hour))
                .unwrap();
        } else if edge == Some(Edge::Falling) {
            ctx.resources.toggle_switch.sample_count = 0;
            ctx.spawn.change_mode(OperatingMode::Normal).unwrap();
        } else if ctx.resources.toggle_switch.sample_count <= DEBOUNCE_SAMPLE_CNT {
            // Re-schedule the timer interrupt to get enough samples
            ctx.schedule
                .poll_toggle_switch(ctx.scheduled + POLL_SWITCH_PERIOD.cycles())
                .unwrap();
        } else {
            debug_only! {hprintln!("nothing ? {:?} {:?}", pressed, ctx.resources.toggle_switch.pin.is_high()).unwrap()}

            debug_only! {
                hprintln!("mode is {:?}, switch is {:?} debouncer is {:?}",
                          ctx.resources.mode,
                          ctx.resources.toggle_switch.pin.is_high(),
                          ctx.resources.toggle_switch.debouncer.is_high())
                    .unwrap()
            }

            ctx.resources.toggle_switch.sample_count = 0;
        }
    }

    #[task(
        resources = [mode, rotary_switch],
        schedule = [poll_rotary_switch],
        spawn = [change_mode]
    )]
    fn poll_rotary_switch(ctx: poll_rotary_switch::Context) {
        // Poll button
        let pressed: bool = ctx.resources.rotary_switch.pin.is_low().unwrap();

        // Update state
        let edge = ctx.resources.rotary_switch.debouncer.update(pressed);
        ctx.resources.rotary_switch.sample_count += 1;

        // Dispatch event
        if edge == Some(Edge::Rising) {
            debug_only! {hprintln!("rotary PRESSED").unwrap()}
            ctx.resources.rotary_switch.sample_count = 0;
            *ctx.resources.mode = match ctx.resources.mode {
                OperatingMode::Normal => OperatingMode::Normal,
                OperatingMode::Configuration(SubConfig::Hour) => {
                    OperatingMode::Configuration(SubConfig::Minute)
                }
                OperatingMode::Configuration(SubConfig::Minute) => {
                    OperatingMode::Configuration(SubConfig::Hour)
                }
            };
        //            ctx.spawn.change_mode(OperatingMode::Configuration);
        } else if edge == Some(Edge::Falling) {
            debug_only! {hprintln!("rotary RELEASED").unwrap()}
            ctx.resources.rotary_switch.sample_count = 0;
        //            ctx.spawn.change_mode(OperatingMode::Normal);
        } else if ctx.resources.rotary_switch.sample_count <= DEBOUNCE_SAMPLE_CNT {
            // Re-schedule the timer interrupt to get enough samples
            ctx.schedule
                .poll_rotary_switch(ctx.scheduled + POLL_SWITCH_PERIOD.cycles())
                .unwrap();
        } else {
            debug_only! {hprintln!("rotary nothing ? {:?} {:?}", pressed, ctx.resources.rotary_switch.pin.is_high()).unwrap()}

            debug_only! {
                hprintln!("rotary switch is {:?} debouncer is {:?}",
                          ctx.resources.rotary_switch.pin.is_high(),
                          ctx.resources.rotary_switch.debouncer.is_high())
                    .unwrap()
            }

            ctx.resources.rotary_switch.sample_count = 0;
        }
    }

    #[task(priority = 3, resources = [leds, leds_data])]
    fn refresh_leds(cx: refresh_leds::Context) {
        cx.resources
            .leds
            .write(cx.resources.leds_data.iter().cloned())
            .unwrap();
    }

    #[task(priority = 3, resources = [leds_data], spawn = [refresh_leds])]
    fn rotate_leds(cx: rotate_leds::Context, rotate_right: bool, num: usize) {
        if rotate_right {
            cx.resources.leds_data[0..].rotate_right(num);
        } else {
            cx.resources.leds_data[0..].rotate_left(num);
        }

        cx.spawn.refresh_leds().unwrap();
    }

    #[task(priority = 3, resources = [leds_data], spawn = [refresh_leds])]
    fn set_leds(cx: set_leds::Context, start_index: usize, number_of_leds: usize, color: RGB8) {
        for led in &mut cx.resources.leds_data[start_index..start_index + number_of_leds] {
            *led = color;
        }
        cx.spawn.refresh_leds().unwrap();
    }

    #[task(priority = 3, spawn = [set_leds], resources = [rtc, screen, ranges, state, next_or_current_range])]
    fn handle_event_alarm(cx: handle_event_alarm::Context, time: NaiveTime) {
        debug_only! {hprintln!("handle event alarm! {} {}", time.hour(), time.minute()).unwrap()}

        if let Some(new_state) = match cx.resources.state {
            FSMState::Idle => None,
            FSMState::WaitNextRange(ref range) => {
                debug_only! {hprintln!("Enter").unwrap()}
                cx.spawn
                    .set_leds(0, 1, cx.resources.ranges[*range].in_color)
                    .unwrap();

                debug_only! {hprintln!("Time: {}, Alarm :{}", time, cx.resources.ranges[*range].end).unwrap()}
                cx.resources
                    .rtc
                    .set_alarm1_hms(cx.resources.ranges[*range].end)
                    .unwrap();

                Some(FSMState::InRange(*range))
            }

            FSMState::InRange(ref range) => {
                debug_only! {hprintln!("Exit").unwrap()}

                cx.spawn.set_leds(0, 1, colors::GREEN).unwrap();
                if *range == cx.resources.ranges.len() - 1 {
                    cx.resources
                        .rtc
                        .set_alarm1_hms(cx.resources.ranges[0].start)
                        .unwrap();
                    Some(FSMState::WaitNextRange(0))
                } else {
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
    }

    #[task(priority = 1, resources = [screen])]
    fn refresh_epd(mut cx: refresh_epd::Context) {
        cx.resources.screen.lock(|screen| {
            screen
                .epd
                .update_and_display_frame(&mut screen.spi, &screen.display.buffer())
                .unwrap();
            screen.epd.sleep(&mut screen.spi).unwrap();
        });
    }

    #[task(priority = 1, resources = [screen, bme680, i2c_error], spawn = [refresh_epd])]
    fn refresh_bme680(mut cx: refresh_bme680::Context) {
        let (temp, _pres, humid, _gas): (i32, i32, i32, i32);

        if let Ok(_) = cx.resources.bme680.set_sensor_mode(PowerMode::ForcedMode) {
            let (data, _state) = cx.resources.bme680.get_sensor_data().unwrap();

            temp = data.temperature_celsius() as i32;
            _pres = data.pressure_hpa() as i32;
            humid = data.humidity_percent() as i32;
            _gas = data.gas_resistance_ohm() as i32;
        } else {
            // For some reason, the bme680 seems to stop responding and a hw
            // reset is needed to make it work again. Maybe caused by the
            // current spaghetti setup used for prototyping.
            // Simply count the errors
            temp = 0i32;
            _pres = 0i32;
            humid = 0i32;
            _gas = 0i32;
            *cx.resources.i2c_error += 1;
            debug_only! {hprintln!("I2C error for bme").unwrap()}
        }

        let cur_i2c_error = *cx.resources.i2c_error;

        let mut temp_s: String<U5> = String::new();
        uwrite!(temp_s, "{}{}°C", if temp < 10 { " " } else { "" }, temp).unwrap();

        let mut humid_s: String<U5> = String::new();
        uwrite!(
            humid_s,
            "{}{}%",
            if humid < 10 {
                "  "
            } else if humid < 100 {
                " "
            } else {
                ""
            },
            humid
        )
        .unwrap();

        cx.resources.screen.lock(|screen| {
            let _ = Text::new(
                &temp_s,
                Point::new(SCR_RIGHT_BAR_TEMP_X_OFF, SCR_RIGHT_BAR_TEMP_Y_OFF),
            )
            .into_styled(text_style!(
                font = FONT_RIGHT_BAR,
                text_color = Black,
                background_color = White
            ))
            .draw(&mut screen.display);

            let _ = Text::new(
                &humid_s,
                Point::new(SCR_RIGHT_BAR_HUMID_X_OFF, SCR_RIGHT_BAR_HUMID_Y_OFF),
            )
            .into_styled(text_style!(
                font = FONT_RIGHT_BAR,
                text_color = Black,
                background_color = White
            ))
            .draw(&mut screen.display);

            let mut errcnt_s: String<U5> = String::new();
            uwrite!(errcnt_s, "#{}", cur_i2c_error).unwrap();

            let _ = Text::new(
                &errcnt_s,
                Point::new(SCR_RIGHT_BAR_ERRCNT_X_OFF, SCR_RIGHT_BAR_ERRCNT_Y_OFF),
            )
            .into_styled(text_style!(
                font = FONT_RIGHT_BAR,
                text_color = Black,
                background_color = White
            ))
            .draw(&mut screen.display);
        });
    }

    #[task(priority = 1, resources = [screen], spawn = [refresh_epd])]
    fn refresh_time(mut cx: refresh_time::Context, time: NaiveTime) {
        cx.resources.screen.lock(|screen| {
            // Only "HH:MM"
            let mut hhmm: String<U5> = String::new();

            uwrite!(
                hhmm,
                "{}{}:{}{}",
                if time.hour() < 10 { "0" } else { "" },
                time.hour(),
                if time.minute() < 10 { "0" } else { "" },
                time.minute()
            )
            .unwrap();

            let _ = Text::new(&hhmm, Point::new(SCR_HOUR_X_OFF as i32, SCR_HOUR_Y_OFF))
                .into_styled(text_style!(
                    font = FONT_TIME,
                    text_color = Black,
                    background_color = White
                ))
                .draw(&mut screen.display);
        });

        cx.spawn.refresh_epd().unwrap();
        debug_only! {hprintln!("refresh time with: {} {}", time.hour(), time.minute()).unwrap()}
    }

    #[task(priority = 1, spawn = [refresh_time, rotate_leds], resources =[rtc, new_time, mode])]
    fn knob_turned(cx: knob_turned::Context, dir: Direction) {
        let next_time = cx.resources.new_time;

        match dir {
            Direction::Clockwise => {
                cx.spawn.rotate_leds(true, 1).unwrap();
                if *cx.resources.mode == OperatingMode::Configuration(SubConfig::Hour) {
                    next_time.0 += 1;
                } else {
                    next_time.1 += 1;
                }
            }
            Direction::CounterClockwise => {
                cx.spawn.rotate_leds(false, 1).unwrap();
                if *cx.resources.mode == OperatingMode::Configuration(SubConfig::Hour) {
                    next_time.0 -= 1;
                } else {
                    next_time.1 -= 1;
                }
            }
            _ => (),
        }

        cx.spawn
            .refresh_time(NaiveTime::from_hms(next_time.0, next_time.1, 0))
            .unwrap();
    }

    #[task(binds = GPIOB, resources = [rotary, mode, toggle_switch, rotary_switch], spawn = [ knob_turned, poll_toggle_switch, poll_rotary_switch ])]
    fn gpiob_rotary_switch(cx: gpiob_rotary_switch::Context) {
        // whatever happens, clear these ITs.
        cx.resources.rotary.pin_a().clear_interrupt();
        cx.resources.rotary.pin_b().clear_interrupt();

        if cx.resources.toggle_switch.pin.get_interrupt_status() {
            debug_only! {hprintln!("toggle switch IT raised, starting debouncing sampling!").unwrap()}
            cx.resources.toggle_switch.pin.clear_interrupt();

            // debug_only! {
            //     hprintln!("mode is {:?}, tog switch is {:?} debouncer is {:?}",
            //               cx.resources.mode,
            //               cx.resources.toggle_switch.pin.is_high(),
            //               cx.resources.toggle_switch.debouncer.is_high())
            //         .unwrap()
            // }

            cx.spawn.poll_toggle_switch().unwrap();
        }

        if cx.resources.rotary_switch.pin.get_interrupt_status() {
            cx.resources.rotary_switch.pin.clear_interrupt();
            cx.spawn.poll_rotary_switch().unwrap();
        }

        match *cx.resources.mode {
            OperatingMode::Normal => (),

            OperatingMode::Configuration(_) => {
                let dir = cx.resources.rotary.update().unwrap();
                match dir {
                    Direction::Clockwise | Direction::CounterClockwise => {
                        cx.spawn.knob_turned(dir).unwrap();
                    }
                    Direction::None => {}
                }
            }
        }
    }

    #[task(binds = GPIOD, resources = [rtc, mode, rtc_int_pin], spawn = [refresh_time, refresh_bme680, handle_event_alarm] )]
    fn gpiod(mut cx: gpiod::Context) {
        let mut a1 = false;
        let mut a2 = false;

        let time = cx.resources.rtc.lock(|rtc| {
            if rtc.has_alarm1_matched().unwrap() {
                rtc.clear_alarm1_matched_flag().unwrap();
                a1 = true;
            }
            if rtc.has_alarm2_matched().unwrap() {
                rtc.clear_alarm2_matched_flag().unwrap();
                a2 = true;
            }
            rtc.get_time().unwrap()
        });

        if a1 {
            cx.spawn.handle_event_alarm(time).unwrap();
        }
        if a2 && *cx.resources.mode == OperatingMode::Normal {
            cx.spawn.refresh_bme680().unwrap();
            cx.spawn.refresh_time(time).unwrap();
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

fn draw_config_hint(display: &mut Display2in13, is_config: bool) {
    let text = if is_config { "C" } else { " " };

    let _ = Text::new(text, Point::new(SCR_CONFIG_X_OFF, SCR_CONFIG_Y_OFF))
        .into_styled(text_style!(
            font = FONT_CONFIG,
            text_color = Black,
            background_color = White
        ))
        .draw(display);
}
