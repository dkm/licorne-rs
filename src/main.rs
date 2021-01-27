#![deny(unsafe_code)]
// #![deny(warnings)]
#![no_main]
#![no_std]

//#[macro_use]
extern crate cortex_m_rt;

extern crate nb;

extern crate chrono;
extern crate cortex_m_semihosting;
extern crate ds323x;

extern crate embedded_graphics;
extern crate panic_semihosting;
extern crate smart_leds;
extern crate tm4c123x_hal;
extern crate tm4c_hal;
extern crate ws2812_spi;

extern crate embedded_time;

use embedded_hal::{
    blocking::delay::{DelayMs, DelayUs},
    digital::v2::InputPin,
};

use embedded_graphics::{
    fonts::Text, pixelcolor::BinaryColor::Off as White, pixelcolor::BinaryColor::On as Black,
    prelude::*, text_style,
};

use epd_waveshare::epd2in13_v2::{Display2in13, EPD2in13};

const NUM_LEDS: usize = 3;

const FONT_TIME: embedded_various_fonts::fonts::GNUTypeWriter60Point =
    embedded_various_fonts::fonts::GNUTypeWriter60Point {};

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

impl drogue_bme680::Delay for SimpleAsmDelay {
    fn delay_ms(&self, duration_ms: u16) {
        cortex_m::asm::delay(self.freq * cast::u32(duration_ms) * 1000 / 1_000_000)
    }
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
// PA2: SSI0 [epd]
// PA4: SSI0 [epd unconnected]
// PA5: SSI0 [epd]
// PA6: I2C1 (scl) [bme680]
// PA7: I2C1 (sca) [bme680]
//
//
// PB1: Rotary Encoder (pinA)
// PB2: I2C0 (SCL) [rtc]
// PB3: I2C0 (SDA) [rtc]
// PB4: Rotary Encoder (pinB)
// PB5: Toggle Switch (config mode)
// PB6:
// PC4: Rotary switch
//
// PC6: GPIO [EPD-cs]
//
// PD0: SSI1 [WS2812 unconnected]
// PD1: [EPD-reset]
// PD2: SSI1 [WS2812 unconnected]
// PD3: SSI1 [WS2812 DIN]
// PD6: RTC / INT
//
// PE2: GPIO Floating [EPD-busy]
// PE4: [EPD-dc]

use ds323x::{Ds323x, NaiveTime};
use tm4c123x_hal::{
    gpio::gpioa::{PA2, PA4, PA5, PA6, PA7},
    gpio::gpiob::{PB1, PB2, PB3, PB4, PB5},
    gpio::gpioc::{PC4, PC6},
    gpio::gpiod::{PD0, PD1, PD2, PD3},
    gpio::gpioe::{PE2, PE4},
    gpio::{
        AlternateFunction, Floating, Input, OpenDrain, Output, PullDown, PullUp, PushPull, AF2, AF3,
    },
    i2c::I2c,
    spi::Spi,
    tm4c123x::{I2C0, I2C1, SSI0, SSI1},
};

use drogue_bme680::{Bme680Controller, StaticProvider};

use debouncr::{debounce_stateful_12, DebouncerStateful, Repeat12};
use smart_leds::RGB8;

use rotary_encoder_hal::Rotary;
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

type Bme680T = Bme680Controller<I2c1T, SimpleAsmDelay, StaticProvider>;

// spi for EPD
type Spi0T = Spi<
    SSI0,
    (
        PA2<AlternateFunction<AF2, PushPull>>,
        PA4<AlternateFunction<AF2, PushPull>>,
        PA5<AlternateFunction<AF2, PullDown>>,
    ),
>;

// spi for WS2812 screen
type Spi1T = Spi<
    SSI1,
    (
        PD0<AlternateFunction<AF2, PushPull>>,
        PD2<AlternateFunction<AF2, PushPull>>,
        PD3<AlternateFunction<AF2, PullDown>>,
    ),
>;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EPDModeHint {
    None,
    ForceFull,
    ForcePartial,
}

pub type EpdT = EPD2in13<
    Spi0T,
    PC6<Output<PushPull>>,
    PE2<Input<Floating>>,
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
    Configuration,
}

const MAX_NUM_QUICK_REFRESH: u32 = 0;

pub struct Screen {
    epd: EpdT,
    spi: Spi0T,
    delay: SimpleAsmDelay,
    quick_refresh_count: u32,
}

pub struct Switch<P>
where
    P: InputPin,
{
    pin: P,
    debouncer: DebouncerStateful<u16, Repeat12>,
    sample_count: i8,
}

impl<P> Switch<P>
where
    P: InputPin,
{
    pub fn new(pin: P, state: bool) -> Switch<P> {
        Switch {
            pin,
            debouncer: debounce_stateful_12(state),
            sample_count: -1,
        }
    }
}

type ToggleSwitchPinT = PB5<Input<PullUp>>;
type ToggleSwitchT = Switch<ToggleSwitchPinT>;

type RotarySwitchPinT = PC4<Input<PullUp>>;
type RotarySwitchT = Switch<RotarySwitchPinT>;

#[rtic::app(device = tm4c123x_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT, dispatchers = [UART1, UART2, UART3, UART4])]
mod app {

    use crate::{
        draw_config_hint, Bme680T, EPDModeHint, FSMState, OperatingMode, RotarySwitchT, RotaryT,
        RtcT, Screen, SimpleAsmDelay, Spi1T, SubConfig, TimeRange, ToggleSwitchT, FONT_RIGHT_BAR,
        FONT_TIME, MAX_NUM_QUICK_REFRESH, NUM_LEDS, SCR_HOUR_X_OFF, SCR_HOUR_Y_OFF,
        SCR_RIGHT_BAR_ERRCNT_X_OFF, SCR_RIGHT_BAR_ERRCNT_Y_OFF, SCR_RIGHT_BAR_HUMID_X_OFF,
        SCR_RIGHT_BAR_HUMID_Y_OFF, SCR_RIGHT_BAR_TEMP_X_OFF, SCR_RIGHT_BAR_TEMP_Y_OFF,
    };
    use rtic::cyccnt::Instant;

    use tm4c123x_hal::{
        gpio::{gpiod::PD6, Floating, GpioExt, Input, InterruptMode, PullUp, AF2, AF3},
        i2c::I2c,
        spi::{Spi, MODE_0},
        sysctl::{self, SysctlExt},
        time::U32Ext,
    };

    use embedded_graphics::{
        fonts::Text, pixelcolor::BinaryColor::Off as White, pixelcolor::BinaryColor::On as Black,
        prelude::*, text_style,
    };

    use heapless::{consts::U5, String};

    use ufmt::uwrite;

    use rtic::cyccnt::U32Ext as _;

    use drogue_bme680::{Address, Bme680Controller, Bme680Sensor, Configuration, StaticProvider};

    use embedded_hal::digital::v2::InputPin;

    #[cfg(feature = "usesemihosting")]
    use cortex_m_semihosting::hprintln;

    use debouncr::Edge;
    use rotary_encoder_hal::{Direction, Rotary};

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

    // Runnig at 80Mhz
    const ONE_MUSEC: u32 = 80;
    const ONE_MSEC: u32 = ONE_MUSEC * 1000;
    const ONE_SEC: u32 = ONE_MSEC * 1000;

    const TRANSITION_STEPS: u32 = 50;
    const TRANSITION_STEP_CYCLES: u32 = (1000 / TRANSITION_STEPS) * ONE_MSEC;

    // At 80Mhz, this is ~1ms
    const POLL_SWITCH_PERIOD: u32 = 10 * ONE_MUSEC;
    const DEBOUNCE_SAMPLE_CNT: i8 = 20;
    const ROTARY_SAMPLING_PERIOD: u32 = 10 * ONE_MUSEC;

    #[resources]
    struct Resources {
        mode: OperatingMode,
        submode: SubConfig,

        // The RTC periph
        #[cfg(feature = "rtc")]
        rtc: RtcT,

        // e-ink
        #[cfg(feature = "epd")]
        screen: Screen,

        display: Display2in13,

        #[cfg(feature = "rtc")]
        rtc_int_pin: PD6<Input<PullUp>>,

        // ws2812
        leds: ws2812::Ws2812<Spi1T>,
        leds_data: [RGB8; NUM_LEDS],

        // temp & cov
        #[cfg(feature = "bme")]
        bme680: Bme680T,

        toggle_switch: ToggleSwitchT,
        rotary_switch: RotarySwitchT,

        #[cfg(feature = "rtc")]
        new_time: NaiveTime,

        rotary: RotaryT,

        i2c_error: u32,

        #[cfg(feature = "rtc")]
        next_or_current_range: usize,

        #[cfg(feature = "rtc")]
        ranges: [TimeRange; 2],

        #[cfg(feature = "rtc")]
        state: FSMState,
    }

    #[init]
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

        #[cfg(feature = "rtc")]
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
        #[cfg(feature = "rtc")]
        let mut rtc = Ds323x::new_ds3231(i2c_rtc_dev);

        let mut spi0 = Spi::spi0(
            p.SSI0,
            (
                porta.pa2.into_af_push_pull::<AF2>(&mut porta.control), // SCK
                porta.pa4.into_af_push_pull::<AF2>(&mut porta.control), // Miso
                porta.pa5.into_af_pull_down::<AF2>(&mut porta.control), // Mosi
            ), // Mosi
            MODE_0,
            2_000_000.hz(),
            &clocks,
            &sc.power_control,
        );
        let cs_pin = portc.pc6.into_push_pull_output(); // cs
        let rst_pin = portd.pd1.into_push_pull_output();
        let dc_pin = porte.pe4.into_push_pull_output(); // dc
        let busy_pin = porte.pe2.into_floating_input();

        // Cortex-M peripherals
        // : cortex_m::Peripherals

        #[cfg(feature = "bme")]
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

        #[cfg(feature = "bme")]
        let bme680_sensor = Bme680Sensor::from(i2c_bme680, Address::Secondary).unwrap();

        #[cfg(feature = "bme")]
        let bme680 = Bme680Controller::new(
            bme680_sensor,
            SimpleAsmDelay { freq: 80_000_000 },
            Configuration::standard(),
            StaticProvider(25),
        )
        .unwrap();

        let mut delay = SimpleAsmDelay { freq: 80_000_000 };

        #[cfg(feature = "epd")]
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
                portd.pd3.into_af_pull_down::<AF2>(&mut portd.control), // Mosi (connected)
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
        let rotary_pin = portc.pc4.into_pull_up_input();

        // GPIO for interrupt
        // SQW/INT pin wired to PD6
        #[cfg(feature = "rtc")]
        let mut rtc_int_pin = portd.pd6.into_pull_up_input();

        #[cfg(feature = "rtc")]
        let ranges;

        #[cfg(feature = "rtc")]
        let (mut next_or_current_range, mut state);

        #[cfg(feature = "rtc")]
        {
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
            refresh_time::spawn(time, false).unwrap();

            #[cfg(feature = "testranges")]
            {
                ranges = [
                    TimeRange {
                        start: time
                            .overflowing_add_signed(chrono::Duration::seconds(60 * 1))
                            .0,
                        end: time
                            .overflowing_add_signed(chrono::Duration::seconds(60 * 2))
                            .0,

                        // start: NaiveTime::from_hms(0, 0, 0),
                        // end: NaiveTime::from_hms(5, 0, 0),
                        in_color: colors::RED,
                        name: "0:0->5:0",
                    },
                    TimeRange {
                        start: time
                            .overflowing_add_signed(chrono::Duration::seconds(60 * 3))
                            .0,
                        end: time
                            .overflowing_add_signed(chrono::Duration::seconds(60 * 4))
                            .0,

                        // start: NaiveTime::from_hms(5, 1, 0),
                        // end: NaiveTime::from_hms(7, 0, 0),
                        in_color: colors::GREEN,
                        name: "5:1->7:0",
                    },
                ];
            }

            #[cfg(not(feature = "testranges"))]
            {
                ranges = [
                    TimeRange {
                        start: NaiveTime::from_hms(20, 0, 0),
                        end: NaiveTime::from_hms(7, 30, 0),
                        in_color: colors::PINK,
                        name: "night",
                    },
                    TimeRange {
                        start: NaiveTime::from_hms(14, 00, 0),
                        end: NaiveTime::from_hms(16, 0, 0),
                        in_color: colors::PINK,
                        name: "nap",
                    },
                ];
            }
            state = FSMState::Idle;
            let mut found = false;
            next_or_current_range = 0;

            let mut current_color = &IDLE_COLOR;

            debug_only! {
                hprintln!(
                    "Testing with time  #{:?}",
                    time
                ).unwrap()
            }

            for (i, range) in ranges.iter().enumerate() {
                next_or_current_range = i;

                debug_only! {
                    hprintln!(
                        "Testing range #{}:  {} -> {}",
                        i,
                        range.start,
                        range.end
                    ).unwrap()
                }

                if time < range.start {
                    state = FSMState::WaitNextRange(i);
                    debug_only! {
                        hprintln!(
                            "Waiting for next range to activate: {} -> {}",
                            range.start,
                            range.end
                        ).unwrap()
                    }

                    rtc.set_alarm1_hms(range.start).unwrap();
                    found = true;
                    break;
                }

                if time >= range.start && (time < range.end || range.end < range.start) {
                    state = FSMState::InRange(i);
                    debug_only! {hprintln!("In range: {}:{}", range.start, range.end).unwrap()}

                    transition_leds::spawn(range.in_color, 50).unwrap();

                    rtc.set_alarm1_hms(range.end).unwrap();

                    found = true;
                    break;
                }
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

            #[cfg(feature = "epd")]
            epd.update_and_display_frame(&mut spi0, &display.buffer())
                .unwrap();

            // enable partial refresh from now on
            #[cfg(feature = "epd")]
            epd.set_refresh(&mut spi0, &mut delay, RefreshLUT::QUICK)
                .unwrap();
        }

        #[cfg(feature = "bme")]
        refresh_bme680::spawn(false).unwrap();

        let is_rot_switch_pressed = rotary_pin.is_low().unwrap();
        let rotary_switch = RotarySwitchT::new(rotary_pin, is_rot_switch_pressed);

        debug_only! {
            hprintln!("init switch {:?}", is_rot_switch_pressed).unwrap()
        }

        // switch is using PUR, is_low() <=> pressed <=> 'true' state
        let toggle_pin = portb.pb5.into_pull_up_input();
        let toggle_init_state = toggle_pin.is_high().unwrap();
        let mut toggle_switch = ToggleSwitchT::new(toggle_pin, toggle_init_state);
        toggle_switch
            .pin
            .set_interrupt_mode(InterruptMode::EdgeBoth);
        toggle_switch.pin.clear_interrupt();

        let switch_state = toggle_switch.pin.is_high().unwrap();

        let mode = if switch_state {
            OperatingMode::Configuration
        } else {
            OperatingMode::Normal
        };

        change_mode::spawn(mode).unwrap();

        debug_only! {
            hprintln!("init done, mode will be {:?}, switch is {:?} debouncer is {:?}",
                      mode,
                      switch_state,
                      toggle_switch.debouncer.is_high())
                .unwrap()
        }

        init::LateResources {
            mode: OperatingMode::Normal,
            submode: SubConfig::Hour,

            #[cfg(feature = "rtc")]
            rtc,

            #[cfg(feature = "epd")]
            screen: Screen {
                epd,
                spi: spi0,
                delay,
                quick_refresh_count: 0,
            },

            display,

            #[cfg(feature = "rtc")]
            rtc_int_pin,

            leds,
            leds_data: [colors::BLACK; NUM_LEDS],

            #[cfg(feature = "bme")]
            bme680,

            #[cfg(feature = "rtc")]
            ranges,

            #[cfg(feature = "rtc")]
            next_or_current_range,

            #[cfg(feature = "rtc")]
            state,
            rotary,

            #[cfg(feature = "rtc")]
            new_time: NaiveTime::from_hms(0, 0, 0),

            i2c_error: 0,

            toggle_switch,
            rotary_switch,
        }
    }

    // TOP Priority task.

    // These tasks can't be interrupted by other or they may break. They are
    // relying on the naive Delay implementation counting some nop. If they are
    // interrupted, timing will be incorrect.

    #[cfg(feature = "epd")]
    #[task(priority = 5, resources = [screen, display])]
    fn refresh_epd(cx: refresh_epd::Context, mode_hint: EPDModeHint) {
        (cx.resources.screen, cx.resources.display).lock(|screen, display| {
            let do_full_refresh = (screen.quick_refresh_count > MAX_NUM_QUICK_REFRESH
                && mode_hint != EPDModeHint::ForcePartial)
                || mode_hint == EPDModeHint::ForceFull;

            let mut errcnt_s: String<U5> = String::new();
            uwrite!(errcnt_s, "#{}", screen.quick_refresh_count).unwrap();

            let _ = Text::new(
                &errcnt_s,
                Point::new(SCR_RIGHT_BAR_ERRCNT_X_OFF, SCR_RIGHT_BAR_ERRCNT_Y_OFF + 16),
            )
            .into_styled(text_style!(
                font = FONT_RIGHT_BAR,
                text_color = Black,
                background_color = White
            ))
            .draw(display);

            if do_full_refresh {
                screen.quick_refresh_count = 0;
                screen
                    .epd
                    .set_refresh(&mut screen.spi, &mut screen.delay, RefreshLUT::FULL)
                    .unwrap();
            }

            screen
                .epd
                .update_and_display_frame(&mut screen.spi, &display.buffer())
                .unwrap();

            if do_full_refresh {
                screen
                    .epd
                    .set_refresh(&mut screen.spi, &mut screen.delay, RefreshLUT::QUICK)
                    .unwrap();
            }
            screen.quick_refresh_count += 1;
        });
    }

    #[cfg(feature = "bme")]
    #[task(priority = 5, resources = [display, bme680, i2c_error])]
    fn refresh_bme680(cx: refresh_bme680::Context, refresh_epd: bool) {
        (cx.resources.bme680, cx.resources.i2c_error, cx.resources.display ).lock(|bme680, i2c_error, display| {
            let (temp, _pres, humid, _gas): (i32, i32, i32, i32);

            if let Ok(opt_data) = bme680.measure_default() {
                if let Some(data) = opt_data {
                    temp = data.temperature as i32;
                    _pres = if let Some(f) = data.pressure {
                        f as i32
                    } else {
                        0
                    };
                    humid = data.humidity as i32;
                    _gas = data.gas_resistance as i32;
                } else {
                    temp = 0i32;
                    _pres = 0i32;
                    humid = 0i32;
                    _gas = 0i32;
                    *i2c_error += 1;
                    debug_only! {hprintln!("I2C OK, but empty result from drogue_bme680").unwrap()}
                }
            } else {
                // For some reason, the bme680 seems to stop responding and a hw
                // reset is needed to make it work again. Maybe caused by the
                // current spaghetti setup used for prototyping.
                // Simply count the errors
                temp = 0i32;
                _pres = 0i32;
                humid = 0i32;
                _gas = 0i32;
                *i2c_error += 1;
                debug_only! {hprintln!("I2C error drogue_bme680 for bme").unwrap()}
                bme680.soft_reinit().unwrap();
            }

            let cur_i2c_error = *i2c_error;

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

            let _ = Text::new(
                &temp_s,
                Point::new(SCR_RIGHT_BAR_TEMP_X_OFF, SCR_RIGHT_BAR_TEMP_Y_OFF),
            )
                .into_styled(text_style!(
                    font = FONT_RIGHT_BAR,
                    text_color = Black,
                    background_color = White
                ))
                .draw(display);

            let _ = Text::new(
                &humid_s,
                Point::new(SCR_RIGHT_BAR_HUMID_X_OFF, SCR_RIGHT_BAR_HUMID_Y_OFF),
            )
                .into_styled(text_style!(
                    font = FONT_RIGHT_BAR,
                    text_color = Black,
                    background_color = White
                ))
                .draw(display);

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
                .draw(display);
        });
        if refresh_epd {
            #[cfg(feature = "epd")]
            refresh_epd::spawn(EPDModeHint::None).unwrap();
        }
    }

    // Priority 4 tasks
    //
    // These must be executed ASAP: change mode and ISR.
    #[task(
        priority = 4,
        resources = [mode, rotary, rotary_switch, display, new_time, rtc, rtc_int_pin],
    )]
    fn change_mode(mut ctx: change_mode::Context, mode: OperatingMode) {
        let must_continue = ctx.resources.mode.lock(|current_mode| {
            debug_only! {hprintln!("change mode from {:?} to {:?}", *current_mode, mode).unwrap()}

            let prev_mode = *current_mode;
            *current_mode = mode;

            (prev_mode == OperatingMode::Normal) ^ (mode == OperatingMode::Normal)
        });

        if !must_continue {
            debug_only! {hprintln!("ret").unwrap()}
            return;
        }

        match mode {
            OperatingMode::Normal => {
                ctx.resources.display.lock(|mut display| {
                    draw_config_hint(&mut display, false);
                });

                #[cfg(feature = "rtc")]
                {
                    let copy_new_time = ctx.resources.new_time.lock(|t| *t);
                    ctx.resources.rtc.lock(|rtc| {
                        rtc.set_time(&copy_new_time).unwrap();
                        rtc.clear_alarm2_matched_flag().unwrap();
                        rtc.clear_alarm1_matched_flag().unwrap();
                    });

                    // Monitor alarms from RTC
                    ctx.resources.rtc_int_pin.lock(|p| {
                        p.clear_interrupt();
                        p.set_interrupt_mode(InterruptMode::EdgeFalling);
                    });
                }
                // Disable anything comming from rotary switch
                // ctx.resources.rotary_switch.lock(|rs| {
                //     rs.pin.set_interrupt_mode(InterruptMode::Disabled);
                // });
            }
            OperatingMode::Configuration => {
                #[cfg(feature = "rtc")]
                {
                    let cur_time = ctx.resources.rtc.lock(|rtc| rtc.get_time().unwrap());
                    ctx.resources.new_time.lock(|t| {
                        *t = cur_time;
                    });

                    ctx.resources.display.lock(|display| {
                        draw_config_hint(display, true);
                    });
                    // Do not monitor alarms from RTC
                    ctx.resources
                        .rtc_int_pin
                        .lock(|p| p.set_interrupt_mode(InterruptMode::Disabled));
                }
                rotary_sampling::spawn(true).unwrap();
                refresh_during_configuration::spawn().unwrap();
            }
        }
        //        refresh_epd::spawn().unwrap();
    }

    #[task(priority = 4, binds = GPIOB, resources = [toggle_switch])]
    fn gpiob(mut cx: gpiob::Context) {
        debug_only! {hprintln!("toggle switch IT raised!").unwrap()}

        cx.resources.toggle_switch.lock(|ts| {
            if ts.pin.get_interrupt_status() {
                ts.pin.clear_interrupt();

                if ts.sample_count == -1 {
                    debug_only! {hprintln!("Starting toggle switch sampling").unwrap()}
                    ts.sample_count = DEBOUNCE_SAMPLE_CNT;
                    poll_toggle_switch::spawn(true).unwrap();
                } else {
                    debug_only! {hprintln!("Not starting toggle switch sampling as it's already running {}", ts.sample_count).unwrap()}
                    ts.sample_count = DEBOUNCE_SAMPLE_CNT;
                }
            }
        });
    }

    #[cfg(feature = "rtc")]
    #[task(priority = 4, binds = GPIOD, resources = [rtc, mode, rtc_int_pin])]
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
            handle_event_alarm::spawn().unwrap();
        }

        cx.resources.rtc_int_pin.lock(|p| p.clear_interrupt());

        if cx
            .resources
            .mode
            .lock(|mode| a2 && *mode == OperatingMode::Normal)
        {
            #[cfg(feature = "bme")]
            refresh_bme680::spawn(false).unwrap();

            #[cfg(feature = "rtc")]
            refresh_time::spawn(time, false).unwrap();
        }
        #[cfg(feature = "epd")]
        refresh_epd::spawn(EPDModeHint::None).unwrap();

        debug_only! {hprintln!("hep").unwrap()}
    }

    // Priority 3 tasks.
    //
    // These are not really critical and can afford being preempted.
    // Mostly the polling tasks.

    #[task(
        priority = 3,
        resources = [mode, new_time],
    )]
    fn refresh_during_configuration(mut ctx: refresh_during_configuration::Context) {
        let scheduled = Instant::now(); // ctx.scheduled;

        #[cfg(feature = "rtc")]
        let newtime = ctx.resources.new_time.lock(|nt| *nt);

        ctx.resources.mode.lock(|mode| {
            if *mode == OperatingMode::Configuration {
                #[cfg(feature = "rtc")]
                refresh_time::spawn(newtime, false).unwrap();

                refresh_during_configuration::schedule(scheduled + (2 * ONE_SEC).cycles()).unwrap();

                #[cfg(feature = "epd")]
                refresh_epd::spawn(EPDModeHint::ForcePartial).unwrap();
            }
        });
    }

    #[task(
        priority = 3,
        resources = [mode, toggle_switch],
    )]
    fn poll_toggle_switch(mut ctx: poll_toggle_switch::Context, first: bool) {
        if first {
            debug_only! {hprintln!("poll toggle sampling").unwrap()}
        }

        // Poll button
        let scheduled = Instant::now(); // ctx.scheduled;

        ctx.resources.toggle_switch.lock(|ts| {
            let pressed = ts.pin.is_high().unwrap();
            ts.sample_count -= 1;

            // Update state
            let edge = ts.debouncer.update(pressed);

            // Dispatch event
            if edge == Some(Edge::Rising) {
                ts.sample_count = -1;
                change_mode::spawn(OperatingMode::Configuration)
                    .unwrap();
            } else if edge == Some(Edge::Falling) {
                ts.sample_count = -1;
                change_mode::spawn(OperatingMode::Normal).unwrap();
            } else if ts.sample_count > 0 {
                // Re-schedule the timer interrupt to get enough samples
                poll_toggle_switch::schedule(scheduled + POLL_SWITCH_PERIOD.cycles(), false).unwrap();
            } else {
                debug_only! {hprintln!("nothing ? {:?} {:?}", pressed, ts.pin.is_high().unwrap()).unwrap()}
                ts.sample_count = -1;
            }
        });
    }

    #[cfg(feature = "rtc")]
    #[task(priority = 3, resources = [rtc, ranges, state, next_or_current_range])]
    fn handle_event_alarm(cx: handle_event_alarm::Context) {
        debug_only! {hprintln!("handle event alarm!").unwrap()}

        (cx.resources.state, cx.resources.ranges, cx.resources.rtc).lock(|state, ranges, rtc| {
            let ns = match state {
                FSMState::Idle => None,
                FSMState::WaitNextRange(ref range) => {
                    debug_only! {hprintln!("Enter").unwrap()}

                    transition_leds::spawn(ranges[*range].in_color, TRANSITION_STEPS).unwrap();

                    debug_only! {hprintln!("Alarm :{}", ranges[*range].end).unwrap()}
                    rtc.set_alarm1_hms(ranges[*range].end).unwrap();

                    Some(FSMState::InRange(*range))
                }

                FSMState::InRange(ref range) => {
                    debug_only! {hprintln!("Exit").unwrap()}

                    transition_leds::spawn(colors::GREEN, 50).unwrap();

                    if *range == ranges.len() - 1 {
                        rtc.set_alarm1_hms(ranges[0].start).unwrap();
                        Some(FSMState::WaitNextRange(0))
                    } else {
                        rtc.set_alarm1_hms(ranges[*range + 1].start).unwrap();
                        Some(FSMState::WaitNextRange(*range + 1))
                    }
                }
            };

            if let Some(new_state) = ns {
                *state = new_state;
            }
        });
    }

    #[task(priority = 3, capacity = 2, resources = [display])]
    fn refresh_time(mut cx: refresh_time::Context, time: NaiveTime, refresh_epd: bool) {
        cx.resources.display.lock(|display| {
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

            debug_only! {hprintln!("refresh time with: {}", hhmm).unwrap()}

            let _ = Text::new(&hhmm, Point::new(SCR_HOUR_X_OFF as i32, SCR_HOUR_Y_OFF))
                .into_styled(text_style!(
                    font = FONT_TIME,
                    text_color = Black,
                    background_color = White
                ))
                .draw(display);
        });

        if refresh_epd {
            #[cfg(feature = "epd")]
            refresh_epd::spawn(EPDModeHint::None).unwrap();
        }
    }

    #[task(priority = 3,
           resources =[mode, submode, rotary, rotary_switch, new_time])]
    fn rotary_sampling(mut cx: rotary_sampling::Context, first: bool) {
        if first {
            debug_only! {hprintln!("rotary sampling").unwrap()}
        }
        let scheduled = Instant::now(); //cx.scheduled;

        #[cfg(feature = "rtc")]
        let mut new_time = cx.resources.new_time.lock(|t| *t);

        (
            cx.resources.mode,
            cx.resources.submode,
            cx.resources.rotary_switch,
            cx.resources.rotary,
        )
            .lock(|mode, submode, rs, r| {
                let current_mode = *mode;

                match current_mode {
                    OperatingMode::Normal => {
                        debug_only! {hprintln!("Stopping rotary sampling").unwrap()}
                    }

                    OperatingMode::Configuration => {
                        // Refresh Rotary encoder and maybe spawn handler
                        let dir = r.update().unwrap();
                        match dir {
                            Direction::Clockwise | Direction::CounterClockwise => {
//                                knob_turned::spawn(dir).unwrap();
                                let sign = match dir {
                                    Direction::Clockwise => 1,
                                    Direction::CounterClockwise => -1,
                                    _ => 0,
                                };

                                let add = match *submode {
                                    SubConfig::Hour => chrono::Duration::hours(1 * sign),
                                    SubConfig::Minute => chrono::Duration::minutes(1 * sign),
                                };

                                #[cfg(feature = "rtc")]
                                {
                                    new_time = new_time.overflowing_add_signed(add).0;
                                }

                                debug_only! {hprintln!("knob turned").unwrap()}
                                // Should be quick as EPD in partial refresh during configuration.
                                // refresh_time::spawn(*new_time, false).unwrap();
                            }
                            Direction::None => {}
                        }

                        // Refresh switch and maybe spawn handler
                        let pressed: bool = rs.pin.is_low().unwrap();

                        let edge = rs.debouncer.update(pressed);
//                        debug_only! {hprintln!("rotary switch status {:?}", pressed).unwrap()}
                        // Dispatch event
                        if edge == Some(Edge::Rising) {
                            debug_only! {hprintln!("rotary PRESSED").unwrap()}
                        } else if edge == Some(Edge::Falling) {
                            debug_only! {hprintln!("rotary RELEASED").unwrap()}
                            *submode = match *submode {
                                SubConfig::Hour => SubConfig::Minute,
                                SubConfig::Minute => SubConfig::Hour,
                            };
                        }

                        // Continue sampling
                        rotary_sampling::schedule(
                            scheduled + ROTARY_SAMPLING_PERIOD.cycles(),
                            false,
                        )
                        .unwrap();
                    }
                }
            });

        #[cfg(feature = "rtc")]
        cx.resources.new_time.lock(|t| *t = new_time);
    }

    // Priority 1 tasks.
    //
    // Low prio tasks that can really be executed when nothing else is left. It
    // should not break anything nor provide a bad feedback to the user.
    #[task(
        priority = 1,
        resources = [leds_data],
    )]
    fn transition_leds(mut cx: transition_leds::Context, to: RGB8, trans_step: u32) {
        cx.resources.leds_data.lock(|leds| {
            for led in leds.iter_mut() {
                if trans_step > 1 {
                    let dr = to.r - led.r;
                    let dg = to.g - led.g;
                    let db = to.b - led.b;
                    *led = RGB8::new(
                        led.r + (dr as u32 / trans_step) as u8,
                        led.g + (dg as u32 / trans_step) as u8,
                        led.b + (db as u32 / trans_step) as u8,
                    );
                } else {
                    *led = to;
                }
            }
        });

        if trans_step > 1 {
            transition_leds::schedule(
                Instant::now() // cx.scheduled
                    + TRANSITION_STEP_CYCLES.cycles(),
                to,
                trans_step - 1,
            )
            .unwrap();
        }
        refresh_leds::spawn().unwrap();
    }

    #[task(priority = 1, resources = [leds, leds_data])]
    fn refresh_leds(cx: refresh_leds::Context) {
        (cx.resources.leds_data, cx.resources.leds).lock(|leds_data, leds| {
            leds.write(leds_data.iter().cloned()).unwrap();
        });
    }

    // #[task(priority = 3, resources = [leds_data])]
    // fn rotate_leds(mut cx: rotate_leds::Context, rotate_right: bool, num: usize) {
    //     cx.resources.leds_data.lock(|ld| {
    //         if rotate_right {
    //             ld[0..].rotate_right(num);
    //         } else {
    //             ld[0..].rotate_left(num);
    //         }
    //     });
    //     refresh_leds::spawn().unwrap();
    // }

    // #[task(priority = 3, resources = [leds_data])]
    // fn set_leds(mut cx: set_leds::Context, start_index: usize, number_of_leds: usize, color: RGB8) {
    //     cx.resources.leds_data.lock(|ld| {
    //         for led in &mut ld[start_index..start_index + number_of_leds] {
    //             *led = color;
    //         }
    //     });
    //     refresh_leds::spawn().unwrap();
    // }

    // #[task(capacity = 10, priority = 1, resources =[rtc, new_time, submode])]
    // fn knob_turned(cx: knob_turned::Context, dir: Direction) {
    //     (cx.resources.submode, cx.resources.new_time).lock(|submode, new_time| {
    //         debug_only! {hprintln!("knob turned {:?} {:?}", dir, submode).unwrap()}

    //     });
    // }
}

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
