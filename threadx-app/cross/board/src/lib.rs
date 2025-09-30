#![no_std]
use core::ffi::c_void;
use core::future::Future;
use core::task::Waker;
use core::time::Duration;
use core::{arch::asm, cell::RefCell};

use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::syst::SystClkSource;
use ssd1306::prelude::I2CInterface;
use stm32f4xx_hal::gpio::{AF2, Alternate, ExtiPin, Input, Output, Pin, gpioa, gpiob};

use stm32f4xx_hal::interrupt;
use stm32f4xx_hal::pac::{EXTI, NVIC, TIM2, TIM3};
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::syscfg::SysCfgExt;
use stm32f4xx_hal::time::Hertz;
use stm32f4xx_hal::timer::{PwmChannel, PwmExt, PwmManager, pwm};
use stm32f4xx_hal::{
    gpio::GpioExt,
    i2c::{I2c, Mode},
    pac::{self, I2C1},
    rcc::RccExt,
};

pub use embedded_hal::i2c;
pub use hts221;

use ssd1306::{
    I2CDisplayInterface, Ssd1306, mode::DisplayConfig, prelude::DisplayRotation,
    size::DisplaySize128x64,
};
/// Low level initialization. The low level initialization function will
/// perform basic low level initialization of the hardware.
/// TODO: How to make it generic to work with other boards?
/// Failure on this level should result in panic so we directly return the board
pub trait LowLevelInit {
    /// The input is the number of ticks per second that ThreadX will be
    /// expecting. The output is an initialized Board struct
    fn low_level_init(ticks_per_second: u32) -> Self;
}

// cortexm-rt crate defines the _stack_start function. Due to the action of flip-link, the stack pointer
// is moved lower down in memory after leaving space for the bss and data sections.
unsafe extern "C" {
    static _stack_start: u32;
}

pub type DisplayType<I2C> = Ssd1306<
    ssd1306::prelude::I2CInterface<I2C>,
    DisplaySize128x64,
    ssd1306::mode::BufferedGraphicsMode<DisplaySize128x64>,
>;
type TempSensorType<I2C> = hts221::HTS221<I2C, stm32f4xx_hal::i2c::Error>;

pub struct BoardMxAz3166<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    pub display: Option<DisplayType<I2C>>,
    pub temp_sensor: Option<TempSensorType<I2CBus>>,
    pub i2c_bus: Option<I2CBus>,
    pub btn_a: Option<InputButton<'A', 4>>,
    pub btn_b: Option<InputButton<'A', 10>>,
    pub pwm_red: PwmChannel<TIM3, 0>,
    pub pwm_green: PwmChannel<TIM3, 1>,
    pub pwm_blue: PwmChannel<TIM2, 1>,
}

#[derive(Clone, Copy)]
pub struct I2CBus {
    pub i2c: &'static Mutex<RefCell<Option<I2c<I2C1>>>>,
}
impl embedded_hal::i2c::ErrorType for I2CBus {
    type Error = stm32f4xx_hal::i2c::Error;
}

impl embedded_hal::i2c::I2c for I2CBus {
    fn transaction(
        &mut self,
        address: u8,
        operations: &mut [embedded_hal::i2c::Operation<'_>],
    ) -> Result<(), Self::Error> {
        cortex_m::interrupt::free(|cs| {
            let mut binding = self.i2c.borrow(cs).borrow_mut();
            let bus = binding.as_mut().unwrap();
            bus.transaction_slice(address, operations)
        })
    }
}

static SHARED_BUS: Mutex<RefCell<Option<I2c<I2C1>>>> = Mutex::new(RefCell::new(None));

impl LowLevelInit for BoardMxAz3166<I2CBus> {
    fn low_level_init(ticks_per_second: u32) -> BoardMxAz3166<I2CBus> {
        unsafe {
            let stack_start = &raw const _stack_start;
            threadx_sys::_tx_thread_system_stack_ptr = stack_start as *mut c_void;
            defmt::info!(
                "Low level init.  Stack at: {:x} Ticks per second:{}",
                stack_start.addr(),
                ticks_per_second
            );

            defmt::info!("Stack size {}", stack_start.wrapping_sub(0x2000_0000));
        }
        let p = pac::Peripherals::take().unwrap();

        let rcc = p.RCC.constrain();
        // Setup clocks. Reference (https://github.com/Eclipse-SDV-Hackathon-Chapter-Two/challenge-threadx-and-beyond/tree/main)
        let clocks = rcc
            .cfgr
            .sysclk(Hertz::MHz(96))
            .hclk(Hertz::MHz(96))
            .pclk1(Hertz::MHz(36))
            .pclk2(Hertz::MHz(64))
            .use_hse(Hertz::MHz(26))
            .freeze();

        let cp = cortex_m::Peripherals::take().unwrap();

        let mut syst = cp.SYST;
        let mut dcb = cp.DCB;
        dcb.enable_trace();
        let mut dbg = cp.DWT;
        // configures the system timer to trigger a SysTick exception every second
        dbg.enable_cycle_counter();

        syst.set_clock_source(SystClkSource::Core);
        syst.set_reload((96_000_000 / ticks_per_second) - 1);
        syst.enable_counter();
        syst.enable_interrupt();

        let gpioa = p.GPIOA.split();

        let mut syscfg = p.SYSCFG.constrain();
        let mut exti = p.EXTI;

        let mut button_a = gpioa.pa4.into_input();
        button_a.enable_interrupt(&mut exti);
        button_a.make_interrupt_source(&mut syscfg);
        button_a.clear_interrupt_pending_bit();
        button_a.trigger_on_edge(&mut exti, stm32f4xx_hal::gpio::Edge::RisingFalling);

        let mut button_b = gpioa.pa10.into_input();
        button_b.enable_interrupt(&mut exti);
        button_b.make_interrupt_source(&mut syscfg);
        button_b.clear_interrupt_pending_bit();
        button_b.trigger_on_edge(&mut exti, stm32f4xx_hal::gpio::Edge::RisingFalling);

        unsafe {
            NVIC::unmask(button_a.interrupt());
            NVIC::unmask(button_b.interrupt());
        }

        let gpiob = p.GPIOB.split();

        // Configure I2C1
        let scl = gpiob.pb8;
        let sda = gpiob.pb9;

        // RGB LED PWM Konfiguration für AZ3166 (STM32F412)
        // Pin-Zuordnung:
        // - Rot:  PB4 -> TIM3_CH1 (AF2)
        // - Blau: PB3 -> TIM2_CH2 (AF1)
        // - Grün: PB5 -> TIM3_CH2 (AF2)

        // GPIO Pins konfigurieren mit korrekten Alternate Functions
        let blue_pin = gpiob.pb3.into_alternate::<1>(); // TIM2_CH2 (AF1)
        let red_pin = gpiob.pb4.into_alternate::<2>(); // TIM3_CH1 (AF2)
        let green_pin = gpiob.pb5.into_alternate::<2>(); // TIM3_CH2 (AF2)

        // TIM2 für Blau (PB3)
        let (_, (_, pwm_blue_ch, _, _)) = p.TIM2.pwm_us(100.micros(), &clocks);
        let mut pwm_blue = pwm_blue_ch.with(blue_pin);
        
        // TIM3 für Rot und Grün (PB4, PB5)
        let (_, (pwm_red, pwm_green, _, _)) = p.TIM3.pwm_us(100.micros(), &clocks);
        let mut pwm_red = pwm_red.with(red_pin);
        let mut pwm_green = pwm_green.with(green_pin);

        // PWM-Kanäle aktivieren
        pwm_red.enable();
        pwm_green.enable();
        pwm_blue.enable();

        // Beispiel: Duty Cycle setzen (0-100%)
        let max_duty = pwm_red.get_max_duty();
        pwm_red.set_duty(max_duty / 2); // 50% Helligkeit für Rot
        pwm_green.set_duty(max_duty / 4); // 25% Helligkeit für Grün
        pwm_blue.set_duty(max_duty * 3 / 4); // 75% Helligkeit für Blau

        let i2c = I2c::new(p.I2C1, (scl, sda), Mode::standard(Hertz::kHz(400)), &clocks);
        cortex_m::interrupt::free(|cs| SHARED_BUS.borrow(cs).replace(Some(i2c)));
        let mut bus = I2CBus { i2c: &SHARED_BUS };
        defmt::info!("Low level init");

        let hts221 = hts221::Builder::new()
            .with_data_rate(hts221::DataRate::Continuous1Hz)
            .build(&mut bus)
            .unwrap();

        let interface: I2CInterface<I2CBus> = I2CDisplayInterface::new(bus);

        let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();

        //Set up the priorities for SysTick and PendSV and SVC
        unsafe {
            asm!(
                "MOV     r0, #0xE000E000",
                "LDR     r1, =0x00000000",
                "STR     r1, [r0, #0xD18]",
                "LDR     r1, =0xFF000000",
                "STR     r1, [r0, #0xD1C]",
                "LDR     r1, =0x40FF0000",
                "STR     r1, [r0, #0xD20]",
                out("r1") _,
                out("r0") _,
            );
        }
        defmt::info!("Int prio set");
        Self {
            display: Some(display),
            temp_sensor: Some(hts221),
            i2c_bus: Some(bus),
            btn_a: Some(InputButton::new(button_a)),
            btn_b: Some(InputButton::new(button_b)),
            pwm_red,
            pwm_green,
            pwm_blue,
        }
    }
}

pub struct InputButton<const P: char, const N: u8> {
    pin: Pin<P, N, Input>,
}

enum ButtonState {
    Pressed,
    Released,
}

impl<const P: char, const N: u8> InputButton<P, N> {
    pub const fn new(pin: Pin<P, N, Input>) -> Self {
        Self { pin }
    }

    pub fn is_high(&self) -> bool {
        self.pin.is_high()
    }
    pub fn is_low(&self) -> bool {
        self.pin.is_low()
    }

    pub async fn wait_for_button_pressed(&self) {
        InputButtonFuture::new(self, ButtonState::Pressed).await;
    }

    pub async fn wait_for_button_released(&self) {
        InputButtonFuture::new(self, ButtonState::Released).await;
    }
}

struct InputButtonFuture<'a, const P: char, const N: u8> {
    pin: &'a InputButton<P, N>,
    expected_button_state: ButtonState,
}

impl<'a, const P: char, const N: u8> InputButtonFuture<'a, P, N> {
    const fn new(pin: &'a InputButton<P, N>, expected_state: ButtonState) -> Self {
        InputButtonFuture {
            pin,
            expected_button_state: expected_state,
        }
    }
}

static BTN_WKER: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));
static BTN_B_WKER: Mutex<RefCell<Option<Waker>>> = Mutex::new(RefCell::new(None));

impl<const P: char, const N: u8> Future for InputButtonFuture<'_, P, N> {
    type Output = ();

    fn poll(
        self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<Self::Output> {
        let waker = cx.waker().clone();
        if N == 4 {
            cortex_m::interrupt::free(|cs| {
                BTN_WKER.borrow(cs).borrow_mut().replace(waker);
            });
        } else {
            cortex_m::interrupt::free(|cs| {
                BTN_B_WKER.borrow(cs).borrow_mut().replace(waker);
            });
        }
        match self.expected_button_state {
            ButtonState::Pressed => {
                if self.pin.is_low() {
                    return core::task::Poll::Ready(());
                }
            }
            ButtonState::Released => {
                if self.pin.is_high() {
                    return core::task::Poll::Ready(());
                }
            }
        }
        core::task::Poll::Pending
    }
}

pub enum BUTTONS {
    ButtonA = 4,
    ButtonB = 10,
}

/// .
#[interrupt]
fn EXTI4() {
    cortex_m::interrupt::free(|cs| {
        if let Some(wker) = BTN_WKER.borrow(cs).borrow_mut().as_ref() {
            wker.wake_by_ref();
        }
        unsafe {
            (*EXTI::ptr())
                .pr()
                .write(|w| w.bits(1 << BUTTONS::ButtonA as u32));
        };
    });
}

#[interrupt]
fn EXTI15_10() {
    cortex_m::interrupt::free(|cs| {
        if let Some(wker) = BTN_B_WKER.borrow(cs).borrow_mut().as_ref() {
            wker.wake_by_ref();
        }
        unsafe {
            (*EXTI::ptr())
                .pr()
                .write(|w| w.bits(1 << BUTTONS::ButtonB as u32));
        };
    });
}
