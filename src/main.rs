//! examples/init.rs

#![deny(unsafe_code)]
#![no_main]
#![no_std]

use rtic::app;
use panic_rtt_target as _;


#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [TIM2])]
mod app {
    use am2320::Am2320;
    use rtt_target::{rprintln, rtt_init_print};
    use stm32f1xx_hal::{pac, prelude::*, i2c::{BlockingI2c, DutyCycle}, timer::{Timer, Tim2NoRemap, Tim2PartialRemap2, Timer2, Channel}, device::{TIM2, TIM3, TIM1}};
    use systick_monotonic::Systick;

    const CLOCK_HZ: u32 = 72_000_000;

    #[shared]
    struct Shared {
        am2320: Am2320<BlockingI2c<stm32f1xx_hal::device::I2C1, (stm32f1xx_hal::gpio::Pin<'B', 6, stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>>, stm32f1xx_hal::gpio::Pin<'B', 7, stm32f1xx_hal::gpio::Alternate<stm32f1xx_hal::gpio::OpenDrain>>)>, stm32f1xx_hal::timer::Delay<stm32f1xx_hal::device::TIM3, 1000000>>,
    }

    #[local]
    struct Local {
        led_pwm: stm32f1xx_hal::timer::PwmHz<TIM2, Tim2NoRemap, (stm32f1xx_hal::timer::Ch<0>, stm32f1xx_hal::timer::Ch<1>, stm32f1xx_hal::timer::Ch<2>, stm32f1xx_hal::timer::Ch<3>), (stm32f1xx_hal::gpio::Pin<'A', 0, stm32f1xx_hal::gpio::Alternate>, stm32f1xx_hal::gpio::Pin<'A', 1, stm32f1xx_hal::gpio::Alternate>, stm32f1xx_hal::gpio::Pin<'A', 2, stm32f1xx_hal::gpio::Alternate>, stm32f1xx_hal::gpio::Pin<'A', 3, stm32f1xx_hal::gpio::Alternate>)>,
        out_delay: stm32f1xx_hal::timer::Delay<TIM1, 1000000> ,
    }

    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<1000>;


    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Cortex-M peripherals (Core Peripherals)
        let mut cp = cx.core;

        // Device specific peripherals (Peripheral Access Crate)
        let pac = cx.device;

        // Setup RTT for logging
        rtt_init_print!();
        rprintln!("Hello, world!");

        // Clock setup
        let mut flash = pac.FLASH.constrain();
        let mut rcc = pac.RCC.constrain();
        let clocks = rcc
        .cfgr.use_hse(8.MHz())  // use external oscillator (8 MHz)
        .sysclk(72.MHz())  // system clock, PLL multiplier should be 6
        .hclk(8.MHz())     // clock used for timers
        .freeze(&mut flash.acr);

        let mut gpioa = pac.GPIOA.split();
        let mut afio = pac.AFIO.constrain();
        // led
        let p1 = gpioa.pa0.into_alternate_push_pull(&mut gpioa.crl);
        let p2 = gpioa.pa1.into_alternate_push_pull(&mut gpioa.crl);
        let p3 = gpioa.pa2.into_alternate_push_pull(&mut gpioa.crl);
        let p4 = gpioa.pa3.into_alternate_push_pull(&mut gpioa.crl);

        let pins = (p1, p2, p3, p4);

        let mut led_pwm: stm32f1xx_hal::timer::PwmHz<TIM2, Tim2NoRemap, (stm32f1xx_hal::timer::Ch<0>, stm32f1xx_hal::timer::Ch<1>, stm32f1xx_hal::timer::Ch<2>, stm32f1xx_hal::timer::Ch<3>), (stm32f1xx_hal::gpio::Pin<'A', 0, stm32f1xx_hal::gpio::Alternate>, stm32f1xx_hal::gpio::Pin<'A', 1, stm32f1xx_hal::gpio::Alternate>, stm32f1xx_hal::gpio::Pin<'A', 2, stm32f1xx_hal::gpio::Alternate>, stm32f1xx_hal::gpio::Pin<'A', 3, stm32f1xx_hal::gpio::Alternate>)> = pac
        .TIM2
        .pwm_hz::<Tim2NoRemap, _, _>(pins, &mut afio.mapr, 1000.Hz(), &clocks);

        // Enable clock on each of the channels
        led_pwm.enable(stm32f1xx_hal::timer::Channel::C1);
        let max_duty = led_pwm.get_max_duty();
        led_pwm.set_duty(Channel::C1, max_duty);

        rprintln!("max duty - {}", led_pwm.get_max_duty());

        let mut delay = pac.TIM3.delay_us(&clocks);
        let mut out_delay = pac.TIM1.delay_us(&clocks);
               
        // Acquire the GPIOB peripheral
        let mut gpiob = pac.GPIOB.split();

        let am2320_scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
        let am2320_sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);

        let am2320_i2c = BlockingI2c::i2c1(
            pac.I2C1,
            (am2320_scl, am2320_sda),
            &mut afio.mapr,
            stm32f1xx_hal::i2c::Mode::Standard {
                frequency: 100.kHz(),
            },
            clocks,
            1000,
            10,
            1000,
            1000,
        );

        let mut am2320 = Am2320::new(am2320_i2c, delay);


        // Initialize (enable) the monotonic timer (CYCCNT)
        cp.DCB.enable_trace();
        cp.DWT.enable_cycle_counter();

        let mono = Systick::new(cp.SYST, CLOCK_HZ);

        (
            Shared { am2320 },
            Local { led_pwm, out_delay
            },
            init::Monotonics(mono),
        )
    }

    #[idle(local=[led_pwm, out_delay], shared=[am2320])]
    fn idle(mut cx: idle::Context) -> ! {
        let mut pwm_counter: u16 = 0;
        let pwm = cx.local.led_pwm;
        let out_delay = cx.local.out_delay;
        let mut is_up = true;
        loop{
            if is_up {
                pwm_counter += 50;
                if pwm_counter > pwm.get_max_duty() {
                    is_up = false;
                }
            } else {
                pwm_counter -= 50;
                if pwm_counter < 1 {
                    is_up = true;
                }
            }
            pwm.set_duty(Channel::C1, pwm_counter);
            rprintln!("duty - {}", pwm_counter);

            let data = cx.shared.am2320.lock(|am2320| {
                am2320.read()
            });
            
            let measurement = data.expect("can't get measurement");
            rprintln!("{:?}", measurement);
            out_delay.delay_ms(500_u16); // TODO timer interrupt for read data from am2320
            
        }
    }

}
