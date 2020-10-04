#![no_std]
#![no_main]

use core::sync::atomic::{AtomicUsize, Ordering};

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::{self, asm};
use cortex_m_rt::{entry, exception};
use futures::future::join;
use stm32f3::stm32f303;

use crate::executor::Executor;
use futures::TryFutureExt;

use panic_rtt_target as _;
use rtt_target::{rprintln, rtt_init_print};
mod bmp;
mod executor;
mod i2c;
mod lsm;

static TICKS: AtomicUsize = AtomicUsize::new(0);

#[inline(always)]
fn get_ticks() -> usize {
    return TICKS.load(Ordering::Relaxed);
}

#[exception]
fn SysTick() {
    let _ = TICKS.fetch_add(1, Ordering::Relaxed);
}

#[entry]
fn main() -> ! {
    rtt_init_print!();
    let mut cp = stm32f303::CorePeripherals::take().unwrap();
    cp.SYST.set_clock_source(SystClkSource::Core);
    cp.SYST.set_reload(72000000 / 1000 - 1);
    cp.SYST.clear_current();
    cp.SYST.enable_counter();
    cp.SYST.enable_interrupt();

    let dp = stm32f303::Peripherals::take().unwrap();
    dp.FLASH.acr.write(|w| w.latency().ws2());
    while !dp.FLASH.acr.read().latency().is_ws2() {}
    dp.RCC.cr.write(|w| w.hsion().on());
    while !dp.RCC.cr.read().hsirdy().bit_is_set() {}
    dp.RCC
        .cfgr
        .write(|w| w.pllsrc().hsi_div_prediv().pllmul().mul9());
    dp.RCC.cr.write(|w| w.pllon().on());
    while !dp.RCC.cr.read().pllrdy().bit_is_set() {}
    dp.RCC.cfgr.write(|w| w.ppre1().div2());
    dp.RCC.cfgr.write(|w| w.sw().pll());
    while !dp.RCC.cfgr.read().sws().is_pll() {}

    dp.RCC.ahbenr.write(|w| w.iopben().enabled());
    let gpiob = dp.GPIOB;
    gpiob
        .moder
        .write(|w| w.moder8().alternate().moder9().alternate());
    gpiob
        .ospeedr
        .write(|w| w.ospeedr8().high_speed().ospeedr9().high_speed());
    gpiob
        .pupdr
        .write(|w| w.pupdr8().pull_up().pupdr9().pull_up());
    gpiob.afrh.write(|w| w.afrh8().af4().afrh9().af4());
    dp.RCC.apb1enr.write(|w| w.i2c1en().enabled());
    i2c::init(dp.I2C1);
    rprintln!("Starting executor...");
    let exec = Executor::new();
    exec.run(test_future());
}

async fn test_future() {
    //let lsm = lsm::Settings::default().init().await.unwrap();
    //loop {
    //    let mut xvg = 0.0;
    //    let mut yvg = 0.0;
    //    let mut zvg = 0.0;
    //    for _ in 0..1000 {
    //        let sample = lsm.acceleration().await.unwrap();
    //        xvg +=  sample.ax / 1000.0;
    //        yvg += sample.ay / 1000.0;
    //        zvg += sample.az / 1000.0;
    //    }
    //    rprintln!("Acc Sample: ({}, {}, {})", xvg, yvg, zvg);
    //}
    let lsm = lsm::Settings::default().init().await.unwrap();
    let bmp = bmp::Settings::default().init().await.unwrap();
    let mut last_ticks = 0;
    loop {
        if last_ticks - get_ticks() > 500 {
            last_ticks = get_ticks();
            asm::delay(8_000_000);
            let acc_sample = lsm.acceleration().await.unwrap();
            let rate_sample = lsm.angular_rate().await.unwrap();
            let bmp_sample = bmp.pressure_temperature().await.unwrap();
            rprintln!(
                "(ax, ay, az): ({}, {}, {}), (gx, gy, gz): ({}, {}, {}), (p, t): ({}, {})",
                acc_sample.ax,
                acc_sample.ay,
                acc_sample.az,
                rate_sample.gx,
                rate_sample.gy,
                rate_sample.gz,
                bmp_sample.press,
                bmp_sample.temp
            );
        }
    }
}
