#![no_std]
#![no_main]

use core::sync::atomic::{AtomicUsize, Ordering};

use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::{self, asm};
use cortex_m_rt::{entry, exception};
use defmt;
use defmt_rtt;
use futures::future::join;
use stm32f3::stm32f303;

use crate::executor::Executor;
use futures::TryFutureExt;

mod defmt_config;
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

#[defmt::timestamp]
fn timestamp() -> u64 {
    return (get_ticks() * 1000) as u64;
}

#[entry]
fn main() -> ! {
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
    defmt::info!("Entering loop");
    let exec = Executor::new();
    exec.run(test_future());
}

async fn test_future() {
    let lsm = lsm::Settings::default().init().await.unwrap();
    loop {
        let sample = lsm.acceleration().await.unwrap();
        defmt::info!("({:f32}, {:f32}, {:f32})", sample.ax, sample.ay, sample.az);
    }
}
