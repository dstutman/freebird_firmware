#![no_std]
#![no_main]

mod defmt_config;

use cortex_m::{self, asm};
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m_rt::{entry, exception};
use stm32f3::stm32f303;
use defmt;
use defmt_rtt;

use core::sync::atomic::{AtomicUsize, Ordering};

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
    // SysTick setup
    let mut syst = cortex_m::Peripherals::take().unwrap().SYST;
    syst.set_clock_source(SystClkSource::Core);
    // Internal 8 MHz HSI by default
    syst.set_reload(8_000_000 - 1);
    syst.clear_current();
    syst.enable_counter();
    syst.enable_interrupt();

    defmt::info!("Entering loop");
    loop {asm::nop()};
}
