use cortex_m;
use defmt;

//#[defmt::timestamp]
//fn timestamp() -> u64 {
//    static TICK: AtomicUsize = AtomicUsize::new(0);
//    TICK.fetch_add(1, Ordering::Relaxed) as u64
//}

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    if let Some(loc) = info.location() {
        defmt::error!(
            "panicked at {:str}:{:u32}:{:u32}",
            loc.file(),
            loc.line(),
            loc.column()
        )
    } else {
        defmt::error!("panicked")
    }

    exit()
}

/// Terminates the application and makes `probe-run` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}