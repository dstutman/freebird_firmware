use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::*;

use cortex_m::asm;

pub struct Executor {
    flag: AtomicBool,
}

static VTABLE: RawWakerVTable = {
    unsafe fn clone(data: *const ()) -> RawWaker {
        // Our job here is just to return a new `RawWaker`
        // with a pointer to the same flag as before.
        // We guarantee that the flag is `static` so
        // this is safe.
        return RawWaker::new(data, &VTABLE);
    }
    unsafe fn wake(data: *const ()) {
        // Because the flag is static
        // there is nothing dropped in `wake`. This
        // makes `wake_by_ref` and wake functionally
        // identical.
        wake_by_ref(data);
    }
    unsafe fn wake_by_ref(data: *const ()) {
        // Each task has a reference to a flag inside
        // the executor as its data.
        (*(data as *const AtomicBool)).store(true, Ordering::Relaxed);
    }
    unsafe fn drop(_: *const ()) {
        // Nothing to drop.
    }
    RawWakerVTable::new(clone, wake, wake_by_ref, drop)
};

impl Executor {
    pub fn new() -> Executor {
        return Executor {
            flag: AtomicBool::new(true),
        };
    }
    // Run the executor forever.
    pub fn run<T: Future<Output = ()>>(&self, mut future: T) -> ! {
        // This constructs the context for all futures running on this executor.
        // All futures share the same `AtomicBool` as data, so they can all share one
        // context.
        let raw_waker = RawWaker::new(&self.flag as *const _ as *const _, &VTABLE);
        let waker = unsafe { Waker::from_raw(raw_waker) };
        let mut context = Context::from_waker(&waker);
        loop {
            // Once a future has been moved to the `run` function it becomes
            // an infinite future. To uphold the `Pin` guarantees, it must never
            // move. We can guarantee this within `run` because we will never allow
            // it to escape.
            let infinite_future = unsafe {Pin::new_unchecked(&mut future)};
            if self.flag.swap(false, Ordering::Relaxed) {
                // Safe because infinite_future must live at least as long as `run` to be passed.
               // let mut pinned_infinity = unsafe { Pin::new_unchecked(&mut infinite_future) };
                if let Poll::Pending = infinite_future.poll(&mut context) {
                    // The system now waits for new input. New input must either trigger
                    // an interrupt, or wait for a `SysTick` exception to fire.
                    asm::wfi();
                } else {
                    panic!();
                }
            }
        }
    }
}
