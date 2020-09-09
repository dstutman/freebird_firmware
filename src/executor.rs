use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::*;

use cortex_m::asm;

static flag: AtomicBool = AtomicBool::new(true);

pub struct Executor;

// This is required by the task API,
// but is basically useless in Avum's case.
static VTABLE: RawWakerVTable = {
    unsafe fn clone(data: *const ()) -> RawWaker {
        return RawWaker::new(data, &VTABLE);
    }
    unsafe fn wake(data: *const ()) {
        wake_by_ref(data);
    }
    unsafe fn wake_by_ref(_: *const ()) {
        flag.store(true, Ordering::Relaxed);
    }
    unsafe fn drop(_: *const ()) {}
    RawWakerVTable::new(clone, wake, wake_by_ref, drop)
};

impl Executor {
    pub fn new() -> Executor {
        return Executor;
    }

    // Run the executor forever.
    pub fn run<T: Future<Output = ()>>(&self, mut future: T) -> ! {
        // This constructs the context for all futures running on this executor.
        // All futures share the same `AtomicBool` as data, so they can all share one
        // context.
        let raw_waker = RawWaker::new(&() as *const _ as *const _, &VTABLE);
        let waker = unsafe { Waker::from_raw(raw_waker) };
        let mut context = Context::from_waker(&waker);
        loop {
            // Once a future has been moved to the `run` function it becomes
            // an infinite future. To uphold the `Pin` guarantees, it must never
            // move. We can guarantee this within `run` because we will never allow
            // it to escape.
            let infinite_future = unsafe { Pin::new_unchecked(&mut future) };
            if flag.swap(false, Ordering::Relaxed) {
                if let Poll::Pending = infinite_future.poll(&mut context) {
                    // If a driver signaled completion during the poll, we run
                    // again to ensure any tasks waiting on that driver can begin
                    // executing without waiting for the next `SysTick` exception.
                    if flag.load(Ordering::Relaxed) {
                        continue;
                    }
                    // The system now waits for new input. New input must either trigger
                    // an interrupt, or wait for a `SysTick` exception to fire.
                    asm::wfe()
                } else {
                    panic!();
                }
            }
        }
    }
}

pub fn force_wakeup() {
    flag.store(true, Ordering::Relaxed);
}
