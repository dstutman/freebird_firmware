use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::*;

use cortex_m::asm;

static flag: AtomicBool = AtomicBool::new(true);

pub struct Executor;

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
pub fn force_wakeup() {
    flag.store(true, Ordering::Relaxed);
}
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
                // let mut pinned_infinity = unsafe { Pin::new_unchecked(&mut infinite_future) };
                if let Poll::Pending = infinite_future.poll(&mut context) {
                    // The system now waits for new input. New input must either trigger
                    // an interrupt, or wait for a `SysTick` exception to fire.
                    if flag.load(Ordering::Relaxed) {
                        continue;
                    }
                    asm::wfe()
                } else {
                    panic!();
                }
            }
        }
    }
}
