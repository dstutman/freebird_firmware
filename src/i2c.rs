use core::future::{Future};
use core::pin::Pin;
use core::task::{Context, Poll, Waker};
use core::ptr;
use stm32f3::stm32f303::{interrupt, NVIC, Interrupt};
use core::borrow::{Borrow, BorrowMut};
use core::sync::atomic::{AtomicPtr, Ordering};
use core::mem::transmute;

// TODO: Document all the internal contracts.
pub enum Error {
    TooManyBytes,
    PeripheralFailure
}

#[derive(Clone)]
enum Operation {
    PlainWrite { addr: u8, reg: u8, n_bytes: u8},
    PlainRead,
    RegisterRead,
    RegisterWrite,
}


#[derive(Clone)]
struct I2CFuture {
    op: Operation,
    success: bool,
    waker: Option<Waker>,
    handler_initialized: bool,
}

impl I2CFuture {
    pub fn new(op: Operation) -> I2CFuture {
        return I2CFuture {
            op,
            success: false,
            waker: None,
            handler_initialized: false,
        };
    }
}

impl Future for I2CFuture {
    type Output = Result<(), Error>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        // Safe because the transition to null is only in handler mode and we
        // are in thread mode. Also this is running in a single core system
        // so memory order isn't a problem.
        if unsafe { LIVE_OPERATION.is_null() } {
            if !self.handler_initialized {
                // WARNING: This is the least safe operation in this project. The LIVE_OPERATION static raw pointer
                // allows the interrupt to service and wake the I2CFuture. I believe this to be safe for the following
                // reason. LIVE_OPERATION is generally a ptr::null_mut(). The future is pinned, and cannot move in memory.
                // When an operation is requested, the future sets up the transfer by giving LIVE_OPERATION a static mutable
                // pointer to itself. It then kicks off the ISR. The ISR runs until it has completed or failed the transfer,
                // at which point it sets the executor to wake the Future, and sets the pointer back to null. In reality, the
                // actual required lifetime in the static is less than the period between the start of the Future, and the Future
                // returning Poll::Ready, but this cannot be expressed to the compiler.
                self.waker = Some(cx.waker().clone());
                unsafe{LIVE_OPERATION = transmute::<&mut Pin<&mut I2CFuture>, *mut Pin<&'static mut I2CFuture>>(&mut self)};
                self.handler_initialized = true;
                unsafe{
                    NVIC::unmask(Interrupt::I2C1_EV_EXTI23);
                    NVIC::pend(Interrupt::I2C1_EV_EXTI23);
                }
                return Poll::Pending
            } else {
                if self.success {
                    return Poll::Ready(Ok(()))
                } else {
                    return Poll::Ready(Err(Error::PeripheralFailure))
                }
            }
        } else {
            self.waker = Some(cx.waker().clone());
            return Poll::Pending
        }
    }
}

enum State {
    Init,
    RegAddressed,
    WritingBytes,
    ReadingBytes,
    Stopping,
}

// TODO: Make an atomic RWLock type
//struct LiveOperation {
//    waker: Waker,
//    operation: Operation,
//    success: bool
//}

// Oh no, null pointer we're all gonna die...
static mut LIVE_OPERATION: *mut Pin<&mut I2CFuture> = ptr::null_mut();
static mut LIVE: Option<I2CFuture> = None;
// State::Stopping for testing
static mut OPERATIONAL_STATE: State = State::Stopping;

#[interrupt]
fn I2C1_EV_EXTI23() {
    // This is safe because only this interrupt and the corresponding
    // error interrupt may modify OPERATIONAL_STATE, and they will never
    // interrupt each other (Not strictly impossible, but not a priority).
    // TODO: Further protections here if necessary,
    let state = unsafe { &OPERATIONAL_STATE };

    // This is safe when all the code in this module operates as required.
    // Specifically, LIVE_OPERATION must point to a valid I2CFuture before
    // the NVIC is used to trigger the I2C1_EV_EXTI23 interrupt. Further,
    // thread mode code may not mutate LIVE_OPERATION while it is non-null.
    let future = unsafe {LIVE_OPERATION.as_ref().unwrap()};

    match state {
        State::Stopping => {
            unsafe {
                OPERATIONAL_STATE = State::Init;
                future.waker.clone().unwrap().wake();
                LIVE_OPERATION = ptr::null_mut();
            }
        }
        _ => {}
    }
}

pub async fn read_register(addr: u8, reg: u8, data: &mut [u8]) -> Result<(), Error> {
    if data.len() > 255 {
        defmt::warn!("Tried to read more than 255 bytes over i2c");
        return Err(Error::TooManyBytes);
    } else {
        I2CFuture::new(Operation::RegisterRead).await;
        return Ok(());
    }
}

//pub async fn write_register(addr: u8, reg: u8, data: &[u8]) -> Result<(), Error> {}