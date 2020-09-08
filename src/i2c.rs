use crate::executor;
use atomic::Atomic;
use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::{Context, Poll};
use cortex_m::asm;
use defmt::info;
use stm32f3::stm32f303::{interrupt, Interrupt, I2C1, NVIC};

pub enum Error {
    TooManyBytes,
    PeripheralError,
}

#[derive(Copy, Clone)]
enum HandlerContext {
    RegisterRead { addr: u8, reg: u8, n_bytes: u8 },
    RegisterWrite { addr: u8, reg: u8, n_bytes: u8 },
}

enum Operation<'a> {
    RegisterRead {
        addr: u8,
        reg: u8,
        n_bytes: u8,
        buff: &'a mut [u8],
    },
    RegisterWrite {
        addr: u8,
        reg: u8,
        n_bytes: u8,
        data: &'a [u8],
    },
}

enum FutureState {
    Init,
    Waiting,
    Completed,
}

struct I2CFuture<'a> {
    op: Operation<'a>,
    state: FutureState,
}

impl<'a> I2CFuture<'a> {
    pub fn new_read(op: Operation<'a>) -> I2CFuture<'a> {
        return I2CFuture {
            op,
            state: FutureState::Init,
        };
    }
}

impl<'a> Future for I2CFuture<'a> {
    type Output = Result<(), Error>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        match self.state {
            // Try to acquire the handler "lock".
            FutureState::Init => {
                if !HANDLER_IN_USE.swap(true, Ordering::Relaxed) {
                    match &self.op {
                        Operation::RegisterRead {
                            addr,
                            reg,
                            n_bytes,
                            buff,
                        } => {
                            if *n_bytes > unsafe { BUFFER.len() as u8 } {
                                self.state = FutureState::Completed;
                                return Poll::Ready(Err(Error::TooManyBytes));
                            }
                            HANDLER_CONTEXT.store(
                                Some(HandlerContext::RegisterRead {
                                    addr: *addr,
                                    reg: *reg,
                                    n_bytes: *n_bytes,
                                }),
                                Ordering::Relaxed,
                            );
                        }
                        Operation::RegisterWrite {
                            addr,
                            reg,
                            n_bytes,
                            data,
                        } => {
                            // If we are writing, the data to be written needs to be moved into the handler's buffer.
                            if *n_bytes <= unsafe { BUFFER.len() as u8 } {
                                for i in 0..*n_bytes as usize {
                                    unsafe { BUFFER[i] = data[i] };
                                }
                            } else {
                                self.state = FutureState::Completed;
                                return Poll::Ready(Err(Error::TooManyBytes));
                            }
                            HANDLER_CONTEXT.store(
                                Some(HandlerContext::RegisterWrite {
                                    addr: *addr,
                                    reg: *reg,
                                    n_bytes: *n_bytes,
                                }),
                                Ordering::Relaxed,
                            );
                        }
                    }
                    TRANSACTION_COMPLETE.store(false, Ordering::Relaxed);
                    self.state = FutureState::Waiting;
                    unsafe { NVIC::pend(Interrupt::I2C1_EV_EXTI23) }
                    return Poll::Pending;
                } else {
                    return Poll::Pending;
                }
            }
            FutureState::Waiting => {
                if TRANSACTION_COMPLETE.load(Ordering::Relaxed) {
                    match &mut self.op {
                        Operation::RegisterRead { n_bytes, buff, .. } => {
                            // If we are reading, the data to be written needs to be moved from the handler's buffer.
                            for i in 0..*n_bytes as usize {
                                buff[i] = unsafe { BUFFER[i] };
                            }
                        }
                        Operation::RegisterWrite { .. } => {}
                    }
                    self.state = FutureState::Completed;
                    HANDLER_IN_USE.store(false, Ordering::Relaxed);
                    cx.waker().clone().wake();
                    return Poll::Ready(Ok(()));
                } else {
                    return Poll::Pending;
                }
            }
            FutureState::Completed => panic!("Future polled after completion"),
        }
    }
}

static HANDLER_IN_USE: AtomicBool = AtomicBool::new(false);
static HANDLER_CONTEXT: Atomic<Option<HandlerContext>> = Atomic::new(None);
static TRANSACTION_COMPLETE: AtomicBool = AtomicBool::new(false);
static mut BUFFER: [u8; 16] = [0; 16];

pub fn init(I2C1: I2C1) {
    defmt::info!("Initialized");
    I2C1.timingr.write(|w| unsafe { w.bits(0x2000090E) });
    I2C1.cr1.write(|w| {
        w.tcie()
            .enabled()
            .stopie()
            .enabled()
            .txie()
            .enabled()
            .rxie()
            .enabled()
            .errie()
            .enabled()
            .pe()
            .enabled()
    });
    unsafe { NVIC::unmask(Interrupt::I2C1_EV_EXTI23) }
}

enum HandlerState {
    Init,
    ReadRegAddressed,
    WritingBytes,
    ReadingBytes,
    Stopped,
}

#[interrupt]
fn I2C1_EV_EXTI23() {
    static mut HANDLER_STATE: HandlerState = HandlerState::Init;
    static mut N_BYTES: u8 = 0;

    defmt::info!("Handler");
    // This is safe because only the EV or ER interrupts may manage the I2C peripheral.
    // and they are mutually exclusive (scheduled with equal priority).
    let I2C1 = unsafe { stm32f3::stm32f303::Peripherals::steal().I2C1 };
    let op = match HANDLER_CONTEXT.load(Ordering::Relaxed) {
        Some(op) => op,
        None => return,
    };

    match op {
        HandlerContext::RegisterRead { addr, reg, n_bytes } => match *HANDLER_STATE {
            HandlerState::Init => {
                I2C1.cr2.write(|w| {
                    I2C1.txdr.write(|w| w.txdata().bits(reg));
                    w.sadd()
                        .bits((addr << 1) as u16)
                        .autoend()
                        .software()
                        .rd_wrn()
                        .write()
                        .nbytes()
                        .bits(1)
                        .start()
                        .start()
                });
                *HANDLER_STATE = HandlerState::ReadRegAddressed;
            }
            HandlerState::ReadRegAddressed => {
                I2C1.cr2.write(|w| {
                    w.sadd()
                        .bits((addr << 1) as u16)
                        .rd_wrn()
                        .read()
                        .nbytes()
                        .bits(n_bytes)
                        .start()
                        .start()
                });
                *HANDLER_STATE = HandlerState::ReadingBytes;
            }
            HandlerState::WritingBytes => {
                panic!("WritingBytes reached but not writing");
            }
            HandlerState::ReadingBytes => {
                unsafe { BUFFER[*N_BYTES as usize] = I2C1.rxdr.read().rxdata().bits() };
                *N_BYTES += 1;
                if *N_BYTES == n_bytes {
                    I2C1.cr2.write(|w| w.stop().stop());
                    *N_BYTES = 0;
                    *HANDLER_STATE = HandlerState::Stopped;
                }
            }
            HandlerState::Stopped => {
                I2C1.icr.write(|w| w.stopcf().clear());
                HANDLER_CONTEXT.store(None, Ordering::Relaxed);
                TRANSACTION_COMPLETE.store(true, Ordering::Relaxed);
                executor::force_wakeup();
                *HANDLER_STATE = HandlerState::Init;
            }
        },
        HandlerContext::RegisterWrite { addr, reg, n_bytes } => {}
    }
}

/* Transitions:
Future created
Polls... if can claim HANDLER_IN_USE
    Sets HANDLER_CONTEXT
    Continues until TRANSACTION_COMPLETE == false
    If read, reads from BUFFER into buffer
    Clears HANDLER_IN_USE

*/

pub async fn read_register(addr: u8, reg: u8, buff: &mut [u8]) -> Result<(), Error> {
    if buff.len() > 255 {
        return Err(Error::TooManyBytes);
    } else {
        return I2CFuture::new_read(Operation::RegisterRead {
            addr,
            reg,
            n_bytes: buff.len() as u8,
            buff,
        })
        .await;
    }
}
