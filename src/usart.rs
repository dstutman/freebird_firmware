// TODO: Really need error detection and/or handling.

use core::future::Future;
use core::pin::Pin;
use core::sync::atomic::{AtomicBool, Ordering};
use core::task::{Context, Poll};

use atomic::Atomic;
use cortex_m::asm;
use stm32f3::stm32f303::{interrupt, Interrupt, I2C1, NVIC, USART2};

use rtt_target::rprintln;

use crate::executor;

#[derive(Debug)]
pub enum USARTError {
    TooManyBytes,
    PeripheralError,
}

#[derive(Copy, Clone)]
enum HandlerContext {
    //    MessageRead { n_bytes: u8 },
    MessageWrite { n_bytes: u8 },
}

enum Operation<'a> {
    MessageWrite { n_bytes: u8, data: &'a [u8] },
    //    MessageRead {
    //        n_bytes: u8,
    //        buff: &'a mut [u8],
    //    }
}

enum FutureState {
    Init,
    Waiting,
    Completed,
}

struct USARTFuture<'a> {
    op: Operation<'a>,
    state: FutureState,
}

impl<'a> USARTFuture<'a> {
    pub fn new(op: Operation<'a>) -> USARTFuture<'a> {
        return USARTFuture {
            op,
            state: FutureState::Init,
        };
    }
}

impl<'a> Future for USARTFuture<'a> {
    type Output = Result<(), USARTError>;

    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        match self.state {
            // Try to acquire the handler "lock".
            FutureState::Init => {
                if !HANDLER_IN_USE.swap(true, Ordering::Relaxed) {
                    match &self.op {
                        //Operation::MessageRead {
                        //    n_bytes,
                        //    buff,
                        //} => {
                        //    if *n_bytes > unsafe { BUFFER.len() as u8 } {
                        //        self.state = FutureState::Completed;
                        //        return Poll::Ready(Err(USARTError::TooManyBytes));
                        //    }
                        //    HANDLER_CONTEXT.store(
                        //        Some(HandlerContext::RegisterRead {
                        //            n_bytes: *n_bytes,
                        //        }),
                        //        Ordering::Relaxed,
                        //    );
                        //}
                        Operation::MessageWrite { n_bytes, data } => {
                            // If we are writing, the data to be written needs to be moved into the handler's buffer.
                            if *n_bytes <= unsafe { BUFFER.len() as u8 } {
                                for i in 0..*n_bytes as usize {
                                    unsafe { BUFFER[i] = data[i] };
                                }
                            } else {
                                self.state = FutureState::Completed;
                                return Poll::Ready(Err(USARTError::TooManyBytes));
                            }
                            HANDLER_CONTEXT.store(
                                Some(HandlerContext::MessageWrite { n_bytes: *n_bytes }),
                                Ordering::Relaxed,
                            );
                        }
                    }
                    TRANSACTION_COMPLETE.store(false, Ordering::Relaxed);
                    self.state = FutureState::Waiting;
                    unsafe { NVIC::pend(Interrupt::USART2_EXTI26) }
                    return Poll::Pending;
                } else {
                    return Poll::Pending;
                }
            }
            FutureState::Waiting => {
                if TRANSACTION_COMPLETE.load(Ordering::Relaxed) {
                    match &mut self.op {
                        //Operation::RegisterRead { n_bytes, buff, .. } => {
                        //    // If we are reading, the data to be written needs to be moved from the handler's buffer.
                        //    for i in 0..*n_bytes as usize {
                        //        buff[i] = unsafe { BUFFER[i] };
                        //    }
                        //}
                        Operation::MessageWrite { .. } => {}
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
// TODO: This can just be an AtomicU8 later
static HANDLER_CONTEXT: Atomic<Option<HandlerContext>> = Atomic::new(None);
static TRANSACTION_COMPLETE: AtomicBool = AtomicBool::new(false);
const MAX_ALLOWABLE_BYTES: usize = 128;
static mut BUFFER: [u8; MAX_ALLOWABLE_BYTES] = [0; MAX_ALLOWABLE_BYTES];

enum HandlerState {
    Init,
    WritingBytes,
    //    ReadingBytes,
    Stopped,
}

#[interrupt]
fn USART2_EXTI26() {
    static mut HANDLER_STATE: HandlerState = HandlerState::Init;
    static mut N_BYTES: u8 = 0;

    // This is safe because only the EV or ER interrupts may manage the I2C peripheral.
    // and they are mutually exclusive (scheduled with equal priority).
    let USART2 = unsafe { stm32f3::stm32f303::Peripherals::steal().USART2 };
    let op = match HANDLER_CONTEXT.load(Ordering::Relaxed) {
        Some(op) => op,
        None => return,
    };

    match op {
        //        HandlerContext::MessageRead { n_bytes } => match *HANDLER_STATE {
        //            HandlerState::Init => {
        //                *N_BYTES = 0;
        //                USART2.brr.write(|w| w.brr().);
        //                USART2.cr2.write(|w| w.stop().stop1());
        //                USART2.cr1.write(|w| w.m().bit8().txeie().enabled().te().enabled());
        //                *HANDLER_STATE = HandlerState::ReadRegAddressed;
        //            }
        //            HandlerState::WritingBytes => {
        //                panic!("WritingBytes reached but not writing");
        //            }
        //            HandlerState::ReadingBytes => {
        //                unsafe { BUFFER[*N_BYTES as usize] = I2C1.rxdr.read().rxdata().bits() };
        //                *N_BYTES += 1;
        //                if *N_BYTES == n_bytes {
        //                    *HANDLER_STATE = HandlerState::Stopped;
        //                }
        //            }
        //            HandlerState::Stopped => {
        //                I2C1.icr.write(|w| w.stopcf().clear());
        //                HANDLER_CONTEXT.store(None, Ordering::Relaxed);
        //                TRANSACTION_COMPLETE.store(true, Ordering::Relaxed);
        //                executor::force_wakeup();
        //                *HANDLER_STATE = HandlerState::Init;
        //            }
        //        },
        HandlerContext::MessageWrite { n_bytes } => match *HANDLER_STATE {
            HandlerState::Init => {
                *N_BYTES = 0;
                USART2.cr1.modify(|_, w| w.txeie().enabled());
                USART2
                    .tdr
                    .write(|w| unsafe { w.tdr().bits(BUFFER[*N_BYTES as usize] as u16) });
                *N_BYTES += 1;
                if *N_BYTES == n_bytes {
                    *HANDLER_STATE = HandlerState::Stopped;
                }
                *HANDLER_STATE = HandlerState::WritingBytes;
            }
            //            HandlerState::ReadRegAddressed => panic!("ReadRegAddressed reached but not reading"),
            HandlerState::WritingBytes => {
                USART2
                    .tdr
                    .write(|w| unsafe { w.tdr().bits(BUFFER[*N_BYTES as usize] as u16) });
                *N_BYTES += 1;
                if *N_BYTES == n_bytes {
                    *HANDLER_STATE = HandlerState::Stopped;
                }
            }
            //            HandlerState::ReadingBytes => panic!("ReadingBytes reached but not reading"),
            HandlerState::Stopped => {
                // NOTE: Does not ensure last bit was actually sent
                // if in future this will shutdown the peripheral (for power saving, for example)
                // ensure TC before TE disable
                USART2.cr1.modify(|_, w| w.txeie().disabled());
                HANDLER_CONTEXT.store(None, Ordering::Relaxed);
                TRANSACTION_COMPLETE.store(true, Ordering::Relaxed);
                executor::force_wakeup();
                *HANDLER_STATE = HandlerState::Init;
            }
        },
    }
}

pub fn init(USART2: USART2) {
    USART2.brr.write(|w| w.brr().bits(625));
    USART2.cr2.write(|w| w.stop().stop1());
    USART2.cr1.write(|w| {
        w.m()
            .bit8()
            .over8()
            .oversampling16()
            .te()
            .set_bit()
            .ue()
            .enabled()
    });
    unsafe { NVIC::unmask(Interrupt::USART2_EXTI26) }
}

//pub async fn read_message(buff: &mut [u8]) -> Result<(), USARTError> {
//    if buff.len() > 255 {
//        return Err(USARTError::TooManyBytes);
//    } else {
//        return USARTError::new(Operation::RegisterRead {
//            addr,
//            reg,
//            n_bytes: buff.len() as u8,
//            buff,
//        })
//            .await;
//    }
//}

pub async fn write_message(data: &[u8]) -> Result<(), USARTError> {
    if data.len() > MAX_ALLOWABLE_BYTES {
        return Err(USARTError::TooManyBytes);
    } else {
        return USARTFuture::new(Operation::MessageWrite {
            n_bytes: data.len() as u8,
            data,
        })
        .await;
    }
}
