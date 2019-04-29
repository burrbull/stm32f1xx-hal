//! # Direct Memory Access
#![allow(dead_code)]

use as_slice::{AsMutSlice, AsSlice};
use core::pin::Pin;
use core::marker::PhantomData;
use core::ops;
use core::sync::atomic::{self, Ordering};

use crate::rcc::AHB;

#[derive(Debug)]
pub enum Error {
    Overrun,
    #[doc(hidden)]
    _Extensible,
}

pub enum Event {
    HalfTransfer,
    TransferComplete,
}

#[derive(Clone, Copy, PartialEq)]
pub enum Half {
    First,
    Second,
}

/*
pub struct CircBuffer<BUFFER, CHANNEL>
where
    BUFFER: 'static,
{
    buffer: &'static mut [BUFFER; 2],
    channel: CHANNEL,
    readable_half: Half,
}

impl<BUFFER, CHANNEL> CircBuffer<BUFFER, CHANNEL> {
    pub(crate) fn new(buf: &'static mut [BUFFER; 2], chan: CHANNEL) -> Self {
        CircBuffer {
            buffer: buf,
            channel: chan,
            readable_half: Half::Second,
        }
    }
}
*/

pub trait DmaExt {
    type Channels;

    fn split(self, ahb: &mut AHB) -> Self::Channels;
}

pub trait Stop {
    /// Returns `true` if there's a transfer in progress
    fn stop(&mut self);
}

pub trait InProgress {
    fn in_progress() -> bool;
}

/// A DMA transfer
pub struct Transfer<BUFFER, PAYLOAD> where PAYLOAD: Stop {
    pub(crate) inner: Option<Inner<BUFFER, PAYLOAD>>,
}

pub(crate) struct Inner<BUFFER, PAYLOAD> {
    pub(crate) buffer: Pin<BUFFER>,
    pub(crate) payload: PAYLOAD,
}

impl<BUFFER, PAYLOAD> Drop for Transfer<BUFFER, PAYLOAD> where PAYLOAD: Stop {
    fn drop(&mut self) {
        if let Some(inner) = self.inner.as_mut() {
            // NOTE: this is a volatile write
            inner.payload.stop();

            // we need a read here to make the Acquire fence effective
            // we do *not* need this if `dma.stop` does a RMW operation
            unsafe {
                core::ptr::read_volatile(&0);
            }

            // we need a fence here for the same reason we need one in `Transfer.wait`
            atomic::compiler_fence(Ordering::Acquire);
        }
    }
}

impl<BUFFER, PAYLOAD, CHANNEL> Transfer<BUFFER, RxDma<PAYLOAD, CHANNEL>>
where
    RxDma<PAYLOAD, CHANNEL>: Stop,
    CHANNEL: InProgress,
{
    /// Returns `true` if the DMA transfer has finished
    pub fn is_done(&self) -> bool {
        !CHANNEL::in_progress()
    }
    pub fn wait(mut self) -> (Pin<BUFFER>, RxDma<PAYLOAD, CHANNEL>) {
        while self.is_done() {}

        atomic::compiler_fence(Ordering::Acquire);

        let inner = self
            .inner
            .take()
            .unwrap_or_else(|| unsafe { core::hint::unreachable_unchecked() });
        (inner.buffer, inner.payload)
    }
}

impl<BUFFER, PAYLOAD, CHANNEL> Transfer<BUFFER, TxDma<PAYLOAD, CHANNEL>>
where
    TxDma<PAYLOAD, CHANNEL>: Stop,
    CHANNEL: InProgress,
{
    /// Returns `true` if the DMA transfer has finished
    pub fn is_done(&self) -> bool {
        !CHANNEL::in_progress()
    }
    pub fn wait(mut self) -> (Pin<BUFFER>, TxDma<PAYLOAD, CHANNEL>) {
        while self.is_done() {}

        atomic::compiler_fence(Ordering::Acquire);

        let inner = self
            .inner
            .take()
            .unwrap_or_else(|| unsafe { core::hint::unreachable_unchecked() });
        (inner.buffer, inner.payload)
    }
}

macro_rules! dma {
    ($($DMAX:ident: ($dmaX:ident, $dmaXen:ident, $dmaXrst:ident, {
        $($CX:ident: (
            $chX:ident,
            $htifX:ident,
            $tcifX:ident,
            $chtifX:ident,
            $ctcifX:ident,
            $cgifX:ident
        ),)+
    }),)+) => {
        $(
            pub mod $dmaX {
                use crate::pac::{$DMAX, dma1};

                use crate::dma::{/*CircBuffer, */DmaExt, /*Error, */Event, /*Half, */RxDma, TxDma};
                use crate::rcc::AHB;

                pub struct Channels((), $(pub $CX),+);

                $(
                    /// A singleton that represents a single DMAx channel (channel X in this case)
                    ///
                    /// This singleton has exclusive access to the registers of the DMAx channel X
                    pub struct $CX { _0: () }

                    impl $CX {
                        /// Associated peripheral `address`
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        pub fn set_peripheral_address(&mut self, address: u32, inc: bool) {
                            self.ch().par.write(|w| w.pa().bits(address) );
                            self.ch().cr.modify(|_, w| w.pinc().bit(inc) );
                        }

                        /// `address` where from/to data will be read/write
                        ///
                        /// `inc` indicates whether the address will be incremented after every byte transfer
                        pub fn set_memory_address(&mut self, address: u32, inc: bool) {
                            self.ch().mar.write(|w| w.ma().bits(address) );
                            self.ch().cr.modify(|_, w| w.minc().bit(inc) );
                        }

                        /// Number of bytes to transfer
                        pub fn set_transfer_length(&mut self, len: usize) {
                            self.ch().ndtr.write(|w| w.ndt().bits(cast::u16(len).unwrap()));
                        }

                        /// Starts the DMA transfer
                        pub fn start(&mut self) {
                            self.ch().cr.modify(|_, w| w.en().set_bit() );
                        }

                        /// Stops the DMA transfer
                        pub fn stop(&mut self) {
                            self.ifcr().write(|w| w.$cgifX().set_bit());
                            self.ch().cr.modify(|_, w| w.en().clear_bit() );
                        }
                    }

                    impl super::InProgress for $CX {
                        fn in_progress() -> bool {
                            unsafe { (*$DMAX::ptr()).isr.read() }.$tcifX().bit_is_clear()
                        }
                    }

                    impl $CX {
                        pub fn listen(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => self.ch().cr.modify(|_, w| w.htie().set_bit()),
                                Event::TransferComplete => {
                                    self.ch().cr.modify(|_, w| w.tcie().set_bit())
                                }
                            }
                        }

                        pub fn unlisten(&mut self, event: Event) {
                            match event {
                                Event::HalfTransfer => {
                                    self.ch().cr.modify(|_, w| w.htie().clear_bit())
                                },
                                Event::TransferComplete => {
                                    self.ch().cr.modify(|_, w| w.tcie().clear_bit())
                                }
                            }
                        }

                        pub fn ch(&mut self) -> &dma1::CH {
                            unsafe { &(*$DMAX::ptr()).$chX }
                        }

                        pub fn isr(&self) -> dma1::isr::R {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { (*$DMAX::ptr()).isr.read() }
                        }

                        pub fn ifcr(&self) -> &dma1::IFCR {
                            unsafe { &(*$DMAX::ptr()).ifcr }
                        }

                        pub fn get_ndtr(&self) -> u32 {
                            // NOTE(unsafe) atomic read with no side effects
                            unsafe { &(*$DMAX::ptr())}.$chX.ndtr.read().bits()
                        }
                    }

/*
                    impl<B> CircBuffer<B, $CX> {
                        /// Peeks into the readable half of the buffer
                        pub fn peek<R, F>(&mut self, f: F) -> Result<R, Error>
                            where
                            F: FnOnce(&B, Half) -> R,
                        {
                            let half_being_read = self.readable_half()?;

                            let buf = match half_being_read {
                                Half::First => &self.buffer[0],
                                Half::Second => &self.buffer[1],
                            };

                            // XXX does this need a compiler barrier?
                            let ret = f(buf, half_being_read);


                            let isr = self.channel.isr();
                            let first_half_is_done = isr.$htifX().bit_is_set();
                            let second_half_is_done = isr.$tcifX().bit_is_set();

                            if (half_being_read == Half::First && second_half_is_done) ||
                                (half_being_read == Half::Second && first_half_is_done) {
                                Err(Error::Overrun)
                            } else {
                                Ok(ret)
                            }
                        }

                        /// Returns the `Half` of the buffer that can be read
                        pub fn readable_half(&mut self) -> Result<Half, Error> {
                            let isr = self.channel.isr();
                            let first_half_is_done = isr.$htifX().bit_is_set();
                            let second_half_is_done = isr.$tcifX().bit_is_set();

                            if first_half_is_done && second_half_is_done {
                                return Err(Error::Overrun);
                            }

                            let last_read_half = self.readable_half;

                            Ok(match last_read_half {
                                Half::First => {
                                    if second_half_is_done {
                                        self.channel.ifcr().write(|w| w.$ctcifX().set_bit());

                                        self.readable_half = Half::Second;
                                        Half::Second
                                    } else {
                                        last_read_half
                                    }
                                }
                                Half::Second => {
                                    if first_half_is_done {
                                        self.channel.ifcr().write(|w| w.$chtifX().set_bit());

                                        self.readable_half = Half::First;
                                        Half::First
                                    } else {
                                        last_read_half
                                    }
                                }
                            })
                        }
                    }

                    impl<BUFFER, PAYLOAD> Transfer<&'static mut BUFFER, RxDma<PAYLOAD, $CX>> {
                        pub fn peek<T>(&self) -> &[T]
                        where
                            BUFFER: AsRef<[T]>,
                        {
                            let pending = self.payload.channel.get_ndtr() as usize;

                            let slice = self.buffer.as_ref();
                            let capacity = slice.len();

                            &slice[..(capacity - pending)]
                        }
                    }
*/

                    impl<PAYLOAD> RxDma<PAYLOAD, $CX> {
                        pub fn start(&mut self) {
                            self.channel.start()
                        }
                        pub fn stop(&mut self) {
                            self.channel.stop()
                        }
                    }

                    impl<PAYLOAD> crate::dma::Stop for RxDma<PAYLOAD, $CX> {
                        fn stop(&mut self) {
                            self.stop()
                        }
                    }

                    impl<PAYLOAD> TxDma<PAYLOAD, $CX> {
                        pub fn start(&mut self) {
                            self.channel.start()
                        }
                        pub fn stop(&mut self) {
                            self.channel.stop()
                        }
                    }

                    impl<PAYLOAD> crate::dma::Stop for TxDma<PAYLOAD, $CX> {
                        fn stop(&mut self) {
                            self.stop()
                        }
                    }

                )+

                impl DmaExt for $DMAX {
                    type Channels = Channels;

                    fn split(self, ahb: &mut AHB) -> Channels {
                        ahb.enr().modify(|_, w| w.$dmaXen().set_bit());

                        // reset the DMA control registers (stops all on-going transfers)
                        $(
                            self.$chX.cr.reset();
                        )+

                        Channels((), $($CX { _0: () }),+)
                    }
                }
            }
        )+
    }
}

dma! {
    DMA1: (dma1, dma1en, dma1rst, {
        C1: (
            ch1,
            htif1, tcif1,
            chtif1, ctcif1, cgif1
        ),
        C2: (
            ch2,
            htif2, tcif2,
            chtif2, ctcif2, cgif2
        ),
        C3: (
            ch3,
            htif3, tcif3,
            chtif3, ctcif3, cgif3
        ),
        C4: (
            ch4,
            htif4, tcif4,
            chtif4, ctcif4, cgif4
        ),
        C5: (
            ch5,
            htif5, tcif5,
            chtif5, ctcif5, cgif5
        ),
        C6: (
            ch6,
            htif6, tcif6,
            chtif6, ctcif6, cgif6
        ),
        C7: (
            ch7,
            htif7, tcif7,
            chtif7, ctcif7, cgif7
        ),
    }),

    DMA2: (dma2, dma2en, dma2rst, {
        C1: (
            ch1,
            htif1, tcif1,
            chtif1, ctcif1, cgif1
        ),
        C2: (
            ch2,
            htif2, tcif2,
            chtif2, ctcif2, cgif2
        ),
        C3: (
            ch3,
            htif3, tcif3,
            chtif3, ctcif3, cgif3
        ),
        C4: (
            ch4,
            htif4, tcif4,
            chtif4, ctcif4, cgif4
        ),
        C5: (
            ch5,
            htif5, tcif5,
            chtif5, ctcif5, cgif5
        ),
    }),
}

/// DMA Receiver
pub struct RxDma<PAYLOAD, RXCH> {
    pub(crate) _payload: PhantomData<PAYLOAD>,
    pub(crate) channel: RXCH,
}

/// DMA Transmitter
pub struct TxDma<PAYLOAD, TXCH> {
    pub(crate) _payload: PhantomData<PAYLOAD>,
    pub(crate) channel: TXCH,
}

/// DMA Receiver/Transmitter
pub struct RxTxDma<PAYLOAD, RXCH, TXCH> {
    pub(crate) _payload: PhantomData<PAYLOAD>,
    pub(crate) rxchannel: RXCH,
    pub(crate) txchannel: TXCH,
}

pub trait Receive {
    type RxChannel;
    type TransmittedWord;
}

pub trait Transmit {
    type TxChannel;
    type ReceivedWord;
}

/*
pub trait CircReadDma<B, RS>: Receive
where
    B: AsMut<[RS]>,
    Self: core::marker::Sized,
{
    fn circ_read(self, buffer: &'static mut [B; 2]) -> CircBuffer<B, Self::RxChannel>;
}
*/

pub trait ReadDma<B>: Receive + Stop
where
    B: ops::DerefMut + 'static,
    B::Target: AsMutSlice<Element = Self::TransmittedWord> + Unpin,
    Self: core::marker::Sized,
{
    /// Receives data into the given `buffer` until it's filled
    ///
    /// Returns a value that represents the in-progress DMA transfer
    fn read(self, buffer: Pin<B>) -> Transfer<B, Self>;
}

pub trait WriteDma<B>: Transmit + Stop
where
    B: ops::Deref + 'static,
    B::Target: AsSlice<Element = Self::ReceivedWord>,
    Self: core::marker::Sized,
{
    /// Sends out the given `buffer`
    ///
    /// Returns a value that represents the in-progress DMA transfer
    fn write(self, buffer: Pin<B>) -> Transfer<B, Self>;
}
