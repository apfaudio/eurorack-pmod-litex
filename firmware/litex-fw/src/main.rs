#![no_std]
#![no_main]
#![allow(dead_code)]

/// Simple example of Rust firmware to do real-time audio DSP on
/// a LiteX-created VexRiscV SMP SoC. Assumes a single core.
///
/// This example implements a fixed-point low-pass filter of which
/// you can see the implementation in `dsp.rs`.
///
/// - Uses RISCV-PLIC (platform-level interrupt controller) to
///   claim and handle interrupts. This makes it easy to scale to
///   more cores as SMP VexRISCV only supports PLIC by default.
/// - Static buffers are managed by the 'DMA router' which is a separate
///   LiteX peripheral shuffling data back/forth to eurorack-pmod.
/// - IRQs fire when the buffer is half-full AND full such that the
///   ISR can process audio samples in real-time without glitches.
///

use heapless::*;
use litex_hal::prelude::*;
use litex_hal::uart::UartError;
use litex_pac as pac;
use litex_sys;
use riscv_rt::entry;
use riscv;
use core::arch::asm;
use aligned_array::{Aligned, A4};

use bresenham::Bresenham;

mod log;
mod plic;
mod dsp;
use log::*;
use plic::*;
use dsp::*;

/// Number of channels per section (4x input, 4x output)
const N_CHANNELS: usize = 4;

/// Some global state to track how long IRQs are taking to service.
static mut LAST_IRQ: u32 = 0;
static mut LAST_IRQ_LEN: u32 = 0;
static mut LAST_IRQ_PERIOD: u32 = 0;

static mut RANDOM: u32 = 92;

// Map the RISCV IRQ PLIC onto the fixed address present in the VexRISCV implementation.
// TODO: ideally fetch this from the svf, its currently not exported by `svd2rust`!
riscv::plic_context!(PLIC0, 0xf0c00000, 0, VexInterrupt, VexPriority);

const FB_SIZE_X: usize = 720;
const FB_SIZE_Y: usize = 720;
const FB_XOFFS: usize = 80;
static mut FB: Aligned<A4, [u8; FB_SIZE_X*FB_SIZE_Y]> = Aligned([0u8; FB_SIZE_X*FB_SIZE_Y]);

// Create the HAL bindings for the remaining LiteX peripherals.

litex_hal::uart! {
    Uart: litex_pac::UART,
}

litex_hal::timer! {
    Timer: litex_pac::TIMER0,
}

/// Handler for ALL IRQs.
#[export_name = "DefaultHandler"]
unsafe fn irq_handler() {
    // First, claim the IRQ for this core.
    // This should only ever fail if we have multiple cores claiming irqs
    // in an SMP environment (which is not the case for this simple example).
    let pending_irq = PLIC0::claim().unwrap();
    let peripherals = pac::Peripherals::steal();

    // Keep track of how long we spend in IRQs
    peripherals.TIMER0.uptime_latch().write(|w| w.bits(1));
    let trace = peripherals.TIMER0.uptime_cycles0().read().bits();
    LAST_IRQ_PERIOD = trace - LAST_IRQ;
    LAST_IRQ = trace;

    match pending_irq.pac_irq {
        pac::Interrupt::EURORACK_PMOD0 => {

            let pending_subtype = peripherals.EURORACK_PMOD0.ev_pending().read().bits();

            while peripherals.EURORACK_PMOD0.rlevel().read().bits() > 8 {
                let rdat = core::ptr::read_volatile(0xb100_0000 as *mut u32);
                let mut ch0raw = rdat as i16;
                let mut ch1raw = (rdat >> 16) as i16;

                /*
                RANDOM ^= RANDOM << 13;
                RANDOM ^= RANDOM >> 17;
                RANDOM ^= RANDOM << 5;
                let ch0raw = RANDOM as i16;
                let ch1raw = (RANDOM >> 16) as i16;
                */

                let ix: isize = (FB_SIZE_Y/2 - FB_XOFFS + ((ch0raw as i32 * (FB_SIZE_X) as i32) >> 16) as usize) as isize;
                let iy: isize = (FB_SIZE_X/2 + ((ch1raw as i32 * (FB_SIZE_Y) as i32) >> 16) as usize) as isize;

                if iy > 0 && iy < FB_SIZE_Y as isize {
                    let fb_ix = (FB_SIZE_Y*(iy as usize)) + (ix as usize);
                    FB[fb_ix] = 0xFF;
                    /*
                    if FB[fb_ix] < (0xFF - 64) {
                        FB[fb_ix] += 64;
                    }
                    */
                }
            }

            peripherals.EURORACK_PMOD0.ev_pending().write(|w| w.bits(pending_subtype));
        },
        _ => {
            // We shouldn't have any other types of IRQ...
        }
    }


    PLIC0::complete(pending_irq);

    peripherals.TIMER0.uptime_latch().write(|w| w.bits(1));
    let trace_end = peripherals.TIMER0.uptime_cycles0().read().bits();
    LAST_IRQ_LEN = trace_end - trace;
}


#[entry]
fn main() -> ! {

    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    log::info!("hello from litex-fw!");

    let mut timer = Timer::new(peripherals.TIMER0, litex_sys::CONFIG_CLOCK_FREQUENCY);

    unsafe {
        peripherals.EURORACK_PMOD0.ev_enable().write(|w| w.almost_full().bit(true));

        // RISC-V PLIC configuration.
        let mut plic = PLIC0::new();
        let dma_irq = VexInterrupt::from(pac::Interrupt::EURORACK_PMOD0);
        plic.set_threshold(VexPriority::from(0));
        plic.set_priority(dma_irq, VexPriority::from(1));
        plic.enable_interrupt(dma_irq);

        peripherals.VIDEO_FRAMEBUFFER.dma_base().write(|w| w.bits(FB.as_mut_ptr() as u32));
        peripherals.VIDEO_FRAMEBUFFER.dma_enable().write(|w| w.bits(1u32));

        // Enable machine external interrupts (basically everything added on by LiteX).
        riscv::register::mie::set_mext();

        // Finally enable interrupts.
        riscv::interrupt::enable();
    }

    loop {
        unsafe {
            for p in 0..(FB_SIZE_X*FB_SIZE_Y) {
                FB[p] >>= 1;
            }
        }

    }
}
