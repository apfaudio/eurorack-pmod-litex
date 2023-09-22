#![no_std]
#![no_main]
#![allow(dead_code)]

use heapless::*;
use litex_hal::prelude::*;
use litex_hal::uart::UartError;
use litex_pac as pac;
use riscv_rt::entry;
use core::sync::atomic::fence;
use core::sync::atomic::compiler_fence;
use core::sync::atomic::Ordering;

mod log;
use log::*;

const SYSTEM_CLOCK_FREQUENCY: u32 = 50_000_000;

litex_hal::uart! {
    Uart: litex_pac::UART,
}

litex_hal::timer! {
    Timer: litex_pac::TIMER0,
}

const BUF_SZ_WORDS: usize = 32;
const BUF_SZ_BYTES: usize = BUF_SZ_WORDS * 4;

static mut BUF_OUT: [u32; BUF_SZ_WORDS] = [0; BUF_SZ_WORDS];
static mut BUF_IN: [u32; BUF_SZ_WORDS] = [0; BUF_SZ_WORDS];


#[entry]
fn main() -> ! {
    let peripherals = unsafe { pac::Peripherals::steal() };

    log::init(peripherals.UART);
    log::info!("hello from litex-fw!");

    let mut elapsed: f32 = 0.0f32;

    let mut timer = Timer::new(peripherals.TIMER0, SYSTEM_CLOCK_FREQUENCY);

    for i in 0..BUF_SZ_WORDS {
        unsafe {
            BUF_OUT[i] = i as u32;
        }
    }

    unsafe {
        peripherals.DMA_READER0.base0.write(|w| w.bits(BUF_OUT.as_ptr() as u32));
        peripherals.DMA_READER0.length.write(|w| w.bits(BUF_SZ_BYTES as u32));
        peripherals.DMA_READER0.loop_.write(|w| w.bits(1u32));
        peripherals.DMA_READER0.enable.write(|w| w.bits(1u32));

        peripherals.DMA_WRITER0.base0.write(|w| w.bits(BUF_IN.as_mut_ptr() as u32));
        peripherals.DMA_WRITER0.length.write(|w| w.bits(BUF_SZ_BYTES as u32));
        peripherals.DMA_WRITER0.loop_.write(|w| w.bits(1u32));
        peripherals.DMA_WRITER0.enable.write(|w| w.bits(1u32));
    }

    loop {
        log::info!("READ");
        unsafe {
            fence(Ordering::Release);
            compiler_fence(Ordering::Release);
            let buf_read = BUF_IN.clone();
            for i in 0..BUF_SZ_WORDS {
                log::info!("{:x}@{:x}", i, buf_read[i]);

            }
        }
        /*
        log::info!("jack_detect {:x}", peripherals.EURORACK_PMOD0.csr_jack.read().bits() as u8);
        log::info!("input0 {}", peripherals.EURORACK_PMOD0.csr_cal_in0.read().bits() as i16);
        log::info!("input1 {}", peripherals.EURORACK_PMOD0.csr_cal_in1.read().bits() as i16);
        log::info!("input2 {}", peripherals.EURORACK_PMOD0.csr_cal_in2.read().bits() as i16);
        log::info!("input3 {}", peripherals.EURORACK_PMOD0.csr_cal_in3.read().bits() as i16);
        log::info!("serial {:x}", peripherals.EURORACK_PMOD0.csr_eeprom_serial.read().bits() as u32);

        timer.delay_ms(10u32);
        log::info!("tick - elapsed {} msec", (elapsed * 1000.0) as u32);
        elapsed += 0.01f32;
        */
    }
}
