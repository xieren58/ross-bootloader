#![no_std]
#![no_main]

use panic_semihosting as _;

use cortex_m::asm::bootload;
use cortex_m_rt::entry;
use stm32f1xx_hal::prelude::*;

static PROGRAM_ADDRESS: u32 = 0x0800_4000;

#[entry]
fn main() -> ! {
    boot();
}

fn boot() -> ! {
    unsafe {
        bootload(PROGRAM_ADDRESS as *const u32);
    }
}
