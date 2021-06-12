#![no_std]
#![no_main]

use panic_semihosting as _;

use cortex_m::asm;
use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    loop {
        asm::nop();
    }
}
