#![no_std]
#![no_main]

use panic_semihosting as _;

use cortex_m::asm::{bootload, nop};
use cortex_m_rt::entry;
use embedded_hal::digital::v2::InputPin;
use stm32f1xx_hal::prelude::*;
use stm32f1xx_hal::pac::Peripherals;

static PROGRAM_ADDRESS: u32 = 0x0800_4000;

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    rcc.cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .hclk(72.mhz())
        .pclk1(36.mhz())
        .pclk2(72.mhz())
        .freeze(&mut flash.acr);

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    let upgrade_input = gpioa.pa1.into_pull_down_input(&mut gpioa.crl);

    // If no firmware upgrade is requested, proceed with bootloading the program
    if upgrade_input.is_low().unwrap() {
        boot();
    }

    loop {
        nop();
    }
}

fn boot() -> ! {
    unsafe {
        bootload(PROGRAM_ADDRESS as *const u32);
    }
}
