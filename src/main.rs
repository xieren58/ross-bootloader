#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

use panic_itm as _;

use alloc_cortex_m::CortexMHeap;
use bxcan::filter::Mask32;
use bxcan::Instance;
use core::alloc::Layout;
use cortex_m::asm::bootload;
use cortex_m::iprint;
use cortex_m_rt::entry;
use eeprom24x::{Eeprom24x, SlaveAddr};
use embedded_hal::digital::v2::InputPin;
use nb::block;
use stm32f1xx_hal::can::Can;
use stm32f1xx_hal::i2c::{BlockingI2c, Mode};
use stm32f1xx_hal::pac::{CorePeripherals, Peripherals, ITM};
use stm32f1xx_hal::prelude::*;

use ross_eeprom::{RossEeprom, RossDeviceInfo};
use ross_protocol::ross_packet::RossPacket;
use ross_protocol::ross_convert_packet::RossConvertPacket;
use ross_protocol::ross_event::ross_bootloader_event::RossBootloaderHelloEvent;
use ross_protocol::ross_event::ross_programmer_event::*;
use ross_protocol::ross_event::ross_general_event::*;
use ross_protocol::ross_interface::ross_can::{RossCan, RossCanError};

const PROGRAM_ADDRESS: u32 = 0x0801_0000;

const EEPROM_BITRATE: u32 = 400_000;

const CAN_BITRATE: u32 = 50_000;
const CAN_TSEG1: u32 = 13;
const CAN_TSEG2: u32 = 2;
const CAN_SJW: u32 = 1;

const DEBUG: bool = true;
const HEAP_SIZE: usize = 4096;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

static mut ITM_PERIPHERAL: Option<ITM> = None;

macro_rules! debug {
    ($fmt:expr) => {
        if DEBUG {
            iprint!(&mut unsafe { ITM_PERIPHERAL.as_mut().unwrap() }.stim[0], concat!($fmt, "\r\n"));
        }
    };
    ($fmt:expr, $($arg:tt)*) => {
        if DEBUG {
            iprint!(&mut unsafe { ITM_PERIPHERAL.as_mut().unwrap() }.stim[0], concat!($fmt, "\r\n"), $($arg)*);
        }
    };
}

#[entry]
fn main() -> ! {
    let dp = Peripherals::take().unwrap();
    let cp = CorePeripherals::take().unwrap();

    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    let clocks = rcc
        .cfgr
        .use_hse(8.mhz())
        .sysclk(72.mhz())
        .hclk(72.mhz())
        .pclk1(36.mhz())
        .pclk2(72.mhz())
        .freeze(&mut flash.acr);

    unsafe {
        ITM_PERIPHERAL = Some(cp.ITM);
    }

    debug!("Bootloader initialized.");

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    let upgrade_input = gpioa.pa1.into_pull_down_input(&mut gpioa.crl);

    // If no firmware upgrade is requested, proceed with bootloading the program
    if upgrade_input.is_low().unwrap() {
        debug!("Booting firmware.");
        boot();
    }

    debug!("Entering upgrade mode.");

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);

    let mut eeprom = {
        let i2c1 = {
            let scl = gpiob.pb6.into_alternate_open_drain(&mut gpiob.crl);
            let sda = gpiob.pb7.into_alternate_open_drain(&mut gpiob.crl);
            // TODO: put better values in the last 4 arguments
            BlockingI2c::i2c1(
                dp.I2C1,
                (scl, sda),
                &mut afio.mapr,
                Mode::standard(EEPROM_BITRATE.hz()),
                clocks,
                &mut rcc.apb1,
                10,
                10,
                10,
                10,
            )
        };

        let eeprom = Eeprom24x::new_24x02(i2c1, SlaveAddr::Alternative(false, false, false));

        RossEeprom::new(eeprom, 0)
    };

    let device_info = eeprom.read_device_info().unwrap();
    debug!("Loaded device information from EEPROM ({:?}).", device_info);

    let mut can = {
        let mut can1 = {
            let can = Can::new(dp.CAN1, &mut rcc.apb1, dp.USB);

            let rx = gpioa.pa11.into_floating_input(&mut gpioa.crh);
            let tx = gpioa.pa12.into_alternate_push_pull(&mut gpioa.crh);
            can.assign_pins((tx, rx), &mut afio.mapr);

            bxcan::Can::new(can)
        };

        can1.configure(|c| {
            c.set_bit_timing(calc_can_btr(clocks.pclk1().0));
            c.set_loopback(false);
            c.set_silent(false);
        });

        let mut filters = can1.modify_filters();
        filters.enable_bank(0, Mask32::accept_all());
        drop(filters);

        block!(can1.enable()).unwrap();
        
        RossCan::new(can1)
    };

    allocate_heap();

    let mut new_device_info = None;

    loop {
        let packet = wait_for_packet(&mut can);

        if let Ok(event) = RossProgrammerHelloEvent::try_from_packet(&packet) {    
            debug!("Received `programmer_hello_event` ({:?}).", event);

            transmit_bootloader_hello_event(&mut can, &device_info, &event);
        } else if let Ok(event) = RossProgrammerStartUploadEvent::try_from_packet(&packet) {
            debug!("Received `programmer_start_upload_event` ({:?}).", event);

            new_device_info = Some(RossDeviceInfo {
                device_address: device_info.device_address,
                firmware_version: event.new_firmware_version,
                peripheral_info_address: device_info.peripheral_info_address,
                program_info_address: device_info.program_info_address,
            });

            transmit_ack_event(&mut can, &device_info, event.programmer_address);
        } else if let Ok(event) = RossDataEvent::try_from_packet(&packet) {
            debug!("Received `data_event` ({:?}).", event);

            if let Some(ref _new_device_info) = new_device_info {
                transmit_ack_event(&mut can, &device_info, event.device_address);
                
                // TODO: write data to flash
                //  if last_data_event {
                //      debug!("Writing new device info to EEPROM ({:?}).", new_device_info);
                //      eeprom.write_device_info(new_device_info, &mut delay);
                //  }
            } else {
                debug!("Unexpected `data_event` before `programmer_start_upload_event`.");
            }
        } else {
            debug!("Received unexpected event ({:?}).", packet);
        }
    }
}

fn boot() -> ! {
    unsafe {
        bootload(PROGRAM_ADDRESS as *const u32);
    }
}

fn wait_for_packet<I: Instance>(can: &mut RossCan<I>) -> RossPacket {
    loop {
        let packet = match can.try_get_packet() {
            Ok(packet) => return packet,
            Err(err) => {
                if let RossCanError::NoPacketReceived = err {
                    continue;
                }

                debug!("Failed to get packet ({:?}).", err);
            },
        };
    }
} 

fn transmit_bootloader_hello_event<I: Instance>(
    can: &mut RossCan<I>,
    device_info: &RossDeviceInfo,
    programmer_hello_event: &RossProgrammerHelloEvent,
) {
    let event = RossBootloaderHelloEvent {
        device_address: device_info.device_address,
        programmer_address: programmer_hello_event.programmer_address,
        firmware_version: device_info.firmware_version,
    };

    if let Err(err) = can.try_send_packet(&event.to_packet()) {
        debug!("Failed to send `bootloader_hello_event` ({:?}).", err);
    } else {
        debug!("Sent `bootloader_hello_event` ({:?}).", event);
    }
}

fn transmit_ack_event<I: Instance>(can: &mut RossCan<I>, device_info: &RossDeviceInfo, transmitter_address: u16) {
    let event = RossAckEvent {
        device_address: device_info.device_address,
        transmitter_address,
    };

    if let Err(err) = can.try_send_packet(&event.to_packet()) {
        debug!("Failed to send `ack_event` ({:?}).", err);
    } else {
        debug!("Sent `ack_event` ({:?}).", event);
    }
}

fn calc_can_btr(clock_rate: u32) -> u32 {
    let brp = clock_rate / CAN_BITRATE / (CAN_TSEG1 + CAN_TSEG2);

    (brp - 1) | ((CAN_TSEG1 - 1) << 16) | ((CAN_TSEG2 - 1) << 20) | ((CAN_SJW - 1) << 24)
}

fn allocate_heap() {
    let start = cortex_m_rt::heap_start() as usize;
    unsafe { ALLOCATOR.init(start, HEAP_SIZE) }
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    loop {}
}
