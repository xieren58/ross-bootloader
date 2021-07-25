#![no_std]
#![no_main]
#![feature(alloc_error_handler)]

extern crate alloc;

use panic_itm as _;

use alloc::format;
use alloc_cortex_m::CortexMHeap;
use bxcan::filter::Mask32;
use bxcan::Instance;
use core::alloc::Layout;
use cortex_m::asm::{bootload, nop};
use cortex_m_rt::entry;
use eeprom24x::{Eeprom24x, SlaveAddr};
use embedded_hal::digital::v2::InputPin;
use nb::block;
use stm32f1xx_hal_bxcan::can::Can;
use stm32f1xx_hal_bxcan::i2c::{BlockingI2c, Mode};
use stm32f1xx_hal_bxcan::pac::{CorePeripherals, Peripherals};
use stm32f1xx_hal_bxcan::flash::{SectorSize, FlashSize};
use stm32f1xx_hal_bxcan::delay::Delay;
use stm32f1xx_hal_bxcan::prelude::*;

use ross_eeprom::{RossEeprom, RossDeviceInfo};
use ross_protocol::ross_packet::RossPacket;
use ross_protocol::ross_convert_packet::RossConvertPacket;
use ross_protocol::ross_event::ross_bootloader_event::RossBootloaderHelloEvent;
use ross_protocol::ross_event::ross_programmer_event::*;
use ross_protocol::ross_event::ross_general_event::*;
use ross_protocol::ross_interface::ross_can::{RossCan, RossCanError};
use ross_logger::{log_debug, log_warning, log_error, RossLogger, RossLogLevel};

const PROGRAM_ADDRESS: u32 = 0x0801_0000;
const FLASH_OFFSET: u32 = 0x0001_0000;
const FLASH_SECTOR_SIZE: usize = 1024;

const EEPROM_BITRATE: u32 = 400_000;

const CAN_BITRATE: u32 = 50_000;
const CAN_TSEG1: u32 = 13;
const CAN_TSEG2: u32 = 2;
const CAN_SJW: u32 = 1;

const HEAP_SIZE: usize = 4096;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

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

    let mut logger = RossLogger::new(RossLogLevel::Debug, cp.ITM);
    
    log_debug!(logger, "Bootloader initialized.");

    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    let upgrade_input = gpioa.pa1.into_pull_down_input(&mut gpioa.crl);

    // If no firmware upgrade is requested, proceed with bootloading the program
    if upgrade_input.is_low().unwrap() {
        log_debug!(logger, "Booting firmware.");
        boot();
    }

    log_debug!(logger, "Entering upgrade mode.");

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut delay = Delay::new(cp.SYST, clocks);

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

    log_debug!(logger, "Loaded device information from EEPROM ({:?}).", device_info);

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

    let mut flash_writer = flash.writer(SectorSize::Sz1K, FlashSize::Sz128K);

    flash_writer.change_verification(true);

    let mut firmware_size: u32 = 0;
    let mut new_device_info = None;

    let mut buf = [0x00; FLASH_SECTOR_SIZE];
    let mut buf_offset = 0;
    let mut flash_offset = FLASH_OFFSET;

    loop {
        let packet = wait_for_packet(&mut can, &mut logger);

        if let Ok(event) = RossProgrammerHelloEvent::try_from_packet(&packet) {    
            log_debug!(logger, "Received `programmer_hello_event` ({:?}).", event);

            transmit_bootloader_hello_event(&mut can, &device_info, &event, &mut logger);
        } else if let Ok(event) = RossProgrammerStartUploadEvent::try_from_packet(&packet) {
            log_debug!(logger, "Received `programmer_start_upload_event` ({:?}).", event);

            firmware_size = event.firmware_size;
            new_device_info = Some(RossDeviceInfo {
                device_address: device_info.device_address,
                firmware_version: event.new_firmware_version,
                peripheral_info_address: device_info.peripheral_info_address,
                program_info_address: device_info.program_info_address,
            });

            transmit_ack_event(&mut can, &device_info, event.programmer_address, &mut logger);
        } else if let Ok(event) = RossDataEvent::try_from_packet(&packet) {
            if let Some(ref device_info) = new_device_info {
                transmit_ack_event(&mut can, &device_info, event.transmitter_address, &mut logger);
                
                for i in 0..event.data_len {
                    buf[buf_offset + i as usize] = event.data[i as usize];
                }

                buf_offset += event.data_len as usize;

                if buf_offset >= buf.len() {
                    if buf_offset != buf.len() {
                        log_error!(logger, "Data packet length is not a divisor of {}.", buf.len());
                        break;
                    }

                    if let Err(err) = flash_writer.erase(flash_offset as u32, buf.len()) {
                        log_error!(logger, "Failed to erase FLASH at offset {:#010x} with error ({:?}).", flash_offset, err);
                        break;
                    }

                    if let Err(err) = flash_writer.write(flash_offset as u32, &buf[..]) {
                        log_error!(logger, "Failed to write to FLASH at offset {:#010x} with error ({:?}).", flash_offset, err);
                        break;
                    }

                    flash_offset += buf.len() as u32;
                    buf_offset = 0;

                    if flash_offset == FLASH_OFFSET + firmware_size {
                        log_debug!(logger, "Writing new device info to EEPROM ({:?}).", new_device_info);

                        if let Err(err) = eeprom.write_device_info(&device_info, &mut delay) {
                            log_error!(logger, "Failed to write new device info to EEPROM with error ({:?}).", err);
                        }

                        log_debug!(logger, "Booting firmware.");
                        boot();
                    }
                }
            } else {
                log_warning!(logger, "Unexpected `data_event` before `programmer_start_upload_event`.");
            }
        } else {
            log_warning!(logger, "Received unexpected event ({:?}).", packet);
        }
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

fn wait_for_packet<I: Instance>(can: &mut RossCan<I>, logger: &mut RossLogger) -> RossPacket {
    loop {
        let packet = match can.try_get_packet() {
            Ok(packet) => return packet,
            Err(err) => {
                if let RossCanError::NoPacketReceived = err {
                    continue;
                }

                log_warning!(logger, "Failed to get packet ({:?}).", err);
            },
        };
    }
} 

fn transmit_bootloader_hello_event<I: Instance>(
    can: &mut RossCan<I>,
    device_info: &RossDeviceInfo,
    programmer_hello_event: &RossProgrammerHelloEvent,
    logger: &mut RossLogger,
) {
    let event = RossBootloaderHelloEvent {
        device_address: device_info.device_address,
        programmer_address: programmer_hello_event.programmer_address,
        firmware_version: device_info.firmware_version,
    };

    if let Err(err) = can.try_send_packet(&event.to_packet()) {
        log_error!(logger, "Failed to send `bootloader_hello_event` ({:?}).", err);
    } else {
        log_debug!(logger, "Sent `bootloader_hello_event` ({:?}).", event);
    }
}

fn transmit_ack_event<I: Instance>(can: &mut RossCan<I>, device_info: &RossDeviceInfo, transmitter_address: u16, logger: &mut RossLogger) {
    let event = RossAckEvent {
        device_address: device_info.device_address,
        transmitter_address,
    };

    if let Err(err) = can.try_send_packet(&event.to_packet()) {
        log_error!(logger, "Failed to send `ack_event` ({:?}).", err);
    } else {
        log_debug!(logger, "Sent `ack_event` ({:?}).", event);
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
