#!/bin/sh
cargo objcopy --release -- -O binary ./target/thumbv7m-none-eabi/release/ross-bootloader.bin
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "init; reset halt; flash erase_address 0x08000000 0x00004000; flash write_bank 0 ./target/thumbv7m-none-eabi/release/ross-bootloader.bin 0x00000000; reset run; shutdown;"
