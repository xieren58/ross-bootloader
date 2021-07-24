#!/bin/bash
cargo build --release

if [ $? -ne 0 ]; then
    exit 1
fi

objcopy -I elf32-little -O binary ./target/thumbv7m-none-eabi/release/ross-bootloader ./target/thumbv7m-none-eabi/release/ross-bootloader.bin

if [ $? -ne 0 ]; then
    exit 1
fi

openocd -f ./openocd.cfg                                                                        \
    -c "init;"                                                                                  \
    -c "reset halt;"                                                                            \
    -c "flash erase_address 0x08000000 0x00010000;"                                             \
    -c "flash write_bank 0 ./target/thumbv7m-none-eabi/release/ross-bootloader.bin 0x00000000;" \
    -c "tpiu config external uart off 72000000 2000000"                                         \
    -c "itm port 0 on"                                                                          \
    -c "reset run;"

if [ $? -ne 0 ]; then
    exit 1
fi
