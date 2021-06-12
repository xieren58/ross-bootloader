# Rusty Old Smart System
This repository contains a bootloader for the `Rusty Old Smart System` project. 

# Features
- Utilizes the STM32F103CBT microcontroller

# Getting Started

## Dependencies
To build and flash this project you will need:

- OpenOCD. [Installation instructions](http://openocd.org/getting-openocd/).
- Rust toolchain. [Installation instructions](https://www.rust-lang.org/learn/get-started).
- `rust-std` components for the `thumbv7m-none-eabi` target. Run:
    ```
    $ rustup target add thumbv7m-none-eabi
    ```
- `cargo-binutils`. Run:
    ```
    $ cargo install cargo-binutils
    ```

## Building
To build this project, run:
```
$ cargo build --release
```

## Flashing
To flash the built bootloader, run:
```
$ cargo objcopy --release -- -O binary ./target/thumbv7m-none-eabi/release/ross-bootloader.bin
$ openocd -f interface/stlink.cfg -f target/stm32f1x.cfg -c "init; reset halt; stm32f1x mass_erase 0; flash write_bank 0 ./target/thumbv7m-none-eabi/release/ross-bootloader.bin ; reset run; shutdown;"
```

# License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
