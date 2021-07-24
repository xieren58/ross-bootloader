# Rusty Old Smart System
This repository contains a bootloader for the `Rusty Old Smart System` project. 

# Features
- Utilizes the STM32F103CBT microcontroller

# Getting Started

## Dependencies
To build and flash this project you will need:

- OpenOCD. [Installation instructions](http://openocd.org/getting-openocd/).
- Rust toolchain. [Installation instructions](https://www.rust-lang.org/learn/get-started). After installation run:
    ```
    $ cd ross-bootloader/
    $ rustup override set nightly
    ```
- `rust-std` components for the `thumbv7m-none-eabi` target. Run:
    ```
    $ rustup target add thumbv7m-none-eabi
    ```
- `binutils`. [Installation instructions](https://www.gnu.org/software/binutils/).

## Building
To build this project, run:
```
$ cargo build --release
```

## Flashing
To flash the bootloader, run:
```
$ ./flash.sh
```

# License
This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details.
