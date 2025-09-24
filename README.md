# NetX rust integration

This takes the build process and most of the structures from https://github.com/sabaton-systems/threadx-rust/ and builds a ThreadX + NetX variant.
Compared to the original we did:

- Fix some UB
- Generate NetX bindings
- Implement simple async executor based on https://github.com/zesterer/pollster
- Implement embedded-nal interface for NetX/Wiced Wifi
- Async interface for buttons

# Prerequisites and Tool Installation

**All steps below are for Linux.**

## Update Linux

```sh
sudo apt update
sudo apt upgrade
```

## Install Arm GCC

```sh
sudo apt install gcc-arm-none-eabi
sudo apt install build-essential
```

## Install pkgconf

```sh
sudo apt install pkgconf
```

## Install Rust tools

```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

*Close your Linux shell and open it again to update environment variables.*

## Install dependencies

1. **flip-link**:
	```sh
	cargo install flip-link
	```

2. **Rust target** (for MXAZ3166):
	```sh
	rustup target add thumbv7m-none-eabi
	rustup target add thumbv7em-none-eabihf
	```

3. **Other dependencies**:
	```sh
	sudo apt install ninja-build libclang-dev cmake usbutils
	```

4. **Probe-RS to flash the board**:
	```sh
	sudo apt-get install libudev-dev
	cargo install probe-rs-tools
	```

## Allow access to debug probe-rs-tools

Follow instructions from [probe.rs Linux udev rules](https://probe.rs/docs/getting-started/probe-setup/#linux-udev-rules)

# Quickstart

## Executor example

The `executor.rs`example switches between 2 screens via tha A button and shows either a welcome screen or the actual temperature. It is a demonstration of the async executor based on ThreadX EventFlags.

Goto `threadx-app/cross/app` and run:

`cargo run --release --target thumbv7em-none-eabihf --bin executor`

## Network example

This examples connects to a WiFi Network and to an MQTT5 broker and regularly publishes the current temperature as uMessage (see: Link to uProtocol). 

In the `network.rs` example adapt the SSID, WLAN-Passwort and the MQTT settings accordingly.  

Goto `threadx-app/cross/app` and run:

`cargo run --release --target thumbv7em-none-eabihf --bin network`

## Shortcomings

- Only supports the MXAZ3166 board!
- No production ready error handling 
- General structure needs to be improved 
- Some more abstractions see embassy

### embedded-nal

- Only a single socket can be used
- As for now the implementation is unsound as there are no checks to assure that buffers are big enough to hold incoming packets 

### Async executor

- 32 parallel async tasks are supported
- Simple executor which blocks the thread it runs on 

## Control structures

Control structures should be checked if they are moveable ie. can be copied via a simple memcopy. Often this is not explicitely documented within the
ThreadX documentation hence we should assume that they cannot be moved. There are at least 2 obvious solutions:

- Make the control structures static and limit to a fixed number of for example mutexes
- Use the "std library" approach ie. pin box the control structure

## Further ideas

### Static tasks / threads

Veecle and embassy use statically allocated tasks via the type-impl-in-trait nightly feature. Maybe we should do the same to avoid dynamic allocation and the global allocator. 
