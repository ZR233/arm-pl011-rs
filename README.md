# Arm PL011 驱动

本库实现了PL011 UART驱动的同步和异步接口。

[![Latest Version](https://img.shields.io/crates/v/arm-pl011-rs.svg)](https://crates.io/crates/arm-pl011-rs)
[![Documentation](https://docs.rs/arm-pl011-rs/badge.svg)](https://docs.rs/arm-pl011-rs)
[![License](https://img.shields.io/crates/l/arm-pl011-rs.svg)](https://github.com/ZR233/arm-pl011-rs#license)

## 示例

### Async

```rust
use core::ptr::NonNull;

use arm_pl011_rs::{Config, DataBits, Parity, Pl011, StopBits};
use embedded_io_async::*;

pub async fn write() {
    let mut uart = Pl011::new(
        NonNull::new(0x0900_0000 as *mut u8).unwrap(),
        Some(Config {
            baud_rate: 115200,
            clock_freq: 24000000,
            data_bits: DataBits::Bits8,
            stop_bits: StopBits::STOP1,
            parity: Parity::None,
        }),
    )
    .await;

    uart.write_all("uart output\n".as_bytes()).await;
}
```

### Sync

```rust
use core::ptr::NonNull;

use arm_pl011_rs::{Config, DataBits, Parity, Pl011, StopBits};
use embedded_io::*;

pub fn write() {
    let mut uart = Pl011::new_sync(
        NonNull::new(0x0900_0000 as *mut u8).unwrap(),
        Some(Config {
            baud_rate: 115200,
            clock_freq: 24000000,
            data_bits: DataBits::Bits8,
            stop_bits: StopBits::STOP1,
            parity: Parity::None,
        }),
    );

    uart.write_all("uart output\n".as_bytes());
}
```

## License

Licensed under Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>) or MIT ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)) at your choice.
