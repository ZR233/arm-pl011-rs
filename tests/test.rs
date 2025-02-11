#![no_std]
#![no_main]
#![feature(used_with_arg)]

extern crate alloc;

use alloc::vec::Vec;
use bare_test::{
    GetIrqConfig,
    async_std::time,
    globals::global_val,
    irq::{IrqHandleResult, IrqInfo, IrqParam},
    mem::mmu::{iomap, page_size},
};
use core::{cell::UnsafeCell, time::Duration};
use log::*;

// struct Host(UnsafeCell<USBHost<Xhci>>);
// unsafe impl Send for Host {}
// unsafe impl Sync for Host {}

#[bare_test::tests]
mod tests {
    use core::{hint::spin_loop, ptr::NonNull};

    use alloc::sync::Arc;
    use arm_pl011_rs::{IMSC, Pl011};
    use bare_test::{
        irq::{IrqHandleResult, IrqParam},
        platform::cpu_id,
        task::TaskConfig,
        time::sleep,
    };
    use embedded_io::Write;
    use log::warn;

    use super::*;

    #[test]
    fn test_cmd() {
        let info = get_uart().unwrap();
        // for one in &info.irq.cfgs {
        //     IrqParam {
        //         irq_chip: info.irq.irq_parent,
        //         cfg: one.clone(),
        //     }
        //     .register_builder({
        //         move |irq| {
        //             debug!("irq ");
        //             IrqHandleResult::Handled
        //         }
        //     })
        //     .cpu_list(alloc::vec![cpu_id()])
        //     .register();
        // }

        let mut uart = Pl011::new_sync(info.addr, None);
        uart.write_imsc(IMSC::TXIM | IMSC::RXIM);

        for _ in 0..100 {
            uart.data_write(b'a');
        }

        loop {
            spin_loop();
        }
    }

    fn get_uart() -> Option<Info> {
        let fdt = match &global_val().platform_info {
            bare_test::globals::PlatformInfoKind::DeviceTree(fdt) => fdt,

            _ => panic!("unsupported platform"),
        };

        let fdt = fdt.get();
        let node = fdt.chosen().unwrap().stdout().unwrap().node;

        debug!("pl011 node: {}", node.name);
        let regs = node.reg().unwrap().collect::<Vec<_>>();
        debug!("regs: {:?}", regs);

        let addr = iomap(
            (regs[0].address as usize).into(),
            regs[0].size.unwrap_or(0x1000),
        );

        let irq = node.irq_info().unwrap();

        return Some(Info { irq, addr });

        None
    }

    struct Info {
        irq: IrqInfo,
        addr: NonNull<u8>,
    }
}
