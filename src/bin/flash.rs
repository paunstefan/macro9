#![no_std]
#![no_main]

use core::fmt::Write;

use hal::rom_data::flash_enter_cmd_xip;
// The macro for our start-up function
use rp_pico::entry;

// GPIO traits
use embedded_hal::digital::v2::{InputPin, OutputPin};

// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
use panic_halt as _;

// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use rp_pico::hal::pac;

// A shorter alias for the Hardware Abstraction Layer, which provides
// higher-level drivers.
use rp_pico::hal;
use rp_pico::hal::rom_data::{
    connect_internal_flash, flash_exit_xip, flash_flush_cache, flash_range_erase,
    flash_range_program,
};

// Flash constants
const FLASH_START: usize = 0x10000000;
const FLASH_SIZE: usize = 2048 * 1024;
const FLASH_SECTOR_SIZE: usize = 4096;
const FLASH_PAGE_SIZE: usize = 256;
const FLASH_BLOCK_SIZE: u32 = 65536;
const FLASH_OFFSET: usize = FLASH_SIZE - FLASH_SECTOR_SIZE;
const FLASH_WR: *mut u8 = (FLASH_START + FLASH_OFFSET) as *mut u8;

static mut UART: Option<
    hal::uart::UartPeripheral<
        hal::uart::Enabled,
        pac::UART0,
        (
            hal::gpio::Pin<hal::gpio::bank0::Gpio16, hal::gpio::Function<hal::gpio::Uart>>,
            hal::gpio::Pin<hal::gpio::bank0::Gpio17, hal::gpio::Function<hal::gpio::Uart>>,
        ),
    >,
> = None;
/// Entry point to our bare-metal application.
///
/// The `#[entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables are initialised.
///
/// The function configures the RP2040 peripherals, then just reads the button
/// and sets the LED appropriately.
#[entry]
fn main() -> ! {
    // Grab our singleton objects
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    //
    // The default is to generate a 125 MHz system clock
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins up according to their function on this particular board
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Our LED output
    let mut led_pin = pins.gpio0.into_push_pull_output();

    let uart_pins = (
        // UART TX (characters sent from RP2040) on pin 1 (GPIO0)
        pins.gpio16.into_mode::<hal::gpio::FunctionUart>(),
        // UART RX (characters received by RP2040) on pin 2 (GPIO1)
        pins.gpio17.into_mode::<hal::gpio::FunctionUart>(),
    );

    let mut uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            hal::uart::common_configs::_9600_8_N_1,
            hal::Clock::freq(&clocks.peripheral_clock),
        )
        .unwrap();
    uart.write_full_blocking(b"UART works\n\r");
    unsafe {
        UART = Some(uart);
    }

    // write to flash
    let mut buf = [0u8; 256];
    buf[0] = 0xCA;
    buf[1] = 0xFE;

    cortex_m::interrupt::free(|_| unsafe {
        flash_range_erase_safe(FLASH_OFFSET as u32, 4096);
        unsafe {
            UART.as_mut().unwrap().write_full_blocking(b"b\n\r");
        }
        flash_range_program_safe(FLASH_OFFSET as u32, &buf, 256);
    });

    // read flash
    let mut a = [0xAB, 0xCD];
    unsafe {
        UART.as_mut()
            .unwrap()
            .write_fmt(format_args!("intial: {:x} {:x}\n\r", a[0], a[1]))
            .unwrap();
    }

    cortex_m::interrupt::free(|_| unsafe {
        a[0] = core::ptr::read_volatile(FLASH_WR);
        a[1] = core::ptr::read_volatile(FLASH_WR.offset(1));
    });

    unsafe {
        UART.as_mut()
            .unwrap()
            .write_fmt(format_args!("after: {:x} {:x}\n\r", a[0], a[1]))
            .unwrap();
    }

    if a[0] == 0xCA && a[1] == 0xFE {
        led_pin.set_high().unwrap();
    }

    // new method
    buf[0] = 0xBE;
    buf[1] = 0xEF;
    write_flash(FLASH_OFFSET as u32, &buf);

    let r = read_flash(FLASH_OFFSET, 2);
    unsafe {
        UART.as_mut()
            .unwrap()
            .write_fmt(format_args!("new method: {:x} {:x}\n\r", r[0], r[1]))
            .unwrap();
    }

    // Run forever, setting the LED according to the button
    loop {}
}

const SECTOR_ERASE: u8 = 0x20; // Tested and works with W25Q16JV flash chip
const BLOCK32_ERASE: u8 = 0x52;
const BLOCK64_ERASE: u8 = 0xD8;

#[inline(never)]
#[link_section = ".data.ram_code"]
fn write_flash(addr: u32, data: &[u8]) {
    let size = data.len();
    assert!(addr as usize + size <= FLASH_SIZE);
    assert!(addr as usize & (FLASH_SECTOR_SIZE - 1) == 0);
    assert!(size & (FLASH_PAGE_SIZE - 1) == 0);

    cortex_m::interrupt::free(|_| unsafe {
        connect_internal_flash();
        flash_exit_xip();

        flash_range_erase(addr, FLASH_SECTOR_SIZE, FLASH_BLOCK_SIZE, SECTOR_ERASE);
        flash_range_program(addr, data.as_ptr(), size);

        flash_flush_cache();
        flash_enter_cmd_xip();
    });
}

fn read_flash(addr: usize, size: usize) -> &'static [u8] {
    let addr = (FLASH_START + addr) as *mut u8;
    let data = cortex_m::interrupt::free(|_| unsafe { core::slice::from_raw_parts(addr, size) });

    data
}

#[inline(never)]
#[link_section = ".data.ram_code"]
fn flash_range_erase_safe(addr: u32, size: usize) {
    assert!(addr as usize + size <= FLASH_SIZE);
    assert!(addr as usize & (FLASH_SECTOR_SIZE - 1) == 0);
    assert!(size & (FLASH_SECTOR_SIZE - 1) == 0);

    unsafe {
        connect_internal_flash();
        flash_exit_xip();

        flash_range_erase(addr, size, FLASH_BLOCK_SIZE, SECTOR_ERASE);

        flash_flush_cache();
        flash_enter_cmd_xip();
    }
}

#[inline(never)]
#[link_section = ".data.ram_code"]
fn flash_range_program_safe(addr: u32, data: &[u8], count: usize) {
    assert!(addr as usize + count <= FLASH_SIZE);
    assert!(addr as usize & (FLASH_PAGE_SIZE - 1) == 0);
    assert!(count & (FLASH_PAGE_SIZE - 1) == 0);

    unsafe {
        connect_internal_flash();
        flash_exit_xip();

        flash_range_program(addr, data.as_ptr(), count);

        flash_flush_cache();
        flash_enter_cmd_xip();
    }
}

// #[inline(never)]
// #[link_section = ".data.ram_code"]
// fn flash_init_boot2_copyout() {
//     if unsafe { BOOT2_COPYOUT_VALID } {
//         return;
//     }

//     let XIP_BASE = FLASH_START as *mut u32;

//     for i in 0..BOOT_SIZE_WORDS {
//         unsafe {
//             BOOT2_COPYOUT[i] = core::ptr::read(XIP_BASE.add(i));
//         }
//     }
//     atomic::compiler_fence(atomic::Ordering::SeqCst);
//     unsafe {
//         BOOT2_COPYOUT_VALID = true;
//     }
// }

// #[inline(never)]
// #[link_section = ".data.ram_code"]
// fn flash_enable_xip_via_boot2() {
//     unsafe {
//         let ptr = BOOT2_COPYOUT.as_ptr().add(1) as *const ();
//         let code: extern "C" fn() = core::mem::transmute(ptr);
//         (code)();
//     }
// }
