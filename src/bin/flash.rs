#![no_std]
#![no_main]

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

// Flash constants
const FLASH_START: usize = 0x10000000;
const FLASH_SIZE: usize = 2048 * 1024;
const FLASH_SECTOR_SIZE: usize = 4096;
const FLASH_PAGE_SIZE: usize = 256;
const FLASH_OFFSET: usize = (FLASH_SIZE - FLASH_SECTOR_SIZE);
const FLASH_WR: *mut u8 = (FLASH_START + FLASH_OFFSET) as *mut u8;

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

    let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            hal::uart::common_configs::_9600_8_N_1,
            hal::Clock::freq(&clocks.peripheral_clock),
        )
        .unwrap();
    uart.write_full_blocking(b"UART works\n\r");

    // write to flash
    let mut buf = [0u8; 256];
    buf[0] = 0xCA;
    buf[1] = 0xFE;

    uart.write_full_blocking(b"a");

    cortex_m::interrupt::free(|_| unsafe {
        hal::rom_data::flash_range_erase(FLASH_OFFSET as u32, 4096, 4096, 0xd8);
        uart.write_full_blocking(b"b");
        hal::rom_data::flash_range_program(FLASH_OFFSET as u32, buf.as_ptr(), 256);
    });

    //uart.write_full_blocking(b"written data\r\n");

    // read flash
    let mut a = [0xAB, 0xCD];
    uart.write_full_blocking(&a[0..1]);
    uart.write_full_blocking(&a[1..2]);

    cortex_m::interrupt::free(|_| unsafe {
        a[0] = core::ptr::read_volatile(FLASH_WR);
        a[1] = core::ptr::read_volatile(FLASH_WR.offset(1));
    });

    uart.write_full_blocking(&a[0..1]);
    uart.write_full_blocking(&a[1..2]);

    if a[0] == 0xCA && a[1] == 0xFE {
        led_pin.set_high().unwrap();
    }

    // Run forever, setting the LED according to the button
    loop {}
}
