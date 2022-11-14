//! # MACRO9 Rust keypad
#![no_std]
#![no_main]

use embedded_hal::digital::v2::InputPin;
use panic_halt as _;
use rp_pico::entry;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::pac::interrupt;
use rp_pico::hal::prelude::*;

use usb_device::{class_prelude::*, prelude::*};
use usbd_hid::descriptor::generator_prelude::*;
use usbd_hid::descriptor::KeyboardReport;
use usbd_hid::hid_class::HIDClass;
use usbd_serial::SerialPort;

mod config;
mod flash;
mod keys;

/// The USB Device Driver (shared with the interrupt).
static mut USB_DEVICE: Option<UsbDevice<hal::usb::UsbBus>> = None;

/// The USB Bus Driver (shared with the interrupt).
static mut USB_BUS: Option<UsbBusAllocator<hal::usb::UsbBus>> = None;

/// The USB Human Interface Device Driver (shared with the interrupt).
static mut USB_HID: Option<HIDClass<hal::usb::UsbBus>> = None;

/// The USB Serial Driver (shared with the interrupt).
static mut USB_SERIAL: Option<SerialPort<hal::usb::UsbBus>> = None;

#[entry]
fn main() -> ! {
    // Initialize START
    let mut pac = pac::Peripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

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

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Set up the USB driver
    let usb_bus = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_BUS = Some(usb_bus);
    }

    // Grab a reference to the USB Bus allocator. We are promising to the
    // compiler not to take mutable access to this global variable whilst this
    // reference exists!
    let bus_ref = unsafe { USB_BUS.as_ref().unwrap() };

    // Set up the USB HID Class Device driver, providing Mouse Reports
    let usb_hid = HIDClass::new(bus_ref, KeyboardReport::desc(), 60);
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet.
        USB_HID = Some(usb_hid);
    }

    // Set up the USB Communications Class Device driver
    let serial = SerialPort::new(bus_ref);
    unsafe {
        USB_SERIAL = Some(serial);
    }

    // Create a USB device with a fake VID and PID
    let usb_dev = UsbDeviceBuilder::new(bus_ref, UsbVidPid(0x16c0, 0x27db))
        .manufacturer("PWN")
        .product("MACRO9")
        .device_class(0)
        .build();
    unsafe {
        // Note (safety): This is safe as interrupts haven't been started yet
        USB_DEVICE = Some(usb_dev);
    }

    unsafe {
        // Enable the USB interrupt
        pac::NVIC::unmask(hal::pac::Interrupt::USBCTRL_IRQ);
    };
    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Initialize END

    // Check flash config existence
    let flag = flash::read_flash(flash::FLASH_OFFSET as u32, 1);
    if flag[0] == 0x39 {
        let config = flash::read_flash((flash::FLASH_OFFSET + 1) as u32, 63);
        let key_struct = keys::KeypadConfig::deserialize(config);

        if let Some(key_struct) = key_struct {
            // Replace running config
            cortex_m::interrupt::free(|cs| {
                (*keys::KEYS.borrow(cs).borrow_mut()).keys = key_struct.keys;
            });
        }
    }
    let keys: &[&dyn InputPin<Error = core::convert::Infallible>; 9] = &[
        &pins.gpio2.into_pull_up_input(),
        &pins.gpio1.into_pull_up_input(),
        &pins.gpio4.into_pull_up_input(),
        &pins.gpio9.into_pull_up_input(),
        &pins.gpio7.into_pull_up_input(),
        &pins.gpio22.into_pull_up_input(),
        &pins.gpio13.into_pull_up_input(),
        &pins.gpio14.into_pull_up_input(),
        &pins.gpio17.into_pull_up_input(),
    ];

    loop {
        delay.delay_ms(30);

        let reports = keys::get_keys(keys);
        reports.into_iter().flatten().for_each(|report| {
            push_keyboard_report(report);
        });
    }
}

/// Submit a new keyboard report to the USB stack.
///
/// We do this with interrupts disabled, to avoid a race hazard with the USB IRQ.
fn push_keyboard_report(report: KeyboardReport) -> Result<usize, usb_device::UsbError> {
    cortex_m::interrupt::free(|_| unsafe {
        // Now interrupts are disabled, grab the global variable and, if
        // available, send it a HID report
        USB_HID.as_mut().map(|hid| hid.push_input(&report))
    })
    .unwrap()
}

/// Reads into buffer until an error (like WouldBlock) is encountered
/// If the first read is successful, it continues reading into the buffer
/// If the first read returns WouldBlock, it returns Ok(0)
///
/// Because function might be called too fast, it retries a number of times
/// to read a packet
fn read_full_packet(
    port: &mut SerialPort<hal::usb::UsbBus>,
    buf: &mut [u8],
) -> Result<usize, UsbError> {
    let mut index = 0;
    let mut retries = 0;
    loop {
        let rd = port.read(&mut buf[index..]);

        match rd {
            Ok(n) => index += n,
            Err(usb_device::UsbError::WouldBlock) => {
                if index == 0 || index == 4 || index == 67 || retries > 32 {
                    return Ok(index);
                }
            }
            Err(e) => return Err(e),
        };
        retries += 1;
    }
}

/// This function is called whenever the USB Hardware generates an Interrupt
/// Request.
#[allow(non_snake_case, unused_must_use)]
#[interrupt]
unsafe fn USBCTRL_IRQ() {
    // Handle USB request
    let usb_dev = USB_DEVICE.as_mut().unwrap();
    let usb_hid = USB_HID.as_mut().unwrap();
    let serial = USB_SERIAL.as_mut().unwrap();

    if usb_dev.poll(&mut [usb_hid, serial]) {
        let mut buf = [0u8; 67];
        let rd = read_full_packet(serial, &mut buf);

        let response = match rd {
            Ok(0) => None,
            Ok(n) => config::process_command(&mut buf, n),
            Err(_) => None,
        };

        if let Some(response) = response {
            serial.write(response);
        }
    }
}
