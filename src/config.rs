use rp_pico::hal;
use usbd_serial::SerialPort;

use crate::keys::{KeypadConfig, KEYS};

const CONFIG_SIZE: usize = 63;
const PACKET_START_SIZE: usize = 2;
const CRC_SIZE: usize = 1;
const TYPES_SIZE: usize = 1;
const PACKET_SIZE: usize = 67;

const PACKET_START: [u8; 2] = [0x4D, 0x39]; // M9

// Response packets with precompiled CRC
const SET_ACK: [u8; 4] = [0x4D, 0x39, 0x4B, 0xA5];
const SET_NACK: [u8; 4] = [0x4D, 0x39, 0x45, 0x8F];

#[repr(u8)]
enum Type {
    GetRequest = 0x47,  // G
    GetResponse = 0x52, // R
    SetRequest = 0x53,  // S
    SetAck = 0x4B,      // K
    SetError = 0x45,    // E
}

impl TryFrom<u8> for Type {
    type Error = ();

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x47 => Ok(Type::GetRequest),
            0x52 => Ok(Type::GetResponse),
            0x53 => Ok(Type::SetRequest),
            0x4B => Ok(Type::SetAck),
            0x45 => Ok(Type::SetError),
            _ => Err(()),
        }
    }
}

fn calculate_crc(data: &[u8]) -> u8 {
    simple_crc::simple_crc8(data, 0x07, 0x00, false, false, 0x00)
}

pub fn process_command(serial: &mut SerialPort<hal::usb::UsbBus>, buf: &mut [u8], n: usize) {
    // let mut buf = [0u8; 67];
    // let rd = serial.read(&mut buf);

    // // ISSUE: serial read does not work without the UART print here
    // // with it here, it works sometimes

    // unsafe {
    //     use core::fmt::Write;

    //     crate::UART
    //         .as_mut()
    //         .unwrap()
    //         .write_fmt(format_args!("Read {:?}\n\r", rd));
    // }

    //if let Ok(n) = rd {
    if n < 3 && buf[0..2] != PACKET_START {
        unsafe {
            use core::fmt::Write;

            crate::UART
                .as_mut()
                .unwrap()
                .write_fmt(format_args!("Read {} bytes\n\r", n));
        }
        return;
    }

    match Type::try_from(buf[2]) {
        Ok(Type::GetRequest) => {
            unsafe {
                crate::UART
                    .as_mut()
                    .unwrap()
                    .write_full_blocking(b"GET request received\n\r");
            }
            if n != 4 || calculate_crc(&buf[0..3]) != buf[3] {
                unsafe {
                    crate::UART
                        .as_mut()
                        .unwrap()
                        .write_full_blocking(b"Request wrong\n\r");
                }
                return;
            }
            if process_get(buf).is_ok() {
                unsafe {
                    crate::UART
                        .as_mut()
                        .unwrap()
                        .write_full_blocking(b"Writing to serial\n\r");
                }
                serial.write(&buf[0..67]);
                unsafe {
                    crate::UART
                        .as_mut()
                        .unwrap()
                        .write_full_blocking(b"Write complete\n\r");
                }
            }
        }
        Ok(Type::SetRequest) => {
            if n != 67 || calculate_crc(&buf[0..66]) != buf[66] {
                return;
            }

            match process_set(&buf[3..66]) {
                Ok(_) => {
                    serial.write(&SET_ACK);
                }
                Err(_) => {
                    serial.write(&SET_NACK);
                }
            }
        }
        _ => return,
    };
    //}
}

/// Receives a buffer in which the current key config will be written
fn process_get(buf: &mut [u8]) -> Result<(), ()> {
    cortex_m::interrupt::free(|cs| KEYS.borrow(cs).borrow().serialize(&mut buf[3..66]))?;

    buf[0..2].copy_from_slice(&PACKET_START);
    buf[2] = 0x52;
    buf[66] = calculate_crc(&buf[0..66]);
    unsafe {
        crate::UART
            .as_mut()
            .unwrap()
            .write_full_blocking(b"Got the config and built the packet\n\r");
    }

    Ok(())
}

/// Receives a buffer with the key config to be applied to the device
fn process_set(buf: &[u8]) -> Result<(), ()> {
    let key_struct = match KeypadConfig::deserialize(&buf) {
        Some(k) => k,
        None => {
            return Err(());
        }
    };

    unsafe {
        use core::fmt::Write;

        crate::UART
            .as_mut()
            .unwrap()
            .write_fmt(format_args!("Deserialized config {:?}\n\r", key_struct));
    }

    Ok(())
}

// pub fn process_command(serial: &mut SerialPort<hal::usb::UsbBus>) {
//     let mut buf = [0u8; 4];
//     if let Ok(4) = serial.read(&mut buf) {
//         if buf == [b'P', b'I', b'N', b'G'] {
//             serial.write(b"PONG");
//         } else {
//             serial.write(b"Wrong command");
//         }
//     }
// }
