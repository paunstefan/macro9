use crate::flash;
use cortex_m::interrupt;

use crate::keys::{KeypadConfig, KEYS};

const CONFIG_PACKET_SIZE: usize = 67;
const REQUEST_PACKET_SIZE: usize = 4;

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

pub fn process_command(buf: &mut [u8], n: usize) -> Option<&[u8]> {
    if n < REQUEST_PACKET_SIZE && buf[0..2] != PACKET_START {
        return None;
    }

    match Type::try_from(buf[2]) {
        Ok(Type::GetRequest) => {
            if n != REQUEST_PACKET_SIZE || calculate_crc(&buf[0..3]) != buf[3] {
                return None;
            }
            if process_get(buf).is_ok() {
                return Some(&buf[0..CONFIG_PACKET_SIZE]);
            }
        }
        Ok(Type::SetRequest) => {
            if n != CONFIG_PACKET_SIZE || calculate_crc(&buf[0..66]) != buf[66] {
                return None;
            }

            match process_set(&buf[3..66]) {
                Ok(_) => {
                    return Some(&SET_ACK);
                }
                Err(_) => {
                    return Some(&SET_NACK);
                }
            }
        }
        _ => return None,
    };

    None
}

/// Receives a buffer in which the current key config will be written
fn process_get(buf: &mut [u8]) -> Result<(), ()> {
    cortex_m::interrupt::free(|cs| KEYS.borrow(cs).borrow().serialize(&mut buf[3..66]))?;

    buf[0..2].copy_from_slice(&PACKET_START);
    buf[2] = 0x52;
    buf[66] = calculate_crc(&buf[0..66]);

    Ok(())
}

/// Receives a buffer with the key config to be applied to the device
fn process_set(buf: &[u8]) -> Result<(), ()> {
    let key_struct = match KeypadConfig::deserialize(buf) {
        Some(k) => k,
        None => {
            return Err(());
        }
    };

    // Replace running config
    interrupt::free(|cs| {
        (*KEYS.borrow(cs).borrow_mut()).keys = key_struct.keys;
    });

    // Write config to flash
    let mut flash_buffer = [0; 256];
    flash_buffer[0] = 0x39; // Config present flag (magic number)
    flash_buffer[1..64].copy_from_slice(buf);
    flash::write_flash(flash::FLASH_OFFSET as u32, &flash_buffer);

    Ok(())
}
