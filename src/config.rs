use rp_pico::hal;
use usbd_serial::SerialPort;

const CONFIG_SIZE: usize = 63;
const PACKET_START_SIZE: usize = 2;
const CRC_SIZE: usize = 1;
const TYPES_SIZE: usize = 1;
const PACKET_SIZE: usize = 67;

const PACKET_START: [u8; 2] = [0x4D, 0x39]; // M9

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

pub fn process_command(serial: &mut SerialPort<hal::usb::UsbBus>) {
    let mut buf = [0u8; 67];
    if let Ok(n) = serial.read(&mut buf) {
        if n < 3 && buf[0..2] != PACKET_START {
            return;
        }

        match Type::try_from(buf[2]) {
            Ok(Type::GetRequest) => {
                if n != 4 {
                    return;
                }

                if calculate_crc(&buf[0..3]) != buf[3] {
                    return;
                }
            }
            Ok(Type::SetRequest) => {
                if n != 67 {
                    return;
                }

                if calculate_crc(&buf[0..66]) != buf[66] {
                    return;
                }
            }
            _ => return,
        };
    }
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
