use core::{cell::RefCell, convert::Infallible};

use cortex_m::interrupt::{self, Mutex};
use embedded_hal::digital::v2::InputPin;
use usbd_hid::descriptor::KeyboardReport;

const NUM_KEYS: usize = 9;

/// Configuration of a single key on the keypad
#[derive(Clone, Copy)]
pub struct Key {
    pub modifier: u8,
    pub keycodes: [u8; 6],
}

pub struct KeypadConfig([Key; NUM_KEYS]);

/// Configuration for each of the 9 buttons of the keypad
/// These default values may be changed by the configurator
pub static KEYS: Mutex<RefCell<KeypadConfig>> = Mutex::new(RefCell::new(KeypadConfig([
    Key {
        modifier: 0x01,
        keycodes: [0x06, 0x00, 0x00, 0x00, 0x00, 0x00],
    },
    Key {
        modifier: 0x01,
        keycodes: [0x19, 0x00, 0x00, 0x00, 0x00, 0x00],
    },
    Key {
        modifier: 0,
        keycodes: [0; 6],
    },
    Key {
        modifier: 0,
        keycodes: [0; 6],
    },
    Key {
        modifier: 0,
        keycodes: [0; 6],
    },
    Key {
        modifier: 0,
        keycodes: [0; 6],
    },
    Key {
        modifier: 0,
        keycodes: [0; 6],
    },
    Key {
        modifier: 0,
        keycodes: [0; 6],
    },
    Key {
        modifier: 0,
        keycodes: [0; 6],
    },
])));

impl KeypadConfig {
    fn serialize_key(key: &Key) -> [u8; 7] {
        let mut ret = [0; 7];

        ret[0] = key.modifier;
        ret[1..7].copy_from_slice(&key.keycodes[..]);

        ret
    }

    fn deserialize_key(buf: &[u8]) -> Option<Key> {
        if buf.len() != 7 {
            return None;
        }
        let modifier = buf[0];
        let mut keycodes = [0u8; 6];
        keycodes.copy_from_slice(&buf[1..]);

        Some(Key { modifier, keycodes })
    }

    pub fn serialize(&self, buf: &mut [u8]) -> Result<(), ()> {
        if buf.len() != 63 {
            return Err(());
        }

        for i in 0..NUM_KEYS {
            let index = i * 7;
            buf[index..(index + 7)].copy_from_slice(&KeypadConfig::serialize_key(&self.0[i]))
        }

        Ok(())
    }

    pub fn deserialize(buf: &[u8]) -> Option<Self> {
        if buf.len() != 63 {
            return None;
        }

        let mut keys = [Key {
            modifier: 0,
            keycodes: [0; 6],
        }; NUM_KEYS];

        for i in 0..NUM_KEYS {
            let index = i * 7;
            let key = KeypadConfig::deserialize_key(&buf[index..(index + 7)])?;

            keys[i] = key;
        }

        Some(KeypadConfig(keys))
    }
}

fn new_report() -> KeyboardReport {
    KeyboardReport {
        modifier: 0,
        reserved: 0,
        leds: 0,
        keycodes: [0; 6],
    }
}

pub fn get_keys(keys: &[&dyn InputPin<Error = Infallible>]) -> [Option<KeyboardReport>; 2] {
    let mut ret = [None; 2];
    let mut pressed = false;

    for i in 0..keys.len() {
        if keys[i].is_low().unwrap() {
            let mut report = new_report();
            interrupt::free(|cs| {
                let buttons = &KEYS.borrow(cs).borrow().0;
                report.modifier = buttons[i].modifier;
                report.keycodes = buttons[i].keycodes;
            });
            ret[i] = Some(report);
            pressed = true;
        }
    }

    if !pressed {
        ret[0] = Some(new_report());
    }

    ret
}

// #[test]
// fn serialize_key() {
//     let key = Key {
//         modifier: 0x01,
//         keycodes: [0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B],
//     };

//     let expected: [u8; 7] = [0x01, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B];
//     assert_eq!(expected, KeypadConfig::serialize_key(&key));
// }
