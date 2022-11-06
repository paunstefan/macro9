# MACRO9 keypad

MACRO9 is a 9 key macropad made using a Raspberry Pi Pico, based on the
[VOID9 macropad](https://victorlucachi.ro/showcase/void9/).

It it recognized by the computer as a standard USB HID keyboard, so it should work
on any OS without additional configuration.

## Configuring the keys

Each of the 9 keys is configurable and can act as 6 different button presses
and any combination of modifier keys (CTRL, SHIFT, ALT, META). 

To be able to be configured, the MACRO9 opens an USB serial port, in addition to the
USB HID interface. The communication is done using a few simple commands.

The config is saved in the Pico on-board flash memory so it will persist after a power cycle.

### Configuration protocol

Key config structure:
```
 0 1 2 3 4 5 6
+-+-+-+-+-+-+-+
|M|B|B|B|B|B|B|
+-+-+-+-+-+-+-+

M = Modifier keys bitmap
B = HID keycodes for each of the 6 buttons, 0 if empty

Modifier keys:
L_CTRL  = 0x01
L_SHIFT = 0x02
L_ALT   = 0x04
L_META  = 0x08
R_CTRL  = 0x10
R_SHIFT = 0x20
R_ALT   = 0x40
R_META  = 0x80
```

Packet types:
```
S = Start sequence [0x4D, 0x39]
T = Type
C = CRC-8 of all bytes in the packet, except the CRC field(Poly: 0x07, Init: 0, RefIn: 0, RefOut: 0, XOROut: 0)
BUT = 9 key config structures (63 bytes)

GET request -> ask the device for the current key config
 0 1  2   3
+-+-+----+-+
|S S|0x47|C|
+-+-+----+-+

GET response -> current key config received from the device
 0 1  2   3..65 66
+-+-+----+-----+--+
|S S|0x52| BUT |C |
+-+-+----+-----+--+

SET request -> ask device to apply the provided config
 0 1  2   3..65 66
+-+-+----+-----+--+
|S S|0x53| BUT |C |
+-+-+----+-----+--+

SET acknowledge -> response if set was successful
 0 1  2   3
+-+-+----+-+
|S S|0x4B|C|
+-+-+----+-+

SET error -> response if set failed
 0 1  2   3
+-+-+----+-+
|S S|0x45|C|
+-+-+----+-+
```

An implementation that can be used to configure the device can be found
in the `config.py` file in the repo.

## Resources

* <https://gist.github.com/MightyPork/6da26e382a7ad91b5496ee55fdc73db2>
* <https://blog.jmdawson.co.uk/raspberry-pi-pico-macro-pad>