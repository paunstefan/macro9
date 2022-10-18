import serial
import sys

# Change this to the serial port of the MACRO9
SERIAL_PORT = '/dev/ttyACM0'


class Key:
    def __init__(self, modifiers=0, keys=[]):
        if len(keys) != 6:
            print("Invalid keys configuration")
            sys.exit(1)
        self.modifiers = modifiers
        self.keys = keys


class Keypad:
    def __init__(self, keys):
        if len(keys) != 9:
            print("Invalid keypad configuration")
            sys.exit(1)

        config = []
        for k in keys:
            config.append(k.modifiers)
            config += k.keys
        self.structs = bytes(config)


# Change the keys to the HID keycodes you want
KEY1 = Key(modifiers=0, keys=[0, 0, 0, 0, 0, 0])
KEY2 = Key(modifiers=0, keys=[0, 0, 0, 0, 0, 0])
KEY3 = Key(modifiers=0, keys=[0, 0, 0, 0, 0, 0])
KEY4 = Key(modifiers=0, keys=[0, 0, 0, 0, 0, 0])
KEY5 = Key(modifiers=0, keys=[0, 0, 0, 0, 0, 0])
KEY6 = Key(modifiers=0, keys=[0, 0, 0, 0, 0, 0])
KEY7 = Key(modifiers=0, keys=[0, 0, 0, 0, 0, 0])
KEY8 = Key(modifiers=0, keys=[0, 0, 0, 0, 0, 0])
KEY9 = Key(modifiers=0, keys=[0, 0, 0, 0, 0, 0])


KEYPAD = Keypad([KEY1, KEY2, KEY3, KEY4, KEY5, KEY6, KEY7, KEY8, KEY9])


def main():
    if len(sys.argv) != 2:
        print(f"Usage: python3 {sys.argv[0]} [COMMAND]\n")
        print("COMMAND: GET|SET")
        print("For the SET command, edit the KEY1-9 in file")
        sys.exit(1)

    if sys.argv[1] == "GET":
        print("Getting the MACRO9 configuration")
        get_command()
    elif sys.argv[1] == "SET":
        print("Setting the MACRO9 configuration")
        set_command()
    else:
        print("Invalid command")


def set_command():
    ser = serial.Serial(SERIAL_PORT)

    print("Setting")

    request = bytes([0x4D, 0x39, 0x53] + KEYPAD.structs + [0x00])
    request[66] = calculate_crc(request[0:66])

    ser.write(request)
    response = ser.read(4)

    crc = calculate_crc(response[:4])
    if crc != response[4]:
        print("Invalid CRC")
        return

    if response[:2] != b'M9':
        print("Invalid packet received")
        return

    if response[2] == 0x4B:
        print("Set successful")
    elif response[2] == 0x45:
        print("Set failed")


def get_command():
    ser = serial.Serial(SERIAL_PORT)

    request = bytes([0x4D, 0x39, 0x47, 0x00])
    request[3] = calculate_crc(request[0:3])

    ser.write(request)
    response = ser.read(67)

    crc = calculate_crc(response[:66])
    if crc != response[66]:
        print("Invalid CRC")
        return

    if response[:2] != b'M9' or response[2] != 0x52:
        print("Invalid packet received")
        return

    button_structs = response[3:66]

    for i in range(0, 9):
        index = i * 7
        struct = button_structs[index:(index + 7)]
        print(f"Key {i+1}")
        print_buttons_config(struct)


def print_buttons_config(struct):
    print(f"Modifiers: {hex(struct[0])}")
    print("Buttons: ", end='')
    for button in struct[1:]:
        print(f"{hex(button)} ", end='')
    print()


def calculate_crc(data):
    crc = 0
    for value in data:
        crc ^= value
        for i in range(8):
            if crc & (1 << (8 - 1)):
                crc = (crc << 1) ^ 0x07
            else:
                crc = crc << 1
            crc &= (1 << 8) - 1
    return crc ^ 0


if __name__ == '__main__':
    main()
