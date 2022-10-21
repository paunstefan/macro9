import serial

SERIAL_PORT = "/dev/ttyACM0"

ser = serial.Serial(SERIAL_PORT)
ser.write(b"PING")

resp = ser.read(4)

print(resp)
