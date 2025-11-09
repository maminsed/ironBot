# pip install pyserial
import serial, time

PORT = "COM4"   # change to your COM port
BAUD = 115200   # match ESP32 (can be 9600, 115200, etc.)
ser = serial.Serial(PORT, BAUD, timeout=1)

# Send a one-word command
cmd = "FORWARD\n"           # add \n if your ESP32 reads lines
ser.write(cmd.encode())

# Read a line back (optional)
time.sleep(0.05)
resp = ser.readline().decode(errors="ignore").strip()
print("ESP32:", resp)

ser.close()
