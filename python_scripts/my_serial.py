import serial
import time
import struct

# Adjust the port to match yours (e.g., COM3 on Windows, /dev/ttyACM0 or /dev/ttyUSB0 on Linux)
PORT = '/dev/ttyACM0'
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # Wait for Pico to reset
    print("Connected to Pico")

    # Send data to Pico
    # ser.write(b"Hello Pico!\n")

    while True:
        if ser.in_waiting:
            msg = ser.readline().decode('utf-8').strip()
            print(f"Pico \: {msg}")

        data = struct.pack('<BBB', 69, 20, 50)  # B = unsigned char (8 bits)
        ser.write(data)

        # user_input = input("Send to Pico (or 'exit'): ")
        # if user_input.lower() == 'exit':
        #     break
        # Kp = 0.1
        # Ki = 0.01
        # ser.write((f'{str(Kp)} {str(Ki)}' + '\n').encode())

except serial.SerialException as e:
    print(f"Error: {e}")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial connection closed.")
