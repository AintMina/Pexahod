import serial

# Set up the serial port
# Replace 'COM3' with your port name (e.g., '/dev/ttyUSB0' for Linux or Mac)
port = '/dev/ttyACM1'
baudrate = 115200

# Initialize the serial connection
ser = serial.Serial(port, baudrate)

print(f"Connected to {port} at {baudrate} baudrate.")

try:
    while True:
        if ser.in_waiting > 0:  # Check if there are incoming bytes
            byte = ser.read(1)  # Read one byte from the port
            print(f"Received byte: {byte.hex()}")
except KeyboardInterrupt:
    print("\nProgram interrupted by user.")
finally:
    ser.close()  # Close the serial connection
    print("Serial connection closed.")