import evdev
import serial
import struct
import time
from evdev import ecodes, InputDevice

# Constants for the serial message
PREFIX = 0x69
ID = 0x00
COMMAND_LEFT_J_X = 1  # Enum value for LEFT_J_X
COMMAND_LEFT_J_Y = 2  # Enum value for LEFT_J_Y
COMMAND_RIGHT_J_X = 4  # Enum value for LEFT_J_Y
COMMAND_RIGHT_J_Y = 5  # Enum value for RIGHT_J_Y

# Device paths (change as needed)
device_path = '/dev/input/event23'  # Replace 'X' with your joystick's event number
serial_port = '/dev/ttyACM1'       # Path to the USB serial device

# Initialize the joystick input device
try:
    device = InputDevice(device_path)
    print(f"Listening to {device.name} at {device_path}")
except FileNotFoundError:
    print("Device not found. Make sure the joystick is connected and the path is correct.")
    exit(1)

# Initialize serial communication
try:
    ser = serial.Serial(serial_port, 115200, timeout=1)
    print(f"Opened serial port {serial_port}")
except serial.SerialException:
    print("Could not open serial port. Make sure the device is connected.")
    exit(1)

# Define joystick axis codes for the left joystick
LEFT_JOYSTICK_X = ecodes.ABS_X
LEFT_JOYSTICK_Y = ecodes.ABS_Y
RIGHT_JOYSTICK_X = ecodes.ABS_RX
RIGHT_JOYSTICK_Y = ecodes.ABS_RY

# Initialize joystick position
joystick_position = {'left_x': 0, 'left_y': 0, 'right_x': 0, 'right_y': 0}

# Rate limiting configuration
rate_limit_interval = 0.02  # 10 updates per second
last_sent_time = {'left_x': 0, 'left_y': 0, 'right_x': 0, 'right_y': 0}

def calculate_crc(data):
    sum_bytes = sum(data[:])  # Sum all bytes except the last one
    crc = (0x100 - (sum_bytes & 0xFF)) & 0xFF  # CRC calculation
    return crc

def normalize(value, min_input, max_input):
    """Normalize the joystick value to a float between -1.0 and 1.0."""
    return (2 * ((value - min_input) / (max_input - min_input))) - 1

def send_serial_message(command, data_float):
    """Send data to the serial port using the nrf_message_t structure."""
    data = struct.pack('<f', data_float)  # Little-endian float
    length = len(data)
    message = struct.pack('<BBB', PREFIX, length, ID) + bytes([command]) + data
    crc = calculate_crc(message)
    message += bytes([crc])
    ser.write(message)

try:
    # Event loop to read joystick data
    for event in device.read_loop():
        current_time = time.time()
        
        if event.type == ecodes.EV_ABS:
            if event.code == LEFT_JOYSTICK_X:
                joystick_position['left_x'] = normalize(event.value, device.absinfo(LEFT_JOYSTICK_X).min, device.absinfo(LEFT_JOYSTICK_X).max)
                if current_time - last_sent_time['left_x'] >= rate_limit_interval:
                    send_serial_message(COMMAND_LEFT_J_X, joystick_position['left_x'])
                    last_sent_time['left_x'] = current_time

            elif event.code == LEFT_JOYSTICK_Y:
                joystick_position['left_y'] = normalize(event.value, device.absinfo(LEFT_JOYSTICK_Y).min, device.absinfo(LEFT_JOYSTICK_Y).max)
                if current_time - last_sent_time['left_y'] >= rate_limit_interval:
                    send_serial_message(COMMAND_LEFT_J_Y, joystick_position['left_y'])
                    last_sent_time['left_y'] = current_time

            elif event.code == RIGHT_JOYSTICK_X:  # Use ABS_RZ or axis 4 if necessary
                joystick_position['right_x'] = normalize(event.value, device.absinfo(RIGHT_JOYSTICK_X).min, device.absinfo(RIGHT_JOYSTICK_X).max)
                if current_time - last_sent_time['right_x'] >= rate_limit_interval:
                    send_serial_message(COMMAND_RIGHT_J_X, joystick_position['right_x'])
                    last_sent_time['right_x'] = current_time

            elif event.code == RIGHT_JOYSTICK_Y:  # Use ABS_RY or axis 5 if necessary
                joystick_position['right_y'] = normalize(event.value, device.absinfo(RIGHT_JOYSTICK_Y).min, device.absinfo(RIGHT_JOYSTICK_Y).max)
                if current_time - last_sent_time['right_y'] >= rate_limit_interval:
                    send_serial_message(COMMAND_RIGHT_J_Y, joystick_position['right_y'])
                    last_sent_time['right_y'] = current_time

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()