import serial
import struct

# Define the struct format based on your C struct `command_message_t`
# Format string explanation:
#   B  -> unsigned char (1 byte)
#   B  -> unsigned char (1 byte)
#   B  -> enum value as an unsigned char (1 byte)
#   8B -> 8 unsigned chars (8 bytes for the data array)
#   B  -> unsigned char (1 byte for CRC)
nrf_message_format = "BBBB4BB"
message_size = struct.calcsize(nrf_message_format)

# Adjust the serial port settings
SERIAL_PORT = "/dev/ttyACM0"  # Change to the appropriate port (e.g., "COM3" on Windows)
BAUDRATE = 115200             # Match this with your device's baud rate

def read_nrf_message(serial_port):
    """Reads and parses messages, synchronizing to the prefix byte `0x69`."""
    while True:
        # Read one byte at a time to find the sync byte (0x69)
        prefix_byte = serial_port.read(1)
        
        if len(prefix_byte) == 1 and prefix_byte[0] == 0x69:
            # Potential start of a message; try reading the rest
            remaining_data = serial_port.read(message_size - 1)
            
            if len(remaining_data) == message_size - 1:
                # Complete message received
                data = prefix_byte + remaining_data
                
                # Unpack the binary data into fields
                prefix, length, msg_id, command, *data_array, crc = struct.unpack(nrf_message_format, data)
                
                # Print the message details
                print("Received Synchronized Message:")
                print(f"  Prefix: {prefix}")
                print(f"  Length: {length}")
                print(f"  ID: {msg_id}")
                print(f"  Command: {command}")

                byte_array = bytearray([data_array[0], data_array[1], data_array[2], data_array[3]])
                float_value = struct.unpack('f', byte_array)[0]
                print(f"  Value: {float_value}")
                
                print(f"  Data: {data_array}")
                print(f"  CRC: {crc}")
                print("-" * 30)
            else:
                print("Incomplete message after finding prefix, resynchronizing...")
        else:
            # If prefix_byte isn't 0x69, we continue searching
            print("Searching for sync byte 0x69...")

# Initialize and open the serial port
with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
    print(f"Listening on {SERIAL_PORT} at {BAUDRATE} baud...")
    read_nrf_message(ser)