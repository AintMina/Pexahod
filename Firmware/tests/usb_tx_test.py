import serial
import struct
import random
import time


def calculate_crc(data):
    sum_bytes = sum(data[:-1])  # Sum all bytes except the last one
    crc = (0x100 - (sum_bytes & 0xFF)) & 0xFF  # CRC calculation
    return crc

# Define the command_message_t struct
class NrfMessage:
    def __init__(self, prefix, id, command, data):
        self.prefix = prefix
        self.id = id
        self.command = command
        self.data = data[:8]  # Ensure data is at most 8 bytes
        self.crc = 0          # Will calculate CRC later

    def pack(self):
        # Pack the data according to the struct layout
        packed_data = struct.pack('B B B 8B B',
                                   self.prefix,
                                   self.id,
                                   self.command,
                                   *self.data,
                                   self.crc)
        return packed_data

    def finalize(self):
        # Calculate and set the CRC
        data_to_crc = struct.pack('B B B 8B B', self.prefix, self.id, self.command, *self.data, self.crc)
        self.crc = calculate_crc(data_to_crc)
        return self.pack()

# Configure serial port
serial_port = '/dev/ttyACM0'  # Change to your actual serial port
baud_rate = 115200             # Set baud rate
timeout = 1                    # 1 second timeout

# Create a serial connection
with serial.Serial(serial_port, baud_rate, timeout=timeout) as ser:
    # Example data to send
    prefix = 0x69     # Example prefix
    id = 0x01         # Example ID
    command = 0x05    # Example command
    data = [random.randint(0, 255) for _ in range(8)]  # Example data (8 bytes)

    # Create an instance of the message
    message = NrfMessage(prefix, id, command, data)

    # Finalize the message and get the packed data
    packed_message = message.finalize()

    # Send the packed message to the MCU
    ser.write(packed_message)

    # Unpack the packed message to display each field
    unpacked_message = struct.unpack('B B B 8B B', packed_message)
    sent_prefix = unpacked_message[0]
    sent_id = unpacked_message[1]
    sent_command = unpacked_message[2]
    sent_data = list(unpacked_message[3:11])  # Extract the 8 data bytes
    sent_crc = unpacked_message[11]

    # Print the sent message in a structured format
    print("Sent message:")
    print(f"Prefix: {sent_prefix}")
    print(f"ID: {sent_id}")
    print(f"Command: {sent_command}")
    print(f"Data: {sent_data}")
    print(f"CRC: {sent_crc}")
    print("")

    # Wait for a response
    time.sleep(0.1)  # Small delay to allow MCU to respond
    response = ser.read(12)  # Read the expected number of bytes (struct size)

    # Unpack the response
    unpacked_data = struct.unpack('B B B 8B B', response)

    # Extract fields from the unpacked data
    response_prefix = unpacked_data[0]
    response_id = unpacked_data[1]
    response_command = unpacked_data[2]
    response_data = list(unpacked_data[3:11])  # Extract the 8 data bytes
    response_crc = unpacked_data[11]

    print("Received response:")
    print(f"Prefix: {response_prefix}")
    print(f"ID: {response_id}")
    print(f"Command: {response_command}")
    print(f"Data: {response_data}")
    print(f"CRC: {response_crc}")
    print("-" * 30)

    # Verify CRC
    data_to_crc = struct.pack('B B B 8B B', response_prefix, response_id, response_command, *response_data, response_crc)
    calculated_crc = calculate_crc(data_to_crc)
    if calculated_crc == response_crc:
        print("CRC is valid.")
    else:
        print("CRC is invalid.")
