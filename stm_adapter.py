import serial

# Open a serial connection
ser = serial.Serial(
    port='/dev/ttyACM0',  # Replace with your serial port
    baudrate=115200,        # Set your baud rate (make sure it matches the device)
    timeout=1             # Optional: Set a timeout for reading
)

# Check if the port is open
if ser.is_open:
    print("Serial port is open.")

try:
    while True:
        if ser.in_waiting > 0:  # Check if data is waiting in the buffer
            data = ser.readline().decode('utf-8').strip()  # Read and decode the data
            print(f"Received: {data}")  # Display the received data
    #TODO : Publish Quaternion to IMU topic
except KeyboardInterrupt:
    print("Program interrupted.")

finally:
    ser.close()  # Close the serial port
    print("Serial port closed.")