import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
import tf_transformations

class RaspiAdapter(Node):
    def __init__(self):
        super().__init__('serial_data_node')

        # Set up the serial connection
        self.ser = serial.Serial(
            port='/dev/ttyACM0',  # Replace with your serial port
            baudrate=115200,      # Set your baud rate (ensure it matches the device)
            timeout=1             # Optional: Set a timeout for reading
        )

        # Check if the port is open
        if self.ser.is_open:
            self.get_logger().info("Serial port is open.")
        # Timer to read data periodically (adjust the period as needed)
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def euler_to_quaternion(self,roll, pitch, yaw):
        # Convert Euler angles (rpy) to a Quaternion
        quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
        return quaternion[0], quaternion[1], quaternion[2], quaternion[3]


    def read_serial_data(self):
        if self.ser.in_waiting > 0:  # Check if data is available
            data = self.ser.readline().decode('utf-8').strip()  # Read and decode the data
            self.get_logger().info(f"Received: {data}")

            # Split the data into individual readings
            values = data.split(',')  # ['0.0', '0.0', '0.0', '1.0', '2.0', '3.0']
            msg = Imu()
            try:
                # Convert the values to floating-point numbers
                x = float(values[0])
                y = float(values[1])
                z = float(values[2])

                r = float(values[3])
                p = float(values[4])
                y = float(values[5])

                quat_x, quat_y, quat_z, quat_w = self.euler_to_quaternion(r,p,y)
                msg.

            except ValueError:
                self.get_logger().error(f"Error parsing values: {data}")

    def close_serial(self):
        self.ser.close()  # Close the serial port
        self.get_logger().info("Serial port closed.")

def main(args=None):
    rclpy.init(args=args)
    # Create the node
    node = RaspiAdapter()

    try:
        # Spin the node to keep it active
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.close_serial()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
