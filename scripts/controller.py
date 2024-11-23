#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import tf_transformations
from enum import Enum

class States(Enum):
    ROLL = 0
    PITCH = 1
    THROTTLE = 2
    YAW = 3

class Controller(Node):
    def __init__(self):
        super().__init__('imu_listener')
        self.subscription = self.create_subscription(
            Imu,
            'imu',  # Change this to your IMU topic name
            self.listener_callback,
            10
        )
        self.timer = self.create_timer(0.1,self.loop)
        self.stable_state = [0.0,0.0,0.0,3.0] # rpy throttle
        self.error = [0,0,0]
        self.prev_error = [0,0,0]
        self.integral_error = [0,0,0]
        self.pid_error_pub = self.create_publisher(PIDError,"/pid_error",10)
        self.pwm_pub = self.create_publisher(PWM,"/pwm",10)

		self.Kp = [0, 0.0, 20]
		self.Ki = [0.0, 0.0, 0.05]
		self.Kd = [0, 0, 100]
		self.dt = 0.1 # This is the sample time in which you 

    def listener_callback(self, msg):
        # Extract quaternion from the IMU message
        orientation = msg.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)

        # Convert quaternion to roll, pitch, yaw
        self.rpy = tf_transformations.euler_from_quaternion(quaternion)

        # Log the quaternion and RPY values
        self.get_logger().info(f'Received quaternion: {quaternion}')
        self.get_logger().info(f'Converted to RPY: roll={self.rpy[0]}, pitch={self.rpy[1]}, yaw={self.rpy[2]}')

    def loop(self):
        self.pid(States.ROLL.value,States.ROLL.name)
        self.pid(States.PITCH.value,States.PITCH.name)
        self.pid(States.THROTTLE.value,States.THROTTLE.name)
        self.pid(States.YAW.value,States.YAW.name)

        self.error[state] = self.stable_state[]

    def pid(self, state, name):
		"""
		PID controller to compute the control action.

		Arguments:
		state(x / y / z) -- To which state PID is being applied 
		name -- The name of the state (roll, pitch, etc.)

		Returns:
		control_action -- The computed PID value for velocities
		"""

		# Calculate the current error for the state
		self.error[state] = self.drone_position[state] - self.setpoint[state]

		# Update integral error
		self.integral_error[state] += self.error[state]

		# Calculate the control action using PID formula
		self.control_action[state]  = 	self.Kp[state] * self.error[state] + \
										self.Ki[state] * self.integral_error[state] + \
										self.Kd[state] * ((self.error[state] - self.prev_error[state]) / self.dt)

		# Print PID coefficients for debugging
		print(f"{name} Kp : {self.Kp[state]}")
		print(f"{name} Ki : {self.Ki[state]}")
		print(f"{name} Kd : {self.Kd[state]}")

    def publish_pid_errors(self) :
            print(self.error)
            err = PIDError()
            err.roll_error = float(self.error[0])
            err.pitch_error = float(self.error[1])
            err.throttle_error = float(self.error[2])
            # err.yaw_error = self.error

            self.pid_error_pub.publish(err)

def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
