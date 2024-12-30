#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from .parser import parse_frame, euler_to_quaternion
from sensor_msgs.msg import Imu, Temperature
from im1r_ros2_interface.msg import Im1rExtra
import math

# Constants
TEMP_DBL = -1.0
USED_FRAME_LEN = 68
FRAME_ID = "IM1R"
DEFAULT_PORT = '/dev/ttyUSB0'
DEFAULT_BAUDRATE = 115200

class RealTimeCOM:
    def __init__(self, port, rate=115200, timeout=2):
        self.port = port
        self.rate = rate
        self.timeout = timeout
        self.com = None

    def open(self):
        self.com = serial.Serial(self.port, self.rate, timeout=self.timeout)

    def close(self):
        if self.com and self.com.is_open:
            self.com.close()

    def get_data(self):
        return self.com.readline() if self.com else b''

    def clear_buff(self):
        self.com.reset_input_buffer()

class IM1RDriverNode(Node):
    def __init__(self):
        super().__init__('im1r_driver_node')

        # Declare parameters for serial port and baud rate
        self.declare_parameter('serial_port', DEFAULT_PORT)
        self.declare_parameter('baud_rate', DEFAULT_BAUDRATE)

        # Get parameters passed at runtime
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.serial_baudrate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Initialize publishers
        self.pub_imu_data = self.create_publisher(Imu, 'imu/data', 10)
        self.pub_temperature = self.create_publisher(Temperature, 'temperature', 10)
        self.pub_im1r_extra = self.create_publisher(Im1rExtra, 'im1r/extra', 10)

        # Initialize serial communication
        self.serial_com = RealTimeCOM(self.serial_port, self.serial_baudrate, timeout=1)
        
        # Open serial port
        self.serial_com.open()
        self.serial_com.clear_buff()
        self.get_logger().info(f"Serial Port {self.serial_port} Opened at {self.serial_baudrate} baudrate")

    def publish_imu_data(self, stamp, data):
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_ID
        msg.linear_acceleration.x = data['AccX']
        msg.linear_acceleration.y = data['AccY']
        msg.linear_acceleration.z = data['AccZ']
        msg.angular_velocity.x = data['GyroX'] * (math.pi / 180)
        msg.angular_velocity.y = data['GyroY'] * (math.pi / 180)
        msg.angular_velocity.z = data['GyroZ'] * (math.pi / 180)
        quaternion = euler_to_quaternion(data['Roll'], data['Pitch'], data['Yaw'])
        msg.orientation.w = quaternion[0]
        msg.orientation.x = quaternion[1]
        msg.orientation.y = quaternion[2]
        msg.orientation.z = quaternion[3]
        # msg.orientation_covariance[0] = msg.orientation_covariance[4] = msg.orientation_covariance[8] = TEMP_DBL
        self.pub_imu_data.publish(msg)

    def publish_temperature(self, stamp, data):
        msg = Temperature()
        msg.header.stamp = stamp
        msg.header.frame_id = FRAME_ID
        msg.temperature = data['Temperature']
        # msg.variance = TEMP_DBL
        self.pub_temperature.publish(msg)
    
    def publish_extra_data(self, data):
        msg = Im1rExtra()
        msg.count = data['Count']
        msg.timestamp = data['Timestamp']
        msg.pitch = data['Pitch']
        msg.roll = data['Roll']
        msg.yaw = data['Yaw']
        msg.imu_status = data['IMUStatus']
        msg.gyro_bias_x = data['GyroBiasX'] * (math.pi / 180)
        msg.gyro_bias_y = data['GyroBiasY'] * (math.pi / 180)
        msg.gyro_bias_z = data['GyroBiasZ'] * (math.pi / 180)
        msg.gyro_static_bias_x = data['GyroStaticBiasX'] * (math.pi / 180)
        msg.gyro_static_bias_y = data['GyroStaticBiasY'] * (math.pi / 180)
        msg.gyro_static_bias_z = data['GyroStaticBiasZ'] * (math.pi / 180)
        self.pub_im1r_extra.publish(msg)

    def run(self):
        # Main loop that continuously checks for data from the serial port
        while rclpy.ok():
            data = self.serial_com.get_data()
            while len(data) < USED_FRAME_LEN:
                data += self.serial_com.get_data()
            
            try:
                parsed_data = parse_frame(data)
                if parsed_data:
                    stamp = self.get_clock().now().to_msg()
                    self.publish_imu_data(stamp, parsed_data)
                    self.publish_temperature(stamp, parsed_data)
                    self.publish_extra_data(parsed_data)
                else:
                    pass
            except ValueError as e:
                self.get_logger().warn(f"Value error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    # Get serial port and baud rate from the command line arguments and set parameters
    node = IM1RDriverNode()

    # The parameters are set by the ros2 run command or launch file, so no need to set them here.
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.serial_com.close()  # Close serial port when shutting down
        rclpy.shutdown()

if __name__ == '__main__':
    main()
