import time
import datetime
import platform
import threading
import time
import math
import serial
import struct
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from sensor_msgs.msg import Temperature
from nav_msgs.msg import Odometry

import lib.device_model as deviceModel
from lib.data_processor.roles.dataProcessor import CMA10ADataProcessor
from lib.protocol_resolver.roles.protocol_resolver import WitProtocolResolver

key = 0
flag = 0
buff = {}
angularVelocity = [0, 0, 0]
acceleration = [0, 0, 0]
magnetometer = [0, 0, 0]
angle_degree = [0, 0, 0]
odometry = [0, 0, 0, 0, 0]  # x, y, yaw, linear velocity, angular velocity
# 위치 정보 초기화 (m 단위)
position = [0.0, 0.0, 0.0]  # x, y, z
velocity = [0.0, 0.0, 0.0]  # x, y, z



def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qx, qy, qz, qw]


def handle_serial_data(node, deviceModel):
        
        # 데이터 추출 및 저장
        # node.get_logger().info(
        # "\tTemp:\t" + str(deviceModel.getDeviceData("temperature")) + "\n"
        # , "\tAccel:\t\t" + str(deviceModel.getDeviceData("accX")) +"\t"+  str(deviceModel.getDeviceData("accY")) +"\t"+ str(deviceModel.getDeviceData("accZ")) + "\n"
        # , "\tAngular:\t" + str(deviceModel.getDeviceData("gyroX")) +"\t"+ str(deviceModel.getDeviceData("gyroY")) +"\t"+ str(deviceModel.getDeviceData("gyroZ")) + "\n"
        # , "\tAngle:\t\t" + str(deviceModel.getDeviceData("angleX")) +"\t"+ str(deviceModel.getDeviceData("angleY")) +"\t"+ str(deviceModel.getDeviceData("angleZ")) + "\n"
        # , "\tMagnet:\t\t" + str(deviceModel.getDeviceData("magX")) +"\t"+ str(deviceModel.getDeviceData("magY"))+"\t"+ str(deviceModel.getDeviceData("magZ")) + "\n" 
        # )
        # node.get_logger().info("--------------------------------------------------\n")
        
        temperature = float(deviceModel.getDeviceData("temperature") or 0)
        angle_degree = [float(deviceModel.getDeviceData(f"angle{i}") or 0) for i in ['X', 'Y', 'Z']]
        acceleration = [float(deviceModel.getDeviceData(f"acc{i}") or 0) for i in ['X', 'Y', 'Z']]
        angularVelocity = [float(deviceModel.getDeviceData(f"gyro{i}") or 0) for i in ['X', 'Y', 'Z']]
        magnetometer = [float(deviceModel.getDeviceData(f"mag{i}") or 0) for i in ['X', 'Y', 'Z']]

        node.tmp_msg.header.stamp = node.get_clock().now().to_msg()
        node.tmp_msg.temperature = temperature

        node.imu_msg.header.stamp = node.get_clock().now().to_msg()

        angle_radian = [angle_degree[i] * math.pi / 180.0 for i in range(3)]
        qua = get_quaternion_from_euler(angle_radian[0], angle_radian[1], angle_radian[2])

        node.imu_msg.orientation.x = qua[0]
        node.imu_msg.orientation.y = qua[1]
        node.imu_msg.orientation.z = qua[2]
        node.imu_msg.orientation.w = qua[3]
        node.imu_msg.angular_velocity.x = angularVelocity[0]
        node.imu_msg.angular_velocity.y = angularVelocity[1]
        node.imu_msg.angular_velocity.z = angularVelocity[2]

        node.imu_msg.linear_acceleration.x = acceleration[0]
        node.imu_msg.linear_acceleration.y = acceleration[1]
        node.imu_msg.linear_acceleration.z = acceleration[2]

        node.mag_msg.header.stamp = node.get_clock().now().to_msg()
        node.mag_msg.magnetic_field.x = magnetometer[0]
        node.mag_msg.magnetic_field.y = magnetometer[1]
        node.mag_msg.magnetic_field.z = magnetometer[2]

        node.tmp_pub.publish(node.tmp_msg)
        node.imu_pub.publish(node.imu_msg)
        node.mag_pub.publish(node.mag_msg)


class IMUDriverNode(Node):
    def __init__(self):
        super().__init__('imu_driver_node')

        self.device = deviceModel.DeviceModel(
            "CMP10A",
            WitProtocolResolver(),
            CMA10ADataProcessor(self),
            "51_0"
        )

        self.declare_parameter('port', '/dev/imu_usb')
        self.declare_parameter('baud', 9600)

        port_name = self.get_parameter('port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud').get_parameter_value().integer_value

        self.tmp_msg = Temperature()
        self.tmp_msg.header.frame_id = 'base_link'

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = 'base_link'

        self.mag_msg = MagneticField()
        self.mag_msg.header.frame_id = 'base_link'

        self.tmp_pub = self.create_publisher(Temperature, 'imu/temp_raw', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.mag_pub = self.create_publisher(MagneticField, "imu/mag_raw", 10)

        if (platform.system().lower() == 'linux'):
            self.device.serialConfig.portName = port_name       #Set serial port
        else:
            self.device.serialConfig.portName = "COM17"         #Set serial port
        self.device.serialConfig.baud = baud_rate               #Set baud rate
        

        self.get_logger().info(f"PORT:{port_name}, BAUD:{baud_rate}")
        ret = self.device.openDevice()                          #Open serial port
        if ret:
            self.get_logger().info("\033[32mSerial port opened successfully...\033[0m")
        else:
            self.get_logger().info("\033[31mSerial port opening failure\033[0m")
            exit(0)

        self.readConfig()                                 #Read configuration information
        self.setConfig()
        self.device.dataProcessor.onVarChanged.append(handle_serial_data)  #Data update event

    
    def readConfig(self):
        """
        Example of reading configuration information
        :param device: Device model
        """
        tVals = self.device.readReg(0x02,3)  # Read data content, return rate, communication rate
        if (len(tVals)>0):
            self.get_logger().info("Return results: " + str(tVals))
        else:
            self.get_logger().info("No return")
        
        tVals = self.device.readReg(0x23,2)  # Read the installation direction, algorithm
        if (len(tVals)>0):
            self.get_logger().info("Return results: " + str(tVals))
        else:
            self.get_logger().info("No return")

    def setConfig(self):
        """
        Example of setting configuration information
        :param device: Device model
        """
        self.device.unlock()                # unlock
        time.sleep(0.1)                # Sleep 100ms
        self.device.writeReg(0x03, 6)       # Set the transmission back rate to 10HZ
        time.sleep(0.1)                # Sleep 100ms
        self.device.writeReg(0x23, 0)       # Set the installation direction: horizontal and vertical
        time.sleep(0.1)                # Sleep 100ms
        self.device.writeReg(0x24, 0x01)       # Set the installation direction: nine axis, six axis
        time.sleep(0.1)                # Sleep 100ms
        self.AccelerationCalibration()
        #FieldCalibration(device)
        self.device.save()                  # Save


    def AccelerationCalibration(self):
        """
        Acceleration calibration
        :param device: Device model
        :return:
        """
        self.device.AccelerationCalibration()                 # Acceleration calibration
        self.get_logger().info("Acceleration calibration completed")
        
def main(args=None):
    rclpy.init(args=args)

    try:
        imu_node = IMUDriverNode()
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        imu_node.get_logger().info('Keyboard Interrupt (Ctrl+C) detected. Exiting...')
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()