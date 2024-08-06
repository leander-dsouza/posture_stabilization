#! /usr/bin/env python3
"""Program to generate boilerplate code for ROS1 scripts."""

import asyncio
import numpy as np
from bleak import BleakClient

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion


# Constants
MOVELLA_BT_DOT_UUID = "D4:22:CD:00:A7:18"
MEASUREMENT_UUID = '15172001-4947-11e9-8646-d663bd873d93'
MEDIUM_PAYLOAD_UUID = "15172003-4947-11E9-8646-D663BD873D93"


CUSTOM_MODE_1 = bytes([1, 1, 22]) # [Timestamp, Euler, Free Acceleration, Angular Velocity]
RATE_WITH_MAG = bytes([1, 1, 20]) # [Timestamp, Acceleration, Angular Velocity, Magnetic Field]


class WirelessXSensDriver():
    """Wireless XSens Driver Class."""

    def __init__(self):
        """Initialize the Wireless XSens Driver Class."""
        rospy.init_node('wireless_xsens_driver', anonymous=True)

        # Define Publisher
        self.imu_pub = rospy.Publisher('/imu_raw', Imu, queue_size=10)

        asyncio.run(self.bt_listen())

        # Shutdown Function
        rospy.on_shutdown(self.terminate)

    def notification_callback(self, _, data):
        """Callback function for the notification."""
        encoded_data = self.encode_data(data)
        # print(encoded_data)

        # Publish IMU Data
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "base_link"

        # Set Orientation
        euler = encoded_data['euler'][0]
        quaternion = quaternion_from_euler(euler[0], euler[1], euler[2])
        imu_msg.orientation = Quaternion(*quaternion)
        imu_msg.orientation_covariance = [1e-9, 0, 0, 0, 1e-9, 0, 0, 0, 1e-9]

        # Set Angular Velocity
        angular_velocity = encoded_data['angular_velocity'][0]
        imu_msg.angular_velocity.x = angular_velocity[0]
        imu_msg.angular_velocity.y = angular_velocity[1]
        imu_msg.angular_velocity.z = angular_velocity[2]
        imu_msg.angular_velocity_covariance = [1e-9, 0, 0, 0, 1e-9, 0, 0, 0, 1e-9]

        # Set Linear Acceleration
        free_acceleration = encoded_data['free_acceleration'][0]
        imu_msg.linear_acceleration.x = free_acceleration[0]
        imu_msg.linear_acceleration.y = free_acceleration[1]
        imu_msg.linear_acceleration.z = free_acceleration[2]
        imu_msg.linear_acceleration_covariance = [1e-9, 0, 0, 0, 1e-9, 0, 0, 0, 1e-9]

        self.imu_pub.publish(imu_msg)


    def encode_data(self, bytes_):
        """Encode the data."""
        custom_mode_1_segments = np.dtype([
            ('timestamp', np.uint32, 1),
            ('euler', np.float32, 3),
            ('free_acceleration', np.float32, 3), # earth's gravity is deducted
            ('angular_velocity', np.float32, 3),
            ])

        rate_with_mag_segments = np.dtype([
            ('timestamp', np.uint32, 1),
            ('acceleration', np.float32, 3),
            ('angular_velocity', np.float32, 3),
            ('magnetic_field', np.float32, 3),
            ])

        formatted_data = np.frombuffer(bytes_, dtype=custom_mode_1_segments)
        return formatted_data

    async def bt_listen(self):
        """Listen to the Bluetooth."""
        async with BleakClient(MOVELLA_BT_DOT_UUID) as client:
            # Check if connection was successful
            print(f"Client connection: {client.is_connected}") # prints True or False

            # Subscribe to notifications from the Medium Payload Characteristic
            await client.start_notify(MEDIUM_PAYLOAD_UUID,
                                        self.notification_callback)

            binary_message = CUSTOM_MODE_1
            await client.write_gatt_char(MEASUREMENT_UUID,
                                         binary_message, response=True)

            # Wait for the user to press Ctrl+C
            await asyncio.sleep(10)


    def terminate(self):
        """Terminate the node."""
        rospy.logwarn("Shutting down the Wireless XSens Driver Node")


def main():
    """Mimic Main Function."""
    try:
        WirelessXSensDriver()
        rospy.spin()
    except (rospy.exceptions.ROSInitException,
            rospy.exceptions.ROSException, KeyboardInterrupt):
        rospy.signal_shutdown("Done")


if __name__ == '__main__':
    main()
