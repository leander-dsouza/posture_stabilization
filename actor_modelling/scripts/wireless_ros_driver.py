#! /usr/bin/env python3
"""Program to extract wireless data from XSens IMU."""

import math
import asyncio
import numpy as np
from bleak import BleakClient

import rospy
from sensor_msgs.msg import Imu, MagneticField
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
        rospy.init_node('wireless_xsens_driver_node', anonymous=True)

        # Get parameters
        self.operating_mode = rospy.get_param('~operating_mode', 'rate_with_mag')

        # Define Publishers
        self.imu_pub = rospy.Publisher('/imu/data_raw', Imu, queue_size=10)
        self.mag_pub = rospy.Publisher('/imu/mag', MagneticField, queue_size=10)

        asyncio.run(self.bt_listen())

        # Shutdown Function
        rospy.on_shutdown(self.terminate)

    def notification_callback(self, _, data):
        """Callback function for the notification."""
        encoded_data = self.encode_data(data)

        # Publish IMU Data
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "base_link"


        if self.operating_mode == 'custom_mode_1':
            # Set Orientation
            euler = encoded_data['euler'][0]
            quaternion = quaternion_from_euler(math.radians(euler[0]),
                                               math.radians(euler[1]),
                                               math.radians(euler[2]))
            imu_msg.orientation = Quaternion(*quaternion)
            # Set Linear Acceleration
            acceleration = encoded_data['free_acceleration'][0]

        elif self.operating_mode == 'rate_with_mag':
            # Set Orientation
            # imu_msg.orientation = Quaternion(0, 0, 0, 1)
            # Set Linear Acceleration
            acceleration = encoded_data['acceleration'][0]
            # Publish Magnetic Field Data
            mag_msg = MagneticField()
            mag_msg.header.stamp = rospy.Time.now()
            mag_msg.header.frame_id = "base_link"

            magnetic_field = encoded_data['magnetic_field'][0]
            mag_msg.magnetic_field.x = float(magnetic_field[0])
            mag_msg.magnetic_field.y = float(magnetic_field[1])
            mag_msg.magnetic_field.z = float(magnetic_field[2])
            self.mag_pub.publish(mag_msg)

        imu_msg.linear_acceleration.x = acceleration[0]
        imu_msg.linear_acceleration.y = acceleration[1]
        imu_msg.linear_acceleration.z = acceleration[2]

        # Set Angular Velocity
        angular_velocity = encoded_data['angular_velocity'][0]
        imu_msg.angular_velocity.x = math.radians(angular_velocity[0])
        imu_msg.angular_velocity.y = math.radians(angular_velocity[1])
        imu_msg.angular_velocity.z = math.radians(angular_velocity[2])

        self.imu_pub.publish(imu_msg)


    def encode_data(self, bytes_):
        """Encode the data."""
        # timestamp: 4 bytes
        # euler: 3 * 4 bytes
        # free_acceleration: 3 * 4 bytes
        # angular_velocity: 3 * 4 bytes
        # Total: 4 + 3*4 + 3*4 + 3*4 = 40 bytes
        custom_mode_1_segments = np.dtype([
            ('timestamp', np.uint32),
            ('euler', np.float32, 3),
            ('free_acceleration', np.float32, 3), # earth's gravity is deducted
            ('angular_velocity', np.float32, 3),
            ])

        # timestamp: 4 bytes
        # acceleration: 3 * 4 bytes
        # angular_velocity: 3 * 4 bytes
        # magnetic_field: 3 * 2 bytes - fixed point
        # Total: 4 + 3*4 + 3*4 + 3*2 = 34 bytes
        rate_with_mag_segments = np.dtype([
            ('timestamp', np.uint32),
            ('acceleration', np.float32, 3),
            ('angular_velocity', np.float32, 3),
            ('magnetic_field', np.int16, 3),
            ('zero_padding', np.uint16, 3),
            ])

        if self.operating_mode == 'custom_mode_1':
            formatted_data = np.frombuffer(bytes_, dtype=custom_mode_1_segments)
        elif self.operating_mode == 'rate_with_mag':
            formatted_data = np.frombuffer(bytes_, dtype=rate_with_mag_segments)
        return formatted_data

    async def bt_listen(self):
        """Listen to the Bluetooth."""
        async with BleakClient(MOVELLA_BT_DOT_UUID) as client:
            # Check if connection was successful
            print(f"Client connection: {client.is_connected}") # prints True or False

            # Subscribe to notifications from the Medium Payload Characteristic
            await client.start_notify(MEDIUM_PAYLOAD_UUID,
                                        self.notification_callback)

            if self.operating_mode == 'custom_mode_1':
                binary_message = CUSTOM_MODE_1
            elif self.operating_mode == 'rate_with_mag':
                binary_message = RATE_WITH_MAG

            await client.write_gatt_char(MEASUREMENT_UUID,
                                         binary_message, response=True)

            # Wait for the user to press Ctrl+C
            await asyncio.sleep(100)

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
