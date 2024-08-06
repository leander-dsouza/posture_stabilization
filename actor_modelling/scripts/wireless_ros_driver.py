#! /usr/bin/env python3
"""Program to generate boilerplate code for ROS1 scripts."""

import asyncio
import numpy as np
from bleak import BleakClient

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Quaternion



measurement_characteristic_uuid = '15172001-4947-11e9-8646-d663bd873d93'
short_payload_characteristic_uuid = "15172004-4947-11e9-8646-d663bd873d93"
medium_payload_characteristic_uuid = "15172003-4947-11E9-8646-D663BD873D93"


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
        print(self.encode_data(data))

        # Publish IMU Data
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "base_link"

        # Set the orientation
        orientation = self.encode_data(data)
        q = quaternion_from_euler(orientation['x'], orientation['y'], orientation['z'])
        imu_msg.orientation = Quaternion(*q)

        self.imu_pub.publish(imu_msg)


    def encode_data(self, bytes_):
        """Encode the data."""
        # data_segments = np.dtype([
        #     ('timestamp', np.uint32),
        #     ('roll', np.float32),
        #     ('pitch', np.float32),
        #     ('yaw', np.float32),
        #     ('freeAccX', np.float32),
        #     ('freeAccY', np.float32),
        #     ('freeAccZ', np.float32),
        #     ('gyroX', np.float32),
        #     ('gyroY', np.float32),
        #     ('gyroZ', np.float32),
        #     ('zero_padding', np.uint32)
        #     ])

        data_segments = np.dtype([
            ('timestamp', np.uint32),
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('zero_padding', np.uint32)
            ])
        formatted_data = np.frombuffer(bytes_, dtype=data_segments)
        return formatted_data

    async def bt_listen(self):
        """Listen to the Bluetooth."""
        address = "D4:22:CD:00:A7:18" # Movella DOT UUID

        async with BleakClient(address) as client:
            # Check if connection was successful
            print(f"Client connection: {client.is_connected}") # prints True or False

            # Subscribe to notifications from the Short Payload Characteristic
            await client.start_notify(short_payload_characteristic_uuid,
                                      self.notification_callback)

            # Subscribe to notifications from the Medium Payload Characteristic
            # await client.start_notify(medium_payload_characteristic_uuid,
            #                           self.notification_callback)

            # Set and turn on the Custom mode 1 - Timestamp, Euler, Free Acc, Free Angular Vel
            # binary_message = b"\x01\x01\x22"
            binary_message = b"\x01\x01\x04"
            await client.write_gatt_char(measurement_characteristic_uuid,
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
