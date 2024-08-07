#! /usr/bin/env python3
"""Program to convert the orientation data to joint states."""

import rospy
from sensor_msgs.msg import Imu, JointState
from tf.transformations import euler_from_quaternion


class TorsoControl():
    """Torso Control Class."""

    def __init__(self):
        """Initialize the Torso Control Class."""
        rospy.init_node('torso_control_node', anonymous=True)

        update_rate = 50
        time_period = 1. / update_rate

        self.torso_roll = 0.0
        self.torso_pitch = 0.0
        self.torso_yaw = 0.0

        # Define Subscribers
        rospy.Subscriber('/imu/data_raw', Imu, self.imu_callback)

        # Define Publishers
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Define Timer
        rospy.Timer(rospy.Duration(time_period), self.timer_callback)

        # Shutdown Function
        rospy.on_shutdown(self.terminate)

    def imu_callback(self, msg):
        """Callback function for the IMU data."""
        orientation = msg.orientation
        orientation_list = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)

        self.torso_roll = roll
        self.torso_pitch = pitch
        self.torso_yaw = yaw

    def timer_callback(self, _):
        """Callback function for the timer."""
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = [
            'torsoYaw', 'torsoPitch', 'torsoRoll',
            'lowerNeckPitch', 'neckYaw', 'upperNeckPitch',
            'rightHipYaw', 'rightHipRoll', 'rightHipPitch',
            'rightKneePitch', 'rightAnklePitch', 'rightAnkleRoll',
            'leftHipYaw', 'leftHipRoll', 'leftHipPitch',
            'leftKneePitch', 'leftAnklePitch', 'leftAnkleRoll',
            'rightShoulderPitch', 'rightShoulderRoll', 'rightShoulderYaw',
            'rightElbowPitch', 'rightForearmYaw', 'rightWristRoll',
            'rightWristPitch','rightThumbRoll', 'rightThumbPitch1',
            'rightIndexFingerPitch1', 'rightMiddleFingerPitch1', 'rightPinkyPitch1',
            'leftShoulderPitch', 'leftShoulderRoll', 'leftShoulderYaw',
            'leftElbowPitch', 'leftForearmYaw', 'leftWristRoll',
            'leftWristPitch', 'leftThumbRoll', 'leftThumbPitch1',
            'leftIndexFingerPitch1', 'leftMiddleFingerPitch1', 'leftPinkyPitch1'
            ]
        joint_state_msg.position = [
            self.torso_yaw, self.torso_pitch, self.torso_roll,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
            ]
        self.joint_state_pub.publish(joint_state_msg)


    def terminate(self):
        """Terminate the node."""
        rospy.logwarn("Shutting down the Torso Control Node.")


def main():
    """Mimic Main Function."""
    try:
        TorsoControl()
        rospy.spin()
    except (rospy.exceptions.ROSInitException,
            rospy.exceptions.ROSException, KeyboardInterrupt):
        rospy.signal_shutdown("Done")


if __name__ == '__main__':
    main()
