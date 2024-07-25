#! /usr/bin/env python3
"""Program to get the spine angle of the actor."""

import math
import rospy
from gazebo_msgs.msg import LinkStates
from tf.transformations import euler_from_quaternion


class GetSpineAngle():
    """Get the Spine Angle of the Actor."""

    def __init__(self):
        """Initialize the GetSpineAngle Class."""
        rospy.init_node('spine_angle_node', anonymous=True)

        update_rate = 50
        time_period = 1. / update_rate

        self.spine_angle = [0, 0, 0]

        # Subscribers
        rospy.Subscriber("/gazebo/link_states", LinkStates, self.link_states_callback)

        # Timers
        rospy.Timer(rospy.Duration(time_period), self.timer_update)

        # Shutdown Function
        rospy.on_shutdown(self.terminate)

    def link_states_callback(self, msg):
        """Get the spine angle of the actor."""
        # Get the index of the link
        index = msg.name.index("npc::mixamorig_Spine")

        # Get the orientation of the link
        orientation = msg.pose[index].orientation

        # Convert the orientation to euler angles
        euler_angles = euler_from_quaternion([orientation.x, orientation.y,
                                              orientation.z, orientation.w])

        # Get the spine roll, pitch and yaw
        roll, pitch, yaw = math.degrees(euler_angles[0]), math.degrees(euler_angles[1]), \
            math.degrees(euler_angles[2])

        self.spine_angle = [roll, pitch, yaw]

    def timer_update(self, _):
        """Print the spine angle of the actor."""
        rospy.loginfo(f"Spine Angle: {round(self.spine_angle[1], 2)}")

    def terminate(self):
        """Terminate the node."""
        rospy.logwarn("Shutting down the spine_angle_node.")


def main():
    """Mimic Main Function."""
    try:
        GetSpineAngle()
        rospy.spin()
    except (rospy.exceptions.ROSInitException,
            rospy.exceptions.ROSException, KeyboardInterrupt):
        rospy.signal_shutdown("Done")


if __name__ == '__main__':
    main()
