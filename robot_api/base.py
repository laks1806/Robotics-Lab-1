#! /usr/bin/env python

# Import the necessary ROS message type
from geometry_msgs.msg import Twist
import rospy

class Base(object):
    """Base controls the mobile base portion of the Fetch robot.

    Sample usage:
        base = robot_api.Base()
        while CONDITION:
            base.move(0.2, 0)
        base.stop()
    """

    def __init__(self):
        # Create publisher for sending velocity commands
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def move(self, linear_speed, angular_speed):
        """Moves the base instantaneously at given linear and angular speeds.

        "Instantaneously" means that this method must be called continuously in
        a loop for the robot to move.

        Args:
            linear_speed: The forward/backward speed, in meters/second. A
                positive value means the robot should move forward.
            angular_speed: The rotation speed, in radians/second. A positive
                value means the robot should rotate clockwise.
        """
        # Create Twist message
        msg = Twist()
        # Fill out message with provided speeds
        msg.linear.x = linear_speed
        msg.angular.z = angular_speed
        # Publish the message
        self.cmd_vel_pub.publish(msg)

    def stop(self):
        """Stops the mobile base from moving.
        """
        # Convenience method to stop the robot by setting linear and angular speeds to 0
        self.move(0, 0)
        rospy.logerr('Not implemented.')
