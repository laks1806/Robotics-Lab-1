#!/usr/bin/env python

# TODO: import ?????????
# TODO: import ???????_msgs.msg
# TODO: import ??????????_msgs.msg
import rospy
import actionlib
import control_msgs.msg
import trajectory_msgs.msg
import rospy

# TODO: ACTION_NAME = ???
# TODO: JOINT_NAME = ???
ACTION_NAME = "torso_controller/follow_joint_trajectory"
JOINT_NAME = ["torso_lift_joint"]
TIME_FROM_START = 5  # How many seconds it should take to set the torso height.


class Torso(object):
    """Torso controls the robot's torso height.
    """
    MIN_HEIGHT = 0.0
    MAX_HEIGHT = 0.4

    def __init__(self):
        # TODO: Create actionlib client
        # TODO: Wait for server
        self.client = actionlib.SimpleActionClient(ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        self.client.wait_for_server()

    def set_height(self, height):
        """Sets the torso height.

        This will always take ~5 seconds to execute.

        Args:
            height: The height, in meters, to set the torso to. Values range
                from Torso.MIN_HEIGHT (0.0) to Torso.MAX_HEIGHT(0.4).
        """
        # TODO: Check that the height is between MIN_HEIGHT and MAX_HEIGHT.
        if Torso.MIN_HEIGHT <= height <= Torso.MAX_HEIGHT:
            point = trajectory_msgs.msg.JointTrajectoryPoint()
            point.positions = [height]
            point.time_from_start = rospy.Duration(TIME_FROM_START)
        # TODO: Create a trajectory point
        # TODO: Set position of trajectory point
        # TODO: Set time of trajectory point

        # TODO: Create goal
        # TODO: Add joint name to list
        # TODO: Add the trajectory point created above to trajectory
            goal = control_msgs.msg.FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = JOINT_NAME
            goal.trajectory.points.append(point)


        # TODO: Send goal
            self.client.send_goal(goal)
        # TODO: Wait for result
            self.client.wait_for_result(rospy.Duration(5.0))
        else:
            rospy.logwarn("Height should be between {} and {} meters.".format(Torso.MIN_HEIGHT, Torso.MAX_HEIGHT))

