#! /usr/bin/env python

# Import necessary modules
from http import client
import rospy
import control_msgs.msg
import actionlib

# Define action name
ACTION_NAME = control_msgs.msg.GripperCommandAction
CLOSED_POS = 0.0  # The position for a fully-closed gripper (meters).
OPENED_POS = 0.10  # The position for a fully-open gripper (meters).


class Gripper(object):
    """Gripper controls the robot's gripper.
    """
    MIN_EFFORT = 35  # Min grasp force, in Newtons
    MAX_EFFORT = 100  # Max grasp force, in Newtons

    def __init__(self):
        # Create actionlib client for gripper action
        self.client = actionlib.SimpleActionClient('gripper_controller/gripper_action', ACTION_NAME)
        # Wait for the action server to come up
        self.client.wait_for_server()

    def open(self):
        """Opens the gripper.
        """
        # Create goal for opening the gripper
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = OPENED_POS
        goal.command.max_effort = self.MAX_EFFORT
        
        # Send goal to action server
        self.client.send_goal(goal)
        # Wait for action server to complete the goal
        self.client.wait_for_result()
        
        # Return result of the action
        return self.client.get_result()

    def close(self, max_effort=MAX_EFFORT):
        """Closes the gripper.

        Args:
            max_effort: The maximum effort, in Newtons, to use. Note that this
                should not be less than 35N, or else the gripper may not close.
        """
        # Create goal for closing the gripper
        goal = control_msgs.msg.GripperCommandGoal()
        goal.command.position = CLOSED_POS
        goal.command.max_effort = max_effort
        
        # Send goal to action server
        self.client.send_goal(goal)
        # Wait for action server to complete the goal
        self.client.wait_for_result()
        
        # Return result of the action
        return self.client.get_result()
