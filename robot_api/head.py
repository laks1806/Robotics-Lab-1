#!/usr/bin/env python

# Import necessary modules
import math
import rospy
import actionlib
from geometry_msgs.msg import PointStamped
from control_msgs.msg import PointHeadAction, PointHeadGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import trajectory_msgs.msg

# Define action and joint names
LOOK_AT_ACTION_NAME = 'head_controller/point_head' 
PAN_TILT_ACTION_NAME = 'head_controller/follow_joint_trajectory' 
PAN_JOINT = ['head_pan_joint'] 
TILT_JOINT = ['head_tilt_joint'] 
PAN_TILT_TIME = 2.5  # How many seconds it should take to move the head.

class Head(object):
    """Head controls the Fetch's head.
    
    It provides two interfaces:
        head.look_at(frame_id, x, y, z)
        head.pan_tilt(pan, tilt) # In radians
    
    For example:
        head = robot_api.Head()
        head.look_at('base_link', 1, 0, 0.3)
        head.pan_tilt(0, math.pi/4)
    """
    # Define minimum and maximum pan/tilt angles
    MIN_PAN = -1.5707963267948966  
    MAX_PAN = 1.5707963267948966   
    MIN_TILT = -0.7853981633974483 
    MAX_TILT = 0.7853981633974483  

    def __init__(self):
        # Initialize actionlib clients for look-at and pan/tilt actions
        self.look_at_client = actionlib.SimpleActionClient(LOOK_AT_ACTION_NAME, PointHeadAction)
        self.pan_tilt_client = actionlib.SimpleActionClient(PAN_TILT_ACTION_NAME, FollowJointTrajectoryAction)
        # Wait for both action servers to come up
        self.look_at_client.wait_for_server()
        self.pan_tilt_client.wait_for_server()

    def look_at(self, frame_id, x, y, z):
        """Moves the head to look at a point in space.
        
        Args:
            frame_id: The name of the frame in which x, y, and z are specified.
            x: The x value of the point to look at.
            y: The y value of the point to look at.
            z: The z value of the point to look at.
        """
        # Create goal for look-at action
        goal = PointHeadGoal()
        target = PointStamped()
        target.header.frame_id = frame_id
        target.point.x = x
        target.point.y = y
        target.point.z = z
        goal.target = target
        goal.min_duration = rospy.Duration(1.0)
        # Send goal to action server and wait for result
        self.look_at_client.send_goal(goal)
        self.look_at_client.wait_for_result()

    def pan_tilt(self, pan, tilt):
        """Moves the head by setting pan/tilt angles.
        
        Args:
            pan: The pan angle, in radians. A positive value is clockwise.
            tilt: The tilt angle, in radians. A positive value is downwards.
        """
        # Check if pan/tilt angles are within joint limits
        if not (self.MIN_PAN <= pan <= self.MAX_PAN) and (self.MIN_TILT <= tilt <= self.MAX_TILT):
            rospy.logerr("Pan or tilt value out of bounds")
            return
        
        # Create trajectory point for pan/tilt action
        trajectory = trajectory_msgs.msg.JointTrajectory()
        trajectory.joint_names = [PAN_JOINT[0], TILT_JOINT[0]]
        
        # Set positions of the two joints in the trajectory point
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = [pan, tilt]
        
        # Set time of the trajectory point
        point.time_from_start = rospy.Duration(PAN_TILT_TIME)
        
        # Create goal for pan/tilt action
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        
        # Add trajectory point to trajectory
        goal.trajectory.points.append(point)
        
        # Send the goal to action server and wait for result
        self.pan_tilt_client.send_goal(goal)
        self.pan_tilt_client.wait_for_result()
