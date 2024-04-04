import rospy
import actionlib
import control_msgs.msg
import trajectory_msgs.msg

from .arm_joints import ArmJoints

ACTION_NAME = "arm_controller/follow_joint_trajectory"
TIME_FROM_START = 5  # How many seconds it should take to set the arm position.
JOINT_NAME=["shoulder_pan_joint","shoulder_lift_joint","upperarm_roll_joint","elbow_flex_joint","forearm_roll_joint","wrist_flex_joint","wrist_roll_joint"]

class Arm(object):
    """Arm controls the robot's arm.

    Joint space control:
        joints = ArmJoints()
        # Fill out joint states
        arm = robot_api.Arm()
        arm.move_to_joints(joints)
    """

    def __init__(self):
        # Initialize actionlib client for arm control
        self.client = actionlib.SimpleActionClient(ACTION_NAME, control_msgs.msg.FollowJointTrajectoryAction)
        # Wait for the action server to come up
        self.client.wait_for_server()

    def move_to_joints(self, arm_joints):
        """Moves the robot's arm to the given joints.

        Args:
            arm_joints: An ArmJoints object that specifies the joint values for
                the arm.
        """
        # Create a trajectory point for the arm movement
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = arm_joints.values()  # Set joint positions
        point.time_from_start = rospy.Duration(TIME_FROM_START)  # Set time duration

        # Create a goal for the arm movement
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = arm_joints.names()  # Add joint names
        goal.trajectory.points.append(point)  # Add trajectory point

        # Send the goal to the action server and wait for result
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(5.0))
