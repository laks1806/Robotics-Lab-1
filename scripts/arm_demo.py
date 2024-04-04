#! /usr/bin/env python

import robot_api
import rospy

def wait_for_time():
    """Wait for simulated time to begin."""
    # This function waits until the current ROS time is not 0 seconds,
    # indicating that the simulated time has started.
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    """Main function for the arm demo."""
    # Initialize ROS node for the arm demo
    rospy.init_node('arm_demo')
    # Wait for simulated time to start
    wait_for_time()
    # Get command line arguments
    argv = rospy.myargv()

    # Define disco poses for the arm
    DISCO_POSES = [[1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0],
                   [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
                   [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
                   [-1.5, 1.1, -3.0, -0.5, -3.0, -1.0, -3.0],
                   [-0.8, 0.0, 0.0, 2.0, 0.0, -2.0, 0.0],
                   [0.8, 0.75, 0.0, -2.0, 0.0, 2.0, 0.0],
                   [1.5, -0.6, 3.0, 1.0, 3.0, 1.0, 3.0]]

    # Initialize the Arm object
    arm = robot_api.Arm()
    # Move the arm to each disco pose
    for vals in DISCO_POSES:
        arm.move_to_joints(robot_api.ArmJoints.from_list(vals))

if __name__ == '__main__':
    main()

