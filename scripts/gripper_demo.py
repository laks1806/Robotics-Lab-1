#! /usr/bin/env python

import robot_api
import rospy

def print_usage():
    """Prints usage instructions for the gripper demo.
    """
    print('Usage: rosrun applications gripper_demo.py open')
    print('       rosrun applications gripper_demo.py close 40')

def wait_for_time():
    """Wait for simulated time to begin.
    """
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    """Main function for the gripper demo.
    """
    # Initialize ROS node
    rospy.init_node('gripper_demo')
    # Wait for simulated time to start
    wait_for_time()
    # Get command line arguments
    argv = rospy.myargv()
    # Check if command line arguments are valid
    if len(argv) < 2:
        # Print usage instructions if command line arguments are invalid
        print_usage()
        return
    # Extract command from command line arguments
    command = argv[1]

    # Create Gripper object
    gripper = robot_api.Gripper()
    # Set default effort to maximum effort
    effort = gripper.MAX_EFFORT
    # Check if command is 'close' and if additional argument is provided
    if command == 'close' and len(argv) > 2:
        # Set effort to the provided value
        effort = float(argv[2])

    # Execute command based on user input
    if command == 'open':
        gripper.open()
    elif command == 'close':
        gripper.close(effort)
    else:
        # Print usage instructions if command is invalid
        print_usage()

if __name__ == '__main__':
    main()

