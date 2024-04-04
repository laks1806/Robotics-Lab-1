#! /usr/bin/env python

import rospy
import robot_api

def print_usage():
    """Prints usage instructions for the torso demo."""
    print ('Moves the torso to a certain height between [0.0, 0.4]')
    print ('Usage: rosrun applications torso_demo.py 0.4')

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    """Main function for the torso demo."""
    # Initialize ROS node
    rospy.init_node('torso_demo')
    # Wait for simulated time to start
    wait_for_time()
    # Get command line arguments
    argv = rospy.myargv()
    # Check if command line arguments are valid
    if len(argv) < 2:
        # Print usage instructions if command line arguments are invalid
        print_usage()
        return
    # Extract height from command line arguments
    height = float(argv[1])
    # Create Torso object
    torso = robot_api.Torso()
    # Set the torso height
    torso.set_height(height)

if __name__ == '__main__':
    main()

