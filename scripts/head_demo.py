#! /usr/bin/env python

import rospy
import robot_api

def print_usage():
    """Prints usage instructions for the head demo."""
    # NOTE: We don't expect you to implement look_at for Kuri
    # But if you do, show us because that would be impressive ;)
    # `eyes`, naturally, is Kuri only.
    print('Usage:')
    print('    rosrun applications head_demo.py look_at FRAME_ID X Y Z')
    print('    rosrun applications head_demo.py pan_tilt PAN_ANG TILT_ANG')
    print('    rosrun applications head_demo.py eyes ANG')
    print('Examples:')
    print('    rosrun applications head_demo.py look_at base_link 1 0 0.3')
    print('    rosrun applications head_demo.py pan_tilt 0 0.707')
    print('    rosrun applications head_demo.py eyes .50')

def wait_for_time():
    """Wait for simulated time to begin."""
    while rospy.Time().now().to_sec() == 0:
        pass

def main():
    """Main function for the head demo."""
    # Initialize ROS node
    rospy.init_node('head_demo')
    # Wait for simulated time to start
    wait_for_time()
    # Get command line arguments
    argv = rospy.myargv()

    # Create Head object
    head = robot_api.Head()

    # Check if command line arguments are valid
    if len(argv) < 2:
        # Print usage instructions if command line arguments are invalid
        print_usage()
        return

    # Extract command from command line arguments
    command = argv[1]

    # Execute command based on user input
    if command == 'look_at':
        if len(argv) < 6:
            print_usage()
            return
        frame_id, x, y, z = argv[2], float(argv[3]), float(argv[4]), float(argv[5])
        # Call look_at method
        head.look_at(frame_id, x, y, z)  # implemented

    elif command == 'pan_tilt':
        if len(argv) < 4:
            print_usage()
            return
        pan, tilt = float(argv[2]), float(argv[3])
        # Call pan_tilt method
        head.pan_tilt(pan, tilt)  # implemented

    elif command == 'eyes':
        if len(argv) < 3:
            print_usage()
            return
        angle = float(argv[2])
        # Log error message indicating 'eyes' command is not implemented
        rospy.logerr('Not implemented.')  # Fetch doesn't have eyes. 
    
    else:
        # Print usage instructions if command is invalid
        print_usage()

if __name__ == '__main__':
    main()

