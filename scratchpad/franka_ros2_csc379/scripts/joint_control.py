import copy
import threading
import time

import rclpy
from rclpy.node import Node
from interfaces import FrankaStateInterface

# Solution for A2

def main(args=None):
    rclpy.init(args=args)

    node_handle = Node('joint_control')
    fsi = FrankaStateInterface(node_handle)
    spin_func = lambda _ : rclpy.spin(node_handle)
    spin_thread = threading.Thread(target=spin_func, args=(0,))
    spin_thread.start()
    time.sleep(1) # sleep to allow spin thread to get some messages
    
    # Task: Get the current joint state as in problem set 1
    time.sleep(1) # let it get messages for a bit

    start_joint_positions = copy.deepcopy(fsi.joint_positions)
    command_joint_positions = copy.deepcopy(fsi.joint_positions)


    increment = 0.01
    velocity = 0.2
    sleep_s = increment / velocity

    goal_joint_1 = command_joint_positions[0] + 0.5

    while (command_joint_positions[0] < goal_joint_1):
        command_joint_positions[0] += increment
        fsi.publish_joints(command_joint_positions)
        time.sleep(sleep_s)

    command_joint_positions[0] = goal_joint_1
    fsi.publish_joints(command_joint_positions)
    print("goal finished")

    # Not needed, but I am a lazy TA that wants the robot to move
    # back to the start position, to rerun this multiple times
    while (command_joint_positions[0] > start_joint_positions[0]):
        command_joint_positions[0] -= increment
        fsi.publish_joints(command_joint_positions)
        time.sleep(sleep_s)

    command_joint_positions[0] = start_joint_positions[0]
    fsi.publish_joints(command_joint_positions)

    print("back to start")

    # Task: Create a joint trajectory with a desired goal joint state
    # within the franka q limits
    # and the start joint state as the robots current state
    
    # Task: Send your trajectory and move the Franka robot
    while rclpy.ok():
        # Task: Print your trajectory first before uncommenting
        # the publisher

        fsi.publish_joints(command_joint_positions)
        # Task: Important to set your sleep! 
        # This is how fast you are sending the points,
        # which sets the velocity
        time.sleep(1) # The default 1s will be too slow

    spin_thread.join()
    fsi.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
