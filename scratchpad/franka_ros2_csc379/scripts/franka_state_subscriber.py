import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import JointState
import numpy as np

# Taken from ros docs minimal subscriber

# Solution for A1

def DH(a, d, alpha, theta):
    cos_thet = np.cos(theta)
    sin_thet = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)
    return np.array([[cos_thet, -sin_thet, 0, a],
              [sin_thet * cos_alpha, cos_thet * cos_alpha, -sin_alpha, -d * sin_alpha],
              [sin_thet * sin_alpha, cos_thet * sin_alpha, cos_alpha, d * cos_alpha],
              [0, 0, 0, 1]])

class FrankaStateSubscriber():
    def __init__(self, node_handle):
        self.subscription = node_handle.create_subscription(
            JointState, # Task: Change to the correct topic type
            '/franka/measured_js', # Task: Change to the correct topic name
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Task: Read the joint states and print
        joint_positions = msg.position
        print(joint_positions)
        # Task: Create an FK DH model and print the Franka flange position and rotation

        dh_parameters_chain = [
            [0, 0.33, 0, 0],
            [0, 0.0,   -np.pi/2, 0],
            [0, 0.316,  np.pi/2, 0],
            [0.0825, 0, np.pi/2, 0],
            [-0.0825, 0.384, -np.pi/2, 0],
            [0,     0, np.pi/2, 0],
            [0.088, 0, np.pi/2, 0],
            [0, 0.107, 0, 0]
        ]

        O_T_EE = np.eye(4)
        for i in range(len(joint_positions)):
            dh_parameters = dh_parameters_chain[i]
            O_T_EE = np.matmul(O_T_EE, DH(dh_parameters[0], dh_parameters[1], dh_parameters[2], joint_positions[i]))
        
        O_T_EE = np.matmul(O_T_EE, DH(dh_parameters_chain[7][0], dh_parameters_chain[7][1], dh_parameters_chain[7][2], 0.0))
        print(O_T_EE)

def main(args=None):
    rclpy.init(args=args)
    node_handle = Node('joint_state_subscriber')
    joint_state_subscriber = FrankaStateSubscriber(node_handle)
    rclpy.spin(node_handle)
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
