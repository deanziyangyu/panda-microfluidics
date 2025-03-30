import copy
import numpy as np
import pygame
import threading
import time

from spatialmath  import SE3
from spatialmath.base import *
import roboticstoolbox as rtb

from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from franka_msgs.action import Grasp, Homing, Move #GripperCommand
from std_srvs.srv import Trigger

from interfaces import FrankaStateInterface

import time

class GripperActionClient(Node):
    def __init__(self):
        print("inited")
        super().__init__('gripper_action_client')
        self._homing_action_client = ActionClient(self, Homing, '/fr3_gripper/homing')
        self._move_action_client = ActionClient(self, Move, '/fr3_gripper/move')
        self._grasp_action_client = ActionClient(self, Grasp, '/fr3_gripper/grasp')
        self._cancel_action_client = self.create_client(Trigger, '/fr3_gripper/stop')
        while not self._cancel_action_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('cancel action service not available, waiting again...')        
        self.cancel_action_req = Trigger.Request()
        # self._gripper_cmd_action_client = ActionClient(self, GripperCommand, 'gripper_action')
        # self._grasping_action_client = ActionClient(self, Fibonacci, 'fibonacci')

    # def send_goal(self, order):
    #     goal_msg = Fibonacci.Goal()
    #     goal_msg.order = order
    #     self._action_client.wait_for_server()
    #     return self._action_client.send_goal_async(goal_msg)

    def _blocking_helper(self, future):
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()

    def do_homing_async(self):
        homing_msg = Homing.Goal()
        self._homing_action_client.wait_for_server()
        return self._homing_action_client.send_goal_async(homing_msg)

    def do_homing_blocking(self):
        future = self.do_homing_async()
        return self._blocking_helper(future)

    def do_move_async(self, width, speed):
        move_msg = Move.Goal() 
        move_msg.width = width
        move_msg.speed = speed
        self._move_action_client.wait_for_server() 
        return self._move_action_client.send_goal_async(move_msg)

    def do_move_blocking(self, width, speed):
        future = self.do_move_async(width, speed)
        return self._blocking_helper(future)

    def do_grasp_async(self, width, speed, force = 50.0): 
        grasp_msg = Grasp.Goal()
        grasp_msg.width = width 
        grasp_msg.speed = speed 
        grasp_msg.force = force
        self._grasp_action_client.wait_for_server()
        return self._grasp_action_client.send_goal_async(grasp_msg)

    def do_grasp_blocking(self, width, speed, force = 50.0):
        future = self.do_grasp_async(width, speed, force)
        return self._blocking_helper(future)

    def cancel_action_async(self, ):
        self.future = self._cancel_action_client.call_async(self.cancel_action_req)
        return rclpy.spin_until_future_complete(self, self.future)


def main(args=None):
    rclpy.init(args=args)
    node_handle = Node('teleoperation_node')
    fsi = FrankaStateInterface(node_handle)
    gripper_client = GripperActionClient()

    spin_func = lambda _ : rclpy.spin(node_handle)
    spin_thread = threading.Thread(target=spin_func, args=(0,))
    spin_thread.start()
    time.sleep(2) # sleep to allow spin thread to get some messages

    pygame.init()
    screen = pygame.display.set_mode((640, 200))
    pygame.display.set_caption("Click and Press Keys to Teleop")
    pygame.mouse.set_visible(1) 

    print("homing...")
    gripper_client.do_homing_async()
    print("finished homing...")


    panda = rtb.models.Panda()
    panda.tool = SE3()

    initial_joint_pos = np.array(fsi.joint_positions)

    print(initial_joint_pos)

    # Task 1: Measure the tool offset from the flange
    tool_offset = np.array([0.0, 0.0, 0.145])
    desired_transform_se3 =  panda.fkine(initial_joint_pos) * SE3(tool_offset)
    current_joint_positions = copy.deepcopy(initial_joint_pos)

    print("initial Pos :", desired_transform_se3)
    print("Origin rotation rpy", SE3(desired_transform_se3).rpy())
    print("Origin translation",SE3(desired_transform_se3).t)

    grasp_width_pipet_holding = 0.0115
    grasp_width_pipet_squeezing = [0.001, 0.01 ,0.005]

    velocity = 0.001 # m
    rot_vel = 0.004 # rad

    joint_solutions = copy.deepcopy(current_joint_positions)

    # cartesian_increment = np.array([.01, .01, .01]) * velocity
    # rpy_increment = np.array([0.0, 0.0, 0.0]) * rot_vel

    while (rclpy.ok()):
        cartesian_increment = np.array([0.0, 0.0, 0.0])
        rpy_increment = np.array([0.0, 0.0, 0.0])
        events = pygame.event.get()
        keys = pygame.key.get_pressed()
            # if event.type == pygame.KEYDOWN:
        if keys[pygame.K_LEFT]:
            cartesian_increment[1] += 1 * velocity
        if keys[pygame.K_RIGHT]:
            cartesian_increment[1] -= 1 * velocity
        if keys[pygame.K_UP]:
            cartesian_increment[0] += 1 * velocity
        if keys[pygame.K_DOWN]:
            cartesian_increment[0] -= 1 * velocity
        if keys[pygame.K_b]:
            cartesian_increment[2] -= 1 * velocity
        if keys[pygame.K_t]:
            cartesian_increment[2] += 1 * velocity

        # if keys[pygame.K_LEFT]:
        #     cartesian_increment[0] -= 1 * velocity
        # if keys[pygame.K_RIGHT]:
        #     cartesian_increment[0] += 1 * velocity
        # if keys[pygame.K_UP]:
        #     cartesian_increment[1] -= 1 * velocity
        # if keys[pygame.K_DOWN]:
        #     cartesian_increment[1] += 1 * velocity
        # if keys[pygame.K_b]:
        #     cartesian_increment[2] -= 1 * velocity
        # if keys[pygame.K_t]:
        #     cartesian_increment[2] += 1 * velocity

        if keys[pygame.K_w]:
            rpy_increment[0] -= 1 * rot_vel
        if keys[pygame.K_s]:
            rpy_increment[0] += 1 * rot_vel
        if keys[pygame.K_a]:
            rpy_increment[1] -= 1 * rot_vel
        if keys[pygame.K_d]:
            rpy_increment[1] += 1 * rot_vel
        if keys[pygame.K_q]:
            rpy_increment[2] -= 1 * rot_vel
        if keys[pygame.K_e]:
            rpy_increment[2] += 1 * rot_vel

        if keys[pygame.K_y]:
            gripper_client.do_homing_async()
        if keys[pygame.K_u]:
            gripper_client.do_move_async(width=grasp_width_pipet_holding, speed=0.05)
        if keys[pygame.K_i]:
            gripper_client.do_move_async(width=grasp_width_pipet_squeezing[0], speed=0.05)
        if keys[pygame.K_o]:
            gripper_client.do_move_async(width=grasp_width_pipet_squeezing[1], speed=0.05)
        if keys[pygame.K_p]:
            gripper_client.do_move_async(width=grasp_width_pipet_squeezing[2], speed=0.05)
        if keys[pygame.K_m]:
            curr_pos_ee = panda.fkine(fsi.joint_positions) * SE3(tool_offset)
            print("\n\n\n\n\n\n\n\n\n\n\n")
            print("Curr Pos mat: ", curr_pos_ee)
            print("Origin rotation rpy", SE3(curr_pos_ee).rpy())
            print("Origin translation",SE3(curr_pos_ee).t)


        # print("cartesian_increment", cartesian_increment)
        # print("rpy_increment", rpy_increment)
        
        
        # Task: Control the robot to move tooltip in the world frame(fixed frame), and
        # rotate in end effector’s local frame(body frame)
        desired_transform_se3 = SE3(cartesian_increment) * \
                desired_transform_se3 * SE3.RPY(rpy_increment, order='xyz')
        
        back_prop = np.array([0.0,  0.0, -0.145])        
        desired_transform_se3_flange = desired_transform_se3 * SE3(back_prop)

        # Task: Solve the joint positions of this new transform using robotics toolbox panka IK (remember to set q0)
        sol = panda.ikine_LM(desired_transform_se3_flange, q0=current_joint_positions)
        
        if sol.success:
            joint_solutions = sol.q
            current_joint_positions = sol.q
        # print("desired_transform_se3", desired_transform_se3)
        # print("initial_joint_pos", initial_joint_pos)
            # print("joint_solutions", joint_solutions)
            fsi.publish_joints(joint_solutions.tolist())
        time.sleep(0.01)

    # time.sleep(2)

    # print("holding pipette")
    # gripper_client.do_move_async(width=grasp_width_pipet_holding, speed=0.05)
    # print("finished holding ...")

    # time.sleep(2)

    # print("squeezing...")
    # gripper_client.do_grasp_blocking(width=grasp_width_pipet_squeezing[0], speed=0.05)
    # print("finished squeezing...")

    # time.sleep(1)

    # print("steadystate...")
    # gripper_client.do_grasp_blocking(width=grasp_width_pipet_squeezing[1], speed=0.05)
    # print("finished steadystate...")

    # time.sleep(1)

    # print("releasing squeeze...")
    # gripper_client.do_grasp_blocking(width=grasp_width_pipet_squeezing[2], speed=0.05)
    # print("finished squeeze...")

    # time.sleep(1)

    # print("holding pipette")
    # gripper_client.do_move_blocking(width=grasp_width_pipet_holding, speed=0.05)
    # print("finished holding ...")


    spin_thread.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
