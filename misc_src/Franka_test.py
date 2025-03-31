import copy
import numpy as np
import sys
import threading
import time

from spatialmath  import SE3
from spatialmath.base import *
import roboticstoolbox as rtb
from interfaces import FrankaStateInterface
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from franka_msgs.action import Grasp, Homing, Move #GripperCommand
from std_srvs.srv import Trigger
import datetime
from queue import Queue, Empty

import socket
import json

# Stuff to launch in other terminals
# ros2 launch franka_gripper gripper.launch.py robot_ip:=192.168.1.107
# ros2 run franka_ros2_csc379 run_franka_impedance_control_ros2

HOST = '127.0.0.1'  # IP address on host PC
PORT = 12345       # client port number on host PC
PORT_SERVER = 12346 # server port number on host PC

# Reading Serial target pose function?
#             return target_pose  # Expected format: {"x": 0.5, "y": 0.2, "z": 0.3, "rpy": [-3.13, 0.097, 0.035]}

class SocketCommServer:
    def __init__(self):
        super().__init__()
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Allow address reuse
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((HOST, PORT_SERVER))
        self.server_socket.listen(1)
        self.server_socket.settimeout(None)  # Use blocking accept
        self.client_socket = None
        self.running = True
        # This event will control whether the server should be active.
        self.active_event = threading.Event()
        self.active_event.set()  # Start in an active state

    def start_server(self):
        print("Waiting for connection...")
        self.client_socket, addr = self.server_socket.accept()
        print("Connected to:", addr)
        return self.client_socket

    def listen(self):
        # Block until the server is resumed.
        self.active_event.wait()
        try:
            if self.client_socket:
                # This is a blocking call that waits for data
                data = self.client_socket.recv(1024).decode('utf-8')
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                return data, timestamp
        except socket.error as e:
            print("Socket error:", e)
        return None, None

    def suspend(self):
        """Suspend listening (no CPU cycles will be used while suspended)."""
        print("Server suspended.")
        self.active_event.clear()

    def resume(self):
        """Resume listening."""
        print("Server resumed.")
        self.active_event.set()

    def close_all_conn(self):
        """Close all connections."""
        self.running = False
        try:
            self.server_socket.shutdown(socket.SHUT_RDWR)
            self.server_socket.close()
        except Exception:
            pass
        self.server_socket = None
        self.client_socket = None
    

class SocketCommClient:
    def __init__(self):
        super().__init__()
        self.client_socket = None
        # self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # # Reuse IP:PORT if available
        # self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self.server_socket.bind((HOST, PORT))
        # self.server_socket.listen(1)
        # self.server_socket.settimeout(20)
        # data = self.client_socket.recv(1024).decode('utf-8')

    def sendall(self, data):
        while data:
            sent = self.client_socket.send(data)
            data = data[sent:]

    def start_capture(self):
        if not self.client_socket:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((HOST, PORT))
        try:
            self.sendall(b'PIKSTART')
            print("START PICKING --> 127.0.0.1:12345")
        except Exception as e:
            print(e)

    def stop_capture(self):
        if not self.client_socket:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((HOST, PORT))
        try:
            self.sendall(b'PIKSTOP')
            print("STOP PICKING --> 127.0.0.1:12345")
        except Exception as e:
            print(e)

    def start_rec(self):
        if not self.client_socket:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((HOST, PORT))
        try:
            self.sendall(b'RECSTART')
            print("START REC --> 127.0.0.1:12345")
        except Exception as e:
            print(e)

    def stop_rec(self):
        if not self.client_socket:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((HOST, PORT))
        try:
            self.sendall(b'RECSTOP')
            print("STOP REC --> 127.0.0.1:12345")
        except Exception as e:
            print(e)

    def send_str(self, send_str):
        if not self.client_socket:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((HOST, PORT))
        try:
            self.sendall(bytes("STR$$$$"+send_str, 'utf-8'))
            print("Send STR --> 127.0.0.1:12345")
        except Exception as e:
            print(e)
    
    def send_joint_pos(self, send_str):
        if not self.client_socket:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((HOST, PORT))
        try:
            self.sendall(bytes("JOINTPOS"+send_str, 'utf-8'))
            print("Send STR --> 127.0.0.1:12345")
        except Exception as e:
            print(e)


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
    
def move_to_pose(fsi, panda, start_pose):
    intermediate_pos = -1 # if no sol, ret -1
    initial_joint_pos = np.array(fsi.joint_positions)
    print("initial Joint pos", initial_joint_pos)
    current_joint_positions = copy.deepcopy(initial_joint_pos)
    desired_transform_se3 = start_pose
    print("Desire pose",desired_transform_se3)
    # Solve inverse kinematics
    sol = panda.ikine_LM(desired_transform_se3, q0=current_joint_positions)
    print(sol)
    if sol.success:
        print("IK solution found!")
        joint_solutions = sol.q
        joint_distance = np.linalg.norm(joint_solutions - current_joint_positions)
        num_steps = int(np.clip(joint_distance * 100, 10, 1000))
        print("Joint angles (radians):", joint_solutions)
        # Send to robot
        for alpha in np.linspace(0,1, num_steps):
            intermediate_pos = (1-alpha) * current_joint_positions + alpha*joint_solutions
            fsi.publish_joints(intermediate_pos.tolist())
            time.sleep(0.05)
    else:
        print("Failed to find IK solution!")
    
    return intermediate_pos


def main(args=None):
    rclpy.init(args=args)
    node_handle = Node('inverse_kinematics_node')
    fsi = FrankaStateInterface(node_handle)
    gripper_client = GripperActionClient()
    socket_client = SocketCommClient() # for communication to the qtgui
    socket_server = SocketCommServer() # for communication from the                             

    # Start the server and wait for a client connection (blocking)
    socket_server.start_server()

    data_queue = Queue()

    def server_loop():
        while socket_server.running:
            data, timestamp = socket_server.listen()
            if data:
                print(f"[{timestamp}] Thread Received: {data}")
                # Place received data and its timestamp into the queue
                data_queue.put((data, timestamp))

    # Start the server_loop in a separate thread
    server_thread = threading.Thread(target=server_loop)
    server_thread.start()

    socket_server.suspend() #suspend the server

    # Initialize ROS2 spin thread
    spin_func = lambda _ : rclpy.spin(node_handle)
    spin_thread = threading.Thread(target=spin_func, args=(0,))
    spin_thread.start()
    time.sleep(2)  # Allow time for spin thread to initialize

    print("homing...")
    gripper_client.do_homing_async()
    print("finished homing...")
    time.sleep(5)

    # Create Panda robot model
    panda = rtb.models.Panda()
    panda.tool = SE3()  # Reset tool transform
    # Define your desired end effector position (x, y, z in meters)

    # Main Loop

    # Position, Orientation (RPY) 

    start_pose = SE3(0.5, 0, 0.5)*SE3.RPY(-3.13, 0.097, 0.035) #transition pose:
    device_view_pose = SE3(0.506, 0.3, 0.3)*SE3.RPY(-3.13, 0.097, 0.035)
    device_pickup_pose = SE3(0.506, 0.3, 0.08)*SE3.RPY(-3.13, 0.097, 0.035)
    placement_view_pose = SE3(0.506, 0, 0.3)*SE3.RPY(-3.13, 0.097, 0.035)
    device_place_pose = SE3(0.506, 0, 0.08)*SE3.RPY(-3.13, 0.097, 0.035)
    pipette_view_pose = SE3(0.506, 0.4, 0.4)*SE3.RPY(-3.13, 0.097, 0.035)
    pipette_pickup_pose = SE3(0.506, 0.4, 0.2)*SE3.RPY(-3.13, 0.097, 0.035)
    pipette_alignment_pose = SE3(0.45, 0, 0.3)*SE3.RPY(-3.13, 0.097, 0.035)

    grasp_width_open = 0.05
    grasp_width_pipet_holding = 0.0115
    grasp_width_pipet_squeezing = [0.001, 0.01 ,0.005]
    grasp_device = 0.027

    # Step 1: Moving to start pose
    move_to_pose(fsi, panda, start_pose)
    time.sleep(2)

    # Step 2: Move to above the device for pickup view
    ret_pose = move_to_pose(fsi, panda, device_view_pose)
    if isinstance(ret_pose, np.ndarray):
        socket_client.send_joint_pos(json.dumps(ret_pose.tolist()))
    else:
        print("NO POSE")
    # wait_for_pose_refinement = True
    socket_server.resume()
    while socket_server.running:
        try:
            # Try to get data from the queue with a timeout
            data, timestamp = data_queue.get(timeout=.5)
            break
        except Empty:
            continue
    socket_server.suspend()
    print(f"Main thread captured data at [{timestamp}]: {data}")

    socket_server.resume()
    while socket_server.running:
        try:
            # Try to get data from the queue with a timeout
            data, timestamp = data_queue.get(timeout=.5)
            break
        except Empty:
            continue
    socket_server.suspend()
    print(f"Main thread captured data at [{timestamp}]: {data}")


    # Step 3: Move to above device via a targeted posed from camera
    # Move only along the xy plane when given a target pose through serial communication
    # based on read_target_pose?

    # Step 4: Lower the ee to pick up device in pickup pose
    gripper_client.do_move_async(width = grasp_width_open, speed = 0.05)
    time.sleep(3)

    ret_pose = move_to_pose(fsi, panda, device_pickup_pose)
    if isinstance(ret_pose, np.ndarray):
        socket_client.send_joint_pos(json.dumps(ret_pose.tolist()))
    else:
        print("NO POSE")

    time.sleep(2)
    gripper_client.do_grasp_async(width = grasp_device,speed=0.05)

    time.sleep(3)
    ret_pose = move_to_pose(fsi, panda, device_view_pose) #return to view pose
    if isinstance(ret_pose, np.ndarray):
        socket_client.send_joint_pos(json.dumps(ret_pose.tolist()))
    else:
        print("NO POSE")
    time.sleep(2)

    # Step 5: Move to placement view pose
    ret_pose = move_to_pose(fsi, panda, start_pose) #transition pose
    if isinstance(ret_pose, np.ndarray):
        socket_client.send_joint_pos(json.dumps(ret_pose.tolist()))
    else:
        print("NO POSE")
    time.sleep(2)
    ret_pose = move_to_pose(fsi, panda, placement_view_pose)
    if isinstance(ret_pose, np.ndarray):
        socket_client.send_joint_pos(json.dumps(ret_pose.tolist()))
    else:
        print("NO POSE")
    time.sleep(2)

    # Step 7: Move to above placement location based on april tags in workspace (target location provided through serial)

    # Step 8: Lower to place device
    ret_pose = move_to_pose(fsi, panda, device_place_pose)
    if isinstance(ret_pose, np.ndarray):
        socket_client.send_joint_pos(json.dumps(ret_pose.tolist()))
    else:
        print("NO POSE")
    time.sleep(2)
    
    gripper_client.do_move_async(width = grasp_width_open, speed = 0.05)
    time.sleep(3)

    ret_pose = move_to_pose(fsi, panda, placement_view_pose)
    if isinstance(ret_pose, np.ndarray):
        socket_client.send_joint_pos(json.dumps(ret_pose.tolist()))
    else:
        print("NO POSE")
    time.sleep(2)

    ret_pose = move_to_pose(fsi, panda, start_pose) #transition pose
    if isinstance(ret_pose, np.ndarray):
        socket_client.send_joint_pos(json.dumps(ret_pose.tolist()))
    else:
        print("NO POSE")
    time.sleep(2)

    # Step 9: Move to syring view pose
    ret_pose = move_to_pose(fsi, panda, pipette_view_pose)
    if isinstance(ret_pose, np.ndarray):
        socket_client.send_joint_pos(json.dumps(ret_pose.tolist()))
    else:
        print("NO POSE")
    time.sleep(2)

    # Step 10: assuming syring is same place everytime and fixed, we just open loop pick up pipette
    ret_pose = move_to_pose(fsi, panda, pipette_pickup_pose)
    if isinstance(ret_pose, np.ndarray):
        socket_client.send_joint_pos(json.dumps(ret_pose.tolist()))
    else:
        print("NO POSE")
    time.sleep(2)
    gripper_client.do_grasp_async(width = grasp_width_pipet_squeezing[2],speed=0.05) #grasp pipette
    time.sleep(3)

    # In between here we add more steps to get fluids from a beaker etc.

    ret_pose = move_to_pose(fsi, panda, pipette_pickup_pose) #return to orignal pose and then transition pose
    if isinstance(ret_pose, np.ndarray):
        socket_client.send_joint_pos(json.dumps(ret_pose.tolist()))
    else:
        print("NO POSE")
    time.sleep(2)
    ret_pose = move_to_pose(fsi, panda, start_pose)
    if isinstance(ret_pose, np.ndarray):
        socket_client.send_joint_pos(json.dumps(ret_pose.tolist()))
    else:
        print("NO POSE")
    time.sleep(2)
    ret_pose = move_to_pose(fsi, panda, pipette_alignment_pose) #move to pose to view device and start aligning pipette
    if isinstance(ret_pose, np.ndarray):
        socket_client.send_joint_pos(json.dumps(ret_pose.tolist()))
    else:
        print("NO POSE")
    time.sleep(2)

    # Step 11: Move pipette to inlet and squeeze. Movement based on match circles on device and pipette


    # End of cycle of movement

    # Clean up
    spin_thread.join()
    rclpy.shutdown()

if __name__ == '__main__':
    main()