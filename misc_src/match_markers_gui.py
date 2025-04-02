import numpy as np

""" A OpenCV-based Webcam Server Listening on [IP] [PORT]
Usage:
    for startup without path and timestamp specified:
        py ./webcam_server.py --mode s
    for startup specifying path and timestamp:
        py ./webcam_server.py --mode v --path ./path/to/data_collection_folder/YYMMDD_HHMMSS \
            --init_time YYMMDD_HHMMSS

Shell launch cmd from other .py program:
    silent: Popen(["py", "./8_utilities/webcam_server.py", "--mode", "s"])
    verbose: Popen(["py", "./8_utilities/webcam_server.py",
        "--mode", "v",
        "--path", "./path/to/data_collection_folder/YYMMDD_HHMMSS",
        "--init_time", "YYMMDD_HHMMSS"])
"""
import os
import sys
import time
import json
import socket
import logging
import datetime
import argparse
from pathlib import Path

import csv
import cv2
import apriltag

from PyQt5.QtWidgets import QApplication, QLabel, QHBoxLayout, QVBoxLayout, QWidget, QPushButton
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QObject, QTimer, QSize

import time
import roboticstoolbox as rtb
from spatialmath import SE3

FILE_SET = False
USE_CAMERA = True
CAMERA_NUM = 0

HOST = '127.0.0.1'  # Server IP address
PORT = 12345       # Server port number
PORT_CLIENT = 12346

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)
formatter = logging.Formatter(
    '%(asctime)s - %(filename)s - %(funcName)s - %(levelname)s - %(message)s'
)

source_path = Path(__file__).resolve()
source_dir = source_path.parent

csv_fields = ['TimeStamp', 'Event']
csv_fpath = ""
log_fpath = ""
csv_fname = "_log_100_server_events.csv"
log_fname = "_video_capture_server_debug.log"
vid_fdir  = ""
vid_dir   = "_rec_vids/"
vid_fname = "_rec_vids.mp4"
cap_video_path = "/../misc_files/20250401_214655_rec_vids.mp4"


# source = cv2.imread("fiducial_test.png", cv2.IMREAD_GRAYSCALE)
# template_a = cv2.imread(f"{source_dir}/../misc_files/fiducial_template_a.png", cv2.IMREAD_GRAYSCALE)
# template_chip = cv2.imread(f"{source_dir}/../misc_files/fiducial_template_chip.png", cv2.IMREAD_GRAYSCALE)
# template_chip = cv2.resize(template_chip, (120, 200))


def match_fiducial_orb(source, template,
                       template_kp_desc=None,
                       min_match_count=8, 
                       good_match_ratio=0.75):
    """
    Detects the fiducial marker (template) in the source image using ORB feature matching.
    Returns:
        homography: The 3x3 homography matrix if enough matches are found, otherwise None
        corners_in_scene: The projected corner coordinates of the template in the source image
                          as a numpy array of shape (4, 2). None if not found.
        matches_mask: A mask array that indicates which matches are inliers (for visualization)
    """

    # Initialize ORB detector
    orb = cv2.ORB_create(
        nfeatures = 200,                    # The maximum number of features to retain.
        scaleFactor = 1.3,                  # Pyramid decimation ratio, greater than 1
        nlevels = 6,                        # The number of pyramid levels.
        edgeThreshold = 7,                  # This is size of the border where the features are not detected. It should roughly match the patchSize parameter
        firstLevel = 0,                     # It should be 0 in the current implementation.
        WTA_K = 2,                          # The number of points that produce each element of the oriented BRIEF descriptor.
        scoreType = cv2.ORB_HARRIS_SCORE,   # The default HARRIS_SCORE means that Harris algorithm is used to rank features (the score is written to KeyPoint::score and is 
                                            # used to retain best nfeatures features); 
        # scoreType = cv2.ORB_FAST_SCORE,     # FAST_SCORE is alternative value of the parameter that produces slightly less stable 
                                            # keypoints, but it is a little faster to compute.
        patchSize = 7                      
    )


    if isinstance(template_kp_desc, type(None)):
        kp_template, desc_template = orb.detectAndCompute(template, None)
        template_shape = template.shape
        template_kp_desc = (kp_template, desc_template, template_shape)
    else:
        kp_template, desc_template, template_shape = template_kp_desc

    # 3. Detect keypoints and compute descriptors
    # kp_template, desc_template = orb.detectAndCompute(gray_template, None)
    kp_source, desc_source = orb.detectAndCompute(source, None)

    # Check if descriptors are valid
    if desc_template is None or desc_source is None:
        return -1
    # 4. Use a brute-force matcher or FLANN-based matcher
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

    # 5. Match descriptors (KNN)
    matches_knn = bf.knnMatch(desc_template, desc_source, k=2)
    
    # 6. Lowe's ratio test to filter good matches
    good_matches = []
    for m, n in matches_knn:
        if m.distance < good_match_ratio * n.distance:
            good_matches.append(m)

    # 7. Check if enough matches are present
    if len(good_matches) < min_match_count:
        # Not enough matches to reliably compute homography
        return None, None, None

    # 8. Extract matched keypoints’ coordinates
    src_pts = np.float32([kp_template[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp_source[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

    # 9. Compute homography with RANSAC
    homography, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
    matches_mask = mask.ravel().tolist()

    if homography is None:
        return -1

    # 10. Once we have the homography, transform the corners of the template
    hT, wT = template_shape[:2]
    template_corners = np.float32([[0, 0],
                                   [wT, 0],
                                   [wT, hT],
                                   [0, hT]]).reshape(-1, 1, 2)

    corners_in_scene = cv2.perspectiveTransform(template_corners, homography)

    return homography, corners_in_scene.reshape(-1, 2), matches_mask, src_pts, dst_pts, template_kp_desc



def match_markers(source, template_to_match):
    """
    Simple heuristic to match a template to a source image using scale-invariant template matching.
    Args:
        source: The source image to search for the template.
        template_to_match: The template image to search for in the source image
    Returns:
        matches: A list of tuples (score, location, template_size) of the top-k matches found.
    """
    # Perform scale-invariant template matching
    # best_match = None
    # best_val = -np.inf
    # Store the top-k matches

    matches = []

    top_k = 1
    scales = [0.25, 0.35, 0.5, ]  # Define scales to resize the template
    # angles = [0, 11.25, 22.5, 33.75, 45, 56.25, 67.5, 78.75, 90]  # Define angles to rotate the template
    angles = [0, 22.5, 45, 67.5,]  # Define angles to rotate the template

    for angle in angles:
        # Rotate the template
        center = (template_to_match.shape[1] // 2, template_to_match.shape[0] // 2)
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
        rotated_template = cv2.warpAffine(template_to_match, rotation_matrix, (template_to_match.shape[1], template_to_match.shape[0]))

        for scale in scales:
            # Resize the rotated template
            resized_template = cv2.resize(rotated_template, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            result = cv2.matchTemplate(source, resized_template, cv2.TM_CCOEFF_NORMED)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

            # Store the match details
            matches.append((max_val, max_loc, resized_template.shape[::-1]))

    # Remove duplicate matches based on location
    unique_matches = []
    seen_locations = set()

    for match in matches:
        _, location, _ = match
        if location not in seen_locations:
            unique_matches.append(match)
            seen_locations.add(location)

    matches = unique_matches

    # Sort matches by their score in descending order and keep the top-k
    matches = sorted(matches, key=lambda x: x[0], reverse=True)[:top_k]

    return matches

class SocketCommClient:
    def __init__(self):
        super().__init__()
        self.client_socket = None
        self.host = HOST
        self.port = PORT_CLIENT

    def sendall(self, data):
        while data:
            sent = self.client_socket.send(data)
            data = data[sent:]

    def send_str(self, send_str):
        if not self.client_socket:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.host, self.port))
        try:
            self.sendall(bytes("STR$$$$$"+send_str, 'utf-8'))
            logger.debug(f"Send STR --> {self.host}:{self.port}")
        except Exception as e:
            logger.debug(e)
    
    def send_pose_rpy(self, xyz, rpy):
        """
        Send the pose in the form of a dictionary with keys 'x', 'y', 'z', and 'rpy'.
        # Expected format: {"x": 0.5, "y": 0.2, "z": 0.3, "rpy": [-3.13, 0.097, 0.035]}
        """
        send_str = f'{{"x": {xyz[0]}, "y": {xyz[1]}, "z": {xyz[2]}, "r": {rpy[0]}, "p": {rpy[1]}, "y": {rpy[2]}}}'
        if not self.client_socket:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.host, self.port))
        try:
            self.sendall(bytes("POSERPY$"+send_str, 'utf-8'))
            logger.debug(f"Send STR --> {self.host}:{self.port}")
        except Exception as e:
            logger.debug(e)


class SpatialProcessingWorker(QObject):
    finished = pyqtSignal()
    frameUpdated = pyqtSignal(object)
    dataUpdated = pyqtSignal(object) 

    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True
        self.robot = rtb.models.Panda()
        # physical dimensions of the chip and rack
        self.corners_phy_meter_chip = np.array([[0., 0.], [0.028, 0.], [0.028, 0.023], [0., 0.023]])
        self.corners_phy_meter_rack = np.array([[0., 0.], [0.081, 0.], [0.081, 0.1215], [0., 0.1215]])

        # camera intrinsics for Realsense D415
        width, height = 1920, 1080
        ppx = 951.094909667969
        ppy = 529.039794921875
        fx = 1366.31372070312
        fy = 1363.66088861788
        fov_deg = (70.18, 43.2)
        distortion = "inverse brown conrady"

        self.camera_K = np.array([
            [fx, 0, ppx],
            [0, fy, ppy],
            [0, 0, 1]
        ])

        # calibration matrix HAND_EYE_TSAI for realsense D415
        self.T_ee_cam =  SE3(
            [[ 0.01212702, -0.9994259,  0.03163564,  0.0537774 ],
            [ 0.99985468,  0.01174098, -0.01236027, -0.03114828],
            [ 0.01198174, 0.03178094,  0.99942304, -0.05437236],
            [ 0.,          0.,          0. ,         1.        ]]
        )

    def compute_item_pose_base(self, H, K, T_base_cam):
        # Decompose homography to get rotation and translation
        H_normalized = np.linalg.inv(K) @ H
        H_normalized /= np.linalg.norm(H_normalized[:, 0])
        
        r1, r2, t = H_normalized[:, 0], H_normalized[:, 1], H_normalized[:, 2]
        r3 = np.cross(r1, r2)
        R_cam_to_item = np.column_stack((r1, r2, r3))
        
        # Ensure orthogonal rotation matrix
        U, _, Vt = np.linalg.svd(R_cam_to_item)
        R_cam_to_item = U @ Vt
        
        # Item's rotation in camera frame (yaw)
        yaw_cam = np.arctan2(R_cam_to_item[1,0], R_cam_to_item[0,0])
        
        # Item's position in camera frame (origin of item's plane)
        item_pos_cam = -R_cam_to_item.T @ t  # Homography translation adjustment
        
        # Transform item pose to base frame
        item_pos_base = T_base_cam * item_pos_cam
        item_rot_base = T_base_cam.R @ R_cam_to_item.T
        
        # Extract yaw in base frame
        yaw_base = np.arctan2(item_rot_base[1,0], item_rot_base[0,0])
        return item_pos_base, yaw_base


    def run(self):
        while self.running:
            time.sleep(0.1)
            pass

    def stop_processing(self):
        self.running = False
        self.finished.emit()

    def compute_homography(self, corners_in_scene, corners_phy_meter):
        # Compute the homography matrix
        h, status = cv2.findHomography(corners_phy_meter, corners_in_scene)
        return h, status
    
    def compute_pose(self, homography, q_current):
        # Compute the pose of the item in the camera frame
        # Compute the pose of the item in the base frame

        # Current end-effector and camera poses
        # T_base_ee = robot.fkine(q_current)
        T_base_ee = self.robot.fkine(q_current)
        T_base_cam = T_base_ee * self.T_ee_cam

        # Compute item's base pose
        item_pos_base, item_yaw_base = self.compute_item_pose_base(
            homography, self.camera_K, T_base_cam
        )

        # Desired gripper pose (orthogonal orientation)
        # desired_yaw = item_yaw_base # 90° offset
        desired_yaw = item_yaw_base + np.pi
        desired_pos = np.append(item_pos_base[:2], T_base_ee.t[2])  # Keep current height
        desired_pose = SE3(desired_pos) * SE3.Rz(desired_yaw) * SE3.Rx(-np.pi)

        # Solve inverse kinematics
        sol = self.robot.ikine_LM(desired_pose, q0=q_current, tol=1e-6,)

        if sol.success:
            logger.debug("IK success for orthogonal pose")
            return desired_pose, sol.q
        else:
            logger.debug("IK failed to converge")
            return desired_pose, None

class ClientHandlingWorker(QObject):
    """Continuously Listen to & Receive from Client
    parse msg, ignores timeout"""
    finished = pyqtSignal()
    stampUpdated = pyqtSignal(object)
    recStatusUpdated = pyqtSignal(object)
    jointPosUpdated = pyqtSignal(object)
    fileUpdated = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_capturing = True
        self.running = True
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Reuse IP:PORT if available
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((HOST, PORT))
        self.server_socket.listen(1)
        self.server_socket.settimeout(20)
        self.client_socket = None

    def run(self):
        while self.running:
            try:
                if self.client_socket:
                    # Receive data from the client
                    data = self.client_socket.recv(1024).decode('utf-8')
                    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

                    global FILE_SET
                    if data[:8] == 'PIKSTART':
                        logger.debug("START Pick at %r", data[8:])
                        self.stampUpdated.emit(["ON", data[8:]])
                        if FILE_SET:
                            with open(csv_fpath,'a',newline='', encoding='utf8') as fhdl:
                                f_csv = csv.writer(fhdl)
                                f_csv.writerow([data[8:],"PICK_STARTED"])
                        else:
                            logger.debug("Need to Create File Objects before Logging !!")

                    elif data[:7] == 'PIKSTOP':
                        logger.debug("STOP Pick at %r", data[7:])
                        self.stampUpdated.emit(["OFF", data[7:]])
                        if FILE_SET:
                            with open(csv_fpath,'a',newline='', encoding='utf8') as fhdl:
                                f_csv = csv.writer(fhdl)
                                f_csv.writerow([data[7:],"PICK_FINISHED"])
                        else:
                            logger.debug("Need to Create File Objects before Logging !!")

                    elif data == 'RECSTART':
                        if FILE_SET:
                            self.recStatusUpdated.emit(["ON", stamp])
                        else:
                            logger.debug("Need to Create File Objects before Recording !!")

                    elif data == 'RECSTOP':
                        if FILE_SET:
                            self.recStatusUpdated.emit(["OFF", stamp])
                        else:
                            logger.debug("Recording Not Started. Need to Create File Objects")

                    elif data[:8] == 'JOINTPOS':
                        joint_pos_str = data[8:]
                        calc_pos = ""
                        self.jointPosUpdated.emit([joint_pos_str, stamp, calc_pos])
                        if FILE_SET:
                            with open(csv_fpath,'a',newline='', encoding='utf8') as fhdl:
                                f_csv = csv.writer(fhdl)
                                f_csv.writerow([stamp, joint_pos_str])
                        else:
                            logger.debug("Need to Create File Objects before Logging !!")


                    elif data[:8] == 'JNTCALC0':
                        joint_pos_str = data[8:]
                        calc_pos = "chip"
                        self.jointPosUpdated.emit([joint_pos_str, stamp, calc_pos])
                        if FILE_SET:
                            with open(csv_fpath,'a',newline='', encoding='utf8') as fhdl:
                                f_csv = csv.writer(fhdl)
                                f_csv.writerow([stamp, joint_pos_str])
                        # else:
                        #     logger.debug("Need to Create File Objects before Logging !!")


                    elif data[:8] == 'JNTCALC1':
                        joint_pos_str = data[8:]
                        calc_pos = "rack"
                        self.jointPosUpdated.emit([joint_pos_str, stamp, calc_pos])
                        if FILE_SET:
                            with open(csv_fpath,'a',newline='', encoding='utf8') as fhdl:
                                f_csv = csv.writer(fhdl)
                                f_csv.writerow([stamp, joint_pos_str])
                        # else:
                        #     logger.debug("Need to Create File Objects before Logging !!")


                    elif data[:8] == 'JNTCALC2':
                        joint_pos_str = data[8:]
                        calc_pos = "drop"
                        self.jointPosUpdated.emit([joint_pos_str, stamp, calc_pos])
                        if FILE_SET:
                            with open(csv_fpath,'a',newline='', encoding='utf8') as fhdl:
                                f_csv = csv.writer(fhdl)
                                f_csv.writerow([stamp, joint_pos_str])
                        # else:
                        #     logger.debug("Need to Create File Objects before Logging !!")



                    # elif data[:8] == 'FNAME$$$':
                    #     self.fileUpdated.emit("")
                    #     logger.debug("Creating File Objects...")
                    #     FILE_SET = False
                    #     create_file_obj(data[8:], stamp)
                    #     start_logging()
                    #     FILE_SET = True
                    #     logger.debug("GET FNAME %r at %r", data[8:], stamp)
                    #     logger.debug("File Objects Created.")
                    #     self.fileUpdated.emit(data[8:])

                    # elif data[:8] == 'PIKID$$$':
                    #     picking_id = data[8:]
                    #     logger.debug("NEW PICKID %r at %r", picking_id, stamp)
                    #     if FILE_SET:
                    #         with open(csv_fpath,'a',newline='', encoding='utf8') as fhdl:
                    #             f_csv = csv.writer(fhdl)
                    #             f_csv.writerow([stamp,f"NEWPICKID:{picking_id}"])
                    #     else:
                    #         logger.debug("Need to Create File Objects before Logging !!")

                    elif data[:8] == 'STR$$$$$':
                        logger.debug("GET STR %r at %r", data[8:], stamp)
                    
                    elif data == '':
                        pass
                    else:
                        logger.debug("Got DATA %r", data)
                else:
                    logger.debug("Server is listening on %s:%s", HOST, PORT)
                    self.client_socket, addr = self.server_socket.accept()
                    logger.debug("Connection from %s:%s", addr[0], addr[1])
            except socket.timeout:
                if self.running:
                    pass
                else:
                    break

    def close_all_conn(self):
        """Close all Connections"""
        self.running = False
        try:
            self.server_socket.shutdown(socket.SHUT_RDWR)
            self.server_socket.close()
        except Exception:
            pass
        self.server_socket = None
        self.client_socket = None
        self.finished.emit()

class ImageProcessingWorker(QObject):
    """Process opencv images to detect markers"""
    finished = pyqtSignal()
    frameUpdated = pyqtSignal(object)
    dataUpdated = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.running = True
        self.template = None
    
    def run(self):
        while self.running:
            time.sleep(0.1)
            pass

    def stop_processing(self):
        self.running = False
        self.finished.emit()


    def visualize_atag(self, frame, at_det_result, combined_corners):
        for at_detections in at_det_result:
            corners = at_detections.corners
            tag_id = at_detections.tag_id
            cv2.polylines(frame, [corners.astype(int)], True, (0,255,0), 2)
            
            tag_center = tuple(at_detections.center.astype(int))
            cv2.putText(frame, f"tagid:{tag_id}", tag_center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
            cv2.polylines(frame, [combined_corners.astype(int)], True, (255,0,0), 2)
            combined_corners_center = combined_corners.mean(axis=0).astype(int)
            cv2.putText(frame, f"center: {combined_corners_center}", combined_corners_center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)

        return frame
    
    def visualize_circles(self, frame, circles_detected):
        for i in range(circles_detected.shape[1]):
            center = (int(circles_detected[0][i][0]), int(circles_detected[0][i][1])) # Circle center
            radius = int(circles_detected[0][i][2])  # Circle radius
            cv2.circle(frame, center, radius, (0, 255, 255), 3)  # Draw the circle's perimeter
            cv2.circle(frame, center, 3, (255, 0, 0), -1)  # Draw the circle's center
            cv2.putText(frame, f"Circle Center:  r:{radius}", center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        return frame


    def process_image_once(self, frame, detect_rack= False):
        # logger.debug(frame)

        combined_corners = None
        combined_rack_corners = None

        if isinstance(frame, np.ndarray):

            frame_grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # frame_grey_center_crop = frame_grey[300:780, 700:1320] # middle crop of 1920:
            # frame_grey_center_crop_circles = frame_grey[300:1080, 700:1320] # bottom right crop for the pipet tip
            frame_grey_center_crop_circles = frame_grey


            try:
                # apriltag detection
                at_det = apriltag.Detector(apriltag.DetectorOptions(families="tag16h5"))
                at_det_result = sorted(at_det.detect(frame_grey), key=lambda x: x.tag_id)[:4]

                tag_0_top_left = at_det_result[0].corners[0]
                tag_1_top_right = at_det_result[1].corners[1]
                tag_2_bottom_right = at_det_result[2].corners[2]
                tag_3_bottom_left = at_det_result[3].corners[3]

                combined_corners = np.stack([tag_0_top_left,
                    tag_1_top_right,
                    tag_2_bottom_right, 
                    tag_3_bottom_left])
                
                at_det_rack_result = []
                if detect_rack:
                    at_det_rack = apriltag.Detector(apriltag.DetectorOptions(families="tag25h9"))
                    at_det_rack_result = at_det_rack.detect(frame_grey)

                    tag_rack_0_top_left = at_det_rack_result[0].corners[0]
                    tag_rack_1_top_right = at_det_rack_result[1].corners[1]
                    tag_rack_2_bottom_right = at_det_rack_result[2].corners[2]
                    tag_rack_3_bottom_left = at_det_rack_result[3].corners[3]

                    combined_rack_corners = np.stack([tag_rack_0_top_left,
                        tag_rack_1_top_right,
                        tag_rack_2_bottom_right, 
                        tag_rack_3_bottom_left])

                # # use a center crop for dmf checker template matching
                # ret_template_matched = match_markers(frame_grey_center_crop, template_a)

                # # hg_mat, corners, mask, src_pts, dest_pts 
                # if self.template == None:
                #     ret_template_matched =  match_fiducial_orb(
                #         frame_grey_center_crop, template_chip, None, 
                #         min_match_count=1, good_match_ratio=0.75
                #     )
                #     self.template = ret_template_matched[-1] # cache the template keyypoints and descriptors
                # else:
                #     ret_template_matched =  match_fiducial_orb(
                #         frame_grey_center_crop, template_chip, self.template,
                #         min_match_count=1, good_match_ratio=0.75
                #     )
                # ret_template_matched = list(ret_template_matched[:-1])
                # logger.debug(len(ret_template_matched))

                # Hough circle detection
                frame_grey_center_crop_circle_gauss = cv2.GaussianBlur(frame_grey_center_crop_circles, (7,7,), 2)
                circles_detected = cv2.HoughCircles(
                    frame_grey_center_crop_circle_gauss,
                    cv2.HOUGH_GRADIENT,
                    dp=1.2,
                    minDist=200,
                    param1=50,
                    param2=30,
                    minRadius=5,
                    maxRadius=30,
                )
            except Exception as exp:
                logger.debug(exp)
                return
            
            atdet_success, circle_success = (
                isinstance(at_det_result, list) , isinstance(circles_detected, np.ndarray)
            )

            if atdet_success == True and circle_success == True:
                try:

                    frame = self.visualize_atag(frame, at_det_result, combined_corners)
                    if detect_rack:
                        frame = self.visualize_atag(frame, at_det_rack_result, combined_rack_corners)
                    frame = self.visualize_circles(frame, circles_detected)
                except Exception as e:
                    logger.debug(f"visualization failed : {e}")
                    pass
            else:
                logger.debug(f"INFO: Values partially extracted: atag {atdet_success}; circle {circle_success}")

            self.frameUpdated.emit(frame) 
            # logger.debug(ret_template_matched)
            self.dataUpdated.emit((at_det_result,
                                   combined_corners,
                                   at_det_rack_result,
                                    combined_rack_corners,
                                   circles_detected))
      

class VideoCaptureWorker(QObject):
    """Capture and Save opencv cam port to the required Spec"""
    finished = pyqtSignal()
    frameUpdated = pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.is_capturing = False
        self.running = True
        self.timestamp = ""
        self.filename = ""

        self.cap_api = None
        if sys.platform == "linux":
            self.cap_api = cv2.CAP_V4L2
        elif sys.platform == "win32":
            self.cap_api = cv2.CAP_DSHOW
        elif sys.platform == "darwin":
            self.cap_api = cv2.CAP_AVFOUNDATION
        else:
            self.cap_api = cv2.CAP_ANY

        if USE_CAMERA:
            self.capture = cv2.VideoCapture(CAMERA_NUM, self.cap_api) # Direct Show for MS Windows
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            self.capture.set(cv2.CAP_PROP_EXPOSURE, -6)
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            self.capture.set(cv2.CAP_PROP_FPS, 1)
        else:
            self.capture = cv2.VideoCapture(str(source_dir) + cap_video_path,)
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 2)
            self.capture.set(cv2.CAP_PROP_FPS, 1)

        _, frame = self.capture.read()
        fshape = frame.shape
        self.fheight = fshape[0]
        self.fwidth = fshape[1]
        self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = None

    def run(self):
        while self.running:
            _, frame = self.capture.read()

            if self.is_capturing:
                if FILE_SET:
                    self.writer.write(frame)
            self.frameUpdated.emit(frame)

    def stop_capturing(self):
        self.is_capturing = False
        if FILE_SET:
            logger.debug("Recording: STOP")
            self.finished.emit()
        else:
            logger.debug("Recording not started")
        time.sleep(0.1)
        if self.writer:
            self.writer.release()

    def start_capturing(self):
        if FILE_SET:
            logger.debug("Recording: START")
            self.timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            logger.debug("Current Timestamp: %s", self.timestamp)
            self.filename = vid_fdir + self.timestamp + vid_fname
            self.writer = cv2.VideoWriter(
                self.filename, self.fourcc, 30.0, (self.fwidth, self.fheight)
            )
            self.is_capturing = True
        else:
            logger.debug("Need to Create File Objects before record START !!")

    def update_once(self):
        _, frame = self.capture.read()
        self.frameUpdated.emit(frame)

class WebcamCaptureApp(QWidget):
    """GUI APP"""
    def __init__(self):
        super().__init__()
        self.base_time = time.time()
        self.rec_start_time = None
        self.video_timer = QTimer()
        self.video_timer.setInterval(1000)

        logch = logging.StreamHandler()
        logch.setLevel(logging.DEBUG)
        logch.setFormatter(formatter)
        logger.addHandler(logch)

        self.init_ui()

        (self.at_det_result,
        self.combined_corners,
        self.at_det_rack_result,
        self.combined_rack_corners,
        self.circles_detected) = None, None, None, None, None

        try:
            # Video Capture Worker
            self.video_worker = VideoCaptureWorker()
            self.start_rec.clicked.connect(self.video_worker.start_capturing)
            self.stop_rec.clicked.connect(self.video_worker.stop_capturing)
            self.start_rec.clicked.connect(self.update_ui_rec_on)
            self.stop_rec.clicked.connect(self.update_ui_rec_off)
            self.video_timer.timeout.connect(self.update_timer_label)
            # self.video_worker.frameUpdated.connect(self.update_video)

            self.video_thread = QThread()
            self.video_worker.moveToThread(self.video_thread)
            self.video_thread.started.connect(self.video_worker.run)
            self.video_thread.start()

            # Client Handling Worker
            self.client_handler = ClientHandlingWorker()
            self.client_thread = QThread()
            self.client_handler.moveToThread(self.client_thread)
            self.client_thread.started.connect(self.client_handler.run)
            self.client_handler.stampUpdated.connect(self.rec_timestamp)
            self.client_handler.recStatusUpdated.connect(self.rec_video_updated)
            self.client_handler.fileUpdated.connect(self.rec_dir_updated)
            self.client_thread.start()

            # Image Processing Worker
            self.image_processing_worker = ImageProcessingWorker()
            self.video_worker.frameUpdated.connect(self.image_processing_worker.process_image_once)

            self.image_processing_thread = QThread()
            self.image_processing_worker.moveToThread(self.image_processing_thread)
            self.image_processing_thread.started.connect(self.image_processing_worker.run)
            self.image_processing_worker.frameUpdated.connect(self.update_video)
            # while this is potentially dangerous for thread contention, we dont have a choice
            self.image_processing_worker.dataUpdated.connect(self.rec_extracted_features)
            self.image_processing_thread.start()

            self.ip_client = SocketCommClient()
            try:
                self.ip_client.send_str("Webcam Capture Viewer Started.")
            except Exception as e:
                logger.debug(e)
                pass

            # Spatial Processing Worker
            self.spatial_processing_worker = SpatialProcessingWorker()
            self.client_handler.jointPosUpdated.connect(self.compute_and_send_pose)

        except KeyboardInterrupt:
            logger.debug("Keyboard interrupt received. Shutting down webcam server...")
            sys.exit(0)

    def init_ui(self):
        """Set Up GUI"""
        self.setWindowTitle("Webcam Capture Viewer")
        self.video_label = QLabel(self)
        self.rec_status_label = QLabel("Recording OFF")
        self.rec_status_label.setStyleSheet("color: rgb(0, 0, 0); font-size: 12pt")
        self.timer_label = QLabel("0:00:00")
        self.rec_timestamp_label = QLabel(" ")
        self.rec_timestamp_label.setStyleSheet("font-size: 12pt")
        self.rec_dir_label = QLabel(" ")
        self.start_rec = QPushButton("Start Recording")
        self.stop_rec = QPushButton("Stop Recording")
        self.start_rec.setEnabled(True)
        self.stop_rec.setEnabled(False)

        layout_main = QVBoxLayout()
        layout_rec = QHBoxLayout()
        layout_time = QHBoxLayout()
        layout_button = QHBoxLayout()

        layout_rec.addWidget(self.rec_status_label)
        layout_rec.addWidget(self.rec_timestamp_label)
        layout_button.addWidget(self.start_rec)
        layout_button.addWidget(self.stop_rec)
        layout_time.addWidget(self.timer_label)
        layout_time.addWidget(self.rec_dir_label)

        layout_main.addWidget(self.video_label)
        layout_main.addLayout(layout_rec)
        layout_main.addLayout(layout_time)
        layout_main.addLayout(layout_button)

        self.setLayout(layout_main)
        self.show()

    def update_timer_label(self):
        time_str = f"{datetime.timedelta(seconds = int(time.time() - self.rec_start_time))}"
        self.timer_label.setText(time_str)

    def rec_timestamp(self, recv):
        status_str = "Pick Started: " if recv[0] == "ON" else "Pick Finished: "
        self.rec_timestamp_label.setText(status_str + recv[1])

    def compute_and_send_pose(self, recv):
        joint_pos_str, stamp, calc_pos = recv

        if calc_pos != "":
            # convert string to array
            recv_arr = np.array(json.loads(joint_pos_str))

            if calc_pos == "chip":
                h, status = self.spatial_processing_worker.compute_homography(
                    self.spatial_processing_worker.corners_phy_meter_chip, self.combined_corners, 
                )
            elif calc_pos == "rack":
                h, status = self.spatial_processing_worker.compute_homography(
                    self.spatial_processing_worker.corners_phy_meter_rack, self.combined_rack_corners, 
                )
            elif calc_pos == "drop":
                h, status = self.spatial_processing_worker.compute_homography(
                    self.spatial_processing_worker.corners_phy_meter_chip, self.combined_corners, 
                )
            # print(h, status)
            if status.all() == False:
                logger.debug("Homography computation failed")
            else:
                desired_pose, sol = self.spatial_processing_worker.compute_pose(h, recv_arr)
                self.ip_client.send_pose_rpy(desired_pose.t, desired_pose.rpy())
                print(desired_pose, sol)
            if not isinstance(sol, type(None)):
                try:
                    if isinstance(desired_pose, np.ndarray):
                        desired_pose  = SE3(desired_pose)
                    elif isinstance(desired_pose, SE3):
                        pass
                    self.ip_client.send_pose_rpy(desired_pose.t, desired_pose.rpy())
                except Exception as e:
                    logger.debug(e)
                    pass
            else:
                logger.debug("MAIN: IK failed to converge")


    def rec_extracted_features(self, recv):
        (self.at_det_result,
        self.combined_corners,
        self.at_det_rack_result,
        self.combined_rack_corners,
        self.circles_detected) = recv


    def rec_video_updated(self, recv):
        if recv[0] == "ON":
            if not self.video_worker.is_capturing:
                self.start_rec.click()
        elif recv[0] == "OFF":
            if self.video_worker.is_capturing:
                self.stop_rec.click()

    def rec_dir_updated(self, recv):
        if str(recv) == "":
            self.rec_dir_label.setText("Rec Dir updating...")
        else:
            self.rec_dir_label.setText("Current Rec Dir: "+str(recv))

    def update_ui_rec_on(self):
        if FILE_SET:
            self.rec_status_label.setText("Recording ON")
            self.rec_status_label.setStyleSheet("color: rgb(255, 0, 0); font-size: 12pt")
            self.rec_start_time = time.time()
            self.video_timer.start()
            self.stop_rec.setEnabled(True)
            self.start_rec.setEnabled(False)
            with open(csv_fpath,'a',newline='', encoding='utf8') as fhdl:
                f_csv = csv.writer(fhdl)
                f_csv.writerow(
                    [datetime.datetime.now().strftime("%Y%m%d_%H%M%S"),"REC_STARTED"]
                )

    def update_ui_rec_off(self):
        if FILE_SET:
            self.rec_status_label.setText("Recording OFF")
            self.rec_status_label.setStyleSheet("color: rgb(0, 0, 0); font-size: 12pt")
            self.video_timer.stop()
            self.rec_start_time = None
            logger.debug("Recording time elapsed: %s", self.timer_label.text())
            self.timer_label.setText("0:00:00")
            self.start_rec.setEnabled(True)
            self.stop_rec.setEnabled(False)
            with open(csv_fpath,'a',newline='', encoding='utf8') as fhdl:
                f_csv = csv.writer(fhdl)
                f_csv.writerow(
                    [datetime.datetime.now().strftime("%Y%m%d_%H%M%S"),"REC_STOPPED"]
                )

    def update_video(self, frame):
        height, width, _ = frame.shape
        bytes_per_line = 3 * width
        q_image = QImage(frame.data, width, height,
                         bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        pixmap = QPixmap.fromImage(q_image)
        pixmap_scaled = pixmap.scaled(QSize(960,540), Qt.KeepAspectRatio)
        self.video_label.setPixmap(pixmap_scaled)

    def closeEvent(self, _):
        """Program Exit handling"""
        self.client_handler.close_all_conn()
        self.video_worker.stop_capturing()
        self.video_worker.running = False
        self.image_processing_worker.stop_processing()
        # self.video_thread.wait()
        # self.client_thread.wait()
        self.video_thread.quit()
        self.image_processing_thread.quit()
        self.client_thread.quit()
        # self.video_thread.deleteLater()
        # self.client_handler.deleteLater()
        logger.debug("Shutting down webcam server...")

def start_logging():
    logfh = logging.FileHandler(log_fpath)
    logfh.setLevel(logging.DEBUG)
    logfh.setFormatter(formatter)
    logger.addHandler(logfh)

    with open(csv_fpath,'a',newline='', encoding='utf8') as fhdl:
        f_csv = csv.writer(fhdl)
        f_csv.writerow(csv_fields)

def create_file_obj(base_path, init_time):
    """Update global fnames and fpaths"""
    global csv_fpath, log_fpath, vid_fdir
    csv_fpath = base_path + "_csv_files/" + init_time + csv_fname
    log_fpath = base_path + log_fname
    vid_fdir = base_path + vid_dir
    if not os.path.isdir(vid_fdir):
        os.mkdir(vid_fdir)
    if not os.path.isdir(base_path + "_csv_files/"):
        os.mkdir(base_path + "_csv_files/")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='webcam_server',
        description='A Webcam server listening on 127.0.0.1, port 12345',
        epilog='Text at the bottom of help'
    )
    parser.add_argument('-m', '--mode', action='store', nargs=1)  # optional flag
    parser.add_argument('-p', '--path', action='store', nargs=1)  # optional flag
    parser.add_argument('-i', '--init_time', action='store', nargs=1)  # optional flag
    args = parser.parse_args()
    mode = args.mode[0]
    if mode == "v":
        FILE_SET = True
        create_file_obj(args.path[0], args.init_time[0])
        start_logging()
    app = QApplication(sys.argv)
    window = WebcamCaptureApp()
    sys.exit(app.exec_())