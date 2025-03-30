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
# import roboticstoolbox as rtb
from spatialmath import SE3

FILE_SET = False
CAMERA_NUM = 4

HOST = '127.0.0.1'  # Server IP address
PORT = 12345       # Server port number

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


# source = cv2.imread("fiducial_test.png", cv2.IMREAD_GRAYSCALE)
template_a = cv2.imread(f"{source_dir}/../misc_files/fiducial_template_a.png", cv2.IMREAD_GRAYSCALE)


def match_markers(source, template_to_match):
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


class ClientHandlingWorker(QObject):
    """Continuously Listen to & Receive from Client
    parse msg, ignores timeout"""
    finished = pyqtSignal()
    stampUpdated = pyqtSignal(object)
    recStatusUpdated = pyqtSignal(object)
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

                    elif data[:8] == 'FNAME$$$':
                        self.fileUpdated.emit("")
                        logger.debug("Creating File Objects...")
                        FILE_SET = False
                        create_file_obj(data[8:], stamp)
                        start_logging()
                        FILE_SET = True
                        logger.debug("GET FNAME %r at %r", data[8:], stamp)
                        logger.debug("File Objects Created.")
                        self.fileUpdated.emit(data[8:])

                    elif data[:8] == 'PIKID$$$':
                        picking_id = data[8:]
                        logger.debug("NEW PICKID %r at %r", picking_id, stamp)
                        if FILE_SET:
                            with open(csv_fpath,'a',newline='', encoding='utf8') as fhdl:
                                f_csv = csv.writer(fhdl)
                                f_csv.writerow([stamp,f"NEWPICKID:{picking_id}"])
                        else:
                            logger.debug("Need to Create File Objects before Logging !!")
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
    
    def run(self):
        while self.running:
            time.sleep(0.05)
            pass

    def stop_processing(self):
        self.running = False
        self.finished.emit()

    def draw_visualization_cv2(self, frame, ret_template_matched, at_det_result, circles_detected):
        """
        Draw the matched templates, detected apriltags, and detected circles on the frame using cv2
        """

        # Draw the matched templates using cv2
        matched_num = 0
        for _, top_left, (h,w) in ret_template_matched:
            top_left = (top_left[0] + 700, top_left[1] + 300)
            bottom_right = (top_left[0] + w, top_left[1] + h)
            cv2.rectangle(frame, top_left, bottom_right, 255, 2)
            cv2.putText(frame, f"{len(ret_template_matched)-matched_num}",
                            top_left, cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
            matched_num +=1
        
        # Draw the detected apriltags using cv2
        for at_detections in at_det_result:
            corners = at_detections.corners
            tag_id = at_detections.tag_id

            for i in range(4):
                start_pt = tuple(corners[i].astype(int))
                end_pt = tuple(corners[(i+1)%4].astype(int))
                cv2.line(frame, start_pt, end_pt, (0,255,0),2)
            
            tag_center = tuple(at_detections.center.astype(int))
            cv2.putText(frame, f"tagid:{tag_id}", tag_center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

        # Draw the detected circle using cv2
        for i in range(circles_detected.shape[1]):

            center = (int(circles_detected[0][i][0]) + 700, int(circles_detected[0][i][1]) + 300) # Circle center
            radius = int(circles_detected[0][i][2])  # Circle radius
            cv2.circle(frame, center, radius, (0, 255, 255), 3)  # Draw the circle's perimeter
            cv2.circle(frame, center, 3, (255, 0, 0), -1)  # Draw the circle's center
            cv2.putText(frame, f"Circle Center: {center}, r:{radius}", center, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        return frame


    def process_image_once(self, frame):
        # print(frame)

        if isinstance(frame, np.ndarray):

            frame_grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            frame_grey_center_crop = frame_grey[300:780, 700:1320] # middle crop of 1920:
            frame_grey_center_crop_circles = frame_grey[300:1080, 700:1320] # bottom right crop for the pipet tip

            try:
                # apriltag detection
                at_det = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
                at_det_result = at_det.detect(frame_grey)

                # use a center crop for dmf checker template matching
                ret_template_matched = match_markers(frame_grey_center_crop, template_a)

                # Hough circle detection
                frame_grey_center_crop_circle_gauss = cv2.GaussianBlur(frame_grey_center_crop_circles, (7,7,), 2)
                circles_detected = cv2.HoughCircles(
                    frame_grey_center_crop_circle_gauss,
                    cv2.HOUGH_GRADIENT,
                    dp=1.2,
                    minDist=200,
                    param1=50,
                    param2=30,
                    minRadius=10,
                    maxRadius=45,
                )
            except Exception as exp:
                print(exp)
                return
            
            template_success, atdet_success, circle_success = (
                isinstance(ret_template_matched, list), isinstance(at_det_result, list) , isinstance(circles_detected, np.ndarray)
            )

            if template_success == True and atdet_success == True and circle_success == True:
            
                frame = self.draw_visualization_cv2(
                    frame, ret_template_matched, at_det_result, circles_detected
                )
            else:
                print(f"INFO: Values partially extracted: template {template_success}; atag {atdet_success}; circle {circle_success}")

            self.frameUpdated.emit(frame)  
            self.dataUpdated.emit((ret_template_matched, at_det_result, circles_detected))
            

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
        self.capture = cv2.VideoCapture(CAMERA_NUM, cv2.CAP_V4L2) # Direct Show for MS Windows
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.capture.set(cv2.CAP_PROP_EXPOSURE, -6)
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
            self.image_processing_thread.start()

            # Spatial Processing Worker
            # self.spatial_processing_worker = 

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