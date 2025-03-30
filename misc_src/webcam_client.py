""" An example GUI client for use with web cam server.
Sends msgs on 127.0.0.1, port 12345
Please launch ./webcam_server.py before launching ./webcam_client.py for demo purpose
"""

import sys
import socket
from subprocess import Popen
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton

HOST = '127.0.0.1'  # Server IP address
PORT = 12345       # Server port number

class WebcamCaptureClient(QWidget):
    def __init__(self):
        super().__init__()
        self.client_socket = None
        self.initUI()

    def initUI(self):
        self.setWindowTitle("Webcam Capture")
        self.launch_button = QPushButton("=> LAUNCH SERVER <=")
        self.rec_start_button = QPushButton("Send REC Start")
        self.rec_stop_button = QPushButton("Send REC Stop")
        self.pick_start_button = QPushButton("Send Pick Start")
        self.pick_stop_button = QPushButton("Send Pick Finished")
        self.fname_button = QPushButton("Send FNAME")
        self.fname_button2 = QPushButton("Send FNAME2")
        self.pickid_button = QPushButton("Send PICKID")

        layout = QVBoxLayout()
        layout.addWidget(self.launch_button)
        layout.addWidget(self.rec_start_button)
        layout.addWidget(self.rec_stop_button)
        layout.addWidget(self.pick_start_button)
        layout.addWidget(self.pick_stop_button)
        layout.addWidget(self.pickid_button)
        layout.addWidget(self.fname_button)
        layout.addWidget(self.fname_button2)
        

        self.setLayout(layout)

        self.launch_button.clicked.connect(self.start_server)
        self.rec_start_button.clicked.connect(self.start_rec)
        self.rec_stop_button.clicked.connect(self.stop_rec)
        self.pick_start_button.clicked.connect(self.start_capture)
        self.pick_stop_button.clicked.connect(self.stop_capture)
        self.fname_button.clicked.connect(self.send_fname)
        self.fname_button2.clicked.connect(self.send_fname2)
        self.pickid_button.clicked.connect(self.send_pickid)

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

    def send_fname(self):
        if not self.client_socket:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((HOST, PORT))
        send_str = "./7_picking_app/test_data/07251733/20230726_123350"
        try:
            self.sendall(bytes("FNAME$$$"+send_str, 'utf-8'))
            print("Send FNAME --> 127.0.0.1:12345")
        except Exception as e:
            print(e)

    def send_fname2(self):
        if not self.client_socket:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((HOST, PORT))
        send_str = "./7_picking_app/test_data/072802/20230728_170156"
        try:
            self.sendall(bytes("FNAME$$$"+send_str, 'utf-8'))
            print("Send FNAME --> 127.0.0.1:12345")
        except Exception as e:
            print(e)


    def send_pickid(self):
        if not self.client_socket:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((HOST, PORT))
        send_str = f"PIKID$$${20230726_123350}"
        try:
            self.sendall(bytes(send_str, 'utf-8'))
            print("SEND PICKID --> 127.0.0.1:12345")
        except Exception as e:
            print(e)

    def start_server(self):
        # Slient mode
        Popen(["py", "./8_utilities/webcam_server.py", "--mode", "s"])
        # Verbose mode
        # Popen(["py", "./8_utilities/webcam_server.py",
        #     "--mode", "l",
        #     "--path", "./7_picking_app/test_data/07251733/20230726_123350",
        #     "--init_time", "20230726_123350"])

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WebcamCaptureClient()
    window.show()
    sys.exit(app.exec_())