import socket
import datetime
import threading
import time
from queue import Queue, Empty

# Define host and port for the server
HOST = '127.0.0.1'
PORT_SERVER = 12345

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

def main():
    server = SocketCommServer()
    # Start the server and wait for a client connection (blocking)
    server.start_server()

    wait_for_pose_refinement = False
    refined_pose = None

    def server_loop():
        while server.running:
            data, timestamp = server.listen()
            if data:
                print(f"[{timestamp}] Received: {data}")
                # global refined_pose, wait_for_pose_refinement
                refined_pose  = data
                wait_for_pose_refinement = True
                return refined_pose, timestamp

    # Run the server listening loop in a separate thread
    # server_thread = threading.Thread(target=server_loop)
    # server_thread.start()

    # Create a thread-safe queue to capture data from the server loop
    data_queue = Queue()

    def server_loop():
        while server.running:
            data, timestamp = server.listen()
            if data:
                print(f"[{timestamp}] Received: {data}")
                # Place received data and its timestamp into the queue
                data_queue.put((data, timestamp))

    # Start the server_loop in a separate thread
    server_thread = threading.Thread(target=server_loop)
    server_thread.start()

    # Simulate server activity: run active for 5 seconds, then suspend for 5 seconds, then resume.
    wait_for_pose_refinement = False
    time.sleep(5)         # Server is active for 5 seconds
    try:
        wait_for_pose_refinement = True
        print("Server is running. Listening for messages...")
        time.sleep(5)         # Server is active for 5 seconds
        while server.running:
            try:
                # Try to get data from the queue with a timeout
                data, timestamp = data_queue.get(timeout=.5)
                print(f"Main thread captured data at [{timestamp}]: {data}")
                break
            except Empty:
                continue
        print(f"[{timestamp}] MAIN Received: {data}")
        server.suspend()      # Suspend server: the listen() call will block on active_event.wait()
        print("Server suspended.")
        time.sleep(5)         # Server remains suspended (using no CPU cycles)
        server.resume()       # Resume listening
        print("Server resumed.")
        print(f"[{timestamp}]MAIN  Received: {data}")
        time.sleep(10)        # Continue listening for 10 seconds
    except KeyboardInterrupt:
        print("Keyboard interrupt received.")
    finally:
        server.close_all_conn()
        server_thread.join()
        print("Server closed.")

if __name__ == "__main__":
    main()
