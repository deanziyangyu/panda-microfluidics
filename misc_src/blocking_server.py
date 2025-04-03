import socket
import datetime
import time

# Define host and port for the server
HOST = '127.0.0.1'      # Localhost address
PORT_SERVER = 12345     # Example port number

class SocketCommServer:
    def __init__(self):
        super().__init__()
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Allow address reuse
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((HOST, PORT_SERVER))
        self.server_socket.listen(1)
        self.server_socket.settimeout(20)
        self.client_socket = None
        self.running = True  # Running flag for the listening loop

    def start_server(self):
        print("Waiting for connection...")
        self.client_socket, addr = self.server_socket.accept()
        print("Connected to:", addr)
        return self.client_socket
    
    def suspend_server(self):
        self.server_socket.settimeout(None)
        print("Server suspended.")

    def listen(self):
        data, stamp = None, None
        try:
            if self.client_socket:
                # Receive data from the client
                data = self.client_socket.recv(1024).decode('utf-8')
                stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                return data, stamp
        except socket.timeout:
            # Timeout reached without receiving data
            pass
        return data, stamp

    def close_all_conn(self):
        """Close all connections"""
        self.running = False
        try:
            self.server_socket.shutdown(socket.SHUT_RDWR)
            self.server_socket.close()
        except Exception:
            pass
        self.server_socket = None
        self.client_socket = None
        # If using PyQt or similar, you might emit a signal here
        # self.finished.emit()

def main():
    # Instantiate the server
    server = SocketCommServer()
    
    try:
        # Start the server and wait for a client connection
        server.start_server()
        print("Server is running. Listening for messages...")
        
        # Continuously listen for incoming data until interrupted
        while server.running:
            data, timestamp = server.listen()
            if data:
                print(f"[{timestamp}] Received: {data}")
            time.sleep(0.1)
            # Optionally add a short sleep to reduce CPU usage
    except KeyboardInterrupt:
        print("Keyboard interrupt received. Shutting down server.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Always close the connections gracefully
        server.close_all_conn()
        print("Server closed.")

if __name__ == "__main__":
    main()
