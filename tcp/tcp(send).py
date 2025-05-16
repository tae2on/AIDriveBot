import threading
import socket

class Client:
    def __init__(self, host = '192.168.0.25', port = 12345):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))

    def send_message(self):
        while True:
            msg = input("Enter your message: ")
            self.sock.sendall(msg.encode())

    def receive_message(self):
        while True:
            data = self.sock.recv(1024)
            if not data:
                break
            print('Received:', data.decode())

if __name__ == "__main__":
    client = Client()
    send_thread = threading.Thread(target = client.send_message)
    send_thread.start()

    receive_thread = threading.Thread(target = client.receive_message)
    receive_thread.start()
