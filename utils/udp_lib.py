import socket


class UdpServerCom:
    def __init__(self, addr='localhost', port=6340):
        self.server_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.local_addr = addr
        self.port = port
        self.server_socket.bind((self.local_addr, self.port))
        print("UDP running!")

    def disconnect(self):
        self.server_socket.close()
        print("disconnection successful")

    def receive_msg(self):
        output = self.server_socket.recv(1024).decode('utf-8')

        return output


class UdpClientCom:
    def __init__(self, addr='localhost', port=6340):
        self.client_socket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
        self.local_addr = addr
        self.port = port
        print("UDP running!")

    def disconnect(self):
        self.client_socket.close()
        print("disconnection successful")

    def send_msg(self, message):
        print(message)
        self.client_socket.sendto(message.encode(), (self.local_addr, self.port))
