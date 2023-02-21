from socket import *


class TCPServer:
    def __init__(self):
        self.client_add = "localhost"
        self.port = 6340
        self.server_socket = None
        self.connection_socket = None

    def connect_to_client(self):
        self.server_socket = socket(AF_INET, SOCK_STREAM)
        self.server_socket.bind((self.client_add, self.port))
        self.server_socket.listen(1)
        print("waiting client connection...")

        self.connection_socket, add = self.server_socket.accept()
        print("connection from", str(add[0]))

    def disconnect(self):
        self.connection_socket.close()
        print("disconnection successful")

    def send_msg(self, message):
        self.connection_socket.send(len(message).to_bytes(4, byteorder='little'))
        self.connection_socket.send(message.encode('utf-8'))

    def receive_msg(self):
        receive_msg_len = int.from_bytes(self.connection_socket.recv(4), byteorder='little')
        receive_msg = self.connection_socket.recv(receive_msg_len).decode('utf-8')

        return receive_msg


class TCPClient:
    def __init__(self, address='localhost', port=6340):
        self.__host_add = address
        self.__client_socket = None
        self.__host_port = port

    def connect_to_server(self):
        print("connecting from server...")
        self.__client_socket = socket(AF_INET, SOCK_STREAM)
        self.__client_socket.connect((self.__host_add, self.__host_port))
        print("successful connected to server " + f'(address: {self.__host_add}, port: {self.__host_port})')

    def disconnect(self):
        self.__client_socket.close()
        print("disconnection successful")

    def send_msg(self, message):
        self.__client_socket.send(len(message).to_bytes(4, byteorder='little'))
        self.__client_socket.send(message.encode('utf-8'))

    def receive_msg(self):
        receive_msg_len = int.from_bytes(self.__client_socket.recv(4), byteorder='little')
        receive_msg = self.__client_socket.recv(receive_msg_len).decode('utf-8')

        return receive_msg
