import json

from PyQt5.QtCore import QObject, pyqtSlot, pyqtSignal, QByteArray
from PyQt5.QtNetwork import QTcpSocket
from config import WIFI_IP, WIFI_PORT

TIMEOUT_MSEC = 5000


class TcpClient(QObject):
    finished = pyqtSignal()
    connected = pyqtSignal()
    connectionLost = pyqtSignal()
    interpretCmd = pyqtSignal(str)

    def __init__(self):
        super(TcpClient, self).__init__()
        self.__tcp_socket = None
        self.connectionLost.connect(self.finished)
        self.connectionLost.connect(lambda: print('## Disconnected from the robot ##'))

    @pyqtSlot()
    def start_client(self):
        print(f'## Connecting to robot at IP: {WIFI_IP}, PORT: {WIFI_PORT} ##')
        self.__tcp_socket = QTcpSocket()
        self.__tcp_socket.disconnected.connect(self.connectionLost)
        self.__tcp_socket.readyRead.connect(self.read_ready)
        self.__tcp_socket.connected.connect(self.connected)
        self.__tcp_socket.connectToHost(WIFI_IP, WIFI_PORT)
        if not self.__tcp_socket.waitForConnected(TIMEOUT_MSEC):
            print(f'## Timeout! {TIMEOUT_MSEC/1000}s has passed and it still hasn\'t connect ##\n')
            self.__tcp_socket.disconnectFromHost()
            self.finished.emit()
        else:
            print(f'## Connected to robot at IP: {WIFI_IP}, PORT: {WIFI_PORT} ##\n')

    def stop_client(self):
        if self.__tcp_socket is not None:
            self.__tcp_socket.disconnectFromHost()
            self.finished.emit()

    @pyqtSlot()
    def read_ready(self):
        try:
            data = self.__tcp_socket.readAll()
            self.interpretCmd.emit(str(data, "utf-8").strip())
        except Exception as err:
            print(f'tcpClient::read_ready() error msg: {err}')

    @pyqtSlot(str)
    def send_message(self, payload):
        try:
            # print(f'send_message::payload: {payload}\n')
            # the payload may be part of 2 or more strings. because of thread. so need delete it
            #
            # get the first 3 char
            # if ec|, get the next char
            # if tp|, just send
            # if ob|, get char until , once, then get char until not int
            # if ip|, get char until , twice, then get char until not int

            print(f'sending msg: {payload}')
            self.__tcp_socket.write(QByteArray().append(payload + '\n'))
            print('msg sent\n')

            # multi_string = False
            # if payload[0:2] == 'EC':
            #     if len(payload) != 4:
            #         multi_string = True
            # elif payload[0:2] == 'TP':
            #     if len(payload) != 3:
            #         multi_string = True
            # elif payload[0:2] == 'IP':
            #     if len(payload) != 11:
            #         multi_string = True
            # elif payload[0:2] == 'OB':
            #     if len(payload) != 8:
            #         multi_string = True
            #
            # if not multi_string:
            #     print(f'sending msg: {payload}')
            #     self.__tcp_socket.write(QByteArray().append(payload))
            #     print('msg sent\n')
            # else:
            #     while payload:
            #         print(f'while payload: {payload}')
            #         if payload[0:2] == 'EC':
            #             print(f'sending msg: {payload[0:4]}')
            #             self.__tcp_socket.write(QByteArray().append(payload[0:4]))
            #             print('msg sent\n')
            #             payload = payload[4:len(payload)]
            #         elif payload[0:2] == 'TP':
            #             print(f'sending msg: {payload[0:3]}')
            #             self.__tcp_socket.write(QByteArray().append(payload[0:3]))
            #             print('msg sent\n')
            #             payload = payload[3:len(payload)]
            #         elif payload[0:2] == 'IP':
            #             print(f'sending msg: {payload[0:11]}')
            #             self.__tcp_socket.write(QByteArray().append(payload[0:11]))
            #             print('msg sent\n')
            #             payload = payload[8:len(payload)]
            #         elif payload[0:2] == 'OB':
            #             print(f'sending msg: {payload[0:8]}')
            #             self.__tcp_socket.write(QByteArray().append(payload[0:8]))
            #             print('msg sent\n')
            #             payload = payload[6:len(payload)]
            #         # self.__tcp_socket.flush()
        except Exception as err:
            print(f'tcpClient::send_message() error msg: {err}')
