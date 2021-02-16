from PyQt5.QtCore import QObject, pyqtSlot, pyqtSignal
from PyQt5.QtNetwork import QTcpSocket
from config import WIFI_IP, WIFI_PORT

TIMEOUT_MSEC = 5000


class TcpClient(QObject):
    finished = pyqtSignal()
    connectionLost = pyqtSignal()

    def __init__(self):
        super(TcpClient, self).__init__()
        self.__tcp_socket = None

    @pyqtSlot()
    def start_client(self):
        print(f'## Connecting to robot at IP: {WIFI_IP}, PORT: {WIFI_PORT} ##')
        self.__tcp_socket = QTcpSocket()
        self.__tcp_socket.disconnected.connect(self.connectionLost)
        self.connectionLost.connect(self.finished)
        self.connectionLost.connect(lambda: print('## Disconnected from the robot ##'))
        self.__tcp_socket.connectToHost(WIFI_IP, WIFI_PORT)
        self.__tcp_socket.disconnectFromHost()
        if not self.__tcp_socket.waitForConnected(TIMEOUT_MSEC):
            print(f'## Timeout! {TIMEOUT_MSEC/1000}s has passed and it still hasn\'t connect ##')
            self.__tcp_socket.disconnectFromHost()
            self.finished.emit()
        else:
            print(f'## Connected to robot at IP: {WIFI_IP}, PORT: {WIFI_PORT} ##')

    def stop_client(self):
        if self.__tcp_socket is not None:
            self.__tcp_socket.disconnectFromHost()
            self.finished.emit()
