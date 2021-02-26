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
            print(f'## Timeout! {TIMEOUT_MSEC/1000}s has passed and it still hasn\'t connect ##')
            self.__tcp_socket.disconnectFromHost()
            self.finished.emit()
        else:
            print(f'## Connected to robot at IP: {WIFI_IP}, PORT: {WIFI_PORT} ##')

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
    def send_message(self, message):
        try:
            self.__tcp_socket.write(QByteArray().append(message))
        except Exception as err:
            print(f'tcpClient::send_message() error msg: {err}')
