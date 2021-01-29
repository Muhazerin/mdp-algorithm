# I hardcoded the file location here
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QDialog, QFileDialog

from ui import mapdialog


class MapDialog(QDialog, mapdialog.Ui_mapDialog):
    def __init__(self, map):
        super(MapDialog, self).__init__()
        self.setupUi(self)
        self.__map = map
        self.__p1FileName = ""
        self.__p2FileName = ""

        self.btnP1LoadFile.clicked.connect(self.btnP1LoadFileClicked)
        self.btnP2LoadFile.clicked.connect(self.btnP2LoadFileClicked)
        self.btnLoadFastPath.clicked.connect(self.btnLoadFastPathClicked)

        self.__hexConvertorDict = {
            '0': '0000',
            '1': '0001',
            '2': '0010',
            '3': '0011',
            '4': '0100',
            '5': '0101',
            '6': '0110',
            '7': '0111',
            '8': '1000',
            '9': '1001',
            'A': '1010',
            'B': '1011',
            'C': '1100',
            'D': '1101',
            'E': '1110',
            'F': '1111'
        }

    @pyqtSlot()
    def btnP1LoadFileClicked(self):
        try:
            blankTuple = ([], '')
            fname = QFileDialog.getOpenFileNames(self, 'Open file',
                                                 'D:\\Python Projects\\mdp-algorithm\\arena\\p1',
                                                 'Text files (*.txt)')
            if fname == blankTuple:
                self.__p1FileName = ""
                self.leP1FileLocation.setText("")
                return

            self.__p1FileName = fname[0][0]
            self.leP1FileLocation.setText(self.__p1FileName)
        except Exception as err:
            print(f"[Error] mapDialog::btnP1LoadFileClicked()! Error msg: {err}")

    @pyqtSlot()
    def btnP2LoadFileClicked(self):
        try:
            blankTuple = ([], '')
            fname = QFileDialog.getOpenFileNames(self, 'Open file',
                                                 'D:\\Python Projects\\mdp-algorithm\\arena\\p2',
                                                 'Text files (*.txt)')

            if fname == blankTuple:
                self.__p2FileName = ""
                self.leP2FileLocation.setText("")
                return

            self.__p2FileName = fname[0][0]
            self.leP2FileLocation.setText(self.__p2FileName)
        except Exception as err:
            print(f"[Error] mapDialog::btnP2LoadFileClicked()! Error msg: {err}")

    @pyqtSlot()
    def btnLoadFastPathClicked(self):
        try:
            with open(self.__p1FileName, 'r') as f:
                p1HexStr = f.read()
                if len(p1HexStr) > 76:
                    p1HexStr = p1HexStr[0:76]

                p1BinStr = ''
                for c in p1HexStr:
                    p1BinStr += self.__hexConvertorDict[c]
                p1BinStr = p1BinStr[2:-2]

            with open(self.__p2FileName, 'r') as f:
                p2HexStr = f.read()
                p2BinStr = ''
                for c in p2HexStr:
                    p2BinStr += self.__hexConvertorDict[c]

            self.__map.loadFastPathMap(p1BinStr, p2BinStr)
            self.accept()
        except Exception as err:
            print(f"[ERROR] mapDialog::btnLoadFastPathClicked! Error msg: {err}")
