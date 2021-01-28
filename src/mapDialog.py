# I hardcoded the file location here
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QDialog, QFileDialog

from ui import mapdialog


class MapDialog(QDialog, mapdialog.Ui_mapDialog):
    def __init__(self, map):
        super(MapDialog, self).__init__()
        self.setupUi(self)
        self.__map = map
        self.__p1FileName = None
        self.__p2FileName = None

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
        fname = QFileDialog.getOpenFileNames(self, 'Open file',
                                             'D:\\Python Projects\\mdp-algorithm\\arena\\p1',
                                             'Text files (*.txt)')
        self.__p1FileName = fname[0][0]
        self.leP1FileLocation.setText(self.__p1FileName)

    @pyqtSlot()
    def btnP2LoadFileClicked(self):
        fname = QFileDialog.getOpenFileNames(self, 'Open file',
                                             'D:\\Python Projects\\mdp-algorithm\\arena\\p2',
                                             'Text files (*.txt)')
        self.__p2FileName = fname[0][0]
        self.leP2FileLocation.setText(self.__p2FileName)

    @pyqtSlot()
    def btnLoadFastPathClicked(self):
        try:
            with open(self.__p1FileName, 'r') as f:
                hexStr = f.read()
                binStr = ''
                for c in hexStr:
                    binStr += self.__hexConvertorDict[c]
                binStr = binStr[2:-2]
                i = 0
                print("p1")
                for c in binStr:
                    if i % 15 == 0:
                        print()
                    print(c, end='')
                    i = i + 1
            with open(self.__p2FileName, 'r') as f:
                hexStr = f.read()
                binStr = ''
                for c in hexStr:
                    binStr += self.__hexConvertorDict[c]
                i = 0
                print("\n\n\np2")
                for c in binStr:
                    if i % 15 == 0:
                        print()
                    print(c, end='')
                    i = i + 1
        except Exception as err:
            print(f"[ERROR] mapDialog::btnLoadFastPathClicked! Error msg: {err}")
