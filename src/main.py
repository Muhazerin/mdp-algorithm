import sys

from PyQt5.QtWidgets import QApplication

from mdpAlgoApp import MDPAlgoApp

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MDPAlgoApp()
    window.show()
    sys.exit(app.exec())