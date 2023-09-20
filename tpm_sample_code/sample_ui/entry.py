import sys

from PySide2 import QtWidgets
from PySide2.QtWidgets import QApplication
from MainForm import MainWindow

if '__main__' == __name__:
    app = QApplication(sys.argv)
    mainform = MainWindow()
    mainform.show()

    ret = app.exec_()