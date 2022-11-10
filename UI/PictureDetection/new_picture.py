import os
from model.Opt import Opt
import cv2
from PyQt5.QtGui import QStandardItem
from PyQt5.QtWidgets import QFileDialog, QMessageBox, QApplication
from PyQt5 import QtGui
# from Service.hkDetect import hkDetect
import glob
from utils.CommonHelper import CommonHelper
from UI.PictureDetection.PictureDetectionWin1 import Ui_Form
from model.QclickableImage import QClickableImage
from PyQt5.QtCore import QThread, pyqtSignal


class PictureDetection(Ui_Form):
    def __init__(self,configuration):
        super(PictureDetection, self).__init__()
        self.setupUi(self)
        styleFile = '../../resource/PictureDetection.qss'
        # 换肤时进行全局修改，只需要修改不同的QSS文件即可
        style = CommonHelper.readQss(styleFile)
        self.imgName=[]
        self.configuration =configuration
        self.setStyleSheet(style)





