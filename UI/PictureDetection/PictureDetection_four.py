import os
from model.Opt import Opt
import cv2
from PyQt5.QtGui import QStandardItem
from PyQt5.QtWidgets import QFileDialog, QMessageBox, QApplication
from PyQt5 import QtGui
# from Service.hkDetect import hkDetect
import glob
from utils.CommonHelper import CommonHelper
from UI.PictureDetection.PictureDetectionWin_four import PictureDetectionWin_four
from model.QclickableImage import QClickableImage
from PyQt5.QtCore import QThread, pyqtSignal


class PictureDetection_four(PictureDetectionWin_four):
    def __init__(self,configuration):
        super(PictureDetection_four, self).__init__()
        self.setupUi(self)
        styleFile = '../../resource/PictureDetection.qss'
        # 换肤时进行全局修改，只需要修改不同的QSS文件即可
        style = CommonHelper.readQss(styleFile)
        self.imgName=[]
        self.configuration =configuration
        self.setStyleSheet(style)
        self.cwd = os.getcwd()  # 获取当前程序文件位置
        self.opt = Opt()
        self.pushputton_new1.clicked.connect(self.zhifang)
        self.pushputton_new2.clicked.connect(self.zhencha)
        # self.start.setEnabled(False)
        # self.stop.setEnabled(False)
        self.badNum=0
    def zhifang(self):
        self.QStackedWidget.setCurrentIndex(0)
    def zhencha(self):
        self.QStackedWidget.setCurrentIndex(1)



