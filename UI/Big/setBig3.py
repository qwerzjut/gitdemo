from UI.Big.untitled import Ui_Form
from UI.Big.setBigWin3 import DetectionWin1
from utils.CommonHelper import CommonHelper
from PyQt5.QtCore import QSettings, pyqtSignal
from PyQt5.QtWidgets import QFileDialog
from MvCameraControl_class import *
import os
import cv2
from PyQt5 import QtCore, QtWidgets, QtGui
from UI.Detection.Detection import Detection
from PyQt5.QtGui import QIcon, QPixmap, QStandardItem
import threading

class setbig(DetectionWin1):

    def __init__(self,configuration):
        super(setbig, self).__init__()
        print(1)
        self.setupUi(self)
        styleFile = '../../resource/setbig.qss'
        # 换肤时进行全局修改，只需要修改不同的QSS文件即可
        # self.Detection_test=Detection(configuration)
        # self.a=self.Detection_test.origin_image_show()
        # self.label.setScaledContents(True)
        # self.label.setPixmap(self.a)
        self.Detection_test = Detection(configuration)
        self.Detection_test.GreenButton.clicked.connect(self.signal)
        #
        print(2)
        self.g_bExit=True
        #self.origin_image_show_new()
        style = CommonHelper.readQss(styleFile)
        self.setStyleSheet(style)
        self.configuration = configuration


        #threading.Thread(target=self.show(),).start()
        #self.show()
    # def show(self):
    #     while True:
    #
    #         self.a=self.Detection_test.origin_image_show()
    #         self.label.setScaledContents(True)
    #         self.label.setPixmap(self.a)

    def signal(self):
        #Detection_test = Detection(self.configuration)
        self.Detection_test.image_show_big.connect(self.origin_image_show_new)
        print(10)
    def origin_image_show_new(self,hello):
        #self.label.setPixmap(hello)
        # pixmap = QPixmap("D:\\daima\\QT2\\new_detect11\\UI\\Big\\1.png")
        #
        # #print(pixmap)
        # #self.image_show_big.emit("hello")
        # self.label.setPixmap(QPixmap("D:\\daima\\QT2\\new_detect11\\UI\\Big\\1.png"))


        print(hello)








