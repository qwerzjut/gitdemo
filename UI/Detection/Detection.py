import sys
import time
import os
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtGui import QIcon, QPixmap, QStandardItem
from UI.Detection.DetectionWin import DetectionWin
from UI.Setting.Setting import Setting
from utils.CommonHelper import CommonHelper
# from Service.hkDetect import hkDetect
import PIL.Image as Image
import numpy as np
import cv2
from PyQt5.QtCore import pyqtSignal, QObject
from model.Opt import Opt
from model.Camera2 import CameraDiscrete,CameraContinuous
from matplotlib import pyplot as plt
#from UI.Big.setBig3 import setbig
#from UI.Frame.LeftTabWidget import LeftTabWidget
from UI.Setting.Setting import Setting
from  MvCameraControl_class import *
winfun_ctype = WINFUNCTYPE
DETECTION_THRESHOLD = 4000
OUT_PATH = "../../output/Image/"




class Detection(DetectionWin):
    deviceList = MV_CC_DEVICE_INFO_LIST()
    cam = MvCamera()
    image_show_big=pyqtSignal(object,object,object)
    def __init__(self,configuration):
        super(Detection,self).__init__()
        self.setupUi(self)
        styleFile = '../../resource/Detection.qss'
        # 换肤时进行全局修改，只需要修改不同的QSS文件即可
        style = CommonHelper.readQss(styleFile)
        self.setStyleSheet(style)
        #self.Setbig=setbig(configuration)
#        self.setting=Setting(configuration)
        self.detect_num = 0
        self.good_num = 0
        self.bad_num = 0
        self.bad_ratio = 0
        self.cams={}
        self.cam=0
       # print(self.setting.configuration.value("EPOCH"))
        self.configuration = configuration
        self.g_bExit = False
        self.cameraDetectionLabels = {"1":self.camera1_detection_label,"2":self.camera2_detection_label,"3":self.camera3_detection_label,"4":self.camera4_detection_label}
        #self.cameraDetectionLabels = {"1": self.camera1_detection_label, "2": self.camera2_detection_label}
        self.cameraResultLabels = {"1":self.camera1_result_label,"2":self.camera2_result_label,"3":self.camera3_result_label,"4":self.camera4_result_label}
        #self.cameraResultLabels = {"1": self.camera1_result_label, "2": self.camera2_result_label}
        self.GreenButton.clicked.connect(self.origin_image_show)
        self.RedButton.clicked.connect(self.stop)
        self.BlueButton.clicked.connect(self.reset)
#        self.grayButton1.clicked.connect(self.origin_image_show1)
        #self.grayButton1.clicked.connect(self.enlarge)
        self.result = open('../../result.txt', 'w')

    def detect(self):
        if(len(self.cams) == 0):
            self.opt = Opt()
            self.opt.cfg = self.configuration.value("CFG_PATH")
            self.opt.output = self.configuration.value("SAVE_IMG_PATH")
            self.opt.weights = self.configuration.value("WEIGHTS_PATH")
            # self.hkDetect = hkDetect(self.opt)
            self.GreenButton.setEnabled(False)
            self.RedButton.setEnabled(True)
            # 1号相机离散拍摄，2号相机连续拍摄
            for camNo in self.configuration.value('CAM_LIST'):
                if camNo == "5":
                    self.cam = CameraContinuous(camNo, self.opt, 1)
                else:
                    self.cam = CameraDiscrete(camNo, self.opt, 1)
                # cam = CameraDiscrete(camNo, self.opt, 1)
                self.cam.show_picture_signal.connect(self.origin_image_show)
                self.cam.detect_show_signal.connect(self.image_show)
                self.cams.update({camNo: self.cam})
                self.cam.openCam()
        else:
            for camNo in self.cams:
                self.cams[camNo].openCam()

    def recording(self):
        while True:
            pass


    def stop(self):
        for key in self.cams:
            self.cams[key].g_bExit = True
            self.cams[key].closeCam()
        self.result.close()
        self.GreenButton.setEnabled(True)
        self.RedButton.setEnabled(False)

    def reset(self):
        for camNo in self.cams:
            row = int(camNo)-1
            self.model.setItem(row, 1, QStandardItem("0"))
            self.model.setItem(row, 2,QStandardItem("0"))
            self.model.setItem(row, 3, QStandardItem("0"))
            self.model.setItem(row, 4, QStandardItem("0.0%"))
        self.detect_num = 0
        self.good_num = 0
        self.bad_num = 0
        self.bad_ratio = 0
        self.model.setItem(4, 1, QStandardItem("0"))
        self.model.setItem(4, 2, QStandardItem("0"))
        self.model.setItem(4, 3, QStandardItem("0"))
        self.model.setItem(4, 4, QStandardItem("0.0%"))
        for key in self.cams:
            self.cams[key].g_bExit = True
            self.cams[key].closeCam()
        self.hkDetect = None
        self.cams.clear()
        self.GreenButton.setEnabled(True)
        self.RedButton.setEnabled(False)


    # def origin_image_show(self,image,camNo):
    #     image = cv2.resize(image, (self.opt.width, self.opt.height))
    #     result_image = QtGui.QImage(image.data, image.shape[1], image.shape[0],
    #                                 QtGui.QImage.Format_RGB888)  # 把读取到的视频数据变成QImage形式
    #     self.cameraDetectionLabels[camNo].setPixmap(QtGui.QPixmap.fromImage(result_image))

    def origin_image_show(self):
        img=cv2.imread("D:\\daima\\QT2\\new_detect11\\UI\\Big\\2.former.jpg")
        pixmap1 = QPixmap("D:\\daima\\QT2\\new_detect11\\UI\\Big\\2.former.jpg")
        pixmap2 = QPixmap("D:\\daima\\QT2\\new_detect11\\UI\\Big\\0.bia.jpg")

        self.list=[]
        #print(pixmap)
        #self.image_show_big.emit("hello")

        #self.camera1_detection_label.setPixmap(QPixmap("D:\\daima\\QT2\\new_detect11\\UI\\Frame\\1.jpg"))
        self.camera2_detection_label.setPixmap(QPixmap("D:\\daima\\QT2\\new_detect11\\UI\\Frame\\2.jpg"))

        self.camera2_detection_label.setScaledContents(True)
        self.camera1_detection_label.setScaledContents(True)

        fig=plt.figure()
        plt.hist(img.ravel(), 256, [0, 256])
        fig.canvas.draw()

        # Get the RGBA buffer from the figure
        w, h = fig.canvas.get_width_height()
        buf = np.fromstring(fig.canvas.tostring_argb(), dtype=np.uint8)
        buf.shape = (w, h, 4)

        # canvas.tostring_argb give pixmap in ARGB mode. Roll the ALPHA channel to have it in RGBA mode
        buf = np.roll(buf, 3, axis=2)
        image = Image.frombytes("RGBA", (w, h), buf.tostring())
        image = np.asarray(image)
        image=cv2.cvtColor(image,cv2.COLOR_RGBA2RGB)
        # print(image)
        # cv2.imwrite("D:\\daima\\QT2\\new_detect11\\UI\\Frame\\5.jpg",image)
        cv2.imshow("1",image)
        result_image = QtGui.QImage(image.data, image.shape[1], image.shape[0],
                                          QtGui.QImage.Format_RGB888)  # 把读取到的视频数据变成QImage形式
        self.camera1_detection_label.setPixmap(QtGui.QPixmap.fromImage(result_image))
        self.camera1_detection_label.setScaledContents(True)
        self.image_show_big.emit(result_image,pixmap1,pixmap2)





        # # print(plt.show())
        # #a=plt.savefig("2.jpg")
        #pixmap = QPixmap.fromImage(image)
        # # #
        # # #
        #self.camera1_detection_label.setPixmap(pixmap)


        #plt.savefig("1.png")
        #self.camera2_detection_label.setPixmap(QPixmap())
        #self.Setbig.label.setPixmap(pixmap)
        #return pixmap
    # def origin_image_show1(self):
    #     #pixmap = QPixmap("D:\\daima\\QT2\\new_detect11\\UI\\Big\\1.png")
    #
    #     # print(pixmap)
    #     self.image_show_big.emit(1)
    #     self.camera1_detection_label.setPixmap(QPixmap("D:\\daima\\QT2\\new_detect11\\UI\\Big\\1.png"))
    #     self.camera2_detection_label.setPixmap(QPixmap("D:\\daima\\QT2\\new_detect11\\UI\\Big\\1.png"))
    #
    #     self.Setbig.label.setPixmap(pixmap)

    def image_show(self,image,flag,camNo):
        self.detect_num+=1
        image = cv2.resize(image , (self.opt.width, self.opt.height))
        result_image = QtGui.QImage(image.data, image.shape[1], image.shape[0],
                                QtGui.QImage.Format_RGB888)  # 把读取到的视频数据变成QImage形式
        self.cameraDetectionLabels[camNo].setPixmap(QtGui.QPixmap.fromImage(result_image))
        if flag:
            self.good_num+=1
        # row = int(camNo)-1
        # self.model.setItem(row,1, QStandardItem(str(camera.detect_num)))
        # self.model.setItem(row,2, QStandardItem(str(camera.good_num)))
        # self.model.setItem(row,3, QStandardItem(str(camera.bad_num)))
        # self.model.setItem(row,4, QStandardItem(str(round(camera.bad_num/camera.detect_num,3)*100)+"%"))
        self.model.setItem(4, 1, QStandardItem(str(self.detect_num)))
        self.model.setItem(4, 2, QStandardItem(str(self.good_num)))
        self.model.setItem(4, 3, QStandardItem(str(self.bad_num)))
        self.model.setItem(4, 4, QStandardItem(str(self.bad_ratio) + "%"))







