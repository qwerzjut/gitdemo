import sys
import time
import os
from PyQt5 import QtCore, QtWidgets, QtGui
from PyQt5.QtGui import QIcon, QPixmap, QStandardItem
from UI.Detection.DetectionWin import DetectionWin
from UI.Setting.Setting import Setting
from utils.CommonHelper import CommonHelper
# from Service.hkDetect import hkDetect
import numpy as np
import cv2
from model.Opt import Opt
from model.Camera import Camera

from  MvCameraControl_class import *
winfun_ctype = WINFUNCTYPE
DETECTION_THRESHOLD = 4000
OUT_PATH = "../../output/Image/"




class Detection(DetectionWin):
    deviceList = MV_CC_DEVICE_INFO_LIST()
    cam = MvCamera()
    def __init__(self,configuration):
        super(Detection,self).__init__()
        self.setupUi(self)
        styleFile = 'D:/defect_detection/resource/Detection.qss'
        # 换肤时进行全局修改，只需要修改不同的QSS文件即可
        style = CommonHelper.readQss(styleFile)
        self.setStyleSheet(style)
        self.detect_num = 0
        self.good_num = 0
        self.bad_num = 0
        self.bad_ratio = 0
        self.cams={}
        self.configuration = configuration
        self.g_bExit = False
        self.cameraDetectionLabels = {"1":self.camera1_detection_label,"2":self.camera2_detection_label,"3":self.camera3_detection_label,"4":self.camera4_detection_label}
        self.cameraResultLabels = {"1":self.camera1_result_label,"2":self.camera2_result_label,"3":self.camera3_result_label,"4":self.camera4_result_label}
        self.GreenButton.clicked.connect(self.detect)
        self.RedButton.clicked.connect(self.stop)
        self.BlueButton.clicked.connect(self.reset)
        self.result = open('D:/defect_detection/result.txt', 'w')

    def detect(self):
        if(len(self.cams) == 0):
            self.opt = Opt()
            self.opt.cfg = self.configuration.value("CFG_PATH")
            self.opt.output = self.configuration.value("SAVE_IMG_PATH")
            self.opt.weights = self.configuration.value("WEIGHTS_PATH")
            # self.hkDetect = hkDetect(self.opt)
            self.GreenButton.setEnabled(False)
            self.RedButton.setEnabled(True)
            for camNo in self.configuration.value('CAM_LIST'):
                cam = Camera(camNo, self.opt, 2)
                cam.show_picture_signal.connect(self.origin_image_show)
                cam.detect_show_signal.connect(self.image_show)
                self.cams.update({camNo: cam})
                cam.openCam()
        else:
            for camNo in self.cams:
                self.cams[camNo].openCam()


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


    def origin_image_show(self,image,camNo):
        image = cv2.resize(image, (self.opt.width, self.opt.height))
        result_image = QtGui.QImage(image.data, image.shape[1], image.shape[0],
                                    QtGui.QImage.Format_RGB888)  # 把读取到的视频数据变成QImage形式
        self.cameraDetectionLabels[camNo].setPixmap(QtGui.QPixmap.fromImage(result_image))


    def image_show(self,image,flag,camNo):
        # camera = self.cams[camNo]
        self.detect_num+=1
        image = cv2.resize(image,
                           (self.opt.width, self.opt.height))
        result_image = QtGui.QImage(image.data, image.shape[1], image.shape[0],
                                QtGui.QImage.Format_RGB888)  # 把读取到的视频数据变成QImage形式
        self.cameraDetectionLabels[camNo].setPixmap(QtGui.QPixmap.fromImage(result_image))
        if flag:
            self.good_num+=1
        row = int(camNo)-1
        # self.model.setItem(row,1, QStandardItem(str(camera.detect_num)))
        # self.model.setItem(row,2, QStandardItem(str(camera.good_num)))
        # self.model.setItem(row,3, QStandardItem(str(camera.bad_num)))
        # self.model.setItem(row,4, QStandardItem(str(round(camera.bad_num/camera.detect_num,3)*100)+"%"))
        self.model.setItem(4, 1, QStandardItem(str(self.detect_num)))
        self.model.setItem(4, 2, QStandardItem(str(self.good_num)))
        self.model.setItem(4, 3, QStandardItem(str(self.bad_num)))
        self.model.setItem(4, 4, QStandardItem(str(self.bad_ratio) + "%"))


