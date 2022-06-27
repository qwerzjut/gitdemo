import os
import copy
import numpy as np
from MvCameraControl_class import *
from PyQt5.QtCore import pyqtSignal, QObject
# from Service.hkDetect import hkDetect
import threading
import cv2
import time
import random
# from Service.pre_treat import roi_split
from project.parse_config import parse_rbc_name

winfun_ctype = WINFUNCTYPE
stFrameInfo = POINTER(MV_FRAME_OUT_INFO_EX)
stEventInfo = POINTER(MV_EVENT_OUT_INFO)
pData = POINTER(c_ubyte)
FrameInfoCallBack = winfun_ctype(None, pData, stFrameInfo, c_void_p)
EventInfoCallBack = winfun_ctype(None, stEventInfo, c_void_p)
# DETECTION_THRESHOLD = 4000
deviceList = MV_CC_DEVICE_INFO_LIST()
tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
path=r'D:\picture'
distance = 75.0




# CALL_BACK_FUN = EventInfoCallBack(event_callback)

class Camera(QObject):
    show_picture_signal = pyqtSignal(object,object)
    show_detect_info = pyqtSignal(object,object)
    sample_signal = pyqtSignal(object,object)
    detect_show_signal = pyqtSignal(object,object,object)
    def __init__(self,camNo,opt,slot):
        super(Camera, self).__init__()
        self.camNo = camNo
        # self.show_picture_signal = pyqtSignal(object, object, object)
        self.detect_num = 0
        self.bad_num = 0
        self.good_num = 0
        self.call_back_fun=EventInfoCallBack(self.event_callback)
        self.defectStatistic={}
        lambda :self.defectStatistic_init(opt.names)
        self.speedValue=0.0
        self.ratio = 0.0
        self.opt = opt
        self.imgs = []
        # if slot == 1:
        #  self.hkDetect = hkDetect(opt)
        # self.hkDetect = hkDetect(opt)
        self.g_bExit = False
        self.slot = slot
        # self.timeout = timeout

    def event_callback(self,pEventInfo, pUser):
        # stPEventInfo = cast(pEventInfo, POINTER(MV_EVENT_OUT_INFO)).contents
        # nBlockId = stPEventInfo.nBlockIdHigh
        # nBlockId = (nBlockId << 32) + stPEventInfo.nBlockIdLow
        # nTimestamp = stPEventInfo.nTimestampHigh
        # nTimestamp = (nTimestamp << 32) + stPEventInfo.nTimestampLow
        # if stPEventInfo:
        #     print("EventName[%s], EventId[%u], BlockId[%d], Timestamp[%d]" % (
        #     stPEventInfo.EventName, stPEventInfo.nEventID, nBlockId, nTimestamp))
        time.sleep(2.175000)
        ret =self.cam.MV_CC_SetCommandValue('TriggerSoftware')
        if ret != 0:
            print("Warning: Set Trigger Software fail! ret[0x%x]" % ret)

    def defectStatistic_init(self, path=None):
        if not os.path.exists(path) and os.path.exists('data' + os.sep + path):  # add data/ prefix if omitted
            path = 'data' + os.sep + path

        with open(path, 'r') as f:
            lines = f.readlines()
        for line in lines:
            line = line.strip()
            if line == '' or line.startswith('#'):
                continue
            self.defectStatistic[line]=0



    def openCam(self):

        self.start = time.time()
        ret = MvCamera.MV_CC_EnumDevices(tlayerType, deviceList)
        self.cam = MvCamera()
        # ch:选择设备并创建句柄 | en:Select device and create handle
        stDeviceList = cast(deviceList.pDeviceInfo[int(self.camNo) - 1], POINTER(MV_CC_DEVICE_INFO)).contents
        ret = self.cam.MV_CC_CreateHandle(stDeviceList)
        # ch:打开设备 | en:Open device
        ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        # ch:探测网络最佳包大小(只对GigE相机有效) | en:Detection network optimal package size(It only works for the GigE camera)
        if stDeviceList.nTLayerType == MV_GIGE_DEVICE:
            nPacketSize = self.cam.MV_CC_GetOptimalPacketSize()
            if int(nPacketSize) > 0:
                ret = self.cam.MV_CC_SetIntValue("GevSCPSPacketSize", nPacketSize)
                if ret != 0:
                    print("Warning: Set Packet Size fail! ret[0x%x]" % ret)
            else:
                print("Warning: Get Packet Size fail! ret[0x%x]" % nPacketSize)


        # ch:设置触发模式为ON(外触发) | en:Set trigger mode as on
        ret = self.cam.MV_CC_SetEnumValue("TriggerMode", 1)
        if ret != 0:
            print("set trigger mode fail! ret[0x%x]" % ret)
            sys.exit()

        # ret = device.MV_CC_SetEnumValue("TriggerSource", 7)
        # if ret != 0:
        #     print("Set TriggerSource failed! ret[0x%x]" % ret)
        #     sys.exit()


        ret = self.cam.MV_CC_SetFloatValue("TriggerDelay", 0)
        if ret != 0:
            print("set trigger delay fail! ret[0x%x]" % ret)
            sys.exit()

        ret = self.cam.MV_CC_SetBoolValue("TriggerCacheEnable", True)
        if ret != 0:
            print("set trigger cache enable fail! ret[0x%x]" % ret)
            sys.exit()
        #设置自动增益
        ret = self.cam.MV_CC_SetEnumValue("GainAuto", 0)
        if ret != 0:
            print("set enum value  fail! ret[0x%x]" % ret)
            sys.exit()
        # 设置具体增益
        ret = self.cam.MV_CC_SetFloatValue("Gain", 12)
        if ret != 0:
            print("set float value  fail! ret[0x%x]" % ret)
            sys.exit()

        ret = self.cam.MV_CC_SetFloatValue("ExposureTime", 12500.0)
        if ret != 0:
            print("set Exposure Time fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:获取数据包大小 | en:Get payload size
        stParam = MVCC_INTVALUE()
        memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
        ret = self.cam.MV_CC_GetIntValue("PayloadSize", stParam)
        if ret != 0:
            print("get payload size fail! ret[0x%x]" % ret)
            sys.exit()
        nPayloadSize = stParam.nCurValue

             # ch:开启Event | en:Set Event of ExposureEnd On
        ret = self.cam.MV_CC_SetEnumValueByString("EventSelector","Line0RisingEdge")
        if ret != 0:
            print ("set enum value by string fail! ret[0x%x]" % ret)
            sys.exit()



        ret = self.cam.MV_CC_SetEnumValueByString("EventNotification","On")
        if ret != 0:
            print ("set enum value by string fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:注册事件回调 | en:Register event callback
        ret = self.cam.MV_CC_RegisterEventCallBackEx("Line0RisingEdge", self.call_back_fun,None)
        if ret != 0:
            print ("register event callback fail! ret [0x%x]" % ret)
            sys.exit()


        # ch:开始取流 | en:Start grab image
        ret = self.cam.MV_CC_StartGrabbing()
        if ret != 0:
            print("start grabbing fail! ret[0x%x]" % ret)
            sys.exit()

        data_buf = (c_ubyte * nPayloadSize)()

        try:
            if (self.slot == 1):
                threading.Thread(target=self.detect_work_thread,
                                 args=(self.cam, data_buf, nPayloadSize, self.camNo)).start()
            else:
                threading.Thread(target=self.show_work_thread,
                                 args=(self.cam, data_buf, nPayloadSize, self.camNo)).start()

        except:
            print("error: unable to start thread")


    def closeCam(self):
        # ch:停止取流 | en:Stop grab image
        ret = self.cam.MV_CC_StopGrabbing()
        if ret != 0:
            print("stop grabbing fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:关闭设备 | Close device
        ret = self.cam.MV_CC_CloseDevice()
        if ret != 0:
            print("close deivce fail! ret[0x%x]" % ret)
            sys.exit()

        # ch:销毁句柄 | Destroy handle
        ret = self.cam.MV_CC_DestroyHandle()
        if ret != 0:
            print("destroy handle fail! ret[0x%x]" % ret)
            sys.exit()


    def detect_work_thread(self, cam=0, pData=0, nDataSize=0, camNo=0):
        print("Frame method")
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
        former=[]
        r=2140000
        while True:
            if r==2140000:
                r=r+40000
            else:
                r=r-40000
            if len(former):   #不为空
                ret = cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
                if ret == 0:
                    self.detect_num+=1
                    ret = self.cam.MV_CC_SetFloatValue("TriggerDelay", r)
                    if ret != 0:
                        print("set trigger delay fail! ret[0x%x]" % ret)
                        sys.exit()
                    print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                         stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))
                    latter = np.asarray(pData).reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                    latter = cv2.cvtColor(latter, cv2.COLOR_BGR2RGB)
                    # cv2.imshow('latter',latter)
                    # cv2.waitKey(0)

                    # self.show_picture_signal.emit(latter,camNo)
                    detected_image, flag = self.detect(former,latter)
                    # detected_image=cv2.cvtColor(detected_image,cv2.COLOR_GRAY2RGB)
                    self.show_picture_signal.emit(detected_image,camNo)
                    print(r)
                    former=latter
                    if flag:
                        self.good_num += 1
                else:
                    print("no data[0x%x]" % ret)
                if self.g_bExit == True:
                    break
            else:
                ret = cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
                if ret == 0:
                    self.detect_num += 1
                    print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                        stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))
                    former = np.asarray(pData).reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                    former = cv2.cvtColor(former, cv2.COLOR_BGR2RGB)
                    self.show_picture_signal.emit(former,camNo)
                    print(r)
                    # cv2.waitKey(0)
                    # self.show_picture_signal.emit(former,camNo)
                else:
                    print("no data[0x%x]" % ret)
                if self.g_bExit == True:
                    break

    def show_work_thread(self,cam=0, pData=0, nDataSize=0, camNo=0):

        print("Background method")
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
        former=[]
        while True:
            ret = cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
            if ret==0:
                if len(former):   #不为空
                    self.detect_num+=1
                    print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                         stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))
                    latter = np.asarray(pData).reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                    latter = cv2.cvtColor(latter, cv2.COLOR_BGR2RGB)
                    detected_image, flag = self.detect(former,latter)
                    if flag:
                        self.good_num += 1
                    self.detect_show_signal.emit(detected_image,flag,camNo)
                    former=[]
                else:
                    self.detect_num += 1
                    print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                    stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))
                    former = np.asarray(pData).reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                    former = cv2.cvtColor(former, cv2.COLOR_BGR2RGB)
                    # self.show_picture_signal.emit(former,camNo)
            else:
                print("no data[0x%x]" % ret)
            if self.g_bExit == True:
                break

    def train_work_thread(self, cam=0, pData=0, nDataSize=0, camNo=0):
        stFrameInfo = MV_FRAME_OUT_INFO_EX()
        memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
        while True:
            ret = cam.MV_CC_GetOneFrameTimeout(pData, nDataSize, stFrameInfo, 1000)
            if ret == 0:
                print("get one frame: Width[%d], Height[%d], nFrameNum[%d]" % (
                    stFrameInfo.nWidth, stFrameInfo.nHeight, stFrameInfo.nFrameNum))


            else:
                ret = cam.MV_CC_SetCommandValue("TriggerSoftware")
                if ret !=0:
                    print("Software Trigger failed ret[0x%x]" % ret)
                print("no data[0x%x]" % ret)
            if self.g_bExit == True:
                break

    def detect(self,former,latter):
        latter=cv2.cvtColor(latter,cv2.COLOR_RGB2GRAY)
        former=cv2.cvtColor(former,cv2.COLOR_RGB2GRAY)
        (mean, stddv) = cv2.meanStdDev(latter)
        dif=cv2.subtract(former,latter)
        thresh, bins = cv2.threshold(dif, 0.4*mean[0, 0], 255, cv2.THRESH_BINARY)
        kernel = np.ones((3, 3), np.uint8)
        median = cv2.medianBlur(bins, 3)
        dilate = cv2.dilate(median, kernel, iterations=1)
        contours, hierarchy = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        nums = len(contours)
        max = 0
        sign = 0
        flag = False
        latter=cv2.cvtColor(latter,cv2.COLOR_GRAY2RGB)
        image=latter
        for i in range(nums):
            rect = cv2.minAreaRect(contours[i])  # rect返回矩形的特征信息，其结构为【最小外接矩形的中心（x，y），（宽度，高度），旋转角度】
            area = rect[1][0] * rect[1][1]
            if max < area:
                max = area
                sign = i
        print(max)
        if max>50000:
            rect = cv2.minAreaRect(contours[sign])
            points = cv2.boxPoints(rect)  # 得到最小外接矩形的四个点坐标
            points = np.int0(points)  # 坐标值取整
            image = cv2.drawContours(latter, [points], 0, (0, 0, 255), 5)  # 直接在原图上绘制矩形框
            flag=True
        return image,flag


