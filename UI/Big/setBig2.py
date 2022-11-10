from UI.Big.untitled import Ui_Form
from utils.CommonHelper import CommonHelper
from PyQt5.QtCore import QSettings, pyqtSignal
from PyQt5.QtWidgets import QFileDialog
from MvCameraControl_class import *
import os


class setbig(Ui_Form):
    train_modal_signal = pyqtSignal(object)
    def __init__(self,configuration):
        super(setbig, self).__init__()
        self.setupUi(self)
        styleFile = '../../resource/setbig.qss'
        # 换肤时进行全局修改，只需要修改不同的QSS文件即可

        style = CommonHelper.readQss(styleFile)
        self.setStyleSheet(style)
        self.configuration = configuration
