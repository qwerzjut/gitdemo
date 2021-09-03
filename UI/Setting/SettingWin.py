# -*- coding: utf-8 -*-

# Setting implementation generated from reading ui file 'SettingWin.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5.QtGui import  QPalette, QColor
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget
class SettingWin(QWidget):
    def setupUi(self, Setting):
        Setting.setObjectName("Setting")
        Setting.resize(1293, 850)
        self.Carmer_control = QtWidgets.QGroupBox(Setting)
        self.Carmer_control.setGeometry(QtCore.QRect(100, 40, 311, 361))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.Carmer_control.setFont(font)
        self.Carmer_control.setObjectName("Carmer_control")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.Carmer_control)
        self.verticalLayout.setObjectName("verticalLayout")
        self.carmer_one = QtWidgets.QCheckBox(self.Carmer_control)

        font = QtGui.QFont()
        font.setPointSize(15)
        self.carmer_one.setFont(font)
        self.carmer_one.setObjectName("carmer_one")
        self.verticalLayout.addWidget(self.carmer_one)
        self.carmer_two = QtWidgets.QCheckBox(self.Carmer_control)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.carmer_two.setFont(font)
        self.carmer_two.setObjectName("carmer_two")
        self.verticalLayout.addWidget(self.carmer_two)
        self.carmer_three = QtWidgets.QCheckBox(self.Carmer_control)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.carmer_three.setFont(font)
        self.carmer_three.setObjectName("carmer_three")
        self.verticalLayout.addWidget(self.carmer_three)
        self.carmer_four = QtWidgets.QCheckBox(self.Carmer_control)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.carmer_four.setFont(font)
        self.carmer_four.setObjectName("carmer_four")
        self.verticalLayout.addWidget(self.carmer_four)
        self.carmer_all = QtWidgets.QCheckBox(self.Carmer_control)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.carmer_all.setFont(font)
        self.carmer_all.setObjectName("carmer_all")
        self.verticalLayout.addWidget(self.carmer_all)
        self.model_setting = QtWidgets.QGroupBox(Setting)
        self.model_setting.setGeometry(QtCore.QRect(110, 470, 1021, 231))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.model_setting.setFont(font)
        self.model_setting.setObjectName("model_setting")
        self.weights_label = QtWidgets.QLabel(self.model_setting)
        self.weights_label.setGeometry(QtCore.QRect(20, 40, 211, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.weights_label.setFont(font)
        self.weights_label.setObjectName("weights_label")
        self.img_save_path_label = QtWidgets.QLabel(self.model_setting)
        self.img_save_path_label.setGeometry(QtCore.QRect(20, 90, 161, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.img_save_path_label.setFont(font)
        self.img_save_path_label.setObjectName("img_save_path_label")
        self.weights_lineEdit = QtWidgets.QLineEdit(self.model_setting)
        self.weights_lineEdit.setGeometry(QtCore.QRect(280, 40, 441, 31))
        self.weights_lineEdit.setObjectName("weights_lineEdit")
        self.save_path_lineEdit = QtWidgets.QLineEdit(self.model_setting)
        self.save_path_lineEdit.setGeometry(QtCore.QRect(280, 90, 441, 31))
        self.save_path_lineEdit.setObjectName("save_path_lineEdit")
        self.weights_upload = QtWidgets.QPushButton(self.model_setting)
        self.weights_upload.setGeometry(QtCore.QRect(800, 40, 81, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.weights_upload.setFont(font)
        self.weights_upload.setObjectName("weights_upload")
        self.img_path_open = QtWidgets.QPushButton(self.model_setting)
        self.img_path_open.setGeometry(QtCore.QRect(800, 90, 81, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.img_path_open.setFont(font)
        self.img_path_open.setObjectName("img_path_open")
        self.cfg_label = QtWidgets.QLabel(self.model_setting)
        self.cfg_label.setGeometry(QtCore.QRect(20, 140, 131, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.cfg_label.setFont(font)
        self.cfg_label.setObjectName("cfg_label")
        self.cfg_lineEdit = QtWidgets.QLineEdit(self.model_setting)
        self.cfg_lineEdit.setGeometry(QtCore.QRect(280, 140, 441, 31))
        self.cfg_lineEdit.setObjectName("cfg_lineEdit")
        self.cfg_upload = QtWidgets.QPushButton(self.model_setting)
        self.cfg_upload.setGeometry(QtCore.QRect(800, 140, 81, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.cfg_upload.setFont(font)
        self.cfg_upload.setObjectName("cfg_upload")
        self.confirm = QtWidgets.QPushButton(Setting)
        self.confirm.setGeometry(QtCore.QRect(1000, 760, 91, 41))
        self.confirm.setObjectName("confirm")
        self.cancel = QtWidgets.QPushButton(Setting)
        self.cancel.setGeometry(QtCore.QRect(1140, 760, 81, 41))
        self.cancel.setObjectName("cancel")
        self.Acquisition_Control = QtWidgets.QGroupBox(Setting)
        self.Acquisition_Control.setGeometry(QtCore.QRect(440, 40, 411, 91))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.Acquisition_Control.setFont(font)
        self.Acquisition_Control.setContextMenuPolicy(QtCore.Qt.NoContextMenu)
        self.Acquisition_Control.setObjectName("Acquisition_Control")
        self.layoutWidget = QtWidgets.QWidget(self.Acquisition_Control)
        self.layoutWidget.setGeometry(QtCore.QRect(11, 26, 391, 41))
        self.layoutWidget.setObjectName("layoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.layoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.fps_label = QtWidgets.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.fps_label.setFont(font)
        self.fps_label.setObjectName("fps_label")
        self.horizontalLayout.addWidget(self.fps_label)
        self.fps_LineEdit = QtWidgets.QLineEdit(self.layoutWidget)
        self.fps_LineEdit.setObjectName("fps_LineEdit")
        self.horizontalLayout.addWidget(self.fps_LineEdit)
        self.Image_Settingat_Control = QtWidgets.QGroupBox(Setting)
        self.Image_Settingat_Control.setGeometry(QtCore.QRect(870, 40, 321, 351))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.Image_Settingat_Control.sizePolicy().hasHeightForWidth())
        self.Image_Settingat_Control.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.Image_Settingat_Control.setFont(font)
        self.Image_Settingat_Control.setContextMenuPolicy(QtCore.Qt.DefaultContextMenu)
        self.Image_Settingat_Control.setObjectName("Image_Settingat_Control")
        self.HeightMax_value_label = QtWidgets.QLabel(self.Image_Settingat_Control)
        self.HeightMax_value_label.setGeometry(QtCore.QRect(150, 110, 131, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.HeightMax_value_label.setFont(font)
        self.HeightMax_value_label.setText("")
        self.HeightMax_value_label.setObjectName("HeightMax_value_label")
        self.WidthMax_value_label = QtWidgets.QLabel(self.Image_Settingat_Control)
        self.WidthMax_value_label.setGeometry(QtCore.QRect(150, 50, 131, 31))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.WidthMax_value_label.setFont(font)
        self.WidthMax_value_label.setText("")
        self.WidthMax_value_label.setObjectName("WidthMax_value_label")
        self.Height_spinBox = QtWidgets.QSpinBox(self.Image_Settingat_Control)
        self.Height_spinBox.setGeometry(QtCore.QRect(130, 230, 161, 31))
        self.Height_spinBox.setObjectName("Height_spinBox")
        self.Height_spinBox.setMaximum(2048)
        self.HeightMax_label = QtWidgets.QLabel(self.Image_Settingat_Control)
        self.HeightMax_label.setGeometry(QtCore.QRect(20, 110, 119, 19))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.HeightMax_label.setFont(font)
        self.HeightMax_label.setObjectName("HeightMax_label")
        self.WidthMax_label = QtWidgets.QLabel(self.Image_Settingat_Control)
        self.WidthMax_label.setGeometry(QtCore.QRect(20, 50, 109, 19))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.WidthMax_label.setFont(font)
        self.WidthMax_label.setObjectName("WidthMax_label")
        self.Width_label = QtWidgets.QLabel(self.Image_Settingat_Control)
        self.Width_label.setGeometry(QtCore.QRect(20, 170, 79, 20))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.Width_label.setFont(font)
        self.Width_label.setObjectName("Width_label")
        self.Height_label = QtWidgets.QLabel(self.Image_Settingat_Control)
        self.Height_label.setGeometry(QtCore.QRect(20, 230, 80, 19))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.Height_label.setFont(font)
        self.Height_label.setObjectName("Height_label")
        self.Width_spinBox = QtWidgets.QSpinBox(self.Image_Settingat_Control)
        self.Width_spinBox.setGeometry(QtCore.QRect(130, 170, 161, 31))
        self.Width_spinBox.setObjectName("Width_spinBox")
        self.Width_spinBox.setMaximum(2448)
        self.Skin_Setting = QtWidgets.QGroupBox(Setting)
        self.Skin_Setting.setGeometry(QtCore.QRect(440, 180, 411, 91))
        font = QtGui.QFont()
        font.setPointSize(14)
        self.Skin_Setting.setFont(font)
        self.Skin_Setting.setObjectName("Skin_Setting")
        self.bright_radioButton = QtWidgets.QRadioButton(self.Skin_Setting)
        self.bright_radioButton.setGeometry(QtCore.QRect(30, 40, 101, 20))
        self.bright_radioButton.setObjectName("bright_radioButton")
        self.dark_radioButton = QtWidgets.QRadioButton(self.Skin_Setting)
        self.dark_radioButton.setGeometry(QtCore.QRect(190, 40, 101, 20))
        self.dark_radioButton.setObjectName("dark_radioButton")

        self.retranslateUi(Setting)
        QtCore.QMetaObject.connectSlotsByName(Setting)

    def retranslateUi(self, Setting):
        _translate = QtCore.QCoreApplication.translate
        Setting.setWindowTitle(_translate("Setting", "Setting"))
        self.Carmer_control.setTitle(_translate("Setting", "Carmer Control"))
        self.carmer_one.setText(_translate("Setting", "Carmer one"))
        self.carmer_two.setText(_translate("Setting", "Carmer two"))
        self.carmer_three.setText(_translate("Setting", "Carmer three"))
        self.carmer_four.setText(_translate("Setting", "Carmer four"))
        self.carmer_all.setText(_translate("Setting", "All"))
        self.model_setting.setTitle(_translate("Setting", "Model Setting"))
        self.weights_label.setText(_translate("Setting", "import model weights:"))
        self.img_save_path_label.setText(_translate("Setting", "image save path:"))
        self.weights_upload.setText(_translate("Setting", "upload"))
        self.img_path_open.setText(_translate("Setting", "open"))
        self.cfg_label.setText(_translate("Setting", "import cfg:"))
        self.cfg_upload.setText(_translate("Setting", "upload"))
        self.confirm.setText(_translate("Setting", "确定"))
        self.cancel.setText(_translate("Setting", "取消"))
        self.Acquisition_Control.setTitle(_translate("Setting", "Acquisition Control"))
        self.fps_label.setText(_translate("Setting", "Acquisition Frame Rate(Fps):"))
        self.Image_Settingat_Control.setTitle(_translate("Setting", "Image Settingat Control"))
        self.HeightMax_label.setText(_translate("Setting", " HeightMax："))
        self.WidthMax_label.setText(_translate("Setting", " WidthMax："))
        self.Width_label.setText(_translate("Setting", " Width："))
        self.Height_label.setText(_translate("Setting", " Height:"))
        self.Skin_Setting.setTitle(_translate("Setting", "Skin Setting"))
        self.bright_radioButton.setText(_translate("Setting", "bright"))
        self.dark_radioButton.setText(_translate("Setting", "dark"))
        pe = QPalette()
        Setting.setAutoFillBackground(True)
        pe.setColor(QPalette.Window, QColor("#5A5A5A"))  # 设置背景色
        Setting.setPalette(pe)
        self.carmer_one.setStyleSheet("color: white")
        self.carmer_two.setStyleSheet("color: white")
        self.carmer_three.setStyleSheet("color: white")
        self.carmer_four.setStyleSheet("color: white")
        self.carmer_all.setStyleSheet("color: white")
        self.bright_radioButton.setStyleSheet("color: white;font-size:20px")
        self.dark_radioButton.setStyleSheet("color: white;font-size:20px")
