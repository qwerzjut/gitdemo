# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'DetectionWin1.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Detection(object):
    def setupUi(self, Detection):
        Detection.setObjectName("Detection")
        Detection.resize(1320, 888)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(Detection)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.camera4 = QtWidgets.QWidget(Detection)
        self.camera4.setObjectName("camera4")
        self.camera4_detection_label = QtWidgets.QLabel(self.camera4)
        self.camera4_detection_label.setGeometry(QtCore.QRect(80, 10, 440, 300))
        self.camera4_detection_label.setText("")
        self.camera4_detection_label.setObjectName("camera4_detection_label")
        self.camera4_label = QtWidgets.QLabel(self.camera4)
        self.camera4_label.setGeometry(QtCore.QRect(10, 10, 61, 21))
        self.camera4_label.setObjectName("camera4_label")
        self.camera4_result_label = QtWidgets.QLabel(self.camera4)
        self.camera4_result_label.setGeometry(QtCore.QRect(530, 10, 80, 30))
        self.camera4_result_label.setText("")
        self.camera4_result_label.setObjectName("camera4_result_label")
        self.gridLayout.addWidget(self.camera4, 1, 1, 1, 1)
        self.carmer1 = QtWidgets.QWidget(Detection)
        self.carmer1.setObjectName("carmer1")
        self.camera1_detection_label = QtWidgets.QLabel(self.carmer1)
        self.camera1_detection_label.setGeometry(QtCore.QRect(80, 10, 440, 300))
        self.camera1_detection_label.setText("")
        self.camera1_detection_label.setObjectName("camera1_detection_label")
        self.camera1_label = QtWidgets.QLabel(self.carmer1)
        self.camera1_label.setGeometry(QtCore.QRect(10, 10, 60, 20))
        self.camera1_label.setObjectName("camera1_label")
        self.camera1_result_label = QtWidgets.QLabel(self.carmer1)
        self.camera1_result_label.setGeometry(QtCore.QRect(530, 10, 81, 30))
        self.camera1_result_label.setText("")
        self.camera1_result_label.setObjectName("camera1_result_label")
        self.gridLayout.addWidget(self.carmer1, 0, 0, 1, 1)
        self.camera3 = QtWidgets.QWidget(Detection)
        self.camera3.setObjectName("camera3")
        self.camera3_detection_label = QtWidgets.QLabel(self.camera3)
        self.camera3_detection_label.setGeometry(QtCore.QRect(80, 10, 440, 300))
        self.camera3_detection_label.setText("")
        self.camera3_detection_label.setObjectName("camera3_detection_label")
        self.camera3_label = QtWidgets.QLabel(self.camera3)
        self.camera3_label.setGeometry(QtCore.QRect(10, 10, 60, 20))
        self.camera3_label.setObjectName("camera3_label")
        self.camera3_result_label = QtWidgets.QLabel(self.camera3)
        self.camera3_result_label.setGeometry(QtCore.QRect(530, 10, 80, 30))
        self.camera3_result_label.setText("")
        self.camera3_result_label.setObjectName("camera3_result_label")
        self.gridLayout.addWidget(self.camera3, 1, 0, 1, 1)
        self.camera2 = QtWidgets.QWidget(Detection)
        self.camera2.setObjectName("camera2")
        self.camera2_detection_label = QtWidgets.QLabel(self.camera2)
        self.camera2_detection_label.setGeometry(QtCore.QRect(80, 10, 440, 300))
        self.camera2_detection_label.setText("")
        self.camera2_detection_label.setObjectName("camera2_detection_label")
        self.camera2_label = QtWidgets.QLabel(self.camera2)
        self.camera2_label.setGeometry(QtCore.QRect(10, 10, 60, 20))
        self.camera2_label.setObjectName("camera2_label")
        self.camera2_result_label = QtWidgets.QLabel(self.camera2)
        self.camera2_result_label.setGeometry(QtCore.QRect(530, 10, 80, 30))
        self.camera2_result_label.setText("")
        self.camera2_result_label.setObjectName("camera2_result_label")
        self.gridLayout.addWidget(self.camera2, 0, 1, 1, 1)
        self.verticalLayout_2.addLayout(self.gridLayout)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.result_view = QtWidgets.QTableView(Detection)
        self.result_view.setObjectName("result_view")
        self.horizontalLayout.addWidget(self.result_view)
        self.widget = QtWidgets.QWidget(Detection)
        self.widget.setObjectName("widget")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.widget)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.product_name = QtWidgets.QLabel(self.widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.product_name.sizePolicy().hasHeightForWidth())
        self.product_name.setSizePolicy(sizePolicy)
        self.product_name.setObjectName("product_name")
        self.gridLayout_2.addWidget(self.product_name, 0, 0, 1, 1)
        self.product_name_label = QtWidgets.QLabel(self.widget)
        self.product_name_label.setText("")
        self.product_name_label.setObjectName("product_name_label")
        self.gridLayout_2.addWidget(self.product_name_label, 0, 1, 1, 1)
        self.product_id = QtWidgets.QLabel(self.widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.product_id.sizePolicy().hasHeightForWidth())
        self.product_id.setSizePolicy(sizePolicy)
        self.product_id.setObjectName("product_id")
        self.gridLayout_2.addWidget(self.product_id, 1, 0, 1, 1)
        self.product_id_label = QtWidgets.QLabel(self.widget)
        self.product_id_label.setText("")
        self.product_id_label.setObjectName("product_id_label")
        self.gridLayout_2.addWidget(self.product_id_label, 1, 1, 1, 1)
        self.detect_time = QtWidgets.QLabel(self.widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.detect_time.sizePolicy().hasHeightForWidth())
        self.detect_time.setSizePolicy(sizePolicy)
        self.detect_time.setObjectName("detect_time")
        self.gridLayout_2.addWidget(self.detect_time, 2, 0, 1, 1)
        self.detect_time_label = QtWidgets.QLabel(self.widget)
        self.detect_time_label.setText("")
        self.detect_time_label.setObjectName("detect_time_label")
        self.gridLayout_2.addWidget(self.detect_time_label, 2, 1, 1, 1)
        self.speed = QtWidgets.QLabel(self.widget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.speed.sizePolicy().hasHeightForWidth())
        self.speed.setSizePolicy(sizePolicy)
        self.speed.setObjectName("speed")
        self.gridLayout_2.addWidget(self.speed, 3, 0, 1, 1)
        self.speed_label = QtWidgets.QLabel(self.widget)
        self.speed_label.setText("")
        self.speed_label.setObjectName("speed_label")
        self.gridLayout_2.addWidget(self.speed_label, 3, 1, 1, 1)
        self.horizontalLayout.addWidget(self.widget)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.pushButton = QtWidgets.QPushButton(Detection)
        self.pushButton.setMinimumSize(QtCore.QSize(45, 45))
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout.addWidget(self.pushButton)
        self.pushButton_2 = QtWidgets.QPushButton(Detection)
        self.pushButton_2.setObjectName("pushButton_2")
        self.verticalLayout.addWidget(self.pushButton_2)
        self.pushButton_3 = QtWidgets.QPushButton(Detection)
        self.pushButton_3.setObjectName("pushButton_3")
        self.verticalLayout.addWidget(self.pushButton_3)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.horizontalLayout.setStretch(0, 7)
        self.horizontalLayout.setStretch(1, 3)
        self.horizontalLayout.setStretch(2, 1)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        self.verticalLayout_2.setStretch(0, 4)
        self.verticalLayout_2.setStretch(1, 1)

        self.retranslateUi(Detection)
        QtCore.QMetaObject.connectSlotsByName(Detection)

    def retranslateUi(self, Detection):
        _translate = QtCore.QCoreApplication.translate
        Detection.setWindowTitle(_translate("Detection", "Form"))
        self.camera4_label.setText(_translate("Detection", "4号相机"))
        self.camera1_label.setText(_translate("Detection", "1号相机"))
        self.camera3_label.setText(_translate("Detection", "3号相机"))
        self.camera2_label.setText(_translate("Detection", "2号相机"))
        self.product_name.setText(_translate("Detection", "产品名称:"))
        self.product_id.setText(_translate("Detection", "产品批次:"))
        self.detect_time.setText(_translate("Detection", "检测时间："))
        self.speed.setText(_translate("Detection", "运行速度："))
        self.pushButton.setText(_translate("Detection", "PushButton"))
        self.pushButton_2.setText(_translate("Detection", "PushButton"))
        self.pushButton_3.setText(_translate("Detection", "PushButton"))
