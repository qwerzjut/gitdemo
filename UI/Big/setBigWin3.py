# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'DetectionWin.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5.QtGui import QStandardItemModel, QPalette, QColor, QStandardItem
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QDesktopWidget, QHeaderView,QWidget


class DetectionWin1(QWidget):
    # def setupUi(self, Detection):
    #     Detection.setObjectName("Detection")
    #     Detection.resize(1320, 888)
    #     self.verticalLayout_2 = QtWidgets.QWidget(Detection)
    #     self.verticalLayout_2.setObjectName("verticalLayout_2")
    #     self.gridLayout = QtWidgets.QGridLayout()
    #     self.gridLayout.setObjectName("gridLayout")
    #     self.retranslateUi(Detection)
    #     QtCore.QMetaObject.connectSlotsByName(Detection)
    #
    # def retranslateUi(self, Detection):
    #     _translate = QtCore.QCoreApplication.translate
    #     Detection.setWindowTitle(_translate("Detection", "Form"))
    #     pe = QPalette()
    #     Detection.setAutoFillBackground(True)
    #     pe.setColor(QPalette.Window, QColor("#5A5A5A"))  # 设置背景色
    #     Detection.setPalette(pe)



    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1299, 763)
        # self.grid=QtWidgets.QGridLayout(Form)
        # self.camera4 = QtWidgets.QWidget(Form)
        # self.camera4.setObjectName("camera4")
        # self.camera3 = QtWidgets.QWidget(Form)
        # self.camera3.setObjectName("camera3")
        self.H_layout=QtWidgets.QHBoxLayout(Form)
        self.v_layout=QtWidgets.QVBoxLayout()
        self.groupBox = QtWidgets.QGroupBox(Form)
        self.groupBox.setGeometry(QtCore.QRect(200, 20, 1100, 800))
        #self.groupBox.setTitle("")
        self.groupBox.setObjectName("groupBox")

        self.label = QtWidgets.QLabel(self.groupBox)
        self.label.setGeometry(QtCore.QRect(100, 20, 800, 700))
        self.label.setObjectName("label")
        self.gridLayoutWidget = QtWidgets.QWidget(self.groupBox)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(100, 720, 800, 20))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.label_3 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_3.setObjectName("label_3")
        self.gridLayout.addWidget(self.label_3, 0, 2, 1, 1)
        self.label_4 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_4.setObjectName("label_4")
        self.gridLayout.addWidget(self.label_4, 0, 3, 1, 1)
        self.label_5 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_5.setObjectName("label_5")
        self.gridLayout.addWidget(self.label_5, 0, 4, 1, 1)
        self.label_7 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_7.setObjectName("label_7")
        self.gridLayout.addWidget(self.label_7, 0, 1, 1, 1)
        self.label8 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label8.setObjectName("label8")
        self.gridLayout.addWidget(self.label8, 0, 0, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_6.setObjectName("label_6")
        self.gridLayout.addWidget(self.label_6, 0, 5, 1, 1)
        # self.grid.addWidget(self.camera4,0,0,1,1)
        # self.grid.addWidget(self.camera3, 0, 1, 1, 1)
        #self.grid.addWidget(self.pushButton,0,1,1,1)
        self.weight = QtWidgets.QWidget(Form)
        self.weight.setGeometry(QtCore.QRect(15,200, 175, 50))
        self.formLayout = QtWidgets.QFormLayout(self.weight)
        self.formLayout.setContentsMargins(0, 0, 0, 0)
        self.formLayout.setObjectName("formLayout")
        self.label1 = QtWidgets.QLabel(self.weight)
        self.label1.setObjectName("label1")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.LabelRole, self.label1)
        self.pushButton1 = QtWidgets.QPushButton(self.weight)
        self.pushButton1.setFixedSize(80, 20)
        self.pushButton1.setObjectName("pushButton1")
        self.formLayout.setWidget(0, QtWidgets.QFormLayout.FieldRole, self.pushButton1)
        self.label_2 = QtWidgets.QLabel(self.weight)
        self.label_2.setObjectName("label_2")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.LabelRole, self.label_2)
        self.v_layout.addLayout(self.formLayout)
        self.pushButton_2 = QtWidgets.QPushButton(self.weight)
        self.pushButton_2.setFixedSize(80, 20)
        self.pushButton_2.setObjectName("pushButton_2")
        self.formLayout.setWidget(1, QtWidgets.QFormLayout.FieldRole, self.pushButton_2)
        self.pushButton = QtWidgets.QPushButton(self.weight)

        self.pushButton.setGeometry(QtCore.QRect(50, 60, 75, 23))
        self.pushButton.setObjectName("pushButton")
        self.v_layout.addWidget(self.pushButton)
        #self.weight.addlayout(self.v_layout)


        self.H_layout.addWidget(self.groupBox)
        self.H_layout.addWidget(self.weight)
        self.H_layout.setStretch(0, 7)
        self.H_layout.setStretch(1, 1)




        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Form", "Form"))
        self.pushButton.setText(_translate("Form", "还原"))
        self.label.setText(_translate("Form", "图片"))
        self.label1.setText(_translate("Form", "绘制ROI"))
        self.pushButton1.setText(_translate("Form", "重绘"))
        self.label_2.setText(_translate("Form", "恢复至最大画幅"))
        self.pushButton_2.setText(_translate("Form", "执行"))

        self.label_3.setText(_translate("Form", "2:"))
        self.label_4.setText(_translate("Form", "内容2"))
        self.label_5.setText(_translate("Form", "3:"))
        self.label_7.setText(_translate("Form", "内容1"))
        self.label8.setText(_translate("Form", "1:"))
        self.label_6.setText(_translate("Form", "内容3"))
        pe = QPalette()
        Form.setAutoFillBackground(True)
        pe.setColor(QPalette.Window, QColor("#5A5A5A"))  # 设置背景色
        Form.setPalette(pe)