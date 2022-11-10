# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'PictureDetection.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.

from PyQt5.QtGui import QStandardItemModel, QPalette, QColor
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget, QHeaderView


class PictureDetectionWin(QWidget):
    def setupUi(self, PictureDetection):
        PictureDetection.setObjectName("PictureDetection")
        PictureDetection.resize(988, 828)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(PictureDetection)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")

        #self.QStackedWidget = QtWidgets.QStackedWidget()

        self.gridLayout_new1=QtWidgets.QGridLayout()
        self.pushputton_new1=QtWidgets.QPushButton()
        self.pushputton_new1.setObjectName("pushputton_new1")
        self.pushputton_new2=QtWidgets.QPushButton()
        self.pushputton_new2.setObjectName("pushputton_new2")
        self.gridLayout_new1.addWidget(self.pushputton_new1,0,0,1,1)
        self.gridLayout_new1.addWidget(self.pushputton_new2,0,1,1,1)
        self.verticalLayout.addLayout(self.gridLayout_new1)

        #self.new_group=QtWidgets.QGroupBox(PictureDetection)
        #self.G_box=QtWidgets.QGroupBox(self.new_weight)
        self.QStackedWidget = QtWidgets.QStackedWidget(PictureDetection)

        #self.scrollAreaWidgetContents = QtWidgets.QWidget(PictureDetection)
        self.origin_scrollArea = QtWidgets.QGroupBox(self.QStackedWidget)
        self.p=QtWidgets.QLabel(self.origin_scrollArea)
        #self.p.setGeometry(QtCore.QRect(100, 20, 800, 700))
        self.p.setObjectName("p")
        self.origin_scrollArea.setObjectName("origin_scrollArea")

        #self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.origin_scrollArea1 = QtWidgets.QGroupBox(self.QStackedWidget)
        self.p1=QtWidgets.QLabel(self.origin_scrollArea1)
        #self.p.setGeometry(QtCore.QRect(100, 20, 800, 700))
        self.p1.setObjectName("p1")
        self.origin_scrollArea1.setObjectName("origin_scrollArea1")


        self.QStackedWidget.addWidget(self.origin_scrollArea)
        self.QStackedWidget.addWidget(self.origin_scrollArea1)
        self.verticalLayout.addWidget(self.QStackedWidget)
        self.horizontalLayout.addLayout(self.verticalLayout)
        self.QStackedWidget.setCurrentIndex(1)



        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.pushputton_new3 = QtWidgets.QPushButton(PictureDetection)
        font = QtGui.QFont()
        font.setPointSize(20)
        self.pushputton_new3.setFont(font)
        self.pushputton_new3.setObjectName("pushputton_new3")
        self.verticalLayout_2.addWidget(self.pushputton_new3)
        self.scrollAreaWidgetContents_2 = QtWidgets.QWidget(PictureDetection)
        self.scrollAreaWidgetContents_2.setObjectName("scrollAreaWidgetContents_2")
        self.detection_scrollArea = QtWidgets.QGroupBox(self.scrollAreaWidgetContents_2)
        self.detection_scrollArea.setObjectName("detection_scrollArea")
        self.gridLayout = QtWidgets.QGridLayout(self.scrollAreaWidgetContents_2)

        self.verticalLayout_2.addWidget(self.detection_scrollArea)
        self.horizontalLayout.addLayout(self.verticalLayout_2)
        self.horizontalLayout.setStretch(0, 2)
        self.horizontalLayout.setStretch(1, 2)
        self.verticalLayout_3.addLayout(self.horizontalLayout)

        spacerItem = QtWidgets.QSpacerItem(20, 100, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)
        # self.verticalLayout_3.addItem(spacerItem)
        # self.model = QStandardItemModel(0, 2);
        #
        # self.model.setHorizontalHeaderLabels(['图片编号','缺陷类型'])
        # self.resultView = QtWidgets.QTableView()
        # self.resultView.setModel(self.model)
        # self.resultView.setObjectName("resultView")
        # self.resultView.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        # self.verticalLayout_3.addWidget(self.resultView)

        self.verticalLayout_3.addItem(spacerItem)
        self.horizontalLayout_2.addLayout(self.verticalLayout_3)
        self.verticalLayout_4 = QtWidgets.QVBoxLayout()
        self.verticalLayout_4.setSpacing(0)
        self.verticalLayout_4.setObjectName("verticalLayout_4")

        spacerItem6 = QtWidgets.QSpacerItem(20,15, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        self.verticalLayout_4.addItem(spacerItem6)
        self.Qlabel_1 = QtWidgets.QLabel(PictureDetection)
        self.Qlabel_1.setMinimumSize(QtCore.QSize(91, 41))
        self.Qlabel_1.setObjectName("num_1")
        self.verticalLayout_4.addWidget(self.Qlabel_1)


        spacerItem1 = QtWidgets.QSpacerItem(20,200, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        self.verticalLayout_4.addItem(spacerItem1)
        self.num_1 = QtWidgets.QPushButton(PictureDetection)
        self.num_1.setMinimumSize(QtCore.QSize(91, 41))
        self.num_1.setObjectName("num_1")
        self.verticalLayout_4.addWidget(self.num_1)
        spacerItem2 = QtWidgets.QSpacerItem(60, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        self.verticalLayout_4.addItem(spacerItem2)
        self.num_2 = QtWidgets.QPushButton(PictureDetection)
        self.num_2.setMinimumSize(QtCore.QSize(91, 41))
        self.num_2.setObjectName("num_2")
        self.verticalLayout_4.addWidget(self.num_2)
        spacerItem3 = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        self.verticalLayout_4.addItem(spacerItem3)
        self.num_3 = QtWidgets.QPushButton(PictureDetection)
        self.num_3.setMinimumSize(QtCore.QSize(91, 41))
        self.num_3.setObjectName("num_3")
        self.verticalLayout_4.addWidget(self.num_3)
        spacerItem5 = QtWidgets.QSpacerItem(20, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        self.verticalLayout_4.addItem(spacerItem5)

        self.num_5 = QtWidgets.QPushButton(PictureDetection)
        self.num_5.setMinimumSize(QtCore.QSize(91, 41))
        self.num_5.setObjectName("num_5")
        self.verticalLayout_4.addWidget(self.num_5)

        spacerItem7 = QtWidgets.QSpacerItem(20,80, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        self.verticalLayout_4.addItem(spacerItem7)
        self.gridLayout_3 = QtWidgets.QGridLayout(PictureDetection)
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.spinBox = QtWidgets.QSpinBox(PictureDetection)
        self.spinBox.setObjectName("spinBox")
        self.gridLayout_3.addWidget(self.spinBox, 0, 1, 1, 1)
        self.label_4 = QtWidgets.QLabel(PictureDetection)
        self.label_4.setObjectName("label_4")
        self.gridLayout_3.addWidget(self.label_4, 0, 0, 1, 1)
        self.label_5 = QtWidgets.QLabel(PictureDetection)
        self.label_5.setObjectName("label_5")
        self.gridLayout_3.addWidget(self.label_5, 1, 0, 1, 1)
        self.spinBox_2 = QtWidgets.QSpinBox(PictureDetection)
        self.spinBox_2.setObjectName("spinBox_2")
        self.gridLayout_3.addWidget(self.spinBox_2, 1, 1, 1, 1)
        self.verticalLayout_4.addLayout(self.gridLayout_3)
        # self.pushButton_7 = QtWidgets.QPushButton(self.weight_1_new)
        # self.pushButton_7.setGeometry(QtCore.QRect(310, 680, 75, 23))
        # self.pushButton_7.setObjectName("pushButton_7")








        spacerItem4 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)


        self.verticalLayout_4.addItem(spacerItem4)
        self.horizontalLayout_2.addLayout(self.verticalLayout_4)
        self.horizontalLayout_2.setStretch(0,8)
        self.horizontalLayout_2.setStretch(1, 1)
        self.retranslateUi(PictureDetection)
        QtCore.QMetaObject.connectSlotsByName(PictureDetection)

    def retranslateUi(self, PictureDetection):
        _translate = QtCore.QCoreApplication.translate
        PictureDetection.setWindowTitle(_translate("PictureDetection", "Form"))
        self.pushputton_new1.setText(_translate("PictureDetection", "直方图"))
        self.pushputton_new2.setText(_translate("PictureDetection", "帧差图"))
        self.pushputton_new3.setText(_translate("PictureDetection", "结果图"))
        self.num_1.setText(_translate("PictureDetection", "一号相机"))
        self.num_2.setText(_translate("PictureDetection", "二号相机"))
        self.num_3.setText(_translate("PictureDetection", "三号相机"))
        self.num_5.setText(_translate("PictureDetection", "四号相机"))
        self.Qlabel_1.setText(_translate("PictureDetection", "相机编号：一号相机"))
        self.label_4.setText(_translate("PictureDetection", "阈值："))
        self.label_5.setText(_translate("PictureDetection", "面积："))
        self.p.setText(_translate("PictureDetection", "直方图"))
        self.p1.setText(_translate("PictureDetection", "帧差图"))


        pe = QPalette()
        PictureDetection.setAutoFillBackground(True)
        pe.setColor(QPalette.Window, QColor("#5A5A5A"))  # 设置背景色
        PictureDetection.setPalette(pe)