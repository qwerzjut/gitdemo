# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'TrainWin.ui'
#
# Created by: PyQt5 UI code generator 5.15.4
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.
from PyQt5.QtChart import QChartView
from PyQt5.QtGui import QPalette, QColor
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QWidget

class TrainWin(QWidget):
    def setupUi(self, Train):
        Train.setObjectName("Train")
        Train.resize(1337, 915)
        self.verticalLayout = QtWidgets.QVBoxLayout(Train)
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.start_sample = QtWidgets.QPushButton(Train)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.start_sample.sizePolicy().hasHeightForWidth())
        self.start_sample.setSizePolicy(sizePolicy)
        self.start_sample.setMinimumSize(QtCore.QSize(100, 35))
        self.start_sample.setObjectName("start_sample")
        self.horizontalLayout.addWidget(self.start_sample)
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem)
        self.stop_sample = QtWidgets.QPushButton(Train)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.stop_sample.sizePolicy().hasHeightForWidth())
        self.stop_sample.setSizePolicy(sizePolicy)
        self.stop_sample.setMinimumSize(QtCore.QSize(100, 35))
        self.stop_sample.setObjectName("stop_sample")
        self.horizontalLayout.addWidget(self.stop_sample)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem1)
        self.generate_dataSet = QtWidgets.QPushButton(Train)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.generate_dataSet.sizePolicy().hasHeightForWidth())
        self.generate_dataSet.setSizePolicy(sizePolicy)
        self.generate_dataSet.setMinimumSize(QtCore.QSize(100, 35))
        self.generate_dataSet.setObjectName("generate_dataSet")
        self.horizontalLayout.addWidget(self.generate_dataSet)
        spacerItem2 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem2)
        self.start_train = QtWidgets.QPushButton(Train)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.start_train.sizePolicy().hasHeightForWidth())
        self.start_train.setSizePolicy(sizePolicy)
        self.start_train.setMinimumSize(QtCore.QSize(100, 35))
        self.start_train.setObjectName("start_train")
        self.horizontalLayout.addWidget(self.start_train)
        spacerItem3 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem3)

        self.suspend_train = QtWidgets.QPushButton(Train)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.suspend_train.sizePolicy().hasHeightForWidth())
        self.suspend_train.setSizePolicy(sizePolicy)
        self.suspend_train.setMinimumSize(QtCore.QSize(100, 35))
        self.suspend_train.setObjectName("suspend_train")
        self.horizontalLayout.addWidget(self.suspend_train)

        self.resume_train = QtWidgets.QPushButton(Train)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Maximum, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.resume_train.sizePolicy().hasHeightForWidth())
        self.resume_train.setSizePolicy(sizePolicy)
        self.resume_train.setMinimumSize(QtCore.QSize(100, 35))
        self.resume_train.setObjectName("resume_train")
        self.horizontalLayout.addWidget(self.resume_train)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.tabWidget = QtWidgets.QTabWidget(Train)
        self.tabWidget.setObjectName("tabWidget")
        self.sample = QtWidgets.QWidget()
        self.sample.setObjectName("sample")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.sample)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.label = QtWidgets.QLabel(self.sample)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        self.label.setMinimumSize(QtCore.QSize(0, 30))
        self.label.setObjectName("label")
        self.verticalLayout_3.addWidget(self.label)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.carmer1 = QtWidgets.QLabel(self.sample)
        self.carmer1.setText("")
        self.carmer1.setObjectName("carmer1")
        self.gridLayout.addWidget(self.carmer1, 0, 0, 1, 1)
        self.carmer2 = QtWidgets.QLabel(self.sample)
        self.carmer2.setText("")
        self.carmer2.setObjectName("carmer2")
        self.gridLayout.addWidget(self.carmer2, 0, 1, 1, 1)
        self.carmer3 = QtWidgets.QLabel(self.sample)
        self.carmer3.setText("")
        self.carmer3.setObjectName("carmer3")
        self.gridLayout.addWidget(self.carmer3, 1, 0, 1, 1)
        self.carmer4 = QtWidgets.QLabel(self.sample)
        self.carmer4.setText("")
        self.carmer4.setObjectName("carmer4")
        self.gridLayout.addWidget(self.carmer4, 1, 1, 1, 1)
        self.verticalLayout_3.addLayout(self.gridLayout)
        self.verticalLayout_3.setStretch(0, 1)
        self.verticalLayout_3.setStretch(1, 10)
        self.tabWidget.addTab(self.sample, "")
        self.train = QtWidgets.QWidget()
        self.train.setObjectName("train")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.train)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.progressBar = QtWidgets.QProgressBar(self.train)
        self.progressBar.setProperty("value", 0)
        self.progressBar.setObjectName("progressBar")
        self.progressBar.setRange(0, 100)
        self.verticalLayout_2.addWidget(self.progressBar)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.chartView = QChartView(self.train)
        self.chartView.setObjectName("chartView")
        self.horizontalLayout_2.addWidget(self.chartView)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        self.tabWidget.addTab(self.train, "")
        self.verticalLayout.addWidget(self.tabWidget)
        self.retranslateUi(Train)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(Train)

    def retranslateUi(self, Train):
        _translate = QtCore.QCoreApplication.translate
        Train.setWindowTitle(_translate("Train", "Form"))
        self.start_sample.setText(_translate("Train", "开始采样"))
        self.stop_sample.setText(_translate("Train", "停止采样"))
        self.generate_dataSet.setText(_translate("Train", "生成数据集"))
        self.start_train.setText(_translate("Train", "开始训练"))
        self.suspend_train.setText(_translate("Train", "中断训练"))
        self.resume_train.setText(_translate("Train", "恢复训练"))
        self.label.setText(_translate("Train", "采样数："))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.sample), _translate("Train", "sample"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.train), _translate("Train", "train"))
        pe = QPalette()
        Train.setAutoFillBackground(True)
        pe.setColor(QPalette.Window, QColor("#5A5A5A"))  # 设置背景色
        Train.setPalette(pe)