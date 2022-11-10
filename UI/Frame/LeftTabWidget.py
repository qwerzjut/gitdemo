from PyQt5.QtWidgets import QListWidget,QStackedWidget
from PyQt5.QtWidgets import QListWidgetItem
from PyQt5.QtWidgets import QWidget
from PyQt5.QtWidgets import QHBoxLayout
from PyQt5.QtCore import QSize, Qt
from PyQt5 import QtGui
from UI.Detection.Detection import Detection
from UI.Setting.Setting import Setting
from UI.PictureDetection.PictureDetection_one import  PictureDetection_one
from UI.PictureDetection.PictureDetection_two import  PictureDetection_two
from UI.PictureDetection.PictureDetection_three import  PictureDetection_three
from UI.PictureDetection.PictureDetection_four import  PictureDetection_four
#from UI.Big.setbig import setbig
from UI.Big.setBig3 import setbig
from UI.Annotation.Annotation import Annotation
#from UI.Train.Train import Train


class LeftTabWidget(QWidget):
     '''左侧选项栏'''
     def __init__(self,configuration):
         super(LeftTabWidget, self).__init__()
         self.setObjectName('LeftTabWidget')
         self.setWindowTitle('LeftTabWidget')
         with open('../../resource/leftTab.qss', 'r', encoding='utf-8') as f:   #导入QListWidget的qss样式
             self.list_style = f.read()
         self.main_layout = QHBoxLayout(self, spacing=0)     #窗口的整体布局
         self.main_layout.setContentsMargins(0,0,0,0)
         self.left_widget = QListWidget()     #左侧选项列表
         self.left_widget.setStyleSheet(self.list_style)
         self.main_layout.addWidget(self.left_widget)
         self.right_widget = QStackedWidget()
         self.main_layout.addWidget(self.right_widget)
         self.showWin = None
         self.configuration = configuration
         self.setbig = setbig(self.configuration)
         self.Detection = Detection(self.configuration)
         self.PictureDetection=PictureDetection_one(self.configuration)
         # self.Annotation = Annotation(self.configuration)
         # self.Train = Train(self.configuration)
         self.Setting = Setting(configuration)

         self.picture_two=PictureDetection_two(self.configuration)
         self.picture_three=PictureDetection_three(self.configuration)
         self.picture_four=PictureDetection_four(self.configuration)
#         self.Setting.train_modal_signal.connect(self.update_train)
         self._setup_ui()
         self.Detection.grayButton1.clicked.connect(self.Enlarge)
         self.Detection.grayButton2.clicked.connect(self.Enlarge)
         self.Detection.grayButton3.clicked.connect(self.Enlarge)
         self.Detection.grayButton4.clicked.connect(self.Enlarge)
         self.setbig.pushButton.clicked.connect(self.restore)

         self.PictureDetection.num_1.clicked.connect(self.picture_show_one)
         self.PictureDetection.num_2.clicked.connect(self.picture_show_two)
         self.PictureDetection.num_3.clicked.connect(self.picture_show_three)
         self.PictureDetection.num_5.clicked.connect(self.picture_show_four)
         self.picture_two.num_1.clicked.connect(self.picture_show_one)
         self.picture_two.num_2.clicked.connect(self.picture_show_two)
         self.picture_two.num_3.clicked.connect(self.picture_show_three)
         self.picture_two.num_5.clicked.connect(self.picture_show_four)
         self.picture_three.num_1.clicked.connect(self.picture_show_one)
         self.picture_three.num_2.clicked.connect(self.picture_show_two)
         self.picture_three.num_3.clicked.connect(self.picture_show_three)
         self.picture_three.num_5.clicked.connect(self.picture_show_four)
         self.picture_four.num_1.clicked.connect(self.picture_show_one)
         self.picture_four.num_2.clicked.connect(self.picture_show_two)
         self.picture_four.num_3.clicked.connect(self.picture_show_three)
         self.picture_four.num_5.clicked.connect(self.picture_show_four)
         self.Detection.image_show_big.connect(self.img_showbig)


     # def update_train(self,title):
     #     self.Train.chart.setTitle("训练模式: "+title)
     def _setup_ui(self):
         '''加载界面ui'''
         #self.left_widget.currentRowChanged.connect(self.right_widget.setCurrentIndex)   #list和右侧窗口的index对应绑定
         self.left_widget.setFrameShape(QListWidget.NoFrame)    #去掉边框
         self.left_widget.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)  #隐藏滚动条
         self.left_widget.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
         #list_str = ['实时检测','图片检测','图片标注','模型训练','设置']
         list_str = ['实时检测', '图片检测', '设置']
         #win_list=[self.Detection,self.PictureDetection,self.Annotation,self.Train,self.Setting]
         win_list = [self.Detection, self.PictureDetection, self.Setting]
         for i in range(3):
             self.item = QListWidgetItem(list_str[i],self.left_widget)   #左侧选项的添加
             self.item.setSizeHint(QSize(30,60))
             self.item.setTextAlignment(Qt.AlignCenter)                  #居中显示
             self.right_widget.addWidget(win_list[i])
         self.left_widget.currentRowChanged.connect(self.right_widget.setCurrentIndex)  # list和右侧窗口的index对应绑定
         self.right_widget.addWidget(self.setbig)
         self.right_widget.addWidget(self.picture_two)
         self.right_widget.addWidget(self.picture_three)
         self.right_widget.addWidget(self.picture_four)
     def Enlarge(self):
         #self.right_widget.addWidget(self.setbig)
         #d=Detection
         self.right_widget.setCurrentIndex(3)
     def restore(self):
         self.right_widget.setCurrentIndex(0)
     def picture_show_one(self):
         self.right_widget.setCurrentIndex(1)
     def picture_show_two(self):
        self.right_widget.setCurrentIndex(4)
     def picture_show_three(self):
         #self.right_widget.addWidget(self.picture_three)
         # d=Detection
         self.right_widget.setCurrentIndex(5)
     def picture_show_four(self):
         #self.right_widget.addWidget(self.picture_four)
         # d=Detection
         self.right_widget.setCurrentIndex(6)
     def img_showbig(self,a,b,c):
         self.setbig.label.setPixmap(QtGui.QPixmap.fromImage(a))
         self.setbig.label.setScaledContents(True)
         self.PictureDetection.p.setPixmap(QtGui.QPixmap.fromImage(a))
         self.PictureDetection.p.setScaledContents(True)
         self.PictureDetection.p2.setPixmap(b)
         self.PictureDetection.p2.setScaledContents(True)
         self.PictureDetection.p1.setPixmap(c)
         self.PictureDetection.p1.setScaledContents(True)





