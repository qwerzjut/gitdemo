from PyQt5.QtCore import QObject, pyqtSignal
from UI.Detection.Detection import Detection
A=Detection.image_show_big.connect(img)
def img(a):
    print(a)