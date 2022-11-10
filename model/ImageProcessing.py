import cv2
import numpy as np
import winsound
from PIL import Image, ImageDraw, ImageFont
import time
import sys
def detect(former, latter, threshold,x, y, w, h):
    # start=time.time()
    (mean, stddv) = cv2.meanStdDev(former)
    dif = cv2.absdiff(latter,former)
    mask=roi(former,x,y,w,h)
    ro=cv2.bitwise_and(dif,mask)
    grey=cv2.cvtColor(ro,cv2.COLOR_RGB2GRAY)
    bia=cv2.bilateralFilter(grey,5,150,150)
    thresh, bins = cv2.threshold(bia,mean[0, 0], 255, cv2.THRESH_BINARY)
    kernel = np.ones((5, 5), np.uint8)
    median = cv2.medianBlur(bins, 3)
    dilate = cv2.dilate(median, kernel, iterations=1)
    # 只计算外轮廓
    contours, hierarchy = cv2.findContours(dilate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    nums = len(contours)
    # print(nums)
    flag = False
    area=[]
    perimeter=[]
    centroid=[]
    shape=[]
    # 防止image成为局部变量
    image = latter
    for i in range(nums):
        area_ = cv2.contourArea(contours[i])
        if area_>threshold:
            area.append(area_)
            # rect返回矩形的特征信息，其结构为【最小外接矩形的中心（x，y），（宽度，高度），旋转角度】
            rect = cv2.minAreaRect(contours[i])
            # arearect = rect[1][0] * rect[1][1]
            # 计算质心
            M=cv2.moments(contours[i])
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            centroid.append((cx,cy))
            # 发出警报
            # winsound.Beep(440,200)
            # 计算周长
            perimeter_ = cv2.arcLength(contours[i], True)
            perimeter.append(perimeter_)
            # 判断形状
            shape_type=discriminate_shape(contours[i],perimeter_)
            shape.append(shape_type)
            image = cv2.drawContours(image, (contours[i],), 0, (0, 0, 255), 3)
            xmin=min(contours[i][:,0,0])
            ymin=min(contours[i][:,0,1])
            # 加文字各参数依次是：图片，添加的文字，左上角坐标，字体，字体大小，颜色，字体粗细
            text = f"形状:{shape_type}\n" + "周长:{:.1f}\n".format(perimeter_) + "面积:{:.0f}\n".format(
                area_) + "质心:({:.0f},{:.0f})".format(cx, cy)
            image=cv2ImgAddText(image,text,xmin,ymin)
            flag = True
        # 输出标注后图片，是否有轮廓，轮廓数，轮廓周长、面积、质心、形状的序列
    # end=time.time()
    # print(end-start)
    return image,nums,flag,perimeter,area,centroid,shape


# 掩膜方法
def roi(picture, widthstart, heigthstart, width, heigth):
    if (heigth + heigthstart > picture.shape[0]) | (width + widthstart > picture.shape[1]):
        print("结束值越界")
        sys.exit()
    if (heigthstart < 0) | (widthstart < 0) | (width < 0) | (heigth < 0):
        print("开始值越界")
        sys.exit()
    mask = np.zeros(picture.shape, np.uint8)
    mask[heigthstart:heigthstart + heigth, widthstart:widthstart + width] = 255
    return mask

def discriminate_shape(contour,peri):
    approx = cv2.approxPolyDP(contour,0.02*peri,True)
    corners = len(approx)
    if corners == 3:
        shape_type = "三角形"
            # "triangle"
    if corners == 4:
        shape_type = "矩形"
            # "rectangle"
    if corners >= 10:
        # circles = cv2.HoughCircles(image,cv2.HOUGH_GRADIENT, 1, 100, param1=100, param2=10, minRadius=50, maxRadius=500)
        shape_type ="圆形"
            # "round"
    if 4 < corners < 10:
        shape_type = "多边形"
            # "polygon"
    return shape_type

# OpenCV图片格式转换成PIL的图片格式；
# 使用PIL绘制文字；
# PIL图片格式转换成OpenCV的图片格式；
def cv2ImgAddText(img, text, left, top, textColor=(0, 255, 0), textSize=100):
    if (isinstance(img, np.ndarray)):  # 判断是否OpenCV图片类型
        img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    # 创建一个可以在给定图像上绘图的对象
    draw = ImageDraw.Draw(img)
    # 字体的格式
    fontStyle = ImageFont.truetype(
        "font/simsun.ttc", textSize, encoding="utf-8")
    # 绘制文本
    draw.text((left, top), text, textColor, font=fontStyle)
    # 转换回OpenCV格式
    return cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)

def cv2ImgAddText2(image,left,top,shape_type,perimeter,area,cx,cy):
    text = ["shape:" + str(shape_type), "perimeter:%0.1f" % perimeter, "area:%d" % area,
            "centroid:(%d,%d)" % (cx, cy)]
    # f"shape:{shape_type}\n"+f"perimeter:{perimeter}\n"+f"area:{area_}\n"+f"centroid:({cx},{cy})"
    cv2.putText(image, text[0],
                (left - 15, top - 105), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    cv2.putText(image, text[1],
                (left - 15, top - 75), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    cv2.putText(image, text[2],
                (left - 15, top - 45), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    cv2.putText(image, text[3],
                (left - 15, top - 15), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
    return image


if __name__=="__main__":
    # former=cv2.imread(r"C:\Users\52590\Desktop\Euphoira.jpg")
    # latter=cv2.imread(r"C:\Users\52590\Desktop\Euphoira2.jpg")
    former = cv2.imread(r"C:\Users\52590\Desktop\6.bmp")
    latter = cv2.imread(r"C:\Users\52590\Desktop\7.bmp")
    x,y,w,h=cv2.selectROI('roi',former,False,False)
    cv2.destroyAllWindows()
    image,flag,nums,perimeter,area,centroid,shape=detect(former,latter,12500,x,y,w,h)
    cv2.imshow('image',image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

