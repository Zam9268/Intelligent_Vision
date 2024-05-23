#用A4纸求出逆透视的矩阵
import sensor, image, time
import os
#while(1):
#    print("begin\n")
#    print(os.getcwd())
#    print(sys.path)
#    print("end\r")
import sys
sys.path.append('D:\\Smart_Car\\Project\\Intelligent_Vision\\openart')#这个路径为对应的openmv_numpy的路径
os.system('pip install --upgrade openmv_numpy')
import openmv_numpy as np
from machine import UART
import time
import pyb
from pyb import LED
from machine import Pin

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)#160,120 320,240
sensor.skip_frames(time = 2000)
sensor.set_auto_exposure(True)

#320*240 第一个矩阵 [[1.617274, -0.03811955, -7.224679], [0.03712427, 1.351323, -6.452198], [0.0005643311, -0.003098827, 1]]
#A4纸参数
#可以换成其它尺寸
#总钻风距离车中心底部坐标为225mm
a4_w = 297
a4_h = 210

world_coordinates = [[(-a4_w/2),0],[a4_w/2,0],[(a4_w/2),a4_h],[(-a4_w/2),a4_h]]#这个顺序是对的，坐标对的上
#定义A4纸在现实世界的4个坐标，分别对应A4纸的4个角的现实坐标

#返回透视矩阵
#XY为世界坐标，UV为相机坐标

def cal_mtx(UV: np.array, XY: np.array) -> np.array:
    """
    使用给定的UV和XY坐标计算变换矩阵H。

    参数:
    UV (np.array): 形状为(4, 2)的数组，表示UV坐标。
    XY (np.array): 形状为(4, 2)的数组，表示XY坐标。

    返回:
    np.array: 形状为(3, 3)的变换矩阵H。
    """

    A = []
    B = []
    for i in range(4):
       a = [[UV[i][0], UV[i][1], 1, 0, 0, 0, -XY[i][0] * UV[i][0], -XY[i][0] * UV[i][1]],
           [0, 0, 0, UV[i][0], UV[i][1], 1, -XY[i][1] * UV[i][0], -XY[i][1] * UV[i][1]]]
       B += [[XY[i][0]],
            [XY[i][1]]]
       A += a

    A = np.array(A)
    B = np.array(B)

    x = np.solve(A, B)

    H = [[x[0][0], x[1][0], x[2][0]],
        [x[3][0], x[4][0], x[5][0]],
        [x[6][0], x[7][0], 1]]

    return np.array(H)

show =True
while(True):
    while(True):
        img = sensor.snapshot()
        new_img_coordinate=[]
        new_img_coordinate.append([-30,101])
        new_img_coordinate.append([30,101])
        new_img_coordinate.append([62,32])
        new_img_coordinate.append([-62,32])
        new_world_coordinates=[]
        new_world_coordinates.append([-210,590])
        new_world_coordinates.append([210,590])
        new_world_coordinates.append([210,80])
        new_world_coordinates.append([-210,80])#偏移坐标210
        H= cal_mtx(new_img_coordinate,new_world_coordinates)
        pyb.mdelay(1000)
        print(H)
    img = sensor.snapshot()
    for r in img.find_rects(threshold = 20000):#这个矩形包含的像素点至少为20000个，防止矩形误判
        img.draw_rectangle(r.rect(), color = (255, 0, 0))#画出矩形,这个矩形框为红色
        img_coordinate=[]#定义一个列表，用来存放矩形的四个角的坐标
        print("********")#打印间隔符
        if show:#如果show为True，则显示矩形的四个角的坐标
            for p in r.corners():#存储矩形的四个角点的坐标，r.corners返回对象的4个角的4 (x,y)元组列表。从左上方开始按顺时针顺序返回角的坐标
                img.draw_circle(p[0], p[1], 2, color = (0, 255, 0))#画出矩形的四个角点，半径为2个像素点，颜色为绿色
                img_coordinate.append([p[0]-80, 120-p[1]])
                print(p[0]-80,120-p[1])
        dn_cx = (img_coordinate[0][0]+img_coordinate[1][0])/2
        dn_cy = (img_coordinate[1][0]+img_coordinate[1][1])/2
        up_cx = (img_coordinate[2][0]+img_coordinate[3][0])/2
        up_cy = (img_coordinate[2][1]+img_coordinate[3][1])/2
        print((dn_cx+up_cx)/2)
        print(abs((dn_cx-up_cx)/(dn_cy-up_cy)))
        print(abs((dn_cx+up_cx)/2))
        #居中判定
        if abs((dn_cx-up_cx)/(dn_cy-up_cy))<=0.2 and abs((dn_cx+up_cx)/2)<=20:
            img_coordinate =np.array(img_coordinate)
            world_coordinates =np.array(world_coordinates)
            H= cal_mtx(img_coordinate,world_coordinates)
            pyb.mdelay(1000)
            print(H)
    img.draw_line(80, 120, 80, 0, color = (255, 0, 0), thickness = 1)
