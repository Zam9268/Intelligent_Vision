import seekfree, pyb
import sensor, image, time, tf, gc
import os,sys
import openmv_numpy as np
from pyb import LED
from machine import UART
sys.path.append('D:\\Smart_Car\\Project\\Intelligent_Vision\\openart')#这个路径为对应的openmv_numpy的路径
os.system('pip install --upgrade openmv_numpy')

def send_data(data,num):
   data_packet = []
   data_packet.append(0xB7) #发送包头，这里的append是增长数组的数据
   data_packet.append(num)#本次发送数据的数量
   if isinstance(data, list): # 如果data是列表，将其元素添加到data_packet
       data_packet.extend(data)
   else:
       data_packet.append(data) #发送数据
   data_packet.append(0x98) #发送包尾
   uart.write(bytearray(data_packet))#发送数据
   print(data_packet)#打印发送的数据
   time.sleep_ms(400)#发送数据后延时100ms,保证发送完成

sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
red = LED(1)    # 定义一个LED1   红灯
clock = time.clock()                # Create a clock object to track the FPS.
#目标检测思路：在找到一张卡片后，直接在该卡片中心的周围进行二次检测即可，不需要扫描全图
#设置模型路径
face_detect = 'yolo3_iou_smartcar_final_with_post_processing_5_11.tflite'
#载入模型
net = tf.load(face_detect)
uart = UART(2, baudrate=115200)#初始化UART2，波特率设置为115200
Inverse_Perspective=[[3.249708, -0.01971811, -15.86647], [0.0, 2.525229, -2.525229], [0.000294108, -0.005915673, 1]]
#下面的具体计算也要改，不能只改这里大哥啊啊啊啊啊
fixed_deta_y=440  #y坐标上的偏差
while(True):
    clock.tick()
    red.on()
    img = sensor.snapshot()
    #部分图像点的坐标测试
    #while(1):
    #   point=np.array([[20],[0],[1]])#创建3*1矩阵
    #   my_new_perspective=np.array([[2.470379, -0.07614613, 2.842548], [0.1327659, 2.200122, -9.900547], [-0.0001568885, -0.007409661, 1]])
    #   result_matrix=my_new_perspective*point
    #   finnal_matrix = np.array([[result_matrix[0][0]/result_matrix[2][0]-780], [result_matrix[1][0]/result_matrix[2][0]], [1]])
    #   print(finnal_matrix)#将坐标转化为笛卡尔坐标系
    #使用模型进行识别
    detected_objects = []
    for obj in tf.detect(net,img):
        x1,y1,x2,y2,label,scores = obj

        if(scores>0.70):
            print(obj)
            w = x2- x1
            h = y2 - y1
            x1 = int((x1-0.1)*img.width())
            y1 = int(y1*img.height())
            w = int(w*img.width())
            h = int(h*img.height())
            img.draw_rectangle((x1,y1,w,h),thickness=2)#thickness表示控制线的宽度，值越大线越粗。。
            center_x = x1 + w/2
            center_y = y1 + h/2
            center_x=center_x-80;#坐标矫正
            center_y=120-center_y;#坐标矫正，一定要加上去哇
            point=np.array([[center_x],[center_y],[1]])#创建3*1矩阵
            my_new_perspective=np.array([[3.249708, -0.01971811, -15.86647], [0.0, 2.525229, -2.525229], [0.000294108, -0.005915673, 1]])
            result_matrix=my_new_perspective*point
            finnal_matrix = np.array([[result_matrix[0][0]/result_matrix[2][0]], [result_matrix[1][0]/result_matrix[2][0]+fixed_deta_y], [1]])
            detected_objects.append(finnal_matrix)
            print(len(detected_objects))
            my_tabel=[]#创建待发送列表
            if(result_matrix[0][0]>0):
                my_tabel.append(1)#第一位表示x坐标的正负，1为正
                if(int(finnal_matrix[0][0])>=256):
                    my_tabel.append(1)#发送1表示x坐标存在溢出，否则不存在溢出
                else:
                    my_tabel.append(0)#表示x坐标不存在溢出
                #在添加x坐标的时候要判断是否存在溢出,前面已经判断了，后面就存入对应的取余后的坐标
                my_tabel.append(int(finnal_matrix[0][0]))#发送对应存储的x坐标（注意是取余后）
                #计算对应y坐标的溢出等级，0：无溢出，1：溢出一个256，后面以此类推，一般都不会超过1024个单位
                overflow_level = int(finnal_matrix[1][0]) // 256
                my_tabel.append(overflow_level)
                my_tabel.append(int(finnal_matrix[1][0]))#发送取余后的坐标
                send_data(my_tabel,5)
            else:#对x坐标判断是否为负
                my_tabel.append(0)#x坐标为负数
                x_zheng=-int(finnal_matrix[0][0])#对x坐标取正
                #判断x坐标的溢出等级
                if(x_zheng>=256 and x_zheng<=511):
                    my_tabel.append(1)#x坐标存在溢出
                else:
                    my_tabel.append(0)#不存在溢出
                #加入对应的取余后的x坐标
                my_tabel.append(x_zheng)
                #计算对应y坐标的溢出等级，0：无溢出，1：溢出一个256，后面以此类推，一般都不会超过1024个单位
                overflow_level = int(finnal_matrix[1][0]) // 256
                my_tabel.append(overflow_level)
                my_tabel.append(int(finnal_matrix[1][0]))#发送取余后的坐标
                send_data(my_tabel,5)
#    print(clock.fps())
