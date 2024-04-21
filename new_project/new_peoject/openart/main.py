import seekfree, pyb
import sensor, image, time, tf, gc
import os,sys
import openmv_numpy as np
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
clock = time.clock()                # Create a clock object to track the FPS.
#目标检测思路：在找到一张卡片后，直接在该卡片中心的周围进行二次检测即可，不需要扫描全图
#设置模型路径
face_detect = 'yolo3_iou_smartcar_final_with_post_processing.tflite'
#载入模型
net = tf.load(face_detect)

Inverse_Perspective=[[2.470379, -0.07614613, 2.842548], [0.1327659, 2.200122, -9.900547], [-0.0001568885, -0.007409661, 1]]
#上面这个是最新的逆透视矩阵，效果非常好
fixed_deta_y=25  #y坐标上的偏差
uart = UART(2, baudrate=115200)#初始化UART2，波特率设置为115200
while(True):
    while(True):
        send_data([0x05,0x02],2)
    clock.tick()
    img = sensor.snapshot()
    #部分图像点的坐标测试
    #while(1):
    #   point=np.array([[20],[0],[1]])#创建3*1矩阵
    #   my_new_perspective=np.array([[2.470379, -0.07614613, 2.842548], [0.1327659, 2.200122, -9.900547], [-0.0001568885, -0.007409661, 1]])
    #   result_matrix=my_new_perspective*point
    #   finnal_matrix = np.array([[result_matrix[0][0]/result_matrix[2][0]-780], [result_matrix[1][0]/result_matrix[2][0]], [1]])
    #   print(finnal_matrix)#将坐标转化为笛卡尔坐标系
    #使用模型进行识别
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
            my_new_perspective=np.array([[2.470379, -0.07614613, 2.842548], [0.1327659, 2.200122, -9.900547], [-0.0001568885, -0.007409661, 1]])
            result_matrix=my_new_perspective*point
            finnal_matrix = np.array([[result_matrix[0][0]/result_matrix[2][0]], [result_matrix[1][0]/result_matrix[2][0]+fixed_deta_y], [1]])
            if(result_matrix[0][0]>0):
                send_data([[1,result_matrix[0][0]/result_matrix[2][0]], [result_matrix[1][0]/result_matrix[2][0]+fixed_deta_y]],3)
            else
                send_data([[2,-result_matrix[0][0]/result_matrix[2][0]], [result_matrix[1][0]/result_matrix[2][0]+fixed_deta_y]],3)
            print(finnal_matrix)#将坐标转化为笛卡尔坐标系
    print(clock.fps())
