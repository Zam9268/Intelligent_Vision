# IO口接线：接TFT液晶显示屏的
# 蓝线：GND->GND   黄线：VCC->VCC     灰线：SCL->B0      黑线：SDA->B1      紫线：RES->B12     绿色：DC->B13  橙色：CS->B3   白色：BL->B16
import seekfree, pyb
import sensor, image, time, math
import ustruct
import os, tf
from pyb import LED
from machine import UART

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

k=0
map_flag=0
photo_flag=1
sending=1
signal='get'#单片机正确接收到数据之后返回字符串“get”，signal用于验证从uart读取到的信号
start_signal='sta'#uart接收到start数据后才开始识别图片
key_signal='map'
uart_num = 0#art从uart里获取到的数据的个数
baotou=[0xB7]#包头
baowei=[0x98]#包尾，都可用于验证数据集的准确性
testabc=[0x07]
# 初始化屏幕
lcd = seekfree.LCD180(3)

blob_threshold=(31, 100, -128, 127, -2, 127)#
black_threshold=(83, 100, -33, 127, -126, 127)#反黑色滤波（防止黑色）
#blob_threshold=(-128,127,-128,127,-128,127)#检测时的无滤波
# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)#经典RGB三通道
sensor.set_framesize(sensor.QVGA)#QVGA：240*320
sensor.set_brightness(950)#亮度设置
sensor.skip_frames(time = 20)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False,(0,0x80,0))  # must turn this off to prevent image washout...
#sensor.set_auto_whitebal(True,(0,0,0))
lcd = seekfree.LCD180(3)#初始化屏幕
lcd.full()  # 将背景颜色显示到整个屏幕
uart = UART(2, baudrate=115200)#初始化UART2，波特率设置为115200

net_path = "mobilenet_v2-2024-02-26T12-37-27.164Z_in-int8_out-int8_channel_ptq.tflite"                                  # 定义模型的路径，这个模型由eiq进行提供，自己要训练
labels = [line.rstrip() for line in open("/sd/mobilenet_v2_ABC_labels.txt")]   # 加载标签
net = tf.load(net_path, load_to_fb=True)#new_path：预训练模型的文件路径 load_to_fb：模型会被加载到帧缓冲区

while(1):

    sensor.set_auto_whitebal(False)#关闭白平衡
    img = sensor.snapshot()
    #这个是通过色块来找图片
    for blobs in img.find_blobs([black_threshold]):
        if blobs.h() < 50 or blobs.w() < 50:#当找到的色块大于一定值才会进行识别
            continue#小于的话会进行不断识别
        img = img.draw_rectangle(blobs.rect(),color = (255, 0, 0))    # 绘制矩形外框，便于在IDE上查看识别到的矩形位置，
        #img = img.draw_string(10,10, "%s = %f" % (sorted_list[i][0], sorted_list[i][1]),color=(255, 0, 0), scale=3)
        img1 = img.copy(1,1,blobs.rect())  # 拷贝矩形框内的图像，提高检测效率
        # 默认设置只是进行一次检测...更改它们以搜索图像...
        for obj in tf.classify(net , img1, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
            sorted_list = sorted(zip(labels, obj.output()), key = lambda x: x[1], reverse = True)
            for i in range(1):#只运行一次，好像没有什么用，但是不要删，取出第一个元素值（也就是对应概率最高的元素），将这个类别和概率作为字符串显示
                #print("%s = %f" % (sorted_list[i][0], sorted_list[i][1]))#sorted_list[i][0]指的是标签内的名字
                img = img.draw_string(10,10, "%s=%f" % (sorted_list[i][0], sorted_list[i][1]),color=(255, 0, 0), scale=3)                                                      #sorted_list[i][1]指的是模型输出的概率值
            #对概率最高的进行匹配，选择最恰当的那一个进行发送数据
            if sorted_list[i][0]=='A':
               print('A')
               send_data([0x01,0x01],2)#发送数据
            elif sorted_list[i][0]=='B':
               print('B')
               send_data([0x01,0x02],2)
            elif sorted_list[i][0]=='C':
               print('C')
               send_data([0x01,0x03],2)
            elif sorted_list[i][0]=='D':
               print('D')
               send_data([0x01,0x04],2)
            elif sorted_list[i][0]=='E':
               print('E')
               send_data([0x01,0x05],2)
            elif sorted_list[i][0]=='F':
               print('F')
               send_data([0x01,0x06],2)
            elif sorted_list[i][0]=='G':
               print('G')
               send_data([0x01,0x07],2)
            elif sorted_list[i][0]=='H':
               print('H')
               send_data([0x01,0x08],2)
            elif sorted_list[i][0]=='I':
               print('I')
               send_data([0x01,0x09],2)
            elif sorted_list[i][0]=='J':
               print('J')
               send_data(0x01,[0x0A],2)
            elif sorted_list[i][0]=='K':
               print('K')
               send_data([0x01,0x0B],2)
            elif sorted_list[i][0]=='L':
               print('L')
               send_data([0x01,0x0C],2)
            elif sorted_list[i][0]=='M':
               print('M')
               send_data([0x01,0x0D],2)
            elif sorted_list[i][0]=='N':
               print('N')
               send_data([0x01,0x0E],2)
            elif sorted_list[i][0]=='O':
               print('O')
               send_data([0x01,0x0F],2)
            elif sorted_list[i][0]=='one':
               print('one')
               send_data([0x02,0x01],2) 
            elif sorted_list[i][0]=='two':
               print('two')
               send_data([0x02,0x02],2)
            elif sorted_list[i][0]=='three':
               print('three')
               send_data([0x02,0x03],2)

#2023年5月10日22:50:11
#直接用.decode来解码应该是可以的，之所以会出现乱码的情况应该是因为：openART通过uart串口与MCU通信，如果
#MCU此时断电或者重新上电，容易对uart通信产生不可预测的干扰，从而导致openMVIDE里报错（程序跑飞），正确的
#操作应该是最先上电MCU，然后在测试过程中不让mcu重启或者掉电。
#在调试时要特别注意上面这一点，在小车实际跑动的过程中，MCU是不会人为断电或者重启的，所以实际跑的时候可能
#不会出现这种情况
