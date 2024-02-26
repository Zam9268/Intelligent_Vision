#放一段我们比赛前写的代码，最新的找不到了哈哈
#地图识别部分没有什么参考价值，通信部分和找色块分类图片可以参考一下。
#我们当时art没有做太复杂的任务，所以代码量比较小
#写于2023年11月27日
#思路总结：传统赛道（没有遇到环岛十字，就yolo目标检测图片+路边的识别分类检测）；如果是特殊赛道就是yolo目标先检测环岛中心图片位置+然后yolo检测环岛周围黑框图片+环岛周边的ABCDE识别分类检测
# 导入需要的库

# IO口接线：接TFT液晶显示屏的
# 蓝线：GND->GND   黄线：VCC->VCC     灰线：SCL->B0      黑线：SDA->B1      紫线：RES->B12     绿色：DC->B13  橙色：CS->B3   白色：BL->B16
import seekfree, pyb
import sensor, image, time, math
import ustruct
import os, tf
from pyb import LED
from machine import UART

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

blob_threshold=(31, 100, -128, 127, -2, 127)#原来的反蓝色滤波，因为要检测，就暂时不用这个了
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

net_path = "mobilenet_v2_ABC.tflite"                                  # 定义模型的路径，这个模型由eiq进行提供，自己要训练
labels = [line.rstrip() for line in open("/sd/mobilenet_v2_ABC_labels.txt")]   # 加载标签
net = tf.load(net_path, load_to_fb=True)#new_path：预训练模型的文件路径 load_to_fb：模型会被加载到帧缓冲区

while(1):

    sensor.set_auto_whitebal(False)#关闭白平衡
    img = sensor.snapshot()
    #这个是通过色块来找图片
    for blobs in img.find_blobs([blob_threshold]):
        if blobs.h() < 70 or blobs.w() < 70:#当找到的色块大于一定值才会进行识别
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
            elif sorted_list[i][0]=='B':
               print('B')
            elif sorted_list[i][0]=='C':
               print('C')
            elif sorted_list[i][0]=='D':
               print('D')
            elif sorted_list[i][0]=='E':
               print('E')
            elif sorted_list[i][0]=='F':
               print('F')
            elif sorted_list[i][0]=='G':
               print('G')
            elif sorted_list[i][0]=='H':
               print('H')
            elif sorted_list[i][0]=='I':
               print('I')
            elif sorted_list[i][0]=='J':
               print('J')
            elif sorted_list[i][0]=='K':
               print('K')
            elif sorted_list[i][0]=='L':
               print('L')
            elif sorted_list[i][0]=='M':
               print('M')
            elif sorted_list[i][0]=='N':
               print('N')
            elif sorted_list[i][0]=='O':
               print('O')
            elif sorted_list[i][0]=='one':
                print('one')
            elif sorted_list[i][0]=='two':
                print('two')
            elif sorted_list[i][0]=='three':
                print('three')

#2023年5月10日22:50:11
#直接用.decode来解码应该是可以的，之所以会出现乱码的情况应该是因为：openART通过uart串口与MCU通信，如果
#MCU此时断电或者重新上电，容易对uart通信产生不可预测的干扰，从而导致openMVIDE里报错（程序跑飞），正确的
#操作应该是最先上电MCU，然后在测试过程中不让mcu重启或者掉电。
#在调试时要特别注意上面这一点，在小车实际跑动的过程中，MCU是不会人为断电或者重启的，所以实际跑的时候可能
#不会出现这种情况
