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

def send_data(data):
   data_packet = []
   data_packet.append(0xB7) #发送包头，这里的append是增长数组的数据
   if isinstance(data, list): # 如果data是列表，将其元素添加到data_packet
       data_packet.extend(data)
   else:
       data_packet.append(data) #发送数据
   data_packet.append(0x98) #发送包尾
   uart.write(bytearray(data_packet))#发送数据
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

blob_threshold=(100, 40, -128, 14, -128, 127)#原来的反蓝色滤波，因为要检测，就暂时不用这个了
#阈值2(100, 40, -128, 14, -128, 127)
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

net_path = "mobilenet_v2-2024-03-03T08-27-26.947Z_in-int8_out-int8_channel_ptq.tflite"                                  # 定义模型的路径，这个模型由eiq进行提供，自己要训练
labels = [line.rstrip() for line in open("/sd/mobilenet_v2_total_labels.txt")]   # 加载标签
net = tf.load(net_path, load_to_fb=True)#new_path：预训练模型的文件路径 load_to_fb：模型会被加载到帧缓冲区

test_data=[0x10,0x20,0x30]


while(1):
    while(1):
        send_data([0x02,0x01])
        print("aaaaa")
    sensor.set_auto_whitebal(False)#关闭白平衡
    img = sensor.snapshot()
    #这个是通过色块来找图片
    for blobs in img.find_blobs([blob_threshold]):
        if blobs.h() < 70 or blobs.w() < 70:#当找到的色块大于一定值才会进行识别
            continue#小于的话会进行不断识别
        img = img.draw_rectangle(blobs.rect(),color = (255, 0, 0))    # 绘制矩形外框，便于在IDE上查看识别到的矩形位置，
        #img = img.draw_string(10,10, "%s = %f" % (sorted_list[i][0], sorted_list[i][1]),color=(255, 0, 0), scale=3)
        img1 = img.copy(1,1,blobs.rect())  # 拷贝矩形框内的图像，提高检测效率
        #lcd.show_image(img, 320, 240, zoom=2)#

        # 将矩形框内的图像使用训练好的模型进行分类
        # tf.classify()将在图像的roi上运行网络(如果没有指定roi，则在整个图像上运行)，整个图像运行会降低速度
        # 将为每个位置生成一个分类得分输出向量。
        # 在每个比例下，检测窗口都以x_overlap（0-1）和y_overlap（0-1）为指导在ROI中移动。
        # 如果将重叠设置为0.5，那么每个检测窗口将与前一个窗口重叠50%。
        # 请注意，重叠越多，计算工作量就越大。因为每搜索/滑动一次都会运行一下模型。
        # 最后，对于在网络沿x/y方向滑动后的多尺度匹配，检测窗口将由scale_mul（0-1）缩小到min_scale（0-1）。
        # 下降到min_scale(0-1)。例如，如果scale_mul为0.5，则检测窗口将缩小50%。
        # 请注意，如果x_overlap和y_overlap较小，则在较小的比例下可以搜索更多区域...

        # 默认设置只是进行一次检测...更改它们以搜索图像...
        for obj in tf.classify(net , img1, min_scale=1.0, scale_mul=0.5, x_overlap=0.0, y_overlap=0.0):
            #net：预训练模型,EIQ传入 img1：待检测的图像，min_scale：滑动窗口缩小的最小比例，这里设置为1.0，意味着滑动窗口大小不会改变
            #scale_mul：滑动窗口缩小的比例，这里设置为0.5，意味着滑动窗口每次缩小50%，但是因为min_scale=1.0，所以滑动窗口大小不会改变
            #x_overlap：滑动窗口在x方向的重叠率，这里设置为0.0，意味着滑动窗口在x方向上不重叠，同理y_overlap也是一样
            #print("**********\nTop 1 Detections at [x=%d,y=%d,w=%d,h=%d]" % obj.rect())
            sorted_list = sorted(zip(labels, obj.output()), key = lambda x: x[1], reverse = True)
            #zip函数：将多个可迭代对象组合成一个新的元组（列表），这里是将labels和obj.output组成一个元组
            #其中labels是标签,例如labels = ['bean', 'potato', 'peanut', 'rice', 'corn', 'cabbage']
            #而obj.output指的是对应的概率值，比如返回[0.1, 0.2, 0.1, 0.3, 0.2, 0.1]，则图片为'bean'的概率为0.1
            #最终zip迭代出来的对象为[
            #('bean', 0.1),
            #('potato', 0.2),
            #('peanut', 0.1),
            #('rice', 0.3),
            #('corn', 0.2),
            #('cabbage', 0.1)
            #key= lambda x:x[1]表示对zip后面的元组按照第二个元素进行排序，也就是按照概率值进行排序
            #而reverse = True表示对zip后面的元组进行降序排序，由大到小进行排序
            # 打印准确率最高的结果
            for i in range(1):#只运行一次，好像没有什么用，但是不要删，取出第一个元素值（也就是对应概率最高的元素），将这个类别和概率作为字符串显示
                #print("%s = %f" % (sorted_list[i][0], sorted_list[i][1]))#sorted_list[i][0]指的是标签内的名字
                img = img.draw_string(10,10, "%s=" % (sorted_list[i][0]),color=(255, 0, 0), scale=3)
                img = img.draw_string(10,50, "%f" % (sorted_list[i][1]),color=(255, 0, 0), scale=3)
                print("%s=%f",sorted_list[i][0],sorted_list[i][1])
                lcd.show_image(img, 320, 240, zoom=2)#zoom=2表示将图像放大2倍
                #time.sleep_ms(200)                                                         #sorted_list[i][1]指的是模型输出的概率值
            #对概率最高的进行匹配，选择最恰当的那一个进行发送数据
            if sorted_list[i][0]=='ambulance':
               print('ambulance')

            elif sorted_list[i][0]=='armoredcar':
               print('armoredcar')

            elif sorted_list[i][0]=='bulletproof':
               print('bulletproof')

            elif sorted_list[i][0]=='dagger':
               print('dagger')

            elif sorted_list[i][0]=='explosive':
               print('explosive')

            elif sorted_list[i][0]=='fire_axe':
               print('fire_axe')

            elif sorted_list[i][0]=='fire_engine':
               print('fire_engine')

            elif sorted_list[i][0]=='firearms':
               print('firearms')

            elif sorted_list[i][0]=='first_aid_kit':
               print('first_aid_kit')

            elif sorted_list[i][0]=='flashlight':
               print('flashlight')

            elif sorted_list[i][0]=='helmet':
               print('helmet')

            elif sorted_list[i][0]=='intercom':
               print('intercom')

            elif sorted_list[i][0]=='motorcycle':
               print('motorcycle')

            elif sorted_list[i][0]=='spontoon':
               print('spontoon')

            elif sorted_list[i][0]=='telescope':
               print('telescope')

#2023年5月10日22:50:11
#直接用.decode来解码应该是可以的，之所以会出现乱码的情况应该是因为：openART通过uart串口与MCU通信，如果
#MCU此时断电或者重新上电，容易对uart通信产生不可预测的干扰，从而导致openMVIDE里报错（程序跑飞），正确的
#操作应该是最先上电MCU，然后在测试过程中不让mcu重启或者掉电。
#在调试时要特别注意上面这一点，在小车实际跑动的过程中，MCU是不会人为断电或者重启的，所以实际跑的时候可能
#不会出现这种情况
