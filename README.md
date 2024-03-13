

2024/2/23   目标检测思路：把15分类卡片和ABC标识板放一起，然后对于路边的卡片，其距离离路中间较近，对于环岛和十字的卡片，可以确定其中心的位置（如果目标检测的现实坐标和透视后的现实坐标距离相近），进入环岛时要记录对应的5个现实坐标，还要求出中心的坐标，通过比较中心坐标和现实坐标的差值来判断是否为卡片堆
那就考虑用总钻风用逆透视。

视觉当前任务：尝试把ABC分类弄成全角度识别（不行的话再考虑其他）

2024/2/25   重新制作了15分类数据集和数字字母数据集，以及目标检测数据集，待验证
————————————————————第一周————————————————————————
2024/2/26   跑了目标检测模型，问题：1、不能同时检测多个图片 2、检测的图片会很歪     3、会有两个图片出现重合的现象
                            思考：可能是因为拍摄的角度问题，同时勾图片要勾全图，尽量不要选择一半
            跑了字母数组检测模型，发现存在误判问题，解决方法：增加黑边数据集；或者增加反黑色滤波

            解决了字母数组检测的问题，目前正确识别率挺高的
            目标检测由于没有车模，暂时无法检测
待解决的问题：目标检测视频数据集采集；15分类的倒转识别不够准确

2024/2/27   重新制作目标检测数据集，开始跑15分类模型

2024/3/1    重新制作15分类数据集

2024/3/2    1.跑出目标检测模型（正确率挺高的0.96），15分类模型（正确率也挺高的），不过遇到一个问题就是在i7上生成模型总是会报错，目前找不到原因，打算用自己的电脑来跑一次模型。明天就检测模型
            2.完成图像处理的基本函数（最长白列+误差处理+部分元素识别），还有逆透视没有加上去
            明天的工作：验证以上模型的正确性和可行性。
2024/3/3    1. 对于3/2的问题，解决方案是只能在自己的电脑上面跑模型，在实验室的电脑上面跑就不行（我也不知道为什么），然后重新制作数据集在自己电脑上面跑
            2.花了点时间裁剪一下打印的图片。然后晚上的时候跑完了模型，准确率还挺高的。明天验证。
————————————————————第二周————————————————————————
2024/3/4    1.验证了大概一百多张打印的数据集，实际验证的正确率挺高的（个人感觉能有98%），而且比前面训练的具有一个优点（可以从四个方向进行识别，不再是单一方向上的识别），但是还有待改进的地方（看能不能调成360°的识别，如果成功的话识别效率大大提高了）
            2.总钻风不知到为什么不能显示图像，留着明天弄

2024/3/6    尝试总钻风，发现scchar算法和大津法结合效果还挺好的，抗干扰能力还不错，但是如果直接用大津法的话就会出现卡死（目前任然不知道原因——应该是数组越界了！！！！）沃区忽然想明白了。（校赛也遇到过）

2024/3/7    1.发现是函数名字调用错了，改一下就好    2.数据集2.0制作完成，准备炼丹
            3.重写边缘检测的白列检测法

2024/3/9    1.加了一点视觉的边线处理的算法  2.完成Vofa的显示波形    3.pit定时器重新修复
            遇到的问题：车轮有个动不了了，明天看看是不是电机的问题    
2024/3/10   1.发现车的齿轮有点问题，打算拆了修一下（驱动有一个没输出）  2.修改完善control.c的代码

2024/3/13   车轮转速基本没太大误差，问题就是转化成速度的公式有点奇怪，在找问题。