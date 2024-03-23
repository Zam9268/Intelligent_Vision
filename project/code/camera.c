#include "camera.h"

/**
 * @brief 摄像头初始化
 *
 * @return uint8 初始化完成标志：1-完成，0-未完成
 */
uint8 Camera_Init(void)
{
    uint8 init_finishflag=0;//初始化完成标志位
    init_finishflag=mt9v03x_init();//摄像头初始化
    interrupt_global_enable(0);//开启总中断
    return init_finishflag;
}

/**
 * @brief 简单二值化（手动设置阈值），做了修改
 * @time_consuming 720us
 * @return uint8*，返回对应的地址
 */
void Simple_Binaryzation(uint8 *image0,uint8 threshold)
{
    uint16 temp=0;//中间变量
    // static uint8 simple_bina[IMAGE_HEIGHT][IMAGE_WIDTH];
    for(int i=0;i<IMAGE_HEIGHT;i++)
    {
        for(int j=0;j<IMAGE_WIDTH;j++)
        {
           temp=*(image0+i*IMAGE_WIDTH+j);//求出对应坐标值的灰度值
           if(temp<threshold)   image0[i*IMAGE_WIDTH+j]=0;
           else                 image0[i*IMAGE_WIDTH+j]=255;
        }
    }
	// return *simple_bina;
}

/**
 * @brief 均值二值化
 * @time_consuming：1100us
 * @param image1
 * @return uint8*
 
uint8 *Mean_Binaryzation(uint8 *image1)
{
    
    uint32 gray_sum=0;//灰度值总和
    uint16 temp1,temp2,meannum=0;//中间变量
    for(int i=0;i<IMAGE_HEIGHT;i++)
    {
        for(int j=0;j<IMAGE_WIDTH;j++)
        {
            gray_sum+=*(image1+i*IMAGE_WIDTH+j);//求出灰度值总和
        }
    }
    meannum=gray_sum/(IMAGE_HEIGHT*IMAGE_WIDTH);//求出灰度值平均值，也就是平均阈值
    for(int i=0;i<IMAGE_HEIGHT;i++)
    {
        for(int j=0;j<IMAGE_WIDTH;j++)
        {
            temp1=*(image1+i*IMAGE_WIDTH+j);//求出对应坐标值的灰度值
            if(temp1<meannum)   Image_Use[i][j]=0;
            else                Image_Use[i][j]=255;
        }
    }
    for(int i=0;i<IMAGE_HEIGHT;i++)
    {
        for(int j=0;j<IMAGE_WIDTH;j++)
        {
            temp2=*(image1+i*IMAGE_WIDTH+j);//求出对应坐标值的灰度值
            if(temp2<meannum)   Image_Use[i][j]=0;
            else                Image_Use[i][j]=255;
        }
    }
    return (uint8 *)Image_Use;
}
*/

/**
 * @brief 高斯模糊
 * @time_consuming：5600us（数组索引表示）/4325us（指针表示）
 * @param image2
 * @return uint8*
 */
uint8 *Gaussian_Blur(uint8 *image2)
{
    static uint8 gauss_blur[IMAGE_HEIGHT][IMAGE_WIDTH];//高斯模糊图像
    float Gaussian_Blur[9]={0.0947416, 0.118318, 0.0947416, 0.118318, 0.147761, 0.118318, 0.0947416, 0.118318, 0.0947416};
    float fix_sum=0;//加权均值为0
    uint8 temp_fix[9];//3*3卷积核
    for(int j=0;j<IMAGE_WIDTH;++j)//由于第一行不作处理，故选择++j
    {
        gauss_blur[0][j]=*(image2);
        image2++;//地址自增
    }
    for(int i=1;i<IMAGE_HEIGHT-1;++i)
    {
        gauss_blur[i][0]=*(image2);//第一列不做处理
        image2++;//地址自增
        for(int j=1;j<IMAGE_WIDTH-1;++j)//各个像素点的赋值
        {
            temp_fix[0]=*(image2-IMAGE_WIDTH-1);
            temp_fix[1]=*(image2-IMAGE_WIDTH);
            temp_fix[2]=*(image2-IMAGE_WIDTH+1);
            temp_fix[3]=*(image2-1);
            temp_fix[4]=*(image2);
            temp_fix[5]=*(image2+1);
            temp_fix[6]=*(image2+IMAGE_WIDTH-1);
            temp_fix[7]=*(image2+IMAGE_WIDTH);
            temp_fix[8]=*(image2+IMAGE_WIDTH+1);
            for(int k=0;k<9;++k)//加权均值
            {
                fix_sum+=temp_fix[k]*Gaussian_Blur[k];
            }
            gauss_blur[i][j]=fix_sum;
            fix_sum=0;
            image2++;//地址自增
        }
        gauss_blur[i][IMAGE_WIDTH-1]= *(image2);//最后一列不做处理
        image2++;//地址自增
    }
    for(int j=0;j<IMAGE_WIDTH;++j)//最后一行不做处理
    {
        gauss_blur[IMAGE_HEIGHT-1][j]=*(image2);
        image2++;//地址自增
    }
    return *gauss_blur;
}

/**
 * @brief Sobel算子边缘检测
 * @time_consuming：4350us(InvSqrt,很稳定)/4200us(sqrt,波动大，最高4380，最低3890)
 * @param image4
 * @return uint8*，注意返回的图像仍然为灰度图像，要和Canny算法相结合（也可以和大津法相结合，和大津法相结合效果还挺好？）
 */

uint8 *Sobel_Edge(uint8 *image4)
{
    static uint8 sobel_image[IMAGE_HEIGHT][IMAGE_WIDTH];//Sobel边缘检测图像数组
    int Gx,Gy,G;//定义梯度的值
    uint8 soble_fix[9];//3*3卷积核
    for(int j=0;j<IMAGE_WIDTH;++j)//由于第一行不作处理，故选择++j
    {
        sobel_image[0][j]=*(image4);
        image4++;//地址自增
    }
    for(int i=1;i<IMAGE_WIDTH-1;i++)//第一列不作处理
    {
        sobel_image[i][0]=*(image4);
        image4++;//地址自增
        for(int j=1;j<IMAGE_WIDTH-1;j++)//各个像素点的赋值
        {
            soble_fix[0]=*(image4-IMAGE_WIDTH-1);
            soble_fix[1]=*(image4-IMAGE_WIDTH);
            soble_fix[2]=*(image4-IMAGE_WIDTH+1);
            soble_fix[3]=*(image4-1);
            soble_fix[4]=*(image4);
            soble_fix[5]=*(image4+1);
            soble_fix[6]=*(image4+IMAGE_WIDTH-1);
            soble_fix[7]=*(image4+IMAGE_WIDTH);
            soble_fix[8]=*(image4+IMAGE_WIDTH+1);
            Gx=(soble_fix[2]+2*soble_fix[5]+soble_fix[8])-(soble_fix[0]+2*soble_fix[3]+soble_fix[6]);//x方向梯度
            Gy=(soble_fix[0]+2*soble_fix[1]+soble_fix[2])-(soble_fix[6]+2*soble_fix[7]+soble_fix[8]);//y方向梯度
            G=InvSqrt(Gx*Gx+Gy*Gy);//梯度
            sobel_image[i][j]=G;
            image4++;//地址自增
        }
        sobel_image[i][IMAGE_WIDTH-1]=*(image4);//最后一列不做处理
        image4++;//地址自增
    }
    for(int j=0;j<IMAGE_WIDTH;j++)
    {
        sobel_image[IMAGE_HEIGHT-1][j]=*(image4);
        image4++;//地址自增
    }
	return *sobel_image;
}

/**
 * @brief Scharr算子（Sobel算子改进版，边缘更明显）
 * @time_consuming：3760us(InvSqrt,很稳定)/3400us(sqrt,波动大，最高3680，最低3300)
 * @param image5
 * @return uint8*
 */
uint8 *Scharr_Edge(uint8 *image5)
{
    static uint8 scharr_image[IMAGE_HEIGHT][IMAGE_WIDTH];//Scharr边缘检测图像数组
    int Gx,Gy,G;//定义梯度的值
    uint8 scharr_fix[9];//3*3卷积核
    for(int j=0;j<IMAGE_WIDTH;j++)
    {
        scharr_image[0][j]=*(image5);
        image5++;//地址自增
    }
    for(int i=1;i<IMAGE_HEIGHT-1;i++)
    {
        scharr_image[i][0]=*(image5);//第一列不作处理
        image5++;//地址自增
        for(int j=1;j<IMAGE_WIDTH-1;j++)
        {
            scharr_fix[0]=*(image5-IMAGE_WIDTH-1);
            scharr_fix[1]=*(image5-IMAGE_WIDTH);
            scharr_fix[2]=*(image5-IMAGE_WIDTH+1);
            scharr_fix[3]=*(image5-1);
            scharr_fix[4]=*(image5);
            scharr_fix[5]=*(image5+1);
            scharr_fix[6]=*(image5+IMAGE_WIDTH-1);
            scharr_fix[7]=*(image5+IMAGE_WIDTH);
            scharr_fix[8]=*(image5+IMAGE_WIDTH+1);
            Gx=(3*scharr_fix[2]+10*scharr_fix[5]+3*scharr_fix[8])-(3*scharr_fix[0]+10*scharr_fix[3]+3*scharr_fix[6]);//x方向梯度
            Gy=(3*scharr_fix[0]+10*scharr_fix[1]+3*scharr_fix[2])-(3*scharr_fix[6]+10*scharr_fix[7]+3*scharr_fix[8]);//y方向梯度
            G=InvSqrt(Gx*Gx+Gy*Gy);//梯度
            G=G>255?255:G;//限幅
            scharr_image[i][j]=G;
            image5++;//地址自增
        }
        scharr_image[i][IMAGE_WIDTH-1]=*(image5);//最后一列不做处理
        image5++;//地址自增
    }
    for(int j=0;j<IMAGE_WIDTH;j++)
    {
        scharr_image[IMAGE_HEIGHT-1][j]=*(image5);
        image5++;//地址自增
    }
    return *scharr_image;
}

/**
 * @brief 非极大值抑制
 * @time_consuming：8100us(包括Sobel)/8200us(包括Scharr)
 * @param image6
 * @param choose_edge,0为Sobel,1为Scharr
 * @return uint8*
 */
uint8 *NMS(uint8 *image6,uint8 choose_edge)
{
    static uint8 NMS_image[IMAGE_HEIGHT][IMAGE_WIDTH];//非极大值抑制图像数组
    uint8 edge_image[IMAGE_HEIGHT][IMAGE_WIDTH];//边缘图像数组
    uint8 edge_fix[9];//3*3卷积核
    int Gx,Gy,G=0;//定义梯度的值
    uint8 dir[IMAGE_HEIGHT][IMAGE_WIDTH];//定义各个方向的梯度
    const uint8 NMS_LOSS =2;//非极大值抑制补偿
    //边缘算法
    for(int j=0;j<IMAGE_WIDTH;j++)//第一行不处理
    {
        edge_image[0][j]=*(image6);
        image6++;//地址自增
    }
    for(int i=1;i<IMAGE_HEIGHT-1;i++)
    {
        edge_image[i][0]=*(image6);//第一列不处理
        image6++;//地址自增
        for(int j=1;j<IMAGE_WIDTH-1;j++)
        {
            edge_fix[0]=*(image6-IMAGE_WIDTH-1);
            edge_fix[1]=*(image6-IMAGE_WIDTH);
            edge_fix[2]=*(image6-IMAGE_WIDTH+1);
            edge_fix[3]=*(image6-1);
            edge_fix[4]=*(image6);
            edge_fix[5]=*(image6+1);
            edge_fix[6]=*(image6+IMAGE_WIDTH-1);
            edge_fix[7]=*(image6+IMAGE_WIDTH);
            edge_fix[8]=*(image6+IMAGE_WIDTH+1);
            if (choose_edge == 0) // Sobel
            {
                Gx = edge_fix[2] - edge_fix[0] + 2 * edge_fix[5] - 2 * edge_fix[3] + edge_fix[8] - edge_fix[6];
                Gy = edge_fix[0] - edge_fix[6] + 2 * edge_fix[1] - 2 * edge_fix[7] + edge_fix[2] - edge_fix[8];
            }
            else if (choose_edge == 1) // Scharr
            {
                Gx = 3 * edge_fix[2] - 3 * edge_fix[0] + 10 * edge_fix[5] - 10 * edge_fix[3] + 3 * edge_fix[8] - 3 * edge_fix[6];
                Gy = 3 * edge_fix[0] - 3 * edge_fix[6] + 10 * edge_fix[1] - 10 * edge_fix[7] + 3 * edge_fix[2] - 3 * edge_fix[8];
            }
            G=(int)InvSqrt(Gx*Gx+Gy*Gy);//开方
            G=G>255?255:G;//限幅
            edge_image[i][j]=G;//更新边缘图像
            dir[i][j]=Atan2(Gy,Gx);//用到了Atan2算法，dir表示像素梯度的方向，会在NMS()函数中用到
            image6++;
        }
        edge_image[i][IMAGE_WIDTH-1]=*(image6);//最后一列不处理
        image6++;//地址自增
    }
    for(int j=0;j<IMAGE_WIDTH;j++)//最后一行不处理
    {
        edge_image[IMAGE_HEIGHT-1][j]=*(image6);
        image6++;//地址自增
    }
    //非极大值抑制
    for(int j=0;j<IMAGE_WIDTH;j++)// 第一行和最后一行不处理
    {
        NMS_image[0][j]=edge_image[0][j];
        NMS_image[IMAGE_HEIGHT-1][j]=edge_image[IMAGE_HEIGHT-1][j];
    }
    for (int i = 1; i < IMAGE_HEIGHT - 1; i++) // 第一列和最后一列不处理
    {
        NMS_image[i][0] = edge_image[i][0];
        NMS_image[i][IMAGE_WIDTH - 1] = edge_image[i][IMAGE_WIDTH - 1];
    }
    for(int i=1;i<IMAGE_HEIGHT-1;i--)
    {
        for(int j=1;i<IMAGE_WIDTH-1;j++)
        {
            switch(dir[i][j])
            {
                case 0:
                {
                    if((edge_image[i][j]>edge_image[i][j+1]-NMS_LOSS)&&(edge_image[i][j]>edge_image[i][j-1]-NMS_LOSS))
                    {
                        NMS_image[i][j]=edge_image[i][j];
                    }
                    else
                    {
                        NMS_image[i][j]=0;
                    }
                }break;
                case 1:
                {
                    if((edge_image[i][j]>edge_image[i+1][j-1]-NMS_LOSS)&&(edge_image[i][j]>edge_image[i-1][j+1]-NMS_LOSS))
                    {
                        NMS_image[i][j]=edge_image[i][j];
                    }
                    else
                    {
                        NMS_image[i][j]=0;
                    }
                }break;
                case 2:
                {
                    if ((edge_image[i][j] > edge_image[i - 1][j - 1] - NMS_LOSS) && (edge_image[i][j] > edge_image[i + 1][j + 1] - NMS_LOSS))
                        NMS_image[i][j] = edge_image[i][j];
                    else
                        NMS_image[i][j] = 0;
                }break;
                case 3:
                {
                    if ((edge_image[i][j] > edge_image[i + 1][j] - NMS_LOSS) && (edge_image[i][j] > edge_image[i - 1][j] - NMS_LOSS))
                        NMS_image[i][j] = edge_image[i][j];
                    else
                        NMS_image[i][j] = 0;
                }break;
                default:    break;     
            }
        }

    }
    return *NMS_image;//返回图像
}

/**
 * @brief 双边缘阈值链接
 * 用时：1000us
 * image_debug = TwoThreshold(NMS(*mt9v03x_image, 1));9100us
 * @param image7
 * @return uint8*
 */
void *TwoThreshold(uint8 *image7)
{
    static uint8 Link_image[IMAGE_HEIGHT][IMAGE_WIDTH];//双边缘阈值链接图像数组
    uint8 Link_fix[8];//读取某个点的相邻8个像素点
    uint8 lowThr,highThr;//低阈值和高阈值
    highThr=230;//高阈值
    lowThr=highThr/2;//高阈值等于低阈值的一半
    image7= image7+IMAGE_WIDTH;//从第二行首列开始，这里算的是地址
    for(int i=1;i<IMAGE_HEIGHT-1;i++)
    {
        image7++;
        for(int j=1;j<IMAGE_WIDTH-1;j++)
        {
            if(*image7<lowThr)  Link_image[i][j]=0;
            else if(*image7>highThr)    Link_image[i][j]=255;
            else
            {
                Link_fix[0]=*(image7-IMAGE_WIDTH-1);
                Link_fix[1]=*(image7-IMAGE_WIDTH);
                Link_fix[2]=*(image7-IMAGE_WIDTH+1);
                Link_fix[3]=*(image7-1);
                Link_fix[4]=*(image7+1);
                Link_fix[5]=*(image7+IMAGE_WIDTH-1);
                Link_fix[6]=*(image7+IMAGE_WIDTH);
                Link_fix[7]=*(image7+IMAGE_WIDTH+1);

                quick_sort(Link_fix,0,8);//快速排序，对八邻域的灰度值进行排序，7最大，0最小
                if(Link_fix[4]>highThr) Link_image[i][j]=255;//满足条件，链接
                else Link_image[i][j]=0;//不满足，则灰度值置为0
            }
            image7++;//地址自增
        }
    }
    return *Link_image;//返回图像
}

/**
 * @brief OSTU大津法求最佳阈值
 * 用时：2000us(不稳定)
 * @param tmImage
 * @return uint8
 */
uint8 Camera_GetOSTU(uint8 *tmImage)
{
    signed short i, j;
    unsigned long Amount = 0;
    unsigned long PixelBack = 0;
    unsigned long PixelIntegralBack = 0;
    unsigned long PixelIntegral = 0;
    signed long PixelIntegralFore = 0;
    signed long PixelFore = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    signed short MinValue, MaxValue;
    uint8 Threshold = 0;
    unsigned short HistoGram[256]; //

    for (j = 0; j < 256; j++)
        HistoGram[j] = 0; //初始化灰度直方图

    for (j = 0; j < IMAGE_HEIGHT; j++)
    {
        for (i = 0; i < IMAGE_WIDTH; i++)
        {
            HistoGram[*(tmImage + j * IMAGE_HEIGHT + i)]++; //统计灰度级中每个像素在整幅图像中的个数
        }
    }
    for (MinValue = 0; MinValue < 256 && HistoGram[MinValue] == 0; MinValue++); //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistoGram[MaxValue] == 0; MaxValue--); //获取最大灰度的值
    if (MaxValue == MinValue)   return MaxValue; // 图像中只有一个颜色
    if (MinValue + 1 == MaxValue)   return MinValue; // 图像中只有二个颜色
    for (j = MinValue; j <= MaxValue; j++)
    {
        Amount += HistoGram[j]; //  像素总数
    }
    PixelIntegral = 0;
    for (j = MinValue; j <= MaxValue; j++)
    {
        PixelIntegral += HistoGram[j] * j; //灰度值总数
    }
    SigmaB = -1;
    for (j = MinValue; j < MaxValue; j++)
    {
        PixelBack = PixelBack + HistoGram[j];                                              //前景像素点数
        PixelFore = Amount - PixelBack;                                                    //背景像素点数
        OmegaBack = (double)PixelBack / Amount;                                            //前景像素百分比
        OmegaFore = (double)PixelFore / Amount;                                            //背景像素百分比
        PixelIntegralBack += HistoGram[j] * j;                                             //前景灰度值
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;                             //背景灰度值
        MicroBack = (double)PixelIntegralBack / PixelBack;                                 //前景灰度百分比
        MicroFore = (double)PixelIntegralFore / PixelFore;                                 //背景灰度百分比
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore); //计算类间方差
        if (Sigma > SigmaB)                                                                //遍历最大的类间方差g //找出最大类间方差以及对应的阈值
        {
            SigmaB = Sigma;
            Threshold = j;
        }
    }
    return Threshold; //返回最佳阈值;
}

/**
 * @brief 图像裁剪-Height(待改进)
 *
 * @param image8
 * @param height
 * @param width
 * @return uint8*
 */
uint8 *CutImageH(uint8 *image8, uint16 height, uint16 width)
{
    static uint8 cuth_image[IMAGE_HEIGHT][IMAGE_WIDTH]; // 裁剪h后的图像
    uint16 error_h = 0;                          // 目标图像与实际图像的偏差的绝对值

    if (IMAGE_HEIGHT > height) // 目标高度大于实际高度
    {
        error_h = IMAGE_HEIGHT - height;
        for (int temp = 0, i = 0; i < IMAGE_HEIGHT; i++, temp++)
        {
            for (int j = 0; j < width; j++)
            {
                cuth_image[i][j] = *(image8);
                image8++;
            }

            if ((temp + 1) % (height / error_h) == 0)
            {
                i += 1;
                for (int j = 0; j < width; j++)
                {
                    cuth_image[i][j] = cuth_image[i - 1][j];
                }
            }
        }
    }

    else if (IMAGE_HEIGHT < height) // 目标高度小于实际高度
    {
        error_h = height - IMAGE_HEIGHT;
        for (int temp = 0, i = 0; i < IMAGE_HEIGHT; i++, temp++)
        {
            if ((temp + 1) % (height / error_h) == 0)
            {
                temp += 1;
                for (int j = 0; j < width; j++)
                {
                    image8++;
                }
            }
            for (int j = 0; j < width; j++)
            {
                cuth_image[i][j] = *(image8);
                image8++;
            }
        }
    }

    else if (IMAGE_HEIGHT == height) // 目标高度等于实际高度
    {
        return image8;
    }

    return *cuth_image;
}

/**
 * @brief 图像裁剪-Width(待改进)
 *
 * @param image9
 * @param height
 * @param width
 * @return uint8*
 */
uint8 *CutImageW(uint8 *image9, uint16 height, uint16 width)
{
    static uint8 cutw_image[IMAGE_HEIGHT][IMAGE_WIDTH]; // 裁剪h后的图像
    uint16 error_w = 0;                          // 目标图像与实际图像的偏差的绝对值

    if (IMAGE_WIDTH > width) // 目标宽度大于实际宽度
    {
        error_w = IMAGE_WIDTH - width;
        for (int i = 0; i < IMAGE_HEIGHT; i++)
        {
            for (int temp = 0, j = 0; j < IMAGE_WIDTH; j++, temp++)
            {
                cutw_image[i][j] = *(image9);
                image9++;
                if ((temp + 1) % (width / error_w) == 0)
                {
                    j++;
                    cutw_image[i][j] = cutw_image[i][j - 1];
                }
                // printf(" %3d ",cutw_image[i][j]);
            }
        }
    }

    else if (IMAGE_WIDTH < width) // 目标宽度小于实际宽度
    {
        error_w = width - IMAGE_WIDTH;
        for (int i = 0; i < IMAGE_HEIGHT; i++)
        {
            for (int temp = 0, j = 0; j < IMAGE_WIDTH; j++, temp++)
            {
                if ((temp + 1) % (width / error_w) == 0)
                {
                    temp += 1;
                    image9++;
                }
                cutw_image[i][j] = *(image9);
                image9++;
                // printf(" %3d ",cutw_image[i][j]);
            }
        }
    }

    else if (IMAGE_WIDTH == width)
    {
        return image9;
    }

    return *cutw_image;
}
