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
 * @brief 简单二值化（手动设置阈值）
 * @time_consuming 720us
 * @return uint8*，返回对应的地址
 */
uint8 *Simple_Binaryzation(uint8 *image0,uint8 threshold)
{
    uint16 temp=0;//中间变量
    static uint8 simple_bina[IMAGE_HEIGHT][IMAGE_WIDTH];
    for(int i=0;i<IMAGE_HEIGHT;i++)
    {
        for(int j=0;j<IMAGE_WIDTH;j++)
        {
           temp=*(image0+i*IMAGE_WIDTH+j);//求出对应坐标值的灰度值
           if(temp<threshold)   Image_Use[i][j]=0;
           else                 Image_Use[i][j]=255;
        }
    }
}

/**
 * @brief 均值二值化
 * @time_consuming：1100us
 * @param image1
 * @return uint8*
 */
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
    for(int j=0;i<IMAGE_WIDTH;++j)//由于第一行不作处理，故选择++j
    {
        gauss_blur[0][j]=*(iamge2);
        image2++;//地址自增
    }
    for(int i=1;i<IMAGE_HEIGHT-1;++i)
    {
        gauss_blur[i][0]=*(iamge2);//第一列不做处理
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
    for(int j=0;i<IMAGE_WIDTH;++j)//最后一行不做处理
    {
        gauss_blur[IMAGE_HEIGHT-1][j]=*(iamge2);
        image2++;//地址自增
    }
    return *gauss_blur;
}

/**
 * @brief Sobel算子边缘检测
 * @time_consuming：4350us(InvSqrt,很稳定)/4200us(sqrt,波动大，最高4380，最低3890)
 * @param image4
 * @return uint8*
 */

uint8 *Sobel_Edge(uint8 *image4)
{
    static uint8 sobel_image[IMAGE_HEIGHT][IMAGE_WIDTH];//Sobel边缘检测图像数组
    int Gx,Gy,G;//定义梯度的值
    uint8 soble_fix[9];//3*3卷积核
    for(int j=0;i<IMAGE_WIDTH;++j)//由于第一行不作处理，故选择++j
    {
        sobel_image[0][j]=*(iamge4);
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
        sobel_image[IMAGE_HEIGHT-1][j]=*(iamge4);
        image4++;//地址自增
    }
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
        scharr_image[0][j]=*(iamge5);
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
        scharr_image[IMAGE_HEIGHT-1][j]=*(iamge5);
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
        edge_image[0][j]=*(iamge6);
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
            dir[i][j]=
        }
    }
}
