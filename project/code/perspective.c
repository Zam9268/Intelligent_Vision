//#include "perspective.h"
//#include "stdio.h"

//#define A4_W 297
//#define A4_H 210 //定义出A4纸的长宽

///**
// * @brief 总钻风计算逆透视矩阵函数
// * @param uint8 image_pos[4][2]图像坐标的四个点  uint8 world_pos[4][2]显示坐标的四个点
// * @return 无
// */
//void calculate_matrix( uint8 image_pos[4][2],uint8 world_pos[4][2])
//{
//    uint8 A[8][8]={0};//由4个点的图像上的坐标生成，每个点生成两行数据
//    uint8 B[8]={0};
//    uint8 a[2][8]={0};//用于存储计算变换矩阵H所需的部分数据
//    uint8 j=0;//用于循环遍历B下标的变量
//    uint8 jj=0;
//    for(uint8 i=0;i<4;i++)
//    {
//        a[0][0] = image_pos[i][0];
//        a[0][1] = image_pos[i][1];
//        a[0][2] = 1;
//        a[0][3] = 0;
//        a[0][4] = 0;
//        a[0][5] = 0;
//        a[0][6] = -image_pos[i][0] * world_pos[i][0];
//        a[0][7] = -image_pos[i][1] * world_pos[i][0];

//        a[1][0] = 0;
//        a[1][1] = 0;
//        a[1][2] = 0;
//        a[1][3] = image_pos[i][0];
//        a[1][4] = image_pos[i][1];
//        a[1][5] = 1;
//        a[1][6] = -image_pos[i][0] * world_pos[i][1];
//        a[1][7] = -image_pos[i][1] * world_pos[i][1];

//        B[j][0]=world_pos[i][0];
//        j++;
//        B[j][0]=world_pos[i][1];
//        j++;//存储世界坐标

//        for(uint8 g=0;g<8;g++)
//        {
//            A[jj][g]=a[0][g];//存储对应的坐标
//        }
//        jj++;
//        for(uint8 g=0;g<8;g++)
//        {
//            A[jj][g]=a[1][g];
//        }
//        jj++;
//    }
//}
