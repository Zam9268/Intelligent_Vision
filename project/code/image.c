#include "image.h"

uint8 Image_Use[IMAGE_HEIGHT][IMAGE_WIDTH];


/*边线数组变量定义*/
uint8 left_line[IMAGE_HEIGHT],right_line[IMAGE_HEIGHT];//左右边线数组
int center[IMAGE_HEIGHT];//中线数组
uint8 the_maxlen_position;//最长赛道位??
uint8 num;//定义有效行数
uint8 Longest_White_Column_Left[2];//从左到右最长白列（白列长度和所在列数）
uint8 Longest_White_Column_Right[2];//从右到左最长白??
uint8 Right_Lost_Flag[IMAGE_WIDTH];//右边界丢线标??
uint8 Left_Lost_Flag[IMAGE_WIDTH];//左边界丢线标??
uint8 Left_Lost_Time=0;//定义左丢线数
uint8 Right_Lost_Time=0;//定义右边界丢线数
uint8 Both_Lost_Time=0;//定义两个边界同时丢线??
uint8 Search_Stop_Line;//搜索终止??
uint8 Boundry_Start_Left,Boundry_Start_Right;//左右边界起始??
uint8 Road_Wide[IMAGE_HEIGHT];//定义赛道宽度数组
RoadType Road_Type;//定义赛道类型
float Left_derivative[IMAGE_HEIGHT]={0.0};
float Right_derivative[IMAGE_HEIGHT]={0.0};

//加权控制数组，可以利用该数组来调节前瞻
const uint8 Weight[IMAGE_HEIGHT]=
{
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 图像最远端00 —??09 行权??
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 图像最远端10 —??19 行权??
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // 图像最远端20 —??29 行权??
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,        // 图像最远端30 —??39 行权??
    1, 1, 1, 1, 1, 1, 1, 3, 4, 5,        // 图像最远端40 —??49 行权??
    6, 7, 9, 11, 13, 15, 17, 19, 20, 20, // 图像最远端50 —??59 行权??
    19, 17, 15, 13, 11, 9, 7, 5, 3, 1,   // 图像最远端60 —??69 行权??
};

/**
 * @brief 数组赋值
 * @param 无
 * @return 无
 */
void Image_Change(void)
{
    for (int i = 0; i < IMAGE_HEIGHT; i++)
    {
        for (int j = 0; j < IMAGE_WIDTH; j++)
        {
            Image_Use[i][j] = mt9v03x_image[i][j];
        }
    }
}
volatile int White_Column[IMAGE_WIDTH];//每列白列长度
/**
 * @brief 中线处理（这里用的大津法的最长白列法——sobel,canny的最长白列法还需要补充）
 * @param H（赛道长度）
 * @return ??
 */
void Center_line_deal(uint8 start_column,uint8 end_column)
{
    for(uint8 i=0;i<IMAGE_HEIGHT-1;i++)
	{
		left_line[i]=0;
		right_line[i]=0;
	}
    int x=0,y=0;//设x为行,y为列
    uint8 middle=the_maxlen_position;//定义赛道最长位??
    uint8 x_num;
    /*寻找最长白列（待优化） */
    for(uint8 j=start_column;j<end_column;j++)
    {
        for(uint8 i=IMAGE_HEIGHT-1;i>0;i--)
        {
            if(Image_Use[i][j]==BLACK_POINT)
            {
                break;
            }
            else
            {
                White_Column[j]++;
            }
        }
    }
    /*从左到右寻找最长白??*/
    Longest_White_Column_Left[0]=0;//白列长度清零
    for(uint8 i=start_column;i<end_column;i++)
    {
        if(White_Column[i]>Longest_White_Column_Left[0])//最大值更??
        {
            Longest_White_Column_Left[0]=White_Column[i];
            Longest_White_Column_Left[1]=i;
        }
    }
    /*从右到左寻找最长白??*/
    Longest_White_Column_Right[0]=0;//白列长度清零
    for(uint8 i=end_column;i>start_column;i--)
    {
        if(White_Column[i]>Longest_White_Column_Right[0])//最大值更??
        {
            Longest_White_Column_Right[0]=White_Column[i];
            Longest_White_Column_Right[1]=i;//最长列的长度和所在列数更??
        }
    }
    /*终止行赋??*/
    Search_Stop_Line=Longest_White_Column_Left[0];//搜索截止行的赋??
    int right_border,left_border;//定义边界中间变量
    for(int i=IMAGE_HEIGHT-1;i>=0;i--)
    {
        /*先寻右边??*/
        for(int j=Longest_White_Column_Left[1];j<=Longest_White_Column_Right[1];j++)
        {
            if(Image_Use[i][j]==WHITE_POINT&&Image_Use[i][j+1]==BLACK_POINT&&Image_Use[i][j+2]==BLACK_POINT)
            {
                right_border=j;//记录当前边界??
                Right_Lost_Flag[i]=0;//没有丢线，就??0
                break;
            }
            else if(j>=IMAGE_HEIGHT-1-2)//没有找到右边界时，就把最右边界赋值给右边，然后丢线标志位??1
            {
                right_border=j;
                Right_Lost_Flag[i]=1;
                break;
            }
        }
        for(uint8 j=Longest_White_Column_Left[1];j>=2;j--)
        {
            if(Image_Use[i][j]==WHITE_POINT&&Image_Use[i][j-1]==BLACK_POINT&&Image_Use[i][j-2]==BLACK_POINT)
            {
                left_border=j;//记录当前边界??
                Left_Lost_Flag[i]=0;//没有丢线，就??0
                break;
            }
            else if(j<=2)//没有找到左边界时，就把最左边界赋值给左边，然后丢线标志位??1
            {
                left_border=j;
                Left_Lost_Flag[i]=1;
                break;
            }
        }
        left_line[i]=left_border;//赋值给左边界数??
        right_line[i]=right_border;//赋值给右边界数??
    }
}

/**
 * @brief 中线处理（边缘检测的最长白列巡线，自己写的，可能有bug），要去除一些噪??
 * @param H（赛道长度）
 * @return ??
 */
void Center_line_deal_plus(uint8 start_column,uint8 end_column)
{
    /*边界数组清零*/
    for(uint8 i=0;i<IMAGE_HEIGHT;i++)
    {
        left_line[i]=0;
        right_line[i]=0;
    }
    /*最长白列计??*/
    for(uint8 j=start_column;j<end_column;j++)
    {
        for(uint8 i=IMAGE_HEIGHT-1;i>0;i--)
        {
            if(Image_Use[i][j]==BLACK_POINT)
            {
                White_Column[j]++;
            }//当遇到白色边缘的时候就退出检??
            else    break;
        }
    }
    /*从左到右寻找最长白??*/
    Longest_White_Column_Left[0]=0;//白列长度清零
    for(uint8 i=start_column;i<end_column;i++)
    {
        if(White_Column[i]>Longest_White_Column_Left[0])//最大值更??
        {
            Longest_White_Column_Left[0]=White_Column[i];//对应的白列点的数??
            Longest_White_Column_Left[1]=i;//对应最长白列的坐标
        }
    }
    /*边线数组赋??*/
    int right_border,left_border;//定义边界列坐标值中间变??
    for(int i=IMAGE_HEIGHT-1;i>=0;i--)
    {
        for(int j=Longest_White_Column_Left[1];j>=2;j--)//从左到右开始扫??
        {
            if(Image_Use[i][j]==BLACK_POINT&&Image_Use[i][j-1]==WHITE_POINT&&Image_Use[i][j-2]==WHITE_POINT)
            {
                left_border=j;//记录当前边界??
                Left_Lost_Flag[i]=0;//没有丢线，就??0
                break;
            }
            else if(j<=2)//如果在最左点的时候没有遇到白色的跳变点，就会默认找不到边??
            {
                left_border=j;//记录当前边界的坐??
                Left_Lost_Flag[i]=1;//丢线了，就会直接??0
                break;
            }
        }
        for(int j=Longest_White_Column_Left[1];j<=IMAGE_WIDTH-3;j++)
        {
            if(Image_Use[i][j]==BLACK_POINT&&Image_Use[i][j+1]==WHITE_POINT&&Image_Use[i][j+2]==WHITE_POINT)
            {
                right_border=j;//记录当前边界??
                Right_Lost_Flag[i]=0;//没有丢线，就??0
                break;
            }
            else if(j>=IMAGE_HEIGHT-1-2)//如果在最右点的时候没有遇到白色的跳变点，就会默认找不到边??
            {
                right_border=j;//记录当前边界的坐??
                Right_Lost_Flag[i]=1;//丢线了，就会直接??0
                break;
            }
        }
        left_line[i]=left_border;//赋值给左边界数??
        right_line[i]=right_border;//赋值给右边界数??
    }
}
/**
 * @brief 边线数组分析（后期会加上逆透视，目前只是提取原始的边线数组??
 * @param ??
 * @return ??
 */
void Outer_Analyse(void)
{
    /*剩下条件补充*/
    for(uint8 i=IMAGE_HEIGHT-1;i>=0;i--)
    {
        if(Left_Lost_Flag[i]==1)    Left_Lost_Time++;
        if(Right_Lost_Flag[i]==1)   Right_Lost_Time++;
        if(Left_Lost_Flag[i]==1&&Right_Lost_Flag[i]==1)   Both_Lost_Time++;
        if(Boundry_Start_Left==0&&Left_Lost_Flag[i]==0)   Boundry_Start_Left=i;//记录第一??
        if(Boundry_Start_Right==0&&Right_Lost_Flag[i]==0) Boundry_Start_Right=i;//记录第一??
        Road_Wide[i]=right_line[i]-left_line[i];//赛道宽度数组
    }
    /*赛道类型判断*/
    if(Left_Lost_Time<=15&&Right_Lost_Time<=15&&Both_Lost_Time<=15) Road_Type=STRAIGHT_ROAD;
    if(Left_Lost_Time<15&&Right_Lost_Time>=30&&Both_Lost_Time<15)   Road_Type=RIGHT_TURN;
    if(Right_Lost_Time<15&&Left_Lost_Time>=30&&Both_Lost_Time<15)   Road_Type=LEFT_TURN;
    if(Right_Lost_Time>=30&&Left_Lost_Time>=30&&Both_Lost_Time>=30) Road_Type=CROSSING;

}

/**
 * @brief 连续性检测
 * @param height:截止行数
 * @return 返回最大的连续性列数差值
 */
uint8 Continuity_detect(uint8 *line)
{
    uint8 max_uncontinuity=0;//最大相差值
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        if(line[i]-line[i-1]>max_uncontinuity)
        {
            max_uncontinuity=line[i]-line[i-1];
        }
    }
    return max_uncontinuity;//返回最大的连续性列数差值
}

void Derivative_Change(void)
{
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        Left_derivative[i]=(left_line[i]-left_line[i-1])/2;
        Right_derivative[i]=(right_line[i]-right_line[i-1])/2;//求出各自的导数值
    }
}

float Derivative_detect_max(uint8 *line)
{
    float max_derivative=0.00;
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        if(line[i]>max_derivative)
        {
            max_derivative=line[i];
        }
    }
    return max_derivative;
}

float Derivative_detect_min(uint8 *line)
{
    float min_derivative=0.00;
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        if(line[i]<min_derivative)
        {
            min_derivative=line[i];
        }
    }
    return min_derivative;
}
/**
 * @brief 误差处理函数
 * @param height:截止行数
 * @return 误差
 */
float Err_Handle(uint8 height)
{
    /*这种写法会使得前瞻不够远，故舍弃
    float err=0.00;//值为负，往左偏；值为正，往右偏
    int sum_err[IMAGE_HEIGHT]={0};
    int sum_hight=0;
    for(uint8 i=IMAGE_HEIGHT-1;i>=height;i--)
    {
        sum_err[i]=(right_line[i]+left_line[i])/2-94;
        sum_hight+=i;
    }
    for(uint8 i=IMAGE_HEIGHT-1;i>=height;i--)
    {
        err+=sum_err[i]/sum_hight;//均值分??
    }
    */
    
    float err=0.00;
    int weight_count=0;//权重总??
    for(int i=IMAGE_HEIGHT-1;i>IMAGE_HEIGHT/2;i--)
    {
        err+=(IMAGE_WIDTH/2-((left_line[i]+right_line[i])>>1))*Weight[i];
        weight_count+=Weight[i];//权重总值相??
    }
    err=err/weight_count;//权重均??
    return err;
}

/**
 * @brief 测试函数（把测试的丢在这里）
 * @param ??
 * @return ??
 */
//void test(void)
//{
//   Image_Change();
//	ips114_show_char(188,120,'Q');
//	ips114_displayimage03x(*Image_Use,188,120);
//}
void test(void)
{
    Image_Change();
	uint8 *output_address;//输入地址指针
    
    output_address=Scharr_Edge(*mt9v03x_image);
    memcpy(Image_Use,output_address,IMAGE_HEIGHT*IMAGE_WIDTH*sizeof(uint8));
	uint8 threshold=Camera_GetOSTU((uint8 *)mt9v03x_image);

    Simple_Binaryzation(*Image_Use,threshold);
    Center_line_deal(10,178);//白列寻边线
//`	
//    for(uint8 i=0;i<=IMAGE_HEIGHT-1;i++)
//    {
//        ips114_draw_point((left_line[i]+right_line[i])/2,i,RGB565_BLUE);
//        
//    }
    // ips114_show_uint(188,120,Longest_White_Column_Left[1],3);      
	ips114_displayimage03x(*Image_Use,188,120);
}
