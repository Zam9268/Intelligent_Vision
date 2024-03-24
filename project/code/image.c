#include "image.h"
#include "stdbool.h"

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
uint8 Right_Down_Find=0;
uint8 Left_Down_Find=0;//左右找点标志位清零
uint8 Left_Up_Find=0;//左上找点标志位清零
uint8 Right_Up_Find=0;//右上找点标志位清零
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

uint8 OSTU_GetThreshold(uint8 *image, uint16 Width, uint16 Height)
{
    uint8 HistGram[257] = {0}; // 将数组大小改为 257
    uint16 x, y;
    int16 Y;
    uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma;
    int16 MinValue, MaxValue;
    uint8 Threshold = 0;
    uint8 *data = image;
    for (y = 0; y < Height; y++)
    {
        for (x = 0; x < Width; x++)
        {
            HistGram[data[y * Width + x]]++;
        }
    }
    HistGram[255] = 0; // 将像素值为 255 的像素单独处理

    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++)
        ;
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MaxValue] == 0; MaxValue--)
        ;

    if (MaxValue == MinValue)
    {
        return MaxValue;
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;
    }
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
        PixelBack = PixelBack + HistGram[Y];
        PixelFore = Amount - PixelBack;
        OmegaBack = (double)PixelBack / Amount;
        OmegaFore = (double)PixelFore / Amount;
        PixelIntegralBack += HistGram[Y] * Y;
        PixelIntegralFore = PixelIntegral - PixelIntegralBack;
        MicroBack = (double)PixelIntegralBack / PixelBack;
        MicroFore = (double)PixelIntegralFore / PixelFore;
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            Threshold = Y;
        }
    }
    return Threshold;
}
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
		right_line[i]=IMAGE_WIDTH-1;
        Right_Lost_Flag[i]=0;
        Left_Lost_Flag[i]=0;
	}
    for(uint8 i=0;i<IMAGE_WIDTH-1;i++)
    {
        White_Column[i]=0;
    }
    int x=0,y=0;//设x为行,y为列
    uint8 middle=the_maxlen_position;//定义赛道最长位??
    uint8 x_num;
    /*寻找最长白列（待优化） */
    for(uint8 j=start_column;j<=end_column;j++)
    {
        for(uint8 i=IMAGE_HEIGHT-1;i>=0;i--)
        {
            if(Image_Use[i][j]==BLACK_POINT)
            {
                break;
            }
            else
            {
                White_Column[j]++;
                if(White_Column[j]==120)    break;
            }
        }
    }
    /*从左到右寻找最长白??*/
    Longest_White_Column_Left[0]=0;//白列长度清零
    for(uint8 i=start_column;i<=end_column;i++)
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
        if(White_Column[i]>Longest_White_Column_Right[0])//最大值更更新
        {
            Longest_White_Column_Right[0]=White_Column[i];
            Longest_White_Column_Right[1]=i;//最长列的长度和所在列数更新
        }
    }
    /*终止行赋??*/
    Search_Stop_Line=Longest_White_Column_Left[0];//搜索截止行的赋?值
    int right_border,left_border;//定义边界中间变量
    for(int i=IMAGE_HEIGHT-1;i>=IMAGE_HEIGHT-Search_Stop_Line;i--)
    {
        /*先寻右边??*/
        for(int j=Longest_White_Column_Left[1];j<=IMAGE_WIDTH-1;j++)
        {
            if(Image_Use[i][j]==WHITE_POINT&&Image_Use[i][j+1]==BLACK_POINT&&Image_Use[i][j+2]==BLACK_POINT)
            {
                right_border=j;//记录当前边界??
                Right_Lost_Flag[i]=0;//没有丢线，就??0
                break;
            }
            else if(j>=IMAGE_WIDTH-1-2)//没有找到右边界时，就把最右边界赋值给右边，然后丢线标志位??1
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
 * @brief 左赛道连续性检测
 * @param start:起始行数 end:终止行数
 * @return 返回最大的连续性列数差值
 */
int Continuity_Change_Left(int start, int end)
{
    int i,t,continuity_change_flag=0;
    if(Left_Lost_Time >=0.9*IMAGE_HEIGHT)   return 1;//大部分丢线，没必要判断
    if(Search_Stop_Line <=5) return 1; //搜索截止行很短，没必要判断
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//数组越界保护
    if(end<=5)  end=5;//数组越界保护
    if(start<end)//原则上start要大于end（从下往上扫）
    {
        t=start;
        start=end;
        end=t;
    }
    for(i=start;i>=end;i--)
    {
        if(left_line[i]-left_line[i-1]>=5)//这里5是连续性判断的阈值，可以更改
        {
            continuity_change_flag=i;
            break;//返回第一个检测到的坐标点
        }
    }
    return continuity_change_flag;//如果为0，说明连续性较好
}

/**
 * @brief 右赛道连续性检测
 * @param start:起始行数 end:终止行数
 * @return 返回最大的连续性列数差值
 */
int Continuity_Change_Right(int start, int end)
{
    int i,t,continuity_change_flag=0;
    if(Right_Lost_Time >=0.9*IMAGE_HEIGHT)   return 1;//大部分丢线，没必要判断
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//数组越界保护
    if(end <=5) end=5;//数组越界保护
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    for(i=start;i>=end;i--)
    {
        if(abs(right_line[i]-right_line[i-1])>=5)//这里5是连续性判断的阈值，可以更改
        {
            continuity_change_flag=i;
            break;//返回第一个检测到的坐标点
        }
    }
    return continuity_change_flag;//如果为0，说明连续性较好，返回超过阈值检测的第一个行坐标（从下往上扫的）
}

/**
 * @brief 连续性检测
 * @param line:边线数组
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

/**
 * @brief 边线变化率处理
 * @param 无（同时对左边线和右边线进行导数处理）
 * @return 无
 * @attention 这个函数可以优化，比如选择处理对应的导数行数
 */
void Derivative_Change(void)
{
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        Left_derivative[i]=(left_line[i]-left_line[i-1])/2;
        Right_derivative[i]=(right_line[i]-right_line[i-1])/2;//求出各自的导数值
    }
}

/**
 * @brief 求出边线最大变化率的值（大小比较）
 * @param uint8 *line 边线数组
 * @return 无
 * @attention 无
 */
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

/**
 * @brief 求出边线最小变化率的值（大小比较）
 * @param uint8 *line 边线数组
 * @return 无
 * @attention 无
 */
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
 * @brief 求出左边线不满足单调性的第一个行数
 * @param int start,int end 起始行，终止行
 * @return 无
 * @attention 无
 */
int Monotonicity_Change_Left(int start, int end)
{
    int i,monotonicity_change_line=0;
    if(Left_Lost_Time>=0.9*IMAGE_HEIGHT)   return 0;//大部分丢线，没必要判断
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//数组越界保护
    if(end<=5) end=5;//数组越界保护
    if(start<=end)  return 0; //如果入口算反了的话，会直接返回0
    for(i=start;i>=end;i--)
    {
        /*从某一个点向上和向下分别取5个点，如果这个点的横坐标是最大的，那就默认该点不符合单调性*/
        if(left_line[i] >= left_line[i + 5] && left_line[i] >= left_line[i - 5] &&
                 left_line[i] >= left_line[i + 4] && left_line[i] >= left_line[i - 4] &&
                 left_line[i] >= left_line[i + 3] && left_line[i] >= left_line[i - 3] &&
                 left_line[i] >= left_line[i + 2] && left_line[i] >= left_line[i - 2] &&
                 left_line[i] >= left_line[i + 1] && left_line[i] >= left_line[i - 1])
        {
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;
}

/**
 * @brief 求出右边线不满足单调性的第一个行数
 * @param int start,int end 起始行，终止行
 * @return 无
 * @attention 无
 */
int Monotonicity_Change_Right(int start,int end)
{
    int i,monotonicity_change_line=0;
    if(Right_Lost_Time >=0.9*IMAGE_HEIGHT)   return 1;//大部分丢线，没必要判断
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//数组越界保护
    if(end <=5) end=5;//数组越界保护
    if(start<=end)  return monotonicity_change_line;//如果输入错误直接返回
    for(i=start;i>=end;i--)//从下往上扫
    {
        /*对于左边线，如果该点坐标是在上下5个点中最靠左的话就认为符合单调性*/
        if(right_line[i] <= right_line[i + 5] && right_line[i] <= right_line[i - 5] &&
            right_line[i] <= right_line[i + 4] && right_line[i] <= right_line[i - 4] &&
            right_line[i] <= right_line[i + 3] && right_line[i] <= right_line[i - 3] &&
            right_line[i] <= right_line[i + 2] && right_line[i] <= right_line[i - 2] &&
            right_line[i] <= right_line[i + 1] && right_line[i] <= right_line[i - 1])
        {
            monotonicity_change_line=i;
            break;
        }
    }
    return monotonicity_change_line;//如果为0，说明单调性比较良好
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
    int weight_count=0;//权重总值
    for(int i=IMAGE_HEIGHT-1;i>IMAGE_HEIGHT/2;i--)
    {
        err+=(IMAGE_WIDTH/2-((left_line[i]+right_line[i])>>1))*Weight[i];
        weight_count+=Weight[i];//权重总值相??
    }
    err=err/weight_count;//权重均??
    return err;
}

/**
 * @brief 左补线函数
 * @param int x1, int y1, int x2,int y2 补线的起始点和终点的坐标
 * @return 无（直接修改左边线数组的值）
 */
void Left_Add_Line(int x1, int y1, int x2,int y2)
{
    int i,max,a1,a2,hx;
    //对输入的值进行限幅处理
    if(x1>=IMAGE_WIDTH) x1=IMAGE_WIDTH-1;//限幅处理
    else if(x1<=0)  x1=0;
    if(x2>=IMAGE_WIDTH) x2=IMAGE_WIDTH-1;//限幅处理
    else if(x2<=0)  x2=0;
    if(y1>=IMAGE_HEIGHT) y1=IMAGE_HEIGHT-1;//限幅处理
    else if(y1<=0)  y1=0;
    if(y2>=IMAGE_HEIGHT) y2=IMAGE_HEIGHT-1;//限幅处理
    else if(y2<=0)  y2=0;
    a1=y1;
    a2=y2;//记录中间的值
    if(a1>a2)//一般原则上a1要小于a2
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//在图像上从上到下进行补线
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;//求出原本的行对应的列坐标
        if(hx >= IMAGE_WIDTH) hx = IMAGE_WIDTH-1;//限幅处理
        else if(hx <= 0) hx = 0;
        left_line[i]=hx;//赋值给左边线数组（这里千万不要只是改变原来图像的百白点，因为这样的话还要再扫一次得到边线数组，这样的话扫描效率就会很低）
    }
}

/**
 * @brief 右补线函数
 * @param int x1, int y1, int x2,int y2 补线的起始点和终点的坐标
 * @return 无（直接修改右边线数组的值）
 */
void Right_Add_Line(int x1, int y1, int x2, int y2)
{
    int i,max,a1,a2,hx;
    if(x1>=IMAGE_WIDTH) x1=IMAGE_WIDTH-1;//限幅处理
    else if(x1<=0)  x1=0;
    if(x2>=IMAGE_WIDTH) x2=IMAGE_WIDTH-1;//限幅处理
    else if(x2<=0)  x2=0;
    if(y1>=IMAGE_HEIGHT) y1=IMAGE_HEIGHT-1;//限幅处理
    else if(y1<=0)  y1=0;
    if(y2>=IMAGE_HEIGHT) y2=IMAGE_HEIGHT-1;//限幅处理
    else if(y2<=0)  y2=0;
    a1=y1;
    a2=y2;//记录中间的值
    if(a1>a2)//一般原则上a1要小于a2
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//在图像上从上到下进行补线
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;//求出原本的行对应的列坐标
        if(hx >= IMAGE_WIDTH) hx = IMAGE_WIDTH-1;//限幅处理
        else if(hx <= 0) hx = 0;
        right_line[i]=hx;//赋值给右边线数组（这里千万不要只是改变原来图像的百白点，因为这样的话还要再扫一次得到边线数组，这样的话扫描效率就会很低）
    }
}

/**
 * @brief 十字元素处理（供十字使用）
 * @param int start, int end ：搜索的范围起点和终点
 * @return 修改两个全局变量 Right_Down_Find=0;Left_Down_Find=0;
 */
void Find_Down_Point(int start, int end)
{
    int i,t;
    Right_Down_Find=0;
    Left_Down_Find=0;//左右找点标志位清零
    if(start<end)//原则上start要大于end
    {
        t=start;
        start=end;
        end=t;
    }
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//下面5行的数据不稳定，不能作为边界点来判断
    if(end<=IMAGE_HEIGHT-Search_Stop_Line)  end=IMAGE_HEIGHT-Search_Stop_Line;//搜索终止行
    if(end<=5)  end=5;
    /*角点判断方法：与近处点连续，与远处点不连续（或者判断变化率）*/
    for(i=start;i>=end;i--)
    {
        if(Left_Down_Find == 0 && abs(left_line[i]-left_line[i+1])<=5 && abs(left_line[i+1]-left_line[i+2])<=5 &&
        abs(left_line[i+2]-left_line[i+3])<=5 && abs(left_line[i]-left_line[i-2])>=8 && abs(left_line[i]-left_line[i-2])>=15 &&
        abs(left_line[i]-left_line[i-4])>=15)//左边线找点
        {
            Left_Down_Find=i;//获取对应的行数
        }
        if(Right_Down_Find == 0 &&abs(right_line[i]-right_line[i+1])<=5 && abs(right_line[i+1]-right_line[i+2])<=5 &&
        abs(right_line[i+2]-right_line[i+3])<=5 && abs(right_line[i]-right_line[i-2])>=8 && abs(right_line[i]-right_line[i-2])>=15)
        {
            Right_Down_Find=i;//获取对应的行数
        }
        if(Left_Down_Find!=0 && Right_Down_Find!=0)    break;//找到左右点就退出
    }
}

/**
 * @brief 找到上面的两个拐点，供十字使用
 * @param start:搜索范围的起点；end:搜索范围的终点
 * @return 无（返回的是两个全局变量Left_Up_Find和Right_Up_Find）
 */
void Find_Up_Point(int start, int end)
{
    int i,t;//中间变量
    Left_Up_Find=0;//左上找点标志位清零
    Right_Up_Find=0;//右上找点标志位清零
    if(start<end)//原则上start要大于end
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=IMAGE_HEIGHT-Search_Stop_Line)  end=IMAGE_HEIGHT-Search_Stop_Line;//搜索终止行
    if(end<=5)  end=5;
    if(start >=IMAGE_HEIGHT -1-5)   start=IMAGE_HEIGHT-1-5;//下面5行的数据不稳定，不能作为边界点来判断
    /*从该点往上扫的点为连续点，从该点往下扫的点为非连续点*/
    for(i=start;i>=end;i--)//从行数的底下往上扫
    {
        if(Left_Up_Find == 0 && 
        abs(left_line[i]-left_line[i-1])<=5 &&
        abs(left_line[i-1] -  left_line[i-2])<=5 &&
        abs(left_line[i-2] -  left_line[i-3])<=5 &&
        abs(left_line[i] - left_line[i+2])>=8 &&
        abs(left_line[i] - left_line[i+3])>=15 &&
        abs(left_line[i] - left_line[i+4])>=15)//左边线找点
        {
            Left_Up_Find=i;//获取对应的行数
        }
        if(Right_Up_Find == 0 &&
        abs(right_line[i]-right_line[i-1]) <=5 &&
        abs(right_line[i-1]-right_line[i-2]) <=5 &&
        abs(right_line[i-2]-right_line[i-3]) <=5 &&
        abs(right_line[i]-right_line[i+2]) >=8 &&
        abs(right_line[i]-right_line[i+3]) >=15 &&
        abs(right_line[i]-right_line[i+4]) >=15)//右边线找点
        {
            Right_Up_Find=i;//获取对应的行数
        }
        if(Left_Up_Find!=0 && Right_Up_Find!=0)    break;//找到左右点就退出
    }
    if(abs(Right_Up_Find-Left_Up_Find)>=30)//如果拐点的纵向撕裂过大，就视为误判
    {
        Right_Up_Find=0;
        Left_Up_Find=0;
    }
}

/**
 * @brief 左边界延长
 * @param 延长起始行数，延长到某行
 * @return null
 */
void Lengthen_Left_Boundry(int start, int end)
{
    int i,t;
    float k=0.0;
    if(start >=IMAGE_HEIGHT -1) start=IMAGE_HEIGHT-1; //起始点位置矫正
    else if(start <=0)  start=0;//限幅
    if(end>=IMAGE_HEIGHT-1) end=IMAGE_HEIGHT-1;//限幅
    else if(end<=0)  end=0;//限幅
    if(end < start) //原则上end要大于start
    {
        t=start;
        start=end;
        end=t;
    }
    if(start <=5)   Left_Add_Line(left_line[start],start,left_line[end],end);//如果起始点过于靠上，就不能延长（误差太大了）
    else
    {
        k=(float)(left_line[start]-left_line[start-4])/5.0; //这里的k是1/斜率
        for(i=start;i<=end;i++)
        {
            left_line[i]=(int)(i-start)*k+left_line[start];//延长左边界
            if(left_line[i]>=IMAGE_WIDTH-1) left_line[i]=IMAGE_WIDTH-1;//限幅处理
            else if(left_line[i]<=0) left_line[i]=0;//限幅处理
        }
    }
}

/**
 * @brief 边右界延长
 * @param 延长起始行数，延长到某行
 * @return null
 */
void Lengthen_Right_Boundry(int start, int end)
{
    int i,t;
    float k=0.0;
    if(start >=IMAGE_HEIGHT -1) start=IMAGE_HEIGHT-1; //起始点位置矫正
    else if(start <=0)  start=0;//限幅
    if(end>=IMAGE_HEIGHT-1) end=IMAGE_HEIGHT-1;//限幅
    else if(end<=0)  end=0;//限幅
    if(end < start) //原则上end要大于start
    {
        t=start;
        start=end;
        end=t;
    }
    if(start <=5)   Right_Add_Line(right_line[start],start,right_line[end],end);//如果起始点过于靠上，就不能延长（误差太大了）
    else
    {
        k=(float)(right_line[start]-right_line[start-4])/5.0; //这里的k是1/斜率
        for(i=start;i<=end;i++)
        {
            right_line[i]=(int)(i-start)*k+right_line[start];//延长右边界
            if(right_line[i]>=IMAGE_WIDTH-1) right_line[i]=IMAGE_WIDTH-1;//限幅处理
            else if(right_line[i]<=0) right_line[i]=0;//限幅处理
        }
    }
}

/**
 * @brief 十字检测
 * @param null
 * @return null
 */
void Cross_Detect(void)
{
    int down_search_start = 0;//定义下面开始搜索的行
    if(Road_Type == CROSSING)//如果检测到十字路口
    {
        Left_Up_Find=0;
        Right_Up_Find=0;
        if(Both_Lost_Time >=15)//出现双边丢线开始找点
        {
            Find_Up_Point(IMAGE_HEIGHT-1,30);//找上面的两个拐点
            if(Left_Up_Find ==0 && Right_Up_Find ==0) return ;//找不到就直接返回
        }
        if(Left_Up_Find !=0 &&Right_Up_Find !=0)
        {
            down_search_start=Left_Up_Find>Right_Up_Find? Left_Up_Find:Right_Up_Find;//取最大值，方便下拐点寻找
            Find_Down_Point(IMAGE_HEIGHT -5,down_search_start+2);//找下面的两个拐点
            if(Left_Down_Find<=Left_Up_Find)    Left_Down_Find=0;//如果下拐点在上拐点的上面，就视为误判
            if(Right_Down_Find<=Right_Up_Find)  Right_Down_Find=0;//如果下拐点在上拐点的上面，就视为误判
            if(Left_Down_Find!=0 && Right_Down_Find!=0)//四个点都在就直接连线
            {
                Left_Add_Line(left_line[Left_Up_Find],Left_Up_Find,left_line[Left_Down_Find],Left_Down_Find);//左边补线
                Right_Add_Line(right_line[Right_Up_Find],Right_Up_Find,right_line[Right_Down_Find],Right_Down_Find);//右边补线
            }
            else if(Left_Down_Find == 0 && Right_Down_Find !=0)//斜入十字
            {
                Lengthen_Left_Boundry(Left_Up_Find-1,IMAGE_HEIGHT-1);//左边界延长
                Right_Add_Line(Right_Up_Find,Right_Up_Find,right_line[Right_Down_Find],Right_Down_Find);//右边界补线
            }
            else if (Left_Down_Find !=0 && Right_Down_Find ==0)//斜入十字
            {
                Lengthen_Right_Boundry(Right_Up_Find-1,IMAGE_HEIGHT-1);//右边界延长
                Left_Add_Line(Left_Up_Find,Left_Up_Find,left_line[Left_Down_Find],Left_Down_Find);//左边界补线
            }
            else if(Left_Down_Find == 0 && Right_Down_Find == 0)//直入十字
            {
                Lengthen_Left_Boundry(Left_Up_Find-1,IMAGE_HEIGHT-1);//左边界延长
                Lengthen_Right_Boundry(Right_Up_Find-1,IMAGE_HEIGHT-1);//右边界延长
            }
        }
    }
}

/**
 * @brief 检测黑白跳变点（一般用于斑马线检测）
 * @param uint8 row 检测的行数 uint8 start_column 起始列数 uint8 end_column 终止列数
 * @return 返回黑白跳变点的个数（一般大于5个）
 */
uint8 Black_White_Dump(uint8 row,uint8 start_column,uint8 end_column)
{
    if(row>=IMAGE_HEIGHT-1) row=IMAGE_HEIGHT-1;//限幅处理
    else if(row<=0) row=0;//限幅处理
    if(row<=5)  return 0;//如果行数过小，就不检测
    if(start_column>=IMAGE_WIDTH-1) start_column=IMAGE_WIDTH-1;//限幅处理
    else if(start_column<=0) start_column=0;//限幅处理
    if(end_column>=IMAGE_WIDTH-1) end_column=IMAGE_WIDTH-1;//限幅处理
    else if(end_column<=0) end_column=0;//限幅处理
    uint8 count=0;
    for(uint8 i=start_column;i<=end_column;i++)
    {
        if(Image_Use[row][i]==WHITE_POINT&&Image_Use[row][i+1]==BLACK_POINT)
        {
            count++;
        }
    }
    return count;
}

/**
 * @brief 环岛检测
 * @param 无
 * @return 无
 */
uint8 Island_State=0;
void Island_Detect(void)
{
    static int state1_down_guai[2]={0};//状态1下拐点
    static int state1_up_guai[2]={0};//状态1上拐点
    int monotonicity_change_left_flag=0;
    int monotonicity_change_right_flag=0;//单调点坐标
    int continuity_change_left_flag=0;//连续性点的坐标
    int continuity_change_right_flag=0;//连续性点的坐标

}
/**
 * @brief 斑马线检测
 * @param 无
 * @return 无
 */
void Zebra_Stripes_Detect(void)
{
    int continuity_change_right_flag =0;//连续是0
    int continuity_change_left_flag =0;//连续是0
    int monotonicity_change_right_flag =0;//单调性是0
    int monotonicity_change_left_flag =0;//单调性是0

    continuity_change_left_flag = Continuity_Change_Left(IMAGE_HEIGHT-1,5);//左边线连续性检测
    continuity_change_right_flag = Continuity_Change_Right(IMAGE_HEIGHT-1,5);//右边线连续性检测
    monotonicity_change_left_flag = Monotonicity_Change_Left(IMAGE_HEIGHT-1,5);//左边线单调性检测
    monotonicity_change_right_flag = Monotonicity_Change_Right(IMAGE_HEIGHT-1,5);//右边线单调性检测

    int i=0,j=0,change_count=0,start_line=0,endl_line=0,narrow_road_count=0;
    if(Search_Stop_Line >=60 && 30<=Longest_White_Column_Left[1] && Longest_White_Column_Left[1]<=IMAGE_WIDTH-30  &&
    Longest_White_Column_Right[1] >=30  &&Longest_White_Column_Right[1]<=IMAGE_WIDTH-30 &&continuity_change_left_flag!=0 &&continuity_change_right_flag!=0)
    {
        if(Black_White_Dump(continuity_change_left_flag-3,left_line[continuity_change_left_flag-3],right_line[continuity_change_left_flag-3])>=5)
        {
            if(Road_Type==STRAIGHT_ROAD)    Road_Type=BANMAXIAN;//元素类型切换
        }
    }
}

/**
 * @brief 测试函数（把测试的丢在这里）
 * @param ??
 * @return ??
 */
void test(void)
{
  Image_Change();
	uint8 *output_address;//输入地址指针
    
    // output_address=Scharr_Edge(*mt9v03x_image);
//    output_address=Camera_GetOSTU(*mt9v03x_image);//获取OSTU二值化后的图像
    uint8 threshold=OSTU_GetThreshold((uint8 *)mt9v03x_image,IMAGE_WIDTH,IMAGE_HEIGHT);
    // memcpy(Image_Use,output_address,IMAGE_HEIGHT*IMAGE_WIDTH*sizeof(uint8));
	

    Simple_Binaryzation(*Image_Use,threshold);
    Center_line_deal(0,188);//白列寻边线
//`	
//   for(uint8 i=0;i<=IMAGE_HEIGHT-1;i++)
//   {
//       ips114_draw_point((left_line[i]+right_line[i])/2,i,RGB565_RED);
//       
//   }
    ips114_show_uint(188,120,threshold,3);      
	ips114_displayimage03x(*Image_Use,188,120);
	ips114_show_uint(188,0,Longest_White_Column_Left[1],3);
    
    // ips114_show_uint(188,20,White_Column[85],3);
    // ips114_show_uint(188,40,White_Column[105],3);
    // ips114_show_uint(188,60,White_Column[110],3);
}
