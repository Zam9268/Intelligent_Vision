#include "image.h"
#include "stdbool.h"

uint8 Image_Use[IMAGE_HEIGHT][IMAGE_WIDTH];

/*???????????????*/
uint8 left_line[IMAGE_HEIGHT],right_line[IMAGE_HEIGHT];//???????????
int center[IMAGE_HEIGHT];//????????
uint8 the_maxlen_position;//??????λ??
uint8 num;//??????Ч????
uint8 Longest_White_Column_Left[2]; // Record the longest white column in this iteration
uint8 Last_Longest_White_Column_Left[2]; // Record the longest white column in the previous iteration to prevent white column fluctuations in some areas
uint8 Longest_White_Column_Right[2]; // The longest white column on the right side, not used
uint8 Right_Lost_Flag[IMAGE_HEIGHT]; // Lost line flag for the right boundary
uint8 Left_Lost_Flag[IMAGE_HEIGHT]; // Lost line flag for the left boundary
uint8 Left_Lost_Time = 0; // Number of times the left line is lost
uint8 Right_Lost_Time = 0; // Number of times the right line is lost
uint8 Both_Lost_Time = 0; // Number of times both lines are lost in the same row
uint8 Search_Stop_Line; // Stop line for searching
uint8 Boundry_Start_Left, Boundry_Start_Right; // Starting points of the left and right boundaries
uint8 Road_Wide[IMAGE_HEIGHT]; // Road width
RoadType Road_Type; // Type of road element
uint8 Right_Down_Find = 0;
uint8 Left_Down_Find = 0; // Finding the left bottom turning point
uint8 Left_Up_Find = 0; // Finding the left top turning point
uint8 Last_Left_Up_Find=0;
uint8 Last_Right_Up_Find=0;//记录上次的位置
uint8 Right_Up_Find = 0; // Finding the right top turning point
float Left_derivative[IMAGE_HEIGHT]={0.0};
float Right_derivative[IMAGE_HEIGHT]={0.0};
float err=0.00;
float last_err=0.00;

/*以下是其他函数中外部声明的变量*/
extern uint8 right_data[64];//存储最终的数据    
extern uint32 fifo_data_count;//单次接收的数组个数
extern uint8 data_length;//数据长度
extern uint8 i;//下标指针
// Corresponding image height weight array (counting from bottom to top)
const uint8 Weight[IMAGE_HEIGHT]=
{
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // Weight of rows 0 to 9
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // Weight of rows 10 to 19
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // Weight of rows 20 to 30
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, // Weight of rows 30 to 39
    1, 1, 1, 1, 1, 1, 1, 3, 4, 5, // Weight of rows 40 to 49
    6, 7, 9, 11, 13, 15, 17, 19, 20, 20, // Weight of rows 50 to 59
    19, 17, 15, 13, 11, 9, 7, 5, 3, 1, // Weight of rows 60 to 69
};

const uint8 Zebra[60]={
    30,30,30,30,30,30,30,30,30,30,
    35,35,35,35,35,35,35,35,35,35,
    45,45,45,45,45,45,45,45,45,45,
    55,55,55,55,55,55,55,55,55,55,
    65,65,65,65,65,65,65,65,65,65,
    85,85,85,85,85,85,85,85,85,85,
};
uint8 OSTU_GetThreshold(uint8 *image, uint16 Width, uint16 Height)
{
    uint8 HistGram[257] = {0}; // ???????С??? 257
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
    HistGram[255] = 0; // ???????? 255 ?????????????

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
 * @brief ???鸳?
 * @param ??
 * @return ??
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
volatile int White_Column[IMAGE_WIDTH];//??а??г???
/**
 * @brief ????????????????????????з?????sobel,canny???????з??????????
 * @param H???????????
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
    int x=0,y=0;//??x???,y???
    uint8 middle=the_maxlen_position;//??????????λ??
    uint8 x_num;
    /*????????У???????? */
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
    /*???????????????*/
    Longest_White_Column_Left[0]=0;//???г???????
    for(uint8 i=start_column;i<=end_column;i++)
    {
        if(White_Column[i]>Longest_White_Column_Left[0])//????????
        {
            Longest_White_Column_Left[0]=White_Column[i];
            Longest_White_Column_Left[1]=i;
        }
    }
    /*????????????????*/
    Longest_White_Column_Right[0]=0;//???г???????
    for(uint8 i=end_column;i>start_column;i--)
    {
        if(White_Column[i]>Longest_White_Column_Right[0])//??????????
        {
            Longest_White_Column_Right[0]=White_Column[i];
            Longest_White_Column_Right[1]=i;//???е?????????????????
        }
    }
    /*????и???*/
    Search_Stop_Line=Longest_White_Column_Left[0];//????????е????
    int right_border,left_border;//???????м????
    for(int i=IMAGE_HEIGHT-1;i>=IMAGE_HEIGHT-Search_Stop_Line;i--)
    {
        /*????????*/
        for(int j=Longest_White_Column_Left[1];j<=IMAGE_WIDTH-1;j++)
        {
            if(Image_Use[i][j]==WHITE_POINT&&Image_Use[i][j+1]==BLACK_POINT&&Image_Use[i][j+2]==BLACK_POINT)
            {
                right_border=j;//???????????
                Right_Lost_Flag[i]=0;//??ж????????0
                break;
            }
            else if(j>=IMAGE_WIDTH-1-2)//?????????????????????縳??????????????λ??1
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
                left_border=j;//???????????
                Left_Lost_Flag[i]=0;//??ж????????0
                break;
            }
            else if(j<=2)//?????????????????????縳??????????????λ??1
            {
                left_border=j;
                Left_Lost_Flag[i]=1;
                break;
            }
        }
        left_line[i]=left_border;//?????????????
        right_line[i]=right_border;//?????????????
    }
}

/**
 * @brief Longest white column plus version (written by myself, used for finding the longest white column in image detection)
 * @param uint8 start_column, uint8 end_column: The starting and ending columns for finding the longest white column
 * @return None (actually returns the edge line array)
 */
void Center_line_deal_plus(uint8 start_column,uint8 end_column)
{
    for(uint8 i=0;i<IMAGE_HEIGHT-1;i++)
    {
        left_line[i]=0;
        right_line[i]=0;
        Right_Lost_Flag[i]=0; // Clear the right line lost flag to 0
        Left_Lost_Flag[i]=0; // Clear the left line lost flag to 0
        
    }
    Left_Lost_Time=0;
    Right_Lost_Time=0;//计数值清零
    /* Reset white column count */
    for(uint8 i=0;i<=IMAGE_WIDTH-1;i++)
    {
        White_Column[i]=0;
    }
    /*Counting white columns*/
    for(uint8 j=start_column;j<=end_column;j++)
    {
        for(uint8 i=IMAGE_HEIGHT-3;i>=0;i--)//由于最底下是白边，所以就跳过最底下的白边
        {
            if(Image_Use[i][j]==BLACK_POINT)// Stop counting when encountering a white boundary point, otherwise increment
            {
                White_Column[j]++;
                if(White_Column[j]==120)    break;//if count is encough,stop countting
            }
            else    
            {
                break;
            }
        }
    }
    /* Find the longest white column */
    Last_Longest_White_Column_Left[0]=Longest_White_Column_Left[0];// Record the longest white column in the previous iteration
    Last_Longest_White_Column_Left[1]=Longest_White_Column_Left[1];//record the column number of the longest white column in the previous iteration
    Longest_White_Column_Left[0]=0;// Clear the information of the longest white column
    for(uint8 i=start_column;i<=end_column;i++)
    {
        if(White_Column[i]>Longest_White_Column_Left[0])// Replace the longest white column with the maximum value
        {
            Longest_White_Column_Left[0]=White_Column[i];// Record the length of the corresponding longest white column
            Longest_White_Column_Left[1]=i;// Record the column number where the corresponding longest white column is located
        }
    }
    Search_Stop_Line=Longest_White_Column_Left[0];// Set the stop line for searching to the length of the longest white column
    /* To prevent significant changes in the position of the longest white column at the turning point, set a verification for the change */
    if(abs(Longest_White_Column_Left[1]-Last_Longest_White_Column_Left[1])>=60)// If the longest white column position changes by more than 60 columns
    {
        Longest_White_Column_Left[0]=Last_Longest_White_Column_Left[0];// Then the longest white column is set to the previous value
        Longest_White_Column_Left[1]=Last_Longest_White_Column_Left[1];
    }
    
    /* Start searching for boundaries */
    int right_border,left_border;// Define intermediate variables for boundaries
    for(int i=IMAGE_HEIGHT-1;i>=IMAGE_HEIGHT-Search_Stop_Line;i--)
    {
        for(int j=Longest_White_Column_Left[1];j>=2;j--)// Search for the left boundary from the middle to the left
        {
            if(Image_Use[i][j]==BLACK_POINT&&Image_Use[i][j-1]==WHITE_POINT&&Image_Use[i][j-2]==WHITE_POINT)
            {
                left_border=j;// Record the column coordinate of the corresponding boundary
                Left_Lost_Flag[i]=0;// No line lost, set the line lost flag to 0
                break;
            }
            else if(j<=2)// If encountering a boundary
            {
                left_border=j;// Directly record the position of the boundary
                Left_Lost_Flag[i]=1;// Set the line lost flag to 1
                break;
            }
        }
        for(int j=Longest_White_Column_Left[1];j<=IMAGE_WIDTH-3;j++)// Search for the right boundary from the middle to the right
        {
            if(Image_Use[i][j]==BLACK_POINT&&Image_Use[i][j+1]==WHITE_POINT&&Image_Use[i][j+2]==WHITE_POINT)
            {
                right_border=j;// Store the boundary information
                Right_Lost_Flag[i]=0;// Set the boundary flag to 0
                break;
            }
            else if(j>=IMAGE_WIDTH-1-2)// If reaching the right boundary
            {
                right_border=j;// Directly record the position of the right boundary
                Right_Lost_Flag[i]=1;// Then set the line lost flag to 1
                break;
            }
        }
        left_line[i]=left_border;// Store the corresponding boundary information
        right_line[i]=right_border;
    }
}

/**
 * @brief 简单的膨胀操作
 * @param uint8 start_row：起始行；uint8 end_row：终止行；uint8 start_column：起始列；uint8 end_column 终止列   uint8 threshold：阈值
 * @return 无
 * @attention 一般start_row>end_row,start_column<end_column
 */
void Easy_Filtering(uint8 start_row,uint8 end_row,uint8 start_column,uint8 end_column,uint8 threshold)
{
    for(uint8 i=start_row-1;i>=end_row+1;i--)//从下往上扫，边界条件
    {
        for(uint8 j=start_column+1;j<=end_column-1;j++)//从左往右扫
        {
            if(Image_Use[i-1][j-1]+Image_Use[i-1][j]+Image_Use[i-1][j+1]+Image_Use[i][j-1]
                +Image_Use[i][j+1]+Image_Use[i+1][j-1]+Image_Use[i+1][j]+Image_Use[i+1][j+1]>=threshold*WHITE_POINT)//如果周围有5个白点
                {
                    Image_Use[i][j]=WHITE_POINT;//则将该点设置为白点
                }
        }
    }
}
/**
 * @brief 边线数组分析
 * @param 无
 * @return 无
 */
void Outer_Analyse(void)
{
    /*其他有用的标志位的分析*/
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        if(Left_Lost_Flag[i]==1)    Left_Lost_Time++;
        if(Right_Lost_Flag[i]==1)   Right_Lost_Time++;
        if(Left_Lost_Flag[i]==1&&Right_Lost_Flag[i]==1)   Both_Lost_Time++;
        if(Boundry_Start_Left==0&&Left_Lost_Flag[i]==0)   Boundry_Start_Left=i;// Record the starting point of the left boundary
        if(Boundry_Start_Right==0&&Right_Lost_Flag[i]==0) Boundry_Start_Right=i;// Record the starting point of the right boundary
        Road_Wide[i]=right_line[i]-left_line[i];// Record the road width
    }
        /* Preliminary analysis of different flags for track elements */
    if(Left_Lost_Time<=15&&Right_Lost_Time<=15&&Both_Lost_Time<=15) Road_Type=STRAIGHT_ROAD;
    if(Left_Lost_Time<15&&Right_Lost_Time>=30&&Both_Lost_Time<15&&Search_Stop_Line<=100)   Road_Type=RIGHT_TURN;
    if(Right_Lost_Time<15&&Left_Lost_Time>=30&&Both_Lost_Time<15&&Search_Stop_Line<=100)   Road_Type=LEFT_TURN;
    if(Right_Lost_Time>=30&&Left_Lost_Time>=30&&Both_Lost_Time>=30) Road_Type=CROSSING;

    if(Road_Type==STRAIGHT_ROAD)    Zebra_Stripes_Detect();
}

/**
 * @brief 左边线连续性检测（有改进的空间，比如增加一个阈值的参数）
 * @param start:起始行 end:终止行
 * @return 返回不连续的行数
 */
int Continuity_Change_Left(int start, int end,int mode)
{
    int i,t,continuity_change_flag=0;
    if(Left_Lost_Time >=0.9*IMAGE_HEIGHT)   return 1;//丢线数过多就返回1（这句话没必要，在直道上检测肯定不会出现这种情况）
    if(Search_Stop_Line <=5) return 1; //如果截止行过小（最长白列的白列点数小于等于5行），更没必要找了（这种情况出现的比较少）
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//起始行限幅（防止后续判断出现数组越界）
    if(end<=5)  end=5;//终止行限幅（防止后续判断出现数组越界）
    if(start<end)//原则上start要大于end
    {
        t=start;
        start=end;
        end=t;
    }
    if(mode==0)
    {
        for(i=start;i>=end;i--)//在图像上从下到上开始扫描
        {
            if(abs(left_line[i]-left_line[i-1])>=5&&left_line[i-1]>=20&&left_line[i-3]>=20)//如果两行之间的差值大于5（这个阈值可以调整），防止过于靠近边界
            {
                continuity_change_flag=i;
                break;//找到不连续的行就跳出循环
            }
        }
    }
    else if(mode==1)
    {
        for(i=end;i<=start;i++)//在图像上从下到上开始扫描
        {
            if(abs(left_line[i]-left_line[i-1])>=5&&left_line[i-1]>=20&&left_line[i-3]>=20)//如果两行之间的差值大于5（这个阈值可以调整），防止过于靠近边界
            {
                continuity_change_flag=i;
                break;//找到不连续的行就跳出循环
            }
        }
    }
    return continuity_change_flag;//返回0说明没有不连续的行，返回其他值说明有不连续的行
}

/**
 * @brief 右边线连续性检测
 * @param start:起始行 end:终止行
 * @return 返回不连续的行数
 */
int Continuity_Change_Right(int start, int end,int mode)
{
    int i,t,continuity_change_flag=0;
    if(Right_Lost_Time >=0.9*IMAGE_HEIGHT)   return 1;//丢线数过多就返回1
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//起始行限幅
    if(end <=5) end=5;//终止行限幅
    if(start<end)
    {
        t=start;
        start=end;
        end=t;
    }
    if(mode==0)
    {
        for(i=start;i>=end;i--)
        {
            if(abs(right_line[i]-right_line[i-1])>=5)//如果两行之间的差值大于5
            {
                continuity_change_flag=i;
                break;//找到不连续的行就跳出循环
            }
        }
    }
    else if(mode==1)
    {
        for(i=end;i>=start;i++)
        {
            if(abs(right_line[i]-right_line[i-1])>=5)//如果两行之间的差值大于5
            {
                continuity_change_flag=i;
                break;//找到不连续的行就跳出循环
            }
        }
    }
    return continuity_change_flag;//返回0说明没有不连续的行，返回其他值说明有不连续的行
}

/**
 * @brief 连续性检测
 * @param line:待检测的数组
 * @return 返回最大的不连续值
 */
uint8 Continuity_detect(uint8 *line)
{
    uint8 max_uncontinuity=0;//最大不连续值
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        if(line[i]-line[i-1]>max_uncontinuity)
        {
            max_uncontinuity=line[i]-line[i-1];
        }
    }
    return max_uncontinuity;//返回最大不连续值
}

/**
 * @brief 求出对应数组的变化值（变化值可以调整）
 * @param 无
 * @return 无
 * @attention 无
 */
void Derivative_Change(void)
{
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        Left_derivative[i]=(left_line[i]-left_line[i-1])/2;
        Right_derivative[i]=(right_line[i]-right_line[i-1])/2;//求出对应数组的变化值
    }
}

/**
 * @brief ??????????仯????????С????
 * @param uint8 *line ????????
 * @return ??
 * @attention ??
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
 * @brief ?????????С?仯????????С????
 * @param uint8 *line ????????
 * @return ??
 * @attention ??
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
 * @brief 左边界单调性检测
 * @param int start 起始行, int end 终止行
 * @return 返回单调性变化的行数
 * @attention 无
 */
int Monotonicity_Change_Left(int start, int end)
{
    int i,monotonicity_change_line=0;
    if(Left_Lost_Time>=0.9*IMAGE_HEIGHT)   return 1;//丢线数过多就返回1
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//起始行限幅
    if(end<=5) end=5;//终止行限幅
    if(start<=end)  return 1; //如果起始行小于等于终止行，说明没有检测的必要
    for(i=start;i>=end;i--)
    {
        /*如果左边线某一点在相邻上下5行的点中的列坐标最大（最靠右），那么就默认该点为单调点*/
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
    return monotonicity_change_line;//返回单调点所在的行数
}

/**
 * @brief 右边界单调性检测
 * @param int start 起始行, int end 终止行
 * @return 返回单调性变化的行数
 * @attention 无
 */
int Monotonicity_Change_Right(int start,int end)
{
    int i,monotonicity_change_line=0;
    if(Right_Lost_Time >=0.9*IMAGE_HEIGHT)   return 1;//丢线数过多就返回1
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//起始行限幅
    if(end <=5) end=5;//终止行限幅
    if(start<=end)  return monotonicity_change_line;//如果起始行小于等于终止行，说明没有检测的必要
    for(i=start;i>=end;i--)//从下往上扫
    {
        /*如果右边线某一点在相邻上下5行的点中的列坐标最小（最靠左），那么就默认该点为单调点*/
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
    return monotonicity_change_line;//返回单调点所在的行数
}
/**
 * @brief 误差处理函数
 * @param 无
 * @return 对应的误差，左正右负
 */
float Err_Handle(void)
{
    /*????д??????????????????????
    float err=0.00;//????????????????????????
    int sum_err[IMAGE_HEIGHT]={0};
    int sum_hight=0;
    for(uint8 i=IMAGE_HEIGHT-1;i>=height;i--)
    {
        sum_err[i]=(right_line[i]+left_line[i])/2-94;
        sum_hight+=i;
    }
    for(uint8 i=IMAGE_HEIGHT-1;i>=height;i--)
    {
        err+=sum_err[i]/sum_hight;//???????
    }
    */
   /*误差消除方法：判断白列的位置，判断误差的正负；或者通过取平均来减小误差*/
    last_err=err;//上次的误差传递
    
    int weight_count=0;//权重计算
    for(int i=IMAGE_HEIGHT-1;i>IMAGE_HEIGHT/2;i--)
    {
        err+=(IMAGE_WIDTH/2-((left_line[i]+right_line[i])>>1))*Weight[i];
        weight_count+=Weight[i];//计算权重总和
    }
    err=err/weight_count;//计算误差
    if(abs(last_err-err)>=5)    err=last_err;//如果本次误差太大，就返回上次误差（防止部分元素误差突变）
    return err;
}

/**
 * @brief 左边界补线函数
 * @param int x1, int y1, int x2,int y2 起始点终止点的坐标
 * @return 在图像上对原来的左边线数组进行修改
 */
void Left_Add_Line(int x1, int y1, int x2,int y2)
{
    int i,max,a1,a2,hx;
    //???????????????????
    if(x1>=IMAGE_WIDTH) x1=IMAGE_WIDTH-1;//???????
    else if(x1<=0)  x1=0;
    if(x2>=IMAGE_WIDTH) x2=IMAGE_WIDTH-1;//???????
    else if(x2<=0)  x2=0;
    if(y1>=IMAGE_HEIGHT) y1=IMAGE_HEIGHT-1;//???????
    else if(y1<=0)  y1=0;
    if(y2>=IMAGE_HEIGHT) y2=IMAGE_HEIGHT-1;//???????
    else if(y2<=0)  y2=0;
    a1=y1;
    a2=y2;//????м???
    if(a1>a2)//????????a1?С??a2
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//???????????????в???
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;//?????????ж??????????
        if(hx >= IMAGE_WIDTH) hx = IMAGE_WIDTH-1;//???????
        else if(hx <= 0) hx = 0;
        left_line[i]=hx;//?????????????飨?????????????????????????????????????????ε?????????飬??????????Ч???????
    }
}

/**
 * @brief ????????
 * @param int x1, int y1, int x2,int y2 ???????????????????
 * @return ??????????????????????
 */
void Right_Add_Line(int x1, int y1, int x2, int y2)
{
    int i,max,a1,a2,hx;
    if(x1>=IMAGE_WIDTH) x1=IMAGE_WIDTH-1;//???????
    else if(x1<=0)  x1=0;
    if(x2>=IMAGE_WIDTH) x2=IMAGE_WIDTH-1;//???????
    else if(x2<=0)  x2=0;
    if(y1>=IMAGE_HEIGHT) y1=IMAGE_HEIGHT-1;//???????
    else if(y1<=0)  y1=0;
    if(y2>=IMAGE_HEIGHT) y2=IMAGE_HEIGHT-1;//???????
    else if(y2<=0)  y2=0;
    a1=y1;
    a2=y2;//????м???
    if(a1>a2)//????????a1?С??a2
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//???????????????в???
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;//?????????ж??????????
        if(hx >= IMAGE_WIDTH) hx = IMAGE_WIDTH-1;//???????
        else if(hx <= 0) hx = 0;
        right_line[i]=hx;//?????????????飨?????????????????????????????????????????ε?????????飬??????????Ч???????
    }
}

/**
 * @brief ????????????????????
 * @param int start, int end ?????????Χ???????
 * @return ????????????? Right_Down_Find=0;Left_Down_Find=0;
 */
void Find_Down_Point(int start, int end)
{
    int i,t;
    Right_Down_Find=0;
    Left_Down_Find=0;//?????????λ????
    if(start<end)//?????start?????end
    {
        t=start;
        start=end;
        end=t;
    }
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//????5?е?????????????????????????ж?
    if(end<=IMAGE_HEIGHT-Search_Stop_Line)  end=IMAGE_HEIGHT-Search_Stop_Line;//?????????
    if(end<=5)  end=5;
    /*????ж?????????????????????????????????????ж?仯???*/
    for(i=start;i>=end;i--)
    {
        if(Left_Down_Find == 0 && abs(left_line[i]-left_line[i+1])<=5 && abs(left_line[i+1]-left_line[i+2])<=5 &&
        abs(left_line[i+2]-left_line[i+3])<=5 && abs(left_line[i]-left_line[i-2])>=8 && abs(left_line[i]-left_line[i-2])>=15 &&
        abs(left_line[i]-left_line[i-4])>=15)//????????
        {
            Left_Down_Find=i;//????????????
        }
        if(Right_Down_Find == 0 &&abs(right_line[i]-right_line[i+1])<=5 && abs(right_line[i+1]-right_line[i+2])<=5 &&
        abs(right_line[i+2]-right_line[i+3])<=5 && abs(right_line[i]-right_line[i-2])>=8 && abs(right_line[i]-right_line[i-2])>=15
        &&abs(left_line[i]-left_line[i-4])>=15)
        {
            Right_Down_Find=i;//????????????
        }
        if(Left_Down_Find!=0 && Right_Down_Find!=0)    break;//????????????
    }
}

/**
 * @brief ???????????????????????
 * @param start:??????Χ?????end:??????Χ?????
 * @return ????????????????????Left_Up_Find??Right_Up_Find??
 */
void Find_Up_Point(int start, int end)
{
    int i,t;//?м????
    if(Left_Down_Find!=0)   Last_Left_Up_Find=Left_Down_Find;//记录上一次的左下点
    if(Right_Down_Find!=0)  Last_Right_Up_Find=Right_Down_Find;//记录上一次的右下点
    Left_Up_Find=0;//?????????λ????
    Right_Up_Find=0;//?????????λ????

    if(start<end)//?????start?????end
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=IMAGE_HEIGHT-Search_Stop_Line)  end=IMAGE_HEIGHT-Search_Stop_Line;//?????????
    if(end<=5)  end=5;
    if(start >=IMAGE_HEIGHT -1-5)   start=IMAGE_HEIGHT-1-5;//????5?е?????????????????????????ж?
    /*????????????????????????????????????????*/
    for(i=end;i<=start;i++)//????????????????
    {
        if(Left_Up_Find == 0 && 
        abs(left_line[i]-left_line[i-1])<=5 &&
        abs(left_line[i-1] -  left_line[i-2])<=5 &&
        abs(left_line[i-2] -  left_line[i-3])<=5 &&
        abs(left_line[i] - left_line[i+2])>=8 &&
        abs(left_line[i] - left_line[i+3])>=15 &&
        abs(left_line[i] - left_line[i+4])>=15)//????????
        {
            Left_Up_Find=i;//????????????
        }
        if(Right_Up_Find == 0 &&
        abs(right_line[i]-right_line[i-1]) <=5 &&
        abs(right_line[i-1]-right_line[i-2]) <=5 &&
        abs(right_line[i-2]-right_line[i-3]) <=5 &&
        abs(right_line[i]-right_line[i+2]) >=8 &&
        abs(right_line[i]-right_line[i+3]) >=15 &&
        abs(right_line[i]-right_line[i+4]) >=15)//????????
        {
            Right_Up_Find=i;//????????????
        }
        if(Left_Up_Find!=0 && Right_Up_Find!=0)    break;//????????????
    }
    if(abs(Right_Up_Find-Left_Up_Find)>=30&&left_line[Left_Up_Find]>=right_line[Right_Up_Find])//?????????????????????????
    {
        Right_Up_Find=0;
        Left_Up_Find=0;
    }
    // if(right_line[Right_Up_Find]<=left_line[Left_Up_Find])
    // {
    //     Right_Up_Find=Last_Right_Up_Find;
    // }
}

/**
 * @brief ???????
 * @param ????????????????????
 * @return null
 */
void Lengthen_Left_Boundry(int start, int end)
{
    int i,t;
    float k=0.0;
    if(start >=IMAGE_HEIGHT -1) start=IMAGE_HEIGHT-1; //?????λ?????
    else if(start <=0)  start=0;//???
    if(end>=IMAGE_HEIGHT-1) end=IMAGE_HEIGHT-1;//???
    else if(end<=0)  end=0;//???
    if(end < start) //?????end?????start
    {
        t=start;
        start=end;
        end=t;
    }
    if(start <=5)   Left_Add_Line(left_line[start],start,left_line[end],end);//??????????????????????????????????
    else
    {
        k=(float)(left_line[start]-left_line[start-4])/5.0; //?????k??1/б??
        for(i=start;i<=end;i++)
        {
            left_line[i]=(int)(i-start)*k+left_line[start];//???????
            if(left_line[i]>=IMAGE_WIDTH-1) left_line[i]=IMAGE_WIDTH-1;//???????
            else if(left_line[i]<=0) left_line[i]=0;//???????
        }
    }
}

/**
 * @brief ????????
 * @param ????????????????????
 * @return null
 */
void Lengthen_Right_Boundry(int start, int end)
{
    int i,t;
    float k=0.0;
    if(start >=IMAGE_HEIGHT -1) start=IMAGE_HEIGHT-1; //?????λ?????
    else if(start <=0)  start=0;//???
    if(end>=IMAGE_HEIGHT-1) end=IMAGE_HEIGHT-1;//???
    else if(end<=0)  end=0;//???
    if(end < start) //?????end?????start
    {
        t=start;
        start=end;
        end=t;
    }
    if(start <=5)   Right_Add_Line(right_line[start],start,right_line[end],end);//??????????????????????????????????
    else
    {
        k=(float)(right_line[start]-right_line[start-4])/5.0; //?????k??1/б??
        for(i=start;i<=end;i++)
        {
            right_line[i]=(int)(i-start)*k+right_line[start];//???????
            if(right_line[i]>=IMAGE_WIDTH-1) right_line[i]=IMAGE_WIDTH-1;//???????
            else if(right_line[i]<=0) right_line[i]=0;//???????
        }
    }
}

/**
 * @brief ?????
 * @param null
 * @return null
 */
void Cross_Detect(void)
{
    int down_search_start = 0;//???????濪?????????
    if(Road_Type == CROSSING)//?????????·??
    {
        Left_Up_Find=0;
        Right_Up_Find=0;
        if(Both_Lost_Time >=15)//???????????????
        {
            Find_Up_Point(110,6);//??????????????
            if(Left_Up_Find ==0 && Right_Up_Find ==0) return ;//?????????????
        }
        if(Left_Up_Find !=0 &&Right_Up_Find !=0)
        {
            down_search_start=Left_Up_Find>Right_Up_Find? Left_Up_Find:Right_Up_Find;//??????????????????
            Find_Down_Point(IMAGE_HEIGHT -5,down_search_start+2);//??????????????
            if(Left_Down_Find<=Left_Up_Find)    Left_Down_Find=0;//?????????????????棬?????????
            if(Right_Down_Find<=Right_Up_Find)  Right_Down_Find=0;//?????????????????棬?????????
            if(Left_Down_Find!=0 && Right_Down_Find!=0)//???????????????
            {
                Left_Add_Line(left_line[Left_Up_Find],Left_Up_Find,left_line[Left_Down_Find],Left_Down_Find);//??????
                Right_Add_Line(right_line[Right_Up_Find],Right_Up_Find,right_line[Right_Down_Find],Right_Down_Find);//??????
            }
            else if(Left_Down_Find == 0 && Right_Down_Find !=0)//б?????
            {
                Lengthen_Left_Boundry(Left_Up_Find-1,IMAGE_HEIGHT-1);//???????
                Right_Add_Line(right_line[Right_Up_Find],Right_Up_Find,right_line[Right_Down_Find],Right_Down_Find);//???粹??
            }
            else if (Left_Down_Find !=0 && Right_Down_Find ==0)//б?????
            {
                Lengthen_Right_Boundry(Right_Up_Find-1,IMAGE_HEIGHT-1);//???????
                Left_Add_Line(left_line[Left_Up_Find],Left_Up_Find,left_line[Left_Down_Find],Left_Down_Find);//???粹??
            }
            else if(Left_Down_Find == 0 && Right_Down_Find == 0)//??????
            {
                Lengthen_Left_Boundry(Left_Up_Find-1,IMAGE_HEIGHT-1);//???????
                Lengthen_Right_Boundry(Right_Up_Find-1,IMAGE_HEIGHT-1);//???????
            }
        }
    // }
    }
}

/**
 * @brief 检测黑白跳变点的个数
 * @param uint8 row 检测行数 uint8 start_column 检测行数的起始列 uint8 end_column 检测行数的终止列
 * @return 返回是否判断为斑马线
 */
uint8 Black_White_Dump(uint8 row,uint8 start_column,uint8 end_column)
{
    if(row>=IMAGE_HEIGHT-1) row=IMAGE_HEIGHT-1;//行数限幅
    else if(row<=0) row=0;//列数限幅
    if(row<=5)  return 0;//如果行数小于等于5，直接返回0
    if(start_column>=IMAGE_WIDTH-1) start_column=IMAGE_WIDTH-1;//起始列限幅
    else if(start_column<=0) start_column=0;//起始列限幅
    if(end_column>=IMAGE_WIDTH-1) end_column=IMAGE_WIDTH-1;//终止列限幅
    else if(end_column<=0) end_column=0;//终止列限幅
    if(row<=30) row=30;//防止后续数组越界
    else if(row>=89)  row=89;//防止后续数组越界
    uint8 count=0;
    uint8 count_for_temp=0;
    uint8 first_white_column=0;
    uint8 first_black_column=0;
    uint8 white_point_count=0;
    uint8 mode=0;
    for(uint8 i=start_column;i<=end_column;i++)
    {
        // if(mode==1)
        // {
        //     if(Image_Use[row][i]==WHITE_POINT&&first_white_column==0)//只有重置后方可重新记录
        //     {
        //         first_white_column=i;//记录第一个白点的列坐标
        //     }
        //     else if(Image_Use[row][i]==BLACK_POINT&&first_black_column==0)
        //     {
        //         first_black_column=i;//记录第一个黑点的列坐标
        //         if(abs(first_black_column-first_white_column)<=10)//这个阈值可以调整，可以通过输入row的所在行转化到对应列的距离
        //         {
        //             first_white_column=0;//在成功记录后重置坐标
        //             first_black_column=0;
        //             count++;
        //             if(count>=5)   return count;//如果黑白跳变点的个数大于等于5，直接返回
        //         }
        //     }
        // }
        // else if(mode==0)
        // {
        //     if(Image_Use[row][i]==WHITE_POINT)  white_point_count++;//计算白色点的值
        // }
        
        if(Image_Use[row][i]==WHITE_POINT)  
        {
            ips114_draw_point(i,row,RGB565_BLUE);
            white_point_count++;//计算白色点的值
        }
        
    }
    
    ips114_show_uint(188,60,row,3);
    ips114_show_uint(188,80,end_column-start_column,3);
    ips114_show_uint(188,100,white_point_count,3);
    ips114_show_uint(188,120,abs(white_point_count-(end_column-start_column)),3);
    if(mode==0)
    {
        if(abs(white_point_count-(end_column-start_column))<=Zebra[row])//如果白色点的个数和列数的差值小于等于（这个阈值要修改成可自动化调整的）
        {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief ???????
 * @param ??
 * @return ??
 */
uint8 Island_State=0;
void Island_Detect(void)
{
    static int state1_down_guai[2]={0};//??1????
    static int state1_up_guai[2]={0};//??1????
    int monotonicity_change_left_flag=0;
    int monotonicity_change_right_flag=0;//??????????
    int continuity_change_left_flag=0;//????????????
    int continuity_change_right_flag=0;//????????????

}
/**
 * @brief 斑马线检测（待改进，可以计算出距离斑马线的大概的距离）
 * @param 无
 * @return 无
 * @attention 一般在直道上进行检测
 */
void Zebra_Stripes_Detect(void)
{
    int continuity_change_right_flag =0;//右边线连续性标志位
    int continuity_change_left_flag =0;//左边线连续性标志位
    int monotonicity_change_right_flag =0;//右边线单调性标志位
    int monotonicity_change_left_flag =0;//左边线单调性标志位
    
    continuity_change_left_flag = Continuity_Change_Left(IMAGE_HEIGHT-1,5,0);//求出左边线不连续性点的行坐标
    continuity_change_right_flag = Continuity_Change_Right(IMAGE_HEIGHT-1,5,0);//求出右边线不连续点的行坐标
    monotonicity_change_left_flag = Continuity_Change_Left(IMAGE_HEIGHT-1,5,1);//求出左边单调性变化的行坐标
    monotonicity_change_right_flag = Continuity_Change_Right(IMAGE_HEIGHT-1,5,1);//求出右边单调性变化的行坐标
    if(continuity_change_left_flag<=30||continuity_change_right_flag<=30)   
    {
        if(Road_Type==BANMAXIAN) Road_Type=STRAIGHT_ROAD;//从斑马线切回到直道
        return ;
    }
    ips114_draw_line(94,60,left_line[continuity_change_left_flag],continuity_change_left_flag,RGB565_RED);
    ips114_draw_line(98,0,left_line[monotonicity_change_left_flag],monotonicity_change_left_flag,RGB565_BLUE);
    // ips114_show_uint(188,60,continuity_change_right_flag,3);
    // ips114_show_uint(188,80,continuity_change_left_flag,3);
    
    int i=0,j=0,change_count=0,start_line=0,endl_line=0,narrow_road_count=0;
    if(Search_Stop_Line >=60 && 30<=Longest_White_Column_Left[1] && Longest_White_Column_Left[1]<=IMAGE_WIDTH-30  &&
    abs(continuity_change_left_flag-continuity_change_right_flag)<=30 &&continuity_change_left_flag!=0 &&continuity_change_right_flag!=0)
    {
        uint8 count=Black_White_Dump(continuity_change_left_flag-3,left_line[continuity_change_left_flag-3],right_line[continuity_change_left_flag-3]);
        uint8 higher_flag= (continuity_change_left_flag < continuity_change_right_flag) ? continuity_change_left_flag : continuity_change_right_flag; // 如果A小于B，那么C的值为A，否则C的值为B
        uint8 lower_flag= (monotonicity_change_left_flag > monotonicity_change_right_flag) ? monotonicity_change_left_flag : monotonicity_change_right_flag; // 如果A大于B，那么C的值为A，否则C的值为B
        if(Black_White_Dump((higher_flag+lower_flag)/2,left_line[higher_flag]-5,right_line[higher_flag]-5))
        {
            Road_Type=BANMAXIAN;
            // if(Road_Type==STRAIGHT_ROAD)    Road_Type=BANMAXIAN;//斑马线是在直道的基础上进行判断的（但是一定要归类回直道，但是只用跑一圈似乎也没必要）
        }
    }
}

void test2(void)
{
    uint8 type=0;
    if(Road_Type==STRAIGHT_ROAD)    type=1;
    else if(Road_Type==RIGHT_TURN)  type=2;
    else if(Road_Type==LEFT_TURN)   type=3;
    else if(Road_Type==CROSSING)    type=4;
    else if(Road_Type==BANMAXIAN)   type=5;
    if(Road_Type==CROSSING) Cross_Detect();
    for(uint8 i=0;i<=IMAGE_HEIGHT-1;i++)
    {
        // ips114_draw_point((left_line[i]+right_line[i])/2,i,RGB565_RED);
        ips114_draw_point(left_line[i],i,RGB565_BLUE);
        // ips114_draw_point(right_line[i],i,RGB565_GREEN);
    }
    // ips114_draw_line(158,80,left_line[Left_Up_Find],Left_Up_Find,RGB565_PURPLE);
    // ips114_draw_line(98,60,right_line[Right_Up_Find],Right_Up_Find,RGB565_BLUE);
//    ips114_show_uint(188,120,threshold,3);      
	ips114_displayimage03x(*Image_Use,188,120);
	ips114_show_uint(188,0,Longest_White_Column_Left[1],3);
    float my_err=Err_Handle();
    ips114_show_float(188,20,my_err,2,2);
    ips114_show_uint(188,40,type,3);
    
    
    
}

/**
 * @brief ??????????????????????
 * @param ??
 * @return ??
 */
void test(void)
{
    uint8 mode=0;//模式为1表示为大津法，模式为2表示为边缘检测算子
    
    if(mode==1)
    {
        Image_Change();
        uint8 threshold=OSTU_GetThreshold((uint8 *)mt9v03x_image,IMAGE_WIDTH,IMAGE_HEIGHT);
        Simple_Binaryzation(*Image_Use,threshold);
        Center_line_deal(5,183);//?????????
    }
    else if(mode==0)
    {
        uint8 *output_address;//?????????
        output_address=Scharr_Edge(*mt9v03x_image);
		uint8 threshold=OSTU_GetThreshold((uint8 *)mt9v03x_image,IMAGE_WIDTH,IMAGE_HEIGHT);
        memcpy(Image_Use,output_address,IMAGE_HEIGHT*IMAGE_WIDTH*sizeof(uint8));
		Simple_Binaryzation(*Image_Use,threshold);
        Center_line_deal_plus(23,163);//Cannot set too high or too low boundary, otherwise it will cause an error
        Outer_Analyse();//Analyze the elements of the edge line array
    }
    
	test2();
  
}
