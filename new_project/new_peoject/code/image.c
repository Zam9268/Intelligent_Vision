#include "image.h"
#include "stdbool.h"

uint8 Image_Use[IMAGE_HEIGHT][IMAGE_WIDTH];

/*???????????????*/
uint8 left_line[IMAGE_HEIGHT],right_line[IMAGE_HEIGHT];//????????�?????????
int center[IMAGE_HEIGHT];//???????��?????��????
uint8 the_maxlen_position;//????????
uint8 num;//??????????��??
uint8 Longest_White_Column_Left[2]; // Record the longest white column in this iteration
uint8 Last_Longest_White_Column_Left[2]; // Record the longest white column in the previous iteration to prevent white column fluctuations in some areas
uint8 Left_Line_Start, Right_Line_Start; // Starting point of the left and right lines
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
uint8 Last_Right_Up_Find=0;//the last time the right up point is found
uint8 Right_Up_Find = 0; // Finding the right top turning point
uint8 flag_test=0;
uint8 pick_up_mode=0;//when the value is 1, it is in the card picking state; when the value is 0, it is in the free patrol state
float Left_derivative[IMAGE_HEIGHT]={0.0};
float Right_derivative[IMAGE_HEIGHT]={0.0};
float err=0.00;
float last_err=0.00;

/*the following is the information for receiving data through the serial port*/
extern uint8 right_data[64];//store the data received from the serial port,it only store 64 bytes  
extern uint32 fifo_data_count;//the number of data lied in the buffer
extern uint8 data_length;//the length of the data received
extern uint8 i;//the state of get data,this is unuseful
extern int count;//this is unuseful
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

/*Zebra*/
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
    uint8 HistGram[257] = {0}; // ???????????? 257
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
 * @brief ?????
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
volatile int White_Column[IMAGE_WIDTH];//???????????
/**
 * @brief This longest white line patrol can only be used to process images of Otsu method,it is not applicable to images after edge detection
 * @param start_column, end_column: The starting and ending columns for finding the longest white column
 * @return None (actually returns the edge line array)
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
    uint8 middle=the_maxlen_position;//??????????????
    uint8 x_num;
    /*?????????????????? */
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
    Longest_White_Column_Left[0]=0;//????????????
    for(uint8 i=start_column;i<=end_column;i++)
    {
        if(White_Column[i]>Longest_White_Column_Left[0])//????????
        {
            Longest_White_Column_Left[0]=White_Column[i];
            Longest_White_Column_Left[1]=i;
        }
    }
    /*????????????????*/
    Longest_White_Column_Right[0]=0;//????????????
    for(uint8 i=end_column;i>start_column;i--)
    {
        if(White_Column[i]>Longest_White_Column_Right[0])//??????????
        {
            Longest_White_Column_Right[0]=White_Column[i];
            Longest_White_Column_Right[1]=i;//??????????????????????
        }
    }
    /*?????????*/
    Search_Stop_Line=Longest_White_Column_Left[0];//??????????????
    int right_border,left_border;//?????????????
    for(int i=IMAGE_HEIGHT-1;i>=IMAGE_HEIGHT-Search_Stop_Line;i--)
    {
        /*????????*/
        for(int j=Longest_White_Column_Left[1];j<=IMAGE_WIDTH-1;j++)
        {
            if(Image_Use[i][j]==WHITE_POINT&&Image_Use[i][j+1]==BLACK_POINT&&Image_Use[i][j+2]==BLACK_POINT)
            {
                right_border=j;//???????????
                Right_Lost_Flag[i]=0;//????????????0
                break;
            }
            else if(j>=IMAGE_WIDTH-1-2)//??????????????????????x??????????????????1
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
                Left_Lost_Flag[i]=0;//????????????0
                break;
            }
            else if(j<=2)//??????????????????????x??????????????????1
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
    begin:
    
    for(uint8 i=0;i<IMAGE_HEIGHT-1;i++)
    {
        left_line[i]=0;
        right_line[i]=0;
        Right_Lost_Flag[i]=0; // Clear the right line lost flag to 0
        Left_Lost_Flag[i]=0; // Clear the left line lost flag to 0
    }
    Left_Lost_Time=0;//????????????
    Right_Lost_Time=0;//?????????????
    Both_Lost_Time=0;//??????????????????
    Boundry_Start_Left=0;
    Boundry_Start_Right=0;//????????????
    /* Reset white column count */
    for(uint8 i=0;i<=IMAGE_WIDTH-1;i++)
    {
        White_Column[i]=0;
    }
    /*Counting white columns*/
    for(uint8 j=start_column;j<=end_column;j++)
    {
        for(uint8 i=IMAGE_HEIGHT-3;i>=0;i--)//???????????????????????????????
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
        if(Image_Use[119][j]==BLACK_POINT&&Image_Use[118][j]==BLACK_POINT)//???????��?��??????????��????????
        {
            White_Column[j]=0;//???????????
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
    uint8 left_start_flag=0;
    uint8 right_start_flag=0;
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
                if(right_start_flag==0)
                {
                    if(Right_Lost_Flag[i-1]==1)      //?????��???????????????????????????
                    {
                        Right_Line_Start=i;
                        right_start_flag=1;
                    }
                }
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
    /*?????��???????????????????????????????????*/
    Outer_Analyse();
    if(Longest_White_Column_Left[1]<=60&&Left_Lost_Time>=60&&Right_Lost_Time<=5)
    {
        if(right_line[Boundry_Start_Right]<=(IMAGE_WIDTH/2))//????????????
        {
            Last_Longest_White_Column_Left[1]=94;
	        Longest_White_Column_Left[1]=94;
            flag_test++;
            goto begin;//???????????????????????
        }
    }
    else if(Longest_White_Column_Left[1]>=128&&Right_Lost_Time>=60&&Left_Lost_Time<=5)
    {
        if(left_line[Boundry_Start_Left]>=(IMAGE_WIDTH/2))//????????????
        {
            Last_Longest_White_Column_Left[1]=94;
	        Longest_White_Column_Left[1]=94;
            flag_test++;
            goto begin;//???????????????????????
        }
    }
}

/**
 * @brief ?????????????????????????????
 * @param ??
 * @return ????????8?????????????????????????????????????8-?????
 * @attention ??
 */
uint8 Get_White_Point(uint8 x,uint8 y)
{
    if(x<=1|| x>=IMAGE_WIDTH-2|| y<=1|| y>=IMAGE_HEIGHT-2)    return 0;//???????
    uint8 white_point=0;
    for(uint8 i=x-1;i<=x+1;i++)
    {
        for(uint8 j=y-1;j<=y+1;j++)
        {
            if(Image_Use[j][i]==WHITE_POINT)    white_point++;
        }
    }
    return white_point;//???????????????????
}

/**
 * @brief ???????????????
 * @param ??
 * @return ??
 * @attention ??
 */
void Border_Car_Detect(void)
{
    
}

int deviation[8][2]={{0,-1},{-1,-1},{-1,0},{-1,1},{0,1},{1,1},{1,0},{1,-1}};//??????x??????????y????
int devitation_right[8][2]={{0,-1},{1,-1},{1,0},{1,1},{0,1},{-1,1},{-1,0},{-1,-1}};//???????????
struct Line_Edge
{
    uint8 row;//??
    uint8 column;//??
    uint8 flag;//???????��
    uint8 grow;//????????
};
struct Line_Edge left_edge[80];
struct Line_Edge right_edge[80];
/**
 * @brief Function for searching the center
 * @param None
 * @return None
 * @attention Please note the following:
 *              1. The function assumes certain conditions for the image to be detected.
 *              2. The function performs specific operations based on the detection mode.
 */
void Search_Center(void)
{
    uint8 my_detect_mode=1;
    if(my_detect_mode==0)//If detection mode is 0
    {
        uint8 Top_h=0,Top_w=0,Bottom_h=0,Botton_w=0;//Variables for storing coordinates
        uint8 ter_h =100,ter_w=60,Mid_h=0,Mid_w=0;
        float cam_dis_h=0,cam_dis_w=0;
        for(uint8 i=30;i<IMAGE_HEIGHT-2;i++)//Loop through the image
        {
            for(uint8 j=2;j<IMAGE_WIDTH-2;j++)
            {
                 if (Image_Use[i][j] == 255 && (Image_Use[i - 2][j] == 0 && Image_Use[i - 2][j - 1] == 0 && Image_Use[i][j - 1] == 0) 
                 && (Image_Use[i][j + 1] == 255 && Image_Use[i][j + 3] == 255 && Image_Use[i][j + 6] == 255 && Image_Use[i][j + 9] == 255) && 
                 Image_Use[i + 1][j] == 255 && Image_Use[i + 3][j] == 255 
                 && Image_Use[i + 1][j - 5] == 0 && Image_Use[i + 2][j - 10] == 0 && Image_Use[i + 3][j - 10] == 0 && Image_Use[i + 2][j - 15] == 0)
            {
                Top_h = i;
                Top_w = j;
                break;
            }
            }

        }
    }
    else if(my_detect_mode==1)//If detection mode is 1,this detection is wrote by me
    {
        uint8 lower_black_row=0;//count the number of black points in a row
        for(uint8 i=IMAGE_HEIGHT-20;i>=30;i--)
        {
            uint8 count_for_black_point=0;
            for(uint8 j=0;j<=IMAGE_WIDTH-1;j++)
            {
                if(Image_Use[i][j]==BLACK_POINT)
                {
                    count_for_black_point++;
                }
            }
            if(count_for_black_point==188)//自下而上寻找最近黑列
            {
                lower_black_row=i;
                break;
            }
        }
        /*针对斜边形状卡片，当从左边和从右边扫到的顶点一样的话，就说明可以*/
        /*扫描方法：从起始行的左列，右列向中间扫，当扫到白点的时候就记录该点位置，记录每行的点对应的列坐标
        然后从上到下优先取坐标靠上的，当该坐标满足条件：1.和下5行的列坐标相近 2.且和下2个坐标相连（）*/
        uint8 left_count[IMAGE_HEIGHT/2]={0};
        uint8 right_count[IMAGE_HEIGHT/2]={0};
        uint8 end_row;
        if(abs(IMAGE_HEIGHT-20-lower_black_row)>=60)    end_row=lower_black_row+60;
        else  end_row=IMAGE_HEIGHT-20;//限幅处理
        for(uint8 i=lower_black_row;i<=end_row;i++)
        {
            /*先从左边界扫到右边*/
            for(uint8 j=20;j<IMAGE_WIDTH-1-20;j++)
            {
                if(Image_Use[i][j]==WHITE_POINT&&Image_Use[i][j-1]==BLACK_POINT&&Image_Use[i][j-2]==BLACK_POINT)
				{
                    left_count[i-lower_black_row]=j;//记录对应的列坐标
                    break;//找到就直接退出
				}
                else
                {
                    left_count[i-lower_black_row]=0;//找不到就记录为0
                }
            }
            /*再从右边界扫到左边*/
            for(uint8 j=IMAGE_WIDTH-1-20;j>=20;j--)
            {
                if(Image_Use[i][j]==WHITE_POINT&&Image_Use[i][j+1]==BLACK_POINT&&Image_Use[i][j+2]==BLACK_POINT)
                {
                    right_count[i-lower_black_row]=j;//记录对应的列坐标
                    break;//找到就直接退出
                }
                else
                {
                    right_count[i-lower_black_row]=IMAGE_WIDTH-1-20;//找不到就记录为0
                }
            }
        }
        // for(uint8 i=0;i<=end_row-lower_black_row;i++)
        // {
        //     ips114_draw_point(left_count[i],i+lower_black_row,RGB565_RED);
        //     ips114_draw_point(right_count[i],i+lower_black_row,RGB565_BLUE);
        // }
        /*开始寻找边界点*/
        uint8 left_up_point[2]={0};//左上角拐点坐标
        uint8 right_up_point[2]={0};//右下角拐点坐标
        for(uint8 i=0;i<=end_row-lower_black_row;i++)
        {
            if(abs(left_count[i]-left_count[i+1])<=1&&abs(left_count[i]-left_count[i+2])<=2&&abs(left_count[i]-left_count[i+3])<=3&&
            abs(left_count[i]-left_count[i+4])<=4&&abs(left_count[i]-left_count[i+5])<=5)
            {
                left_up_point[0]=left_count[i];//记录列坐标
                left_up_point[1]=i+lower_black_row;//记录行坐标
                break;
            }
        }
        for(uint8 i=0;i<=end_row-lower_black_row;i++)
        {
            if(abs(right_count[i]-right_count[i+1])<=1&&abs(right_count[i]-right_count[i+2])<=2&&abs(right_count[i]-right_count[i+3])<=3&&
            abs(right_count[i]-right_count[i+4])<=4&&abs(right_count[i]-right_count[i+5])<=5)
            {
                right_up_point[0]=right_count[i];//记录列坐标
                right_up_point[1]=i+lower_black_row;//记录行坐标
                break;
            }
        }
        ips114_show_uint(188,20,right_up_point[0],3);
        ips114_show_uint(188,40,right_up_point[1],3);
        ips114_show_uint(188,60,left_up_point[0],3);
        ips114_show_uint(188,80,left_up_point[1],3);
        ips114_draw_line(0,0,left_up_point[0],left_up_point[1],RGB565_RED);
        ips114_draw_line(0,0,right_up_point[0],right_up_point[1],RGB565_BLUE);
        // ips114_draw_line(0,0,,RGB565_RED);
        // ips114_draw_line(0,0,right_count[0],right_count[1],RGB565_BLUE);
        /*
        uint8 start_x=94,start_y=60;//Starting coordinates
        for(uint8 i=start_y;i<=IMAGE_HEIGHT-1;i++)
        {
            if(Image_Use[i][start_x]==WHITE_POINT&&Image_Use[i-1][start_x]==BLACK_POINT&&Image_Use[i-2][start_x]==BLACK_POINT
            &&Image_Use[i-5][start_x]==BLACK_POINT&&Image_Use[i-3][start_x-3]==BLACK_POINT&&Image_Use[i-3][start_x+3]==BLACK_POINT
            &&Get_White_Point(start_x,i)>=5)//Check if the pixel meets certain conditions
            {
                start_y=i;//Update the starting y-coordinate
                break;
            }
        }
        uint8 temp_x=start_x,temp_y=start_y;
        uint8 search_cout=80;
        uint8 my_count_left=0,my_count_right=0;
        while(search_cout--)
        {
            for(uint8 i=0;i<=7;i++)//Loop through the directions
            {
                if(Image_Use[start_x+deviation[i][0]][start_y+deviation[i][1]]==WHITE_POINT)
                {
                    start_x=start_x+deviation[i][0];
                    start_y=start_y+deviation[i][1];
                    left_edge[my_count_left].row=start_y;
                    left_edge[my_count_left].column=start_x;
                    left_edge[my_count_left].flag=1;
                    left_edge[my_count_left].grow=i;
                    my_count_left++;
                    break;
                }
                if(i==7)//If no white pixel is found in any direction
                {
                    goto end;
                }
            }
        }
        end:
        search_cout=80;
        while(search_cout--)
        {
            for(uint8 i=0;i<=7;i++)
            {
                if(Image_Use[temp_x+devitation_right[i][0]][temp_y+devitation_right[i][1]]==WHITE_POINT)
                {
                    temp_x=temp_x+devitation_right[i][0];
                    temp_y=temp_y+devitation_right[i][1];
                    right_edge[my_count_right].row=temp_y;
                    right_edge[my_count_right].column=temp_x;
                    right_edge[my_count_right].flag=1;
                    right_edge[my_count_right].grow=i;
                    my_count_right++;
                    break;
                }
                if(i==7)
                {
                    goto finnal_end;
                }
            }
			finnal_end: break;
        }
        
        //Calculate the center coordinates of the left and right edges
        int left_center_x=0;
		int left_center_y=0;
		int right_center_x=0;
		int right_center_y=0;
        for(uint8 i=5;i<=my_count_left-6;i++)
        {
            if(abs(left_edge[i].row-left_edge[i-5].row)<=2&&abs(left_edge[i].column-left_edge[i-5].column)>=4
            &&abs(left_edge[i].row-left_edge[i+5].row)>=4&&abs(left_edge[i].column-left_edge[i+5].column)<=2)
            {
                left_center_x=left_edge[i].column;
                left_center_y=left_edge[i].row;
                break;
            }
        }
        for(uint8 i=5;i<=my_count_right-6;i++)
        {
            if(abs(right_edge[i].row-right_edge[i-5].row)<=2&&abs(right_edge[i].column-right_edge[i-5].column)>=4
            &&abs(right_edge[i].row-right_edge[i+5].row)>=4&&abs(right_edge[i].column-right_edge[i+5].column)<=2)
            {
                right_center_x=right_edge[i].column;
                right_center_y=right_edge[i].row;
                break;
            }
        }
        */
        
        /*Perform further operations based on the calculated center coordinates*/
    }
}

/**
 * @brief Easy filtering function
 * @param uint8 start_row: starting row index, uint8 end_row: ending row index, 
 *        uint8 start_column: starting column index, uint8 end_column: ending column index, 
 *        uint8 threshold: threshold value
 * @return None
 * @attention Make sure start_row > end_row and start_column < end_column
 */
void Easy_Filtering(uint8 start_row, uint8 end_row, uint8 start_column, uint8 end_column, uint8 threshold)
{
    for(uint8 i = start_row - 1; i >= end_row + 1; i--) // Iterate through the rows
    {
        for(uint8 j = start_column + 1; j <= end_column - 1; j++) // Iterate through the columns
        {
            if(Image_Use[i-1][j-1] + Image_Use[i-1][j] + Image_Use[i-1][j+1] + Image_Use[i][j-1]
                + Image_Use[i][j+1] + Image_Use[i+1][j-1] + Image_Use[i+1][j] + Image_Use[i+1][j+1] >= threshold * WHITE_POINT)
            {
                Image_Use[i][j] = WHITE_POINT; // Set the pixel to white
            }
        }
    }
}
/**
 * @brief the function Edge array analysis
 * @param none
 * @return none
 */
void Outer_Analyse(void)
{
    static uint8 my_init_flag=0;//????????��???????????
    /*???????????��?????*/
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        if(Left_Lost_Flag[i]==1)    Left_Lost_Time++;
        if(Right_Lost_Flag[i]==1)   Right_Lost_Time++;
        if(Left_Lost_Flag[i]==1&&Right_Lost_Flag[i]==1)   Both_Lost_Time++;
        if(Boundry_Start_Left==0&&Left_Lost_Flag[i]==0)   Boundry_Start_Left=i;// Record the starting point of the left boundary
        if(Boundry_Start_Right==0&&Right_Lost_Flag[i]==0) Boundry_Start_Right=i;// Record the starting point of the right boundary
        Road_Wide[i]=right_line[i]-left_line[i];// Record the road width
    }
    if(my_init_flag==0)//????��?????????????????????????��?
    {
        /* Preliminary analysis of different flags for track elements */
        if(Left_Lost_Time<=15&&Right_Lost_Time<=15&&Both_Lost_Time<=15) Road_Type=STRAIGHT_ROAD;
        if(Left_Lost_Time<15&&Right_Lost_Time>=30&&Both_Lost_Time<15&&Search_Stop_Line<=100)   Road_Type=RIGHT_TURN;
        if(Right_Lost_Time<15&&Left_Lost_Time>=30&&Both_Lost_Time<15&&Search_Stop_Line<=100)   Road_Type=LEFT_TURN;
        if(Left_Lost_Time>=15&&Right_Lost_Time<=5&&Both_Lost_Time<=5&&Search_Stop_Line>=100)    Road_Type=LEFT_HUANDAO;
        if(Left_Lost_Time<=5&&Right_Lost_Time>=15&&Both_Lost_Time<=5&&Search_Stop_Line>=100)    Road_Type=RIGHT_HUANDAO;
        if(Right_Lost_Time>=30&&Left_Lost_Time>=30&&Both_Lost_Time>=30) Road_Type=CROSSING;
        my_init_flag++;
    }
    if(Road_Type!=RAMP)
    {
        if(Left_Lost_Time<=15&&Right_Lost_Time<=15&&Both_Lost_Time<=15) Road_Type=STRAIGHT_ROAD;
        if(Left_Lost_Time<15&&Right_Lost_Time>=30&&Both_Lost_Time<15&&Search_Stop_Line<=100)   Road_Type=RIGHT_TURN;
        if(Right_Lost_Time<15&&Left_Lost_Time>=30&&Both_Lost_Time<15&&Search_Stop_Line<=100)   Road_Type=LEFT_TURN;
        if(Left_Lost_Time>=15&&Right_Lost_Time<=5&&Both_Lost_Time<=5&&Search_Stop_Line>=100)    Road_Type=LEFT_HUANDAO;
        if(Left_Lost_Time<=5&&Right_Lost_Time>=15&&Both_Lost_Time<=5&&Search_Stop_Line>=100)    Road_Type=RIGHT_HUANDAO;
        if(Right_Lost_Time>=30&&Left_Lost_Time>=30&&Both_Lost_Time>=30) Road_Type=CROSSING;
    }
    if(Road_Type==STRAIGHT_ROAD)    Ramp_Detect();
    if(Road_Type==STRAIGHT_ROAD)    Zebra_Stripes_Detect();
    if(Road_Type==RAMP)   Ramp_Detect();//??????
}

/**
 * @brief Function for continuity change detection on the left side
 * @param start: starting index, end: ending index, mode: detection mode
 * @return Continuity change flag
 */
int Continuity_Change_Left(int start, int end, int mode)
{
    int i, t, continuity_change_flag = 0;
    if (Left_Lost_Time >= 0.9 * IMAGE_HEIGHT) return 1; // Return 1 if left line is lost for more than 90% of the image height
    if (Search_Stop_Line <= 5) return 1; // Return 1 if search stop line is less than or equal to 5
    if (start >= IMAGE_HEIGHT - 1 - 5) start = IMAGE_HEIGHT - 1 - 5; // Adjust start index if it exceeds the image height
    if (end <= 5) end = 5; // Adjust end index if it is less than or equal to 5
    if (start < end) {
        t = start;
        start = end;
        end = t;
    }
    if (mode == 0) {
        for (i = start; i >= end; i--) {
            if (abs(left_line[i] - left_line[i - 1]) >= 5 && left_line[i - 1] >= 20 && left_line[i - 3] >= 20) {
                continuity_change_flag = i;
                break;
            }
        }
    } else if (mode == 1) {
        for (i = end; i <= start; i++) {
            if (abs(left_line[i] - left_line[i - 1]) >= 5 && left_line[i - 1] >= 20 && left_line[i - 3] >= 20) {
                continuity_change_flag = i;
                break;
            }
        }
    }
    return continuity_change_flag;
}

/**
 * @brief ?????????????
 * @param start:????? end:?????
 * @return ???????????????
 */
int Continuity_Change_Right(int start, int end,int mode)
{
    int i,t,continuity_change_flag=0;
    if(Right_Lost_Time >=0.9*IMAGE_HEIGHT)   return 1;//??????????????1
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//????????
    if(end <=5) end=5;//????????
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
            if(abs(right_line[i]-right_line[i-1])>=5)//?????????????????5
            {
                continuity_change_flag=i;
                break;//????????????��????????
            }
        }
    }
    else if(mode==1)
    {
        for(i=end;i>=start;i++)
        {
            if(abs(right_line[i]-right_line[i-1])>=5)//?????????????????5
            {
                continuity_change_flag=i;
                break;//????????????��????????
            }
        }
    }
    return continuity_change_flag;//????0?????��????????��??????????????��?????????
}

/**
 * @brief ????????
 * @param line:??????????
 * @return ??????????????
 */
uint8 Continuity_detect(uint8 *line)
{
    uint8 max_uncontinuity=0;//????????
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        if(line[i]-line[i-1]>max_uncontinuity)
        {
            max_uncontinuity=line[i]-line[i-1];
        }
    }
    return max_uncontinuity;//????????????
}

/**
 * @brief ???????????��????��??????????
 * @param ??
 * @return ??
 * @attention ??
 */
void Derivative_Change(void)
{
    for(uint8 i=IMAGE_HEIGHT-1;i>=1;i--)
    {
        Left_derivative[i]=(left_line[i]-left_line[i-1])/2;
        Right_derivative[i]=(right_line[i]-right_line[i-1])/2;//???????????��?
    }
}

/**
 * @brief ??????????????????????????
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
 * @brief ????????????????????????????
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
 * @brief ???�n??????
 * @param int start ?????, int end ?????
 * @return ?????????��??????
 * @attention ??
 */
int Monotonicity_Change_Left(int start, int end)
{
    int i,monotonicity_change_line=0;
    if(Left_Lost_Time>=0.9*IMAGE_HEIGHT)   return 1;//??????????????1
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//????????
    if(end<=5) end=5;//????????
    if(start<=end)  return 1; //????????��?????????��??????��?????
    for(i=start;i>=end;i--)
    {
        /*??????????????????????5?��???��??????????????????????????????????*/
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
    return monotonicity_change_line;//??????????????????
}

/**
 * @brief ???�n??????
 * @param int start ?????, int end ?????
 * @return ?????????��??????
 * @attention ??
 */
int Monotonicity_Change_Right(int start,int end)
{
    int i,monotonicity_change_line=0;
    if(Right_Lost_Time >=0.9*IMAGE_HEIGHT)   return 1;//??????????????1
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//????????
    if(end <=5) end=5;//????????
    if(start<=end)  return monotonicity_change_line;//????????��?????????��??????��?????
    for(i=start;i>=end;i--)//?????????
    {
        /*??????????????????????5?��???��?????????��????????????????????????*/
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
    return monotonicity_change_line;//??????????????????
}
/**
 * @brief ?????????
 * @param ??
 * @return ???????????????
 */
float Err_Handle(void)
{
    /*????????????????????????????
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
   /*??????????????��???��?��????��??????????????????????????��???*/
    last_err=err;//??��??????
    
    int weight_count=0;//??????
    for(int i=IMAGE_HEIGHT-1;i>IMAGE_HEIGHT/2;i--)//?????int i=IMAGE_HEIGHT-1;i>IMAGE_HEIGHT/2;i--?????????????????
    {
        err+=(IMAGE_WIDTH/2-((left_line[i]+right_line[i])>>1))*Weight[i];
        weight_count+=Weight[i];//??????????
    }
    err=err/weight_count;//???????
//    if(last_err==0&&err==0)
//    {
//        return err;
//    }
//    else   
//    {
//        if((abs(last_err-err)>=15)&&Road_Type==CROSSING)    err=last_err;//???????????????????????????????????????)
//    } 
    return err;
}

/**
 * @brief ???��?????
 * @param int x1, int y1, int x2,int y2 ???????????????
 * @return ????????????????????????????
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
    a2=y2;//?????????
    if(a1>a2)//????????a1?????a2
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//????????????????????
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;//?????????????????????
        if(hx >= IMAGE_WIDTH) hx = IMAGE_WIDTH-1;//???????
        else if(hx <= 0) hx = 0;
        left_line[i]=hx;//???????????????????????????????????????????????????????????????????????????????????????
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
    a2=y2;//?????????
    if(a1>a2)//????????a1?????a2
    {
        max=a1;
        a1=a2;
        a2=max;
    }
    for(i=a1;i<=a2;i++)//????????????????????
    {
        hx=(i-y1)*(x2-x1)/(y2-y1)+x1;//?????????????????????
        if(hx >= IMAGE_WIDTH) hx = IMAGE_WIDTH-1;//???????
        else if(hx <= 0) hx = 0;
        right_line[i]=hx;//???????????????????????????????????????????????????????????????????????????????????????
    }
}

/**
 * @brief ????????????????????
 * @param int start, int end ??????????????????
 * @return ????????????? Right_Down_Find=0;Left_Down_Find=0;
 */
void Find_Down_Point(int start, int end)
{
    int i,t;
    Right_Down_Find=0;
    Left_Down_Find=0;//???????????????
    if(start<end)//?????start?????end
    {
        t=start;
        start=end;
        end=t;
    }
    if(start >=IMAGE_HEIGHT-1-5)    start=IMAGE_HEIGHT-1-5;//????5???????????????????????????????
    if(end<=IMAGE_HEIGHT-Search_Stop_Line)  end=IMAGE_HEIGHT-Search_Stop_Line;//?????????
    if(end<=5)  end=5;
    /*???????????????????????????????????????????????????*/
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
        &&abs(left_line[i]-left_line[i-4])>=15)//?????????????????????��?????
        {
            Right_Down_Find=i;//????????????
        }
        if(Left_Down_Find!=0 && Right_Down_Find!=0)    break;//????????????
    }
}

/**
 * @brief the function to find the upper point
 * @param start: the starting index, end: the ending index
 * @return null
 */
void Find_Up_Point(int start, int end)
{
    int i,t;//???????
    if(Left_Down_Find!=0)   Last_Left_Up_Find=Left_Down_Find;//initialize the last left up find
    if(Right_Down_Find!=0)  Last_Right_Up_Find=Right_Down_Find;//initialize the last right up find
    Left_Up_Find=0;//
    Right_Up_Find=0;//???????????????

    if(start<end)//?????start?????end
    {
        t=start;
        start=end;
        end=t;
    }
    if(end<=IMAGE_HEIGHT-Search_Stop_Line)  end=IMAGE_HEIGHT-Search_Stop_Line;//?????????
    if(end<=5)  end=5;
    if(start >=IMAGE_HEIGHT -1-5)   start=IMAGE_HEIGHT-1-5;//????5???????????????????????????????
    /*????????????????????????????????????????*/
    for(i=end;i<=start;i++)//???????????????? ??????????????i++????????????��
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
 * @brief lengthen the left boundary
 * @param start The starting index of the boundary
 * @return null
 */
void Lengthen_Left_Boundry(int start, int end)
{
    int i,t;
    float k=0.0;
    if(start >=IMAGE_HEIGHT -1) start=IMAGE_HEIGHT-1; //limitaion
    else if(start <=0)  start=0;//limitation
    if(end>=IMAGE_HEIGHT-1) end=IMAGE_HEIGHT-1;//the limitation
    else if(end<=0)  end=0;//end limitation
    if(end < start) //start must be less than end
    {
        t=start;
        start=end;
        end=t;
    }
    if(start <=5)   Left_Add_Line(left_line[start],start,left_line[end],end);//if the start is less than or equal to 5, add a line to the left boundary
    else
    {
        k=(float)(left_line[start]-left_line[start-4])/5.0; //calculate the slope of the line
        for(i=start;i<=end;i++)
        {
            left_line[i]=(int)(i-start)*k+left_line[start];//lengthen the left boundary
            if(left_line[i]>=IMAGE_WIDTH-1) left_line[i]=IMAGE_WIDTH-1;//if the value exceeds the maximum width
            else if(left_line[i]<=0) left_line[i]=0;//if the value is less than or equal to 0
        }
    }
}

/**
 * @brief Lengthen the right boundary
 * @param start The starting index of the boundary
 * @param end The ending index of the boundary
 * @return null
 */
void Lengthen_Right_Boundry(int start, int end)
{
    int i,t;
    float k=0.0;
    if(start >=IMAGE_HEIGHT -1) start=IMAGE_HEIGHT-1; // Check if start is greater than or equal to IMAGE_HEIGHT - 1
    else if(start <=0)  start=0; // Check if start is less than or equal to 0
    if(end>=IMAGE_HEIGHT-1) end=IMAGE_HEIGHT-1; // Check if end is greater than or equal to IMAGE_HEIGHT - 1
    else if(end<=0)  end=0; // Check if end is less than or equal to 0
    if(end < start) // Swap start and end if end is less than start
    {
        t=start;
        start=end;
        end=t;
    }
    if(start <=5)   Right_Add_Line(right_line[start],start,right_line[end],end); // Add a line to the right boundary if start is less than or equal to 5
    else
    {
        k=(float)(right_line[start]-right_line[start-4])/5.0; // Calculate the slope of the line
        for(i=start;i<=end;i++)
        {
            right_line[i]=(int)(i-start)*k+right_line[start]; // Lengthen the right boundary
            if(right_line[i]>=IMAGE_WIDTH-1) right_line[i]=IMAGE_WIDTH-1; // Check if the value exceeds the maximum width
            else if(right_line[i]<=0) right_line[i]=0; // Check if the value is less than or equal to 0
        }
    }
}

/**
 * @brief the function to detect the crossing
 * @param null
 * @return null
 */
void Cross_Detect(void)
{
    int down_search_start = 0;//the down point of finding the crossing
    if(Road_Type == CROSSING)//?????????????
    {
        Left_Up_Find=0;
        Right_Up_Find=0;
        if(Both_Lost_Time >=15)//???????????????
        {
            Find_Up_Point(110,6);//??????????????
            if(Left_Up_Find ==0 && Right_Up_Find ==0) 
            return ;//?????????????
        }
        else    return;//?????????????????????��???
        if(Left_Up_Find !=0 &&Right_Up_Find !=0)
        {
            down_search_start=Left_Up_Find>Right_Up_Find? Left_Up_Find:Right_Up_Find;//??????????????????
            Find_Down_Point(IMAGE_HEIGHT -5,down_search_start+10);//?????????????????????????????????????????????
            if(Left_Down_Find<=Left_Up_Find)    Left_Down_Find=0;//????????????????????????????
            if(Right_Down_Find<=Right_Up_Find)  Right_Down_Find=0;//????????????????????????????
            if(Left_Down_Find!=0 && Right_Down_Find!=0)//???????????????
            {
                Left_Add_Line(left_line[Left_Up_Find],Left_Up_Find,left_line[Left_Down_Find],Left_Down_Find);//??????
                Right_Add_Line(right_line[Right_Up_Find],Right_Up_Find,right_line[Right_Down_Find],Right_Down_Find);//??????
            }
            else if(Left_Down_Find == 0 && Right_Down_Find !=0)//???????
            {
                Lengthen_Left_Boundry(Left_Up_Find-1,IMAGE_HEIGHT-1);//???????
                Right_Add_Line(right_line[Right_Up_Find],Right_Up_Find,right_line[Right_Down_Find],Right_Down_Find);//???????
            }
            else if (Left_Down_Find !=0 && Right_Down_Find ==0)//???????
            {
                Lengthen_Right_Boundry(Right_Up_Find-1,IMAGE_HEIGHT-1);//???????
                Left_Add_Line(left_line[Left_Up_Find],Left_Up_Find,left_line[Left_Down_Find],Left_Down_Find);//???????
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

unsigned int Road_Min_Width[2]={188,0};//Record the number of rows and width corresponding to the minimum road width
/**
 * @brief the function to detect the ramp
 * @param null
 * @return null
 * @attention 1.The principle of this function is to determine the difference between the standard track width and the measured track width
 * 2. The difference is that this function does not require recording the standard track width, as the minimum track width has already been recorded in the straight track
 * 3.The area that needs improvement in this function is to record the width of the top 5 rows of the straight track. It is not very good to only record 1 row
 */
void Ramp_Detect(void)
{
    if(Road_Type!=STRAIGHT_ROAD)    return;//misjudgment detection
    /*Update minimum road width*/
    for(uint8 i=IMAGE_HEIGHT-1;i>=IMAGE_HEIGHT-Search_Stop_Line;i--)
    {
        if(Road_Wide[i]<Road_Min_Width[0])
        {
            Road_Min_Width[0]=(right_line[i]-left_line[i]);//Obtain the corresponding road width by subtracting
            Road_Min_Width[1]=i;//Record the number of rows corresponding to the minimum road width
        }
    }
    
    uint8 my_count=0;
    /*Judging whether it is a ramp based on standard road width*/
    for(uint8 i=IMAGE_HEIGHT-Search_Stop_Line;i<=IMAGE_HEIGHT-Search_Stop_Line+5;i++)
    {
        //Starting from the five lines down from the cut-off line, determine if there will be any exceeding the standard line width
        if((Road_Wide[i]-Road_Min_Width[0])>=20)
        {
            my_count++;
        }
    }
    if(my_count>=5) Road_Type=RAMP;//If the abnormal width of the row exceeds a certain value, it is judged as a ramp
    else Road_Type=STRAIGHT_ROAD;//Otherwise, the element will remain straight (without any changes)
}
/**
 * @brief Black and white detection function
 * @param uint8 row : the row that was fixed and counts for pixle 
 *              uint8 start_colum the beginning column of start count
 *              uint8 end_column the end coulumn of end count
 * @return the number of white pixle
 */
uint8 Black_White_Dump(uint8 row,uint8 start_column,uint8 end_column)
{
    if(row>=IMAGE_HEIGHT-1) row=IMAGE_HEIGHT-1;//???????
    else if(row<=0) row=0;//???????
    if(row<=5)  return 0;//???????��?????5????????0
    if(start_column>=IMAGE_WIDTH-1) start_column=IMAGE_WIDTH-1;//????????
    else if(start_column<=0) start_column=0;//????????
    if(end_column>=IMAGE_WIDTH-1) end_column=IMAGE_WIDTH-1;//????????
    else if(end_column<=0) end_column=0;//????????
    if(row<=30) row=30;//??????????????
    else if(row>=89)  row=89;//??????????????
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
        //     if(Image_Use[row][i]==WHITE_POINT&&first_white_column==0)//??????��????????
        //     {
        //         first_white_column=i;//??????????????????
        //     }
        //     else if(Image_Use[row][i]==BLACK_POINT&&first_black_column==0)
        //     {
        //         first_black_column=i;//??????????????????
        //         if(abs(first_black_column-first_white_column)<=10)//??????????????????????????row?????????????????��????
        //         {
        //             first_white_column=0;//?????????????????
        //             first_black_column=0;
        //             count++;
        //             if(count>=5)   return count;//??????????????????????5????????
        //         }
        //     }
        // }
        // else if(mode==0)
        // {
        //     if(Image_Use[row][i]==WHITE_POINT)  white_point_count++;//??????????
        // }
        
        if(Image_Use[row][i]==WHITE_POINT)  
        {
            // ips114_draw_point(i,row,RGB565_BLUE);
            white_point_count++;//??????????
        }
        
    }
    
    // ips114_show_uint(188,60,row,3);
    // ips114_show_uint(188,80,end_column-start_column,3);
    // ips114_show_uint(188,100,white_point_count,3);
    // ips114_show_uint(188,120,abs(white_point_count-(end_column-start_column)),3);
    if(mode==0)
    {
        if(abs(white_point_count-(end_column-start_column))<=Zebra[row])//??????????????????????��??????????????????????????????
        {
            return 1;
        }
    }
    return 0;
}

/**
 * @brief Island detection function
 * @param none
 * @return none
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
 * @brief Zebra crossing detection function
 * @param none
 * @return none
 * @attention none
 */
void Zebra_Stripes_Detect(void)
{
    int continuity_change_right_flag =0;//Find the position of the left boundary continuity row
    int continuity_change_left_flag =0;//Find the position of the right boundary continuity row
    int monotonicity_change_right_flag =0;//Find the position of the left boundary monotonicity row
    int monotonicity_change_left_flag =0;//Find the position of the right boundary monotonicity row
    
    continuity_change_left_flag = Continuity_Change_Left(IMAGE_HEIGHT-1,5,0);//
    continuity_change_right_flag = Continuity_Change_Right(IMAGE_HEIGHT-1,5,0);//
    monotonicity_change_left_flag = Continuity_Change_Left(IMAGE_HEIGHT-1,5,1);//
    monotonicity_change_right_flag = Continuity_Change_Right(IMAGE_HEIGHT-1,5,1);//
    if(continuity_change_left_flag<=30||continuity_change_right_flag<=30)   
    {
        if(Road_Type==BANMAXIAN) Road_Type=STRAIGHT_ROAD;//If the number of lost wires on both sides is too small, stop judging
        return ;
    }
    // ips114_draw_line(94,60,left_line[continuity_change_left_flag],continuity_change_left_flag,RGB565_RED);
    // ips114_draw_line(98,0,left_line[monotonicity_change_left_flag],monotonicity_change_left_flag,RGB565_BLUE);
    // ips114_show_uint(188,60,continuity_change_right_flag,3);
    // ips114_show_uint(188,80,continuity_change_left_flag,3);
    
    int i=0,j=0,change_count=0,start_line=0,endl_line=0,narrow_road_count=0;
    if(Search_Stop_Line >=60 && 30<=Longest_White_Column_Left[1] && Longest_White_Column_Left[1]<=IMAGE_WIDTH-30  &&
    abs(continuity_change_left_flag-continuity_change_right_flag)<=30 &&continuity_change_left_flag!=0 &&continuity_change_right_flag!=0)
    {
        uint8 count=Black_White_Dump(continuity_change_left_flag-3,left_line[continuity_change_left_flag-3],right_line[continuity_change_left_flag-3]);
        uint8 higher_flag= (continuity_change_left_flag < continuity_change_right_flag) ? continuity_change_left_flag : continuity_change_right_flag; // ???A��??B?????C????A??????C????B
        uint8 lower_flag= (monotonicity_change_left_flag > monotonicity_change_right_flag) ? monotonicity_change_left_flag : monotonicity_change_right_flag; // ???A????B?????C????A??????C????B
        // if(Black_White_Dump((higher_flag+lower_flag)/2,left_line[higher_flag]-5,right_line[higher_flag]-5))
        // {
            //            Road_Type=BANMAXIAN;
             // if(Road_Type==STRAIGHT_ROAD)    Road_Type=BANMAXIAN;//????????????????????????��??????????????????????????????????????????
        // }
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
    // for(uint8 i=0;i<IMAGE_HEIGHT-1;i++)
    // {
    //     ips114_draw_point((left_line[i]+right_line[i])/2,i,RGB565_RED);
    //     ips114_draw_point(left_line[i],i,RGB565_BLUE);
    //     ips114_draw_point(right_line[i],i,RGB565_GREEN);
    // }
//    ips114_draw_line(158,80,left_line[Left_Up_Find],Left_Up_Find,RGB565_PURPLE);
//    ips114_draw_line(98,60,right_line[Right_Up_Find],Right_Up_Find,RGB565_BLUE);
//    ips114_show_uint(188,120,threshold,3);      
	ips114_displayimage03x(*Image_Use,188,120);
	ips114_show_uint(188,0,Longest_White_Column_Left[1],3);
    float my_err=Err_Handle();
    ips114_show_uint(188,20,flag_test,2);
    ips114_show_uint(188,40,type,3);
    ips114_show_uint(188,60,right_line[Boundry_Start_Right],3);
    ips114_show_uint(188,80,left_line[Boundry_Start_Left],3);
    ips114_show_uint(188,100,Left_Lost_Time,3);   
    ips114_show_uint(188,120,Right_Lost_Time,3);
}

/**
 * @brief the main function of the program
 * @param none
 * @return none
 */
void test(void)
{
    uint8 mode=0;//choose the find line mode
    
    if(mode==1)
    {
        Image_Change();
        uint8 threshold=OSTU_GetThreshold((uint8 *)mt9v03x_image,IMAGE_WIDTH,IMAGE_HEIGHT);
        Simple_Binaryzation(*Image_Use,threshold);
        Center_line_deal(5,183);//?????????
    }
    else if(mode==0)
    {
        uint8 *output_address;//the address that located in the first pixel of the image
        output_address=Scharr_Edge(*mt9v03x_image);//use the way of sccan edge to get the image
		uint8 threshold=OSTU_GetThreshold((uint8 *)mt9v03x_image,IMAGE_WIDTH,IMAGE_HEIGHT);
        memcpy(Image_Use,output_address,IMAGE_HEIGHT*IMAGE_WIDTH*sizeof(uint8));
		Simple_Binaryzation(*Image_Use,threshold);/*to handle one picture,it needs nearly 9000us*/
        if(pick_up_mode==0)
        {
            Center_line_deal_plus(23,163);//Cannot set too high or too low boundary, otherwise it will cause an error
        }
        else//if the state is picking the card
        {
            Easy_Filtering(110,60,30,130,5);
            Search_Center();
        }
    }	
    test2();
}
