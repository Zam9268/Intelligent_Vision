#include "image.h"
#include "stdbool.h"
#include "stdio.h"
#include "math.h"

uint8 Image_Use[IMAGE_HEIGHT][IMAGE_WIDTH];
uint8 type = 0;

/*The following are the global variables used, but there may be some that are not used*/
uint8 left_line[IMAGE_HEIGHT], right_line[IMAGE_HEIGHT]; // record the left line's column and the right line's column
int center[IMAGE_HEIGHT];                                // record the center line's column
uint8 the_maxlen_position;                               // record  the max length of the white column
uint8 num;                                               //
uint8 Longest_White_Column_Left[2];                      // Record the longest white column in this iteration
uint8 Last_Longest_White_Column_Left[2];                 // Record the longest white column in the previous iteration to prevent white column fluctuations in some areas
uint8 Left_Line_Start, Right_Line_Start;                 // Starting point of the left and right lines
uint8 Longest_White_Column_Right[2];                     // The longest white column on the right side, not used
uint8 Right_Lost_Flag[IMAGE_HEIGHT];                     // Lost line flag for the right boundary
uint8 Left_Lost_Flag[IMAGE_HEIGHT];                      // Lost line flag for the left boundary
uint8 Left_Lost_Time = 0;                                // Number of times the left line is lost
uint8 Right_Lost_Time = 0;                               // Number of times the right line is lost
uint8 Both_Lost_Time = 0;                                // Number of times both lines are lost in the same row
uint8 Search_Stop_Line;                                  // Stop line for searching
uint8 Boundry_Start_Left, Boundry_Start_Right;           // Starting points of the left and right boundaries
uint8 Road_Wide[IMAGE_HEIGHT];                           // Road width
RoadType Road_Type;                                      // Type of road element
uint8 Right_Down_Find = 0;
uint8 Left_Down_Find = 0; // Finding the left bottom turning point
uint8 Left_Up_Find = 0;   // Finding the left top turning point
uint8 Last_Left_Up_Find = 0;
uint8 Last_Right_Up_Find = 0; // the last time the right up point is found
uint8 Right_Up_Find = 0;      // Finding the right top turning point
uint8 flag_test = 0;
uint8 pick_up_mode = 0;                    // when the value is 1, it is in the card picking state; when the value is 0, it is in the free patrol state
uint8 card_left_up_find_flag = 0;          // the lef up corner of the card lying on the side of the road is found
uint8 card_right_up_find_flag = 0;         // the right up corner of the card lying on the side of the road is found
uint8 left_island_flag, right_island_flag; // the flag of the island on the left and right
uint8 ramp_flag = 0;                       // the flag of the ramp
uint8 Island_State = 0;                    // record the state of the island on the left or right
uint8 max_left_line = 0;                   // record the max left line
uint8 last_max_left_line = 0;              // record the last max left line
uint8 start_row = 0;                       // record the start row
int left_up_state3_point[2] = {0};         // 左上角顶点的坐标
uint8 Island_surrond[IMAGE_WIDTH] = {0};
int center_x, center_y;
int left_up_point[2] = {0};  // 左上角拐点坐标
int right_up_point[2] = {0}; // 右下角拐点坐标
int island_state3_real_x = 0;
int island_state3_real_y = 0; // 环岛状态3的固定点坐标
int camera_island_state3_x = 0;
int camera_island_state3_y = 0; // 环岛状态3的固定点相机坐标
float Left_derivative[IMAGE_HEIGHT] = {0.0};
float Right_derivative[IMAGE_HEIGHT] = {0.0};
float err = 0.00;
float last_err = 0.00;
float island_err = 0.00; // 记录环岛时的误差
/*the following is the information for receiving data through the serial port*/
extern uint8 right_data[64];        // store the data received from the serial port,it only store 64 bytes
extern uint32 fifo_data_count;      // the number of data lied in the buffer
extern uint8 data_length;           // the length of the data received
extern uint8 i;                     // the state of get data,this is unuseful
extern int count;                   // this is unuseful
extern unsigned int the_max_G;      // the scchar's threshold
extern int now_distance_x;          // the distance made by the target detection algorithm,left is negative,right is positive
extern unsigned int now_distance_y; // the distance made by the target detection algorithm,up is always positive
extern uint8 init_flag;             // the flag of the initialization
uint8 state3_left_up_guai[2] = {0};
// Corresponding image height weight array (counting from bottom to top)
const uint8 Weight[IMAGE_HEIGHT] =
    {
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,        // Weight of rows 0 to 9
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,        // Weight of rows 10 to 19
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,        // Weight of rows 20 to 30
        1, 1, 1, 1, 1, 1, 1, 1, 1, 1,        // Weight of rows 30 to 39
        1, 1, 1, 1, 1, 1, 1, 3, 4, 5,        // Weight of rows 40 to 49
        6, 7, 9, 11, 13, 15, 17, 19, 20, 20, // Weight of rows 50 to 59
        19, 17, 15, 13, 11, 9, 7, 5, 3, 1,   // Weight of rows 60 to 69
};

/*Zebra*/
const uint8 Zebra[60] = {
    30,
    30,
    30,
    30,
    30,
    30,
    30,
    30,
    30,
    30,
    35,
    35,
    35,
    35,
    35,
    35,
    35,
    35,
    35,
    35,
    45,
    45,
    45,
    45,
    45,
    45,
    45,
    45,
    45,
    45,
    55,
    55,
    55,
    55,
    55,
    55,
    55,
    55,
    55,
    55,
    65,
    65,
    65,
    65,
    65,
    65,
    65,
    65,
    65,
    65,
    85,
    85,
    85,
    85,
    85,
    85,
    85,
    85,
    85,
    85,
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
volatile int White_Column[IMAGE_WIDTH]; //???????????
/**
 * @brief This longest white line patrol can only be used to process images of Otsu method,it is not applicable to images after edge detection
 * @param start_column, end_column: The starting and ending columns for finding the longest white column
 * @return None (actually returns the edge line array)
 */
void Center_line_deal(uint8 start_column, uint8 end_column)
{
    for (uint8 i = 0; i < IMAGE_HEIGHT - 1; i++)
    {
        left_line[i] = 0;
        right_line[i] = IMAGE_WIDTH - 1;
        Right_Lost_Flag[i] = 0;
        Left_Lost_Flag[i] = 0;
    }
    for (uint8 i = 0; i < IMAGE_WIDTH - 1; i++)
    {
        White_Column[i] = 0;
    }
    int x = 0, y = 0;                   //??x???,y???
    uint8 middle = the_maxlen_position; //??????????????
    uint8 x_num;
    /*?????????????????? */
    for (uint8 j = start_column; j <= end_column; j++)
    {
        for (uint8 i = IMAGE_HEIGHT - 1; i >= 0; i--)
        {
            if (Image_Use[i][j] == BLACK_POINT)
            {
                break;
            }
            else
            {
                White_Column[j]++;
                if (White_Column[j] == 120)
                    break;
            }
        }
    }
    /*???????????????*/
    Longest_White_Column_Left[0] = 0; //????????????
    for (uint8 i = start_column; i <= end_column; i++)
    {
        if (White_Column[i] > Longest_White_Column_Left[0]) //????????
        {
            Longest_White_Column_Left[0] = White_Column[i];
            Longest_White_Column_Left[1] = i;
        }
    }
    /*????????????????*/
    Longest_White_Column_Right[0] = 0; //????????????
    for (uint8 i = end_column; i > start_column; i--)
    {
        if (White_Column[i] > Longest_White_Column_Right[0]) //??????????
        {
            Longest_White_Column_Right[0] = White_Column[i];
            Longest_White_Column_Right[1] = i; //??????????????????????
        }
    }
    /*?????????*/
    Search_Stop_Line = Longest_White_Column_Left[0]; //??????????????
    int right_border, left_border;                   //?????????????
    for (int i = IMAGE_HEIGHT - 1; i >= IMAGE_HEIGHT - Search_Stop_Line; i--)
    {
        /*????????*/
        for (int j = Longest_White_Column_Left[1]; j <= IMAGE_WIDTH - 1; j++)
        {
            if (Image_Use[i][j] == WHITE_POINT && Image_Use[i][j + 1] == BLACK_POINT && Image_Use[i][j + 2] == BLACK_POINT)
            {
                right_border = j;       //???????????
                Right_Lost_Flag[i] = 0; //????????????0
                break;
            }
            else if (j >= IMAGE_WIDTH - 1 - 2) //??????????????????????x??????????????????1
            {
                right_border = j;
                Right_Lost_Flag[i] = 1;
                break;
            }
        }
        for (uint8 j = Longest_White_Column_Left[1]; j >= 2; j--)
        {
            if (Image_Use[i][j] == WHITE_POINT && Image_Use[i][j - 1] == BLACK_POINT && Image_Use[i][j - 2] == BLACK_POINT)
            {
                left_border = j;       //???????????
                Left_Lost_Flag[i] = 0; //????????????0
                break;
            }
            else if (j <= 2) //??????????????????????x??????????????????1
            {
                left_border = j;
                Left_Lost_Flag[i] = 1;
                break;
            }
        }
        left_line[i] = left_border;   //?????????????
        right_line[i] = right_border; //?????????????
    }
}

/**
 * @brief Longest white column plus version (written by myself, used for finding the longest white column in image detection)
 * @param uint8 start_column, uint8 end_column: The starting and ending columns for finding the longest white column
 * @return None (actually returns the edge line array)
 */
void Center_line_deal_plus(uint8 start_column, uint8 end_column)
{
begin:

    for (uint8 i = 0; i < IMAGE_HEIGHT - 1; i++)
    {
        left_line[i] = 0;
        right_line[i] = 0;
        Right_Lost_Flag[i] = 0; // Clear the right line lost flag to 0
        Left_Lost_Flag[i] = 0;  // Clear the left line lost flag to 0
    }
    Left_Lost_Time = 0;  //????????????
    Right_Lost_Time = 0; //?????????????
    Both_Lost_Time = 0;  //??????????????????
    Boundry_Start_Left = 0;
    Boundry_Start_Right = 0; //????????????
    /* Reset white column count */
    for (uint8 i = 0; i <= IMAGE_WIDTH - 1; i++)
    {
        White_Column[i] = 0;
    }
    /*Counting white columns*/
    for (uint8 j = start_column; j <= end_column; j++)
    {
        for (uint8 i = IMAGE_HEIGHT - 3; i >= 0; i--) //???????????????????????????????
        {
            if (Image_Use[i][j] == BLACK_POINT) // Stop counting when encountering a white boundary point, otherwise increment
            {
                White_Column[j]++;
                if (White_Column[j] == 120)
                    break; // if count is encough,stop countting
            }
            else
            {
                break;
            }
        }
        if (Image_Use[119][j] == BLACK_POINT && Image_Use[118][j] == BLACK_POINT) //???????��?��??????????��????????
        {
            White_Column[j] = 0; //???????????
        }
    }
    /* Find the longest white column */
    Last_Longest_White_Column_Left[0] = Longest_White_Column_Left[0]; // Record the longest white column in the previous iteration
    Last_Longest_White_Column_Left[1] = Longest_White_Column_Left[1]; // record the column number of the longest white column in the previous iteration
    Longest_White_Column_Left[0] = 0;                                 // Clear the information of the longest white column
    for (uint8 i = start_column; i <= end_column; i++)
    {
        if (White_Column[i] > Longest_White_Column_Left[0]) // Replace the longest white column with the maximum value
        {
            Longest_White_Column_Left[0] = White_Column[i]; // Record the length of the corresponding longest white column
            Longest_White_Column_Left[1] = i;               // Record the column number where the corresponding longest white column is located
        }
    }
    Search_Stop_Line = Longest_White_Column_Left[0]; // Set the stop line for searching to the length of the longest white column
    /* To prevent significant changes in the position of the longest white column at the turning point, set a verification for the change */
    if (abs(Longest_White_Column_Left[1] - Last_Longest_White_Column_Left[1]) >= 60) // If the longest white column position changes by more than 60 columns
    {
        Longest_White_Column_Left[0] = Last_Longest_White_Column_Left[0]; // Then the longest white column is set to the previous value
        Longest_White_Column_Left[1] = Last_Longest_White_Column_Left[1];
    }

    /* Start searching for boundaries */
    int right_border, left_border; // Define intermediate variables for boundaries
    uint8 left_start_flag = 0;
    uint8 right_start_flag = 0;
    for (int i = IMAGE_HEIGHT - 1; i >= IMAGE_HEIGHT - Search_Stop_Line; i--)
    {
        for (int j = Longest_White_Column_Left[1]; j >= 2; j--) // Search for the left boundary from the middle to the left
        {
            if (Image_Use[i][j] == BLACK_POINT && Image_Use[i][j - 1] == WHITE_POINT && Image_Use[i][j - 2] == WHITE_POINT)
            {
                left_border = j;       // Record the column coordinate of the corresponding boundary
                Left_Lost_Flag[i] = 0; // No line lost, set the line lost flag to 0
                break;
            }
            else if (j <= 2) // If encountering a boundary
            {
                left_border = j;       // Directly record the position of the boundary
                Left_Lost_Flag[i] = 1; // Set the line lost flag to 1
                break;
            }
        }
        for (int j = Longest_White_Column_Left[1]; j <= IMAGE_WIDTH - 3; j++) // Search for the right boundary from the middle to the right
        {
            if (Image_Use[i][j] == BLACK_POINT && Image_Use[i][j + 1] == WHITE_POINT && Image_Use[i][j + 2] == WHITE_POINT)
            {
                right_border = j;       // Store the boundary information
                Right_Lost_Flag[i] = 0; // Set the boundary flag to 0
                if (right_start_flag == 0)
                {
                    if (Right_Lost_Flag[i - 1] == 1) //?????��???????????????????????????
                    {
                        Right_Line_Start = i;
                        right_start_flag = 1;
                    }
                }
                break;
            }
            else if (j >= IMAGE_WIDTH - 1 - 2) // If reaching the right boundary
            {
                right_border = j;       // Directly record the position of the right boundary
                Right_Lost_Flag[i] = 1; // Then set the line lost flag to 1
                break;
            }
        }
        left_line[i] = left_border; // Store the corresponding boundary information
        right_line[i] = right_border;
    }
    /*?????��???????????????????????????????????*/

    if (Longest_White_Column_Left[1] <= 60 && Left_Lost_Time >= 60 && Right_Lost_Time <= 5)
    {
        if (right_line[Boundry_Start_Right] <= (IMAGE_WIDTH / 2)) //????????????
        {
            Last_Longest_White_Column_Left[1] = 94;
            Longest_White_Column_Left[1] = 94;
            flag_test++;
            goto begin; //???????????????????????
        }
    }
    else if (Longest_White_Column_Left[1] >= 128 && Right_Lost_Time >= 60 && Left_Lost_Time <= 5)
    {
        if (left_line[Boundry_Start_Left] >= (IMAGE_WIDTH / 2)) //????????????
        {
            Last_Longest_White_Column_Left[1] = 94;
            Longest_White_Column_Left[1] = 94;
            flag_test++;
            goto begin; //???????????????????????
        }
    }
}

/**
 * @brief Function to get the number of white points in the neighborhood of a given point
 * @param x, y: The coordinates of the point
 * @return The number of white points in the 8-neighborhood of the given point
 * @attention None
 */
uint8 Get_White_Point(uint8 x, uint8 y)
{
    if (x <= 1 || x >= IMAGE_WIDTH - 2 || y <= 1 || y >= IMAGE_HEIGHT - 2)
        return 0; // Check if the point is within the image boundaries
    uint8 white_point = 0;
    for (uint8 i = x - 1; i <= x + 1; i++)
    {
        for (uint8 j = y - 1; j <= y + 1; j++)
        {
            if (Image_Use[j][i] == WHITE_POINT)
                white_point++;
        }
    }
    return white_point; // Return the number of white points
}

/**
 * @brief the function of transforming the camera coordinate to the world coordinate
 * @param camera_x,camera_y: The camera coordinate int *real_x,int *real_y: The real world coordinate
 * @return no,but in fact the point is stored in the real world coordinate
 * @attention no
 */
void Pespective_point(int camera_x, int camera_y, int *real_x, int *real_y)
{
    float x, y1, w; // 定义现实3坐标系的坐标
    /*先进行坐标系的转换*/
    camera_x -= 94;
    camera_y = 120 - camera_y;
    x = getx(camera_x, camera_y);
    y1 = gety(camera_x, camera_y);
    w = getw(camera_x, camera_y);
    *real_x = (int)(x / w);
    *real_y = (int)(y1 / w) + 210; // 对坐标进行齐次坐标变换，加上y的平移
}

/**
 * @brief the function of transforming the world coordinate to the camera coordinate
 * @param real_x,real_y: The real world coordinate  int *camera_x,int *camera_y: The camera coordinate
 * @return no,but in fact the point is stored in the camera coordinate
 * @attention the martix is the inverse matrix of the matrix in the function Pespective_point
 */
void Pespective_point_b(int real_x, int real_y, int *camera_x, int *camera_y)
{
    real_y -= 210; // 减去y的平移
    float x, y, w; // 定义相机3坐标系的坐标
    x = getx_b(real_x, real_y);
    y = gety_b(real_x, real_y);
    w = getw_b(real_x, real_y);
    *camera_x = (int)(x / w) + 94;
    *camera_y = 120 - (int)(y / w); // 对坐标进行齐次坐标变换
}

/**
 * @brief the function of detecting the card lying near the road
 * @param no
 * @return no
 * @attention no
 */
void Border_Card_Detect(void)
{
}

int deviation[8][2] = {{0, -1}, {-1, -1}, {-1, 0}, {-1, 1}, {0, 1}, {1, 1}, {1, 0}, {1, -1}};        //??????x??????????y????
int devitation_right[8][2] = {{0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}}; //???????????
struct Line_Edge
{
    uint8 row;    //??
    uint8 column; //??
    uint8 flag;   //???????��
    uint8 grow;   //????????
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
    uint8 my_detect_mode = 1;
    if (my_detect_mode == 0) // If detection mode is 0
    {
        uint8 Top_h = 0, Top_w = 0, Bottom_h = 0, Botton_w = 0; // Variables for storing coordinates
        uint8 ter_h = 100, ter_w = 60, Mid_h = 0, Mid_w = 0;
        float cam_dis_h = 0, cam_dis_w = 0;
        for (uint8 i = 30; i < IMAGE_HEIGHT - 2; i++) // Loop through the image
        {
            for (uint8 j = 2; j < IMAGE_WIDTH - 2; j++)
            {
                if (Image_Use[i][j] == 255 && (Image_Use[i - 2][j] == 0 && Image_Use[i - 2][j - 1] == 0 && Image_Use[i][j - 1] == 0) && (Image_Use[i][j + 1] == 255 && Image_Use[i][j + 3] == 255 && Image_Use[i][j + 6] == 255 && Image_Use[i][j + 9] == 255) &&
                    Image_Use[i + 1][j] == 255 && Image_Use[i + 3][j] == 255 && Image_Use[i + 1][j - 5] == 0 && Image_Use[i + 2][j - 10] == 0 && Image_Use[i + 3][j - 10] == 0 && Image_Use[i + 2][j - 15] == 0)
                {
                    Top_h = i;
                    Top_w = j;
                    break;
                }
            }
        }
    }
    else if (my_detect_mode == 1) // If detection mode is 1,this detection is wrote by me
    {
        uint8 lower_black_row = 0; // count the number of black points in a row
        for (uint8 i = IMAGE_HEIGHT - 20; i >= 30; i--)
        {
            uint8 count_for_black_point = 0;
            for (uint8 j = 0; j <= IMAGE_WIDTH - 1; j++)
            {
                if (Image_Use[i][j] == BLACK_POINT)
                {
                    count_for_black_point++;
                }
            }
            if (count_for_black_point == 188) // 自下而上寻找最近黑列
            {
                lower_black_row = i;
                break;
            }
        }
        /*针对斜边形状卡片，当从左边和从右边扫到的顶点一样的话，就说明可以*/
        /*扫描方法：从起始行的左列，右列向中间扫，当扫到白点的时候就记录该点位置，记录每行的点对应的列坐标
        然后从上到下优先取坐标靠上的，当该坐标满足条件：1.和下5行的列坐标相近 2.且和下2个坐标相连（）*/
        uint8 left_count[IMAGE_HEIGHT / 2] = {0};
        uint8 right_count[IMAGE_HEIGHT / 2] = {0};
        uint8 end_row;
        if (abs(IMAGE_HEIGHT - 20 - lower_black_row) >= 60)
            end_row = lower_black_row + 60;
        else
            end_row = IMAGE_HEIGHT - 20; // 限幅处理
        for (uint8 i = lower_black_row; i <= end_row; i++)
        {
            /*先从左边界扫到右边*/
            for (uint8 j = 20; j < IMAGE_WIDTH - 1 - 20; j++)
            {
                if (Image_Use[i][j] == WHITE_POINT && Image_Use[i][j - 1] == BLACK_POINT && Image_Use[i][j - 2] == BLACK_POINT)
                {
                    left_count[i - lower_black_row] = j; // 记录对应的列坐标
                    break;                               // 找到就直接退出
                }
                else
                {
                    left_count[i - lower_black_row] = 0; // 找不到就记录为0
                }
            }
            /*再从右边界扫到左边*/
            for (uint8 j = IMAGE_WIDTH - 1 - 20; j >= 20; j--)
            {
                if (Image_Use[i][j] == WHITE_POINT && Image_Use[i][j + 1] == BLACK_POINT && Image_Use[i][j + 2] == BLACK_POINT)
                {
                    right_count[i - lower_black_row] = j; // 记录对应的列坐标
                    break;                                // 找到就直接退出
                }
                else
                {
                    right_count[i - lower_black_row] = IMAGE_WIDTH - 1 - 20; // 找不到就记录为0
                }
            }
        }
        // for(uint8 i=0;i<=end_row-lower_black_row;i++)
        // {
        //     ips114_draw_point(left_count[i],i+lower_black_row,RGB565_RED);
        //     ips114_draw_point(right_count[i],i+lower_black_row,RGB565_BLUE);
        // }
        /*开始寻找边界点*/
        left_up_point[0] = 0;
        left_up_point[1] = 0;
        right_up_point[0] = 0;
        right_up_point[1] = 0; // 清零计数
        card_left_up_find_flag = 0;
        card_right_up_find_flag = 0; // 初始化角点寻找标志位
        for (uint8 i = 0; i <= end_row - lower_black_row; i++)
        {
            if (abs(left_count[i] - left_count[i + 1]) <= 1 && abs(left_count[i] - left_count[i + 2]) <= 2 && abs(left_count[i] - left_count[i + 3]) <= 3 &&
                abs(left_count[i] - left_count[i + 4]) <= 4 && abs(left_count[i] - left_count[i + 5]) <= 5)
            {
                left_up_point[0] = left_count[i];       // 记录列坐标
                left_up_point[1] = i + lower_black_row; // 记录行坐标
                card_left_up_find_flag = 1;             // 找到对应的拐点
                break;
            }
        }
        for (uint8 i = 0; i <= end_row - lower_black_row; i++)
        {
            if (abs(right_count[i] - right_count[i + 1]) <= 1 && abs(right_count[i] - right_count[i + 2]) <= 2 && abs(right_count[i] - right_count[i + 3]) <= 3 &&
                abs(right_count[i] - right_count[i + 4]) <= 4 && abs(right_count[i] - right_count[i + 5]) <= 5)
            {
                right_up_point[0] = right_count[i];      // 记录列坐标
                right_up_point[1] = i + lower_black_row; // 记录行坐标
                card_right_up_find_flag = 1;             // 找到对应的拐点
                break;
            }
        }
        // ips114_show_uint(188,75,right_up_point[0],3);
        // ips114_show_uint(188,90,right_up_point[1],3);
        // ips114_show_uint(188,105,left_up_point[0],3);
        // ips114_show_uint(188,120,left_up_point[1],3);
        // ips114_draw_line(0,0,left_up_point[0],left_up_point[1],RGB565_RED);
        // ips114_draw_line(0,0,right_up_point[0],right_up_point[1],RGB565_BLUE);
    }
    if (card_right_up_find_flag == 1 && card_left_up_find_flag == 1)
    {
        int real_left_up_x = 0, real_left_up_y = 0, real_right_up_x = 0, real_right_up_y = 0;

        Get_Card_Center_coordinate(left_up_point[0], left_up_point[1], right_up_point[0], right_up_point[1], &center_x, &center_y);
        // Pespective_point(left_up_point[0], left_up_point[1], &real_left_up_x, &real_left_up_y);
        // Pespective_point(right_up_point[0], right_up_point[1], &real_right_up_x, &real_right_up_y);
        // ips114_show_int(188,75,real_left_up_x,3);
        // ips114_show_int(188,90,real_left_up_y,3);
        // ips114_show_int(188,105,real_right_up_x,3);
        // ips114_show_int(188,120,real_right_up_y,3);
    }
}

/**
 * @brief the function of get card center coordinate
 * @param int left_up_camera: the coordinate of left up point,
 *        int right_up_camera: the coordinate of right up point,
 *        int *real_x: the real x coordinate, int *real_y: the real y coordinate
 * @return None,in fact the canshu is transform by pointer
 */
void Get_Card_Center_coordinate(int left_up_camera_x, int left_up_camera_y, int right_up_camera_x, int right_up_camera_y, int *real_x, int *real_y)
{
    int x1 = 0, y1 = 0, x2 = 0, y2 = 0;                               // 定义左上和右上角的现实坐标
    Pespective_point(left_up_camera_x, left_up_camera_y, &x1, &y1);   // 将相机坐标转换为现实坐标
    Pespective_point(right_up_camera_x, right_up_camera_y, &x2, &y2); // 将相机坐标转换为现实坐标
    // 注意：卡片为正方形，求中心坐标就需要求出对角线的中心坐标

    /*用数学坐标系求解*/
    // int x0=(x1+x2)/2;
    // int y0=(y1+y2)/2;//计算中点坐标
    // double a = pow((x2 - x1), 2) / pow((y2 - y1), 2) + 1;
    // double b = -2 * x0 * pow((x2 - x1), 2) / pow((y2 - y1), 2) + 2 * x0;
    // double c = pow(x0, 2) * pow((x2 - x1), 2) / pow((y2 - y1), 2) - pow(x0, 2) - 3600;
    // double delta = pow(b, 2) - 4 * a * c;//计算判别式
    // double my_x1,my_x2,my_y1,my_y2;
    // if(delta < 0)   return;//如果判别式小于0，说明无解
    // else
    // {
    //      my_x1 = (-b + sqrt(delta)) / (2 * a);
    //      my_x2 = (-b - sqrt(delta)) / (2 * a);

    //      my_y1 = (-(x1 - x0) * (x2 - x1) / (y2 - y1)) + y0;
    //      my_y2 = (-(x2 - x0) * (x2 - x1) / (y2 - y1)) + y0;
    // }
    // if(abs(my_y1<=100)||abs(my_y2)<=100) return;
    // if((pow(my_x1,2)+pow(my_y1,2))>(pow(my_x2,2)+pow(my_y2,2)))//取坐标相近的点
    // {
    //     *real_x=my_x1;
    //     *real_y=my_y1;
    // }
    // else
    // {
    //     *real_x=my_x2;
    //     *real_y=my_y2;
    // }
    // 计算右下顶点的坐标
    double dx = x2 - x1;
    double dy = y2 - y1;
    double len = sqrt(dx * dx + dy * dy);
    double ratio = 120.0 / len;
    double x3 = x2 - dy * ratio;
    double y3 = y2 - dx * ratio;

    // 计算中心点的坐标
    // // 计算中心点的坐标
    *real_x = (x1 + x3) / 2.0;
    *real_y = (y1 + y3) / 2.0;
    /*display*/
    // ips114_show_int(188,0,left_up_x,3);
    // ips114_show_int(188,15,left_up_y,3);
    // ips114_show_int(188,30,right_up_x,3);
    // ips114_show_int(188,45,right_up_y,3);
    ips114_show_int(188, 0, *real_x, 3);
    ips114_show_int(188, 15, *real_y, 3);
    ips114_show_int(188, 60, x1, 3);
    ips114_show_int(188, 75, y1, 3);
    ips114_show_int(188, 90, x2, 3);
    ips114_show_int(188, 105, y2, 3);
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
    for (uint8 i = start_row - 1; i >= end_row + 1; i--) // Iterate through the rows
    {
        for (uint8 j = start_column + 1; j <= end_column - 1; j++) // Iterate through the columns
        {
            if (Image_Use[i - 1][j - 1] + Image_Use[i - 1][j] + Image_Use[i - 1][j + 1] + Image_Use[i][j - 1] + Image_Use[i][j + 1] + Image_Use[i + 1][j - 1] + Image_Use[i + 1][j] + Image_Use[i + 1][j + 1] >= threshold * WHITE_POINT)
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
    //????????��???????????
    /*???????????��?????*/
    for (uint8 i = IMAGE_HEIGHT - 1; i >= 1; i--)
    {
        if (Left_Lost_Flag[i] == 1)
            Left_Lost_Time++;
        if (Right_Lost_Flag[i] == 1)
            Right_Lost_Time++;
        if (Left_Lost_Flag[i] == 1 && Right_Lost_Flag[i] == 1)
            Both_Lost_Time++;
        if (Boundry_Start_Left == 0 && Left_Lost_Flag[i] == 0)
            Boundry_Start_Left = i; // Record the starting point of the left boundary
        if (Boundry_Start_Right == 0 && Right_Lost_Flag[i] == 0)
            Boundry_Start_Right = i;                 // Record the starting point of the right boundary
        Road_Wide[i] = right_line[i] - left_line[i]; // Record the road width
    }

    if (Road_Type != RAMP)
    {
        if (Left_Lost_Time <= 15 && Right_Lost_Time <= 15 && Both_Lost_Time <= 15)
            Road_Type = STRAIGHT_ROAD;
        if (Left_Lost_Time < 15 && Right_Lost_Time >= 30 && Both_Lost_Time < 15 && Search_Stop_Line <= 100)
            Road_Type = RIGHT_TURN;
        if (Right_Lost_Time < 15 && Left_Lost_Time >= 30 && Both_Lost_Time < 15 && Search_Stop_Line <= 100)
            Road_Type = LEFT_TURN;
        if (Left_Lost_Time >= 15 && Right_Lost_Time <= 5 && Both_Lost_Time <= 5 && Search_Stop_Line >= 100)
            Road_Type = LEFT_HUANDAO;
        left_island_flag = 1;
        if (Left_Lost_Time <= 5 && Right_Lost_Time >= 15 && Both_Lost_Time <= 5 && Search_Stop_Line >= 100)
            Road_Type = RIGHT_HUANDAO;
        right_island_flag = 1;
        if (Right_Lost_Time >= 30 && Left_Lost_Time >= 30 && Both_Lost_Time >= 30)
            Road_Type = CROSSING;
    }
    // if (Road_Type == STRAIGHT_ROAD)
    //     Ramp_Detect();
    if (Road_Type == STRAIGHT_ROAD)
        Zebra_Stripes_Detect();
    // if (Road_Type == RAMP)
    //     Ramp_Detect(); //??????
}

/**
 * @brief Function for continuity change detection on the left side
 * @param start: starting index, end: ending index, mode: detection mode
 * @return Continuity change flag
 */
int Continuity_Change_Left(int start, int end, int mode)
{
    int i, t, continuity_change_flag = 0;
    if (Left_Lost_Time >= 0.9 * IMAGE_HEIGHT)
        return 1; // Return 1 if left line is lost for more than 90% of the image height
    if (Search_Stop_Line <= 5)
        return 1; // Return 1 if search stop line is less than or equal to 5
    if (start >= IMAGE_HEIGHT - 1 - 5)
        start = IMAGE_HEIGHT - 1 - 5; // Adjust start index if it exceeds the image height
    if (end <= 5)
        end = 5; // Adjust end index if it is less than or equal to 5
    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }
    if (mode == 0)
    {
        for (i = start; i >= end; i--)
        {
            if (abs(left_line[i] - left_line[i - 1]) >= 5 && left_line[i - 1] >= 20 && left_line[i - 3] >= 20)
            {
                continuity_change_flag = i;
                break;
            }
        }
    }
    else if (mode == 1)
    {
        for (i = end; i <= start; i++)
        {
            if (abs(left_line[i] - left_line[i - 1]) >= 5 && left_line[i - 1] >= 20 && left_line[i - 3] >= 20)
            {
                continuity_change_flag = i;
                break;
            }
        }
    }
    return continuity_change_flag;
}

int Continuity_Change_Left_Island_State3(int start, int end) // 连续性阈值设置为5
{
    int i;
    int t;
    int continuity_change_flag = 0;
    if (Left_Lost_Time >= 0.9 * MT9V03X_H) // 大部分都丢线，没必要判断了
        return 1;
    if (Search_Stop_Line <= 5) // 搜所截止行很矮
        return 1;
    if (start >= MT9V03X_H - 1 - 5) // 数组越界保护
        start = MT9V03X_H - 1 - 5;
    if (end <= 5)
        end = 5;
    if (start > end) // 都是从下往上计算的，反了就互换一下
    {
        t = start;
        start = end;
        end = t;
    }

    for (i = start; i <= end; i++)
    {
        if (abs(left_line[i] - left_line[i + 1]) >= 5) // 连续判断阈值是5,可更改
        {
            continuity_change_flag = i;
            break;
        }
    }
    return continuity_change_flag;
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     左赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数
  Sample     Continuity_Change_Left(int start,int end);
  @note      连续性的阈值设置为5，可更改
-------------------------------------------------------------------------------------------------------------------*/
int Continuity_Change_Left_Island(int start, int end) // 连续性阈值设置为5
{
    int i;
    int t;
    int continuity_change_flag = 0;
    if (Left_Lost_Time >= 0.9 * MT9V03X_H) // 大部分都丢线，没必要判断了
        return 1;
    if (Search_Stop_Line <= 5) // 搜所截止行很矮
        return 1;
    if (start >= MT9V03X_H - 1 - 5) // 数组越界保护
        start = MT9V03X_H - 1 - 5;
    if (end <= 5)
        end = 5;
    if (start < end) // 都是从下往上计算的，反了就互换一下
    {
        t = start;
        start = end;
        end = t;
    }

    for (i = start; i >= end; i--)
    {
        if (abs(left_line[i] - left_line[i - 1]) >= 5) // 连续判断阈值是5,可更改
        {
            continuity_change_flag = i;
            break;
        }
    }
    return continuity_change_flag;
}

int Continuity_Change_Right_Island(int start, int end)
{
    int i;
    int t;
    int continuity_change_flag = 0;
    if (Right_Lost_Time >= 0.9 * MT9V03X_H) // 大部分都丢线，没必要判断了
        return 1;
    if (start >= MT9V03X_H - 5) // 数组越界保护
        start = MT9V03X_H - 5;
    if (end <= 5)
        end = 5;
    if (start < end) // 都是从下往上计算的，反了就互换一下
    {
        t = start;
        start = end;
        end = t;
    }

    for (i = start; i >= end; i--)
    {
        if (abs(right_line[i] - right_line[i - 1]) >= 5) // 连续性阈值是5，可更改
        {
            continuity_change_flag = i;
            break;
        }
    }
    return continuity_change_flag;
}

/**
 * @brief ?????????????
 * @param start:????? end:?????
 * @return ???????????????
 */
int Continuity_Change_Right(int start, int end, int mode)
{
    int i, t, continuity_change_flag = 0;
    if (Right_Lost_Time >= 0.9 * IMAGE_HEIGHT)
        return 1; //??????????????1
    if (start >= IMAGE_HEIGHT - 1 - 5)
        start = IMAGE_HEIGHT - 1 - 5; //????????
    if (end <= 5)
        end = 5; //????????
    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }
    if (mode == 0)
    {
        for (i = start; i >= end; i--)
        {
            if (abs(right_line[i] - right_line[i - 1]) >= 5) //?????????????????5
            {
                continuity_change_flag = i;
                break; //????????????��????????
            }
        }
    }
    else if (mode == 1)
    {
        for (i = end; i >= start; i++)
        {
            if (abs(right_line[i] - right_line[i - 1]) >= 5) //?????????????????5
            {
                continuity_change_flag = i;
                break; //????????????��????????
            }
        }
    }
    return continuity_change_flag; //????0?????��????????��??????????????��?????????
}

/**
 * @brief ????????
 * @param line:??????????
 * @return ??????????????
 */
uint8 Continuity_detect(uint8 *line)
{
    uint8 max_uncontinuity = 0; //????????
    for (uint8 i = IMAGE_HEIGHT - 1; i >= 1; i--)
    {
        if (line[i] - line[i - 1] > max_uncontinuity)
        {
            max_uncontinuity = line[i] - line[i - 1];
        }
    }
    return max_uncontinuity; //????????????
}

/**
 * @brief ???????????��????��??????????
 * @param ??
 * @return ??
 * @attention ??
 */
void Derivative_Change(void)
{
    for (uint8 i = IMAGE_HEIGHT - 1; i >= 1; i--)
    {
        Left_derivative[i] = (left_line[i] - left_line[i - 1]) / 2;
        Right_derivative[i] = (right_line[i] - right_line[i - 1]) / 2; //???????????��?
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
    float max_derivative = 0.00;
    for (uint8 i = IMAGE_HEIGHT - 1; i >= 1; i--)
    {
        if (line[i] > max_derivative)
        {
            max_derivative = line[i];
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
    float min_derivative = 0.00;
    for (uint8 i = IMAGE_HEIGHT - 1; i >= 1; i--)
    {
        if (line[i] < min_derivative)
        {
            min_derivative = line[i];
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
    int i, monotonicity_change_line = 0;
    if (Left_Lost_Time >= 0.9 * IMAGE_HEIGHT)
        return 1; //??????????????1
    if (start >= IMAGE_HEIGHT - 1 - 5)
        start = IMAGE_HEIGHT - 1 - 5; //????????
    if (end <= 5)
        end = 5; //????????
    if (start <= end)
        return 1; //????????��?????????��??????��?????
    for (i = start; i >= end; i--)
    {
        /*??????????????????????5?��???��??????????????????????????????????*/
        if (left_line[i] >= left_line[i + 5] && left_line[i] >= left_line[i - 5] &&
            left_line[i] >= left_line[i + 4] && left_line[i] >= left_line[i - 4] &&
            left_line[i] >= left_line[i + 3] && left_line[i] >= left_line[i - 3] &&
            left_line[i] >= left_line[i + 2] && left_line[i] >= left_line[i - 2] &&
            left_line[i] >= left_line[i + 1] && left_line[i] >= left_line[i - 1])
        {
            monotonicity_change_line = i;
            break;
        }
    }
    return monotonicity_change_line; //??????????????????
}

void Draw_Line(int startX, int startY, int endX, int endY)
{
    int i, x, y;
    int start = 0, end = 0;
    if (startX >= MT9V03X_W - 1) // �޷�����
        startX = MT9V03X_W - 1;
    else if (startX <= 0)
        startX = 0;
    if (startY >= MT9V03X_H - 1)
        startY = MT9V03X_H - 1;
    else if (startY <= 0)
        startY = 0;
    if (endX >= MT9V03X_W - 1)
        endX = MT9V03X_W - 1;
    else if (endX <= 0)
        endX = 0;
    if (endY >= MT9V03X_H - 1)
        endY = MT9V03X_H - 1;
    else if (endY <= 0)
        endY = 0;
    if (startX == endX) // һ������
    {
        if (startY > endY) // ����
        {
            start = endY;
            end = startY;
        }
        for (i = start; i <= end; i++)
        {
            if (i <= 1)
                i = 1;
            Image_Use[i][startX] = BLACK_POINT;
            Image_Use[i - 1][startX] = BLACK_POINT;
        }
    }
    else if (startY == endY) // ��һ������
    {
        if (startX > endX) // ����
        {
            start = endX;
            end = startX;
        }
        for (i = start; i <= end; i++)
        {
            if (startY <= 1)
                startY = 1;
            Image_Use[startY][i] = BLACK_POINT;
            Image_Use[startY - 1][i] = BLACK_POINT;
        }
    }
    else // ����������ˮƽ����ֱ��������������ǳ������
    {
        if (startY > endY) // ��ʼ�����
        {
            start = endY;
            end = startY;
        }
        else
        {
            start = startY;
            end = endY;
        }
        for (i = start; i <= end; i++) // �����ߣ���֤ÿһ�ж��кڵ�
        {
            x = (int)(startX + (endX - startX) * (i - startY) / (endY - startY)); // ����ʽ����
            if (x >= MT9V03X_W - 1)
                x = MT9V03X_W - 1;
            else if (x <= 1)
                x = 1;
            Image_Use[i][x] = BLACK_POINT;
            Image_Use[i][x - 1] = BLACK_POINT;
        }
        if (startX > endX)
        {
            start = endX;
            end = startX;
        }
        else
        {
            start = startX;
            end = endX;
        }
        for (i = start; i <= end; i++) // �����ߣ���֤ÿһ�ж��кڵ�
        {

            y = (int)(startY + (endY - startY) * (i - startX) / (endX - startX)); // ����ʽ����
            if (y >= MT9V03X_H - 1)
                y = MT9V03X_H - 1;
            else if (y <= 0)
                y = 0;
            Image_Use[y][i] = BLACK_POINT;
        }
    }
}

/**
 * @brief ???�n??????
 * @param int start ?????, int end ?????
 * @return ?????????��??????
 * @attention ??
 */
int Monotonicity_Change_Right(int start, int end)
{
    int i, monotonicity_change_line = 0;
    if (Right_Lost_Time >= 0.9 * IMAGE_HEIGHT)
        return 1; //??????????????1
    if (start >= IMAGE_HEIGHT - 1 - 5)
        start = IMAGE_HEIGHT - 1 - 5; //????????
    if (end <= 5)
        end = 5; //????????
    if (start <= end)
        return monotonicity_change_line; //????????��?????????��??????��?????
    for (i = start; i >= end; i--)       //?????????
    {
        /*??????????????????????5?��???��?????????��????????????????????????*/
        if (right_line[i] <= right_line[i + 5] && right_line[i] <= right_line[i - 5] &&
            right_line[i] <= right_line[i + 4] && right_line[i] <= right_line[i - 4] &&
            right_line[i] <= right_line[i + 3] && right_line[i] <= right_line[i - 3] &&
            right_line[i] <= right_line[i + 2] && right_line[i] <= right_line[i - 2] &&
            right_line[i] <= right_line[i + 1] && right_line[i] <= right_line[i - 1])
        {
            monotonicity_change_line = i;
            break;
        }
    }
    return monotonicity_change_line; //??????????????????
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
    last_err = err; //??��??????

    int weight_count = 0;                                     //??????
    for (int i = IMAGE_HEIGHT - 1; i > IMAGE_HEIGHT / 2; i--) //?????int i=IMAGE_HEIGHT-1;i>IMAGE_HEIGHT/2;i--?????????????????
    {
        err += (IMAGE_WIDTH / 2 - ((left_line[i] + right_line[i]) >> 1)) * Weight[i];
        weight_count += Weight[i]; //??????????
    }
    err = err / weight_count; //???????
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

void K_Draw_Line(float k, int startX, int startY, int endY)
{
    int endX = 0;

    if (startX >= MT9V03X_W - 1) // 限幅处理
        startX = MT9V03X_W - 1;
    else if (startX <= 0)
        startX = 0;
    if (startY >= MT9V03X_H - 1)
        startY = MT9V03X_H - 1;
    else if (startY <= 0)
        startY = 0;
    if (endY >= MT9V03X_H - 1)
        endY = MT9V03X_H - 1;
    else if (endY <= 0)
        endY = 0;
    endX = (int)((endY - startY) / k + startX); //(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
    Draw_Line(startX, startY, endX, endY);
}

/**
 * @brief ???��?????
 * @param int x1, int y1, int x2,int y2 ???????????????
 * @return ????????????????????????????
 */
void Left_Add_Line(int x1, int y1, int x2, int y2)
{
    int i, max, a1, a2, hx;
    //???????????????????
    if (x1 >= IMAGE_WIDTH)
        x1 = IMAGE_WIDTH - 1; //???????
    else if (x1 <= 0)
        x1 = 0;
    if (x2 >= IMAGE_WIDTH)
        x2 = IMAGE_WIDTH - 1; //???????
    else if (x2 <= 0)
        x2 = 0;
    if (y1 >= IMAGE_HEIGHT)
        y1 = IMAGE_HEIGHT - 1; //???????
    else if (y1 <= 0)
        y1 = 0;
    if (y2 >= IMAGE_HEIGHT)
        y2 = IMAGE_HEIGHT - 1; //???????
    else if (y2 <= 0)
        y2 = 0;
    a1 = y1;
    a2 = y2;     //?????????
    if (a1 > a2) //????????a1?????a2
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }
    for (i = a1; i <= a2; i++) //????????????????????
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1; //?????????????????????
        if (hx >= IMAGE_WIDTH)
            hx = IMAGE_WIDTH - 1; //???????
        else if (hx <= 0)
            hx = 0;
        left_line[i] = hx; //???????????????????????????????????????????????????????????????????????????????????????
    }
}

/**
 * @brief ????????
 * @param int x1, int y1, int x2,int y2 ???????????????????
 * @return ??????????????????????
 */
void Right_Add_Line(int x1, int y1, int x2, int y2)
{
    int i, max, a1, a2, hx;
    if (x1 >= IMAGE_WIDTH)
        x1 = IMAGE_WIDTH - 1; //???????
    else if (x1 <= 0)
        x1 = 0;
    if (x2 >= IMAGE_WIDTH)
        x2 = IMAGE_WIDTH - 1; //???????
    else if (x2 <= 0)
        x2 = 0;
    if (y1 >= IMAGE_HEIGHT)
        y1 = IMAGE_HEIGHT - 1; //???????
    else if (y1 <= 0)
        y1 = 0;
    if (y2 >= IMAGE_HEIGHT)
        y2 = IMAGE_HEIGHT - 1; //???????
    else if (y2 <= 0)
        y2 = 0;
    a1 = y1;
    a2 = y2;     //?????????
    if (a1 > a2) //????????a1?????a2
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }
    for (i = a1; i <= a2; i++) //????????????????????
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1; //?????????????????????
        if (hx >= IMAGE_WIDTH)
            hx = IMAGE_WIDTH - 1; //???????
        else if (hx <= 0)
            hx = 0;
        right_line[i] = hx; //???????????????????????????????????????????????????????????????????????????????????????
    }
}

/**
 * @brief the function of find the down point
 * @param int start, int end start row and the end row
 * @return the row index of Right_Down_Find, Left_Down_Find=0;
 */
void Find_Down_Point(int start, int end)
{
    int i, t;
    Right_Down_Find = 0;
    Left_Down_Find = 0; // initialize the left down find
    if (start < end)    // start must bigger than end
    {
        t = start;
        start = end;
        end = t;
    }
    if (start >= IMAGE_HEIGHT - 1 - 5)
        start = IMAGE_HEIGHT - 1 - 5; // limitation
    if (end <= IMAGE_HEIGHT - Search_Stop_Line)
        end = IMAGE_HEIGHT - Search_Stop_Line; // limitation
    if (end <= 5)
        end = 5;
    /*???????????????????????????????????????????????????*/
    for (i = start; i >= end; i--)
    {
        if (Left_Down_Find == 0 && abs(left_line[i] - left_line[i + 1]) <= 5 && abs(left_line[i + 1] - left_line[i + 2]) <= 5 &&
            abs(left_line[i + 2] - left_line[i + 3]) <= 5 && abs(left_line[i] - left_line[i - 2]) >= 8 && abs(left_line[i] - left_line[i - 2]) >= 15 &&
            abs(left_line[i] - left_line[i - 4]) >= 15) // the column must have big change
        {
            Left_Down_Find = i; // record the left down find
        }
        if (Right_Down_Find == 0 && abs(right_line[i] - right_line[i + 1]) <= 5 && abs(right_line[i + 1] - right_line[i + 2]) <= 5 &&
            abs(right_line[i + 2] - right_line[i + 3]) <= 5 && abs(right_line[i] - right_line[i - 2]) >= 8 && abs(right_line[i] - right_line[i - 2]) >= 15 && abs(left_line[i] - left_line[i - 4]) >= 15) // the column must have big change
        {
            Right_Down_Find = i; // record the right down find
        }
        if (Left_Down_Find != 0 && Right_Down_Find != 0)
            break; // if both find, then break to lessen the comsume time
    }
}

/**
 * @brief the function of caculate the inverse of a matrix
 * @param start: the starting index, end: the ending index
 * @return null
 */
void inverse(double a[3][3], double inv[3][3])
{
    double det = a[0][0] * (a[1][1] * a[2][2] - a[2][1] * a[1][2]) -
                 a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
                 a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if (det == 0)
    {
        printf("The matrix is not invertible.\n");
        return;
    }

    double invdet = 1 / det;

    inv[0][0] = (a[1][1] * a[2][2] - a[2][1] * a[1][2]) * invdet;
    inv[0][1] = -(a[0][1] * a[2][2] - a[0][2] * a[2][1]) * invdet;
    inv[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * invdet;
    inv[1][0] = -(a[1][0] * a[2][2] - a[1][2] * a[2][0]) * invdet;
    inv[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * invdet;
    inv[1][2] = -(a[0][0] * a[1][2] - a[1][0] * a[0][2]) * invdet;
    inv[2][0] = (a[1][0] * a[2][1] - a[2][0] * a[1][1]) * invdet;
    inv[2][1] = -(a[0][0] * a[2][1] - a[2][0] * a[0][1]) * invdet;
    inv[2][2] = (a[0][0] * a[1][1] - a[1][0] * a[0][1]) * invdet;
}

int Find_Left_Up_Point(int start, int end) // 找四个角点，返回值是角点所在的行数
{
    int i, t;
    int left_up_line = 0;
    if (Left_Lost_Time >= 0.9 * MT9V03X_H) // 大部分都丢线，没有拐点判断的意义
        return left_up_line;
    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }
    if (end <= MT9V03X_H - Search_Stop_Line) // 搜索截止行往上的全都不判
        end = MT9V03X_H - Search_Stop_Line;
    if (end <= 5) // 及时最长白列非常长，也要舍弃部分点，防止数组越界
        end = 5;
    if (start >= MT9V03X_H - 1 - 5)
        start = MT9V03X_H - 1 - 5;
    for (i = start; i >= end; i--)
    {
        if (left_up_line == 0 && // 只找第一个符合条件的点
            abs(left_line[i] - left_line[i - 1]) <= 5 &&
            abs(left_line[i - 1] - left_line[i - 2]) <= 5 &&
            abs(left_line[i - 2] - left_line[i - 3]) <= 5 &&
            (left_line[i] - left_line[i + 2]) >= 15 &&
            (left_line[i] - left_line[i + 3]) >= 15 &&
            (left_line[i] - left_line[i + 4]) >= 15)
        {
            left_up_line = i; // 获取行数即可
            break;
        }
    }
    return left_up_line; // 如果是MT9V03X_H-1，说明没有这么个拐点
}

int Find_Right_Down_Point(int start, int end) // 找四个角点，返回值是角点所在的行数
{
    int i, t;
    int right_down_line = 0;
    if (Right_Lost_Time >= 0.9 * MT9V03X_H) // 大部分都丢线，没有拐点判断的意义
        return right_down_line;
    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }
    if (start >= MT9V03X_H - 1 - 5) // 下面5行数据不稳定，不能作为边界点来判断，舍弃
        start = MT9V03X_H - 1 - 5;
    if (end <= MT9V03X_H - Search_Stop_Line)
        end = MT9V03X_H - Search_Stop_Line;
    if (end <= 5)
        end = 5;
    for (i = start; i >= end; i--)
    {
        if (right_down_line == 0 &&                        // 只找第一个符合条件的点
            abs(right_line[i] - right_line[i + 1]) <= 5 && // 角点的阈值可以更改
            abs(right_line[i + 1] - right_line[i + 2]) <= 5 &&
            abs(right_line[i + 2] - right_line[i + 3]) <= 5 &&
            (right_line[i] - right_line[i - 2]) <= -5 &&
            (right_line[i] - right_line[i - 3]) <= -10 &&
            (right_line[i] - right_line[i - 4]) <= -10)
        {
            right_down_line = i; // 获取行数即可
            break;
        }
    }
    return right_down_line;
}

int Find_Right_Up_Point(int start, int end) // 找四个角点，返回值是角点所在的行数
{
    int i, t;
    int right_up_line = 0;
    if (Right_Lost_Time >= 0.9 * MT9V03X_H) // 大部分都丢线，没有拐点判断的意义
        return right_up_line;
    if (start < end)
    {
        t = start;
        start = end;
        end = t;
    }
    if (end <= MT9V03X_H - Search_Stop_Line) // 搜索截止行往上的全都不判
        end = MT9V03X_H - Search_Stop_Line;
    if (end <= 5) // 及时最长白列非常长，也要舍弃部分点，防止数组越界
        end = 5;
    if (start >= MT9V03X_H - 1 - 5)
        start = MT9V03X_H - 1 - 5;
    for (i = start; i >= end; i--)
    {
        if (right_up_line == 0 &&                          // 只找第一个符合条件的点
            abs(right_line[i] - right_line[i - 1]) <= 5 && // 下面两行位置差不多
            abs(right_line[i - 1] - right_line[i - 2]) <= 5 &&
            abs(right_line[i - 2] - right_line[i - 3]) <= 5 &&
            (right_line[i] - right_line[i + 2]) <= -8 &&
            (right_line[i] - right_line[i + 3]) <= -15 &&
            (right_line[i] - right_line[i + 4]) <= -15)
        {
            right_up_line = i; // 获取行数即可
            break;
        }
    }
    return right_up_line;
}

/**
 * @brief the function to find the upper point
 * @param start: the starting index, end: the ending index
 * @return null
 */
void Find_Up_Point(int start, int end)
{
    int i, t; // temp variable
    if (Left_Down_Find != 0)
        Last_Left_Up_Find = Left_Down_Find; // initialize the last left up find
    if (Right_Down_Find != 0)
        Last_Right_Up_Find = Right_Down_Find; // initialize the last right up find
    Left_Up_Find = 0;                         //
    Right_Up_Find = 0;                        // initialize the left up find and right up find

    if (start < end) // start must bigger than end
    {
        t = start;
        start = end;
        end = t;
    }
    if (end <= IMAGE_HEIGHT - Search_Stop_Line)
        end = IMAGE_HEIGHT - Search_Stop_Line; // limitation
    if (end <= 5)
        end = 5;
    if (start >= IMAGE_HEIGHT - 1 - 5)
        start = IMAGE_HEIGHT - 1 - 5; // limitation
    /*begin to find*/
    for (i = end; i <= start; i++) // find the point by continuity
    {
        if (Left_Up_Find == 0 &&
            abs(left_line[i] - left_line[i - 1]) <= 5 &&
            abs(left_line[i - 1] - left_line[i - 2]) <= 5 &&
            abs(left_line[i - 2] - left_line[i - 3]) <= 5 &&
            abs(left_line[i] - left_line[i + 2]) >= 8 &&
            abs(left_line[i] - left_line[i + 3]) >= 15 &&
            abs(left_line[i] - left_line[i + 4]) >= 15) // the threshold of the continuity
        {
            Left_Up_Find = i; // record it
        }
        if (Right_Up_Find == 0 &&
            abs(right_line[i] - right_line[i - 1]) <= 5 &&
            abs(right_line[i - 1] - right_line[i - 2]) <= 5 &&
            abs(right_line[i - 2] - right_line[i - 3]) <= 5 &&
            abs(right_line[i] - right_line[i + 2]) >= 8 &&
            abs(right_line[i] - right_line[i + 3]) >= 15 &&
            abs(right_line[i] - right_line[i + 4]) >= 15) // the threshold of the continuity
        {
            Right_Up_Find = i; // record it
        }
        if (Left_Up_Find != 0 && Right_Up_Find != 0)
            break; // if both find, then break to lessen the comsume time
    }
    if (abs(Right_Up_Find - Left_Up_Find) >= 30 && left_line[Left_Up_Find] >= right_line[Right_Up_Find]) // if the difference between the left and the right point is greater than 30
    {
        Right_Up_Find = 0;
        Left_Up_Find = 0;
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
    int i, t;
    float k = 0.0;
    if (start >= IMAGE_HEIGHT - 1)
        start = IMAGE_HEIGHT - 1; // limitaion
    else if (start <= 0)
        start = 0; // limitation
    if (end >= IMAGE_HEIGHT - 1)
        end = IMAGE_HEIGHT - 1; // the limitation
    else if (end <= 0)
        end = 0;     // end limitation
    if (end < start) // start must be less than end
    {
        t = start;
        start = end;
        end = t;
    }
    if (start <= 5)
        Left_Add_Line(left_line[start], start, left_line[end], end); // if the start is less than or equal to 5, add a line to the left boundary
    else
    {
        k = (float)(left_line[start] - left_line[start - 4]) / 5.0; // calculate the slope of the line
        for (i = start; i <= end; i++)
        {
            left_line[i] = (int)(i - start) * k + left_line[start]; // lengthen the left boundary
            if (left_line[i] >= IMAGE_WIDTH - 1)
                left_line[i] = IMAGE_WIDTH - 1; // if the value exceeds the maximum width
            else if (left_line[i] <= 0)
                left_line[i] = 0; // if the value is less than or equal to 0
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
    int i, t;
    float k = 0.0;
    if (start >= IMAGE_HEIGHT - 1)
        start = IMAGE_HEIGHT - 1; // Check if start is greater than or equal to IMAGE_HEIGHT - 1
    else if (start <= 0)
        start = 0; // Check if start is less than or equal to 0
    if (end >= IMAGE_HEIGHT - 1)
        end = IMAGE_HEIGHT - 1; // Check if end is greater than or equal to IMAGE_HEIGHT - 1
    else if (end <= 0)
        end = 0;     // Check if end is less than or equal to 0
    if (end < start) // Swap start and end if end is less than start
    {
        t = start;
        start = end;
        end = t;
    }
    if (start <= 5)
        Right_Add_Line(right_line[start], start, right_line[end], end); // Add a line to the right boundary if start is less than or equal to 5
    else
    {
        k = (float)(right_line[start] - right_line[start - 4]) / 5.0; // Calculate the slope of the line
        for (i = start; i <= end; i++)
        {
            right_line[i] = (int)(i - start) * k + right_line[start]; // Lengthen the right boundary
            if (right_line[i] >= IMAGE_WIDTH - 1)
                right_line[i] = IMAGE_WIDTH - 1; // Check if the value exceeds the maximum width
            else if (right_line[i] <= 0)
                right_line[i] = 0; // Check if the value is less than or equal to 0
        }
    }
}

uint8 Find_Max_left_line(void)
{
    uint8 max[2] = {0};
    for (uint8 i = Boundry_Start_Left; i >= 10; i--) // 限幅
    {
        if (left_line[i] > max[0])
        {
            max[0] = left_line[i];
            max[1] = i;
        }
    }
    return max[1];
}
/**
 * @brief the function to detect the crossing
 * @param null
 * @return null
 */
void Cross_Detect(void)
{
    int down_search_start = 0; // the down point of finding the crossing
    if (Road_Type == CROSSING) // start to analyze the crossing if the state is corssing
    {
        Left_Up_Find = 0;
        Right_Up_Find = 0;
        if (Both_Lost_Time >= 15) // only find the left and the right point if the both lost time is greater than 15
        {
            Find_Up_Point(110, 6); // find the up point between the row 110 and 6
            if (Left_Up_Find == 0 && Right_Up_Find == 0)
                return; // return 0 if the left and the right point are not found
        }
        else
            return; // return 0 if the both lost time is less than 15(this isn't crossing
        if (Left_Up_Find != 0 && Right_Up_Find != 0)
        {
            down_search_start = Left_Up_Find > Right_Up_Find ? Left_Up_Find : Right_Up_Find; // find the max value of the left and the right point
            Find_Down_Point(IMAGE_HEIGHT - 5, down_search_start + 10);                       // find the down point between the row IMAGE_HEIGHT -5 and down_search_start+10
            if (Left_Down_Find <= Left_Up_Find)
                Left_Down_Find = 0; // if the left down point is less than the left up point, set the left down point to 0
            if (Right_Down_Find <= Right_Up_Find)
                Right_Down_Find = 0;                         // if the right down point is less than the right up point, set the right down point to 0
            if (Left_Down_Find != 0 && Right_Down_Find != 0) // if left down point and the right down point are not found, set the left down point and the right down point to fixed point
            {
                Left_Add_Line(left_line[Left_Up_Find], Left_Up_Find, left_line[Left_Down_Find], Left_Down_Find);        // left add line
                Right_Add_Line(right_line[Right_Up_Find], Right_Up_Find, right_line[Right_Down_Find], Right_Down_Find); // right add line
            }
            else if (Left_Down_Find == 0 && Right_Down_Find != 0)
            {
                Lengthen_Left_Boundry(Left_Up_Find - 1, IMAGE_HEIGHT - 1);                                              // lengthen the left boundary
                Right_Add_Line(right_line[Right_Up_Find], Right_Up_Find, right_line[Right_Down_Find], Right_Down_Find); // right add line
            }
            else if (Left_Down_Find != 0 && Right_Down_Find == 0) // if the left down point if found and the right down point if not find
            {
                Lengthen_Right_Boundry(Right_Up_Find - 1, IMAGE_HEIGHT - 1);                                     // lengthen the right boundary
                Left_Add_Line(left_line[Left_Up_Find], Left_Up_Find, left_line[Left_Down_Find], Left_Down_Find); // left add line
            }
            else if (Left_Down_Find == 0 && Right_Down_Find == 0) // if the left down point and the right down point are not found
            {
                Lengthen_Left_Boundry(Left_Up_Find - 1, IMAGE_HEIGHT - 1);   // lengthen the left boundary
                Lengthen_Right_Boundry(Right_Up_Find - 1, IMAGE_HEIGHT - 1); // lengthen the right boundary
            }
        }
    }
}

unsigned int Road_Min_Width[2] = {188, 0}; // Record the number of rows and width corresponding to the minimum road width
unsigned int Road_up_wide[5] = {0};
uint8 my_count = 0;
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
    my_count = 0;
    if (Road_Type != STRAIGHT_ROAD)
        return; // misjudgment detection
    /*Update minimum road width*/
    uint8 temp = 0;
    for (uint8 i = IMAGE_HEIGHT - 1; i >= IMAGE_HEIGHT - Search_Stop_Line; i--)
    {
        if (Road_Wide[i] < Road_Min_Width[0])
        {
            Road_up_wide[temp] = Road_Wide[i];
            Road_Min_Width[0] = (right_line[i] - left_line[i]); // Obtain the corresponding road width by subtracting
            Road_Min_Width[1] = i;                              // Record the number of rows corresponding to the minimum road width
        }
    }
    if (Road_Min_Width[1] >= 30)
        return;
    /*Judging whether it is a ramp based on standard road width*/
    for (uint8 i = Road_Min_Width[1]; i <= (Road_Min_Width[1] + 5); i++)
    {
        // Starting from the five lines down from the cut-off line, determine if there will be any exceeding the standard line width
        if ((Road_Wide[i] - Road_Min_Width[0]) >= 20)
        {
            my_count++;
        }
    }
    if (my_count >= 5 && init_flag == 1)
    {
        type = 8;
        Road_Type = RAMP; // If the abnormal width of the row exceeds a certain value, it is judged as a ramp
        ramp_flag = 1;
    }
    else
        Road_Type = STRAIGHT_ROAD; // Otherwise, the element will remain straight (without any changes)
}
/**
 * @brief Black and white detection function
 * @param uint8 row : the row that was fixed and counts for pixle
 *              uint8 start_colum the beginning column of start count
 *              uint8 end_column the end coulumn of end count
 * @return the number of white pixle
 */
uint8 Black_White_Dump(uint8 row, uint8 start_column, uint8 end_column)
{
    if (row >= IMAGE_HEIGHT - 1)
        row = IMAGE_HEIGHT - 1; // limitation
    else if (row <= 0)
        row = 0; // limitation
    if (row <= 5)
        return 0; // if row is too small,stop detecting
    else if (row >= IMAGE_HEIGHT - 1)
        return 0; // if row is too big,stop detecting
    if (start_column >= IMAGE_WIDTH - 1)
        start_column = IMAGE_WIDTH - 1; // limitation
    else if (start_column <= 0)
        start_column = 0; // limitation
    if (end_column >= IMAGE_WIDTH - 1)
        end_column = IMAGE_WIDTH - 1; // limitation
    else if (end_column <= 0)
        end_column = 0; // limitation
    if (row <= 30)
        row = 30; // limitation
    else if (row >= 89)
        row = 89; // limitation
    uint8 count = 0;
    uint8 count_for_temp = 0;
    uint8 first_white_column = 0;
    uint8 first_black_column = 0;
    uint8 white_point_count = 0;
    uint8 mode = 0;
    for (uint8 i = start_column; i <= end_column; i++)
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

        if (Image_Use[row][i] == WHITE_POINT)
        {
            // ips114_draw_point(i,row,RGB565_BLUE);
            white_point_count++; // count the white pixle
        }
    }

    // ips114_show_uint(188,60,row,3);
    // ips114_show_uint(188,80,end_column-start_column,3);
    // ips114_show_uint(188,100,white_point_count,3);
    // ips114_show_uint(188,120,abs(white_point_count-(end_column-start_column)),3);
    if (mode == 0)
    {
        if (abs(white_point_count - (end_column - start_column)) <= Zebra[row]) // judge by compare the number of white pixle and the width of the road
        {
            return 1;
        }
    }
    return 0;
}

void Top_Add_Line(int x1, int y1, int x2, int y2)
{
    int i, max, a1, a2, hy;
    // Ensure x1 and x2 are within the image width
    if (y1 >= IMAGE_HEIGHT)
        y1 = IMAGE_HEIGHT - 1;
    else if (y1 <= 0)
        y1 = 0;
    if (y2 >= IMAGE_HEIGHT)
        y2 = IMAGE_HEIGHT - 1;
    else if (y2 <= 0)
        y2 = 0;

    // Ensure y1 and y2 are within the image height
    if (x1 >= IMAGE_WIDTH)
        x1 = IMAGE_WIDTH - 1;
    else if (x1 <= 0)
        x1 = 0;
    if (x2 >= IMAGE_WIDTH)
        x2 = IMAGE_WIDTH - 1;
    else if (x2 <= 0)
        x2 = 0;

    a1 = x1;
    a2 = x2; // Set the start and end of the line

    if (a1 > a2) // Ensure a1 is always less than a2
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }

    for (i = a1; i <= a2; i++) // For each column between a1 and a2
    {
        hy = (i - x1) * (y2 - y1) / (x2 - x1) + y1; // Calculate the vertical position of the line in this column

        if (hy >= IMAGE_HEIGHT)
            hy = IMAGE_HEIGHT - 1; // Ensure hy is within the image height
        else if (hy <= 0)
            hy = 0;

        Island_surrond[i] = hy; // Store the vertical position of the line in this column
    }
}

uint8 Surround_continious_detect(uint8 start_column, uint8 end_column)
{
    uint8 rrtern = 0;
    if (start_column >= IMAGE_WIDTH - 5)
        start_column = IMAGE_WIDTH - 5;
    else if (start_column <= 5)
        start_column = 5;
    if (end_column >= IMAGE_WIDTH - 1)
        end_column = IMAGE_WIDTH - 5;
    else if (end_column <= 5)
        end_column = 5;

    if (start_column > end_column)
    {
        uint8 temp = start_column;
        start_column = end_column;
        end_column = temp;
    }

    for (uint8 i = start_column; i <= end_column; i++)
    {
        if (abs(Island_surrond[i] - Island_surrond[i - 1]) <= 5 && abs(Island_surrond[i] - Island_surrond[i - 2]) <= 5 && abs(Island_surrond[i] - Island_surrond[i - 4]) <= 5 && abs(Island_surrond[i] - Island_surrond[i + 1] >= 30) && abs(Island_surrond[i] - Island_surrond[i + 2] >= 30) && abs(Island_surrond[i] - Island_surrond[i + 4] >= 30))
        {
            rrtern = i;
            break;
        }
    }
    return rrtern;
}

uint8 last_right_point = 0;
void Surround_Analyse(void)
{
    for (uint8 i = 5; i < IMAGE_WIDTH - 5; i++)
    {
        if (Island_surrond[i] == IMAGE_HEIGHT - 2 && Island_surrond[i + 1] == 0 && Island_surrond[i + 3] == 0 && Island_surrond[i + 4] == 0)//如果检测到越变在整个范围的点
        {
            last_right_point = i;
            break;
        }
    }
}

float Island_Surround(uint8 target_row)
{
    uint8 continuious_flag = 0;
    for (uint8 i = 0; i < IMAGE_WIDTH - 1; i++)
    {
        for (uint8 j = IMAGE_HEIGHT - 2; j >= 2; j--)
        {
            if (Image_Use[j][i] == BLACK_POINT && Image_Use[j + 1][i] == WHITE_POINT)
            {
                Island_surrond[i] = j + 1;
                break;
            }
            else if (j == 2)
            {
                Island_surrond[i] = 0; // 此时丢线
            }
        }
    }
    Surround_Analyse();
    continuious_flag = Surround_continious_detect(IMAGE_WIDTH - 1, 10);
    if (continuious_flag != 0)
    {
        Top_Add_Line(continuious_flag,Island_surrond[continuious_flag],last_right_point,IMAGE_HEIGHT-2);//列补线，和前面的行补线不一样
    }
    island_err = 0.0;                             // 使用前先清零
    for (uint8 i = 10; i < IMAGE_WIDTH - 11; i++) // 记录对应的误差
    {
        island_err += Island_surrond[i] - target_row;
    }
    island_err = island_err / (IMAGE_WIDTH - 21); // 取平均值，不加权重了
    return island_err;
}

/**
 * @brief Island detection function
 * @param none
 * @return none
 */
void Island_Detect(void)
{
    static float k = 0;                  // 补线的斜率k
    static int left_down_guai[2] = {0};  // record the position of the the down point of the state1
    static int right_down_guai[2] = {0}; // record the position of the the up point of the state1
    int monotonicity_change_left_flag = 0;
    int monotonicity_change_right_flag = 0; // record the position of the left and right boundary monotonicity row
    int continuity_change_left_flag = 0;    // record the position of the left boundary continuity row
    int continuity_change_right_flag = 0;   // record the position of the right boundary continuity row
    int monotonicity_change_line[2];
    int Left_Up_Guai[2] = {0};                                                           // 定义左上角拐点的坐标
    continuity_change_left_flag = Continuity_Change_Left_Island(IMAGE_HEIGHT - 1, 10);   // find the position of the left boundary continuity row
    continuity_change_right_flag = Continuity_Change_Right_Island(IMAGE_HEIGHT - 1, 10); // find the position of the right boundary continuity row
    monotonicity_change_right_flag = Monotonicity_Change_Right(MT9V03X_H - 1 - 10, 10);
    monotonicity_change_left_flag = Monotonicity_Change_Left(MT9V03X_H - 1 - 10, 10);

    // ips114_show_uint(188, 40, Island_State, 3);
    // ips114_show_uint(188, 50, continuity_change_left_flag, 3);
    // ips114_show_uint(188, 60, Boundry_Start_Left, 3);

    /*the code of ips*/
    /*test the left island firstly*/
    switch (Island_State)
    {
    case 0: // the state of the island is 0:There is no access to the roundabout, only one side of the road is repaired
    {

        if (monotonicity_change_right_flag == 0 && continuity_change_left_flag != 0 && continuity_change_right_flag == 0 && continuity_change_left_flag >= 40)
        {
            left_down_guai[1] = left_line[continuity_change_left_flag]; // record the column of the left down point
            left_down_guai[0] = continuity_change_left_flag;            // record the row of the left down point
            if (left_down_guai[0] >= 30)
            {
                Island_State = 1; // change the state to 1
            }
        }
    }
    break;
    case 1:
    {
        /*patching line first*/
        left_down_guai[1] = left_line[continuity_change_left_flag]; // record the column of the left down point
        left_down_guai[0] = continuity_change_left_flag;            // record the row of the left down point
        // ips114_show_uint(188, 70, left_down_guai[0], 3);
        // ips114_show_uint(188, 80, left_down_guai[1], 3);
        Left_Add_Line(Longest_White_Column_Left[1], 3, left_down_guai[1], left_down_guai[0]);
        if (Boundry_Start_Left < 50) // 当左边线起点小于60时，进入状态2
        {
            Island_State = 2;
        }
    }
    break;

    case 2: // 左下角丢线，找到左边线的列坐标最大处
    {
        last_max_left_line = max_left_line;
        max_left_line = Find_Max_left_line(); // 寻找单调点
        // ips114_draw_line(94, 60, left_line[max_left_line], max_left_line, RGB565_RED);
        // monotonicity_change_line[0] = Monotonicity_Change_Left(70, 10); // 寻找单调性的点
        // monotonicity_change_line[1] = left_line[monotonicity_change_line[0]];
        if (Boundry_Start_Left <= IMAGE_HEIGHT - 5)
            Left_Add_Line(left_line[max_left_line], max_left_line, 3, 117);
        if (Boundry_Start_Left >= IMAGE_HEIGHT - 3 && abs(last_max_left_line - max_left_line) >= 30) // 单调点出现巨大撕裂
        {
            Island_State = 3;
        }
        // if ((Boundry_Start_Left >= IMAGE_HEIGHT - 5 || monotonicity_change_line[0] > 50))
        // {
        //     Island_State = 3; // 当圆弧靠下的时候，就进入状态3
        // }
    }
    break;

    case 3: // 找到左边丢线的起始点，然后开始往上扫点（和最长白列不一样），也可以用最长白列的寻找连续点方法，他那个似乎也好用？
    {

        left_up_state3_point[0] = Continuity_Change_Left_Island_State3(IMAGE_HEIGHT - 1, 5);

        ips114_draw_line(94, 60, left_line[left_up_state3_point[0]], left_up_state3_point[0], RGB565_BLUE);
        if (left_up_state3_point[0] != 0 && left_up_state3_point[1] != 0) // 如果找到点的话，直接补线
        {
            left_up_state3_point[1] = left_line[left_up_state3_point[0]];
        }
        Pespective_point(left_line[left_up_state3_point[0]], left_up_state3_point[0], &island_state3_real_x, &island_state3_real_y);
        ips114_show_int(188, 70, island_state3_real_x, 3);
        ips114_show_int(188, 80, island_state3_real_y, 3);
        Pespective_point_b(island_state3_real_x, island_state3_real_y, &camera_island_state3_x, &camera_island_state3_y);
        ips114_show_int(188, 90, camera_island_state3_x, 3);
        ips114_show_int(188, 100, camera_island_state3_y, 3);

        /*下面这个补线是旧的方案*/
        // Right_Add_Line(left_line[left_up_state3_point[0]], left_up_state3_point[0], right_line[117], 117); // 拉死线
        /*新的方案：此时补左边线*/
        // Left_Add_Line(left_line[left_up_state3_point[0]], left_up_state3_point[0], left_line[117], 117);
        // else // 找不到的话启动planb，这个拐点一般都是能找到的
        // {

        // for (uint8 i = IMAGE_HEIGHT - 5; i >= 5; i--)
        // {
        //     if (left_line[i] == 2 && left_line[i + 1] != 2) // 如果出现左边丢线就记录
        //     {
        //         start_row = i;
        //         break;
        //     }
        // }
        // // row如果找不到的话该怎么办？——一般都会找到，不会真找不到吧？

        // if (start_row != 0)
        // {
        //     for (uint8 j = 3; j <= Longest_White_Column_Left[1] + 10; j++) // 自左而右扫
        //     {
        //         for (uint8 i = start_row; i >= 5; i--)
        //         {
        //             if (Image_Use[i][j] == WHITE_POINT && Image_Use[i + 1][j] == BLACK_POINT)
        //             {
        //                 if (i >= state3_left_up_guai[0]&&j>=20) // 当行坐标达到最大值时，就取行坐标最大值
        //                 {
        //                     state3_left_up_guai[0] = i;
        //                     state3_left_up_guai[1] = j;
        //                 }
        //                 break;
        //             }
        //         }
        //     }
        // }
        // ips114_show_uint(188, 90, state3_left_up_guai[0], 3);
        // ips114_show_uint(188, 100, state3_left_up_guai[0], 3);
        // ips114_show_uint(188, 110, start_row, 3);
        // if (state3_left_up_guai[0] >= 0 && state3_left_up_guai[1] >= 0 && state3_left_up_guai[0] <= IMAGE_HEIGHT - 1 && state3_left_up_guai[1] <= IMAGE_WIDTH - 1)
        //     ips114_draw_line(94, 60, state3_left_up_guai[1], state3_left_up_guai[0], RGB565_BLUE);
        // if (state3_left_up_guai[0] != 0)
        // {
        //     Right_Add_Line(state3_left_up_guai[1], state3_left_up_guai[0], right_line[117], 117);
        // }
        // else // 如果还是没找到的话，就拉一条死线
        // {
        //     Right_Add_Line(3, 117, right_line[117], 117);
        // }
        // }
    }
    case 4: // 开始沿环岛跑，换一种求误差方式，当检测到
    {
        island_err = Island_Surround(80); // 目标行选择为80
        ips114_show_float(188, 0, island_err, 3, 3);
    }
    break;
    }
}
/**
 * @brief Zebra crossing detection function
 * @param none
 * @return none
 * @attention none
 */
void Zebra_Stripes_Detect(void)
{
    int continuity_change_right_flag = 0;   // Find the position of the left boundary continuity row
    int continuity_change_left_flag = 0;    // Find the position of the right boundary continuity row
    int monotonicity_change_right_flag = 0; // Find the position of the left boundary monotonicity row
    int monotonicity_change_left_flag = 0;  // Find the position of the right boundary monotonicity row

    continuity_change_left_flag = Continuity_Change_Left(IMAGE_HEIGHT - 1, 5, 0);     //
    continuity_change_right_flag = Continuity_Change_Right(IMAGE_HEIGHT - 1, 5, 0);   //
    monotonicity_change_left_flag = Continuity_Change_Left(IMAGE_HEIGHT - 1, 5, 1);   //
    monotonicity_change_right_flag = Continuity_Change_Right(IMAGE_HEIGHT - 1, 5, 1); //
    if (continuity_change_left_flag <= 30 || continuity_change_right_flag <= 30)
    {
        if (Road_Type == BANMAXIAN)
            Road_Type = STRAIGHT_ROAD; // If the number of lost wires on both sides is too small, stop judging
        return;
    }
    // ips114_draw_line(94,60,left_line[continuity_change_left_flag],continuity_change_left_flag,RGB565_RED);
    // ips114_draw_line(98,0,left_line[monotonicity_change_left_flag],monotonicity_change_left_flag,RGB565_BLUE);
    // ips114_show_uint(188,60,continuity_change_right_flag,3);
    // ips114_show_uint(188,80,continuity_change_left_flag,3);

    int i = 0, j = 0, change_count = 0, start_line = 0, endl_line = 0, narrow_road_count = 0;
    if (Search_Stop_Line >= 60 && 30 <= Longest_White_Column_Left[1] && Longest_White_Column_Left[1] <= IMAGE_WIDTH - 30 &&
        abs(continuity_change_left_flag - continuity_change_right_flag) <= 30 && continuity_change_left_flag != 0 && continuity_change_right_flag != 0)
    {
        uint8 count = Black_White_Dump(continuity_change_left_flag - 3, left_line[continuity_change_left_flag - 3], right_line[continuity_change_left_flag - 3]);
        uint8 higher_flag = (continuity_change_left_flag < continuity_change_right_flag) ? continuity_change_left_flag : continuity_change_right_flag;        // ???A��??B?????C????A??????C????B
        uint8 lower_flag = (monotonicity_change_left_flag > monotonicity_change_right_flag) ? monotonicity_change_left_flag : monotonicity_change_right_flag; // ???A????B?????C????A??????C????B
                                                                                                                                                              // if(Black_White_Dump((higher_flag+lower_flag)/2,left_line[higher_flag]-5,right_line[higher_flag]-5))
        {
            Road_Type = BANMAXIAN;
            if (Road_Type == STRAIGHT_ROAD)
                Road_Type = BANMAXIAN; // 如果当前道路类型为直行，则将道路类型设置为斑马线
        }
    }
}

void test2(void)
{

    if (Road_Type == STRAIGHT_ROAD)
        type = 1;
    else if (Road_Type == RIGHT_TURN)
        type = 2;
    else if (Road_Type == LEFT_TURN)
        type = 3;
    else if (Road_Type == CROSSING)
        type = 4;
    else if (Road_Type == BANMAXIAN)
        type = 5;
    else if (Road_Type == LEFT_HUANDAO)
        type = 6;
    else if (Road_Type == RIGHT_HUANDAO)
        type = 7;
//    else if (Road_Type == RAMP)
//        type = 8;
//    if (Road_Type == CROSSING)
//        Cross_Detect();
//    if (left_island_flag || right_island_flag)
//        Island_Detect();
    for (uint8 i = 0; i < IMAGE_HEIGHT - 1; i++)
    {
        ips114_draw_point((right_line[i] + left_line[i]) / 2, i, RGB565_RED);
    }
    // ips114_draw_point((left_line[i]+right_line[i])/2,i,RGB565_RED);

    //     ips114_draw_point(right_line[i],i,RGB565_GREEN);

    if (type == 4)
    {
        // ips114_draw_line(98,60,left_line[Left_Up_Find],Left_Up_Find,RGB565_GREEN);
        // ips114_draw_line(98,60,right_line[Right_Up_Find],Right_Up_Find,RGB565_BLUE);
        // ips114_draw_line(98,60,left_line[Left_Down_Find],Left_Down_Find,RGB565_RED);
        // ips114_draw_line(98,60,right_line[Right_Down_Find],Right_Down_Find,RGB565_YELLOW);
        // int real_left_down_x,real_left_down_y;
        // Pespective_point(left_line[Left_Down_Find],Left_Down_Find,&real_left_down_x,&real_left_down_y);
        // ips114_show_int(188,0,real_left_down_x,3);
        // ips114_show_int(188,15,real_left_down_y,3);
        // int real_right_down_x,real_right_down_y;
        // Pespective_point(right_line[Right_Down_Find],Right_Down_Find,&real_right_down_x,&real_right_down_y);
        // ips114_show_int(188,30,real_right_down_x,4);
        // ips114_show_int(188,45,real_right_down_y,4);
    }

    //    ips114_show_uint(188,120,threshold,3);
    ips114_displayimage03x(*Image_Use, 188, 120);

    float my_err = Err_Handle();
    extern int now_distance_x;
    extern unsigned int now_distance_y;
    //    ips114_show_int(0,0,now_distance_x,3);//显示卡片x坐标
    //    ips114_show_int(0,30,now_distance_y,3);//显示卡片x坐标
    /*
    66 16 131 15 39 84 158 85
    偏移21
    远点 72
    近点y 30-
    [[2.680653, 0.0, 7.629395e-06], [-5.036292e-08, 1.996892, 0.4895172], [2.452266e-10, -0.005439005, 1]]
    20
    */
    // ips114_show_float(188, 0, my_err, 2, 2);
    // ips114_show_uint(188, 10, Longest_White_Column_Left[1], 3);
    // ips114_show_uint(188, 20, type, 3);
    // ips114_show_int(188, 30, Search_Stop_Line, 3);
    // ips114_show_uint(188, 40, Boundry_Start_Left, 3);
    // ips114_show_uint(188, 50, Boundry_Start_Right, 3);

    // ips114_show_int(188, 90, Left_Lost_Time, 3);
    // ips114_show_int(188, 100, Right_Lost_Time, 3);
    // ips114_show_uint(188, 110, Both_Lost_Time, 3);
    //  ips114_show_int(188,90,now_distance_y,3);
    /*计算矩阵

    */
    // ips114_show_uint(188, 0, left_line[Left_Up_Find], 3);
    // ips114_show_uint(188, 15, Left_Up_Find, 3);
    // ips114_show_uint(188, 30, right_line[Right_Up_Find], 3);
    // ips114_show_uint(188, 45, Right_Up_Find, 3);
    // ips114_show_uint(188, 60, left_line[Left_Down_Find], 3);
    // ips114_show_uint(188, 75, Left_Down_Find, 3);
    // ips114_show_uint(188, 90, right_line[Right_Down_Find], 3);
    // ips114_show_uint(188, 105, Right_Down_Find, 3);
    // ips114_show_uint(188,30,type,3);
    // ips114_show_uint(188,45,right_line[Boundry_Start_Right],3);
    // ips114_show_uint(188,80,left_line[Boundry_Start_Left],3);
    // ips114_show_uint(188,100,Left_Lost_Time,3);
    // ips114_show_uint(188,120,Right_Lost_Time,3);
}

/**
 * @brief 程序的主函数
 * @param 无
 * @return 无
 */
void test(void)
{
    uint8 mode = 0; // 选择寻线模式

    if (mode == 1)
    {
        Image_Change();
        uint8 threshold = OSTU_GetThreshold((uint8 *)mt9v03x_image, IMAGE_WIDTH, IMAGE_HEIGHT);
        Simple_Binaryzation(*Image_Use, threshold);
        Center_line_deal(5, 183); // 处理中线
    }
    else if (mode == 0)
    {
        uint8 *output_address; // 图像第一个像素的地址
        /*注意：如果阈值在*/
        if (pick_up_mode == 0)
        {
            output_address = Scharr_Edge(*mt9v03x_image, 1700); // 使用扫描边缘的方式获取图像
            // uint8 threshold=OSTU_GetThreshold((uint8 *)mt9v03x_image,IMAGE_WIDTH,IMAGE_HEIGHT);
            // ips114_show_uint(188,15,the_max_G,4);
            memcpy(Image_Use, output_address, IMAGE_HEIGHT * IMAGE_WIDTH * sizeof(uint8));
            Center_line_deal_plus(23, 163); // 不能设置太高或太低的边界，否则会导致错误
            Outer_Analyse();
            island_err = Island_Surround(80); // 目标行选择为80
            ips114_show_float(188, 0, island_err, 3, 3);
            //    Easy_Filtering(110,20,30,170,5);
        }
        else // 如果处于拾取卡片的状态
        {
            output_address = Scharr_Edge_Simple(*mt9v03x_image); // 使用扫描边缘的方式获取图像
            uint8 threshold = OSTU_GetThreshold((uint8 *)mt9v03x_image, IMAGE_WIDTH, IMAGE_HEIGHT);
            // ips114_show_uint(188,15,the_max_G,4);
            memcpy(Image_Use, output_address, IMAGE_HEIGHT * IMAGE_WIDTH * sizeof(uint8));
            Simple_Binaryzation(*Image_Use, threshold); /*处理一张图片需要近9000us*/
            Easy_Filtering(110, 60, 30, 130, 5);
            Search_Center();
        }
    }
    test2();
}
