#include "image.h"


/*边线数组变量定义*/
uint8 left_line[IMAGE_HEIGHT],right_line[IMAGE_HEIGHT];//左右边线数组
int center[IMAGE_HEIGHT];//中线数组
uint8 the_maxlen_position;//最长赛道位置
uint8 num;//定义有效行数
uint8 Longest_White_Column_Left[2];//从左到右最长白列（白列长度和所在列数）
uint8 Longest_White_Column_Right[2];//从右到左最长白列
uint8 Right_Lost_Flag[IMAGE_WIDTH];//右边界丢线标志
uint8 Left_Lost_Flag[IMAGE_WIDTH];//左边界丢线标志
uint8 Left_Lost_Time=0;//定义左丢线数
uint8 Right_Lost_Time=0;//定义右边界丢线数
uint8 Both_Lost_Time=0;//定义两个边界同时丢线数
uint8 Search_Stop_Line;//搜索终止行
uint8 Boundry_Start_Left,Boundry_Start_Right;//左右边界起始行
uint8 Road_Wide[IMAGE_HEIGHT];//定义赛道宽度数组
RoadType Road_Type;//定义赛道类型

/**
 * @brief 图像去噪（取周围8个邻域的点进行去噪）
 * @time_consuming：
 * @param bin_image
 * @return 无
 */
void Image_denoising(uint8 *bin_image)
{
    int bai;
    for(int j=1;j<IMAGE_HEIGHT-1;j++)
    {
        for(int i=1;i<IMAGE_WIDTH-1;i++)
        {
            if(bin_image[i+j*IMAGE_WIDTH]==255) continue;
            bin_image[i-1+j*IMAGE_WIDTH] 	+ bin_image[i+1+j*IMAGE_WIDTH] + 
					bin_image[i-1+(j-1)*IMAGE_WIDTH] +	bin_image[i+(j-1)*IMAGE_WIDTH] 	+bin_image[i+1+(j-1)*IMAGE_WIDTH] +	
					bin_image[i-1+(j+1)*IMAGE_WIDTH] +	bin_image[i+(j+1)*IMAGE_WIDTH] 	+bin_image[i+1+(j+1)*IMAGE_WIDTH] ;
            if( bai >= 1785 )
            bin_image[i+j*IMAGE_WIDTH] = 255;
        }
    }
    for(int j=1;j<IMAGE_HEIGHT-1;j++)
    {
        for(int i=1;i<IMAGE_WIDTH-1;i++)
        {
            if(bin_image[i+j*IMAGE_WIDTH]==0) continue;
            bin_image[i-1+j*IMAGE_WIDTH] 	+ bin_image[i+1+j*IMAGE_WIDTH] + 
					bin_image[i-1+(j-1)*IMAGE_WIDTH] +	bin_image[i+(j-1)*IMAGE_WIDTH] 	+bin_image[i+1+(j-1)*IMAGE_WIDTH] +	
					bin_image[i-1+(j+1)*IMAGE_WIDTH] +	bin_image[i+(j+1)*IMAGE_WIDTH] 	+bin_image[i+1+(j+1)*IMAGE_WIDTH] ;
            if( bai <=255 )
            bin_image[i+j*IMAGE_WIDTH] = 0;
        }
    }
    /*扫描底下三行*/
    for(x = IMAGE_HEIGHT-CENTER_LINE_START;x >= IMAGE_HEIGHT-CENTER_LINE_START-3;x--)    //x是减163-160  num是加0—4       
    {
        for(y = middle;y <= IMAGE_WIDTH-2;y++)    //中间向右找跳变
        {
            if(bin_image[x][y] == 0 && bin_image[x][y+1] == 0 && bin_image[x][y-1] == 255)  //两个连续黑点触发
            {
                right_line[0][num]=x;
                right_line[1][num]=y; 
                break;
            }
            else if(y==bin_width-2)       //到最右边了都没扫到黑点a
            {
                right_line[0][num]=x;
                right_line[1][num]=y;
                break;
            }
        }
        
        for(y = middle;y >= 1 ; y--)            //中间向左找跳变
        {
            if(bin_image[x][y] == 0 && bin_image[x][y-1] == 0 && bin_image[x][y+1] == 255)  //两个连续黑点触发
            {
                left_line[0][num]=x;
                left_line[1][num]=y;
                num=num+1;
                break;
            }
            else if(y==1)                 //到最左边了都没扫到黑点
            {
                left_line[0][num]=x;
                left_line[1][num]=y;
                num=num+1;
                break;
            } 
        }
    }
}

volatile int White_Column[IMAGE_WIDTH];//每列白列长度
/**
 * @brief 中线处理（这里用的大津法的最长白列法——sobel,canny的最长白列法还需要补充）
 * @param H（赛道长度）
 * @return 无
 */
void Center_line_deal(uint8 start_column,uint8 end_column,uint8 *image)
{
    /*数组清零*/
    for(int i=0;i<H-CENTER_LINE_START;ql++)   //清零函数
    {
        left_line[0][i]=0;
        left_line[1][i]=0;
        right_line[0][i]=0;
        right_line[1][i]=0;
        center[i]=0;
    }
    int x=0,y=0;//设x为行,y为列
    uint8 middle=the_maxlen_position;//定义赛道最长位置
    uint8 x_num;
    /*寻找最长白列（待优化） */
    for(j=start_column;j<end_column;j++)
    {
        for(i=0;i<IMAGE_HEIGHT;i++)
        {
            if(bin_image[i][j]==0)
            {
                White_Column[j]++;
            }
        }
    }
    /*从左到右寻找最长白列*/
    Longest_White_Column_Left[0]=0;//白列长度清零
    for(i=start_column;i<end_column;i++)
    {
        if(White_Column[i]>Longest_White_Column_Left[0])//最大值更替
        {
            Longest_White_Column_Left[0]=White_Column[i];
            Longest_White_Column_Left[1]=i;
        }
    }
    /*从右到左寻找最长白列*/
    Longest_White_Column_Right[0]=0;//白列长度清零
    for(i=end_column;i>start_column;i--)
    {
        if(White_Column[i]>Longest_White_Column_Right[0])//最大值更替
        {
            Longest_White_Column_Right[0]=White_Column[i];
            Longest_White_Column_Right[1]=i;//最长列的长度和所在列数更替
        }
    }
    /*终止行赋值*/
    Search_Stop_Line=Longest_White_Column_Left[0];//搜索截止行的赋值
    int right_border,left_border;//定义边界中间变量
    for(int i=IMAGE_HEIGHT-1;i>=IMAGE_HEIGHT-Search_Stop_Line;i--)
    {
        /*先寻右边界*/
        for(int j=Longest_White_Column_Left[1];j<=Longest_White_Column_Right[1];j++)
        {
            if(Image_Use[i][j]==WHITE_POINT&&Image_Use[i][j+1]==BLACK_POINT&&Image_Use[i][j+2]==BLACK_POINT)
            {
                right_border=j;//记录当前边界值
                Right_Lost_Flag[i]=0;//没有丢线，就置0
                break;
            }
            else if(j>=IMAGE_HEIGHT-1-2)//没有找到右边界时，就把最右边界赋值给右边，然后丢线标志位加1
            {
                right_border=j;
                Right_Lost_Flag[i]=1;
                break;
            }
        }
        for(j=Longest_White_Column_Left[1];j>=2;j--)
        {
            if(Image_Use[i][j]==WHITE_POINT&&Image_Use[i][j-1]==BLACK_POINT&&Image_Use[i][j-2]==BLACK_POINT)
            {
                left_border=j;//记录当前边界值
                Left_Lost_Flag[i]=0;//没有丢线，就置0
                break;
            }
            else if(j<=2)//没有找到左边界时，就把最左边界赋值给左边，然后丢线标志位加1
            {
                left_border=j;
                Left_Lost_Flag[i]=1;
                break;
            }
        }
        left_line[i]=left_border;//赋值给左边界数组
        right_line[i]=right_border;//赋值给右边界数组
    }
}

/**
 * @brief 边线数组分析（后期会加上逆透视，目前只是提取原始的边线数组）
 * @param 无
 * @return 无
 */
void Outer_Analyse(void)
{
    /*剩下条件补充*/
    for(uint8 i=IMAGE_HEIGHT-1;i>=0;i--)
    {
        if(Left_Lost_Flag[i]==1)    Left_Lost_Time++;
        if(Right_Lost_Flag[i]==1)   Right_Lost_Time++;
        if(Left_Lost_Flag[i]==1&&Right_Lost_Flag[i]==1)   Both_Lost_Time++;
        if(Boundry_Start_Left==0&&Left_Lost_Flag[i]==0)   Boundry_Start_Left=i;//记录第一个
        if(Boundry_Start_Right==0&&Right_Lost_Flag[i]==0) Boundry_Start_Right=i;//记录第一个
        Road_Wide[i]=right_line[i]-left_line[i];//赛道宽度数组
    }
    /*赛道类型判断*/
    if(Left_Lost_Time<=15&&Right_Lost_Time<=15&&Both_Lost_Time<=15) Road_Type=STRAIGHT_ROAD;
    if(Left_Lost_Time<15&&Right_Lost_Time>=30&&Both_Lost_Time<15)   Road_Type=RIGHT_TURN;
    if(Right_Lost_Time<15&&Left_Lost_Time>=30&&Both_Lost_Time<15)   Road_Type=LEFT_TURN;
    if(Right_Lost_Time>=30&&Left_Lost_Time>=30&&Both_Lost_Time>=30) Road_Type=CROSSING;

}

/**
 * @brief 误差处理函数
 * @param height:截止行数
 * @return 无
 */
float Err_Handle(uint8 height)
{
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
        err+=sum_err[i]/sum_hight;//均值分配
    }
    return err;
}
