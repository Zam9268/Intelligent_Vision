#include "image.h"


/*边线数组变量定义*/
uint8 left_line[2][IMAGE_HEIGHT],right_line[2][IMAGE_HEIGHT];//左右边线数组
int center[IMAGE_HEIGHT];//中线数组
uint8 the_maxlen_position;//最长赛道位置
uint8 num;//定义有效行数
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

/**
 * @brief 中线处理（这里用的最长白列法）
 * @param H（赛道长度）
 * @return 无
 */
void Center_line_deal(uint8 H)
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

}
