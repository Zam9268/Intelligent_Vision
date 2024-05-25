#include "communication.h"
#include "zf_driver_uart.h"
#include "zf_common_fifo.h"
#include "mymath.h"
#include "math.h"

uint8 uart_get_data[64];
uint8 fifo_get_data[64];//定义缓冲区信息存储数组
uint8 get_data =0;//定义接收数据
uint32 fifo_data_count =0;//定义缓冲区数据计数
fifo_struct uart_data_fifo;//定义缓冲区结构体
uint8 get_states=0;//定义接收状态
uint8 right_data[64]={0};//定义接收数据数组   
uint8 arm_uart_flag =0 ;//定义机械臂串口标志位
uint8 arm_uart_flag_on=0;
uint8 testuart_flag =0;//定义测试串口标志位
uint8 data_length=0;//定义数据长度
uint8 transform_counts=0;//定义数据转换计数
char str[]="why";//定义接收字符串get
/**
 * @brief 串口通信初始化
 * @param 无
 * @return 无
 */
void My_Communication_Init(void)
{
    fifo_init(&uart_data_fifo,FIFO_DATA_8BIT,uart_get_data,64);//初始化缓冲区
    uart_init(UART_1,115200,UART1_TX_B12,UART1_RX_B13);//初始化串口1通信模块
//    uart_init(UART_4,115200,UART4_TX_C16,UART4_RX_C17);//初始化串口2通信模块
    uart_rx_interrupt(UART_1,1);//串口1接收中断使能
//    uart_rx_interrupt(UART_4,1);//串口2接收中断使能
    NVIC_SetPriority(LPUART1_IRQn,0);//设置串口1中断优先级
//    NVIC_SetPriority(LPUART4_IRQn,1);//设置串口2中断优先级
}

/**
 * @brief 串口1接收中断处理函数
 * @param 无
 * @return 无
 * @attention 1. 这个是中断接收数据，省时间，不用等待，直接存入缓冲区（微机原理讲过的）
 */
void uart1_rx_interrupt_handler(void)
{
    uart_query_byte(UART_1,&get_data);//查询串口1接收数据，将接收数据存入get_data中，注意get_data是一个全局变量
    fifo_write_buffer(&uart_data_fifo,&get_data,1);//将get_data中的数据存入缓冲区中
}

/**
 * @brief 串口4接收中断处理函数
 * @param 无
 * @return 无
 */
void uart4_rx_interrupt_handler(void)
{
    uart_query_byte(UART_4,&get_data);//查询串口2接收数据，将接收数据存入get_data中，注意get_data是一个全局变量
    fifo_write_buffer(&uart_data_fifo,&get_data,1);//将get_data中的数据存入缓冲区中
}

/**
 * @brief 串口1,4接收数据函数
 * @param 无
 * @return 无
 * @attention  1.通过状态机来实现，和上一届师兄的有差别，改了一下，这个可以一次性多数据收发
 *             2.串口1用于art1（15分类识别）    串口4用于art2（目标检测）
 *              对于art1只需要发送一个数字，0~F，而对于art2需要发送两个数字为对应的坐标（发送的数目可以调整）
 */
void get_uartdata(void)
{
    fifo_data_count = fifo_used(&uart_data_fifo); //获取缓冲区数据计数
    
    if(fifo_data_count!=0)
    {
        if(get_states==0)//判断接收状态
        {
            fifo_read_buffer(&uart_data_fifo,fifo_get_data,&fifo_data_count,FIFO_READ_AND_CLEAN);
            //读取缓冲区数据并清空缓冲区
            if(fifo_get_data[0]==0xB7)  get_states=1;//判断是否为帧头，如果是帧头则进入下一个状态
            else get_states=0;//否则状态为0
            fifo_get_data[0]=0;//清空
        }
        else if(get_states==1)//接收状态为1
        {
            fifo_read_buffer(&uart_data_fifo,fifo_get_data,&fifo_data_count,FIFO_READ_AND_CLEAN);
            if(fifo_get_data[0]>=1&&fifo_get_data[0]<=16)   //判断是否为有效数据，数据的数量必须为1-16
            {
                transform_counts=fifo_get_data[0];//将接收到的数据存入transform_counts中
                fifo_get_data[0]=0;//清空
                get_states=2;//接收状态为2
            }
            else
            {
                get_states=0;//否则状态为0
                fifo_get_data[0]=0;//清空
            }
        }
        else if(get_states==2)//接收状态为2，开始读取对应的数据
        {
            static uint8 i = 0;//定义数据计数，注意这个是静态变量，每次进入这个函数都会保留上一次的值
            fifo_read_buffer(&uart_data_fifo,fifo_get_data,&fifo_data_count,FIFO_READ_AND_CLEAN);
            if(fifo_get_data[0]==0x98) // 判断是否为帧尾
            {
                if(transform_counts==i)//双重检测，判断实际上接收到的数据和发送端发送的数据是否一致
                {
                    data_length=i;//数据长度为i
                    i=0;//数组指针计数清零
                    get_states=0;//接收状态为0
                    for(uint8 j=data_length;j<64;j++)
                    {
                        right_data[j]=0;//清空数组
                    }
                    uart_write_string(UART_1,str);//发送字符串get，注意如果main.c里面用了vofa的话，就要注释掉(while1的printf函数)，否则也会发送给art，这样发送就会有问题
                    uart_data_handle();//数据处理函数
                }
                else
                {
                    for(uint8 j=0;j<i;j++)
                    {
                        right_data[j]=0;
                        get_states=0;
                    }
                }
            }
            else
            {
                right_data[i] = fifo_get_data[0]; // 将接收到的数据存入right_data数组中
                i++;//数据计数加1
                fifo_get_data[0]=0;//清空
            }
        }
        else
        {
            get_states=0;//否则状态为0
            fifo_get_data[0]=0;//清空
        }
    }
}

int last_distance_x;//目标检测算法中得到的目标x坐标
unsigned int last_distance_y;//得到的y坐标
int now_distance_x;
unsigned long now_distance_y;
unsigned int card_count;//目标检测算法中得到的卡片目标总数量
float center_distance;//目标检测算法中得到的目标中心距离
float last_center_distance;//目标检测算法中得到的上一次目标中心距离
uint8 find_card_flag=0;//寻找卡片标志位
/**
 * @brief 串口1和串口4接收的数据总处理函数
 * @param 无
 * @return 无
 * @attention 1. 如何区分串口1和串口4发送的数据呢？很简单，这里art（目标检测）连在串口1上面，一般目标检测都是发两个数据，所以这里的数据长度是2。
 *            而如果是art（分类检测）连在串口4上面，这个时候都是发一个数据，故这里的数据长度是1。
 *            2. 如果检测到的距离和上一次的距离距离较小，那么就默认为同一个目标，否则就是新的目标，对于同一个目标，只更新坐标，不更新数量。
 */
void uart_data_handle(void)
{
    if(data_length==5)//如果是目标检测接收到的数据
    {
        /*第一位：x轴正负 第二位:x坐标溢出系数  第三位：x坐标取余值
        第四位： y坐标溢出系数  第五位：y坐标取余值*/
        if(right_data[0]==1)
        {
            now_distance_x=(right_data[1]*256+right_data[2]);
        }
        else if(right_data[0]==0)//取负值
        {
            now_distance_x=-(right_data[1]*256+right_data[2]);
        }
        now_distance_y=(right_data[3]*255+right_data[4]);
        center_distance=sqrt(now_distance_x*now_distance_x+now_distance_y*now_distance_y);//求出中心距离
        if((center_distance - last_center_distance > 0 ? center_distance - last_center_distance : last_center_distance - center_distance) > 100.0) 
            card_count++;//坐标距离相差较大，就默认为检测到了新的卡片
        /*当卡片与字数距离非常接近时，就不再更新坐标，直接记录当前坐标*/
    }
}
/*
??????????
??????
01 firearms??????
02 explosives???????
03 dagger??????
04 spontoon????????
05 fire_axe ??????????
?????
06 first_aid_kit?????????
07 flashlight????????
08 intercom ?????????
09 bulletproof???????????
10 telescope?????????
11 helmet???????
????????
12 fire_engine??????????
13 ambulance?????????
14 armoredcar????????
15 motorcycle????г???

*/
