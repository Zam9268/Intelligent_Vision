#include "communication.h"
#include "zf_driver_uart.h"
#include "zf_common_fifo.h"

uint8 uart_get_data[64];
uint8 fifo_get_data[64];//帧缓冲区，用于暂时存储运输过来的字和字节
uint8 get_data =0;//接收的单个的数据变量
uint32 fifo_data_count =0;//单次接收的数组个数
fifo_struct uart_data_fifo;//定义一个fifo结构体
uint8 get_states=0;//接收数据的状态
uint8 right_data[64]={0};//存储最终的数据    
uint8 arm_uart_flag =0 ;//发送给机械臂的拾取标志位
uint8 arm_uart_flag_on=0;
uint8 testuart_flag =0;//测试串口标志位

/**
 * @brief 串口及各种功能的初始化
 * @param 无
 * @return 无
 */
void My_Communication_Init(void)
{
    fifo_init(&uart_data_fifo,FIFO_DATA_8BIT,uart_get_data,64);//初始化fifo结构体
    uart_init(UART_1,115200,UART1_TX_B12,UART1_RX_B13);//初始化串口1，用于第一个art模块
    uart_init(UART_2,115200,UART2_TX_B18,UART2_RX_B19);//初始化串口2，用于第二个art模块
    uart_rx_interrupt(UART_1,1);//开启串口1接收中断
    // uart_rx_interrupt(UART_2,1);//开启串口2接收中断，这个语句无法执行，执行程序会卡死（目前找不到原因）！！！
    NVIC_SetPriority(LPUART1_IRQn,0);//设置串口1中断优先级，由于高版本不支持隐性函数声明，就替换成这个函数了
    // NVIC_SetPriority(LPUART2_IRQn,1);//设置串口2中断优先级
}

/**
 * @brief 串口接收回调中断（查询中断）
 * @param 无
 * @return 无
 * @attention 1. 虽然在微机原理中学的查询中断和这个查询中断确实是一样的，查询中断会浪费时间
 *            但是由于这个查询放在了中断回调，因此不会存在浪费时间的问题，在微机原理中一般都是把这个放在主函数中
 *            2. 这个fifo就类似于微机原理中的缓冲区，用于存储接收到的数据，因为主机接收数据如果没接收完，从机是不能发的。
 */
void uart1_rx_interrupt_handler(void)
{
    uart_query_byte(UART_1,&get_data);//查询串口1的数据,如果会返回数据，则会将数据存入get_data中（注意get_data是一个变量）
    fifo_write_buffer(&uart_data_fifo,&get_data,1);//将get_data中的数据存入fifo结构体（写入缓冲区）中
}

/**
 * @brief 串口接收回调中断（查询中断）
 * @param 无
 * @return 无
 */
void uart4_rx_interrupt_handler(void)
{
    uart_query_byte(UART_2,&get_data);//查询串口2的数据,如果会返回数据，则会将数据存入get_data中（注意get_data是一个变量）
    fifo_write_buffer(&uart_data_fifo,&get_data,1);//将get_data中的数据存入fifo结构体（写入缓冲区）中
}

/**
 * @brief 取出缓冲区中的数据
 * @param 无
 * @return 无
 * @attention  通过状态机的方式取出缓冲区中的数据
 */
void get_uartdata(void)
{
    fifo_data_count = fifo_used(&uart_data_fifo); //查看缓冲区是否存在数据
    static uint8 get_counts=0;//定义发送数据的数量
    if(fifo_data_count!=0)
    {
        if(get_states==0)//对对应的帧头
        {
            fifo_read_buffer(&uart_data_fifo,fifo_get_data,&fifo_data_count,FIFO_READ_AND_CLEAN);
            //将fifo结构体中的数据读取到fifo_get_data中
            if(fifo_get_data[0]==0xB7)  get_states=1;//如果接收到了帧头0xB7，则将get_states置为1（进入状态读取1）
            else get_states=0;//否则读取状态置0
            fifo_get_data[0]=0;//读取完信号后就将fifo_get_data[0]置0，防止重复读取
        }
        else if(get_states==1)//帧头读完，开始读取接收数据的个数
        {
            fifo_read_buffer(&uart_data_fifo,fifo_get_data,&fifo_data_count,FIFO_READ_AND_CLEAN);
            if(fifo_get_data[0]>=1&&fifo_get_data[0]<=16)   //读取的数据个数只能介于1和16个之间
            {
                get_counts=fifo_get_data[0];//存储发送的数据的个数
                fifo_get_data[0]=0;//清零
                get_states=2;//将读取状态置为2
            }
            else
            {
                get_states=0;//否则读取状态置0
                fifo_get_data[0]=0;//清零
            }
        }
        else if(get_states==2)//读取状态为2，读取对应的帧的内容
        {
            /*
            下面是原来师兄写的版本，但是我发现只能读取一个字节，当art发送一个列表时，还是只能接收前面一个数据，所以我觉得不是很好
            fifo_read_buffer(&uart_data_fifo,fifo_get_data,&fifo_data_count,FIFO_READ_AND_CLEAN);
            memcpy(right_data,fifo_get_data,sizeof(right_data));//对内容进行复制和拷贝
            get_states=2;//将读取状态置为2*/
            int i = 0;//数组下标指针
            while(1) //这段代码可以优化，等后面再优化
            {
                fifo_read_buffer(&uart_data_fifo,fifo_get_data,&fifo_data_count,FIFO_READ_AND_CLEAN);
                if(fifo_get_data[0]==0x98) // 如果读取到了帧尾
                { 
                    get_states=2;//将读取状态置为2
                    break;
                }
                right_data[i] = fifo_get_data[0]; // 没读到帧尾就将数据添加到right_data
                i++;//下标指针增加
                if(i==get_counts) //判断读取的数据是否等于发送的数据个数
                {
                    get_states=3;//将读取状态置3，这句话别放在Break后面，不然无法执行
                    break;//防止数组溢出）
                }
            }
        }
        else if(get_states==3)//读取状态为3，读取对应的包尾的内容
        {
            fifo_read_buffer(&uart_data_fifo,fifo_get_data,&fifo_data_count,FIFO_READ_AND_CLEAN);//保存到帧缓冲区
            if(fifo_get_data[0]==0x98)//如果读取到了帧尾
            {
                get_states=0;//将读取的状态置0
                uart_write_string(UART_1,"get");//发送回get信号
                fifo_get_data[0]=0;
                if(arm_uart_flag_on)    arm_uart_flag = 1;//如果拾取标志位开启，则将拾取标志位置1
            }
            else//这个时候如果读不到帧尾，说明读取出现了错误
            {
                get_states=0;//将读取的状态置0
                memset(right_data,0,sizeof(right_data));//将right_data数组清零（也可以用for循环，for循环可能快一点）
            }
        }
        else
        {
            get_states=0;//读取状态清零
            fifo_get_data[0]=0;
        }
    }
}
/*
对应的分类标志
武器：
01 firearms（枪支）
02 explosives（爆炸物）
03 dagger（匕首）
04 spontoon（警棍）
05 fire_axe （消防斧）
物资：
06 first_aid_kit（急救包）
07 flashlight（手电筒）
08 intercom （对讲机）
09 bulletproof（防弹背心）
10 telescope（望远镜）
11 helmet（头盔）
交通工具：
12 fire_engine（消防车）
13 ambulance（救护车）
14 armoredcar（装甲车）
15 motorcycle（摩托车）

*/
