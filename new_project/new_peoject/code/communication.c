#include "communication.h"
#include "zf_driver_uart.h"
#include "zf_common_fifo.h"
#include "zf_device_gnss.h"
#include "mymath.h"
#include "control.h"
#include "math.h" 

uint8 uart_get_data[64];
uint8 fifo_get_data[64];    // 用于获取FIFO中的数据
uint8 get_data = 0;         // 用于获取的数据
uint32 fifo_data_count = 0; // FIFO中的数据数量
fifo_struct uart_data_fifo; // UART数据FIFO结构体
uint8 get_states = 0;       // 获取状态
uint8 right_data[64] = {0}; // 接收到的数据数组
uint8 arm_uart_flag = 0;    // ARM串口标志位
uint8 arm_uart_flag_on = 0;
uint8 testuart_flag = 0;         // 测试串口标志位
uint8 data_length = 0;           // 数据长度
uint8 transform_counts = 0;      // 转换计数
char str[] = "why";              // 字符串
char uart_4_begin[] = "fift";    // UART4开始字符串
char uart_4_begin_abc[] = "abc"; // UART4开始字符串abc
/**
 * @brief 初始化通信模块
 * @param 无
 * @return 无
 */
void My_Communication_Init(void)
{
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64); // 初始化UART数据FIFO
    uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);         // 初始化UART1串口
    uart_init(UART_4, 115200, UART4_TX_C16, UART4_RX_C17);         // 初始化UART4串口
    uart_rx_interrupt(UART_1, 1);                                  // 使能UART1接收中断
    uart_rx_interrupt(UART_4, 1);                                  // 使能UART4接收中断
    NVIC_SetPriority(LPUART1_IRQn, 0);                             // 设置UART1中断优先级
    NVIC_SetPriority(LPUART4_IRQn, 1);                             // 设置UART4中断优先级
}

/**
 * @brief UART1接收中断处理函数
 * @param 无
 * @return 无
 * @attention 1. 通过UART1接收到的数据存入get_data中，并将get_data写入FIFO中
 */
void uart1_rx_interrupt_handler(void)
{
    uart_query_byte(UART_1, &get_data);               // 查询UART1接收到的数据，并存入get_data中
    fifo_write_buffer(&uart_data_fifo, &get_data, 1); // 将get_data写入FIFO中
}

/**
 * @brief UART4接收中断处理函数
 * @param 无
 * @return 无
 */
void uart4_rx_interrupt_handler(void)
{
    uart_query_byte(UART_4, &get_data);               // 查询UART4接收到的数据，并存入get_data中
    fifo_write_buffer(&uart_data_fifo, &get_data, 1); // 将get_data写入FIFO中
}

/**
 * @brief 接收UART1和UART4数据
 * @param 无
 * @return 无
 * @attention  1. 如果接收到的数据为0xB7，则将状态切换为1
 *             2. 如果状态为1，则判断接收到的数据是否在1-16之间，如果是，则将transform_counts设置为接收到的数据，并将状态切换为2
 *             3. 如果状态为2，表示正在接收数据，如果接收到的数据为0x98，并且接收到的数据长度等于transform_counts，则将data_length设置为接收到的数据长度
 */
void get_uartdata(void)
{
    fifo_data_count = fifo_used(&uart_data_fifo); // 获取FIFO中的数据数量

    if (fifo_data_count != 0)
    {
        if (get_states == 0) // 如果状态为0
        {
            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN); // 从FIFO中读取数据并清空FIFO
            if (fifo_get_data[0] == 0xB7)
                get_states = 1; // 如果接收到的数据为0xB7，则将状态切换为1
            else
                get_states = 0;   // 否则将状态置为0
            fifo_get_data[0] = 0; // 清空获取数据数组
        }
        else if (get_states == 1) // 如果状态为1
        {
            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN); // 从FIFO中读取数据并清空FIFO
            if (fifo_get_data[0] >= 1 && fifo_get_data[0] <= 16)
            {
                transform_counts = fifo_get_data[0]; // 如果接收到的数据在1-16之间，则将transform_counts设置为接收到的数据
                fifo_get_data[0] = 0;                // 清空获取数据数组
                get_states = 2;                      // 将状态切换为2
            }
            else
            {
                get_states = 0;
                fifo_get_data[0] = 0; // 清空获取数据数组
            }
        }
        else if (get_states == 2) // 如果状态为2，表示正在接收数据
        {
            static uint8 i = 0;                                                                      // 用于记录接收到的数据长度
            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN); // 从FIFO中读取数据并清空FIFO
            if (fifo_get_data[0] == 0x98)                                                            // 如果接收到的数据为0x98
            {
                if (transform_counts == i) // 如果接收到的数据长度等于transform_counts
                {
                    data_length = i; // 数据长度为i
                    i = 0;           // 重置i
                    get_states = 0;  // 状态置为0
                    for (uint8 j = data_length; j < 64; j++)
                    {
                        right_data[j] = 0; // 填充剩余的数组元素为0
                    }

                    uart_data_handle(); // 处理接收到的数据
                }
                else
                {
                    for (uint8 j = 0; j < i; j++)
                    {
                        right_data[j] = 0;
                        get_states = 0;
                    }
                }
            }
            else
            {
                right_data[i] = fifo_get_data[0]; // 将接收到的数据存入right_data数组
                i++;                              // i加1
                fifo_get_data[0] = 0;             // 清空获取数据数组
            }
        }
        else
        {
            get_states = 0;       // 状态置为0
            fifo_get_data[0] = 0; // 清空获取数据数组
        }
    }
}
extern float Car_dis_x, Car_dis_y; //??????????x??y?????
extern float Angle_world;          //?????????

int last_distance_x;          // 上一次接收到的x距离
unsigned int last_distance_y; // 上一次接收到的y距离
int now_distance_x;           // 当前接收到的x距离
unsigned int now_distance_y;  // 当前接收到的y距离
unsigned int card_count;      // 卡片计数
float center_distance;        // 中心距离
float last_center_distance;   // 上一次的中心距离
int near_card_distance;       // 最近卡片的距离
int near_card_x;
int near_card_y;
uint8 find_card_flag = 0; // 是否找到卡片的标志
uint8 card_type = 0;      // 卡片类型，取值范围为1~15
Card card_position[100];
int one_time=1;
int card_word_ready;//卡片世界坐标解算完成的标志位
float Card_angle=0;//卡片方位角
float delta_card_angle;//用来查看
int find_oldcard_flag;//找到相似卡片
float last_card_world_x,last_card_world_y;//上次记录的卡片世界坐标
float card_world_x,card_world_y;//卡片世界坐标
float card_world_angle;//卡片世界坐标解算出的世界方位角
int watch_card_world_angle;
int card_world_distance;//卡片在全局坐标上与原点的距离
float car_card_angle;//车辆与原点的连线与卡片与原点的连线的所夹角
int car_card_diatance;//车辆与卡片的直线距离
uint8 card_abc=0;
uint8 card_num=0;

extern float car_world_angle;//车辆世界坐标解算出的方位角
/**
 * @brief 卡片全局坐标数组初始化
 * @param 对卡片结构体数组赋初值
 * @return 无
 */
void card_position_init(void)
{
  for (uint8 i = 0; i < 100; i++)
  {
    card_position[i].x_distance=0;	        //卡片x坐标
	card_position[i].y_distance=0;	        //卡片y坐标
	card_position[i].world_distance=0;	    //卡片与原点的距离
	card_position[i].world_angle=0.0;      //卡片在全局坐标的方位角
    card_position[i].pick_doen_flag=0;       //卡片拾取完成标志位 = 0.00;
  }
}
/**
 * @brief 处理接收到的串口数据
 * @param 无
 * @return 无
 * @attention 1. 如果接收到的数据为0xB7，表示接收到了卡片数据，将卡片数据存入card_position数组中
 *             2. 如果接收到的数据为0x98，表示接收到了卡片类型数据，将卡片类型存入card_type变量中
 *             3. 如果接收到的数据长度为5，表示接收到了x和y距离数据，计算中心距离并判断是否为有效数据
 */
void uart_data_handle(void)
{
    /**************** 有坐标传入时***********************/
    if (data_length == 5) // 如果数据长度为5
    {
        /* 处理x距离数据 */
        if (right_data[0] == 1)
        {
            now_distance_x = (right_data[1] * 256 + right_data[2]); // 计算x距离
        }
        else if (right_data[0] == 0)
        {
            now_distance_x = -(right_data[1] * 256 + right_data[2]); // 计算x距离（负值）
        }
        now_distance_y = (right_data[3] * 255 + right_data[4]);                                   // y坐标
        record_now_distance_x = now_distance_x;//记录下x坐标，防止最后x坐标清0后无法观察
        record_now_distance_y = now_distance_y;//记录下y坐标，防止最后y坐标清0后无法观察
		if(now_distance_x!=0 &&now_distance_y!=0)//测试使用，当有坐标传入时,此时x坐标y坐标已赋值
		{	
            /**************解算在这一次的传入坐标所解算出的世界坐标*********************/
            center_distance = (int)sqrt((now_distance_x/10) * (now_distance_x/10) + (now_distance_y/10) * (now_distance_y/10)); // 卡片的直线距离
			Card_angle = atan2((now_distance_y/10),(now_distance_x/10))/PI*180*1.0;//捕获到卡片时的偏转角，转换成角度制
			delta_card_angle = (Card_angle-Angle_world)/180*PI;//转化成弧度制
			card_world_x = Car_dis_x + (float)center_distance * cos(delta_card_angle);//解算出世界x坐标,新卡片只记录一次！！！
            card_world_y = Car_dis_y + (float)center_distance * sin(delta_card_angle);//解算出世界y坐标
            /*************************利用这张卡片的世界坐标来解算角度******************/
            card_world_distance = sqrt( card_world_x * card_world_x + card_world_y * card_world_y);//原点与卡片的距离
            card_world_angle = atan2(card_world_y,card_world_x)/PI*180*1.0;//角度制，卡片相对于原点解算出来的角度，这个还是会记录最后识别出的角度
            // card_word_ready = YES;//卡片坐标已准备完毕
            //后续解算在control.c的里程计中断中进行解算
		}
        for(uint8 i=0; i<card_count+1; i++)//遍历记录但未被拾取的卡片坐标
       {
		  find_count=i;//find_count记录变量i
          if(card_position[i].pick_doen_flag==0)//只对未拾取的卡片作对比
         {
           if(fabsf(card_world_x-card_position[i].x_distance)<10 && fabsf(card_world_y-card_position[i].y_distance)<10)//当发现有卡片坐标与该卡片世界坐标很相近，认为是旧1卡片
           {
            find_oldcard_flag=YES;//找到了旧卡片
            break;
           }
         }
		}
        if(find_count==card_count && find_oldcard_flag== NO)//遍历到了最后的一位，还是没有找到相同的
       {
         card_position[card_count].x_distance=card_world_x;//更新世界坐标x
         card_position[card_count].y_distance=card_world_y;//更新世界坐标y
		 card_position[card_count].world_distance=card_world_distance;//更新世界坐标y
         card_position[card_count].world_angle=card_world_angle;//更新卡片方位角
         card_position[card_count].card_word_ready = YES;//卡片坐标已准备完毕
		 card_count++; //该张卡片已存入，卡片数量++，扩展数组的下一位
		 find_count=0;//清空find_count
       }
		// now_distance_x=0;//清空x坐标
		// now_distance_y=0;//清空y坐标，避免跳出函数时art数据仍保留，导致卡片中心坐标仍在更新
        // card_word_ready=NO;//清空卡片世界坐标准备标志位
        find_oldcard_flag=NO;//清空旧卡片标志位
    }
       /* ????????????: ???????????????*/
        /* ???????????? */
     if (data_length == 1) // ????????????????????else if
     {
        now_distance_y = (right_data[3] * 255 + right_data[4]);                                    // 计算y距离
        center_distance = sqrt(now_distance_x * now_distance_x + now_distance_y * now_distance_y); // 计算中心距离
//        if ((center_distance - last_center_distance > 0 ? center_distance - last_center_distance : last_center_distance - center_distance) > 100.0)
        last_center_distance = center_distance; // 更新上一次的中心距离
        uart_write_string(UART_1, str);         // 发送串口数据
    }
    /* 处理其他情况：发送串口数据 */
    /* 处理卡片类型数据 */
    if (data_length == 6) // 如果数据长度为6,表示接收到了卡片类型数据
    {
        if (right_data[0] >= 1 && right_data[0] <= 15)
        {
            card_type = right_data[0];
        }
        if (right_data[1] == 0) // 如果x坐标是负的
        {
            near_card_x = -(right_data[2] * 256 + right_data[3]);
        }
        else
        {
            near_card_x = right_data[2] * 256 + right_data[3];
        }
        near_card_y = right_data[4] * 256 + right_data[5];
        uart_write_string(UART_4, str); // 发送串口数据
    }
    if (data_length == 7) // 如果数据长度为7，表示接收到了字母数字数据
    {
        if (right_data[0] == 0x01)
        {
            card_abc = right_data[1];
            card_num = 0; // 卡片数量清零
        }
        else if (right_data[0] == 0x02)
        {
            card_num = right_data[1];
            card_abc = 0; // 卡片类型清零
        }
        if (right_data[2] == 0) // 如果x坐标是负的
        {
            near_card_x = -(right_data[3] * 256 + right_data[4]);
        }
        else
        {
            near_card_x = right_data[3] * 256 + right_data[4];
        }
        near_card_y = right_data[5] * 256 + right_data[6];
        uart_write_string(UART_4, str); // 发送串口数据
    }
}
