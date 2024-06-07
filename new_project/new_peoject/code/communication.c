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
extern float Car_dis_x, Car_dis_y;//??????????x??y?????
extern float Angle_world;//?????????

int find_count;
int last_distance_x;          // ????????????????x????
unsigned int last_distance_y; // ????????????????y????
int now_distance_x;           // ??????????????x????
unsigned long now_distance_y;  // ??????????????y????
unsigned int card_count=0;      // 已记录的卡片数目
unsigned int pick_doen_count=0; // 已经被拾取的卡片数目
int center_distance;        // ???????????????????????
int last_center_distance;   // ?????????????????????????
uint8 find_card_flag = 0;     // ???????????????
uint8 card_type = 0;          // ?????????????1~15
Card card_position[100];
int one_time=1;
float Card_angle=0;//卡片方位角
float delta_card_angle;//用来查看
int find_newcard_flag;//是否找到新卡片
float last_card_world_x,last_card_world_y;//上次记录的卡片世界坐标
float card_world_x,card_world_y;//卡片世界坐标
float card_world_angle;//卡片世界坐标解算出的世界方位角
float car_card_angle;//车辆与原点的连线与卡片与原点的连线的所夹角
int watch_card_world_angle;
int card_world_distance;//卡片在全局坐标上与原点的距离
int car_card_diatance;//车辆与卡片的直线距离
uint8 card_abc=0;
uint8 card_num=0;

extern float car_world_angle;//车辆世界坐标解算出的方位角
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
        now_distance_y = (right_data[3] * 255 + right_data[4]);                                    // y坐标
				if(now_distance_x!=0 &&now_distance_y!=0)//测试使用，当有坐标传入时
				{	
            center_distance = (int)sqrt((now_distance_x/10) * (now_distance_x/10) + (now_distance_y/10) * (now_distance_y/10)); // 卡片的直线距离
					  Card_angle = atan2((now_distance_y/10),(now_distance_x/10))/PI*180*1.0;//捕获到卡片时的偏转角，转换成角度制
						delta_card_angle = (Card_angle-Angle_world)/180*PI;//转化成弧度制
						card_world_x = Car_dis_x + (float)center_distance * cos(delta_card_angle);//解算出世界x坐标,新卡片只记录一次！！！
            card_world_y = Car_dis_y + (float)center_distance * sin(delta_card_angle);//解算出世界y坐标
            card_world_distance = sqrt( card_world_x * card_world_x + card_world_y * card_world_y);//原点与卡片的距离
            card_world_angle = atan2(card_world_y,card_world_x)/PI*180*1.0;//角度制，卡片相对于原点解算出来的角度
//					  now_distance_x=0;//清空x坐标
//					  now_distance_y=0;//清空y坐标，避免跳出函数时art数据仍保留，导致卡片中心坐标仍在更新
				}
        for(uint8 i=0; i<card_count+1; i++)//遍历记录但未被拾取的卡片坐标
       {
				  find_count=i;//find_count记录变量
          if(card_position[i].pick_doen_flag==0)//只对未拾取的卡片作对比
         {
           if(fabsf(card_world_x-card_position[i].x_distance)<10 && fabsf(card_world_y-card_position[i].y_distance)<10)//当发现有卡片坐标与该卡片世界坐标很相近，认为是旧1卡片
           {
             find_newcard_flag=0;//不更新标志位
             break;
           }
         }
			 }
        if(find_count==card_count && find_newcard_flag==0)//遍历到了最后的一位，还是没有找到相同的
       {
         card_position[card_count].x_distance=card_world_x;//更新世界坐标x
         card_position[card_count].y_distance=card_world_y;//更新世界坐标y
				 card_position[card_count].world_distance=card_world_distance;//更新世界坐标y
				 card_count++; //卡片数量++，扩展数组
				 find_count=0;//清空find_count
       }
			 now_distance_x=0;//清空x坐标
			 now_distance_y=0;//清空y坐标，避免跳出函数时art数据仍保留，导致卡片中心坐标仍在更新
    }
       /* ????????????: ???????????????*/
        /* ???????????? */
     if (data_length == 1) // ????????????????????else if
        now_distance_y = (right_data[3] * 255 + right_data[4]);                                    // 计算y距离
        center_distance = sqrt(now_distance_x * now_distance_x + now_distance_y * now_distance_y); // 计算中心距离
        if ((center_distance - last_center_distance > 0 ? center_distance - last_center_distance : last_center_distance - center_distance) > 100.0)
        {
            for (uint8 i = 0; i < card_count; i++)
            {
                if (card_position[i].pick_doen_flag == 1) // 如果卡片已经被拾取
                    continue;
                else // 如果卡片未被拾取
                {
                    if (center_distance > card_position[card_count].add_distance) // 如果当前距离大于卡片的加权距离
                    {
                        card_position[card_count].x_distance = now_distance_x;                                                                                                                                                    // 更新卡片的x距离
                        card_position[card_count].y_distance = now_distance_y;                                                                                                                                                    // 更新卡片的y距离
                        card_position[card_count].add_distance = sqrt(card_position[card_count].x_distance * card_position[card_count].x_distance + card_position[card_count].y_distance * card_position[card_count].y_distance); // 更新加权距离
                        card_count++;                                                                                                                                                                                             // 卡片计数加1
                    }
                }
            }
            // card_count++; // 卡片计数加1
            // card_position[card_count-1].x_distance=now_distance_x;//更新卡片的x距离
            // card_position[card_count-1].y_distance=now_distance_y;//更新卡片的y距离
        }
        last_center_distance = center_distance; // 更新上一次的中心距离
    }
    /* 处理其他情况：发送串口数据 */
    /* 处理卡片类型数据 */
    uart_write_string(UART_1, str); // 发送串口数据
    if (data_length == 1)           // 如果数据长度为1
    {
        if (right_data[0] >= 1 && right_data[0] <= 15)
        {
            card_type = right_data[0];
        }
        uart_write_string(UART_4, str); // 发送串口数据
    }
    else if (data_length == 2) // 如果数据长度为2，表示接收到了卡片类型或卡片数量数据
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
    }
}
