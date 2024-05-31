#include "communication.h"
#include "zf_driver_uart.h"
#include "zf_common_fifo.h"
#include "mymath.h"
#include "math.h"

uint8 uart_get_data[64];
uint8 fifo_get_data[64];    // 用于存储接收到的数据
uint8 get_data = 0;         // 接收数据的变量
uint32 fifo_data_count = 0; // 用于存储接收数据的个数
fifo_struct uart_data_fifo; // 定义一个接收数据的结构体
uint8 get_states = 0;       // 接收状态
uint8 right_data[64] = {0}; // 接收到的数据存储数组
uint8 arm_uart_flag = 0;    // ARM串口通信标志位
uint8 arm_uart_flag_on = 0;
uint8 testuart_flag = 0;    // 测试串口通信标志位
uint8 data_length = 0;      // 数据长度
uint8 transform_counts = 0; // 数据转换计数
char str[] = "why";         // 发送字符串get
/**
 * @brief 串口通信初始化
 * @param 无
 * @return 无
 */
void My_Communication_Init(void)
{
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64); // 初始化接收数据缓冲区
    uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13);         // 初始化串口1通道模块
    uart_init(UART_4, 115200, UART4_TX_C16, UART4_RX_C17);         // 初始化串口2通道模块
    uart_rx_interrupt(UART_1, 1);                                  // 使能串口1接收中断
    uart_rx_interrupt(UART_4, 1);                                  // 使能串口2接收中断
    NVIC_SetPriority(LPUART1_IRQn, 0);                             // 设置串口1中断优先级
    NVIC_SetPriority(LPUART4_IRQn, 1);                             // 设置串口2中断优先级
}

/**
 * @brief 第1个串口接收中断处理函数
 * @param 无
 * @return 无
 * @attention 1. 该函数用于接收串口1的数据，并将接收到的数据存入get_data变量中，注意get_data是一个全局变量。
 */
void uart1_rx_interrupt_handler(void)
{
    uart_query_byte(UART_1, &get_data);               // 查询串口1的数据，并将数据存入get_data变量中
    fifo_write_buffer(&uart_data_fifo, &get_data, 1); // 将get_data中的数据写入缓冲区
}

/**
 * @brief 第4个串口接收中断处理函数
 * @param 无
 * @return 无
 */
void uart4_rx_interrupt_handler(void)
{
    uart_query_byte(UART_4, &get_data);               // 查询串口4的数据，并将数据存入get_data变量中
    fifo_write_buffer(&uart_data_fifo, &get_data, 1); // 将get_data中的数据写入缓冲区
}

/**
 * @brief 获取串口1和4的数据
 * @param 无
 * @return 无
 * @attention  1. 通过状态机实现数据的解析，首先判断是否为帧头，帧头为0xB7，如果是帧头则进入状态1。
 *             2. 在状态1下，判断接收到的数据是否为有效数据，有效数据范围为1-16，如果是有效数据，则将数据存入transform_counts变量中，进入状态2。
 *             3. 在状态2下，开始接收对应数据，直到接收到帧尾0x98，判断接收到的数据个数是否与transform_counts相等，如果相等则表示接收完整，进入状态0，否则进入状态0并清空数据。
 */
void get_uartdata(void)
{
    fifo_data_count = fifo_used(&uart_data_fifo); //获取缓冲区的数据个数

    if (fifo_data_count != 0)
    {
        if (get_states == 0) // 判断状态
        {
            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN); // 获取缓冲区的数据并清空缓冲区
            if (fifo_get_data[0] == 0xB7)
                get_states = 1; // 判断是否为帧头1，如果是
            else
                get_states = 0;   // ???0
            fifo_get_data[0] = 0; // ???????
        }
        else if (get_states == 1) // ???1
        {
            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN); // ??????????е??????????????
            if (fifo_get_data[0] >= 1 && fifo_get_data[0] <= 16)
            {
                transform_counts = fifo_get_data[0]; // ?ж???????Ч???????Ч?????Χ?1-16???????????transform_counts??????
                fifo_get_data[0] = 0;                // ???????
                get_states = 2;                      // ??????2
            }
            else
            {
                get_states = 0;       // ???0
                fifo_get_data[0] = 0; // ???????
            }
        }
        else if (get_states == 2) // ???2???????????????
        {
            static uint8 i = 0;                                                                      // ??????????????????????
            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN); // ??????????е??????????????
            if (fifo_get_data[0] == 0x98)                                                            // ?ж??????β
            {
                if (transform_counts == i) // ?ж???????????????????transform_counts??????
                {
                    data_length = i; // ????????i
                    i = 0;           // ????????
                    get_states = 0;  // ???0
                    for (uint8 j = data_length; j < 64; j++)
                    {
                        right_data[j] = 0; // ???????
                    }
                    
                    uart_data_handle();             // 数据处理函数
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
                right_data[i] = fifo_get_data[0]; // ????????????????right_data??????
                i++;                              // ??????1
                fifo_get_data[0] = 0;             // ???????
            }
        }
        else
        {
            get_states = 0;       // ???0
            fifo_get_data[0] = 0; // ???????
        }
    }
}

int last_distance_x;          // 上一次算法得到的目标点x坐标
unsigned int last_distance_y; // 上一次算法得到的目标点y坐标
int now_distance_x;           // 当前算法得到的目标点x坐标
unsigned long now_distance_y;  // 当前算法得到的目标点y坐标
unsigned int card_count;      // 算法得到的目标点上的卡片数量
float center_distance;        // 当前算法得到的目标点与原点的距离
float last_center_distance;   // 上一次算法得到的目标点与原点的距离
uint8 find_card_flag = 0;     // 是否找到卡片的标志位
uint8 card_type = 0;          // 卡片类型，范围为1~15
/**
 * @brief ????????1??4???????????
 * @param ??
 * @return ??
 * @attention 1. ?????????????????????????ж????????????0xB7????????????????1??
 *             2. ????1????ж?????????????????Ч???????Ч?????Χ?1-16?????????Ч??????????????transform_counts?????У???????2??
 *             3. ????2??????????????????????????β0x98???ж???????????????????transform_counts??????????????????????????????0???????????0??????????
 */
void uart_data_handle(void)
{
    if (data_length == 5) // ?ж??????????????????5
    {
        /* ???λ?x???????λ?????λ?x???????λ??????λ?y???????λ??????λ?y???????λ */
        if (right_data[0] == 1)
        {
            now_distance_x = (right_data[1] * 256 + right_data[2]); // x????????
        }
        else if (right_data[0] == 0)
        {
            now_distance_x = -(right_data[1] * 256 + right_data[2]); // x????????
        }
        now_distance_y = (right_data[3] * 255 + right_data[4]);                                    // y????
        center_distance = sqrt(now_distance_x * now_distance_x + now_distance_y * now_distance_y); // ???????
        if ((center_distance - last_center_distance > 0 ? center_distance - last_center_distance : last_center_distance - center_distance) > 100.0)
            card_count++; // 如果距离变化大于100.0，则默认为发现了新的卡片
        /* 处理卡片的逻辑 */
        uart_write_string(UART_1, str); // 向串口1发送字符串
    }
    else if (data_length == 1) // 此时为发送分类模式
    {
        if (right_data[0] >= 1 && right_data[0] <= 15)
        {
            card_type = right_data[0];
        }
        uart_write_string(UART_4, str); // 向串口1发送字符串
    }
}
