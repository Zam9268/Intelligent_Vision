/*********************************************************************************************************************
 * RT1064DVL6A Opensourec Library ????RT1064DVL6A ???????????????? SDK ??????????????
 * Copyright (c) 2022 SEEKFREE ?????
 *
 * ??????? RT1064DVL6A ???????????
 *
 * RT1064DVL6A ????? ?????????
 * ?????????????????????????? GPL??GNU General Public License???? GNU??��????????????????
 * ?? GPL ???3?��?? GPL3.0????????????�ʦ�?????��?????��?????/???????
 *
 * ????????????????????????????????��???????�ʦ�???
 * ?????????????????????????????????
 * ?????????��? GPL
 *
 * ?????????????????????????? GPL ?????
 * ?????��??????<https://www.gnu.org/licenses/>
 *
 * ?????????
 * ?????????? GPL3.0 ????????��?? ?????????????????��
 * ?????????????? libraries/doc ???????? GPL3_permission_statement.txt ?????
 * ??????????? libraries ??????? ???????????? LICENSE ???
 * ?????��??��??????????? ?????????????????????????????????????????
 *
 * ???????          isr
 * ???????          ??????????????
 * ?��???          ?? libraries/doc ??????? version ??? ?��???
 * ????????          IAR 8.32.4 or MDK 5.33
 * ??????          RT1064DVL6A
 * ????????          https://seekfree.taobao.com/
 *
 * ?????
 * ????              ????                ???
 * 2022-09-21        SeekFree            first version
 ********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "isr.h"
#include "control.h"
#include "take.h"
#include "imu660ra.h"
#include "my_key.h"
#include "image.h"

extern int uart1_flag;
extern int uart4_flag;
extern int correct_art2_flag;
extern int classify_art2_flag;
extern char uart_4_begin[];
extern char uart_4_begin_abc[]; // 外部声明
extern pid_info Speed[4];       // �ⲿ����
extern uint8 step;
extern char send_mode;
extern char uart_4_begin[];
extern char uart_4_begin_abc[];
extern fifo_struct uart_data_fifo; // UART数据FIFO结构体
extern uint8 uart_get_data[64];
int count = 0;
int arm_flag = 0;
unsigned int init_count = 0;
extern uint8 init_flag;
unsigned int my_sceond_count = 0;
uint8 seconds = 0;
uint8 ramp_begin_detect_flag = 0;
uint8 change = 0;

char uart_1_begin[] = " start"; // UART4开始字符串
char uart_1_stop[] = " stop";   // UART4开始字符串abc

void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler(); // ????SDK??????��???? ?????????????????????????????
    __DSB();                // ???????????
}

void PIT_IRQHandler(void)
{
    if (pit_flag_get(PIT_CH0))
    {
        // 读取编码器w
        if (init_flag)
        {
            Read_Encoder();
            increment_pid();
            motor_close_control();
            pit_flag_clear(PIT_CH0);
        }
    }

    if (pit_flag_get(PIT_CH1))
    {
        static uint8 key_xiaodou=0;
        key_xiaodou++;
        if(key_xiaodou==4)//消抖间隔为20ms
        {
            key_scan();
			key_xiaodou=0;
        }
        // ��ȡ������
        //  Read_Encoder();
        Get_angle();
        Encoder_odometer();
        pit_flag_clear(PIT_CH1);
    }

    if (pit_flag_get(PIT_CH2))
    {
        if (init_flag == 0) // 开机后计时1s，用于定时初始化（防止开机就�?坡道�?
        {
            init_count++;
            if (init_count == 15)
            {
                init_flag = 1;
                Motor_Init(); // 电机初始化
            }
        }

        pit_flag_clear(PIT_CH2); //
    }

    if (pit_flag_get(PIT_CH3))
    {

        my_sceond_count++;
        if (my_sceond_count == 2)
        {
            my_sceond_count = 0;
            seconds++;
        }
        if (seconds >= 10) // 十秒过后才会开始检测坡道
        {
            ramp_begin_detect_flag = 1;
        }
        if (correct_art2_flag == 1)
        {
            // send_deal();
            uart_write_string(UART_4, uart_4_begin);
            //           correct_art2_flag=0;
        }
		if(classify_art2_flag == 1)
		{
			uart_write_string(UART_4, uart_4_begin_abc);
    }
        pit_flag_clear(PIT_CH3);
    }

    __DSB();
}

void LPUART1_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
        // 接收中断
        // #if DEBUG_UART_USE_INTERRUPT       // 如果使用 debug 中断
        //         debug_interrupr_handler(); // 调用 debug 中断处理函数，将接收到的数据保存到 debug 缓冲区中
        // #endif                             // 如果没有修改 DEBUG_UART_INDEX 的话就不需要去掉
        extern void UART1_handler(void); //?????????
                                         //			 if(uart1_flag==OPEN)
                                         //			 {
        UART1_handler();
        //			 }
    }

    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag); // 清除接收溢出标志位
}

void LPUART2_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
        // 接收中断
    }

    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag); // 清除接收溢出标志位
}

void LPUART3_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        // 接收中断
    }

    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag); // 清除接收溢出标志位
}

void LPUART4_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        // 接收中断
        extern void UART4_handler(void); //?????????
                                         //			 if(uart4_flag==OPEN)
                                         //			 {
        UART4_handler();
        //			 }
    }

    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag); // 清除接收溢出标志位
}

void LPUART5_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        // 接收中断
        camera_uart_handler();
    }

    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag); // 清除接收溢出标志位
}

void LPUART6_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        // 接收中断
    }

    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag); // 清除接收溢出标志位
}

void LPUART8_IRQHandler(void)
{
    if (kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        // 接收中断
        wireless_module_uart_handler();
    }

    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag); // 清除接收溢出标志位
}

void GPIO1_Combined_0_15_IRQHandler(void)
{
    if (exti_flag_get(B0))
    {
        exti_flag_clear(B0); // 清除中断标志位
    }
}

void GPIO1_Combined_16_31_IRQHandler(void)
{
    wireless_module_spi_handler();
    if (exti_flag_get(B16))
    {
        exti_flag_clear(B16); // 清除中断标志位
    }
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    flexio_camera_vsync_handler();

    if (exti_flag_get(C0))
    {
        exti_flag_clear(C0); // 清除中断标志位
    }
}

void GPIO2_Combined_16_31_IRQHandler(void)
{
    // -----------------* ToF INT 中断预处理函数 *-----------------
    tof_module_exti_handler();
    // -----------------* ToF INT 中断预处理函数 *-----------------

    if (exti_flag_get(C16))
    {
        exti_flag_clear(C16); // 清除中断标志位
    }
}

void GPIO3_Combined_0_15_IRQHandler(void)
{

    if (exti_flag_get(D4))
    {
        exti_flag_clear(D4); // 清除中断标志位
    }
}

/*
中断服务函数命名规则，根据对应的中断源命名
Sample usage:当前工程使用的中断
void PIT_IRQHandler(void)
{
    //处理中断标志位
    __DSB();
}
以下是已经定义的中断服务函数
CTI0_ERROR_IRQHandler
CTI1_ERROR_IRQHandler
CORE_IRQHandler
FLEXRAM_IRQHandler
KPP_IRQHandler
TSC_DIG_IRQHandler
GPR_IRQ_IRQHandler
LCDIF_IRQHandler
CSI_IRQHandler
PXP_IRQHandler
WDOG2_IRQHandler
SNVS_HP_WRAPPER_IRQHandler
SNVS_HP_WRAPPER_TZ_IRQHandler
SNVS_LP_WRAPPER_IRQHandler
CSU_IRQHandler
DCP_IRQHandler
DCP_VMI_IRQHandler
Reserved68_IRQHandler
TRNG_IRQHandler
SJC_IRQHandler
BEE_IRQHandler
PMU_EVENT_IRQHandler
Reserved78_IRQHandler
TEMP_LOW_HIGH_IRQHandler
TEMP_PANIC_IRQHandler
USB_PHY1_IRQHandler
USB_PHY2_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
Reserved86_IRQHandler
Reserved87_IRQHandler
GPIO1_INT0_IRQHandler
GPIO1_INT1_IRQHandler
GPIO1_INT2_IRQHandler
GPIO1_INT3_IRQHandler
GPIO1_INT4_IRQHandler
GPIO1_INT5_IRQHandler
GPIO1_INT6_IRQHandler
GPIO1_INT7_IRQHandler
GPIO1_Combined_0_15_IRQHandler
GPIO1_Combined_16_31_IRQHandler
GPIO2_Combined_0_15_IRQHandler
GPIO2_Combined_16_31_IRQHandler
GPIO3_Combined_0_15_IRQHandler
GPIO3_Combined_16_31_IRQHandler
GPIO4_Combined_0_15_IRQHandler
GPIO4_Combined_16_31_IRQHandler
GPIO5_Combined_0_15_IRQHandler
GPIO5_Combined_16_31_IRQHandler
WDOG1_IRQHandler
RTWDOG_IRQHandler
EWM_IRQHandler
CCM_1_IRQHandler
CCM_2_IRQHandler
GPC_IRQHandler
SRC_IRQHandler
Reserved115_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
PWM1_FAULT_IRQHandler
SEMC_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XBAR1_IRQ_0_1_IRQHandler
XBAR1_IRQ_2_3_IRQHandler
ADC_ETC_IRQ0_IRQHandler
ADC_ETC_IRQ1_IRQHandler
ADC_ETC_IRQ2_IRQHandler
ADC_ETC_ERROR_IRQ_IRQHandler
PIT_IRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved143_IRQHandler
Reserved144_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM2_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
PWM4_FAULT_IRQHandler
Reserved171_IRQHandler
GPIO6_7_8_9_IRQHandler*/
