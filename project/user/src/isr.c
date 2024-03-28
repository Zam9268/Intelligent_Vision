/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ????RT1064DVL6A ???????????????? SDK ??????????????
* Copyright (c) 2022 SEEKFREE ?????
* 
* ??????? RT1064DVL6A ???????????
* 
* RT1064DVL6A ????? ?????????
* ?????????????????????????? GPL??GNU General Public License???? GNU????????????????????
* ?? GPL ???3????? GPL3.0??????????????????????????????????/???????
* 
* ???????????????????????????????????????????????
* ?????????????????????????????????
* ???????????? GPL
* 
* ?????????????????????????? GPL ?????
* ?????????????<https://www.gnu.org/licenses/>
* 
* ?????????
* ?????????? GPL3.0 ???????????? ???????????????????
* ?????????????? libraries/doc ???????? GPL3_permission_statement.txt ?????
* ??????????? libraries ??????? ???????????? LICENSE ???
* ?????????????????????? ?????????????????????????????????????????
* 
* ???????          isr
* ???????          ??????????????
* ??????          ?? libraries/doc ??????? version ??? ??????
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

extern pid_info Speed[4];//??????
extern uint8 step;
int count = 0;

void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();     // ????SDK???????????? ?????????????????????????????
    __DSB();                    // ???????????
}

void PIT_IRQHandler(void)
{
    if(pit_flag_get(PIT_CH0))//
    {
        turnloc_pid();//λ��ʽpid
        motor_close_control();
        pit_flag_clear(PIT_CH0);
    // if(pit_flag_get(PIT_CH0))
    // {
    //     void Read_imu (void);
    //     Read_imu;
    //     pit_flag_clear(PIT_CH0);
    // }
    }
    if(pit_flag_get(PIT_CH1))
    {
       Read_Encoder();//?????????????????????????????
       
       pit_flag_clear(PIT_CH1);
    }
    
    if(pit_flag_get(PIT_CH2))
    {
        extern uint8 arm_flag;
        count++;
        if(count>100)
        {
            arm_flag = 1;
            pit_disable(PIT_CH2);//?????????????
        }
        pit_flag_clear(PIT_CH2);
    }
    
    if(pit_flag_get(PIT_CH3))
    {
        pit_flag_clear(PIT_CH3);
    }

    __DSB();
    
}

void LPUART1_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
        //????
    // #if DEBUG_UART_USE_INTERRUPT                        // ???? debug ????
    //     debug_interrupr_handler();                      // ?? debug ???????? ???? debug ???????
    // #endif                                              // ????? DEBUG_UART_INDEX ?????????????????
    extern void UART1_handler(void);//?????????
    UART1_handler();
    }
        
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);    // ?????
}

void LPUART2_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
        // ????
        
    }
        
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);    // ?????
}

void LPUART3_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        // ????
        
    }
        
    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);    // ?????
}

void LPUART4_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        // ????
        flexio_camera_uart_handler();
        
        gps_uart_callback();
    }
        
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);    // ?????
}

void LPUART5_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        // ????
        camera_uart_handler();
    }
        
    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);    // ?????
}

void LPUART6_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        // ????
        
    }
        
    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);    //?????
}


void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        //????
        wireless_module_uart_handler();
        
    }
        
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);    //?????
}


void GPIO1_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(B0))
    {
        exti_flag_clear(B0);//?????
    }
    
}


void GPIO1_Combined_16_31_IRQHandler(void)
{
    wireless_module_spi_handler();
    if(exti_flag_get(B16))
    {
        exti_flag_clear(B16); //?????
    }

    
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    flexio_camera_vsync_handler();
    
    if(exti_flag_get(C0))
    {
        exti_flag_clear(C0);//?????
    }

}

void GPIO2_Combined_16_31_IRQHandler(void)
{
    // -----------------* ToF INT ???????? ?????????????? *-----------------
    tof_module_exti_handler();
    // -----------------* ToF INT ???????? ?????????????? *-----------------
    
    if(exti_flag_get(C16))
    {
        exti_flag_clear(C16); // ???????????
    }
    
}




void GPIO3_Combined_0_15_IRQHandler(void)
{

    if(exti_flag_get(D4))
    {
        exti_flag_clear(D4);// ???????????
    }
}









/*
??????????????????????????????????
Sample usage:?????????????????????
void PIT_IRQHandler(void)
{
    //???????????
    __DSB();
}
???????????????????
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



