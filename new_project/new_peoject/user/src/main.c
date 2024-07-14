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
 * ???????          main
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
// 1.救护车 2.装甲车 3.防弹背心 4.匕首//
// 5.炸药   6.消防斧 7.消防车   8.枪支//
// 9.急救包 10.手电筒 11.头盔   12.对讲机//
// 13.摩托车 14.警棍 15.望远镜

#include "zf_common_headfile.h"
#include "zf_driver_uart.h"
#include "image.h"
#include "camera.h"
#include "take.h"
#include "Vofa.h"
#include "math.h"
#include "control.h"
#include "communication.h"
#include "imu660ra.h"
#include "my_key.h"

char send_str[30] = {0};
extern float Car_dis_x1;
extern float Car_dis_y1;
extern uint8 Imgae_Use[IMAGE_HEIGHT][IMAGE_WIDTH];
extern int pid_motor[4];  //???pid?????????
extern pid_info Speed[4]; //???pid????
Vofa_HandleTypedef vofa1; // vofa????????
extern int test_count;
extern uint8 right_data[64];
extern uint8 Last_Longest_White_Column_Left[2];
extern uint8 Longest_White_Column_Left[2];
extern char str[];                   //?????????????why
extern int last_distance_x;          //?????????????????x????
extern unsigned int last_distance_y; //?????y????
extern int now_distance_x;
extern unsigned int now_distance_y;
extern unsigned int now_distance_y;
extern unsigned int card_count;    //?????????????????????????
extern float center_distance;      //????????????????????????
extern float last_center_distance; //?????????????????????????????'
extern uint8 Image_Use[IMAGE_HEIGHT][IMAGE_WIDTH];
extern uint8 uart_send_flag;
extern RoadType Road_Type;
extern int card_type; // 卡片类型，范围为1~15
extern uint8 init_flag;
extern int Edge_threshold; // 外部声明，边缘检测的阈值
char str1[] = "begin";
extern int card_abc;           // 卡片字母数字，值的范围为1~15
extern int card_num;           // 卡片数字，值的范围为1~3
extern int near_card_distance; // 最近卡片的距离
extern int near_card_x;
extern int near_card_y;
extern int correct_art2_flag;
extern unsigned int my_sceond_count;
extern uint8 seconds;
extern uint8 change;
extern uint8 chance;
extern int classify_art2_flag;
extern int correct_art2_flag;
extern uint8 visual_show2; // 按键处理显示模式
extern int Island_State;
extern uint8 left_island_flag;
extern uint8 record_abc_card_type;
extern uint8 transform_buffer[16];
extern uint8 shabi_saoxian_step;
// ????????????????????????????????????
// ????? ?????????????????
// ????? project->clean  ?????????????????

// ?????????????????????
float test_card_angle;
float zuobiao_x = 0;
float zuobiao_y = 0;
double test_delta_card_x, test_delta_card_y;
double test_tan, test_delta_angle;
extern float Now_angle;
extern int record_abc_flag;
int main_step = 0; // 直行
int main(void)
{
        clock_init(SYSTEM_CLOCK_600M); //??????????
        CLOCK_EnableClock(kCLOCK_Pit); // pit
        debug_init();                  // debug��ʼ��
        system_delay_ms(300);

        //    system_delay_ms(10000);         //
        // key_init(10);//?????????
        // pit_ms_init(PIT_CH3,10);    // ???3?????, 10ms????????????
        // while(1)//????????1s???????
        // {
        //     static unsigned int key_count=0;
        //     if(key_get_state(KEY_1)==KEY_LONG_PRESS)   //????1????
        //     {
        //         key_count++;
        //         key_clear_state(KEY_1);
        //     }
        //     if(key_count>100)
        //     {
        //         break;
        //     }
        // }
        //----------pid初始化---------------------//
        //    uart_init(UART_1, 115200, UART1_TX_B12, UART1_RX_B13); // 串口一初始化，用于art
        //    Vofa_Init(&vofa1, VOFA_MODE_SKIP);
        // wireless_uart_init(); // 无线串口初始化
        // key_init(20);//按键初始化
        // seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
        // seekfree_assistant_oscilloscope_struct oscilloscope_data;

        //        oscilloscope_data.data[0] = 0.1111 + 2;
        //        oscilloscope_data.data[1] = 0.3333 - 1;
        //        oscilloscope_data.data[2] = 4.222;
        //        oscilloscope_data.data[3] = 5.222;
        //        oscilloscope_data.channel_num = 4;
        // 设置为4个通道，通道数量最大为8个

        My_Communication_Init(); // 通信初始化
        PidInit();               // 增量式pid初始化
        card_island_init();      // 卡片岛初始化
        //   Pos_PidInit();//位置式pid初始化，现已弃用
        Distance_PidInit(); // 距离环初始化

        ips114_init(); // 屏幕初始化
        ips114_set_dir(IPS114_PORTAIT);
        ips114_set_font(IPS114_6X8_FONT);
        ips114_set_color(RGB565_RED, RGB565_BLACK);
        //----------模块初始化--------------------//
        ips114_clear();     // 清屏
                            // Motor_Init();              // 电机初始化
        Encoder_Init();     // 编码器初始化
        Camera_Init();      // 摄像头初始化
        my_imu660ra_init(); // 陀螺仪初始化，开机需静置一段时间
        my_pwm_gpio();      // 机械臂初始化
        // -- -- -- -- -- --中断初始化-- -- -- -- -- -- -- -- -- - //
        pit_ms_init(PIT_CH0, 5);   // 5ms
        pit_ms_init(PIT_CH1, 5);   // 10ms
        pit_ms_init(PIT_CH2, 100); // 15ms
        pit_ms_init(PIT_CH3, 500); // 25ms
        //

//        uart_write_string(UART_1, uart1_begin);
        // target_motor[1]=1000;
        // target_motor[3]=1000;
        //    float other_data[5]={1.0,2.0,3.0,4.0,5.0};
        /*视觉处理部分代码初始化*/
        Last_Longest_White_Column_Left[1] = 94;
        Longest_White_Column_Left[1] = 94;

        // Road_Type = STRAIGHT_ROAD;
        //             Speed[3].target_speed=30.0;
        //             Speed[2].target_speed=30.0;
        //             Speed[1].target_speed=30.0;
        //             Speed[0].target_speed=30.0;//?????

        int once = 1;
        int one_time = 1;
        uint8 temp = 0;
        uint8 type_count = 0;
        int car_run_mode = 0;
        //       find_ramp = OPEN;
        //    float zuobiao_x=0;
        //	  float zuobiao_y=0;
        //    int test_delta_card_x,test_delta_card_y;
        //    float test_tan,test_delta_angle;
        interrupt_global_enable(0); // 开中断使能
        //		int b = 1;
        //		float start_angle = 100.0;
        while (1)
        {
                my_key_handle(); // 别删，调总钻风的阈值
//					uart_write_string(UART_1, uart1_begin);
                //                seekfree_assistant_oscilloscope_send(&oscilloscope_data);
                //                oscilloscope_data.data[0] = Speed[0].now_speed;
                //                oscilloscope_data.data[1] = Speed[1].now_speed;
                //                oscilloscope_data.data[2] = Speed[2].now_speed;
                //                oscilloscope_data.data[3] = Speed[3].now_speed;
                // ips114_show_int(0, 0, near_card_x, 3);
                // ips114_show_int(0, 20, near_card_y, 3);
                // ips114_show_uint(0, 40, card_type, 3);
                // ips114_show_uint(0, 60, card_abc, 3);
               ips114_show_uint(0, 80, right_data[0], 3);
               ips114_show_uint(0, 100, right_data[1], 3);
               ips114_show_uint(0, 120, right_data[2], 3);
					// Top_Line_Center_Get_Center();
        test();
//        car_run_upline_left(95);
//	right_lie_island_upline_position=Top_Top_Line_Search_Island(110,50, 0);//持续从119行往上到20行找下边线数组最右侧行坐标标
        // ips114_show_int(0,20,right_lie_island_upline_position,4);//一次扫线
//	 car_run_upline(95);
        // CSI_correct_island_correct(int Island_center_card_x, int Island_center_card_y);
//         right_lie_island_upline_position=Top_Top_Line_Search_Island(100,0);//持续从第100行往上扫上边线
//        right_lie_island_upline_position = Top_Top_Line_Search_Island(110, 0);;//从119行开始往40行扫下边线，取最右列的行坐标
	//    Left_Island_pick_and_move(&Island_mode);
        // Left_Crossing_pick_and_move(&Crossing_mode);
        // right_lie_island_upline_position = Top_Top_Line_Search_Island(40, 1);

	// car_run_upline(90);
//					Car_Inverse_kinematics_solution(Vx, Vy, Vz);        //麦轮控制，为target_speed赋值
//	car_findcard(&car_mode);//模式选择
//					correct_art2_flag=1;
		// test_arm();
//					gpio_set_level(C11, 1);
		// ips114_show_int(188,60,Edge_threshold,4);
       ips114_show_float(0,0,Cross_State,3,4);//显示上边线归一化后的误差
//      ips114_show_float(0,40,left_top_error,3,4);//显示上边线归一化后的误差
//      ips114_show_float(90,0,Speed[0].target_speed,3,2);//显示上边线归一化后的误差
//      ips114_show_float(90,20,Speed[1].target_speed,3,2);//显示上边线归一化后的误差
//      ips114_show_float(90,40,Speed[2].target_speed,3,2);//显示上边线归一化后的误差
//      ips114_show_float(90,60,Speed[3].target_speed,3,2);//显示上边线归一化后的误差
//        ips114_show_int(0,20,car_run_mode,4);//卡片坐标,即时更新
//        ips114_show_int(0,40,Traffic_count,3);
//        ips114_show_int(0,60,Weapon_count,3);//第一次捕捉到卡片的y坐标
//	ips114_show_int(0,80,Supply_count,3);//第一次捕捉到卡片的y坐标
//	ips114_show_int(120,0,banmaxian_allow_flag,4);//卡片坐标,即时更新
//			 
	 ips114_show_int(120,0,now_distance_x,4);//卡片坐标,即时更新
         ips114_show_int(120,20,now_distance_y,4);//卡片坐标,即时更新
         ips114_show_int(120,40, delta_crossing_class_x,4);//卡片坐标,即时更新
	 ips114_show_int(120,60, delta_crossing_class_y,4);//卡片坐标,即时更新
        //  ips114_show_int(120,40, delta_x,4);//卡片坐标,即时更新
	//  ips114_show_int(120,60, delta_y,4);//卡片坐标,即时更新
        //  ips114_show_int(120,80, near_card_x,4);//卡片坐标,即时更新
	//  ips114_show_int(120,100, near_card_y,4);//卡片坐标,即时更新
        //  ips114_show_int(120,40, delta_crossing_x,4);//卡片坐标,即时更新
	//  ips114_show_int(120,60, delta_crossing_y,4);//卡片坐标,即时更新
         ips114_show_int(0,0,Angle_Crossing_Panduan,3);//左十字补线显示
         ips114_show_int(0,20,Crossing_mode,3);//左十字模式显示
				 
//				 ips114_show_int(0,30,crossing_correct_again_x,3);//左十字卡片定位坐标
//				 ips114_show_int(0,40,crossing_correct_again_y,3);//左十字卡片定位坐标
//				 ips114_show_int(0,30,crossing_card_center_x,3);//左十字卡片定位坐标
//				 ips114_show_int(0,40,crossing_card_center_y,3);//左十字卡片定位坐标
//        ips114_show_int(90,40,record_island_zone_x,4);//卡片坐标,即时更新
//        ips114_show_int(90,60,record_island_zone_y,4);//卡片坐标,即时更新
//****************************************测试圆环卡片的类型记录*****************//
    ips114_show_int(60,0,cross_card[0].Card_Type,4);//卡片坐标,即时更新
    ips114_show_int(60,20,cross_card[1].Card_Type,4);//卡片坐标,即时更新
 	 ips114_show_int(60,40,cross_card[2].Card_Type,4);//卡片坐标,即时更新
 	 ips114_show_int(60,60,cross_card[3].Card_Type,4);//卡片坐标,即时更新
    ips114_show_int(60,80,cross_card[4].Card_Type,4);//校准步数
//****************************************************************************//
//	 ips114_show_int(80,0,Island_card[0].Card_PWM_Duty,4);//卡片坐标,即时更新
//   ips114_show_int(80,20,Island_card[1].Card_PWM_Duty,4);//卡片坐标,即时更新
//	 ips114_show_int(80,40,Island_card[2].Card_PWM_Duty,4);//卡片坐标,即时更新
//	 ips114_show_int(80,60,Island_card[3].Card_PWM_Duty,4);//卡片坐标,即时更新
//   ips114_show_int(80,80,Island_card[4].Card_PWM_Duty,4);//校准步数
// 	 ips114_show_int(80,100,record_abc_flag,4);//记录的字母类型
//         ips114_show_int(120,100,record_abc_card_type,4);//记录的字母类型
//         ips114_show_int(120,120,card_abc,4);//传入的字母类型
//        ips114_show_int(90,40,classify_mode,4);//卡片坐标,即时更新
//        ips114_show_int(90,60,class_step,4);//卡片坐标,即时更新
//        ips114_show_int(90,80,classify_correct_finish,4);//卡片坐标,即时更新

//         ips114_show_int(120,0,Angle_z,4);//卡片坐标,即时更新
//         ips114_show_int(120,20,delta_card_x,4);//卡片坐标,即时更新
//         ips114_show_int(120,40,delta_card_y,4);//数字类型
//         ips114_show_int(120,60,turn_angle,4);//数字类型
//        ips114_show_int(120,20,card_center_y,4);//卡片坐标,即时更新
//        ips114_show_int(120,40,correct_x,4);//卡片坐标,即时更新
//        ips114_show_int(120,60,correct_y,4);//卡片坐标,即时更新
//        ips114_show_int(120,80,delta_x,4);//卡片坐标,即时更新
//        ips114_show_int(120,100,delta_y,4);//卡片坐标,即时更新
        
// 	ips114_show_int(90,0,now_distance_x,4);//卡片坐标,即时更新
//        ips114_show_int(90,20,now_distance_y,4);//卡片坐标,即时更新
//        ips114_show_float(90,40,test_top_error,2,3);//卡片坐标,即时更新
// 	ips114_show_float(90,60,top_error,2,3);//卡片坐标,即时更新
	//*****************测试art1对正(粗对正和对正卡片区域)时的变量*******************//
	//  ips114_show_int(120,0,Island_x,4);//卡片坐标,即时更新
        //  ips114_show_int(120,20,Island_y,4);//卡片坐标,即时更新
	//  ips114_show_int(120,40,delta_island_class_x,4);//卡片坐标,即时更新
	//  ips114_show_int(120,60,delta_island_class_y,4);//卡片坐标,即时更新
        // ips114_show_int(120,100,island_class_step,4);//校准步数
        //  ips114_show_int(90,40,delta_island_x,4);//卡片坐标,即时更新
        //  ips114_show_int(90,60,delta_island_y,4);//卡片坐标,即时更新
//*******************测试art4的变量*********************//
//				ips114_show_int(120,60,delta_x,4);//卡片坐标,即时更新
//				ips114_show_int(120,80,delta_y,4);//卡片坐标,即时更新
//				ips114_show_int(90,40,near_card_x,4);//卡片坐标,即时更新
//        ips114_show_int(90,60,near_card_y,4);//卡片坐标,即时更新
//				ips114_show_int(90,80,CSI_correct_flag,4);//卡片坐标,即时更新
//******************************************************//
//	 ips114_show_int(90,80,right_lie_island_upline_position,4);//最右列的行坐标
        //   ips114_show_int(0,20,Island_mode,3);//左环岛模式显示
        //  ips114_show_int(0,20,right_lie_island_upline_position,4);//一次扫线
        //  ips114_show_int(0,40,second_right_lie_island_upline_position,4);//二次扫线
        //  ips114_show_int(0,60,shabi_saoxian_step,4);//傻逼扫线的步数
	//  ips114_show_int(0,100,card_classify_count,4);//卡片坐标,即时更新
	//  ips114_show_int(0,80,left_island_flag,4);//卡片坐标,即时更新
        // ips114_show_int(90,80,CSI_correct_flag,4);//卡片坐标,即时更新

                //					/*****************测试上边线巡线(仅直道)成功***********************/
                //        car_run_upline();
                //        Turn_Angle_PD(Angle_Z);
                //        Car_Inverse_kinematics_solution(Vx, Vy, Vz); //麦轮控制，为target_speed赋值
                //        ips114_show_float(0,0,top_error,3,4);//上边线误差
                //        ips114_show_float(0,20,Vx,3,4);
                // 	ips114_show_float(0,40,Vy,3,4);
                //					/***************************************************************/
                //                // ips114_show_int(0,60,type,4);
                // if(type==5)
                // {
                //    type_count++;
                //    if(type_count>5)
                //   {
                //     car_run_mode=1;//更改寻迹模式
                //     now_distance_x=0;
                //     now_distance_y=0;
                //   }
                // }
                // if(type==8)
                // {
                //    if(once)
                // {
                //   Angle_ramp=0;
                //   ramp_x=0;
                //   ramp_y=0;
                //   ramp_step=1;
                //   car_run_mode=2;
                //   car_stop();
                //   find_ramp = 1;
                //   once=0;
                // }
                // }
                       switch(car_run_mode)
                	{
                	 case 0:
                         if(left_island_flag || right_island_flag && Island_Allow_flag == READY && Left_Island_Done ==NOT_FINISH)//环岛识别+未完成+允许处理
                         {
                             car_run_mode = 4;               //环岛处理
                             Find_card_allow = NOT_READY;    //不允许寻卡
                             Cross_Allow_flag = NOT_READY;   //不允许十字
                             Zebra_Allow_flag = NOT_READY;   //不允许斑马线
                             Cross_Handle_Flag = 0;          //干废十字识别条件
                             Zebra_catch_flag = 0;           //干废斑马线识别条件
                             break;
                         }
                         else if(Cross_Handle_Flag = 1 && Cross_Allow_flag = READY && Crossing_Finish == NOT_FINISH)
                         {
                             car_run_mode = 5;               //十字处理
                             Find_card_allow = NOT_READY;    //不允许寻卡
                             Island_Allow_flag = NOT_READY;  //不允许环岛
                             Zebra_Allow_flag = NOT_READY;   //不允许斑马线
                             left_island_flag = 0;           //干废环岛识别条件
                             right_island_flag = 0;          //干废环岛识别条件
                             Zebra_catch_flag = 0;          //干废斑马线识别条件
                             break;
                         }
                         else if(Zebra_catch_flag = 1 && Zebra_Allow_flag = READY && Zebra_Finish == NOT_FINISH)
                         {
                             car_run_mode = 1;               //斑马线处理
                             Find_card_allow = NOT_READY;    //不允许寻卡
                             Island_Allow_flag = NOT_READY;  //不允许环岛
                             Cross_Allow_flag = NOT_READY;   //不允许十字
                             left_island_flag = 0;           //干废环岛识别条件
                             right_island_flag = 0;          //干废环岛识别条件
                             Cross_Handle_Flag = 0;          //干废十字识别条件
                             break;
                         }
                         else if(Find_card_allow==READY)
                         {
                            car_findcard(&car_mode);//模式选择
                            car_run_mode=0;
                            break;
                         }
                	 case 1://斑马线处理
                          if(banmaxian_finish==FINISH)
                          {
                             car_run_mode=3;   //只做循迹任务 
                          }
                          else
                          {
                            car_run_mode=1;    //锁住状态
                          }
                             card_final_classify(&classify_mode);
                	 break;
                	 case 2:
                	  ramp_cross(60, 140);//坡道绕行函数
                	  Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
                	  if(ramp_finish==1)
                	    car_run_mode=0;
                         else
                	    car_run_mode=2;
                	 break;
                         case 3:
                            car_run_mode = 3;//锁状态
                            car_run();
                            break;
                         case 4:             //环岛处理
                            if(Left_Island_Done == NOT_FINISH && Island_Allow_flag == READY )
                            {
                               car_run_mode = 4;               //锁住状态
                               Cross_Handle_Flag = 0;          //干废十字识别条件
                               Zebra_catch_flag = 0;           //干废斑马线识别条件
                               Find_card_allow = NOT_READY;    //不允许寻卡
                               Cross_Allow_flag = NOT_READY;   //不允许十字
                               Zebra_Allow_flag = NOT_READY;   //不允许斑马线
                               Left_Island_pick_and_move(&Island_mode);
                               break;
                            }
                            else if(Left_Island_Done == FINISH)
                            {
                                car_run_mode = 0;
                                Island_Allow_flag = NOT_READY;  //不再允许环岛
                                Find_card_allow = READY;        //允许寻卡
                                Zebra_Allow_flag = READY;       //允许斑马线
                                left_island_flag = 0;           //重置环岛识别条件
                                right_island_flag = 0;          //重置环岛识别条件
                                if(Crossing_Finish == NOT_FINISH)
                                {
                                   Cross_Allow_flag = READY;   //允许十字     
                                }
                                else if(Crossing_Finish == FINISH)//十字未完成
                                {
                                   Cross_Allow_flag = NOT_READY;  //不再允许十字
                                }
                                break;       
                            }
                        case 5:             //十字处理
                           if(Cross_Allow_flag = READY && Crossing_Finish == NOT_FINISH)//允许处理十字且十字处理未完成
                           {
                              car_run_mode = 5;               //锁住状态
                              Cross_Allow_flag = READY;       //允许十字
                               Find_card_allow = NOT_READY;    //不允许寻卡
                               Zebra_Allow_flag = NOT_READY;   //不允许斑马线
                               Island_Allow_flag = NOT_READY;  //不允许环岛
                               left_island_flag = 0;           //干废环岛识别条件
                               right_island_flag = 0;          //干废环岛识别条件
                               Zebra_catch_flag = 0;           //干废斑马线识别条件
                              Left_Crossing_pick_and_move(&Crossing_mode);
                              break;
                           }
                           else if(Crossing_Finish == FINISH)//十字处理完成
                           {
                              car_run_mode = 0;
                              Cross_Allow_flag = NOT_READY;   //不再允许十字
                              Find_card_allow = READY;        //允许寻卡
                              if(Left_Island_Done == NOT_FINISH)//环岛未完成
                              {
                                 Island_Allow_flag = READY;  //允许环岛
                              }
                              else if(Left_Island_Done == FINISH)//环岛完成
                              { 
                                 Island_Allow_flag = NOT_READY;  //不再允许环岛
                              }    
                                                   
                              Zebra_Allow_flag = READY;       //允许斑马线
                              Cross_Handle_Flag = 0;          //重置十字识别条件
                              Zebra_catch_flag = 0;           //重置斑马线识别条件
                              break;
                           }                            
                	}
                //     ips114_show_int(0,0,car_run_mode,4);
                //     ips114_show_int(0,20,target_type,4);
                //     ips114_show_int(0,40,near_card_x/10,4);
                //     ips114_show_int(0,40,near_card_y/10,4);
                //     ips114_show_int(0,60,delta_x,4);
                //     ips114_show_int(0,80,delta_y,4);
                //     ips114_show_int(0,20,classify_type,4);
                // 		ips114_show_int(0,40,numcard_classify,4);
                // 		ips114_show_int(120,0,near_card_x/10,4);
                //     ips114_show_int(120,20,near_card_y/10,4);//卡片坐标,即时更新
                /*************************测试上边线巡线**********************/

                //    ips114_show_float(0,60,Vx,3,4);
                //    ips114_show_float(0,80,Vy,3,4);//x,y速度
                //    ips114_show_int(0,100,delta_class_x,4);
                //    ips114_show_int(0,120,delta_class_y,4);
                //		ips114_show_int(120,0,now_distance_x/10,4);
                //    ips114_show_int(120,20,now_distance_y/10,4);//卡片坐标,即时更新
                //		ips114_show_int(120,40,num_card_x,4);
                //    ips114_show_int(120,60,num_card_y,4);//卡片坐标,即时更新
                //		ips114_show_int(120,80,Card_dis_car_x,4);
                //    ips114_show_int(120,100,Card_dis_car_y,4);//卡片坐标,即时更新
                // 坡道绕行函数
                //                if(type == 8)
                //                {
                //                   if(once)
                //                   {
                //                     Angle_ramp=0;
                //                     ramp_x=0;
                //                     ramp_y=0;
                //                     ramp_step=1;
                //										 step=1;
                //										 car_stop();
                ////									find_ramp = 1;
                //                     once=0;
                //                   }
                //                }
                //								if(step==0)
                //								{
                //								  car_run();
                //								}
                //								if(step==1)
                //								{
                //                  ramp_cross(60, 140);//坡道绕行函数
                //								  Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
                //									if(ramp_finish==1)
                //									{
                //										step=0;
                //										type=0;
                //									}
                //								}
                //                ips114_show_float(0,0,Angle_ramp,3,4);
                //                ips114_show_float(0,20,ramp_x,3,4);
                //                ips114_show_float(0,40,ramp_y,3,4);//绕行的里程计x,y
                //                ips114_show_int(90,0,ramp_step,4);
                //                ips114_show_float(0,60,Vx,3,4);
                //                ips114_show_float(0,80,Vy,3,4);//x,y速度
                //                   car_run();

                //*********************测试总的车辆行进打包函数**********************//
                //    car_findcard(&car_mode);//模式选择
                // //         //**************************观察卡片坐标和里程计*********************//
                //         ips114_show_int(120,0,now_distance_x,4);
                //         ips114_show_int(120,20,now_distance_y,4);//卡片坐标,即时更新
                // 							ips114_show_int(90,0, card_classify,4);
                //         ips114_show_int(90,20,card_center_x,4);
                //         ips114_show_int(90,40,card_center_y,4);//显示卡片中心坐标
                //         ips114_show_int(0,60,right_data[0],4);
                //         ips114_show_int(0,80,right_data[1],4);
                //         ips114_show_int(0,100,right_data[2],4);
                //					      Turn_Angle_PD(90);
                //					      Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
                //					      if(fabs(Angle_Z-90)<=3)
                //								{
                //									if(once)
                //									{
                //										arrive_card_flag=OPEN;
                //					          Angle_arrive_card=0;//清零角度
                //										correct_x=0;
                //										correct_y=0;
                //									  once=0;
                //									}
                //								}
                //								ips114_show_float(0,0,Angle_arrive_card,3,4);
                //                ips114_show_float(150,60,correct_x,3,4);
                //                ips114_show_float(150,90,correct_y,3,4);//修正的里程计x,y
                //         ips114_show_float(90,60,Card_dis_car_x,3,4);
                // 		     ips114_show_float(90,90,Card_dis_car_y,3,4);//卡片里程计x,y
                // //*******************************************************************//
                // //**************************观察行进变量*****************************//
                //                   ips114_show_int(0,60,card_y[0],3);//第一次捕捉到卡片的y坐标
                //    ips114_show_int(120, 60, target_type, 3); // 用于观测行进函数的步数
                //    ips114_show_int(120, 80, correct_art2_flag, 3); // 用于观测行进函数的步数
                //    ips114_show_int(90,80,delta_card_y,3);//卡片y坐标与里程计的差值
                //     ips114_show_int(90,100,delta_card_x,3);//卡片x坐标与里程计的差值
                // //*******************************************************************//
                // //**************************观察行进变量*****************************//
                // ips114_show_int(0, 0, pick_up_mode, 3);                // 观察摄像头模式
                // ips114_show_float(0, 20, Speed[0].target_speed, 3, 4); // 用于观测目标速度是否改变
                //            	 	 ips114_show_int(90,80,delta_card_y,3);//卡片y坐标与里程计的差值
                //           			 ips114_show_int(90,100,delta_card_x,3);//卡片x坐标与里程计的差值
                // //*******************************************************************//
                // //**************************观察角度*********************************//
                //                    ips114_show_float(0,0,Angle_Z,3,2);
                //                    ips114_show_float(0,20,Angle_z,3,2);
                //                    ips114_show_float(0,40,delta_angle,3,2);//显示现在的偏转角
                //   		// 	ips114_show_float(0,40,turn_angle,3,2);
                //  ips114_show_float(0,60,Vz,3,2);
                //         // ips114_show_float(0,20,delta_card_y/delta_card_x,3,2);
                // //*******************************************************************//

                // //*********************测试弯道卡片坐标******************************//
                //  			if(abs(now_distance_y)>0&&abs(now_distance_y)<800&&abs(now_distance_x)>0)//识别到卡片
                //  			{
                // 				if(once)
                // 				{
                // 					Card_dis_car_x = 0;
                // 					Card_dis_car_y = 0;
                // 					zuobiao_x = 28;//卡片坐标x  now_distance_x/10
                // 					zuobiao_y = now_distance_y/10;//卡片坐标y，第一次捕获到的坐标 now_distance_y/10
                // 					Angle_z=0;//清零angle_z
                //           catch_card_flag = 1;//捕获成功，记得要重新关闭,打开里程计的第二种模式
                // 					once = 0;
                // 				}
                // 			}
                //            test_delta_card_x = zuobiao_x-(int)Card_dis_car_x;//算出在更新后的坐标轴下的x差值
                //            test_delta_card_y = zuobiao_y-(int)Card_dis_car_y;//算出在更新后的坐标轴下的y差值
                // 			     test_tan = test_delta_card_y/test_delta_card_x*1.0;
                //            test_delta_angle = atan((double)(test_delta_card_y/test_delta_card_x))/PI*180;//算出即时偏移角
                // 			ips114_show_float(150,0,zuobiao_x,3,4);
                // 		    ips114_show_float(150,20,zuobiao_y,3,4);//卡片里程计x,y
                //             ips114_show_float(150,40,Card_dis_car_x,3,4);
                // 		    ips114_show_float(150,60,Card_dis_car_y,3,4);//卡片里程计x,y
                //             ips114_show_int(150,80,test_delta_card_x,3);//卡片x坐标与里程计的差值
                //             ips114_show_int(150,100,test_delta_card_y,3);//卡片y坐标与里程计的差值
                // 			      ips114_show_float(0,60,test_tan,3,2);
                //             ips114_show_float(0,80,test_delta_angle,3,2);
                //             ips114_show_float(0,100,test_delta_angle-Angle_z,3,2);
                //         if((test_delta_angle-Angle_z<10 && test_delta_angle-Angle_z>-15)||(test_delta_angle-Angle_z<1.0 && test_delta_angle-Angle_z>-1.0))//因为车身姿态与采样频率的问题，有且只有一个相交点，给出在符合角度的波动区间,前一个条件判断弯道
                //         {
                //              ips114_show_string( 90 , 100,   "SUCCESS");
                //         }
                //*******************************************************************//

                //*********************测试总钻风微调********************************//
                // CSI_dis_correct((float)center_x, (float)center_y);//总钻风坐标对正
                //*******************************************************************//

                //*********************测试360舵机********************************//
                // arm_control(4);//测试舵机模式
                //*******************************************************************//

                //*********************测试侧面舵机********************************//
                // gpio_set_level(C11, 1);
                //          test_arm();
                //*********************测试总钻风/art校正********************************//

                //  if(near_card_x!=0 && near_card_y!=0)
                // 	{
                // 	  if(once)
                // 	  {
                // 	   card_center_x=near_card_x;
                // 	   card_center_y=near_card_y;
                // 									card_classify=card_type;
                // 	   correct_art2_flag = CLOSE;//立即关闭art4发数据，防止堵塞数据缓冲区
                // 	   once=0;//只记录一次
                // 	  }
                // 	}
                //        if(card_classify==1 || card_classify==2 || card_classify==7 || card_classify==13)//交通工具类
                //        {
                //           classify_360(Traffic);
                //        }
                //        else if(card_classify==3 || card_classify==4 || card_classify==5 || card_classify==6 || card_classify==8 ||card_classify==14)//武器类
                //        {
                //            classify_360(Weapon);
                //        }
                //        else if(card_classify==9 || card_classify==10 || card_classify==11 || card_classify==12 || card_classify==15)//物资类
                //        {
                //            classify_360(Supply);
                //        }
                // 	CSI_dis_new_correct(card_center_x, card_center_y);//总钻风坐标对正，准备x,y速度
                //         Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
                // 									    ips114_show_float(0,0,Vx,3,4);
                // 									    ips114_show_float(0,20,Vy,3,4);
                // 									    ips114_show_int(0,40, correct_x,4);
                // 									    ips114_show_int(0,60, correct_y,4);
                //                                                                             ips114_show_int(0,80, delta_x,4);
                //                                                                             ips114_show_int(0,100, delta_y,4);
                // 				  ips114_show_int(90,0, near_card_x,4);
                // 									    ips114_show_int(90,20, near_card_y,4);
                //           ips114_show_int(90,40,card_center_x,4);
                //           ips114_show_int(90,60,card_center_y,4);//显示卡片中心坐标
                //  ips114_show_int(90,40,delta_x,4);
                //  ips114_show_int(90,60,delta_y,4);//显示delta
                // ips114_show_int(90, 80, correct_x_flag, 4);  // 显示x调整标志位
                // ips114_show_int(90, 100, correct_y_flag, 4); // 显示y调整标志位
                //  ips114_show_int(0,20,near_card_x,3);//第一次捕捉到卡片的y坐标
                //  ips114_show_int(0,40,near_card_y, 3); // 用于观测行进函数的步数
                // 			 ips114_show_int(40,0,my_sceond_count,3);
                // 			 ips114_show_int(40,20,seconds,3);
                // 			 ips114_show_int(40,40,change,1);
                //                          ips114_show_uint(40,60,chance,3);
                //                          ips114_show_uint(40,80,correct_art2_flag,3);
                //            ips114_show_float(0,80,Vx,3,2);
                //            ips114_show_float(0,100,Vy,3,2);//显示x,y速度
                //					ips114_show_float(90,100,Vz,3,2);//显示x,y速度
                //        //*******************************************************************//
                //**************************观察卡片世界坐标*****************************//
                //
                //		          ips114_show_float(0,0,Angle_world,3,4);
                //							ips114_show_float(0,20,card_world_angle,3,4);
                //							ips114_show_float(0,40,car_world_angle,3,4);
                //          	  ips114_show_float(150,20,card_world_x,3,5);//捕获的卡片的世界x坐标
                //							ips114_show_float(150,40,card_world_y,3,5);//捕获的卡片世界y坐标
                //							ips114_show_float(150,60,card_world_distance,3,5);//捕获的卡片的世界x坐标
                //							ips114_show_float(150,80,car_world_distance,3,5);//捕获的卡片世界y坐标
                //							ips114_show_int(0,60,car_card_angle,4);//观察所夹角
                //							ips114_show_int(0,80,car_card_diatance,4);//卡片与车辆的直线距离
                //							if(car_card_diatance<30 && card_world_y!=0 && card_world_y!=0)//为车身姿态与采样频率的问题，有且只有一个相交点，给出在符合角度的波动区间,前一个条件判断弯道
                //           {
                //                      ips114_show_string( 90 , 100,   "SUCCESS");
                //           }
                //							ips114_show_int(90,60,target_type,3);//用于观测行进函数的步数
                //           	 	ips114_show_int(90,80,delta_card_y,3);//卡片y坐标与里程计的差值
                //          		ips114_show_int(90,100,delta_card_x,3);//卡片x坐标与里程计的差值
                // //*******************************************************************//
                //**************************观察多张卡片的情况*********************************//
                //                   ips114_show_int(0,0,card_count,3);//用于观测卡片数目
                //                 ips114_show_float(0,20,Angle_z,3,2);
                //                 ips114_show_float(0,40,delta_angle,3,2);//显示现在的偏转角
                //   		// 	ips114_show_float(0,40,turn_angle,3,2);
                //          		 	 ips114_show_float(0,60,Vz,3,2);
                //         // ips114_show_float(0,20,delta_card_y/delta_card_x,3,2);
                // //*******************************************************************//
                //*********************测试总的车辆行进打包函数**********************//
                //*******************************************************************//

                //                		car_run();
                //  ips114_show_float(0,60,center_distance,3,2);
                //  ips114_show_int(188,80,right_data[3],3);
                // for(uint8 i=0;i<4;i++)
                // {
                //     target_motor[i]=1000;cc
                // }
                // ips114_show_string( 0 , 10,   "SUCCESS");                          // ????????
                // for(uint16 i=0;i<1800;i++)
                // {
                //     Speed[2].target_speed=3.00*sin(2*PI*i/180.0);
                //     // Speed[2].target_speed=3.00*sin(2*PI*i/180.0);
                //     // // Speed[3].target_speed=3.00*sin(2*PI*i/180.0);
                //     // printf("%d,%d,%d\r\n",(int)PID_motor[1],(int)PID_motor[2],(int)PID_motor[3]);
                // 	system_delay_ms(100);
                // 	motor_close_control();
                // }
                //        Move_Transfrom(1000,1000,0);
                ////        text_arm();
                // uint8 *output_address;                              // 图像第一个像素的地址
                // output_address = Scharr_Edge(*mt9v03x_image, 1700); // 使用扫描边缘的方式获取图像
                // memcpy(Image_Use, output_address, IMAGE_HEIGHT * IMAGE_WIDTH * sizeof(uint8));
                // ips114_displayimage03x(*Image_Use, 188, 120);
                //            car_findcard(1);
                //			 Distance_Motor();
                //			 if(Speed[0].target_speed == 0)//��Ϊ�ĸ����������ͬ���ٶȣ������ȡһ�����Ӽ�⼴�ɣ���ʱ�ѵ��￨Ƭy����ص㣬�ٶ�Ϊ0
                //      {
                ////         mode = Car_turn;//ģʽת��
                ////         now_angle = Angle_Z;//��¼��ת��ǰ�ĽǶ�
                //         Car_dis_y = 0;//�����̼�y�ļ���ֵ
                //				 Turn_Angle_PD(90);
                //      }
                // car_findcard(1);
                //  test();
                //   ips114_show_float(0,20,Speed[1].output,2,2);
                //        Vofa_JustFloat(&vofa1,other_data,5);
                //        uart_write_buffer(UART_1,other_data,5);
                //		printf("\n");
                //        uart_write_buffer(UART_8,other_data,5);
                //		printf("test!\n");
                // Vofa_SendData(&vofa1,other_data,5);
                //		printf("abcd\r\n");
                //        sprintf(send_str, "my_name");
                //        printf("%d,%d,%d,%d\r\n", encoder[0], encoder[1], encoder[2], encoder[3]);
                // printf("%.2f,%.2f,%.2f,%.2f\r\n", Speed[0].now_speed, Speed[1].now_speed, Speed[2].now_speed, Speed[3].now_speed);
                // // printf("%.2f,%.2f,%.2f,%.2f\r\n",Speed[0].now_speed,Speed[0].target_speed,Speed[0].error,Speed[0].output);
                //        printf("%.2f,%.2f,%.2f,%.2f\r\n", Speed[0].now_speed,Speed[1].now_speed, Speed[2].now_speed,Speed[3].now_speed);
                //       printf("test");
                // ips114_show_int(0,0,encoder[0],4);
                // ips114_show_int(    0 , 20,   `[1],         4);
                //        ips114_show_int(0, 0, encoder[0], 4);
                //        ips114_show_int(0, 20, encoder[1], 4);
                //        ips114_show_int(0, 40, encoder[2], 4);
                //        ips114_show_int(0, 60, encoder[3], 4);
                //		ips114_show_int(    0 , 0,   right_data[0],         4);
                //		ips114_show_int(   0 , 20,   right_data[1],         4);
                //		ips114_show_int(    0 , 40,   right_data[2],         4);
                //		ips114_show_int(   0 , 60,   right_data[3],         4);
        }
}

/**
 * @brief ????1???????
 * @param ??
 * @return ??
 */
void UART1_handler(void)
{
        uart1_rx_interrupt_handler(); //????1???????????????
        get_uartdata();               //?????????
}

/**
 * @brief ????4???????
 * @param ??
 * @return ??
 */
void UART4_handler(void)
{
        uart4_rx_interrupt_handler(); //????1???????????????
        get_uartdata();               //?????????
}
