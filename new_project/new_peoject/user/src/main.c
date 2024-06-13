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

char send_str[30] = {0};
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
extern uint8 card_type; // 卡片类型，范围为1~15
extern uint8 init_flag;
char str1[] = "begin";
extern uint8 card_abc;         // 卡片字母数字，值的范围为1~15
extern uint8 card_num;         // 卡片数字，值的范围为1~3
extern int near_card_distance; // 最近卡片的距离
extern int near_card_x;
extern int near_card_y;
extern int correct_art2_flag;
// ????????????????????????????????????
// ????? ?????????????????
// ????? project->clean  ?????????????????

// ?????????????????????
float test_card_angle;
float zuobiao_x = 0;
float zuobiao_y = 0;
double test_delta_card_x, test_delta_card_y;
double test_tan, test_delta_angle;

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
        // seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIRELESS_UART);
        // seekfree_assistant_oscilloscope_struct oscilloscope_data;

        // oscilloscope_data.data[0] = 0.1111 + 2;
        // oscilloscope_data.data[1] = 0.3333 - 1;
        // oscilloscope_data.data[2] = 4.222;
        // oscilloscope_data.data[3] = 5.222;
        // oscilloscope_data.channel_num = 4;
        // 设置为4个通道，通道数量最大为8个
        My_Communication_Init(); // 通信初始化
        PidInit(); // 增量式pid初始化
        //   Pos_PidInit();//位置式pid初始化，现已弃用
        // Distance_PidInit(); // 距离环初始化
//        correct_art2_flag=OPEN;
        ips114_init(); // 屏幕初始化
        ips114_set_dir(IPS114_PORTAIT);
        ips114_set_font(IPS114_6X8_FONT);
        ips114_set_color(RGB565_RED, RGB565_BLACK);
        //----------模块初始化--------------------//
        ips114_clear();            // 清屏
        Motor_Init();              // 电机初始化
        Encoder_Init();            // 编码器初始化
        Camera_Init();             // 摄像头初始化
        my_imu660ra_init();        // 陀螺仪初始化，开机需静置一段时间
        my_pwm_gpio();             // 机械臂初始化
        // -- -- -- -- -- --中断初始化-- -- -- -- -- -- -- -- -- - //
			pit_ms_init(PIT_CH0, 5);   // 5ms
			pit_ms_init(PIT_CH1, 5);   // 10ms
			pit_ms_init(PIT_CH2, 100); // 15ms
			pit_ms_init(PIT_CH3, 500); // 25ms
                                   //
                                   // target_motor[1]=1000;
                                   // target_motor[3]=1000;

        //    float other_data[5]={1.0,2.0,3.0,4.0,5.0};
        /*视觉处理部分代码初始化*/
        Last_Longest_White_Column_Left[1] = 94;
        Longest_White_Column_Left[1] = 94;
        Road_Type = STRAIGHT_ROAD;
        //     Speed[3].target_speed=40.0;
        //     Speed[2].target_speed=40.0;
        //     Speed[1].target_speed=40.0;
        //     Speed[0].target_speed=40.0;//?????

        int once = 1;
        uint8 temp = 0;
        //    float zuobiao_x=0;
        //	  float zuobiao_y=0;
        //    int test_delta_card_x,test_delta_card_y;
        //    float test_tan,test_delta_angle;
        interrupt_global_enable(0); // 开中断使能
        //		int b = 1;
        //		float start_angle = 100.0;
        while (1)
        {
                // seekfree_assistant_oscilloscope_send(&oscilloscope_data);
                // oscilloscope_data.data[0] = encoder[0];
                // oscilloscope_data.data[1] = encoder[1];
                // oscilloscope_data.data[2] = encoder[2];
                // oscilloscope_data.data[3] = encoder[3];
                // ips114_show_int(0, 0, near_card_x, 3);
                // ips114_show_int(0, 20, near_card_y, 3);
                // ips114_show_uint(0, 40, card_type, 3);
                // ips114_show_uint(0, 60, card_abc, 3);
                // ips114_show_uint(0, 80, right_data[0], 3);
                // ips114_show_uint(0, 100, right_data[1], 3);
                // ips114_show_uint(0, 120, right_data[2], 3);

               test();
                //**************************观察卡片坐标和里程计*********************//
                           	//    	 ips114_show_int(90,0,now_distance_x,4);
                                //  ips114_show_int(90,20,now_distance_y,4);//卡片坐标,即时更新
                // 		// ips114_show_float(150,60,Car_dis_x,3,4);
                // 		// ips114_show_float(150,90,Car_dis_y,3,4);//里程计x,y
                //         ips114_show_float(90,60,Card_dis_car_x,3,4);
                // 		     ips114_show_float(90,90,Card_dis_car_y,3,4);//卡片里程计x,y
                // //*******************************************************************//
                // //**************************观察行进变量*****************************//
                        //    ips114_show_int(90,40,card_y[0],3);//第一次捕捉到卡片的y坐标
                        //    ips114_show_int(90, 60, target_type, 3); // 用于观测行进函数的步数
                //            	 	 ips114_show_int(90,80,delta_card_y,3);//卡片y坐标与里程计的差值
                //           			 ips114_show_int(90,100,delta_card_x,3);//卡片x坐标与里程计的差值
                // //*******************************************************************//
                // //**************************观察行进变量*****************************//
                // ips114_show_int(0, 0, pick_up_mode, 3);                // 观察摄像头模式
                // ips114_show_float(0, 20, Speed[0].target_speed, 3, 4); // 用于观测目标速度是否改变
                //            	 	 ips114_show_int(90,80,delta_card_y,3);//卡片y坐标与里程计的差值
                //           			 ips114_show_int(90,100,delta_card_x,3);//卡片x坐标与里程计的差值
                // //*******************************************************************//
                // //**************************观察角度*********************************//
                           	// 	   ips114_show_float(0,0,Angle_Z,3,2);
                                //  ips114_show_float(0,20,Angle_z,3,2);
                                //  ips114_show_float(0,40,delta_angle,3,2);//显示现在的偏转角
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
//         test_arm();
        //*********************测试总钻风/art校正********************************//
//          pick_up_mode = 1; 
// if(once)
//	{
//	  if(near_card_x!=0 && near_card_y!=0)
//	 {
//		card_center_x=near_card_x;
//		card_center_y=near_card_y;
//		once=0;//只记录一次
//    }
//	}
//        CSI_dis_new_correct(card_center_x, card_center_y);//总钻风坐标对正，准备x,y速度
//        Turn_Angle_PD(turn_angle);//准备Vz转速，作用是锁住车头方向
//        Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
////          CSI_dis_new_correct(center_x, center_y);
////          Turn_Angle_PD(90.0);//准备Vz转速，作用是锁住车头方向
////          Car_Inverse_kinematics_solution(Vx, Vy, Vz); // 麦轮控制，为target_speed赋值
//        ips114_show_int(0,0, card_center_x,4);
//        ips114_show_int(0,20,card_center_y,4);//显示卡片中心坐标
//        ips114_show_int(90,0,delta_x,4);
//        ips114_show_int(90,20,delta_y,4);//显示x调整标志位
//        ips114_show_int(90,40,correct_x_flag,4);//显示y调整标志位
//        ips114_show_int(90,60,correct_y_flag,4);//显示y调整标志位
        // ips114_show_int(150,20,card_center_x,3);//第一次捕捉到卡片的y坐标
        // ips114_show_int(150,40,card_center_y, 3); // 用于观测行进函数的步数
        //    ips114_show_float(0,80,Vx,3,2);
        //    ips114_show_float(0,100,Vy,3,2);//显示x,y速度
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
        // car_findcard(&car_mode);//模式选择
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
