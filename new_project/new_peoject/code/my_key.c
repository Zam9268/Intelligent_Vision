#include "my_key.h"
#include "zf_common_headfile.h"
struct key keys[4]; // 定义结构体

/*按键扫描函数*/
void key_scan(void) // 最好隔20ms进入一次中断
{
    /*读取对应IO口的状态*/
    keys[0].key_now_state = gpio_get_level(C12);
    keys[1].key_now_state = gpio_get_level(C14);
    keys[2].key_now_state = gpio_get_level(C13);
    keys[3].key_now_state = gpio_get_level(C15);
    for (uint8 i = 0; i < 4; i++)
    {
        switch (keys[i].key_short_state)
        {
        case 0:
        {
            if (keys[i].key_now_state == 0)
            {
                keys[i].key_short_state = 1;
            }
        }
        break;
        case 1:
        {
            if (keys[i].key_now_state == 0)
            {
                keys[i].key_short_state = 2;
            }
            else
            {
                keys[i].key_short_state = 0;
            }
        }
        break;
        case 2:
        {
            if (keys[i].key_now_state == 1)
            {
                keys[i].single_flag = 1;
                keys[i].key_short_state = 0; // 状态归0
            }
        }
        break;
        }
    }
}

/*按键处理函数
菜单：
第一级：视觉处理 （1.总钻风阈值调节   2.  ）
第二级：电控处理 （1.速度调节    ）
按键3  发车
按键4：复位键

*/
uint8 first_menu = 1;    // 第一级菜单，一开始初始化为第一级菜单
uint8 second_menu = 0;   // 第二级菜单
uint8 third_menu = 0;    // 第三级菜单
uint8 visual_menu = 0;   // 视觉处理菜单
uint8 electric_menu = 0; // 电控处理菜单
uint8 visual_mode = 0;   // 视觉处理模式，1选择总钻风阈值调节 2选择识别分类art亮度调节    3.选择目标检测art亮度调节
uint8 visual_show2 = 0;
int test_flag = 0;
typedef enum
{
    FIRST_MENU,
    SECOND_MENU,
    THIRD_MENU
} menu; // 定义赛道元素类型枚举

void my_key_handle(void)
{
    if (keys[0].single_flag == 1)
    {
        Edge_threshold += 100;   // 阈值连接增加100
        keys[0].single_flag = 0; // 清除标志位
        if (first_menu == 1)     // 如果现在是第一级菜单
        {
            second_menu = 1; // 进入第二级菜单
            visual_menu = 1; // 视觉处理菜单启动
            first_menu = 0;  // 第一级菜单清零
        }
        else if (second_menu == 1 && visual_menu == 1) // 如果是第二级菜单且是视觉处理菜单
        {
            visual_mode = 1; // 总钻风阈值调节选择
        }
    }
    else if (keys[1].single_flag == 1)
    {
        Edge_threshold -= 100; // 阈值连接减少100
        keys[1].single_flag = 0;
        if (first_menu == 1)
        {
            second_menu = 1;
            electric_menu = 1;
            first_menu = 0;
        }
        else if (second_menu == 1 && electric_menu == 1)
        {
        }
        else if (second_menu == 1 && visual_menu == 1)
        {
            visual_mode = 2; // 选择分类识别art亮度调节
        }
    }
    else if (keys[2].single_flag == 1)
    {
        test_flag ++;
        ips114_clear();
        visual_show2 = 1;
        keys[2].single_flag = 0;
    }
    else if (keys[3].single_flag == 1)
    {
        test_flag --;
        visual_show2 = 0;
        keys[3].single_flag = 0;
    }
}

/*
按键主要选择情况：上移光标，下移光标，选择，返回上一层

按键第一层 ：visual_mode  ----光标
            eletric_mode
            car_move
            meal_fuwei
*/

uint8 guangbiao_position = 1; // 光标位置，初始化为1

uint8 choose_mode_flag = 0;   // 选择模式标志位
uint8 return_flag = 0;        // 返回标志位
uint8 reset_flag = 0;         // 复位标志位
uint8 cengji_count = 0;       // 层级计数
uint8 first_mode = 1;   //第一级
char mode_name[10][20]={"Edge_threshold","normal_speed","top_line_speed","cross_island","MT9V034_light"};
extern float target_all_speed;       // 正常循迹目标速度
extern float target_upline_speed;    // 巡线上边界目标速度
uint8 muce_left_or_right_pick = 0;   // 目测捡卡片的是左环岛还是右十字，0默认为左十字
uint8 muce_island_left_or_right = 0; // 目测卡片环岛，0为左环岛
int MT9V30X_pugaodu = 0;             // MT9V30X图像附加曝光度
void my_key_handle_plus(void)
{
    if (keys[0].single_flag == 1)
    {
        ips114_clear(); // 清屏
        guangbiao_position++;    // 光标位置加1
        keys[0].single_flag = 0; // 清除标志
    }
    else if (keys[1].single_flag == 1)
    {
        ips114_clear();//清屏
        guangbiao_position--; // 光标位置减1
        keys[1].single_flag = 0;
    }
    else if (keys[2].single_flag == 1)
    {
        ips114_clear();//清屏
        choose_mode_flag = 1; // 光标位置减1
        keys[2].single_flag = 0;
    }
    else if (keys[3].single_flag == 1)
    {
        ips114_clear();//清屏
        reset_flag = 1;
        keys[3].single_flag = 0;
    }

    /*对光标位置进行限幅*/
    if (guangbiao_position == 0)
    {
        guangbiao_position = 16;
    }
    else if (guangbiao_position == 17)
    {
        guangbiao_position = 1;
    }

    /*选择模式*/
    if (choose_mode_flag == 1)
    {
        if (cengji_count == 1)
        {
            first_mode = guangbiao_position; // 记录当前的选择模式
            cengji_count++;                  // 层级数递增
        }
        else if (cengji_count == 2) // 如果进入第二级菜单
        {
            switch (first_mode)
            {
            case 1: // 阈值调节
            {
                if (guangbiao_position == 1)
                {
                    Edge_threshold += 100;
                }
                else if (guangbiao_position == 2)
                {
                    Edge_threshold -= 100;
                }
            }
            break;
            case 2: // 正常循迹速度调节
            {
                if (guangbiao_position == 1)
                {
                    target_all_speed += 1.00;
                }
                else if (guangbiao_position == 2)
                {
                    target_all_speed -= 1.00;
                }
            }
            break;

            case 3: // 十字圆环速度调节
            {
                if (guangbiao_position == 1)
                {
                    target_upline_speed += 1.00;
                }
                else if (guangbiao_position == 2)
                {
                    target_upline_speed -= 1.00;
                }
            }
            break;

            case 4: // 目测选择左十字还是右十字模式，左环岛还是右环岛模式
            {
                if (guangbiao_position == 1)
                {
                    muce_left_or_right_pick = 1; // 卡片放在左环岛
                }
                else if (guangbiao_position == 2)
                {
                    muce_island_left_or_right = 1; // 卡片放在右环岛
                }
            }
            break;

            case 5://选择曝光度模式
            {
                if (guangbiao_position == 1)
                {
                    MT9V30X_pugaodu += 1; // 曝光度增加
                }
                else if (guangbiao_position == 2)
                {
                    target_upline_speed -= 1;//削减对应的亮度
                }
            }
            break;
            }
        }
    }

    uint8 firt_mode_show=first_mode%3;//取余，只能在0-2之间
    /*菜单选择界面显示：只选择同时显示3行变量*/
    if(choose_mode_flag==1)//如果是第一级菜单
    {
        /*显示部分菜单*/
        if(first_mode==1)
        {
            ips114_show_string(0,0,&mode_name[0][20]);
            ips114_show_string(0,20,&mode_name[1][20]);
            ips114_show_string(0,40,&mode_name[2][20]);

            /*光标显示*/
            ips114_show_string(0,90,"<------");
        }
        else
        {
            ips114_show_string(0,0,&mode_name[first_mode-1][20]);
            ips114_show_string(0,20,&mode_name[first_mode][20]);
            ips114_show_string(0,40,&mode_name[first_mode+1][20]);

            /*光标显示*/
            ips114_show_string(0,20,"<------");
        }
    }
    else if(choose_mode_flag==2)//进入单级菜单调节
    {
			switch(first_mode)
			{
        case  1://阈值调节
        {
            ips114_show_string(0,0,"+100");
            ips114_show_string(0,20,"-100");
            ips114_show_int(0,40,Edge_threshold,4);
					
        }break;
        case 2:
        {
            ips114_show_string(0,0,"+1.00");
            ips114_show_string(0,20,"-1.00");
            ips114_show_float(0,40,target_all_speed,3,3);
					break;
        }
        case 3:
        {
            ips114_show_string(0,0,"+1.00");
            ips114_show_string(0,20,"-1.00");
            ips114_show_float(0,40,target_upline_speed,3,3);
					break;
        }
        
        case 4:
        {
            ips114_show_string(0,0,"crossing");
            ips114_show_string(0,20,"island");
            ips114_show_string(0,40,"cross_or_isand");
					break;
        }
        case 5:
        {
            ips114_show_string(0,0,"+1");
            ips114_show_string(0,20,"-1");
            ips114_show_uint(0,40,MT9V30X_pugaodu+32,3);//32是固定值
					break;
        }
        
        default:break;
			}
    }
}