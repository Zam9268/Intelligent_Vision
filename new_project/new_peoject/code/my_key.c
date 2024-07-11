#include "my_key.h"

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
uint8 test_flag = 0;
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
        test_flag = 1;
        visual_show2 = 1;
        keys[2].single_flag = 0;
    }
    else if (keys[3].single_flag == 1)
    {
        test_flag = 0;
        visual_show2 = 0;
        keys[3].single_flag = 0;
    }
}
