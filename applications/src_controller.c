/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-25     Tery       the first version
 */
#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include <string.h>

#define DBG_TAG "src_controller"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "src_controller.h"
#include "om_1_sensor.h"
#include "calibration.h"
#include "om_1_data.h"
#include "om_1_power.h"
#include "om_1_ddc112.h"
#include "om_1_session.h"

#define OM_BT_DATA_REPORT_INTERVAL_TIME  300 /*ms*/
#define OM_BT_DATA_DELAY_INTERVAL_TIME  10 /*ms*/

#define PREVIOU_KEY     GET_PIN(E, 9)
#define NEXT_KEY  GET_PIN(E,10)
#define COMFIRM_KEY   GET_PIN(E,11)
#define CANCEL_KEY  GET_PIN(E,12)

static rt_thread_t om_src_ctr_thread = RT_NULL;
uint8_t g_skip_mode = 0;
uint8_t g_current_page_num = 0;

uint32_t cal_1_value = 0;
uint32_t cal_2_value = 50000;
uint32_t cal_3_value = 1000000;
uint32_t current = 0;
uint64_t fid_total = 0;
rt_uint32_t _fid_ave_buf[20] = {0};
uint8_t fid_ave_count = 0;
uint32_t fid_ave_value = 0;
static rt_device_t src_uart6;

uint8_t cal_1_data[10] =  {0x5a,0xa5,0x07,0x82,0x10,0x00,0x00,0x00,0x00,0x00};
uint8_t cal_2_data[10] =  {0x5a,0xa5,0x07,0x82,0x10,0x08,0x00,0x00,0x00,0x00};
uint8_t cal_3_data[10] =  {0x5a,0xa5,0x07,0x82,0x10,0x16,0x00,0x00,0x00,0x00};
uint8_t FID_data[10] =  {0x5a,0xa5,0x07,0x82,0x10,0xC0,0x00,0x00,0x00,0x00};
uint8_t signal_value_data[10] =  {0x5a,0xa5,0x07,0x82,0x12,0x40,0x00,0x00,0x00,0x00};
uint8_t warning_data[10] =  {0x5a,0xa5,0x07,0x82,0x12,0x60,0x00,0x00,0x00,0x00};
uint8_t data_save_time_data[10] =  {0x5a,0xa5,0x07,0x82,0x12,0x68,0x00,0x00,0x00,0x00};
uint8_t fid_value_red_color_data[8] =  {0x5a,0xa5,0x05,0x82,0x50,0xC3,0xF8,0x00};
uint8_t fid_value_while_color_data[8] =  {0x5a,0xa5,0x05,0x82,0x50,0xC3,0xFF,0xFF};
uint8_t bat_value_red_color_data[8] =  {0x5a,0xa5,0x05,0x82,0x50,0x73,0xF8,0x00};
uint8_t bat_value_while_color_data[8] =  {0x5a,0xa5,0x05,0x82,0x50,0x73,0xFF,0xFF};
uint8_t mol_select_data[15] =  {0x5a,0xa5,0x0C,0x82,0x10,0x50,0x08,0x75,0x6d,0x6f,0x6c,0x2f,0x6d,0x6f,0x6c};
uint8_t ppm_select_data[15] =  {0x5a,0xa5,0x0C,0x82,0x10,0x50,0x08,0x70,0x70,0x6d,0x00,0x00,0x00,0x00,0x00};
uint8_t mg_select_data[15] =  {0x5a,0xa5,0x0C,0x82,0x10,0x50,0x08,0x6d,0x67,0x2f,0x6d,0x33,0x00,0x00,0x00};
uint8_t fid_curve_data[16] =  {0x5a,0xa5,0x0D,0x82,0x03,0x10,0x5A,0xA5,0x01,0x00,0x07,0x02,0x00,0x00,0x00,0x00};
/* umol/mol  5a a5 0C 82 12 10 08 75 6d 6f 6c 2f 6d 6f 6c
 * ppm       5a a5 0C 82 12 10 08 70 70 6d 00 00 00 00 00
 * mg/m3     5a a5 0C 82 12 10 08 6d 67 2f 6d 33 00 00 00*/

static struct _m_device_set_data m_device_set_data = {0,0,0,0,0,0,0,0,0};
struct _fid_save_data fid_save_data = {0,0,0};

/*
 * struct om_page_data
{
    uint8_t confirm_page_num;
    uint8_t previous_page_num;
    uint8_t next_page_num;
    uint8_t cancel_page_num;
};
*/

struct om_page_data page_index_list[] = {{0,0,0,0},   //0
                                         {0,0,0,0},   //1
                                         {0,0,0,0},   //2
                                         {0,0,0,0},   //3
                                         {0,0,0,0},   //4
                                         {0,0,0,0},   //5
                                         {0,0,0,0},   //6
                                         {7,7,8,7},      //7
                                         {22,7,9,8},      //8
                                         {26,8,10,9},     //9
                                         {38,9,11,10},     //10
                                         {11,10,12,11},     //11  /*检测*/
                                         {50,11,13,12},     //12
                                         {58,12,13,13},      //13
                                         {0,0,0,0},   //14
                                         {0,0,0,0},   //15
                                         {0,0,0,0},   //16
                                         {0,0,0,0},   //17
                                         {0,0,0,0},   //18
                                         {0,0,0,0},   //19
                                         {7,7,7,7},   //20
                                         {7,21,21,21},   //21
                                         {8,23,23,8},  //22
                                         {8,22,22,8},     //23
                                         {26,25,25,9},    //24
                                         {26,24,24,9},      //25
                                         {29,27,27,9},   //26
                                         {29,26,26,9},   //27
                                         {28,28,28,28},   //28
                                         {30,29,31,9},   //29
                                         {30,30,30,30},   //30
                                         {31,29,32,9},   //31
                                         {33,31,34,9},   //32
                                         {33,33,33,33},   //33
                                         {34,32,35,9},   //34
                                         {35,34,37,9},   //35
                                         {36,36,36,36},   //36
                                         {38,35,37,9},   //37
                                         {40,38,38,10},   //38
                                         {40,39,39,10},   //39
                                         {38,41,41,38},   //40
                                         {42,40,40,38},   //41
                                         {38,38,38,38},   //42
                                         {0,0,0,0},   //43
                                         {0,0,0,0},   //44
                                         {0,0,0,0},   //45
                                         {0,0,0,0},   //46
                                         {0,0,0,0},   //47
                                         {0,0,0,0},   //48
                                         {11,11,11,11},   //49
                                         {54,50,51,12},   //50
                                         {55,50,52,12},   //51
                                         {57,51,53,12},   //52
                                         {56,52,53,12},   //53
                                         {54,54,54,50},   //54
                                         {55,55,55,51},   //55
                                         {56,56,56,52},   //56
                                         {57,57,57,53},   //57
                                         {58,58,59,13},   //58
                                         {59,58,60,13},   //59
                                         {60,59,61,13},   //60
                                         {61,60,62,13},   //61
                                         {62,61,63,13},   //62
                                         {63,62,64,23},   //63
                                         {64,63,65,13},   //64
                                         {65,64,66,13},   //65
                                         {66,65,67,13},   //66
                                         {67,66,68,13},   //67
                                         {68,67,69,13},   //68
                                         {69,68,70,13},   //69
                                         {70,69,71,13},   //70
                                         {71,70,72,13},   //71
                                         {72,71,73,13},   //72
                                         {73,72,74,13},   //73
                                         {74,73,75,13},   //74
                                         {75,74,76,13},   //75
                                         {76,75,77,13},   //76
                                         {77,76,78,13},   //77
                                         {78,77,79,13},   //78
                                         {81,78,79,13},   //79
                                         {0,0,0,0},   //80
                                         {13,13,13,13},   //81
                                         {82,82,83,82},   //82
                                         {83,82,84,83},   //83
                                         {84,83,85,84},   //84
                                         {85,84,86,85},   //85
                                         {86,85,87,86},   //86
                                         {87,86,88,87},   //87
                                         {88,87,89,88},   //88
                                         {89,88,90,89},   //89
                                         {90,89,91,90},   //90
                                         {91,90,92,91},   //91
                                         {92,91,93,92},   //92
                                         {93,92,94,93},   //93
                                         {94,93,94,94},   //94
                                         };

static void om_src_ctr(void *parameter);
void om_key_init(void);

int om_src_uart_init(void);
void om_paging(uint8_t page_num);
void bat_power_show(void);
void bg_value_show(uint32_t _fid_value);
void fid_value_show(uint32_t _fid_value);
void signal_value_show(uint32_t _signal_value);
void fire_tem_value_show(uint32_t _fire_tem_value);
uint32_t keyboard_input(uint8_t previou_page,uint32_t previou_value);
void byte_order_conversion_uint32(rt_uint8_t* conv_buf,rt_uint32_t conv_value);
void om_set_device_config(void);
void om_multi_sensor_set(void);
void fid_max_value_show(uint32_t _fid_max_value);
void tem_value_show(uint32_t _tem_value);
void fid_ave_value_show(uint32_t _fid_ave_value);
void om_fid_curve_show(uint32_t curve_data);
void om_history_data_show(uint8_t current_index);
void om_history_data_save(uint32_t _fid_value,uint8_t unit);
void om_history_data_show_init(void);

void om_src_monitor(void)
{
    om_key_init();
    om_src_uart_init();

    byte_order_conversion_uint32(&cal_1_data[6],cal_1_value);
    rt_device_write(src_uart6, 0, cal_1_data, 10);
    byte_order_conversion_uint32(&cal_2_data[6],cal_2_value);
    rt_device_write(src_uart6, 0, cal_2_data, 10);
    byte_order_conversion_uint32(&cal_3_data[6],cal_3_value);
    rt_device_write(src_uart6, 0, cal_3_data, 10);

    om_src_ctr_thread = rt_thread_create("om_src_ctr_thread", om_src_ctr, RT_NULL, 2068, 13, 500);
    if (om_src_ctr_thread != RT_NULL)
    {
        rt_thread_startup(om_src_ctr_thread);
        LOG_D("om_monitor_ctr_thread is running\n");
    }
}

void byte_order_conversion_uint32(rt_uint8_t* conv_buf,rt_uint32_t conv_value)
{
    conv_buf[3] = conv_value;
    conv_buf[2] = (conv_value >> 8);
    conv_buf[1] = (conv_value >> 16);
    conv_buf[0] = (conv_value >> 24);
}

uint32_t keyboard_input(uint8_t previou_page,uint32_t previou_value)
{
    uint32_t keyborad_value = 0;
    uint8_t select_value = 0;

    uint8_t keyborad_data[10] =  {0x5a,0xa5,0x07,0x82,0x12,0x70,0x00,0x00,0x00,0x00};

    g_skip_mode = SKIP_MODE_NONE;
    g_current_page_num = 82;

    om_paging(g_current_page_num);
    rt_device_write(src_uart6, 0, keyborad_data, 10);
    while (1)
    {
        select_value = g_current_page_num - 82;
        if(g_skip_mode == SKIP_MODE_PREVIOU)
        {
            g_current_page_num = page_index_list[g_current_page_num].previous_page_num;
            om_paging(g_current_page_num);
        }
        else if(g_skip_mode == SKIP_MODE_NEXT)
        {
            g_current_page_num = page_index_list[g_current_page_num].next_page_num;
            om_paging(g_current_page_num);
        }
        else if(g_skip_mode == SKIP_MODE_CANCEL)
        {
            g_current_page_num = previou_page;
            om_paging(g_current_page_num);
            keyborad_value = previou_value;
            break;
        }
        else if(g_skip_mode == SKIP_MODE_CONFIRM)
        {
            if(g_current_page_num == 93)    //确认
            {
                g_current_page_num = previou_page;
                om_paging(g_current_page_num);
                break;
            }
            else if(g_current_page_num == 94)   //删除
            {
                keyborad_value = (keyborad_value / 10);
            }
            else if(g_current_page_num == 92)   //小数点
            {
                keyborad_value = keyborad_value;
            }
            else
            {

                if(keyborad_value != 0)
                {
                    if(keyborad_value < 100000000)
                    {
                        keyborad_value = (keyborad_value*10 + select_value);
                    }
                }
                else {
                    keyborad_value = select_value;
                }
            }
            g_current_page_num = page_index_list[g_current_page_num].confirm_page_num;
            om_paging(g_current_page_num);
        }
        byte_order_conversion_uint32(&keyborad_data[6],keyborad_value);
        rt_device_write(src_uart6, 0, keyborad_data, 10);
        g_skip_mode = SKIP_MODE_NONE;
        rt_thread_mdelay(600);
    }
    return keyborad_value;
}

uint32_t test_signal = 16000,test_signal2 = 50000, test_signal3 = 100000;
void om_src_ctr(void *parameter)
{
    uint32_t fid_value  = 0;
    uint32_t fid_max_value  = 0;
    uint32_t waring_value  = 0;
    uint32_t data_save_time_value  = 0;
    uint32_t bg_value = 0;
    uint8_t history_page_index = 0;

    om_device_set_data_read((uint8_t*)&m_device_set_data,14);
    om_set_device_config();
    om_multi_sensor_set();
    om_history_data_read((uint8_t*)&fid_save_data,sizeof(fid_save_data));
    om_history_data_show_init();

    data_save_time_value = m_device_set_data.data_save_time;
    waring_value = m_device_set_data.threshould_value;

    byte_order_conversion_uint32(&warning_data[6],waring_value);
    rt_device_write(src_uart6, 0, warning_data, 10);
    byte_order_conversion_uint32(&data_save_time_data[6],data_save_time_value);
    rt_device_write(src_uart6, 0, data_save_time_data, 10);

    bat_power_show();
    bg_value_show(bg_value);
    /*<<< 开机动画 */
    g_current_page_num = 0;
    om_paging(g_current_page_num);//加载中
    rt_thread_mdelay(1000);
    for(int i =0;i<18;i++)
    {
        g_current_page_num = i%6 + 1;
        om_paging(g_current_page_num);//......
        rt_thread_mdelay(300);
    }
    /* 开机动画 >>>*/
    g_current_page_num = 7;
    om_paging(g_current_page_num);
    while (1)
    {
        if(g_skip_mode == SKIP_MODE_PREVIOU)
        {
            if(g_current_page_num == 54)   //查看历史数据
            {
                if(history_page_index != 0)
                {
                    history_page_index = history_page_index - 7;
                    om_history_data_show(history_page_index);
                }

            }
            {
                g_current_page_num = page_index_list[g_current_page_num].previous_page_num;
                om_paging(g_current_page_num);
            }
        }
        else if(g_skip_mode == SKIP_MODE_NEXT)
        {
            if((g_current_page_num == 38) || (g_current_page_num == 39))
            {
                fid_max_value = 0;
            }
            else if(g_current_page_num == 54)   //查看历史数据
            {
                if(history_page_index <20 && fid_save_data.save_counts > history_page_index)
                {
                    history_page_index = history_page_index + 7;
                    om_history_data_show(history_page_index);
                }

            }

            g_current_page_num = page_index_list[g_current_page_num].next_page_num;
            om_paging(g_current_page_num);
        }
        else if(g_skip_mode == SKIP_MODE_CANCEL)
        {
            g_current_page_num = page_index_list[g_current_page_num].cancel_page_num;
            om_paging(g_current_page_num);
        }
        else if(g_skip_mode == SKIP_MODE_CONFIRM)
        {
            /*点火
             * 当点火流程执行完后，判断火焰温度，当温度大于120时，点火成功*/
            if(g_current_page_num == 7)
            {
                g_current_page_num = page_index_list[g_current_page_num].confirm_page_num;
                om_paging(g_current_page_num);
                om_ignite_set();
                while(om_get_ignite_step() != 7)  /*点火流程判断*/
                {
                    /*<<< 点火动画 */
                    for(int i =0;i<6;i++)
                    {
                        g_current_page_num = i%6 + 14;
                        om_paging(g_current_page_num);
                        rt_thread_mdelay(300);
                    }
                    /* 点火动画 >>>*/
                }
                if(om_thermocouple_read() > 3084) /*温度大于120判断点火成功*/
                {
                    /*todo 换图*/
                    g_current_page_num = 20;
                    om_paging(g_current_page_num);
                    rt_thread_mdelay(2000);
                    g_current_page_num = 38;
                    om_paging(g_current_page_num);
                }
                else {
                    om_ignite_reset();
                    g_current_page_num = 21;
                    om_paging(g_current_page_num);
                    rt_thread_mdelay(1000);
                }
            }
            else if(g_current_page_num == 8)
            {
                if(om_get_ignite_step() == 7)
                {
                    g_current_page_num = page_index_list[g_current_page_num].confirm_page_num;
                    om_paging(g_current_page_num);
                }
            }
            else if(g_current_page_num == 9)
            {
                if(m_device_set_data.sensor_select != 0)
                {
                    g_current_page_num = 24;
                }
                else {
                    g_current_page_num = page_index_list[g_current_page_num].confirm_page_num;
                }
                om_paging(g_current_page_num);
            }
            /*关火确认*/
            else if(g_current_page_num == 23)
            {
                g_current_page_num = page_index_list[g_current_page_num].confirm_page_num;
                om_ignite_reset();
                om_paging(g_current_page_num);
            }
            else if(g_current_page_num == 10)   //进入到检测界面
            {
                if(m_device_set_data.sensor_select == 0)  /*检测多传感器配置*/
                {
                    g_current_page_num = 38;
                }
                else {
                    g_current_page_num = 39;
                }
                om_paging(g_current_page_num);
            }
            else if(g_current_page_num == 9)   //进入到校准界面
            {
                if(m_device_set_data.sensor_select == 0)  /*检测多传感器配置*/
                {
                    g_current_page_num = 26;
                }
                else {
                    g_current_page_num = 24;
                }
                om_paging(g_current_page_num);
            }
            else if(g_current_page_num == 27)   //校准曲线清除
            {
#ifndef SELF_CALIBRATION
                curve_reset(FID_CURVE);
#else
				auto_curve_updata(FID_CURVE);
#endif
				g_current_page_num = page_index_list[g_current_page_num].confirm_page_num;
                om_paging(g_current_page_num);
            }
            else if(g_current_page_num == 29 ||
                    g_current_page_num == 32 ||
                    g_current_page_num == 35)
            {
                rt_thread_mdelay(100);
                if(g_current_page_num == 29)
                {
                	curve_reset(FID_CURVE);
					test_signal+=500;
                    curve_updata(FID_CURVE,0,test_signal);//om_get_ddc112_value()*20/19);
                }
                else if(g_current_page_num == 32){
					test_signal2+=700;
                    curve_updata(FID_CURVE,500,test_signal2);//om_get_ddc112_value());
                }
                else if(g_current_page_num == 35){
					test_signal3+=1000;
                    curve_updata(FID_CURVE,10000,test_signal3);//om_get_ddc112_value());
                }
                om_paging(g_current_page_num + 1);
                rt_thread_mdelay(4000);
                om_paging(g_current_page_num);
                rt_thread_mdelay(100);
            }
            else if(g_current_page_num == 37)   //进入到检测界面
            {
                if(m_device_set_data.sensor_select == 0)  /*检测多传感器配置*/
                {
                    g_current_page_num = 38;
                }
                else {
                    g_current_page_num = 39;
                }
                om_paging(g_current_page_num);
            }
//            else if((g_current_page_num == 38)   //手动保存功能选择
//                    ||(g_current_page_num == 39))
//            {
//                if(m_device_set_data.data_save_type == 0)  /*检测多传感器配置*/
//                {
//                    g_current_page_num = 40;
//                    om_paging(g_current_page_num);
//                }
//            }
            else if(g_current_page_num == 42)     //手动保存功能确认
            {
                if(m_device_set_data.data_save_type == 0)  /*检测多传感器配置*/
                {
                    g_current_page_num = 38;
                    om_paging(g_current_page_num);
                }
                else {
                    g_current_page_num = 39;
                    om_paging(g_current_page_num);
                }
                om_history_data_save(get_fid_calbarate_value(),m_device_set_data.unit_select);
            }
            else if(g_current_page_num == 11)  /*背景值扣除*/
            {
                /*<<< 背景值检测动画 */
                for(int i =0;i<30;i++)
                {
                    g_current_page_num = i%6 + 43;
                    om_paging(g_current_page_num);
                    rt_thread_mdelay(300);
                }
                /* 背景值检测动画 >>>*/
                g_current_page_num = 49;
                om_paging(g_current_page_num);
                bg_value = get_fid_calbarate_value();
                bg_value_show(bg_value);
                rt_thread_mdelay(2000);

                g_current_page_num = 11;
            }
            else if(g_current_page_num == 50)   //查看历史数据
            {
                g_current_page_num = 54;
                om_paging(g_current_page_num);
                history_page_index = 0;
                om_history_data_show(history_page_index);
            }
            else if(g_current_page_num == 61)   //报警值设置
            {
                waring_value = keyboard_input(61,waring_value);
                m_device_set_data.threshould_value = waring_value;
                byte_order_conversion_uint32(&warning_data[6],waring_value);
                rt_device_write(src_uart6, 0, warning_data, 10);
            }
            else if(g_current_page_num == 70)   //存储值设置
            {
                data_save_time_value = keyboard_input(70,data_save_time_value);
                if(data_save_time_value == 0)
                {
                    data_save_time_value = 10;
                }
                m_device_set_data.data_save_time = data_save_time_value;
                byte_order_conversion_uint32(&data_save_time_data[6],data_save_time_value);
                rt_device_write(src_uart6, 0, data_save_time_data, 10);
            }
            else if((g_current_page_num == 58))/*单位选择*/
            {
                m_device_set_data.unit_select = 0;
                om_set_device_config();
                rt_device_write(src_uart6, 0, ppm_select_data, 15);
            }
            else if((g_current_page_num == 59))/*单位选择*/
            {
                m_device_set_data.unit_select = 1;
                om_set_device_config();
                rt_device_write(src_uart6, 0, mol_select_data, 15);
            }
            else if((g_current_page_num == 60))/*单位选择*/
            {
                m_device_set_data.unit_select = 2;
                om_set_device_config();
                rt_device_write(src_uart6, 0, mg_select_data, 15);
            }
            else if((g_current_page_num == 62))/*手动存储选择*/
             {
                m_device_set_data.data_save_type = 0;
                om_set_device_config();
             }
            else if((g_current_page_num == 63))/*自动存储选择*/
             {
                m_device_set_data.data_save_type = 1;
                om_set_device_config();
             }
            else if((g_current_page_num == 64))/*多传感器不配置*/
            {
                m_device_set_data.sensor_select = 0;
                om_set_device_config();
            }
            else if((g_current_page_num == 65))/*PID传感器选择*/
            {
                m_device_set_data.sensor_select = 1;
                om_set_device_config();
                om_multi_sensor_set();
            }
            else if((g_current_page_num == 66))/*氧气传感器选择*/
            {
                m_device_set_data.sensor_select = 2;
                om_set_device_config();
                om_multi_sensor_set();
            }
            else if((g_current_page_num == 67))/*CO传感器选择*/
             {
                m_device_set_data.sensor_select = 3;
                om_set_device_config();
                om_multi_sensor_set();
             }
            else if((g_current_page_num == 68))/*LEL传感器选择*/
             {
                m_device_set_data.sensor_select = 4;
                om_set_device_config();
                om_multi_sensor_set();
             }
            else if((g_current_page_num == 69))/*H2S传感器选择*/
             {
                m_device_set_data.sensor_select = 5;
                om_set_device_config();
                om_multi_sensor_set();
             }
            else if((g_current_page_num == 71))/*背景值仅检测选择*/
             {
                m_device_set_data.bg_value_type = 0;
                om_set_device_config();
             }
            else if((g_current_page_num == 72))/*背景值扣除选择*/
             {
                m_device_set_data.bg_value_type = 1;
                om_set_device_config();
             }
            else if((g_current_page_num == 73))/*通讯开选择*/
             {
                m_device_set_data.bt_switch = 1;
                om_set_device_config();
                esp32_at_open();
             }
            else if((g_current_page_num == 74))/*通讯关选择*/
             {
                m_device_set_data.bt_switch = 0;
                om_set_device_config();
                esp32_at_close();
             }
            else if((g_current_page_num == 75))/*关机吹扫开选择*/
             {
                m_device_set_data.sweep_switch = 1;
                om_set_device_config();
             }
            else if((g_current_page_num == 76))/*关机吹扫关选择*/
             {
                m_device_set_data.sweep_switch = 0;
                om_set_device_config();
             }
            else if((g_current_page_num == 77))/*精度单位ppm选择*/
             {
                m_device_set_data.accuracy_type = 0;
                om_set_device_config();
             }
            else if((g_current_page_num == 78))/*精度单位ppb选择*/
             {
                m_device_set_data.accuracy_type = 1;
                om_set_device_config();
             }
            else if((g_current_page_num == 79))/*确认选择*/
            {
                om_device_set_data_save((uint8_t*)&m_device_set_data,14);
                g_current_page_num = page_index_list[g_current_page_num].confirm_page_num;
                om_paging(g_current_page_num);
                rt_thread_mdelay(2000);
                g_current_page_num = 13;
            }
            else {
                g_current_page_num = page_index_list[g_current_page_num].confirm_page_num;
            }

            om_paging(g_current_page_num);
        }

        if(fid_ave_count < 19)
        {
            fid_ave_count ++;
        }
        else
        {
            fid_ave_count = 0;
        }
        _fid_ave_buf[fid_ave_count] = get_fid_calbarate_value();
        for(int i = 0;i<20;i++)
        {
            fid_total = fid_total + _fid_ave_buf[i];

        }
        fid_ave_value =  fid_total/20;
        fid_total = 0;

        if(om_thermocouple_read() > 3084)
        {
            fid_value = get_fid_calbarate_value();
            fid_max_value = (fid_value > fid_max_value) ? fid_value : fid_max_value;
            if(m_device_set_data.bg_value_type == 1)
            {
                fid_value =  (fid_value > bg_value) ? (fid_value - bg_value) : 0;
            }
            fid_value_show(fid_value);
            if((fid_value > (waring_value * 100) ) && (waring_value > 0))
            {
                rt_device_write(src_uart6, 0, fid_value_red_color_data, 8);
            }
            else {
                rt_device_write(src_uart6, 0, fid_value_while_color_data, 8);
            }
        }
        else {
            if(om_get_ignite_step() == 7)
            {
                om_ignite_reset();
                g_current_page_num = 21;
                om_paging(g_current_page_num);
                rt_thread_mdelay(2000);
            }
            fid_value_show( 0);
            rt_device_write(src_uart6, 0, fid_value_while_color_data, 8);

        }
        om_fid_curve_show(fid_value);
        signal_value_show( om_get_ddc112_value());
        fire_tem_value_show((om_thermocouple_read() -2731)*25 +320);
        tem_value_show((om_temperature_sensor_read() -2731)*25 +320);

        bat_power_show();
        fid_max_value_show(fid_max_value);
        fid_ave_value_show(fid_ave_value);
        g_skip_mode = SKIP_MODE_NONE;
        rt_thread_mdelay(600);
    }
}

void om_fid_curve_show(uint32_t curve_data)
{
    fid_curve_data[12] = (curve_data/10000) >> 8;
    fid_curve_data[13] = curve_data/10000;
    fid_curve_data[14] = (curve_data/10000) >> 8;
    fid_curve_data[15] = curve_data/10000;
    rt_device_write(src_uart6, 0, fid_curve_data, 16);
}
void om_paging(uint8_t page_num)
{
    uint8_t page_data[10] =  {0x5a,0xa5,0x07,0x82,0x00,0x84,0x5a,0x01,0x00,0x00};
    page_data[9] = page_num;
    rt_device_write(src_uart6, 0, page_data, 10);
}

void bat_power_show(void)
{
    uint8_t page_data[8] =  {0x5a,0xa5,0x05,0x82,0x10,0x70,0x00,0x00};
    uint16_t bat_power = om_battery_level_read();

    if(bat_power > 7600)
    {
        bat_power = 7600;
    }
    else if(bat_power < 6000)
    {
        bat_power = 6000;
    }
    if((bat_power - 6000)/16 < 10)
    {
        rt_device_write(src_uart6, 0, bat_value_red_color_data, 8);
    }
    else {
        rt_device_write(src_uart6, 0, bat_value_while_color_data, 8);
    }
    page_data[7] = (bat_power - 6000)/16;
    rt_device_write(src_uart6, 0, page_data, 8);
}

void bg_value_show(uint32_t _bg_value)
{
    uint8_t page_data[10] =  {0x5a,0xa5,0x07,0x82,0x10,0xC0,0x00,0x00,0x00,0x00};
    page_data[9] = _bg_value;
    page_data[8] = _bg_value >> 8;
    page_data[7] = _bg_value >> 16;
    page_data[6] = _bg_value >> 24;
    page_data[5] = 0xD8;
    rt_device_write(src_uart6, 0, page_data, 10);
}

void fid_ave_value_show(uint32_t _fid_ave_value)
{
    uint8_t page_data[10] =  {0x5a,0xa5,0x07,0x82,0x12,0xD0,0x00,0x00,0x00,0x00};
    page_data[9] = _fid_ave_value;
    page_data[8] = _fid_ave_value >> 8;
    page_data[7] = _fid_ave_value >> 16;
    page_data[6] = _fid_ave_value >> 24;
    page_data[5] = 0xD0;
    rt_device_write(src_uart6, 0, page_data, 10);
}
void fid_max_value_show(uint32_t _fid_max_value)
{
    uint8_t page_data[10] =  {0x5a,0xa5,0x07,0x82,0x10,0xC0,0x00,0x00,0x00,0x00};
    page_data[9] = _fid_max_value;
    page_data[8] = _fid_max_value >> 8;
    page_data[7] = _fid_max_value >> 16;
    page_data[6] = _fid_max_value >> 24;
    page_data[5] = 0xC8;
    rt_device_write(src_uart6, 0, page_data, 10);
}

void fid_value_show(uint32_t _fid_value)
{
    uint8_t page_data[10] =  {0x5a,0xa5,0x07,0x82,0x10,0xC0,0x00,0x00,0x00,0x00};
    page_data[9] = _fid_value;
    page_data[8] = _fid_value >> 8;
    page_data[7] = _fid_value >> 16;
    page_data[6] = _fid_value >> 24;
    page_data[5] = 0xC0;
    rt_device_write(src_uart6, 0, page_data, 10);
}

void signal_value_show(uint32_t _signal_value)
{
    uint8_t page_data[10] =  {0x5a,0xa5,0x07,0x82,0x12,0x40,0x00,0x00,0x00,0x00};
    page_data[9] = _signal_value;
    page_data[8] = _signal_value >> 8;
    page_data[7] = _signal_value >> 16;
    page_data[6] = _signal_value >> 24;
    page_data[5] = 0x40;
    rt_device_write(src_uart6, 0, page_data, 10);
}

void tem_value_show(uint32_t _tem_value)
{
    uint8_t page_data[10] =  {0x5a,0xa5,0x07,0x82,0x12,0x58,0x00,0x00,0x00,0x00};
    page_data[9] = _tem_value;
    page_data[8] = _tem_value >> 8;
    page_data[7] = _tem_value >> 16;
    page_data[6] = _tem_value >> 24;
    page_data[5] = 0x58;
    rt_device_write(src_uart6, 0, page_data, 10);
}
void fire_tem_value_show(uint32_t _fire_tem_value)
{
    uint8_t page_data[10] =  {0x5a,0xa5,0x07,0x82,0x12,0x50,0x00,0x00,0x00,0x00};
    page_data[9] = _fire_tem_value;
    page_data[8] = _fire_tem_value >> 8;
    page_data[7] = _fire_tem_value >> 16;
    page_data[6] = _fire_tem_value >> 24;
    page_data[5] = 0x50;
    rt_device_write(src_uart6, 0, page_data, 10);
}
void om_history_data_show_init(void)
{
    uint8_t inde_color_data[8] =  {0x5a,0xa5,0x05,0x82,0x53,0x63,0xFF,0xFF};
    uint8_t value_color_data[8] =  {0x5a,0xa5,0x05,0x82,0x53,0x9D,0xFF,0xFF};
    uint8_t unit_color_data[8] =  {0x5a,0xa5,0x05,0x82,0x53,0x03,0xFF,0xFF};
    LOG_I("fid_save_data.save_counts :%d\r\n",fid_save_data.save_counts);
    for(uint8_t i;i<7;i++)
    {
        inde_color_data[5] = 0x63 + 8*i;
        inde_color_data[6] = 0x00;
        inde_color_data[7] = 0x00;
        value_color_data[5] = 0x9D + 8*i;
        value_color_data[6] = 0x00;
        value_color_data[7] = 0x00;
        unit_color_data[5] = 0x03 + 12*i;
        unit_color_data[6] = 0x00;
        unit_color_data[7] = 0x00;
        rt_device_write(src_uart6, 0, inde_color_data, 8);
        rt_device_write(src_uart6, 0, value_color_data, 8);
        rt_device_write(src_uart6, 0, unit_color_data, 8);
    }
}

void om_history_data_show(uint8_t current_index)
{
    uint8_t index_data[8] =  {0x5a,0xa5,0x05,0x82,0x13,0x60,0x00,0x00};
    uint8_t value_data[10] =  {0x5a,0xa5,0x07,0x82,0x13,0x9A,0x00,0x00,0x00,0x00};
    uint8_t unit_ppm_data[14] =   {0x5a,0xa5,0x0B,0x82,0x13,0x00,0x70,0x70,0x6d,0x00,0x00,0x00,0x00,0x00};
    uint8_t unit_mg_data[14] =    {0x5a,0xa5,0x0B,0x82,0x13,0x00,0x6d,0x67,0x2f,0x6d,0x33,0x00,0x00,0x00};
    uint8_t unit_mol_data[14] =   {0x5a,0xa5,0x0B,0x82,0x13,0x00,0x75,0x6d,0x6f,0x6c,0x2f,0x6d,0x6f,0x6c};
    uint8_t inde_color_data[8] =  {0x5a,0xa5,0x05,0x82,0x53,0x63,0xFF,0xFF};
    uint8_t value_color_data[8] =  {0x5a,0xa5,0x05,0x82,0x53,0x9D,0xFF,0xFF};
    uint8_t unit_color_data[8] =  {0x5a,0xa5,0x05,0x82,0x53,0x03,0xFF,0xFF};
    LOG_I("fid_save_data.save_counts :%d\r\n",fid_save_data.save_counts);
    for(uint8_t i;i<7;i++)
    {
        index_data[7] = i + current_index + 1; /*序号从1开始*/
        if(fid_save_data.save_counts >= index_data[7])
        {
            inde_color_data[5] = 0x63 + 8*i;
            inde_color_data[6] = 0xFF;
            inde_color_data[7] = 0xFF;
            value_color_data[5] = 0x9D + 8*i;
            value_color_data[6] = 0xFF;
            value_color_data[7] = 0xFF;
            unit_color_data[5] = 0x03 + 12*i;
            unit_color_data[6] = 0xFF;
            unit_color_data[7] = 0xFF;
            rt_device_write(src_uart6, 0, inde_color_data, 8);
            rt_device_write(src_uart6, 0, value_color_data, 8);
            rt_device_write(src_uart6, 0, unit_color_data, 8);

            index_data[5] = 0x60 + 8*i; /*序号从1开始*/
            index_data[6] = 0;

            rt_device_write(src_uart6, 0, index_data, 8);

            value_data[5] = 0x9A + 8*i;
            value_data[9] = fid_save_data.fid_value[i+current_index];
            value_data[8] = fid_save_data.fid_value[i+current_index] >> 8;
            value_data[7] = fid_save_data.fid_value[i+current_index] >> 16;
            value_data[6] = fid_save_data.fid_value[i+current_index] >> 24;
            rt_device_write(src_uart6, 0, value_data, 10);


            if(fid_save_data.value_unit[i+current_index] == 0)
            {
                unit_ppm_data[5] = 0x00 + 12*i;
                rt_device_write(src_uart6, 0, unit_ppm_data, 14);
            }
            else if(fid_save_data.value_unit[i+current_index] == 1)
            {
                unit_mg_data[5] = 0x00 + 12*i;
                rt_device_write(src_uart6, 0, unit_mg_data, 14);
            }
            else
            {
                unit_mol_data[5] = 0x00 + 12*i;
                rt_device_write(src_uart6, 0, unit_mol_data, 14);
            }
        }
        else
        {
            inde_color_data[5] = 0x63 + 8*i;
            inde_color_data[6] = 0x00;
            inde_color_data[7] = 0x00;
            value_color_data[5] = 0x9D + 8*i;
            value_color_data[6] = 0x00;
            value_color_data[7] = 0x00;
            unit_color_data[5] = 0x03 + 12*i;
            unit_color_data[6] = 0x00;
            unit_color_data[7] = 0x00;
            rt_device_write(src_uart6, 0, inde_color_data, 8);
            rt_device_write(src_uart6, 0, value_color_data, 8);
            rt_device_write(src_uart6, 0, unit_color_data, 8);
        }
    }
}

void om_history_data_save(uint32_t _fid_value,uint8_t unit)
{
    if(fid_save_data.save_counts < SAVE_VALUE_COUNT)
    {
        fid_save_data.fid_value[fid_save_data.save_counts] = _fid_value;
        fid_save_data.value_unit[fid_save_data.save_counts] = unit;
        fid_save_data.save_counts++;
    }
    else {
        rt_memmove(fid_save_data.fid_value,&fid_save_data.fid_value[1],SAVE_VALUE_COUNT - 1);
        rt_memmove(fid_save_data.value_unit,&fid_save_data.value_unit[1],SAVE_VALUE_COUNT - 1);
        fid_save_data.fid_value[SAVE_VALUE_COUNT - 1] = _fid_value;
        fid_save_data.value_unit[SAVE_VALUE_COUNT - 1] = unit;
    }
}

/*
 *  PID  5a a5 07 82 10 80 50 49 44 00
 *  O2   5a a5 07 82 10 80 4f 32 00 00
 *  CO   5a a5 07 82 10 80 43 4f 00 00
 *  LEL  5a a5 07 82 10 80 4c 45 4c 00
 *  H2S  5a a5 07 82 10 80 48 32 53 00
 * */
void om_multi_sensor_set(void)
{
    uint8_t mutli_sensor_data[10] =  {0x5a,0xa5,0x07,0x82,0x10,0x80,0x08,0x70,0x70,0x00};

    if(m_device_set_data.sensor_select == 1)
    {
        mutli_sensor_data[6] = 0x50;
        mutli_sensor_data[7] = 0x49;
        mutli_sensor_data[8] = 0x44;
    }
    else if(m_device_set_data.sensor_select == 2)
    {
        mutli_sensor_data[6] = 0x4f;
        mutli_sensor_data[7] = 0x32;
        mutli_sensor_data[8] = 0x00;
    }
    else if(m_device_set_data.sensor_select == 3)
    {
        mutli_sensor_data[6] = 0x43;
        mutli_sensor_data[7] = 0x4f;
        mutli_sensor_data[8] = 0x00;
    }
    else if(m_device_set_data.sensor_select == 4)
    {
        mutli_sensor_data[6] = 0x4c;
        mutli_sensor_data[7] = 0x45;
        mutli_sensor_data[8] = 0x4c;
    }
    else if(m_device_set_data.sensor_select == 5)
    {
        mutli_sensor_data[6] = 0x48;
        mutli_sensor_data[7] = 0x32;
        mutli_sensor_data[8] = 0x53;
    }

    rt_device_write(src_uart6, 0, mutli_sensor_data, 10);
}

void om_set_device_config(void)
{
    uint8_t set_data[8] =  {0x5a,0xa5,0x05,0x82,0x10,0x00,0x00,0x00};

    if(m_device_set_data.unit_select == 0)
    {
        set_data[5] = 0x00;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x04;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x08;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        rt_device_write(src_uart6, 0, ppm_select_data, 15);
    }
    else if(m_device_set_data.unit_select == 1)
    {
        set_data[5] = 0x00;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x04;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x08;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        rt_device_write(src_uart6, 0, mol_select_data, 15);
    }
    else if(m_device_set_data.unit_select == 2)
    {
        set_data[5] = 0x00;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x04;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x08;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
        rt_device_write(src_uart6, 0, mg_select_data, 15);

    }

    if(m_device_set_data.data_save_type == 0)
    {
        set_data[5] = 0x0C;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x0F;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
    }
    else if(m_device_set_data.data_save_type == 1)
    {
        set_data[5] = 0x0C;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x0F;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
    }

    if(m_device_set_data.sensor_select == 0)
    {
        set_data[5] = 0x14;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x18;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x1C;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x1F;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x24;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x28;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
    }
    else if(m_device_set_data.sensor_select == 1)
    {
        set_data[5] = 0x14;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x18;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x1C;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x1F;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x24;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x28;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
    }
    else if(m_device_set_data.sensor_select == 2)
    {
        set_data[5] = 0x14;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x18;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x1C;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x1F;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x24;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x28;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
    }
    else if(m_device_set_data.sensor_select == 3)
    {
        set_data[5] = 0x14;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x18;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x1C;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x1F;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x24;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x28;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
    }
    else if(m_device_set_data.sensor_select == 4)
    {
        set_data[5] = 0x14;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x18;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x1C;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x1F;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x24;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x28;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
    }
    else if(m_device_set_data.sensor_select == 5)
    {
        set_data[5] = 0x14;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x18;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x1C;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x1F;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x24;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x28;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
    }

    if(m_device_set_data.bg_value_type == 0)
    {
        set_data[5] = 0x2C;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x2F;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
    }
    else if(m_device_set_data.bg_value_type == 1)
    {
        set_data[5] = 0x2C;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x2F;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
    }

    if(m_device_set_data.bt_switch == 0)
    {
        set_data[5] = 0x34;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x38;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
    }
    else if(m_device_set_data.bt_switch == 1)
    {
        set_data[5] = 0x34;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x38;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
    }

    if(m_device_set_data.sweep_switch == 0)
    {
        set_data[5] = 0x3C;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x3F;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
    }
    else if(m_device_set_data.sweep_switch == 1)
    {
        set_data[5] = 0x3C;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x3F;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
    }

    if(m_device_set_data.accuracy_type == 0)
    {
        set_data[5] = 0x44;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x48;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
    }
    else if(m_device_set_data.accuracy_type == 1)
    {
        set_data[5] = 0x44;
        set_data[7] = 0x01;
        rt_device_write(src_uart6, 0, set_data, 8);
        set_data[5] = 0x48;
        set_data[7] = 0x00;
        rt_device_write(src_uart6, 0, set_data, 8);
    }

}


#define UART_SRC_NAME                 "uart6"
int om_src_uart_init(void)
{
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* åˆå§‹åŒ–é…ç½®å‚æ•° */

    rt_strncpy(uart_name, UART_SRC_NAME, RT_NAME_MAX);


    src_uart6 = rt_device_find(uart_name);
    if (!src_uart6)
    {
        LOG_D("find %s failed!\n", uart_name);
        return RT_ERROR;
    }

    LOG_I("find %s success!\n", uart_name);

    config.baud_rate = BAUD_RATE_115200;
    config.data_bits = DATA_BITS_8;
    config.stop_bits = STOP_BITS_1;
    config.bufsz     = 512;
    config.parity    = PARITY_NONE;

    rt_device_control(src_uart6, RT_DEVICE_CTRL_CONFIG, &config);
    rt_device_open(src_uart6, RT_DEVICE_FLAG_INT_RX);

    return ret;
}

static void next_key_push(void)
{
    if(g_skip_mode == SKIP_MODE_NONE)
    {
        g_skip_mode = SKIP_MODE_NEXT;  // dvalid pin is high,could read data
    }
    LOG_I("g_skip_mode = SKIP_MODE_NEXT\r\n");
}
static void previou_key_push(void)
{
    if(g_skip_mode == SKIP_MODE_NONE)
    {
        g_skip_mode = SKIP_MODE_PREVIOU;  // dvalid pin is high,could read data
    }
    LOG_I("g_skip_mode = SKIP_MODE_PREVIOU\r\n");
}
static void cancel_key_push(void)
{
    if(g_skip_mode == SKIP_MODE_NONE)
    {
        g_skip_mode = SKIP_MODE_CANCEL;  // dvalid pin is high,could read data
    }
    LOG_I("g_skip_mode = SKIP_MODE_CANCEL\r\n");
}
static void comfirm_key_push(void)
{
    if(g_skip_mode == SKIP_MODE_NONE)
    {
        g_skip_mode = SKIP_MODE_CONFIRM;  // dvalid pin is high,could read data
    }
    LOG_I("g_skip_mode = SKIP_MODE_COMFIRE\r\n");
}
void om_key_init(void)
{
    rt_pin_mode(NEXT_KEY, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(NEXT_KEY, PIN_IRQ_MODE_FALLING, (void *)previou_key_push, RT_NULL);
    rt_pin_irq_enable(NEXT_KEY, PIN_IRQ_ENABLE);
    rt_pin_mode(PREVIOU_KEY, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(PREVIOU_KEY, PIN_IRQ_MODE_FALLING, (void *)next_key_push, RT_NULL);
    rt_pin_irq_enable(PREVIOU_KEY, PIN_IRQ_ENABLE);
    rt_pin_mode(CANCEL_KEY, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(CANCEL_KEY, PIN_IRQ_MODE_FALLING, (void *)cancel_key_push, RT_NULL);
    rt_pin_irq_enable(CANCEL_KEY, PIN_IRQ_ENABLE);
    rt_pin_mode(COMFIRM_KEY, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(COMFIRM_KEY, PIN_IRQ_MODE_FALLING, (void *)comfirm_key_push, RT_NULL);
    rt_pin_irq_enable(COMFIRM_KEY, PIN_IRQ_ENABLE);
    rt_pin_write(NEXT_KEY, PIN_LOW);
    rt_pin_write(PREVIOU_KEY, PIN_LOW);
    rt_pin_write(CANCEL_KEY, PIN_LOW);
    rt_pin_write(COMFIRM_KEY, PIN_LOW);

}
