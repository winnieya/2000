/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-14     Tery       the first version
 */
#include <board.h>
#include <rtdevice.h>

#define DBG_TAG "om_1_power"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "om_1_power.h"
#include "om_1_sensor.h"
#include "om_1_data.h"
#include "om_1_ddc112.h"


#define PUMP_PIN    GET_PIN(E, 11)
#define IGNITE_PIN2    GET_PIN(B, 7)

#define OM_EXCLUDE_PRESURE_STANDARD     175  /* psi * 100 */

#define OM_BT_MONITOR_DELAY_TIME  100 /*ms*/

#define OM_PUMP_AJUST_DELAY_TIME  250 /*ms*/

#define OM_IGNITE_TIME  3000 /*ms*/
#define OM_PUMP_AJUST1_TIME  5000 /*ms,用于点火前*/
#define OM_PUMP_AJUST2_TIME  1000 /*ms用于点火后实时调整泵功率，使废气压力传感器压力为1.75psi左右*/
#define OM_PUMP_CLOSE_DELAY_TIME  5000 /*ms,用于关火后，持续尾吹气*/
#define OM_SOL2_OPEN_DELAY_TIME  1000 /*ms*/

#define OM_PUMP_POWER_MAX           3600
#define OM_PUMP_POWER_STABILIZATION  1950
#define OM_PUMP_AJUST_ERROR          5
#define OM_PUMP_AJUDT_START          850
#define OM_PUMP_AJUST_STEP           10
#define OM_PUMP_POWER_OFF            1
#define OM_PUMP_POWER_NORMAL         2500
#define OM_PUMP_POWER_MIN           750

#define OM_PUMP_POWER_INCREASE_AMPLITUDE  5
#define OM_PUMP_POWER_REDUCE_AMPLITUDE  5

static rt_uint16_t dac_vol = 0;
static rt_tick_t g_om_ignite_tick = 0;
static rt_tick_t g_om_pump_close_delay_tick = 0;
static rt_tick_t g_om_wait_tick = 0;
static rt_tick_t g_om_sol2_close_tick = 0;
static rt_thread_t om_monitor_ctr_thread = RT_NULL;

static rt_uint8_t  om_detection_step = 0;
static uint8_t pump_flag = 0;

static void om_monitor_ctr(void *parameter);

/*----------------monitor----------------
 * 气路、点火进程控制
 */
void om_start_monitor(void)
{
    om_monitor_ctr_thread = rt_thread_create("om_monitor_ctr_thread", om_monitor_ctr, RT_NULL, 548, 11, 10);
    if (om_monitor_ctr_thread != RT_NULL)
    {
        rt_thread_startup(om_monitor_ctr_thread);
        LOG_D("om_monitor_ctr_thread is running\n");
    }
}

void om_set_pin(rt_base_t pin,rt_bool_t value)
{
    if(rt_pin_read(pin) != value)
    {
        rt_pin_write(pin,value);
    }
}
uint8_t om_get_ignite_step(void)
{
    return om_detection_step;
}

void om_monitor_ctr(void *parameter)
{
    rt_tick_t pump_ajust_tick = 0;

    while(1)
    {
        if(om_is_need_ignite())
        {
            switch (om_detection_step) {
            case OM_NONE_STEP:
                om_set_pump_power(OM_PUMP_POWER_NORMAL);
                om_set_pump_power(OM_PUMP_POWER_MIN);
                g_om_wait_tick = rt_tick_get();
                om_detection_step = OM_SWEEPING;
                break;
            case OM_SWEEPING:  /*点火前吹扫*/
                if(om_is_timeout(g_om_wait_tick,OM_PUMP_AJUST1_TIME))
                {
                    om_set_pump_power(10);
                    g_om_wait_tick = rt_tick_get();
                    om_detection_step = OM_PUMP_START;
                }
                break;
            case OM_PUMP_START:
                if(om_is_timeout(g_om_wait_tick,OM_PUMP_AJUST1_TIME))
                {
                    om_set_pump_power(OM_PUMP_POWER_MIN);
                    om_1_ignite_start();
                    g_om_ignite_tick = rt_tick_get();
                    om_detection_step = OM_IGNITE_STEP_PRE;
                }
                break;
            case OM_IGNITE_STEP_PRE:
                if(om_is_timeout(g_om_ignite_tick,2*OM_SOL2_OPEN_DELAY_TIME))
                {
                    om_set_pump_power(OM_PUMP_POWER_MIN);
                    om_detection_step = OM_IGNITE_STEP;
                    g_om_ignite_tick = rt_tick_get();
                }
                break;
            case OM_IGNITE_STEP:
                if(om_is_timeout(g_om_ignite_tick,OM_IGNITE_TIME + 2*OM_SOL2_OPEN_DELAY_TIME))
                {
                    om_1_ignite_end();
                    om_detection_step = OM_IGNITE_END;
                    g_om_sol2_close_tick = rt_tick_get();
                }
                break;
                case OM_IGNITE_END:
                    for(uint8_t m =0 ;m < 61;m++)
                    {
                        om_set_pump_power(OM_PUMP_POWER_MIN + m * 29);
                        rt_thread_mdelay(250);
                    }
                    om_detection_step = OM_DETECTING;
                    pump_ajust_tick = rt_tick_get();
                    break;

                case OM_DETECTING:
                    if(!pump_flag)
                    {
                        if(om_is_timeout(pump_ajust_tick,OM_PUMP_AJUST2_TIME))
                        {
                            pump_ajust_tick = rt_tick_get();

                            pump_flag = 1;
                        }
                    }
                    g_om_pump_close_delay_tick = rt_tick_get();
                    break;
                default:
                    break;
            }
        }
        else
        {
            /*关火后吹扫*/
//            if(om_is_timeout(g_om_pump_close_delay_tick,3*OM_PUMP_CLOSE_DELAY_TIME))
//            {
//                om_set_pump_power(OM_PUMP_POWER_NORMAL+700);
//            }
            om_set_pump_power(OM_PUMP_POWER_OFF);
            rt_pin_write(IGNITE_PIN2,PIN_LOW);

            om_detection_step  = OM_NONE_STEP;
            pump_flag = 0;
       }
        rt_thread_mdelay(OM_BT_MONITOR_DELAY_TIME);
    }
}

/*----------------pump----------------
 * 泵控制
 */
rt_bool_t om_get_pump_state(void)
{
    return 0;//todo //rt_pin_read(PUMP_PIN);
}

void om_set_pump_power(rt_uint16_t power)
{
    if(power ==0){
        return;
    }
    dac_vol = power;
    om_dac_set_dac_value(dac_vol);
}

rt_uint16_t om_get_pump_power_adjust(void)
{
    if( dac_vol == 0)
    {
        dac_vol = OM_PUMP_POWER_STABILIZATION;
    }
    return dac_vol;
}

void set_pid_onoff(uint8_t on_off)
{
    rt_pin_write(GET_PIN(E,0), on_off);
}
/*----------------ignite----------------
 * 点火控制
 * 点火分为两部分：点火线圈硬件电路上电容充电、电容放电
 * om_1_ignite_start是拉高GPIO给电容充电
 * 充电时间目前测试是2000ms
 * om_1_ignite_end是拉低GPIO，让电容放电
 */
void om_1_ignite_init(void)
{
    rt_pin_mode(IGNITE_PIN2, PIN_MODE_OUTPUT);
    rt_pin_write(IGNITE_PIN2,PIN_LOW);
    om_set_pump_power(OM_PUMP_POWER_OFF);
}

void om_1_ignite_start(void)
{
    om_set_pin(IGNITE_PIN2,OM_ON);

    LOG_I("ignite start \n");
}

void om_1_ignite_end(void)
{
    om_set_pin(IGNITE_PIN2,OM_OFF);
    LOG_I("ignite end \n");
}


rt_bool_t om_get_solenoidB_state(void)
{
    return 1;//rt_pin_read(SOLENOID_PIN1);
}

/*软件滴答定时器
 *分为三种情况：
 *1、计时tick + 参考tick，不会溢出，检查tick也没有溢出（判断tick差值就可完成计时）
 *2、计时tick + 参考tick，不会溢出，检查tick有溢出（此时计时已经完成）
 *3、计时tick + 参考tick，会溢出，检查tick也需要溢出（根据两个溢出后的值大小，即可完成计时判断）
 * */
rt_bool_t om_is_timeout(rt_tick_t ref_tick,rt_uint16_t timing_time)
{
    if(ref_tick + rt_tick_from_millisecond(timing_time) < 0xFFFFFFFF)   /* tick 反转前可以计时完成*/
    {
        if(rt_tick_get() > (ref_tick + rt_tick_from_millisecond(timing_time)))
        {
            return RT_TRUE;
        }
        else if(rt_tick_get() < ref_tick) /*延时导致tick反转，实际已经超过定时时间*/
        {
            return RT_TRUE;
        }
    }
    else if((rt_tick_get() < ref_tick) && (rt_tick_get() > (ref_tick + rt_tick_from_millisecond(timing_time))))
    {
        return RT_TRUE;
    }
    return RT_FALSE;
}



