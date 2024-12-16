/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-08     RT-Thread    first version
 */

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "om_1_ddc112.h"
#include "om_1_sensor.h"
#include "om_1_power.h"
#include "om_1_session.h"
#include "om_1_ble.h"
#include "om_1_data.h"
#include "calibration.h"
#include "src_controller.h"

#define OM_BT_DATA_REPORT_INTERVAL_TIME  300 /*ms*/
#define OM_BT_DATA_DELAY_INTERVAL_TIME  10 /*ms*/

#define DC5V_POWER    GET_PIN(E, 3)

static void om_init(void);

int main(void)
{
    uint8_t count = 0;
    om_init();

    om_start_monitor();
    om_src_monitor();
    om_start_ddc112_read();

    while (1)
    {
        ppm_calbarate();
        om_bt_uarts_data_parsing();

        rt_thread_mdelay(OM_BT_DATA_DELAY_INTERVAL_TIME*3);
        LOG_D("bt data handle thread is running\n");
        if(count == 15)
        {
            if(!get_esp32_state())
            {
                om_1_assemble_packet();
            }
            count = 0;
        }
        count++;
    }
    return RT_EOK;
}

static void om_init(void)
{

    rt_pin_mode(DC5V_POWER, PIN_MODE_OUTPUT);
    rt_pin_write(DC5V_POWER, PIN_HIGH);

    /*屏幕供电*/
    rt_pin_mode(GET_PIN(E, 4), PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(E, 4), PIN_HIGH);

    /*PID 供电,默认关闭PID*/
    rt_pin_mode(GET_PIN(E, 0), PIN_MODE_OUTPUT);
    rt_pin_write(GET_PIN(E, 0), PIN_HIGH);

    om_1_ignite_init();
    om_ddc112_init(); /*ddc112*/

    om_adc_vol_init(); /*adc初始化*/

    om_ble_uarts_init(); /*蓝牙模块*/

}
