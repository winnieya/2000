/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-23     Tery       the first version
 */
#include <rtdevice.h>

#include "om_1_ddc112.h"
#include "om_1_data.h"
#include "om_1_sensor.h"
#include "om_1_power.h"
#include "om_1_session.h"
#include "calibration.h"

#define DBG_TAG "om_1_data"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define OM_IGNITE_CMD   0x20
#define OM_RANGE_CMD   0x04
#define OM_INTEGRATION_CMD   0x0C
#define OM_PID_ONOFF_CMD   0x36
#define OM_READ_CMD   0x25
#define OM_FID_CALIBRATION_CMD   0x31
#define OM_PID_CALIBRATION_CMD   0x32
#define OM_GET_CALIBRATION_CMD   0x23
#define OM_SET_CALIBRATION_CMD   0x21

rt_uint8_t om_1_bt_calccrc(rt_uint8_t* buf, rt_uint8_t paramInt);


/*指示点火线圈是否需要执行点火动作
 * RT_FALSE    表示点火线圈不需要工作
 * RT_TRUE     表示点火线圈需要工作
 * */
static rt_bool_t g_om_need_ignite = RT_FALSE; /*指示点火线圈是否需要执行点火动作*/
static uint8_t g_pid_onoff = 0;
/* 蓝牙上报数据结构体，包含所有需要上报的字段 */

void om_byte_order_conversion_uint32(rt_uint8_t* conv_buf,rt_uint32_t conv_value)
{
    conv_buf[0] = (rt_uint8_t)conv_value;
    conv_buf[1] = (rt_uint8_t)(conv_value >> 8);
    conv_buf[2] = (rt_uint8_t)(conv_value >> 16);
    conv_buf[3] = (rt_uint8_t)(conv_value >> 24);
}

void om_byte_order_conversion_uint16(rt_uint8_t* conv_buf,rt_uint16_t conv_value)
{
    conv_buf[0] = (rt_uint8_t)conv_value;
    conv_buf[1] = (rt_uint8_t)(conv_value >> 8);
}

uint16_t get_troubleshooting_tips(void)
{
    return 0;
}

uint8_t om_get_pid_state(void)
{
    return g_pid_onoff;
}
/*
 * 组包函数
 * 用于向APP端发送数据
 * */
uint8_t zero_flag = 0;
void om_1_assemble_packet(void)
{
    uint16_t temp = 0;
//    uint8_t data_len = sizeof(struct om_1_report_data);
    struct om_1_report_data report_data = {0};//(struct om_1_report_data)om_report_collection_data;
    struct om_1_report_data_21 report_data_21 = {0};//(struct om_1_report_data)om_report_collection_data;
    int32_t message_value = 0;
    uint32_t fid_curve_value = 0;

    if(om_get_read_type())
    {
        report_data_21.cmd_type = 0xA5;
        report_data_21.data_len = 0x28;
        report_data_21.cmd_id = 0x25;
        report_data_21.dev_state = 1 | om_get_solenoidB_state() << 2;
        om_byte_order_conversion_uint16(report_data_21.thermo_couple,(rt_uint16_t)om_thermocouple_read());
        om_byte_order_conversion_uint16(report_data_21.battery_voltage,(rt_uint16_t)om_battery_level_read());
        om_byte_order_conversion_uint16(report_data_21.chamber_outer_temp,(rt_uint16_t)om_temperature_sensor_read());
        om_byte_order_conversion_uint16(report_data_21.sample_pressure,0);
        om_byte_order_conversion_uint16(report_data_21.air_pressure,0);
        om_byte_order_conversion_uint16(report_data_21.tank_pressure,0);
        report_data_21.fid_range = om_get_ddc112_range_value();
        message_value = om_get_ddc112_value();
        if (message_value > 65000) {
            report_data_21.fid_range = 3;
        }
        else if (message_value < 60000) {
            report_data_21.fid_range = 0;
        }

        om_byte_order_conversion_uint32(report_data_21.pico_amps,message_value);

        if(om_get_ignite_step() == OM_DETECTING)
        {
            fid_curve_value = get_fid_calbarate_value()/10;
            if( fid_curve_value < 25)
            {
                fid_curve_value = fid_curve_value / 5;
            }
            om_byte_order_conversion_uint32(report_data_21.fid_raw_ppm,fid_curve_value);
        }
        else {
            om_byte_order_conversion_uint32(report_data_21.fid_raw_ppm,0);
        }
        om_byte_order_conversion_uint16(report_data_21.system_current,(rt_uint16_t)om_system_current_read());//(rt_uint16_t)om_somtem_current_read());
        report_data_21.pump_power = 100 - (100*(om_get_pump_power_adjust()))/4096;

        /*crc*/
        report_data_21.checksum = om_1_bt_calccrc((rt_uint8_t *)&report_data_21,sizeof(struct om_1_report_data_21) - 1);

        om_send_data((rt_uint8_t *)&report_data_21, sizeof(struct om_1_report_data_21));
    }
    else {
        report_data.cmd_type = 0xAA;
        report_data.data_len = 0x2E;
        report_data.cmd_id = 0x25;
        report_data.dev_state = 1 | om_get_solenoidB_state() << 2;
        om_byte_order_conversion_uint16(report_data.thermo_couple,(rt_uint16_t)om_thermocouple_read());
        om_byte_order_conversion_uint16(report_data.battery_voltage,(rt_uint16_t)om_battery_level_read());
        om_byte_order_conversion_uint16(report_data.chamber_outer_temp,(rt_uint16_t)om_temperature_sensor_read());
        om_byte_order_conversion_uint16(report_data.sample_pressure,0);
        om_byte_order_conversion_uint16(report_data.air_pressure,0);
        om_byte_order_conversion_uint16(report_data.tank_pressure,0);
        report_data.fid_range = om_get_ddc112_range_value();
        message_value = om_get_ddc112_value();

        if(get_cal_flag())
        {
            message_value = message_value * 97 / 100;
        }

        om_byte_order_conversion_uint32(report_data.pico_amps,message_value);

        report_data.fid_calibrate_is_or_not = get_detect_flag(FID_CURVE);
        report_data.reserve0 = get_detect_flag(FID_CURVE);
        if(om_get_ignite_step() == OM_DETECTING)
        {
            fid_curve_value = get_detect_value(FID_CURVE,om_get_ddc112_value())/10;
            if( fid_curve_value < 25)
            {
                fid_curve_value = fid_curve_value / 5;
            }
            om_byte_order_conversion_uint32(report_data.fid_raw_ppm,fid_curve_value);
        }
        else {
            om_byte_order_conversion_uint32(report_data.fid_raw_ppm,0);
        }

        report_data.pid_calibrate_is_or_not = get_detect_flag(PID_CURVE);
        om_byte_order_conversion_uint32(report_data.pid_raw_ppm,get_detect_value(PID_CURVE,om_get_pid_value()));

        om_byte_order_conversion_uint16(report_data.system_current,(rt_uint16_t)om_system_current_read());//(rt_uint16_t)om_somtem_current_read());
        report_data.pump_power = 100 - (100*(om_get_pump_power_adjust()))/4096;
        temp = om_get_troubleshooting_tips();
        if(temp)
        {
            LOG_I("Fault detected %d\r\n",temp);
        }

        om_byte_order_conversion_uint16(report_data.troubleshooting_tips,temp);

        om_byte_order_conversion_uint32(report_data.pid_value,om_get_pid_value());


        /*crc*/
        report_data.checksum = om_1_bt_calccrc((rt_uint8_t *)&report_data,sizeof(struct om_1_report_data) - 1);

        om_send_data((rt_uint8_t *)&report_data, sizeof(struct om_1_report_data));
    }

    LOG_D("om_report_collection_data=");
}

void cal_process(uint8_t curve_type,uint16_t con_value)
{
    if(curve_type == FID_CURVE)
    {
        if(con_value == 0)
        {
            zero_flag = 1;
            curve_reset(FID_CURVE);
        }
        else {
            zero_flag = 0;
        }
        curve_updata(FID_CURVE,con_value,om_get_ddc112_value());
    }
    else {
        if(con_value == 0)
        {
            curve_reset(PID_CURVE);
        }
        curve_updata(PID_CURVE,con_value,om_get_pid_value());
    }
}
static uint8_t g_set_cal_time_delay_flag = 0;
void set_cal_flag(void)
{
    g_set_cal_time_delay_flag = 1;
}

void reset_cal_flag(void)
{
    g_set_cal_time_delay_flag = 0;
}

uint8_t get_cal_flag(void)
{
    return g_set_cal_time_delay_flag;
}

/*蓝牙命令处理函数*/
void om_bt_data_handle(rt_uint8_t* bt_data,rt_uint8_t data_len)
{
    rt_uint8_t cmdid = 0;
    struct om_1_ignite_data* ignite_data = RT_NULL;
    uint16_t ajust_value = 0;
    struct om_1_report_cal_21 report_data_21 = {0};

    /*get cmd id*/
    cmdid = bt_data[2];

    if(bt_data[data_len - 1] !=  om_1_bt_calccrc(bt_data,data_len - 1))
    {
        LOG_I("data=");
        for(int i=0;i<data_len;i++)
        {
            LOG_I("%x",bt_data[i]);
        }
        LOG_I("\r\n");
        LOG_E("crc check fail! \r\n");
        LOG_E("CRC %d",om_1_bt_calccrc(bt_data,data_len - 1));
        return;
    }
    switch(cmdid)
    {
        case OM_IGNITE_CMD:
            ignite_data  = (struct om_1_ignite_data*)bt_data;
            LOG_D("is ignite cmd %d! \r\n",ignite_data->ignite);
            if(ignite_data->ignite == 1)  /*ignite or close fire*/
            {
                LOG_D("open ignite 2 cmd! \r\n");
                om_ignite_set();
            }
            else
            {
                LOG_D("close fire cmd! \r\n");
                om_ignite_reset();
            }
            break;
        case OM_RANGE_CMD:
            LOG_D("is set range cmd! \r\n");
            break;
        case OM_INTEGRATION_CMD:
            LOG_D("is set integration cmd! \r\n");
            break;
        case OM_PID_ONOFF_CMD:
            LOG_I("is set OM_PID_ONOFF_CMD cmd! \r\n");
            set_pid_onoff(bt_data[3]);
            g_pid_onoff = bt_data[3];
            break;

        case OM_READ_CMD:
//            om_1_assemble_packet();
            LOG_D("is read cmd! \r\n");
            break;
        case OM_PID_CALIBRATION_CMD:
            ajust_value = (uint16_t)(bt_data[3] + (bt_data[4] << 8));
            if(ajust_value == 0)
            {
                zero_flag = 1;
                curve_reset(PID_CURVE);
            }
            else {
                zero_flag = 0;
            }

            LOG_I("is PID CALIBRATION  %d \r\n",ajust_value);
            LOG_I("is PID CALIBRATION ddc112 %d \r\n",om_get_pid_value());
            curve_updata(PID_CURVE,ajust_value,om_get_pid_value());
            break;
        case OM_FID_CALIBRATION_CMD:
            ajust_value = (uint16_t)(bt_data[3] + (bt_data[4] << 8));
            if(ajust_value == 0)
            {
                curve_reset(FID_CURVE);
            }
            else {
//                set_cal_flag();
            }

            LOG_I("is FID CALIBRATION ddc112 %d \r\n",om_get_ddc112_value());
            curve_updata(FID_CURVE,ajust_value,om_get_ddc112_value());
            break;
        case OM_SET_CALIBRATION_CMD:
            ajust_value = ((bt_data[4] + (bt_data[5] << 8)+ (bt_data[6] << 16)+ (bt_data[7] << 24)))/10;
            if(ajust_value < 1)
            {
                curve_reset(FID_CURVE);
                zero_flag = 1;
            }
            else {
                 zero_flag = 0;
            }
            curve_updata(FID_CURVE,ajust_value,om_get_ddc112_value());
            LOG_I("is OM_SET_CALIBRATION_CMD cmd! %d \r\n",ajust_value);
            break;
        case OM_GET_CALIBRATION_CMD:
            report_data_21.cmd_type = 0xA5;
            report_data_21.data_len = 0x2C;
            report_data_21.cmd_id = 0x23;
            report_data_21.reserve3[3] = bt_data[3];
            om_byte_order_conversion_uint32(report_data_21.fid_raw_ppm,get_curve_value(bt_data[3])*10);
            om_byte_order_conversion_uint32(report_data_21.reserve4,om_get_ddc112_value());
            om_byte_order_conversion_uint16(report_data_21.reserve5,0);
            report_data_21.reserve6 = get_detect_flag(FID_CURVE);

            /*crc*/
            report_data_21.checksum = om_1_bt_calccrc((rt_uint8_t *)&report_data_21,sizeof(struct om_1_report_data_21) - 1);

            om_send_data((rt_uint8_t *)&report_data_21, sizeof(struct om_1_report_data_21));
            LOG_I("is OM_GET_CALIBRATION_CMD cmd! \r\n");
            break;
        default:
            LOG_D("is error cmd! \r\n");
            break;
    }
}


/*指示点火线圈是否需要执行点火动作*/
rt_bool_t om_is_need_ignite(void)
{
    return g_om_need_ignite ;
}

/*表示点火线圈不需要执行点火动作*/
void om_ignite_reset(void)
{
    g_om_need_ignite = RT_FALSE;
}

/*表示点火线圈需要执行点火动作*/
void om_ignite_set(void)
{
    g_om_need_ignite = RT_TRUE;
}
/*CRC校验，用于APP端数据检查*/
rt_uint8_t om_1_bt_calccrc(rt_uint8_t* buf, rt_uint8_t paramInt)
{
    rt_uint8_t i = 213;   /* 手机端配置值为213*/
    for (rt_uint8_t b = 0; b < paramInt; b++)
    {
        rt_uint8_t b1 = buf[b];
        if (b1 < 0)
        b1 += 256;
        i = ((i << 1 | i >> 7) + b1) % 256;
    }
    return i;
}










