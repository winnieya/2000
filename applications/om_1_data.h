/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-23     Tery       the first version
 */
#ifndef OM_1_DATA_H_
#define OM_1_DATA_H_

#include <rtthread.h>

struct om_1_report_data
{
    rt_uint8_t cmd_type;
    rt_uint8_t data_len;
    rt_uint8_t cmd_id;
    rt_uint8_t reserve0;
    rt_uint8_t dev_state;
    rt_uint8_t fid_calibrate_is_or_not;
    rt_uint8_t pid_calibrate_is_or_not;
    rt_uint8_t thermo_couple[2];
    rt_uint8_t battery_voltage[2];
    rt_uint8_t chamber_outer_temp[2];
    rt_uint8_t sample_pressure[2];
    rt_uint8_t air_pressure[2];
    rt_uint8_t tank_pressure[2];
    rt_uint8_t fid_range;
    rt_uint8_t reserve2[4];
    rt_uint8_t pico_amps[4];
    rt_uint8_t reserve3[4];
    rt_uint8_t fid_raw_ppm[4];
    rt_uint8_t system_current[2];
    rt_uint8_t pump_power;
    rt_uint8_t pid_value[4];
    rt_uint8_t pid_raw_ppm[2];
    rt_uint8_t troubleshooting_tips[2];
    rt_uint8_t checksum;
};

struct om_1_report_data_21
{
    rt_uint8_t cmd_type;
    rt_uint8_t data_len;
    rt_uint8_t cmd_id;
    rt_uint8_t reserve0;
    rt_uint8_t dev_state;
    rt_uint8_t reserve1[2];
    rt_uint8_t thermo_couple[2];
    rt_uint8_t battery_voltage[2];
    rt_uint8_t chamber_outer_temp[2];
    rt_uint8_t sample_pressure[2];
    rt_uint8_t air_pressure[2];
    rt_uint8_t tank_pressure[2];
    rt_uint8_t fid_range;
    rt_uint8_t reserve2[4];
    rt_uint8_t pico_amps[4];
    rt_uint8_t reserve3[4];
    rt_uint8_t fid_raw_ppm[4];
    rt_uint8_t system_current[2];
    rt_uint8_t pump_power;
    rt_uint8_t checksum;
};

struct om_1_report_cal_21
{
    rt_uint8_t cmd_type;
    rt_uint8_t data_len;
    rt_uint8_t cmd_id;
    rt_uint8_t reserve0;
    rt_uint8_t dev_state;
    rt_uint8_t reserve1[2];
    rt_uint8_t thermo_couple[2];
    rt_uint8_t battery_voltage[2];
    rt_uint8_t chamber_outer_temp[2];
    rt_uint8_t sample_pressure[2];
    rt_uint8_t air_pressure[2];
    rt_uint8_t tank_pressure[2];
    rt_uint8_t fid_range;
    rt_uint8_t reserve2[4];
    rt_uint8_t pico_amps[4];
    rt_uint8_t reserve3[4];
    rt_uint8_t fid_raw_ppm[4];
    rt_uint8_t reserve4[4];
    rt_uint8_t reserve5[2];
    rt_uint8_t reserve6;
    rt_uint8_t checksum;
};

/* 蓝牙下发点火、关火数据结构体 */
struct om_1_ignite_data
{
    rt_uint8_t cmd_type;
    rt_uint8_t data_len;
    rt_uint8_t cmd_id;
    rt_uint8_t ignite;
    rt_uint8_t reserve0[20];
    rt_uint8_t ignite_type;
    rt_uint8_t reserve1;;
    rt_uint8_t checksum;
};

struct om_1_control_cmd_data
{
    rt_uint8_t cmd_type;
    rt_uint8_t data_len;
    rt_uint8_t cmd_id;
    rt_uint8_t reserve[50];
};

/* 蓝牙下发配置量程数据结构体 */
struct om_1_set_range_data
{
    rt_uint8_t cmd_type;
    rt_uint8_t data_len;
    rt_uint8_t cmd_id;
    rt_uint8_t range;
    rt_uint8_t reserve;
    rt_uint8_t checksum;
};

/* 蓝牙下发配置量程数据结构体 */
struct om_1_set_avg_data
{
    rt_uint8_t cmd_type;
    rt_uint8_t data_len;
    rt_uint8_t cmd_id;
    rt_uint8_t reserve0;
    rt_uint32_t reserve1;
    rt_uint8_t avg;
    rt_uint16_t reserve2;
    rt_uint8_t checksum;
};
/*
 * report test data protocol format
 * 0       固定（发0x5a,收0xa5）
 * 1       Len（发，收待确认）
 * 2       0x25 cmdid
 * 3       未知
 * 4       IsPumpAOn  [0]
 *         IsSolenoidAOn   [2]
 *         IsSolenoidBOn   [3]
 * 5~6     保留
 * 7~8     ThermoCouple，2个字节，ConvertKelvinToFahrenheit()*0.1f
 * 9~10    BatteryVoltage，2个字节，*0.001f
 * 11~12   ChamberOuterTemp，2个字节，ConvertKelvinToFahrenheit()*0.1f
 * 13~14   SamplePressure，2个字节，*0.01f
 * 15~16   AirPressure，2个字节，*0.01f
 * 17~18   TankPressure，2个字节
 * 19      FIDRange，1个字节
 * 20~23   保留
 * 24~27   PicoAmps，4个字节，*0.1f
 * 28~31   保留
 * 32~35   RawPpm，4个字节，*0.1f
 * 36~37   SystemCurrent，2个字节
 * 38      PumpPower，1个字节
 * 39      CRC校验码
 * */

void om_bt_data_handle(rt_uint8_t* bt_data,rt_uint8_t data_len);

void om_1_assemble_packet(void);
rt_uint8_t om_get_ignite_type_and_state(void);
rt_bool_t om_is_need_ignite(void);
void om_ignite_reset(void);
void om_ignite_set(void);
void cal_process(uint8_t curve_type,uint16_t con_value);
void rs485_recv_data_parsing(uint8_t* rs485_data,uint16_t data_len);
rt_uint8_t om_1_bt_calccrc(rt_uint8_t* buf, rt_uint8_t paramInt);
uint8_t om_get_pid_state(void);

void set_cal_flag(void);
void reset_cal_flag(void);
uint8_t get_cal_flag(void);

void om_byte_order_conversion_uint32(rt_uint8_t* conv_buf,rt_uint32_t conv_value);
#endif /* OM_1_DATA_H_ */
