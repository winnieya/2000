/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-03-25     Tery       the first version
 */
#ifndef APPLICATIONS_SRC_CONTROLLER_H_
#define APPLICATIONS_SRC_CONTROLLER_H_

#include <rtthread.h>

#define START_MAIN      0

#define IGNITE_SELECT     1
#define IGNITE_PRCOESS    5
#define IGNITE_FAIL       9
#define IGNITE_AGAIN      10
#define IGNITE_COMPLETE_PPM   6
#define IGNITE_COMPLETE_MGL   7
#define IGNITE_COMPLETE_MOL   8

#define CALIBRATION_SELECT 2
#define CALIBRATION_PROCESS 11
#define CALIBRATION_FID_SELECT 50
#define CALIBRATION_PID_SELECT 51
#define CALIBRATION_MAIN 11
#define CALIBRATION_0 12
#define CALIBRATION_1 13
#define CALIBRATION_2 14

#define BG_VALUE_DEDUCT     3
#define BG_VALUE_PRCOESS    15
#define BG_VALUE_COMPLETE    16

#define OTHER_SELECT       4
#define OTHER_MAIN       17
#define OTHER_DEVICE_INFO  18
#define OTHER_DEVICE_INFO_SHOW   22
#define OTHER_BAT_SELECT   19
#define OTHER_BAT_SHOW     23
#define OTHER_LIGHT_SELCT  20
#define OTHER_LIGHT_MAIN  24
#define OTHER_LIGHT_SHOW_AND_SET   25
#define OTHER_LIGHT_SHOW_AND_SAVE  26
#define OTHER_LIGHT_SHOW_AND_SET_MAIN   27
#define OTHER_LIGHT_INCREASE   28
#define OTHER_LIGHT_REDUCE   29
#define OTHER_LIGHT_SET_CONFIRM 30

#define OTHER_UNIT_SELECT    21
#define OTHER_UNIT_MAIN    31
#define OTHER_UNIT_PPM    32
#define OTHER_UNIT_MGM    33
#define OTHER_UNIT_MOL    34

#define NUMBER_KEYBOARD_MAIN 35
#define NUMBER_KEYBOARD_0 36
#define NUMBER_KEYBOARD_1 37
#define NUMBER_KEYBOARD_2 38
#define NUMBER_KEYBOARD_3 39
#define NUMBER_KEYBOARD_4 40
#define NUMBER_KEYBOARD_5 41
#define NUMBER_KEYBOARD_6 42
#define NUMBER_KEYBOARD_7 43
#define NUMBER_KEYBOARD_8 44
#define NUMBER_KEYBOARD_9 45
#define NUMBER_KEYBOARD_POINT 46
#define NUMBER_KEYBOARD_CANCEL 48
#define NUMBER_KEYBOARD_CONFIRM 47


#define SKIP_MODE_CONFIRM    1
#define SKIP_MODE_CANCEL     2
#define SKIP_MODE_NEXT       3
#define SKIP_MODE_PREVIOU    4
#define SKIP_MODE_NONE       0

#define UNIT_PPM    0
#define UNIT_MGM    1
#define UNIT_MOL    2

#define HISTORY_SAVE_VALUE_COUNT    28

struct om_page_data
{
    uint8_t confirm_page_num;
    uint8_t previous_page_num;
    uint8_t next_page_num;
    uint8_t cancel_page_num;
};

struct _fid_save_data{
    uint8_t save_counts;
    uint32_t fid_value[HISTORY_SAVE_VALUE_COUNT];
    uint8_t value_unit[HISTORY_SAVE_VALUE_COUNT];
};

struct _m_device_set_data
{
    uint8_t unit_select;         //  0 ppm   1  umol/mol   2 mg/m3
    uint16_t threshould_value;
    uint8_t data_save_type;      //  0  手动       2 自动
    uint8_t sensor_select;       // 0  无     1  PID  2 O2  3 CO  4 LEL   5  H2S
    uint16_t data_save_time;
    uint8_t bg_value_type;       //  0  仅检测     1   检测扣除
    uint8_t bt_switch;           // 0  关      1  开
    uint8_t sweep_switch;        // 0  关      1开
    uint8_t accuracy_type;       // 0  ppm  1 ppb
};

void om_src_monitor(void);

#endif /* APPLICATIONS_SRC_CONTROLLER_H_ */
