/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-01-11     Tery       the first version
 */
#ifndef APPLICATIONS_CALIBRATION_H_
#define APPLICATIONS_CALIBRATION_H_

#include <rtthread.h>



#define FID_CURVE  1
#define PID_CURVE  2

#define MAX_POINT_COUNT     8
#define MAX_CURVE_COUNT     7
#define SAVE_VALUE_COUNT    28

#define MIN_CURVE_CALCULATION_POINT   2

#define MAGNIFICATION_TIMES     100000
#define TOFIXED                 100

struct _calibration_point{
    uint8_t calibration_flag;
    uint8_t point_counts;
    int64_t  k_value[MAX_CURVE_COUNT];
    int64_t  b_value[MAX_CURVE_COUNT];
    uint32_t gas_value[MAX_POINT_COUNT];
    uint32_t signal_value[MAX_POINT_COUNT];
};


void curve_reset(uint8_t curve_type);
void curve_updata(uint8_t curve_type,uint32_t _gas_value,uint32_t _signal_value);
uint32_t get_detect_value(uint8_t curve_type,uint32_t read_signal);
uint8_t get_detect_flag(uint8_t curve_type);
void ppm_calbarate(void);
uint32_t get_curve_value(uint8_t index);
uint32_t get_fid_calbarate_value(void);
uint32_t get_othner_sensor_calbarate_value(void);

void om_device_set_data_read(uint8_t* read_data,uint8_t data_len);
void om_device_set_data_save(uint8_t* set_data,uint8_t data_len);
void om_history_data_write(uint8_t* set_fid_data,uint8_t data_len);
void om_history_data_read(uint8_t* fid_read_data,uint8_t data_len);
#endif /* APPLICATIONS_CALIBRATION_H_ */
