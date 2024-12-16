/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-14     Tery       the first version
 */
#ifndef OM_1_DDC112_H_
#define OM_1_DDC112_H_

#include <rtthread.h>



struct om_comm_pin
{
    rt_uint8_t id;
    rt_base_t pin;
};

uint32_t om_get_ddc112_value(void);
rt_uint8_t om_get_ddc112_range_value(void);
void om_start_ddc112_read(void);
void om_set_ddc112_range(rt_uint8_t range_value);
void om_set_ddc112_conv(rt_uint8_t conv_value);
void om_set_current_hardware_avg(rt_uint8_t current_hardware_avg_value);
rt_uint32_t _om_get_ddc112_value(void);
void om_ddc112_init();
void om_ddc112_data_read(void);
rt_uint32_t _curve_om_get_ddc112_value(void);
#endif /* OM_1_DDC112_H_ */
