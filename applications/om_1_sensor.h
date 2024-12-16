/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-14     Tery       the first version
 */
#ifndef OM_1_SENSOR_H_
#define OM_1_SENSOR_H_

#include <rtthread.h>

#define MAX_TROUBLESHOOTING_TIP_NUM       16
#define MAX_TROUBLESHOOTING_TIP_COUNT    10

#define GAS_CIRCUIT_ANOMALY_FLAG          0
#define BATTERY_LESS_FLAG                 1
#define HYDROGEN_LESS_FLAG                2
#define HYDROGEN_ANOMALY_FLAG             3
#define IGNITION_COIL_ANOMALY_FLAG        4
#define TEMPERATURE_SENSOR_ANOMALY_FLAG   5
#define SIGNAL_ACQUISITION_ANOMALY_FLAG   6



int om_adc_vol_init(void);

rt_uint32_t om_thermocouple_read(void);
rt_uint32_t om_temperature_sensor_read(void);
uint16_t om_battery_level_read(void);
rt_uint32_t om_system_current_read(void);
rt_uint32_t om_get_sample_pressure(void);


int  pid_spi_init(void);
void pid_read_data(void);
uint32_t om_get_pid_value(void);
uint32_t om_get_pid_signal(void);
uint16_t om_get_troubleshooting_tips(void);
void om_set_troubleshooting_tips(uint8_t flag);

#endif /* OM_1_SENSOR_H_ */
