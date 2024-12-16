/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-14     Tery       the first version
 */
#ifndef OM_1_POWER_H_
#define OM_1_POWER_H_

#include <rtthread.h>

#define OM_ON  RT_TRUE
#define OM_OFF  RT_FALSE

#define IGNITE_START    1
//#define IGNITING        2
#define IGNITE_COMPLTE  3
#define NO_NEDD_IGNITE  4
//#define NEDD_IGNITE     5

#define OM_SOLENOIDA    2
#define OM_SOLENOIDB    1

#define OM_NONE_STEP        0   /* open pump、SOL1 */
#define OM_PUMP_START       1   /* pump全功率运行5s */
#define OM_IGNITE_STEP      3   /* 点火线圈IO拉高1s后，打开SOL2 */
#define OM_IGNITE_STEP_PRE      2   /* 点火线圈IO拉高1s后，打开SOL2 */
#define OM_IGNITE_END       5   /* 点火线圈IO拉高1s后，关闭SOL2 */
#define OM_IGNITE_STEADY_FLOW       6   /* 点火线圈IO拉高1s后，关闭SOL2 */
#define OM_DETECTING        7
#define OM_SWEEPING        8

void om_set_pin(rt_base_t pin,rt_bool_t value);

void om_start_monitor(void);

void om_1_pump_init(void);
rt_bool_t om_get_pump_state(void);
void om_set_pump_power(rt_uint16_t power);
rt_uint16_t om_get_pump_power_adjust(void);

void om_dac_set_dac_value(rt_uint16_t vol);
rt_uint16_t om_dac_get_dac_value(void);

void om_ht_gpio_init(void);
void om_1_ht_on_off(rt_bool_t on_off);

void om_1_solenoid_init(void);
rt_bool_t om_get_solenoidA_state(void);
rt_bool_t om_get_solenoidB_state(void);

void om_1_ignite_init(void);
void om_1_ignite_start(void);
void om_1_ignite_end(void);
uint8_t om_get_ignite_step(void);
rt_bool_t om_is_timeout(rt_tick_t ref_tick,rt_uint16_t timing_time);

void set_pid_onoff(uint8_t on_off);

#endif /* APPLICATIONS_OM_1_POWER_H_ */
