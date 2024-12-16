/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-15     Tery       the first version
 */
#ifndef OM_1_BLE_H_
#define OM_1_BLE_H_

#include <rtthread.h>

void om_ble_pin_init(void);
void om_rst_ble(void);
rt_bool_t om_ble_is_connecet(void);
rt_bool_t om_ble_is_could_send_data(void);

#endif /* OM_1_BLE_H_ */
