/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-15     Tery       the first version
 */

#include <board.h>
#include <rtdevice.h>

#define DBG_TAG "om_1_ble"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "om_1_ble.h"

#define BLE_BOOT_PIN  GET_PIN(E,4)
#define BLE_SLEEP_PIN      GET_PIN(C,5)
#define BLE_DISCONTECT_PIN      GET_PIN(A,7)
#define BLE_CTS_PIN      GET_PIN(B,0)
#define BLE_RTS_PIN      GET_PIN(D,11)

#define BLE_RST_PIN      GET_PIN(D,12)

void om_ble_pin_init(void)
{
    rt_pin_mode(BLE_BOOT_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(BLE_BOOT_PIN, PIN_HIGH);

//    rt_pin_mode(BLE_SLEEP_PIN, PIN_MODE_OUTPUT);
//    rt_pin_write(BLE_SLEEP_PIN, PIN_LOW);

//    rt_pin_mode(BLE_CTS_PIN, PIN_MODE_OUTPUT);
//    rt_pin_write(BLE_CTS_PIN, PIN_LOW);
//
//    rt_pin_mode(BLE_RTS_PIN, PIN_MODE_INPUT);

    rt_pin_mode(BLE_DISCONTECT_PIN, PIN_MODE_OUTPUT);
    rt_pin_write(BLE_DISCONTECT_PIN, PIN_HIGH);
    rt_pin_mode(BLE_RST_PIN, PIN_MODE_OUTPUT);
//    rt_pin_write(BLE_RST_PIN, PIN_LOW);
//    rt_thread_mdelay(100);
    rt_pin_write(BLE_RST_PIN, PIN_HIGH);
}

