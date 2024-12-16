/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-15     Tery       the first version
 */

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>
#include <string.h>

#define DBG_TAG "om_1_session"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "om_1_session.h"
#include "om_1_data.h"
#include "om_1_power.h"

#define UART_BLE_NAME                 "uart3"
#define CARRIAGE_RETURN                0x0D       /* 结束位设置为 \r，即回车符 */
#define LINE_FEED                      0x0A       /* 结束位设置为 \n，即回车符 */
#define DATA_HEART                     0x55       /* 结束位设置为 \n，即回车符 */
#define DATA_HEART_21                     0x5A       /* 结束位设置为 \n，即回车符 */
#define DATA_END_LEN                   2


#define BLE_AT_INIT     "AT+BTINIT=1\r\n"
#define BLE_AT_SPINIT   "AT+BTSPPINIT=2\r\n"
#define BLE_AT_NAME     "AT+BTNAME=\"OM2000\"\r\n"
#define BLE_AT_MODE     "AT+BTSCANMODE=2\r\n"
#define BLE_AT_BT_POWER     "AT+BTPOWER=5,5\r\n"
#define BLE_AT_BTSECPARAM "AT+BTSECPARAM=3,0\r\n"
#define BLE_AT_SP_START  "AT+BTSPPSTART\r\n"
#define BLE_AT_SPSEND    "AT+BTSPPSEND\r\n"
#define BLE_AT_RST    "AT+RST\r\n"
#define BLE_AT_SYSMSG    "AT+SYSMSG=5\r\n"
#define ESP32_SLEEP1   "AT+SLEEP=2"

#define READ_DATA              1
#define READ_CARRIAGE_RETURN   2
#define READ_LINE_FEED         3
#define READ_HEART             4
#define READ_NONE              5

#define READ_FRAME             6
#define READ_ACK               7
#define READ_LEN               8
#define CHECKSUM_LEN           1

#define DATA_LEN_MAX           64

#define REC_DATA_LEN    10


static rt_device_t ble_serial3;

static char om_g_ble_rev_data[DATA_LEN_MAX]={0};
static char *p_read;
static rt_uint8_t g_om_read_state = READ_NONE;
static rt_uint8_t g_om_cur_buf_len = 0;
static uint8_t g_bt_close_flag = 1;

/* 蓝牙数据发送，使用串口透传到蓝牙模块 */
void om_send_data(void *buffer, rt_size_t size)
{

    rt_device_write(ble_serial3, 0, buffer, size);
    LOG_D("send data %s and size %d!\n", buffer,size);
}

void esp32_init(void)
{
    om_send_data(BLE_AT_RST,rt_strlen(BLE_AT_RST));
    rt_thread_mdelay(1000);

    /*"AT+SYSMSG=5\r\n"*/
    om_send_data(BLE_AT_SYSMSG,rt_strlen(BLE_AT_SYSMSG));
    rt_thread_mdelay(50);
    /*"AT+BTINIT=1\r\n"*/
    om_send_data(BLE_AT_INIT,rt_strlen(BLE_AT_INIT));
    rt_thread_mdelay(1000);
    /*"AT+BTSPPINIT=2\r\n"*/
    om_send_data(BLE_AT_SPINIT,rt_strlen(BLE_AT_SPINIT));
    rt_thread_mdelay(50);
    /*"AT+BTNAME=\"***\"\r\n"*/
    om_send_data(BLE_AT_NAME,rt_strlen(BLE_AT_NAME));
    rt_thread_mdelay(50);
    /*"AT+BTPOWER=5,5\r\n"*/
    om_send_data(BLE_AT_BT_POWER,rt_strlen(BLE_AT_BT_POWER));
    rt_thread_mdelay(50);
    /*"AT+BTSCANMODE=2\r\n"*/
    om_send_data(BLE_AT_MODE,rt_strlen(BLE_AT_MODE));
    rt_thread_mdelay(50);
    /*"AT+BTSECPARAM=3,1,\"9527\"\r\n"*/
    om_send_data(BLE_AT_BTSECPARAM,rt_strlen(BLE_AT_BTSECPARAM));
    rt_thread_mdelay(50);
    /*"AT+BTSPPSTART\r\n"*/
    om_send_data(BLE_AT_SP_START,rt_strlen(BLE_AT_SP_START));
    rt_thread_mdelay(50);
}
/* 数据解析，使用状态机流转操作
 * 当没有检测到数据头部时，是READ_NONE状态，当检测到头部时，变成READ_HEART，继续有字节检测时，将状态改为
 * READ_DATA，在READ_DATA状态时，如果检测字节是CARRIAGE_RETURN，将状态改为READ_CARRIAGE_RETURN，
 * 如果检测到LINE_FEED，并且当前状态时READ_CARRIAGE_RETURN，则表示单个命令接收完成，处理命令，处理完后，将状态改为
 * READ_NONE，如果检测到LINE_FEED，并且当前状态不是READ_CARRIAGE_RETURN，则将状态改为READ_DATA
 * */
rt_uint8_t bt_data_len = 0;
rt_uint8_t bt_data_len_tmp = 0;

static uint8_t is_21_cmd = 0;
uint8_t om_get_read_type(void)
{
    return is_21_cmd;
}

void om_bt_uarts_data_parsing(void)
{
    rt_uint8_t read_len = 0;

    read_len = rt_device_read(ble_serial3, 0, om_g_ble_rev_data, 64);

    if(read_len == 0 || read_len > 255||read_len < 4)
    {
        goto loop2;
        return;
    }
    if(memcmp(om_g_ble_rev_data,"+BT",3) == 0)
    {
        /*"AT+BTSPPSEND\r\n"*/
        om_send_data(BLE_AT_SPSEND,rt_strlen(BLE_AT_SPSEND));
        rt_memset(om_g_ble_rev_data, 0, sizeof(om_g_ble_rev_data));

        LOG_I("BLE_AT_SPSEND  \r\n");
        LOG_I("receive error  \r\n");
        goto loop2;
        return;
    }

    for(int i=0;i<read_len;i++)
    {
        switch(g_om_read_state)
        {
        case READ_NONE:
            if(*p_read == DATA_HEART || *p_read == DATA_HEART_21)
            {
                g_om_read_state = READ_HEART;
                if(*p_read == DATA_HEART_21)
                {
                    is_21_cmd = 1;
                }
                else {
                    is_21_cmd = 0;
                }
            }
            p_read++;
            break;
        case READ_HEART:
            bt_data_len_tmp = *p_read;
            om_bt_data_handle((rt_uint8_t *)om_g_ble_rev_data,bt_data_len_tmp);
            goto loop2;
            break;
        default:
            p_read++;
            break;
        }

    }
loop2:
    rt_memset(om_g_ble_rev_data, 0, sizeof(om_g_ble_rev_data));
    g_om_cur_buf_len = 0;

    bt_data_len = 0;
    bt_data_len_tmp = 0;
    g_om_read_state = READ_NONE;
    p_read = om_g_ble_rev_data;
}

/*蓝牙使用到的串口初始化*/
void esp32_at_init(void)
{
    om_send_data(BLE_AT_RST,rt_strlen(BLE_AT_RST));
    rt_thread_mdelay(1500);

    /*"AT+SYSMSG=5\r\n"*/
    om_send_data(BLE_AT_SYSMSG,rt_strlen(BLE_AT_SYSMSG));
    rt_thread_mdelay(50);
    /*"AT+BTINIT=1\r\n"*/
    om_send_data(BLE_AT_INIT,rt_strlen(BLE_AT_INIT));
    rt_thread_mdelay(1500);
    /*"AT+BTSPPINIT=2\r\n"*/
    om_send_data(BLE_AT_SPINIT,rt_strlen(BLE_AT_SPINIT));
    rt_thread_mdelay(50);
    /*"AT+BTNAME=\"***\"\r\n"*/
    om_send_data(BLE_AT_NAME,rt_strlen(BLE_AT_NAME));
    rt_thread_mdelay(50);
    /*"AT+BTPOWER=5,5\r\n"*/
    om_send_data(BLE_AT_BT_POWER,rt_strlen(BLE_AT_BT_POWER));
    rt_thread_mdelay(50);
    /*"AT+BTSCANMODE=2\r\n"*/
    om_send_data(BLE_AT_MODE,rt_strlen(BLE_AT_MODE));
    rt_thread_mdelay(50);
    /*"AT+BTSECPARAM=3,1,\"9527\"\r\n"*/
    om_send_data(BLE_AT_BTSECPARAM,rt_strlen(BLE_AT_BTSECPARAM));
    rt_thread_mdelay(50);
    /*"AT+BTSPPSTART\r\n"*/
    om_send_data(BLE_AT_SP_START,rt_strlen(BLE_AT_SP_START));
}

void esp32_at_close(void)
{
    g_bt_close_flag = 1;
}
void esp32_at_open(void)
{

    g_bt_close_flag = 0;
}

uint8_t get_esp32_state(void)
{

    return g_bt_close_flag;
}

int om_ble_uarts_init(void)
{
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */

    rt_strncpy(uart_name, UART_BLE_NAME, RT_NAME_MAX);

    /* 查找系统中的串口设备 */
    ble_serial3 = rt_device_find(uart_name);
    if (!ble_serial3)
    {
        LOG_D("find %s failed!\n", uart_name);
        return RT_ERROR;
    }

    /* step2：修改串口配置参数 */
    config.baud_rate = BAUD_RATE_115200;        //修改波特率为 115200
    config.data_bits = DATA_BITS_8;           //数据位 8
    config.stop_bits = STOP_BITS_1;           //停止位 1
    config.bufsz     = 256;                   //修改缓冲区 buff size 为 256
    config.parity    = PARITY_NONE;           //无奇偶校验位

    /* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
    rt_device_control(ble_serial3, RT_DEVICE_CTRL_CONFIG, &config);
    /* 以中断接收及轮询发送模式打开串口设备 */
    rt_device_open(ble_serial3, RT_DEVICE_FLAG_INT_RX);

    esp32_at_init();

    rt_memset(om_g_ble_rev_data, 0, sizeof(om_g_ble_rev_data));
    g_om_cur_buf_len = 0;

    bt_data_len = 0;
    bt_data_len_tmp = 0;
    g_om_read_state = READ_NONE;
    p_read = om_g_ble_rev_data;
    return ret;
}




