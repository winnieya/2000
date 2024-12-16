/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-04-14     Tery       the first version
 */
#include <board.h>
#include <rtdevice.h>

#define DBG_TAG "om_1_sensor"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#include "om_1_sensor.h"
#include "om_1_session.h"
#include "om_1_power.h"


#define ADC_DEV_NAME        "adc1"      /* ADC 设备名称 */
#define ADC_DEV_CHANNEL0     0           /* ADC 通道 */
#define ADC_DEV_CHANNEL1     1           /* ADC 通道 */
#define ADC_DEV_CHANNEL2     2           /* ADC 通道 */
#define ADC_DEV_CHANNEL3     3           /* ADC 通道 */
#define ADC_DEV_CHANNEL9     9           /* ADC 通道 */
#define ADC_DEV_CHANNEL10     10           /* ADC 通道 */
#define ADC_DEV_CHANNEL11     11           /* ADC 通道 */
#define ADC_DEV_CHANNEL14     14           /* ADC 通道 */
#define ADC_DEV_CHANNEL25     25           /* ADC 通道 */

#define REFER_VOLTAGE       33000         /* 参考电压 3.3V,数据精度乘以100保留2位小数*/
#define CONVERT_BITS        (1 << 12)   /* 转换位数为12位 */

#define OM_DDC112_I_M      65625

#define THEORETICAL_SERVICE_TIME_BAT      12*60        //min
#define THEORETICAL_SERVICE_TIME_HYDROGEN      15*60   //min
#define VOLTAGE_MIN                             5800   //mv
#define VOLTAGE_MAX                             8100   //mv


/*
 */
struct troubleshooting_tips_
{
    uint16_t troubleshooting_tips_report_flag;
    uint16_t troubleshooting_tips_detection_flag;
    uint8_t report_count[MAX_TROUBLESHOOTING_TIP_NUM];
};

struct troubleshooting_tips_ troubleshooting_tips_data;

static rt_adc_device_t adc_dev;
static rt_uint32_t g_om_sample_presure = 0;
static rt_uint32_t g_om_sample_presure_k = 0;

/*获取ADC设备句柄*/
rt_adc_device_t om_get_adc_dev_handle(void)
{
    return adc_dev;
}

/*adc 驱动初始化*/
int om_adc_vol_init(void)
{
    rt_err_t ret = RT_EOK;

    /* 查找设备 */
    adc_dev = (rt_adc_device_t)rt_device_find(ADC_DEV_NAME);
    if (adc_dev == RT_NULL)
    {
        LOG_D("adc sample run failed! can't find %s device!\n", ADC_DEV_NAME);
        return RT_ERROR;
    }

    /* 使能设备 */

    ret = rt_adc_enable(adc_dev, ADC_DEV_CHANNEL9);
    ret = rt_adc_enable(adc_dev, ADC_DEV_CHANNEL3);
    ret = rt_adc_enable(adc_dev, ADC_DEV_CHANNEL11);
    ret = rt_adc_enable(adc_dev, ADC_DEV_CHANNEL14);
    return ret;
}

/*
 * 热电偶
 */
rt_uint32_t om_thermocouple_read(void)
{
    rt_uint32_t value, vol;

    /* 读取采样值 */
    value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL9);
    LOG_D("the channel 9 value is :%d \n", value);
    /* 转换为对应电压值 */
    vol = value * REFER_VOLTAGE / CONVERT_BITS;
    LOG_D("the channel 9 voltage is :%d.%d mV\n", vol / 10, vol % 10);
    LOG_D("the om_thermocouple_read  value is :%d \n", (vol * 2442 /100/101) );

    return (12*(vol * 2442 /100/101)*18/25/10  + 2731); // 40.9uv/℃  电压放大101倍 app端放大10倍

}

/*
 * 温度传感器
 */
rt_uint32_t om_temperature_sensor_read(void)
{
    rt_uint32_t value, vol;
    rt_uint16_t cur_temp = 0;

    /* 读取采样值 */
    value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL11);
    LOG_D("the channel 11 value is :%d \n", value);
    /* 转换为对应电压值 */
    vol = value * REFER_VOLTAGE / CONVERT_BITS;
    cur_temp = vol >= 10000 ? (50*100 + (vol - 10000)) : (50*100 - (10000 - vol));
    LOG_D("the channel 11 voltage is :%d.%d \n", vol / 10000, vol % 10000);

    LOG_D("the channel 11 cur_temp is :%d.%d \n", cur_temp/100, cur_temp % 100);
    LOG_D("the channel 11 value is :%x \n", cur_temp/10);

    LOG_D("the channel 11 cur_temp is :%d \n", (cur_temp/10/4 + 2731));
    return cur_temp/10/4 + 2731;  //app端显示是华氏温度  cur_temp/10/4 温度降低4倍

}



rt_uint32_t om_get_sample_pressure(void)
{
    return g_om_sample_presure;
}
rt_uint32_t om_get_sample_pressure_k(void)
{
    return g_om_sample_presure_k;
}


/*
 * 电池电量
 */
uint16_t om_battery_level_read(void)
{
    rt_uint32_t value, vol;
    uint32_t time_remaining  = 0;
    uint32_t vol_tmp = 0;

    /* 读取采样值 */
    value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL3);
    LOG_D("the channel 3 value is :%d \n", value);
    /* 转换为对应电压值 */
    vol = value * REFER_VOLTAGE / CONVERT_BITS;
    LOG_D("the channel 3 voltage is :%d.%d mV\n", vol *11 / 10, vol *11  % 10);


    if((vol *11 / 10 + vol *11  % 10) < VOLTAGE_MIN)
    {
        return 0;
    }

    return vol *11 / 10;
}

/*
 * 系统电流
 */
rt_uint32_t om_system_current_read(void)
{
    rt_uint32_t value, vol;

    /* 读取采样值 */
    value = rt_adc_read(adc_dev, ADC_DEV_CHANNEL14);
    LOG_D("the channel 14 value is :%d \n", value);
    /* 转换为对应电压值 */
    vol = value * REFER_VOLTAGE / CONVERT_BITS;

//    LOG_I("the channel 14 voltage is :%d.%d mV\n", vol / 10, vol % 10);


    return vol*10/85;  //mA  (Vol/10)/20/0.01   mV/放大倍数/电阻
}

uint16_t om_get_troubleshooting_tips(void)
{
    uint16_t flags = 0;
    uint8_t index = 0;

    for(index = 0;index<MAX_TROUBLESHOOTING_TIP_NUM;index++)
    {
        /*检测故障标志位*/
        if((troubleshooting_tips_data.troubleshooting_tips_detection_flag >> index) & 0x01)
        {
            /*判断该故障是否已经提示*/
            if(!((troubleshooting_tips_data.troubleshooting_tips_report_flag >> index) & 0x01))
            {
                /*如果没有提示，提示该故障。并标记已经提示，以免重复提示，并记录提示次数*/
                troubleshooting_tips_data.troubleshooting_tips_report_flag |= (0x01 << index);
                flags |= (0x01 << index);
                troubleshooting_tips_data.report_count[index] = 1;
            }
            else {
                /*如果有提示，但是提示次数少于10次，则继续提示并记录提示次数*/
                if(troubleshooting_tips_data.report_count[index] < MAX_TROUBLESHOOTING_TIP_COUNT)
                {
                    flags |= (0x01 << index);
                    troubleshooting_tips_data.report_count[index]++;
                }
            }
        }
    }
    return flags;
}

void om_set_troubleshooting_tips(uint8_t flag)
{
    troubleshooting_tips_data.troubleshooting_tips_detection_flag |= (0x01 << flag);
}


SPI_HandleTypeDef SpiHandle;
int  pid_spi_init(void)
{
    SpiHandle.Instance               = SPI3;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    SpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    SpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    SpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    SpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    SpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    SpiHandle.Init.TIMode            = SPI_TIMODE_DISABLE;
    SpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    SpiHandle.Init.CRCPolynomial     = 7;
    SpiHandle.Init.NSS               = SPI_NSS_SOFT;
    SpiHandle.Init.Mode = SPI_MODE_MASTER;
    if(HAL_SPI_Init(&SpiHandle) != HAL_OK)
    {
      /* Initialization Error */
      Error_Handler();
    }
    rt_pin_mode(GET_PIN(A,15), PIN_MODE_OUTPUT);
    return 0;
}


static rt_uint32_t g_om_pid_value = 0;
static rt_uint32_t g_om_pid_value_buf[55] = {0};
static rt_uint8_t g_om_pid_read_count = 0;
static rt_uint8_t g_om_current_pid_avg = 30;

uint32_t om_get_pid_value(void)
{
    return (uint32_t)(g_om_pid_value);
}

uint32_t om_get_pid_signal(void)
{
    uint8_t pid_read[4] = {0};
    uint16_t value = 0;
    rt_uint32_t total_pid_value = 0;
    rt_uint32_t max_value = 0;

    rt_pin_write(GET_PIN(A,15), PIN_LOW);

     if(HAL_SPI_Receive(&SpiHandle,(uint8_t*)pid_read, 4,1000) != HAL_OK)
     {
       /* Transfer error in transmission process */
         LOG_I("HAL_SPI_Receive error \r\n");
       return 0;
     }
     rt_pin_write(GET_PIN(A,15), PIN_HIGH);

     value = (pid_read[1] >> 2) + ((pid_read[0] & 0x03F) << 6) ;

     g_om_pid_value_buf[g_om_pid_read_count] = value;

     if(g_om_pid_read_count < (g_om_current_pid_avg - 1))
     {
         g_om_pid_read_count ++;
     }
     else
     {
         g_om_pid_read_count = 0;
     }

     for(int i = 0;i<g_om_current_pid_avg;i++)
     {
         total_pid_value = total_pid_value + g_om_pid_value_buf[i];
         if(max_value < g_om_pid_value_buf[i])
         {
             max_value = g_om_pid_value_buf[i];
         }
     }
     g_om_pid_value = (total_pid_value - max_value) / (g_om_current_pid_avg - 1);
     total_pid_value = 0;

     return (uint32_t)(value);
}

