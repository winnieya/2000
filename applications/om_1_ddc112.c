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

#include "om_1_ddc112.h"
#include "om_1_power.h"


#define DBG_TAG "om_1_ddc112"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define DDC_DVALID    GET_PIN(D, 1)
#define DDC_DXMIT    GET_PIN(A, 7)
#define DDC_TEST    GET_PIN(D, 7)

#define DDC_RANGE2    GET_PIN(D, 2)
#define DDC_RANGE1    GET_PIN(D, 3)
#define DDC_RANGE0    GET_PIN(D, 4)

#define PWM_CONV_NAME        "pwm1"  /* PWM设备名称 */
#define PWM_CONV_CHANNEL4     4     /* PWM通道 */
struct rt_device_pwm *pwm_conv;      /* PWM设备句柄 */

#define PWM_CLK_NAME        "pwm4"  /* PWM设备名称 */
#define PWM_CLK_CHANNEL3     3       /* PWM通道 */
struct rt_device_pwm *pwm_clk;      /* PWM设备句柄 */

#define DDC112_CLK_PIN   GET_PIN(B, 8)
#define DDC112_DATA_OUT_PIN   GET_PIN(A, 6)

#define OM_DDC112_I_M_0      65625
#define OM_DDC112_I_M_3      165

static rt_uint8_t g_om_ddc_dvalid_value = 0;
static rt_uint64_t g_om_ddc112_value = 0;
static rt_uint64_t g_om_ddc112_value_buf[100] = {0};
static rt_uint8_t g_om_ddc112_read_count = 0;
static rt_uint8_t g_om_ddc112_range = 0;
static rt_uint8_t g_om_current_hardware_avg = 50;
static rt_uint8_t g_om_ddc112_read_count_flag = 0;

static rt_uint64_t g_om_ddc112_cal_value_buf[55] = {0};
static rt_uint8_t g_om_ddc112_cal_count = 0;

static rt_uint64_t _report_value_buf[20] = {0};
static rt_uint8_t _average_report_value = 20;
static rt_uint8_t _average_report_value_count = 0;

struct om_comm_pin g_ddc112_range_pins[] = {{0,DDC_RANGE0},
                                            {1,DDC_RANGE1},
                                            {2,DDC_RANGE2}};



static void ddc_dvalid_mask(void);
static void ddc_dvalid_reset(void);
static rt_uint8_t ddc_dvalid_get(void);
static void ddc112_read_data(void);
static void ddc112_pwm_set(void);
static int ddc112_pwm_init(void);

extern rt_uint32_t om_get_sample_pressure_k(void);


static rt_thread_t om_ddc112_read_thread = RT_NULL;
static void om_ddc112_read(void *parameter);

void om_start_ddc112_read(void)
{
    om_ddc112_read_thread = rt_thread_create("om_ddc112_read_thread", om_ddc112_read, RT_NULL, 548, 11, 10);
    if (om_ddc112_read_thread != RT_NULL)
    {
        rt_thread_startup(om_ddc112_read_thread);
        LOG_D("om_monitor_ctr_thread is running\n");
    }
}


void om_ddc112_read(void *parameter)
{
    while(1)
    {

        om_ddc112_data_read();
        om_get_ddc112_value();
        rt_thread_mdelay(5);
    }
}

rt_uint8_t om_get_ddc112_range_value(void)
{
    return g_om_ddc112_range;
}

/* 获取ddc112测量电流值
 * g_om_ddc112_range 为0  电容是87.5，分辨率是6.5625fA
 * g_om_ddc112_range 为3 电容是220，220:87.5= 88:35*/

uint32_t om_get_ddc112_value(void)
{
    uint32_t om_pico_amps = 0;
    uint32_t om_max_pico_amps = 0;
    uint64_t total_pico = 0;
    uint64_t total_cal = 0;
    uint32_t om_max_collect = 0;

    /*<<<<<ddc112采集信号值二次平均，平滑处理，平滑长度50,并去掉最大值*/
    g_om_ddc112_cal_value_buf[g_om_ddc112_cal_count] = g_om_ddc112_value;

    if(g_om_ddc112_cal_count < (g_om_current_hardware_avg - 1))
    {
        g_om_ddc112_cal_count ++;
    }
    else
    {
        g_om_ddc112_cal_count = 0;
    }

    for(int i = 0;i<g_om_current_hardware_avg;i++)
    {
        total_cal = total_cal + g_om_ddc112_cal_value_buf[i];
        om_max_collect = (g_om_ddc112_cal_value_buf[i] > om_max_collect) ? g_om_ddc112_cal_value_buf[i] : om_max_collect;

    }
    g_om_ddc112_value = (total_cal - om_max_collect) /(g_om_current_hardware_avg - 1);
    /*>>>>>*/

    if(g_om_ddc112_range == 0)
    {
        /*除以10000，是电流放大10000倍，除以100，加上app除以10，电流单位转换成pA*/
        om_pico_amps = ((g_om_ddc112_value) * OM_DDC112_I_M_0 / 10000);
    }
    else if(g_om_ddc112_range == 3)
    {
        om_pico_amps = ((g_om_ddc112_value) * OM_DDC112_I_M_3 / 10);
    }

    if(g_om_ddc112_range == 0)
    {
        /*除以10000，是电流放大10000倍，除以100，加上app除以10，电流单位转换成pA*/
        LOG_D("g_om_ddc112_value  %d ,\r\n",g_om_ddc112_value);
        om_pico_amps = ((g_om_ddc112_value) * OM_DDC112_I_M_0 / 10000);
//        LOG_I("om_pico_amps  %d ,\r\n",om_pico_amps);
        if(om_pico_amps > 6500000)
        {
            om_set_ddc112_range(3);
            rt_memset(g_om_ddc112_value_buf,0,sizeof(g_om_ddc112_value_buf));
            g_om_ddc112_read_count = 0;
            g_om_ddc112_read_count_flag = 1;
            rt_thread_mdelay(500);
        }
        LOG_D("g_om_ddc112_range  %d ,\r\n",om_get_ddc112_range_value());
    }
    else if(g_om_ddc112_range == 3)
    {
        LOG_D("g_om_ddc112_value  %d ,\r\n",g_om_ddc112_value);
        om_pico_amps = ((g_om_ddc112_value) * OM_DDC112_I_M_3 / 10);
//        LOG_I("om_pico_amps  %d ,\r\n",om_pico_amps);
        if(om_pico_amps < 6000000)
        {
            om_set_ddc112_range(0);
            rt_memset(g_om_ddc112_value_buf,0,sizeof(g_om_ddc112_value_buf));
            g_om_ddc112_read_count = 0;
            g_om_ddc112_read_count_flag = 1;
            rt_thread_mdelay(500);
        }
        LOG_D("g_om_ddc112_range  %d ,\r\n",om_get_ddc112_range_value());
    }

    _report_value_buf[_average_report_value_count] = om_pico_amps;

    if(_average_report_value_count < (_average_report_value - 1))
    {
        _average_report_value_count ++;
    }
    else
    {
        _average_report_value_count = 0;
    }

    for(int i = 0;i<_average_report_value;i++)
    {
        total_pico = total_pico + _report_value_buf[i];
        om_max_pico_amps = (_report_value_buf[i] > om_max_pico_amps) ? _report_value_buf[i] : om_max_pico_amps;
    }

    return (total_pico - om_max_pico_amps) / (_average_report_value - 1);
}


/* ddc112数据采集判断，如果dvaild引脚为高，才可以读到ddc112采集值 */
void om_ddc112_data_read(void)
{
    if(ddc_dvalid_get())
    {
        ddc112_read_data();
        ddc_dvalid_reset();
    }
//    ddc112_read_data();
}
SPI_HandleTypeDef SpiHandle;
int  ddc112_spi_init(void)
{
    SpiHandle.Instance               = SPI1;
    SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
    return 0;
}
/* ddc112初始化，包括GPIO、spi、pwm*/
void om_ddc112_init(void)
{
    rt_pin_mode(DDC_DVALID, PIN_MODE_INPUT);  //dvalid

    rt_pin_mode(DDC_DXMIT, PIN_MODE_OUTPUT);  //dxmit
    rt_pin_write(DDC_DXMIT, PIN_HIGH);

    rt_pin_mode(DDC_RANGE0, PIN_MODE_OUTPUT);   //range 0
    rt_pin_write(DDC_RANGE0, PIN_HIGH);
    rt_pin_mode(DDC_RANGE1, PIN_MODE_OUTPUT);   //range 1
    rt_pin_write(DDC_RANGE1, PIN_HIGH);
    rt_pin_mode(DDC_RANGE2, PIN_MODE_OUTPUT);   //range 2
    rt_pin_write(DDC_RANGE2, PIN_HIGH);

    rt_pin_mode(DDC_TEST, PIN_MODE_OUTPUT);  //test
    rt_pin_write(DDC_TEST, PIN_LOW);

    rt_pin_mode(DDC_DVALID, PIN_MODE_INPUT_PULLUP);
    rt_pin_attach_irq(DDC_DVALID, PIN_IRQ_MODE_FALLING, (void *)ddc_dvalid_mask, RT_NULL);
    rt_pin_irq_enable(DDC_DVALID, PIN_IRQ_ENABLE);
//
//    rt_pin_mode(DDC112_CLK_PIN, PIN_MODE_OUTPUT);   //range 1
//    rt_pin_write(DDC112_CLK_PIN, PIN_LOW);
    rt_pin_mode(DDC112_DATA_OUT_PIN, PIN_MODE_INPUT);   //range 2
    rt_pin_write(DDC112_DATA_OUT_PIN, PIN_HIGH);

    ddc112_spi_init();
    ddc112_pwm_init();
    ddc112_pwm_set();
}

void om_set_ddc112_range(rt_uint8_t range_value)
{
    rt_bool_t pin_value = 0;
    g_om_ddc112_range = range_value;
    if(range_value == 3)
    {
        pin_value = 0;
    }
    else if(range_value == 0)
    {
        pin_value = 1;
    }
    for(int i = 0; i <sizeof(g_ddc112_range_pins) / sizeof(struct om_comm_pin); i++)
    {
        om_set_pin(g_ddc112_range_pins[i].pin,pin_value);
    }
}

void om_set_ddc112_conv(rt_uint8_t conv_value)
{
    rt_uint32_t period1, pulse1;

    period1 = 5000000;    /* 单位为纳秒ns 5000000ns = 5000us = 5ms vocs命令配置是50ms*/
    pulse1 = 2500000;

    /* 设置PWM周期和脉冲宽度 */
    rt_pwm_set(pwm_clk, PWM_CONV_CHANNEL4, period1, pulse1); // CONV
}

void om_set_current_hardware_avg(rt_uint8_t current_hardware_avg_value)
{
    g_om_current_hardware_avg = current_hardware_avg_value;
    g_om_ddc112_read_count = 0;
}

/* ddc112积分数据读取*/
static void ddc112_read_data(void)
{
    rt_uint64_t total_ddc112_value = 0;
    rt_uint8_t g_om_ddc112_read[5] = {0};
    uint64_t get_ddc112_value = 0;
    uint64_t get_max_ddc112_value = 0;

    rt_pin_write(DDC_DXMIT, PIN_LOW);  //dxmit

    if(HAL_SPI_Receive(&SpiHandle,g_om_ddc112_read, sizeof(g_om_ddc112_read),1000) != HAL_OK)
    {
        LOG_I("HAL_SPI_Receive() error \r\n");
      /* Transfer error in transmission process */
      return;
    }
    rt_pin_write(DDC_DXMIT, PIN_HIGH);  //dxmit

    get_ddc112_value = (0xFFFFFFFF & ((0x0f &g_om_ddc112_read[2]) << 16 | g_om_ddc112_read[3] << 8|  g_om_ddc112_read[4]));
    rt_memset(g_om_ddc112_read,0, sizeof(g_om_ddc112_read));
    if(get_ddc112_value == 0 )
    {
        LOG_D("ddc112_sample() read ddc1122 0\n");
        return;
    }

    LOG_D("g_om_ddc112_read  %d\n",0xFFFFFFFF & ((0x0f &g_om_ddc112_read[2]) << 16 | g_om_ddc112_read[3] << 8|  g_om_ddc112_read[4]));

    g_om_ddc112_value_buf[g_om_ddc112_read_count] = get_ddc112_value;

    if(g_om_ddc112_read_count < (g_om_current_hardware_avg - 1))
    {
        g_om_ddc112_read_count ++;
    }
    else
    {
        g_om_ddc112_read_count = 0;
        g_om_ddc112_read_count_flag = 0;
    }

    for(int i = 0;i<g_om_current_hardware_avg;i++)
    {
        total_ddc112_value = total_ddc112_value + g_om_ddc112_value_buf[i];
        get_max_ddc112_value = (g_om_ddc112_value_buf[i] > get_max_ddc112_value) ? g_om_ddc112_value_buf[i] : get_max_ddc112_value;
    }
    if(g_om_ddc112_read_count_flag == 0)
    {
        g_om_ddc112_value = (total_ddc112_value - get_max_ddc112_value) / (g_om_current_hardware_avg - 1);
    }
    else {
        g_om_ddc112_value = (total_ddc112_value - get_max_ddc112_value) / g_om_ddc112_read_count;
    }
    total_ddc112_value = 0;
}
/*拉高ddc112引脚dvaild状态，指示ddc112积分完成，可以读取数据*/
static void ddc_dvalid_mask(void)
{
    g_om_ddc_dvalid_value = 1;  // dvalid pin is high,could read data
}

/*复位ddc112引脚dvaild状态为低*/
static void ddc_dvalid_reset(void)
{
    g_om_ddc_dvalid_value = 0; // clear data,after read data
}

/*返回ddc112引脚dvaild状态*/
static rt_uint8_t ddc_dvalid_get(void)
{
    return g_om_ddc_dvalid_value;
}

/*ddc112用到的pwm初始化*/
static int ddc112_pwm_init(void)
{
    /* 查找设备 */

    pwm_conv = (struct rt_device_pwm *)rt_device_find(PWM_CONV_NAME);
    if (pwm_conv == RT_NULL)
    {
        LOG_I("pwm sample run failed! can't find %s device!\n", PWM_CONV_NAME);
        return RT_ERROR;
    }
    pwm_clk = (struct rt_device_pwm *)rt_device_find(PWM_CLK_NAME);
    if (pwm_clk == RT_NULL)
    {
        LOG_D("pwm sample run failed! can't find %s device!\n", PWM_CLK_NAME);
        return RT_ERROR;
    }
    /* 使能设备 */
    rt_pwm_enable(pwm_conv, PWM_CONV_CHANNEL4);
    rt_pwm_enable(pwm_clk, PWM_CLK_CHANNEL3);
    LOG_D("PWM init passS\n");
    return RT_EOK;
}

/*配置 PWM波，使ddc112正常工作*/
static void ddc112_pwm_set(void)
{
    rt_uint32_t period1, pulse1,period2, pulse2;

    period2 = 128;    /* 单位为纳秒ns */
    pulse2 =64;          /* 单位为纳秒ns */
    period1 = 10000000;    /* 单位为纳秒ns */
    pulse1 = period1/2;

    /* 设置PWM周期和脉冲宽度 */

    rt_pwm_set(pwm_conv, PWM_CONV_CHANNEL4, period1, pulse1);  // CONV
    rt_pwm_set(pwm_clk, PWM_CLK_CHANNEL3, period2, pulse2); //CLK
}

