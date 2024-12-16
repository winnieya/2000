/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-01-11     Tery       the first version
 */
#include <board.h>
#include <rtdevice.h>
#include <string.h>

#include "calibration.h"
#include "file_operate.h"
#include "om_1_sensor.h"
#include "om_1_ddc112.h"
#include "om_1_power.h"

#define DBG_TAG "om_1_calibration"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define FID_CURVE_FILE    "/fid_curve_file.txt"
#define PID_CURVE_FILE    "/pid_curve_file.txt"
#define HISTORY_DATA_FILE    "/history_data_file.txt"
#define DECICE_SET_DATA_FILE    "/device_set_data_file.txt"

struct _calibration_point fid_calibration_point = {0};
struct _calibration_point pid_calibration_point = {0};

static uint32_t zore_signal_value = 0;

/*曲线计算
 * 原型是y=kx+b
 * y 浓度值
 * x FID信号板信号值，即微电流
 * */
static void curve_calculate(struct _calibration_point *cal_p)
{
    for(uint8_t i = 0;i<cal_p->point_counts - 1;i++)
    {
        cal_p->k_value[i] = (cal_p->gas_value[i+1] - cal_p->gas_value[i])*MAGNIFICATION_TIMES/(cal_p->signal_value[i+1] - cal_p->signal_value[i]);
        cal_p->b_value[i] = cal_p->gas_value[i+1] - cal_p->signal_value[i+1]*cal_p->k_value[i]/MAGNIFICATION_TIMES;
        LOG_I("k_value %d\r\nb_value %d\r\n",cal_p->k_value[i],cal_p->b_value[i]);
    }
}


/*零点信号自动更新*/
void fid_auto_update_zero(void)
{
    struct _calibration_point *calibration_point = NULL;
    calibration_point = &fid_calibration_point;
    if((om_get_ignite_step() != 7) && (om_thermocouple_read() > 3084))  /*点火流程判断*/
    {
        return;
    }
//    if((zore_signal_value == 0 )
//            &&(calibration_point->calibration_flag != 0)
//            &&(calibration_point->signal_value[1]  > om_get_ddc112_value()))
//    {
//        zore_signal_value = calibration_point->signal_value[0];
//        calibration_point->signal_value[0] = om_get_ddc112_value();
//        curve_calculate(calibration_point);
//
//    }

    if(((om_get_ddc112_value() + 50) < calibration_point->signal_value[0]))
    {
        calibration_point->signal_value[0] = calibration_point->signal_value[0] - 10;
        curve_calculate(calibration_point);
    }
}

/*冒泡排序算法*/
static void bubble_sort(uint32_t arr[],uint8_t size)
{
    uint8_t i,j;
    uint32_t temp;

    for(i=0;i<size - 1;i++)
    {
        uint8_t count = 0;
        for(j=0;j<size - 1 - i;j++)
        {
            if(arr[j] > arr[j+i])
            {
                temp = arr[j];
                arr[j] = arr[j+i];
                arr[j+i] = temp;
                count = 1;
            }
        }
        if(count == 0)
        {
            break;
        }
    }
}


static void curve_save(uint8_t curve_type)
{
    if(curve_type == FID_CURVE)
    {
        file_write(FID_CURVE_FILE,(uint8_t*)&fid_calibration_point,sizeof(fid_calibration_point));
    }
    else
    {
        file_write(PID_CURVE_FILE,(uint8_t*)&pid_calibration_point,sizeof(pid_calibration_point));
    }

    LOG_I("curve save write\r\n");
}

/*曲线清除
 * 更新情况：
 * 当有新的校准点时，曲线开始更新*/
void curve_reset(uint8_t curve_type)
{
    struct _calibration_point *temp_calibration_point = NULL;
    if(curve_type == FID_CURVE)
    {
        temp_calibration_point = &fid_calibration_point;
    }
    else
    {
        temp_calibration_point = &pid_calibration_point;
    }
    memset(temp_calibration_point ,0,sizeof(struct _calibration_point));
}
uint32_t get_curve_value(uint8_t index)
{
    struct _calibration_point *temp_calibration_point = NULL;
    temp_calibration_point = &fid_calibration_point;
    return temp_calibration_point->gas_value[index];
}
/*曲线更新*/
void curve_updata(uint8_t curve_type,uint32_t _gas_value,uint32_t _signal_value)
{
    struct _calibration_point *temp_calibration_point = NULL;
    if(curve_type == FID_CURVE)
    {
        temp_calibration_point = &fid_calibration_point;
    }
    else
    {
        temp_calibration_point = &pid_calibration_point;
    }

    for(int i=0;i<MAX_POINT_COUNT;i++)
    {
        if(_gas_value == temp_calibration_point->gas_value[i])
        {
//            if(i==0 && (temp_calibration_point->point_counts > 0))
//            {
//                /*曲线只能有一个零点，当出现第二个零点时，不进行校准*/
//                LOG_I("repeat calibration_point 0 \r\n");
//                return;
//            }
            if((i == 0) &&(temp_calibration_point->point_counts == 0))
            {
                            temp_calibration_point->gas_value[i] = _gas_value;
                            temp_calibration_point->signal_value[i] = _signal_value;
                            temp_calibration_point->point_counts+=1;
            }
            else
            {
                /*当新校准值已经存在时，曲线更新*/
                temp_calibration_point->gas_value[i] = _gas_value;
                temp_calibration_point->signal_value[i] = _signal_value;

                if(temp_calibration_point->point_counts < MIN_CURVE_CALCULATION_POINT)
                {
                    LOG_I("temp_calibration_point.point_counts < MIN_CURVE_CALCULATION_POINT\r\n");
                    return;
                }
                LOG_I("temp_calibration_point.point_counts %d\r\n",temp_calibration_point->point_counts);
                bubble_sort(temp_calibration_point->gas_value,temp_calibration_point->point_counts);
                bubble_sort(temp_calibration_point->signal_value,temp_calibration_point->point_counts);

                curve_calculate(temp_calibration_point);
                curve_save(curve_type);
                temp_calibration_point->calibration_flag = 1;
                return;
            }
            break;
        }
    }
    temp_calibration_point->gas_value[temp_calibration_point->point_counts] = _gas_value;
    temp_calibration_point->signal_value[temp_calibration_point->point_counts] = _signal_value;
    if(temp_calibration_point->point_counts < MAX_POINT_COUNT)
    {
        temp_calibration_point->point_counts+=1;
    }
    if(temp_calibration_point->point_counts < MIN_CURVE_CALCULATION_POINT)
    {
        LOG_I("temp_calibration_point.point_counts < MIN_CURVE_CALCULATION_POINT\r\n");
        return;
    }
    LOG_I("temp_calibration_point.point_counts %d\r\n",temp_calibration_point->point_counts);
    bubble_sort(temp_calibration_point->gas_value,temp_calibration_point->point_counts);
    bubble_sort(temp_calibration_point->signal_value,temp_calibration_point->point_counts);

    curve_calculate(temp_calibration_point);
    curve_save(curve_type);
    temp_calibration_point->calibration_flag = 1;
}

uint8_t get_detect_flag(uint8_t curve_type)
{
    struct _calibration_point *temp_calibration_point = NULL;
    if(curve_type == FID_CURVE)
    {
        temp_calibration_point = &fid_calibration_point;
    }
    else
    {
        temp_calibration_point = &pid_calibration_point;
    }
    return temp_calibration_point->calibration_flag;
}

uint32_t get_detect_value(uint8_t curve_type,uint32_t read_signal)
{
    uint8_t curve_index = 0;
    struct _calibration_point *temp_calibration_point = NULL;
    if(curve_type == FID_CURVE)
    {
        temp_calibration_point = &fid_calibration_point;
    }
    else
    {
        temp_calibration_point = &pid_calibration_point;
    }
    if(!temp_calibration_point->calibration_flag)
    {
        return 0;
    }
    /* 获取曲线索引 */
    if(read_signal < temp_calibration_point->signal_value[0])
    {
        return 0;
    }
    for(curve_index = 0;curve_index<temp_calibration_point->point_counts - 1;curve_index++)
    {
        if(read_signal < temp_calibration_point->signal_value[curve_index])
        {
            break;
        }
    }

    if(curve_index!=0)
    {
        curve_index = curve_index - 1;
    }

    return (uint32_t)((int64_t)(temp_calibration_point->k_value[curve_index]*read_signal*TOFIXED/MAGNIFICATION_TIMES + TOFIXED*temp_calibration_point->b_value[curve_index]));
}

static uint32_t g_om_fid_ppm_value = 0;
static uint32_t g_om_fid_ppm_value_buf[51] = {0};
static rt_uint8_t g_om_fid_ppm_read_count = 0;
static rt_uint8_t g_om_current_hardware_avg = 50;

static uint32_t g_om_other_sensor_value = 0;
static uint32_t g_om_other_sensor_value_buf[51] = {0};
static rt_uint8_t g_om_other_sensor_read_count = 0;

void ppm_calbarate(void)
{
    struct _calibration_point *temp_calibration_point = NULL;
    uint64_t total_fid_ppm_value = 0;
    uint64_t total_other_sensor_value = 0;

//    fid_auto_update_zero();
    temp_calibration_point = &fid_calibration_point;
    if(temp_calibration_point->calibration_flag)
    {
        g_om_fid_ppm_value_buf[g_om_fid_ppm_read_count] = get_detect_value(FID_CURVE,om_get_ddc112_value());
        if(g_om_fid_ppm_read_count < (g_om_current_hardware_avg - 1))
        {
            g_om_fid_ppm_read_count ++;
        }
        else
        {
            g_om_fid_ppm_read_count = 0;
        }
        for(int i = 0;i<g_om_current_hardware_avg;i++)
        {
            total_fid_ppm_value = total_fid_ppm_value + g_om_fid_ppm_value_buf[i];
        }
        g_om_fid_ppm_value = total_fid_ppm_value / (g_om_current_hardware_avg);
    }

    temp_calibration_point = &pid_calibration_point;
    if(temp_calibration_point->calibration_flag)
    {
        g_om_other_sensor_value_buf[g_om_other_sensor_read_count] = get_detect_value(PID_CURVE,om_get_pid_value());
        if(g_om_other_sensor_read_count < (g_om_current_hardware_avg - 1))
        {
            g_om_other_sensor_read_count ++;
        }
        else
        {
            g_om_other_sensor_read_count = 0;
        }
        for(int i = 0;i<g_om_current_hardware_avg;i++)
        {
            total_other_sensor_value = total_other_sensor_value + g_om_other_sensor_value_buf[i];
        }
        g_om_other_sensor_value = total_other_sensor_value / (g_om_current_hardware_avg);
    }
}
uint32_t get_fid_calbarate_value(void)
{
    return g_om_fid_ppm_value;
}

uint32_t get_othner_sensor_calbarate_value(void)
{
    return g_om_other_sensor_value;
}

int om_file_init(void)
{
    /*文件系统初始化，SPI flash挂载*/
    file_system_init();

    /*曲线文件打开，如果文件不存在，创建文件*/
    file_create(FID_CURVE_FILE);
    file_create(PID_CURVE_FILE);
    file_create(HISTORY_DATA_FILE);
    file_create(DECICE_SET_DATA_FILE);

    if(0)//(!file_is_exist(CURVE_FILE))
    {
        file_write(FID_CURVE_FILE,(uint8_t*)&fid_calibration_point,sizeof(fid_calibration_point));
        file_write(PID_CURVE_FILE,(uint8_t*)&pid_calibration_point,sizeof(pid_calibration_point));
        LOG_I("curve init write\r\n");
    }
    else{
        file_read(FID_CURVE_FILE,(uint8_t*)&fid_calibration_point,sizeof(fid_calibration_point));
        LOG_I("fid curve init read\r\n");
        file_read(PID_CURVE_FILE,(uint8_t*)&pid_calibration_point,sizeof(pid_calibration_point));
        LOG_I("pid curve init read\r\n");
    }

    return 0;
}

void om_device_set_data_save(uint8_t* set_data,uint8_t data_len)
{
    file_write(DECICE_SET_DATA_FILE,set_data,data_len);
}

void om_device_set_data_read(uint8_t* read_data,uint8_t data_len)
{
    file_read(DECICE_SET_DATA_FILE,read_data,data_len);
}



void om_history_data_write(uint8_t* set_fid_data,uint8_t data_len)
{
    file_write(HISTORY_DATA_FILE,(uint8_t*)&set_fid_data,data_len);
}

void om_history_data_read(uint8_t* fid_read_data,uint8_t data_len)
{
    file_read(HISTORY_DATA_FILE,(uint8_t*)&fid_read_data,data_len);
}

INIT_ENV_EXPORT(om_file_init);

