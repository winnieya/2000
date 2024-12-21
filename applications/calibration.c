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
#include <dfs_posix.h>

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
#define HISTORY_FID_CURVE_FILE    "/history_fid_curve_file.txt"
#define HISTORY_PID_CURVE_FILE    "/history_pid_curve_file.txt"
#define HISTORY_DATA_FILE    "/history_data_file.txt"
#define DECICE_SET_DATA_FILE    "/device_set_data_file.txt"

struct _calibration_point fid_calibration_point = {0};
struct _calibration_point pid_calibration_point = {0};

#ifdef SELF_CALIBRATION
calibration_index history_fid_curve_desc = {0};
calibration_index history_pid_curve_desc = {0};

#endif
uint8_t g_history_curve_update_flag = 0;

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
            if(arr[j] > arr[j+1])
            {
                temp = arr[j];
                arr[j] = arr[j+1];
                arr[j+1] = temp;
                count = 1;
            }
        }
        if(count == 0)
        {
            break;
        }
    }
}

#ifndef SELF_CALIBRATION
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
#else
static void curve_save(uint8_t curve_type,uint8_t calibration_type)
{
	off_t tmp_offset = 0;
	char filename[32];
	struct _calibration_point *tmp_calibration_p = NULL;
	struct _calibration_point curr_read_calibration = {0};
	calibration_index *calibration_desc_p = NULL;
    if(curve_type == FID_CURVE)
    {
    	strcpy(filename,HISTORY_FID_CURVE_FILE);
    	tmp_calibration_p = &fid_calibration_point;
		calibration_desc_p = &history_fid_curve_desc;
		file_write(FID_CURVE_FILE,(uint8_t*)&fid_calibration_point,sizeof(fid_calibration_point));
    }
    else
    {
    	strcpy(filename,HISTORY_PID_CURVE_FILE);
    	tmp_calibration_p = &pid_calibration_point;
		calibration_desc_p = &history_pid_curve_desc;
		file_write(PID_CURVE_FILE,(uint8_t*)&pid_calibration_point,sizeof(pid_calibration_point));
    }

	if(calibration_type == MANUAL_CALIBRATION){
		if(!g_history_curve_update_flag){
			g_history_curve_update_flag = 1;
			//save new curve
			
			if(calibration_desc_p->curve_index < CURVE_MAX_NUM)
	    	{
				calibration_desc_p->curve_index++;
			}else{
				calibration_desc_p->curve_index = 1;
			}
			if(calibration_desc_p->save_num < CURVE_MAX_NUM)
				calibration_desc_p->save_num++;
			file_lseek_write(filename,(uint8_t*)calibration_desc_p,sizeof(calibration_index),0);
		}
		//update curve
		tmp_offset = (sizeof(curr_read_calibration)*(calibration_desc_p->curve_index-1))+sizeof(calibration_index);
	    file_lseek_write(filename, (uint8_t*)tmp_calibration_p, sizeof(struct _calibration_point), tmp_offset);
	}

    LOG_I("curve save write\r\n");
}	
#endif

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
#ifdef SELF_CALIBRATION
int signal_value_is_valid(struct _calibration_point *calibration_point,uint32_t _gas_value,uint32_t _signal_value, uint8_t curve_type){
	int32_t max_gas = 0, min_gas = 0, tmp_gas = 0;
	struct _calibration_point read_calib = {0}, *tmp_calib = calibration_point;
	max_gas = (_gas_value*100+_gas_value*8);
	min_gas = (_gas_value*100-_gas_value*8);
	
	if(!tmp_calib->calibration_flag){
		if(history_fid_curve_desc.save_num == 0)
			return 0;
		if(curve_type == FID_CURVE){
			file_lseek_read(HISTORY_FID_CURVE_FILE, (uint8_t*)&read_calib, sizeof(read_calib), (sizeof(read_calib)*(history_fid_curve_desc.curve_index-1))+sizeof(calibration_index));
		}else{
			file_lseek_read(HISTORY_PID_CURVE_FILE, (uint8_t*)&read_calib, sizeof(read_calib), (sizeof(read_calib)*(history_pid_curve_desc.curve_index-1))+sizeof(calibration_index));
		}
		tmp_calib = &read_calib;

	}
	if(_gas_value == 0){
		for(int i = 1; i < tmp_calib->point_counts; i++){
			uint32_t tmp_signal = 0;
			if(tmp_calib->gas_value[i] < 100){
				continue;
			}
			tmp_signal = (uint32_t)((int64_t)(GAS_ONE_HUNDRED*TOFIXED-(tmp_calib->b_value[i-1]*TOFIXED))/TOFIXED*MAGNIFICATION_TIMES/tmp_calib->k_value[i-1]);
			if(_signal_value > tmp_signal || tmp_signal - _signal_value < 8000){
				return -1;
			}
			return 0;
		}

	}else{
		
		for(int i = 1; i < tmp_calib->point_counts; i++){
			if(_gas_value >= tmp_calib->gas_value[i-1] && _gas_value <= tmp_calib->gas_value[i-1]){
				tmp_gas = (uint32_t)((int64_t)(tmp_calib->k_value[i-1]*_signal_value*TOFIXED/MAGNIFICATION_TIMES + TOFIXED*tmp_calib->b_value[i-1]));
				break;
			}
			tmp_gas = (uint32_t)((int64_t)(tmp_calib->k_value[i-1]*_signal_value*TOFIXED/MAGNIFICATION_TIMES + TOFIXED*tmp_calib->b_value[i-1]));
		}
	
		LOG_I("range [%d-%d] set gas [%d]\r\n",min_gas/100, max_gas/100, tmp_gas/100);
		if(tmp_gas >= min_gas && tmp_gas <= max_gas){
			return 0;
		}else{
			return -1;
		}
	}
	return 0;
}

#endif

/*曲线更新*/
int curve_updata(uint8_t curve_type,uint32_t _gas_value,uint32_t _signal_value)
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
	/*先校准0ppm后才能校准其他ppm值*/
	if(_gas_value != 0 && temp_calibration_point->point_counts < 1){
		return 82;
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
            if(temp_calibration_point->signal_value[i] == 0 && i > temp_calibration_point->point_counts-1)
                break;
            if((i == 0) &&(temp_calibration_point->point_counts == 0))
            {
                            //temp_calibration_point->gas_value[i] = _gas_value;
                            //temp_calibration_point->signal_value[i] = _signal_value;
                            //temp_calibration_point->point_counts+=1;
                            break;
            }
            else
            {
            	if(signal_value_is_valid(temp_calibration_point, _gas_value, _signal_value, curve_type) < 0){
					return 83;
				}
                /*当新校准值已经存在时，曲线更新*/
                temp_calibration_point->gas_value[i] = _gas_value;
                temp_calibration_point->signal_value[i] = _signal_value;

                if(temp_calibration_point->point_counts < MIN_CURVE_CALCULATION_POINT)
                {
                    LOG_I("temp_calibration_point.point_counts < MIN_CURVE_CALCULATION_POINT\r\n");
                    return 0;
                }
                LOG_I("temp_calibration_point.point_counts %d\r\n",temp_calibration_point->point_counts);
                bubble_sort(temp_calibration_point->gas_value,temp_calibration_point->point_counts);
                bubble_sort(temp_calibration_point->signal_value,temp_calibration_point->point_counts);

                curve_calculate(temp_calibration_point);
				temp_calibration_point->calibration_flag = 1;
#ifndef SELF_CALIBRATION
                curve_save(curve_type);
#else
				curve_save(curve_type,MANUAL_CALIBRATION);
#endif   
                return 0;
            }
            break;
        }
    }
	if(signal_value_is_valid(temp_calibration_point, _gas_value, _signal_value, curve_type) < 0){
		return 83;
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
        return 0;
    }
    LOG_I("temp_calibration_point.point_counts %d\r\n",temp_calibration_point->point_counts);
    bubble_sort(temp_calibration_point->gas_value,temp_calibration_point->point_counts);
    bubble_sort(temp_calibration_point->signal_value,temp_calibration_point->point_counts);

    curve_calculate(temp_calibration_point);

    temp_calibration_point->calibration_flag = 1;

#ifndef SELF_CALIBRATION
	curve_save(curve_type);
#else
	curve_save(curve_type,MANUAL_CALIBRATION);
#endif
	return 0;
}

#ifdef SELF_CALIBRATION

void reconstruction_curr_curve(struct _calibration_point *calibration_point, calibration_index* calibration_desc)
{
	uint8_t i;
	//当前曲线无效
	if(!calibration_point->calibration_flag ){
		file_lseek_read(HISTORY_FID_CURVE_FILE, 
						 (uint8_t*)calibration_point, 
						 sizeof(struct _calibration_point), 
						 (sizeof(struct _calibration_point)*(calibration_desc->curve_index-1))+sizeof(calibration_index));
	}
	
	if(calibration_point->point_counts == MIN_CURVE_CALCULATION_POINT){
		calibration_point->gas_value[2] = calibration_point->gas_value[1];
		calibration_point->signal_value[2] = calibration_point->signal_value[1];
		calibration_point->k_value[1] = calibration_point->k_value[0];
		calibration_point->b_value[1] = calibration_point->b_value[0];
	}

	if(calibration_point->gas_value[2] == GAS_FIVE_HUNDRED){
		calibration_point->gas_value[1] = calibration_point->gas_value[2];
		calibration_point->signal_value[1] = calibration_point->signal_value[2];
		calibration_point->k_value[0] = calibration_point->k_value[1];
		calibration_point->b_value[0] = calibration_point->b_value[1];
	}
	for(i = 3; i < calibration_point->point_counts; i++){
		if(calibration_point->gas_value[i] == GAS_FIVE_HUNDRED){
			calibration_point->gas_value[1] = calibration_point->gas_value[i];
			calibration_point->signal_value[1] = calibration_point->signal_value[i];
			calibration_point->k_value[0] = calibration_point->k_value[i-1];
			calibration_point->b_value[0] = calibration_point->b_value[i-1];
			
		}else if(calibration_point->gas_value[i] == GAS_TEN_THOUSAND){
			calibration_point->gas_value[2] = calibration_point->gas_value[i];
			calibration_point->signal_value[2] = calibration_point->signal_value[i];
			calibration_point->k_value[2] = calibration_point->k_value[i-1];
			calibration_point->b_value[2] = calibration_point->b_value[i-1];
		}
		calibration_point->gas_value[i] = 0;
		calibration_point->signal_value[i] = 0;
		calibration_point->k_value[i-1] = 0;
		calibration_point->b_value[i-1] = 0;
	}
	if(calibration_point->gas_value[1] != GAS_FIVE_HUNDRED){
		//calibration_point->signal_value[1] = ((((calibration_point->signal_value[1] - calibration_point->signal_value[0])*100)/calibration_point->gas_value[1])*GAS_FIVE_HUNDRED)/100;
		calibration_point->signal_value[1] = (uint32_t)((int64_t)(GAS_FIVE_HUNDRED*TOFIXED-(calibration_point->b_value[0]*TOFIXED))/TOFIXED*MAGNIFICATION_TIMES/calibration_point->k_value[0]);
		calibration_point->gas_value[1] = GAS_FIVE_HUNDRED;
	}

	if(calibration_point->gas_value[2] != GAS_TEN_THOUSAND){
		//calibration_point->signal_value[2] = ((((calibration_point->signal_value[2] - calibration_point->signal_value[0])*100)/calibration_point->gas_value[2])*GAS_TEN_THOUSAND)/100;
		calibration_point->signal_value[2] = (uint32_t)((int64_t)(GAS_TEN_THOUSAND*TOFIXED-(calibration_point->b_value[1]*TOFIXED))/TOFIXED*MAGNIFICATION_TIMES/calibration_point->k_value[1]);
		calibration_point->gas_value[2] = GAS_TEN_THOUSAND;
	}
	calibration_point->point_counts = 3;
}

/*自动校准*/
void auto_curve_updata(uint8_t curve_type)
{
    struct _calibration_point *temp_calibration_point = NULL;
	struct _calibration_point read_calibration_point = {0};
	calibration_index *temp_calibration_desc = NULL;
	calibration_index read_calibration_desc = {0};
	signal_table tmp_signal = {0};
	int32_t ave_zero = 0, ave_five_hundred = 0, ave_ten_thousand = 0;
	int fd;
    int length;
	
    if(curve_type == FID_CURVE)
    {
        temp_calibration_point = &fid_calibration_point;
		temp_calibration_desc = &history_fid_curve_desc;
		fd = open(HISTORY_FID_CURVE_FILE, O_RDONLY, 0);
    }
    else
    {
        temp_calibration_point = &pid_calibration_point;
		temp_calibration_desc = &history_pid_curve_desc;
		fd = open(HISTORY_PID_CURVE_FILE, O_RDONLY, 0);
    }
	
    if (fd < 0)
    {
        rt_kprintf("auto_curve_updata open file failed\n");
        return;
    }

    /* 读取数据 */
    length = read(fd, (uint8_t *)&read_calibration_desc, sizeof(read_calibration_desc));
	if(length != sizeof(read_calibration_desc)){
		rt_kprintf("read calibration desc failed\n");
		close(fd);
		return;
	}
	if(read_calibration_desc.save_num < 2){
		rt_kprintf("Built-in curves less than 2 cannot be automatically calibrated\n");
		return;
	}
	rt_kprintf("curve_type = %d save number = %d current index = %d\n",curve_type,temp_calibration_desc->save_num,temp_calibration_desc->curve_index);

	for(int i = 0; i < temp_calibration_desc->save_num; i++){
		length = read(fd, (uint8_t *)&read_calibration_point, sizeof(read_calibration_point));
		if(length != sizeof(read_calibration_point)){
			rt_kprintf("lseek read data failed\n");
			close(fd);
			return;
		}

		if(!read_calibration_point.calibration_flag){
			continue;
		}
		
		for(int j = 0; j < read_calibration_point.point_counts; j++){
			if(read_calibration_point.gas_value[j] == GAS_ZERO){
				tmp_signal.zero_signal[tmp_signal.zero_num++] = read_calibration_point.signal_value[j];
			}else if(read_calibration_point.gas_value[j] == GAS_FIVE_HUNDRED){
				tmp_signal.five_hundred_signal[tmp_signal.five_hundred_num++] = read_calibration_point.signal_value[j];
			}else if(read_calibration_point.gas_value[j] == GAS_TEN_THOUSAND){
				tmp_signal.ten_thousand_signal[tmp_signal.ten_thousand_num++] = read_calibration_point.signal_value[j];
			}
			rt_kprintf("gas value = %d signal index = %d\n",read_calibration_point.gas_value[j],read_calibration_point.signal_value[j]);
		}				
	}
	
    /* 关闭文件 */
    close(fd);
	for(int i = 1; i < 10; i++){
		if(tmp_signal.zero_signal[i])
			ave_zero += (int32_t)(tmp_signal.zero_signal[i] - tmp_signal.zero_signal[i-1])/(tmp_signal.zero_num-1);
		if(tmp_signal.five_hundred_signal[i])
			ave_five_hundred += (int32_t)(tmp_signal.five_hundred_signal[i] - tmp_signal.five_hundred_signal[i-1])/(tmp_signal.five_hundred_num-1);
		if(tmp_signal.ten_thousand_signal[i])
			ave_ten_thousand += (int32_t)(tmp_signal.ten_thousand_signal[i] - tmp_signal.ten_thousand_signal[i-1])/(tmp_signal.ten_thousand_num-1);
	}
	reconstruction_curr_curve(temp_calibration_point, temp_calibration_desc);
	temp_calibration_point->calibration_flag = 0;
	temp_calibration_point->signal_value[0] += ave_zero;
	temp_calibration_point->signal_value[1] += ave_five_hundred;
	temp_calibration_point->signal_value[2] += ave_ten_thousand;
	
    curve_calculate(temp_calibration_point);
	temp_calibration_point->calibration_flag = 1;
    curve_save(curve_type,AUTO_CALIBRATION);
    
}

#endif
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

#ifndef SELF_CALIBRATION
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

#else
int om_file_init(void)
{
    /*文件系统初始化，SPI flash挂载*/
    file_system_init();
	//rm_fid_curve();
    /*曲线文件打开，如果文件不存在，创建文件*/
    file_create(FID_CURVE_FILE);
    file_create(PID_CURVE_FILE);
	file_create(HISTORY_FID_CURVE_FILE);
    file_create(HISTORY_PID_CURVE_FILE);
    file_create(HISTORY_DATA_FILE);
    file_create(DECICE_SET_DATA_FILE);

    file_read(HISTORY_FID_CURVE_FILE,(uint8_t*)&history_fid_curve_desc,sizeof(history_fid_curve_desc));
	file_read(FID_CURVE_FILE,(uint8_t*)&fid_calibration_point,sizeof(fid_calibration_point));
    LOG_I("fid curve init read\r\n");
    file_read(HISTORY_PID_CURVE_FILE,(uint8_t*)&history_pid_curve_desc,sizeof(history_pid_curve_desc));
	file_read(PID_CURVE_FILE,(uint8_t*)&pid_calibration_point,sizeof(pid_calibration_point));
    LOG_I("pid curve init read\r\n");
    return 0;
}

#if 1
void sread(void){
	int fd;
	struct _calibration_point usage = {0};
	calibration_index fid_calibration_desc = {0};
	
	fd = open(HISTORY_FID_CURVE_FILE, O_RDONLY,0);
	if(fd < 0){
		rt_kprintf("open file for lseek read failed\n");
		return;
	}
	read(fd, (char*)&fid_calibration_desc, sizeof(fid_calibration_desc));
	rt_kprintf("save number = %d current index = %d\n",fid_calibration_desc.save_num,fid_calibration_desc.curve_index);
    while(read(fd, (char*)&usage, sizeof(usage)) != 0){
		int i = 0;
		while(usage.signal_value[i]){
			rt_kprintf("gas_value %d  signal_value %d\n",usage.gas_value[i],usage.signal_value[i]);
			i++;
		}
    }

	close(fd);
	return;
}


MSH_CMD_EXPORT(sread,sread file);

void rm_fid_curve(void){
    int res = 0;
    int fd = 0;
    open("FID_CURVE_FILE", O_RDONLY);
    if (fd >= 0)
    {
        close(fd);
    }

    res = remove(FID_CURVE_FILE);
	if(res){
		rt_kprintf("delete %s fail  result = %d\n\r",FID_CURVE_FILE,res);
	}
	open(HISTORY_FID_CURVE_FILE, O_RDONLY);
    if (fd >= 0)
    {
        close(fd);
    }
	res = remove(HISTORY_FID_CURVE_FILE);
    if(res){
        rt_kprintf("delete %s fail  result = %d  errno = %d\n\r",HISTORY_FID_CURVE_FILE,res,errno);
    }
	return;
}


MSH_CMD_EXPORT(rm_fid_curve,rm_fid_curve file);

#endif

#endif

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

