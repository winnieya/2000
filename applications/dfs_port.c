/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-01-12     Tery       the first version
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include <fal.h>
#include <dfs_fs.h>

#define DBG_TAG "dfs_port"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include "dfs_port.h"
#include "drv_spi.h"
#include "spi_flash_sfud.h"



rt_spi_flash_device_t norflash0 = NULL;
#define FS_PARTITION_NAME "filesystem"

int rt_hw_spi_flash_init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    rt_hw_spi_device_attach("spi2", "spi20", GPIOB, GPIO_PIN_12);

    if(rt_sfud_flash_probe("norflash0","spi20"))
    {
        rt_kprintf("rt sfud flash probe seccess \r\n");
        return RT_EOK;

    }
    else {
        rt_kprintf("rt sfud flash probe fail \r\n");
        return -RT_ERROR;
    }
}

int user_fal_init(void)
{
    fal_init();

    return 0;
}


int dfs_mount_init(void)
{
    /* 在 spi flash 中名为 "filesystem" 的分区上创建一个块设备 */
    struct rt_device *flash_dev = fal_blk_device_create(FS_PARTITION_NAME);
    if (flash_dev == NULL)
    {
        LOG_E("Can't create a block device on '%s' partition.", FS_PARTITION_NAME);
    }
    else
    {
        LOG_D("Create a block device on the %s partition of flash successful.", FS_PARTITION_NAME);
    }

    /* 挂载 spi flash 中名为 "filesystem" 的分区上的文件系统 */
    if (dfs_mount(flash_dev->parent.name, "/", "elm", 0, 0) == 0)
    {
        LOG_I("Filesystem initialized!");
    }
    else /*如果挂载失败，格式化文件系统类型，重新挂载*/
    {
        dfs_mkfs("elm", flash_dev->parent.name);
        if (dfs_mount(flash_dev->parent.name, "/", "elm", 0, 0) == 0)
        {
            LOG_I("Filesystem initialized!");
        }
        else {
            LOG_E("Failed to initialize filesystem!");
            LOG_D("You should create a filesystem on the block device first!");
        }
    }
    return 0;
}


