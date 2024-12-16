/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-01-13     Tery       the first version
 */

#include <rtthread.h>
#include <dfs_posix.h> /* 当需要使用文件操作时，需要包含这个头文件 */

#include "file_operate.h"
#include "dfs_port.h"

#define DBG_TAG "file_operate"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

void file_system_init(void)
{
    rt_hw_spi_flash_init();
    user_fal_init();
    dfs_mount_init();
}

/* 创建文件文件，返回文件操作符，否则返回*/
int file_create(const char *file_name)
{
    int fd;
    /* 打开/text.txt 作写入，如果该文件不存在则建立该文件*/
    fd = open(file_name, O_CREAT);
    if (fd < 0)
    {
        LOG_I("file create fail");
    }
    close(fd);
    return fd;
}

/*从文件中读取特定长度数据*/
int file_read(const char *file_name,uint8_t *buffer,uint16_t read_len)
{
    int fd;
    int length;

    /* 只写 &  打开 */
    fd = open(file_name, O_RDONLY, 0);
    if (fd < 0)
    {
        rt_kprintf("open file for read  failed\n");
        return -1;
    }

    /* 读取数据 */
    length = read(fd, buffer, read_len);
    if (length != read_len)
    {
        rt_kprintf("check: read file failed %d %d\n",length,read_len);
        close(fd);
        return -1;
    }
    return length;
}

/*从文件中写入特定长度数据*/
int file_write(const char *file_name,uint8_t *buffer,uint16_t write_len)
{
    int fd;
    int length;

    /* 只写 & 打开 */
    fd = open(file_name, O_WRONLY  | O_TRUNC, 0);
    if (fd < 0)
    {
        rt_kprintf("open file for write failed\n");
        return -1;
    }


    /* 写入数据 */
    length = write(fd, buffer, write_len);
    if (length != write_len)
    {
        rt_kprintf("write data failed\n");
        close(fd);
        return -1;
    }

    /* 关闭文件 */
    close(fd);

    return 1;
}

int file_is_exist(const char *file_name)
{
    struct stat buf;

    stat(file_name, &buf);

    rt_kprintf("text.txt file size = %d\n", buf.st_size);

    if(buf.st_size > 0)
    {
        return 1;
    }
    return 0;
}


