/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-01-13     Tery       the first version
 */
#ifndef APPLICATIONS_FILE_OPERATE_H_
#define APPLICATIONS_FILE_OPERATE_H_

void file_system_init(void);
int file_create(const char *file_name);
int file_read(const char *file_name,uint8_t *buffer,uint16_t read_len);
int file_write(const char *file_name,uint8_t *buffer,uint16_t write_len);
int file_is_exist(const char *file_name);
int file_lseek_read(const char *filename,uint8_t* buffer,uint16_t write_len,off_t offset);
int file_lseek_write(const char *filename,uint8_t* buffer,uint16_t write_len,off_t offset);

#endif /* APPLICATIONS_FILE_OPERATE_H_ */
