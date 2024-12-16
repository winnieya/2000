/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-01-12     Tery       the first version
 */
#ifndef APPLICATIONS_DFS_PORT_H_
#define APPLICATIONS_DFS_PORT_H_

int rt_hw_spi_flash_init(void);
int user_fal_init(void);
int dfs_mount_init(void);

#endif /* APPLICATIONS_DFS_PORT_H_ */
