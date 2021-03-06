/*
 * include/linux/wakeup_reason.h
 *
 * Logs the reason which caused the kernel to resume
 * from the suspend mode.
 *
 * Copyright (C) 2014 Google, Inc.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _LINUX_WAKEUP_REASON_H
#define _LINUX_WAKEUP_REASON_H

#include <linux/types.h>

void log_wakeup_reason(int irq);
const int* get_wakeup_reasons(size_t *len);
void clear_wakeup_reasons(void);

#endif /* _LINUX_WAKEUP_REASON_H */
