/*
 * Device model for NUC970 UART
 *
 * Copyright (c) 2008 OKL
 * Originally Written by Hans Jiang
 * Copyright (c) 2011 NICTA Pty Ltd.
 * Updated by Jean-Christophe Dubois <jcd@tribudubois.net>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef NUC970_UART_H
#define NUC970_UART_H

#include "hw/sysbus.h"
#include "chardev/char-fe.h"
#include "qom/object.h"

#ifndef DEBUG_NUC970_UART
#define DEBUG_NUC970_UART 0
#endif

#define DPRINTF(fmt, args...) \
    do { \
        if (DEBUG_NUC970_UART) { \
            fprintf(stderr, "[%s]%s: " fmt , "nuc970.uart", \
                                             __func__, ##args); \
        } \
    } while (0)

 /*
  * NUC970 UART
  */
DeviceState* nuc970_uart_create(hwaddr addr,
    int fifo_size,
    int channel,
    Chardev* chr,
    qemu_irq irq);

#endif
