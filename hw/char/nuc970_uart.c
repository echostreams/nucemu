/*
 *  NUC970 UART Emulation
 *
 *  Copyright (C) 2011 Samsung Electronics Co Ltd.
 *    Maksim Kozlov, <m.kozlov@samsung.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "chardev/char-fe.h"
#include "chardev/char-serial.h"

#include "hw/char/nuc970_uart.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"

#include "trace.h"
#include "qom/object.h"

 /*
  *  Offsets for UART registers relative to SFR base address
  *  for UARTn
  *
  */
#define ULCON      0x0000 /* Line Control             */
#define UCON       0x0004 /* Control                  */
#define UFCON      0x0008 /* FIFO Control             */
#define UMCON      0x000C /* Modem Control            */
#define UTRSTAT    0x0010 /* Tx/Rx Status             */
#define UERSTAT    0x0014 /* UART Error Status        */
#define UFSTAT     0x0018 /* FIFO Status              */
#define UMSTAT     0x001C /* Modem Status             */
#define UTXH       0x0020 /* Transmit Buffer          */
#define URXH       0x0024 /* Receive Buffer           */
#define UBRDIV     0x0028 /* Baud Rate Divisor        */
#define UFRACVAL   0x002C /* Divisor Fractional Value */
#define UINTP      0x0030 /* Interrupt Pending        */
#define UINTSP     0x0034 /* Interrupt Source Pending */
#define UINTM      0x0038 /* Interrupt Mask           */

#define UA_RBR     0x0000
#define UA_THR     0x0000
#define UA_IER     0x0004 // 04: UART Interrupt Enable Register
#define UA_FCR     0x0008 // 08: UART FIFO Control Register
#define UA_LCR     0x000c // 0c:
#define UA_MCR     0x0010 // addReg(0x10, "UA_MCR", 0x0000d200);
#define UA_MSR     0x0014 // addReg(0x14, "UA_MSR", 0x000001f0);
#define UA_FSR     0x0018 // 18
#define UA_ISR     0x001c // 1c: UART Interrupt Status Register
#define UA_TOR     0x0020 // Time-out Register [15:8] DLY, [7:0] TOIC
#define UA_BAUD    0x0024 // 24
#define UA_IRCR    0x0028 // 28
#define UA_ALT_CSR 0x002c // 2c
#define UA_FUN_SEL 0x0030 // 30
#define UA_LIN_CTL 0x0034 // 34
#define UA_LIN_SR  0x0038 // 38
#define UA_SC_CTL  0x0040 // 40
#define UA_SC_FST  0x0044 // 44

  /*
   * for indexing register in the uint32_t array
   *
   * 'reg' - register offset (see offsets definitions above)
   *
   */
#define I_(reg) (reg / sizeof(uint32_t))

typedef struct NUC970UartReg {
    const char* name; /* the only reason is the debug output */
    hwaddr  offset;
    uint32_t            reset_value;
} NUC970UartReg;

static const NUC970UartReg nuc970_uart_regs[] = {
    {"UA_RBR",   ULCON,    0x00000000},
    {"UA_IER",   UCON,     0x00000000},
    {"UFCON",    UFCON,    0x00000000},
    {"UMCON",    UMCON,    0x00000000},
    {"UTRSTAT",  UTRSTAT,  0x00000006}, /* RO */
    {"UERSTAT",  UERSTAT,  0x00000000}, /* RO */
    {"UFSTAT",   UFSTAT,   0x10404000}, /* 18: FIFO Status Register */
    {"UMSTAT",   UMSTAT,   0x00000002}, /* RO */
    {"UTXH",     UTXH,     0x00000000}, /* WO, undefined reset value*/
    {"URXH",     URXH,     0x0F000000}, /* RO */
    {"UBRDIV",   UBRDIV,   0x00000040},
    {"UFRACVAL", UFRACVAL, 0x0000000C},
    {"UINTP",    UINTP,    0x00000000},
    {"UINTSP",   UINTSP,   0x000c0000},
    {"UINTM",    UINTM,    0x00000000},
};

#define NUC970_UART_REGS_MEM_SIZE    0x3C

/* UART FIFO Control */
#define UFCON_FIFO_ENABLE                    0x1
#define UFCON_Rx_FIFO_RESET                  (1<<1)
#define UFCON_Tx_FIFO_RESET                  (1<<2)
#define UFCON_Tx_FIFO_TRIGGER_LEVEL_SHIFT    16
#define UFCON_Tx_FIFO_TRIGGER_LEVEL (0xf << UFCON_Tx_FIFO_TRIGGER_LEVEL_SHIFT)
#define UFCON_Rx_FIFO_TRIGGER_LEVEL_SHIFT    4
#define UFCON_Rx_FIFO_TRIGGER_LEVEL (0xf << UFCON_Rx_FIFO_TRIGGER_LEVEL_SHIFT)

/* Uart FIFO Status */
#define UFSTAT_Rx_FIFO_COUNT        0xff
#define UFSTAT_Rx_FIFO_FULL         0x100
#define UFSTAT_Rx_FIFO_ERROR        0x200
#define UFSTAT_Tx_FIFO_COUNT_SHIFT  16
#define UFSTAT_Tx_FIFO_COUNT        (0xff << UFSTAT_Tx_FIFO_COUNT_SHIFT)
#define UFSTAT_Tx_FIFO_FULL_SHIFT   24
#define UFSTAT_Tx_FIFO_FULL         (1 << UFSTAT_Tx_FIFO_FULL_SHIFT)

/* UART Interrupt Source Pending */
#define UINTSP_RXD      0x1 /* Receive interrupt  */
#define UINTSP_ERROR    0x2 /* Error interrupt    */
#define UINTSP_TXD      0x4 /* Transmit interrupt */
#define UINTSP_MODEM    0x8 /* Modem interrupt    */

/* UART Line Control */
#define ULCON_IR_MODE_SHIFT   6
#define ULCON_PARITY_SHIFT    3
#define ULCON_STOP_BIT_SHIFT  1

/* UART Tx/Rx Status */
#define UTRSTAT_Rx_TIMEOUT              0x8
#define UTRSTAT_TRANSMITTER_EMPTY       0x4
#define UTRSTAT_Tx_BUFFER_EMPTY         0x2
#define UTRSTAT_Rx_BUFFER_DATA_READY    0x1

/* UART Error Status */
#define UERSTAT_OVERRUN  0x1
#define UERSTAT_PARITY   0x2
#define UERSTAT_FRAME    0x4
#define UERSTAT_BREAK    0x8

typedef struct {
    uint8_t* data;
    uint32_t    sp, rp; /* store and retrieve pointers */
    uint32_t    size;
} NUC970UartFIFO;

#define TYPE_NUC970_UART "nuc970.uart"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970UartState, NUC970_UART)

struct NUC970UartState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint32_t             reg[NUC970_UART_REGS_MEM_SIZE / sizeof(uint32_t)];
    NUC970UartFIFO   rx;
    NUC970UartFIFO   tx;

    QEMUTimer* fifo_timeout_timer;
    uint64_t wordtime;        /* word time in ns */

    CharBackend       chr;
    qemu_irq          irq;
    qemu_irq          dmairq;

    uint32_t channel;

};


/* Used only for tracing */
static const char* nuc970_uart_regname(hwaddr  offset)
{

    int i;

    for (i = 0; i < ARRAY_SIZE(nuc970_uart_regs); i++) {
        if (offset == nuc970_uart_regs[i].offset) {
            return nuc970_uart_regs[i].name;
        }
    }

    return NULL;
}


static void fifo_store(NUC970UartFIFO* q, uint8_t ch)
{
    q->data[q->sp] = ch;
    q->sp = (q->sp + 1) % q->size;
}

static uint8_t fifo_retrieve(NUC970UartFIFO* q)
{
    uint8_t ret = q->data[q->rp];
    q->rp = (q->rp + 1) % q->size;
    return  ret;
}

static int fifo_elements_number(const NUC970UartFIFO* q)
{
    if (q->sp < q->rp) {
        return q->size - q->rp + q->sp;
    }

    return q->sp - q->rp;
}

static int fifo_empty_elements_number(const NUC970UartFIFO* q)
{
    return q->size - fifo_elements_number(q);
}

static void fifo_reset(NUC970UartFIFO* q)
{
    g_free(q->data);
    q->data = NULL;

    q->data = (uint8_t*)g_malloc0(q->size);

    q->sp = 0;
    q->rp = 0;
}

static uint32_t nuc970_uart_FIFO_trigger_level(uint32_t channel,
    uint32_t reg)
{
    uint32_t level;

    switch (channel) {
    case 0:
        level = reg * 32;
        break;
    case 1:
    case 4:
        level = reg * 8;
        break;
    case 2:
    case 3:
        level = reg * 2;
        break;
    default:
        level = 0;
        trace_exynos_uart_channel_error(channel);
        break;
    }
    return level;
}

static uint32_t
nuc970_uart_Tx_FIFO_trigger_level(const NUC970UartState* s)
{
    uint32_t reg;

    reg = (s->reg[I_(UFCON)] & UFCON_Tx_FIFO_TRIGGER_LEVEL) >>
        UFCON_Tx_FIFO_TRIGGER_LEVEL_SHIFT;

    return nuc970_uart_FIFO_trigger_level(s->channel, reg);
}

static uint32_t
nuc970_uart_Rx_FIFO_trigger_level(const NUC970UartState* s)
{
    uint32_t reg;

    reg = ((s->reg[I_(UFCON)] & UFCON_Rx_FIFO_TRIGGER_LEVEL) >>
        UFCON_Rx_FIFO_TRIGGER_LEVEL_SHIFT) + 1;

    return nuc970_uart_FIFO_trigger_level(s->channel, reg);
}

/*
 * Update Rx DMA busy signal if Rx DMA is enabled. For simplicity,
 * mark DMA as busy if DMA is enabled and the receive buffer is empty.
 */
static void nuc970_uart_update_dmabusy(NUC970UartState* s)
{
    bool rx_dma_enabled = (s->reg[I_(UCON)] & 0x03) == 0x02;
    uint32_t count = fifo_elements_number(&s->rx);

    if (rx_dma_enabled && !count) {
        qemu_irq_raise(s->dmairq);
        trace_exynos_uart_dmabusy(s->channel);
    }
    else {
        qemu_irq_lower(s->dmairq);
        trace_exynos_uart_dmaready(s->channel);
    }
}

static void nuc970_uart_update_irq(NUC970UartState* s)
{
    /*
     * The Tx interrupt is always requested if the number of data in the
     * transmit FIFO is smaller than the trigger level.
     */
    if (s->reg[I_(UFCON)] & UFCON_FIFO_ENABLE) {
        uint32_t count = (s->reg[I_(UFSTAT)] & UFSTAT_Tx_FIFO_COUNT) >>
            UFSTAT_Tx_FIFO_COUNT_SHIFT;

        if (count <= nuc970_uart_Tx_FIFO_trigger_level(s)) {
            s->reg[I_(UINTSP)] |= UINTSP_TXD;
        }

        /*
         * Rx interrupt if trigger level is reached or if rx timeout
         * interrupt is disabled and there is data in the receive buffer
         */
        count = fifo_elements_number(&s->rx);
        if ((count && !(s->reg[I_(UCON)] & 0x80)) ||
            count >= nuc970_uart_Rx_FIFO_trigger_level(s)) {
            nuc970_uart_update_dmabusy(s);
            s->reg[I_(UINTSP)] |= UINTSP_RXD;
            timer_del(s->fifo_timeout_timer);
        }
    }
    else if (s->reg[I_(UTRSTAT)] & UTRSTAT_Rx_BUFFER_DATA_READY) {
        nuc970_uart_update_dmabusy(s);
        s->reg[I_(UINTSP)] |= UINTSP_RXD;
    }

    s->reg[I_(UINTP)] = s->reg[I_(UINTSP)] & ~s->reg[I_(UINTM)];

    if (s->reg[I_(UINTP)]) {
        qemu_irq_raise(s->irq);
        trace_exynos_uart_irq_raised(s->channel, s->reg[I_(UINTP)]);
    }
    else {
        qemu_irq_lower(s->irq);
        trace_exynos_uart_irq_lowered(s->channel);
    }
}

static void nuc970_uart_timeout_int(void* opaque)
{
    NUC970UartState* s = opaque;

    trace_exynos_uart_rx_timeout(s->channel, s->reg[I_(UTRSTAT)],
        s->reg[I_(UINTSP)]);

    if ((s->reg[I_(UTRSTAT)] & UTRSTAT_Rx_BUFFER_DATA_READY) ||
        (s->reg[I_(UCON)] & (1 << 11))) {
        s->reg[I_(UINTSP)] |= UINTSP_RXD;
        s->reg[I_(UTRSTAT)] |= UTRSTAT_Rx_TIMEOUT;
        nuc970_uart_update_dmabusy(s);
        nuc970_uart_update_irq(s);
    }
}

static void nuc970_uart_update_parameters(NUC970UartState* s)
{
    int speed, parity, data_bits, stop_bits;
    QEMUSerialSetParams ssp;
    uint64_t uclk_rate;

    if (s->reg[I_(UBRDIV)] == 0) {
        return;
    }

    if (s->reg[I_(ULCON)] & 0x20) {
        if (s->reg[I_(ULCON)] & 0x28) {
            parity = 'E';
        }
        else {
            parity = 'O';
        }
    }
    else {
        parity = 'N';
    }

    if (s->reg[I_(ULCON)] & 0x4) {
        stop_bits = 2;
    }
    else {
        stop_bits = 1;
    }

    data_bits = (s->reg[I_(ULCON)] & 0x3) + 5;

    uclk_rate = 24000000;

    speed = uclk_rate / ((16 * (s->reg[I_(UBRDIV)]) & 0xffff) +
        (s->reg[I_(UFRACVAL)] & 0x7) + 16);

    ssp.speed = speed;
    ssp.parity = parity;
    ssp.data_bits = data_bits;
    ssp.stop_bits = stop_bits;

    s->wordtime = NANOSECONDS_PER_SECOND * (data_bits + stop_bits + 1) / speed;

    qemu_chr_fe_ioctl(&s->chr, CHR_IOCTL_SERIAL_SET_PARAMS, &ssp);

    trace_exynos_uart_update_params(
        s->channel, speed, parity, data_bits, stop_bits, s->wordtime);
}

static void nuc970_uart_rx_timeout_set(NUC970UartState* s)
{
    if (s->reg[I_(UCON)] & 0x80) {
        uint32_t timeout = ((s->reg[I_(UCON)] >> 12) & 0x0f) * s->wordtime;

        timer_mod(s->fifo_timeout_timer,
            qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) + timeout);
    }
    else {
        timer_del(s->fifo_timeout_timer);
    }
}

static void nuc970_uart_write(void* opaque, hwaddr offset,
    uint64_t val, unsigned size)
{
    NUC970UartState* s = (NUC970UartState*)opaque;
    Chardev* chr = qemu_chr_fe_get_driver(&s->chr);
    uint8_t ch;

    DPRINTF("write(offset=0x%" HWADDR_PRIx ", value = 0x%x) to %s\n",
        offset, (unsigned int)val, chr ? chr->label : "NODEV");

    trace_exynos_uart_write(s->channel, offset,
        nuc970_uart_regname(offset), val);

    switch (offset) {
    //case ULCON:
    case UBRDIV:
    case UFRACVAL:
        s->reg[I_(offset)] = val;
        nuc970_uart_update_parameters(s);
        break;
    case UFCON:
        s->reg[I_(UFCON)] = val;
        if (val & UFCON_Rx_FIFO_RESET) {
            fifo_reset(&s->rx);
            s->reg[I_(UFCON)] &= ~UFCON_Rx_FIFO_RESET;
            trace_exynos_uart_rx_fifo_reset(s->channel);
        }
        if (val & UFCON_Tx_FIFO_RESET) {
            fifo_reset(&s->tx);
            s->reg[I_(UFCON)] &= ~UFCON_Tx_FIFO_RESET;
            trace_exynos_uart_tx_fifo_reset(s->channel);
        }
        break;

    case 0x0:
        if (qemu_chr_fe_backend_connected(&s->chr)) {
            s->reg[I_(UTRSTAT)] &= ~(UTRSTAT_TRANSMITTER_EMPTY |
                UTRSTAT_Tx_BUFFER_EMPTY);
            ch = (uint8_t)val;
            /* XXX this blocks entire thread. Rewrite to use
             * qemu_chr_fe_write and background I/O callbacks */
            qemu_chr_fe_write_all(&s->chr, &ch, 1);
            trace_exynos_uart_tx(s->channel, ch);
            s->reg[I_(UTRSTAT)] |= UTRSTAT_TRANSMITTER_EMPTY |
                UTRSTAT_Tx_BUFFER_EMPTY;
            s->reg[I_(UINTSP)] |= UINTSP_TXD;
            nuc970_uart_update_irq(s);
        }
        break;

    case UINTP:
        s->reg[I_(UINTP)] &= ~val;
        s->reg[I_(UINTSP)] &= ~val;
        trace_exynos_uart_intclr(s->channel, s->reg[I_(UINTP)]);
        nuc970_uart_update_irq(s);
        break;
    case UTRSTAT:
        if (val & UTRSTAT_Rx_TIMEOUT) {
            s->reg[I_(UTRSTAT)] &= ~UTRSTAT_Rx_TIMEOUT;
        }
        break;
    case UERSTAT:
    case UFSTAT:
    case UMSTAT:
    case URXH:
        trace_exynos_uart_ro_write(
            s->channel, nuc970_uart_regname(offset), offset);
        break;
    case UINTSP:
        s->reg[I_(UINTSP)] &= ~val;
        break;
    case UINTM:
        s->reg[I_(UINTM)] = val;
        nuc970_uart_update_irq(s);
        break;
    case UCON:
    case UMCON:
    default:
        s->reg[I_(offset)] = val;
        break;
    }
}

static uint64_t nuc970_uart_read(void* opaque, hwaddr offset,
    unsigned size)
{
    NUC970UartState* s = (NUC970UartState*)opaque;
    uint32_t res;

    DPRINTF("read(offset=0x%" HWADDR_PRIx ")\n", offset);

    switch (offset) {
    case UERSTAT: /* Read Only */
        res = s->reg[I_(UERSTAT)];
        s->reg[I_(UERSTAT)] = 0;
        trace_exynos_uart_read(s->channel, offset,
            nuc970_uart_regname(offset), res);
        return res;
    case UFSTAT: /* FSR */
        res = 0;
        {
            int tx_fill = fifo_elements_number(&s->tx);
            int rx_fill = fifo_elements_number(&s->rx);
            if (rx_fill <= 0) {
                res |= 1 << 14; // RX_EMPTY;
            }
            if (tx_fill <= 0) {
                res |= 1 << 22; // TX_EMPTY: the last byte of TXFIFO has been transferred to Transmitter Shift Register
                res |= 1 << 28; // TE_FLAG: the STOP bit of the last byte has been transmitted.
            }
            res |= (tx_fill & 0x3f) << 16;	// TX_POINTER
            res |= (rx_fill & 0x3f) << 8;	// RX_POINTER
            
            if (fifo_empty_elements_number(&s->rx) == 0) {
                res |= 1 << 15; // RX_FULL;
            }
            if (fifo_empty_elements_number(&s->tx) == 0) {
                res |= 1 << 23; // TX_FULL;
            }


        }
        trace_exynos_uart_read(s->channel, offset,
            nuc970_uart_regname(offset),
            res);
        return res;
    case 0://URXH:
        //if (s->reg[I_(UFCON)] & UFCON_FIFO_ENABLE) {
        if (1) {
            if (fifo_elements_number(&s->rx)) {
                res = fifo_retrieve(&s->rx);
                trace_exynos_uart_rx(s->channel, res);
                if (!fifo_elements_number(&s->rx)) {
                    s->reg[I_(UTRSTAT)] &= ~UTRSTAT_Rx_BUFFER_DATA_READY;
                }
                else {
                    s->reg[I_(UTRSTAT)] |= UTRSTAT_Rx_BUFFER_DATA_READY;
                }
            }
            else {
                trace_exynos_uart_rx_error(s->channel);
                s->reg[I_(UINTSP)] |= UINTSP_ERROR;
                nuc970_uart_update_irq(s);
                res = 0;
            }
        }
        else {
            s->reg[I_(UTRSTAT)] &= ~UTRSTAT_Rx_BUFFER_DATA_READY;
            res = s->reg[I_(URXH)];
        }
        qemu_chr_fe_accept_input(&s->chr);
        nuc970_uart_update_dmabusy(s);
        trace_exynos_uart_read(s->channel, offset,
            nuc970_uart_regname(offset), res);
        return res;
    case UTXH:
        trace_exynos_uart_wo_read(s->channel, nuc970_uart_regname(offset),
            offset);
        break;
    default:
        trace_exynos_uart_read(s->channel, offset,
            nuc970_uart_regname(offset),
            s->reg[I_(offset)]);
        return s->reg[I_(offset)];
    }

    trace_exynos_uart_read(s->channel, offset, nuc970_uart_regname(offset),
        0);
    return 0;
}

static const MemoryRegionOps nuc970_uart_ops = {
    .read = nuc970_uart_read,
    .write = nuc970_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .max_access_size = 4,
        .unaligned = false
    },
};

static int nuc970_uart_can_receive(void* opaque)
{
    NUC970UartState* s = (NUC970UartState*)opaque;

    if (s->reg[I_(UFCON)] & UFCON_FIFO_ENABLE) {
        return fifo_empty_elements_number(&s->rx);
    }
    else {
        return !(s->reg[I_(UTRSTAT)] & UTRSTAT_Rx_BUFFER_DATA_READY);
    }
}

static void nuc970_uart_receive(void* opaque, const uint8_t* buf, int size)
{
    NUC970UartState* s = (NUC970UartState*)opaque;
    int i;

    //if (s->reg[I_(UFCON)] & UFCON_FIFO_ENABLE) {
    if (1) {
        if (fifo_empty_elements_number(&s->rx) < size) {
            size = fifo_empty_elements_number(&s->rx);
            s->reg[I_(UINTSP)] |= UINTSP_ERROR;
        }
        for (i = 0; i < size; i++) {
            fifo_store(&s->rx, buf[i]);
        }
        nuc970_uart_rx_timeout_set(s);
    }
    else {
        s->reg[I_(URXH)] = buf[0];
    }
    s->reg[I_(UTRSTAT)] |= UTRSTAT_Rx_BUFFER_DATA_READY;

    nuc970_uart_update_irq(s);
}


static void nuc970_uart_event(void* opaque, QEMUChrEvent event)
{
    NUC970UartState* s = (NUC970UartState*)opaque;

    if (event == CHR_EVENT_BREAK) {
        /* When the RxDn is held in logic 0, then a null byte is pushed into the
         * fifo */
        fifo_store(&s->rx, '\0');
        s->reg[I_(UERSTAT)] |= UERSTAT_BREAK;
        nuc970_uart_update_irq(s);
    }
}


static void nuc970_uart_reset(DeviceState* dev)
{
    NUC970UartState* s = NUC970_UART(dev);
    int i;

    for (i = 0; i < ARRAY_SIZE(nuc970_uart_regs); i++) {
        s->reg[I_(nuc970_uart_regs[i].offset)] =
            nuc970_uart_regs[i].reset_value;
    }

    fifo_reset(&s->rx);
    fifo_reset(&s->tx);

    trace_exynos_uart_rxsize(s->channel, s->rx.size);
}

static int nuc970_uart_post_load(void* opaque, int version_id)
{
    NUC970UartState* s = (NUC970UartState*)opaque;

    nuc970_uart_update_parameters(s);
    nuc970_uart_rx_timeout_set(s);

    return 0;
}

static const VMStateDescription vmstate_nuc970_uart_fifo = {
    .name = "nuc970.uart.fifo",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(sp, NUC970UartFIFO),
        VMSTATE_UINT32(rp, NUC970UartFIFO),
        VMSTATE_VBUFFER_UINT32(data, NUC970UartFIFO, 1, NULL, size),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_nuc970_uart = {
    .name = "nuc970.uart",
    .version_id = 1,
    .minimum_version_id = 1,
    .post_load = nuc970_uart_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT(rx, NUC970UartState, 1,
                       vmstate_nuc970_uart_fifo, NUC970UartFIFO),
        VMSTATE_UINT32_ARRAY(reg, NUC970UartState,
                             NUC970_UART_REGS_MEM_SIZE / sizeof(uint32_t)),
        VMSTATE_END_OF_LIST()
    }
};

DeviceState* nuc970_uart_create(hwaddr addr,
    int fifo_size,
    int channel,
    Chardev* chr,
    qemu_irq irq)
{
    DeviceState* dev;
    SysBusDevice* bus;

    dev = qdev_new(TYPE_NUC970_UART);

    qdev_prop_set_chr(dev, "chardev", chr);
    qdev_prop_set_uint32(dev, "channel", channel);
    qdev_prop_set_uint32(dev, "rx-size", fifo_size);
    qdev_prop_set_uint32(dev, "tx-size", fifo_size);

    bus = SYS_BUS_DEVICE(dev);
    sysbus_realize_and_unref(bus, &error_fatal);
    if (addr != (hwaddr)-1) {
        sysbus_mmio_map(bus, 0, addr);
    }
    sysbus_connect_irq(bus, 0, irq);

    return dev;
}

static void nuc970_uart_init(Object* obj)
{
    SysBusDevice* dev = SYS_BUS_DEVICE(obj);
    NUC970UartState* s = NUC970_UART(dev);

    s->wordtime = NANOSECONDS_PER_SECOND * 10 / 9600;

    /* memory mapping */
    memory_region_init_io(&s->iomem, obj, &nuc970_uart_ops, s,
        "nuc970.uart", NUC970_UART_REGS_MEM_SIZE);
    sysbus_init_mmio(dev, &s->iomem);

    sysbus_init_irq(dev, &s->irq);
    sysbus_init_irq(dev, &s->dmairq);
}

static void nuc970_uart_realize(DeviceState* dev, Error** errp)
{
    NUC970UartState* s = NUC970_UART(dev);

    s->fifo_timeout_timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
        nuc970_uart_timeout_int, s);

    qemu_chr_fe_set_handlers(&s->chr, nuc970_uart_can_receive,
        nuc970_uart_receive, nuc970_uart_event,
        NULL, s, NULL, true);
}

static Property nuc970_uart_properties[] = {
    DEFINE_PROP_CHR("chardev", NUC970UartState, chr),
    DEFINE_PROP_UINT32("channel", NUC970UartState, channel, 0),
    DEFINE_PROP_UINT32("rx-size", NUC970UartState, rx.size, 16),
    DEFINE_PROP_UINT32("tx-size", NUC970UartState, tx.size, 16),
    DEFINE_PROP_END_OF_LIST(),
};

static void nuc970_uart_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->realize = nuc970_uart_realize;
    dc->reset = nuc970_uart_reset;
    device_class_set_props(dc, nuc970_uart_properties);
    dc->vmsd = &vmstate_nuc970_uart;
}

static const TypeInfo nuc970_uart_info = {
    .name = TYPE_NUC970_UART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC970UartState),
    .instance_init = nuc970_uart_init,
    .class_init = nuc970_uart_class_init,
};

static void nuc970_uart_register(void)
{
    type_register_static(&nuc970_uart_info);
}

type_init(nuc970_uart_register)