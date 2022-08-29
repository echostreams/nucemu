/*
 *  NUC970 UART Emulation
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

#define UART_REG_RBR	0x00
#define UART_REG_THR	0x00

#define UART_REG_IER	0x04
#define RDA_IEN			0x00000001
#define THRE_IEN		0x00000002
#define RTO_IEN			0x00000010
#define TIME_OUT_EN		0x00000800

#define UART_REG_FCR	0x08
#define RFR			0x00000002
#define TFR			0x00000004

#define UART_REG_LCR	0x0C
#define	NSB			0x00000004
#define PBE			0x00000008
#define EPE			0x00000010
#define SPE			0x00000020
#define BCB			0x00000040

#define UART_REG_MCR	0x10
#define UART_REG_MSR	0x14

#define UART_REG_FSR	0x18
#define RX_OVER_IF		0x00000001
#define PEF			0x00000010
#define FEF			0x00000020
#define BIF			0x00000040
#define RX_EMPTY		0x00004000
#define TX_EMPTY		0x00400000
#define TX_FULL			0x00800000
#define RX_FULL			0x00008000
#define TE_FLAG			0x10000000

#define UART_REG_ISR	0x1C
#define RDA_IF			0x00000001
#define THRE_IF			0x00000002
#define TOUT_IF			0x00000010
#define THRE_INT		0x00000200

#define UART_REG_TOR	0x20
#define UART_REG_BAUD	0x24

#define UART_REG_IRCR	0x28

#define UART_REG_ALT_CSR 0x2C

#define UART_FUN_SEL    0x30
#define FUN_SEL_UART    0x00000000
#define FUN_SEL_LIN     0x00000001
#define FUN_SEL_IrDA    0x00000002
#define FUN_SEL_RS485	0x00000003
#define FUN_SEL_Msk		0x00000007

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

/* modem lines */
/*
	    TIOCM_LE    DSR (data set ready/line enable)
	    TIOCM_DTR   DTR (data terminal ready)
	    TIOCM_RTS   RTS (request to send)
	    TIOCM_ST    Secondary TXD (transmit)
	    TIOCM_SR    Secondary RXD (receive)
	    TIOCM_CTS   CTS (clear to send)
	    TIOCM_CAR   DCD (data carrier detect)
	    TIOCM_CD    see TIOCM_CAR
	    TIOCM_RNG   RNG (ring)
	    TIOCM_RI    see TIOCM_RNG
	    TIOCM_DSR   DSR (data set ready)
*/
#define TIOCM_LE	0x001	// DCTSF?
#define TIOCM_DTR	0x002	
#define TIOCM_RTS	0x004
#define TIOCM_ST	0x008
#define TIOCM_SR	0x010
#define TIOCM_CTS	0x020	// 5
#define TIOCM_CAR	0x040	// 6
#define TIOCM_RNG	0x080	// 7
#define TIOCM_DSR	0x100
#define TIOCM_CD	TIOCM_CAR
#define TIOCM_RI	TIOCM_RNG
#define TIOCM_OUT1	0x2000
#define TIOCM_OUT2	0x4000
#define TIOCM_LOOP	0x8000

static const NUC970UartReg nuc970_uart_regs[] = {
	{"UA_RBR",   ULCON,    0x00000000},
	{"UA_IER",   UCON,     0x00000000},
	{"UA_FCR",   UFCON,    0x00000000},
	{"UA_LCR",   UMCON,    0x00000000},
	{"UA_MCR",   UTRSTAT,  0x0000f202},
	{"UA_MSR",   UA_MSR,   0x000001e0},
	{"UA_FSR",   UFSTAT,   0x10404000}, /* 18: FIFO Status Register */
	{"UA_ISR",   UMSTAT,   0x00000002}, /* RO */
	{"UA_TOR",   UTXH,     0x00000000}, /* WO, undefined reset value*/
	{"UA_BAUD",  URXH,     0x0F000000}, /* RO */
	{"UA_IRCR",  UBRDIV,   0x00000040},
	{"UA_ACSR",  UFRACVAL, 0x0000000C},
	{"UA_FSEL",  UINTP,    0x00000000},
	{"UA_LCTL",  UINTSP,   0x000c0000},
	{"UA_LISR",  UINTM,    0x00000000},
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

/*
 * NUC970 UART up to 11 channels
 * 0/3/5/7/9    Normal-speed
 * 1/2/4/6/8/10 High-speed
 */

int FCR_HIGH_TRI_LEV[16] = { 1, 4, 8, 14, 30, 46, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62 };
int FCR_NORM_TRI_LEV[16] = { 1, 4, 8, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14 };

static uint32_t nuc970_uart_FIFO_trigger_level(uint32_t channel,
	uint32_t reg)
{
	switch (channel) {
	case 0:
	case 3:
	case 5:
	case 7:
	case 9:
	default:
		return FCR_NORM_TRI_LEV[reg];
	case 1:
	case 2:
	case 4:
	case 6:
	case 8:
	case 10:
		return FCR_HIGH_TRI_LEV[reg];
	}
}
/*
static uint32_t
nuc970_uart_Tx_FIFO_trigger_level(const NUC970UartState* s)
{
	uint32_t reg;

	reg = (s->reg[I_(UA_FCR)] & UFCON_Tx_FIFO_TRIGGER_LEVEL) >>
		UFCON_Tx_FIFO_TRIGGER_LEVEL_SHIFT;

	return nuc970_uart_FIFO_trigger_level(s->channel, reg);
}
*/
static uint32_t
nuc970_uart_Rx_FIFO_trigger_level(const NUC970UartState* s)
{
	uint32_t reg;

	reg = ((s->reg[I_(UA_FCR)] & UFCON_Rx_FIFO_TRIGGER_LEVEL) >>
		UFCON_Rx_FIFO_TRIGGER_LEVEL_SHIFT);

	return nuc970_uart_FIFO_trigger_level(s->channel, reg);
}


 /*
 The UART Controller supports seven types of interrupts including
 (1). LIN function interrupt (INT_LIN)
 (2). Buffer error interrupt (INT_BUF_ERR)
 (3). time-out interrupt (INT_TOUT)
 (4). MODEM status interrupt (INT_MODEM)
 (5). Line status interrupt (break error, parity error, framing error or RS-485 interrupt) (INT_RLS)
 (6). Receiver threshold level reaching interrupt (INT_RDA)
 (7). Transmitter FIFO empty interrupt (INT_THRE)
 */

static void nuc970_uart_update_irq(NUC970UartState* s)
{
	uint32_t count;


	count = fifo_elements_number(&s->rx);
	if (count && count >= nuc970_uart_Rx_FIFO_trigger_level(s)) {
		s->reg[I_(UA_ISR)] |= 0x01; // RDA_IF;
		timer_del(s->fifo_timeout_timer);
		s->reg[I_(UA_ISR)] &= ~(1 << 4);
	}
	else if ( count && (s->reg[I_(UA_ISR)] & (1<<4)) ) {

	}
	else {
		s->reg[I_(UA_ISR)] &= ~0x01; // RDA_IF;
	}

	if (fifo_elements_number(&s->tx) == 0) {
		s->reg[I_(UA_ISR)] |= 0x02;
	}
	else {
		s->reg[I_(UA_ISR)] &= ~0x02;
	}



	//s->reg[I_(UINTP)] = s->reg[I_(UINTSP)] & ~s->reg[I_(UINTM)];
	//
	// 
	//int interrupt = ((s->reg[I_(UA_ISR)] & 0x13) & (s->reg[I_(UA_IER)] & 0x13)) ||
	//		((s->reg[I_(UA_ISR)] >> 12 & 0x1) & (s->reg[I_(UA_IER)] >> 11 & 0x1));
	int interrupt = ((s->reg[I_(UA_ISR)] & 0x13) & (s->reg[I_(UA_IER)] & 0x13));

	DPRINTF(" uart[%d] interrupt: %x (%x & %x)\n", s->channel, interrupt, 
		s->reg[I_(UA_ISR)], s->reg[I_(UA_IER)]);
	//fprintf(stderr, " uart[%d] interrupt: %x (%08x & %08x)\n", s->channel, interrupt,
	//	s->reg[I_(UA_ISR)], s->reg[I_(UA_IER)]);

	if (interrupt) {
		//fprintf(stderr, " uart[%d] interrupt: %x (%08x & %08x)\n", s->channel, interrupt,
		//		s->reg[I_(UA_ISR)], s->reg[I_(UA_IER)]);
		qemu_irq_raise(s->irq);
		trace_exynos_uart_irq_raised(s->channel, s->reg[I_(UA_ISR)]);
	}
	else {
		qemu_irq_lower(s->irq);
		trace_exynos_uart_irq_lowered(s->channel);
	}
}

static void nuc970_uart_timeout_int(void* opaque)
{
	NUC970UartState* s = opaque;

	trace_exynos_uart_rx_timeout(s->channel, s->reg[I_(UA_IER)],
		s->reg[I_(UA_ISR)]);

	if (fifo_elements_number(&s->rx) > 0 && s->reg[I_(UA_IER)] & (1 << 4)) {
		s->reg[I_(UA_ISR)] |= (1 << 4);
		if (s->reg[I_(UA_IER)] & (1 << 11)) {
			s->reg[I_(UA_ISR)] |= (1 << 12);
		}
		
		nuc970_uart_update_irq(s);
	}
}

static int get_baudrate(uint32_t value)
{
	/*
	[31:30] Reserved    Reserved.
	[29]    DIV_X_EN    Divider X Enable Control
		0 = Divider X Disabled (the equation of M = 16).
		1 = Divider X Enabled (the equation of M = X+1, but DIVIDER_X(UA_BAUD[27:24]) must
			>= 8).
		Refer to the table below for more information.
		Note: In IrDA mode, this bit must disable.
		Note: The BRD = Baud Rate Divider, and the baud rate equation is
		Baud Rate = Clock / [M * (BRD + 2)]; The default value of M is 16.
	[28]    DIV_X_ONE   Divider X Equal to 1
		0 = Divider M = X (the equation of M = X+1, but DIVIDER_X(UA_BAUD[27:24]) must >=
		8).
		1 = Divider M = 1 (the equation of M = 1, but BRD(UA_BAUD[15:0]) must >= 3).
	[27:24] DIVIDER_X   Divider X
		The baud rate divider M = X+1.
	[23:16] Reserved    Reserved.
	[15:0]  BRD         Baud Rate Divider
		The field indicated the baud rate divider

	 * The baud rate equation is: Baud Rate = UART_CLK / M * [BRD + 2]
	 * where M and BRD are defined in Baud Rate Divider Register (UA_BAUD).
	 *   Mode DIV_X_EN DIV_X_ONE DIVIDER X  BRD Baud Rate Equation
	 *   0    Disable  0         Don’t Care A   UART_CLK / [16 * (A+2)]
	 *   1    Enable   0         B          A   UART_CLK / [(B+1) * (A+2)] , B must >= 8
	 *   2    Enable   1         Don’t care A   UART_CLK / (A+2), A must >=9
	 */
	uint32_t xEn = (value >> 29) & 0x1;
	uint32_t xOne = (value >> 28) & 0x1;
	uint32_t divX = (value >> 24) & 0xf;
	uint32_t brd = value & 0xffff;
	uint32_t m = 1;
	if (!xEn)
		m = 16;
	else if (!xOne)
		m = divX + 1;

	int baudrate = 12000000 / (m * (brd + 2));
	return baudrate;
}

static void nuc970_uart_update_parameters(NUC970UartState* s)
{
	int speed, parity, data_bits, stop_bits;
	QEMUSerialSetParams ssp;
	//uint64_t uclk_rate;

	if (s->reg[I_(UA_BAUD)] == 0) {
		return;
	}

	if (s->reg[I_(UA_LCR)] & (1 << 3)) {  // PBE: Parity Bit Enable Control
		if (s->reg[I_(UA_LCR)] & (1 << 4)) {  // EPE: Even Parity Enable Control
			parity = 'E';
		}
		else {
			parity = 'O';
		}
	}
	else {
		parity = 'N';
	}

	if (s->reg[I_(UA_LCR)] & (1 << 2)) {
		stop_bits = 2;
	}
	else {
		stop_bits = 1;
	}

	data_bits = (s->reg[I_(UA_LCR)] & 0x3) + 5;

	//uclk_rate = 24000000;

	//speed = uclk_rate / ((16 * (s->reg[I_(UBRDIV)]) & 0xffff) +
	//    (s->reg[I_(UFRACVAL)] & 0x7) + 16);
	speed = get_baudrate(s->reg[I_(UA_BAUD)]);

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
	if (s->reg[I_(UA_IER)] & (1 << 11)) {	// TIME_OUT_EN
		uint32_t timeout = (s->reg[I_(UA_TOR)] & 0xff) * s->wordtime;

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

	case UA_BAUD:
	case UA_LCR:
		s->reg[I_(offset)] = val;
		nuc970_uart_update_parameters(s);
		break;
	case UA_FCR:
		s->reg[I_(UA_FCR)] = val;
		fprintf(stderr, "  UA_FCR[%d]: %lx\n", s->channel, val);
		if (val & UFCON_Rx_FIFO_RESET) {
			fifo_reset(&s->rx);
			s->reg[I_(UA_FCR)] &= ~UFCON_Rx_FIFO_RESET;
			//trace_exynos_uart_rx_fifo_reset(s->channel);
		}
		if (val & UFCON_Tx_FIFO_RESET) {
			fifo_reset(&s->tx);
			s->reg[I_(UA_FCR)] &= ~UFCON_Tx_FIFO_RESET;
			//trace_exynos_uart_tx_fifo_reset(s->channel);
		}
		break;

	case UA_THR:

		

		if (qemu_chr_fe_backend_connected(&s->chr)) {
			//s->reg[I_(UTRSTAT)] &= ~(UTRSTAT_TRANSMITTER_EMPTY | UTRSTAT_Tx_BUFFER_EMPTY);
			ch = (uint8_t)val;
			/* XXX this blocks entire thread. Rewrite to use
			 * qemu_chr_fe_write and background I/O callbacks */
			qemu_chr_fe_write_all(&s->chr, &ch, 1);
			trace_exynos_uart_tx(s->channel, ch);
			//s->reg[I_(UTRSTAT)] |= UTRSTAT_TRANSMITTER_EMPTY | UTRSTAT_Tx_BUFFER_EMPTY;
			//s->reg[I_(UINTSP)] |= UINTSP_TXD;
			nuc970_uart_update_irq(s);
		}
		break;

	case UINTP:
		s->reg[I_(UINTP)] &= ~val;
		s->reg[I_(UINTSP)] &= ~val;
		trace_exynos_uart_intclr(s->channel, s->reg[I_(UINTP)]);
		nuc970_uart_update_irq(s);
		break;
	case UA_MCR:
		//if (val & UTRSTAT_Rx_TIMEOUT) {
		//	s->reg[I_(UTRSTAT)] &= ~UTRSTAT_Rx_TIMEOUT;
		//}
		s->reg[I_(UA_MCR)] = val & ~(1<<13); // RTS Pin State (Read Only)
		fprintf(stderr, "  UA_MCR[%d] W: %lx\n", s->channel, val);
		break;
	case UA_MSR:
		s->reg[I_(UA_MSR)] = val;
		fprintf(stderr, "  UA_MSR[%d] W: %lx\n", s->channel, val);
		break;
	case UFSTAT:
		trace_exynos_uart_ro_write(
			s->channel, nuc970_uart_regname(offset), offset);
		break;
		//case UINTSP:
		//    s->reg[I_(UINTSP)] &= ~val;
		//    break;
		//case UINTM:
		//    s->reg[I_(UINTM)] = val;
		//    nuc970_uart_update_irq(s);
		//    break;
	case UA_IER:
		//fprintf(stderr, "\033[0;32m UA_IER[%d]: %" PRIx64 "\033[0m\n", s->channel, val);
		s->reg[I_(offset)] = val;
		// need to update irq
		nuc970_uart_update_irq(s);
		break;
	case UA_ISR:
		if (val & (1 << 0))
		{

		}
		break;
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

	switch (offset) {
		//case UERSTAT: /* Read Only */
		//    res = s->reg[I_(UERSTAT)];
		//    s->reg[I_(UERSTAT)] = 0;
		//    trace_exynos_uart_read(s->channel, offset,
		//        nuc970_uart_regname(offset), res);
		//    return res;
	case UA_FSR: /* FSR */
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
		break;
	case UA_RBR:
		
		if (fifo_elements_number(&s->rx)) {
				res = fifo_retrieve(&s->rx);
				trace_exynos_uart_rx(s->channel, res);
				if (!fifo_elements_number(&s->rx)) {
					s->reg[I_(UTRSTAT)] &= ~UTRSTAT_Rx_BUFFER_DATA_READY;

					
				}
				else {
					s->reg[I_(UTRSTAT)] |= UTRSTAT_Rx_BUFFER_DATA_READY;
				}

				s->reg[I_(UA_ISR)] &= ~TOUT_IF;	// clear TOUT_IF
				s->reg[I_(UA_ISR)] &= ~(1 << 12);	// clear TOUT_INT

				//s->reg[I_(UA_ISR)] &= ~(1);
				nuc970_uart_update_irq(s);
		}
		else {
				trace_exynos_uart_rx_error(s->channel);
				s->reg[I_(UINTSP)] |= UINTSP_ERROR;
				nuc970_uart_update_irq(s);
				res = 0;
		}
		
		

		qemu_chr_fe_accept_input(&s->chr);
		//nuc970_uart_update_dmabusy(s);
		trace_exynos_uart_read(s->channel, offset,
			nuc970_uart_regname(offset), res);
		break;
		//case UTXH:
		//    trace_exynos_uart_wo_read(s->channel, nuc970_uart_regname(offset),
		//        offset);
		//    break;
	case UA_MCR:
		res = s->reg[I_(UA_MCR)]; // RTS_ST
		fprintf(stderr, "  UA_MCR[%d] R: %x\n", s->channel, res);
		break;
	case UA_MSR:
		res = s->reg[I_(UA_MSR)];
		fprintf(stderr, "  UA_MSR[%d] R: %x\n", s->channel, res);
		break;
	default:
		trace_exynos_uart_read(s->channel, offset,
			nuc970_uart_regname(offset),
			s->reg[I_(offset)]);
		res = s->reg[I_(offset)];
		break;
	}

	trace_exynos_uart_read(s->channel, offset, nuc970_uart_regname(offset),
		0);

	DPRINTF("read(offset=0x%" HWADDR_PRIx ", value=%08x)\n", offset, res);
	//if (offset > 0 && offset != 0x1c && offset != 0x18) {
	//	fprintf(stderr, "read(offset=0x%" HWADDR_PRIx ", value=%08x)\n", offset, res);
	//}
	return res;
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

	//if (s->reg[I_(UFCON)] & UFCON_FIFO_ENABLE) {
	if (1) {
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
	
		if (fifo_empty_elements_number(&s->rx) < size) {
			size = fifo_empty_elements_number(&s->rx);
			s->reg[I_(UINTSP)] |= UINTSP_ERROR;
		}
		for (i = 0; i < size; i++) {
			fifo_store(&s->rx, buf[i]);
		}
		DPRINTF("receive(%0x) %d\n", buf[0], size);
		nuc970_uart_rx_timeout_set(s);
	
	//s->reg[I_(UTRSTAT)] |= UTRSTAT_Rx_BUFFER_DATA_READY;

	nuc970_uart_update_irq(s);
}


static void nuc970_uart_event(void* opaque, QEMUChrEvent event)
{
	NUC970UartState* s = (NUC970UartState*)opaque;

	if (event == CHR_EVENT_BREAK) {
		/* When the RxDn is held in logic 0, then a null byte is pushed into the
		 * fifo */
		fifo_store(&s->rx, '\0');
		//s->reg[I_(UERSTAT)] |= UERSTAT_BREAK;
		nuc970_uart_update_irq(s);
	}
}


static void nuc970_uart_reset(DeviceState* dev)
{
	NUC970UartState* s = NUC970_UART(dev);
	int i;
	//fprintf(stderr, "*** uart_reset ***\n");
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
		"nuc970.uart", 0x100);
	sysbus_init_mmio(dev, &s->iomem);

	sysbus_init_irq(dev, &s->irq);
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