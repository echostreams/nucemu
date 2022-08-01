/*
 */

#ifndef NUC970_I2C_H
#define NUC970_I2C_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_NUC970_I2C "nuc970.i2c"
OBJECT_DECLARE_SIMPLE_TYPE(NUC970I2CState, NUC970_I2C)

#define NUC970_I2C_MEM_SIZE           0x18

/* NUC970 I2C memory map */
/*
    i2c register offset
*/
#define     I2C_CSR		(0x00)  /*!< Control and Status Register */
#define     I2C_DIVIDER	(0x04)  /*!< Clock Prescale Register */
#define     I2C_CMDR	(0x08)  /*!< Command Register */
#define     I2C_SWR		(0x0C)  /*!< Software Mode Control Register */
#define     I2C_RxR		(0x10)  /*!< Data Receive Register */
#define     I2C_TxR		(0x14)  /*!< Data Transmit Register */

/* nuc970 i2c CSR register bits */

#define IRQEN       0x003
#define I2CBUSY     0x400
#define I2CSTART    0x018
#define IRQFLAG     0x004
#define ARBIT_LOST  0x200
#define SLAVE_ACK   0x800

/* nuc970 i2c CMDR register bits */

#define I2C_CMD_START   0x10
#define I2C_CMD_STOP    0x08
#define I2C_CMD_READ    0x04
#define I2C_CMD_WRITE   0x02
#define I2C_CMD_NACK    0x01

#define IADR_ADDR                  0x00  /* address register */
#define IFDR_ADDR                  0x04  /* frequency divider register */
#define I2CR_ADDR                  0x08  /* control register */
#define I2SR_ADDR                  0x0c  /* status register */
#define I2DR_ADDR                  0x10  /* data register */

#define IADR_MASK                  0xFE
#define IADR_RESET                 0

#define IFDR_MASK                  0x3F
#define IFDR_RESET                 0

#define I2CR_IEN                   (1 << 0)
#define I2CR_IIEN                  (1 << 1)

#define I2CR_MSTA                  (1 << 5)
#define I2CR_MTX                   (1 << 4)
#define I2CR_TXAK                  (1 << 3)
#define I2CR_RSTA                  (1 << 2)
#define I2CR_MASK                  0xFC
#define I2CR_RESET                 0

#define I2SR_ICF                   (1 << 7)
#define I2SR_IAAF                  (1 << 6)
#define I2SR_IBB                   (1 << 5)
#define I2SR_IAL                   (1 << 4)
#define I2SR_SRW                   (1 << 2)
#define I2SR_IIF                   (1 << 2)
#define I2SR_RXAK                  (1 << 0)
#define I2SR_MASK                  0xE9
#define I2SR_RESET                 0x81

#define I2DR_MASK                  0xFF
#define I2DR_RESET                 0

#define ADDR_RESET                 0xFF00

struct NUC970I2CState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    MemoryRegion iomem;
    I2CBus* bus;
    qemu_irq irq;

    uint16_t csr;
    uint16_t divider;
    uint8_t cmdr;
    uint8_t swr;
    uint8_t rxr;
    uint32_t txr;

};

#endif /* NUC970_I2C_H */
