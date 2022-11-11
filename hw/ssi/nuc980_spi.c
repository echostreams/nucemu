#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/fifo32.h"

#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/ssi/ssi.h"
#include "qom/object.h"
#include "qapi/error.h"
#include "sysemu/blockdev.h"
#include "hw/block/flash.h"
#include "qemu/error-report.h"
#include "qemu/log.h"

#include "trace.h"

/*
QSPIx_CTL       QSPIx_BA+0x00 R/W QSPI Control Register              0x0000_0034
QSPIx_CLKDIV    QSPIx_BA+0x04 R/W QSPI Clock Divider Register        0x0000_0000
QSPIx_SSCTL     QSPIx_BA+0x08 R/W QSPI Slave Select Control Register 0x0000_0000
QSPIx_PDMACTL   QSPIx_BA+0x0C R/W QSPI PDMA Control Register         0x0000_0000
QSPIx_FIFOCTL   QSPIx_BA+0x10 R/W QSPI FIFO Control Register         0x4400_0000
QSPIx_STATUS    QSPIx_BA+0x14 R/W QSPI Status Register               0x0005_0110
QSPIx_TX        QSPIx_BA+0x20 W   QSPI Data Transmit Register        0x0000_0000
QSPIx_RX        QSPIx_BA+0x30 R   QSPI Data Receive Register         0x0000_0000
*/


#define NUC980_SPI_ERR_DEBUG 1

#if NUC980_SPI_ERR_DEBUG
#define DPRINTF(...) do { \
    fprintf(stderr,  ": %s: ", __func__); \
    fprintf(stderr, ## __VA_ARGS__); \
    } while (0)
#else
#define DPRINTF(...)
#endif

#define TYPE_NUC980_SPI "nuc980-spi"
OBJECT_DECLARE_SIMPLE_TYPE(NUC980SPI, NUC980_SPI)

struct NUC980SPI {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    qemu_irq irq;
    int irqline;

    uint8_t num_cs;
    qemu_irq* cs_lines;

    SSIBus* spi;

    Fifo32 rx_fifo;
    Fifo32 tx_fifo;

    /**
     * @var SPI_T::CTL
     * Offset: 0x00  SPI Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SPIEN     |SPI Transfer Control Enable Bit
     * |        |          |In Master mode, the transfer will start when there is data in the FIFO buffer after this bit is set to 1
     * |        |          |In Slave mode, this device is ready to receive data when this bit is set to 1.
     * |        |          |0 = Transfer control Disabled.
     * |        |          |1 = Transfer control Enabled.
     * |        |          |Note: Before changing the configurations of SPIx_CTL, SPIx_CLKDIV, SPIx_SSCTL and SPIx_FIFOCTL registers, user shall clear the SPIEN (SPIx_CTL[0]) and confirm the SPIENSTS (SPIx_STATUS[15]) is 0.
     * |[1]     |RXNEG     |Receive on Negative Edge
     * |        |          |0 = Received data input signal is latched on the rising edge of SPI bus clock.
     * |        |          |1 = Received data input signal is latched on the falling edge of SPI bus clock.
     * |[2]     |TXNEG     |Transmit on Negative Edge
     * |        |          |0 = Transmitted data output signal is changed on the rising edge of SPI bus clock.
     * |        |          |1 = Transmitted data output signal is changed on the falling edge of SPI bus clock.
     * |[3]     |CLKPOL    |Clock Polarity
     * |        |          |0 = SPI bus clock is idle low.
     * |        |          |1 = SPI bus clock is idle high.
     * |[7:4]   |SUSPITV   |Suspend Interval (Master Only)
     * |        |          |The four bits provide configurable suspend interval between two successive transmit/receive transaction in a transfer
     * |        |          |The definition of the suspend interval is the interval between the last clock edge of the preceding transaction word and the first clock edge of the following transaction word
     * |        |          |The default value is 0x3
     * |        |          |The period of the suspend interval is obtained according to the following equation.
     * |        |          |(SUSPITV[3:0] + 0.5) * period of SPICLK clock cycle
     * |        |          |Example:
     * |        |          |SUSPITV = 0x0 .... 0.5 SPICLK clock cycle.
     * |        |          |SUSPITV = 0x1 .... 1.5 SPICLK clock cycle.
     * |        |          |.....
     * |        |          |SUSPITV = 0xE .... 14.5 SPICLK clock cycle.
     * |        |          |SUSPITV = 0xF .... 15.5 SPICLK clock cycle.
     * |[12:8]  |DWIDTH    |Data Width
     * |        |          |This field specifies how many bits can be transmitted / received in one transaction
     * |        |          |The minimum bit length is 8 bits and can up to 32 bits.
     * |        |          |DWIDTH = 0x08 .... 8 bits.
     * |        |          |DWIDTH = 0x09 .... 9 bits.
     * |        |          |.....
     * |        |          |DWIDTH = 0x1F .... 31 bits.
     * |        |          |DWIDTH = 0x00 .... 32 bits.
     * |        |          |Note: For SPI1~SPI4, this bit field will decide the depth of TX/RX FIFO configuration in SPI mode
     * |        |          |Therefore, changing this bit field will clear TX/RX FIFO by hardware automatically in SPI1~SPI4.
     * |[13]    |LSB       |Send LSB First
     * |        |          |0 = The MSB, which bit of transmit/receive register depends on the setting of DWIDTH, is transmitted/received first.
     * |        |          |1 = The LSB, bit 0 of the SPI TX register, is sent first to the SPI data output pin, and the first bit received from the SPI data input pin will be put in the LSB position of the RX register (bit 0 of SPI_RX).
     * |[14]    |HALFDPX   |SPI Half-duplex Transfer Enable Bit
     * |        |          |This bit is used to select full-duplex or half-duplex for SPI transfer
     * |        |          |The bit field DATDIR (SPIx_CTL[20]) can be used to set the data direction in half-duplex transfer.
     * |        |          |0 = SPI operates in full-duplex transfer.
     * |        |          |1 = SPI operates in half-duplex transfer.
     * |[15]    |RXONLY    |Receive-only Mode Enable Bit (Master Only)
     * |        |          |This bit field is only available in Master mode
     * |        |          |In receive-only mode, SPI Master will generate SPI bus clock continuously for receiving data bit from SPI slave device and assert the BUSY status.
     * |        |          |0 = Receive-only mode Disabled.
     * |        |          |1 = Receive-only mode Enabled.
     * |[17]    |UNITIEN   |Unit Transfer Interrupt Enable Bit
     * |        |          |0 = SPI unit transfer interrupt Disabled.
     * |        |          |1 = SPI unit transfer interrupt Enabled.
     * |[18]    |SLAVE     |Slave Mode Control
     * |        |          |0 = Master mode.
     * |        |          |1 = Slave mode.
     * |[19]    |REORDER   |Byte Reorder Function Enable Bit
     * |        |          |0 = Byte Reorder function Disabled.
     * |        |          |1 = Byte Reorder function Enabled
     * |        |          |A byte suspend interval will be inserted among each byte
     * |        |          |The period of the byte suspend interval depends on the setting of SUSPITV.
     * |        |          |Note: Byte Reorder function is only available if DWIDTH is defined as 16, 24, and 32 bits.
     * |[20]    |DATDIR    |Data Port Direction Control
     * |        |          |This bit is used to select the data input/output direction in half-duplex transfer and Dual/Quad transfer
     * |        |          |0 = SPI data is input direction.
     * |        |          |1 = SPI data is output direction.
     * |[21]    |DUALIOEN  |Dual I/O Mode Enable Bit (Only Supported in QSPI0)
     * |        |          |0 = Dual I/O mode Disabled.
     * |        |          |1 = Dual I/O mode Enabled.
     * |[22]    |QUADIOEN  |Quad I/O Mode Enable Bit (Only Supported in QSPI0)
     * |        |          |0 = Quad I/O mode Disabled.
     * |        |          |1 = Quad I/O mode Enabled.
     * @var SPI_T::CLKDIV
     * Offset: 0x04  SPI Clock Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:0]   |DIVIDER   |Clock Divider
     * |        |          |The value in this field is the frequency divider for generating the peripheral clock, fspi_eclk, and the SPI bus clock of SPI Master
     * |        |          |The frequency is obtained according to the following equation.
     * |        |          |where
     * |        |          |is the peripheral clock source, which is defined in the clock control register, CLK_CLKSEL2.
     * |        |          |Note: Not supported in I2S mode.
     * @var SPI_T::SSCTL
     * Offset: 0x08  SPI Slave Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SS        |Slave Selection Control (Master Only)
     * |        |          |If AUTOSS bit is cleared to 0,
     * |        |          |0 = set the SPIx_SS line to inactive state.
     * |        |          |1 = set the SPIx_SS line to active state.
     * |        |          |If the AUTOSS bit is set to 1,
     * |        |          |0 = Keep the SPIx_SS line at inactive state.
     * |        |          |1 = SPIx_SS line will be automatically driven to active state for the duration of data transfer, and will be driven to inactive state for the rest of the time
     * |        |          |The active state of SPIx_SS is specified in SSACTPOL (SPIx_SSCTL[2]).
     * |[2]     |SSACTPOL  |Slave Selection Active Polarity
     * |        |          |This bit defines the active polarity of slave selection signal (SPIx_SS).
     * |        |          |0 = The slave selection signal SPIx_SS is active low.
     * |        |          |1 = The slave selection signal SPIx_SS is active high.
     * |[3]     |AUTOSS    |Automatic Slave Selection Function Enable Bit (Master Only)
     * |        |          |0 = Automatic slave selection function Disabled
     * |        |          |Slave selection signal will be asserted/de-asserted according to SS (SPIx_SSCTL[0]).
     * |        |          |1 = Automatic slave selection function Enabled.
     * |[4]     |SLV3WIRE  |Slave 3-wire Mode Enable Bit (Only Supported in QSPI0)
     * |        |          |Slave 3-wire mode is only available in QSPI0
     * |        |          |In Slave 3-wire mode, the QSPI controller can work with 3-wire interface including QSPI0_CLK, QSPI0_MISO and QSPI0_MOSI pins.
     * |        |          |0 = 4-wire bi-direction interface.
     * |        |          |1 = 3-wire bi-direction interface.
     * |[5]     |SLVTOIEN  |Slave Mode Time-out Interrupt Enable Bit (Only Supported in QSPI0)
     * |        |          |0 = Slave mode time-out interrupt Disabled.
     * |        |          |1 = Slave mode time-out interrupt Enabled.
     * |[6]     |SLVTORST  |Slave Mode Time-out Reset Control (Only Supported in QSPI0)
     * |        |          |0 = When Slave mode time-out event occurs, the TX and RX control circuit will not be reset.
     * |        |          |1 = When Slave mode time-out event occurs, the TX and RX control circuit will be reset by hardware.
     * |[8]     |SLVBEIEN  |Slave Mode Bit Count Error Interrupt Enable Bit
     * |        |          |0 = Slave mode bit count error interrupt Disabled.
     * |        |          |1 = Slave mode bit count error interrupt Enabled.
     * |[9]     |SLVURIEN  |Slave Mode TX Under Run Interrupt Enable Bit
     * |        |          |0 = Slave mode TX under run interrupt Disabled.
     * |        |          |1 = Slave mode TX under run interrupt Enabled.
     * |[12]    |SSACTIEN  |Slave Select Active Interrupt Enable Bit
     * |        |          |0 = Slave select active interrupt Disabled.
     * |        |          |1 = Slave select active interrupt Enabled.
     * |[13]    |SSINAIEN  |Slave Select Inactive Interrupt Enable Bit
     * |        |          |0 = Slave select inactive interrupt Disabled.
     * |        |          |1 = Slave select inactive interrupt Enabled.
     * |[31:16] |SLVTOCNT  |Slave Mode Time-out Period (Only Supported in QSPI0)
     * |        |          |In Slave mode, these bits indicate the time-out period when there is bus clock input during slave select active
     * |        |          |The clock source of the time-out counter is Slave peripheral clock
     * |        |          |If the value is 0, it indicates the slave mode time-out function is disabled.
     * @var SPI_T::PDMACTL
     * Offset: 0x0C  SPI PDMA Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TXPDMAEN  |Transmit PDMA Enable Bit
     * |        |          |0 = Transmit PDMA function Disabled.
     * |        |          |1 = Transmit PDMA function Enabled.
     * |        |          |Note: In SPI Master mode with full duplex transfer, if both TX and RX PDMA functions are enabled, RX PDMA function cannot be enabled prior to TX PDMA function
     * |        |          |User can enable TX PDMA function firstly or enable both functions simultaneously.
     * |[1]     |RXPDMAEN  |Receive PDMA Enable Bit
     * |        |          |0 = Receive PDMA function Disabled.
     * |        |          |1 = Receive PDMA function Enabled.
     * |[2]     |PDMARST   |PDMA Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the PDMA control logic of the SPI controller. This bit will be automatically cleared to 0.
     * @var SPI_T::FIFOCTL
     * Offset: 0x10  SPI FIFO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXRST     |Receive Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset receive FIFO pointer and receive circuit
     * |        |          |The RXFULL bit will be cleared to 0 and the RXEMPTY bit will be set to 1
     * |        |          |This bit will be cleared to 0 by hardware about 3 system clock cycles + 2 peripheral clock cycles after it is set to 1
     * |        |          |User can read TXRXRST (SPIx_STATUS[23]) to check if reset is accomplished or not.
     * |[1]     |TXRST     |Transmit Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset transmit FIFO pointer and transmit circuit
     * |        |          |The TXFULL bit will be cleared to 0 and the TXEMPTY bit will be set to 1
     * |        |          |This bit will be cleared to 0 by hardware about 3 system clock cycles + 2 peripheral clock cycles after it is set to 1
     * |        |          |User can read TXRXRST (SPIx_STATUS[23]) to check if reset is accomplished or not.
     * |        |          |Note: If TX underflow event occurs in SPI Slave mode, this bit can be used to make SPI return to idle state.
     * |[2]     |RXTHIEN   |Receive FIFO Threshold Interrupt Enable Bit
     * |        |          |0 = RX FIFO threshold interrupt Disabled.
     * |        |          |1 = RX FIFO threshold interrupt Enabled.
     * |[3]     |TXTHIEN   |Transmit FIFO Threshold Interrupt Enable Bit
     * |        |          |0 = TX FIFO threshold interrupt Disabled.
     * |        |          |1 = TX FIFO threshold interrupt Enabled.
     * |[4]     |RXTOIEN   |Slave Receive Time-out Interrupt Enable Bit
     * |        |          |0 = Receive time-out interrupt Disabled.
     * |        |          |1 = Receive time-out interrupt Enabled.
     * |[5]     |RXOVIEN   |Receive FIFO Overrun Interrupt Enable Bit
     * |        |          |0 = Receive FIFO overrun interrupt Disabled.
     * |        |          |1 = Receive FIFO overrun interrupt Enabled.
     * |[6]     |TXUFPOL   |TX Underflow Data Polarity
     * |        |          |0 = The SPI data out is keep 0 if there is TX underflow event in Slave mode.
     * |        |          |1 = The SPI data out is keep 1 if there is TX underflow event in Slave mode.
     * |        |          |Note:
     * |        |          |1. The TX underflow event occurs if there is no any data in TX FIFO when the slave selection signal is active.
     * |        |          |2. This bit should be set as 0 in I2S mode.
     * |        |          |3. When TX underflow event occurs, SPIx_MISO pin state will be determined by this setting even though TX FIFO is not empty afterward
     * |        |          |Data stored in TX FIFO will be sent through SPIx_MISO pin in the next transfer frame.
     * |[7]     |TXUFIEN   |TX Underflow Interrupt Enable Bit
     * |        |          |When TX underflow event occurs in Slave mode, TXUFIF (SPIx_STATUS[19]) will be set to 1
     * |        |          |This bit is used to enable the TX underflow interrupt.
     * |        |          |0 = Slave TX underflow interrupt Disabled.
     * |        |          |1 = Slave TX underflow interrupt Enabled.
     * |[8]     |RXFBCLR   |Receive FIFO Buffer Clear
     * |        |          |0 = No effect.
     * |        |          |1 = Clear receive FIFO pointer
     * |        |          |The RXFULL bit will be cleared to 0 and the RXEMPTY bit will be set to 1
     * |        |          |This bit will be cleared to 0 by hardware about 1 system clock after it is set to 1.
     * |        |          |Note: The RX shift register will not be cleared.
     * |[9]     |TXFBCLR   |Transmit FIFO Buffer Clear
     * |        |          |0 = No effect.
     * |        |          |1 = Clear transmit FIFO pointer
     * |        |          |The TXFULL bit will be cleared to 0 and the TXEMPTY bit will be set to 1
     * |        |          |This bit will be cleared to 0 by hardware about 1 system clock after it is set to 1.
     * |        |          |Note: The TX shift register will not be cleared.
     * |[26:24] |RXTH      |Receive FIFO Threshold
     * |        |          |If the valid data count of the receive FIFO buffer is larger than the RXTH setting, the RXTHIF bit will be set to 1, else the RXTHIF bit will be cleared to 0
     * |        |          |For SPI1~SPI4, the MSB of this bit field is only meaningful while SPI mode 8~16 bits of data length.
     * |[30:28] |TXTH      |Transmit FIFO Threshold
     * |        |          |If the valid data count of the transmit FIFO buffer is less than or equal to the TXTH setting, the TXTHIF bit will be set to 1, else the TXTHIF bit will be cleared to 0
     * |        |          |For SPI1~SPI4, the MSB of this bit field is only meaningful while SPI mode 8~16 bits of data length
     * @var SPI_T::STATUS
     * Offset: 0x14  SPI Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BUSY      |Busy Status (Read Only)
     * |        |          |0 = SPI controller is in idle state.
     * |        |          |1 = SPI controller is in busy state.
     * |        |          |The following listing are the bus busy conditions:
     * |        |          |a. SPIx_CTL[0] = 1 and TXEMPTY = 0.
     * |        |          |b
     * |        |          |For SPI Master mode, SPIx_CTL[0] = 1 and TXEMPTY = 1 but the current transaction is not finished yet.
     * |        |          |c. For SPI Master mode, SPIx_CTL[0] = 1 and RXONLY = 1.
     * |        |          |d
     * |        |          |For SPI Slave mode, the SPIx_CTL[0] = 1 and there is serial clock input into the SPI core logic when slave select is active.
     * |        |          |For SPI Slave mode, the SPIx_CTL[0] = 1 and the transmit buffer or transmit shift register is not empty even if the slave select is inactive.
     * |[1]     |UNITIF    |Unit Transfer Interrupt Flag
     * |        |          |0 = No transaction has been finished since this bit was cleared to 0.
     * |        |          |1 = SPI controller has finished one unit transfer.
     * |        |          |Note: This bit will be cleared by writing 1 to it.
     * |[2]     |SSACTIF   |Slave Select Active Interrupt Flag
     * |        |          |0 = Slave select active interrupt was cleared or not occurred.
     * |        |          |1 = Slave select active interrupt event occurred.
     * |        |          |Note: Only available in Slave mode. This bit will be cleared by writing 1 to it.
     * |[3]     |SSINAIF   |Slave Select Inactive Interrupt Flag
     * |        |          |0 = Slave select inactive interrupt was cleared or not occurred.
     * |        |          |1 = Slave select inactive interrupt event occurred.
     * |        |          |Note: Only available in Slave mode. This bit will be cleared by writing 1 to it.
     * |[4]     |SSLINE    |Slave Select Line Bus Status (Read Only)
     * |        |          |0 = The slave select line status is 0.
     * |        |          |1 = The slave select line status is 1.
     * |        |          |Note: This bit is only available in Slave mode
     * |        |          |If SSACTPOL (SPIx_SSCTL[2]) is set 0, and the SSLINE is 1, the SPI slave select is in inactive status.
     * |[6]     |SLVBEIF   |Slave Mode Bit Count Error Interrupt Flag
     * |        |          |In Slave mode, when the slave select line goes to inactive state, if bit counter is mismatch with DWIDTH, this interrupt flag will be set to 1.
     * |        |          |0 = No Slave mode bit count error event.
     * |        |          |1 = Slave mode bit count error event occurs.
     * |        |          |Note: If the slave select active but there is no any bus clock input, the SLVBEIF also active when the slave select goes to inactive state
     * |        |          |This bit will be cleared by writing 1 to it.
     * |[7]     |SLVURIF   |Slave Mode TX Under Run Interrupt Flag
     * |        |          |In Slave mode, if TX underflow event occurs and the slave select line goes to inactive state, this interrupt flag will be set to 1.
     * |        |          |0 = No Slave TX under run event.
     * |        |          |1 = Slave TX under run event occurs.
     * |        |          |Note: This bit will be cleared by writing 1 to it.
     * |[8]     |RXEMPTY   |Receive FIFO Buffer Empty Indicator (Read Only)
     * |        |          |0 = Receive FIFO buffer is not empty.
     * |        |          |1 = Receive FIFO buffer is empty.
     * |[9]     |RXFULL    |Receive FIFO Buffer Full Indicator (Read Only)
     * |        |          |0 = Receive FIFO buffer is not full.
     * |        |          |1 = Receive FIFO buffer is full.
     * |[10]    |RXTHIF    |Receive FIFO Threshold Interrupt Flag (Read Only)
     * |        |          |0 = The valid data count within the receive FIFO buffer is smaller than or equal to the setting value of RXTH.
     * |        |          |1 = The valid data count within the receive FIFO buffer is larger than the setting value of RXTH.
     * |[11]    |RXOVIF    |Receive FIFO Overrun Interrupt Flag
     * |        |          |When the receive FIFO buffer is full, the follow-up data will be dropped and this bit will be set to 1.
     * |        |          |0 = No FIFO is overrun.
     * |        |          |1 = Receive FIFO is overrun.
     * |        |          |Note: This bit will be cleared by writing 1 to it.
     * |[12]    |RXTOIF    |Receive Time-out Interrupt Flag
     * |        |          |0 = No receive FIFO time-out event.
     * |        |          |1 = Receive FIFO buffer is not empty and no read operation on receive FIFO buffer over 64 SPI peripheral clock periods in Master mode or over 576 SPI peripheral clock periods in Slave mode
     * |        |          |When the received FIFO buffer is read by software, the time-out status will be cleared automatically.
     * |        |          |Note: This bit will be cleared by writing 1 to it.
     * |[15]    |SPIENSTS  |SPI Enable Status (Read Only)
     * |        |          |0 = The SPI controller is disabled.
     * |        |          |1 = The SPI controller is enabled.
     * |        |          |Note: The SPI peripheral clock is asynchronous with the system clock
     * |        |          |In order to make sure the SPI control logic is disabled, this bit indicates the real status of SPI controller.
     * |[16]    |TXEMPTY   |Transmit FIFO Buffer Empty Indicator (Read Only)
     * |        |          |0 = Transmit FIFO buffer is not empty.
     * |        |          |1 = Transmit FIFO buffer is empty.
     * |[17]    |TXFULL    |Transmit FIFO Buffer Full Indicator (Read Only)
     * |        |          |0 = Transmit FIFO buffer is not full.
     * |        |          |1 = Transmit FIFO buffer is full.
     * |[18]    |TXTHIF    |Transmit FIFO Threshold Interrupt Flag (Read Only)
     * |        |          |0 = The valid data count within the transmit FIFO buffer is larger than the setting value of TXTH.
     * |        |          |1 = The valid data count within the transmit FIFO buffer is less than or equal to the setting value of TXTH.
     * |[19]    |TXUFIF    |TX Underflow Interrupt Flag
     * |        |          |When the TX underflow event occurs, this bit will be set to 1, the state of data output pin depends on the setting of TXUFPOL.
     * |        |          |0 = No effect.
     * |        |          |1 = No data in Transmit FIFO and TX shift register when the slave selection signal is active.
     * |        |          |Note 1: This bit will be cleared by writing 1 to it.
     * |        |          |Note 2: If reset slave's transmission circuit when slave selection signal is active, this flag will be set to 1 after 2 peripheral clock cycles + 3 system clock cycles since the reset operation is done.
     * |[23]    |TXRXRST   |TX or RX Reset Status (Read Only)
     * |        |          |0 = The reset function of TXRST or RXRST is done.
     * |        |          |1 = Doing the reset function of TXRST or RXRST.
     * |        |          |Note: Both the reset operations of TXRST and RXRST need 3 system clock cycles + 2 peripheral clock cycles
     * |        |          |User can check the status of this bit to monitor the reset function is doing or done.
     * |[27:24] |RXCNT     |Receive FIFO Data Count (Read Only)
     * |        |          |This bit field indicates the valid data count of receive FIFO buffer.
     * |[31:28] |TXCNT     |Transmit FIFO Data Count (Read Only)
     * |        |          |This bit field indicates the valid data count of transmit FIFO buffer.
     * @var SPI_T::TX
     * Offset: 0x20  SPI Data Transmit Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TX        |Data Transmit Register
     * |        |          |The data transmit registers pass through the transmitted data into the 4-level transmit FIFO buffers
     * |        |          |The number of valid bits depends on the setting of DWIDTH (SPIx_CTL[12:8]) in SPI mode or WDWIDTH (SPIx_I2SCTL[5:4]) in I2S mode.
     * |        |          |In SPI mode, if DWIDTH is set to 0x08, the bits TX[7:0] will be transmitted
     * |        |          |If DWIDTH is set to 0x00 , the SPI controller will perform a 32-bit transfer.
     * |        |          |In I2S mode, if WDWIDTH (SPIx_I2SCTL[5:4]) is set to 0x2, the data width of audio channel is 24-bit and corresponding to TX[23:0]
     * |        |          |If WDWIDTH is set as 0x0, 0x1, or 0x3, all bits of this field are valid and referred to the data arrangement in I2S mode FIFO operation section
     * |        |          |Note: In Master mode, SPI controller will start to transfer the SPI bus clock after 1 APB clock and 6 peripheral clock cycles after user writes to this register.
     * @var SPI_T::RX
     * Offset: 0x30  SPI Data Receive Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RX        |Data Receive Register
     * |        |          |There are 4-level FIFO buffers in this controller
     * |        |          |The data receive register holds the data received from SPI data input pin
     * |        |          |If the RXEMPTY (SPIx_STATUS[8] or SPIx_I2SSTS[8]) is not set to 1, the receive FIFO buffers can be accessed through software by reading this register
     * |        |          |This is a read only register.
     */
    uint32_t CTL;                   /*!< [0x0000] SPI Control Register                                             */
    uint32_t CLKDIV;                /*!< [0x0004] SPI Clock Divider Register                                       */
    uint32_t SSCTL;                 /*!< [0x0008] SPI Slave Select Control Register                                */
    uint32_t PDMACTL;               /*!< [0x000c] SPI PDMA Control Register                                        */
    uint32_t FIFOCTL;               /*!< [0x0010] SPI FIFO Control Register                                        */
    uint32_t STATUS;                /*!< [0x0014] SPI Status Register                                              */
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVE0[2];
    /// @endcond //HIDDEN_SYMBOLS
    uint32_t TX;                    /*!< [0x0020] SPI Data Transmit Register                                       */
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVE1[3];
    /// @endcond //HIDDEN_SYMBOLS
    uint32_t RX;                    /*!< [0x0030] SPI Data Receive Register                                        */
    /// @cond HIDDEN_SYMBOLS
    //uint32_t RESERVE2[11];
    /// @endcond //HIDDEN_SYMBOLS
    int transfer_busy;
    int active_cs;
};

/**
    @addtogroup SPI_CONST SPI Bit Field Definition
    Constant Definitions for SPI Controller
@{ */

#define SPI_CTL_SPIEN_Pos                (0)                                               /*!< SPI_T::CTL: SPIEN Position             */
#define SPI_CTL_SPIEN_Msk                (0x1ul << SPI_CTL_SPIEN_Pos)                      /*!< SPI_T::CTL: SPIEN Mask                 */

#define SPI_CTL_RXNEG_Pos                (1)                                               /*!< SPI_T::CTL: RXNEG Position             */
#define SPI_CTL_RXNEG_Msk                (0x1ul << SPI_CTL_RXNEG_Pos)                      /*!< SPI_T::CTL: RXNEG Mask                 */

#define SPI_CTL_TXNEG_Pos                (2)                                               /*!< SPI_T::CTL: TXNEG Position             */
#define SPI_CTL_TXNEG_Msk                (0x1ul << SPI_CTL_TXNEG_Pos)                      /*!< SPI_T::CTL: TXNEG Mask                 */

#define SPI_CTL_CLKPOL_Pos               (3)                                               /*!< SPI_T::CTL: CLKPOL Position            */
#define SPI_CTL_CLKPOL_Msk               (0x1ul << SPI_CTL_CLKPOL_Pos)                     /*!< SPI_T::CTL: CLKPOL Mask                */

#define SPI_CTL_SUSPITV_Pos              (4)                                               /*!< SPI_T::CTL: SUSPITV Position           */
#define SPI_CTL_SUSPITV_Msk              (0xful << SPI_CTL_SUSPITV_Pos)                    /*!< SPI_T::CTL: SUSPITV Mask               */

#define SPI_CTL_DWIDTH_Pos               (8)                                               /*!< SPI_T::CTL: DWIDTH Position            */
#define SPI_CTL_DWIDTH_Msk               (0x1ful << SPI_CTL_DWIDTH_Pos)                    /*!< SPI_T::CTL: DWIDTH Mask                */

#define SPI_CTL_LSB_Pos                  (13)                                              /*!< SPI_T::CTL: LSB Position               */
#define SPI_CTL_LSB_Msk                  (0x1ul << SPI_CTL_LSB_Pos)                        /*!< SPI_T::CTL: LSB Mask                   */

#define SPI_CTL_HALFDPX_Pos              (14)                                              /*!< SPI_T::CTL: HALFDPX Position           */
#define SPI_CTL_HALFDPX_Msk              (0x1ul << SPI_CTL_HALFDPX_Pos)                    /*!< SPI_T::CTL: HALFDPX Mask               */

#define SPI_CTL_RXONLY_Pos               (15)                                              /*!< SPI_T::CTL: RXONLY Position            */
#define SPI_CTL_RXONLY_Msk               (0x1ul << SPI_CTL_RXONLY_Pos)                     /*!< SPI_T::CTL: RXONLY Mask                */

#define SPI_CTL_UNITIEN_Pos              (17)                                              /*!< SPI_T::CTL: UNITIEN Position           */
#define SPI_CTL_UNITIEN_Msk              (0x1ul << SPI_CTL_UNITIEN_Pos)                    /*!< SPI_T::CTL: UNITIEN Mask               */

#define SPI_CTL_SLAVE_Pos                (18)                                              /*!< SPI_T::CTL: SLAVE Position             */
#define SPI_CTL_SLAVE_Msk                (0x1ul << SPI_CTL_SLAVE_Pos)                      /*!< SPI_T::CTL: SLAVE Mask                 */

#define SPI_CTL_REORDER_Pos              (19)                                              /*!< SPI_T::CTL: REORDER Position           */
#define SPI_CTL_REORDER_Msk              (0x1ul << SPI_CTL_REORDER_Pos)                    /*!< SPI_T::CTL: REORDER Mask               */

#define SPI_CTL_DATDIR_Pos               (20)                                              /*!< SPI_T::CTL: DATDIR Position            */
#define SPI_CTL_DATDIR_Msk               (0x1ul << SPI_CTL_DATDIR_Pos)                     /*!< SPI_T::CTL: DATDIR Mask                */

#define SPI_CLKDIV_DIVIDER_Pos           (0)                                               /*!< SPI_T::CLKDIV: DIVIDER Position        */
#define SPI_CLKDIV_DIVIDER_Msk           (0x1fful << SPI_CLKDIV_DIVIDER_Pos)               /*!< SPI_T::CLKDIV: DIVIDER Mask            */

#define SPI_SSCTL_SS_Pos                 (0)                                               /*!< SPI_T::SSCTL: SS Position              */
#define SPI_SSCTL_SS_Msk                 (0x1ul << SPI_SSCTL_SS_Pos)                       /*!< SPI_T::SSCTL: SS Mask                  */

#define SPI_SSCTL_SSACTPOL_Pos           (2)                                               /*!< SPI_T::SSCTL: SSACTPOL Position        */
#define SPI_SSCTL_SSACTPOL_Msk           (0x1ul << SPI_SSCTL_SSACTPOL_Pos)                 /*!< SPI_T::SSCTL: SSACTPOL Mask            */

#define SPI_SSCTL_AUTOSS_Pos             (3)                                               /*!< SPI_T::SSCTL: AUTOSS Position          */
#define SPI_SSCTL_AUTOSS_Msk             (0x1ul << SPI_SSCTL_AUTOSS_Pos)                   /*!< SPI_T::SSCTL: AUTOSS Mask              */

#define QSPI_SSCTL_SLV3WIRE_Pos          (4)                                                /*!< QSPI_T::SSCTL: SLV3WIRE Position        */
#define QSPI_SSCTL_SLV3WIRE_Msk          (0x1ul << QSPI_SSCTL_SLV3WIRE_Pos)                 /*!< QSPI_T::SSCTL: SLV3WIRE Mask            */

#define QSPI_SSCTL_SLVTOIEN_Pos          (5)                                                /*!< QSPI_T::SSCTL: SLVTOIEN Position        */
#define QSPI_SSCTL_SLVTOIEN_Msk          (0x1ul << QSPI_SSCTL_SLVTOIEN_Pos)                 /*!< QSPI_T::SSCTL: SLVTOIEN Mask            */

#define QSPI_SSCTL_SLVTORST_Pos          (6)                                                /*!< QSPI_T::SSCTL: SLVTORST Position        */
#define QSPI_SSCTL_SLVTORST_Msk          (0x1ul << QSPI_SSCTL_SLVTORST_Pos)                 /*!< QSPI_T::SSCTL: SLVTORST Mask            */

#define SPI_SSCTL_SLVBEIEN_Pos           (8)                                               /*!< SPI_T::SSCTL: SLVBEIEN Position        */
#define SPI_SSCTL_SLVBEIEN_Msk           (0x1ul << SPI_SSCTL_SLVBEIEN_Pos)                 /*!< SPI_T::SSCTL: SLVBEIEN Mask            */

#define SPI_SSCTL_SLVURIEN_Pos           (9)                                               /*!< SPI_T::SSCTL: SLVURIEN Position        */
#define SPI_SSCTL_SLVURIEN_Msk           (0x1ul << SPI_SSCTL_SLVURIEN_Pos)                 /*!< SPI_T::SSCTL: SLVURIEN Mask            */

#define SPI_SSCTL_SSACTIEN_Pos           (12)                                              /*!< SPI_T::SSCTL: SSACTIEN Position        */
#define SPI_SSCTL_SSACTIEN_Msk           (0x1ul << SPI_SSCTL_SSACTIEN_Pos)                 /*!< SPI_T::SSCTL: SSACTIEN Mask            */

#define SPI_SSCTL_SSINAIEN_Pos           (13)                                              /*!< SPI_T::SSCTL: SSINAIEN Position        */
#define SPI_SSCTL_SSINAIEN_Msk           (0x1ul << SPI_SSCTL_SSINAIEN_Pos)                 /*!< SPI_T::SSCTL: SSINAIEN Mask            */

#define SPI_SSCTL_SLVTOCNT_Pos           (16)                                              /*!< SPI_T::SSCTL: SLVTOCNT Position        */
#define SPI_SSCTL_SLVTOCNT_Msk           (0xfffful << SPI_SSCTL_SLVTOCNT_Pos)              /*!< SPI_T::SSCTL: SLVTOCNT Mask            */

#define SPI_PDMACTL_TXPDMAEN_Pos         (0)                                               /*!< SPI_T::PDMACTL: TXPDMAEN Position      */
#define SPI_PDMACTL_TXPDMAEN_Msk         (0x1ul << SPI_PDMACTL_TXPDMAEN_Pos)               /*!< SPI_T::PDMACTL: TXPDMAEN Mask          */

#define SPI_PDMACTL_RXPDMAEN_Pos         (1)                                               /*!< SPI_T::PDMACTL: RXPDMAEN Position      */
#define SPI_PDMACTL_RXPDMAEN_Msk         (0x1ul << SPI_PDMACTL_RXPDMAEN_Pos)               /*!< SPI_T::PDMACTL: RXPDMAEN Mask          */

#define SPI_PDMACTL_PDMARST_Pos          (2)                                               /*!< SPI_T::PDMACTL: PDMARST Position       */
#define SPI_PDMACTL_PDMARST_Msk          (0x1ul << SPI_PDMACTL_PDMARST_Pos)                /*!< SPI_T::PDMACTL: PDMARST Mask           */

#define SPI_FIFOCTL_RXRST_Pos            (0)                                               /*!< SPI_T::FIFOCTL: RXRST Position         */
#define SPI_FIFOCTL_RXRST_Msk            (0x1ul << SPI_FIFOCTL_RXRST_Pos)                  /*!< SPI_T::FIFOCTL: RXRST Mask             */

#define SPI_FIFOCTL_TXRST_Pos            (1)                                               /*!< SPI_T::FIFOCTL: TXRST Position         */
#define SPI_FIFOCTL_TXRST_Msk            (0x1ul << SPI_FIFOCTL_TXRST_Pos)                  /*!< SPI_T::FIFOCTL: TXRST Mask             */

#define SPI_FIFOCTL_RXTHIEN_Pos          (2)                                               /*!< SPI_T::FIFOCTL: RXTHIEN Position       */
#define SPI_FIFOCTL_RXTHIEN_Msk          (0x1ul << SPI_FIFOCTL_RXTHIEN_Pos)                /*!< SPI_T::FIFOCTL: RXTHIEN Mask           */

#define SPI_FIFOCTL_TXTHIEN_Pos          (3)                                               /*!< SPI_T::FIFOCTL: TXTHIEN Position       */
#define SPI_FIFOCTL_TXTHIEN_Msk          (0x1ul << SPI_FIFOCTL_TXTHIEN_Pos)                /*!< SPI_T::FIFOCTL: TXTHIEN Mask           */

#define SPI_FIFOCTL_RXTOIEN_Pos          (4)                                               /*!< SPI_T::FIFOCTL: RXTOIEN Position       */
#define SPI_FIFOCTL_RXTOIEN_Msk          (0x1ul << SPI_FIFOCTL_RXTOIEN_Pos)                /*!< SPI_T::FIFOCTL: RXTOIEN Mask           */

#define SPI_FIFOCTL_RXOVIEN_Pos          (5)                                               /*!< SPI_T::FIFOCTL: RXOVIEN Position       */
#define SPI_FIFOCTL_RXOVIEN_Msk          (0x1ul << SPI_FIFOCTL_RXOVIEN_Pos)                /*!< SPI_T::FIFOCTL: RXOVIEN Mask           */

#define SPI_FIFOCTL_TXUFPOL_Pos          (6)                                               /*!< SPI_T::FIFOCTL: TXUFPOL Position       */
#define SPI_FIFOCTL_TXUFPOL_Msk          (0x1ul << SPI_FIFOCTL_TXUFPOL_Pos)                /*!< SPI_T::FIFOCTL: TXUFPOL Mask           */

#define SPI_FIFOCTL_TXUFIEN_Pos          (7)                                               /*!< SPI_T::FIFOCTL: TXUFIEN Position       */
#define SPI_FIFOCTL_TXUFIEN_Msk          (0x1ul << SPI_FIFOCTL_TXUFIEN_Pos)                /*!< SPI_T::FIFOCTL: TXUFIEN Mask           */

#define SPI_FIFOCTL_RXFBCLR_Pos          (8)                                               /*!< SPI_T::FIFOCTL: RXFBCLR Position       */
#define SPI_FIFOCTL_RXFBCLR_Msk          (0x1ul << SPI_FIFOCTL_RXFBCLR_Pos)                /*!< SPI_T::FIFOCTL: RXFBCLR Mask           */

#define SPI_FIFOCTL_TXFBCLR_Pos          (9)                                               /*!< SPI_T::FIFOCTL: TXFBCLR Position       */
#define SPI_FIFOCTL_TXFBCLR_Msk          (0x1ul << SPI_FIFOCTL_TXFBCLR_Pos)                /*!< SPI_T::FIFOCTL: TXFBCLR Mask           */

#define SPI_FIFOCTL_RXTH_Pos             (24)                                              /*!< SPI_T::FIFOCTL: RXTH Position          */
#define SPI_FIFOCTL_RXTH_Msk             (0x7ul << SPI_FIFOCTL_RXTH_Pos)                   /*!< SPI_T::FIFOCTL: RXTH Mask              */

#define SPI_FIFOCTL_TXTH_Pos             (28)                                              /*!< SPI_T::FIFOCTL: TXTH Position          */
#define SPI_FIFOCTL_TXTH_Msk             (0x7ul << SPI_FIFOCTL_TXTH_Pos)                   /*!< SPI_T::FIFOCTL: TXTH Mask              */

#define SPI_STATUS_BUSY_Pos              (0)                                               /*!< SPI_T::STATUS: BUSY Position           */
#define SPI_STATUS_BUSY_Msk              (0x1ul << SPI_STATUS_BUSY_Pos)                    /*!< SPI_T::STATUS: BUSY Mask               */

#define SPI_STATUS_UNITIF_Pos            (1)                                               /*!< SPI_T::STATUS: UNITIF Position         */
#define SPI_STATUS_UNITIF_Msk            (0x1ul << SPI_STATUS_UNITIF_Pos)                  /*!< SPI_T::STATUS: UNITIF Mask             */

#define SPI_STATUS_SSACTIF_Pos           (2)                                               /*!< SPI_T::STATUS: SSACTIF Position        */
#define SPI_STATUS_SSACTIF_Msk           (0x1ul << SPI_STATUS_SSACTIF_Pos)                 /*!< SPI_T::STATUS: SSACTIF Mask            */

#define SPI_STATUS_SSINAIF_Pos           (3)                                               /*!< SPI_T::STATUS: SSINAIF Position        */
#define SPI_STATUS_SSINAIF_Msk           (0x1ul << SPI_STATUS_SSINAIF_Pos)                 /*!< SPI_T::STATUS: SSINAIF Mask            */

#define SPI_STATUS_SSLINE_Pos            (4)                                               /*!< SPI_T::STATUS: SSLINE Position         */
#define SPI_STATUS_SSLINE_Msk            (0x1ul << SPI_STATUS_SSLINE_Pos)                  /*!< SPI_T::STATUS: SSLINE Mask             */

#define SPI_STATUS_SLVBEIF_Pos           (6)                                               /*!< SPI_T::STATUS: SLVBEIF Position        */
#define SPI_STATUS_SLVBEIF_Msk           (0x1ul << SPI_STATUS_SLVBEIF_Pos)                 /*!< SPI_T::STATUS: SLVBEIF Mask            */

#define SPI_STATUS_SLVURIF_Pos           (7)                                               /*!< SPI_T::STATUS: SLVURIF Position        */
#define SPI_STATUS_SLVURIF_Msk           (0x1ul << SPI_STATUS_SLVURIF_Pos)                 /*!< SPI_T::STATUS: SLVURIF Mask            */

#define SPI_STATUS_RXEMPTY_Pos           (8)                                               /*!< SPI_T::STATUS: RXEMPTY Position        */
#define SPI_STATUS_RXEMPTY_Msk           (0x1ul << SPI_STATUS_RXEMPTY_Pos)                 /*!< SPI_T::STATUS: RXEMPTY Mask            */

#define SPI_STATUS_RXFULL_Pos            (9)                                               /*!< SPI_T::STATUS: RXFULL Position         */
#define SPI_STATUS_RXFULL_Msk            (0x1ul << SPI_STATUS_RXFULL_Pos)                  /*!< SPI_T::STATUS: RXFULL Mask             */

#define SPI_STATUS_RXTHIF_Pos            (10)                                              /*!< SPI_T::STATUS: RXTHIF Position         */
#define SPI_STATUS_RXTHIF_Msk            (0x1ul << SPI_STATUS_RXTHIF_Pos)                  /*!< SPI_T::STATUS: RXTHIF Mask             */

#define SPI_STATUS_RXOVIF_Pos            (11)                                              /*!< SPI_T::STATUS: RXOVIF Position         */
#define SPI_STATUS_RXOVIF_Msk            (0x1ul << SPI_STATUS_RXOVIF_Pos)                  /*!< SPI_T::STATUS: RXOVIF Mask             */

#define SPI_STATUS_RXTOIF_Pos            (12)                                              /*!< SPI_T::STATUS: RXTOIF Position         */
#define SPI_STATUS_RXTOIF_Msk            (0x1ul << SPI_STATUS_RXTOIF_Pos)                  /*!< SPI_T::STATUS: RXTOIF Mask             */

#define SPI_STATUS_SPIENSTS_Pos          (15)                                              /*!< SPI_T::STATUS: SPIENSTS Position       */
#define SPI_STATUS_SPIENSTS_Msk          (0x1ul << SPI_STATUS_SPIENSTS_Pos)                /*!< SPI_T::STATUS: SPIENSTS Mask           */

#define SPI_STATUS_TXEMPTY_Pos           (16)                                              /*!< SPI_T::STATUS: TXEMPTY Position        */
#define SPI_STATUS_TXEMPTY_Msk           (0x1ul << SPI_STATUS_TXEMPTY_Pos)                 /*!< SPI_T::STATUS: TXEMPTY Mask            */

#define SPI_STATUS_TXFULL_Pos            (17)                                              /*!< SPI_T::STATUS: TXFULL Position         */
#define SPI_STATUS_TXFULL_Msk            (0x1ul << SPI_STATUS_TXFULL_Pos)                  /*!< SPI_T::STATUS: TXFULL Mask             */

#define SPI_STATUS_TXTHIF_Pos            (18)                                              /*!< SPI_T::STATUS: TXTHIF Position         */
#define SPI_STATUS_TXTHIF_Msk            (0x1ul << SPI_STATUS_TXTHIF_Pos)                  /*!< SPI_T::STATUS: TXTHIF Mask             */

#define SPI_STATUS_TXUFIF_Pos            (19)                                              /*!< SPI_T::STATUS: TXUFIF Position         */
#define SPI_STATUS_TXUFIF_Msk            (0x1ul << SPI_STATUS_TXUFIF_Pos)                  /*!< SPI_T::STATUS: TXUFIF Mask             */

#define SPI_STATUS_TXRXRST_Pos           (23)                                              /*!< SPI_T::STATUS: TXRXRST Position        */
#define SPI_STATUS_TXRXRST_Msk           (0x1ul << SPI_STATUS_TXRXRST_Pos)                 /*!< SPI_T::STATUS: TXRXRST Mask            */

#define SPI_STATUS_RXCNT_Pos             (24)                                              /*!< SPI_T::STATUS: RXCNT Position          */
#define SPI_STATUS_RXCNT_Msk             (0xful << SPI_STATUS_RXCNT_Pos)                   /*!< SPI_T::STATUS: RXCNT Mask              */

#define SPI_STATUS_TXCNT_Pos             (28)                                              /*!< SPI_T::STATUS: TXCNT Position          */
#define SPI_STATUS_TXCNT_Msk             (0xful << SPI_STATUS_TXCNT_Pos)                   /*!< SPI_T::STATUS: TXCNT Mask              */

#define SPI_TX_TX_Pos                    (0)                                               /*!< SPI_T::TX: TX Position                 */
#define SPI_TX_TX_Msk                    (0xfffffffful << SPI_TX_TX_Pos)                   /*!< SPI_T::TX: TX Mask                     */

#define SPI_RX_RX_Pos                    (0)                                               /*!< SPI_T::RX: RX Position                 */
#define SPI_RX_RX_Msk                    (0xfffffffful << SPI_RX_RX_Pos)                   /*!< SPI_T::RX: RX Mask                     */

/**@}*/ /* SPI_CONST */
/**@}*/ /* end of SPI register group */
/**@}*/ /* end of REGISTER group */
#define QSPI0                 ((QSPI_T *)  QSPI0_BA)
#define SPI0                  ((SPI_T *)   SPI0_BA)
#define SPI1                  ((SPI_T *)   SPI1_BA)

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SPI_Driver SPI Driver
  @{
*/

/** @addtogroup SPI_EXPORTED_CONSTANTS SPI Exported Constants
  @{
*/
#define SPI_MODE_0        (SPI_CTL_TXNEG_Msk)                             /*!< CLKPOL=0; RXNEG=0; TXNEG=1 \hideinitializer */
#define SPI_MODE_1        (SPI_CTL_RXNEG_Msk)                             /*!< CLKPOL=0; RXNEG=1; TXNEG=0 \hideinitializer */
#define SPI_MODE_2        (SPI_CTL_CLKPOL_Msk | SPI_CTL_RXNEG_Msk)        /*!< CLKPOL=1; RXNEG=1; TXNEG=0 \hideinitializer */
#define SPI_MODE_3        (SPI_CTL_CLKPOL_Msk | SPI_CTL_TXNEG_Msk)        /*!< CLKPOL=1; RXNEG=0; TXNEG=1 \hideinitializer */

#define SPI_SLAVE         (SPI_CTL_SLAVE_Msk)                             /*!< Set as slave \hideinitializer */
#define SPI_MASTER        (0x0U)                                          /*!< Set as master \hideinitializer */

#define SPI_SS                (SPI_SSCTL_SS_Msk)                          /*!< Set SS \hideinitializer */
#define SPI_SS_ACTIVE_HIGH    (SPI_SSCTL_SSACTPOL_Msk)                    /*!< SS active high \hideinitializer */
#define SPI_SS_ACTIVE_LOW     (0x0U)                                      /*!< SS active low \hideinitializer */

/* SPI Interrupt Mask */
#define SPI_UNIT_INT_MASK                (0x001U)                          /*!< Unit transfer interrupt mask \hideinitializer */
#define SPI_SSACT_INT_MASK               (0x002U)                          /*!< Slave selection signal active interrupt mask \hideinitializer */
#define SPI_SSINACT_INT_MASK             (0x004U)                          /*!< Slave selection signal inactive interrupt mask \hideinitializer */
#define SPI_SLVUR_INT_MASK               (0x008U)                          /*!< Slave under run interrupt mask \hideinitializer */
#define SPI_SLVBE_INT_MASK               (0x010U)                          /*!< Slave bit count error interrupt mask \hideinitializer */
#define SPI_TXUF_INT_MASK                (0x040U)                          /*!< Slave TX underflow interrupt mask \hideinitializer */
#define SPI_FIFO_TXTH_INT_MASK           (0x080U)                          /*!< FIFO TX threshold interrupt mask \hideinitializer */
#define SPI_FIFO_RXTH_INT_MASK           (0x100U)                          /*!< FIFO RX threshold interrupt mask \hideinitializer */
#define SPI_FIFO_RXOV_INT_MASK           (0x200U)                          /*!< FIFO RX overrun interrupt mask \hideinitializer */
#define SPI_FIFO_RXTO_INT_MASK           (0x400U)                          /*!< FIFO RX time-out interrupt mask \hideinitializer */

/* SPI Status Mask */
#define SPI_BUSY_MASK                    (0x01U)                           /*!< Busy status mask \hideinitializer */
#define SPI_RX_EMPTY_MASK                (0x02U)                           /*!< RX empty status mask \hideinitializer */
#define SPI_RX_FULL_MASK                 (0x04U)                           /*!< RX full status mask \hideinitializer */
#define SPI_TX_EMPTY_MASK                (0x08U)                           /*!< TX empty status mask \hideinitializer */
#define SPI_TX_FULL_MASK                 (0x10U)                           /*!< TX full status mask \hideinitializer */
#define SPI_TXRX_RESET_MASK              (0x20U)                           /*!< TX or RX reset status mask \hideinitializer */
#define SPI_SPIEN_STS_MASK               (0x40U)                           /*!< SPIEN status mask \hideinitializer */
#define SPI_SSLINE_STS_MASK              (0x80U)                           /*!< SPIx_SS line status mask \hideinitializer */

/*@}*/ /* end of group SPI_EXPORTED_CONSTANTS */


/** @addtogroup SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief      Clear the unit transfer interrupt flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Write 1 to UNITIF bit of SPI_STATUS register to clear the unit transfer interrupt flag.
  * \hideinitializer
  */
#define SPI_CLR_UNIT_TRANS_INT_FLAG(spi)   ((spi)->STATUS = SPI_STATUS_UNITIF_Msk)

  /**
    * @brief      Trigger RX PDMA function.
    * @param[in]  spi The pointer of the specified SPI module.
    * @return     None.
    * @details    Set RXPDMAEN bit of SPI_PDMACTL register to enable RX PDMA transfer function.
    * \hideinitializer
    */
#define SPI_TRIGGER_RX_PDMA(spi)   ((spi)->PDMACTL |= SPI_PDMACTL_RXPDMAEN_Msk)

    /**
      * @brief      Trigger TX PDMA function.
      * @param[in]  spi The pointer of the specified SPI module.
      * @return     None.
      * @details    Set TXPDMAEN bit of SPI_PDMACTL register to enable TX PDMA transfer function.
      * \hideinitializer
      */
#define SPI_TRIGGER_TX_PDMA(spi)   ((spi)->PDMACTL |= SPI_PDMACTL_TXPDMAEN_Msk)

      /**
        * @brief      Disable RX PDMA transfer.
        * @param[in]  spi The pointer of the specified SPI module.
        * @return     None.
        * @details    Clear RXPDMAEN bit of SPI_PDMACTL register to disable RX PDMA transfer function.
        * \hideinitializer
        */
#define SPI_DISABLE_RX_PDMA(spi) ( (spi)->PDMACTL &= ~SPI_PDMACTL_RXPDMAEN_Msk )

        /**
          * @brief      Disable TX PDMA transfer.
          * @param[in]  spi The pointer of the specified SPI module.
          * @return     None.
          * @details    Clear TXPDMAEN bit of SPI_PDMACTL register to disable TX PDMA transfer function.
          * \hideinitializer
          */
#define SPI_DISABLE_TX_PDMA(spi) ( (spi)->PDMACTL &= ~SPI_PDMACTL_TXPDMAEN_Msk )

          /**
            * @brief      Get the count of available data in RX FIFO.
            * @param[in]  spi The pointer of the specified SPI module.
            * @return     The count of available data in RX FIFO.
            * @details    Read RXCNT (SPI_STATUS[27:24]) to get the count of available data in RX FIFO.
            * \hideinitializer
            */
#define SPI_GET_RX_FIFO_COUNT(spi)   (((spi)->STATUS & SPI_STATUS_RXCNT_Msk) >> SPI_STATUS_RXCNT_Pos)

            /**
              * @brief      Get the RX FIFO empty flag.
              * @param[in]  spi The pointer of the specified SPI module.
              * @retval     0 RX FIFO is not empty.
              * @retval     1 RX FIFO is empty.
              * @details    Read RXEMPTY bit of SPI_STATUS register to get the RX FIFO empty flag.
              * \hideinitializer
              */
#define SPI_GET_RX_FIFO_EMPTY_FLAG(spi)   (((spi)->STATUS & SPI_STATUS_RXEMPTY_Msk)>>SPI_STATUS_RXEMPTY_Pos)

              /**
                * @brief      Get the TX FIFO empty flag.
                * @param[in]  spi The pointer of the specified SPI module.
                * @retval     0 TX FIFO is not empty.
                * @retval     1 TX FIFO is empty.
                * @details    Read TXEMPTY bit of SPI_STATUS register to get the TX FIFO empty flag.
                * \hideinitializer
                */
#define SPI_GET_TX_FIFO_EMPTY_FLAG(spi)   (((spi)->STATUS & SPI_STATUS_TXEMPTY_Msk)>>SPI_STATUS_TXEMPTY_Pos)

                /**
                  * @brief      Get the TX FIFO full flag.
                  * @param[in]  spi The pointer of the specified SPI module.
                  * @retval     0 TX FIFO is not full.
                  * @retval     1 TX FIFO is full.
                  * @details    Read TXFULL bit of SPI_STATUS register to get the TX FIFO full flag.
                  * \hideinitializer
                  */
#define SPI_GET_TX_FIFO_FULL_FLAG(spi)   (((spi)->STATUS & SPI_STATUS_TXFULL_Msk)>>SPI_STATUS_TXFULL_Pos)

                  /**
                    * @brief      Get the datum read from RX register.
                    * @param[in]  spi The pointer of the specified SPI module.
                    * @return     Data in RX register.
                    * @details    Read SPI_RX register to get the received datum.
                    * \hideinitializer
                    */
#define SPI_READ_RX(spi)   ((spi)->RX)

                    /**
                      * @brief      Write datum to TX register.
                      * @param[in]  spi The pointer of the specified SPI module.
                      * @param[in]  u32TxData The datum which user attempt to transfer through SPI bus.
                      * @return     None.
                      * @details    Write u32TxData to SPI_TX register.
                      * \hideinitializer
                      */
#define SPI_WRITE_TX(spi, u32TxData)   ((spi)->TX = (u32TxData))

                      /**
                        * @brief      Set SPIx_SS pin to high state.
                        * @param[in]  spi The pointer of the specified SPI module.
                        * @return     None.
                        * @details    Disable automatic slave selection function and set SPIx_SS pin to high state.
                        * \hideinitializer
                        */
#define SPI_SET_SS_HIGH(spi)   ((spi)->SSCTL = ((spi)->SSCTL & (~SPI_SSCTL_AUTOSS_Msk)) | (SPI_SSCTL_SSACTPOL_Msk | SPI_SSCTL_SS_Msk))

                        /**
                          * @brief      Set SPIx_SS pin to low state.
                          * @param[in]  spi The pointer of the specified SPI module.
                          * @return     None.
                          * @details    Disable automatic slave selection function and set SPIx_SS pin to low state.
                          * \hideinitializer
                          */
#define SPI_SET_SS_LOW(spi)   ((spi)->SSCTL = ((spi)->SSCTL & (~(SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SSACTPOL_Msk))) | SPI_SSCTL_SS_Msk)

                          /**
                            * @brief      Enable Byte Reorder function.
                            * @param[in]  spi The pointer of the specified SPI module.
                            * @return     None.
                            * @details    Enable Byte Reorder function. The suspend interval depends on the setting of SUSPITV (SPI_CTL[7:4]).
                            * \hideinitializer
                            */
#define SPI_ENABLE_BYTE_REORDER(spi)   ((spi)->CTL |=  SPI_CTL_REORDER_Msk)

                            /**
                              * @brief      Disable Byte Reorder function.
                              * @param[in]  spi The pointer of the specified SPI module.
                              * @return     None.
                              * @details    Clear REORDER bit field of SPI_CTL register to disable Byte Reorder function.
                              * \hideinitializer
                              */
#define SPI_DISABLE_BYTE_REORDER(spi)   ((spi)->CTL &= ~SPI_CTL_REORDER_Msk)

                              /**
                                * @brief      Set the length of suspend interval.
                                * @param[in]  spi The pointer of the specified SPI module.
                                * @param[in]  u32SuspCycle Decides the length of suspend interval. It could be 0 ~ 15.
                                * @return     None.
                                * @details    Set the length of suspend interval according to u32SuspCycle.
                                *             The length of suspend interval is ((u32SuspCycle + 0.5) * the length of one SPI bus clock cycle).
                                * \hideinitializer
                                */
#define SPI_SET_SUSPEND_CYCLE(spi, u32SuspCycle)   ((spi)->CTL = ((spi)->CTL & ~SPI_CTL_SUSPITV_Msk) | ((u32SuspCycle) << SPI_CTL_SUSPITV_Pos))

                                /**
                                  * @brief      Set the SPI transfer sequence with LSB first.
                                  * @param[in]  spi The pointer of the specified SPI module.
                                  * @return     None.
                                  * @details    Set LSB bit of SPI_CTL register to set the SPI transfer sequence with LSB first.
                                  * \hideinitializer
                                  */
#define SPI_SET_LSB_FIRST(spi)   ((spi)->CTL |= SPI_CTL_LSB_Msk)

                                  /**
                                    * @brief      Set the SPI transfer sequence with MSB first.
                                    * @param[in]  spi The pointer of the specified SPI module.
                                    * @return     None.
                                    * @details    Clear LSB bit of SPI_CTL register to set the SPI transfer sequence with MSB first.
                                    * \hideinitializer
                                    */
#define SPI_SET_MSB_FIRST(spi)   ((spi)->CTL &= ~SPI_CTL_LSB_Msk)

                                    /**
                                      * @brief      Set the data width of a SPI transaction.
                                      * @param[in]  spi The pointer of the specified SPI module.
                                      * @param[in]  u32Width The bit width of one transaction.
                                      * @return     None.
                                      * @details    The data width can be 8 ~ 32 bits.
                                      * \hideinitializer
                                      */
#define SPI_SET_DATA_WIDTH(spi, u32Width)   ((spi)->CTL = ((spi)->CTL & ~SPI_CTL_DWIDTH_Msk) | (((u32Width)&0x1F) << SPI_CTL_DWIDTH_Pos))

                                      /**
                                        * @brief      Get the SPI busy state.
                                        * @param[in]  spi The pointer of the specified SPI module.
                                        * @retval     0 SPI controller is not busy.
                                        * @retval     1 SPI controller is busy.
                                        * @details    This macro will return the busy state of SPI controller.
                                        * \hideinitializer
                                        */
#define SPI_IS_BUSY(spi)   ( ((spi)->STATUS & SPI_STATUS_BUSY_Msk)>>SPI_STATUS_BUSY_Pos )

                                        /**
                                          * @brief      Enable SPI controller.
                                          * @param[in]  spi The pointer of the specified SPI module.
                                          * @return     None.
                                          * @details    Set SPIEN (SPI_CTL[0]) to enable SPI controller.
                                          * \hideinitializer
                                          */
#define SPI_ENABLE(spi)   ((spi)->CTL |= SPI_CTL_SPIEN_Msk)

                                          /**
                                            * @brief      Disable SPI controller.
                                            * @param[in]  spi The pointer of the specified SPI module.
                                            * @return     None.
                                            * @details    Clear SPIEN (SPI_CTL[0]) to disable SPI controller.
                                            * \hideinitializer
                                            */
#define SPI_DISABLE(spi)   ((spi)->CTL &= ~SPI_CTL_SPIEN_Msk)

static const VMStateDescription vmstate_nuc980_spi = {
    .name = "nuc980_spi",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        //VMSTATE_FIFO8(tx_fifo, NUC980SPI),
        //VMSTATE_FIFO8(rx_fifo, NUC980SPI),
        //VMSTATE_UINT32_ARRAY(regs, NUC980SPI, R_MAX),
        VMSTATE_UINT32(CTL, NUC980SPI),
        VMSTATE_UINT32(CLKDIV, NUC980SPI),
        VMSTATE_UINT32(SSCTL, NUC980SPI),
        //VMSTATE_UINT32_ARRAY(rxtx, NUC980SPI, 4),
        //VMSTATE_UINT32_ARRAY(rx, NUC980SPI, 4),
        //VMSTATE_UINT32_ARRAY(tx, NUC980SPI, 4),
        VMSTATE_END_OF_LIST()
    }
};


static Property nuc980_spi_properties[] = {
    DEFINE_PROP_UINT8("num-ss-bits", NUC980SPI, num_cs, 1),
    DEFINE_PROP_END_OF_LIST(),
};

static void
update_interrupt(NUC980SPI* spi)
{
    /*
    if ((spi->cntrl & (1 << 17)) && (spi->cntrl & (1 << 16))) {
        //fprintf(stderr, " spi update irq 1, %x\n", spi->tx[0]);
        qemu_set_irq(spi->irq, 1);
    }
    else {
        //fprintf(stderr, " spi update irq 0, %x\n", spi->tx[0]);
        qemu_set_irq(spi->irq, 0);
    }
    */
}

/* Assert the chip select specified in the UMA Control/Status Register. */
static void nuc980_spi_select(NUC980SPI* s, unsigned cs_id)
{
    trace_npcm7xx_fiu_select(DEVICE(s)->canonical_path, cs_id);

    if (cs_id < s->num_cs) {
        //qemu_irq_lower(s->cs_lines[cs_id]);
        qemu_irq_raise(s->cs_lines[cs_id]);
        s->active_cs = cs_id;
    }
    else {
        qemu_log_mask(LOG_GUEST_ERROR,
            "%s: UMA to CS%d; this module has only %d chip selects",
            DEVICE(s)->canonical_path, cs_id, s->num_cs);
        s->active_cs = -1;
    }
}

/* Deassert the currently active chip select. */
static void nuc980_spi_deselect(NUC980SPI* s)
{
    if (s->active_cs < 0) {
        return;
    }

    trace_npcm7xx_fiu_deselect(DEVICE(s)->canonical_path, s->active_cs);

    //qemu_irq_raise(s->cs_lines[s->active_cs]);
    qemu_irq_lower(s->cs_lines[s->active_cs]);
    s->active_cs = -1;
}

static void nuc980_spi_do_reset(NUC980SPI* s)
{
    //memset(s->regs, 0, sizeof s->regs);

    //s->regs[R_SPISSR] = ~0;
    //xlx_spi_update_irq(s);
    //xlx_spi_update_cs(s);
    s->CTL = 0x00000034;
    s->CLKDIV = 0;
    s->SSCTL = 0;
    s->PDMACTL = 0;
    s->FIFOCTL = 0x22000000;
    s->STATUS = 0x00050110;
    s->RX = s->TX = 0;
    s->transfer_busy = 0;

    fifo32_reset(&s->tx_fifo);
    fifo32_reset(&s->rx_fifo);
}

static void nuc980_spi_reset(DeviceState* d)
{
    nuc980_spi_do_reset(NUC980_SPI(d));
}

static uint64_t
spi_read(void* opaque, hwaddr addr, unsigned int size)
{
    NUC980SPI* s = opaque;
    uint32_t r = 0;
    switch (addr) {
    case 0x00:
        r = (s->CTL & (~1)) | (s->transfer_busy);
        break;
    case 0x04:
        r = s->CLKDIV;
        break;
    case 0x08:
        r = s->SSCTL;
        break;
    case 0x0c:
        r = s->PDMACTL;
        break;
    case 0x10:
        r = s->FIFOCTL;
        break;
    case 0x14:
        r =   SPI_STATUS_SPIENSTS_Msk
            | (fifo32_is_empty(&s->tx_fifo) ? 1 : 0) << SPI_STATUS_TXEMPTY_Pos
            | (fifo32_is_empty(&s->rx_fifo) ? 1 : 0) << SPI_STATUS_RXEMPTY_Pos
            | (fifo32_is_full(&s->tx_fifo) ? 1 : 0) << SPI_STATUS_TXFULL_Pos
            | (fifo32_is_full(&s->rx_fifo) ? 1: 0) << SPI_STATUS_RXFULL_Pos
            ;
        break;
    case 0x30:
        if (fifo32_is_empty(&s->rx_fifo)) {
            /* value is undefined */
            r = 0xdeadbeef;
        }
        else {
            /* read from the RX FIFO */
            r = fifo32_pop(&s->rx_fifo);
        }
        break;
    }

    DPRINTF("addr=" TARGET_FMT_plx " = %x\n", addr, r);
    //if (s->cntrl >> 17 & 0x01) {
        //fprintf(stderr, "addr=" TARGET_FMT_plx " = %x\n", addr, r);
    //}
    //if (addr >= 0x10 && addr <= 0x1c) {
    //    fprintf(stderr, "addr=" TARGET_FMT_plx " = %x\n", addr, r);
    //}
    return r;

}

static void
run_transfer(void* clientData)
{
    NUC980SPI* cspi = (NUC980SPI*)clientData;

    int Tx_NUM = (cspi->CTL >> 8) & 0x03;		// [9:8] Tx_NUM
    int Tx_BIT_LEN = (cspi->CTL >> 3) & 0x1f; // [7:3] Tx_BIT_LEN
    int i;
    uint32_t rx[4] = { 0, 0, 0, 0 };
    for (i = 0; i < (Tx_NUM + 1); i++)
    {
        if (!Tx_BIT_LEN) // 32 bit
        {
            uint8_t ret = ssi_transfer(cspi->spi, cspi->TX >> 0);
            //fprintf(stderr, " %02x => %08x\n", cspi->SPI_TxRx[i] >> 0 & 0xff, ret);
            rx[i] |= (ret & 0xff) << 24;

            ret = ssi_transfer(cspi->spi, cspi->TX >> 8);
            //fprintf(stderr, " %02x => %08x\n", cspi->SPI_TxRx[i] >> 8 & 0xff, ret);
            rx[i] |= (ret & 0xff) << 16;

            ret = ssi_transfer(cspi->spi, cspi->TX >> 16);
            //fprintf(stderr, " %02x => %08x\n", cspi->SPI_TxRx[i] >> 16 & 0xff, ret);
            rx[i] |= (ret & 0xff) << 8;

            ret = ssi_transfer(cspi->spi, cspi->TX >> 24);
            //fprintf(stderr, " %02x => %08x\n", cspi->SPI_TxRx[i] >> 24 & 0xff, ret);
            rx[i] |= (ret & 0xff) << 0;

            cspi->RX = rx[i];
        }
        else
        {
            rx[i] = ssi_transfer(cspi->spi, cspi->TX);
            cspi->RX = rx[i];
        }
    }

    cspi->transfer_busy = 0;
    //if (cspi->cntrl >> 17 & 0x01) { // Interrupt Enabled
    //    cspi->cntrl |= 1 << 16;
    //    update_interrupt(cspi);		// transfer done
    //}

}

static void nuc980_spi_flush_txfifo(NUC980SPI* s)
{
    uint32_t tx;
    uint32_t rx;
    int valid_bytes;
    DPRINTF("Begin: TX Fifo Size = %d, RX Fifo Size = %d\n",
        fifo32_num_used(&s->tx_fifo), fifo32_num_used(&s->rx_fifo));

    while (!fifo32_is_empty(&s->tx_fifo)) {
        int tx_burst = 0;
        int burst_length = (s->CTL & SPI_CTL_DWIDTH_Msk) >> SPI_CTL_DWIDTH_Pos;
        if (burst_length == 0)
            burst_length = 32;
        DPRINTF("Burst length = %d\n", burst_length);
        
        tx = fifo32_pop(&s->tx_fifo);

        DPRINTF("data tx:0x%08x\n", tx);

        tx_burst = (burst_length % 32) ? (burst_length % 32) : 32;

        rx = 0;
        valid_bytes = 0;
        while (tx_burst > 0) {
            uint8_t byte = tx >> (tx_burst - 8);
            uint32_t retval;
            DPRINTF("writing 0x%02x\n", (uint32_t)byte);

            /* We need to write one byte at a time */
            retval = ssi_transfer(s->spi, byte);

            DPRINTF("0x%08x read\n", retval);
            if (retval != 0x80000000) {
                rx = (rx << 8) | (retval & 0xff);
                valid_bytes++;
            }
            /* Remove 8 bits from the actual burst */
            tx_burst -= 8;
            burst_length -= 8;
        }

        DPRINTF("data rx:0x%08x\n", rx);

        if (valid_bytes) {
            if (fifo32_is_full(&s->rx_fifo)) {
                //s->regs[ECSPI_STATREG] |= ECSPI_STATREG_RO;
            }
            else {
                fifo32_push(&s->rx_fifo, rx);
            }
        }

        /*
        if (s->burst_length <= 0) {
            if (!imx_spi_is_multiple_master_burst(s)) {
                s->regs[ECSPI_STATREG] |= ECSPI_STATREG_TC;
                break;
            }
        }
        */
    }

    if (fifo32_is_empty(&s->tx_fifo)) {
        //s->regs[ECSPI_STATREG] |= ECSPI_STATREG_TC;
        //s->regs[ECSPI_CONREG] &= ~ECSPI_CONREG_XCH;
    }

    /* TODO: We should also use TDR and RDR bits */

    DPRINTF("End: TX Fifo Size = %d, RX Fifo Size = %d\n",
        fifo32_num_used(&s->tx_fifo), fifo32_num_used(&s->rx_fifo));
}

static void
spi_write(void* opaque, hwaddr addr,
    uint64_t val64, unsigned int size)
{
    NUC980SPI* s = opaque;
    uint32_t value = val64;
    switch (addr)
    {
    case 0x00:
        s->CTL = value;
        if ((value >> 17) & 0x01) {
            //fprintf(stderr, "\033[;32m[%ld] Enable SPI Interrupt %08x \033[0m\n", CycleCounter_Get(), value);

            if ((value >> 16) & 0x01) { // This bit is read only, but can be cleared by writing 1 to this bit.
                //s->cntrl &= ~(1 << 16); // clear interrupt flag
                //fprintf(stderr, "...clear SPI IF...\n");
                update_interrupt(s);
            }
        }

        if (value & 1) {	// GO BUSY
            //fprintf(stderr, "Start XCH ...\n");
            int Tx_NUM = (value >> 8) & 0x03;		// [9:8] Tx_NUM
            int Tx_BIT_LEN = (value >> 3) & 0x1f; // [7:3] Tx_BIT_LEN
            int t;

            s->transfer_busy = 1;
            //s->cntrl &= ~(1 << 16); // clear interrupt flag
            
            run_transfer(s);
            
        }        
        break;
    case 0x04:
        s->CLKDIV = value;
        break;
    case 0x08:
        s->SSCTL = value;
        if (value & 0x01)
            nuc980_spi_select(s, 0);
        else
            nuc980_spi_deselect(s);
        if (value & 0x02)
            nuc980_spi_select(s, 1);
        else
            nuc980_spi_deselect(s);
        break;
    case 0x0c:
        s->PDMACTL = value;
        break;
    case 0x10:
        s->FIFOCTL = value;
        break;
    case 0x14:
        s->STATUS = value;
        break;
    case 0x20:
        s->TX = value;
        if (fifo32_is_full(&s->tx_fifo)) {
            /* Ignore writes if queue is full */
            break;
        }
        fifo32_push(&s->tx_fifo, (uint32_t)value);
        if (!(s->CTL & SPI_CTL_SLAVE_Msk)
            // && (s->regs[ECSPI_CONREG] & ECSPI_CONREG_SMC)
            ) {
            /*
             * Start emitting if current channel is master and SMC bit is set.
             */
            nuc980_spi_flush_txfifo(s);
        }
        break;

    }

    DPRINTF("addr=" TARGET_FMT_plx " = %x\n", addr, value);
}

static const MemoryRegionOps spi_ops = {
    .read = spi_read,
    .write = spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4
    }
};

void nuc980_connect_flash(DeviceState* dev, int cs_no,
    const char* flash_type, DriveInfo* dinfo)
{
    DeviceState* flash;
    qemu_irq flash_cs;

    NUC980SPI* s = NUC980_SPI(dev);
    flash = qdev_new(flash_type);

    if (dinfo) {
        BlockBackend* blk = blk_by_legacy_dinfo(dinfo);
        int64_t size;

        /*
         * The block backend size should have already been 'validated' by
         * the creation of the m25p80 object.
         */
        size = blk_getlength(blk);
        if (size <= 0) {
            error_report("failed to get flash size");
            return;
        }
        else {
            info_report("blk_getlength: %lx\n", size);
        }

        qdev_prop_set_drive_err(flash, "drive", blk, &error_fatal);
    }
    qdev_realize_and_unref(flash, BUS(s->spi), &error_fatal);

    flash_cs = qdev_get_gpio_in_named(flash, SSI_GPIO_CS, 0);
    qdev_connect_gpio_out_named(DEVICE(s), "cs", cs_no, flash_cs);
}


static void nuc980_spi_realize(DeviceState* dev, Error** errp)
{
    SysBusDevice* sbd = SYS_BUS_DEVICE(dev);
    NUC980SPI* s = NUC980_SPI(dev);
    int i;

    s->spi = ssi_create_bus(dev, "spi");

    sysbus_init_irq(sbd, &s->irq);
    s->cs_lines = g_new0(qemu_irq, s->num_cs);
    for (i = 0; i < s->num_cs; ++i) {
        sysbus_init_irq(sbd, &s->cs_lines[i]);
    }
    qdev_init_gpio_out_named(DEVICE(s), s->cs_lines, "cs", s->num_cs);

    memory_region_init_io(&s->mmio, OBJECT(s), &spi_ops, s,
        "nuc980-spi", 0x100);
    sysbus_init_mmio(sbd, &s->mmio);

    s->irqline = -1;

    fifo32_create(&s->tx_fifo, /*FIFO_CAPACITY*/8);
    fifo32_create(&s->rx_fifo, /*FIFO_CAPACITY*/8);

    nuc980_connect_flash(dev, 0, "w25q256", drive_get(IF_MTD, 0, 0));
    //nuc980_connect_flash(dev, 0, "w25q80", drive_get(IF_MTD, 0, 0));
    
    //nuc980_connect_flash(dev, 0, "s25sl12801", drive_get(IF_MTD, 0, 0));
    //nuc980_connect_flash(dev, 0, "sst25vf032b", drive_get(IF_MTD, 0, 0));
    //nuc980_connect_flash(dev, 0, "W25Q128BV", drive_get(IF_MTD, 0, 0));
    //nuc980_connect_flash(dev, 0, "s25fl016k", drive_get(IF_MTD, 0, 0));
    
    
}

static void nuc980_spi_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);

    dc->realize = nuc980_spi_realize;
    dc->reset = nuc980_spi_reset;
    device_class_set_props(dc, nuc980_spi_properties);
    dc->vmsd = &vmstate_nuc980_spi;
}

static const TypeInfo nuc980_spi_info = {
    .name = TYPE_NUC980_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NUC980SPI),
    .class_init = nuc980_spi_class_init,
};

static void nuc980_spi_register_types(void)
{
    type_register_static(&nuc980_spi_info);
}

type_init(nuc980_spi_register_types)





