#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/register.h"
#include "qemu/bitops.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/irq.h"
#include "qemu/main-loop.h" /* iothread mutex */
#include "sysemu/dma.h"

#include "usbip.h"
#include "nuc970-usbd.h"
#include "usbip_server.h"

typedef struct {

    union {
        uint32_t EPDAT;
        uint8_t  EPDAT_BYTE;

    } ep;
    uint32_t EPINTSTS;
    uint32_t EPINTEN;
    uint32_t EPDATCNT;
    uint32_t EPRSPCTL;
    uint32_t EPMPS;
    uint32_t EPTXCNT;
    uint32_t EPCFG;
    uint32_t EPBUFSTART;
    uint32_t EPBUFEND;

} USBD_EP_T;

typedef struct {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    usbip_cfg_t usbip_cfg;
    QemuThread usbip_thread;
    QemuMutex usbip_mutex;
    QemuCond usbip_cond;

    uint32_t GINTSTS;
    uint32_t RESERVE0[1];
    uint32_t GINTEN;
    uint32_t RESERVE1[1];
    uint32_t BUSINTSTS;
    uint32_t BUSINTEN;
    uint32_t OPER;
    uint32_t FRAMECNT;
    uint32_t FADDR;
    uint32_t TEST;
    union {
        uint32_t CEPDAT;
        uint8_t  CEPDAT_BYTE;
    } cep;
    uint32_t CEPCTL;
    uint32_t CEPINTEN;
    uint32_t CEPINTSTS;
    uint32_t CEPTXCNT;
    uint32_t CEPRXCNT;
    uint32_t CEPDATCNT;
    uint32_t SETUP1_0;
    uint32_t SETUP3_2;
    uint32_t SETUP5_4;
    uint32_t SETUP7_6;
    uint32_t CEPBUFSTART;
    uint32_t CEPBUFEND;
    uint32_t DMACTL;
    uint32_t DMACNT;
    USBD_EP_T EP[12];
    uint32_t RESERVE2[303];
    uint32_t DMAADDR;
    uint32_t PHYCTL;

} USBD_T;

#define TYPE_NUC970_USBD "nuc970-usbd"
#define NUC970_USBD(obj) \
        OBJECT_CHECK(USBD_T, (obj), TYPE_NUC970_USBD)

#define USBIP
#ifdef USBIP

/* USB Descriptor Type */
#define DESC_DEVICE         0x01
#define DESC_CONFIG         0x02
#define DESC_STRING         0x03
#define DESC_INTERFACE      0x04
#define DESC_ENDPOINT       0x05
#define DESC_QUALIFIER      0x06
#define DESC_OTHERSPEED     0x07
#define DESC_IFPOWER        0x08
#define DESC_OTG            0x09

/* USB Descriptor Length */
#define LEN_DEVICE          18
#define LEN_QUALIFIER       10
#define LEN_CONFIG          9
#define LEN_INTERFACE       9
#define LEN_ENDPOINT        7
#define LEN_OTG             5
#define LEN_HID             9

/* USB Endpoint Type */
#define EP_ISO              0x01
#define EP_BULK             0x02
#define EP_INT              0x03

#define EP_INPUT            0x80
#define EP_OUTPUT           0x00

/* Device Descriptor */
const USB_DEVICE_DESCRIPTOR _dev_dsc =
{
    0x12,                   // Size of this descriptor in bytes
    0x01,                   // DEVICE descriptor type
    0x0200,                 // USB Spec Release Number in BCD format
    0x00,                   // Class Code
    0x00,                   // Subclass code
    0x00,                   // Protocol code
    0x40,                   // Max packet size for EP0, see usb_config.h
    0x0416,                 // Vendor ID
    0x5963,                 // Product ID
    0x0100,                 // Device release number in BCD format
    0x01,                   // Manufacturer string index
    0x02,                   // Product string index
    0x00,                   // Device serial number string index
    0x01                    // Number of possible configurations
};

const USB_DEVICE_QUALIFIER_DESCRIPTOR _dev_qua = {
    0x0A,       // bLength
    0x06,       // bDescriptorType
    0x0200,     // bcdUSB
    0x00,       // bDeviceClass
    0x00,       // bDeviceSubClass
    0x00,       // bDeviceProtocol
    0x40,       // bMaxPacketSize
    0x01,       // iSerialNumber
    0x00        //bNumConfigurations*/
};

//Configuration
typedef struct __attribute__((__packed__)) _CONFIG_MSC
{
    USB_CONFIGURATION_DESCRIPTOR dev_conf0;
    USB_INTERFACE_DESCRIPTOR dev_int0;
    USB_ENDPOINT_DESCRIPTOR dev_ep0;
    USB_ENDPOINT_DESCRIPTOR dev_ep1;
} CONFIG_MSC;

const CONFIG_MSC  configuration_msc = {
    {
        /* Configuration Descriptor */
        0x09,//sizeof(USB_CFG_DSC),    // Size of this descriptor in bytes
        USB_DESCRIPTOR_CONFIGURATION,                // CONFIGURATION descriptor type
        0x0020, //sizeof(CONFIG_CDC),                // Total length of data for this cfg
        1,                      // Number of interfaces in this cfg
        1,                      // Index value of this configuration
        0,                      // Configuration string index
        0xC0,
        50,                     // Max power consumption (2X mA)
        },
        {
            /* Interface Descriptor */
            0x09,//sizeof(USB_INTF_DSC),   // Size of this descriptor in bytes
            USB_DESCRIPTOR_INTERFACE,               // INTERFACE descriptor type
            0,                      // Interface Number
            0,                      // Alternate Setting Number
            2,                      // Number of endpoints in this intf
            0,                      // Class code
            0,                      // Subclass code
            0,                      // Protocol code
            0                       // Interface string index
            },
            {
                /* Endpoint Descriptor */
                0x07,/*sizeof(USB_EP_DSC)*/
                USB_DESCRIPTOR_ENDPOINT,    //Endpoint Descriptor
                0x81,                       //EndpointAddress
                0x02,                       //Attributes
                0x0200,                     //size
                0x01                        //Interval
                },{
                    /* Endpoint Descriptor */
                    0x07,/*sizeof(USB_EP_DSC)*/
                    USB_DESCRIPTOR_ENDPOINT,    //Endpoint Descriptor
                    0x02,                       //EndpointAddress
                    0x02,                       //Attributes
                    0x0200,                     //size
                    0x01                        //Interval
                }
};

const unsigned char _string_0[] = { // available languages descriptor
                0x04,
                USB_DESCRIPTOR_STRING,
                0x09,
                0x04
};

const unsigned char _string_1[] = { //
                0x16,
                USB_DESCRIPTOR_STRING, // bLength, bDscType
                'U', 0x00, //
                'S', 0x00, //
                'B', 0x00, //
                ' ', 0x00, //
                'D', 0x00, //
                'e', 0x00, //
                'v', 0x00, //
                'i', 0x00, //
                'c', 0x00, //
                'e', 0x00, //
};

const unsigned char _string_2[] = { //
                0x34,
                USB_DESCRIPTOR_STRING, //
                'N', 0x00, //
                'u', 0x00, //
                'v', 0x00, //
                'o', 0x00, //
                't', 0x00, //
                'o', 0x00, //
                'n', 0x00, //
                ' ', 0x00, //
                'A', 0x00, //
                'R', 0x00, //
                'M', 0x00, //
                ' ', 0x00, //
                '9', 0x00, //
                '2', 0x00, //
                '6', 0x00, //
                '-', 0x00, //
                'B', 0x00, //
                'a', 0x00, //
                's', 0x00, //
                'e', 0x00, //
                'd', 0x00, //
                ' ', 0x00, //
                'M', 0x00, //
                'C', 0x00, //
                'U', 0x00, //
};


const char* _configuration = (const char*)&configuration_msc;

const USB_INTERFACE_DESCRIPTOR* _interfaces[] = { &configuration_msc.dev_int0 };

const unsigned char* _strings[] = { _string_0, _string_1, _string_2 };

void handle_device_list(const USB_DEVICE_DESCRIPTOR* dev_dsc, OP_REP_DEVLIST* list)
{
    CONFIG_GEN* conf = (CONFIG_GEN*)_configuration;
    int i;
    list->header.version = htons(273);
    list->header.command = htons(5);
    list->header.status = 0;
    list->header.nExportedDevice = htonl(1);
    memset(list->device.usbPath, 0, 256);
    strcpy(list->device.usbPath, "/sys/devices/pci0000:00/0000:00:01.2/usb1/1-1");
    memset(list->device.busID, 0, 32);
    strcpy(list->device.busID, "1-1");
    list->device.busnum = htonl(1);
    list->device.devnum = htonl(2);
    list->device.speed = htonl(2);
    list->device.idVendor = htons(dev_dsc->idVendor);
    list->device.idProduct = htons(dev_dsc->idProduct);
    list->device.bcdDevice = htons(dev_dsc->bcdDevice);
    list->device.bDeviceClass = dev_dsc->bDeviceClass;
    list->device.bDeviceSubClass = dev_dsc->bDeviceSubClass;
    list->device.bDeviceProtocol = dev_dsc->bDeviceProtocol;
    list->device.bConfigurationValue = conf->dev_conf.bConfigurationValue;
    list->device.bNumConfigurations = dev_dsc->bNumConfigurations;
    list->device.bNumInterfaces = conf->dev_conf.bNumInterfaces;
    list->interfaces = malloc(list->device.bNumInterfaces * sizeof(OP_REP_DEVLIST_INTERFACE));
    for (i = 0; i < list->device.bNumInterfaces; i++)
    {
        list->interfaces[i].bInterfaceClass = _interfaces[i]->bInterfaceClass;
        list->interfaces[i].bInterfaceSubClass = _interfaces[i]->bInterfaceSubClass;
        list->interfaces[i].bInterfaceProtocol = _interfaces[i]->bInterfaceProtocol;
        list->interfaces[i].padding = 0;
    }
};

void handle_attach(const USB_DEVICE_DESCRIPTOR* dev_dsc, OP_REP_IMPORT* rep)
{
    CONFIG_GEN* conf = (CONFIG_GEN*)_configuration;

    rep->version = htons(273);
    rep->command = htons(3);
    rep->status = 0;
    memset(rep->usbPath, 0, 256);
    strcpy(rep->usbPath, "/sys/devices/pci0000:00/0000:00:01.2/usb1/1-1");
    memset(rep->busID, 0, 32);
    strcpy(rep->busID, "1-1");
    rep->busnum = htonl(1);
    rep->devnum = htonl(2);
    rep->speed = htonl(2);
    rep->idVendor = dev_dsc->idVendor;
    rep->idProduct = dev_dsc->idProduct;
    rep->bcdDevice = dev_dsc->bcdDevice;
    rep->bDeviceClass = dev_dsc->bDeviceClass;
    rep->bDeviceSubClass = dev_dsc->bDeviceSubClass;
    rep->bDeviceProtocol = dev_dsc->bDeviceProtocol;
    rep->bNumConfigurations = dev_dsc->bNumConfigurations;
    rep->bConfigurationValue = conf->dev_conf.bConfigurationValue;
    rep->bNumInterfaces = conf->dev_conf.bNumInterfaces;
}

void pack(int* data, int size)
{
    int i;
    size = size / 4;
    for (i = 0; i < size; i++)
    {
        data[i] = htonl(data[i]);
    }
    //swap setup
    i = data[size - 1];
    data[size - 1] = data[size - 2];
    data[size - 2] = i;
}

void unpack(int* data, int size)
{
    int i;
    size = size / 4;
    for (i = 0; i < size; i++)
    {
        data[i] = ntohl(data[i]);
    }
    //swap setup
    i = data[size - 1];
    data[size - 1] = data[size - 2];
    data[size - 2] = i;
}


void send_usb_req(int sockfd, USBIP_RET_SUBMIT* usb_req, char* data, unsigned int size, unsigned int status)
{
    usb_req->command = 0x3;
    usb_req->status = status;
    usb_req->actual_length = size;
    usb_req->start_frame = 0x0;
    usb_req->number_of_packets = 0x0;

    usb_req->setup = 0x0;
    usb_req->devid = 0x0;
    usb_req->direction = 0x0;
    usb_req->ep = 0x0;

    pack((int*)usb_req, sizeof(USBIP_RET_SUBMIT));

    if (send(sockfd, (char*)usb_req, sizeof(USBIP_RET_SUBMIT), 0) != sizeof(USBIP_RET_SUBMIT))
    {
        printf("send error : %s \n", strerror(errno));
        exit(-1);
    };

    if (size > 0)
    {
        if (send(sockfd, data, size, 0) != size)
        {
            printf("send error : %s \n", strerror(errno));
            exit(-1);
        };
    }
}

int handle_get_descriptor(int sockfd, StandardDeviceRequest* control_req, USBIP_RET_SUBMIT* usb_req)
{
    int handled = 0;
    printf("handle_get_descriptor %u [%u]\n", control_req->wValue1, control_req->wValue0);
    if (control_req->wValue1 == 0x1) // Device
    {
        printf("Device\n");
        handled = 1;
        send_usb_req(sockfd, usb_req, (char*)&_dev_dsc, sizeof(USB_DEVICE_DESCRIPTOR)/*control_req->wLength*/, 0);
    }
    if (control_req->wValue1 == 0x2) // configuration
    {
        printf("Configuration\n");
        handled = 1;
        send_usb_req(sockfd, usb_req, (char*)_configuration, control_req->wLength, 0);
    }
    if (control_req->wValue1 == 0x3) // string
    {
        char str[255];
        int i;
        memset(str, 0, 255);
        for (i = 0; i < (*_strings[control_req->wValue0] / 2) - 1; i++)
            str[i] = _strings[control_req->wValue0][i * 2 + 2];
        printf("String (%s)\n", str);
        handled = 1;
        send_usb_req(sockfd, usb_req, (char*)_strings[control_req->wValue0], *_strings[control_req->wValue0], 0);
    }
    if (control_req->wValue1 == 0x6) // qualifier
    {
        printf("Qualifier\n");
        handled = 1;
        send_usb_req(sockfd, usb_req, (char*)&_dev_qua, control_req->wLength, 0);
    }
    if (control_req->wValue1 == 0xA) // config status ???
    {
        printf("Unknow\n");
        handled = 1;
        send_usb_req(sockfd, usb_req, "", 0, 1);
    }
    return handled;
}

int handle_set_configuration(int sockfd, StandardDeviceRequest* control_req, USBIP_RET_SUBMIT* usb_req)
{
    int handled = 0;
    printf("handle_set_configuration %u[%u]\n", control_req->wValue1, control_req->wValue0);
    handled = 1;
    send_usb_req(sockfd, usb_req, "", 0, 0);
    return handled;
}

#define BSIZE 4096
char usbd_buffer[BSIZE + 1];
int  bsize = 0;

void hex_dump(unsigned char* buf, int size)
{
    int i;
    for (i = 0; i < size; i++) {
        printf("%02X ", buf[i]);
        if ((i + 1) % 16 == 0)
            printf("\n");
    }
    printf("\n");
}

void handle_data(int sockfd, USBIP_RET_SUBMIT* usb_req, int bl, USBD_T *usbd)
{
    printf(">> Data EP: %d, len: %d\n", usb_req->ep, bl);
    int ack;
    if (usb_req->ep == 0x01)
    {
        printf("EP1 received \n");

        if (usb_req->direction == 0) //input
        {
            printf("direction=input\n");
            bsize = recv(sockfd, (char*)usbd_buffer, bl, 0);

            //ack = 0x55aa55aa;
            //send_usb_req(sockfd, usb_req, (char*)&ack, 4, 0);

            usbd_buffer[bsize + 1] = 0; //string terminator
            //printf("received (%s)\n", usbd_buffer);
            hex_dump(usbd_buffer, bsize);
            //send_usb_req(sockfd, usb_req, "", 0, 0);
        }
        else
        {
            //if (_usbd_flash_type == USBD_FLASH_SPI)
            //{
            //    ack = 100;
            //}
            //else 
            if (bsize != 400) // XUSB and others
            {
                ack = bsize;
            }
            else {
                ack = 0x55aa55aa;   // DDR
            }

            //printf("direction=output, flash type: %d, ack: %08x\n", _usbd_flash_type, ack);

            send_usb_req(sockfd, usb_req, (char*)&ack, 4, 0);

        }
        usleep(500);
    }

    if ((usb_req->ep == 0x02))
    {
        printf("EP2 received \n");
        if (usb_req->direction == 0) //input
        {
            printf("direction=input\n");
            bsize = recv(sockfd, (char*)usbd_buffer, bl, 0);

            //ack = 0x55aa55aa;
            //send_usb_req(sockfd, usb_req, (char*)&ack, 4, 0);
            send_usb_req(sockfd, usb_req, "", 0, 0);

            usbd_buffer[bsize + 1] = 0; //string terminator
            //printf("received (%s)\n", usbd_buffer);
            hex_dump(usbd_buffer, bsize);
        }
        else //output
        {
            printf("direction=output\n");
            if (bsize != 0)
            {
                int i;
                for (i = 0; i < bsize; i++)//increment received char
                    usbd_buffer[i] += 1;
                hex_dump(usbd_buffer, bsize);
                send_usb_req(sockfd, usb_req, usbd_buffer, bsize, 0);
                //printf("sending (%s)\n", usbd_buffer);
                bsize = 0;
            }
            else
            {
                send_usb_req(sockfd, usb_req, "", 0, 0);
                usleep(500);
                printf("no data disponible\n");
            }
        }
    }

};

typedef struct usb_cmd
{
    uint8_t	bmRequestType;
    uint8_t	bRequest;
    uint16_t	wValue;
    uint16_t	wIndex;
    uint16_t	wLength;
}	USB_CMD_T;

void handle_unknown_control(int sockfd, StandardDeviceRequest* control_req, USBIP_RET_SUBMIT* usb_req, USBD_T* usbd)
{

    printf(">> Control Type: %d, %d\n", control_req->bmRequestType, control_req->bRequest);
    // vendor command
    USB_CMD_T _usb_cmd_pkt;
    memcpy(&_usb_cmd_pkt, control_req, sizeof(USB_CMD_T));

    if (_usb_cmd_pkt.bmRequestType == 0x40)
    {
        if (_usb_cmd_pkt.bRequest == 0xa0)
        {
            if (_usb_cmd_pkt.wValue == 0x12) {
                //Bulk_Out_Transfer_Size = _usb_cmd_pkt.wIndex;
                //printf(" >> Bulk_Out_Transfer_Size: %d\n", Bulk_Out_Transfer_Size);
                //outpw(REG_USBD_CEP_CTRL_STAT, CEP_NAK_CLEAR);	// clear nak so that sts stage is complete//lsshi
            }
            else if (_usb_cmd_pkt.wValue == 0x13) {
                // reset DMA
                //outpw(REG_USBD_DMA_CTRL_STS, 0x80);
                //outpw(REG_USBD_DMA_CTRL_STS, 0x00);
                //outpw(REG_USBD_EPA_RSP_SC, 0x01);		// flush fifo
                //outpw(REG_USBD_EPB_RSP_SC, 0x01);		// flush fifo

                //outpw(REG_USBD_CEP_CTRL_STAT, CEP_NAK_CLEAR);	// clear nak so that sts stage is complete
            }
            send_usb_req(sockfd, usb_req, "", 0, 0);
        }

        if (_usb_cmd_pkt.bRequest == 0xb0) {
            /*
            if (_usb_cmd_pkt.wValue == (USBD_BURN_TYPE + USBD_FLASH_SDRAM)) {
                _usbd_flash_type = USBD_FLASH_SDRAM;
                outpw(REG_USBD_CEP_CTRL_STAT, CEP_NAK_CLEAR);	// clear nak so that sts stage is complete//lsshi
                printf(" >> FLASH SDRAM\n");
            }
            else if (_usb_cmd_pkt.wValue == (USBD_BURN_TYPE + USBD_FLASH_NAND)) {
                _usbd_flash_type = USBD_FLASH_NAND;
                outpw(REG_USBD_CEP_CTRL_STAT, CEP_NAK_CLEAR);	// clear nak so that sts stage is complete//lsshi
                printf(" >> FLASH NAND\n");
            }
            else if (_usb_cmd_pkt.wValue == (USBD_BURN_TYPE + USBD_FLASH_NAND_RAW)) {
                _usbd_flash_type = USBD_FLASH_NAND_RAW;
                outpw(REG_USBD_CEP_CTRL_STAT, CEP_NAK_CLEAR);	// clear nak so that sts stage is complete//lsshi
                printf(" >> FLASH NAND RAW\n");
            }
            else if (_usb_cmd_pkt.wValue == (USBD_BURN_TYPE + USBD_FLASH_MMC)) {
                _usbd_flash_type = USBD_FLASH_MMC;
                outpw(REG_USBD_CEP_CTRL_STAT, CEP_NAK_CLEAR);	// clear nak so that sts stage is complete//lsshi
                printf(" >> FLASH MMC\n");
            }
            else if (_usb_cmd_pkt.wValue == (USBD_BURN_TYPE + USBD_FLASH_MMC_RAW)) {
                _usbd_flash_type = USBD_FLASH_MMC_RAW;
                outpw(REG_USBD_CEP_CTRL_STAT, CEP_NAK_CLEAR);	// clear nak so that sts stage is complete//lsshi
                printf(" >> FLASH MMC RAW\n");
            }
            else if (_usb_cmd_pkt.wValue == (USBD_BURN_TYPE + USBD_FLASH_SPI)) {
                _usbd_flash_type = USBD_FLASH_SPI;
                outpw(REG_USBD_CEP_CTRL_STAT, CEP_NAK_CLEAR);	// clear nak so that sts stage is complete//lsshi
                printf(" >> FLASH SPI\n");
            }
            else if (_usb_cmd_pkt.wValue == (USBD_BURN_TYPE + USBD_FLASH_SPI_RAW)) {
                _usbd_flash_type = USBD_FLASH_SPI_RAW;
                outpw(REG_USBD_CEP_CTRL_STAT, CEP_NAK_CLEAR);	// clear nak so that sts stage is complete//lsshi
                printf(" >> FLASH SPI RAW\n");
            }
            else if (_usb_cmd_pkt.wValue == (USBD_BURN_TYPE + USBD_MTP)) {
                _usbd_flash_type = USBD_MTP;
                outpw(REG_USBD_CEP_CTRL_STAT, CEP_NAK_CLEAR);	// clear nak so that sts stage is complete//lsshi
                printf(" >> FLASH MTP\n");
            }
            else if (_usb_cmd_pkt.wValue == (USBD_BURN_TYPE + USBD_INFO)) {
                _usbd_flash_type = USBD_INFO;
                printf(" >> FLASH USBD_INFO\n");
            }
            */
            //outpw(REG_USBD_CEP_CTRL_STAT, CEP_NAK_CLEAR);	// clear nak so that sts stage is complete//lsshi
            send_usb_req(sockfd, usb_req, "", 0, 0);
        }

        //outpw(REG_USBD_CEP_IRQ_STAT, 0x400);
        return;

    }

    /* vendor IN for ack */
    if (_usb_cmd_pkt.bmRequestType == 0xc0) {
        // clear flags
        //usbdClearAllFlags();

        //GET_VEN_Flag = 1;
        //outpw(REG_USBD_CEP_IRQ_STAT, 0x408);
        //outpw(REG_USBD_CEP_IRQ_ENB, 0x408);		//suppkt int ,status and in token
        int ack = _usb_cmd_pkt.wValue;
        send_usb_req(sockfd, usb_req, (char*)&ack, 4, 0);
        return;
    }
};

//http://www.usbmadesimple.co.uk/ums_4.htm

void handle_usb_control(int sockfd, USBIP_RET_SUBMIT* usb_req, USBD_T* usbd)
{
    int handled = 0;
    StandardDeviceRequest control_req;
#ifdef LINUX
    printf("%016llX\n", usb_req->setup);
#else
    printf("%016I64X\n", usb_req->setup);
#endif
    control_req.bmRequestType = (usb_req->setup & 0xFF00000000000000) >> 56;
    control_req.bRequest = (usb_req->setup & 0x00FF000000000000) >> 48;    
    control_req.wValue0 = (usb_req->setup & 0x0000FF0000000000) >> 40;
    control_req.wValue1 = (usb_req->setup & 0x000000FF00000000) >> 32;
    control_req.wIndex0 = (usb_req->setup & 0x00000000FF000000) >> 24;
    control_req.wIndex1 = (usb_req->setup & 0x0000000000FF0000) >> 16;
    control_req.wLength = ntohs(usb_req->setup & 0x000000000000FFFF);
    printf("  UC Request Type %u\n", control_req.bmRequestType);
    printf("  UC Request %u\n", control_req.bRequest);
    printf("  UC Value  %u[%u]\n", control_req.wValue1, control_req.wValue0);
    printf("  UCIndex  %u-%u\n", control_req.wIndex1, control_req.wIndex0);
    printf("  UC Length %u\n", control_req.wLength);

    // fill setup registers
    usbd->SETUP1_0 = (control_req.bRequest << 8) | control_req.bmRequestType;
    usbd->SETUP3_2 = (control_req.wValue1 << 8) | control_req.wValue0;
    usbd->SETUP5_4 = (control_req.wIndex1 << 8) | control_req.wIndex0;
    usbd->SETUP7_6 = control_req.wLength;

    if (control_req.bmRequestType == 0x80) // Host Request
    {
        if (control_req.bRequest == 0x06) // Get Descriptor
        {
            handled = handle_get_descriptor(sockfd, &control_req, usb_req);
        }
        if (control_req.bRequest == 0x00) // Get STATUS
        {
            char data[2];
            data[0] = 0x01;
            data[1] = 0x00;
            send_usb_req(sockfd, usb_req, data, 2, 0);
            handled = 1;
            printf("GET_STATUS\n");
        }
    }

    if (control_req.bmRequestType == 0x00) // 
    {
        if (control_req.bRequest == 0x09) // Set Configuration
        {
            handled = handle_set_configuration(sockfd, &control_req, usb_req);
        }
    }

    if (control_req.bmRequestType == 0x01)
    {
        if (control_req.bRequest == 0x0B) //SET_INTERFACE  
        {
            printf("SET_INTERFACE\n");
            send_usb_req(sockfd, usb_req, "", 0, 1);
            handled = 1;
        }
    }

    if (!handled)
        handle_unknown_control(sockfd, &control_req, usb_req, usbd);
}


void handle_usb_request(int sockfd, USBIP_RET_SUBMIT* ret, int bl, USBD_T* usbd)
{
    if (ret->ep == 0)
    {
        printf("#control requests\n");
        handle_usb_control(sockfd, ret, usbd);
    }
    else
    {
        printf("#data requests\n");
        handle_data(sockfd, ret, bl, usbd);
    }
};
#endif

static void nuc970_usbd_update_interrupt(USBD_T* usbd)
{
    if (usbd->GINTSTS & usbd->GINTEN) {
        qemu_irq_raise(usbd->irq);
    }
    else {
        qemu_irq_lower(usbd->irq);
    }
}

static void* nuc970_usbip_thread_run(void* opaque)
{
    //USBD_T* usbd = (USBD_T*)opaque;
    usbip_cfg_t* cfg = (usbip_cfg_t*)opaque;
    struct sockaddr_in serv, cli;
    int listenfd, sockfd, nb;
#ifdef __linux__
    unsigned int clilen;
#else
    int clilen;
#endif
    unsigned char attached;

#ifndef __linux__
    WSAStartup(wVersionRequested, &wsaData);
    if (wsaData.wVersion != wVersionRequested)
    {
        fprintf(stderr, "\n Wrong version\n");
        exit(-1);
    }

#endif

    if ((listenfd = socket(PF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("socket error : %s \n", strerror(errno));
        exit(1);
    };

    int reuse = 1;
    if (setsockopt(listenfd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0)
        perror("setsockopt(SO_REUSEADDR) failed");

    memset(&serv, 0, sizeof(serv));
    serv.sin_family = AF_INET;
    serv.sin_addr.s_addr = htonl(INADDR_ANY);
    serv.sin_port = htons(/*cfg->port*/TCP_SERV_PORT);

    if (bind(listenfd, (sockaddr*)&serv, sizeof(serv)) < 0)
    {
        printf("bind error : %s \n", strerror(errno));
        exit(1);
    };

    if (listen(listenfd, SOMAXCONN) < 0)
    {
        printf("listen error : %s \n", strerror(errno));
        exit(1);
    };

    for (;;)
    {
        clilen = sizeof(cli);
        if (
            (sockfd =
                accept(listenfd, (sockaddr*)&cli, &clilen)) < 0)
        {
            printf("accept error : %s \n", strerror(errno));
            exit(1);
        };
        printf("Connection address:%s\n", inet_ntoa(cli.sin_addr));
        attached = 0;
        cfg->_private_fd = sockfd;
        while (1)
        {
            if (!attached)
            {
                OP_REQ_DEVLIST req;
                if ((nb = recv(sockfd, (char*)&req, sizeof(OP_REQ_DEVLIST), 0)) != sizeof(OP_REQ_DEVLIST))
                {
                    //printf ("receive error : %s \n", strerror (errno));
                    break;
                };
#ifdef _DEBUG
                print_recv((char*)&req, sizeof(OP_REQ_DEVLIST), "OP_REQ_DEVLIST");
#endif
                req.command = ntohs(req.command);
                printf("Header Packet\n");
                printf("command: 0x%02X\n", req.command);
                if (req.command == 0x8005)
                {
                    OP_REP_DEVLIST list;
                    printf("list of devices\n");

                    handle_device_list(&_dev_dsc, &list);

                    if (send(sockfd, (char*)&list.header, sizeof(OP_REP_DEVLIST_HEADER), 0) != sizeof(OP_REP_DEVLIST_HEADER))
                    {
                        printf("send error : %s \n", strerror(errno));
                        break;
                    };
                    if (send(sockfd, (char*)&list.device, sizeof(OP_REP_DEVLIST_DEVICE), 0) != sizeof(OP_REP_DEVLIST_DEVICE))
                    {
                        printf("send error : %s \n", strerror(errno));
                        break;
                    };
                    if (send(sockfd, (char*)list.interfaces, sizeof(OP_REP_DEVLIST_INTERFACE) * list.device.bNumInterfaces, 0) != sizeof(OP_REP_DEVLIST_INTERFACE) * list.device.bNumInterfaces)
                    {
                        printf("send error : %s \n", strerror(errno));
                        break;
                    };
                    free(list.interfaces);
                }
                else if (req.command == 0x8003)
                {
                    char busid[32];
                    OP_REP_IMPORT rep;
                    printf("attach device\n");
                    if ((nb = recv(sockfd, busid, 32, 0)) != 32)
                    {
                        printf("receive error : %s \n", strerror(errno));
                        break;
                    };
#ifdef _DEBUG
                    print_recv(busid, 32, "Busid");
#endif
                    handle_attach(&_dev_dsc, &rep);
                    if (send(sockfd, (char*)&rep, sizeof(OP_REP_IMPORT), 0) != sizeof(OP_REP_IMPORT))
                    {
                        printf("send error : %s \n", strerror(errno));
                        break;
                    };
                    attached = 1;
                }
            }
            else
            {
                printf("------------------------------------------------\n");
                printf("handles requests\n");
                USBIP_CMD_SUBMIT cmd;
                USBIP_RET_SUBMIT usb_req;
                if ((nb = recv(sockfd, (char*)&cmd, sizeof(USBIP_CMD_SUBMIT), 0)) != sizeof(USBIP_CMD_SUBMIT))
                {
                    printf("receive error : %s nb: %d \n", strerror(errno), nb);
                    break;
                };
#ifdef _DEBUG
                print_recv((char*)&cmd, sizeof(USBIP_CMD_SUBMIT), "USBIP_CMD_SUBMIT");
#endif
                unpack((int*)&cmd, sizeof(USBIP_CMD_SUBMIT));
                printf("usbip cmd %u\n", cmd.command);
                printf("usbip seqnum %u\n", cmd.seqnum);
                printf("usbip devid %u\n", cmd.devid);
                printf("usbip direction %u\n", cmd.direction);
                printf("usbip ep %u\n", cmd.ep);
                printf("usbip flags %u\n", cmd.transfer_flags);
                printf("usbip number of packets %u\n", cmd.number_of_packets);
                printf("usbip interval %u\n", cmd.interval);
                //  printf("usbip setup %"PRI"\n",cmd.setup);
                printf("usbip buffer lenght  %u\n", cmd.transfer_buffer_length);
                usb_req.command = 0;
                usb_req.seqnum = cmd.seqnum;
                usb_req.devid = cmd.devid;
                usb_req.direction = cmd.direction;
                usb_req.ep = cmd.ep;
                usb_req.status = 0;
                usb_req.actual_length = 0;
                usb_req.start_frame = 0;
                usb_req.number_of_packets = 0;
                usb_req.error_count = 0;
                usb_req.setup = cmd.setup;

                if (cmd.command == 1)
                {
                    printf("usb_req ep %d\n", usb_req.ep);
                    USBIPServerClass* s = USBIP_SERVER_GET_CLASS(cfg->self);
                    s->usbip_handle_packet(cfg->self, &cmd, &usb_req);
                    //handle_usb_request(sockfd, &usb_req, cmd.transfer_buffer_length, usbd);
                }              

                if (cmd.command == 2) //unlink urb
                {
                    printf("####################### Unlink URB %u  (not working!!!)\n", cmd.transfer_flags);
                    //FIXME
                      /*
                       USBIP_RET_UNLINK ret;
                       printf("####################### Unlink URB %u\n",cmd.transfer_flags);
                       ret.command=htonl(0x04);
                       ret.devid=htonl(cmd.devid);
                       ret.direction=htonl(cmd.direction);
                       ret.ep=htonl(cmd.ep);
                       ret.seqnum=htonl(cmd.seqnum);
                       ret.status=htonl(1);

                       if (send (sockfd, (char *)&ret, sizeof(USBIP_RET_UNLINK), 0) != sizeof(USBIP_RET_UNLINK))
                       {
                         printf ("send error : %s \n", strerror (errno));
                         exit(-1);
                       };
                      */
                }

                if (cmd.command > 2)
                {
                    printf("Unknown USBIP cmd!\n");
                    close(sockfd);
                    cfg->_private_fd = 0;
#ifndef __linux__
                    WSACleanup();
#endif
                    return NULL;
                };

            }
        }
        close(sockfd);
        cfg->_private_fd = 0;
    };
#ifndef __linux__
    WSACleanup();
#endif
}

static uint64_t nuc970_usbd_read(void* opaque, hwaddr offset, unsigned size)
{
    USBD_T* s = (USBD_T*)opaque;
    uint32_t r = 0;

    switch (offset) {
    case 0x78:
        r = s->EP[0].EPMPS;
        break;
    default:
        r = 0xffffffff;
        break;
    }
    fprintf(stderr, "usbd_r (offset=%lx, value=%08x)\n", offset, r);
    return r;
}

static void nuc970_usbd_write(void* opaque, hwaddr offset, uint64_t value, unsigned size)
{
    USBD_T* s = (USBD_T*)opaque;
    switch (offset) {
    case 0x5c:
        if (value & (1 << 5)) { // DMAEN
            if (value & (1 << 4)) { // DMARD
                dma_memory_write(&address_space_memory, s->DMAADDR, usbd_buffer,
                    s->DMACNT, MEMTXATTRS_UNSPECIFIED);                
            }
            else {  // DMAWR
                dma_memory_read(&address_space_memory, s->DMAADDR, usbd_buffer,
                    s->DMACNT, MEMTXATTRS_UNSPECIFIED);
            }
            s->BUSINTSTS |= (1 << USBD_BUSINTEN_DMADONEIEN_Pos);    // DMADONEIF
        }
        if (value & (1 << 7)) {
            s->DMACNT = 0;
            s->DMACTL = 0x80;
            s->DMACTL = 0x00;
        }
        else {
            s->DMACTL = value;
        }
        break;
    case 0x78:
        s->EP[0].EPMPS = value;
        break;
         
    default:
        break;
    }
    fprintf(stderr, "usbd_w (offset=%lx, value=%08lx)\n", offset, value);
}

static const MemoryRegionOps nuc970_usbd_ops = {
    .read = nuc970_usbd_read,
    .write = nuc970_usbd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static void nuc970_usbd_reset(DeviceState* dev)
{
    USBD_T* s = NUC970_USBD(dev);
    unsigned int i;
    s->GINTSTS = s->GINTEN = 0;
    
    qemu_irq_lower(s->irq);
}

static void nuc970_usbd_init(Object* obj)
{
    USBD_T* s = NUC970_USBD(obj);
    SysBusDevice* sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, OBJECT(s), &nuc970_usbd_ops, s, "nuc970-usbd", 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    s->usbip_cfg.self = USBIP_SERVER(s);
}

static void nuc970_usbd_realize(DeviceState* dev, Error** errp)
{
    USBD_T* s = NUC970_USBD(dev);
    qemu_mutex_init(&s->usbip_mutex);
    qemu_cond_init(&s->usbip_cond);
    qemu_thread_create(&s->usbip_thread, "usbip", 
        nuc970_usbip_thread_run, &s->usbip_cfg, QEMU_THREAD_JOINABLE);
}

typedef union {
    uint32_t raw;
    struct {
        uint32_t CHNUM : 4;
        uint32_t BCNT : 11;
        uint32_t DPID : 2;
        uint32_t PKTSTS : 4;
        uint32_t : 11;
    } HOST QEMU_PACKED;
    struct {
        uint32_t EPNUM : 4;
        uint32_t BCNT : 11;
        uint32_t DPID : 2;
        uint32_t PKTSTS : 4;
        uint32_t FRMNUM : 4;
        uint32_t : 6;
        uint32_t INCOMPLETE : 1; // NB- this isn't used by the device, rather by us to handle multi-packet IN transfers.
    } DEV QEMU_PACKED;
} buffer_header_t;

static void nuc970_usbip_handle_packet(USBIPIF* self, USBIP_CMD_SUBMIT* cmd, USBIP_RET_SUBMIT* usb_req)
{
    printf("Handle packet called: Setup: %llx\n", cmd->setup);
    
    //STM32F4xxUSBState* s = STM32F4xx_USB(self);
    USBD_T* s = NUC970_USBD(self);
    printf("****MPS %d\n", s->EP[0].EPMPS);
    /*
    buffer_header_t header;
    char payload[255];
    uint8_t payload_size = 0;
    bool has_setup = false;
    header.raw = 0;
    header.DEV.EPNUM = usb_req->ep;
    header.DEV.BCNT = usb_req->actual_length;
    if (usb_req->direction && usb_req->ep == 0 && usb_req->actual_length == 0)
    {
        header.DEV.PKTSTS = BUFFER_PKTSTS_SETUP;
        has_setup = true;
    }
    else if (!usb_req->direction && usb_req->ep == 0 && usb_req->actual_length == 0)
    {
        if (cmd->transfer_buffer_length)
        {
            header.DEV.PKTSTS = BUFFER_PKTSTS_SETUP;
            header.DEV.INCOMPLETE = 1;
        }
        else
        {
            header.DEV.PKTSTS = BUFFER_PKTSTS_SETUP;
        }
        has_setup = true;
    }
    else if (usb_req->ep == 2 || usb_req->direction == 1)
    {
        // temporarily ignore EP2 interrupts and IN bulk
        return;
    }
    else if (usb_req->ep > 0 && usb_req->actual_length == 0)
    {
        header.DEV.PKTSTS = BUFFER_PKTSTS_OUT;
    }
    else
    {
        printf("UNHANDLED PKTSTS!\n");
    }
    if (cmd->transfer_buffer_length != 0 && cmd->direction == 0)
    {
        if (usbip_read_payload(&s->usbip_cfg, payload, cmd->transfer_buffer_length))
        {
            payload_size = cmd->transfer_buffer_length;
        }
        else
        {
            printf("XFER BUF LENGTH ERR\n");
        }
    }
    uint32_t word_count = sizeof(usb_req->setup) / sizeof(uint32_t);
    uint32_t* setup_words = (uint32_t*)&usb_req->setup;
    pthread_mutex_lock(&s->usbip_rx_mtx);
    s->usbip_rx_buffer[0] = header.raw;
    if (has_setup)
    {
        for (int i = 0; i < word_count; i++)
        {
            s->usbip_rx_buffer[word_count - i] = ntohl(setup_words[i]);
        }
        // memcpy(&s->usbip_rx_buffer[1], &usb_req.setup, sizeof(usb_req.setup));
        s->usbip_rx_level = 1U + word_count;
    }
    if (payload_size)
    {
        // Add the data as an immediate second packet.
        assert(payload_size < 255);
        header.DEV.BCNT = payload_size;
        header.DEV.INCOMPLETE = 0;
        header.DEV.PKTSTS = BUFFER_PKTSTS_OUT;
        s->usbip_rx_buffer[s->usbip_rx_level++] = header.raw;
        memcpy(&s->usbip_rx_buffer[s->usbip_rx_level], payload, payload_size);
        s->usbip_rx_level += ((payload_size + 3) / 4U);
    }
    pthread_mutex_unlock(&s->usbip_rx_mtx);
    pthread_mutex_lock(&s->usbip_tx_mtx);
    s->usbip_tx_wlen = cmd->transfer_buffer_length;
    // if (reply_required) 
    // {
    pthread_cond_wait(&s->usip_reply_ready, &s->usbip_tx_mtx);
    // }
    //printf("Reply ready\n");
    header.raw = s->usbip_tx_buffer[0];
    usbip_send_reply(&s->usbip_cfg, usb_req, (char*)&s->usbip_tx_buffer[1], header.DEV.BCNT, 0);
    // Wipe data from buffer.
    s->usbip_tx_level = 0;
    pthread_mutex_unlock(&s->usbip_tx_mtx);
    */
}

static void nuc970_usbd_class_init(ObjectClass* klass, void* data)
{
    DeviceClass* dc = DEVICE_CLASS(klass);
    dc->realize = nuc970_usbd_realize;
    dc->reset = nuc970_usbd_reset;
    set_bit(DEVICE_CATEGORY_USB, dc->categories);
    USBIPServerClass* sc = USBIP_SERVER_CLASS(dc);
    sc->usbip_handle_packet = nuc970_usbip_handle_packet;
}

static const TypeInfo nuc970_usbd_info = {
 .name = TYPE_NUC970_USBD,
 .parent = TYPE_SYS_BUS_DEVICE,
 .instance_size = sizeof(USBD_T),
 .class_init = nuc970_usbd_class_init,
 .instance_init = nuc970_usbd_init,
 .interfaces = (InterfaceInfo[]) {
    { TYPE_USBIP_SERVER },
    { }
 }
};

static void nuc970_usbd_register_types(void)
{
    type_register_static(&nuc970_usbd_info);
}

type_init(nuc970_usbd_register_types)
