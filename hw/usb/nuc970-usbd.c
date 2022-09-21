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
    uint8_t buf[512];
    uint32_t buf_level;
} USBD_EP_T;

typedef struct {
    SysBusDevice parent_obj;
    MemoryRegion iomem;
    qemu_irq irq;
    usbip_cfg_t usbip_cfg;
    QemuThread usbip_thread;
    QemuMutex usbip_mutex;
    QemuCond usbip_cond;
    QemuCond usip_reply_ready;

    QemuMutex usbip_rx_mtx;
    uint8_t usbip_rx_buffer[4096];
    uint32_t usbip_rx_level;
    uint32_t usbip_rx_index;

    QemuMutex usbip_tx_mtx;
    uint8_t usbip_tx_buffer[4096];
    uint32_t usbip_tx_level;

    uint8_t usbip_cep_buffer[512];  // control endpoint buffer
    uint32_t usbip_cep_level;

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
    /*
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
    */
    LEN_DEVICE,     /* bLength */
    DESC_DEVICE,    /* bDescriptorType */
    0x200,          /* bcdUSB */
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    0x40,           /* bMaxPacketSize0 */
    /* idVendor */
    0x0416,
    /* idProduct */
    0x0970,
    0x000,          /* bcdDevice */
    0x01,           /* iManufacture */
    0x02,           /* iProduct */
    0x00,           /* iSerialNumber - no serial */
    0x01            /* bNumConfigurations */
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
    },
    {
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

static void set_rx_level(USBD_T* usbd, uint32_t level)
{
    qemu_mutex_lock(&usbd->usbip_rx_mtx);
    usbd->usbip_rx_level = level;
    qemu_mutex_unlock(&usbd->usbip_rx_mtx);
}

static uint32_t get_rx_level(USBD_T* usbd)
{
    uint32_t ret;
    qemu_mutex_lock(&usbd->usbip_rx_mtx);
    ret = usbd->usbip_rx_level;
    qemu_mutex_unlock(&usbd->usbip_rx_mtx);
    return ret;
}

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

static void nuc970_usbd_update_interrupt(USBD_T*);
static void usbd_handle_attach(const USB_DEVICE_DESCRIPTOR* _dev_dsc, OP_REP_IMPORT* rep, USBD_T *s)
{
    //CONFIG_GEN* conf = (CONFIG_GEN*)_configuration;

    USB_DEVICE_DESCRIPTOR local_dsc;
    uint64_t setup = 0x8006000100004000;
    {
        s->SETUP1_0 = ntohs((setup & 0xFFFF000000000000) >> 48);
        s->SETUP3_2 = ntohs((setup & 0x0000FFFF00000000) >> 32);
        s->SETUP5_4 = ntohs((setup & 0x00000000FFFF0000) >> 16);
        s->SETUP7_6 = ntohs((setup & 0x000000000000FFFF) >> 0);
        s->CEPINTSTS |= USBD_CEPINTSTS_SETUPPKIF_Msk;
        qemu_mutex_lock_iothread();
        nuc970_usbd_update_interrupt(s);
        qemu_mutex_unlock_iothread();
    }

    g_usleep(G_USEC_PER_SEC * 0.5);
    
    s->CEPINTSTS |= USBD_CEPINTSTS_INTKIF_Msk;
    qemu_mutex_lock_iothread();
        nuc970_usbd_update_interrupt(s);
        qemu_mutex_unlock_iothread();

        qemu_mutex_lock(&s->usbip_tx_mtx);
        

        // if (reply_required) 
        // {
        qemu_cond_wait(&s->usip_reply_ready, &s->usbip_tx_mtx);
        // }
        printf(" reply ready...\n");
        //header.raw = s->usbip_tx_buffer[0];
        //usbip_send_reply(&s->usbip_cfg, usb_req, (char*)&s->usbip_tx_buffer[0],
        //    //header.DEV.BCNT,
        //    s->CEPTXCNT,
        //    0);
        // Wipe data from buffer.
        s->usbip_cep_level = 0;
        qemu_mutex_unlock(&s->usbip_tx_mtx);

        s->CEPINTSTS |= USBD_CEPINTSTS_TXPKIF_Msk;
        qemu_mutex_lock_iothread();
        nuc970_usbd_update_interrupt(s);
        qemu_mutex_unlock_iothread();
        usleep(1000);
        memcpy(&local_dsc, &s->usbip_cep_buffer[0], s->CEPTXCNT);
    
    
    for (int i = 0; i < s->CEPTXCNT; i++)
    {
        printf("  %02x", ((uint8_t*)&local_dsc)[i]);
    }
    printf("\n");

    s->CEPINTSTS |= USBD_CEPINTSTS_STSDONEIF_Msk;
    qemu_mutex_lock_iothread();
    nuc970_usbd_update_interrupt(s);
    qemu_mutex_unlock_iothread();
    usleep(1000);

    // get configuration
    USB_CONFIGURATION_DESCRIPTOR local_config;
    setup = 0x8006000200000900;
    {
        s->SETUP1_0 = ntohs((setup & 0xFFFF000000000000) >> 48);
        s->SETUP3_2 = ntohs((setup & 0x0000FFFF00000000) >> 32);
        s->SETUP5_4 = ntohs((setup & 0x00000000FFFF0000) >> 16);
        s->SETUP7_6 = ntohs((setup & 0x000000000000FFFF) >> 0);
        s->CEPINTSTS |= USBD_CEPINTSTS_SETUPPKIF_Msk;
        qemu_mutex_lock_iothread();
        nuc970_usbd_update_interrupt(s);
        qemu_mutex_unlock_iothread();
    }

    g_usleep(G_USEC_PER_SEC * 0.5);

    s->CEPINTSTS |= USBD_CEPINTSTS_INTKIF_Msk;
    qemu_mutex_lock_iothread();
    nuc970_usbd_update_interrupt(s);
    qemu_mutex_unlock_iothread();

    qemu_mutex_lock(&s->usbip_tx_mtx);


    // if (reply_required) 
    // {
    qemu_cond_wait(&s->usip_reply_ready, &s->usbip_tx_mtx);
    // }
    printf(" reply ready...\n");
    //header.raw = s->usbip_tx_buffer[0];
    //usbip_send_reply(&s->usbip_cfg, usb_req, (char*)&s->usbip_tx_buffer[0],
    //    //header.DEV.BCNT,
    //    s->CEPTXCNT,
    //    0);
    // Wipe data from buffer.
    s->usbip_cep_level = 0;
    qemu_mutex_unlock(&s->usbip_tx_mtx);

    s->CEPINTSTS |= USBD_CEPINTSTS_TXPKIF_Msk;
    qemu_mutex_lock_iothread();
    nuc970_usbd_update_interrupt(s);
    qemu_mutex_unlock_iothread();
    usleep(1000);
    memcpy(&local_config, &s->usbip_cep_buffer[0], s->CEPTXCNT);

    for (int i = 0; i < s->CEPTXCNT; i++)
    {
        printf("  %02x", ((uint8_t*)&local_config)[i]);
    }
    printf("\n");

    s->CEPINTSTS |= USBD_CEPINTSTS_STSDONEIF_Msk;
    qemu_mutex_lock_iothread();
    nuc970_usbd_update_interrupt(s);
    qemu_mutex_unlock_iothread();
    usleep(1000);

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
    rep->idVendor = local_dsc.idVendor;
    rep->idProduct = local_dsc.idProduct;
    rep->bcdDevice = local_dsc.bcdDevice;
    rep->bDeviceClass = local_dsc.bDeviceClass;
    rep->bDeviceSubClass = local_dsc.bDeviceSubClass;
    rep->bDeviceProtocol = local_dsc.bDeviceProtocol;
    rep->bNumConfigurations = local_dsc.bNumConfigurations;
    rep->bConfigurationValue = local_config.bConfigurationValue;// conf->dev_conf.bConfigurationValue;
    rep->bNumInterfaces = local_config.bNumInterfaces;// conf->dev_conf.bNumInterfaces;
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
        printf("\033[0;32m");
        for (int i = 0; i < size; i++) {
            printf(" %02x", ((unsigned char*)data)[i]);
            if ((i + 1) % 16 == 0)
                printf("\n");
        }
        printf("\n\033[0m");
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
unsigned char usbd_buffer[BSIZE + 1];
int bsize = 0;

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
    printf("  UC Index  %u-%u\n", control_req.wIndex1, control_req.wIndex0);
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
    printf(" BUSINT %08x %08x\n", usbd->BUSINTSTS, usbd->BUSINTEN);
    printf(" CEPINT %08x %08x\n", usbd->CEPINTSTS, usbd->CEPINTEN);
    printf(" EPAINT %08x %08x\n", usbd->EP[0].EPINTSTS, usbd->EP[0].EPINTEN);
    printf(" EPBINT %08x %08x\n", usbd->EP[1].EPINTSTS, usbd->EP[1].EPINTEN);

    if (usbd->BUSINTSTS & usbd->BUSINTEN ||
        usbd->CEPINTSTS & usbd->CEPINTEN ||
        usbd->EP[0].EPINTSTS & usbd->EP[0].EPINTEN ||
        usbd->EP[1].EPINTSTS & usbd->EP[1].EPINTEN
        ) 
    {
        printf(" usbd interrupt raised...\n");
        qemu_irq_raise(usbd->irq);
    }
    else {
        printf(" usbd interrupt lowered...\n");
        qemu_irq_lower(usbd->irq);
        qemu_cond_signal(&usbd->usbip_cond);
    }
}

static void interrupt_test(USBD_T* usbd)
{
    //    BUSINT 00000000 0000000e
    //    CEPINT 00000404 00000002
    //    EPAINT 00000042 00000040
    //    EPBINT 00000003 00001010

    sleep(5);

    printf("************ INTERRUPT TEST ****************\n");

    usbd->BUSINTSTS = 0; usbd->BUSINTEN = 0xe;
    usbd->CEPINTSTS = 0x404; usbd->CEPINTEN = 0x2;
    usbd->EP[0].EPINTSTS = 0x42; usbd->EP[0].EPINTEN = 0x40;
    usbd->EP[1].EPINTSTS = 0x3; usbd->EP[1].EPINTEN = 0x1010;

    qemu_mutex_lock_iothread();
    nuc970_usbd_update_interrupt(usbd);
    qemu_mutex_unlock_iothread();
    
    g_usleep(G_USEC_PER_SEC * 0.1);

    printf("**************** TEST 1 ********************\n");
    //EPAINT 0000004e 00000040
    usbd->EP[0].EPINTSTS = 0x4e; usbd->EP[0].EPINTEN = 0x40;

    qemu_mutex_lock_iothread();
    nuc970_usbd_update_interrupt(usbd);
    qemu_mutex_unlock_iothread();

    printf("*************** TEST END *******************\n");
}

static void* nuc970_usbip_thread_run(void* opaque)
{
    USBD_T* usbd = (USBD_T*)opaque;

    interrupt_test(usbd);
    return NULL;

    usbip_cfg_t* cfg = &usbd->usbip_cfg;
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
                    
                    usbd_handle_attach(/*&_dev_dsc*/NULL, &rep, usbd);
                    int ret = send(sockfd, (char*)&rep, sizeof(OP_REP_IMPORT), 0);
                    printf("handle attach send: %d\n", ret);
                    if ( ret != sizeof(OP_REP_IMPORT))
                    {
                        printf("send error : %s \n", strerror(errno));
                        break;
                    };
                    printf("attached\n");
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
                    printf("receive error : %s, nb: %d \n", strerror(errno), nb);
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
                printf("usbip flags 0x%08x\n", cmd.transfer_flags);
                printf("usbip number of packets %u\n", cmd.number_of_packets);
                printf("usbip interval %u\n", cmd.interval);
                //  printf("usbip setup %"PRI"\n",cmd.setup);
                printf("usbip buffer lenght  %u\n", cmd.transfer_buffer_length);
                printf("usbip start frame %u\n", cmd.start_frame);
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

uint32_t read_gintsts(USBD_T* s)
{
    uint32_t r = 0;
    int i;
    if (s->BUSINTSTS & s->BUSINTEN)
        r |= (1 << 0);
    if (s->CEPINTSTS & s->CEPINTEN)
        r |= (1 << 1);
    for (i = 0; i < 12; i++) {
        if (s->EP[i].EPINTSTS & s->EP[i].EPINTEN) {
            r |= (1 << (2 + i));
        }
    }
    return r;
}

static uint64_t nuc970_usbd_read(void* opaque, hwaddr offset, unsigned size)
{
    USBD_T* s = (USBD_T*)opaque;
    uint32_t r = 0;

    switch (offset) {
    case 0x00:
        r = read_gintsts(s); break;
    case 0x08:
        r = s->GINTEN; break;
    case 0x10:
        r = s->BUSINTSTS; break;
    case 0x14:
        r = s->BUSINTEN; break;
    case 0x18:
        r = s->OPER; break;
    case 0x1c:
        r = s->FRAMECNT; break;
    case 0x20:
        r = s->FADDR; break;
    case 0x24:
        r = s->TEST; break;
    case 0x28 ... 0x2b:
        r = s->cep.CEPDAT;
        if (offset % 4 != 0) {
            r = (r >> ((offset % 4) * 8));
        }
        if (size == 2)
            r = r & 0xffff;
        else if (size == 1)
            r = r & 0xff;
        break;
    case 0x2c:
        r = s->CEPCTL; break;
    case 0x30:
        r = s->CEPINTEN; break;
    case 0x34:
        r = s->CEPINTSTS; break;
    case 0x38:
        r = s->CEPTXCNT; break;
    case 0x3c:
        r = s->CEPRXCNT; break;
    case 0x40:
        r = s->CEPDATCNT; break;
    case 0x44:
        r = s->SETUP1_0; break;
    case 0x48:
        r = s->SETUP3_2; break;
    case 0x4c:
        r = s->SETUP5_4; break;
    case 0x50:
        r = s->SETUP7_6; break;
    case 0x54:
        r = s->CEPBUFSTART; break;
    case 0x58:
        r = s->CEPBUFEND; break;
    case 0x5c:
        r = s->DMACTL; break;
    case 0x60:
        r = s->DMACNT; break;
    case 0x64 ... 0x240:
        {
            int epidx = (offset - 0x64) / 40;
            int regidx = ((offset - 0x64) % 40) / 4;
            //printf(" epidx: %d\n", epidx);
            switch (regidx) {
            case 0: // DAT                
                if ((s->EP[epidx].EPCFG >> 3) & 0x01)   // IN (Host IN from Device)
                {
                    // do nothing
                }
                else // OUT (Host OUT to Device)
                {
                    qemu_mutex_lock(&s->usbip_rx_mtx);
                    r = s->usbip_rx_buffer[s->usbip_rx_index++];
                    qemu_mutex_unlock(&s->usbip_rx_mtx);
                }
                break;
            case 1: // INTSTS
                r = s->EP[epidx].EPINTSTS;
                break;
            case 2: // INTEN
                r= s->EP[epidx].EPINTEN;
                break;
            case 3: // DATCNT
                r = s->EP[epidx].EPDATCNT;
                break;
            case 4: // RSPCTL
                r = s->EP[epidx].EPRSPCTL;
                break;
            case 5: // MPS
                r = s->EP[epidx].EPMPS;
                break;
            case 6: // TXCNT
                r = s->EP[epidx].EPTXCNT;
                break;
            case 7: // CFG
                r = s->EP[epidx].EPCFG;
                break;
            case 8: // BUFSTART
                r = s->EP[epidx].EPBUFSTART;
                break;
            case 9: // BUFEND
                r = s->EP[epidx].EPBUFEND;
                break;
            }
        }
        break;
    case 0x700:
        r = s->DMAADDR;
        break;
    default:
        r = 0xffffffff;
        break;
    }
    //fprintf(stderr, "usbd_r (offset=%03lx/%d, value=%08x)\n", offset, size, r);
    return r;
}

static void nuc970_usbd_write(void* opaque, hwaddr offset, uint64_t value, unsigned size)
{
    USBD_T* s = (USBD_T*)opaque;
    switch (offset) {
    case 0x08:
        s->GINTEN = value;
        break;
    case 0x10:
        s->BUSINTSTS = value;
        if (value & 0x17f)  // clear [8,6:0]
            s->BUSINTSTS &= ~(value & 0x17f);
        nuc970_usbd_update_interrupt(s);
        break;
    case 0x14:
        s->BUSINTEN = value; 
        nuc970_usbd_update_interrupt(s);
        break;
    case 0x18:
        s->OPER = value & ~(1 << 0); break; // RESUMEEN bit is self-clearing
    case 0x1c:
        s->FRAMECNT = value; break;
    case 0x20:
        s->FADDR = value & 0x7f; break;
    case 0x24:
        s->TEST = value; break;
    case 0x28 ... 0x2b:
        if (size == 4) {
            s->cep.CEPDAT = value;
            s->usbip_cep_buffer[s->usbip_cep_level++] = ((value >> 0) & 0xff);
            s->usbip_cep_buffer[s->usbip_cep_level++] = ((value >> 8) & 0xff);
            s->usbip_cep_buffer[s->usbip_cep_level++] = ((value >> 16) & 0xff);
            s->usbip_cep_buffer[s->usbip_cep_level++] = ((value >> 24) & 0xff);
        }
        else if (size == 1) {
            s->cep.CEPDAT &= ~(0xff << ((offset % 4) * 8));
            s->cep.CEPDAT |= (value & 0xff) << ((offset % 4) * 8);
            s->usbip_cep_buffer[s->usbip_cep_level++] = value & 0xff;
        }
        break;        
    case 0x2c:
        s->CEPCTL = value;
        break;
    case 0x30:
        s->CEPINTEN = value;
        nuc970_usbd_update_interrupt(s);
        break;
    case 0x34:
        s->CEPINTSTS = value;
        if (value & 0x1fff) // clear [12:0]
            s->CEPINTSTS &= ~(value & 0x1fff);
        nuc970_usbd_update_interrupt(s);
        break;
    case 0x38:  // In-transfer Data Count
        {
            s->CEPTXCNT = value;
            qemu_cond_signal(&s->usip_reply_ready);
        }
        break;
    // 0x3c/40/44/48/4c/50 readonly
    case 0x54:
        s->CEPBUFSTART = value; break;
    case 0x58:
        s->CEPBUFEND = value; break;
    case 0x5c:        
        s->DMACTL = value & ~(1 << 5);  // DMAEN write only???        
        if (value & (1 << 5)) { // DMAEN
            if (value & (1 << 4)) { // DMA read (write to USB buffer)
                printf("DMARD %d %08x...\n", s->DMACNT, s->DMAADDR);
                dma_memory_read(&address_space_memory, s->DMAADDR, s->usbip_tx_buffer,
                    s->DMACNT, MEMTXATTRS_UNSPECIFIED);
                //for (int k = 0; k < s->DMACNT; k++)
                //    printf(" \033[0;32m%02x\033[0m", s->usbip_tx_buffer[k]);
                //printf("\n");
                //if (s->DMACNT) {
                //    qemu_cond_signal(&s->usip_reply_ready);
                //}
                s->EP[0].EPINTSTS &= ~(USBD_EPINTSTS_BUFEMPTYIF_Msk); // not empty
            }
            else {  // DMA write (read from USB buffer)
                
                printf("DMAWR %d %08x, EP: %d...\n", s->DMACNT, s->DMAADDR, s->DMACTL & 0xf);
                
                while (!get_rx_level(s)) {  // wait for rx buffer ready
                    g_usleep(500);
                }
                
                dma_memory_write(&address_space_memory, s->DMAADDR, s->usbip_rx_buffer,
                    s->DMACNT, MEMTXATTRS_UNSPECIFIED);
                for (int k = 0; k < s->DMACNT; k++)
                    printf(" \033[0;33m%02x\033[0m", s->usbip_rx_buffer[k]);
                printf("\n");
                set_rx_level(s, 0); // empty rx buffer
                
            }
            s->BUSINTSTS |= (1 << USBD_BUSINTEN_DMADONEIEN_Pos);    // DMADONEIF
            nuc970_usbd_update_interrupt(s);
        }
        if (value & (1 << 7)) {
            s->DMACNT = 0;
            s->DMACTL = 0x80;
            s->DMACTL = 0x00;
        }        
        break;
    case 0x60:
        s->DMACNT = value; break;
    case 0x64 ... 0x240:
        {
            int epidx = (offset - 0x64) / 40;
            int regidx = ((offset - 0x64) % 40) / 4;
            //printf(" epidx: %d\n", epidx);
            switch (regidx) {
            case 0: // DAT only word or byte access are supported
                if (size == 4) 
                    s->EP[epidx].ep.EPDAT = value;
                else
                    s->EP[epidx].ep.EPDAT_BYTE = value & 0xff;

                if ((s->EP[epidx].EPCFG >> 3) & 0x01)   // IN (Host IN from Device)
                {
                    //qemu_mutex_lock(&s->usbip_tx_mtx);
                    s->usbip_tx_buffer[s->usbip_tx_level++] = s->EP[epidx].ep.EPDAT_BYTE;
                    s->EP[0].EPINTSTS &= ~(USBD_EPINTSTS_BUFEMPTYIF_Msk); // not empty
                    //qemu_mutex_unlock(&s->usbip_tx_mtx);
                    printf(" *** writing tx %02x, level %d\n", s->EP[epidx].ep.EPDAT_BYTE, s->usbip_tx_level);
                }
                else // OUT (Host OUT to Device)
                {
                    // do nothing
                }
                break;
            case 1: // INTSTS
                s->EP[epidx].EPINTSTS = value & ~(3);   // [1:0] readonly
                if (value & 0x1ffc) {
                    s->EP[epidx].EPINTSTS &= ~(value & 0x1ffc);
                }
                nuc970_usbd_update_interrupt(s);
                break;
            case 2: // INTEN
                s->EP[epidx].EPINTEN = value;
                nuc970_usbd_update_interrupt(s);
                break;
            case 3: // DATCNT
                s->EP[epidx].EPDATCNT = value;
                break;
            case 4: // RSPCTL
                s->EP[epidx].EPRSPCTL = value;
                break;
            case 5: // MPS
                s->EP[epidx].EPMPS = value;
                break;
            case 6: // TXCNT
                s->EP[epidx].EPTXCNT = value;
                break;
            case 7: // CFG
                s->EP[epidx].EPCFG = value;
                break;
            case 8: // BUFSTART
                s->EP[epidx].EPBUFSTART = value;
                break;
            case 9: // BUFEND
                s->EP[epidx].EPBUFEND = value;
                break;
            }
        }        
        break;
    case 0x700:
        s->DMAADDR = value;
        break;
         
    default:
        fprintf(stderr, "usbd_w (offset=%lx)\n", offset);
        break;
    }
    if (offset != 0x74)
        fprintf(stderr, "usbd_w (offset=%03lx/%d, value=%08lx)\n", offset, size, value);
}

static const MemoryRegionOps nuc970_usbd_ops = {
    .read = nuc970_usbd_read,
    .write = nuc970_usbd_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid = {
        .min_access_size = 1,   // allow byte access
        .max_access_size = 4,
    },
};

static void nuc970_usbd_reset(DeviceState* dev)
{
    USBD_T* s = NUC970_USBD(dev);
    unsigned int i;
    s->GINTEN = 1;
    s->BUSINTSTS = 0;
    s->BUSINTEN = 0x00000040;
    s->OPER = 0x00000002;   // enable High-speed
    s->FRAMECNT = 0;
    s->FADDR = 0;
    s->TEST = 0;
    s->cep.CEPDAT = 0;
    s->CEPCTL = 0;
    s->CEPINTEN = 0;
    s->CEPINTSTS = 0x00001800;
    s->CEPTXCNT = s->CEPRXCNT = s->CEPDATCNT = 0;
    s->DMACNT = 0;
    s->DMACTL = 0;
    s->EP[0].EPINTSTS = 0x00000003;
    s->EP[0].EPINTEN = USBD_EPINTEN_BUFEMPTYIEN_Msk;
    s->EP[0].EPCFG = 0x00000012;
    s->EP[0].EPDATCNT = 0;
    s->EP[1].EPINTSTS = 0x00000003;
    s->EP[1].EPINTEN = USBD_EPINTEN_BUFEMPTYIEN_Msk;
    s->EP[1].EPCFG = 0x00000022;
    s->EP[1].EPDATCNT = 0;

    s->usbip_cep_level = 0;
    s->usbip_rx_index = 0;
    s->usbip_rx_level = 0;
    s->usbip_tx_level = 0;
    qemu_irq_lower(s->irq);

    s->BUSINTSTS |= USBD_BUSINTSTS_RSTIF_Msk;
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

    qemu_mutex_init(&s->usbip_rx_mtx);
    qemu_mutex_init(&s->usbip_tx_mtx);
    qemu_cond_init(&s->usip_reply_ready);

    qemu_thread_create(&s->usbip_thread, "usbip", 
        nuc970_usbip_thread_run, s, QEMU_THREAD_JOINABLE);
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

enum {
    BUFFER_PKTSTS_GNAK = 1U,
    BUFFER_PKTSTS_OUT = 2U,
    BUFFER_PKTSTS_OUTCPLT = 3U,
    BUFFER_PKTSTS_SETUPCPLT = 4U,
    BUFFER_PKTSTS_SETUP = 6U
};

static void nuc970_usbip_handle_control(USBIPIF* self, USBIP_CMD_SUBMIT* cmd, USBIP_RET_SUBMIT* usb_req)
{
    printf("Handle control packet: Setup: %llx\n", cmd->setup);

    USBD_T* s = NUC970_USBD(self);
    unsigned char payload[512];
    uint8_t payload_size = 0;
    bool has_setup = true;
    
    if (has_setup)
    {
        s->SETUP1_0 = ntohs((usb_req->setup & 0xFFFF000000000000) >> 48);
        s->SETUP3_2 = ntohs((usb_req->setup & 0x0000FFFF00000000) >> 32);
        s->SETUP5_4 = ntohs((usb_req->setup & 0x00000000FFFF0000) >> 16);
        s->SETUP7_6 = ntohs((usb_req->setup & 0x000000000000FFFF) >> 0);
        s->CEPINTSTS |= USBD_CEPINTSTS_SETUPPKIF_Msk;
        qemu_mutex_lock_iothread();
        nuc970_usbd_update_interrupt(s);
        qemu_mutex_unlock_iothread();       
    }

    /*
    printf(" usbip cond wait...\n");
    qemu_mutex_lock(&s->usbip_mutex);
    qemu_cond_wait(&s->usbip_cond, &s->usbip_mutex);
    qemu_mutex_unlock(&s->usbip_mutex);
    printf(" usbip cond ready...\n");
    */
    g_usleep(/*G_USEC_PER_SEC * 0.1*/50000);

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
    printf(" >>>> setup %d >>>>>>> \n", has_setup);
    
    qemu_mutex_lock(&s->usbip_rx_mtx);
    printf(" ==== payload %d ===== \n", payload_size);

    if (payload_size)
    {
        for (int j = 0; j < payload_size; j++)
            printf(" %02x", payload[j]);
        printf("\n");
    }
    qemu_mutex_unlock(&s->usbip_rx_mtx);

    if (usb_req->direction) {
        s->CEPINTSTS |= USBD_CEPINTSTS_INTKIF_Msk;
        qemu_mutex_lock_iothread();
        nuc970_usbd_update_interrupt(s);
        qemu_mutex_unlock_iothread();

        qemu_mutex_lock(&s->usbip_tx_mtx);
        //s->usbip_tx_wlen = cmd->transfer_buffer_length;
        printf(" wait for reply ready, transfer_buffer_length: %d\n", cmd->transfer_buffer_length);

        // if (reply_required) 
        // {
        qemu_cond_wait(&s->usip_reply_ready, &s->usbip_tx_mtx);
        // }
        if (cmd->transfer_buffer_length > 0x40 && 
            cmd->transfer_buffer_length != 0xff) {
            printf(" reply ready 1...\n");
            
            g_usleep(3000);

            while (s->usbip_cep_level < cmd->transfer_buffer_length) {
                s->CEPINTSTS |= USBD_CEPINTSTS_TXPKIF_Msk;
                qemu_mutex_lock_iothread();
                nuc970_usbd_update_interrupt(s);
                qemu_mutex_unlock_iothread();

                g_usleep(3000);

                s->CEPINTSTS |= USBD_CEPINTSTS_INTKIF_Msk;
                qemu_mutex_lock_iothread();
                nuc970_usbd_update_interrupt(s);
                qemu_mutex_unlock_iothread();

                qemu_cond_wait(&s->usip_reply_ready, &s->usbip_tx_mtx);
            }
        }
        printf(" reply ready 2...\n");
        //header.raw = s->usbip_tx_buffer[0];
        usbip_send_reply(&s->usbip_cfg, usb_req, (char*)&s->usbip_cep_buffer[0],
            //header.DEV.BCNT,
            s->usbip_cep_level,
            0);
        // Wipe data from buffer.
        s->usbip_cep_level = 0;
        qemu_mutex_unlock(&s->usbip_tx_mtx);

        s->CEPINTSTS |= USBD_CEPINTSTS_TXPKIF_Msk;
        qemu_mutex_lock_iothread();
        nuc970_usbd_update_interrupt(s);
        qemu_mutex_unlock_iothread();
        usleep(10000);
    }
    else {  // usb_req->direction == 0
        s->CEPINTSTS |= USBD_CEPINTSTS_OUTTKIF_Msk;
        qemu_mutex_lock_iothread();
        nuc970_usbd_update_interrupt(s);
        qemu_mutex_unlock_iothread();

        g_usleep(1000);
        
        qemu_mutex_lock(&s->usbip_tx_mtx);
        usbip_send_reply(&s->usbip_cfg, usb_req, NULL, cmd->transfer_buffer_length, 0);
        qemu_mutex_unlock(&s->usbip_tx_mtx);

        // Wipe data from buffer.
        s->EP[0].EPINTSTS |= USBD_EPINTSTS_BUFEMPTYIF_Msk;
        s->usbip_cep_level = 0;
    }

    s->CEPINTSTS |= USBD_CEPINTSTS_STSDONEIF_Msk;
    qemu_mutex_lock_iothread();
    nuc970_usbd_update_interrupt(s);
    qemu_mutex_unlock_iothread();
    usleep(1000);
}

static void nuc970_usbip_handle_data(USBIPIF* self, USBIP_CMD_SUBMIT* cmd, USBIP_RET_SUBMIT* usb_req, int bl)
{
    /* EPA ==> Bulk IN endpoint, address 1 */
    /* EPB ==> Bulk OUT endpoint, address 2 */

    USBD_T* s = NUC970_USBD(self);
    int ep = usb_req->ep;
    bool has_setup = false;
    if (has_setup)
    {
        s->SETUP1_0 = ntohs((usb_req->setup & 0xFFFF000000000000) >> 48);
        s->SETUP3_2 = ntohs((usb_req->setup & 0x0000FFFF00000000) >> 32);
        s->SETUP5_4 = ntohs((usb_req->setup & 0x00000000FFFF0000) >> 16);
        s->SETUP7_6 = ntohs((usb_req->setup & 0x000000000000FFFF) >> 0);
        s->CEPINTSTS |= USBD_CEPINTSTS_SETUPPKIF_Msk;
        qemu_mutex_lock_iothread();
        nuc970_usbd_update_interrupt(s);
        qemu_mutex_unlock_iothread();
    
        g_usleep(G_USEC_PER_SEC * 0.5);

        s->CEPINTSTS |= cmd->direction == 1 ? USBD_CEPINTSTS_OUTTKIF_Msk : USBD_CEPINTSTS_INTKIF_Msk;
        qemu_mutex_lock_iothread();
        nuc970_usbd_update_interrupt(s);
        qemu_mutex_unlock_iothread();

        g_usleep(G_USEC_PER_SEC * 0.5);
    }

    if (cmd->direction == 0) // HOST OUT
    {
        /*
        s->EP[usb_req->ep - 1].EPINTSTS |= USBD_EPINTSTS_OUTTKIF_Msk;
        qemu_mutex_lock_iothread();
        nuc970_usbd_update_interrupt(s);
        qemu_mutex_unlock_iothread();
        g_usleep(G_USEC_PER_SEC * 0.1);
        */
        int chunk_size = min(cmd->transfer_buffer_length, 4096);
        int chunk_cnt = cmd->transfer_buffer_length / chunk_size;
        int i; 
        for (i = 0; i < chunk_cnt; i++) {


            if (usbip_read_payload(&s->usbip_cfg, s->usbip_rx_buffer, chunk_size))
            {
                for (int j = 0; j < chunk_size; j++)
                    printf(" %02x", (unsigned char)s->usbip_rx_buffer[j]);
                printf("\n");

                set_rx_level(s, chunk_size);
                s->usbip_rx_index = 0;  // reset index
                

                s->EP[ep - 1].EPINTSTS |= USBD_EPINTSTS_RXPKIF_Msk;
                s->EP[ep - 1].EPDATCNT = chunk_size;

                qemu_mutex_lock_iothread();
                nuc970_usbd_update_interrupt(s);
                qemu_mutex_unlock_iothread();
                g_usleep(G_USEC_PER_SEC * 0.05);

                //s->EP[usb_req->ep - 1].EPINTSTS |= USBD_EPINTSTS_TXPKIF_Msk;
                //qemu_mutex_lock_iothread();
                //nuc970_usbd_update_interrupt(s);
                //qemu_mutex_unlock_iothread();
                //g_usleep(G_USEC_PER_SEC * 0.1);
            }
            else
            {
                printf("XFER BUF LENGTH ERR\n");
            }
        }

        qemu_mutex_lock(&s->usbip_tx_mtx);

        usb_req->command = 0x3;
        usb_req->status = 0;
        usb_req->actual_length = cmd->transfer_buffer_length;
        usb_req->start_frame = 0x0;
        usb_req->number_of_packets = 0x0;

        usb_req->setup = 0x0;
        usb_req->devid = 0x0;
        usb_req->direction = cmd->direction;
        usb_req->ep = cmd->ep;
        usb_req->error_count = 0x0;
        pack((int*)usb_req, sizeof(USBIP_RET_SUBMIT));

        if (send(s->usbip_cfg._private_fd, (char*)usb_req, sizeof(USBIP_RET_SUBMIT), 0) !=
            sizeof(USBIP_RET_SUBMIT))
        {
            printf("send error : %s \n", strerror(errno));
            exit(-1);
        };

        qemu_mutex_unlock(&s->usbip_tx_mtx);
    }
    else {  // HOST IN       
        

        if (cmd->ep == 3)   // Interrupt
        {
            // enable EPA INTK
            s->EP[0].EPINTEN |= USBD_EPINTEN_INTKIEN_Msk;
            usbip_send_reply(&s->usbip_cfg, usb_req, "", 0, 0);
        }
        else {

            s->EP[usb_req->ep - 1].EPINTSTS |= USBD_EPINTSTS_INTKIF_Msk;
            qemu_mutex_lock_iothread();
            nuc970_usbd_update_interrupt(s);
            qemu_mutex_unlock_iothread();

            g_usleep(G_USEC_PER_SEC * 0.5);

            qemu_mutex_lock(&s->usbip_tx_mtx);

            // if (reply_required) 
            // {
            //qemu_cond_wait(&s->usip_reply_ready, &s->usbip_tx_mtx);
            // }
            printf(" reply %d, buffer size: %d...\n", cmd->transfer_buffer_length,
                    s->usbip_tx_level
                );

            usbip_send_reply(&s->usbip_cfg, usb_req, s->usbip_tx_buffer, 
                //cmd->transfer_buffer_length,
                s->usbip_tx_level,
                0);
            s->usbip_tx_level = 0;
            qemu_mutex_unlock(&s->usbip_tx_mtx);

            s->EP[0].EPINTSTS |= USBD_EPINTSTS_BUFEMPTYIF_Msk;

            s->EP[ep - 1].EPINTSTS |= USBD_EPINTSTS_SHORTTXIF_Msk;
            s->EP[ep - 1].EPINTSTS |= USBD_EPINTSTS_TXPKIF_Msk;
            qemu_mutex_lock_iothread();
            nuc970_usbd_update_interrupt(s);
            qemu_mutex_unlock_iothread();

            g_usleep(G_USEC_PER_SEC * 0.1);
        }
    }

    //s->CEPINTSTS |= USBD_CEPINTSTS_STSDONEIF_Msk;
    //qemu_mutex_lock_iothread();
    //nuc970_usbd_update_interrupt(s);
    //qemu_mutex_unlock_iothread();
    //usleep(1000);
}

static void nuc970_usbip_handle_packet(USBIPIF* self, USBIP_CMD_SUBMIT* cmd, USBIP_RET_SUBMIT* usb_req)
{
    printf("Handle packet called: Setup: %llx, ep: %d, dir: %d\n", cmd->setup, cmd->ep, cmd->direction);

    if (cmd->ep == 0) {
        nuc970_usbip_handle_control(self, cmd, usb_req);
    }
    else {
        nuc970_usbip_handle_data(self, cmd, usb_req, cmd->transfer_buffer_length);
    }
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
