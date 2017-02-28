#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/spi.h>


#define PAYLOADSIZE 32

typedef enum {
    _NRF24L01P_MODE_UNKNOWN,
    _NRF24L01P_MODE_POWER_DOWN,
    _NRF24L01P_MODE_STANDBY,
    _NRF24L01P_MODE_RX,
    _NRF24L01P_MODE_TX,
} nRF24L01P_Mode_Type;

/*
 * The following FIFOs are present in nRF24L01+:
 *   TX three level, 32 byte FIFO
 *   RX three level, 32 byte FIFO
 */
#define _NRF24L01P_TX_FIFO_COUNT   3
#define _NRF24L01P_RX_FIFO_COUNT   3

#define _NRF24L01P_TX_FIFO_SIZE   32
#define _NRF24L01P_RX_FIFO_SIZE   32

#define _NRF24L01P_SPI_MAX_DATA_RATE     10000000

#define _NRF24L01P_SPI_CMD_RD_REG            0x00
#define _NRF24L01P_SPI_CMD_WR_REG            0x20
#define _NRF24L01P_SPI_CMD_RD_RX_PAYLOAD     0x61   
#define _NRF24L01P_SPI_CMD_WR_TX_PAYLOAD     0xa0
#define _NRF24L01P_SPI_CMD_FLUSH_TX          0xe1
#define _NRF24L01P_SPI_CMD_FLUSH_RX          0xe2
#define _NRF24L01P_SPI_CMD_REUSE_TX_PL       0xe3
#define _NRF24L01P_SPI_CMD_R_RX_PL_WID       0x60
#define _NRF24L01P_SPI_CMD_W_ACK_PAYLOAD     0xa8
#define _NRF24L01P_SPI_CMD_W_TX_PYLD_NO_ACK  0xb0
#define _NRF24L01P_SPI_CMD_NOP               0xff


#define _NRF24L01P_REG_CONFIG                0x00
#define _NRF24L01P_REG_EN_AA                 0x01
#define _NRF24L01P_REG_EN_RXADDR             0x02
#define _NRF24L01P_REG_SETUP_AW              0x03
#define _NRF24L01P_REG_SETUP_RETR            0x04
#define _NRF24L01P_REG_RF_CH                 0x05
#define _NRF24L01P_REG_RF_SETUP              0x06
#define _NRF24L01P_REG_STATUS                0x07
#define _NRF24L01P_REG_OBSERVE_TX            0x08
#define _NRF24L01P_REG_RPD                   0x09
#define _NRF24L01P_REG_RX_ADDR_P0            0x0a
#define _NRF24L01P_REG_RX_ADDR_P1            0x0b
#define _NRF24L01P_REG_RX_ADDR_P2            0x0c
#define _NRF24L01P_REG_RX_ADDR_P3            0x0d
#define _NRF24L01P_REG_RX_ADDR_P4            0x0e
#define _NRF24L01P_REG_RX_ADDR_P5            0x0f
#define _NRF24L01P_REG_TX_ADDR               0x10
#define _NRF24L01P_REG_RX_PW_P0              0x11
#define _NRF24L01P_REG_RX_PW_P1              0x12
#define _NRF24L01P_REG_RX_PW_P2              0x13
#define _NRF24L01P_REG_RX_PW_P3              0x14
#define _NRF24L01P_REG_RX_PW_P4              0x15
#define _NRF24L01P_REG_RX_PW_P5              0x16
#define _NRF24L01P_REG_FIFO_STATUS           0x17
#define _NRF24L01P_REG_DYNPD                 0x1c
#define _NRF24L01P_REG_FEATURE               0x1d

#define _NRF24L01P_REG_ADDRESS_MASK          0x1f

// CONFIG register:
#define _NRF24L01P_CONFIG_PRIM_RX        (1<<0)
#define _NRF24L01P_CONFIG_PWR_UP         (1<<1)
#define _NRF24L01P_CONFIG_CRC0           (1<<2)
#define _NRF24L01P_CONFIG_EN_CRC         (1<<3)
#define _NRF24L01P_CONFIG_MASK_MAX_RT    (1<<4)
#define _NRF24L01P_CONFIG_MASK_TX_DS     (1<<5)
#define _NRF24L01P_CONFIG_MASK_RX_DR     (1<<6)

#define _NRF24L01P_CONFIG_CRC_MASK       (_NRF24L01P_CONFIG_EN_CRC|_NRF24L01P_CONFIG_CRC0)
#define _NRF24L01P_CONFIG_CRC_NONE       (0)
#define _NRF24L01P_CONFIG_CRC_8BIT       (_NRF24L01P_CONFIG_EN_CRC)
#define _NRF24L01P_CONFIG_CRC_16BIT      (_NRF24L01P_CONFIG_EN_CRC|_NRF24L01P_CONFIG_CRC0)

// EN_AA register:
#define _NRF24L01P_EN_AA_NONE            0

// EN_RXADDR register:
#define _NRF24L01P_EN_RXADDR_NONE        0

// SETUP_AW register:
#define _NRF24L01P_SETUP_AW_AW_MASK      (0x3<<0)
#define _NRF24L01P_SETUP_AW_AW_3BYTE     (0x1<<0)
#define _NRF24L01P_SETUP_AW_AW_4BYTE     (0x2<<0)
#define _NRF24L01P_SETUP_AW_AW_5BYTE     (0x3<<0)

// SETUP_RETR register:
#define _NRF24L01P_SETUP_RETR_NONE       0

// RF_SETUP register:
#define _NRF24L01P_RF_SETUP_RF_PWR_MASK          (0x3<<1)
#define _NRF24L01P_RF_SETUP_RF_PWR_0DBM          (0x3<<1)
#define _NRF24L01P_RF_SETUP_RF_PWR_MINUS_6DBM    (0x2<<1)
#define _NRF24L01P_RF_SETUP_RF_PWR_MINUS_12DBM   (0x1<<1)
#define _NRF24L01P_RF_SETUP_RF_PWR_MINUS_18DBM   (0x0<<1)

#define _NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT       (1 << 3)
#define _NRF24L01P_RF_SETUP_RF_DR_LOW_BIT        (1 << 5)
#define _NRF24L01P_RF_SETUP_RF_DR_MASK           (_NRF24L01P_RF_SETUP_RF_DR_LOW_BIT|_NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT)
#define _NRF24L01P_RF_SETUP_RF_DR_250KBPS        (_NRF24L01P_RF_SETUP_RF_DR_LOW_BIT)
#define _NRF24L01P_RF_SETUP_RF_DR_1MBPS          (0)
#define _NRF24L01P_RF_SETUP_RF_DR_2MBPS          (_NRF24L01P_RF_SETUP_RF_DR_HIGH_BIT)

// STATUS register:
#define _NRF24L01P_STATUS_TX_FULL        (1<<0)
#define _NRF24L01P_STATUS_RX_P_NO        (0x7<<1)
#define _NRF24L01P_STATUS_MAX_RT         (1<<4)
#define _NRF24L01P_STATUS_TX_DS          (1<<5)
#define _NRF24L01P_STATUS_RX_DR          (1<<6)

// RX_PW_P0..RX_PW_P5 registers:
#define _NRF24L01P_RX_PW_Px_MASK         0x3F

#define _NRF24L01P_TIMING_Tundef2pd_us     100000   // 100mS
#define _NRF24L01P_TIMING_Tstby2a_us          130   // 130uS
#define _NRF24L01P_TIMING_Thce_us              10   //  10uS
#define _NRF24L01P_TIMING_Tpd2stby_us        4500   // 4.5mS worst case
#define _NRF24L01P_TIMING_Tpece2csn_us          4   //   4uS

#define NRF24L01P_TX_PWR_ZERO_DB         0
#define NRF24L01P_TX_PWR_MINUS_6_DB     -6
#define NRF24L01P_TX_PWR_MINUS_12_DB   -12
#define NRF24L01P_TX_PWR_MINUS_18_DB   -18

#define NRF24L01P_DATARATE_250_KBPS    250
#define NRF24L01P_DATARATE_1_MBPS     1000
#define NRF24L01P_DATARATE_2_MBPS     2000

#define NRF24L01P_CRC_NONE               0
#define NRF24L01P_CRC_8_BIT              8
#define NRF24L01P_CRC_16_BIT            16

#define NRF24L01P_MIN_RF_FREQUENCY    2400
#define NRF24L01P_MAX_RF_FREQUENCY    2525

#define NRF24L01P_PIPE_P0                0
#define NRF24L01P_PIPE_P1                1
#define NRF24L01P_PIPE_P2                2
#define NRF24L01P_PIPE_P3                3
#define NRF24L01P_PIPE_P4                4
#define NRF24L01P_PIPE_P5                5

/**
* Default setup for the nRF24L01+, based on the Sparkfun "Nordic Serial Interface Board"
*  for evaluation (http://www.sparkfun.com/products/9019)
*/
#define DEFAULT_NRF24L01P_ADDRESS       ((unsigned long long) 0xE7E7E7E7E7 )
#define DEFAULT_NRF24L01P_ADDRESS_WIDTH  5
#define DEFAULT_NRF24L01P_CRC            NRF24L01P_CRC_8_BIT
#define DEFAULT_NRF24L01P_RF_FREQUENCY  (NRF24L01P_MIN_RF_FREQUENCY + 2)
#define DEFAULT_NRF24L01P_DATARATE       NRF24L01P_DATARATE_1_MBPS
#define DEFAULT_NRF24L01P_TX_PWR         NRF24L01P_TX_PWR_ZERO_DB
#define DEFAULT_NRF24L01P_TRANSFER_SIZE  4

#define ENDPOINT_ADDRESS_IN         (0x81)
#define ENDPOINT_ADDRESS_OUT        (0x01)

#define LED_PORT GPIOC
#define LED_PIN GPIO13
#define VREF 3.3
#define MAX_PARROTS 4096.0
#define MAXPACKETSIZEIN 64
#define MAXPACKETSIZEOUT 64


static usbd_device *usbd_dev;
volatile uint16_t temperature_in_parrots = 0;
volatile uint16_t Vref_in_parrots = 0;
volatile uint16_t Vwakeup_in_parrots = 0;
float V25 = 1.41;
float Avg_Slope = 4.3e-3;
//float Vref = 3.0;

void my_delay(int del);

void my_delay(int del)
{
    int i;

    for (i = 0; i < del; i++)
        __asm__("nop");
}

void led_toggle(void)
{
	gpio_toggle(GPIOC, GPIO13);
}

void nrf_spi_csh(void)
{
     gpio_set(GPIOB, GPIO_SPI2_NSS);
}

void nrf_spi_csl(void)
{
     gpio_clear(GPIOB, GPIO_SPI2_NSS);
}

void nrf_ceh(void)
{
	gpio_set(GPIOB, GPIO0);
}

void nrf_cel(void)
{
	gpio_clear(GPIOB, GPIO0);
}

unsigned char nrf_spi_xfer_byte(unsigned char data)
{
     return spi_xfer(SPI2, data);
}

uint8_t nrf_read_reg(unsigned char reg)
{
    nrf_spi_csl();
    nrf_spi_xfer_byte(_NRF24L01P_SPI_CMD_RD_REG | reg);
	uint8_t response = nrf_spi_xfer_byte(_NRF24L01P_SPI_CMD_NOP);
	nrf_spi_csh();
	return response;
}

int nrf_write_reg(unsigned char reg, uint8_t data)
{
    nrf_spi_csl();
    nrf_spi_xfer_byte(_NRF24L01P_SPI_CMD_WR_REG | reg);
    uint8_t response = nrf_spi_xfer_byte(data);
    nrf_spi_csh();
	return response;
}

uint8_t nrf_write_reg_multi(uint8_t reg, uint8_t data[], uint8_t length)
{
	nrf_spi_csl();
	uint8_t response = nrf_spi_xfer_byte(_NRF24L01P_SPI_CMD_WR_REG | (_NRF24L01P_REG_ADDRESS_MASK & reg));
	for (uint8_t i = 0; i < length; i++)
	{
		nrf_spi_xfer_byte(data[i]);
	}
	nrf_spi_csh();
	return response;
}

void nrf_send_payload(uint8_t * data)
{
	nrf_spi_csl();
	nrf_spi_xfer_byte(_NRF24L01P_SPI_CMD_FLUSH_TX);
	nrf_spi_csh();

	nrf_spi_csl();
	nrf_spi_xfer_byte(_NRF24L01P_SPI_CMD_WR_TX_PAYLOAD);
	for (uint8_t i = 0; i < PAYLOADSIZE; i++)
	{
		nrf_spi_xfer_byte(data[i]);
	}
	nrf_spi_csh();
}

void nrf_read_payload(uint8_t * data, uint8_t length)
{
	nrf_spi_csl();
	nrf_spi_xfer_byte(_NRF24L01P_SPI_CMD_RD_RX_PAYLOAD);
	for (uint8_t i = 0; i < length; i++)
	{
		*data++ = nrf_spi_xfer_byte(_NRF24L01P_SPI_CMD_NOP);
	}
	nrf_spi_csh();
	nrf_write_reg(_NRF24L01P_REG_STATUS, _NRF24L01P_STATUS_RX_DR);
}

void nrf_init(void)
{
	nrf_cel();
	nrf_spi_csh();
	nrf_write_reg(_NRF24L01P_REG_CONFIG, 0); // power down
	nrf_write_reg(_NRF24L01P_REG_STATUS, _NRF24L01P_STATUS_MAX_RT|_NRF24L01P_STATUS_TX_DS|_NRF24L01P_STATUS_RX_DR); // clear interrupts
	nrf_write_reg(_NRF24L01P_REG_EN_AA, _NRF24L01P_EN_AA_NONE); // disable rx pipes
	nrf_write_reg(_NRF24L01P_REG_RF_CH, 100); // set frequency to 2500Mhz
	//nrf_write_reg(NRF24L01_06_RF_SETUP, 0x26); // 0dBm, 250kbps
	nrf_write_reg(_NRF24L01P_REG_RF_SETUP, 0x0E); // 0dBm, 2mbps
	uint8_t pipe = 0;
	int rxPwPxRegister = _NRF24L01P_REG_RX_PW_P0 + ( pipe - NRF24L01P_PIPE_P0 );
	nrf_write_reg(rxPwPxRegister, (PAYLOADSIZE & _NRF24L01P_RX_PW_Px_MASK));
	nrf_write_reg(_NRF24L01P_REG_EN_AA, _NRF24L01P_EN_AA_NONE); // disable auto ack
	nrf_write_reg(_NRF24L01P_REG_SETUP_RETR, _NRF24L01P_SETUP_RETR_NONE); // disable auto retransmit
	uint8_t config = nrf_read_reg(_NRF24L01P_REG_CONFIG);
	config |= _NRF24L01P_CONFIG_PWR_UP;
	nrf_write_reg(_NRF24L01P_REG_CONFIG, config); // power up
	config |= _NRF24L01P_CONFIG_PRIM_RX;
	nrf_write_reg(_NRF24L01P_REG_CONFIG, config); // set rx mode
	nrf_ceh();
}

const struct usb_device_descriptor dev_descr = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = 0,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0483,
    .idProduct = 0x5710,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] =
{
    /*
      0x06, 0xFF, 0xFF,         // 04|2   , Usage Page (vendordefined?)
      0x09, 0x01,               // 08|1   , Usage      (vendordefined
      0xA1, 0x01,               // A0|1   , Collection (Application)
      // Feature report
      0x09, 0x06,               // 08|1   , Usage      (vendordefined)
      0x09, 0x07,               // 08|1   , Usage      (vendordefined)
      0x15, 0x00,               // 14|1   , LogicalMinimum(0 for signed byte)
      0x75, 0x0F,               // 74|1   , Report Size(16) =field size in bits = 1 byte
      0x95, 0x08,               //_0x04,               // 94|1:ReportCount
      0xB1, 0x02,               // B0|1:   Feature report
      0xC0                      // C0|0    , End Collection
      */
          // Usage Page = 0xFF00 (Vendor Defined Page 1)
    0x06, 0x00, 0xFF,
    // Usage (Vendor Usage 1)
    0x09, 0x01,
    // Collection (Application)
    0xA1, 0x01,
    //   Usage Minimum
    0x19, 0x01,
    //   Usage Maximum. 64 input usages total (0x01 to 0x40).
    0x29, 0x40,
    //   Logical Minimum (data bytes in the report may have minimum value =
    //   0x00).
    0x15, 0x00,
    //   Logical Maximum (data bytes in the report may have
    //     maximum value = 0x00FF = unsigned 255).
    // TODO: Can this be one byte?
    0x26, 0xFF, 0x00,
    //   Report Size: 8-bit field size
    0x75, 0x08,
    //   Report Count: Make sixty-four 8-bit fields (the next time the parser
    //     hits an "Input", "Output", or "Feature" item).
    0x95, 0x40,
    //   Input (Data, Array, Abs): Instantiates input packet fields based on the
    //     above report size, count, logical min/max, and usage.
    0x81, 0x00,
    //   Usage Minimum
    0x19, 0x01,
    //   Usage Maximum. 64 output usages total (0x01 to 0x40)
    0x29, 0x40,
    //   Output (Data, Array, Abs): Instantiates output packet fields. Uses same
    //     report size and count as "Input" fields, since nothing new/different
    //     was specified to the parser since the "Input" item.
    0x91, 0x00,
    // End Collection
    0xC0,
 };


static const struct {
    struct usb_hid_descriptor hid_descriptor;
    struct {
        uint8_t bReportDescriptorType;
        uint16_t wDescriptorLength;
    } __attribute__((packed)) hid_report;
} __attribute__((packed)) hid_function = {
    .hid_descriptor = {
        .bLength = sizeof(hid_function),
        .bDescriptorType = USB_DT_HID,
        .bcdHID = 0x0100,
        .bCountryCode = 0,
        .bNumDescriptors = 1,
    },
    .hid_report = {
        .bReportDescriptorType = USB_DT_REPORT,
        .wDescriptorLength = sizeof(hid_report_descriptor),
    }
};

static const struct usb_endpoint_descriptor hid_endpoints[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = ENDPOINT_ADDRESS_IN,      //0x81
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = MAXPACKETSIZEIN,
    .bInterval = 1,
},
{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = ENDPOINT_ADDRESS_OUT,    //0x01
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = MAXPACKETSIZEOUT,
    .bInterval = 1,
}};

const struct usb_interface_descriptor hid_iface = {
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 0,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_HID,
    .bInterfaceSubClass = 0, /* no boot */
    .bInterfaceProtocol = 0, /* user (no mouse, keyboard, etc...)*/
    .iInterface = 0,

    .endpoint = hid_endpoints,

    .extra = &hid_function,
    .extralen = sizeof(hid_function),
};

const struct usb_interface ifaces[] = {{
    .num_altsetting = 1,
    .altsetting = &hid_iface
}};

const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 1,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0xC0,
    .bMaxPower = 0x32,

    .interface = ifaces,
};

static const char *usb_strings[] = {
    "iegget",
    "nRF24L01 HID Adapter",
    "PROTOTYPE",
};

/* Buffer used for control requests. */
uint8_t usbd_control_buffer[128];

//This function looks identical in all examples.
//And it is as I understand monitors all inbound hid-requests.
static int hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
            void (**complete)(usbd_device *dev, struct usb_setup_data *req))
{
    (void)complete;
    (void)dev;

    if((req->bmRequestType != ENDPOINT_ADDRESS_IN) ||
       (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
       (req->wValue != 0x2200))
        return 0;

    /* Handle the HID report descriptor. */
    *buf = (uint8_t *)hid_report_descriptor;
    *len = sizeof(hid_report_descriptor);

    return 1;
}                                                                               //

//This callback that is executed when the endpoint "IN" request arrives.
static void data_tx(usbd_device *dev, uint8_t ep)
{
    (void)ep;
    (void)dev;

    char buffer[MAXPACKETSIZEIN];
    //led_toggle();
    
    buffer[3] = 0x32;
    usbd_ep_write_packet(dev, ENDPOINT_ADDRESS_IN, buffer, MAXPACKETSIZEIN);
}

uint8_t lol = 0;

//This callback that is executed when the endpoint "OUT" request arrives.
static void data_rx(usbd_device *dev, uint8_t ep)
{
    (void)ep;
    (void)dev;

    char buffer[MAXPACKETSIZEIN];

    //Here we read buffer from the endpoint.
    int len = usbd_ep_read_packet(dev, ENDPOINT_ADDRESS_OUT, buffer, MAXPACKETSIZEOUT);
    /*
    if (len)
    {
        //led_toggle();
	    nrf_read_payload(buffer, PAYLOADSIZE);
	    if (buffer[0] != 0x6B && buffer[0] != 0x00)
	    {
		    //led_toggle();
	    }
        
        buffer[MAXPACKETSIZEIN] = 0;
    }
    */

    memset(buffer, 0, sizeof(buffer));

    nrf_read_payload(buffer, PAYLOADSIZE);
	if (buffer[0] != 0x6B && buffer[0] != 0x00)
	{
		led_toggle();
        usbd_ep_write_packet(dev, ENDPOINT_ADDRESS_IN, buffer, MAXPACKETSIZEIN);
	}
}

//In this function, configure the endpoints and callbacks.
static void hid_set_config(usbd_device *dev, uint16_t wValue)
{
    (void)wValue;

    usbd_ep_setup(dev, ENDPOINT_ADDRESS_IN, USB_ENDPOINT_ATTR_INTERRUPT, MAXPACKETSIZEIN, NULL); //data_tx);;
    usbd_ep_setup(dev, ENDPOINT_ADDRESS_OUT, USB_ENDPOINT_ATTR_INTERRUPT, MAXPACKETSIZEOUT, data_rx);

    usbd_register_control_callback(
                dev,
                USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
                USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
                hid_control_request);
}

void spi_setup(void)
{
	rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPBEN);
    rcc_peripheral_enable_clock(&RCC_APB1ENR, RCC_APB1ENR_SPI2EN);

    /* Configure SCK and MOSI */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO_SPI2_SCK | GPIO_SPI2_MOSI);

    /* Configure MISO */
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT,
                   GPIO_SPI2_MISO);

    /* Configure CS pin on PB12. */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO_SPI2_NSS);

	/* Configure CE pin on PA8. */
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                   GPIO_CNF_OUTPUT_PUSHPULL, GPIO0);

	nrf_cel();
    spi_set_unidirectional_mode(SPI2); 		/* We want to send only. */
    spi_set_dff_8bit(SPI2);
    spi_set_full_duplex_mode(SPI2);
    spi_send_msb_first(SPI2);

    /* Handle the CS signal in software. */
    spi_enable_software_slave_management(SPI2);
    spi_set_nss_high(SPI2);

    /* PCLOCK/8 as clock. */
    spi_set_baudrate_prescaler(SPI2, SPI_CR1_BR_FPCLK_DIV_8);

    /* We want to control everything and generate the clock -> master. */
    spi_set_master_mode(SPI2);
    spi_set_clock_polarity_0(SPI2); /* SCK idle state low. */

    /* Bit is taken on the second (falling edge) of SCK. */
    spi_set_clock_phase_0(SPI2);
    spi_enable_ss_output(SPI2);

    nrf_spi_csh();

    spi_enable(SPI2);
}

static void timer_setup(void)
{
    /* Set up the timer TIM2 for injected sampling */
    uint32_t timer;

    timer   = TIM2;
    rcc_periph_clock_enable(RCC_TIM2);

    /* Time Base configuration */
    timer_reset(timer);
    timer_set_mode(timer, TIM_CR1_CKD_CK_INT,
        TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_period(timer, 0xFF);
    timer_set_prescaler(timer, 0x8);
    timer_set_clock_division(timer, 0x0);
    /* Generate TRGO on every update. */
    timer_set_master_mode(timer, TIM_CR2_MMS_UPDATE);
    timer_enable_counter(timer);
}

static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
    // led
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

//The function which collects peripheral initialization.
static int init_peripheral(void)
{
    rcc_clock_setup_in_hsi_out_48mhz();

    rcc_peripheral_enable_clock(&RCC_APB2ENR, RCC_APB2ENR_IOPCEN);

    timer_setup();
    gpio_setup();
    spi_setup();
	nrf_init();

    usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(usbd_dev, hid_set_config);

    my_delay(0x80000);

    return 1;
}

int main(void)
{
    init_peripheral();

    while (1)
    {
        usbd_poll(usbd_dev);
    }
        
}


