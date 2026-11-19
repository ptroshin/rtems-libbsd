#include <machine/rtems-bsd-kernel-space.h>

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/systm.h>

#include <rtems/bsd/grusbdc.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usb_dynamic.h>
#include <dev/usb/usb_device.h>
#include <dev/usb/usbdi_util.h>

#include <bsp.h>
#if defined(BSP_HAS_GRUSBDC)

#include <bsp/grusbdc.h>

#define GRUSBDC_STATUS_FULL_SPEED  (1u << 14)
#define GRUSBDC_BULK_HS_PACKET     512u
#define GRUSBDC_BULK_FS_PACKET      64u

/* Vendor-specific interface for exercising GRUSBDC bulk endpoint setup. */
static struct {
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface;
	struct usb_endpoint_descriptor bulk_out;
	struct usb_endpoint_descriptor bulk_in;
} grusbdc_config = {
	.config = {
		.bLength = sizeof(struct usb_config_descriptor),
		.bDescriptorType = UDESC_CONFIG,
		.wTotalLength = { 32, 0 },
		.bNumInterface = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = UC_BUS_POWERED,
		.bMaxPower = 25
	},
	.interface = {
		.bLength = sizeof(struct usb_interface_descriptor),
		.bDescriptorType = UDESC_INTERFACE,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 2,
		.bInterfaceClass = UDCLASS_VENDOR,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 0
	},
	.bulk_out = {
		.bLength = sizeof(struct usb_endpoint_descriptor),
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = 0x01,
		.bmAttributes = UE_BULK,
		.wMaxPacketSize = { 0, 2 },
		.bInterval = 0
	},
	.bulk_in = {
		.bLength = sizeof(struct usb_endpoint_descriptor),
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = 0x81,
		.bmAttributes = UE_BULK,
		.wMaxPacketSize = { 0, 2 },
		.bInterval = 0
	}
};

static const struct usb_device_descriptor grusbdc_device = {
	.bLength = sizeof(struct usb_device_descriptor),
	.bDescriptorType = UDESC_DEVICE,
	.bcdUSB = { 0x00, 0x02 },
	.bDeviceClass = UDCLASS_IN_INTERFACE,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize = 64,
	.idVendor = { 0xc0, 0x16 },
	.idProduct = { 0xdd, 0x27 },
	.bcdDevice = { 0x00, 0x01 },
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 0,
	.bNumConfigurations = 1
};

USB_MAKE_STRING_DESC("G\0a\0i\0s\0l\0e\0r", grusbdc_manufacturer);
USB_MAKE_STRING_DESC("G\0R\0U\0S\0B\0D\0C",
	grusbdc_product);

static usb_error_t
grusbdc_template_get_desc(struct usb_device *udev,
	struct usb_device_request *req, const void **data, uint16_t *length)
{
	static const uint8_t language[] = { 4, UDESC_STRING, 0x09, 0x04 };
	static const uint8_t status[] = { 0, 0 };
	static uint8_t configuration;
	uint16_t value;
	uint8_t index;

	*data = NULL;
	*length = 0;
	value = UGETW(req->wValue);
	index = value & 0xff;

	/* Device-mode USB core uses this hook for its local standard requests. */
	switch (req->bRequest) {
	case UR_SET_ADDRESS:
		if (req->bmRequestType == UT_WRITE_DEVICE && value < 128 &&
		    UGETW(req->wLength) == 0)
			return (0);
		return (USB_ERR_STALLED);
	case UR_SET_CONFIG:
		if (req->bmRequestType == UT_WRITE_DEVICE && value <= 1 &&
		    UGETW(req->wLength) == 0)
			return (0);
		return (USB_ERR_STALLED);
	case UR_GET_CONFIG:
		if (req->bmRequestType != UT_READ_DEVICE || udev == NULL ||
		    UGETW(req->wLength) == 0)
			return (USB_ERR_STALLED);
		configuration = udev->curr_config_no;
		*data = &configuration;
		*length = 1;
		return (0);
	case UR_GET_STATUS:
		if (req->bmRequestType != UT_READ_DEVICE ||
		    UGETW(req->wLength) < sizeof(status))
			return (USB_ERR_STALLED);
		*data = status;
		*length = sizeof(status);
		return (0);
	case UR_GET_DESCRIPTOR:
		if (req->bmRequestType != UT_READ_DEVICE)
			return (USB_ERR_STALLED);
		break;
	default:
		return (USB_ERR_STALLED);
	}

	switch (value >> 8) {
	case UDESC_DEVICE:
		if (index != 0)
			break;
		*data = &grusbdc_device;
		*length = sizeof(grusbdc_device);
		return (0);
	case UDESC_CONFIG:
		if (index != 0)
			break;
		if ((bsp_grusbdc_get_status() &
		    GRUSBDC_STATUS_FULL_SPEED) != 0) {
			grusbdc_config.bulk_out.wMaxPacketSize[0] =
			    GRUSBDC_BULK_FS_PACKET;
			grusbdc_config.bulk_out.wMaxPacketSize[1] = 0;
			grusbdc_config.bulk_in.wMaxPacketSize[0] =
			    GRUSBDC_BULK_FS_PACKET;
			grusbdc_config.bulk_in.wMaxPacketSize[1] = 0;
		} else {
			grusbdc_config.bulk_out.wMaxPacketSize[0] = 0;
			grusbdc_config.bulk_out.wMaxPacketSize[1] = 2;
			grusbdc_config.bulk_in.wMaxPacketSize[0] = 0;
			grusbdc_config.bulk_in.wMaxPacketSize[1] = 2;
		}
		*data = &grusbdc_config;
		*length = sizeof(grusbdc_config);
		return (0);
	case UDESC_STRING:
		switch (index) {
		case 0:
			*data = language;
			*length = sizeof(language);
			return (0);
		case 1:
			*data = &grusbdc_manufacturer;
			*length = sizeof(grusbdc_manufacturer);
			return (0);
		case 2:
			*data = &grusbdc_product;
			*length = sizeof(grusbdc_product);
			return (0);
		default:
			break;
		}
		break;
	default:
		break;
	}
	return (USB_ERR_STALLED);
}

static usb_error_t
grusbdc_template_setup(struct usb_device *udev, uint16_t index)
{
	(void)udev;
	return (index == 0 ? 0 : USB_ERR_INVAL);
}

void
rtems_bsd_grusbdc_use_echo_template(void)
{
	/* Select the profile before the USB bus is attached. */
	usb_temp_get_desc_p = grusbdc_template_get_desc;
	usb_temp_setup_by_index_p = grusbdc_template_setup;
	usb_template = 0;
}

#endif /* BSP_HAS_GRUSBDC */
