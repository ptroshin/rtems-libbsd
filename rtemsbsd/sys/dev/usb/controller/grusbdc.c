#include <machine/rtems-bsd-kernel-space.h>

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/systm.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/condvar.h>

#include <machine/bus.h>
#include <bsp.h>
#include <leon.h>
#include <rtems/irq-extension.h>

#if defined(BSP_HAS_GRUSBDC)

#include <bsp/grusbdc.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usb_core.h>
#include <dev/usb/usb_busdma.h>
#include <dev/usb/usb_transfer.h>
#include <dev/usb/usb_process.h>
#include <dev/usb/usb_device.h>
#include <dev/usb/usb_controller.h>
#include <dev/usb/usb_bus.h>
#include <dev/usb/usb_dynamic.h>
#include <dev/usb/usb_hub.h>

#define GRUSBDC_STATUS             0x204
#define GRUSBDC_STATUS_NEPI_MASK   0xf0000000u
#define GRUSBDC_STATUS_NEPI_SHIFT  28
#define GRUSBDC_STATUS_NEPO_MASK   0x0f000000u
#define GRUSBDC_STATUS_NEPO_SHIFT  24
#define GRUSBDC_STATUS_DMA_MASTER  (1u << 23)

/* GRIP 2021.1, section 74.3: three big-endian words per DMA descriptor. */
#define GRUSBDC_DMA_DESC_ALIGN     4u
#define GRUSBDC_DMA_DESC_SIZE      12u
#define GRUSBDC_DESC_SE            (1u << 17)
#define GRUSBDC_DESC_IE            (1u << 15)
#define GRUSBDC_DESC_NX            (1u << 14)
#define GRUSBDC_DESC_EN            (1u << 13)
#define GRUSBDC_DESC_LENGTH_MASK   0x1fffu
#define GRUSBDC_DESC_ADDR_MASK     0xfffffffcu

#define GRUSBDC_EP_STRIDE          0x10u
#define GRUSBDC_OUT_BASE           0x000u
#define GRUSBDC_IN_BASE            0x100u
#define GRUSBDC_EPCTRL_MAXPL_SHIFT 7u
#define GRUSBDC_EPCTRL_PI          (1u << 20)
#define GRUSBDC_EPCTRL_CB          (1u << 19)
#define GRUSBDC_EPCTRL_TT_SHIFT    3u
#define GRUSBDC_EPCTRL_EV          (1u << 0)
#define GRUSBDC_GLOBAL_CTRL        0x200u
#define GRUSBDC_GLOBAL_STATUS      0x204u
#define GRUSBDC_CTRL_SI            (1u << 31)
#define GRUSBDC_CTRL_UI            (1u << 30)
#define GRUSBDC_CTRL_VI            (1u << 29)
#define GRUSBDC_CTRL_SP            (1u << 28)
#define GRUSBDC_CTRL_EP            (1u << 14)
#define GRUSBDC_CTRL_UA_MASK       0x000000feu
#define GRUSBDC_CTRL_UA_SHIFT      1u
#define GRUSBDC_CTRL_SU            (1u << 0)
#define GRUSBDC_STATUS_SUSPENDED   (1u << 17)
#define GRUSBDC_STATUS_USB_RESET   (1u << 16)
#define GRUSBDC_STATUS_VBUS_VALID  (1u << 15)
#define GRUSBDC_STATUS_FULL_SPEED  (1u << 14)
#define GRUSBDC_EPCTRL_BUFSZ_SHIFT 21u
#define GRUSBDC_EPCTRL_CS          (1u << 18)
#define GRUSBDC_DMACTRL_DA         (1u << 0)
#define GRUSBDC_DMACTRL_IE         (1u << 1)
#define GRUSBDC_DMACTRL_AI         (1u << 2)
#define GRUSBDC_DMACTRL_AD         (1u << 3)
#define GRUSBDC_EPSTAT_PT          (1u << 29)
#define GRUSBDC_EPSTAT_PR          (1u << 29)
#define GRUSBDC_EPSTAT_B1          (1u << 2)
#define GRUSBDC_EPSTAT_B0          (1u << 1)
#define GRUSBDC_DESC_IN_PI         (1u << 17)
#define GRUSBDC_DESC_POOL_SIZE     8192u
#define GRUSBDC_DESC_POOL_COUNT    (GRUSBDC_DESC_POOL_SIZE / GRUSBDC_DMA_DESC_SIZE)
#define GRUSBDC_DESC_BITMAP_WORDS  ((GRUSBDC_DESC_POOL_COUNT + 31u) / 32u)
#define GRUSBDC_EP0_DESC_OUT       0u
#define GRUSBDC_EP0_DESC_IN        1u
#define GRUSBDC_EP0_SETUP_OFFSET   4096u
#define GRUSBDC_EP0_REPLY_OFFSET   4160u
#define GRUSBDC_EP0_MAX_PACKET     64u
#define GRUSBDC_EP1_OUT_DESC       2u
#define GRUSBDC_EP1_IN_DESC        3u
#define GRUSBDC_EP1_OUT_OFFSET     5120u
#define GRUSBDC_EP1_IN_OFFSET      5632u
#define GRUSBDC_EP1_BUFFER_SIZE    512u
#define GRUSBDC_EP1_QUEUE_COUNT    16u

struct grusbdc_dma_desc {
	uint32_t control;
	uint32_t buffer;
	uint32_t next;
} __aligned(GRUSBDC_DMA_DESC_ALIGN);

CTASSERT(sizeof(struct grusbdc_dma_desc) == GRUSBDC_DMA_DESC_SIZE);

struct grusbdc_ep {
	struct usb_hw_ep_profile profile;
	uint16_t max_packet;
	uint8_t transfer_type;
	uint8_t enabled;
};


struct grusbdc_softc {
	struct usb_bus bus;
	struct usb_device *devices[USB_MAX_DEVICES];
	struct grusbdc_ep ep_in[4];
	struct grusbdc_ep ep_out[4];
	uint8_t ep1_queue[GRUSBDC_EP1_QUEUE_COUNT][GRUSBDC_EP1_BUFFER_SIZE];
	uint16_t ep1_queue_length[GRUSBDC_EP1_QUEUE_COUNT];
	uint8_t ep1_queue_head;
	uint8_t ep1_queue_count;
	uint8_t ep1_out_armed;
	uint8_t ep1_in_active;
	struct usb_xfer *ep0_xfer;
	uint8_t ep0_stage;
	uint32_t ep1_rx_packets;
	uint32_t ep1_tx_packets;
	struct resource *mem_res;
	struct resource *irq_res;
	void *intr_cookie;
	bus_dma_tag_t desc_dma_tag;
	bus_dmamap_t desc_dma_map;
	struct grusbdc_dma_desc *desc_pool;
	bus_addr_t desc_pool_bus_addr;
	uint32_t desc_bitmap[GRUSBDC_DESC_BITMAP_WORDS];
	volatile uint32_t last_status;
	struct usb_port_status hub_port_status;
	uint16_t hub_port_change;
	uint8_t hub_config;
	bool hub_port_power;
	volatile uint32_t reset_count;
	volatile uint32_t ep0_setup_count;
	volatile uint32_t ep0_stall_count;
	uint8_t ep0_configuration;
	uint8_t ep0_pending_address;
	uint8_t ep0_set_address_pending;
	bus_space_tag_t bst;
	bus_space_handle_t bsh;
};

enum {
	GRUSBDC_EP0_WAIT_SETUP,
	GRUSBDC_EP0_DATA_IN,
	GRUSBDC_EP0_DATA_OUT,
	GRUSBDC_EP0_STATUS_IN,
	GRUSBDC_EP0_STATUS_OUT
};

static void grusbdc_ctrl_start(struct usb_xfer *xfer);
static void grusbdc_ctrl_close(struct usb_xfer *xfer);
static void grusbdc_ctrl_open(struct usb_xfer *xfer);
static void grusbdc_ctrl_enter(struct usb_xfer *xfer);

static const struct usb_pipe_methods grusbdc_ctrl_methods = {
	.open = grusbdc_ctrl_open,
	.close = grusbdc_ctrl_close,
	.enter = grusbdc_ctrl_enter,
	.start = grusbdc_ctrl_start
};

static void grusbdc_endpoint_init(struct usb_device *udev,
	struct usb_endpoint_descriptor *edesc, struct usb_endpoint *ep);
static void grusbdc_get_hw_ep_profile(struct usb_device *udev,
	const struct usb_hw_ep_profile **profile, uint8_t address);
static void grusbdc_ep0_arm_setup(struct grusbdc_softc *sc);
static void grusbdc_ep0_send(struct grusbdc_softc *sc,
	const void *data, uint16_t length);
static void grusbdc_ep0_stall(struct grusbdc_softc *sc);
static void grusbdc_dma_sync(struct grusbdc_softc *sc, int operations);
static void grusbdc_ep1_arm_out(struct grusbdc_softc *sc);

static usb_error_t grusbdc_bus_device_init(struct usb_device *udev);
static void grusbdc_bus_xfer_setup(struct usb_setup_params *parm);
static void grusbdc_bus_xfer_unsetup(struct usb_xfer *xfer);
static void grusbdc_bus_get_dma_delay(struct usb_device *udev,
	uint32_t *delay);
static void grusbdc_bus_device_suspend(struct usb_device *udev);
static void grusbdc_bus_device_resume(struct usb_device *udev);
static void grusbdc_bus_power(struct usb_bus *bus);
static void grusbdc_bus_power_sleep(struct usb_bus *bus, uint32_t state);
static void grusbdc_bus_xfer_stall(struct usb_xfer *xfer);
static void grusbdc_bus_set_stall(struct usb_device *udev,
	struct usb_endpoint *ep, uint8_t *did_stall);
static void grusbdc_bus_clear_stall(struct usb_device *udev,
	struct usb_endpoint *ep);
static usb_handle_req_t grusbdc_roothub_exec;

static const struct usb_bus_methods grusbdc_bus_methods = {
	.roothub_exec = grusbdc_roothub_exec,
	.endpoint_init = grusbdc_endpoint_init,
	.xfer_setup = grusbdc_bus_xfer_setup,
	.xfer_unsetup = grusbdc_bus_xfer_unsetup,
	.get_dma_delay = grusbdc_bus_get_dma_delay,
	.device_suspend = grusbdc_bus_device_suspend,
	.device_resume = grusbdc_bus_device_resume,
	.set_hw_power = grusbdc_bus_power,
	.set_hw_power_sleep = grusbdc_bus_power_sleep,
	.get_hw_ep_profile = grusbdc_get_hw_ep_profile,
	.xfer_stall = grusbdc_bus_xfer_stall,
	.set_stall = grusbdc_bus_set_stall,
	.clear_stall = grusbdc_bus_clear_stall,
	.device_init = grusbdc_bus_device_init
};

/* Temporary board diagnostics; remove after EP0 is validated. */
volatile uint32_t grusbdc_debug[80];

#define GRUSBDC_RESET_TRACE_COUNT 8u
#define GRUSBDC_RESET_TRACE_WORDS 7u

/* GSTAT, then EP0 OUT status, DMA control, DMA address and descriptor,
 * followed by EP0 IN status and DMA control. */
volatile uint32_t grusbdc_reset_trace[GRUSBDC_RESET_TRACE_COUNT][2]
    [GRUSBDC_RESET_TRACE_WORDS];
volatile uint32_t grusbdc_reset_trace_seq;



struct grusbdc_dma_load {
	bus_addr_t address;
	int error;
};

static void
grusbdc_dma_load_cb(void *arg, bus_dma_segment_t *segs, int nsegs,
	int error)
{
	struct grusbdc_dma_load *load;

	load = arg;
	if (error != 0 || nsegs != 1 || segs == NULL ||
	    segs[0].ds_addr > BUS_SPACE_MAXADDR_32BIT ||
	    segs[0].ds_len < GRUSBDC_DESC_POOL_SIZE ||
	    segs[0].ds_addr + GRUSBDC_DESC_POOL_SIZE - 1 >
	    BUS_SPACE_MAXADDR_32BIT) {
		load->error = (error != 0 ? error : EFBIG);
		return;
	}
	load->address = segs[0].ds_addr;
	load->error = 0;
}

static int
grusbdc_dma_pool_init(device_t dev, struct grusbdc_softc *sc)
{
	struct grusbdc_dma_load load;
	int error;

	error = bus_dma_tag_create(USB_GET_DMA_TAG(dev), GRUSBDC_DMA_DESC_ALIGN,
	    0, BUS_SPACE_MAXADDR_32BIT, BUS_SPACE_MAXADDR, NULL, NULL,
	    GRUSBDC_DESC_POOL_SIZE, 1, GRUSBDC_DESC_POOL_SIZE, 0, NULL, NULL,
	    &sc->desc_dma_tag);
	if (error != 0)
		return (error);
	error = bus_dmamem_alloc(sc->desc_dma_tag, (void **)&sc->desc_pool,
	    BUS_DMA_WAITOK | BUS_DMA_COHERENT | BUS_DMA_ZERO, &sc->desc_dma_map);
	if (error != 0)
		goto fail_tag;
	load.address = 0;
	load.error = EIO;
	error = bus_dmamap_load(sc->desc_dma_tag, sc->desc_dma_map,
	    sc->desc_pool, GRUSBDC_DESC_POOL_SIZE, grusbdc_dma_load_cb,
	    &load, BUS_DMA_WAITOK);
	if (error == 0)
		error = load.error;
	if (error != 0)
		goto fail_mem;
	sc->desc_pool_bus_addr = load.address;
	return (0);

fail_mem:
	bus_dmamem_free(sc->desc_dma_tag, sc->desc_pool, sc->desc_dma_map);
	sc->desc_pool = NULL;
fail_tag:
	bus_dma_tag_destroy(sc->desc_dma_tag);
	sc->desc_dma_tag = NULL;
	return (error);
}

static void
grusbdc_dma_pool_uninit(struct grusbdc_softc *sc)
{
	if (sc->desc_pool != NULL) {
		bus_dmamap_unload(sc->desc_dma_tag, sc->desc_dma_map);
		bus_dmamem_free(sc->desc_dma_tag, sc->desc_pool, sc->desc_dma_map);
		sc->desc_pool = NULL;
		sc->desc_dma_map = NULL;
	}
	if (sc->desc_dma_tag != NULL) {
		bus_dma_tag_destroy(sc->desc_dma_tag);
		sc->desc_dma_tag = NULL;
	}
}

/* The USB bus mutex must be held while allocating or freeing descriptors. */
static struct grusbdc_dma_desc *
grusbdc_dma_desc_alloc(struct grusbdc_softc *sc, bus_addr_t *address)
{
	unsigned int i;
	uint32_t bit;

	for (i = 0; i < GRUSBDC_DESC_POOL_COUNT; ++i) {
		bit = 1u << (i & 31u);
		if ((sc->desc_bitmap[i >> 5] & bit) != 0)
			continue;
		sc->desc_bitmap[i >> 5] |= bit;
		memset(&sc->desc_pool[i], 0, sizeof(sc->desc_pool[i]));
		*address = sc->desc_pool_bus_addr + i * sizeof(sc->desc_pool[i]);
		return (&sc->desc_pool[i]);
	}
	return (NULL);
}

static void
grusbdc_dma_desc_free(struct grusbdc_softc *sc,
	struct grusbdc_dma_desc *desc)
{
	uintptr_t offset;
	unsigned int i;

	if (desc < sc->desc_pool || desc >= sc->desc_pool + GRUSBDC_DESC_POOL_COUNT)
		return;
	offset = (uintptr_t)desc - (uintptr_t)sc->desc_pool;
	if (offset % sizeof(*desc) != 0)
		return;
	i = offset / sizeof(*desc);
	memset(desc, 0, sizeof(*desc));
	sc->desc_bitmap[i >> 5] &= ~(1u << (i & 31u));
}

static struct grusbdc_softc *
grusbdc_bus_to_softc(struct usb_bus *bus)
{
	return ((struct grusbdc_softc *)((char *)bus -
	    offsetof(struct grusbdc_softc, bus)));
}

static void
grusbdc_get_hw_ep_profile(struct usb_device *udev,
	const struct usb_hw_ep_profile **profile, uint8_t address)
{
	struct grusbdc_softc *sc;
	uint8_t epno;

	sc = grusbdc_bus_to_softc(udev->bus);
	epno = address & UE_ADDR;
	if (epno >= nitems(sc->ep_in)) {
		*profile = NULL;
		return;
	}
	*profile = &sc->ep_in[epno].profile;
}

static void
grusbdc_init_endpoint_profiles(struct grusbdc_softc *sc)
{
	struct usb_hw_ep_profile *pf;
	unsigned int i;

	for (i = 0; i < nitems(sc->ep_in); ++i) {
		pf = &sc->ep_in[i].profile;
		pf->max_in_frame_size = 1024;
		pf->max_out_frame_size = 1024;
		pf->is_simplex = 0;
		pf->support_multi_buffer = 1;
		pf->support_control = 1;
		pf->support_bulk = 1;
		pf->support_interrupt = 1;
		pf->support_isochronous = 1;
		pf->support_in = 1;
		pf->support_out = 1;
		sc->ep_out[i].profile = *pf;
	}
}

static int
grusbdc_configure_endpoint(struct grusbdc_softc *sc,
	uint8_t epno, uint8_t direction, uint8_t type, uint16_t max_packet)
{
	bus_size_t offset;
	uint32_t control;

	if (epno >= nitems(sc->ep_in) || type > UE_INTERRUPT ||
	    max_packet == 0 || max_packet > 1024)
		return (EINVAL);

	offset = (direction == UE_DIR_IN ? GRUSBDC_IN_BASE :
	    GRUSBDC_OUT_BASE) + epno * GRUSBDC_EP_STRIDE;
	/* Disable first: GRIP requires EV to be cleared before changing type/MPS. */
	bus_space_write_4(sc->bst, sc->bsh, offset, 0);
	control = ((uint32_t)max_packet << GRUSBDC_EPCTRL_MAXPL_SHIFT) |
	    ((uint32_t)type << GRUSBDC_EPCTRL_TT_SHIFT) | GRUSBDC_EPCTRL_EV;
	if (direction == UE_DIR_IN && epno != 0)
		control |= GRUSBDC_EPCTRL_PI;
	if (epno == 0 && direction != UE_DIR_IN)
		control |= GRUSBDC_EPCTRL_PI;
	bus_space_write_4(sc->bst, sc->bsh, offset, control);

	if (direction == UE_DIR_IN) {
		sc->ep_in[epno].max_packet = max_packet;
		sc->ep_in[epno].transfer_type = type;
		sc->ep_in[epno].enabled = 1;
	} else {
		sc->ep_out[epno].max_packet = max_packet;
		sc->ep_out[epno].transfer_type = type;
		sc->ep_out[epno].enabled = 1;
	}
	return (0);
}

static void
grusbdc_endpoint_init(struct usb_device *udev,
	struct usb_endpoint_descriptor *edesc, struct usb_endpoint *ep)
{
	struct grusbdc_softc *sc;
	uint8_t address;
	uint8_t direction;
	uint8_t epno;
	uint8_t type;
	uint16_t max_packet;
	int error;

	/* The IRQ handler owns the physical endpoints, including EP0. */
	if (udev->flags.usb_mode == USB_MODE_HOST || edesc->bEndpointAddress == 0)
		return;
	sc = grusbdc_bus_to_softc(udev->bus);
	address = edesc->bEndpointAddress;
	epno = address & UE_ADDR;
	direction = address & UE_DIR_IN;
	type = edesc->bmAttributes & UE_XFERTYPE;
	max_packet = UGETW(edesc->wMaxPacketSize) & 0x7ff;

	if (epno == 0 && type != UE_CONTROL) {
		device_printf(sc->bus.parent, "EP0 must be a control endpoint\n");
		return;
	}
	error = grusbdc_configure_endpoint(sc, epno, direction, type,
	    max_packet);
	if (error != 0) {
		device_printf(sc->bus.parent,
		    "invalid endpoint %u configuration (type %u, max packet %u)\n",
		    epno, type, max_packet);
		return;
	}
	if (epno == 0)
		ep->methods = &grusbdc_ctrl_methods;
}

static void
grusbdc_ctrl_open(struct usb_xfer *xfer)
{
	(void)xfer;
}

static void
grusbdc_ctrl_enter(struct usb_xfer *xfer)
{
	(void)xfer;
}

static void
grusbdc_ctrl_close(struct usb_xfer *xfer)
{
	struct grusbdc_softc *sc;

	sc = grusbdc_bus_to_softc(xfer->xroot->bus);
	if (sc->ep0_xfer == xfer)
		sc->ep0_xfer = NULL;
}

static void
grusbdc_ctrl_start(struct usb_xfer *xfer)
{
	struct grusbdc_softc *sc;
	struct grusbdc_dma_desc *desc;
	uint8_t *buffer;
	uint8_t dir_in;
	uint16_t length;

	sc = grusbdc_bus_to_softc(xfer->xroot->bus);
	sc->ep0_xfer = xfer;
	grusbdc_debug[69]++;
	grusbdc_debug[70] = (xfer->flags_int.control_hdr << 1) |
	    xfer->flags_int.control_act;
	if (xfer->flags_int.control_hdr) {
		sc->ep0_stage = GRUSBDC_EP0_WAIT_SETUP;
		grusbdc_ep0_arm_setup(sc);
		return;
	}
	dir_in = (xfer->endpointno & UE_DIR_IN) != 0;
	if (xfer->flags_int.control_act) {
		length = usbd_xfer_frame_len(xfer, 1);
		if (length > GRUSBDC_EP0_MAX_PACKET) {
			usbd_xfer_set_stall(xfer);
			return;
		}
		if (dir_in) {
			buffer = usbd_xfer_get_frame_buffer(xfer, 1);
			grusbdc_ep0_send(sc, buffer, length);
			sc->ep0_stage = GRUSBDC_EP0_DATA_IN;
		} else {
			desc = &sc->desc_pool[GRUSBDC_EP0_DESC_OUT];
			desc->control = 0;
			desc->buffer = sc->desc_pool_bus_addr +
			    GRUSBDC_EP0_REPLY_OFFSET;
			desc->next = 0;
			grusbdc_dma_sync(sc,
			    BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
			desc->control = GRUSBDC_DESC_IE | GRUSBDC_DESC_EN;
			grusbdc_dma_sync(sc, BUS_DMASYNC_PREWRITE);
			bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE + 0x08,
			    sc->desc_pool_bus_addr + GRUSBDC_EP0_DESC_OUT *
			    sizeof(struct grusbdc_dma_desc));
			bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE + 0x04,
			    GRUSBDC_DMACTRL_IE | GRUSBDC_DMACTRL_AI |
			    GRUSBDC_DMACTRL_DA);
			sc->ep0_stage = GRUSBDC_EP0_DATA_OUT;
		}
	} else if (dir_in) {
		desc = &sc->desc_pool[GRUSBDC_EP0_DESC_OUT];
		desc->control = 0;
		desc->buffer = sc->desc_pool_bus_addr + GRUSBDC_EP0_REPLY_OFFSET;
		desc->next = 0;
		grusbdc_dma_sync(sc, BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
		desc->control = GRUSBDC_DESC_IE | GRUSBDC_DESC_EN;
		grusbdc_dma_sync(sc, BUS_DMASYNC_PREWRITE);
		bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE + 0x08,
		    sc->desc_pool_bus_addr + GRUSBDC_EP0_DESC_OUT *
		    sizeof(struct grusbdc_dma_desc));
		bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE + 0x04,
		    GRUSBDC_DMACTRL_IE | GRUSBDC_DMACTRL_AI | GRUSBDC_DMACTRL_DA);
		sc->ep0_stage = GRUSBDC_EP0_STATUS_OUT;
	} else {
		if (sc->ep0_set_address_pending) {
			uint32_t control;

			control = bus_space_read_4(sc->bst, sc->bsh,
			    GRUSBDC_GLOBAL_CTRL);
			control &= ~GRUSBDC_CTRL_UA_MASK;
			control |= ((uint32_t)sc->ep0_pending_address <<
			    GRUSBDC_CTRL_UA_SHIFT) & GRUSBDC_CTRL_UA_MASK;
			control |= GRUSBDC_CTRL_SU;
			bus_space_write_4(sc->bst, sc->bsh,
			    GRUSBDC_GLOBAL_CTRL, control);
			sc->ep0_set_address_pending = 0;
		}
		grusbdc_ep0_send(sc, NULL, 0);
		sc->ep0_stage = GRUSBDC_EP0_STATUS_IN;
	}
}

static void
grusbdc_bus_xfer_setup(struct usb_setup_params *parm)
{
	parm->hc_max_packet_size = GRUSBDC_EP0_MAX_PACKET;
	parm->hc_max_frame_size = GRUSBDC_EP0_MAX_PACKET;
	parm->hc_max_packet_count = 1;
	usbd_transfer_setup_sub(parm);
}

static void
grusbdc_bus_xfer_unsetup(struct usb_xfer *xfer)
{
	(void)xfer;
}

static usb_error_t
grusbdc_bus_device_init(struct usb_device *udev)
{
	/*
	 * Serve EP0 setup requests in grusbdc_intr(). The generic
	 * FreeBSD control-transfer path was observed to prepare the device
	 * descriptor but fail to complete enumeration on this controller.
	 * The direct path handles the data and status stages together with
	 * the GRUSBDC DMA descriptors.
	 */
	(void)udev;
	grusbdc_debug[66] = 0;
	grusbdc_debug[67] = 0;
	return (0);
}

static void
grusbdc_bus_get_dma_delay(struct usb_device *udev, uint32_t *delay)
{
	(void)udev;
	*delay = 0;
}

static void
grusbdc_bus_device_suspend(struct usb_device *udev)
{
	(void)udev;
}

static void
grusbdc_bus_device_resume(struct usb_device *udev)
{
	(void)udev;
}

static void
grusbdc_bus_power(struct usb_bus *bus)
{
	(void)bus;
}

static void
grusbdc_bus_power_sleep(struct usb_bus *bus, uint32_t state)
{
	(void)bus;
	(void)state;
}

static void
grusbdc_bus_xfer_stall(struct usb_xfer *xfer)
{
	struct grusbdc_softc *sc;

	sc = grusbdc_bus_to_softc(xfer->xroot->bus);
	grusbdc_ep0_stall(sc);
}

static void
grusbdc_bus_set_stall(struct usb_device *udev, struct usb_endpoint *ep,
	uint8_t *did_stall)
{
	struct grusbdc_softc *sc;
	uint32_t value;

	(void)ep;
	sc = grusbdc_bus_to_softc(udev->bus);
	value = bus_space_read_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE);
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE,
	    value | GRUSBDC_EPCTRL_CS);
	value = bus_space_read_4(sc->bst, sc->bsh, GRUSBDC_IN_BASE);
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_IN_BASE,
	    value | GRUSBDC_EPCTRL_CS);
	*did_stall = 1;
}

static void
grusbdc_bus_clear_stall(struct usb_device *udev, struct usb_endpoint *ep)
{
	struct grusbdc_softc *sc;

	(void)ep;
	sc = grusbdc_bus_to_softc(udev->bus);
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE,
	    bus_space_read_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE) &
	    ~GRUSBDC_EPCTRL_CS);
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_IN_BASE,
	    bus_space_read_4(sc->bst, sc->bsh, GRUSBDC_IN_BASE) &
	    ~GRUSBDC_EPCTRL_CS);
}

static void
grusbdc_dma_cache_flush(void)
{
	/* LEON3 BSP cache range hooks are TODO; use the architectural DFLUSH. */
	__asm__ volatile ("sta %%g0, [%%g0] 0x11" ::: "memory");
}

static uint32_t
grusbdc_dma_read32(const volatile void *address)
{
	uint32_t value;

	/* Match U-Boot's SPARC_NOCACHE_READ for DMA-owned words. */
	__asm__ volatile ("lda [%1] 0x01, %0" : "=r" (value) :
	    "r" (address) : "memory");
	return (value);
}

static uint8_t
grusbdc_dma_read8(const volatile void *address)
{
	uint32_t value;

	__asm__ volatile ("lduba [%1] 0x01, %0" : "=r" (value) :
	    "r" (address) : "memory");
	return ((uint8_t)value);
}

static void
grusbdc_dma_copy_from(void *destination, const volatile void *source,
	size_t length)
{
	uint8_t *dst;
	const volatile uint8_t *src;
	size_t i;

	dst = destination;
	src = source;
	for (i = 0; i < length; ++i)
		dst[i] = grusbdc_dma_read8(&src[i]);
}

static void
grusbdc_dma_sync(struct grusbdc_softc *sc, int operations)
{
	(void)sc;
	if ((operations & (BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD)) != 0)
		grusbdc_dma_cache_flush();
}

static void
grusbdc_ep0_arm_setup(struct grusbdc_softc *sc)
{
	struct grusbdc_dma_desc *desc;

	desc = &sc->desc_pool[GRUSBDC_EP0_DESC_OUT];
	desc->control = 0;
	desc->buffer = sc->desc_pool_bus_addr + GRUSBDC_EP0_SETUP_OFFSET;
	desc->next = 0;
	grusbdc_dma_sync(sc, BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
	/* GRUSBDC requires EN to be written after the descriptor fields. */
	desc->control = GRUSBDC_DESC_IE | GRUSBDC_DESC_EN;
	grusbdc_dma_sync(sc, BUS_DMASYNC_PREWRITE);
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE + 0x08,
	    sc->desc_pool_bus_addr + GRUSBDC_EP0_DESC_OUT *
	    sizeof(struct grusbdc_dma_desc));
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE + 0x04,
	    GRUSBDC_DMACTRL_IE | GRUSBDC_DMACTRL_AI | GRUSBDC_DMACTRL_DA);
	grusbdc_debug[74] = grusbdc_dma_read32(&desc->control);
	grusbdc_debug[75] = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE + 0x04);
	grusbdc_debug[76] = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE + 0x0c);
	grusbdc_debug[77] = 0;
}

/* One-packet bulk loopback used until the generic device xfer backend exists. */
static void
grusbdc_ep1_arm_out(struct grusbdc_softc *sc)
{
	struct grusbdc_dma_desc *desc;

	if (sc->ep1_out_armed ||
	    sc->ep1_queue_count == GRUSBDC_EP1_QUEUE_COUNT)
		return;
	desc = &sc->desc_pool[GRUSBDC_EP1_OUT_DESC];
	desc->control = 0;
	desc->buffer = sc->desc_pool_bus_addr + GRUSBDC_EP1_OUT_OFFSET;
	desc->next = 0;
	grusbdc_dma_sync(sc, BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
	desc->control = GRUSBDC_DESC_IE | GRUSBDC_DESC_EN;
	grusbdc_dma_sync(sc, BUS_DMASYNC_PREWRITE);
	bus_space_write_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE + GRUSBDC_EP_STRIDE + 0x08,
	    sc->desc_pool_bus_addr + GRUSBDC_EP1_OUT_DESC *
	    sizeof(struct grusbdc_dma_desc));
	bus_space_write_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE + GRUSBDC_EP_STRIDE + 0x04,
	    GRUSBDC_DMACTRL_IE | GRUSBDC_DMACTRL_AI | GRUSBDC_DMACTRL_DA);
	sc->ep1_out_armed = 1;
	grusbdc_debug[34] = grusbdc_dma_read32(&desc->control);
	grusbdc_debug[35] = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE + GRUSBDC_EP_STRIDE + 0x04);
}

static void
grusbdc_ep1_send(struct grusbdc_softc *sc, const uint8_t *data,
	uint16_t length)
{
	struct grusbdc_dma_desc *desc;
	uint8_t *buffer;

	buffer = (uint8_t *)sc->desc_pool + GRUSBDC_EP1_IN_OFFSET;
	if (length != 0)
		memcpy(buffer, data, length);
	desc = &sc->desc_pool[GRUSBDC_EP1_IN_DESC];
	desc->control = 0;
	desc->buffer = sc->desc_pool_bus_addr + GRUSBDC_EP1_IN_OFFSET;
	desc->next = 0;
	grusbdc_dma_sync(sc, BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
	desc->control = (length & GRUSBDC_DESC_LENGTH_MASK) |
	    GRUSBDC_DESC_IE | GRUSBDC_DESC_IN_PI | GRUSBDC_DESC_EN;
	grusbdc_dma_sync(sc, BUS_DMASYNC_PREWRITE);
	bus_space_write_4(sc->bst, sc->bsh,
	    GRUSBDC_IN_BASE + GRUSBDC_EP_STRIDE + 0x08,
	    sc->desc_pool_bus_addr + GRUSBDC_EP1_IN_DESC *
	    sizeof(struct grusbdc_dma_desc));
	bus_space_write_4(sc->bst, sc->bsh,
	    GRUSBDC_IN_BASE + GRUSBDC_EP_STRIDE + 0x04,
	    GRUSBDC_DMACTRL_IE | GRUSBDC_DMACTRL_AI | GRUSBDC_DMACTRL_DA);
	grusbdc_debug[37] = grusbdc_dma_read32(&desc->control);
	sc->ep1_in_active = 1;
}

static void
grusbdc_ep1_start_queued(struct grusbdc_softc *sc)
{
	if (!sc->ep1_in_active && sc->ep1_queue_count != 0)
		grusbdc_ep1_send(sc, sc->ep1_queue[sc->ep1_queue_head],
		    sc->ep1_queue_length[sc->ep1_queue_head]);
}

static void
grusbdc_ep0_send(struct grusbdc_softc *sc, const void *data,
	uint16_t length)
{
	struct grusbdc_dma_desc *desc;
	uint8_t *buffer;

	buffer = (uint8_t *)sc->desc_pool + GRUSBDC_EP0_REPLY_OFFSET;
	if (length != 0)
		memcpy(buffer, data, length);
	desc = &sc->desc_pool[GRUSBDC_EP0_DESC_IN];
	desc->control = 0;
	desc->buffer = sc->desc_pool_bus_addr + GRUSBDC_EP0_REPLY_OFFSET;
	desc->next = 0;
	grusbdc_dma_sync(sc, BUS_DMASYNC_PREWRITE | BUS_DMASYNC_PREREAD);
	/* Publish the payload and descriptor before making it DMA-active. */
	desc->control = (length & GRUSBDC_DESC_LENGTH_MASK) |
	    GRUSBDC_DESC_IE | GRUSBDC_DESC_IN_PI | GRUSBDC_DESC_EN;
	grusbdc_dma_sync(sc, BUS_DMASYNC_PREWRITE);
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_IN_BASE + 0x08,
	    sc->desc_pool_bus_addr + GRUSBDC_EP0_DESC_IN *
	    sizeof(struct grusbdc_dma_desc));
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_IN_BASE + 0x04,
	    GRUSBDC_DMACTRL_IE | GRUSBDC_DMACTRL_AI | GRUSBDC_DMACTRL_DA);
}

static void
grusbdc_ep0_stall(struct grusbdc_softc *sc)
{
	uint32_t value;

	value = bus_space_read_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE);
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE,
	    value | GRUSBDC_EPCTRL_CS);
	value = bus_space_read_4(sc->bst, sc->bsh, GRUSBDC_IN_BASE);
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_IN_BASE,
	    value | GRUSBDC_EPCTRL_CS);
	sc->ep0_stall_count++;
	grusbdc_ep0_arm_setup(sc);
}

static void
grusbdc_ep0_handle_setup(struct grusbdc_softc *sc,
	const struct usb_device_request *req)
{
	struct usb_device_request request;
	const void *reply;
	uint8_t small_reply[2];
	uint16_t length;
	uint16_t value;
	uint16_t requested;
	uint16_t ep1_packet;
	usb_error_t error;

	sc->ep0_setup_count++;
	value = UGETW(req->wValue);
	requested = UGETW(req->wLength);
	reply = NULL;
	length = 0;
	grusbdc_debug[20] = ((uint32_t)req->bmRequestType << 24) |
	    ((uint32_t)req->bRequest << 16) | value;
	grusbdc_debug[21] = requested;
	grusbdc_debug[22] = 0xffffffffu;
	grusbdc_debug[23] = 0;

	if (req->bmRequestType == UT_READ_DEVICE ||
	    req->bmRequestType == UT_WRITE_DEVICE) {
		switch (req->bRequest) {
		case UR_GET_DESCRIPTOR:
			if (req->bmRequestType != UT_READ_DEVICE)
				break;
			request = *req;
			error = (usb_temp_get_desc_p)(NULL, &request, &reply,
			    &length);
			grusbdc_debug[22] = error;
			grusbdc_debug[23] = length;
			if (error != 0 || reply == NULL)
				break;
			if (length > requested)
				length = requested;
			if (length > GRUSBDC_EP0_MAX_PACKET)
				length = GRUSBDC_EP0_MAX_PACKET;
			grusbdc_ep0_send(sc, reply, length);
			return;
		case UR_SET_ADDRESS:
			if (req->bmRequestType != UT_WRITE_DEVICE)
				break;
			if ((value & 0xff80) != 0 || requested != 0)
				break;
			value = bus_space_read_4(sc->bst, sc->bsh,
			    GRUSBDC_GLOBAL_CTRL) & ~GRUSBDC_CTRL_UA_MASK;
			value |= ((UGETW(req->wValue) << GRUSBDC_CTRL_UA_SHIFT) &
			    GRUSBDC_CTRL_UA_MASK) | GRUSBDC_CTRL_SU;
			grusbdc_debug[22] = value;
			bus_space_write_4(sc->bst, sc->bsh,
			    GRUSBDC_GLOBAL_CTRL, value);
			grusbdc_debug[23] = bus_space_read_4(sc->bst, sc->bsh,
			    GRUSBDC_GLOBAL_CTRL);
			grusbdc_ep0_send(sc, NULL, 0);
			return;
		case UR_SET_CONFIG:
			if (req->bmRequestType != UT_WRITE_DEVICE)
				break;
			if (requested != 0 || value > 1)
				break;
			if (value == 1) {
				ep1_packet = (bus_space_read_4(sc->bst, sc->bsh,
				    GRUSBDC_GLOBAL_STATUS) &
				    GRUSBDC_STATUS_FULL_SPEED) != 0 ? 64 : 512;
				if (grusbdc_configure_endpoint(sc, 1, UE_DIR_OUT,
				    UE_BULK, ep1_packet) != 0 ||
				    grusbdc_configure_endpoint(sc, 1, UE_DIR_IN,
				    UE_BULK, ep1_packet) != 0)
					break;
				sc->ep1_queue_head = 0;
				sc->ep1_queue_count = 0;
				sc->ep1_out_armed = 0;
				sc->ep1_in_active = 0;
				grusbdc_ep1_arm_out(sc);
			} else {
				bus_space_write_4(sc->bst, sc->bsh,
				    GRUSBDC_OUT_BASE + GRUSBDC_EP_STRIDE, 0);
				bus_space_write_4(sc->bst, sc->bsh,
				    GRUSBDC_IN_BASE + GRUSBDC_EP_STRIDE, 0);
				sc->ep_out[1].enabled = 0;
				sc->ep_in[1].enabled = 0;
				sc->ep1_queue_head = 0;
				sc->ep1_queue_count = 0;
				sc->ep1_out_armed = 0;
				sc->ep1_in_active = 0;
			}
			sc->ep0_configuration = value;
			grusbdc_ep0_send(sc, NULL, 0);
			return;
		case UR_GET_CONFIG:
			if (req->bmRequestType != UT_READ_DEVICE)
				break;
			if (requested == 0)
				break;
			small_reply[0] = sc->ep0_configuration;
			grusbdc_ep0_send(sc, small_reply,
			    MIN(requested, 1));
			return;
		case UR_GET_STATUS:
			if (req->bmRequestType != UT_READ_DEVICE)
				break;
			if (requested < sizeof(small_reply))
				break;
			small_reply[0] = 0;
			small_reply[1] = 0;
			grusbdc_ep0_send(sc, small_reply, sizeof(small_reply));
			return;
		default:
			break;
		}
	}
	grusbdc_ep0_stall(sc);
}

static void
grusbdc_ep0_init(struct grusbdc_softc *sc)
{
	uint32_t endpoint_control;

	/* Each of the two EP0 hardware buffers holds one 64-byte packet. */
	endpoint_control = (8u << GRUSBDC_EPCTRL_BUFSZ_SHIFT) |
	    (GRUSBDC_EP0_MAX_PACKET << GRUSBDC_EPCTRL_MAXPL_SHIFT) |
	    GRUSBDC_EPCTRL_PI | GRUSBDC_EPCTRL_EV;
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE,
	    endpoint_control);
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_IN_BASE,
	    endpoint_control);
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE + 0x04,
	    GRUSBDC_DMACTRL_IE | GRUSBDC_DMACTRL_AI);
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_IN_BASE + 0x04,
	    GRUSBDC_DMACTRL_IE | GRUSBDC_DMACTRL_AI);
	grusbdc_ep0_arm_setup(sc);
}

static void grusbdc_intr(void *arg);
static int grusbdc_detach(device_t dev);

static void
grusbdc_trace_reset(struct grusbdc_softc *sc,
    volatile uint32_t *trace)
{
	static const bus_size_t offsets[] = {
		GRUSBDC_GLOBAL_STATUS, GRUSBDC_OUT_BASE + 0x0c,
		GRUSBDC_OUT_BASE + 0x04, GRUSBDC_OUT_BASE + 0x08,
		GRUSBDC_IN_BASE + 0x0c, GRUSBDC_IN_BASE + 0x04
	};
	unsigned int i;

	for (i = 0; i < 4; ++i)
		trace[i] = bus_space_read_4(sc->bst, sc->bsh, offsets[i]);
	trace[4] = grusbdc_dma_read32(
	    &sc->desc_pool[GRUSBDC_EP0_DESC_OUT].control);
	for (; i < nitems(offsets); ++i)
		trace[i + 1] = bus_space_read_4(sc->bst, sc->bsh, offsets[i]);
}


static void
grusbdc_intr(void *arg)
{
	struct grusbdc_softc *sc;
	struct grusbdc_dma_desc *desc;
	struct usb_device_request request;
	uint32_t control;
	uint32_t endpoint_status;
	uint32_t status;
	uint32_t value;
	unsigned int wait_us;
	bool saved_setup = false;

	sc = arg;
	desc = &sc->desc_pool[GRUSBDC_EP0_DESC_OUT];
	/*
	 * IntMux lock registers latch pulse IRQs.  Clear these early in the ISR
	 * so another USB pulse during endpoint processing can be captured.
	 */
	bsp_grusbdc_interrupt_acknowledge();
	grusbdc_debug[8]++;
	grusbdc_debug[11] = leon3_get_cache_control_register();
	status = bus_space_read_4(sc->bst, sc->bsh, GRUSBDC_GLOBAL_STATUS);
	if ((status & GRUSBDC_STATUS_USB_RESET) != 0) {
		uint32_t seq = grusbdc_reset_trace_seq;
		unsigned int slot = seq % GRUSBDC_RESET_TRACE_COUNT;

		grusbdc_trace_reset(sc, grusbdc_reset_trace[slot][0]);
		/* UR is write-one-to-clear; the other status fields are read-only. */
		bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_GLOBAL_STATUS,
		    GRUSBDC_STATUS_USB_RESET);
		sc->reset_count++;
		sc->ep0_configuration = 0;
		control = bus_space_read_4(sc->bst, sc->bsh,
		    GRUSBDC_GLOBAL_CTRL);
		control &= ~GRUSBDC_CTRL_UA_MASK;
		bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_GLOBAL_CTRL,
		    control);
		/* A SETUP can reach DMA before the reset IRQ is serviced. */
		value = grusbdc_dma_read32(&desc->control);
		if ((value & (GRUSBDC_DESC_EN | GRUSBDC_DESC_SE |
		    GRUSBDC_DESC_LENGTH_MASK)) ==
		    (GRUSBDC_DESC_SE | sizeof(request))) {
			grusbdc_dma_copy_from(&request,
			    (volatile uint8_t *)sc->desc_pool +
			    GRUSBDC_EP0_SETUP_OFFSET, sizeof(request));
			saved_setup = true;
		}
		grusbdc_ep0_init(sc);
		grusbdc_trace_reset(sc, grusbdc_reset_trace[slot][1]);
		grusbdc_reset_trace_seq = seq + 1;
	}
	control = bus_space_read_4(sc->bst, sc->bsh, GRUSBDC_GLOBAL_CTRL);
	if (status & GRUSBDC_STATUS_VBUS_VALID)
		control |= GRUSBDC_CTRL_EP;
	else
		control &= ~GRUSBDC_CTRL_EP;
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_GLOBAL_CTRL, control);

	/* OUT EP0 may interrupt on packet arrival before DMA writes back status. */
	endpoint_status = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE + 0x0c);
	grusbdc_debug[24] = endpoint_status;
	/* Acknowledge packet arrival before waiting for the DMA writeback. */
	if ((endpoint_status & GRUSBDC_EPSTAT_PR) != 0)
		bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_OUT_BASE + 0x0c,
		    GRUSBDC_EPSTAT_PR);
	grusbdc_dma_sync(sc, BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
	value = grusbdc_dma_read32(&desc->control);
	wait_us = 0;
	if ((endpoint_status & (GRUSBDC_EPSTAT_B1 | GRUSBDC_EPSTAT_B0)) != 0) {
		while ((value & GRUSBDC_DESC_EN) != 0 && wait_us < 1000) {
			DELAY(1);
			++wait_us;
			grusbdc_dma_sync(sc,
			    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
			value = grusbdc_dma_read32(&desc->control);
		}
	}
	grusbdc_debug[32] = wait_us;
	grusbdc_debug[33] = value;
	grusbdc_debug[9] = value;
	grusbdc_debug[10] = grusbdc_dma_read32((uint8_t *)sc->desc_pool +
	    GRUSBDC_EP0_SETUP_OFFSET);
	if (saved_setup || (value & GRUSBDC_DESC_EN) == 0) {
		if (saved_setup || ((value & GRUSBDC_DESC_SE) != 0 &&
		    (value & GRUSBDC_DESC_LENGTH_MASK) == sizeof(request))) {
			if (!saved_setup)
				grusbdc_dma_copy_from(&request,
			    (volatile uint8_t *)sc->desc_pool +
				    GRUSBDC_EP0_SETUP_OFFSET, sizeof(request));
			grusbdc_debug[7]++;
			if (sc->ep0_xfer != NULL) {
				grusbdc_debug[72]++;
				if (request.bmRequestType == UT_WRITE_DEVICE &&
				    request.bRequest == UR_SET_ADDRESS) {
					sc->ep0_pending_address =
					    UGETW(request.wValue) & 0x7f;
					sc->ep0_set_address_pending = 1;
				}
				usbd_copy_in(sc->ep0_xfer->frbuffers, 0, &request,
				    sizeof(request));
				usbd_xfer_set_frame_len(sc->ep0_xfer, 0,
				    sizeof(request));
				sc->ep0_stage = GRUSBDC_EP0_WAIT_SETUP;
				USB_BUS_LOCK(&sc->bus);
				usbd_transfer_done(sc->ep0_xfer, 0);
				USB_BUS_UNLOCK(&sc->bus);
			} else {
				grusbdc_debug[73]++;
				grusbdc_ep0_arm_setup(sc);
				grusbdc_ep0_handle_setup(sc, &request);
			}
		} else if (sc->ep0_xfer != NULL &&
	    sc->ep0_stage == GRUSBDC_EP0_DATA_OUT) {
			uint16_t received = value & GRUSBDC_DESC_LENGTH_MASK;
			if (received > usbd_xfer_max_framelen(sc->ep0_xfer)) {
				grusbdc_ep0_stall(sc);
			} else {
				grusbdc_dma_copy_from(
				    usbd_xfer_get_frame_buffer(sc->ep0_xfer, 1),
				    (volatile uint8_t *)sc->desc_pool +
				    GRUSBDC_EP0_REPLY_OFFSET, received);
				usbd_xfer_set_frame_len(sc->ep0_xfer, 1, received);
				USB_BUS_LOCK(&sc->bus);
				usbd_transfer_done(sc->ep0_xfer, 0);
				USB_BUS_UNLOCK(&sc->bus);
			}
		} else if ((value & GRUSBDC_DESC_LENGTH_MASK) == 0) {
			if (sc->ep0_xfer != NULL &&
			    (sc->ep0_stage == GRUSBDC_EP0_DATA_OUT ||
			    sc->ep0_stage == GRUSBDC_EP0_STATUS_OUT)) {
				USB_BUS_LOCK(&sc->bus);
				usbd_transfer_done(sc->ep0_xfer, 0);
				USB_BUS_UNLOCK(&sc->bus);
			} else {
				/* Status stage of control read. */
				grusbdc_ep0_arm_setup(sc);
			}
		} else {
			grusbdc_ep0_stall(sc);
		}
	}
	endpoint_status = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_IN_BASE + 0x0c);
	if ((endpoint_status & GRUSBDC_EPSTAT_PT) != 0) {
		bus_space_write_4(sc->bst, sc->bsh,
		    GRUSBDC_IN_BASE + 0x0c, GRUSBDC_EPSTAT_PT);
		if (sc->ep0_xfer != NULL &&
		    (sc->ep0_stage == GRUSBDC_EP0_DATA_IN ||
		    sc->ep0_stage == GRUSBDC_EP0_STATUS_IN)) {
			USB_BUS_LOCK(&sc->bus);
			usbd_transfer_done(sc->ep0_xfer, 0);
			USB_BUS_UNLOCK(&sc->bus);
		}
	}

	endpoint_status = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_IN_BASE + GRUSBDC_EP_STRIDE + 0x0c);
	grusbdc_debug[39] = endpoint_status;
	if ((endpoint_status & GRUSBDC_EPSTAT_PT) != 0) {
		bus_space_write_4(sc->bst, sc->bsh,
		    GRUSBDC_IN_BASE + GRUSBDC_EP_STRIDE + 0x0c,
		    GRUSBDC_EPSTAT_PT);
		if (sc->ep1_in_active != 0) {
			sc->ep1_in_active = 0;
			sc->ep1_queue_head = (sc->ep1_queue_head + 1) %
			    GRUSBDC_EP1_QUEUE_COUNT;
			sc->ep1_queue_count--;
			sc->ep1_tx_packets++;
		}
		grusbdc_ep1_start_queued(sc);
		grusbdc_ep1_arm_out(sc);
	}

	/* Copy each received packet into the software queue before rearming DMA. */
	endpoint_status = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE + GRUSBDC_EP_STRIDE + 0x0c);
	grusbdc_debug[36] = endpoint_status;
	if ((endpoint_status & GRUSBDC_EPSTAT_PR) != 0)
		bus_space_write_4(sc->bst, sc->bsh,
		    GRUSBDC_OUT_BASE + GRUSBDC_EP_STRIDE + 0x0c,
		    GRUSBDC_EPSTAT_PR);
	desc = &sc->desc_pool[GRUSBDC_EP1_OUT_DESC];
	value = grusbdc_dma_read32(&desc->control);
	wait_us = 0;
	if (sc->ep1_out_armed &&
	    (endpoint_status & (GRUSBDC_EPSTAT_B1 | GRUSBDC_EPSTAT_B0)) != 0) {
		while ((value & GRUSBDC_DESC_EN) != 0 && wait_us < 1000) {
			DELAY(1);
			wait_us++;
			grusbdc_dma_sync(sc,
			    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
			value = grusbdc_dma_read32(&desc->control);
		}
	}
	grusbdc_debug[45] = wait_us;
	grusbdc_debug[38] = value;
	if (sc->ep0_configuration == 1 && sc->ep1_out_armed &&
	    (value & GRUSBDC_DESC_EN) == 0) {
		uint16_t received;
		uint8_t tail;

		sc->ep1_out_armed = 0;
		received = value & GRUSBDC_DESC_LENGTH_MASK;
		if (received <= GRUSBDC_EP1_BUFFER_SIZE &&
		    sc->ep1_queue_count < GRUSBDC_EP1_QUEUE_COUNT) {
			tail = (sc->ep1_queue_head + sc->ep1_queue_count) %
			    GRUSBDC_EP1_QUEUE_COUNT;
			grusbdc_dma_copy_from(sc->ep1_queue[tail],
			    (volatile uint8_t *)sc->desc_pool +
			    GRUSBDC_EP1_OUT_OFFSET, received);
			sc->ep1_queue_length[tail] = received;
			sc->ep1_queue_count++;
			sc->ep1_rx_packets++;
			grusbdc_ep1_start_queued(sc);
		}
		grusbdc_ep1_arm_out(sc);
	}
	grusbdc_debug[40] = sc->ep1_queue_count;
	grusbdc_debug[41] = sc->ep1_rx_packets;
	grusbdc_debug[42] = sc->ep1_tx_packets;
	grusbdc_debug[43] = sc->ep1_in_active;
	grusbdc_debug[44] = sc->ep1_out_armed;
	grusbdc_debug[48] = sc->ep1_queue_head;
	grusbdc_debug[49] = sc->ep1_queue_count;
	for (wait_us = 0; wait_us < GRUSBDC_EP1_QUEUE_COUNT; ++wait_us)
		grusbdc_debug[50 + wait_us] =
		    sc->ep1_queue_length[wait_us];

	/* IN packet transmitted interrupt is write-one-to-clear. */
	endpoint_status = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_IN_BASE + 0x0c);
	if ((endpoint_status & GRUSBDC_EPSTAT_PT) != 0)
		bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_IN_BASE + 0x0c,
		    GRUSBDC_EPSTAT_PT);
	grusbdc_debug[25] = grusbdc_dma_read32(&desc->control);
	grusbdc_debug[26] = desc->buffer;
	grusbdc_debug[27] = desc->next;
	grusbdc_debug[28] = grusbdc_dma_read32((uint8_t *)sc->desc_pool +
	    GRUSBDC_EP0_SETUP_OFFSET);
	grusbdc_debug[29] = grusbdc_dma_read32((uint8_t *)sc->desc_pool +
	    GRUSBDC_EP0_SETUP_OFFSET + 4);
	grusbdc_debug[30] = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE);
	grusbdc_debug[31] = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE + 0x04);
	grusbdc_debug[0] = sc->bsh;
	grusbdc_debug[1] = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE);
	grusbdc_debug[2] = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE + 0x04);
	grusbdc_debug[3] = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE + 0x08);
	grusbdc_debug[4] = status;
	grusbdc_debug[5] = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_GLOBAL_CTRL);
	grusbdc_debug[6] = bus_space_read_4(sc->bst, sc->bsh, 0);
	if ((sc->last_status ^ status) & GRUSBDC_STATUS_VBUS_VALID) {
		USB_BUS_LOCK(&sc->bus);
		sc->hub_port_change |= UPS_C_CONNECT_STATUS;
		USB_BUS_UNLOCK(&sc->bus);
		usb_needs_explore(&sc->bus, 0);
	}
	sc->last_status = status;
}

/* The root hub is visible only to libbsd. USB requests from the cable
 * continue to use the physical EP0 and the peripheral descriptors.
 */
static const struct usb_device_descriptor grusbdc_hub_device = {
	.bLength = sizeof(struct usb_device_descriptor),
	.bDescriptorType = UDESC_DEVICE,
	.bcdUSB = {0, 2},
	.bDeviceClass = UDCLASS_HUB,
	.bDeviceProtocol = UDPROTO_HSHUBSTT,
	.bMaxPacketSize = 64,
	.bNumConfigurations = 1,
};

static const struct {
	struct usb_config_descriptor config;
	struct usb_interface_descriptor iface;
	struct usb_endpoint_descriptor ep;
} __packed grusbdc_hub_config = {
	.config = {
		.bLength = sizeof(struct usb_config_descriptor),
		.bDescriptorType = UDESC_CONFIG,
		.wTotalLength = {sizeof(grusbdc_hub_config), 0},
		.bNumInterface = 1,
		.bConfigurationValue = 1,
		.bmAttributes = UC_BUS_POWERED | UC_SELF_POWERED,
	},
	.iface = {
		.bLength = sizeof(struct usb_interface_descriptor),
		.bDescriptorType = UDESC_INTERFACE,
		.bNumEndpoints = 1,
		.bInterfaceClass = UICLASS_HUB,
	},
	.ep = {
		.bLength = sizeof(struct usb_endpoint_descriptor),
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = UE_DIR_IN | 1,
		.bmAttributes = UE_INTERRUPT,
		.wMaxPacketSize = {1, 0},
		.bInterval = 12,
	},
};

static const struct usb_hub_descriptor_min grusbdc_hub_desc = {
	.bDescLength = sizeof(struct usb_hub_descriptor_min),
	.bDescriptorType = UDESC_HUB,
	.bNbrPorts = 1,
	.wHubCharacteristics = {UHD_PWR_NO_SWITCH | UHD_OC_INDIVIDUAL, 0},
};

static usb_error_t
grusbdc_roothub_exec(struct usb_device *udev, struct usb_device_request *req,
    const void **data, uint16_t *length)
{
	static const uint8_t device_status[] = {1, 0};
	static const uint8_t hub_status[4] = {0};
	struct grusbdc_softc *sc = grusbdc_bus_to_softc(udev->bus);
	uint16_t value = UGETW(req->wValue);
	uint16_t index = UGETW(req->wIndex);
	uint16_t port;
	uint32_t status;

	USB_BUS_LOCK_ASSERT(&sc->bus, MA_OWNED);
	*data = NULL;
	*length = 0;

	switch (req->bmRequestType) {
	case UT_READ_DEVICE:
		switch (req->bRequest) {
		case UR_GET_DESCRIPTOR:
			if ((value & 0xff) != 0)
				break;
			if ((value >> 8) == UDESC_DEVICE) {
				*data = &grusbdc_hub_device;
				*length = sizeof(grusbdc_hub_device);
				return (0);
			}
			if ((value >> 8) == UDESC_CONFIG) {
				*data = &grusbdc_hub_config;
				*length = sizeof(grusbdc_hub_config);
				return (0);
			}
			break;
		case UR_GET_CONFIG:
			*data = &sc->hub_config;
			*length = sizeof(sc->hub_config);
			return (0);
		case UR_GET_STATUS:
			*data = device_status;
			*length = sizeof(device_status);
			return (0);
		}
		break;
	case UT_WRITE_DEVICE:
		if (req->bRequest == UR_SET_ADDRESS && value < 128)
			return (0);
		if (req->bRequest == UR_SET_CONFIG && value <= 1) {
			sc->hub_config = value;
			return (0);
		}
		break;
	case UT_READ_CLASS_DEVICE:
		if (req->bRequest == UR_GET_DESCRIPTOR && value == (UDESC_HUB << 8)) {
			*data = &grusbdc_hub_desc;
			*length = sizeof(grusbdc_hub_desc);
			return (0);
		}
		if (req->bRequest == UR_GET_STATUS) {
			*data = hub_status;
			*length = sizeof(hub_status);
			return (0);
		}
		break;
	case UT_READ_CLASS_OTHER:
		if (index != 1 || req->bRequest != UR_GET_STATUS)
			break;
		status = bus_space_read_4(sc->bst, sc->bsh, GRUSBDC_GLOBAL_STATUS);
		port = UPS_PORT_MODE_DEVICE;
		if (sc->hub_port_power)
			port |= UPS_PORT_POWER;
		if (sc->hub_port_power && (status & GRUSBDC_STATUS_VBUS_VALID)) {
			port |= UPS_CURRENT_CONNECT_STATUS | UPS_PORT_ENABLED;
			if (!(status & GRUSBDC_STATUS_FULL_SPEED))
				port |= UPS_HIGH_SPEED;
		}
		USETW(sc->hub_port_status.wPortStatus, port);
		USETW(sc->hub_port_status.wPortChange, sc->hub_port_change);
		*data = &sc->hub_port_status;
		*length = sizeof(sc->hub_port_status);
		return (0);
	case UT_WRITE_CLASS_OTHER:
		if (index != 1)
			break;
		if (req->bRequest == UR_SET_FEATURE && value == UHF_PORT_POWER) {
			if (!sc->hub_port_power)
				sc->hub_port_change |= UPS_C_CONNECT_STATUS;
			sc->hub_port_power = true;
			return (0);
		}
		if (req->bRequest == UR_CLEAR_FEATURE) {
			switch (value) {
			case UHF_C_PORT_CONNECTION:
				sc->hub_port_change &= ~UPS_C_CONNECT_STATUS;
				return (0);
			case UHF_PORT_POWER:
				sc->hub_port_power = false;
				sc->hub_port_change |= UPS_C_CONNECT_STATUS;
				return (0);
			case UHF_PORT_ENABLE:
			case UHF_C_PORT_ENABLE:
			case UHF_C_PORT_RESET:
			case UHF_C_PORT_OVER_CURRENT:
			case UHF_C_PORT_SUSPEND:
				return (0);
			}
		}
		break;
	}
	return (USB_ERR_STALLED);
}

static int
grusbdc_probe(device_t dev)
{
	device_set_desc(dev, "Gaisler GRUSBDC USB device controller");
	return (BUS_PROBE_DEFAULT);
}

static int
grusbdc_attach(device_t dev)
{
	struct grusbdc_softc *sc;
	uint32_t status;
	unsigned int nepi;
	unsigned int nepo;
	int error;
	int rid;

	sc = device_get_softc(dev);
	sc->bus.parent = dev;
	sc->bus.devices = sc->devices;
	sc->bus.devices_max = nitems(sc->devices);
	sc->bus.dma_bits = 32;
	sc->bus.usbrev = USB_REV_2_0;
	sc->bus.methods = &grusbdc_bus_methods;
	grusbdc_init_endpoint_profiles(sc);
	rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "cannot allocate controller registers\n");
		return (ENXIO);
	}
	sc->bst = rman_get_bustag(sc->mem_res);
	sc->bsh = rman_get_bushandle(sc->mem_res);

	rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "cannot allocate IRQ %u\n", BSP_GRUSBDC_IRQ);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
		sc->mem_res = NULL;
		return (ENXIO);
	}

	bsp_grusbdc_initialize();
	status = bus_space_read_4(sc->bst, sc->bsh, GRUSBDC_STATUS);
	sc->last_status = status;
	nepi = ((status & GRUSBDC_STATUS_NEPI_MASK) >>
	    GRUSBDC_STATUS_NEPI_SHIFT) + 1;
	nepo = ((status & GRUSBDC_STATUS_NEPO_MASK) >>
	    GRUSBDC_STATUS_NEPO_SHIFT) + 1;
	if ((status & GRUSBDC_STATUS_DMA_MASTER) == 0) {
		device_printf(dev, "controller is not configured for master DMA\n");
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
		sc->irq_res = NULL;
		sc->mem_res = NULL;
		return (ENXIO);
	}
	if (grusbdc_dma_pool_init(dev, sc) != 0) {
		device_printf(dev, "cannot allocate 32-bit DMA descriptor pool\n");
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
		sc->irq_res = NULL;
		sc->mem_res = NULL;
		return (ENOMEM);
	}
	if (usb_bus_mem_alloc_all(&sc->bus, USB_GET_DMA_TAG(dev), NULL)) {
		device_printf(dev, "cannot allocate USB bus memory\n");
		grusbdc_dma_pool_uninit(sc);
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
		sc->irq_res = NULL;
		sc->mem_res = NULL;
		return (ENOMEM);
	}
	sc->bus.bdev = device_add_child(dev, "usbus", -1);
	if (sc->bus.bdev == NULL) {
		device_printf(dev, "cannot add usbus child\n");
		usb_bus_mem_free_all(&sc->bus, NULL);
		grusbdc_dma_pool_uninit(sc);
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
		sc->irq_res = NULL;
		sc->mem_res = NULL;
		return (ENOMEM);
	}
	device_set_ivars(sc->bus.bdev, &sc->bus);
	grusbdc_ep0_init(sc);
	grusbdc_debug[0] = sc->bsh;
	grusbdc_debug[1] = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE);
	grusbdc_debug[2] = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE + 0x04);
	grusbdc_debug[3] = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_OUT_BASE + 0x08);
	grusbdc_debug[4] = status;
	grusbdc_debug[5] = bus_space_read_4(sc->bst, sc->bsh,
	    GRUSBDC_GLOBAL_CTRL);
	grusbdc_debug[6] = bus_space_read_4(sc->bst, sc->bsh, 0);
	error = bus_setup_intr(dev, sc->irq_res, INTR_TYPE_BIO | INTR_MPSAFE,
	    NULL, grusbdc_intr, sc, &sc->intr_cookie);
	if (error != 0) {
		device_printf(dev, "cannot install USB interrupt handler (%d)\n",
		    error);
		device_delete_child(dev, sc->bus.bdev);
		sc->bus.bdev = NULL;
		usb_bus_mem_free_all(&sc->bus, NULL);
		grusbdc_dma_pool_uninit(sc);
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
		sc->irq_res = NULL;
		sc->mem_res = NULL;
		return (error);
	}
	/* Install the IRQMP handler before enabling the IntMux path. */
	bsp_grusbdc_interrupt_route_enable();
	/* Enable state-change events; connect only when VBUS is present. */
	bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_GLOBAL_CTRL,
	    GRUSBDC_CTRL_SI | GRUSBDC_CTRL_UI | GRUSBDC_CTRL_VI |
	    GRUSBDC_CTRL_SP);
	if ((status & GRUSBDC_STATUS_VBUS_VALID) != 0)
		bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_GLOBAL_CTRL,
		    GRUSBDC_CTRL_SI | GRUSBDC_CTRL_UI | GRUSBDC_CTRL_VI |
		    GRUSBDC_CTRL_SP | GRUSBDC_CTRL_EP);
	error = device_probe_and_attach(sc->bus.bdev);
	if (error != 0) {
		device_printf(dev, "cannot attach usbus child (%d)\n", error);
		grusbdc_detach(dev);
		return (error);
	}

	device_printf(dev, "master DMA mode, %u IN and %u OUT endpoints; "
	    "EP0 descriptor control enabled\n", nepi, nepo);
	return (0);
}

static int
grusbdc_detach(device_t dev)
{
	struct grusbdc_softc *sc;

	sc = device_get_softc(dev);
	if (sc->mem_res != NULL)
		bus_space_write_4(sc->bst, sc->bsh, GRUSBDC_GLOBAL_CTRL, 0);
	if (sc->intr_cookie != NULL) {
		bus_teardown_intr(dev, sc->irq_res, sc->intr_cookie);
		sc->intr_cookie = NULL;
	}
	if (sc->bus.bdev != NULL) {
		device_delete_child(dev, sc->bus.bdev);
		sc->bus.bdev = NULL;
	}
	if (sc->bus.parent != NULL)
		usb_bus_mem_free_all(&sc->bus, NULL);
	grusbdc_dma_pool_uninit(sc);
	if (sc->irq_res != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq_res);
	if (sc->mem_res != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem_res);
	return (0);
}

static device_method_t grusbdc_methods[] = {
	DEVMETHOD(device_probe, grusbdc_probe),
	DEVMETHOD(device_attach, grusbdc_attach),
	DEVMETHOD(device_detach, grusbdc_detach),
	DEVMETHOD_END
};

static driver_t grusbdc_driver = {
	"grusbdc",
	grusbdc_methods,
	sizeof(struct grusbdc_softc),
};

static devclass_t grusbdc_devclass;

DRIVER_MODULE(grusbdc, nexus, grusbdc_driver, grusbdc_devclass, 0, 0);

#endif /* BSP_HAS_GRUSBDC */
