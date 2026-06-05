/*
 * usb_descriptors_tinyusb.c
 *
 * Example USB descriptors for a standalone FFB device on TinyUSB.
 *
 * IMPORTANT: the FFB library only ships the HID *report* descriptor
 * (the ~1.2 KB blob that describes the force-feedback reports). It does
 * NOT ship the USB *device*, *configuration*, or *string* descriptors -
 * those describe your whole USB device and are project-specific (VID/PID,
 * endpoints, how many interfaces, product name...). You must provide them.
 *
 * This file is the missing piece: a minimal HID-only configuration that
 * exposes exactly one FFB HID interface. Drop it into a TinyUSB project,
 * set your VID/PID/strings, and wire the report callbacks from
 * tinyusb_glue.cpp.
 *
 * Requires the real TinyUSB headers (tusb.h) at build time.
 */

#include <string.h>      /* memcpy, strlen */

#include "tusb.h"
#include "ffb/ffb_c.h"   /* for ffb_descriptor_1axis() */

/*--------------------------------------------------------------------+
 * 1. Device descriptor
 *--------------------------------------------------------------------*/

#define USB_VID   0x1209    /* pid.codes community VID - use your own!  */
#define USB_PID   0xFFB0    /* OpenFFBoard's PID; pick your own product */

static tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,   /* defined per-interface (HID)        */
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const* tud_descriptor_device_cb(void) {
    return (uint8_t const*) &desc_device;
}

/*--------------------------------------------------------------------+
 * 2. HID report descriptor  (THIS is what the library provides)
 *--------------------------------------------------------------------+
 * NOTE: tinyusb_glue.cpp also defines tud_hid_descriptor_report_cb().
 * Keep only ONE definition in your project (this one or that one).
 */

uint8_t const* tud_hid_descriptor_report_cb(uint8_t instance) {
    (void) instance;
    uint16_t len;
    return ffb_descriptor_1axis(&len);   /* or ffb_descriptor_2axis() */
}

/*--------------------------------------------------------------------+
 * 3. Configuration descriptor
 *--------------------------------------------------------------------*/

enum { ITF_NUM_HID = 0, ITF_NUM_TOTAL };

/* FFB needs BOTH an IN endpoint (status reports to the host) and an OUT
 * endpoint (effect reports from the host), so we MUST use the IN+OUT HID
 * template - the plain IN-only TUD_HID_DESCRIPTOR will not work. */
#define EPNUM_HID_OUT   0x01    /* host -> device (effect data)         */
#define EPNUM_HID_IN    0x81    /* device -> host (PID state reports)   */

/* The HID report descriptor length MUST equal what the library returns:
 *   ffb_descriptor_1axis() -> 1196 bytes
 *   ffb_descriptor_2axis() -> 1215 bytes
 * Keep this in sync with the descriptor you return above. */
#define FFB_HID_REPORT_DESC_LEN   1196

#define CONFIG_TOTAL_LEN   (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)

static uint8_t const desc_configuration[] = {
    /* Config number, interface count, string index, total length,
     * attributes, max power (mA). */
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    /* HID IN+OUT interface:
     * itf number, string index, boot protocol, report-desc length,
     * EP OUT addr, EP IN addr, EP size, polling interval (ms).
     * interval 1 ms => 1000 Hz, the FFB default. */
    TUD_HID_INOUT_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE,
                             FFB_HID_REPORT_DESC_LEN,
                             EPNUM_HID_OUT, EPNUM_HID_IN,
                             CFG_TUD_HID_EP_BUFSIZE, 1),
};

uint8_t const* tud_descriptor_configuration_cb(uint8_t index) {
    (void) index;
    return desc_configuration;
}

/*--------------------------------------------------------------------+
 * 4. String descriptors
 *--------------------------------------------------------------------*/

static char const* string_desc_arr[] = {
    (const char[]){ 0x09, 0x04 },   /* 0: language = English (0x0409)   */
    "Open FFBoard",                 /* 1: Manufacturer                  */
    "FFB Wheel",                    /* 2: Product                       */
    "000001",                       /* 3: Serial - ideally from chip ID */
};

static uint16_t _desc_str[32];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;
    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
            return NULL;
        }
        const char* str = string_desc_arr[index];
        chr_count = (uint8_t) strlen(str);
        if (chr_count > 31) chr_count = 31;
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    /* first byte = total length (incl. header), second = string type */
    _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return _desc_str;
}

/*--------------------------------------------------------------------+
 * Appendix: the ORIGINAL OpenFFBoard configuration
 *--------------------------------------------------------------------+
 *
 * OpenFFBoard ships a *composite* CDC + HID device (the CDC channel is its
 * configurator serial port, which this standalone library drops). For
 * reference, its config descriptor is built from this macro
 * (UserExtensions/Inc/usb_descriptors.h):
 *
 *   #define USB_CONF_DESC_HID_CDC(HIDREPSIZE, EPSIZE)                       \
 *     TUD_CONFIG_DESCRIPTOR(1, 3, 0,                                        \
 *         (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_HID_INOUT_DESC_LEN),\
 *         TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP |                             \
 *             TUSB_DESC_CONFIG_ATT_SELF_POWERED, 100),                     \
 *     TUD_CDC_DESCRIPTOR(0, 4, 0x82, 8, 0x01, 0x81, EPSIZE),               \
 *     TUD_HID_INOUT_DESCRIPTOR(2, 5, HID_ITF_PROTOCOL_NONE, HIDREPSIZE,    \
 *         0x83, 0x02, EPSIZE, HID_BINTERVAL)
 *
 * Instantiated as:
 *   const uint8_t usb_cdc_hid_conf_1axis[] =
 *       { USB_CONF_DESC_HID_CDC(USB_HID_1FFB_REPORT_DESC_SIZE, 64) };
 *
 * Note it has THREE interfaces (2 for CDC + 1 for HID) and the HID lives on
 * interface 2 with IN EP 0x83 / OUT EP 0x02. If you keep a CDC channel of
 * your own, model it on this; for a pure FFB device the HID-only config
 * above is all you need.
 */
