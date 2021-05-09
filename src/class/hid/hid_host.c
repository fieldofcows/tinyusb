/* 
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"
#include <stdlib.h>

#if (TUSB_OPT_HOST_ENABLED && HOST_CLASS_HID)

#include "common/tusb_common.h"
#include "hid_host.h"
#include "hidparser/HIDParser.h"

CFG_TUSB_MEM_SECTION TU_ATTR_ALIGNED(4) static uint8_t report_buf[128];

static bool hidh_get_report_complete (uint8_t dev_addr, tusb_control_request_t const * request, xfer_result_t result);

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

typedef struct {
  uint8_t               dev_addr;
  HID_TYPE              hid_type;
  uint8_t               itf_num;
  uint8_t               ep_in;
  uint8_t               ep_out;
  bool                  valid;
  bool                  has_report_info;
  HID_ReportInfo_t      report_info;
  uint8_t               interface_num;
  tusb_desc_endpoint_t  endpoint_desc;
  uint8_t               rhport;
  uint16_t              report_size;
} hidh_interface_t;

static hidh_interface_t hid_device[CFG_TUSB_HOST_DEVICE_MAX];

static hidh_interface_t* hid_device_alloc(uint8_t dev_addr) {
  for (int i = 0; i < CFG_TUSB_HOST_DEVICE_MAX; ++i) {
    if (hid_device[i].dev_addr == 0) {
      hid_device[i].dev_addr = dev_addr;
      return &hid_device[i];
    }
  }
  return NULL;
}

static hidh_interface_t* hid_device_get(uint8_t dev_addr) {
  for (int i = 0; i < CFG_TUSB_HOST_DEVICE_MAX; ++i) {
    if (hid_device[i].dev_addr == dev_addr) {
      return &hid_device[i];
    }
  }
  return NULL;
}

static void hid_device_free(uint8_t dev_addr) {
  for (int i = 0; i < CFG_TUSB_HOST_DEVICE_MAX; ++i) {
    if (hid_device[i].dev_addr == dev_addr) {
      hid_device[i].dev_addr = 0;
      return;
    }
  }
}

//--------------------------------------------------------------------+
// HID Interface common functions
//--------------------------------------------------------------------+
static inline bool hidh_interface_open(uint8_t rhport, uint8_t dev_addr, uint8_t interface_number, tusb_desc_endpoint_t const *p_endpoint_desc, hidh_interface_t *p_hid)
{
  TU_ASSERT( usbh_edpt_open(rhport, dev_addr, p_endpoint_desc) );

  p_hid->ep_in       = p_endpoint_desc->bEndpointAddress;
  p_hid->report_size = p_endpoint_desc->wMaxPacketSize.size; // TODO get size from report descriptor
  p_hid->itf_num     = interface_number;
  p_hid->valid       = true;

  return true;
}

static inline void hidh_interface_close(hidh_interface_t *p_hid)
{
  tu_memclr(p_hid, sizeof(hidh_interface_t));
}

// called from public API need to validate parameters
tusb_error_t hidh_interface_get_report(uint8_t dev_addr, void * report, hidh_interface_t *p_hid)
{
  //------------- parameters validation -------------//
  // TODO change to use is configured function
  TU_ASSERT(TUSB_DEVICE_STATE_CONFIGURED == tuh_device_get_state(dev_addr), TUSB_ERROR_DEVICE_NOT_READY);
  TU_VERIFY(report, TUSB_ERROR_INVALID_PARA);
  TU_ASSERT(!hcd_edpt_busy(dev_addr, p_hid->ep_in), TUSB_ERROR_INTERFACE_IS_BUSY);

  TU_ASSERT( usbh_edpt_xfer(dev_addr, p_hid->ep_in, report, p_hid->report_size) ) ;

  return TUSB_ERROR_NONE;
}

//------------- KEYBOARD PUBLIC API (parameter validation required) -------------//
bool  tuh_hid_is_mounted(uint8_t dev_addr)
{
  hidh_interface_t* intf = hid_device_get(dev_addr);
  return (intf && tuh_device_is_configured(dev_addr) && (intf->ep_in != 0));
}

HID_TYPE tuh_hid_get_type(uint8_t dev_addr)
{
  hidh_interface_t* intf = hid_device_get(dev_addr);
  return intf ? intf->hid_type : HID_UNDEFINED;
}

tusb_error_t tuh_hid_get_report(uint8_t dev_addr, void* p_report)
{
  hidh_interface_t* intf = hid_device_get(dev_addr);
  if (!intf) {
    return TUSB_ERROR_INVALID_PARA;
  }
  return hidh_interface_get_report(dev_addr, p_report, intf);
}

bool tuh_hid_is_busy(uint8_t dev_addr)
{
  hidh_interface_t* intf = hid_device_get(dev_addr);
  return intf && tuh_hid_is_mounted(dev_addr) && hcd_edpt_busy(dev_addr, intf->ep_in);
}

uint16_t tuh_hid_get_report_size(uint8_t dev_addr) 
{
  hidh_interface_t* intf = hid_device_get(dev_addr);
  if (!intf) {
    return 0;
  }
  return intf->report_size;
}

HID_ReportInfo_t* tuh_hid_get_report_info(uint8_t dev_addr) 
{
  hidh_interface_t* intf = hid_device_get(dev_addr);
  if (!intf) {
    return NULL;
  }
  return &intf->report_info;
}

//--------------------------------------------------------------------+
// GENERIC
//--------------------------------------------------------------------+

static HID_TYPE filter_type = HID_UNDEFINED;

bool CALLBACK_HIDParser_FilterHIDReportItem(HID_ReportItem_t* const item)
{
  // Attempt to determine what type of device this is
  if (filter_type == HID_UNDEFINED) {
    // Iterate through the item's collection path, until either the root collection node or a collection with the
    // a supported usage is found
    for (HID_CollectionPath_t* path = item->CollectionPath; path != NULL; path = path->Parent)
    {
      if ((path->Usage.Page  == USAGE_PAGE_GENERIC_DCTRL) && (path->Usage.Usage == USAGE_JOYSTICK)) {
        filter_type = HID_JOYSTICK;
        break;
      }
      else if ((path->Usage.Page  == USAGE_PAGE_GENERIC_DCTRL) && (path->Usage.Usage == USAGE_MOUSE)) {
        filter_type = HID_MOUSE;
        break;
      }
    }
  }
  if (filter_type == HID_JOYSTICK) {
  	return ((item->Attributes.Usage.Page == USAGE_PAGE_BUTTON) ||
	        (item->Attributes.Usage.Page == USAGE_PAGE_GENERIC_DCTRL));
  }
  else if (filter_type == HID_MOUSE) {
  	return ((item->Attributes.Usage.Page == USAGE_PAGE_BUTTON) ||
	        (item->Attributes.Usage.Page == USAGE_PAGE_GENERIC_DCTRL));
  }
  return false;
}

//--------------------------------------------------------------------+
// CLASS-USBH API (don't require to verify parameters)
//--------------------------------------------------------------------+
void hidh_init(void)
{
  tu_memclr(&hid_device, sizeof(hid_device));
}

#if 0
CFG_TUSB_MEM_SECTION uint8_t report_descriptor[256];
#endif

static void hidh_set_protocol(uint8_t dev_addr, uint16_t protocol) {
  tusb_control_request_t const request =
  {
    .bmRequestType_bit =
    {
      .recipient = TUSB_REQ_RCPT_INTERFACE,
      .type      = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_OUT
    },
    .bRequest = HID_REQ_CONTROL_SET_PROTOCOL,
    .wValue   = protocol,
    .wIndex   = 0,
    .wLength  = 0
  };
  tuh_control_xfer(dev_addr, &request, NULL, NULL);
}

bool hidh_open_subtask(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *p_interface_desc, uint16_t *p_length)
{
  uint8_t const *p_desc = (uint8_t const *) p_interface_desc;

  //------------- HID descriptor -------------//
  p_desc += p_desc[DESC_OFFSET_LEN];
  tusb_hid_descriptor_hid_t const *p_desc_hid = (tusb_hid_descriptor_hid_t const *) p_desc;
  TU_ASSERT(HID_DESC_TYPE_HID == p_desc_hid->bDescriptorType, TUSB_ERROR_INVALID_PARA);

  //------------- Endpoint Descriptor -------------//
  p_desc += p_desc[DESC_OFFSET_LEN];
  tusb_desc_endpoint_t const * p_endpoint_desc = (tusb_desc_endpoint_t const *) p_desc;
  TU_ASSERT(TUSB_DESC_ENDPOINT == p_endpoint_desc->bDescriptorType, TUSB_ERROR_INVALID_PARA);

  if ((HID_SUBCLASS_BOOT == p_interface_desc->bInterfaceSubClass) &&
      ( HID_PROTOCOL_KEYBOARD == p_interface_desc->bInterfaceProtocol))
  {    
    #if CFG_TUH_HID_KEYBOARD
    // Make sure the boot protocol is selected.
    hidh_set_protocol(dev_addr, 0);

    hidh_interface_t* intf = hid_device_alloc(dev_addr);
    TU_ASSERT(intf);
    intf->hid_type = HID_KEYBOARD;
    TU_ASSERT( hidh_interface_open(rhport, dev_addr, p_interface_desc->bInterfaceNumber, p_endpoint_desc, intf) );
    TU_LOG2_HEX(intf->ep_in);
    return false;
    #endif
  }
  else {
    hidh_interface_t* intf = hid_device_alloc(dev_addr);
    TU_ASSERT(intf);
    intf->hid_type = HID_UNDEFINED;
    intf->rhport = rhport;
    intf->interface_num = p_interface_desc->bInterfaceNumber;
    memcpy(&intf->endpoint_desc, p_endpoint_desc, sizeof(tusb_desc_endpoint_t));

    // Get a report from the device so we can see what it is.
    tusb_control_request_t const request =
    {
      .bmRequestType_bit =
      {
        .recipient = TUSB_REQ_RCPT_INTERFACE,
        .type      = TUSB_REQ_TYPE_STANDARD,
        .direction = TUSB_DIR_IN
      },
      .bRequest = TUSB_REQ_GET_DESCRIPTOR,
      .wValue   = HID_DESC_TYPE_REPORT << 8,
      .wIndex   = 0,
      .wLength  = p_desc_hid->wReportLength
    };
    tuh_control_xfer(dev_addr, &request, report_buf, hidh_get_report_complete);
  }

  *p_length = sizeof(tusb_desc_interface_t) + sizeof(tusb_hid_descriptor_hid_t) + sizeof(tusb_desc_endpoint_t);

  return true;
}

bool hidh_get_report_complete (uint8_t dev_addr, tusb_control_request_t const * request, xfer_result_t result) {
  hidh_interface_t* intf = hid_device_get(dev_addr);
  TU_ASSERT(intf);

  // Parse the report data to see if it is a device we support.
  // Clear filter_type - the callback called by USB_ProcessHIDReport will determine the device type
  // for us
  filter_type = HID_UNDEFINED;
  if (USB_ProcessHIDReport(report_buf, request->wLength, &intf->report_info) == HID_PARSE_Successful) {
    intf->has_report_info = true;
    intf->hid_type = filter_type;

    if (intf->hid_type == HID_MOUSE) {
      // Set the report protocol rather than the boot protocol
      hidh_set_protocol(dev_addr, 1);
    }

    // The report is supported
    TU_ASSERT (hidh_interface_open(intf->rhport, dev_addr, intf->interface_num, &intf->endpoint_desc, intf));
    TU_LOG2_HEX(intf->ep_in);
  }
  else {
    // Unsupported HID device
    hidh_interface_close(intf);
  }
  return true;
}

bool hidh_set_config(uint8_t dev_addr, uint8_t itf_num)
{
#if 0
  //------------- Get Report Descriptor TODO HID parser -------------//
  if ( p_desc_hid->bNumDescriptors )
  {
    STASK_INVOKE(
        usbh_control_xfer_subtask( dev_addr, bm_request_type(TUSB_DIR_IN, TUSB_REQ_TYPE_STANDARD, TUSB_REQ_RCPT_INTERFACE),
                                   TUSB_REQ_GET_DESCRIPTOR, (p_desc_hid->bReportType << 8), 0,
                                   p_desc_hid->wReportLength, report_descriptor ),
        error
    );
    (void) error; // if error in getting report descriptor --> treating like there is none
  }
#endif

#if 0
  // SET IDLE = 0 request
  // Device can stall if not support this request
  tusb_control_request_t const request =
  {
    .bmRequestType_bit =
    {
      .recipient = TUSB_REQ_RCPT_INTERFACE,
      .type      = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_OUT
    },
    .bRequest = HID_REQ_CONTROL_SET_IDLE,
    .wValue   = 0, // idle_rate = 0
    .wIndex   = p_interface_desc->bInterfaceNumber,
    .wLength  = 0
  };

  // stall is a valid response for SET_IDLE, therefore we could ignore result of this request
  tuh_control_xfer(dev_addr, &request, NULL, NULL);
#endif

  usbh_driver_set_config_complete(dev_addr, itf_num);

  hidh_interface_t* intf = hid_device_get(dev_addr);
  if (intf && (intf->itf_num == itf_num) && intf->valid) {
    tuh_hid_mounted_cb(dev_addr);
  }

  return true;
}

bool hidh_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes)
{
  (void) xferred_bytes; // TODO may need to use this para later

  hidh_interface_t* intf = hid_device_get(dev_addr);
  if (intf && intf->ep_in == ep_addr) {
    tuh_hid_isr(dev_addr, event);
    return true;
  }
  return true;
}

void hidh_close(uint8_t dev_addr)
{
  hidh_interface_t* intf = hid_device_get(dev_addr);
  if (intf && intf->ep_in != 0) {
    tuh_hid_unmounted_cb(dev_addr);
    hidh_interface_close(intf);
  }
}



#endif
