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

/** \addtogroup ClassDriver_HID
 *  @{ */

#ifndef _TUSB_HID_HOST_H_
#define _TUSB_HID_HOST_H_

#include "common/tusb_common.h"
#include "host/usbh.h"
#include "hid.h"
#include "hidparser/HIDParser.h"

/** HID Report Descriptor Usage Page value for a Generic Desktop Control. */
#define USAGE_PAGE_GENERIC_DCTRL    0x01
#define USAGE_JOYSTICK              0x04
#define USAGE_X                     0x30
#define USAGE_Y                     0x31
#define USAGE_PAGE_BUTTON           0x09

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// HID Application API
//--------------------------------------------------------------------+
/** \addtogroup ClassDriver_HID HID
 *  @{ */

/** \defgroup HID_Host Host
 *  The interface API includes status checking function, data transferring function and callback functions
 *  @{ */

extern uint8_t const hid_keycode_to_ascii_tbl[2][128]; // TODO used weak attr if build failed without KEYBOARD enabled

typedef enum {
  HID_UNDEFINED,
  HID_KEYBOARD,
  HID_MOUSE,
  HID_GENERIC
} HID_TYPE;

/** \brief      Check if the device is a HID device
 * \param[in]   dev_addr    device address
 * \retval      true if device supports HID interface
 * \retval      false if device does not support HID interface or is not mounted
 */
bool tuh_hid_is_mounted(uint8_t dev_addr);

/** \brief      Get the type of the HID device if known
 * \param[in]   dev_addr    device address
 * \retval      Device type
 */
HID_TYPE tuh_hid_get_type(uint8_t dev_addr);

/** \brief      Check if the interface is currently busy or not
 * \param[in]   dev_addr device address
 * \retval      true if the interface is busy meaning the stack is still transferring/waiting data from/to device
 * \retval      false if the interface is not busy meaning the stack successfully transferred data from/to device
 * \note        This function is primarily used for polling/waiting result after \ref tuh_hid_keyboard_get_report.
 *              Alternatively, asynchronous event API can be used
 */
bool tuh_hid_is_busy(uint8_t dev_addr);

/** \brief        Perform a get report from HID interface
 * \param[in]		  dev_addr device address
 * \param[in,out] p_report address that is used to store data from device. Must be accessible by usb controller (see \ref CFG_TUSB_MEM_SECTION)
 * \returns       \ref tusb_error_t type to indicate success or error condition.
 * \retval        TUSB_ERROR_NONE on success
 * \retval        TUSB_ERROR_INTERFACE_IS_BUSY if the interface is already transferring data with device
 * \retval        TUSB_ERROR_DEVICE_NOT_READY if device is not yet configured (by SET CONFIGURED request)
 * \retval        TUSB_ERROR_INVALID_PARA if input parameters are not correct
 * \note          This function is non-blocking and returns immediately. The result of usb transfer will be reported by the interface's callback function
 */
tusb_error_t  tuh_hid_get_report(uint8_t dev_addr, void * p_report);

/** \brief        Perform the size of the HID report in bytes
 * \param[in]		  dev_addr device address
 * \returns       The size in bytes
 */
uint16_t tuh_hid_get_report_size(uint8_t dev_addr);

/** \brief        Perform the report info structure used for parsing a report
 * \param[in]		  dev_addr device address
 * \returns       The report info structure.
 */
HID_ReportInfo_t* tuh_hid_get_report_info(uint8_t dev_addr);

//------------- Application Callback -------------//
/** \brief      Callback function that is invoked when an transferring event occurred
 * \param[in]		dev_addr	Address of device
 * \param[in]   event an value from \ref xfer_result_t
 * \note        event can be one of following
 *              - XFER_RESULT_SUCCESS : previously scheduled transfer completes successfully.
 *              - XFER_RESULT_FAILED   : previously scheduled transfer encountered a transaction error.
 *              - XFER_RESULT_STALLED : previously scheduled transfer is stalled by device.
 * \note        Application should schedule the next report by calling \ref tuh_hid_get_report within this callback
 */
void tuh_hid_isr(uint8_t dev_addr, xfer_result_t event);

/** \brief 			Callback function that will be invoked when a device with HID interface is mounted
 * \param[in] 	dev_addr Address of newly mounted device
 * \note        This callback should be used by Application to set-up interface-related data
 */
void tuh_hid_mounted_cb(uint8_t dev_addr);

/** \brief 			Callback function that will be invoked when a device with HID interface is unmounted
 * \param[in] 	dev_addr Address of newly unmounted device
 * \note        This callback should be used by Application to tear-down interface-related data
 */
void tuh_hid_unmounted_cb(uint8_t dev_addr);

/** @} */ // HID_Host
/** @} */ // ClassDriver_HID

//--------------------------------------------------------------------+
// Internal Class Driver API
//--------------------------------------------------------------------+
void hidh_init(void);
bool hidh_open_subtask(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *p_interface_desc, uint16_t *p_length);
bool hidh_set_config(uint8_t dev_addr, uint8_t itf_num);
bool hidh_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes);
void hidh_close(uint8_t dev_addr);

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_HID_HOST_H_ */

/** @} */ // ClassDriver_HID
