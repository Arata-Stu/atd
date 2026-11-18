/*
 * Descriptor layout follows TinyUSB's dual CDC example (MIT).
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <string.h>

#include "pico/unique_id.h"
#include "tusb.h"

/* TinyUSB example development VID/PID, not a production USB allocation. */
#define JPBB_USB_VID 0xCAFE
#define JPBB_USB_PID 0x4002

static const tusb_desc_device_t device_descriptor = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = JPBB_USB_VID,
    .idProduct = JPBB_USB_PID,
    .bcdDevice = 0x0100,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

enum {
  ITF_CONTROL = 0,
  ITF_CONTROL_DATA,
  ITF_SENSORS,
  ITF_SENSORS_DATA,
  ITF_COUNT,
};
#define JPBB_CONFIG_LEN (TUD_CONFIG_DESC_LEN + 2 * TUD_CDC_DESC_LEN)

static const uint8_t configuration_descriptor[] = {
    /* USB supply powers Pico + the optional 3.3 V/10 mA IMU, not the BEC rail. */
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, JPBB_CONFIG_LEN, 0, 100),
    TUD_CDC_DESCRIPTOR(ITF_CONTROL, 4, 0x81, 8, 0x02, 0x82, 64),
    TUD_CDC_DESCRIPTOR(ITF_SENSORS, 5, 0x83, 8, 0x04, 0x84, 64),
};

uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *)&device_descriptor;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  return index == 0 ? configuration_descriptor : NULL;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  static uint16_t descriptor[33];
  static char serial[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
  static const char *const strings[] = {
      NULL, "JPBB", "JPBB Rev6 Pico", NULL,
      "JPBB Control JPB1", "JPBB Sensors JPX1",
  };
  (void)langid;

  if (index == 0) {
    descriptor[0] = (TUSB_DESC_STRING << 8) | 4;
    descriptor[1] = 0x0409;
    return descriptor;
  }
  if (index >= sizeof(strings) / sizeof(strings[0])) {
    return NULL;
  }
  const char *text = strings[index];
  if (index == 3) {
    pico_get_unique_board_id_string(serial, sizeof(serial));
    text = serial;
  }
  size_t length = strlen(text);
  if (length > 32) {
    length = 32;
  }
  for (size_t i = 0; i < length; ++i) {
    descriptor[i + 1] = (uint8_t)text[i];
  }
  descriptor[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * length + 2));
  return descriptor;
}
