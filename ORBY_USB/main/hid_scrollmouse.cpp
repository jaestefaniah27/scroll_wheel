#include "hid_scrollmouse.h"
#include <HID-Project.h>
#include <HID-Settings.h>

// Report descriptor: Mouse con 5 botones, XY y dos ruedas de 16-bit (V/H)
static const uint8_t REPORT_DESCRIPTOR[] PROGMEM = {
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x02,        // USAGE (Mouse)
    0xa1, 0x01,        // COLLECTION (Application)
    0x09, 0x02,        //   USAGE (Mouse)
    0xa1, 0x02,        //   COLLECTION (Logical)
    0x09, 0x01,        //     USAGE (Pointer)
    0xa1, 0x00,        //     COLLECTION (Physical)
    // ------------------------------  Buttons
    0x05, 0x09,        //       USAGE_PAGE (Button)
    0x19, 0x01,        //       USAGE_MINIMUM (Button 1)
    0x29, 0x05,        //       USAGE_MAXIMUM (Button 5)
    0x15, 0x00,        //       LOGICAL_MINIMUM (0)
    0x25, 0x01,        //       LOGICAL_MAXIMUM (1)
    0x75, 0x01,        //       REPORT_SIZE (1)
    0x95, 0x05,        //       REPORT_COUNT (5 Buttons)
    0x81, 0x02,        //       INPUT (Data,Var,Abs)
    // ------------------------------  Padding
    0x75, 0x03,        //       REPORT_SIZE (8-5buttons 3)
    0x95, 0x01,        //       REPORT_COUNT (1)
    0x81, 0x03,        //       INPUT (Cnst,Var,Abs)
    // ------------------------------  X,Y position
    0x05, 0x01,        //       USAGE_PAGE (Generic Desktop)
    0x09, 0x30,        //       USAGE (X)
    0x09, 0x31,        //       USAGE (Y)
    0x15, 0x81,        //       LOGICAL_MINIMUM (-127)
    0x25, 0x7f,        //       LOGICAL_MAXIMUM (127)
    0x75, 0x08,        //       REPORT_SIZE (8)
    0x95, 0x02,        //       REPORT_COUNT (2)
    0x81, 0x06,        //       INPUT (Data,Var,Rel)
    0xa1, 0x02,        //       COLLECTION (Logical)
    // ------------------------------  Vertical wheel res multiplier
    0x09, 0x48,        //         USAGE (Resolution Multiplier)
    0x15, 0x00,        //         LOGICAL_MINIMUM (0)
    0x25, 0x01,        //         LOGICAL_MAXIMUM (1)
    0x35, 0x01,        //         PHYSICAL_MINIMUM (1)
    0x45, 0x78,        //         PHYSICAL_MAXIMUM (120)
    0x75, 0x02,        //         REPORT_SIZE (2)
    0x95, 0x01,        //         REPORT_COUNT (1)
    0xa4,              //         PUSH
    0xb1, 0x02,        //         FEATURE (Data,Var,Abs)
    // ------------------------------  Vertical wheel
    0x09, 0x38,        //         USAGE (Wheel)
    0x16,0x01,0x80,    //         LOGICAL_MINIMUM (-32767)
    0x26,0xFF,0x7F,    //         LOGICAL_MAXIMUM (32767)
    0x35, 0x00,        //         PHYSICAL_MINIMUM (0)        - reset physical
    0x45, 0x00,        //         PHYSICAL_MAXIMUM (0)
    0x75, 0x10,        //         REPORT_SIZE (16)
    0x81, 0x06,        //         INPUT (Data,Var,Rel)
    0xc0,              //       END_COLLECTION
    0xa1, 0x02,        //       COLLECTION (Logical)
    // ------------------------------  Horizontal wheel res multiplier
    0x09, 0x48,        //         USAGE (Resolution Multiplier)
    0xb4,              //         POP
    0xb1, 0x02,        //         FEATURE (Data,Var,Abs)
    // ------------------------------  Padding for Feature report
    0x35, 0x00,        //         PHYSICAL_MINIMUM (0)        - reset physical
    0x45, 0x00,        //         PHYSICAL_MAXIMUM (0)
    0x75, 0x04,        //         REPORT_SIZE (4)
    0xb1, 0x03,        //         FEATURE (Cnst,Var,Abs)
    // ------------------------------  Horizontal wheel
    0x05, 0x0c,        //         USAGE_PAGE (Consumer Devices)
    0x0a, 0x38, 0x02,  //         USAGE (AC Pan)
    0x16,0x01,0x80,    //         LOGICAL_MINIMUM (-32767)
    0x26,0xFF,0x7F,    //         LOGICAL_MAXIMUM (32767)
    0x75, 0x10,        //         REPORT_SIZE (16)
    0x81, 0x06,        //         INPUT (Data,Var,Rel)
    0xc0,              //       END_COLLECTION
    0xc0,              //     END_COLLECTION
    0xc0,              //   END_COLLECTION
    0xc0               // END_COLLECTION
};

// ====== Implementación de CustomMouse ======

CustomMouse::CustomMouse() : PluggableUSBModule(1, 1, _epType) {
  _epType[0] = EP_TYPE_INTERRUPT_IN;
  PluggableUSB().plug(this);
}

int CustomMouse::getInterface(uint8_t* interfaceCount) {
  *interfaceCount += 1;
  const uint8_t desc[] PROGMEM = {
    // Interface
    0x09, 0x04, pluggedInterface, 0x00, 0x01, 0x03, 0x00, 0x02, 0x00,
    // HID
    0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22,
    sizeof(REPORT_DESCRIPTOR) & 0xFF, (sizeof(REPORT_DESCRIPTOR) >> 8) & 0xFF,
    // Endpoint IN interrupt
    0x07, 0x05, USB_ENDPOINT_IN(pluggedEndpoint),
    0x03, USB_EP_SIZE & 0xFF, (USB_EP_SIZE >> 8) & 0xFF, 0x0A
  };
  return USB_SendControl(0, desc, sizeof(desc));
}

int CustomMouse::getDescriptor(USBSetup& setup) {
  if (setup.bmRequestType == REQUEST_DEVICETOHOST_STANDARD_INTERFACE &&
      setup.wValueH == HID_REPORT_DESCRIPTOR_TYPE) {
    return USB_SendControl(TRANSFER_PGM, REPORT_DESCRIPTOR, sizeof(REPORT_DESCRIPTOR));
  }
  return 0;
}

bool CustomMouse::setup(USBSetup& setup) {
  if (setup.wIndex != pluggedInterface) return false;

  const uint8_t bm = setup.bmRequestType;
  const uint8_t rq = setup.bRequest;
  const uint8_t rt = setup.wValueH;

  // GET_FEATURE
  if (bm == 0xA1 && rq == HID_GET_REPORT && rt == HID_REPORT_TYPE_FEATURE) {
    USB_SendControl(0, &_resolutionMultiplier, 1);
    return true;
  }
  // SET_FEATURE
  if (bm == 0x21 && rq == HID_SET_REPORT && rt == HID_REPORT_TYPE_FEATURE) {
    USB_RecvControl(&_resolutionMultiplier, 1);
    return true;
  }
  return false;
}

void CustomMouse::sendReport(uint8_t buttons, int8_t x, int8_t y, int16_t wheelV, int16_t wheelH) {
  uint8_t buf[7];
  buf[0] = buttons & 0x1F;
  buf[1] = (uint8_t)x;
  buf[2] = (uint8_t)y;
  buf[3] = wheelV & 0xFF;
  buf[4] = (wheelV >> 8) & 0xFF;
  buf[5] = wheelH & 0xFF;
  buf[6] = (wheelH >> 8) & 0xFF;
  USB_Send(pluggedEndpoint | TRANSFER_RELEASE, buf, sizeof(buf));
}

// Instancia global
CustomMouse ScrollWheel;

void initHID() {
  Keyboard.begin();
  Consumer.begin();
}
