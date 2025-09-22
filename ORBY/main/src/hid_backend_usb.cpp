#include "hid_adapter.h"

#ifdef HID_BACKEND_USB

#include <Arduino.h>
#include <PluggableUSB.h>
#include <HID.h>
#include <HID-Project.h>
#include <HID-Settings.h>
#include "config.h"
#include "haptics.h"

#ifndef USB_EP_SIZE
#define USB_EP_SIZE 64
#endif

// Report descriptor: Mouse con 5 botones, XY y dos ruedas de 16-bit (V/H)
static const uint8_t MOUSE_REPORT_DESC[] PROGMEM = {
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x02, // USAGE (Mouse)
    0xa1, 0x01, // COLLECTION (Application)
    0x09, 0x02, //   USAGE (Mouse)
    0xa1, 0x02, //   COLLECTION (Logical)
    0x09, 0x01, //     USAGE (Pointer)
    0xa1, 0x00, //     COLLECTION (Physical)
    // ------------------------------  Buttons
    0x05, 0x09, //       USAGE_PAGE (Button)
    0x19, 0x01, //       USAGE_MINIMUM (Button 1)
    0x29, 0x05, //       USAGE_MAXIMUM (Button 5)
    0x15, 0x00, //       LOGICAL_MINIMUM (0)
    0x25, 0x01, //       LOGICAL_MAXIMUM (1)
    0x75, 0x01, //       REPORT_SIZE (1)
    0x95, 0x05, //       REPORT_COUNT (5 Buttons)
    0x81, 0x02, //       INPUT (Data,Var,Abs)
    // ------------------------------  Padding
    0x75, 0x03, //       REPORT_SIZE (8-5buttons 3)
    0x95, 0x01, //       REPORT_COUNT (1)
    0x81, 0x03, //       INPUT (Cnst,Var,Abs)
    // ------------------------------  X,Y position
    0x05, 0x01, //       USAGE_PAGE (Generic Desktop)
    0x09, 0x30, //       USAGE (X)
    0x09, 0x31, //       USAGE (Y)
    0x15, 0x81, //       LOGICAL_MINIMUM (-127)
    0x25, 0x7f, //       LOGICAL_MAXIMUM (127)
    0x75, 0x08, //       REPORT_SIZE (8)
    0x95, 0x02, //       REPORT_COUNT (2)
    0x81, 0x06, //       INPUT (Data,Var,Rel)
    0xa1, 0x02, //       COLLECTION (Logical)
    // ------------------------------  Vertical wheel res multiplier
    0x09, 0x48, //         USAGE (Resolution Multiplier)
    0x15, 0x00, //         LOGICAL_MINIMUM (0)
    0x25, 0x01, //         LOGICAL_MAXIMUM (1)
    0x35, 0x01, //         PHYSICAL_MINIMUM (1)
    0x45, 0x78, //         PHYSICAL_MAXIMUM (120)
    0x75, 0x02, //         REPORT_SIZE (2)
    0x95, 0x01, //         REPORT_COUNT (1)
    0xa4,       //         PUSH
    0xb1, 0x02, //         FEATURE (Data,Var,Abs)
    // ------------------------------  Vertical wheel
    0x09, 0x38,       //         USAGE (Wheel)
    0x16, 0x01, 0x80, //         LOGICAL_MINIMUM (-32767)
    0x26, 0xFF, 0x7F, //         LOGICAL_MAXIMUM (32767)
    0x35, 0x00,       //         PHYSICAL_MINIMUM (0)        - reset physical
    0x45, 0x00,       //         PHYSICAL_MAXIMUM (0)
    0x75, 0x10,       //         REPORT_SIZE (16)
    0x81, 0x06,       //         INPUT (Data,Var,Rel)
    0xc0,             //       END_COLLECTION
    0xa1, 0x02,       //       COLLECTION (Logical)
    // ------------------------------  Horizontal wheel res multiplier
    0x09, 0x48, //         USAGE (Resolution Multiplier)
    0xb4,       //         POP
    0xb1, 0x02, //         FEATURE (Data,Var,Abs)
    // ------------------------------  Padding for Feature report
    0x35, 0x00, //         PHYSICAL_MINIMUM (0)        - reset physical
    0x45, 0x00, //         PHYSICAL_MAXIMUM (0)
    0x75, 0x04, //         REPORT_SIZE (4)
    0xb1, 0x03, //         FEATURE (Cnst,Var,Abs)
    // ------------------------------  Horizontal wheel
    0x05, 0x0c,       //         USAGE_PAGE (Consumer Devices)
    0x0a, 0x38, 0x02, //         USAGE (AC Pan)
    0x16, 0x01, 0x80, //         LOGICAL_MINIMUM (-32767)
    0x26, 0xFF, 0x7F, //         LOGICAL_MAXIMUM (32767)
    0x75, 0x10,       //         REPORT_SIZE (16)
    0x81, 0x06,       //         INPUT (Data,Var,Rel)
    0xc0,             //       END_COLLECTION
    0xc0,             //     END_COLLECTION
    0xc0,             //   END_COLLECTION
    0xc0              // END_COLLECTION
};

// --------- Módulo USB de ratón personalizado ---------
class CustomMouse : public PluggableUSBModule
{
public:
  CustomMouse() : PluggableUSBModule(1, 1, _epType)
  {
    _epType[0] = EP_TYPE_INTERRUPT_IN;
    PluggableUSB().plug(this);
  }

  int getInterface(uint8_t *interfaceCount) override
  {
    *interfaceCount += 1;
    const uint8_t desc[] PROGMEM = {
        // Interface
        0x09, 0x04, pluggedInterface, 0x00, 0x01, 0x03, 0x00, 0x02, 0x00,
        // HID
        0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22,
        sizeof(MOUSE_REPORT_DESC) & 0xFF, (sizeof(MOUSE_REPORT_DESC) >> 8) & 0xFF,
        // Endpoint IN interrupt
        0x07, 0x05, USB_ENDPOINT_IN(pluggedEndpoint),
        0x03, USB_EP_SIZE & 0xFF, (USB_EP_SIZE >> 8) & 0xFF, 0x0A};
    return USB_SendControl(0, desc, sizeof(desc));
  }

  int getDescriptor(USBSetup &setup) override
  {
    if (setup.bmRequestType == REQUEST_DEVICETOHOST_STANDARD_INTERFACE &&
        setup.wValueH == HID_REPORT_DESCRIPTOR_TYPE)
      return USB_SendControl(TRANSFER_PGM, MOUSE_REPORT_DESC, sizeof(MOUSE_REPORT_DESC));
    return 0;
  }

  bool setup(USBSetup &setup) override
  {
    if (setup.wIndex != pluggedInterface)
      return false;

    const uint8_t bm = setup.bmRequestType;
    const uint8_t rq = setup.bRequest;
    const uint8_t rt = setup.wValueH;

    // GET_FEATURE (Resolution Multiplier)
    if (bm == 0xA1 && rq == HID_GET_REPORT && rt == HID_REPORT_TYPE_FEATURE)
    {
      USB_SendControl(0, &_resMult, 1);
      return true;
    }
    // SET_FEATURE
    if (bm == 0x21 && rq == HID_SET_REPORT && rt == HID_REPORT_TYPE_FEATURE)
    {
      USB_RecvControl(&_resMult, 1);
      return true;
    }
    return false;
  }

  void sendReport(uint8_t buttons, int8_t x, int8_t y, int16_t wheelV, int16_t wheelH)
  {
    uint8_t buf[7];
    buf[0] = buttons & 0x1F;
    buf[1] = (uint8_t)x;
    buf[2] = (uint8_t)y;
    buf[3] = (uint8_t)(wheelV & 0xFF);
    buf[4] = (uint8_t)((wheelV >> 8) & 0xFF);
    buf[5] = (uint8_t)(wheelH & 0xFF);
    buf[6] = (uint8_t)((wheelH >> 8) & 0xFF);
    USB_Send(pluggedEndpoint | TRANSFER_RELEASE, buf, sizeof(buf));
  }

private:
  uint8_t _epType[1];
  uint8_t _resMult = 1;
};

static CustomMouse gMouse;

// --------- Backend USB: setup / mouse / consumer / keyboard ---------

void hidSetup()
{
  // Inicia HID-Project (crea interfaces para Keyboard + Consumer)
  Keyboard.begin();
  Consumer.begin();
  delay(50);
}

// Ratón
void hidMouseSend(uint8_t buttons, int8_t x, int8_t y, int16_t wheelV, int16_t wheelH)
{
  gMouse.sendReport(buttons, x, y, wheelV, wheelH);
}

// Consumer Control (mapeo a HID-Project)
static uint16_t mapUsageToConsumerKey(uint16_t usage)
{
  switch (usage)
  {
  case CC_VOL_UP:
    return MEDIA_VOLUME_UP;
  case CC_VOL_DOWN:
    return MEDIA_VOLUME_DOWN;
  case CC_MUTE:
    return MEDIA_VOLUME_MUTE;
  default:
    return 0;
  }
}
void hidConsumerPress(uint16_t usage)
{
  uint16_t k = mapUsageToConsumerKey(usage);
  if (k)
    Consumer.press(k);
}
void hidConsumerRelease(void)
{
  Consumer.releaseAll();
}
void hidConsumerClick(uint16_t usage)
{
  uint16_t k = mapUsageToConsumerKey(usage);
  if (k)
    Consumer.write(k);
}

// Teclado (HID-Project)
static void pressMods(uint8_t m)
{
  if (m & KEY_MOD_LCTRL)
    Keyboard.press(KEY_LEFT_CTRL);
  if (m & KEY_MOD_LSHIFT)
    Keyboard.press(KEY_LEFT_SHIFT);
  if (m & KEY_MOD_LALT)
    Keyboard.press(KEY_LEFT_ALT);
  if (m & KEY_MOD_LGUI)
    Keyboard.press(KEY_LEFT_GUI);
  if (m & KEY_MOD_RCTRL)
    Keyboard.press(KEY_RIGHT_CTRL);
  if (m & KEY_MOD_RSHIFT)
    Keyboard.press(KEY_RIGHT_SHIFT);
  if (m & KEY_MOD_RALT)
    Keyboard.press(KEY_RIGHT_ALT);
  if (m & KEY_MOD_RGUI)
    Keyboard.press(KEY_RIGHT_GUI);
}
static void releaseMods(uint8_t m)
{
  if (m & KEY_MOD_LCTRL)
    Keyboard.release(KEY_LEFT_CTRL);
  if (m & KEY_MOD_LSHIFT)
    Keyboard.release(KEY_LEFT_SHIFT);
  if (m & KEY_MOD_LALT)
    Keyboard.release(KEY_LEFT_ALT);
  if (m & KEY_MOD_LGUI)
    Keyboard.release(KEY_LEFT_GUI);
  if (m & KEY_MOD_RCTRL)
    Keyboard.release(KEY_RIGHT_CTRL);
  if (m & KEY_MOD_RSHIFT)
    Keyboard.release(KEY_RIGHT_SHIFT);
  if (m & KEY_MOD_RALT)
    Keyboard.release(KEY_RIGHT_ALT);
  if (m & KEY_MOD_RGUI)
    Keyboard.release(KEY_RIGHT_GUI);
}

void hidKeyboardPress(uint8_t modifiers, uint8_t keycode)
{
  if (modifiers)
    pressMods(modifiers);
  if (keycode != HID_KEY_NONE)
    Keyboard.press(keycode);
}
void hidKeyboardRelease(uint8_t keycode)
{
  if (keycode != HID_KEY_NONE)
    Keyboard.release(keycode);
}
void hidKeyboardReleaseAll(void)
{
  Keyboard.releaseAll();
}
void hidKeyboardWrite(uint8_t modifiers, uint8_t keycode)
{
  if (modifiers)
    pressMods(modifiers);
  if (keycode != HID_KEY_NONE)
    Keyboard.press(keycode);
  delay(5);
  Keyboard.releaseAll();
}

void hidResetConection(void)
{
  // Simula un reset de USB (no es lo mismo que un reset de MCU)
  // Útil para forzar la reconexión en algunos sistemas operativos
  // Breve test visual/háptico de arranque (opcional)
  for (int i = 0; i < 4; ++i)
  {
    digitalWrite(Pins::LED_R, HIGH);
    digitalWrite(Pins::LED_L, HIGH);
    startHaptic(70);
    delay(120);
    updateHaptics();
    digitalWrite(Pins::LED_R, LOW);
    digitalWrite(Pins::LED_L, LOW);
    delay(100);
  }
}

void hidResetConfig(void)
{
  // Resetea la configuración (actualmente no hace nada)
  for (int i = 0; i < 3; ++i)
  {
    digitalWrite(Pins::LED_R, HIGH);
    digitalWrite(Pins::LED_L, HIGH);
    startHaptic(70);
    delay(130);
    updateHaptics();
    digitalWrite(Pins::LED_R, LOW);
    digitalWrite(Pins::LED_L, LOW);
    delay(100);
  }
}

#endif // HID_BACKEND_USB