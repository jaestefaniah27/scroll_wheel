#include "hid_adapter.h"

#ifdef HID_BACKEND_USB

#include <Arduino.h>
#include <PluggableUSB.h>
#include <HID.h>
#include <HID-Project.h>
#include <HID-Settings.h>
#include "config.h"
#include "haptics.h"

// ====== BLOQUE NUEVO: Interfaz HID "Feature-only" (control EP0) ======
#include <USBCore.h>
#include <avr/pgmspace.h>
#ifndef USB_EP_SIZE
#define USB_EP_SIZE 64
#endif

// --- Constantes estándar USB/HID que usa el core AVR ---
#ifndef USB_INTERFACE_DESCRIPTOR_TYPE
#define USB_INTERFACE_DESCRIPTOR_TYPE 0x04
#endif

#ifndef HID_DESCRIPTOR_TYPE
#define HID_DESCRIPTOR_TYPE 0x21
#endif

#ifndef HID_REPORT_DESCRIPTOR_TYPE
#define HID_REPORT_DESCRIPTOR_TYPE 0x22
#endif

#ifndef GET_DESCRIPTOR
#define GET_DESCRIPTOR 0x06
#endif

// // --- Estructuras de descriptor usadas por USB_SendControl ---
// typedef struct
// {
//   uint8_t bLength;
//   uint8_t bDescriptorType;
//   uint8_t bInterfaceNumber;
//   uint8_t bAlternateSetting;
//   uint8_t bNumEndpoints;
//   uint8_t bInterfaceClass;
//   uint8_t bInterfaceSubClass;
//   uint8_t bInterfaceProtocol;
//   uint8_t iInterface;
// } __attribute__((packed)) InterfaceDescriptor;

// typedef struct
// {
//   uint8_t bLength;
//   uint8_t bDescriptorType;       // = HID_DESCRIPTOR_TYPE
//   uint16_t bcdHID;               // 0x0111
//   uint8_t bCountryCode;          // 0
//   uint8_t bNumDescriptors;       // 1
//   uint8_t bReportDescriptorType; // = HID_REPORT_DESCRIPTOR_TYPE
//   uint16_t wDescriptorLength;    // sizeof(report desc)
// } __attribute__((packed)) HIDDescDescriptor;

// Descriptor de report HID (sólo Features, vendor page 0xFF00):
static const uint8_t _cfgFeatureReportDesc[] PROGMEM = {
    0x06, 0x00, 0xFF, // USAGE_PAGE (Vendor 0xFF00)
    0x09, 0x20,       // USAGE (Config Block)
    0xA1, 0x01,       // COLLECTION (Application)
    0x85, SW_RID_CFG, //   REPORT_ID (5)
    0x75, 0x08,       //   REPORT_SIZE (8)
    0x95, 0x20,       //   REPORT_COUNT (32)  = sizeof(sw_feature_config_t)
    0xB1, 0x02,       //   FEATURE (Data,Var,Abs)
    0x09, 0x21,       //   USAGE (Command)
    0x85, SW_RID_CMD, //   REPORT_ID (6)
    0x75, 0x08,
    0x95, 0x04, //   REPORT_COUNT (4)
    0xB1, 0x02, //   FEATURE (Data,Var,Abs)
    0xC0        // END_COLLECTION
};

// Estructura del descriptor HID para una interfaz sin endpoints (solo control)
typedef struct
{
  InterfaceDescriptor hid;
  HIDDescDescriptor desc;
} __attribute__((packed)) HIDFeatureOnly_IfDesc;

class HIDFeatureOnly : public PluggableUSBModule
{
public:
  HIDFeatureOnly() : PluggableUSBModule(0, 0, epType)
  {
    // 0 endpoints, solo control EP0
    PluggableUSB().plug(this);
    // Cargar defaults al iniciar
    memset(&g_cfg, 0, sizeof(g_cfg));
    g_cfg.version = 1;
    sw_storage_load_defaults(&g_cfg);
  }

  // Descriptor de interfaz (SIN endpoint)
  int getInterface(uint8_t *interfaceCount) override
  {
    *interfaceCount += 1;

    InterfaceDescriptor ifDesc = {
        /* bLength             */ sizeof(InterfaceDescriptor),
        /* bDescriptorType     */ USB_INTERFACE_DESCRIPTOR_TYPE,
        /* bInterfaceNumber    */ pluggedInterface,
        /* bAlternateSetting   */ 0x00,
        /* bNumEndpoints       */ 0x00, // SIN endpoints: solo control EP0
        /* bInterfaceClass     */ 0x03, // HID
        /* bInterfaceSubClass  */ 0x00,
        /* bInterfaceProtocol  */ 0x00,
        /* iInterface          */ 0x00};

    HIDDescDescriptor hidDesc = {
        /* bLength                 */ sizeof(HIDDescDescriptor),
        /* bDescriptorType         */ HID_DESCRIPTOR_TYPE, // 0x21
        /* bcdHID                  */ 0x0111,              // HID 1.11
        /* bCountryCode            */ 0x00,
        /* bNumDescriptors         */ 0x01,
        /* bReportDescriptorType   */ HID_REPORT_DESCRIPTOR_TYPE, // 0x22
        /* wDescriptorLength       */ (uint16_t)sizeof(_cfgFeatureReportDesc)};

    int r = 0;
    r += USB_SendControl(0, &ifDesc, sizeof(ifDesc));
    r += USB_SendControl(0, &hidDesc, sizeof(hidDesc));
    return r;
  }

  int getDescriptor(USBSetup &setup) override
  {
    if (setup.bRequest == GET_DESCRIPTOR &&
        setup.wValueH == HID_REPORT_DESCRIPTOR_TYPE &&
        setup.wIndex == pluggedInterface)
    {
      return USB_SendControl(TRANSFER_PGM, _cfgFeatureReportDesc, sizeof(_cfgFeatureReportDesc));
    }
    return 0;
  }

  bool setup(USBSetup &setup) override
  {
    if (setup.wIndex != pluggedInterface)
      return false;

    const uint8_t req = setup.bRequest;
    const uint8_t reqType = setup.bmRequestType;

    // GET_REPORT (Feature)
    if (reqType == REQUEST_DEVICETOHOST_CLASS_INTERFACE && req == HID_GET_REPORT)
    {
      const uint8_t rid = setup.wValueL;   // Report ID
      const uint8_t rtype = setup.wValueH; // 3 = Feature
      if (rtype == 0x03 && rid == SW_RID_CFG)
      {
        return USB_SendControl(0, &g_cfg, sizeof(g_cfg)) >= 0;
      }
      return false;
    }

    // SET_REPORT (Feature)
    if (reqType == REQUEST_HOSTTODEVICE_CLASS_INTERFACE && req == HID_SET_REPORT)
    {
      const uint8_t rid = setup.wValueL;
      const uint8_t rtype = setup.wValueH;
      if (rtype != 0x03)
        return false; // Solo Feature

      if (rid == SW_RID_CFG)
      {
        if (setup.wLength < sizeof(sw_feature_config_t))
          return false;
        sw_feature_config_t tmp;
        int n = USB_RecvControl(&tmp, sizeof(tmp));
        if (n != (int)sizeof(tmp))
          return false;

        // Sanitiza y aplica
        if (tmp.version != 1)
          return false;
        if (tmp.active_mode >= SW_MAX_MODES)
          tmp.active_mode = 0;
        g_cfg = tmp;
        sw_runtime_apply_config(&g_cfg);
        return true;
      }

      if (rid == SW_RID_CMD)
      {
        if (setup.wLength < sizeof(sw_feature_cmd_t))
          return false;
        sw_feature_cmd_t cmd;
        int n = USB_RecvControl(&cmd, sizeof(cmd));
        if (n != (int)sizeof(cmd))
          return false;

        switch (cmd.op)
        {
        case 0:
          sw_storage_save_config(&g_cfg);
          break; // SAVE
        case 1:
          sw_storage_load_defaults(&g_cfg); // RESET_DEFAULTS
          sw_runtime_apply_config(&g_cfg);
          break;
        case 2: /* REQUEST_STATE: luego GET_FEATURE */
          break;
        default:
          break;
        }
        return true;
      }
      return false;
    }

    return false;
  }

  // Utilidades
  void getConfig(sw_feature_config_t *out)
  {
    if (out)
      *out = g_cfg;
  }
  void resetToDefaults()
  {
    sw_storage_load_defaults(&g_cfg);
    sw_runtime_apply_config(&g_cfg);
  }

private:
  uint8_t epType[0];
  sw_feature_config_t g_cfg;
};

// Instancia única
static HIDFeatureOnly *s_featureOnly = nullptr;

// Exponer funciones C:
extern "C" void hidUsbGetConfig(sw_feature_config_t *out_cfg)
{
  if (s_featureOnly && out_cfg)
    s_featureOnly->getConfig(out_cfg);
}
extern "C" void hidUsbResetToDefaults(void)
{
  if (s_featureOnly)
    s_featureOnly->resetToDefaults();
}
// Inicializar la interfaz (llámalo una vez, por ejemplo en setup())
static void _ensureFeatureOnlyStarted()
{
  if (!s_featureOnly)
    s_featureOnly = new HIDFeatureOnly();
}
extern "C" void hidUsbFeatureBegin(void)
{
  _ensureFeatureOnlyStarted();
}

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