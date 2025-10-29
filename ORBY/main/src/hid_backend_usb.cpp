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

// ====== ORBY CONFIG: definición real de los símbolos (evita "undefined reference") ======
#include <EEPROM.h>

typedef struct __attribute__((packed))
{
  uint8_t version; // =1
  uint8_t btn1_action;
  uint8_t btn2_action;
  uint8_t active_mode;     // 0..3
  uint16_t sensitivity[4]; // LSB primero
  uint8_t flags;
  uint8_t reserved[3];
} OrbyConfig;

#ifndef ORBY_EEPROM_ADDR
#define ORBY_EEPROM_ADDR 0
#endif

// ---- DEFINICIÓN (no 'extern') ----
OrbyConfig gOrbyCfg;

// (opcional pero útil en ESP32: inicializa emulación EEPROM la primera vez)
static inline void orbyEepromMaybeBegin()
{
#if defined(ARDUINO_ARCH_ESP32)
  static bool inited = false;
  if (!inited)
  {
    EEPROM.begin(64);
    inited = true;
  }
#endif
}

// ---- IMPLEMENTACIONES reales ----
void orbyLoadDefaults()
{
  gOrbyCfg.version = 1;
  gOrbyCfg.btn1_action = 0;
  gOrbyCfg.btn2_action = 1;
  gOrbyCfg.active_mode = 0;
  gOrbyCfg.sensitivity[0] = 120;
  gOrbyCfg.sensitivity[1] = 60;
  gOrbyCfg.sensitivity[2] = 30;
  gOrbyCfg.sensitivity[3] = 10;
  gOrbyCfg.flags = 0;
  gOrbyCfg.reserved[0] = gOrbyCfg.reserved[1] = gOrbyCfg.reserved[2] = 0;
}

void orbyLoadFromEEPROM()
{
  orbyEepromMaybeBegin();
  OrbyConfig tmp;
  EEPROM.get(ORBY_EEPROM_ADDR, tmp);
  if (tmp.version == 1)
  {
    gOrbyCfg = tmp;
  }
  else
  {
    orbyLoadDefaults();
    EEPROM.put(ORBY_EEPROM_ADDR, gOrbyCfg);
#if defined(ARDUINO_ARCH_ESP32)
    EEPROM.commit();
#endif
  }
}

void orbySaveToEEPROM()
{
  orbyEepromMaybeBegin();
  EEPROM.put(ORBY_EEPROM_ADDR, gOrbyCfg);
#if defined(ARDUINO_ARCH_ESP32)
  EEPROM.commit();
#endif
}

// --- helpers ---
static uint8_t sum8(const uint8_t *p, uint8_t n)
{
  uint16_t s = 0;
  for (uint8_t i = 0; i < n; i++)
    s += p[i];
  return (uint8_t)s;
}

// Ensambla y envía respuesta
static void orbySendResp(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
  uint8_t hdr[6]; // "ORBY"+cmd+len+chk (chk se pone al final)
  hdr[0] = 'O';
  hdr[1] = 'R';
  hdr[2] = 'B';
  hdr[3] = 'Y';
  hdr[4] = cmd | 0x80; // bit 7=OK
  hdr[5] = len;
  uint8_t chk = sum8(&hdr[4], 2);
  if (len && payload)
    chk += sum8(payload, len);
  Serial.write(hdr, 6);
  if (len && payload)
    Serial.write(payload, len);
  Serial.write(&chk, 1);
}

// Error
static void orbySendErr(uint8_t cmd, uint8_t errcode)
{
  uint8_t hdr[6] = {'O', 'R', 'B', 'Y', 0xFF, 1};
  uint8_t chk = (uint8_t)(hdr[4] + hdr[5] + errcode);
  Serial.write(hdr, 6);
  Serial.write(&errcode, 1);
  Serial.write(&chk, 1);
}

// Serial state machine
void orbyProcessSerial()
{
  static enum { S_SYNC0,
                S_SYNC1,
                S_SYNC2,
                S_SYNC3,
                S_CMD,
                S_LEN,
                S_PAY,
                S_CHK } st = S_SYNC0;
  static uint8_t cmd = 0, len = 0, pay[32], idx = 0;

  while (Serial.available())
  {
    uint8_t b = (uint8_t)Serial.read();
    switch (st)
    {
    case S_SYNC0:
      st = (b == 'O') ? S_SYNC1 : S_SYNC0;
      break;
    case S_SYNC1:
      st = (b == 'R') ? S_SYNC2 : S_SYNC0;
      break;
    case S_SYNC2:
      st = (b == 'B') ? S_SYNC3 : S_SYNC0;
      break;
    case S_SYNC3:
      st = (b == 'Y') ? S_CMD : S_SYNC0;
      break;
    case S_CMD:
      cmd = b;
      st = S_LEN;
      break;
    case S_LEN:
      len = b;
      idx = 0;
      if (len > sizeof(pay))
      {
        st = S_SYNC0;
        orbySendErr(cmd, 0x01);
      }
      else
        st = (len ? S_PAY : S_CHK);
      break;
    case S_PAY:
      pay[idx++] = b;
      if (idx >= len)
        st = S_CHK;
      break;
    case S_CHK:
    {
      uint8_t chk = (uint8_t)(cmd + len + sum8(pay, len));
      if (chk != b)
      {
        orbySendErr(cmd, 0x02);
        st = S_SYNC0;
        break;
      }
      // Ejecuta comando
      if (cmd == 0x10 && len == 0)
      { // GET_CONFIG
        uint8_t out[16];
        out[0] = gOrbyCfg.version;
        out[1] = gOrbyCfg.btn1_action;
        out[2] = gOrbyCfg.btn2_action;
        out[3] = gOrbyCfg.active_mode;
        out[4] = gOrbyCfg.sensitivity[0] & 0xFF;
        out[5] = gOrbyCfg.sensitivity[0] >> 8;
        out[6] = gOrbyCfg.sensitivity[1] & 0xFF;
        out[7] = gOrbyCfg.sensitivity[1] >> 8;
        out[8] = gOrbyCfg.sensitivity[2] & 0xFF;
        out[9] = gOrbyCfg.sensitivity[2] >> 8;
        out[10] = gOrbyCfg.sensitivity[3] & 0xFF;
        out[11] = gOrbyCfg.sensitivity[3] >> 8;
        out[12] = gOrbyCfg.flags;
        out[13] = 0;
        out[14] = 0;
        out[15] = 0;
        orbySendResp(cmd, out, sizeof(out));
      }
      else if (cmd == 0x11 && len == 16)
      { // SET_CONFIG
        if (pay[0] == 1)
        {
          gOrbyCfg.version = pay[0];
          gOrbyCfg.btn1_action = pay[1];
          gOrbyCfg.btn2_action = pay[2];
          gOrbyCfg.active_mode = pay[3];
          gOrbyCfg.sensitivity[0] = (uint16_t)pay[4] | ((uint16_t)pay[5] << 8);
          gOrbyCfg.sensitivity[1] = (uint16_t)pay[6] | ((uint16_t)pay[7] << 8);
          gOrbyCfg.sensitivity[2] = (uint16_t)pay[8] | ((uint16_t)pay[9] << 8);
          gOrbyCfg.sensitivity[3] = (uint16_t)pay[10] | ((uint16_t)pay[11] << 8);
          gOrbyCfg.flags = pay[12];
          orbySendResp(cmd, nullptr, 0);
        }
        else
        {
          orbySendErr(cmd, 0x03);
        }
      }
      else if (cmd == 0x12 && len == 0)
      { // SAVE
        orbySaveToEEPROM();
        orbySendResp(cmd, nullptr, 0);
      }
      else if (cmd == 0x13 && len == 0)
      { // RESET (defaults en RAM)
        orbyLoadDefaults();
        orbySendResp(cmd, nullptr, 0);
      }
      else
      {
        orbySendErr(cmd, 0x04); // comando/len inválidos
      }
      st = S_SYNC0;
    }
    break;
    }
  }
}

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