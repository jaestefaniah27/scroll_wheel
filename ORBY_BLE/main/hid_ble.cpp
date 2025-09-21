#include "hid_ble.h"
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <HIDTypes.h>

// ---------- Config BLE ----------
static NimBLEHIDDevice* hid;
static NimBLECharacteristic* inputReport;     // Report ID 1 (Mouse Input)
static NimBLECharacteristic* featureReport;   // Report ID 1 (Feature: Resolution Multiplier)
static NimBLECharacteristic* inputReportCC;   // Report ID 2 (Consumer Control Input)
static NimBLECharacteristic* inputReportKB;    // Report ID 3 (Keyboard Input)
static NimBLECharacteristic* outputReportKB;   // Report ID 3 (Keyboard LEDs Output)
static uint8_t kbReport[8] = {0};              // [mods,res, k0..k5]
static uint8_t kbLEDs = 0;                     // bits: Num(0), Caps(1), Scroll(2), Compose(3), Kana(4)

static uint8_t resolutionMultiplier = 1;      // 0 o 1 (por tu descriptor, 2 bits útiles)

#define BLE_DEVICE_NAME "Orby Wireless"
#define VID 0xCafe
#define PID 0x4001
#define VERSION 0x0100

// ---------- Usages Consumer Control ----------
#define CC_MUTE     0x00E2
#define CC_VOL_UP   0x00E9
#define CC_VOL_DOWN 0x00EA

// ---------- Report Map (Mouse ID=1 + Consumer Control ID=2) ----------
static const uint8_t REPORT_MAP[] = {
  // ===== Mouse (Report ID 1) =====
  0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
  0x09, 0x02,        // USAGE (Mouse)
  0xA1, 0x01,        // COLLECTION (Application)
    0x85, 0x01,      //   REPORT_ID (1)
    0x09, 0x02,      //   USAGE (Mouse)
    0xA1, 0x02,      //   COLLECTION (Logical)
      0x09, 0x01,    //     USAGE (Pointer)
      0xA1, 0x00,    //     COLLECTION (Physical)

        // ----- Buttons (5) -----
        0x05, 0x09,  0x19, 0x01, 0x29, 0x05,
        0x15, 0x00,  0x25, 0x01, 0x75, 0x01,
        0x95, 0x05,  0x81, 0x02,

        // Padding 3 bits
        0x75, 0x03,  0x95, 0x01, 0x81, 0x03,

        // X, Y (rel 8-bit)
        0x05, 0x01,  0x09, 0x30, 0x09, 0x31,
        0x15, 0x81,  0x25, 0x7F, 0x75, 0x08,
        0x95, 0x02,  0x81, 0x06,

        // --- Vertical wheel con Feature Resolution Multiplier (16-bit rel) ---
        0xA1, 0x02,        // COLLECTION (Logical)
          0x09, 0x48,      //   USAGE (Resolution Multiplier)
          0x15, 0x00,      //   LOGICAL_MINIMUM (0)
          0x25, 0x01,      //   LOGICAL_MAXIMUM (1)
          0x35, 0x01,      //   PHYSICAL_MINIMUM (1)
          0x45, 0x78,      //   PHYSICAL_MAXIMUM (120)
          0x75, 0x02,      //   REPORT_SIZE (2)
          0x95, 0x01,      //   REPORT_COUNT (1)
          0xA4,            //   PUSH
          0xB1, 0x02,      //   FEATURE (Data,Var,Abs)

          0x09, 0x38,      //   USAGE (Wheel)
          0x16, 0x01, 0x80,//   LOGICAL_MINIMUM (-32767)
          0x26, 0xFF, 0x7F,//   LOGICAL_MAXIMUM (32767)
          0x35, 0x00,      //   PHYSICAL_MINIMUM (0)
          0x45, 0x00,      //   PHYSICAL_MAXIMUM (0)
          0x75, 0x10,      //   REPORT_SIZE (16)
          0x81, 0x06,      //   INPUT (Data,Var,Rel)
        0xC0,              // END_COLLECTION

        // --- Horizontal (Consumer AC Pan) 16-bit + relleno de Feature ---
        0xA1, 0x02,        // COLLECTION (Logical)
          0x09, 0x48, 0xB4, 0xB1, 0x02,
          0x35, 0x00, 0x45, 0x00, 0x75, 0x04, 0xB1, 0x03,
          0x05, 0x0C,            // USAGE_PAGE (Consumer)
          0x0A, 0x38, 0x02,      // USAGE (AC Pan)
          0x16, 0x01, 0x80,      // LOGICAL_MINIMUM (-32767)
          0x26, 0xFF, 0x7F,      // LOGICAL_MAXIMUM (32767)
          0x75, 0x10,            // REPORT_SIZE (16)
          0x81, 0x06,            // INPUT (Data,Var,Rel)
        0xC0,                    // END_COLLECTION

      0xC0,  // END_COLLECTION (Physical)
    0xC0,    // END_COLLECTION (Logical)
  0xC0,      // END_COLLECTION (Application)

    // ===== Consumer Control (Report ID 2) =====
    0x05, 0x0C,        // USAGE_PAGE (Consumer)
    0x09, 0x01,        // USAGE (Consumer Control)
    0xA1, 0x01,        // COLLECTION (Application)
    0x85, 0x02,            //   REPORT_ID (2)
    0x15, 0x00,            //   LOGICAL_MINIMUM (0)
    0x26, 0xFF, 0x03,      //   LOGICAL_MAXIMUM (0x03FF)
    0x19, 0x00,            //   USAGE_MINIMUM (0x0000)
    0x2A, 0xFF, 0x03,      //   USAGE_MAXIMUM (0x03FF)
    0x75, 0x10,            //   REPORT_SIZE (16)
    0x95, 0x01,            //   REPORT_COUNT (1)
    0x81, 0x00,            //   INPUT (Data,Array,Abs)
  0xC0    ,               // END_COLLECTION   
  // ===== Keyboard (Report ID 3) =====
  0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
  0x09, 0x06,        // USAGE (Keyboard)
  0xA1, 0x01,        // COLLECTION (Application)
    0x85, 0x03,      //   REPORT_ID (3)

    // Modifiers (8 bits)
    0x05, 0x07,            //   USAGE_PAGE (Keyboard/Keypad)
    0x19, 0xE0,            //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xE7,            //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,            //   LOGICAL_MINIMUM (0)
    0x25, 0x01,            //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,            //   REPORT_SIZE (1)
    0x95, 0x08,            //   REPORT_COUNT (8)
    0x81, 0x02,            //   INPUT (Data,Var,Abs)

    // Reservado (1 byte)
    0x95, 0x01,            //   REPORT_COUNT (1)
    0x75, 0x08,            //   REPORT_SIZE (8)
    0x81, 0x03,            //   INPUT (Const,Var,Abs)

    // 6 keycodes (Array, 6 bytes)
    0x95, 0x06,            //   REPORT_COUNT (6)
    0x75, 0x08,            //   REPORT_SIZE (8)
    0x15, 0x00,            //   LOGICAL_MINIMUM (0)
    0x25, 0x65,            //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,            //   USAGE_PAGE (Keyboard/Keypad)
    0x19, 0x00,            //   USAGE_MINIMUM (0)
    0x29, 0x65,            //   USAGE_MAXIMUM (101)
    0x81, 0x00,            //   INPUT (Data,Array,Abs)

    // LEDs (Num, Caps, Scroll, Compose, Kana)
    0x95, 0x05,            //   REPORT_COUNT (5)
    0x75, 0x01,            //   REPORT_SIZE (1)
    0x05, 0x08,            //   USAGE_PAGE (LEDs)
    0x19, 0x01,            //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,            //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,            //   OUTPUT (Data,Var,Abs)

    // Padding de LEDs (3 bits)
    0x95, 0x01,            //   REPORT_COUNT (1)
    0x75, 0x03,            //   REPORT_SIZE (3)
    0x91, 0x03,            //   OUTPUT (Const,Var,Abs)
  0xC0,               // END_COLLECTION
};

// ---------- Callbacks para Feature (Resolution Multiplier) ----------
class FeatureCB : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    std::string v = c->getValue();
    if (!v.empty()) resolutionMultiplier = (uint8_t)(v[0] & 0x03);
  }
  void onRead(NimBLECharacteristic* c, NimBLEConnInfo& info) override {
    c->setValue(&resolutionMultiplier, 1);
  }
};

// ---------- Envío de reportes de RATÓN (ID 1) : 7 bytes ----------
void bleSendReport(uint8_t buttons, int8_t x, int8_t y, int16_t wheelV, int16_t wheelH) {
  if (!inputReport) return;
  uint8_t buf[7];
  buf[0] = buttons & 0x1F;
  buf[1] = (uint8_t)x;
  buf[2] = (uint8_t)y;
  buf[3] = (uint8_t)(wheelV & 0xFF);
  buf[4] = (uint8_t)((wheelV >> 8) & 0xFF);
  buf[5] = (uint8_t)(wheelH & 0xFF);
  buf[6] = (uint8_t)((wheelH >> 8) & 0xFF);
  inputReport->setValue(buf, sizeof(buf));   // (sin Report ID; NimBLE lo sabe por la característica)
  inputReport->notify();
}

// ---------- Envío de reportes CONSUMER (ID 2) : 2 bytes ----------
void bleConsumerPress(uint16_t usage) {
  if (!inputReportCC) return;
  uint8_t rep[2] = { (uint8_t)(usage & 0xFF), (uint8_t)(usage >> 8) }; // LE
  inputReportCC->setValue(rep, sizeof(rep));
  inputReportCC->notify();
}

void bleConsumerRelease() {
  if (!inputReportCC) return;
  uint8_t rep0[2] = { 0x00, 0x00 };
  inputReportCC->setValue(rep0, sizeof(rep0));
  inputReportCC->notify();
}

void bleConsumerClick(uint16_t usage) {
  bleConsumerPress(usage);
  delay(1);
  bleConsumerRelease();
}
// ---------- Envío de reportes TECLADO (ID 3) : 8 bytes ----------
// Envia el buffer actual de teclado (8 bytes)
static inline void kbSendNow() {
  if (!inputReportKB) return;
  inputReportKB->setValue(kbReport, sizeof(kbReport));
  inputReportKB->notify();
}

// Añade un keycode en el array (si hay hueco)
static bool kbAddKey(uint8_t key) {
  if (key == KEY_NONE) return true;
  for (int i=2; i<8; ++i) {
    if (kbReport[i] == key) return true;  // ya está
  }
  for (int i=2; i<8; ++i) {
    if (kbReport[i] == 0) { kbReport[i] = key; return true; }
  }
  return false; // sin hueco (6KRO)
}

// Elimina un keycode del array
static void kbDelKey(uint8_t key) {
  if (key == KEY_NONE) return;
  for (int i=2; i<8; ++i) {
    if (kbReport[i] == key) { kbReport[i] = 0; break; }
  }
}

// ===== API pública =====
void bleKeyboardPress(uint8_t modifiers, uint8_t keycode) {
  kbReport[0] |= modifiers;         // set mods
  kbAddKey(keycode);                // añade key si hay hueco
  kbSendNow();
}

void bleKeyboardRelease(uint8_t keycode) {
  kbDelKey(keycode);
  kbSendNow();
}

void bleKeyboardReleaseAll(void) {
  memset(kbReport, 0, sizeof(kbReport));
  kbSendNow();
}

void bleKeyboardWrite(uint8_t modifiers, uint8_t keycode) {
  bleKeyboardPress(modifiers, keycode);
  // pequeño gap para compatibilidad con algunos hosts
  delay(5);
  bleKeyboardRelease(keycode);
  // si los modifiers se usan sólo para esta tecla, libéralos también
  kbReport[0] &= ~modifiers;
  kbSendNow();
}


// Antes de setupHID():
class ConnCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    s->updateConnParams(info.getConnHandle(), /*min*/6, /*max*/9, /*lat*/0, /*timeout*/200);
  }
  void onMTUChange(uint16_t, NimBLEConnInfo&) override { /* opcional */ }
};
static ConnCB connCB;

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    s->updateConnParams(info.getConnHandle(), 6, 9, 0, 200);
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->start();
  }
};
static ServerCB g_serverCB;

// ---------- Inicialización HID BLE ----------
void setupHIDble() {
  // —— Identidad y radio ——
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setMTU(23);

  // —— Seguridad ——
  NimBLEDevice::setSecurityAuth(/*bond=*/true, /*mitm=*/false, /*sc=*/true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  // —— Servidor y callbacks ——
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(&connCB);

  // —— HID Device y Report Map ——
  hid = new NimBLEHIDDevice(server);
  hid->setManufacturer("jorgerente");
  hid->setPnp(0x02, VID, PID, VERSION);
  hid->setHidInfo(0x00, 0x01);              // sin remote wake
  hid->setReportMap((uint8_t*)REPORT_MAP, sizeof(REPORT_MAP));

  inputReportKB  = hid->getInputReport(3);   // (o getInputReport(3) según tu NimBLE)
  outputReportKB = hid->getOutputReport(3);  // LEDs


  // —— Características (Input/Feature) ——
  inputReport    = hid->getInputReport(1);  // Mouse
  featureReport  = hid->getFeatureReport(1);
  inputReportCC  = hid->getInputReport(2);  // Consumer Control

  static FeatureCB featCB;
  featureReport->setCallbacks(&featCB);
  featureReport->setValue(&resolutionMultiplier, 1);

  // —— Iniciar servicios HID ——
  hid->startServices();

  // —— Advertising como HID ——
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setAppearance(HID_MOUSE);
  adv->addServiceUUID(hid->getHidService()->getUUID());
  NimBLEAdvertisementData scanData;
  scanData.setName(BLE_DEVICE_NAME);
  adv->setScanResponseData(scanData);
  adv->setMinInterval(32);  // ~20 ms
  adv->setMaxInterval(48);  // ~30 ms
  adv->setPreferredParams(6, 9);
  adv->start();
}
