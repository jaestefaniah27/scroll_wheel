/*
 * ESP32 BLE HID - Mouse con rueda 16-bit y Resolution Multiplier (0x48)
 * Prueba simple: alterna scroll arriba/abajo cada 1 s
 * Librería: NimBLE-Arduino
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <HIDTypes.h>

// ---------- Config BLE ----------
static NimBLEHIDDevice* hid;
static NimBLECharacteristic* inputReport;    // Report ID 1 (Input)
static NimBLECharacteristic* featureReport;  // Report ID 1 (Feature)
static uint8_t resolutionMultiplier = 1;     // 0 o 1 (por tu descriptor, 2 bits útiles)

#define BLE_DEVICE_NAME "ESP32 HiRes Scroll"
#define VID 0xCafe
#define PID 0x4001
#define VERSION 0x0100

// ---------- Report Map (tu layout con Report ID 1 para BLE) ----------
static const uint8_t REPORT_MAP[] = {
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

        // --- Vertical wheel con Feature Resolution Multiplier ---
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
  0xC0       // END_COLLECTION (Application)
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

// ---------- Envío de reportes (7 bytes exactamente como en tu USB) ----------
void sendReport(uint8_t buttons, int8_t x, int8_t y, int16_t wheelV, int16_t wheelH) {
  if (!inputReport) return;
  uint8_t buf[7];
  buf[0] = buttons & 0x1F;
  buf[1] = (uint8_t)x;
  buf[2] = (uint8_t)y;
  buf[3] = wheelV & 0xFF;
  buf[4] = (wheelV >> 8) & 0xFF;
  buf[5] = wheelH & 0xFF;
  buf[6] = (wheelH >> 8) & 0xFF;
  inputReport->setValue(buf, sizeof(buf));
  inputReport->notify();
}

// ---------- Inicialización HID BLE ----------
void setupHID() {
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEServer* server = NimBLEDevice::createServer();

  auto* hidDev = new NimBLEHIDDevice(server);
  hid = hidDev;
  hid->manufacturer()->setValue("jorgerente");
  hid->pnp(0x02, VID, PID, VERSION);
  hid->hidInfo(0x00, 0x01); // sin remote wake

  hid->reportMap((uint8_t*)REPORT_MAP, sizeof(REPORT_MAP));

  // Report ID 1 mapeado a características:
  inputReport   = hid->inputReport(1);
  featureReport = hid->featureReport(1);

  static FeatureCB featCB;
  featureReport->setCallbacks(&featCB);
  featureReport->setValue(&resolutionMultiplier, 1);

  hid->startServices();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setAppearance(HID_MOUSE);
  adv->addServiceUUID(hid->hidService()->getUUID());
  adv->start();
}

// ---------- Demo: alternar scroll arriba/abajo ----------
void setup() {
  setupHID();
}

void loop() {
  static bool up = true;
  static uint32_t t0 = millis();

  if (millis() - t0 >= 1000) {   // cada 1 s
    t0 = millis();

    // rueda vertical 16-bit: envía ±n (prueba con valores pequeños para ver suavidad)
    int16_t step = (up ? +4 : -4);   // prueba +1/-1, +4/-4, +16/-16...
    sendReport(0, 0, 0, step, 0);
    up = !up;
  }

  // también puedes probar horizontal:
  // sendReport(0, 0, 0, 0, +4);

  delay(1);
}
