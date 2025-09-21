#include "ble_adapter.h"
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <HIDTypes.h>

// ---------- Config BLE ----------
static NimBLEHIDDevice* hid;
static NimBLECharacteristic* inputReport;    // Report ID 1 (Input)
static NimBLECharacteristic* featureReport;  // Report ID 1 (Feature)
static uint8_t resolutionMultiplier = 1;     // 0 o 1 (por tu descriptor, 2 bits útiles)

#define BLE_DEVICE_NAME "Orby Wireless"
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
void bleSendReport(uint8_t buttons, int8_t x, int8_t y, int16_t wheelV, int16_t wheelH) {
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

// Antes de setupHID():
class ConnCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    // interval: unidades de 1.25 ms  (6=7.5 ms; 9=11.25 ms)
    // latency:  0 (sin saltos)
    // timeout:  unidades de 10 ms (200 = 2 s)
    s->updateConnParams(info.getConnHandle(), /*min*/6, /*max*/9, /*lat*/0, /*timeout*/200);
  }
  void onMTUChange(uint16_t, NimBLEConnInfo&) override { /* opcional */ }
};
static ConnCB connCB;

class ServerCB : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
    // pide conn interval corto (7.5–11.25 ms), sin latencia
    s->updateConnParams(info.getConnHandle(), 6, 9, 0, 200);
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
    // al desconectar, reanuncia para que el host pueda reconectar
    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->start();
  }
};
static ServerCB g_serverCB;


// ---------- Inicialización HID BLE ----------
void setupHIDble() {
  // —— Identidad y radio ——
  NimBLEDevice::init(BLE_DEVICE_NAME);
  NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);   // MAC estable (no random)
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);              // opcional: más TX
  NimBLEDevice::setMTU(23);                            // suficiente para 7 bytes (por claridad)

  // —— Seguridad: bonding persistente (reconexión tras reinicio) ——
  NimBLEDevice::setSecurityAuth(/*bond=*/true, /*mitm=*/false, /*sc=*/true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); // emparejamiento sin PIN

  // —— Servidor y callbacks (conn params / re-advertise on disconnect) ——
  NimBLEServer* server = NimBLEDevice::createServer();
  server->setCallbacks(&connCB);  // asegúrate de tener onConnect->updateConnParams + onDisconnect->adv->start()

  // —— HID Device y Report Map ——
  hid = new NimBLEHIDDevice(server);
  hid->setManufacturer("jorgerente");       // 2.3.x
  hid->setPnp(0x02, VID, PID, VERSION);     // 2.3.x
  hid->setHidInfo(0x00, 0x01);              // sin remote wake
  hid->setReportMap((uint8_t*)REPORT_MAP, sizeof(REPORT_MAP));

  // —— Características (Report ID 1) + Feature 0x48 —— 
  inputReport   = hid->getInputReport(1);
  featureReport = hid->getFeatureReport(1);
  static FeatureCB featCB;
  featureReport->setCallbacks(&featCB);
  featureReport->setValue(&resolutionMultiplier, 1);

  // —— Iniciar servicios HID ——
  hid->startServices();

  // —— Advertising conectable como HID ——
  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->setAppearance(HID_MOUSE);
  adv->addServiceUUID(hid->getHidService()->getUUID());
  NimBLEAdvertisementData scanData;
  scanData.setName(BLE_DEVICE_NAME);   // o lo que quieras anunciar en scan response
  adv->setScanResponseData(scanData);

  // Advertising "rápido" para reconectar ágil tras encender:
  // interval units = 0.625 ms → 32=20 ms, 48=30 ms
  adv->setMinInterval(32);
  adv->setMaxInterval(48);

  // Opcional: permitir conexión directa desde hosts ya emparejados
  adv->setPreferredParams(6, 9); // hint: conn interval 7.5–11.25 ms

  adv->start();
}
