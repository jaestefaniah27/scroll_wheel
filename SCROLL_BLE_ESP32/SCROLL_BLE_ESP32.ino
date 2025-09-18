/*
 * ESP32 BLE HID - Mouse con rueda 16-bit y Resolution Multiplier (0x48)
 * Prueba simple: alterna scroll arriba/abajo cada 1 s
 * Librería: NimBLE-Arduino
 */

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <NimBLEHIDDevice.h>
#include <HIDTypes.h>
#include <Wire.h>

// ---------- Config BLE ----------
static NimBLEHIDDevice* hid;
static NimBLECharacteristic* inputReport;    // Report ID 1 (Input)
static NimBLECharacteristic* featureReport;  // Report ID 1 (Feature)
static uint8_t resolutionMultiplier = 1;     // 0 o 1 (por tu descriptor, 2 bits útiles)

#define BLE_DEVICE_NAME "Orby Wireless"
#define VID 0xCafe
#define PID 0x4001
#define VERSION 0x0100

// I2C address and register for the AS5600 magnetic encoder
#define AS5600_I2C_ADDR   0x36
#define AS5600_REG_STATUS 0x0B
#define DELAY_THRESHOLD_MS 250

// Sampling parameters for AS5600
constexpr unsigned long SAMPLE_INTERVAL_US   = 1000000UL / 1200;  // ~833 µs → 1200 Hz
constexpr uint8_t          SAMPLES_PER_BLOCK = 10;               // 10 samples → 120 Hz


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
void setupHID() {
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


// Return true if a magnet is present (MD=1, ML=0)
bool magnetPresent() {
    Wire.beginTransmission(AS5600_I2C_ADDR);
    Wire.write(AS5600_REG_STATUS);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_I2C_ADDR, (uint8_t)1);
    if (Wire.available()) {
        uint8_t status = Wire.read();
        return status & 0x20;  // MD bit, detects presence of magnet// -----------------------------------------------------------------------------
// Variables for sample accumulation
// -----------------------------------------------------------------------------
static int64_t sumAngle   = 0;
static uint8_t sampleCount = 0;
static float   prevMean    = 0.0f;
    }
    return false;
}

// Read raw STATUS register (0x0B), or 0xFF on error
uint8_t readStatusRegister() {
    Wire.beginTransmission(AS5600_I2C_ADDR);
    Wire.write(AS5600_REG_STATUS);
    if (Wire.endTransmission() != 0) {
        return 0xFF;
    }
    Wire.requestFrom(AS5600_I2C_ADDR, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0xFF;
}

// Read 12-bit raw angle (0x0E/0x0F)
uint16_t readRawAngle() {
    Wire.beginTransmission(AS5600_I2C_ADDR);
    Wire.write(0x0E);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_I2C_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return 0;
    uint16_t high = Wire.read();
    uint16_t low  = Wire.read();
    return ((high << 8) | low) & 0x0FFF;
}

// Track incremental and absolute position
static uint16_t lastRawAngle = 0;
static int64_t  accumPos = 0;
// -----------------------------------------------------------------------------
// Variables for sample accumulation
// -----------------------------------------------------------------------------
static int64_t sumAngle   = 0;
static uint8_t sampleCount = 0;
static float   prevMean    = 0.0f;

int16_t readDeltaAngle() {
    uint16_t raw = readRawAngle();
    int16_t diff = lastRawAngle - raw;
    if      (diff >  2048) diff -= 4096;
    else if (diff < -2048) diff += 4096;
    lastRawAngle = raw;
    return diff;
}

int64_t readAccumulatedAngle() {
    accumPos += (int64_t)readDeltaAngle();
    return accumPos;
}


// Re-center position tracking to the current raw angle
void centerPosition() {
    lastRawAngle = readRawAngle(); // referencia del delta
    accumPos     = 0;              // acumulado relativo a 0
}


// Timing
static unsigned long lastSampleTime = 0UL;


// ---------- Demo: alternar scroll arriba/abajo ----------
void setup() {
  setupHID();
  Wire.begin();
  // Initialize magnetic encoder tracking
  lastRawAngle   = readRawAngle();

}

void loop() {
  unsigned long now = micros();
  // digitalWrite(LED_BUILTIN_TX, HIGH);
  // Sample at defined interval and only if magnet is present
  if ((now - lastSampleTime) < SAMPLE_INTERVAL_US || !magnetPresent()) {
      return;
  }
  lastSampleTime += SAMPLE_INTERVAL_US;

  // Accumulate position samples
  int64_t pos = readAccumulatedAngle();
  sumAngle += pos;
  ++sampleCount;

  if (sampleCount >= SAMPLES_PER_BLOCK) {
    // Compute mean and difference from previous block
    float mean = float(sumAngle) / SAMPLES_PER_BLOCK;
    float delta = mean - prevMean;
    prevMean = mean;

    delta = constrain(delta, -32767, 32767);
    if (delta != 0) {
      sendReport(0, 0, 0, -delta, 0);
    }

    // Reset accumulators
    sumAngle    = 0;
    sampleCount = 0;
  }
}
