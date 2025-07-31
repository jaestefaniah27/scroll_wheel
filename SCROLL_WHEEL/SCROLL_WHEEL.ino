#include <Arduino.h>
#include <PluggableUSB.h>
#include <HID.h>
#include <Wire.h>
#include <CapacitiveSensor.h>
#include <Keyboard.h>

// I2C address and register for the AS5600 magnetic encoder
#define AS5600_I2C_ADDR   0x36
#define AS5600_REG_STATUS 0x0B

// Built-in LEDs for touch feedback
constexpr uint8_t LED_TOUCH_1 = LED_BUILTIN_RX;
constexpr uint8_t LED_TOUCH_2 = LED_BUILTIN_TX;

// Capacitive touch threshold
constexpr long TOUCH_THRESHOLD = 500;

// Sampling parameters for AS5600
constexpr unsigned long SAMPLE_INTERVAL_US   = 1000000UL / 600;  // ~1666 µs → 600 Hz
constexpr uint8_t          SAMPLES_PER_BLOCK = 10;              // 10 samples → 60 Hz
constexpr int              RAW_TICK_DIVIDER   = 1;               // 1 count = 1 tick

// -----------------------------------------------------------------------------
// Struct to encapsulate a capacitive sensor and its state
// -----------------------------------------------------------------------------
struct CapSenseSensor {
    uint8_t        sendPin;
    uint8_t        receivePin;
    uint8_t        ledPin;
    CapacitiveSensor cs;        // Touch library instance

    // State tracking
    bool    isTouched    = false;
    bool    risingEdge   = false;
    long    rawValue     = 0;
    long    baseline     = 0;

    CapSenseSensor(uint8_t s, uint8_t r, uint8_t led)
      : sendPin(s)
      , receivePin(r)
      , ledPin(led)
      , cs(s, r)
    {}
};

// -----------------------------------------------------------------------------
// List of capacitive sensors in use
// -----------------------------------------------------------------------------
CapSenseSensor sensors[] = {
    { 8,  7,  LED_TOUCH_1 },
    {16, 14,  LED_TOUCH_2 }
};
constexpr uint8_t SENSOR_COUNT = sizeof(sensors) / sizeof(sensors[0]);

// -----------------------------------------------------------------------------
// HID report descriptor (Generic Desktop Mouse with two wheels & 5 buttons)
// -----------------------------------------------------------------------------
const uint8_t REPORT_DESCRIPTOR[] PROGMEM = {
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


// -----------------------------------------------------------------------------
// Custom HID mouse class: handles descriptor and SET/GET_FEATURE for 0x48
// -----------------------------------------------------------------------------
class CustomMouse : public PluggableUSBModule {
public:
    CustomMouse()
      : PluggableUSBModule(1, 1, _epType),
        _resolutionMultiplier(0)
    {
        _epType[0] = EP_TYPE_INTERRUPT_IN;
        PluggableUSB().plug(this);
    }

    // Provide Interface, HID, and Endpoint descriptors
    int getInterface(uint8_t* interfaceCount) override {
        *interfaceCount += 1;
        const uint8_t desc[] PROGMEM = {
            // Interface Descriptor
            0x09, 0x04, pluggedInterface, 0x00, 0x01, 0x03, 0x00, 0x02, 0x00,
            // HID Descriptor
            0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22,
            sizeof(REPORT_DESCRIPTOR) & 0xFF,
            (sizeof(REPORT_DESCRIPTOR) >> 8) & 0xFF,
            // Endpoint Descriptor
            0x07, 0x05, USB_ENDPOINT_IN(pluggedEndpoint),
            0x03, USB_EP_SIZE & 0xFF, (USB_EP_SIZE >> 8) & 0xFF, 0x0A
        };
        return USB_SendControl(0, desc, sizeof(desc));
    }

    // Provide the report descriptor to the host
    int getDescriptor(USBSetup& setup) override {
        if (setup.bmRequestType == REQUEST_DEVICETOHOST_STANDARD_INTERFACE
         && setup.wValueH      == HID_REPORT_DESCRIPTOR_TYPE) {
            return USB_SendControl(TRANSFER_PGM, REPORT_DESCRIPTOR, sizeof(REPORT_DESCRIPTOR));
        }
        return 0;
    }

    // Handle GET_FEATURE / SET_FEATURE for Usage 0x48
    bool setup(USBSetup& setup) override {
        if (setup.wIndex != pluggedInterface) return false;

        uint8_t bm = setup.bmRequestType;
        uint8_t rq = setup.bRequest;
        uint8_t rt = setup.wValueH;  // report type

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

    // Send a 7-byte HID report: buttons, X, Y, vertical wheel, horizontal wheel
    void sendReport(uint8_t buttons,
                    int8_t  x,
                    int8_t  y,
                    int16_t wheelV,
                    int16_t wheelH)
    {
        uint8_t buf[7];
        buf[0] = buttons & 0x1F;
        buf[1] = static_cast<uint8_t>(x);
        buf[2] = static_cast<uint8_t>(y);
        buf[3] = wheelV & 0xFF;
        buf[4] = (wheelV >> 8) & 0xFF;
        buf[5] = wheelH & 0xFF;
        buf[6] = (wheelH >> 8) & 0xFF;
        USB_Send(pluggedEndpoint | TRANSFER_RELEASE, buf, sizeof(buf));
    }

private:
    uint8_t _epType[1];
    uint8_t _resolutionMultiplier;
};

// Global instance of our custom mouse
CustomMouse Mouse;

// -----------------------------------------------------------------------------
// AS5600 helper functions
// -----------------------------------------------------------------------------

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
static long    sumAngle   = 0;
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
static int16_t  lastAccumPos = 0;
// -----------------------------------------------------------------------------
// Variables for sample accumulation
// -----------------------------------------------------------------------------
static long    sumAngle   = 0;
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

int16_t readAccumulatedAngle() {
    int16_t acc = lastAccumPos - readDeltaAngle();
    lastAccumPos = acc;
    return acc;
}

// Re-center position tracking to the current raw angle
void centerPosition() {
    uint16_t raw = readRawAngle();
    lastRawAngle   = raw;
    lastAccumPos   = raw;
    prevMean       = (float)raw;
}



// Timing
static unsigned long lastSampleTime = 0UL;

// -----------------------------------------------------------------------------
// Read all capacitive sensors, update state and LED feedback
// -----------------------------------------------------------------------------
void updateTouchSensors() {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        auto& s = sensors[i];
        long raw = s.cs.capacitiveSensor(30);
        s.rawValue = raw;

        long adjusted = raw - s.baseline;
        bool touched = (adjusted > TOUCH_THRESHOLD);

        s.risingEdge = (!s.isTouched && touched);
        if (touched != s.isTouched) {
            s.isTouched = touched;
            digitalWrite(s.ledPin, touched ? LOW : HIGH);
        }
    }
}

// -----------------------------------------------------------------------------
// Setup: initialize I2C, sensors, encoder, keyboard
// -----------------------------------------------------------------------------
void setup() {
    Wire.begin();
    delay(100);

    pinMode(LED_TOUCH_1, OUTPUT);
    pinMode(LED_TOUCH_2, OUTPUT);

    // Calibrate capacitive sensors
    for (auto& s : sensors) {
        s.cs.set_CS_AutocaL_Millis(0);
        long sum = 0;
        for (int i = 0; i < 50; ++i) {
            sum += s.cs.capacitiveSensor(30);
            delay(10);
        }
        s.baseline = sum / 50;
    }

    // Initialize magnetic encoder tracking
    lastRawAngle   = readRawAngle();
    lastAccumPos   = readRawAngle();
    centerPosition();

    Keyboard.begin();
}

// -----------------------------------------------------------------------------
// Main loop: sample encoder, handle touch-triggered keyboard/mouse events
// -----------------------------------------------------------------------------
void loop() {
    unsigned long now = micros();

    // Sample at defined interval and only if magnet is present
    if ((now - lastSampleTime) < SAMPLE_INTERVAL_US || !magnetPresent()) {
        return;
    }
    lastSampleTime += SAMPLE_INTERVAL_US;

    // Accumulate position samples
    int16_t pos = readAccumulatedAngle();
    sumAngle += pos;
    ++sampleCount;

    if (sampleCount >= SAMPLES_PER_BLOCK) {
        // Compute mean and difference from previous block
        float mean = float(sumAngle) / SAMPLES_PER_BLOCK;
        float delta = mean - prevMean;
        prevMean = mean;

        // Convert to scroll ticks and constrain
        int16_t ticks = int(delta / RAW_TICK_DIVIDER);
        ticks = constrain(ticks, -32767, 32767);

        updateTouchSensors();

        // Touch 1: Ctrl+C
        if (sensors[0].risingEdge) {
            Keyboard.press(KEY_LEFT_CTRL);
            Keyboard.press('c');
            delay(15);
            Keyboard.releaseAll();
        }
        // Touch 2 (when Touch 1 not held): Ctrl+V
        else if (sensors[1].risingEdge && !sensors[0].isTouched) {
            Keyboard.press(KEY_LEFT_CTRL);
            Keyboard.press('v');
            delay(15);
            Keyboard.releaseAll();
        }

        // Send a single HID report if there is scroll activity
        if (ticks != 0) {
            if (sensors[0].isTouched && sensors[1].isTouched) {
                // Two-finger scroll: horizontal wheel
                Mouse.sendReport(0, 0, 0, 0, ticks);
            } else {
                // Single-finger scroll: vertical wheel
                Mouse.sendReport(0, 0, 0, ticks, 0);
            }
        }

        // Re-center if accumulated position drifts beyond two full rotations
        if (abs(pos) > 4095 * 2) {
            centerPosition();
        }

        // Reset accumulators
        sumAngle    = 0;
        sampleCount = 0;
    }
}
