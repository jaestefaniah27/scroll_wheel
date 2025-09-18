#include <Arduino.h>
#include <PluggableUSB.h>
#include <HID.h>
#include <Wire.h>
#include <CapacitiveSensor.h>
#include <HID-Project.h>
#include <HID-Settings.h>
// #include <Keyboard.h>


// I2C address and register for the AS5600 magnetic encoder
#define AS5600_I2C_ADDR   0x36
#define AS5600_REG_STATUS 0x0B

#define BUTTON_PIN_R        10
#define LED_PIN_R           16
#define BUTTON_PIN_L         9
#define LED_PIN_L            8
#define VIBRATION_MOTOR_PIN  5
// SDA = 2
// SCL = 3

// Timing antirebotes
#define DELAY_THRESHOLD_MS 250

// Dividers
constexpr int              VOL_TICK_DIVIDER  = 32;               // 
static float volAccumulator = 0.0f;


// Sampling parameters for AS5600
constexpr unsigned long SAMPLE_INTERVAL_US   = 1000000UL / 1200;  // ~833 µs → 1200 Hz
constexpr uint8_t          SAMPLES_PER_BLOCK = 10;               // 10 samples → 120 Hz

// MODES
typedef enum {
    SCROLL,
    VOLUME,
    PAN,
    ZOOM,
    SELECT
} WHEEL_MODE;

WHEEL_MODE wheel_mode;

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
CustomMouse ScrollWheel;

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


// -----------------------------------------------------------------------------
// Setup: initialize I2C, sensors, encoder, keyboard
// -----------------------------------------------------------------------------
void setup() {
    wheel_mode = SCROLL;

    Wire.begin();
    
    pinMode(BUTTON_PIN_R, INPUT_PULLUP);
    pinMode(BUTTON_PIN_L, INPUT_PULLUP);
    pinMode(LED_PIN_R, OUTPUT);
    pinMode(LED_PIN_L, OUTPUT);
    pinMode(VIBRATION_MOTOR_PIN, OUTPUT);
    for (int i=0; i<3; i++) {
      digitalWrite(LED_PIN_R, HIGH);
      digitalWrite(LED_PIN_L, HIGH);
      digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN_R, LOW);
      digitalWrite(LED_PIN_L, LOW);
      digitalWrite(VIBRATION_MOTOR_PIN, LOW);
      delay(100);
    }
      digitalWrite(LED_PIN_R, LOW);
      digitalWrite(LED_PIN_L, LOW);
    // Initialize magnetic encoder tracking
    lastRawAngle   = readRawAngle();
    // lastAccumPos   = readRawAngle();
    // centerPosition();

    Keyboard.begin();
    Consumer.begin();    // for media keys
}

// -----------------------------------------------------------------------------
// Main loop: sample encoder, handle touch-triggered keyboard/mouse events
// -----------------------------------------------------------------------------
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

    // Si se pulsa el boton derecho de manera corta, se togglea el modo PAN. 
    // Si se pulsa el botón derecho de manera continuada, se activa modo ZOOM mientras se suelte el botón.
    // Si se pulsa el botón izquierdo de manera corta, se togglea el modo VOLUME.
    // Si se pulsa el botón izquierdo de manera continuada, se activa modo SELECT hasta que se suelte el botón.

    // Para distinguir entre pulsación corta y continuada, se checkea una vez, y después se espera DELAY_THRESHOLD_MS para ver si sigue pulsado o no.
    // Si se dejó de pulsar, se activa la acción de pulsación corta.
    // Si sigue pulsado, se activa la acción de pulsación larga.
      static unsigned long lastButtonRPressTime = 0;
      static bool buttonRWasPressed = false;
      if (digitalRead(BUTTON_PIN_R) == LOW) {
        if (!buttonRWasPressed) {
          buttonRWasPressed = true;
          lastButtonRPressTime = millis();
        } else {
          if (millis() - lastButtonRPressTime > DELAY_THRESHOLD_MS) {
            // Acción de pulsación larga
            if (wheel_mode != ZOOM) {
              wheel_mode = ZOOM;
              digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
              delay(130);
              Keyboard.press(KEY_LEFT_CTRL);
              delay(10);
              digitalWrite(VIBRATION_MOTOR_PIN, LOW);
            }
          }
        }
      } else {
        if (buttonRWasPressed) {
          buttonRWasPressed = false;
          if (millis() - lastButtonRPressTime <= DELAY_THRESHOLD_MS) {
            // Acción de pulsación corta
            if (wheel_mode != PAN) {
              wheel_mode = PAN;
              digitalWrite(LED_PIN_R, HIGH);
              digitalWrite(LED_PIN_L, LOW);
              digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
              delay(140);
              digitalWrite(VIBRATION_MOTOR_PIN, LOW);
              delay(500);
            } else {
              wheel_mode = SCROLL;
              digitalWrite(LED_PIN_R, LOW);
              digitalWrite(LED_PIN_L, LOW);
              digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
              delay(140);
              digitalWrite(VIBRATION_MOTOR_PIN, LOW);
              delay(500);
            }
          } else if (wheel_mode == ZOOM) {
              // se suelta el botón
              wheel_mode = SCROLL;
              Keyboard.release(KEY_LEFT_CTRL);
              digitalWrite(LED_PIN_R, LOW);
              digitalWrite(LED_PIN_L, LOW);
              digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
              delay(140);
              digitalWrite(VIBRATION_MOTOR_PIN, LOW);
              delay(500);
          }
        }
      }
                                                                                                                                                              
    static unsigned long lastButtonLPressTime = 0;
    static bool buttonLWasPressed = false;
    if (digitalRead(BUTTON_PIN_L) == LOW) {
      if (!buttonLWasPressed) {
        buttonLWasPressed = true;
        lastButtonLPressTime = millis();
      } else {
        if (millis() - lastButtonLPressTime > DELAY_THRESHOLD_MS) {
          // Acción de pulsación larga
          if (wheel_mode != SELECT) {
            wheel_mode = SELECT;
            // pulsar botón izquierdo del ratón
            Mouse.press(MOUSE_LEFT);
            digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
            delay(140);
            digitalWrite(VIBRATION_MOTOR_PIN, LOW);
          }
        }
      }
    } else {
      if (buttonLWasPressed) {
        buttonLWasPressed = false;
        if (millis() - lastButtonLPressTime <= DELAY_THRESHOLD_MS) {
          // Acción de pulsación corta
          if (wheel_mode != VOLUME) {
            wheel_mode = VOLUME;
            digitalWrite(LED_PIN_L, HIGH);
            digitalWrite(LED_PIN_R, LOW);
            digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
            delay(140);
            digitalWrite(VIBRATION_MOTOR_PIN, LOW);
          } else {
            wheel_mode = SCROLL;
            digitalWrite(LED_PIN_L, LOW);
            digitalWrite(LED_PIN_R, LOW);
            digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
            delay(140);
            digitalWrite(VIBRATION_MOTOR_PIN, LOW);
          }
        } else if (wheel_mode == SELECT) {
          // soltar botón izquierdo del ratón
          wheel_mode = SCROLL;
          Mouse.release(MOUSE_LEFT);
          digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
          delay(140);
          digitalWrite(VIBRATION_MOTOR_PIN, LOW);   
          delay(500);       
        }
      }
    }

    // Handle the delta according to the current mode
    switch (wheel_mode)
    {
    case SCROLL:
      // Handle scroll
      if (delta != 0) {
          ScrollWheel.sendReport(0, 0, 0, -delta, 0);
      }
      break;
    case VOLUME:
      // Handle volume
      volAccumulator += delta;
      while (volAccumulator >= VOL_TICK_DIVIDER) {
          Consumer.write(MEDIA_VOLUME_DOWN);
          volAccumulator -= VOL_TICK_DIVIDER;
          delay(1);
      }
      while (volAccumulator <= -VOL_TICK_DIVIDER) {
          Consumer.write(MEDIA_VOLUME_UP);
          volAccumulator += VOL_TICK_DIVIDER;
          delay(1);
      }
      break;
    case PAN:
      // Handle pan
            if (delta != 0) {
          ScrollWheel.sendReport(0, 0, 0, 0, delta);
      }
    break;
    case ZOOM:
      // Handle zoom
      if (delta != 0) {
          ScrollWheel.sendReport(0, 0, 0, -delta, 0);
      }
      break;
    case SELECT:
      // Handle select
      static bool toggle_up_down = false;
      if (delta != 0) {
        ScrollWheel.sendReport(0, 0, toggle_up_down ? 5 : -5, -delta, 0);
        toggle_up_down = !toggle_up_down;
      }
      break;
    default:
      break;
    }

    // Reset accumulators
    sumAngle    = 0;
    sampleCount = 0;
  }
}