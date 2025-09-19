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
constexpr int              ZOOM_TICK_DIVIDER  = 5;               // 

static float volAccumulator = 0.0f;


// Sampling parameters for AS5600
constexpr unsigned long SAMPLE_INTERVAL_US   = 1000000UL / 1200;  // ~833 µs → 1200 Hz
constexpr uint8_t          SAMPLES_PER_BLOCK = 10;               // 10 samples → 120 Hz


// ---------------------------------------------------
// ------- BUTTONS FSM AND DEFINES -------------------
// ---------------------------------------------------
// ---------- Botonera robusta: antirrebote + corto/largo (al soltar) ----------
constexpr uint16_t DEBOUNCE_MS    = 25;   // antirrebote
constexpr uint16_t LONG_PRESS_MS  = 300;  // umbral de pulsación larga (al soltar)

enum class ButtonEvent : uint8_t {
  NONE = 0,
  R_SHORT, R_LONG,
  L_SHORT, L_LONG
};

struct ButtonFSM {
  uint8_t pin;
  bool activeLow;

  bool stableLevel;
  bool lastReadLevel;
  unsigned long lastChangeMs;
  bool pressed;
  unsigned long pressStartMs;
  bool longNotified;          // <--- NUEVO: ya avisó de larga en esta pulsación

  void begin(uint8_t _pin, bool _activeLow=true) {
    pin = _pin; activeLow = _activeLow;
    pinMode(pin, activeLow ? INPUT_PULLUP : INPUT);
    lastReadLevel = digitalRead(pin);
    stableLevel   = lastReadLevel;
    lastChangeMs  = millis();
    pressed       = false;
    pressStartMs  = 0;
    longNotified  = false;     // <--- init
  }

  bool logicalPressed(bool level) const {
    return activeLow ? (level == LOW) : (level == HIGH);
  }

  // Devuelve:
  // 0=nada, 1=SHORT (al soltar), 2=LONG (al soltar), 3=LONG_HIT (se alcanzó el umbral mientras se mantiene)
  uint8_t poll() {
    bool raw = digitalRead(pin);
    unsigned long now = millis();

    if (raw != lastReadLevel) {
      lastReadLevel = raw;
      lastChangeMs  = now;
    }

    if ((now - lastChangeMs) >= DEBOUNCE_MS && stableLevel != lastReadLevel) {
      stableLevel = lastReadLevel;

      bool isPressed = logicalPressed(stableLevel);

      if (isPressed && !pressed) {
        pressed = true;
        pressStartMs = now;
        longNotified = false;         // <--- reinicia el aviso
      } else if (!isPressed && pressed) {
        pressed = false;
        uint32_t held = now - pressStartMs;
        // reinicia el aviso para la próxima pulsación
        bool wasLong = (held >= LONG_PRESS_MS);
        longNotified = false;
        return wasLong ? 2 : 1;       // LONG o SHORT al soltar
      }
    }

    // Si está pulsado y aún no hemos avisado, comprobar umbral
    if (pressed && !longNotified) {
      uint32_t held = now - pressStartMs;
      if (held >= LONG_PRESS_MS) {
        longNotified = true;
        return 3;                     // LONG_HIT: vibrar ya
      }
    }

    return 0;
  }
};

// Instancias de los dos botones
ButtonFSM btnR, btnL;


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

inline void hapticPulse(uint16_t ms=140) {
  digitalWrite(VIBRATION_MOTOR_PIN, HIGH);
  delay(ms);
  digitalWrite(VIBRATION_MOTOR_PIN, LOW);
}

void setLedsForMode(WHEEL_MODE m) {
  // Ajusta LEDs como en tu lógica actual
  switch (m) {
    case PAN:    digitalWrite(LED_PIN_R, HIGH); digitalWrite(LED_PIN_L, LOW);  break;
    case ZOOM:   digitalWrite(LED_PIN_R, HIGH); digitalWrite(LED_PIN_L, HIGH); break;
    case VOLUME: digitalWrite(LED_PIN_R, LOW);  digitalWrite(LED_PIN_L, HIGH); break;
    case SELECT: digitalWrite(LED_PIN_R, LOW);  digitalWrite(LED_PIN_L, HIGH); break;
    case SCROLL:
    default:     digitalWrite(LED_PIN_R, LOW);  digitalWrite(LED_PIN_L, LOW);  break;
  }
}

// Cambia el estado segun el evento de botón
void changeMode(ButtonEvent ev) {
  WHEEL_MODE prev = wheel_mode;

  switch (ev) {
    case ButtonEvent::R_SHORT: // PAN <-> SCROLL
      wheel_mode = (wheel_mode != PAN) ? PAN : SCROLL;
      break;

    case ButtonEvent::R_LONG:  // ZOOM <-> SCROLL
      wheel_mode = (wheel_mode != ZOOM) ? ZOOM : SCROLL;
      if (wheel_mode == ZOOM) Keyboard.press(KEY_LEFT_CTRL);
      break;

    case ButtonEvent::L_SHORT: // SELECT <-> SCROLL
      wheel_mode = (wheel_mode != SELECT) ? SELECT : SCROLL;
      if (wheel_mode == SELECT) Mouse.press(MOUSE_LEFT);
      break;

    case ButtonEvent::L_LONG:  // VOLUME <-> SCROLL
      wheel_mode = (wheel_mode != VOLUME) ? VOLUME : SCROLL;
      break;

    default:
      return; // nada
  }

  if (wheel_mode != prev) {
    setLedsForMode(wheel_mode);
    hapticPulse(140);
    // Si cambiaste antes el comportamiento del ratón en SELECT/ZOOM “mientras se mantiene”,
    // ya NO hace falta presionar mientras: el cambio ocurre al soltar (evento largo).
    // Si necesitas soltar botones de mouse al salir de SELECT, hazlo aquí:
    if (prev == SELECT && wheel_mode != SELECT) {
      Mouse.release(MOUSE_LEFT);
    }
    if (prev == ZOOM && wheel_mode != ZOOM) {
      Keyboard.release(KEY_LEFT_CTRL);
    }
  }
}

// Procesa el delta de la rueda según el modo actual
static inline void handleWheelDelta(int delta) {
  switch (wheel_mode) {
    case SCROLL:
      if (delta != 0) {
        ScrollWheel.sendReport(0, 0, 0, delta, 0);
      }
      break;

    case VOLUME:
      if (delta != 0) {
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
      }
      break;

    case PAN:
      if (delta != 0) {
        ScrollWheel.sendReport(0, 0, 0, 0, delta);
      }
      break;

    case ZOOM:
      if (delta != 0) {
        float zoom = delta / (float)ZOOM_TICK_DIVIDER;
        ScrollWheel.sendReport(0, 0, 0, -zoom, 0);
      }
      break;

    case SELECT: {
      static bool toggle_up_down = false; // persiste entre llamadas
      if (delta != 0) {
        ScrollWheel.sendReport(0, 0, toggle_up_down ? 5 : -5, delta, 0);
        toggle_up_down = !toggle_up_down;
      }
    } break;

    default:
      break;
  }
}

constexpr uint8_t VIB_PIN = VIBRATION_MOTOR_PIN;
constexpr uint16_t LONG_HIT_PULSE_MS = 80;

struct Haptic {
  bool active = false;
  unsigned long offAt = 0;
  void start(uint16_t ms){ digitalWrite(VIB_PIN, HIGH); active=true; offAt=millis()+ms; }
  void update(){ if (active && (long)(millis()-offAt) >= 0) { digitalWrite(VIB_PIN, LOW); active=false; } }
} haptic;

inline void startHaptic(uint16_t ms){ haptic.start(ms); }
inline void updateHaptics(){ haptic.update(); }

static inline void pollButtonsAndFeedback() {
  uint8_t r = btnR.poll();
  if      (r == 3) startHaptic(LONG_HIT_PULSE_MS);    // feedback temprano
  else if (r == 1) changeMode(ButtonEvent::R_SHORT);
  else if (r == 2) changeMode(ButtonEvent::R_LONG);

  uint8_t l = btnL.poll();
  if      (l == 3) startHaptic(LONG_HIT_PULSE_MS);
  else if (l == 1) changeMode(ButtonEvent::L_SHORT);
  else if (l == 2) changeMode(ButtonEvent::L_LONG);
}



// Devuelve true si hay un delta listo; lo entrega en delta_out (constrain -32767..32767)
static inline bool sampleBlockAndGetDelta(int16_t &delta_out) {
  unsigned long now = micros();

  // Muestrea solo al ritmo fijado y si hay imán
  if ((now - lastSampleTime) < SAMPLE_INTERVAL_US || !magnetPresent()) {
    return false;
  }
  lastSampleTime += SAMPLE_INTERVAL_US;

  // Acumular posición
  int64_t pos = readAccumulatedAngle();
  sumAngle += pos;
  ++sampleCount;

  if (sampleCount < SAMPLES_PER_BLOCK) {
    return false; // aún no hay bloque completo
  }

  // Bloque completo: calcular media y delta respecto al bloque previo
  float mean  = float(sumAngle) / SAMPLES_PER_BLOCK;
  float delta = mean - prevMean;
  prevMean    = mean;

  // Reiniciar acumuladores para el siguiente bloque
  sumAngle = 0;
  sampleCount = 0;

  // Limitar y devolver
  delta = constrain(delta, -32767, 32767);
  delta_out = (int16_t)delta;
  return true;
}


// -----------------------------------------------------------------------------
// Setup: initialize I2C, sensors, encoder, keyboard
// -----------------------------------------------------------------------------
void setup() {
    wheel_mode = SCROLL;

    Wire.begin();
    
    // pinMode(BUTTON_PIN_R, INPUT_PULLUP);
    // pinMode(BUTTON_PIN_L, INPUT_PULLUP);
    btnR.begin(BUTTON_PIN_R, /*activeLow=*/true);
    btnL.begin(BUTTON_PIN_L, /*activeLow=*/true);

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
  // 1) Entrada de usuario y feedback: SIEMPRE
  pollButtonsAndFeedback();   // btnR/btnL.poll + LONG_HIT -> startHaptic(...)
  updateHaptics();            // motor háptico sin bloquear
  
  // 2) Sensado y salida: SOLO cuando haya bloque completo
  int16_t deltaTicks;
  if (sampleBlockAndGetDelta(deltaTicks)) {
    handleWheelDelta(deltaTicks);
  }
}