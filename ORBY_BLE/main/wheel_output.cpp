#include "wheel_output.h"
#include "events.h"
#include "config.h"
#include "hid_ble.h"
#include "modes.h"
// (BLE) HID-Project no usado aquí   // Consumer keys

namespace {
  float volAccumulator = 0.0f;
  unsigned long nextVolAt = 0;
  bool selectToggle = false; // para alternar Y en SELECT
}

void handleWheelDelta(int16_t delta) {
  if (delta == 0) {
    // Aún así drenamos volumen de forma no bloqueante
    if (millis() >= nextVolAt && volAccumulator != 0.0f) {
      if (volAccumulator >= Ui::VOL_TICK_DIV) {
        // Consumer.write (omitido en BLE)(MEDIA_VOLUME_DOWN);
        volAccumulator -= Ui::VOL_TICK_DIV;
        nextVolAt = millis() + 1;
      } else if (volAccumulator <= -Ui::VOL_TICK_DIV) {
        // Consumer.write (omitido en BLE)(MEDIA_VOLUME_UP);
        volAccumulator += Ui::VOL_TICK_DIV;
        nextVolAt = millis() + 1;
      }
    }
    return;
  }

  switch (getWheelMode()) {
    case WheelMode::SCROLL:
      bleSendReport(0, 0, 0, -delta, 0);
      break;

    case WheelMode::PAN:
      bleSendReport(0, 0, 0, 0, delta);
      break;

    case WheelMode::ZOOM: {
      // vertical con divisor (float permitido en descriptor 16-bit)
      const int16_t zoom = (int16_t)( (float)delta / (float)Ui::ZOOM_TICK_DIV );
      if (zoom != 0) {
        bleSendReport(0, 0, 0, -zoom, 0);
      }
    } break;

    case WheelMode::VOLUME: {
      volAccumulator += delta;
      if (millis() >= nextVolAt) {
        if (volAccumulator >= Ui::VOL_TICK_DIV) {
          // Consumer.write (omitido en BLE)(MEDIA_VOLUME_DOWN);
          bleConsumerClick(CC_VOL_DOWN);
          volAccumulator -= Ui::VOL_TICK_DIV;
          nextVolAt = millis() + 1;
        } else if (volAccumulator <= -Ui::VOL_TICK_DIV) {
          // Consumer.write (omitido en BLE)(MEDIA_VOLUME_UP);
          bleConsumerClick(CC_VOL_UP);
          volAccumulator += Ui::VOL_TICK_DIV;
          nextVolAt = millis() + 1;
        }
      }
    } break;

    case WheelMode::SELECT: {
      // Alterna pequeñas subidas/bajadas en Y y aplica scroll vertical
      const int8_t y = selectToggle ? 5 : -5;
      selectToggle = !selectToggle;
      bleSendReport(MOUSE_LEFT, 0, y, -delta, 0);
    } break;
  }
}
