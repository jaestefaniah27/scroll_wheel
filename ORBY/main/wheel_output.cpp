#include "wheel_output.h"
#include "events.h"
#include "config.h"
#include "hid_adapter.h"
#include "modes.h"
// #include <HID-Project.h>   // Consumer keys

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
        hidConsumerClick(CC_VOL_DOWN);
        volAccumulator -= Ui::VOL_TICK_DIV;
        nextVolAt = millis() + 1;
      } else if (volAccumulator <= -Ui::VOL_TICK_DIV) {
        hidConsumerClick(CC_VOL_UP);
        volAccumulator += Ui::VOL_TICK_DIV;
        nextVolAt = millis() + 1;
      }
    }
    return;
  }

  switch (getWheelMode()) {
    case WheelMode::SCROLL:
      hidMouseSend(0, 0, 0, -delta, 0);
      break;

    case WheelMode::PAN:
      hidMouseSend(0, 0, 0, 0, delta);
      break;

    case WheelMode::ZOOM: {
      // vertical con divisor (float permitido en descriptor 16-bit)
      const int16_t zoom = (int16_t)( (float)delta / (float)Ui::ZOOM_TICK_DIV );
      if (zoom != 0) {
        hidMouseSend(0, 0, 0, -zoom, 0);
      }
    } break;

    case WheelMode::VOLUME: {
      volAccumulator += delta;
      if (millis() >= nextVolAt) {
        if (volAccumulator >= Ui::VOL_TICK_DIV) {
          hidConsumerClick(CC_VOL_DOWN);
          volAccumulator -= Ui::VOL_TICK_DIV;
          nextVolAt = millis() + 1;
        } else if (volAccumulator <= -Ui::VOL_TICK_DIV) {
          hidConsumerClick(CC_VOL_UP);
          volAccumulator += Ui::VOL_TICK_DIV;
          nextVolAt = millis() + 1;
        }
      }
    } break;

    case WheelMode::SELECT: {
      // Alterna pequeñas subidas/bajadas en Y y aplica scroll vertical
      const int8_t y = selectToggle ? 5 : -5;
      selectToggle = !selectToggle;
      hidMouseSend(MOUSE_LEFT, 0, y, -delta, 0);
    } break;
  }
}
