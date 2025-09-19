#pragma once
#include <Arduino.h>
#include <PluggableUSB.h>
#include <HID.h>

// Tamaño por defecto del endpoint si no está definido por la core
#ifndef USB_EP_SIZE
  #define USB_EP_SIZE 64
#endif

// Clase del mouse HID personalizado (declaración completa)
class CustomMouse : public PluggableUSBModule {
public:
  CustomMouse();

  // Descriptores
  int getInterface(uint8_t* interfaceCount) override;
  int getDescriptor(USBSetup& setup) override;

  // Control requests (GET/SET_FEATURE)
  bool setup(USBSetup& setup) override;

  // Envía reporte: botones, X, Y, rueda vertical, rueda horizontal
  void sendReport(uint8_t buttons, int8_t x, int8_t y, int16_t wheelV, int16_t wheelH);

private:
  uint8_t _epType[1];
  uint8_t _resolutionMultiplier = 0;
};

// Instancia global (definida en el .cpp)
extern CustomMouse ScrollWheel;

// Inicializa HID-Project (Keyboard/Consumer). El mouse se instancia globalmente.
void initHID();
