#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

    //--------------------------------------------------------------------+
    // Board Specific Configuration
    //--------------------------------------------------------------------+

#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT 0
#endif

#ifndef BOARD_TUD_MAX_SPEED
#define BOARD_TUD_MAX_SPEED OPT_MODE_DEFAULT_SPEED
#endif

#define CFG_TUSB_RHPORT0_MODE OPT_MODE_DEVICE
#define CFG_TUSB_OS OPT_OS_PICO

    //--------------------------------------------------------------------+
    // DEVICE CONFIGURATION
    //--------------------------------------------------------------------+

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE 64
#endif

//------------- CLASS -------------//
#define CFG_TUD_CDC 1
#define CFG_TUD_MSC 0
// Dos interfaces HID: 0 teclado/ratón/multimedia, 1 touchpad de precisión
// (hid_ptp.h).
#define CFG_TUD_HID 2
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0

// Búfer de cada interfaz HID. TinyUSB lo usa también para los Feature reports
// por el endpoint de control, y el blob de certificación del touchpad mide 256
// bytes (+1 del ID): con 16 se truncaba y Windows descartaba el touchpad. Ojo:
// ya NO es el tamaño de paquete de los endpoints; ese va fijo en
// ORBY_HID_EP_SIZE (usb_descriptors.c).
#define CFG_TUD_HID_EP_BUFSIZE 264

// CDC FIFO size of TX and RX
#define CFG_TUD_CDC_RX_BUFSIZE 256
#define CFG_TUD_CDC_TX_BUFSIZE 256

// CDC Endpoint transfer buffer size, more is faster
#define CFG_TUD_CDC_EP_BUFSIZE 64

#ifdef __cplusplus
}
#endif
