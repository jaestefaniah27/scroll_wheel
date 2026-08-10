#include "tusb.h"
#include "pico/unique_id.h"
#include "hid_hires.h"
#include "orby_version.h"

//--------------------------------------------------------------------+
// Identidad USB
//--------------------------------------------------------------------+
// 0xCafe es el VID de los *ejemplos* de TinyUSB. Vale para desarrollar, pero no
// es nuestro: un teclado que se distribuye no puede enumerarse con él, porque
// choca con cualquier otro proyecto que tampoco lo haya cambiado y porque
// delata que el firmware va sin tocar desde la plantilla.
// Sustitución pendiente de que Raspberry Pi conceda un PID bajo su VID 0x2E8A
// (gratis para productos basados en RP2040, se pide en raspberrypi/usb-pid;
// ver OrbyGUI/docs/PUBLICACION.md). Mientras tanto se queda: ocupar un PID de
// ese VID sin que esté concedido es peor que seguir con el de ejemplo.
// Al cambiarlo hay que tocar KNOWN_IDS en OrbyGUI/electron/serial.js **a la
// vez**, o la app deja de encontrar el teclado.
#define ORBY_USB_VID 0xCafeu

// El PID hace también de versión del report descriptor HID: Windows lo cachea
// por VID/PID, así que al cambiar la estructura de un informe hay que subirlo o
// el host seguirá usando el descriptor antiguo.
// 0x4007: el paneo horizontal pasa de 8 a 16 bits y el Feature report gana un
// segundo multiplicador. Sin subir el PID, Windows reutilizaría el descriptor
// viejo y leería el informe descuadrado.
#define ORBY_USB_PID 0x4007u

// La versión de firmware en BCD (ORBY_USB_BCD_DEVICE) sale de orby_version.h,
// el mismo sitio del que sale el `FW=` del handshake: escrita a mano aquí, se
// quedaba atrás cada vez que se subía la del handshake.

// Índices del array de strings. Antes iban como números sueltos repartidos
// entre el descriptor de dispositivo y el de configuración, y era fácil mover
// uno y olvidar el otro.
enum {
    STRID_LANGID = 0,
    STRID_MANUFACTURER,
    STRID_PRODUCT,
    STRID_SERIAL,
    STRID_CDC,
    STRID_HID,
};

// Multiplicadores de resolución negociados con el host (ver hid_hires.h). Son
// dos independientes: el host puede activar la alta resolución en un eje y no
// en el otro.
volatile uint8_t g_hires_multiplier = 0;
volatile uint8_t g_hires_pan        = 0;

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    // Use Composite / IAD Class
    .bDeviceClass       = 0xEF, // TUSB_CLASS_MISC
    .bDeviceSubClass    = 0x02, // MISC_SUBCLASS_COMMON
    .bDeviceProtocol    = 0x01, // MISC_PROTOCOL_IAD
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = ORBY_USB_VID,
    .idProduct          = ORBY_USB_PID,
    .bcdDevice          = ORBY_USB_BCD_DEVICE,

    .iManufacturer      = STRID_MANUFACTURER,
    .iProduct           = STRID_PRODUCT,
    .iSerialNumber      = STRID_SERIAL,

    .bNumConfigurations = 0x01
};

uint8_t const * tud_descriptor_device_cb(void) {
    return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// HID Report Descriptor
//--------------------------------------------------------------------+
// Ratón con "High Resolution Scrolling" (especificación de Microsoft).
// Diferencias frente a TUD_HID_REPORT_DESC_MOUSE:
//   - El Wheel vive dentro de una Collection (Logical) junto a un Feature
//     item con Usage "Resolution Multiplier" (0x48). Ese es el gancho que
//     Windows busca para habilitar el scroll suave.
//   - El campo Wheel pasa de 8 a 16 bits: con multiplicador 120 un solo
//     detent ya consume 120 unidades, así que 8 bits se desbordarían.
//   - Se añade AC Pan para scroll horizontal.
#define ORBY_HID_REPORT_DESC_MOUSE_HIRES \
    0x05, 0x01,        /* Usage Page (Generic Desktop)          */ \
    0x09, 0x02,        /* Usage (Mouse)                         */ \
    0xA1, 0x01,        /* Collection (Application)              */ \
    0x85, REPORT_ID_MOUSE, /*   Report ID                       */ \
    0x09, 0x01,        /*   Usage (Pointer)                     */ \
    0xA1, 0x00,        /*   Collection (Physical)               */ \
    /* --- Botones 1..5 --- */                                     \
    0x05, 0x09,        /*     Usage Page (Button)               */ \
    0x19, 0x01,        /*     Usage Minimum (1)                 */ \
    0x29, 0x05,        /*     Usage Maximum (5)                 */ \
    0x15, 0x00,        /*     Logical Minimum (0)               */ \
    0x25, 0x01,        /*     Logical Maximum (1)               */ \
    0x95, 0x05,        /*     Report Count (5)                  */ \
    0x75, 0x01,        /*     Report Size (1)                   */ \
    0x81, 0x02,        /*     Input (Data,Var,Abs)              */ \
    0x95, 0x01,        /*     Report Count (1)                  */ \
    0x75, 0x03,        /*     Report Size (3)                   */ \
    0x81, 0x03,        /*     Input (Const) -> relleno a 1 byte */ \
    /* --- Desplazamiento X / Y --- */                             \
    0x05, 0x01,        /*     Usage Page (Generic Desktop)      */ \
    0x09, 0x30,        /*     Usage (X)                         */ \
    0x09, 0x31,        /*     Usage (Y)                         */ \
    0x15, 0x81,        /*     Logical Minimum (-127)            */ \
    0x25, 0x7F,        /*     Logical Maximum (127)             */ \
    0x75, 0x08,        /*     Report Size (8)                   */ \
    0x95, 0x02,        /*     Report Count (2)                  */ \
    0x81, 0x06,        /*     Input (Data,Var,Rel)              */ \
    /* --- Rueda vertical de alta resolución --- */                \
    0xA1, 0x02,        /*     Collection (Logical)              */ \
    0x09, 0x48,        /*       Usage (Resolution Multiplier)   */ \
    0x15, 0x00,        /*       Logical Minimum (0)             */ \
    0x25, 0x01,        /*       Logical Maximum (1)             */ \
    0x35, 0x01,        /*       Physical Minimum (1)            */ \
    0x45, 0x78,        /*       Physical Maximum (120)          */ \
    0x75, 0x02,        /*       Report Size (2)                 */ \
    0x95, 0x01,        /*       Report Count (1)                */ \
    0xB1, 0x02,        /*       Feature (Data,Var,Abs)          */ \
    0x09, 0x38,        /*       Usage (Wheel)                   */ \
    0x16, 0x01, 0x80,  /*       Logical Minimum (-32767)        */ \
    0x26, 0xFF, 0x7F,  /*       Logical Maximum (32767)         */ \
    0x35, 0x00,        /*       Physical Minimum (0) -> reset   */ \
    0x45, 0x00,        /*       Physical Maximum (0) -> reset   */ \
    0x75, 0x10,        /*       Report Size (16)                */ \
    0x95, 0x01,        /*       Report Count (1)                */ \
    0x81, 0x06,        /*       Input (Data,Var,Rel)            */ \
    0xC0,              /*     End Collection                    */ \
    /* --- Scroll horizontal (AC Pan) de alta resolución --- */    \
    /* Lleva su propio multiplicador, en una Logical Collection    \
       aparte: el host los negocia por separado. Al entrar aquí la \
       página sigue siendo Generic Desktop, que es donde vive el   \
       Usage 0x48. */                                              \
    0xA1, 0x02,        /*     Collection (Logical)              */ \
    0x09, 0x48,        /*       Usage (Resolution Multiplier)   */ \
    0x15, 0x00,        /*       Logical Minimum (0)             */ \
    0x25, 0x01,        /*       Logical Maximum (1)             */ \
    0x35, 0x01,        /*       Physical Minimum (1)            */ \
    0x45, 0x78,        /*       Physical Maximum (120)          */ \
    0x75, 0x02,        /*       Report Size (2)                 */ \
    0x95, 0x01,        /*       Report Count (1)                */ \
    0xB1, 0x02,        /*       Feature (Data,Var,Abs)          */ \
    0x35, 0x00,        /*       Physical Minimum (0) -> reset   */ \
    0x45, 0x00,        /*       Physical Maximum (0) -> reset   */ \
    0x05, 0x0C,        /*       Usage Page (Consumer)           */ \
    0x0A, 0x38, 0x02,  /*       Usage (AC Pan)                  */ \
    0x16, 0x01, 0x80,  /*       Logical Minimum (-32767)        */ \
    0x26, 0xFF, 0x7F,  /*       Logical Maximum (32767)         */ \
    0x75, 0x10,        /*       Report Size (16)                */ \
    0x95, 0x01,        /*       Report Count (1)                */ \
    0x81, 0x06,        /*       Input (Data,Var,Rel)            */ \
    0xC0,              /*     End Collection                    */ \
    /* Relleno del Feature report hasta 1 byte completo: dos bits  \
       de multiplicador vertical, dos de horizontal y cuatro de    \
       relleno. */                                                 \
    0x75, 0x04,        /*     Report Size (4)                   */ \
    0x95, 0x01,        /*     Report Count (1)                  */ \
    0xB1, 0x03,        /*     Feature (Const,Var,Abs)           */ \
    0xC0,              /*   End Collection                      */ \
    0xC0               /* End Collection                        */

uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_KEYBOARD( HID_REPORT_ID(REPORT_ID_KEYBOARD) ),
    ORBY_HID_REPORT_DESC_MOUSE_HIRES,
    TUD_HID_REPORT_DESC_CONSUMER( HID_REPORT_ID(REPORT_ID_CONSUMER) )
};

uint8_t const * tud_hid_descriptor_report_cb(uint8_t instance) {
    (void) instance;
    return desc_hid_report;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+
enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_HID,
    ITF_NUM_TOTAL
};

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN + TUD_HID_DESC_LEN)

#define EPNUM_CDC_NOTIF   0x81
#define EPNUM_CDC_OUT     0x02
#define EPNUM_CDC_IN      0x82

#define EPNUM_HID         0x83

uint8_t const desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0, 500),

    // Interface number, string index, EP notification address and size, EP data address (out, in) and size.
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, STRID_CDC, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT, EPNUM_CDC_IN, CFG_TUD_CDC_EP_BUFSIZE),

    // Interface number, string index, protocol, report descriptor len, EP In address, size & polling interval.
    // bInterval = 1 ms (1000 Hz): a 5 ms el scroll de alta resolución se
    // percibía escalonado porque limitaba la cadencia de informes a 200 Hz.
    TUD_HID_DESCRIPTOR(ITF_NUM_HID, STRID_HID, HID_ITF_PROTOCOL_NONE, sizeof(desc_hid_report), EPNUM_HID, CFG_TUD_HID_EP_BUFSIZE, 1)
};

uint8_t const * tud_descriptor_configuration_cb(uint8_t index) {
    (void) index; // for multiple configurations
    return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+
// Estos strings son lo que el usuario ve en el Administrador de dispositivos la
// primera vez que enchufa el teclado, así que dicen lo que el aparato es y nada
// más: "Composite" era detalle interno del descriptor, y "Orby Corp" nombraba
// una sociedad que no existe.
char const* string_desc_arr [] = {
    (const char[]) { 0x09, 0x04 }, // 0: idioma admitido, inglés (0x0409)
    "Orby",                        // 1: fabricante
    "Orby V4",                     // 2: producto
    NULL,                          // 3: número de serie, generado en tiempo de ejecución
    "Orby V4 Control",             // 4: interfaz CDC
    "Orby V4 Input"                // 5: interfaz HID
};

// El número de serie sale del ID único de la flash de la Pico. Con el "123456"
// fijo de antes, dos teclados enchufados a la vez presentaban la misma
// identidad: Windows los mezclaba y el descubrimiento de puerto de serial.js no
// podía distinguirlos.
// pico_get_unique_board_id_string() devuelve una copia que el SDK cachea en el
// arranque; no toca la flash. Leerla de verdad desde aquí colgaría la placa,
// porque este callback corre en el núcleo 0 mientras el núcleo 1 ejecuta desde
// XIP y una orden a la flash deja el bus sin mapear.
static char serial_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];

static uint16_t _desc_str[32];

uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void) langid;

    uint8_t chr_count;

    if ( index == 0) {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

        const char* str;
        if ( index == STRID_SERIAL ) {
            if ( serial_str[0] == '\0' ) {
                pico_get_unique_board_id_string(serial_str, sizeof(serial_str));
            }
            str = serial_str;
        } else {
            str = string_desc_arr[index];
        }

        chr_count = strlen(str);
        if ( chr_count > 31 ) chr_count = 31;

        for(uint8_t i=0; i<chr_count; i++) {
            _desc_str[1+i] = str[i];
        }
    }

    _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

    return _desc_str;
}

//--------------------------------------------------------------------+
// HID Callbacks
//--------------------------------------------------------------------+
// El host (Windows) lee el Feature Report del ratón para descubrir el estado
// del multiplicador de resolución. Si esta petición falla, Windows descarta el
// soporte de alta resolución y vuelve al scroll de 3 líneas por detent.
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen) {
    (void) instance;

    if (report_type == HID_REPORT_TYPE_FEATURE && report_id == REPORT_ID_MOUSE && reqlen >= 1) {
        // Bits 0-1 el multiplicador vertical, bits 2-3 el horizontal.
        buffer[0] = (uint8_t)((g_hires_multiplier & 0x03) |
                              ((g_hires_pan & 0x03) << 2));
        return 1;
    }
    return 0;
}

// El host escribe el Feature Report para activar (1) o desactivar (0) la alta
// resolución. Windows lo pone a 1 al enumerar y lo devuelve a 0 al suspender.
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize) {
    (void) instance;

    if (report_type != HID_REPORT_TYPE_FEATURE || bufsize == 0) return;

    uint8_t id = report_id;
    uint8_t const* data = buffer;
    uint16_t len = bufsize;

    // Algunos transportes entregan el Report ID como primer byte del buffer.
    if (id == 0 && len > 1) {
        id = data[0];
        data++;
        len--;
    }

    if (id == REPORT_ID_MOUSE && len >= 1) {
        g_hires_multiplier = (uint8_t)(data[0] & 0x03);
        g_hires_pan        = (uint8_t)((data[0] >> 2) & 0x03);
    }
}

