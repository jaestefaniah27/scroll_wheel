#pragma once
#include <stdint.h>
#include <string.h>
#include "config_hash.h"

// ============================================================================
// ALMACÉN DE LAS MACROS DEL PC
// ============================================================================
// La tecla de una macro guarda solo su id (KEYACT_MACRO + id). Lo que hace la
// macro —abrir una app, escribir un texto, una grabación, el nombre que le puso
// el usuario— lo guardaba únicamente la app del PC en su configuración. Con el
// teclado enchufado en otro ordenador, esa app no sabía qué era la «macro 7» y
// la tecla se quedaba muerta.
//
// Aquí el teclado guarda una copia completa de esas definiciones. No las
// entiende ni las ejecuta: para él es un bloque de bytes opaco que la app
// escribe y lee (qué lleva dentro lo decide OrbyGUI/src/web/macro-blob.mjs).
// Quien ejecuta sigue siendo el PC.
//
// Dos bancos: se escribe siempre en el que no está en uso, y solo pasa a ser el
// bueno cuando su CRC cuadra y se graba su cabecera, que es lo último que se
// escribe. Si se corta la corriente a medias, el banco anterior sigue intacto.
//
// Las operaciones de Flash se reciben de fuera (MStore::Ops) para poder probar
// todo esto en el PC con una Flash de mentira: tools/test/test_macro_store.cpp.

namespace MStore {

static constexpr uint32_t SECTOR = 4096;
static constexpr uint32_t PAGE   = 256;
static constexpr uint32_t OFFSET = 1600 * 1024;     // tras las secuencias del teclado
static constexpr uint32_t BANK   = 224 * 1024;
static constexpr uint32_t HDR    = PAGE;            // la cabecera ocupa la primera página
static constexpr uint32_t MAX    = BANK - HDR;
static constexpr uint32_t MAGIC  = 0x4D53544Fu;
static constexpr uint32_t CHUNK  = 192;             // bytes por línea: 384 hex caben en CMD_BUF_SIZE

struct Header {
    uint32_t magic;
    uint32_t seq;   // el banco con el número más alto es el vigente
    uint32_t len;
    uint32_t crc;
};

// `read` devuelve un puntero a la Flash mapeada; `idle` se llama entre
// operaciones largas (atender el USB, alimentar el watchdog).
struct Ops {
    const uint8_t* (*read)(uint32_t off);
    void (*erase)(uint32_t off, uint32_t len);
    void (*program)(uint32_t off, const uint8_t* data, uint32_t len);
    void (*idle)();
};

enum Result { OK, ERR_SIZE, ERR_ORDER, ERR_SHORT, ERR_CRC, ERR_CLOSED };

class Store {
public:
    explicit Store(const Ops& ops) : ops_(ops) {}

    // Elige el banco vigente. Se llama al arrancar y tras cada escritura.
    void scan() {
        Header a = {}, b = {};
        const bool va = bank_ok(0, &a), vb = bank_ok(1, &b);
        if (va && (!vb || a.seq >= b.seq)) { active_ = 0; cur_ = a; }
        else if (vb)                       { active_ = 1; cur_ = b; }
        else                               { active_ = -1; cur_ = Header{}; }
    }

    uint32_t len() const { return active_ >= 0 ? cur_.len : 0; }
    uint32_t crc() const { return active_ >= 0 ? cur_.crc : 0; }

    // Puntero al contenido vigente, o nullptr si no hay.
    const uint8_t* data() const {
        return active_ >= 0 ? ops_.read(bank_at((uint8_t)active_) + HDR) : nullptr;
    }

    // Prepara el banco libre para `len` bytes con ese CRC. Borra solo los
    // sectores que hacen falta, de uno en uno: borrar el banco entero lleva
    // un par de segundos y el USB no puede quedarse tanto sin atender.
    Result begin(uint32_t len, uint32_t crc) {
        w_.open = false;
        if (len == 0 || len > MAX) return ERR_SIZE;
        w_.bank = (active_ == 0) ? 1 : 0;
        const uint32_t base = bank_at(w_.bank);
        for (uint32_t s = 0; s < HDR + len; s += SECTOR) {
            if (ops_.idle) ops_.idle();
            ops_.erase(base + s, SECTOR);
        }
        w_.len = len;
        w_.crc = crc;
        w_.seq = (active_ >= 0 ? cur_.seq : 0) + 1;
        w_.written = 0;
        w_.fill = 0;
        w_.open = true;
        return OK;
    }

    // Los datos llegan en orden: un trozo fuera de sitio es un error, no un
    // hueco que rellenar. Lo que pase de la longitud anunciada se ignora.
    Result write(uint32_t offset, const uint8_t* src, uint32_t n) {
        if (!w_.open) return ERR_CLOSED;
        if (offset != w_.written) return ERR_ORDER;
        for (uint32_t i = 0; i < n && w_.written < w_.len; i++) {
            w_.page[w_.fill++] = src[i];
            w_.written++;
            if (w_.fill == PAGE) flush_page();
        }
        return OK;
    }

    uint32_t written() const { return w_.written; }

    // Comprueba lo escrito y, solo si cuadra, graba la cabecera: desde ese
    // momento el banco nuevo es el vigente.
    Result end(uint32_t* crc_out = nullptr) {
        if (!w_.open) return ERR_CLOSED;
        if (w_.written != w_.len) return ERR_SHORT;
        flush_page();
        w_.open = false;
        const uint32_t crc = crc_of(w_.bank, w_.len);
        if (crc_out) *crc_out = crc;
        if (crc != w_.crc) return ERR_CRC;
        uint8_t hdr[PAGE];
        memset(hdr, 0xFF, sizeof(hdr));
        const Header h = { MAGIC, w_.seq, w_.len, crc };
        memcpy(hdr, &h, sizeof(h));
        ops_.program(bank_at(w_.bank), hdr, PAGE);
        scan();
        return OK;
    }

private:
    static uint32_t bank_at(uint8_t bank) { return OFFSET + bank * BANK; }

    uint32_t crc_of(uint8_t bank, uint32_t len) const {
        const uint8_t* p = ops_.read(bank_at(bank) + HDR);
        uint32_t crc = CFGHASH_INIT;
        for (uint32_t done = 0; done < len; done += 16384) {
            const uint32_t n = (len - done < 16384) ? (len - done) : 16384;
            crc = crc32_feed(crc, p + done, n);
            if (ops_.idle) ops_.idle();
        }
        return crc ^ 0xFFFFFFFFu;
    }

    // Un banco vale si su cabecera está completa y su contenido da el CRC que
    // dice. Una cabecera a medio grabar o un contenido a medias no pasan.
    bool bank_ok(uint8_t bank, Header* out) const {
        Header h;
        memcpy(&h, ops_.read(bank_at(bank)), sizeof(h));
        if (h.magic != MAGIC || h.len == 0 || h.len > MAX) return false;
        if (crc_of(bank, h.len) != h.crc) return false;
        *out = h;
        return true;
    }

    void flush_page() {
        if (w_.fill == 0) return;
        memset(w_.page + w_.fill, 0xFF, PAGE - w_.fill);
        const uint32_t page_index = (w_.written - 1) / PAGE;
        ops_.program(bank_at(w_.bank) + HDR + page_index * PAGE, w_.page, PAGE);
        w_.fill = 0;
    }

    Ops ops_;
    int8_t active_ = -1;
    Header cur_ = {};
    struct {
        bool     open = false;
        uint8_t  bank = 0;
        uint32_t len = 0, crc = 0, seq = 0, written = 0;
        uint8_t  page[PAGE];
        uint32_t fill = 0;
    } w_;
};

} // namespace MStore
