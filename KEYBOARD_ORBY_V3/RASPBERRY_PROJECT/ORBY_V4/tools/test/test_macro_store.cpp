// Prueba el almacén de macros del PC (include/macro_store.h) sobre una Flash
// de mentira que se comporta como la de verdad: borrar pone 0xFF, programar
// solo puede bajar bits, y programar sobre algo sin borrar es un fallo.
//
//   g++ -std=c++17 -O2 -I../../include test_macro_store.cpp -o test_macro_store
//
// Lo importante es lo que pasa cuando las cosas van mal: un corte de corriente
// a medias tiene que dejar intacta la copia anterior.
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include "macro_store.h"

using namespace MStore;

static std::vector<uint8_t> flash(2048 * 1024, 0xFF);
static int fallos = 0, sin_borrar = 0;
#define CHECK(c, ...) do { if (!(c)) { fallos++; printf("FALLO: " __VA_ARGS__); printf("\n"); } } while (0)

static const uint8_t* f_read(uint32_t off) { return flash.data() + off; }
static void f_erase(uint32_t off, uint32_t len) {
    if (off % SECTOR || len % SECTOR) { fallos++; printf("FALLO: borrado sin alinear %u\n", off); }
    memset(flash.data() + off, 0xFF, len);
}
static void f_program(uint32_t off, const uint8_t* d, uint32_t len) {
    if (off % PAGE || len % PAGE) { fallos++; printf("FALLO: programa sin alinear %u\n", off); }
    for (uint32_t i = 0; i < len; i++) {
        if (flash[off + i] != 0xFF) sin_borrar++;
        flash[off + i] &= d[i];
    }
}
static const Ops OPS = { f_read, f_erase, f_program, nullptr };

static uint32_t crc_of(const std::vector<uint8_t>& v) {
    return crc32_feed(CFGHASH_INIT, v.data(), v.size()) ^ 0xFFFFFFFFu;
}

static std::vector<uint8_t> blob(uint32_t n, uint32_t seed) {
    std::vector<uint8_t> v(n);
    for (uint32_t i = 0; i < n; i++) { seed = seed * 1664525u + 1013904223u; v[i] = (uint8_t)(seed >> 24); }
    return v;
}

// Escribe en trozos de CHUNK como hace la app. `cut` = cuántos bytes llegan
// antes de «cortarse la corriente» (sin end()).
static Result write_all(Store& s, const std::vector<uint8_t>& v, uint32_t crc, size_t cut = SIZE_MAX) {
    Result r = s.begin((uint32_t)v.size(), crc);
    if (r != OK) return r;
    for (size_t off = 0; off < v.size() && off < cut; off += CHUNK) {
        size_t n = std::min<size_t>(CHUNK, v.size() - off);
        if (off + n > cut) n = cut - off;
        r = s.write((uint32_t)off, v.data() + off, (uint32_t)n);
        if (r != OK) return r;
    }
    if (cut != SIZE_MAX) return OK;
    return s.end();
}

static bool holds(const Store& s, const std::vector<uint8_t>& v) {
    return s.len() == v.size() && s.crc() == crc_of(v) && memcmp(s.data(), v.data(), v.size()) == 0;
}

// «Reiniciar»: un Store nuevo sobre la misma Flash, como al arrancar.
static Store reboot() { Store s(OPS); s.scan(); return s; }

int main() {
    setvbuf(stdout, nullptr, _IONBF, 0);   // que un fallo se lea aunque luego reviente
    // El CRC es el estándar: el mismo que calcula la app (macro-blob.mjs).
    const char* nine = "123456789";
    CHECK((crc32_feed(CFGHASH_INIT, (const uint8_t*)nine, 9) ^ 0xFFFFFFFFu) == 0xCBF43926u, "CRC32 no estándar");

    Store s = reboot();
    CHECK(s.len() == 0 && s.data() == nullptr, "una Flash virgen no está vacía");

    const auto A = blob(1000, 1), B = blob(100 * 1024 + 37, 2), C = blob(5000, 3);
    CHECK(write_all(s, A, crc_of(A)) == OK, "no se escribe A");
    CHECK(holds(s, A), "A no se lee igual");
    s = reboot();
    CHECK(holds(s, A), "A no sobrevive a un reinicio");

    CHECK(write_all(s, B, crc_of(B)) == OK, "no se escribe B");
    CHECK(holds(s, B), "B no se lee igual");
    s = reboot();
    CHECK(holds(s, B), "B no sobrevive a un reinicio");

    // Corte a mitad de los datos: sigue B.
    write_all(s, C, crc_of(C), 2000);
    s = reboot();
    CHECK(holds(s, B), "un corte a mitad de C se lleva B");

    // Corte con todos los datos escritos pero sin la cabecera: sigue B.
    write_all(s, C, crc_of(C), C.size());
    s = reboot();
    CHECK(holds(s, B), "un corte justo antes de la cabecera se lleva B");

    // CRC que no cuadra (algo se perdió por el camino): se rechaza, sigue B.
    CHECK(write_all(s, C, crc_of(C) ^ 1) == ERR_CRC, "un CRC malo no se detecta");
    CHECK(holds(s, B), "un CRC malo deja de enseñar B");
    s = reboot();
    CHECK(holds(s, B), "tras un CRC malo, al reiniciar no queda B");

    // Fuera de orden y tamaños imposibles.
    CHECK(s.begin((uint32_t)C.size(), crc_of(C)) == OK, "begin");
    CHECK(s.write(10, C.data(), 5) == ERR_ORDER, "un trozo fuera de sitio se acepta");
    CHECK(s.end() == ERR_SHORT, "se cierra una escritura incompleta");
    CHECK(s.begin(0, 0) == ERR_SIZE && s.begin(MAX + 1, 0) == ERR_SIZE, "tamaños imposibles aceptados");
    CHECK(s.write(0, C.data(), 5) == ERR_CLOSED, "se escribe sin begin");
    CHECK(holds(s, B), "los errores tocan B");

    // Muchas escrituras seguidas: los bancos se alternan y gana la última.
    std::vector<uint8_t> last;
    for (uint32_t i = 0; i < 9; i++) {
        last = blob(300 + i * 777, 10 + i);
        CHECK(write_all(s, last, crc_of(last)) == OK, "escritura %u", i);
    }
    s = reboot();
    CHECK(holds(s, last), "tras alternar bancos no gana la última");

    // El tamaño máximo cabe.
    const auto big = blob(MAX, 99);
    CHECK(write_all(s, big, crc_of(big)) == OK, "el máximo no cabe");
    s = reboot();
    CHECK(holds(s, big), "el máximo no se lee igual");

    // Si la cabecera vigente se estropea, se vuelve a la anterior (mejor una
    // copia algo vieja que ninguna).
    if (s.data()) {
        const uint32_t active_bank_off = (uint32_t)(s.data() - flash.data()) - HDR;
        flash[active_bank_off] ^= 0xFF;
        s = reboot();
        CHECK(holds(s, last), "con la cabecera rota no se vuelve a la copia anterior");
    }

    // ---- El archivo de perfiles (PStore) vive al lado sin pisar las macros ----
    static_assert(PS_OFFSET + 2 * BANK == 2048 * 1024, "el archivo de perfiles no acaba en los 2 MB");
    {
        std::fill(flash.begin(), flash.end(), 0xFF);
        Store m(OPS), ps(OPS, PROFILES);
        m.scan(); ps.scan();
        const auto M = blob(3000, 5), P = blob(20000, 6), P2 = blob(MAX, 7);
        CHECK(write_all(m, M, crc_of(M)) == OK, "macros");
        CHECK(write_all(ps, P, crc_of(P)) == OK, "perfiles");
        CHECK(write_all(m, M, crc_of(M) ^ 1) == ERR_CRC, "macros con CRC malo");
        CHECK(write_all(ps, P2, crc_of(P2)) == OK, "el máximo de perfiles no cabe");
        CHECK(write_all(ps, P, crc_of(P)) == OK, "perfiles otra vez");
        m = reboot();
        Store ps2(OPS, PROFILES); ps2.scan();
        CHECK(holds(m, M), "las macros se estropean al escribir perfiles");
        CHECK(holds(ps2, P), "los perfiles no sobreviven");
        // Cada almacén ignora el magic del otro.
        Store cruzado(OPS, Layout{ OFFSET, PS_MAGIC }); cruzado.scan();
        CHECK(cruzado.len() == 0, "un almacén lee las cabeceras del otro");
    }

    // ---- Paso de bancos de 224 KB (<= 4.26) a los de 112 KB ----
    {
        // Una copia vieja en el banco 1 antiguo, más nueva que la del banco 0.
        std::fill(flash.begin(), flash.end(), 0xFF);
        const auto old0 = blob(2000, 21), old1 = blob(50000, 22);
        auto put = [&](uint32_t at, const std::vector<uint8_t>& v, uint32_t seq) {
            const uint32_t crc = crc_of(v);
            for (uint32_t o = 0; o < v.size(); o += PAGE) {
                uint8_t pg[PAGE]; memset(pg, 0xFF, PAGE);
                memcpy(pg, v.data() + o, std::min<size_t>(PAGE, v.size() - o));
                f_program(at + HDR + o, pg, PAGE);
            }
            uint8_t h[PAGE]; memset(h, 0xFF, PAGE);
            const Header hd = { MAGIC, seq, (uint32_t)v.size(), crc };
            memcpy(h, &hd, sizeof(hd));
            f_program(at, h, PAGE);
        };
        put(OFFSET, old0, 4);
        put(OFFSET + OLD_BANK, old1, 5);
        Store m(OPS); m.scan();
        CHECK(holds(m, old0), "antes de adoptar debería verse la copia del banco 0");
        CHECK(adopt_wide_bank(OPS, m), "no adopta la copia nueva del banco ancho");
        CHECK(holds(m, old1), "tras adoptar no se ve la copia del banco ancho");
        CHECK(!adopt_wide_bank(OPS, m), "adopta dos veces");
        m = reboot();
        CHECK(holds(m, old1), "la copia adoptada no sobrevive al reinicio");
        Store ps(OPS, PROFILES); ps.scan();
        CHECK(ps.len() == 0, "el banco ancho viejo se ve como archivo de perfiles");

        // Si el banco 0 ya era el más nuevo, el viejo solo se borra.
        std::fill(flash.begin(), flash.end(), 0xFF);
        put(OFFSET, old0, 9);
        put(OFFSET + OLD_BANK, old1, 5);
        Store n(OPS); n.scan();
        CHECK(!adopt_wide_bank(OPS, n) && holds(n, old0), "pisa una copia más nueva con una vieja");
        CHECK(*(const uint32_t*)(flash.data() + OFFSET + OLD_BANK) == 0xFFFFFFFFu, "queda la cabecera vieja");

        // Si la copia vieja no cabe en 112 KB, se descarta sin tocar la buena.
        std::fill(flash.begin(), flash.end(), 0xFF);
        const auto huge = blob(150 * 1024, 23);
        put(OFFSET, old0, 4);
        put(OFFSET + OLD_BANK, huge, 5);
        Store q(OPS); q.scan();
        CHECK(!adopt_wide_bank(OPS, q) && holds(q, old0), "una copia que no cabe rompe la buena");
    }

    CHECK(sin_borrar == 0, "se programaron %d bytes sin borrar", sin_borrar);
    if (fallos) { printf("%d fallos\n", fallos); return 1; }
    printf("OK: almacén de macros\n");
    return 0;
}
