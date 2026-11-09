/*
 * plugins.cpp: Common source file for plugin setup
 *
 * Author: CowboyTim
 *
 * This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <https://unlicense.org>
 */

/*
 * Ecodan CN105 link (Mitsubishi service port), read-only for now.
 *
 * - Owns its own serial port (Serial2 on GPIO17 RX / GPIO18 TX, 2400 8E1). The
 *   esp-at UART1 bridge treats incoming bytes as a text string, which breaks
 *   on binary frames, so UART1 is not used for CN105.
 * - Parses frames in loop_pre() and logs them. Nothing is written to the heat
 *   pump except when AT+CN105_CONNECT is sent by hand.
 * - Frame layout (from public CN105 references, not yet checked on an Ecodan):
 *     [0] 0xFC header, [1] type, [2] 0x01, [3] 0x30, [4] data length N,
 *     [5..5+N-1] data, last byte: checksum = (0x100 - sum of bytes 1..n-2) & 0xFF
 */

#include <Arduino.h>
#include "soc/soc_caps.h"
#include <plugins.h>
#include "common.h"

/* ---- settings ---- */

#define CN105_BAUD        2400
#define CN105_CONFIG      SERIAL_8E1
#define CN105_RX_PIN      17
#define CN105_TX_PIN      18
#define CN105_HEADER      0xFC
#define CN105_FRAME_MAX   64    // longest frame accepted, bytes

#if SOC_UART_NUM > 2
#define CN105_SERIAL      Serial2
#else
// ESP32-C3 has no third UART. This only lets the c3 build compile; the link
// is not usable on that chip.
#define CN105_SERIAL      Serial1
#endif

/* ---- state ---- */

namespace PLUGINS {

static bool cn105_started = false;
static uint8_t rx_buf[CN105_FRAME_MAX];
static uint8_t rx_len = 0;
static uint32_t frames_ok = 0;
static uint32_t frames_bad = 0;
static char status_buf[64];

// Connect request, the fixed handshake packet. Sent only on AT+CN105_CONNECT.
static const uint8_t CONNECT_FRAME[] = {0xFC, 0x5A, 0x01, 0x30, 0x02, 0xCA, 0x01, 0xA8};

/* ---- frame handling ---- */

static uint8_t cn105_checksum(const uint8_t *f, uint8_t n) {
    // sum of bytes 1..n-1 (everything after the header), then two's complement
    uint8_t sum = 0;
    for(uint8_t i = 1; i < n; i++)
        sum += f[i];
    return (uint8_t)(0x100 - sum);
}

static void log_frame(const char *tag, const uint8_t *f, uint8_t n) {
    char hex[3 * CN105_FRAME_MAX + 1];
    size_t p = 0;
    for(uint8_t i = 0; i < n && p < sizeof(hex) - 3; i++)
        p += snprintf(hex + p, sizeof(hex) - p, "%02X ", f[i]);
    LOG("[CN105] %s %s", tag, hex);
}

static void rx_byte(uint8_t c) {
    // resync: a frame starts with the header byte, anything else is skipped
    if(rx_len == 0 && c != CN105_HEADER)
        return;
    rx_buf[rx_len++] = c;
    if(rx_len < 5)
        return;
    uint8_t total = 5 + rx_buf[4] + 1;   // header..length, data, checksum
    if(total > CN105_FRAME_MAX) {
        frames_bad++;
        log_frame("TOO LONG", rx_buf, rx_len);
        rx_len = 0;
        return;
    }
    if(rx_len < total)
        return;
    if(rx_buf[total - 1] == cn105_checksum(rx_buf, total - 1)) {
        frames_ok++;
        log_frame("RX", rx_buf, total);
    } else {
        frames_bad++;
        log_frame("BAD CHECKSUM", rx_buf, total);
    }
    rx_len = 0;
}

static void read_cn105() {
    while(CN105_SERIAL.available() > 0) {
        int c = CN105_SERIAL.read();
        if(c < 0)
            break;
        rx_byte((uint8_t)c);
    }
}

/* ---- plugin hooks ---- */

void setup() {
    if(cn105_started)
        return;
    CN105_SERIAL.begin(CN105_BAUD, CN105_CONFIG, CN105_RX_PIN, CN105_TX_PIN);
    cn105_started = true;
    LOG("[CN105] Serial started: %d baud, 8E1, RX GPIO%d, TX GPIO%d", CN105_BAUD, CN105_RX_PIN, CN105_TX_PIN);
}

void loop_pre() {
    if(!cn105_started)
        return;
    read_cn105();
}

const char *at_cmd_handler(const char *at_cmd) {
    if(strcmp(at_cmd, "AT+CN105_CONNECT") == 0) {
        if(!cn105_started)
            return "+ERROR: CN105 serial is not started";
        CN105_SERIAL.write(CONNECT_FRAME, sizeof(CONNECT_FRAME));
        LOG("[CN105] Sent connect request");
        return "OK";
    }
    if(strcmp(at_cmd, "AT+CN105_STATUS?") == 0) {
        snprintf(status_buf, sizeof(status_buf), "frames ok=%lu bad=%lu started=%d",
                 (unsigned long)frames_ok, (unsigned long)frames_bad, cn105_started ? 1 : 0);
        return status_buf;
    }
    return "+ERROR: unknown command";
}

const char *at_get_help_string() {
    return R"EOF(CN105 Commands (Ecodan heat pump, read-only except CONNECT):
  AT+CN105_CONNECT              - Send the CN105 connect request
  AT+CN105_STATUS?              - Frame counts on the CN105 link)EOF";
}

} // namespace PLUGINS
