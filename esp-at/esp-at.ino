/*
 * esp-at.ino - ESP32/ESP8266 AT command firmware with WiFi, BLE UART, and networking support
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

#include <Arduino.h>
#ifdef ARDUINO_ARCH_ESP32
 #ifdef DEBUG
 #define USE_ESP_IDF_LOG
 #define CORE_DEBUG_LEVEL 5
 #define LOG_LOCAL_LEVEL 5
 #endif // DEBUG
 #include <esp_log.h>
 #include <nvs_flash.h>
 #include <nvs.h>
 #include <esp_partition.h>
 #include <esp_spiffs.h>
 #include <SPIFFS.h>
 #include <HardwareSerial.h>
 #ifndef SUPPORT_WIFI
 #define SUPPORT_WIFI
 #endif // SUPPORT_WIFI
 #undef SUPPORT_WIFI
 #ifdef SUPPORT_WIFI
 #include <WiFi.h>
 #include <esp_sleep.h>
 #include <esp_wifi.h>
 #include <esp_wps.h>
 #include <ESPmDNS.h>
 #endif // SUPPORT_WIFI
#else
 #include "EEPROM.h"
#endif // ARDUINO_ARCH_ESP32
#ifdef ARDUINO_ARCH_ESP8266
#ifdef DEBUG
#define USE_ESP_IDF_LOG
#define CORE_DEBUG_LEVEL 5
#define LOG_LOCAL_LEVEL 5
#endif
#include <esp_log.h>
#ifdef SUPPORT_WIFI
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#endif // SUPPORT_WIFI
#endif

#ifdef SUPPORT_NTP
#include <esp_sntp.h>
#endif // SUPPORT_NTP

#include <errno.h>
#include <sys/time.h>
#include "time.h"

#define NOINLINE __attribute__((noinline,noipa))
#define INLINE __attribute__((always_inline))
#define ALIGN(x) __attribute__((aligned(x)))

#ifndef LED
#define LED    8
#endif
#ifndef SUPPORT_LED_BRIGHTNESS
#define SUPPORT_LED_BRIGHTNESS
#endif

#ifdef SUPPORT_LED_BRIGHTNESS
#ifdef ARDUINO_ARCH_ESP32
  // ESP32-C3 uses ledcAttachChannel for PWM setup
#else
  // ESP8266 fallback to regular digital pin
  #undef SUPPORT_LED_BRIGHTNESS
#endif
#endif // SUPPORT_LED_BRIGHTNESS

#ifndef BUTTON
#ifndef BUTTON_BUILTIN
#define BUTTON_BUILTIN 9
#endif
#define BUTTON BUTTON_BUILTIN
#endif

#define LOGUART 0

#ifndef VERBOSE
#define VERBOSE
#endif // VERBOSE

#ifndef ESP_LOG_INFO
#define ESP_LOG_INFO
#endif // ESP_LOG_INFO

#ifndef TIMELOG
#define TIMELOG
#endif // TIMELOG

#ifndef DEFAULT_HOSTNAME
#define DEFAULT_HOSTNAME "uart"
#endif // DEFAULT_HOSTNAME

#ifndef UART_AT
#define UART_AT
#endif // UART_AT

#ifndef SUPPORT_UART1
#define SUPPORT_UART1
#endif // SUPPORT_UART1

#ifdef SUPPORT_WIFI

// WiFi support enabled, enable related features if not explicitly disabled
#ifndef WIFI_WPS
#define WIFI_WPS
#endif // WIFI_WPS

#ifndef SUPPORT_TCP_SERVER
#define SUPPORT_TCP_SERVER
#endif // SUPPORT_TCP_SERVER

#ifndef SUPPORT_TCP
#define SUPPORT_TCP
#endif // SUPPORT_TCP

#ifndef SUPPORT_TLS
#define SUPPORT_TLS
#endif // SUPPORT_TLS

#ifndef SUPPORT_UDP
#define SUPPORT_UDP
#endif // SUPPORT_UDP

#ifndef SUPPORT_NTP
#define SUPPORT_NTP
#endif // SUPPORT_NTP

#ifndef SUPPORT_MDNS
#define SUPPORT_MDNS
#endif // SUPPORT_MDNS

#else

// no WiFi support, disable related features
#undef WIFI_WPS
#undef SUPPORT_TCP_SERVER
#undef SUPPORT_TCP
#undef SUPPORT_TLS
#undef SUPPORT_UDP
#undef SUPPORT_NTP
#undef SUPPORT_MDNS
#endif // !SUPPORT_WIFI

// Include TLS support after all feature definitions
#if defined(SUPPORT_TLS) && defined(SUPPORT_WIFI)
#include <WiFiClientSecure.h>
#endif // SUPPORT_TLS && SUPPORT_WIFI


#ifdef SUPPORT_NTP
#include <esp_sntp.h>
#endif // SUPPORT_NTP

#ifdef SUPPORT_UART1
#define UART1_RX_PIN 0
#define UART1_TX_PIN 1
#endif // SUPPORT_UART1

#if defined(SUPPORT_TCP) || defined(SUPPORT_UDP) || defined(SUPPORT_TCP_SERVER)
#include <fcntl.h>
#include <sys/select.h>
#endif // SUPPORT_TCP || SUPPORT_UDP

#if defined(UART_AT) || defined(BT_CLASSIC)
#include "SerialCommands.h"
#endif

// Function declarations
#ifdef SUPPORT_WIFI
void WiFiEvent(WiFiEvent_t event);
#endif

#if defined(DEBUG) || defined(VERBOSE)
NOINLINE
void print_time_to_serial(const char *tformat = "[\%H:\%M:\%S]: "){
  ALIGN(4) static char _date_outstr[20] = {0};
  static time_t _t;
  static struct tm _tm;
  time(&_t);
  localtime_r(&_t, &_tm);
  memset(_date_outstr, 0, sizeof(_date_outstr));
  strftime(_date_outstr, sizeof(_date_outstr), tformat, &_tm);
  Serial.print(_date_outstr);
}

NOINLINE
void do_printf(uint8_t t, const char *tf, const char *format, ...) {
  ALIGN(4) static char _buf[256] = {0};
  if((t & 2) && tf)
    print_time_to_serial(tf);
  memset(_buf, 0, sizeof(_buf));
  va_list args;
  va_start(args, format);
  vsnprintf(_buf, sizeof(_buf), format, args);
  va_end(args);
  if(t & 1)
    Serial.println(_buf);
  else
    Serial.print(_buf);
}
#endif // DEBUG || VERBOSE

#ifdef VERBOSE
 #ifdef DEBUG
  #define __FILE__ "esp-at.ino"
  #define LOG_TIME_FORMAT "[\%H:\%M:\%S]"
  #define LOG_FILE_LINE "[%hu:%s:%d][info]: ", millis(), __FILE__, __LINE__
  #define LOG(...)    if(cfg.do_verbose){do_printf(2, LOG_TIME_FORMAT, LOG_FILE_LINE); do_printf(1, NULL, __VA_ARGS__);};
  #define LOGT(...)   if(cfg.do_verbose){do_printf(2, LOG_TIME_FORMAT, LOG_FILE_LINE); do_printf(0, NULL, __VA_ARGS__);};
  #define LOGR(...)   if(cfg.do_verbose){do_printf(0, NULL, __VA_ARGS__);};
  #define LOGE(...)   if(cfg.do_verbose){do_printf(2, LOG_TIME_FORMAT, LOG_FILE_LINE); do_printf(0, NULL, __VA_ARGS__);\
                                         do_printf(0, NULL, ", errno: %d (%s)\n", errno, get_errno_string(errno));};
 #else
  #define LOG_TIME_FORMAT "[\%H:\%M:\%S][info]: "
  #define LOG(...)    if(cfg.do_verbose){do_printf(3, LOG_TIME_FORMAT, __VA_ARGS__);};
  #define LOGT(...)   if(cfg.do_verbose){do_printf(2, LOG_TIME_FORMAT, __VA_ARGS__);};
  #define LOGR(...)   if(cfg.do_verbose){do_printf(0, LOG_TIME_FORMAT, __VA_ARGS__);};
  #define LOGE(...)   if(cfg.do_verbose){do_printf(2, LOG_TIME_FORMAT, __VA_ARGS__);\
                                         do_printf(0, NULL, ", errno: %d (%s)\n", errno, get_errno_string(errno));};
 #endif
#else
 #define LOG(...)
 #define LOGT(...)
 #define LOGR(...)
 #define LOGE(...)
#endif // VERBOSE

#ifdef DEBUG
 #define __FILE__ "esp-at.ino"
 #define DEBUG_TIME_FORMAT "[\%H:\%M:\%S]"
 #define DEBUG_FILE_LINE "[%hu:%s:%d][debug]: ", millis(), __FILE__, __LINE__
 #define D(...)   {do_printf(2, DEBUG_TIME_FORMAT, DEBUG_FILE_LINE); do_printf(1, NULL, __VA_ARGS__);};
 #define T(...)   {do_printf(2, DEBUG_TIME_FORMAT, DEBUG_FILE_LINE); do_printf(0, NULL, __VA_ARGS__);};
 #define R(...)   do_printf(0, NULL, __VA_ARGS__);
 #define E(...)   {do_printf(2, DEBUG_TIME_FORMAT, DEBUG_FILE_LINE); do_printf(0, NULL, __VA_ARGS__);\
                  do_printf(0, NULL, ", errno: %d (%s)\n", errno, get_errno_string(errno));};
#else
 #define D(...)
 #define T(...)
 #define R(...)
 #define E(...)
#endif // DEBUG

#ifdef LOOP_DEBUG
#define LOOP_D D
#define LOOP_R R
#define LOOP_T T
#define LOOP_E E
#else
#define LOOP_D(...)
#define LOOP_R(...)
#define LOOP_T(...)
#define LOOP_E(...)
#endif

/* Bluetooth support */
#ifndef BLUETOOTH_UART_AT
#define BLUETOOTH_UART_AT
#endif // BLUETOOTH_UART_AT

#ifdef BLUETOOTH_UART_AT

// Default to BLE if not specified
#define BT_BLE
#undef BT_CLASSIC

#ifndef BLUETOOTH_UART_DEVICE_NAME
#define BLUETOOTH_UART_DEVICE_NAME DEFAULT_HOSTNAME
#endif // BLUETOOTH_UART_DEVICE_NAME

#ifndef BLUETOOTH_UART_DEFAULT_PIN
#define BLUETOOTH_UART_DEFAULT_PIN 123456
#endif

#ifdef BT_CLASSIC
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#warning Bluetooth is not enabled or possible.
#undef BT_CLASSIC
#endif // CONFIG_BT_ENABLED
#if !defined(CONFIG_BT_SPP_ENABLED)
#warning Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#undef BT_CLASSIC
#endif // CONFIG_BT_SPP_ENABLED
#endif // BT_CLASSIC

#ifdef BT_BLE
#include <BLEUUID.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEService.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>
#include "BLESecurity.h"
#include "esp_blufi.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"

#define BLE_ADVERTISING_TIMEOUT 10000   // 10 seconds in milliseconds
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
unsigned long ble_advertising_start = 0;
uint8_t deviceConnected = 0;
uint8_t securityRequestPending = 0;
uint32_t passkeyForDisplay = 0;
#endif // BT_BLE
#endif // BLUETOOTH_UART_AT

#ifdef BT_CLASSIC
/* AT commands over Classic Serial Bluetooth */
BluetoothSerial SerialBT;
ALIGN(4) char atscbt[128] = {""};
SerialCommands ATScBT(&SerialBT, atscbt, sizeof(atscbt), "\r\n", "\r\n");
#endif

#if defined(BT_BLE) && defined(SUPPORT_UART1)
#define SUPPORT_BLE_UART1
#endif // BT_BLE

/* NTP server to use, can be configured later on via AT commands */
#ifndef DEFAULT_NTP_SERVER
#define DEFAULT_NTP_SERVER "at.pool.ntp.org"
#endif

/* Default DNS server for static IPv4 configuration */
#ifndef DEFAULT_DNS_IPV4
#define DEFAULT_DNS_IPV4 "1.1.1.1"
#endif

/* ESP yield, only needed on 1 core ESP (like ESP8266). Multi core ESP32
 * usually runs on CPU1 main arduino sketch and CPU0 for WiFi
 * so yield is not needed there.
 *
 * The esp32c3 is however a single core esp32
 *
 */
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
 #define doYIELD LOOP_D("YIELD %d", __LINE__); yield();
#else
 #define doYIELD
#endif

#ifdef UART_AT
/* our AT commands over UART */
ALIGN(4) char atscbu[128] = {""};
SerialCommands ATSc(&Serial, atscbu, sizeof(atscbu), "\r\n", "\r\n");
#endif // UART_AT

#define CFGVERSION 0x01 // switch between 0x01/0x02/0x03 to reinit the config struct change
#define CFGINIT    0x72 // at boot init check flag

#define IPV4_DHCP    1
#define IPV4_STATIC  2
#define IPV6_SLAAC   4

/* main config */
typedef struct cfg_t {
  uint8_t initialized  = 0;
  uint8_t version      = 0;
  #ifdef VERBOSE
  uint8_t do_verbose   = 1;
  #endif
  #ifdef LOGUART
  uint8_t do_log       = 0;
  #endif
  #ifdef TIMELOG
  uint8_t do_timelog   = 0;
  #endif
  #ifdef LOOP_DELAY
  uint16_t main_loop_delay = 100;
  #endif

  #ifdef SUPPORT_WIFI

  uint8_t wifi_enabled = 1;   // WiFi enabled by default
  char wifi_ssid[32]   = {0}; // max 31 + 1
  char wifi_pass[64]   = {0}; // nax 63 + 1

  #ifdef SUPPORT_NTP
  char ntp_host[64]    = {0}; // max hostname + 1
  #endif // SUPPORT_NTP

  #ifdef SUPPORT_MDNS
  uint8_t mdns_enabled = 1;   // mDNS enabled by default
  char mdns_hostname[64] = {0}; // mDNS hostname, defaults to hostname if empty
  #endif // SUPPORT_MDNS

  uint8_t ip_mode      = IPV4_DHCP | IPV6_SLAAC;
  char hostname[64]    = {0}; // max hostname + 1
  uint8_t ipv4_addr[4] = {0}; // static IP address
  uint8_t ipv4_gw[4]   = {0}; // static gateway
  uint8_t ipv4_mask[4] = {0}; // static netmask
  uint8_t ipv4_dns[4]  = {0}; // static DNS server

  #ifdef SUPPORT_UDP
  // UDP support
  uint16_t udp_port    = 0;
  uint16_t udp_listen_port = 0; // local UDP port to listen on, 0=disabled (IPv4/IPv6 auto-detect)
  uint16_t udp6_listen_port = 0; // local UDP IPv6 port to listen on, 0=disabled
  uint16_t udp_send_port = 0;   // remote UDP port to send to, 0=disabled
  char udp_send_ip[40] = {0};   // remote UDP host to send to, IPv4 or IPv6 string
  char udp_host_ip[40] = {0};   // remote UDP IPv4 or IPv6 string
  #endif // SUPPORT_UDP

  #ifdef SUPPORT_TCP
  // TCP client support
  uint16_t tcp_port    = 0;
  char tcp_host_ip[40] = {0}; // IPv4 or IPv6 string, up to 39 chars for IPv6
  #endif // SUPPORT_TCP

  #ifdef SUPPORT_TLS
  // TLS/SSL configuration
  uint8_t tls_enabled = 0;      // 0=disabled, 1=enabled for TCP connections
  uint8_t tls_verify_mode = 1;  // 0=none, 1=optional, 2=required
  uint8_t tls_use_sni = 1;      // 0=disabled, 1=enabled (Server Name Indication)
  char tls_ca_cert[2048] = {0}; // CA certificate in PEM format
  char tls_client_cert[2048] = {0}; // Client certificate in PEM format
  char tls_client_key[2048] = {0};  // Client private key in PEM format
  char tls_psk_identity[64] = {0};  // PSK identity
  char tls_psk_key[128] = {0};      // PSK key in hex format
  uint16_t tls_port = 0;            // TLS port (if different from tcp_port)
  #endif // SUPPORT_TLS

  #ifdef SUPPORT_TCP_SERVER
  // TCP server support
  uint16_t tcp_server_port = 0; // TCP server port (IPv4/IPv6 dual-stack)
  uint16_t tcp6_server_port = 0; // TCP server IPv6-only port, 0=disabled
  uint8_t tcp_server_max_clients = 8; // maximum concurrent client connections
  #endif // SUPPORT_TCP_SERVER
  #endif // SUPPORT_WIFI

  #ifdef SUPPORT_UART1
  // UART1 configuration
  uint32_t uart1_baud  = 115200;  // baud rate
  uint8_t uart1_data   = 8;       // data bits (5-8)
  uint8_t uart1_parity = 0;       // parity: 0=None, 1=Even, 2=Odd
  uint8_t uart1_stop   = 1;       // stop bits (1-2)
  uint8_t uart1_rx_pin = UART1_RX_PIN; // RX pin
  uint8_t uart1_tx_pin = UART1_TX_PIN; // TX pin
  #endif // SUPPORT_UART1

  #ifdef BLUETOOTH_UART_AT
  // BLE security configuration
  uint32_t ble_pin = BLUETOOTH_UART_DEFAULT_PIN; // PIN code for pairing, 0=none
  uint8_t ble_security_mode = 0;   // Security mode: 0=None, 1=PIN, 2=Bonding
  uint8_t ble_io_cap   = 0;        // IO capability: 0=DisplayOnly, 1=DisplayYesNo, 2=KeyboardOnly, 3=NoInputNoOutput, 4=KeyboardDisplay
  uint8_t ble_auth_req = 0;        // Authentication requirements: 0=None, 1=Bonding, 2=MITM, 3=Bonding+MITM

  // BLE MAC address configuration
  uint8_t ble_addr_type = 0;       // Address type: 0=Public, 1=Random Static, 2=Private Resolvable, 3=Private Non-resolvable
  uint8_t ble_custom_addr[6] = {0}; // Custom MAC address (6 bytes), all zeros = use default
  uint8_t ble_addr_auto_random = 1; // Auto-generate random static address if needed
  #endif // BLUETOOTH_UART_AT

  #if defined(SUPPORT_UART1) && defined(BT_BLE)
  uint8_t ble_uart1_bridge = 0; // 0=disabled, 1=enabled
  #endif // SUPPORT_UART1 && BT_BLE
};
cfg_t cfg;

#if defined(SUPPORT_WIFI) && defined(SUPPORT_NTP)
long last_ntp_log = 0;
uint8_t ntp_is_synced = 0;
int8_t last_hour = -1;
void cb_ntp_synced(struct timeval *tv){
  LOG("[NTP] NTP time synced, system time updated: %s", ctime(&tv->tv_sec));
  ntp_is_synced = 1;
}

void setup_ntp(){
  // if we have a NTP host configured, sync
  if(strlen(cfg.ntp_host)){
    LOG("[NTP] setting up NTP with host: %s, interval: %d, timezone: UTC", cfg.ntp_host, 4 * 3600);
    if(esp_sntp_enabled()){
      LOG("[NTP] already enabled, skipping setup");
      sntp_set_sync_interval(4 * 3600 * 1000UL);
      sntp_setservername(0, (char*)&cfg.ntp_host);
    } else {
      LOG("[NTP] setting up NTP sync");
      esp_sntp_stop();
      sntp_set_sync_interval(4 * 3600 * 1000UL);
      sntp_setservername(0, (char*)&cfg.ntp_host);
      sntp_set_time_sync_notification_cb(cb_ntp_synced);
      sntp_setoperatingmode(SNTP_OPMODE_POLL);
      sntp_init();
    }
    setenv("TZ", "UTC", 1);
    tzset();
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    last_hour = timeinfo.tm_hour;
  }
}
#endif // SUPPORT_WIFI && SUPPORT_NTP

#ifdef SUPPORT_MDNS
void setup_mdns(){
  // Check if mDNS is enabled
  if(!cfg.mdns_enabled){
    LOG("[mDNS] mDNS is disabled, skipping mDNS setup");
    return;
  }

  // Determine hostname to use for mDNS
  const char* hostname_to_use = NULL;
  if(strlen(cfg.mdns_hostname) > 0){
    hostname_to_use = cfg.mdns_hostname;
  } else if(strlen(cfg.hostname) > 0){
    hostname_to_use = cfg.hostname;
  } else {
    hostname_to_use = DEFAULT_HOSTNAME;
  }

  LOG("[mDNS] Starting mDNS responder with hostname: %s.local", hostname_to_use);

  // Start mDNS responder
  if(MDNS.begin(hostname_to_use)){
    LOG("[mDNS] mDNS responder started successfully");

    #if defined(SUPPORT_TCP) || defined(SUPPORT_TCP_SERVER)
    // TCP Server: Add service to MDNS-SD: _uart._tcp._local
    uint16_t tcp_port = cfg.tcp_server_port ? cfg.tcp_server_port : (cfg.tcp6_server_port ? cfg.tcp6_server_port : 0);
    if(tcp_port != 0){
      MDNS.addService("uart", "tcp", tcp_port);
      LOG("[mDNS] Added UART service on port %d", tcp_port);

      // Add additional service information
      MDNS.addServiceTxt("uart", "tcp", "device", "ESP-AT-UART");
      MDNS.addServiceTxt("uart", "tcp", "version", "1.0");
    }
    #endif // SUPPORT_TCP || SUPPORT_TCP_SERVER

    #ifdef SUPPORT_UDP
    // UDP Listener: Add service to MDNS-SD: _uart._udp._local
    uint16_t udp_port = cfg.udp_listen_port ? cfg.udp_listen_port : (cfg.udp6_listen_port ? cfg.udp6_listen_port : 0);
    if(udp_port != 0){
      MDNS.addService("uart", "udp", udp_port);
      LOG("[mDNS] Added UART UDP service on port %d", udp_port);

      // Add additional service information
      MDNS.addServiceTxt("uart", "udp", "device", "ESP-AT-UART");
      MDNS.addServiceTxt("uart", "udp", "version", "1.0");
    }
    #endif // SUPPORT_UDP
  } else {
    LOGE("[mDNS] Error setting up mDNS responder");
  }
}

void stop_mdns(){
  LOG("[mDNS] Stopping mDNS responder");
  MDNS.end();
  LOG("[mDNS] mDNS responder stopped");
}
#endif // SUPPORT_MDNS

/* state flags */
#ifdef SUPPORT_WIFI
long last_wifi_check = 0;
long last_wifi_info_log = 0;
long last_wifi_reconnect = 0;
#endif // SUPPORT_WIFI

#ifdef TIMELOG
long last_time_log = 0;
#endif // TIMELOG

#ifdef ESP_LOG_INFO
long last_esp_info_log = 0;
#endif // DEBUG

#ifdef SUPPORT_UDP
int udp_sock = -1;
int udp_listen_sock = -1;
int udp6_listen_sock = -1;
int udp_out_sock = -1;
#endif // SUPPORT_UDP

#ifdef LED

// PWM settings for LED brightness control
// Uses hardware PWM on ESP32 for smooth brightness control
#define LED_PWM_CHANNEL 2
#define LED_PWM_FREQUENCY 5000  // 5 kHz (above human hearing range)
#define LED_PWM_RESOLUTION 8    // 8-bit resolution (0-255 brightness levels)

// Brightness levels for different states (255=max, 0=min on ESP32)
#define LED_BRIGHTNESS_OFF     255  // LED completely off
#define LED_BRIGHTNESS_ON        0  // LED at full brightness
#define LED_BRIGHTNESS_LOW     200  // LED at low brightness (dimmed)
#define LED_BRIGHTNESS_DIM     150  // LED at dim brightness
#define LED_BRIGHTNESS_MEDIUM  100  // LED at medium brightness
#define LED_BRIGHTNESS_HIGH     50  // LED at high brightness (bright)
#define LED_BRIGHTNESS_FLICKER  20  // LED at very high brightness for flicker

// Blink intervals for different states
#define LED_BLINK_INTERVAL_SLOW     2000  // slow blink (not connected, BLE connected, WPS)
#define LED_BLINK_INTERVAL_NORMAL   1000  // normal blink, startup
#define LED_BLINK_INTERVAL_HALF      500  // half blink (WiFi connecting)
#define LED_BLINK_INTERVAL_QUICK     250  // quick blink (WPS Waiting)
#define LED_BLINK_INTERVAL_FAST      100  // fast blink (BLE advertising)
#define LED_BLINK_INTERVAL_FLICKER    50  // quick flicker for data activity

// LED PWM mode tracking for ESP32
#ifdef ARDUINO_ARCH_ESP32
bool led_pwm_enabled = false; // Track if PWM is working on ESP32
#endif

/* Communication activity tracking for LED */
unsigned long last_tcp_activity = 0;
unsigned long last_udp_activity = 0;
unsigned long last_uart1_activity = 0;
#define COMM_ACTIVITY_LED_DURATION 200  // Show communication activity for 200ms
#endif // LED

/* WPS (WiFi Protected Setup) support */
#if defined(SUPPORT_WIFI) && defined(WIFI_WPS)

#define WPS_TIMEOUT_MS 30000 // 30 seconds

bool wps_running = false;
unsigned long wps_start_time = 0;
esp_wps_config_t wps_config;
#endif // SUPPORT_WIFI && WIFI_WPS

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)
int tcp_server_sock = -1;
int tcp6_server_sock = -1;
#endif // SUPPORT_WIFI && SUPPORT_TCP_SERVER

#ifdef SUPPORT_WIFI
void setup_wifi(){
  LOG("[WiFi] setup started");

  // Check if WiFi is enabled
  if(!cfg.wifi_enabled){
    LOG("[WiFi] WiFi is disabled, skipping WiFi setup");
    WiFi.mode(WIFI_MODE_NULL);
    return;
  }

  WiFi.persistent(false);
  WiFi.disconnect();
  LOG("[WiFi] setting WiFi mode to STA");
  WiFi.mode(WIFI_MODE_STA);
  LOG("[WiFi] adding event handler");
  WiFi.removeEvent(WiFiEvent);
  WiFi.onEvent(WiFiEvent);
  if(WiFi.STA.connected()){
    LOG("[WiFi] Already connected");
    WiFi.STA.disconnect(true); // disconnect and erase old config
  }
  WiFi.STA.begin();
  LOG("[WiFi] MAC: %s", WiFi.macAddress().c_str());
  LOG("[WiFi] IP Mode configured: %s%s%s",
      (cfg.ip_mode & IPV4_DHCP) ? "IPv4 DHCP " : "",
      (cfg.ip_mode & IPV4_STATIC) ? "IPv4 STATIC " : "",
      (cfg.ip_mode & IPV6_SLAAC) ? "IPv6 DHCP " : "");
  LOG("[WiFi] SSID: %s", cfg.wifi_ssid);
  if(strlen(cfg.wifi_pass) == 0) {
    LOG("[WiFi] Pass: none");
  } else {
    // print password as stars, even fake the length
    D("[WiFi] Pass: %s", cfg.wifi_pass);
    LOG("[WiFi] Pass: ******");
  }
  // are we connecting to WiFi?
  if(strlen(cfg.wifi_ssid) == 0){
    LOG("[WiFi] No SSID configured, skipping WiFi setup");
    WiFi.mode(WIFI_MODE_NULL);
    return;
  }
  if(WiFi.status() == WL_CONNECTED){
    LOG("[WiFi] Already connected, skipping WiFi setup");
    return;
  }

  LOG("[WiFi] setting up WiFi");

  // IPv6 configuration, before WiFi.begin() and WiFi.config()
  if(cfg.ip_mode & IPV6_SLAAC){
    LOG("[WiFi] Using DHCP for IPv6");
    if(WiFi.enableIPv6(true)){
      LOG("[WiFi] IPv6 enabled");
    } else {
      LOGE("[WiFi] Failed to enable IPv6");
    }
  } else {
    LOG("[WiFi] Not using IPv6");
    WiFi.enableIPv6(false);
  }


  // IPv4 configuration
  if(cfg.ip_mode & IPV4_DHCP){
    LOG("[WiFi] Using DHCP for IPv4");
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  } else if(cfg.ip_mode & IPV4_STATIC){
    LOG("[WiFi] Using static IPv4 configuration");
    WiFi.config(IPAddress(cfg.ipv4_addr[0], cfg.ipv4_addr[1], cfg.ipv4_addr[2], cfg.ipv4_addr[3]),
                IPAddress(cfg.ipv4_gw[0], cfg.ipv4_gw[1], cfg.ipv4_gw[2], cfg.ipv4_gw[3]),
                IPAddress(cfg.ipv4_mask[0], cfg.ipv4_mask[1], cfg.ipv4_mask[2], cfg.ipv4_mask[3]),
                IPAddress(cfg.ipv4_dns[0], cfg.ipv4_dns[1], cfg.ipv4_dns[2], cfg.ipv4_dns[3]));
  } else {
    LOG("[WiFi] Using no IPv4 configuration, assume loopback address");
    WiFi.config(
      IPAddress(127,0,0,1),
      IPAddress(255,255,255,0),
      IPAddress(127,0,0,1),
      IPAddress(127,0,0,1));
  }

  WiFi.mode(WIFI_MODE_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  if(cfg.hostname){
    WiFi.setHostname(cfg.hostname);
  } else {
    WiFi.setHostname(DEFAULT_HOSTNAME);
  }

  // These need to be called before WiFi.begin()!
  WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK); // require WPA2
  WiFi.setScanMethod(WIFI_FAST_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);

  // set country code if needed
  wifi_country_t country = {0};
  country.schan  = 1;
  country.nchan  = 13;
  country.policy = WIFI_COUNTRY_POLICY_AUTO;
  country.cc[0] = 'B';
  country.cc[1] = 'E';
  esp_err_t e;
  e = esp_wifi_set_country((const wifi_country_t *)&country);
  if(e == ESP_OK){
    LOG("[WiFi] Country code set to BE");
  } else {
    LOGE("[WiFi] Failed to set country code");
  }
  char cc[4] = {0};
  e = esp_wifi_get_country_code((char *)&cc);
  if(e == ESP_OK){
    LOG("[WiFi] Country code: %s", cc);
  } else {
    LOGE("[WiFi] Failed to get country code");
  }

  // Lower power to save battery and reduce interference, mostly reflections
  // due to bad antenna design?
  // See https://forum.arduino.cc/t/no-wifi-connect-with-esp32-c3-super-mini/1324046/12
  // See https://roryhay.es/blog/esp32-c3-super-mini-flaw
  //WiFi.setTxPower(WIFI_POWER_19_5dBm);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  // Get Tx power, map the enum properly
  uint8_t txp = WiFi.getTxPower();
  switch(txp){
    case WIFI_POWER_19_5dBm: txp=19; break; // 78,// 19.5dBm
    case WIFI_POWER_19dBm: txp=19; break; // 76,// 19dBm
    case WIFI_POWER_18_5dBm: txp=18; break; // 72,// 18.5dBm
    case WIFI_POWER_17dBm: txp=17; break; // 68,// 17dBm
    case WIFI_POWER_15dBm: txp=15; break; // 60,// 15dBm
    case WIFI_POWER_13dBm: txp=13; break; // 52,// 13dBm
    case WIFI_POWER_11dBm: txp=11; break; // 44,// 11dBm
    case WIFI_POWER_8_5dBm: txp=8; break; // 34,// 8.5dBm
    case WIFI_POWER_7dBm: txp=7; break; // 30,// 7dBm
    case WIFI_POWER_5dBm: txp=5; break; // 22,// 5dBm
    case WIFI_POWER_2dBm: txp=2; break; // 14,// 2dBm
    case WIFI_POWER_MINUS_1dBm: txp=-1; break; // 6,// -1dBm
    default: txp = 0; break;
  }
  LOG("[WiFi] Tx Power set to %d dBm", txp);

  // after WiFi.config()!
  LOG("[WiFi] Connecting to %s", cfg.wifi_ssid);
  uint8_t ok = 0;
  if(strlen(cfg.wifi_pass) == 0) {
    LOG("[WiFi] No password, connecting to open network");
    ok = WiFi.begin(cfg.wifi_ssid, NULL, 0, NULL, true);
  } else {
    LOG("[WiFi] Connecting with password");
    ok = WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass, 0, NULL, true);
  }
  if(ok != WL_CONNECTED && ok != WL_IDLE_STATUS){
    LOG("[WiFi] waiting for connection");
  } else {
    LOG("[WiFi] connected");
  }

  // setup NTP sync if needed
  #if defined(SUPPORT_WIFI) && defined(SUPPORT_NTP)
  setup_ntp();
  #endif
}
#endif // SUPPORT_WIFI

#ifdef SUPPORT_WIFI
void stop_networking(){
  LOG("[WiFi] Stop networking");
  // first stop WiFi
  WiFi.disconnect(true);
  while(WiFi.status() == WL_CONNECTED){
    doYIELD;
    LOG("[WiFi] waiting for disconnect, status: %d", WiFi.status());
    delay(100);
  }
  WiFi.mode(WIFI_MODE_NULL);
  #ifdef ARDUINO_ARCH_ESP32
  // Ensure WiFi is truly off on ESP32
  wifi_mode_t current_mode;
  esp_err_t err = esp_wifi_get_mode(&current_mode);
  if(err == ESP_OK) {
    // WiFi is initialized, stop it properly
    err = esp_wifi_stop();
    if(err == ESP_OK) {
      LOG("[WiFi] WiFi stopped successfully");
    } else {
      LOG("[WiFi] WiFi stop failed: 0x%x", err);
    }
    err = esp_wifi_deinit();
    if(err == ESP_OK) {
      LOG("[WiFi] WiFi deinitialized successfully");
    } else {
      LOG("[WiFi] WiFi deinit failed: 0x%x", err);
    }
  } else {
    LOG("[WiFi] WiFi was not initialized, skipping stop/deinit");
  }
  #endif
  LOG("[WiFi] Stop networking done");
}

void start_networking(){
  LOG("[WiFi] Start networking");
  // Check if WiFi is enabled before starting
  if(!cfg.wifi_enabled){
    LOG("[WiFi] WiFi is disabled, skipping networking start");
    WiFi.mode(WIFI_MODE_NULL);
    return;
  }
  // now reconnect to WiFi
  setup_wifi();
  LOG("[WiFi] Start networking done");
}

void reset_networking(){
  if(!cfg.wifi_enabled){
    LOG("[WiFi] WiFi is disabled, skipping networking reset");
    return;
  }
  if(strlen(cfg.wifi_ssid) != 0){
    LOG("[WiFi] resetting networking, SSID: %s", cfg.wifi_ssid);
  } else {
    LOG("[WiFi] resetting networking, no SSID configured");
    stop_networking();
    return;
  }
  #if defined(SUPPORT_WIFI) && defined(WIFI_WPS)
  if(wps_running){
      LOG("[WiFi] WPS is running, cannot reset networking now");
      return;
  }
  #endif // SUPPORT_WIFI && WIFI_WPS
  LOG("[WiFi] reset networking");
  // first stop WiFi
  stop_networking();
  // start networking
  start_networking();
  LOG("[WiFi] reset networking done");
}
#endif // SUPPORT_WIFI

NOINLINE
#ifdef SUPPORT_WIFI

void reconfigure_network_connections(){
  // Check if WiFi is enabled
  if(!cfg.wifi_enabled){
    LOG("[WiFi] WiFi is disabled, skipping network connections");
    return;
  }

  LOG("[WiFi] network connections, wifi status: %s", (WiFi.status() == WL_CONNECTED) ? "connected" : "not connected");
  if(WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS){
    // tcp - attempt both IPv4 and IPv6 connections based on target and available addresses
    #ifdef SUPPORT_TCP
    connect_tcp();
    #endif // SUPPORT_TCP

    #if defined(SUPPORT_WIFI) && defined(SUPPORT_TLS)
    connect_tls();
    #endif // SUPPORT_WIFI && SUPPORT_TLS

    #if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)
    stop_tcp_servers();
    start_tcp_servers();
    #endif
  } else {
    #if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)
    // WiFi not connected, stop TCP servers
    stop_tcp_servers();
    #endif
  }

  if(WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS){
    #ifdef SUPPORT_UDP
    // udp - attempt both IPv4 and IPv6 connections based on target and available addresses

    // in/out udp receive/send socket
    in_out_socket_udp();

    // receive-only udp socket (IPv4 only)
    in_socket_udp(udp_listen_sock, cfg.udp_listen_port?cfg.udp_listen_port:cfg.udp6_listen_port);

    // receive-only udp socket (IPv6 only)
    in_socket_udp6(udp6_listen_sock, cfg.udp6_listen_port?cfg.udp6_listen_port:cfg.udp_listen_port);

    // send-only udp socket
    out_socket_udp(udp_out_sock, cfg.udp_send_port, cfg.udp_send_ip);
    #endif // SUPPORT_UDP
  }
  return;
}

void stop_network_connections(){
  LOG("[WiFi] stop network connections");

  #ifdef SUPPORT_TCP
  close_tcp_socket();
  #endif // SUPPORT_TCP

  #if defined(SUPPORT_WIFI) && defined(SUPPORT_TLS)
  close_tls_connection();
  #endif // SUPPORT_WIFI && SUPPORT_TLS

  #ifdef SUPPORT_UDP
  close_udp_socket(udp_sock, "[UDP]");
  close_udp_socket(udp_listen_sock, "[UDP_LISTEN]");
  close_udp_socket(udp6_listen_sock, "[UDP6_LISTEN]");
  close_udp_socket(udp_out_sock, "[UDP_SEND]");
  #endif // SUPPORT_UDP

  #if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)
  stop_tcp_servers();
  #endif // SUPPORT_WIFI && SUPPORT_TCP_SERVER

  LOG("[WiFi] stop network connections done");
}
#else // !SUPPORT_WIFI
void reconfigure_network_connections(){
  // WiFi not supported, no network connections to configure
}

void stop_network_connections(){
  // WiFi not supported, no network connections to stop
}
#endif // SUPPORT_WIFI

// Helper function to get human-readable errno messages
NOINLINE
const char* get_errno_string(int err) {
  switch(err) {
    case EACCES: return "Permission denied";
    case EADDRINUSE: return "Address already in use";
    case EADDRNOTAVAIL: return "Address not available";
    case EAFNOSUPPORT: return "Address family not supported";
    case EAGAIN: return "Resource temporarily unavailable";
    case EALREADY: return "Operation already in progress";
    case EBADF: return "Bad file descriptor";
    case ECONNABORTED: return "Connection aborted";
    case ECONNREFUSED: return "Connection refused";
    case ECONNRESET: return "Connection reset";
    case EFAULT: return "Bad address";
    case EHOSTDOWN: return "Host is down";
    case EHOSTUNREACH: return "Host unreachable";
    case EINPROGRESS: return "Operation in progress";
    case EINTR: return "Interrupted system call";
    case EINVAL: return "Invalid argument";
    case EIO: return "I/O error";
    case EISCONN: return "Already connected";
    case EMFILE: return "Too many open files";
    case EMSGSIZE: return "Message too long";
    case ENETDOWN: return "Network is down";
    case ENETUNREACH: return "Network unreachable";
    case ENOBUFS: return "No buffer space available";
    case ENOMEM: return "Out of memory";
    case ENOTCONN: return "Not connected";
    case ENOTSOCK: return "Not a socket";
    case EPIPE: return "Broken pipe";
    case EPROTONOSUPPORT: return "Protocol not supported";
    case EPROTOTYPE: return "Protocol wrong type for socket";
    case ETIMEDOUT: return "Connection timed out";
    case ENFILE: return "Too many open files in system";
    default: return "Unknown error";
  }
}

#if defined(SUPPORT_WIFI) && (defined(SUPPORT_TCP) || defined(SUPPORT_UDP))
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <lwip/netdb.h>

// Helper: check if string is IPv6
NOINLINE
bool is_ipv6_addr(const char* ip) {
  return strchr(ip, ':') != NULL;
}

#endif // SUPPORT_WIFI && (SUPPORT_UDP || SUPPORT_TCP)

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP)
int tcp_sock = -1;
long last_tcp_check = 0;
uint8_t tcp_connection_writable = 0;
#endif // SUPPORT_WIFI && SUPPORT_TCP

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TLS)
WiFiClientSecure tls_client;
uint8_t tls_connected = 0;
uint8_t tls_handshake_complete = 0;
long last_tls_check = 0;
#endif // SUPPORT_WIFI && SUPPORT_TLS

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP)

void connect_tcp() {
  if(strlen(cfg.tcp_host_ip) == 0 || cfg.tcp_port == 0) {
    D("[TCP] Invalid TCP host IP or port, not setting up TCP");
    return;
  }
  if(tcp_sock >= 0)
    close_tcp_socket();

  // just as a test/debug, leave at 0
  uint8_t blocking_connect = 0;

  int r = 0;
  if(is_ipv6_addr(cfg.tcp_host_ip)) {
    // IPv6
    LOG("[TCP] setting up TCP/ipv6 to:%s, port:%hu", cfg.tcp_host_ip, cfg.tcp_port);
    struct sockaddr_in6 sa6;
    memset(&sa6, 0, sizeof(sa6));
    sa6.sin6_family = AF_INET6;
    sa6.sin6_port = htons(cfg.tcp_port);
    if (inet_pton(AF_INET6, cfg.tcp_host_ip, &sa6.sin6_addr) != 1) {
      LOG("[TCP] Invalid IPv6 address for TCP: %s", cfg.tcp_host_ip);
      return;
    }

    // socket
    tcp_sock = socket(AF_INET6, SOCK_STREAM, 0);
    if (tcp_sock == -1) {
      LOGE("[TCP] Failed to create IPv6 TCP socket");
      return;
    }

    // Set socket options
    int optval = 1;
    if (setsockopt(tcp_sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
      LOGE("[TCP] Failed to set SO_REUSEADDR");
      close(tcp_sock);
      tcp_sock = -1;
      return;
    }

    // Configure for IPv6-only (no IPv4 mapping)
    if (setsockopt(tcp_sock, IPPROTO_IPV6, IPV6_V6ONLY, &optval, sizeof(optval)) < 0) {
      LOGE("[TCP] Failed to set IPV6_V6ONLY");
      close(tcp_sock);
      tcp_sock = -1;
      return;
    }

    // set socket to non-blocking mode and read/write
    int flags = fcntl(tcp_sock, F_GETFL, 0);
    if (blocking_connect == 0)
      flags |= O_NONBLOCK;
    if (flags >= 0)
      fcntl(tcp_sock, F_SETFL, flags | O_RDWR);

    // connect
    r = connect(tcp_sock, (struct sockaddr*)&sa6, sizeof(sa6));

  } else {
    // IPv4
    LOG("[TCP] setting up TCP/ipv4 to: %s, port:%hu", cfg.tcp_host_ip, cfg.tcp_port);
    struct sockaddr_in sa4 = {0};
    sa4.sin_family = AF_INET;
    sa4.sin_port = htons(cfg.tcp_port);
    if (inet_pton(AF_INET, cfg.tcp_host_ip, &sa4.sin_addr) != 1) {
      LOG("[TCP] Invalid IPv4 address for TCP: %s", cfg.tcp_host_ip);
      return;
    }

    // socket
    tcp_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_sock < 0) {
      LOGE("[TCP] Failed to create IPv4 TCP socket");
      return;
    }

    // set socket to non-blocking mode and read/write
    int flags = fcntl(tcp_sock, F_GETFL, 0);
    if (blocking_connect == 0)
      flags |= O_NONBLOCK;
    if (flags >= 0)
      fcntl(tcp_sock, F_SETFL, flags | O_RDWR);

    // connect
    r = connect(tcp_sock, (struct sockaddr*)&sa4, sizeof(sa4));
  }

  // connect, this will be non-blocking, so we get a EINPROGRESS
  if (r == -1) {
    if(errno && errno != EINPROGRESS) {
      // If not EINPROGRESS, connection failed
      LOGE("[TCP] Failed to connect IPv6 TCP socket");
      close_tcp_socket();
      return;
    }
    uint8_t old_errno = errno;
    errno = 0; // clear errno after EINPROGRESS
    int optval, r_o, s_bufsize, r_bufsize;
    socklen_t optlen = sizeof(optval);
    optval = 1;
    r_o = setsockopt(tcp_sock, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen);
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP KEEPALIVE");
    optval = 1;
    r_o = setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPIDLE, &optval, optlen);
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP KEEPIDLE");
    optval = 1;
    r_o = setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPINTVL, &optval, optlen);
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP KEEPINTVL");
    optval = 1;
    r_o = setsockopt(tcp_sock, IPPROTO_TCP, TCP_KEEPCNT, &optval, optlen);
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP KEEPCNT");
    // set receive buffer size
    r_bufsize = 512;
    r_o = setsockopt(tcp_sock, SOL_SOCKET, SO_RCVBUF, &r_bufsize, sizeof(r_bufsize));
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP SO_RCVBUF");
    r_o = getsockopt(tcp_sock, SOL_SOCKET, SO_RCVBUF, &r_bufsize, &optlen);
    if (r_o < 0){
      LOGE("[TCP] Failed to get TCP SO_RCVBUF");
    } else {
      D("[TCP] TCP NEW SO_RCVBUF: %d", r_bufsize);
    }
    // disable Nagle's algorithm, send data immediately
    optval = 1;
    r_o = setsockopt(tcp_sock, IPPROTO_TCP, TCP_NODELAY, &optval, sizeof(optval));
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP NODELAY");
    // set recv/send timeout to 1 second
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    r_o = setsockopt(tcp_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP RCVTIMEO");
    optval = 1000; // milliseconds
    r_o = setsockopt(tcp_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP SNDTIMEO");
    errno = 0; // clear errno
    errno = old_errno; // restore old errno
    LOGE("[TCP] connection on fd:%d initiated to: %s, port:%hu", tcp_sock, cfg.tcp_host_ip, cfg.tcp_port);
    int flags = fcntl(tcp_sock, F_GETFL, 0);
    flags |= O_RDWR;
    flags |= O_NONBLOCK;
    if (flags >= 0)
      fcntl(tcp_sock, F_SETFL, flags);
    LOG("[TCP] connection in progress on fd:%d to %s, port:%hu", tcp_sock, cfg.tcp_host_ip, cfg.tcp_port);

    if(is_ipv6_addr(cfg.tcp_host_ip)) {
      // for debug, get local and peer address
      #ifdef DEBUG
      struct sockaddr_in6 l_sa6 = {0};
      socklen_t optlen = sizeof(l_sa6);
      if(getsockname(tcp_sock, (struct sockaddr*)&l_sa6, &optlen) == 0) {
        char local_addr_str[40] = {0};
        if(inet_ntop(AF_INET6, &l_sa6.sin6_addr, local_addr_str, sizeof(local_addr_str))) {
          D("[TCP] TCP local address: %s, port:%hu", local_addr_str, ntohs(l_sa6.sin6_port));
        }
      } else {
        E("[TCP] Failed to get local IPv6 TCP address");
      }
      struct sockaddr_in6 r_sa6;
      memset(&r_sa6, 0, sizeof(r_sa6));
      if(getpeername(tcp_sock, (struct sockaddr*)&r_sa6, &optlen) == 0) {
        char peer_addr_str[40] = {0};
        if(inet_ntop(AF_INET6, &r_sa6.sin6_addr, peer_addr_str, sizeof(peer_addr_str))) {
          D("[TCP] TCP peer address: %s, port:%hu", peer_addr_str, ntohs(r_sa6.sin6_port));
        }
      } else {
        E("[TCP] Failed to get peer IPv6 TCP address");
      }
      #endif // DEBUG
    } else {

      // for debug, get local and peer address
      #ifdef DEBUG
      struct sockaddr_in l_sa4;
      memset(&l_sa4, 0, sizeof(l_sa4));
      socklen_t optlen = sizeof(l_sa4);
      if(getsockname(tcp_sock, (struct sockaddr*)&l_sa4, &optlen) == 0) {
        char local_addr_str[16] = {0};
        if(inet_ntop(AF_INET, &l_sa4.sin_addr, local_addr_str, sizeof(local_addr_str))) {
          D("[TCP] TCP local address: %s, port:%hu", local_addr_str, ntohs(l_sa4.sin_port));
        }
      } else {
        E("[TCP] Failed to get local IPv4 TCP address");
      }
      struct sockaddr_in r_sa4;
      memset(&r_sa4, 0, sizeof(r_sa4));
      optlen = sizeof(r_sa4);
      if(getpeername(tcp_sock, (struct sockaddr*)&r_sa4, &optlen) == 0) {
        char peer_addr_str[16] = {0};
        if(inet_ntop(AF_INET, &r_sa4.sin_addr, peer_addr_str, sizeof(peer_addr_str))) {
          D("[TCP] TCP peer address: %s, port:%hu", peer_addr_str, ntohs(r_sa4.sin_port));
        }
      } else {
        E("[TCP] Failed to get peer IPv4 TCP address");
      }
      #endif // DEBUG

    }
    return;
  }
  LOG("[TCP] connected fd:%d to %s, port:%hu", tcp_sock, cfg.tcp_host_ip, cfg.tcp_port);
}


NOINLINE
void close_tcp_socket() {
  int fd_orig = tcp_sock;
  if (tcp_sock >= 0) {
    D("[TCP] closing socket %d", fd_orig);
    errno = 0;
    if (shutdown(tcp_sock, SHUT_RDWR) == -1) {
        if (errno && errno != ENOTCONN && errno != EBADF && errno != EINVAL)
            LOGE("[TCP] Failed to shutdown %d socket", fd_orig);
    }
    D("[TCP] socket %d shutdown", fd_orig);
    errno = 0;
    // now close the socket
    if (close(tcp_sock) == -1)
        if (errno && errno != EBADF && errno != ENOTCONN)
            LOGE("[TCP] Failed to close %d socket", fd_orig);
    tcp_sock = -1;
    LOG("[TCP] socket %d closed", fd_orig);
  }
}

// Helper: send TCP data (IPv4/IPv6)
int send_tcp_data(const uint8_t* data, size_t len) {
  D("[TCP] send_tcp_data len: %d", len);
  int result = send(tcp_sock, data, len, 0);
  if (result == -1 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINPROGRESS)) {
    // Would block, try again later
    return -1;
  }
  if (result == -1) {
    // Error occurred, close the socket and mark as invalid
    LOGE("[TCP] send error on socket %d: %d (%s)", tcp_sock, errno, get_errno_string(errno));
    close_tcp_socket();
    return -1;
  }
  if (result == 0) {
    // Connection closed by the remote host
    LOG("[TCP] connection closed by remote host on socket %d", tcp_sock);
    close_tcp_socket();
    return 0;
  }
  return result;
}

// Helper: receive TCP data (IPv4/IPv6)
int recv_tcp_data(uint8_t* buf, size_t maxlen) {
  // IPv6 socket (non-blocking)
  int result = recv(tcp_sock, buf, maxlen, 0);
  if (result == -1 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINPROGRESS)) {
    // No data available right now
    return -1;
  }
  if (result == -1) {
    LOG("[TCP] recv error on socket %d: %d (%s)", tcp_sock, errno, get_errno_string(errno));
    close_tcp_socket(); // Error occurred, close the socket and mark as invalid
    return -1;
  }
  if (result == 0) {
    LOG("[TCP] connection closed by remote host on socket %d", tcp_sock);
    close_tcp_socket(); // Connection closed by the remote host
    return 0;
  }
  return result;
}

// TCP Connection Check: Verify if TCP connection is still alive
int check_tcp_connection(unsigned int tm = 0) {
  if (strlen(cfg.tcp_host_ip) == 0 || cfg.tcp_port == 0) {
    // No TCP host configured
    return 0;
  }

  if (tcp_sock == -1){
    // No valid TCP host or connection needs to be established
    D("[TCP] No valid TCP host, attempting to establish connection");
    return 0;
  }

  // IPv6 socket: use select() to check if socket is ready for read/write
  fd_set readfds, writefds, errorfds;

  FD_ZERO(&readfds);
  FD_ZERO(&writefds);
  FD_ZERO(&errorfds);
  FD_SET(tcp_sock, &readfds);
  FD_SET(tcp_sock, &writefds);
  FD_SET(tcp_sock, &errorfds);

  // non-blocking select with 0 timeout
  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = tm;

  int ready = select(tcp_sock + 1, &readfds, &writefds, &errorfds, &timeout);
  if (ready < 0) {
    LOGE("[TCP] select error");
    close_tcp_socket();
    return 0;
  }

  if (FD_ISSET(tcp_sock, &errorfds)) {
    LOG("[TCP] socket %d has error, checking error", tcp_sock);

    // Check if socket is connected by trying to get socket error
    int socket_error = 0;
    socklen_t len = sizeof(socket_error);
    if (getsockopt(tcp_sock, SOL_SOCKET, SO_ERROR, &socket_error, &len) == 0) {
      if (socket_error != 0) {
        LOG("[TCP] socket %d error detected: %d (%s), reconnecting", tcp_sock, socket_error, get_errno_string(socket_error));
        close_tcp_socket();
        return 0;
      }
    } else {
      LOGE("[TCP] getsockopt failed on socket %d", tcp_sock);
    }
    LOG("[TCP] socket %d error but no error detected, assuming connection OK", tcp_sock);
    close_tcp_socket();
    return 0;
  }

  #ifdef DEBUG
  if (FD_ISSET(tcp_sock, &writefds)) {
    tcp_connection_writable = 1;
    D("[TCP] socket fd:%d writable, connection OK", tcp_sock);
  } else {
    tcp_connection_writable = 0;
    D("[TCP] socket fd:%d not yet writable", tcp_sock);
  }
  #endif
  return 1;
}
#endif // SUPPORT_WIFI && SUPPORT_TCP

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TLS)

// TLS/SSL Connection Management Functions

void connect_tls() {
  if(!cfg.tls_enabled) {
    D("[TLS] TLS is disabled");
    return;
  }
  if(strlen(cfg.tcp_host_ip) == 0 || (cfg.tcp_port == 0 && cfg.tls_port == 0)) {
    D("[TLS] Invalid TLS host IP or port, not setting up TLS");
    return;
  }

  uint16_t port_to_use = cfg.tls_port ? cfg.tls_port : cfg.tcp_port;

  LOG("[TLS] Setting up TLS connection to: %s, port: %d", cfg.tcp_host_ip, port_to_use);

  // Close existing connection if any
  if(tls_connected) {
    close_tls_connection();
  }

  // Configure TLS client
  if(strlen(cfg.tls_ca_cert) > 0) {
    tls_client.setCACert(cfg.tls_ca_cert);
    LOG("[TLS] CA certificate configured");
  } else {
    tls_client.setInsecure(); // Skip certificate verification
    LOG("[TLS] Using insecure mode (no certificate verification)");
  }

  if(strlen(cfg.tls_client_cert) > 0 && strlen(cfg.tls_client_key) > 0) {
    tls_client.setCertificate(cfg.tls_client_cert);
    tls_client.setPrivateKey(cfg.tls_client_key);
    LOG("[TLS] Client certificate and key configured");
  }

  if(strlen(cfg.tls_psk_identity) > 0 && strlen(cfg.tls_psk_key) > 0) {
    // PSK support may vary by platform
    #ifdef ARDUINO_ARCH_ESP32
    // ESP32 may support PSK - uncomment if available
    // tls_client.setPreSharedKey(cfg.tls_psk_identity, cfg.tls_psk_key);
    LOG("[TLS] PSK not implemented for ESP32");
    #elif defined(ARDUINO_ARCH_ESP8266)
    // ESP8266 may support PSK - uncomment if available
    // tls_client.setPreSharedKey(cfg.tls_psk_identity, cfg.tls_psk_key);
    LOG("[TLS] PSK not implemented for ESP8266");
    #else
    LOG("[TLS] PSK not supported on this platform");
    #endif
  }

  // Connect
  if(tls_client.connect(cfg.tcp_host_ip, port_to_use)) {
    tls_connected = 1;
    tls_handshake_complete = 1;
    LOG("[TLS] Connected successfully to %s:%d", cfg.tcp_host_ip, port_to_use);

    // Log connection info (cipher suite method varies by platform)
    #ifdef ARDUINO_ARCH_ESP32
    // ESP32 WiFiClientSecure methods
    LOG("[TLS] TLS connection established (ESP32)");
    #elif defined(ARDUINO_ARCH_ESP8266)
    // ESP8266 WiFiClientSecure methods
    LOG("[TLS] TLS connection established (ESP8266)");
    #else
    LOG("[TLS] TLS connection established");
    #endif
  } else {
    tls_connected = 0;
    tls_handshake_complete = 0;
    LOG("[TLS] Failed to connect to %s:%d", cfg.tcp_host_ip, port_to_use);
  }
}

void close_tls_connection() {
  if(tls_connected) {
    tls_client.stop();
    tls_connected = 0;
    tls_handshake_complete = 0;
    LOG("[TLS] Connection closed");
  }
}

// Helper: send TLS data
int send_tls_data(const uint8_t* data, size_t len) {
  if(!tls_connected || !tls_handshake_complete) {
    return -1;
  }

  size_t written = tls_client.write(data, len);
  if(written != len) {
    LOG("[TLS] Send incomplete: %d/%d bytes", written, len);
    if(written == 0) {
      // Connection might be closed
      if(!tls_client.connected()) {
        LOG("[TLS] Connection lost during send");
        close_tls_connection();
        return -1;
      }
    }
  }
  return written;
}

// Helper: receive TLS data
int recv_tls_data(uint8_t* buf, size_t maxlen) {
  if(!tls_connected || !tls_handshake_complete) {
    return -1;
  }

  if(!tls_client.available()) {
    return -1; // No data available
  }

  int result = tls_client.read(buf, maxlen);
  if(result <= 0) {
    // Check if connection is still alive
    if(!tls_client.connected()) {
      LOG("[TLS] Connection lost during receive");
      close_tls_connection();
      return -1;
    }
  }
  return result;
}

// TLS Connection Check: Verify if TLS connection is still alive
int check_tls_connection() {
  if(!cfg.tls_enabled || strlen(cfg.tcp_host_ip) == 0 || (cfg.tcp_port == 0 && cfg.tls_port == 0)) {
    return 0;
  }

  if(!tls_connected) {
    D("[TLS] No TLS connection, attempting to establish");
    return 0;
  }

  if(!tls_client.connected()) {
    LOG("[TLS] Connection lost, closing");
    close_tls_connection();
    return 0;
  }

  return 1;
}

#endif // SUPPORT_WIFI && SUPPORT_TLS

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)

// support up to 8 clients
#define TCP_CLIENTS_MAX 8
int tcp_server_clients[TCP_CLIENTS_MAX] = {-1, -1, -1, -1, -1, -1, -1, -1};

// TCP Server functions
void start_tcp_server() {
  if(cfg.tcp_server_port == 0 && cfg.tcp6_server_port == 0) {
    D("[TCP_SERVER] TCP server port not configured");
    return;
  }

  uint16_t port_to_use = cfg.tcp_server_port ? cfg.tcp_server_port : cfg.tcp6_server_port;
  if(tcp_server_sock != -1) {
    LOG("[TCP_SERVER] TCP server already running on port %hu", port_to_use);
    return;
  }
  LOG("[TCP_SERVER] Starting TCP server on port %hu", port_to_use);

  // Create socket (supports both IPv4 and IPv6)
  tcp_server_sock = socket(AF_INET, SOCK_STREAM, 0);
  if (tcp_server_sock == -1) {
    LOGE("[TCP_SERVER] Failed to create server socket");
    return;
  }

  // Set socket options
  int optval = 1;
  if (setsockopt(tcp_server_sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
    LOGE("[TCP_SERVER] Failed to set SO_REUSEADDR");
    close(tcp_server_sock);
    tcp_server_sock = -1;
    return;
  }

  // Set non-blocking
  int flags = fcntl(tcp_server_sock, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(tcp_server_sock, F_SETFL, flags | O_NONBLOCK);
  }

  // Bind to port
  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr = in_addr{.s_addr = INADDR_ANY};
  server_addr.sin_port = htons(port_to_use);

  if (bind(tcp_server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    LOGE("[TCP_SERVER] Failed to bind to port %hu", port_to_use);
    close(tcp_server_sock);
    tcp_server_sock = -1;
    return;
  }

  // Start listening
  if (listen(tcp_server_sock, cfg.tcp_server_max_clients) < 0) {
    LOGE("[TCP_SERVER] Failed to listen on port %hu", port_to_use);
    close(tcp_server_sock);
    tcp_server_sock = -1;
    return;
  }

  LOG("[TCP_SERVER] TCP server started successfully on port %hu", port_to_use);
}

void stop_tcp_server() {
  if(tcp_server_sock == -1)
    return;
  LOG("[TCP_SERVER] Stopping TCP server on port %hu", cfg.tcp_server_port);
  close(tcp_server_sock);
  tcp_server_sock = -1;
}

void handle_tcp_server() {
  if(tcp_server_sock == -1)
    return;

  // Accept new connections
  struct sockaddr_in client_addr;
  socklen_t client_len = sizeof(client_addr);
  int new_client = accept(tcp_server_sock, (struct sockaddr*)&client_addr, &client_len);
  if(new_client >= 0) {
    // Find empty slot for new client
    int slot = -1;
    for(int i = 0; i < cfg.tcp_server_max_clients && i < TCP_CLIENTS_MAX; i++) {
      if(tcp_server_clients[i] == -1) {
        slot = i;
        break;
      }
    }

    if(slot >= 0) {
      // Set client socket to non-blocking
      int flags = fcntl(new_client, F_GETFL, 0);
      if (flags >= 0) {
        fcntl(new_client, F_SETFL, flags | O_NONBLOCK);
      }

      // Log client connection
      tcp_server_clients[slot] = new_client;
      char client_ip[INET_ADDRSTRLEN] = {0};
      inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
      LOG("[TCP_SERVER] New client connected from [%s]:%hu in slot %d, fd:%d",
        client_ip, ntohs(client_addr.sin_port), slot, new_client);

    } else {
      // No available slots, reject connection
      LOG("[TCP_SERVER] Connection rejected - server full");
      close(new_client);
    }
  }
}

// Send data to all connected TCP server clients
int send_tcp_server_data(const uint8_t* data, size_t len) {
  int clients_sent = 0;
  for(int i = 0; i < cfg.tcp_server_max_clients && i < TCP_CLIENTS_MAX; i++) {
    if(tcp_server_clients[i] == -1)
      continue;
    errno = 0;
    int result = send(tcp_server_clients[i], data, len, 0);
    if(result > 0) {
      clients_sent++;
    } else if (result == -1) {
      if(errno != EAGAIN && errno != EWOULDBLOCK) {
        // Client connection error
        LOG("[TCP_SERVER] Error sending to client %d fd:%d, disconnecting", i, tcp_server_clients[i]);
        close(tcp_server_clients[i]);
        tcp_server_clients[i] = -1;
      } else {
        // Would block, try again later
        LOOP_E("[TCP_SERVER] Would block sending to client %d fd:%d, try again later", i, tcp_server_clients[i]);
        continue;
      }
    } else if(result == 0) {
      // Client disconnected
      LOGE("[TCP_SERVER] Client %d fd:%d disconnected [SEND]", i, tcp_server_clients[i]);
      close(tcp_server_clients[i]);
      tcp_server_clients[i] = -1;
    }
  }
  return clients_sent;
}

// Receive data from all connected TCP server clients
int recv_tcp_server_data(uint8_t* buf, size_t maxlen) {
  // TODO: randomize client order to avoid starvation
  for(int i = 0; i < cfg.tcp_server_max_clients && i < TCP_CLIENTS_MAX; i++) {
    if(tcp_server_clients[i] == -1)
      continue;
    errno = 0;
    int bytes_received = recv(tcp_server_clients[i], buf, maxlen, MSG_DONTWAIT);
    if(bytes_received > 0) {
      return bytes_received; // Return data from the first client that has data
    } else if(bytes_received == -1){
      if(errno != EAGAIN && errno != EWOULDBLOCK){
        // Client connection error
        LOG("[TCP_SERVER] Error receiving from client %d fd:%d, disconnecting", i, tcp_server_clients[i]);
        close(tcp_server_clients[i]);
        tcp_server_clients[i] = -1;
      } else {
        // No data available right now
        LOOP_E("[TCP_SERVER] No data available from client %d fd:%d right now", i, tcp_server_clients[i]);
        continue;
      }
    } else if(bytes_received == 0) {
      // Client disconnected
      LOGE("[TCP_SERVER] Client %d fd:%d disconnected [RECV]", i, tcp_server_clients[i]);
      close(tcp_server_clients[i]);
      tcp_server_clients[i] = -1;
    }
  }
  return -1; // No data received
}

// Get number of connected TCP server clients
int get_tcp_server_client_count() {
  int count = 0;
  for(int i = 0; i < cfg.tcp_server_max_clients && i < TCP_CLIENTS_MAX; i++) {
    if(tcp_server_clients[i] != -1)
      count++;
  }
  return count;
}

// TCP6 Server functions (IPv6-only)
void start_tcp6_server() {
  if(cfg.tcp6_server_port == 0 && cfg.tcp_server_port == 0) {
    D("[TCP6_SERVER] TCP6 server port not configured");
    return;
  }

  uint16_t port_to_use = cfg.tcp6_server_port ? cfg.tcp6_server_port : cfg.tcp_server_port;
  if(tcp6_server_sock != -1) {
    LOG("[TCP6_SERVER] TCP6 server already running on port %hu", port_to_use);
    return;
  }
  LOG("[TCP6_SERVER] Starting TCP6 server on port %hu", port_to_use);

  // Create IPv6-only socket
  tcp6_server_sock = socket(AF_INET6, SOCK_STREAM, 0);
  if (tcp6_server_sock == -1) {
    LOGE("[TCP6_SERVER] Failed to create server socket");
    return;
  }

  // Set socket options
  int optval = 1;
  if (setsockopt(tcp6_server_sock, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) < 0) {
    LOGE("[TCP6_SERVER] Failed to set SO_REUSEADDR");
    close(tcp6_server_sock);
    tcp6_server_sock = -1;
    return;
  }

  // Configure for IPv6-only (no IPv4 mapping)
  if (setsockopt(tcp6_server_sock, IPPROTO_IPV6, IPV6_V6ONLY, &optval, sizeof(optval)) < 0) {
    LOGE("[TCP6_SERVER] Failed to set IPV6_V6ONLY");
    close(tcp6_server_sock);
    tcp6_server_sock = -1;
    return;
  }

  // Set non-blocking
  int flags = fcntl(tcp6_server_sock, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(tcp6_server_sock, F_SETFL, flags | O_NONBLOCK);
  }

  // Bind to port
  struct sockaddr_in6 server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin6_family = AF_INET6;
  server_addr.sin6_addr = in6addr_any;  // Listen on all IPv6 interfaces
  server_addr.sin6_port = htons(port_to_use);

  if (bind(tcp6_server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    LOGE("[TCP6_SERVER] Failed to bind to port %hu", port_to_use);
    close(tcp6_server_sock);
    tcp6_server_sock = -1;
    return;
  }

  // Start listening
  if (listen(tcp6_server_sock, cfg.tcp_server_max_clients) < 0) {
    LOGE("[TCP6_SERVER] Failed to listen on port %hu", port_to_use);
    close(tcp6_server_sock);
    tcp6_server_sock = -1;
    return;
  }

  LOG("[TCP6_SERVER] TCP6 server started successfully on port %hu", port_to_use);
}

void stop_tcp6_server() {
  if(tcp6_server_sock != -1)
    return;
  // Close server socket
  LOG("[TCP6_SERVER] Stopping TCP6 server on port %hu", cfg.tcp6_server_port);
  close(tcp6_server_sock);
  tcp6_server_sock = -1;
}

void handle_tcp6_server() {
  if(tcp6_server_sock == -1)
    return;

  // Accept new connections
  struct sockaddr_in6 client_addr;
  socklen_t client_len = sizeof(client_addr);
  int new_client = accept(tcp6_server_sock, (struct sockaddr*)&client_addr, &client_len);
  if(new_client >= 0) {
    // Find available slot for new client
    int slot = -1;
    for(int i = 0; i < cfg.tcp_server_max_clients && i < TCP_CLIENTS_MAX; i++) {
      if(tcp_server_clients[i] == -1) {
        slot = i;
        break;
      }
    }

    if(slot >= 0) {
      // Set non-blocking for client socket
      int flags = fcntl(new_client, F_GETFL, 0);
      if (flags >= 0) {
        fcntl(new_client, F_SETFL, flags | O_NONBLOCK);
      }

      // Log client connection
      tcp_server_clients[slot] = new_client;
      char client_ip[INET6_ADDRSTRLEN] = {0};
      inet_ntop(AF_INET6, &client_addr.sin6_addr, client_ip, INET6_ADDRSTRLEN);
      LOG("[TCP6_SERVER] New client connected from [%s]:%hu in slot %d, fd:%d",
        client_ip, ntohs(client_addr.sin6_port), slot, new_client);

      #ifdef LED
      last_tcp_activity = millis();
      #endif
    } else {
      LOG("[TCP6_SERVER] No available slots for new client, rejecting");
      close(new_client);
    }
  }
}

void handle_tcp_server_disconnects(){
  // handle disconnects
  for(int i = 0; i < cfg.tcp_server_max_clients && i < TCP_CLIENTS_MAX; i++) {
    if(tcp_server_clients[i] == -1)
      continue;
    int socket_error = 0;
    socklen_t len = sizeof(socket_error);
    if (getsockopt(tcp_server_clients[i], SOL_SOCKET, SO_ERROR, &socket_error, &len) == 0) {
      if (socket_error != 0) {
        LOG("[TCP_SERVER] socket %d, fd:%d error detected: %d (%s), reconnecting", i, tcp_server_clients[i], socket_error, get_errno_string(socket_error));
        close(tcp_server_clients[i]);
        tcp_server_clients[i] = -1;
        continue;
      }
    } else {
      LOGE("[TCP_SERVER] getsockopt failed on socket %d", tcp_server_clients[i]);
    }
  }
}

void start_tcp_servers(){
  // Initialize client slots
  for(int i = 0; i < TCP_CLIENTS_MAX; i++) {
    if(tcp_server_clients[i] != -1)
      close(tcp_server_clients[i]);
    tcp_server_clients[i] = -1;
  }

  start_tcp_server();
  start_tcp6_server();
}

void stop_tcp_servers(){
  // Close all client connections
  for(int i = 0; i < TCP_CLIENTS_MAX; i++) {
    if(tcp_server_clients[i] == -1)
      continue;
    close(tcp_server_clients[i]);
    tcp_server_clients[i] = -1;
  }

  stop_tcp_server();
  stop_tcp6_server();
}
#endif // SUPPORT_WIFI && SUPPORT_TCP_SERVER

#if defined(SUPPORT_WIFI) && defined(SUPPORT_UDP)

#define UDP_READ_MSG_SIZE 512

void in_out_socket_udp() {
  if(strlen(cfg.udp_host_ip) == 0 || cfg.udp_port == 0) {
    LOG("[UDP] No valid UDP host IP or port, not setting up UDP");
    close_udp_socket(udp_sock, "[UDP]");
    return;
  }
  char *d_ip = cfg.udp_host_ip;
  int16_t port = cfg.udp_port;
  LOG("[UDP] setting up UDP to:%s, port:%hu", d_ip, port);
  if(is_ipv6_addr(d_ip)) {
    if(!udp_socket(udp_sock, 1, "[UDP]"))
      return;

    // local IPv6
    struct sockaddr_in6 l_sa6;
    memset(&l_sa6, 0, sizeof(l_sa6));
    l_sa6.sin6_family = AF_INET6;
    l_sa6.sin6_port = htons(port);
    l_sa6.sin6_addr = in6addr_any;
    if (bind(udp_sock, (const sockaddr *)&l_sa6, sizeof(l_sa6)) == -1) {
      LOGE("[UDP] Failed to bind UDP socket fd:%d to %s:%hu", udp_sock, d_ip, port);
      close_udp_socket(udp_sock, "[UDP]");
      return;
    }
  } else {
    if(!udp_socket(udp_sock, 0, "[UDP]"))
      return;

    // local IPv4 listen IP
    struct sockaddr_in l_sa4;
    memset(&l_sa4, 0, sizeof(l_sa4));
    l_sa4.sin_family = AF_INET;
    l_sa4.sin_port = htons(port);
    l_sa4.sin_addr = in_addr{.s_addr = INADDR_ANY};
    if (bind(udp_sock, (const sockaddr *)&l_sa4, sizeof(l_sa4)) == -1) {
      LOGE("[UDP] Failed to bind UDP socket fd:%d to %s:%hu", udp_sock, d_ip, port);
      close_udp_socket(udp_sock, "[UDP]");
      return;
    }
  }
  LOG("[UDP] UDP socket fd:%d setup to %s:%hu", udp_sock, d_ip, port);
}

void in_socket_udp(int &fd, int16_t port) {
  if(port == 0){
    LOG("[UDP_LISTEN] No UDP listening port configured, disable");
    close_udp_socket(fd, "[UDP_LISTEN]");
    return;
  }

  // Setup listening socket
  LOG("[UDP_LISTEN] setting up UDP listening on port:%hu", port);
  if(!udp_socket(fd, 0, "[UDP_LISTEN]")){
    LOGE("[UDP_LISTEN] Failed to create UDP listening socket, IPv4");
    return;
  }

  struct sockaddr_in t_sa4;
  memset(&t_sa4, 0, sizeof(t_sa4));
  t_sa4.sin_family = AF_INET;
  t_sa4.sin_port = htons(port);
  t_sa4.sin_addr = in_addr{.s_addr = INADDR_ANY};

  if(bind(fd, (struct sockaddr*)&t_sa4, sizeof(t_sa4)) < 0) {
    LOGE("[UDP_LISTEN] Failed to bind ipv4 UDP listening on port:%hu", port);
    close_udp_socket(fd, "[UDP_LISTEN]");
    return;
  }
  LOG("[UDP_LISTEN] UDP listening socket fd:%d setup on port:%hu", fd, port);
}

void in_socket_udp6(int &fd, int16_t port) {
  if(port == 0){
    LOG("[UDP6_LISTEN] No UDP6 listening port configured, disable");
    close_udp_socket(fd, "[UDP6_LISTEN]");
    return;
  }

  // Setup IPv6-only listening socket
  LOG("[UDP6_LISTEN] setting up UDP6 listening on port:%hu", port);
  if(!udp_socket(fd, 1, "[UDP6_LISTEN]")){
    LOGE("[UDP6_LISTEN] Failed to create UDP6 listening socket");
    return;
  }

  // Set IPv6-only mode
  int optval = 1;
  if (setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &optval, sizeof(optval)) < 0) {
    LOGE("[UDP6_LISTEN] Failed to set IPV6_V6ONLY");
    close_udp_socket(fd, "[UDP6_LISTEN]");
    return;
  }

  struct sockaddr_in6 t_sa6;
  memset(&t_sa6, 0, sizeof(t_sa6));
  t_sa6.sin6_family = AF_INET6;
  t_sa6.sin6_port = htons(port);
  t_sa6.sin6_addr = in6addr_any;

  if(bind(fd, (struct sockaddr*)&t_sa6, sizeof(t_sa6)) < 0) {
    LOGE("[UDP6_LISTEN] Failed to bind ipv6 UDP listening on port:%hu", port);
    close_udp_socket(fd, "[UDP6_LISTEN]");
    return;
  }
  LOG("[UDP6_LISTEN] UDP6 listening socket fd:%d setup on port:%hu", fd, port);
}

void out_socket_udp(int &fd, int16_t port, const char* ip) {
  if(port == 0 || ip == NULL || strlen(ip) == 0){
    // No sending port or IP configured
    close_udp_socket(fd, "[UDP_SEND]");
    LOG("[UDP_SEND] No UDP send IP or port configured, disable");
    return;
  }

  // Setup sending socket, no need to bind, we just prepare sa6/sa4/sa for sendto()
  LOG("[UDP_SEND] setting up UDP sending to: %s:%hu", ip, port);
  if(is_ipv6_addr(ip)) {
    // IPv6
    if(!udp_socket(fd, 1, "[UDP_SEND]")){
      LOGE("[UDP_SEND] Failed to create UDP sending socket, IPv6");
      return;
    }
  } else {
    // IPv4
    if(!udp_socket(fd, 0, "[UDP_SEND]")){
      LOGE("[UDP_SEND] Failed to create UDP sending socket, IPv4");
      return;
    }
  }
  LOG("[UDP_SEND] UDP sending socket fd:%d setup to %s:%hu", fd, ip, port);
}

uint8_t udp_socket(int &fd, uint8_t ipv6, const char* tag) {

  // Close any existing socket
  close_udp_socket(fd, tag);

  // Socket
  if(ipv6) {
    // IPv6
    fd = socket(AF_INET6, SOCK_DGRAM, 0);
    if (fd < 0) {
      LOGE("%s Failed to create IPv6 socket", tag);
      return 0;
    }
    LOG("%s socket ipv6 on fd:%d", tag, fd);
  } else {
    // IPv4
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
      LOGE("%s Failed to create IPv4 socket", tag);
      return 0;
    }
    LOG("%s socket ipv4 on fd:%d", tag, fd);
  }
  if(fcntl(fd, F_SETFL, O_NONBLOCK | O_RDWR) < 0) {
    LOGE("%s Failed to set UDP socket to non-blocking", tag);
    close_udp_socket(fd, tag);
    return 0;
  }
  if(setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char[]){1}, sizeof(int)) < 0) {
    LOGE("%s Failed to set SO_REUSEADDR on UDP socket", tag);
    close_udp_socket(fd, tag);
    return 0;
  }

  // all is well
  return 1;
}


void close_udp_socket(int &fd, const char* tag) {
  if(fd < 0)
    return;

  // close()
  int fd_orig = fd;
  if(errno){
    LOGE("%s closing UDP socket fd:%d", tag, fd);
  } else {
    LOG("%s closing UDP socket fd:%d", tag, fd);
  }
  if (close(fd) == -1)
    if (errno && errno != EBADF)
      LOGE("%s Failed to close socket fd:%d", tag, fd_orig);
  fd = -1;
}

// Helper: send UDP data (IPv4/IPv6)
int send_udp_data(int &fd, const uint8_t* data, size_t len, char *d_ip, uint16_t port, char *tag) {
  D("%s Sending %d bytes to %s, fd:%d, port:%hu", tag, len, d_ip, fd, port);
  if(fd < 0)
    return -1;

  struct sockaddr_in s_sa4 = {0};
  struct sockaddr_in6 s_sa6 = {0};
  struct sockaddr *s_sa = NULL;
  size_t s_sa_sz = 0;
  if(is_ipv6_addr(d_ip)) {
    s_sa_sz = sizeof(s_sa6);
    s_sa6.sin6_family = AF_INET6;
    s_sa6.sin6_port = htons(port);
    s_sa = (struct sockaddr*)&s_sa6;
    if (inet_pton(AF_INET6, d_ip, &s_sa6.sin6_addr) != 1) {
      LOG("[UDP] Invalid IPv6 address:%s", d_ip);
      close_udp_socket(fd, tag);
      return -1;
    }
  } else {
    s_sa_sz = sizeof(s_sa4);
    s_sa4.sin_family = AF_INET;
    s_sa4.sin_port = htons(port);
    s_sa = (struct sockaddr*)&s_sa4;
    if (inet_pton(AF_INET, d_ip, &s_sa4.sin_addr) != 1) {
      LOG("[UDP] Invalid IPv4 address:%s", d_ip);
      close_udp_socket(fd, tag);
      return -1;
    }
  }
  size_t n = sendto(fd, data, len, 0, s_sa, s_sa_sz);
  if (n == -1) {
    LOGE("%s sendto failed to %s, port:%hu on fd:%d", tag, d_ip, port, fd);
    close_udp_socket(fd, "[UDP]");
    return -1;
  } else if (n == 0) {
    D("%s send returned 0 bytes, no data sent", tag);
    return 0;
  } else {
    D("%s send_udp_data len: %d, sent: %d", tag, len, n);
  }
  return n;
}

// Helper: receive UDP data (IPv4/IPv6)
int recv_udp_data(int &fd, uint8_t* buf, size_t maxlen) {
  size_t n = recv(fd, buf, maxlen, 0);
  if (n == -1) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      LOOP_E("[UDP] No data available on fd:%d, port:%hu", fd, cfg.udp_port);
      return 0;
    } else {
      LOGE("[UDP] recv failed from fd:%d, port:%hu", fd, cfg.udp_port);
      close_udp_socket(fd, "[UDP]");
      return -1;
    }
  } else if (n == 0) {
    D("[UDP] receive returned 0 bytes, no data received on fd:%d,port:%hu", fd, cfg.udp_port);
  }
  return n;
}

void udp_read(int fd, uint8_t *buf, size_t &len, size_t read_size, size_t maxlen, const char *tag) {
  // ok file descriptor?
  if(fd < 0)
    return;

  // space in outbuf?
  if (len + read_size >= maxlen) {
    D("%s outbuf full, cannot read more data, len: %d, read_size: %d, bufsize: %d", tag, len, read_size, maxlen);
    // no space in outbuf, cannot read more data
    // just yield and wait for outbuf to be cleared
    return;
  }

  // read data
  LOOP_D("%s Receiving up to %d bytes on fd:%d, port:%hu", tag, read_size, fd, cfg.udp_port);
  int os = recv_udp_data(fd, buf + len, read_size);
  if (os > 0) {
    #ifdef LED
    last_udp_activity = millis(); // Trigger LED activity for UDP receive
    #endif // LED
    D("%s Received %d bytes, total: %d, data: >>%s<<", tag, os, len + os, buf);
    len += os;
    return;
  } else if (os < 0) {
    if(errno && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINPROGRESS){
      // Error occurred, log it
      LOGE("%s receive error, closing connection", tag);
    }
  } else {
    // No data available, just yield
    LOOP_D("%s no data available, yielding...", tag);
  }
  return;
}
#endif // SUPPORT_WIFI && SUPPORT_UDP

void(* resetFunc)(void) = 0;

// Power-efficient sleep function for battery usage optimization
void power_efficient_sleep(uint32_t sleep_ms) {
  if (sleep_ms == 0)
    return;

  // The esp32c3 is however a single core esp32, so we need yield there as well
  #ifdef ARDUINO_ARCH_ESP32
    // Use light sleep mode on ESP32 for better battery efficiency
    // Light sleep preserves RAM and allows faster wake-up
    //esp_sleep_enable_timer_wakeup(sleep_ms * 1000); // Convert ms to microseconds
    //esp_light_sleep_start();
    // However, light sleep disconnects WiFi, so we use delay with yield instead
    delay(sleep_ms);
  #elif defined(ARDUINO_ARCH_ESP8266)
    // For ESP8266, use a combination of delay and yield for better power efficiency
    // Note: True light sleep on ESP8266 would disconnect WiFi, which we want to avoid
    uint32_t sleep_chunks = sleep_ms / 10; // Sleep in 10ms chunks
    uint32_t remainder = sleep_ms % 10;

    for (uint32_t i = 0; i < sleep_chunks; i++) {
      delay(10);
      yield(); // Allow WiFi and other background tasks to run
    }

    if (remainder > 0) {
      delay(remainder);
      yield();
    }
  #else
    // Fallback to regular delay for other platforms
    delay(sleep_ms);
  #endif
}

#if defined(BLUETOOTH_UART_AT) && defined(BT_BLE)
char* at_cmd_check(const char *cmd, const char *at_cmd, unsigned short at_len){
  unsigned short l = strlen(cmd); /* AT+<cmd>=, or AT, or AT+<cmd>? */
  if(at_len >= l && strncmp(cmd, at_cmd, l) == 0){
    if(*(cmd+l-1) == '='){
      return (char *)at_cmd+l;
    } else {
      return (char *)at_cmd;
    }
  }
  return NULL;
}
#endif

#if defined(BT_CLASSIC) || defined(UART_AT)
void sc_cmd_handler(SerialCommands* s, const char* atcmdline){
  D("SC: [%s]", atcmdline);
  const char *r = at_cmd_handler(atcmdline);
  if(r != NULL && strlen(r) > 0)
    s->GetSerial()->println(r);
}
#endif // BT_CLASSIC || UART_AT

#if defined(BLUETOOTH_UART_AT) && defined(BT_BLE)
char obuf[10] = {0}; // for utoa
#define AT_R_OK     (const char*)("OK")
#define AT_R(M)     (const char*)(M)
#define AT_R_STR(M) (const char*)(M)
#define AT_R_S(M)   (const char*)(M).c_str()
#define AT_R_INT(M) (const char*)utoa(M, (char *)&obuf, 10)
#define AT_R_F(M)   (const char*)(M)

const char *AT_short_help_string = R"EOF(Available AT Commands:
AT
AT?
AT+?
AT+HELP?
AT+RESET
AT+ERASE=|1
)EOF"

#ifdef SUPPORT_WIFI
R"EOF(AT+WIFI_SSID=|?
AT+WIFI_PASS=
AT+WIFI_STATUS?
AT+WIFI_ENABLED=|?
AT+HOSTNAME=
AT+IPV4=
AT+IPV6=
AT+IP_STATUS?
)EOF"

#if defined(SUPPORT_WPS)
R"EOF(AT+WPS_PBC
AT+WPS_PIN=
AT+WPS_STOP
AT+WPS_STATUS?)EOF"
#endif

#ifdef SUPPORT_MDNS
R"EOF(AT+MDNS=|?
AT+MDNS_HOSTNAME=|?
AT+MDNS_STATUS?)EOF"
#endif

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP)
R"EOF(AT+TCP_PORT=|?
AT+TCP_HOST_IP=|?
AT+TCP_STATUS?
)EOF"
#endif

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TLS)
R"EOF(AT+TLS_ENABLE=|?
AT+TLS_PORT=|?
AT+TLS_VERIFY=|?
AT+TLS_SNI=|?
AT+TLS_CA_CERT=|?
AT+TLS_CLIENT_CERT=|?
AT+TLS_CLIENT_KEY=|?
AT+TLS_PSK_IDENTITY=|?
AT+TLS_PSK_KEY=|?
AT+TLS_STATUS?
AT+TLS_CONNECT
AT+TLS_DISCONNECT
)EOF"
#endif

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)
R"EOF(AT+TCP_SERVER_PORT=|?
AT+TCP_SERVER_MAX_CLIENTS=|?
AT+TCP_SERVER_STATUS?
AT+TCP_SERVER_START
AT+TCP_SERVER_STOP
AT+TCP_SERVER_SEND=
)EOF"
#endif

#if defined(SUPPORT_WIFI) && defined(SUPPORT_UDP)
R"EOF(AT+UDP_PORT=|?
AT+UDP_HOST_IP=|?
AT+UDP_LISTEN_PORT=|?
AT+UDP6_LISTEN_PORT=|?
AT+UDP_SEND=|?
)EOF"
#endif

#if defined(SUPPORT_WIFI) && defined(SUPPORT_NTP)
R"EOF(AT+NTP_HOST=|?
AT+NTP_STATUS?
)EOF"
#endif

#endif // SUPPORT_WIFI

#ifdef SUPPORT_UART1
R"EOF(AT+UART1=|?
)EOF"
#endif

#ifdef VERBOSE
R"EOF(AT+VERBOSE=|?
)EOF"
#endif

#ifdef TIMELOG
R"EOF(AT+TIMELOG=|?
)EOF"
#endif

#ifdef LOGUART
R"EOF(AT+LOG_UART=|?
)EOF"
#endif

#ifdef LOOP_DELAY
R"EOF(AT+LOOP_DELAY=|?
)EOF"
#endif

#if defined(BLUETOOTH_UART_AT) || defined(SUPPORT_BLE_UART1)
R"EOF(AT+BLE_PIN=|?
AT+BLE_SECURITY=|?
AT+BLE_IO_CAP=|?
AT+BLE_AUTH_REQ=|?
AT+BLE_ADDR_TYPE=|?
AT+BLE_ADDR=|?
AT+BLE_ADDR_GEN?
AT+BLE_STATUS?
AT+BLE_UART1=|?
AT+BLE_UART1_PASS=|?
)EOF"
#endif

R"EOF(
Use AT+HELP? for detailed help
)EOF";

const char AT_help_string[] = R"EOF(
ESP-AT Command Help:

Basic Commands:
  AT                            - Test AT startup
  AT?                           - Test AT startup
  AT+?                          - Show this help
  AT+HELP?                      - Show this help
  AT+RESET                      - Restart device
  AT+ERASE                      - Erase all configuration, reset to factory defaults
  AT+ERASE=1                    - Erase all configuration and restart immediately)EOF"

#ifdef SUPPORT_WIFI
R"EOF(
WiFi Commands:
  AT+WIFI_ENABLED=<1|0>         - Enable/Disable WiFi (1=enable, 0=disable)
  AT+WIFI_ENABLED?              - Get WiFi enable status
  AT+WIFI_SSID=<ssid>           - Set WiFi SSID
  AT+WIFI_SSID?                 - Get WiFi SSID
  AT+WIFI_PASS=<pass>           - Set WiFi password
  AT+WIFI_STATUS?               - Get WiFi connection status
  AT+HOSTNAME=<name>            - Set device hostname
  AT+HOSTNAME?                  - Get device hostname
Network Commands:
  AT+IPV4=<config>              - Set IPv4 config (DHCP/DISABLE/ip,mask,gw[,dns])
  AT+IPV4?                      - Get IPv4 configuration
  AT+IPV6=<config>              - Set IPv6 configuration
  AT+IPV6?                      - Get IPv6 configuration
  AT+IP_STATUS?                 - Get current IP addresses)EOF"
#endif // SUPPORT_WIFI

#ifdef WIFI_WPS
R"EOF(
WPS Commands:
  AT+WPS_PBC                    - Start WPS Push Button Configuration
  AT+WPS_PIN=<pin>              - Start WPS PIN method
  AT+WPS_STOP                   - Stop WPS
  AT+WPS_STATUS?                - Get WPS status)EOF"
#endif // WIFI_WPS

#ifdef SUPPORT_MDNS
R"EOF(
mDNS Commands:
  AT+MDNS=<0|1>                 - Enable/disable mDNS responder
  AT+MDNS?                      - Get mDNS responder status
  AT+MDNS_HOSTNAME=<name>       - Set mDNS hostname (defaults to hostname)
  AT+MDNS_HOSTNAME?             - Get mDNS hostname)EOF"
#endif // SUPPORT_MDNS

#if defined(SUPPORT_TCP) || defined(SUPPORT_UDP)
R"EOF(
Network Configuration:
  AT+NETCONF?                   - Get current network configuration
  AT+NETCONF=(protocol,host,port)
                                - Configure TCP/UDP connection
    Examples:
      AT+NETCONF=(TCP,192.168.1.100,8080)
      AT+NETCONF=(UDP,192.168.1.200,9090)
      AT+NETCONF=(TCP,192.168.1.100,8080);(UDP,192.168.1.200,9090)
      AT+NETCONF=(TCP,2001:db8::1,8080);(UDP,2001:db8::2,9090)
      AT+NETCONF=(UDP_LISTEN,5678)
      AT+NETCONF=(UDP6_LISTEN,5679)
      AT+NETCONF=(TCP_SERVER,1234)
      AT+NETCONF=(TCP6_SERVER,1235)
      AT+NETCONF=(UDP_LISTEN,5678);(TCP_SERVER,1234)
      AT+NETCONF=(UDP6_LISTEN,5679);(TCP6_SERVER,1235)
      AT+NETCONF=(UDP_SEND,192.168.1.100,5678);(UDP_LISTEN,5679)
      AT+NETCONF=)EOF"
#endif // SUPPORT_TCP || SUPPORT_UDP

#ifdef SUPPORT_TCP
R"EOF(
TCP Commands (Legacy):
  AT+TCP_PORT=<port>            - Set TCP port
  AT+TCP_PORT?                  - Get TCP port
  AT+TCP_HOST_IP=<ip>           - Set TCP host IP
  AT+TCP_HOST_IP?               - Get TCP host IP
  AT+TCP_STATUS?                - Get TCP connection status)EOF"
#endif // SUPPORT_TCP

#ifdef SUPPORT_TCP_SERVER
R"EOF(
TCP Server Commands:
  AT+TCP_SERVER_PORT=<port>     - Set TCP server port
  AT+TCP_SERVER_PORT?           - Get TCP server port
  AT+TCP_SERVER_MAX_CLIENTS=<n> - Set max clients
  AT+TCP_SERVER_MAX_CLIENTS?    - Get max clients
  AT+TCP_SERVER_STATUS?         - Get TCP server status
  AT+TCP_SERVER_START           - Start TCP server
  AT+TCP_SERVER_STOP            - Stop TCP server
  AT+TCP_SERVER_SEND=<data>     - Send data to clients)EOF"
#endif // SUPPORT_TCP_SERVER

#ifdef SUPPORT_TLS
R"EOF(
TLS/SSL Commands:
  AT+TLS_ENABLE=<0|1>           - Enable/disable TLS for TCP connections
  AT+TLS_ENABLE?                - Get TLS enable status
  AT+TLS_PORT=<port>            - Set TLS port (defaults to TCP port if not set)
  AT+TLS_PORT?                  - Get TLS port
  AT+TLS_VERIFY=<0|1|2>         - Set certificate verification
                                    0=none
                                    1=optional
                                    2=required
  AT+TLS_VERIFY?                - Get certificate verification mode
  AT+TLS_SNI=<0|1>              - Enable/disable Server Name Indication
  AT+TLS_SNI?                   - Get SNI status
  AT+TLS_CA_CERT=<cert>         - Set CA certificate (PEM format)
  AT+TLS_CA_CERT?               - Check if CA certificate is set
  AT+TLS_CLIENT_CERT=<cert>     - Set client certificate (PEM format)
  AT+TLS_CLIENT_CERT?           - Check if client certificate is set
  AT+TLS_CLIENT_KEY=<key>       - Set client private key (PEM format)
  AT+TLS_CLIENT_KEY?            - Check if client key is set
  AT+TLS_PSK_IDENTITY=<id>      - Set PSK identity
  AT+TLS_PSK_IDENTITY?          - Check if PSK identity is set
  AT+TLS_PSK_KEY=<key>          - Set PSK key (hex format)
  AT+TLS_PSK_KEY?               - Check if PSK key is set
  AT+TLS_STATUS?                - Get TLS connection status and cipher info
  AT+TLS_CONNECT                - Manually connect TLS
  AT+TLS_DISCONNECT             - Disconnect TLS)EOF"
#endif // SUPPORT_TLS

#ifdef SUPPORT_UDP
R"EOF(
UDP Commands (Legacy):
  AT+UDP_PORT=<port>            - Set UDP port
  AT+UDP_PORT?                  - Get UDP port
  AT+UDP_LISTEN_PORT=<port>     - Set UDP listen port
  AT+UDP_LISTEN_PORT?           - Get UDP listen port
  AT+UDP6_LISTEN_PORT=<port>    - Set UDP6 listen port (IPv6 only)
  AT+UDP6_LISTEN_PORT?          - Get UDP6 listen port
  AT+UDP_SEND=<ip:port>         - Set UDP send IP and port
  AT+UDP_SEND?                  - Get UDP send IP and port
  AT+UDP_HOST_IP=<ip>           - Set UDP host IP
  AT+UDP_HOST_IP?               - Get UDP host IP)EOF"
#endif // SUPPORT_UDP

#ifdef SUPPORT_NTP
R"EOF(
NTP Commands:
  AT+NTP_HOST=<host>            - Set NTP server hostname
  AT+NTP_HOST?                  - Get NTP server hostname
  AT+NTP_STATUS?                - Get NTP sync status)EOF"
#endif // SUPPORT_NTP

#ifdef SUPPORT_UART1
R"EOF(
UART1 Commands:
  AT+UART1=baud,data,parity,stop[,rx,tx]
                                - Configure UART1 parameters
                                    baud: 300-115200,
                                    data: 5-8 bits,
                                    parity: 0=None/1=Even/2=Odd
                                    stop: 1-2 bits,
                                    rx/tx (optional):
                                      pin 0-39 (ESP32)
                                      pin 0-16 (ESP8266)
  AT+UART1?                     - Get current UART1 configuration)EOF"
#endif // SUPPORT_UART1

R"EOF(
System Commands:
  AT+RESET                      - Restart device)EOF"

#ifdef LOOP_DELAY
R"EOF(
  AT+LOOP_DELAY=<ms>            - Set main loop delay
  AT+LOOP_DELAY?                - Get main loop delay)EOF"
#endif // LOOP_DELAY

#ifdef VERBOSE
R"EOF(
  AT+VERBOSE=<0|1>              - Enable/disable verbose logging
  AT+VERBOSE?                   - Get verbose logging status)EOF"
#endif // VERBOSE

#ifdef TIMELOG
R"EOF(
  AT+TIMELOG=<0|1>              - Enable/disable time logging
  AT+TIMELOG?                   - Get time logging status)EOF"
#endif // TIMELOG

#ifdef LOGUART
R"EOF(
  AT+LOG_UART=<0|1>             - Enable/disable UART logging
  AT+LOG_UART?                  - Get UART logging status)EOF"
#endif // LOGUART

#ifdef BLUETOOTH_UART_AT
R"EOF(
BLE Commands:
  AT+BLE_PIN=<pin>              - Set BLE PIN (6 digits)
  AT+BLE_PIN?                   - Get current BLE PIN
  AT+BLE_SECURITY=<mode>        - Set BLE security mode
                                    0=None
                                    1=PIN
                                    2=Bonding
  AT+BLE_SECURITY?              - Get BLE security mode
  AT+BLE_IO_CAP=<cap>           - Set BLE IO capability
                                    0=DisplayOnly
                                    1=DisplayYesNo
                                    2=KeyboardOnly
                                    3=NoInputNoOutput
                                    4=KeyboardDisplay
  AT+BLE_IO_CAP?                - Get BLE IO capability
  AT+BLE_AUTH_REQ=<req>         - Set authentication requirements
                                    0=None
                                    1=Bonding
                                    2=MITM
                                    3=Bonding+MITM
  AT+BLE_AUTH_REQ?              - Get authentication requirements
  AT+BLE_ADDR_TYPE=<type>       - Set BLE address type
                                    0=Public
                                    1=Random Static
                                    2=Private Resolvable
                                    3=Private Non-resolvable
  AT+BLE_ADDR_TYPE?             - Get BLE address type
  AT+BLE_ADDR=<address>         - Set custom BLE MAC address (format: XX:XX:XX:XX:XX:XX)
  AT+BLE_ADDR?                  - Get current BLE MAC address
  AT+BLE_ADDR_GEN?              - Generate new random static address
  AT+BLE_STATUS?                - Get BLE connection and security status
  AT+BLE_UART1=|?               - Enable/disable BLE UART1 bridge
  AT+BLE_UART1_PASS=<0|1>       - Enable/disable passthrough mode
                                    1=passthrough
                                    0=AT command mode
  AT+BLE_UART1_PASS?            - Get passthrough mode status)EOF"
#endif // BLUETOOTH_UART_AT

R"EOF(

Note: Commands with '?' are queries, commands with '=' set values
)EOF";

#ifdef ARDUINO_ARCH_ESP32
#define CFG_PARTITION "esp-at"
#define CFG_NAMESPACE "esp-at"
#define CFG_STORAGE   "config"
nvs_handle_t nvs_c;
#else
#define CFG_EEPROM 0x00
#endif // ARDUINO_ARCH_ESP32

NOINLINE
void CFG_SAVE(){
  #ifdef ARDUINO_ARCH_ESP8266
  EEPROM.put(CFG_EEPROM, cfg);
  EEPROM.commit();
  #elif defined(ARDUINO_ARCH_ESP32)
  LOG("[NVS] Saving config to NVS.., size: %d bytes", sizeof(cfg));
  esp_err_t err;
  err = nvs_open_from_partition(CFG_PARTITION, CFG_NAMESPACE, NVS_READWRITE, &nvs_c);
  if (err != ESP_OK) {
    LOG("[NVS] Error (%d) opening NVS handle! %s", err, esp_err_to_name(err));
    return;
  }
  err = nvs_set_blob(nvs_c, CFG_STORAGE, &cfg, sizeof(cfg));
  if (err != ESP_OK) {
    LOG("[NVS] Error (%d) setting blob in NVS! %s", err, esp_err_to_name(err));
  }
  err = nvs_commit(nvs_c);
  if (err != ESP_OK) {
    LOG("[NVS] Error (%d) committing blob to NVS! %s", err, esp_err_to_name(err));
  }
  LOG("[NVS] Config saved to NVS");
  #endif
}

NOINLINE
void CFG_CLEAR(){
  #ifdef ARDUINO_ARCH_ESP32

  LOG("[NVS] Clearing config from NVS...");

  // Erase the entire NVS partition used by config
  esp_err_t err;
  err = nvs_flash_erase_partition(CFG_PARTITION);
  if (err != ESP_OK) {
    LOG("[NVS] Error (%d) erasing NVS! %s", err, esp_err_to_name(err));
  }

  // Erase the main "nvs" partition as well, to clear any other data
  err = nvs_flash_erase_partition("nvs");
  if (err != ESP_OK) {
    LOG("[NVS] Error (%d) erasing main NVS! %s", err, esp_err_to_name(err));
  }

  LOG("[NVS] NVS cleared");
  #elif defined(ARDUINO_ARCH_ESP8266)
  // Clear the entire EEPROM section used by config
  for(int i = 0; i < sizeof(cfg); i++) {
    EEPROM.write(CFG_EEPROM + i, 0xFF);
  }
  EEPROM.commit();
  #endif
}

NOINLINE
void CFG_LOAD(){
  #ifdef ARDUINO_ARCH_ESP8266
  EEPROM.get(CFG_EEPROM, cfg);
  #elif defined(ARDUINO_ARCH_ESP32)
  LOG("[NVS] Loading config from NVS...");
  esp_err_t err;
  err = nvs_open_from_partition(CFG_PARTITION, CFG_NAMESPACE, NVS_READONLY, &nvs_c);
  if (err != ESP_OK) {
    LOG("[NVS] Error (%d) opening NVS handle! %s", err, esp_err_to_name(err));
    return;
  }
  size_t required_size = sizeof(cfg);
  err = nvs_get_blob(nvs_c, CFG_STORAGE, &cfg, &required_size);
  if (err == ESP_ERR_NVS_NOT_FOUND) {
    LOG("[NVS] No config found in NVS, using defaults");
    return;
  } else if (err != ESP_OK) {
    LOG("[NVS] Error (%d) reading config from NVS! %s", err, esp_err_to_name(err));
    return;
  }
  LOG("[NVS] Config loaded from NVS, size: %d bytes", required_size);
  #endif
}

#ifdef SUPPORT_BLE_UART1
NOINLINE
#define AT_MODE 1
#define BRIDGE_MODE 0
uint8_t at_mode = AT_MODE; // 1=AT command mode, 0=AT bridge mode
void ble_uart1_at_mode(uint8_t enable){
  if(enable == AT_MODE){
    LOG("[BLE_UART1] Switching to AT command mode");
    at_mode = AT_MODE;
  } else {
    LOG("[BLE_UART1] Switching to BLE UART1 bridge mode");
    at_mode = BRIDGE_MODE;
  }
}
#endif // SUPPORT_BLE_UART1

const char* at_cmd_handler(const char* atcmdline){
  unsigned int cmd_len = strlen(atcmdline);
  char *p = NULL;
  char *r = NULL;
  errno = 0;
  D("[AT] [%s], size: %d", atcmdline, cmd_len);
  if(cmd_len == 2 && (p = at_cmd_check("AT", atcmdline, cmd_len))){
    return AT_R_OK;
  } else if(cmd_len == 3 && (p = at_cmd_check("AT?", atcmdline, cmd_len))){
    return AT_R_OK;
  #ifdef LOOP_DELAY
  } else if(p = at_cmd_check("AT+LOOP_DELAY=", atcmdline, cmd_len)){
    unsigned int new_c = strtoul(p, &r, 10);
    if(errno != 0 || new_c < 10 || new_c > 60000 || (r == p))
      return AT_R("+ERROR: invalid loop delay");
    if(new_c != cfg.main_loop_delay){
      cfg.main_loop_delay = new_c;
      CFG_SAVE();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+LOOP_DELAY?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.main_loop_delay);
  #endif // LOOP_DELAY
  #ifdef TIMELOG
  } else if(p = at_cmd_check("AT+TIMELOG=1", atcmdline, cmd_len)){
    cfg.do_timelog = 1;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TIMELOG=0", atcmdline, cmd_len)){
    cfg.do_timelog = 0;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TIMELOG?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.do_timelog);
  #endif // TIMELOG
  #ifdef VERBOSE
  } else if(p = at_cmd_check("AT+VERBOSE=1", atcmdline, cmd_len)){
    cfg.do_verbose = 1;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+VERBOSE=0", atcmdline, cmd_len)){
    cfg.do_verbose = 0;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+VERBOSE?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.do_verbose);
  #endif // VERBOSE
  #ifdef LOGUART
  } else if(p = at_cmd_check("AT+LOG_UART=1", atcmdline, cmd_len)){
    cfg.do_log = 1;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+LOG_UART=0", atcmdline, cmd_len)){
    cfg.do_log = 0;
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+LOG_UART?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.do_log);
  #endif // LOGUART
  #ifdef SUPPORT_UART1
  } else if(p = at_cmd_check("AT+UART1=", atcmdline, cmd_len)){
    // Parse format: baud,data,parity,stop,rx_pin,tx_pin
    // Example: AT+UART1=115200,8,0,1,0,1

    // Find comma positions
    char *ct = p;
    char *cp[5] = {0};
    int cc = 0;
    while(cc < 5 && ct < p+cmd_len) {
      ct = strchr(ct, ',');
      if(ct == NULL)
        break;
      ct++; // Move past comma
      cp[cc] = ct;
      cc++;
    }
    if(cc < 3)
      return AT_R("+ERROR: Format: baud,data,parity,stop[,rx_pin,tx_pin]");

    // print the strings for debugging
    for(int i = 0; i < 5; i++) {
      if(cp[i] == NULL)
        break;
      D("[AT] UART1 param %d: %s", i, cp[i]);
    }

    // Parse and validate parameters
    uint32_t baud = strtoul(p, &r, 10);
    if(errno != 0 || r == p)
      return AT_R("+ERROR: Invalid baud rate");
    uint8_t data = strtoul(cp[0], &r, 10);
    if(errno != 0 || r == cp[0])
      return AT_R("+ERROR: Invalid data bits");
    uint8_t parity = strtoul(cp[1], &r, 10);
    if(errno != 0 || r == cp[1])
      return AT_R("+ERROR: Invalid parity");
    uint8_t stop = strtoul(cp[2], &r, 10);
    if(errno != 0 || r == cp[2])
      return AT_R("+ERROR: Invalid stop bits");
    uint8_t rx_pin = UART1_RX_PIN;
    uint8_t tx_pin = UART1_TX_PIN;
    if(cc >= 4 && cp[3] != NULL && cp[4] != NULL) {
        rx_pin = strtoul(cp[3], &r, 10);
        if(errno != 0 || r == cp[3])
          return AT_R("+ERROR: Invalid RX pin");
        tx_pin = strtoul(cp[4], &r, 10);
        if(errno != 0 || r == cp[4])
          return AT_R("+ERROR: Invalid TX pin");
    }

    LOG("[AT] UART1 config: baud=%d, data=%d, parity=%d, stop=%d, rx=%d, tx=%d", baud, data, parity, stop, rx_pin, tx_pin);

    // Validate ranges
    if(baud < 300 || baud > 115200)
      return AT_R("+ERROR: Baud rate must be 300-115200");
    if(data < 5 || data > 8)
      return AT_R("+ERROR: Data bits must be 5-8");
    if(parity > 2)
      return AT_R("+ERROR: Parity: 0=None, 1=Even, 2=Odd");
    if(stop < 1 || stop > 2)
      return AT_R("+ERROR: Stop bits must be 1 or 2");
    if(rx_pin > 39 || tx_pin > 39)
      return AT_R("+ERROR: Pin numbers must be 0-39");

    // Update configuration
    cfg.uart1_baud   = baud;
    cfg.uart1_data   = data;
    cfg.uart1_parity = parity;
    cfg.uart1_stop   = stop;
    cfg.uart1_rx_pin = rx_pin;
    cfg.uart1_tx_pin = tx_pin;

    // Save configuration
    CFG_SAVE();

    // Apply new configuration
    setup_uart1();

    return AT_R_OK;
  } else if(p = at_cmd_check("AT+UART1?", atcmdline, cmd_len)){
    String response =
        String(cfg.uart1_baud)   + "," +
        String(cfg.uart1_data)   + "," +
        String(cfg.uart1_parity) + "," +
        String(cfg.uart1_stop)   + "," +
        String(cfg.uart1_rx_pin) + "," +
        String(cfg.uart1_tx_pin);
    return AT_R_S(response);
  #endif // SUPPORT_UART1
  #ifdef SUPPORT_BLE_UART1
  } else if(p = at_cmd_check("AT+BLE_UART1=", atcmdline, cmd_len)){
    int enable_ble_uart1 = strtoul(p, &r, 10);
    if(errno != 0 || r == p)
      return AT_R("+ERROR: Invalid parameter, use 1 to enable, 0 to disable");
    if(enable_ble_uart1 == 1){
      cfg.ble_uart1_bridge = 1;
      CFG_SAVE();
      return AT_R_OK;
    } else if(enable_ble_uart1 == 0){
      cfg.ble_uart1_bridge = 0;
      CFG_SAVE();
      return AT_R_OK;
    } else {
      return AT_R("+ERROR: Use AT+BLE_UART1=1 to enable, AT+BLE_UART1=0 to disable");
    }
  } else if(p = at_cmd_check("AT+BLE_UART1?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.ble_uart1_bridge);
  } else if(p = at_cmd_check("AT+BLE_UART1_PASS=", atcmdline, cmd_len)){
    // Switch between AT command mode and BLE UART1 passthrough mode
    // Only works if BLE UART1 bridge is enabled, otherwise always in AT mode
    if(!cfg.ble_uart1_bridge) {
      // Force AT mode
      ble_uart1_at_mode(AT_MODE);
      return AT_R("+ERROR: BLE UART1 bridge is disabled, enable with AT+BLE_UART1=1");
    }
    // Parse parameter
    uint8_t m_req = strtoul(p, &r, 10);
    if(errno != 0 || r == p)
      return AT_R("+ERROR: Invalid parameter, use 1 for AT mode, 0 for passthrough mode");
    if(m_req != 0 && m_req != 1)
      return AT_R("+ERROR: Use 1 for AT command mode, 0 for BLE UART1 passthrough mode");
    // Set mode
    if(m_req == 1) {
      // Switch to passthrough mode
      ble_uart1_at_mode(BRIDGE_MODE);
      return AT_R(""); // don't reply
    } else {
      // Stay in AT command mode
      ble_uart1_at_mode(AT_MODE);
      return AT_R_OK;  // reply OK
    }
  } else if (p = at_cmd_check("AT+BLE_UART1_PASS?", atcmdline, cmd_len)){
    if(!cfg.ble_uart1_bridge) {
      return AT_R("+ERROR: BLE UART1 bridge is disabled, enable with AT+BLE_UART1=1");
    }
    if(at_mode == AT_MODE)
      return AT_R("0"); // Passthrough mode
    else
      return AT_R("1"); // AT command mode
  #endif // SUPPORT_BLE_UART1
  #ifdef SUPPORT_WIFI
  } else if(p = at_cmd_check("AT+WIFI_SSID=", atcmdline, cmd_len)){
    if(strlen(p) > 31)
      return AT_R("+ERROR: WiFI SSID max 31 chars");
    if(strlen(p) == 0){
      // Empty SSID, clear it
      memset((char *)&cfg.wifi_ssid, 0, sizeof(cfg.wifi_ssid));
      cfg.wifi_ssid[0] = '\0';
      CFG_SAVE();
      reset_networking();
      return AT_R_OK;
    }
    strncpy((char *)&cfg.wifi_ssid, p, sizeof(cfg.wifi_ssid) - 1);
    cfg.wifi_ssid[sizeof(cfg.wifi_ssid) - 1] = '\0';
    CFG_SAVE();
    reset_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+WIFI_SSID?", atcmdline, cmd_len)){
    if(strlen(cfg.wifi_ssid) == 0)
      return AT_R("+ERROR: WiFi SSID not set");
    else
      return AT_R_STR(cfg.wifi_ssid);
  } else if(p = at_cmd_check("AT+WIFI_PASS=", atcmdline, cmd_len)){
    if(strlen(p) > 63)
      return AT_R("+ERROR: WiFi PASS max 63 chars");
    if(strlen(p) == 0){
      // Empty password, clear it
      memset((char *)&cfg.wifi_pass, 0, sizeof(cfg.wifi_pass));
      cfg.wifi_pass[0] = '\0';
      CFG_SAVE();
      reset_networking();
      return AT_R_OK;
    }
    strncpy((char *)&cfg.wifi_pass, p, sizeof(cfg.wifi_pass) - 1);
    cfg.wifi_pass[sizeof(cfg.wifi_pass) - 1] = '\0';
    CFG_SAVE();
    reset_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+WIFI_STATUS?", atcmdline, cmd_len)){
    if(!cfg.wifi_enabled)
      return AT_R("wifi disabled");
    uint8_t wifi_stat = WiFi.status();
    switch(wifi_stat) {
        case WL_CONNECTED:
          return AT_R("connected");
        case WL_CONNECT_FAILED:
          return AT_R("failed");
        case WL_CONNECTION_LOST:
          return AT_R("connection lost");
        case WL_DISCONNECTED:
          return AT_R("disconnected");
        case WL_IDLE_STATUS:
          return AT_R("idle");
        case WL_NO_SSID_AVAIL:
          return AT_R("no SSID configured");
        default:
          return AT_R_INT(wifi_stat);
    }
  #ifdef WIFI_WPS
  } else if(p = at_cmd_check("AT+WPS_PBC", atcmdline, cmd_len)){
    if(start_wps(NULL)) {
      return AT_R_OK;
    } else {
      return AT_R("+ERROR: Failed to start WPS PBC");
    }
  } else if(p = at_cmd_check("AT+WPS_PIN=", atcmdline, cmd_len)){
    if(strlen(p) != 8) {
      return AT_R("+ERROR: WPS PIN must be 8 digits");
    }
    // Verify PIN contains only digits
    for(int i = 0; i < 8; i++) {
      if(p[i] < '0' || p[i] > '9') {
        return AT_R("+ERROR: WPS PIN must contain only digits");
      }
    }
    if(start_wps(p)) {
      return AT_R_OK;
    } else {
      return AT_R("+ERROR: Failed to start WPS PIN");
    }
  } else if(p = at_cmd_check("AT+WPS_STOP", atcmdline, cmd_len)){
    if(stop_wps()) {
      reset_networking();
      return AT_R_OK;
    } else {
      return AT_R("+ERROR: WPS not running");
    }
  } else if(p = at_cmd_check("AT+WPS_STATUS?", atcmdline, cmd_len)){
    return AT_R(get_wps_status());
  #endif // WIFI_WPS
  } else if(p = at_cmd_check("AT+WIFI_ENABLED=1", atcmdline, cmd_len)){
    cfg.wifi_enabled = 1;
    CFG_SAVE();
    reset_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+WIFI_ENABLED=0", atcmdline, cmd_len)){
    cfg.wifi_enabled = 0;
    CFG_SAVE();
    stop_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+WIFI_ENABLED?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.wifi_enabled);
  #ifdef SUPPORT_NTP
  } else if(p = at_cmd_check("AT+NTP_HOST=", atcmdline, cmd_len)){
    if(strlen(p) > 63)
      return AT_R("+ERROR: NTP hostname max 63 chars");
    if(strlen(p) == 0){
      // Empty hostname, clear it
      memset((char *)&cfg.ntp_host, 0, sizeof(cfg.ntp_host));
      cfg.ntp_host[0] = '\0';
      CFG_SAVE();
      return AT_R_OK;
    }
    strncpy((char *)&cfg.ntp_host, p, sizeof(cfg.ntp_host) - 1);
    cfg.ntp_host[sizeof(cfg.ntp_host) - 1] = '\0';
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+NTP_HOST?", atcmdline, cmd_len)){
    if(strlen(cfg.ntp_host) == 0)
      return AT_R("+ERROR: NTP hostname not set");
    else
      return AT_R_STR(cfg.ntp_host);
  } else if(p = at_cmd_check("AT+NTP_STATUS?", atcmdline, cmd_len)){
    if(ntp_is_synced)
      return AT_R("ntp synced");
    else
      return AT_R("not ntp synced");
  #endif // SUPPORT_NTP
  #if defined(SUPPORT_UDP) || defined(SUPPORT_TCP)
  } else if(p = at_cmd_check("AT+NETCONF?", atcmdline, cmd_len)){
    String response = "";
    #ifdef SUPPORT_TCP
    if(cfg.tcp_port > 0 && strlen(cfg.tcp_host_ip) > 0) {
      response += "TCP,";
      response += cfg.tcp_host_ip;
      response += ",";
      response += cfg.tcp_port;
    }
    #endif
    #ifdef SUPPORT_TCP_SERVER
    if(cfg.tcp_server_port > 0) {
      if(response.length() > 0) response += ";";
      response += "TCP_SERVER,";
      response += cfg.tcp_server_port;
      response += ",max_clients=";
      response += cfg.tcp_server_max_clients;
      if(tcp_server_sock != -1) {
        response += ",status=ACTIVE,clients=";
        response += get_tcp_server_client_count();
      } else {
        response += ",status=INACTIVE";
      }
    }
    if(cfg.tcp6_server_port > 0) {
      if(response.length() > 0) response += ";";
      response += "TCP6_SERVER,";
      response += cfg.tcp6_server_port;
      response += ",max_clients=";
      response += cfg.tcp_server_max_clients;
      if(tcp6_server_sock != -1) {
        response += ",status=ACTIVE,clients=";
        response += get_tcp_server_client_count();
      } else {
        response += ",status=INACTIVE";
      }
    }
    #endif
    #ifdef SUPPORT_UDP
    if(cfg.udp_port > 0 && strlen(cfg.udp_host_ip) > 0) {
      if(response.length() > 0) response += ";";
      response += "UDP,";
      response += cfg.udp_host_ip;
      response += ",";
      response += cfg.udp_port;
    }
    if(cfg.udp_listen_port > 0) {
      if(response.length() > 0) response += ";";
      response += "UDP_LISTEN,";
      response += cfg.udp_listen_port;
    }
    if(cfg.udp6_listen_port > 0) {
      if(response.length() > 0) response += ";";
      response += "UDP6_LISTEN,";
      response += cfg.udp6_listen_port;
    }
    if(cfg.udp_send_port > 0 && strlen(cfg.udp_send_ip) > 0) {
      if(response.length() > 0) response += ";";
      response += "UDP_SEND,";
      response += cfg.udp_send_ip;
      response += ",";
      response += cfg.udp_send_port;
    }
    #endif
    return AT_R_S(response);
  } else if(p = at_cmd_check("AT+NETCONF=", atcmdline, cmd_len)){
    // Parse format: (protocol,host,port) or multiple configs separated by ;
    // Examples:
    //   AT+NETCONF=(TCP,192.168.1.100,8080)
    //   AT+NETCONF=(UDP,192.168.1.200,9090)
    //   AT+NETCONF=(TCP,192.168.1.100,8080);(UDP,192.168.1.200,9090)
    //   AT+NETCONF=(UDP_LISTEN,9090)
    //   AT+NETCONF=(UDP_SEND,192.168.1.100,9090)
    //   AT+NETCONF= (empty to disable all)

    if(strlen(p) == 0) {
      // Empty string means disable all network connections
      #ifdef SUPPORT_TCP
      cfg.tcp_port = 0;
      memset(cfg.tcp_host_ip, 0, sizeof(cfg.tcp_host_ip));
      #endif
      #ifdef SUPPORT_TCP_SERVER
      cfg.tcp_server_port = 0;
      cfg.tcp6_server_port = 0;
      #endif
      #ifdef SUPPORT_UDP
      cfg.udp_port = 0;
      cfg.udp_listen_port = 0;
      cfg.udp6_listen_port = 0;
      memset(cfg.udp_host_ip, 0, sizeof(cfg.udp_host_ip));
      cfg.udp_send_port = 0;
      memset(cfg.udp_send_ip, 0, sizeof(cfg.udp_send_ip));
      #endif
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }

    // Reset current configurations
    #ifdef SUPPORT_TCP
    cfg.tcp_port = 0;
    memset(cfg.tcp_host_ip, 0, sizeof(cfg.tcp_host_ip));
    #endif
    #ifdef SUPPORT_TCP_SERVER
    cfg.tcp_server_port = 0;
    #endif
    #ifdef SUPPORT_UDP
    cfg.udp_port = 0;
    cfg.udp_listen_port = 0;
    memset(cfg.udp_host_ip, 0, sizeof(cfg.udp_host_ip));
    cfg.udp_send_port = 0;
    memset(cfg.udp_send_ip, 0, sizeof(cfg.udp_send_ip));
    #endif

    // Parse configuration string
    char *config_str = strdup(p);
    if (!config_str) {
      return AT_R("+ERROR: memory allocation failed");
    }
    char *saveptr1, *saveptr2;
    char *config_token = strtok_r(config_str, ";", &saveptr1);

    while(config_token != NULL) {
      // Remove leading/trailing whitespace
      while(*config_token == ' ') config_token++;
      char *end = config_token + strlen(config_token) - 1;
      while(end > config_token && *end == ' ') end--;
      *(end + 1) = '\0';

      // Check for parentheses format
      if(*config_token != '(' || *(config_token + strlen(config_token) - 1) != ')') {
        free(config_str);
        return AT_R("+ERROR: invalid format, use (protocol,host,port)");
      }

      // Remove parentheses
      config_token++;
      *(config_token + strlen(config_token) - 1) = '\0';

      // Parse protocol,host,port or protocol,port for server
      char *protocol = strtok_r(config_token, ",", &saveptr2);
      char *host_or_port = strtok_r(NULL, ",", &saveptr2);
      char *port_str = strtok_r(NULL, ",", &saveptr2);

      if(!protocol || !host_or_port) {
        free(config_str);
        return AT_R("+ERROR: invalid format, use (protocol,host,port) or (TCP_SERVER,port)");
      }

      // Remove whitespace
      while(*protocol == ' ') protocol++;
      while(*host_or_port == ' ') host_or_port++;
      if(port_str) while(*port_str == ' ') port_str++;

      // Handle TCP_SERVER case (only needs port)
      if(strcasecmp(protocol, "TCP_SERVER") == 0) {
        #ifdef SUPPORT_TCP_SERVER
        uint16_t server_port = (uint16_t)strtol(host_or_port, &r, 10);
        if(server_port == 0 || errno != 0 || server_port > 65535 || (r == host_or_port)) {
          free(config_str);
          return AT_R("+ERROR: invalid TCP server port number");
        }
        cfg.tcp_server_port = server_port;
        #else
        free(config_str);
        return AT_R("+ERROR: TCP_SERVER not supported");
        #endif
      } else if(strcasecmp(protocol, "TCP6_SERVER") == 0) {
        #ifdef SUPPORT_TCP_SERVER
        uint16_t server_port = (uint16_t)strtol(host_or_port, &r, 10);
        if(server_port == 0 || errno != 0 || server_port > 65535 || (r == host_or_port)) {
          free(config_str);
          return AT_R("+ERROR: invalid TCP6 server port number");
        }
        cfg.tcp6_server_port = server_port;
        #else
        free(config_str);
        return AT_R("+ERROR: TCP6_SERVER not supported");
        #endif
      } else {
        // Regular client protocols need host and port
        if(!port_str) {
          free(config_str);
          return AT_R("+ERROR: invalid format, use (protocol,host,port)");
        }

        char *host = host_or_port;
        uint16_t port = (uint16_t)strtol(port_str, &r, 10);
        if(port == 0 || errno != 0 || port > 65535 || (r == port_str)) {
          free(config_str);
          return AT_R("+ERROR: invalid port number");
        }

        if(strlen(host) >= 40) {
          free(config_str);
          return AT_R("+ERROR: host too long (>=40 chars)");
        }

        // Validate IP address
        IPAddress tst;
        if(!tst.fromString(host)) {
          free(config_str);
          return AT_R("+ERROR: invalid host IP address");
        }

        // Set configuration based on protocol
        if(strcasecmp(protocol, "TCP") == 0) {
          #ifdef SUPPORT_TCP
          cfg.tcp_port = port;
          strncpy(cfg.tcp_host_ip, host, 40-1);
          cfg.tcp_host_ip[40-1] = '\0';
          #else
          free(config_str);
          return AT_R("+ERROR: TCP not supported");
          #endif
        } else if(strcasecmp(protocol, "UDP") == 0) {
          #ifdef SUPPORT_UDP
          cfg.udp_port = port;
          strncpy(cfg.udp_host_ip, host, 40-1);
          cfg.udp_host_ip[40-1] = '\0';
          #else
          free(config_str);
          return AT_R("+ERROR: UDP not supported");
          #endif
        } else if(strcasecmp(protocol, "UDP_LISTEN") == 0) {
          #ifdef SUPPORT_UDP
          cfg.udp_listen_port = port;
          #else
          free(config_str);
          return AT_R("+ERROR: UDP_LISTEN not supported");
          #endif
        } else if(strcasecmp(protocol, "UDP6_LISTEN") == 0) {
          #ifdef SUPPORT_UDP
          cfg.udp6_listen_port = port;
          #else
          free(config_str);
          return AT_R("+ERROR: UDP6_LISTEN not supported");
          #endif
        } else if(strcasecmp(protocol, "UDP_SEND") == 0) {
          #ifdef SUPPORT_UDP
          cfg.udp_send_port = port;
          strncpy(cfg.udp_send_ip, host, 40-1);
          cfg.udp_send_ip[40-1] = '\0';
          #else
          free(config_str);
          return AT_R("+ERROR: UDP_SEND not supported");
          #endif
        } else if(strcasecmp(protocol, "TCP_SERVER") == 0) {
          #ifdef SUPPORT_TCP_SERVER
          cfg.tcp_server_port = port;
          #else
          free(config_str);
          return AT_R("+ERROR: TCP_SERVER not supported");
          #endif
        } else if(strcasecmp(protocol, "TCP6_SERVER") == 0) {
          #ifdef SUPPORT_TCP_SERVER
          cfg.tcp6_server_port = port;
          #else
          free(config_str);
          return AT_R("+ERROR: TCP6_SERVER not supported");
          #endif
        } else {
          free(config_str);
          return AT_R("+ERROR: invalid protocol, use TCP, UDP, UDP_LISTEN, UDP6_LISTEN, UDP_SEND, TCP_SERVER or TCP6_SERVER");
        }
      }

      config_token = strtok_r(NULL, ";", &saveptr1);
    }

    free(config_str);
    CFG_SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  #endif // SUPPORT_UDP || SUPPORT_TCP
  #ifdef SUPPORT_UDP
  } else if(p = at_cmd_check("AT+UDP_SEND?", atcmdline, cmd_len)){
    if(cfg.udp_send_port == 0 || strlen(cfg.udp_send_ip) == 0)
      return AT_R("+ERROR: UDP send not configured");
    String response = String(cfg.udp_send_ip) + ":" + String(cfg.udp_send_port);
    return AT_R_S(response);
  } else if(p = at_cmd_check("AT+UDP_SEND=", atcmdline, cmd_len)){
    if(strlen(p) == 0){
      // Empty string means disable UDP send
      cfg.udp_send_port = 0;
      memset(cfg.udp_send_ip, 0, sizeof(cfg.udp_send_ip));
      cfg.udp_send_ip[0] = '\0';
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    // Expect format ip:port
    char *sep = strchr(p, ':');
    if(!sep)
      return AT_R("+ERROR: invalid format, use ip:port");
    *sep = '\0';
    char *ip_str = p;
    char *port_str = sep + 1;
    if(strlen(ip_str) >= 40)
      return AT_R("+ERROR: invalid udp send ip (too long, >=40)");
    uint16_t port = (uint16_t)strtol(port_str, &r, 10);
    if(port == 0 || errno != 0 || port > 65535 || (r == port_str))
      return AT_R("+ERROR: invalid udp send port");
    IPAddress tst;
    if(!tst.fromString(ip_str))
      return AT_R("+ERROR: invalid udp send ip");
    // Accept IPv4 or IPv6 string
    strncpy(cfg.udp_send_ip, ip_str, 40-1);
    cfg.udp_send_ip[40-1] = '\0';
    cfg.udp_send_port = port;
    CFG_SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+UDP_LISTEN_PORT?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.udp_listen_port);
  } else if(p = at_cmd_check("AT+UDP_LISTEN_PORT=", atcmdline, cmd_len)){
    if(strlen(p) == 0){
      // Empty string means disable UDP
      cfg.udp_listen_port = 0;
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_udp_port = (uint16_t)strtol(p, &r, 10);
    if(new_udp_port == 0 || errno != 0 || new_udp_port > 65535 || (r == p))
      return AT_R("+ERROR: invalid UDP port");
    if(new_udp_port != cfg.udp_listen_port){
      cfg.udp_listen_port = new_udp_port;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+UDP6_LISTEN_PORT?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.udp6_listen_port);
  } else if(p = at_cmd_check("AT+UDP6_LISTEN_PORT=", atcmdline, cmd_len)){
    if(strlen(p) == 0){
      // Empty string means disable UDP6
      cfg.udp6_listen_port = 0;
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_udp6_port = (uint16_t)strtol(p, &r, 10);
    if(new_udp6_port == 0 || errno != 0 || new_udp6_port > 65535 || (r == p))
      return AT_R("+ERROR: invalid UDP6 port");
    if(new_udp6_port != cfg.udp6_listen_port){
      cfg.udp6_listen_port = new_udp6_port;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+UDP_PORT?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.udp_port);
  } else if(p = at_cmd_check("AT+UDP_PORT=", atcmdline, cmd_len)){
    if(strlen(p) == 0){
      // Empty string means disable UDP
      cfg.udp_port = 0;
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_udp_port = (uint16_t)strtol(p, &r, 10);
    if(new_udp_port == 0 || errno != 0 || new_udp_port > 65535 || (r == p))
      return AT_R("+ERROR: invalid UDP port");
    if(new_udp_port != cfg.udp_port){
      cfg.udp_port = new_udp_port;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+UDP_HOST_IP?", atcmdline, cmd_len)){
    return AT_R_STR(cfg.udp_host_ip);
  } else if(p = at_cmd_check("AT+UDP_HOST_IP=", atcmdline, cmd_len)){
    if(strlen(p) >= 40)
      return AT_R("+ERROR: invalid udp host ip (too long, >=40)");
    if(strlen(p) == 0){
      // Empty string means disable UDP
      memset(cfg.udp_host_ip, 0, sizeof(cfg.udp_host_ip));
      cfg.udp_host_ip[0] = '\0';
    } else {
      IPAddress tst;
      if(!tst.fromString(p))
        return AT_R("+ERROR: invalid udp host ip");
      // Accept IPv4 or IPv6 string
      strncpy(cfg.udp_host_ip, p, 40-1);
      cfg.udp_host_ip[40-1] = '\0';
    }
    CFG_SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  #endif // SUPPORT_UDP
  #ifdef SUPPORT_TCP
  } else if(p = at_cmd_check("AT+TCP_PORT?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.tcp_port);
  } else if(p = at_cmd_check("AT+TCP_PORT=", atcmdline, cmd_len)){
    if(strlen(p) == 0){
      // Empty string means disable TCP
      cfg.tcp_port = 0;
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_tcp_port = (uint16_t)strtol(p, &r, 10);
    if(new_tcp_port == 0 || errno != 0 || new_tcp_port > 65535 || (r == p))
      return AT_R("+ERROR: invalid TCP port");
    if(new_tcp_port != cfg.tcp_port){
      cfg.tcp_port = new_tcp_port;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TCP_HOST_IP?", atcmdline, cmd_len)){
    return AT_R_STR(cfg.tcp_host_ip);
  } else if(p = at_cmd_check("AT+TCP_HOST_IP=", atcmdline, cmd_len)){
    if(strlen(p) >= 40)
      return AT_R("+ERROR: invalid tcp host ip (too long, >=40)");
    if(strlen(p) == 0){
      // Empty string means disable TCP
      memset(cfg.tcp_host_ip, 0, sizeof(cfg.tcp_host_ip));
      cfg.tcp_host_ip[0] = '\0';
    } else {
      IPAddress tst;
      if(!tst.fromString(p))
        return AT_R("+ERROR: invalid tcp host ip");
      // Accept IPv4 or IPv6 string
      strncpy(cfg.tcp_host_ip, p, 40-1);
      cfg.tcp_host_ip[40-1] = '\0';
    }
    CFG_SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  #endif // SUPPORT_TCP
  #ifdef SUPPORT_TCP_SERVER
  } else if(p = at_cmd_check("AT+TCP_SERVER_PORT?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.tcp_server_port);
  } else if(p = at_cmd_check("AT+TCP_SERVER_PORT=", atcmdline, cmd_len)){
    if(strlen(p) == 0){
      // Empty string means disable TCP server
      cfg.tcp_server_port = 0;
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_tcp_server_port = (uint16_t)strtol(p, &r, 10);
    if(new_tcp_server_port == 0 || errno != 0 || new_tcp_server_port > 65535 || (r == p))
      return AT_R("+ERROR: invalid TCP server port");
    if(new_tcp_server_port != cfg.tcp_server_port){
      cfg.tcp_server_port = new_tcp_server_port;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TCP_SERVER_MAX_CLIENTS?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.tcp_server_max_clients);
  } else if(p = at_cmd_check("AT+TCP_SERVER_MAX_CLIENTS=", atcmdline, cmd_len)){
    uint8_t new_max_clients = (uint8_t)strtol(p, &r, 10);
    if(new_max_clients == 0 || errno != 0 || new_max_clients > 8 || (r == p))
      return AT_R("+ERROR: invalid max clients (1-8)");
    if(new_max_clients != cfg.tcp_server_max_clients){
      cfg.tcp_server_max_clients = new_max_clients;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TCP_SERVER_STATUS?", atcmdline, cmd_len)){
    String response = "";
    if(tcp_server_sock != -1) {
      response += "ACTIVE,port=";
      response += cfg.tcp_server_port;
      response += ",clients=";
      response += get_tcp_server_client_count();
      response += "/";
      response += cfg.tcp_server_max_clients;
    } else {
      response = "INACTIVE";
    }
    return AT_R_S(response);
  } else if(p = at_cmd_check("AT+TCP_SERVER_START", atcmdline, cmd_len)){
    if(cfg.tcp_server_port == 0)
      return AT_R("+ERROR: TCP server port not configured");
    stop_tcp_servers();
    start_tcp_servers();
    if(tcp_server_sock != -1)
      return AT_R_OK;
    else
      return AT_R("+ERROR: failed to start TCP server");
  } else if(p = at_cmd_check("AT+TCP_SERVER_STOP", atcmdline, cmd_len)){
    stop_tcp_servers();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TCP_SERVER_SEND=", atcmdline, cmd_len)){
    if(tcp_server_sock == -1)
      return AT_R("+ERROR: TCP server not active");
    int clients_sent = send_tcp_server_data((const uint8_t*)p, strlen(p));
    if(clients_sent > 0) {
      String response = "SENT to ";
      response += clients_sent;
      response += " clients";
      return AT_R_S(response);
    } else {
      return AT_R("+ERROR: no connected clients");
    }
  #endif // SUPPORT_TCP_SERVER
  } else if(p = at_cmd_check("AT+HOSTNAME=", atcmdline, cmd_len)){
    if(strlen(p) > 63)
      return AT_R("+ERROR: hostname max 63 chars");
    strncpy((char *)&cfg.hostname, p, sizeof(cfg.hostname) - 1);
    cfg.hostname[sizeof(cfg.hostname) - 1] = '\0';
    CFG_SAVE();
    reset_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+HOSTNAME?", atcmdline, cmd_len)){
    if(strlen(cfg.hostname) == 0)
      return AT_R_STR(DEFAULT_HOSTNAME);
    else
      return AT_R_STR(cfg.hostname);
  #ifdef SUPPORT_MDNS
  } else if(p = at_cmd_check("AT+MDNS=", atcmdline, cmd_len)){
    uint8_t enable_mdns = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || (enable_mdns != 0 && enable_mdns != 1) || (r == p))
      return AT_R("+ERROR: use 0 or 1");
    cfg.mdns_enabled = enable_mdns;
    CFG_SAVE();
    if(WiFi.status() == WL_CONNECTED){
      if(cfg.mdns_enabled){
        setup_mdns();
      } else {
        stop_mdns();
      }
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+MDNS?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.mdns_enabled);
  } else if(p = at_cmd_check("AT+MDNS_HOSTNAME=", atcmdline, cmd_len)){
    if(strlen(p) > 63)
      return AT_R("+ERROR: mDNS hostname max 63 chars");
    strncpy((char *)&cfg.mdns_hostname, p, sizeof(cfg.mdns_hostname) - 1);
    cfg.mdns_hostname[sizeof(cfg.mdns_hostname) - 1] = '\0';
    CFG_SAVE();
    if(WiFi.status() == WL_CONNECTED && cfg.mdns_enabled){
      stop_mdns();
      setup_mdns();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+MDNS_HOSTNAME?", atcmdline, cmd_len)){
    if(strlen(cfg.mdns_hostname) == 0){
      if(strlen(cfg.hostname) == 0)
        return AT_R_STR(DEFAULT_HOSTNAME);
      else
        return AT_R_STR(cfg.hostname);
    } else {
      return AT_R_STR(cfg.mdns_hostname);
    }
  #endif // SUPPORT_MDNS
  } else if(p = at_cmd_check("AT+IPV4=", atcmdline, cmd_len)){
    String params = String(p);
    params.trim();

    if(params.equalsIgnoreCase("DHCP")){
      // Enable IPv4 DHCP
      cfg.ip_mode = (cfg.ip_mode & ~IPV4_STATIC) | IPV4_DHCP;
      memset(cfg.ipv4_addr, 0, sizeof(cfg.ipv4_addr));
      memset(cfg.ipv4_gw, 0, sizeof(cfg.ipv4_gw));
      memset(cfg.ipv4_mask, 0, sizeof(cfg.ipv4_mask));
      memset(cfg.ipv4_dns, 0, sizeof(cfg.ipv4_dns));
    } else if(params.equalsIgnoreCase("DISABLE")){
      // Disable IPv4
      cfg.ip_mode &= ~(IPV4_DHCP | IPV4_STATIC);
      memset(cfg.ipv4_addr, 0, sizeof(cfg.ipv4_addr));
      memset(cfg.ipv4_gw, 0, sizeof(cfg.ipv4_gw));
      memset(cfg.ipv4_mask, 0, sizeof(cfg.ipv4_mask));
      memset(cfg.ipv4_dns, 0, sizeof(cfg.ipv4_dns));
    } else {
      // Parse static IPv4: ip,netmask,gateway[,dns]
      int commaPos1 = params.indexOf(',');
      int commaPos2 = params.indexOf(',', commaPos1 + 1);
      int commaPos3 = params.indexOf(',', commaPos2 + 1);

      if(commaPos1 == -1 || commaPos2 == -1)
        return AT_R("+ERROR: IPv4 options: DHCP, DISABLE, or ip,netmask,gateway[,dns]");

      String ip = params.substring(0, commaPos1);
      String netmask = params.substring(commaPos1 + 1, commaPos2);
      String gateway = params.substring(commaPos2 + 1, commaPos3 == -1 ? params.length() : commaPos3);
      String dns = commaPos3 == -1 ? DEFAULT_DNS_IPV4 : params.substring(commaPos3 + 1);

      // Parse IP addresses
      if(!ip.length() || !netmask.length() || !gateway.length())
        return AT_R("+ERROR: missing ip, netmask, or gateway");

      // Parse and validate IP address
      int ip_parts[4], mask_parts[4], gw_parts[4], dns_parts[4];
      if(sscanf(ip.c_str(), "%d.%d.%d.%d", &ip_parts[0], &ip_parts[1], &ip_parts[2], &ip_parts[3]) != 4 ||
         sscanf(netmask.c_str(), "%d.%d.%d.%d", &mask_parts[0], &mask_parts[1], &mask_parts[2], &mask_parts[3]) != 4 ||
         sscanf(gateway.c_str(), "%d.%d.%d.%d", &gw_parts[0], &gw_parts[1], &gw_parts[2], &gw_parts[3]) != 4 ||
         sscanf(dns.c_str(), "%d.%d.%d.%d", &dns_parts[0], &dns_parts[1], &dns_parts[2], &dns_parts[3]) != 4){
        return AT_R("+ERROR: invalid IP address format");
      }

      // Validate IP ranges (0-255)
      for(int i = 0; i < 4; i++){
        if(ip_parts[i] < 0 || ip_parts[i] > 255 || mask_parts[i] < 0 || mask_parts[i] > 255 ||
           gw_parts[i] < 0 || gw_parts[i] > 255 || dns_parts[i] < 0 || dns_parts[i] > 255)
          return AT_R("+ERROR: IP address parts must be 0-255");
        cfg.ipv4_addr[i] = ip_parts[i];
        cfg.ipv4_mask[i] = mask_parts[i];
        cfg.ipv4_gw[i] = gw_parts[i];
        cfg.ipv4_dns[i] = dns_parts[i];
      }

      // Enable static IPv4
      cfg.ip_mode = (cfg.ip_mode & ~IPV4_DHCP) | IPV4_STATIC;
    }

    CFG_SAVE();
    reset_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+IPV4?", atcmdline, cmd_len)){
    String response;
    if(cfg.ip_mode & IPV4_DHCP){
      response = "DHCP";
    } else if(cfg.ip_mode & IPV4_STATIC){
      response = String(cfg.ipv4_addr[0]) + "." + String(cfg.ipv4_addr[1]) + "." +
                 String(cfg.ipv4_addr[2]) + "." + String(cfg.ipv4_addr[3]) + "," +
                 String(cfg.ipv4_mask[0]) + "." + String(cfg.ipv4_mask[1]) + "." +
                 String(cfg.ipv4_mask[2]) + "." + String(cfg.ipv4_mask[3]) + "," +
                 String(cfg.ipv4_gw[0]) + "." + String(cfg.ipv4_gw[1]) + "." +
                 String(cfg.ipv4_gw[2]) + "." + String(cfg.ipv4_gw[3]) + "," +
                 String(cfg.ipv4_dns[0]) + "." + String(cfg.ipv4_dns[1]) + "." +
                 String(cfg.ipv4_dns[2]) + "." + String(cfg.ipv4_dns[3]);
    } else {
      response = "DISABLED";
    }
    return AT_R_S(response);
  } else if(p = at_cmd_check("AT+IPV6=", atcmdline, cmd_len)){
    String params = String(p);
    params.trim();

    if(params.equalsIgnoreCase("DHCP")){
      // Enable IPv6 DHCP
      cfg.ip_mode |= IPV6_SLAAC;
    } else if(params.equalsIgnoreCase("DISABLE")){
      // Disable IPv6
      cfg.ip_mode &= ~IPV6_SLAAC;
    } else {
      return AT_R("+ERROR: IPv6 options: DHCP or DISABLE");
    }

    CFG_SAVE();
    reset_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+IPV6?", atcmdline, cmd_len)){
    if(cfg.ip_mode & IPV6_SLAAC)
      return AT_R("DHCP");
    else
      return AT_R("DISABLED");
  } else if(p = at_cmd_check("AT+IP_STATUS?", atcmdline, cmd_len)){
    String response = "";
    bool hasIP = false;

    // Check WiFi connection status first
    if(WiFi.status() != WL_CONNECTED && WiFi.status() != WL_IDLE_STATUS)
      return AT_R("+ERROR: WiFi not connected");

    // IPv4 status
    IPAddress ipv4 = WiFi.localIP();
    if(ipv4 != IPAddress(0,0,0,0) && ipv4 != IPAddress(127,0,0,1)){
      response += "IPv4: " + ipv4.toString();
      IPAddress gateway = WiFi.gatewayIP();
      IPAddress subnet = WiFi.subnetMask();
      IPAddress dns = WiFi.dnsIP();
      if(gateway != IPAddress(0,0,0,0)){
        response += ", gw: " + gateway.toString();
      }
      if(subnet != IPAddress(0,0,0,0)){
        response += ", nm: " + subnet.toString();
      }
      if(dns != IPAddress(0,0,0,0)){
        response += ", dns: " + dns.toString();
      }
      hasIP = true;
    }

    // IPv6 status
    if(cfg.ip_mode & IPV6_SLAAC){
      IPAddress ipv6_global = WiFi.globalIPv6();
      IPAddress ipv6_local = WiFi.linkLocalIPv6();

      if(ipv6_global != IPAddress((uint32_t)0)){
        if(hasIP)
            response += "\n";
        response += "IPv6 global: " + ipv6_global.toString();
        hasIP = true;
      }

      if(ipv6_local != IPAddress((uint32_t)0)){
        if(hasIP)
            response += "\n";
        response += "IPv6 link-local: " + ipv6_local.toString();
        hasIP = true;
      }
    }

    if(!hasIP)
      response = "No IP addresses assigned";
    return AT_R_S(response);
  #ifdef SUPPORT_TCP
  } else if(p = at_cmd_check("AT+TCP_STATUS?", atcmdline, cmd_len)){
    String response = "";
    if(strlen(cfg.tcp_host_ip) == 0 || cfg.tcp_port == 0){
      response = "TCP not configured";
    } else {
      response = "TCP Host: " + String(cfg.tcp_host_ip) + ":" + String(cfg.tcp_port);
    }
    return AT_R_S(response);
  #endif // SUPPORT_TCP
  #ifdef SUPPORT_TLS
  } else if(p = at_cmd_check("AT+TLS_ENABLE?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.tls_enabled);
  } else if(p = at_cmd_check("AT+TLS_ENABLE=", atcmdline, cmd_len)){
    uint8_t new_tls_enabled = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || (new_tls_enabled != 0 && new_tls_enabled != 1) || (r == p))
      return AT_R("+ERROR: TLS enable must be 0 or 1");
    if(new_tls_enabled != cfg.tls_enabled){
      cfg.tls_enabled = new_tls_enabled;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_PORT?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.tls_port);
  } else if(p = at_cmd_check("AT+TLS_PORT=", atcmdline, cmd_len)){
    if(strlen(p) == 0){
      // Empty string means use tcp_port
      cfg.tls_port = 0;
      CFG_SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_tls_port = (uint16_t)strtol(p, &r, 10);
    if(new_tls_port == 0 || errno != 0 || new_tls_port > 65535 || (r == p))
      return AT_R("+ERROR: invalid TLS port");
    if(new_tls_port != cfg.tls_port){
      cfg.tls_port = new_tls_port;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_VERIFY?", atcmdline, cmd_len)){
    const char* verify_modes[] = {"none", "optional", "required"};
    return AT_R_STR(verify_modes[cfg.tls_verify_mode]);
  } else if(p = at_cmd_check("AT+TLS_VERIFY=", atcmdline, cmd_len)){
    uint8_t new_verify_mode = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || new_verify_mode > 2 || (r == p))
      return AT_R("+ERROR: TLS verify mode must be 0-2 (0=none, 1=optional, 2=required)");
    if(new_verify_mode != cfg.tls_verify_mode){
      cfg.tls_verify_mode = new_verify_mode;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_SNI?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.tls_use_sni);
  } else if(p = at_cmd_check("AT+TLS_SNI=", atcmdline, cmd_len)){
    uint8_t new_sni = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || (new_sni != 0 && new_sni != 1) || (r == p))
      return AT_R("+ERROR: TLS SNI must be 0 or 1");
    if(new_sni != cfg.tls_use_sni){
      cfg.tls_use_sni = new_sni;
      CFG_SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_CA_CERT=", atcmdline, cmd_len)){
    if(strlen(p) >= sizeof(cfg.tls_ca_cert))
      return AT_R("+ERROR: CA certificate too long");
    strncpy(cfg.tls_ca_cert, p, sizeof(cfg.tls_ca_cert) - 1);
    cfg.tls_ca_cert[sizeof(cfg.tls_ca_cert) - 1] = '\0';
    CFG_SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_CA_CERT?", atcmdline, cmd_len)){
    return AT_R_STR(strlen(cfg.tls_ca_cert) > 0 ? "SET" : "NOT_SET");
  } else if(p = at_cmd_check("AT+TLS_CLIENT_CERT=", atcmdline, cmd_len)){
    if(strlen(p) >= sizeof(cfg.tls_client_cert))
      return AT_R("+ERROR: Client certificate too long");
    strncpy(cfg.tls_client_cert, p, sizeof(cfg.tls_client_cert) - 1);
    cfg.tls_client_cert[sizeof(cfg.tls_client_cert) - 1] = '\0';
    CFG_SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_CLIENT_CERT?", atcmdline, cmd_len)){
    return AT_R_STR(strlen(cfg.tls_client_cert) > 0 ? "SET" : "NOT_SET");
  } else if(p = at_cmd_check("AT+TLS_CLIENT_KEY=", atcmdline, cmd_len)){
    if(strlen(p) >= sizeof(cfg.tls_client_key))
      return AT_R("+ERROR: Client key too long");
    strncpy(cfg.tls_client_key, p, sizeof(cfg.tls_client_key) - 1);
    cfg.tls_client_key[sizeof(cfg.tls_client_key) - 1] = '\0';
    CFG_SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_CLIENT_KEY?", atcmdline, cmd_len)){
    return AT_R_STR(strlen(cfg.tls_client_key) > 0 ? "SET" : "NOT_SET");
  } else if(p = at_cmd_check("AT+TLS_PSK_IDENTITY=", atcmdline, cmd_len)){
    if(strlen(p) >= sizeof(cfg.tls_psk_identity))
      return AT_R("+ERROR: PSK identity too long");
    strncpy(cfg.tls_psk_identity, p, sizeof(cfg.tls_psk_identity) - 1);
    cfg.tls_psk_identity[sizeof(cfg.tls_psk_identity) - 1] = '\0';
    CFG_SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_PSK_IDENTITY?", atcmdline, cmd_len)){
    return AT_R_STR(strlen(cfg.tls_psk_identity) > 0 ? "SET" : "NOT_SET");
  } else if(p = at_cmd_check("AT+TLS_PSK_KEY=", atcmdline, cmd_len)){
    if(strlen(p) >= sizeof(cfg.tls_psk_key))
      return AT_R("+ERROR: PSK key too long");
    strncpy(cfg.tls_psk_key, p, sizeof(cfg.tls_psk_key) - 1);
    cfg.tls_psk_key[sizeof(cfg.tls_psk_key) - 1] = '\0';
    CFG_SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_PSK_KEY?", atcmdline, cmd_len)){
    return AT_R_STR(strlen(cfg.tls_psk_key) > 0 ? "SET" : "NOT_SET");
  } else if(p = at_cmd_check("AT+TLS_STATUS?", atcmdline, cmd_len)){
    String response = "";
    if(!cfg.tls_enabled){
      response = "TLS disabled";
    } else if(strlen(cfg.tcp_host_ip) == 0 || (cfg.tcp_port == 0 && cfg.tls_port == 0)){
      response = "TLS not configured";
    } else {
      uint16_t port_to_use = cfg.tls_port ? cfg.tls_port : cfg.tcp_port;
      response = "TLS Host: " + String(cfg.tcp_host_ip) + ":" + String(port_to_use);
      response += ", Status: " + String(tls_connected ? "connected" : "disconnected");
      if(tls_connected && tls_handshake_complete) {
        response += ", Secure: yes";
        // Note: Cipher suite info varies by platform and may not be available
      }
    }
    return AT_R_S(response);
  } else if(p = at_cmd_check("AT+TLS_CONNECT", atcmdline, cmd_len)){
    if(!cfg.tls_enabled)
      return AT_R("+ERROR: TLS is disabled");
    if(strlen(cfg.tcp_host_ip) == 0 || (cfg.tcp_port == 0 && cfg.tls_port == 0))
      return AT_R("+ERROR: TLS host/port not configured");
    connect_tls();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TLS_DISCONNECT", atcmdline, cmd_len)){
    close_tls_connection();
    return AT_R_OK;
  #endif // SUPPORT_TLS
  #endif // SUPPORT_WIFI
  } else if(p = at_cmd_check("AT+ERASE", atcmdline, cmd_len)){
    // Erase all configuration and reset to factory defaults
    LOG("[ERASE] Erasing configuration and resetting to factory defaults");

    #ifdef SUPPORT_WIFI
    // Stop all network connections before erasing config
    stop_networking();
    #endif // SUPPORT_WIFI

    Serial.flush();
    CFG_CLEAR();

    // Clear the config struct in memory
    memset(&cfg, 0, sizeof(cfg));

    // Reset configuration initialized flag to force reinitialization on next boot
    cfg.initialized = 0;
    cfg.version = 0;

    // Restart immediately
    resetFunc();
  } else if(p = at_cmd_check("AT+RESET", atcmdline, cmd_len)){
    resetFunc();
  } else if(p = at_cmd_check("AT+HELP?", atcmdline, cmd_len)){
    return AT_R_F(AT_help_string);
  } else if(p = at_cmd_check("AT+?", atcmdline, cmd_len)){
    return AT_R_F(AT_short_help_string);
  #ifdef BLUETOOTH_UART_AT
  } else if(p = at_cmd_check("AT+BLE_PIN=", atcmdline, cmd_len)){
    if(strlen(p) != 6)
      return AT_R("+ERROR: BLE PIN must be exactly 6 digits");
    uint32_t pin = strtoul(p, &r, 10); // Just to check for conversion errors
    if(errno != 0 || r == p)
      return AT_R("+ERROR: BLE PIN invalid, must be 6 digits");
    cfg.ble_pin = pin;
    CFG_SAVE();
    // Restart BLE with new PIN
    uint8_t want_advertising = (ble_advertising_start != 0);
    destroy_ble();
    setup_ble();
    if(want_advertising)
      start_advertising_ble();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+BLE_PIN?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.ble_pin);
  } else if(p = at_cmd_check("AT+BLE_SECURITY=", atcmdline, cmd_len)){
    uint8_t mode = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || mode > 2 || (p == r))
      return AT_R("+ERROR: BLE security mode must be 0-2 (0=None, 1=PIN, 2=Bonding)");
    if(mode == 1 && cfg.ble_pin == 0)
      return AT_R("+ERROR: Cannot set security mode to PIN when PIN is 0 (no PIN). Set a valid PIN first with AT+BLE_PIN.");
    if(mode == 2 && cfg.ble_pin == 0 && cfg.ble_io_cap == 3)
      return AT_R("+ERROR: Cannot set security mode to Bonding when PIN is 0 and IO capability is NoInputNoOutput. Set a valid PIN or change IO capability first.");
    if(mode != cfg.ble_security_mode){
      LOG("[BLE] Setting security mode to %d", mode);
      cfg.ble_security_mode = mode;
      CFG_SAVE();
      // Restart BLE with new PIN
      uint8_t want_advertising = (ble_advertising_start != 0);
      destroy_ble();
      setup_ble();
      if(want_advertising)
        start_advertising_ble();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+BLE_SECURITY?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.ble_security_mode);
  } else if(p = at_cmd_check("AT+BLE_IO_CAP=", atcmdline, cmd_len)){
    uint8_t cap = strtol(p, &r, 10);
    if(errno != 0 || cap > 4 || (p == r))
      return AT_R("+ERROR: BLE IO capability must be 0-4 (0=DisplayOnly, 1=DisplayYesNo, 2=KeyboardOnly, 3=NoInputNoOutput, 4=KeyboardDisplay)");
    if(cap == 3 && cfg.ble_security_mode == 2 && cfg.ble_pin == 0)
      return AT_R("+ERROR: Cannot set IO capability to NoInputNoOutput when security mode is Bonding and PIN is 0. Set a valid PIN or change security mode first.");
    if(cap != cfg.ble_io_cap){
      LOG("[BLE] Setting IO capability to %d", cap);
      cfg.ble_io_cap = cap;
      // Restart BLE with new IO capability
      uint8_t want_advertising = (ble_advertising_start != 0);
      destroy_ble();
      setup_ble();
      if(want_advertising)
        start_advertising_ble();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+BLE_IO_CAP?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.ble_io_cap);
  } else if(p = at_cmd_check("AT+BLE_AUTH_REQ=", atcmdline, cmd_len)){
    uint8_t req = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || req > 3 || (p == r))
      return AT_R("+ERROR: BLE auth requirements must be 0-3 (0=None, 1=Bonding, 2=MITM, 3=Bonding+MITM)");
    if(req != cfg.ble_auth_req) {
      LOG("[BLE] Setting auth requirements to %d", req);
      cfg.ble_auth_req = req;
      CFG_SAVE();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+BLE_AUTH_REQ?", atcmdline, cmd_len)){
    return AT_R_INT(cfg.ble_auth_req);
  } else if(p = at_cmd_check("AT+BLE_STATUS?", atcmdline, cmd_len)){
    String status = "BLE: ";
    if(ble_advertising_start == 0) {
      status += "disabled";
    } else if(deviceConnected == 1) {
      status += "connected";
      if(securityRequestPending == 1) {
        status += ", security pending";
      } else {
        status += ", authenticated";
      }
      if(passkeyForDisplay != 0) {
        status += ", PIN: " + String(passkeyForDisplay, DEC);
      }
    } else {
      status += "advertising";
    }
    status += ", Security mode: " + String(cfg.ble_security_mode);
    status += ", IO cap: " + String(cfg.ble_io_cap);
    status += ", Auth req: " + String(cfg.ble_auth_req);
    status += ", Addr type: " + String(get_ble_addr_type_name(cfg.ble_addr_type));
    return AT_R_S(status);
  } else if(p = at_cmd_check("AT+BLE_ADDR_TYPE=", atcmdline, cmd_len)){
    uint8_t type = (uint8_t)strtol(p, &r, 10);
    if(errno != 0 || (p == r && type == 0) || type > 3)
      return AT_R("+ERROR: BLE address type must be 0-3 (0=Public, 1=Random Static, 2=Private Resolvable, 3=Private Non-resolvable)");
    if(type != cfg.ble_addr_type){
      LOG("[BLE] Setting address type to %d", type);
      // If changing from or to public address, need to restart BLE
      bool restart_ble = (type == 0 || cfg.ble_addr_type == 0);
      cfg.ble_addr_type = type;
      if(restart_ble){
        uint8_t want_advertising = (ble_advertising_start != 0);
        destroy_ble();
        setup_ble();
        if(want_advertising)
          start_advertising_ble();
      }
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+BLE_ADDR_TYPE?", atcmdline, cmd_len)){
    return AT_R_STR(cfg.ble_addr_type == 0 ? "0 (public)" :
                    cfg.ble_addr_type == 1 ? "1 (random static)" :
                    cfg.ble_addr_type == 2 ? "2 (private resolvable)" :
                    cfg.ble_addr_type == 3 ? "3 (private non-resolvable)" : "unknown");
  } else if(p = at_cmd_check("AT+BLE_ADDR=", atcmdline, cmd_len)){
    // Parse MAC address in format XX:XX:XX:XX:XX:XX
    uint8_t addr[6];
    int parsed = sscanf(p, "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
                       &addr[0], &addr[1], &addr[2], &addr[3], &addr[4], &addr[5]);
    if(parsed != 6) {
      return AT_R("+ERROR: Invalid MAC address format. Use XX:XX:XX:XX:XX:XX");
    }

    // Validate address based on type
    if(cfg.ble_addr_type == 1) { // Random static
      if(!is_valid_static_random_address(addr)) {
        return AT_R("+ERROR: Invalid static random address. First byte must be 0xC0-0xFF");
      }
    } else if(!is_valid_ble_address(addr)) {
      return AT_R("+ERROR: Invalid address (all zeros)");
    }

    memcpy(cfg.ble_custom_addr, addr, 6);
    CFG_SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+BLE_ADDR?", atcmdline, cmd_len)){
    return AT_R_S(get_current_ble_address());
  } else if(p = at_cmd_check("AT+BLE_ADDR_GEN?", atcmdline, cmd_len)){
    // Generate a new random static address
    uint8_t new_addr[6];
    generate_static_random_address(new_addr);
    memcpy(cfg.ble_custom_addr, new_addr, 6);
    cfg.ble_addr_type = 1; // Set to random static
    CFG_SAVE();

    char addr_str[18];
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X",
            new_addr[0], new_addr[1], new_addr[2], new_addr[3], new_addr[4], new_addr[5]);
    return AT_R_S(String("Generated: ") + String(addr_str));
  #endif // BLUETOOTH_UART_AT
  } else {
    return AT_R("+ERROR: unknown command");
  }
  return AT_R("+ERROR: unknown error");
}
#endif // BLUETOOTH_UART_AT && BT_BLE

size_t inlen = 0;

// BLE UART Service - Nordic UART Service UUID
#if defined(BLUETOOTH_UART_AT) && defined(BT_BLE)

BLEServer* pServer = NULL;
BLEService* pService = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pRxCharacteristic = NULL;

// BLE UART buffer
ALIGN(4) char ble_cmd_buffer[128] = {0};
bool bleCommandReady = false;
bool bleCommandProcessing = false; // Flag to prevent re-entrant command processing

// BLE negotiated MTU (default to AT buffer size)
#define BLE_MTU_MIN     128
#define BLE_MTU_MAX     512
#define BLE_MTU_DEFAULT 128

uint16_t ble_mtu = BLE_MTU_DEFAULT;

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      doYIELD;
      deviceConnected = 1;
      securityRequestPending = 0;
      LOG("[BLE] connected, MTU: %d", pServer->getPeerMTU(1));
    };

    void onDisconnect(BLEServer* pServer) {
      doYIELD;
      deviceConnected = 0;
      securityRequestPending = 0;
      passkeyForDisplay = 0;
      ble_advertising_start = 0;
      LOG("[BLE] disconnected");
    }

    // TODO: use/fix once ESP32 BLE MTU negotiation is implemented
    #if defined(ESP32) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
    void onMTU(uint16_t mtu, BLEServer* /*pServer*/) {
      doYIELD;
      if (mtu < BLE_MTU_MIN) {
        LOG("[BLE] MTU request too small (%d), keeping %d", mtu, BLE_MTU_DEFAULT);
        return;
      }
      if (mtu > BLE_MTU_MAX)
          mtu = BLE_MTU_MAX;
      if (mtu > BLE_MTU_MIN) {
        ble_mtu = mtu;
        BLEDevice::setMTU(ble_mtu);
        LOG("[BLE] MTU set to: %d", ble_mtu);
      } else {
        LOG("[BLE] MTU unchanged (current: %d, requested: %d)", ble_mtu, mtu);
      }
    }
    #endif // ESP32
};

// BLE Security Callbacks
class MySecurity : public BLESecurityCallbacks {
  uint32_t onPassKeyRequest() {
    doYIELD;
    LOG("[BLE Security] PassKey Request, using static PIN: %06d", cfg.ble_pin);
    return cfg.ble_pin;
  }

  void onPassKeyNotify(uint32_t pass_key) {
    doYIELD;
    passkeyForDisplay = pass_key;
    LOG("[BLE Security] PassKey Notify: %06d", pass_key);
    // Notify via UART that PIN is being displayed
    ble_send_response(("+BLEPIN: " + String(pass_key, DEC)).c_str());
  }

  bool onConfirmPIN(uint32_t pass_key) {
    doYIELD;
    LOG("[BLE Security] Confirm PIN: %06d", pass_key);
    // Auto-confirm for now, could be enhanced with user interaction
    return true;
  }

  bool onSecurityRequest() {
    doYIELD;
    LOG("[BLE Security] Security Request");
    securityRequestPending = 1;
    return true;
  }

  void onAuthenticationComplete(ble_gap_conn_desc* desc) {
    doYIELD;
    securityRequestPending = 0;
    if (desc) {
      LOG("[BLE Security] Authentication Complete - connection handle: %d", desc->conn_handle);
      ble_send_response("+BLEAUTH: SUCCESS");
    } else {
      LOG("[BLE Security] Authentication Failed");
      ble_send_response("+BLEAUTH: FAILED");
    }
  }
};

// BLE Characteristic Callbacks

// Temporary buffer for incoming BLE data when not in AT mode
#define BLE_UART1_READ_BUFFER_SIZE   BLE_MTU_MAX*8
ALIGN(4) uint8_t ble_rx_buffer[BLE_UART1_READ_BUFFER_SIZE] = {0};
uint16_t ble_rx_len = 0;

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pC) {
      doYIELD;
      // get value written to characteristic
      String v = pC->getValue();
      size_t b_len = v.length();
      uint8_t* ble_rx_buf = (uint8_t*)v.c_str();
      D("[BLE] RX LEN: %d,BUF>>%s<<", b_len, ble_rx_buf);

      // Ignore empty writes
      if(b_len == 0)
        return;

      #ifdef SUPPORT_BLE_UART1
      // When NOT in AT mode, store data in buffer for later use (e.g. UART1 bridge)
      if(cfg.ble_uart1_bridge == 1 && at_mode == BRIDGE_MODE) {
        if(ble_rx_len >= BLE_UART1_READ_BUFFER_SIZE) {
          // Buffer full, drop incoming data
          D("[BLE] RX buffer full, dropping data, got %d bytes, buffer size: %d", b_len, BLE_UART1_READ_BUFFER_SIZE);
          return;
        } else {
          D("[BLE] in AT bridge mode, keeping data in buffer");
          b_len = min(b_len, (size_t)(BLE_UART1_READ_BUFFER_SIZE - ble_rx_len));
          memcpy(ble_rx_buffer + ble_rx_len, ble_rx_buf, b_len);
          ble_rx_len += b_len;
          return;
        }
      }
      #endif // SUPPORT_BLE_UART1

      // When in AT command mode, add data to command buffer + check \n or \r terminators
      // Process each byte individually to handle command terminators properly
      memset(ble_cmd_buffer, 0, sizeof(ble_cmd_buffer));
      char *ble_ptr = ble_cmd_buffer;
      bleCommandReady = false;
      for (size_t i = 0; i < b_len; i++) {
        doYIELD;
        D("[BLE] RX CHAR: %02X '%c'", *ble_rx_buf, isprint(*ble_rx_buf) ? *ble_rx_buf : '.');
        if (*ble_rx_buf == '\n' || *ble_rx_buf == '\r') {
          // Command terminator found, mark command as ready
          bleCommandReady = true;
          break;
        }

        // Check if command buffer is too long
        if (ble_ptr - ble_cmd_buffer >= sizeof(ble_cmd_buffer) - 1) {
          D("[BLE] Command buffer full, clearing buffer");
          memset(ble_cmd_buffer, 0, sizeof(ble_cmd_buffer));
          break;
        }

        // Add character to command buffer
        *ble_ptr++ = (char)*ble_rx_buf++;
      }
      if(bleCommandReady)
        D("[BLE] Command Ready: %d, %d, %s", bleCommandReady, strlen(ble_cmd_buffer), ble_cmd_buffer);
    }
};

// BLE MAC Address utility functions
NOINLINE
bool is_valid_ble_address(const uint8_t* addr) {
  // Check if address is all zeros (invalid)
  for(int i = 0; i < 6; i++) {
    if(addr[i] != 0) return true;
  }
  return false;
}

NOINLINE
bool is_valid_static_random_address(const uint8_t* addr) {
  // Static random addresses must have the two most significant bits set to '11'
  // This means the first byte (MSB) must be in range 0xC0-0xFF
  return (addr[0] & 0xC0) == 0xC0 && is_valid_ble_address(addr);
}

NOINLINE
void generate_static_random_address(uint8_t* addr) {
  // Generate a valid static random address
  // First byte must have MSBs = 11 (0xC0-0xFF)
  addr[0] = 0xC0 + (esp_random() & 0x3F);

  // Remaining 5 bytes can be any value except all zeros
  for(int i = 1; i < 6; i++) {
    addr[i] = esp_random() & 0xFF;
  }

  // Ensure address is not all zeros (except for the fixed MSBs)
  uint32_t sum = 0;
  for(int i = 1; i < 6; i++) {
    sum += addr[i];
  }

  // If all random bytes are zero, set the last one to 1
  if(sum == 0) {
    addr[5] = 1;
  }
}

NOINLINE
const char* get_ble_addr_type_name(uint8_t type) {
  switch(type) {
    case 0: return "Public";
    case 1: return "Random Static";
    case 2: return "Private Resolvable";
    case 3: return "Private Non-resolvable";
    default: return "Unknown";
  }
}

NOINLINE
void setup_ble_address() {
  LOG("[BLE] Configuring MAC address (type: %s)", get_ble_addr_type_name(cfg.ble_addr_type));

  switch(cfg.ble_addr_type) {
    case 0: // Public address
      if(is_valid_ble_address(cfg.ble_custom_addr)) {
        // Try to set custom public address using NimBLE
        ble_addr_t addr;
        addr.type = BLE_ADDR_PUBLIC;
        memcpy(addr.val, cfg.ble_custom_addr, 6);

        int rc = ble_hs_id_set_rnd(addr.val);
        if(rc == 0) {
          LOG("[BLE] Set custom public address: %02X:%02X:%02X:%02X:%02X:%02X",
              cfg.ble_custom_addr[0], cfg.ble_custom_addr[1], cfg.ble_custom_addr[2],
              cfg.ble_custom_addr[3], cfg.ble_custom_addr[4], cfg.ble_custom_addr[5]);
        } else {
          LOG("[BLE] Failed to set custom public address (error: %d), using default", rc);
        }
      } else {
        LOG("[BLE] Using default public address");
      }
      break;

    case 1: // Random static address
      {
        uint8_t static_addr[6];
        bool address_set = false;

        if(is_valid_static_random_address(cfg.ble_custom_addr)) {
          // Use provided static random address
          memcpy(static_addr, cfg.ble_custom_addr, 6);
          LOG("[BLE] Using configured static random address");
        } else if(cfg.ble_addr_auto_random) {
          // Auto-generate static random address
          generate_static_random_address(static_addr);
          LOG("[BLE] Generated new static random address");
          // Save the generated address back to config
          memcpy(cfg.ble_custom_addr, static_addr, 6);
        } else {
          LOG("[BLE] Invalid static random address provided and auto-generation disabled");
          return;
        }

        // Set the random static address using NimBLE API
        int rc = ble_hs_id_set_rnd(static_addr);
        if(rc == 0) {
          LOG("[BLE] Set static random address: %02X:%02X:%02X:%02X:%02X:%02X",
              static_addr[0], static_addr[1], static_addr[2],
              static_addr[3], static_addr[4], static_addr[5]);
          address_set = true;
        } else {
          LOG("[BLE] Failed to set static random address (error: %d)", rc);
        }

        if(!address_set) {
          LOG("[BLE] Falling back to default address generation");
        }
      }
      break;

    case 2: // Private resolvable address
      LOG("[BLE] Private resolvable addresses require IRK setup");
      // Enable privacy mode with resolvable addresses
      // This would typically involve setting up an IRK and enabling privacy
      LOG("[BLE] Privacy mode not fully implemented yet");
      break;

    case 3: // Private non-resolvable address
      LOG("[BLE] Private non-resolvable addresses change automatically");
      // These addresses are handled automatically by the BLE stack
      break;

    default:
      LOG("[BLE] Unknown address type %d, using default public address", cfg.ble_addr_type);
      break;
  }
}

NOINLINE
String get_current_ble_address() {
  char addr_str[18];
  String response;

  // Try to get the actual address being used by the BLE stack
  ble_addr_t addr;
  int rc = ble_hs_id_copy_addr(BLE_ADDR_RANDOM, addr.val, NULL);

  if(rc == 0) {
    // Successfully got random address from stack
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X",
            addr.val[5], addr.val[4], addr.val[3], addr.val[2], addr.val[1], addr.val[0]);
    response = String(addr_str) + " (active random)";
  } else {
    // Try to get public address
    rc = ble_hs_id_copy_addr(BLE_ADDR_PUBLIC, addr.val, NULL);
    if(rc == 0) {
      sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X",
              addr.val[5], addr.val[4], addr.val[3], addr.val[2], addr.val[1], addr.val[0]);
      response = String(addr_str) + " (active public)";
    } else {
      // Fallback to configured address
      if(is_valid_ble_address(cfg.ble_custom_addr)) {
        sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X",
                cfg.ble_custom_addr[0], cfg.ble_custom_addr[1], cfg.ble_custom_addr[2],
                cfg.ble_custom_addr[3], cfg.ble_custom_addr[4], cfg.ble_custom_addr[5]);
        response = String(addr_str) + " (configured)";
      } else {
        response = "Default address (type: " + String(get_ble_addr_type_name(cfg.ble_addr_type)) + ")";
      }
    }
  }

  return response;
}

// called from AT command handler when changes are made
NOINLINE
void destroy_ble() {
  if(ble_advertising_start != 0) {
    ble_advertising_start = 0;
    deviceConnected = 0;
    securityRequestPending = 0;
    passkeyForDisplay = 0;
    BLEDevice::deinit(false);
    delay(100);
    LOG("[BLE] Deinitialized");
  }
}

NOINLINE
void setup_ble() {
  LOG("[BLE] Setup");

  // Create the BLE Device
  BLEDevice::init(BLUETOOTH_UART_DEVICE_NAME);
  BLEDevice::setMTU(ble_mtu); // Request MTU matching AT buffer size

  // Configure BLE MAC address after BLE init but before services
  setup_ble_address();

  // Configure BLE Security based on configuration
  if(cfg.ble_security_mode > 0) {
    LOG("[BLE] Security mode %d requested", cfg.ble_security_mode);

    // Set security callbacks for NimBLE
    BLEDevice::setSecurityCallbacks(new MySecurity());

    // Log PIN configuration
    if(cfg.ble_security_mode == 1) {
      LOG("[BLE] Security: PIN mode with static PIN: %06d", cfg.ble_pin);
    } else if(cfg.ble_security_mode == 1) {
      LOG("[BLE] Security: PIN mode with dynamic PIN");
    } else {
      LOG("[BLE] Security: Basic encryption enabled");
    }
  } else {
    LOG("[BLE] Security: None");
  }

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  LOG("[BLE] Server created");
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  pService = pServer->createService(SERVICE_UUID);
  LOG("[BLE] Service created");

  // Create a BLE Characteristic for TX (notifications to client)
  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  LOG("[BLE] TX Characteristic created");

  pTxCharacteristic->addDescriptor(new BLE2902());
  LOG("[BLE] TX Descriptor added");

  // Create a BLE Characteristic for RX (writes from client)
  pRxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );
  LOG("[BLE] RX Characteristic created");

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();
  LOG("[BLE] Service started");

  // advertising config
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  // set value to 0x00 to not advertise this parameter
  pAdvertising->setMinPreferred(0x0);

  // don't start advertising
  ble_advertising_start = 0;
  deviceConnected = 0;

  LOG("[BLE] Setup complete");
}

void handle_ble_command() {
  // Don't handle commands if BLE is disabled or already processing a command
  if (bleCommandProcessing){
    LOG("[BLE] Command processing already in progress, skipping command %s", ble_cmd_buffer);
    return;
  }

  size_t cmd_len = strlen(ble_cmd_buffer);
  if (bleCommandReady && cmd_len > 0) {
    bleCommandProcessing = true; // Set flag to prevent re-entrant calls
    // Process the BLE command using the same AT command handler
    D("[BLE] Processing command: %d, size:%d, buf:>>%s<<", ble_advertising_start, cmd_len, ble_cmd_buffer);

    #ifdef DEBUG
    // buffer log/debug in hex
    D("[BLE] command buffer in hex: ");
    for (size_t i = 0; i < cmd_len; i++) {
      R("%02X", (unsigned char)ble_cmd_buffer[i]);
    }
    R("\n");
    #endif // DEBUG

    // Check if the command starts with "AT"
    if(cmd_len >= 2 && strncmp(ble_cmd_buffer, "AT", 2) == 0) {
      // Handle AT command
      ble_send_response(at_cmd_handler(ble_cmd_buffer));
    } else {
      ble_send_response((const char*)("+ERROR: invalid command"));
    }

    // Clear command buffer and flags
    memset(ble_cmd_buffer, 0, sizeof(ble_cmd_buffer));
    bleCommandReady = false;
    bleCommandProcessing = false;
    D("[BLE] Command processing complete");
  }
}

NOINLINE
void ble_send_response(const char *response) {
  if (ble_advertising_start == 0 || deviceConnected == 0 || !pTxCharacteristic)
    return;

  // sanity check
  if(response == NULL || strlen(response) == 0)
    return;

  // Send response with line terminator
  uint8_t ok = 1;
  ok &= ble_send_n((const uint8_t *)response, strlen(response));
  ok &= ble_send_n((const uint8_t *)("\r\n"), 2);
  if(!ok) {
    LOG("[BLE] Failed to send response, not connected anymore");
  }
}

NOINLINE
uint8_t ble_send_n(const uint8_t *bstr, size_t len) {
  if (ble_advertising_start == 0)
    return 0;

  /*
  #ifdef DEBUG
  D("[BLE] TX mtu: %d, connected: %d, length: %d", ble_mtu, deviceConnected, len);
  D("[BLE] TX mtu buffer in hex: ");
  for (uint16_t i = 0; i < len; i++) {
    R("%02X", (unsigned char)bstr[i]);
  }
  R("\n");
  D("[BLE] TX mtu buffer in ascii: ");
  for (uint16_t i = 0; i < len; i++) {
    if(bstr[i] == '\n') {
      R("\n");
    } else {
      R("%s", isprint(bstr[i]) ? (char[]){bstr[i], '\0'} : ".");
    }
  }
  R("\n");
  #endif // DEBUG
  */

  if (deviceConnected == 1 && pTxCharacteristic) {
    static size_t snr = 0;
    snr++;
    D("[BLE] Sending response, total length: %d", len);
    // Split response into chunks (BLE characteristic limit), use negotiated MTU
    size_t o = 0;
    size_t cs = 0;
    while (o < len && deviceConnected == 1) {
      doYIELD;
      // multitasking, can unset deviceConnected or pTxCharacteristic
      // double check after doYIELD
      if(pTxCharacteristic == NULL) {
        LOG("[BLE] Stopped sending, characteristic is NULL");
        break;
      }
      if(pService == NULL || pService->getServer() == NULL) {
        LOG("[BLE] Stopped sending, server is NULL while waiting to notify");
        break;
      }

      // chunk size ?
      cs = ble_mtu - 3;      // ATT_MTU-3 for payload
      cs = min(cs, len - o); // smaller of remaining
      if(cs == 0) {
        LOOP_D("[BLE] chunk size is 0");
        break;
      }
      #ifdef DEBUG
      T(""); R("[BLE] NOTIFY #%04d, len:%04d, chunk:%04d, sent:%04d, data: ", snr, len, cs, o);
      for(uint16_t i = 0; i < cs; i++) {
        R("%c", (unsigned char)bstr[o + i]);
      }
      R("\n");
      #endif // DEBUG

      // let's use ble_gatts_notify_custom() directly to get the proper error code
      uint16_t conn_handle = 0;
      for(auto &z: pService->getServer()->getPeerDevices(false)){
        conn_handle = z.first;
        break;
      }
      REDO_SEND: {
        // create m_buf in each loop iteration to avoid memory leak, it gets consumed with each call
        os_mbuf *ble_out_msg = ble_hs_mbuf_from_flat((uint8_t *)(bstr + o), cs);
        if(ble_out_msg == NULL){
          D("[BLE] notify failed, cannot allocate memory for %d bytes", cs);
          delayMicroseconds(100);
          goto REDO_SEND;
        }
        esp_err_t err = ble_gatts_notify_custom(conn_handle, pTxCharacteristic->getHandle(), ble_out_msg);
        if(err != ESP_OK) {
          D("[BLE] notify failed with error: a:%d, l:%d, c:%d, e:%d, %s", snr, inlen, cs, err, err == 6 ? "ENOMEM": "UNKNOWN");

          // doYIELD for other things, we're in a GOTO loop
          doYIELD;
          // after doYIELD, check if still connected and characteristic valid
          if(deviceConnected == 0) {
            LOG("[BLE] Stopped sending, not connected anymore while waiting to notify");
            break;
          }
          if(pTxCharacteristic == NULL) {
            LOG("[BLE] Stopped sending, characteristic is NULL while waiting to notify");
            break;
          }
          if(pService == NULL || pService->getServer() == NULL) {
            LOG("[BLE] Stopped sending, server is NULL while waiting to notify");
            break;
          }
          goto REDO_SEND;
        }
      }

      // advance
      o += cs;
    }
    if(o < len) {
      LOG("[BLE] Stopped sending, not connected anymore, sent %d of %d bytes", o, len);
      return 0;
    } else {
      D("[BLE] Sending complete, total %d bytes sent", o);
      return 1;
    }
  }
}

NOINLINE
uint8_t ble_send(const char *dstr) {
  return ble_send_n((const uint8_t *)dstr, strlen(dstr));
}

NOINLINE
void start_advertising_ble(){
  LOG("[BLE] Enabling Bluetooth and starting advertising");
  if (pServer){
    pServer->getAdvertising()->stop();
    pServer->getAdvertising()->start();
  }
  ble_advertising_start = millis();
  LOG("[BLE] Advertising started, waiting for client connection");
}

NOINLINE
void stop_advertising_ble() {
  // Mark as disabled
  ble_advertising_start = 0;

  if(deviceConnected == 1) {
    LOG("[BLE] Disconnecting from connected device");
    pServer->disconnect(0);
    deviceConnected = 0;
  }

  // Stop advertising
  LOG("[BLE] Stopping advertising and disabling Bluetooth");
  if (pServer)
    pServer->getAdvertising()->stop();

  LOG("[BLE] Bluetooth disabled");
}
#endif // BT_BLE

void setup_cfg(){
  // read
  CFG_LOAD();
  // was (or needs) initialized?
  LOG("[CONFIG] init=%08X ver=%08X", cfg.initialized, cfg.version);
  if(cfg.initialized != CFGINIT || cfg.version != CFGVERSION){
    cfg.do_verbose = 1;
    LOG("[CONFIG] reinitializing");
    // clear
    memset(&cfg, 0, sizeof(cfg));
    // reinit
    cfg.initialized       = CFGINIT;
    cfg.version           = CFGVERSION;
    #ifdef VERBOSE
    cfg.do_verbose        = 1;
    #endif
    #ifdef TIMELOG
    cfg.do_timelog        = 0;
    #endif
    #ifdef LOGUART
    cfg.do_log            = 0;
    #endif
    #ifdef LOOP_DELAY
    cfg.main_loop_delay   = 100;
    #endif
    #if defined(SUPPORT_WIFI) && defined(SUPPORT_NTP)
    strcpy((char *)&cfg.ntp_host, (char *)DEFAULT_NTP_SERVER);
    #endif // SUPPORT_WIFI && SUPPORT_NTP
    #ifdef SUPPORT_WIFI
    cfg.ip_mode = IPV4_DHCP | IPV6_SLAAC;
    #endif // SUPPORT_WIFI
    #ifdef SUPPORT_UART1
    // Initialize UART1 with default values
    cfg.uart1_baud = 115200;
    cfg.uart1_data = 8;
    cfg.uart1_parity = 0;
    cfg.uart1_stop = 1;
    cfg.uart1_rx_pin = UART1_RX_PIN;
    cfg.uart1_tx_pin = UART1_TX_PIN;
    #endif
    // write config
    CFG_SAVE();
    LOG("[CONFIG] reinitializing done");
  }

  // default BLE security to usable values
  cfg.ble_pin = BLUETOOTH_UART_DEFAULT_PIN; // Default PIN
  cfg.ble_security_mode = 0; // No security
  cfg.ble_io_cap = 3;        // NoInputNoOutput
  cfg.ble_auth_req = 0;      // No authentication
}

#define UART1_RX_BUFFER_SIZE   2048 // max size of UART1 buffer Rx
#define UART1_TX_BUFFER_SIZE   2048 // max size of UART1 buffer Tx, 0 means no buffer, direct write and wait

#ifdef SUPPORT_UART1
void setup_uart1(){

  // Convert config values to Arduino constants
  uint32_t config;

  // Build configuration based on data bits, parity, and stop bits
  if(cfg.uart1_data == 5) {
    if(cfg.uart1_parity == 1)
      config = (cfg.uart1_stop == 2) ? SERIAL_5E2 : SERIAL_5E1;
    else if(cfg.uart1_parity == 2)
      config = (cfg.uart1_stop == 2) ? SERIAL_5O2 : SERIAL_5O1;
    else
      config = (cfg.uart1_stop == 2) ? SERIAL_5N2 : SERIAL_5N1;
  } else if(cfg.uart1_data == 6) {
    if(cfg.uart1_parity == 1)
      config = (cfg.uart1_stop == 2) ? SERIAL_6E2 : SERIAL_6E1;
    else if(cfg.uart1_parity == 2)
      config = (cfg.uart1_stop == 2) ? SERIAL_6O2 : SERIAL_6O1;
    else
      config = (cfg.uart1_stop == 2) ? SERIAL_6N2 : SERIAL_6N1;
  } else if(cfg.uart1_data == 7) {
    if(cfg.uart1_parity == 1)
      config = (cfg.uart1_stop == 2) ? SERIAL_7E2 : SERIAL_7E1;
    else if(cfg.uart1_parity == 2)
      config = (cfg.uart1_stop == 2) ? SERIAL_7O2 : SERIAL_7O1;
    else
      config = (cfg.uart1_stop == 2) ? SERIAL_7N2 : SERIAL_7N1;
  } else { // 8 bits (default)
    if(cfg.uart1_parity == 1)
      config = (cfg.uart1_stop == 2) ? SERIAL_8E2 : SERIAL_8E1;
    else if(cfg.uart1_parity == 2)
      config = (cfg.uart1_stop == 2) ? SERIAL_8O2 : SERIAL_8O1;
    else
      config = (cfg.uart1_stop == 2) ? SERIAL_8N2 : SERIAL_8N1;
  }

  // Stop UART1 if already running
  Serial1.flush();
  Serial1.end();

  // Configure UART1 with new parameters
  // Use APB (Advanced Peripheral Bus) clock for better accuracy,
  // uses more power, allows faster baud rates
  // XTAL clock is fixed at 40MHz, APB clock is 80MHz
  Serial1.setClockSource(UART_CLK_SRC_APB);
  Serial1.setMode(UART_MODE_UART);

  // Set buffer sizes, before begin()!
  size_t bufsize = 0;
  bufsize = Serial1.setRxBufferSize(UART1_RX_BUFFER_SIZE);
  LOG("[UART1] RX buffer size set to %d bytes", bufsize);
  bufsize = Serial1.setTxBufferSize(UART1_TX_BUFFER_SIZE);
  LOG("[UART1] TX buffer size set to %d bytes", bufsize);

  // Initialize UART1
  Serial1.begin(cfg.uart1_baud, config, cfg.uart1_rx_pin, cfg.uart1_tx_pin);

  // Non-blocking read
  Serial1.setTimeout(0);

  // Error handling
  Serial1.onReceiveError([](hardwareSerial_error_t event) {
    LOG("[UART1] Receive error: %d, %s", event,
        event == UART_NO_ERROR          ? "NO ERROR" :
        event == UART_BREAK_ERROR       ? "BREAK ERROR" :
        event == UART_BUFFER_FULL_ERROR ? "BUFFER FULL ERROR" :
        event == UART_FIFO_OVF_ERROR    ? "FIFO OVERFLOW ERROR" :
        event == UART_FRAME_ERROR       ? "FRAME ERROR" :
        event == UART_PARITY_ERROR      ? "PARITY ERROR" :
        "UNKNOWN");
  });

  // Enable CTS/RTS hardware flow control, 60 bytes RX FIFO threshold
  Serial1.setHwFlowCtrlMode(UART_HW_FLOWCTRL_CTS_RTS, 60);

  // Trigger RX FIFO interrupt when at least this amount of bytes is available
  Serial1.setRxFIFOFull(64);

  // Trigger the onReceive internal call back when not enough data for the
  // FIFOFull check to happen but still timeout, calculated sleep by IDF.
  // E.g.: For baud: 115200baud, symbol: SERIAL_8N1 (10bit), symbols_timeout: 1t
  // > print(1000ms * 1t / (115200baud / 10bit))
  // 0.086805555555556 ms timeout ~ 86 microseconds
  Serial1.setRxTimeout(1);

  LOG("[UART1] Configured: %lu baud, %d%c%d, RX=%d, TX=%d",
      cfg.uart1_baud, cfg.uart1_data,
      (cfg.uart1_parity == 0) ? 'N' : (cfg.uart1_parity == 1) ? 'E' : 'O',
      cfg.uart1_stop, cfg.uart1_rx_pin, cfg.uart1_tx_pin);
}
#endif // SUPPORT_UART1

#if defined(SUPPORT_WIFI) && defined(WIFI_WPS)
/* WPS (WiFi Protected Setup) Functions both PBC and PIN */
bool start_wps(const char *pin) {
  if (cfg.wifi_enabled == 0) {
    LOG("[WPS] WiFi is disabled in config");
    return false;
  }
  if (wps_running) {
    LOG("[WPS] WPS already running");
    return false;
  }

  LOG("[WPS] Starting WPS Push Button Configuration, timeout %d seconds", WPS_TIMEOUT_MS / 1000);

  // Stop any current WiFi connections
  stop_networking();
  WiFi.removeEvent(WiFiEvent);
  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK); // require WPA2
  WiFi.setScanMethod(WIFI_FAST_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  if(pin == NULL){
    // Configure WPS - modern ESP32 API
    wps_config.wps_type = WPS_TYPE_PBC;

    // set optional device name and manufacturer
    snprintf((char*)wps_config.factory_info.manufacturer, sizeof(wps_config.factory_info.manufacturer), "HOMEKIT");
    snprintf((char*)wps_config.factory_info.model_name, sizeof(wps_config.factory_info.model_name), "UART");
    snprintf((char*)wps_config.factory_info.model_number, sizeof(wps_config.factory_info.model_number), "1.0");
    snprintf((char*)wps_config.factory_info.device_name, sizeof(wps_config.factory_info.device_name), DEFAULT_HOSTNAME);
  } else {
    // Configure WPS - modern ESP32 API
    wps_config.wps_type = WPS_TYPE_PIN;
    LOG("[WPS] Starting WPS with PIN: %s", pin);
  }

  // Start WPS
  esp_err_t result = esp_wifi_wps_enable(&wps_config);
  if (result != ESP_OK) {
    LOG("[WPS] Failed to enable WPS: %s", esp_err_to_name(result));
    return false;
  }

  result = esp_wifi_wps_start(0);
  if (result != ESP_OK) {
    LOG("[WPS] Failed to start WPS: %s", esp_err_to_name(result));
    esp_wifi_wps_disable();
    return false;
  }

  wps_running = true;
  wps_start_time = millis();
  #ifdef LED
  last_tcp_activity = millis(); // Trigger LED activity for WPS
  #endif // LED
  LOG("[WPS] WPS PBC started successfully");
  return true;
}

bool stop_wps() {
  if (!wps_running) {
    LOG("[WPS] WPS not running");
    return false;
  }

  LOG("[WPS] Stopping WPS");
  esp_wifi_wps_disable();
  wps_running = false;
  wps_start_time = 0;
  LOG("[WPS] WPS stopped");
  return true;
}

const char* get_wps_status() {
  if (!wps_running) {
    return "stopped";
  }

  unsigned long elapsed = millis() - wps_start_time;
  if (elapsed > WPS_TIMEOUT_MS) {
    return "timeout";
  }

  return "running";
}
#endif // SUPPORT_WIFI && WIFI_WPS

#ifdef SUPPORT_WIFI
void WiFiEvent(WiFiEvent_t event){
  doYIELD;
  switch(event) {
      case ARDUINO_EVENT_WIFI_READY:
          LOG("[WiFi] ready");
          break;
      case ARDUINO_EVENT_WIFI_STA_START:
          LOG("[WiFi] STA started");
          break;
      case ARDUINO_EVENT_WIFI_STA_STOP:
          LOG("[WiFi] STA stopped");
          break;
      case ARDUINO_EVENT_WIFI_STA_CONNECTED:
          LOG("[WiFi] STA connected to %s", WiFi.SSID().c_str());
          break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
          LOG("[WiFi] STA disconnected");
          #ifdef SUPPORT_MDNS
          stop_mdns();
          #endif // SUPPORT_MDNS
          break;
      case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
          LOG("[WiFi] STA auth mode changed");
          break;
      case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
          {
            LOGT("[WiFi] STA got IPV6: ga: %s", WiFi.globalIPv6().toString().c_str());
            LOGR(", ll: %s", WiFi.linkLocalIPv6().toString().c_str());
            LOGR("\n");
            reconfigure_network_connections();
            #ifdef SUPPORT_MDNS
            setup_mdns();
            #endif // SUPPORT_MDNS
          }
          break;
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          LOG("[WiFi] STA got IP: %s", WiFi.localIP().toString().c_str());
          reconfigure_network_connections();
          #ifdef SUPPORT_MDNS
          setup_mdns();
          #endif // SUPPORT_MDNS
          break;
      case ARDUINO_EVENT_WIFI_STA_LOST_IP:
          LOG("[WiFi] STA lost IP");
          #ifdef SUPPORT_MDNS
          stop_mdns();
          #endif // SUPPORT_MDNS
          stop_network_connections();
          break;
      case ARDUINO_EVENT_WPS_ER_SUCCESS:
          #ifdef WIFI_WPS
          LOG("[WPS] succeeded");
          wps_running = false;
          wps_start_time = 0;
          esp_wifi_wps_disable();

          // Use esp_wifi_get_config() to read saved credentials
          wifi_config_t saved_config;
          if (esp_wifi_get_config(WIFI_IF_STA, &saved_config) == ESP_OK) {
            if(strlen((char*)saved_config.sta.ssid) == 0){
              LOG("[WPS] No SSID received, WPS failed");
              break;
            }
            if(strlen((char*)saved_config.sta.password) == 0){
              LOG("[WPS] No Password received, WPS failed");
              break;
            }
            if(strlen((char*)saved_config.sta.ssid) > sizeof(cfg.wifi_ssid) - 1){
              LOG("[WPS] SSID too long, WPS failed");
              break;
            }
            if(strlen((char*)saved_config.sta.password) > sizeof(cfg.wifi_pass) - 1){
              LOG("[WPS] Password too long, WPS failed");
              break;
            }

            // clear the IP config, we're ok with WPS now
            cfg.ip_mode &= ~IPV4_STATIC;
            cfg.ip_mode |= IPV4_DHCP;
            cfg.ip_mode |= IPV6_SLAAC;
            memset((char*)cfg.ipv4_addr, 0, sizeof(cfg.ipv4_addr));
            memset((char*)cfg.ipv4_gw, 0, sizeof(cfg.ipv4_gw));
            memset((char*)cfg.ipv4_mask, 0, sizeof(cfg.ipv4_mask));
            memset((char*)cfg.ipv4_dns, 0, sizeof(cfg.ipv4_dns));

            // Save new credentials to config
            strncpy((char*)cfg.wifi_ssid, (char*)saved_config.sta.ssid, sizeof(cfg.wifi_ssid) - 1);
            strncpy((char*)cfg.wifi_pass, (char*)saved_config.sta.password, sizeof(cfg.wifi_pass) - 1);
            LOG("[WPS] Saved SSID: %s", cfg.wifi_ssid);
            LOG("[WPS] Saved Pass: ********", cfg.wifi_pass);
            D("[WPS] Saved Pass (clear): %s", cfg.wifi_pass);
            CFG_SAVE();

            // WPS success, credentials are automatically saved
            // Restart WiFi connection with new credentials
            LOG("[WPS] Restarting WiFi with new credentials");
            reset_networking();
          }
          #endif
          break;
      case ARDUINO_EVENT_WPS_ER_FAILED:
          #ifdef WIFI_WPS
          LOG("[WPS] failed");
          wps_running = false;
          wps_start_time = 0;
          esp_wifi_wps_disable();
          #endif
          break;
      case ARDUINO_EVENT_WPS_ER_TIMEOUT:
          #ifdef WIFI_WPS
          LOG("[WPS] timed out");
          wps_running = false;
          wps_start_time = 0;
          esp_wifi_wps_disable();
          #endif
          break;
      case ARDUINO_EVENT_WPS_ER_PIN:
          #ifdef WIFI_WPS
          LOG("[WPS] PIN received");
          #endif
          break;
      default:
          break;
  }
}
#endif // SUPPORT_WIFI

#ifdef BT_CLASSIC
void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  doYIELD;
  if(event == ESP_SPP_START_EVT){
    LOG("BlueTooth UART Initialized SPP");
  } else if(event == ESP_SPP_SRV_OPEN_EVT){
    LOG("BlueTooth UART Client connected");
  } else if(event == ESP_SPP_CLOSE_EVT){
    LOG("BlueTooth UART Client disconnected");
  } else if(event == ESP_SPP_DATA_IND_EVT){
    LOG("BlueTooth UART Data received");
    // any new AT command?
    ATScBT.ReadSerial();
  }
}
#endif

#ifdef ESP_LOG_INFO
void log_esp_info(){
  LOG("[ESP] Firmware version: %s", ESP.getSdkVersion());
  LOG("[ESP] Chip Model: %06X", ESP.getChipModel());
  LOG("[ESP] Chip Revision: %d", ESP.getChipRevision());
  LOG("[ESP] CPU Frequency: %d MHz", ESP.getCpuFreqMHz());
  LOG("[ESP] Flash Size: %d MB", ESP.getFlashChipSize() / (1024 * 1024));
  LOG("[ESP] Free Heap: %d bytes", ESP.getFreeHeap());
  LOG("[ESP] Sketch Size: %d bytes", ESP.getSketchSize());
  LOG("[ESP] Sketch Free Space: %d bytes", ESP.getFreeSketchSpace());
  LOG("[ESP] ESP Core Version: %s", ESP.getCoreVersion());
  LOG("[ESP] Boot Flash Size: %d", ESP.getFlashChipSize());
  LOG("[ESP] Boot Flash Speed: %d", ESP.getFlashChipSpeed());
  LOG("[ESP] Boot Flash Mode: %d", ESP.getFlashChipMode());
  LOG("[ESP] CPU Cores: %d", ESP.getChipCores());
  LOG("[ESP] SDK Version: %s", ESP.getSdkVersion());
  LOG("[ESP] Minimum Free Heap: %d bytes", ESP.getMinFreeHeap());
  LOG("[ESP] PSRAM Size: %d bytes", ESP.getPsramSize());
  LOG("[ESP] Free PSRAM: %d bytes", ESP.getFreePsram());
  LOG("[ESP] Minimum Free PSRAM: %d bytes", ESP.getMinFreePsram());
  LOG("[ESP] Uptime: %lu seconds", millis() / 1000);

#ifdef ARDUINO_ARCH_ESP32
  // Log partition information
  LOG("[ESP] === Partition Information ===");
  esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
  while (it != NULL) {
    const esp_partition_t* partition = esp_partition_get(it);
    LOG("[ESP] Partition: %s, Type: 0x%02x, SubType: 0x%02x, Address: 0x%08x, Size: %d KB",
        partition->label, partition->type, partition->subtype, partition->address, partition->size / 1024);
    it = esp_partition_next(it);
  }
  esp_partition_iterator_release(it);

  // Log NVS statistics
  LOG("[ESP] === NVS Statistics ===");
  nvs_stats_t nvs_stats;
  esp_err_t err = nvs_get_stats(NULL, &nvs_stats);
  if (err == ESP_OK) {
    LOG("[ESP] NVS Used Entries: %d", nvs_stats.used_entries);
    LOG("[ESP] NVS Free Entries: %d", nvs_stats.free_entries);
    LOG("[ESP] NVS Total Entries: %d", nvs_stats.total_entries);
    LOG("[ESP] NVS Utilization: %d%%", (nvs_stats.used_entries * 100) / nvs_stats.total_entries);
    LOG("[ESP] NVS Available Entries: %d", nvs_stats.available_entries);
    LOG("[ESP] NVS Namespace Count: %d", nvs_stats.namespace_count);
  } else {
    LOG("[ESP] Failed to get NVS stats: %s", esp_err_to_name(err));
  }

  // Log SPIFFS statistics
  LOG("[ESP] === SPIFFS Statistics ===");
  if (SPIFFS.begin(true)) {
    size_t total_bytes = SPIFFS.totalBytes();
    size_t used_bytes = SPIFFS.usedBytes();
    size_t free_bytes = total_bytes - used_bytes;
    LOG("[ESP] SPIFFS Total: %d bytes (%.2f KB)", total_bytes, total_bytes / 1024.0);
    LOG("[ESP] SPIFFS Used: %d bytes (%.2f KB)", used_bytes, used_bytes / 1024.0);
    LOG("[ESP] SPIFFS Free: %d bytes (%.2f KB)", free_bytes, free_bytes / 1024.0);
    LOG("[ESP] SPIFFS Utilization: %.1f%%", (used_bytes * 100.0) / total_bytes);
    SPIFFS.end();
  } else {
    LOG("[ESP] SPIFFS not available or failed to mount");
  }

  // Log specific NVS partitions
  LOG("[ESP] === NVS Partition Details ===");
  const esp_partition_t* nvs_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
  if (nvs_partition != NULL) {
    LOG("[ESP] NVS Partition: %s, Size: %d KB, Address: 0x%08x",
        nvs_partition->label, nvs_partition->size / 1024, nvs_partition->address);
    // Now list all namespaces
    nvs_iterator_t it;
    err = nvs_entry_find(nvs_partition->label, NULL, NVS_TYPE_ANY, &it);
    while (err == ESP_OK ) {
      nvs_entry_info_t info;
      if (nvs_entry_info(it, &info) == ESP_OK)
        LOG("[ESP] - NVS Entry Partition: %s, Namespace: %s, Key: %s, Type: %d", nvs_partition->label, info.namespace_name, info.key, info.type);
      err = nvs_entry_next(&it);
    }
  }

  // Log the "esp-at" NVS partition if it exists
  nvs_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, CFG_PARTITION);
  if (nvs_partition != NULL) {
    LOG("[ESP] NVS Partition: %s, Size: %d KB, Address: 0x%08x",
        nvs_partition->label, nvs_partition->size / 1024, nvs_partition->address);
    // Now list all namespaces
    nvs_iterator_t it;
    err = nvs_entry_find(nvs_partition->label, NULL, NVS_TYPE_ANY, &it);
    while (err == ESP_OK ) {
      nvs_entry_info_t info;
      if (nvs_entry_info(it, &info) == ESP_OK)
        LOG("[ESP] - NVS Entry Partition: %s, Namespace: %s, Key: %s, Type: %d", nvs_partition->label, info.namespace_name, info.key, info.type);
      err = nvs_entry_next(&it);
    }
  }

  const esp_partition_t* nvs_keys_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS_KEYS, NULL);
  if (nvs_keys_partition != NULL) {
    LOG("[ESP] NVS Keys Partition: %s, Size: %d KB, Address: 0x%08x",
        nvs_keys_partition->label, nvs_keys_partition->size / 1024, nvs_keys_partition->address);
  }
#endif
}
#endif // ESP_LOG_INFO

#ifdef SUPPORT_WIFI
void log_wifi_info(){
  LOG("[WiFi] status: %d, %s", WiFi.status(),
    WiFi.status() == WL_CONNECTED ? "connected" :
    WiFi.status() == WL_NO_SHIELD ? "no shield" :
    WiFi.status() == WL_IDLE_STATUS ? "idle" :
    WiFi.status() == WL_NO_SSID_AVAIL ? "no ssid available" :
    WiFi.status() == WL_SCAN_COMPLETED ? "scan completed" :
    WiFi.status() == WL_CONNECT_FAILED ? "connect failed" :
    WiFi.status() == WL_CONNECTION_LOST ? "connection lost" :
    WiFi.status() == WL_DISCONNECTED ? "disconnected" : "unknown");
  if(WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS){
    LOGT("[WiFi] connected: SSID:%s", WiFi.SSID().c_str());
    LOGR(", MAC:%s", WiFi.macAddress().c_str());
    LOGR(", RSSI:%hu", WiFi.RSSI());
    LOGR(", BSSID:%s", WiFi.BSSIDstr().c_str());
    LOGR(", CHANNEL:%d", WiFi.channel());
    LOGR("\n");
    LOGT("[IPV4] ADDR:%s", WiFi.localIP().toString().c_str());
    LOGR(", GW:%s", WiFi.gatewayIP().toString().c_str());
    LOGR(", NM:%s", WiFi.subnetMask().toString().c_str());
    LOGR(", DNS:%s", WiFi.dnsIP().toString().c_str());
    LOGR("\n");
    if(cfg.ip_mode & IPV6_SLAAC){
      IPAddress g_ip6 = WiFi.globalIPv6();
      IPAddress l_ip6 = WiFi.linkLocalIPv6();
      LOGT("[IPV6] SLAAC:%s", g_ip6.toString().c_str());
      LOGR(", LINK-LOCAL:%s", l_ip6.toString().c_str());
      LOGR("\n");
    }
  }
}
#endif // SUPPORT_WIFI

NOINLINE
char * PT(const char *tformat = "[\%H:\%M:\%S]"){
  time_t t;
  ALIGN(4) static char T_buffer[512] = {""};
  struct tm gm_new_tm;
  time(&t);
  localtime_r(&t, &gm_new_tm);
  strftime(T_buffer, 512, tformat, &gm_new_tm);
  return T_buffer;
}

#ifdef LED

// LED configuration
volatile bool led_state = false;
bool last_led_state = false;
int last_led_interval = 0;
int8_t last_led_brightness = 0;
int led_brightness_off = LED_BRIGHTNESS_OFF;
int led_brightness_on  = LED_BRIGHTNESS_LOW;

hw_timer_t *led_t = NULL;
portMUX_TYPE led_timer_mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR ledBlinkTimer() {
  portENTER_CRITICAL_ISR(&led_timer_mux);
  led_state = !led_state;
  portEXIT_CRITICAL_ISR(&led_timer_mux);
}

// Helper function to set LED brightness (0-255 on ESP32, digital on/off on ESP8266)
void set_led_brightness(int brightness) {
  if(last_led_brightness == brightness){
    return; // no change
  }
  last_led_brightness = brightness;
  #if defined(SUPPORT_LED_BRIGHTNESS)
  if (led_pwm_enabled) {
    // Use hardware PWM for smooth brightness control with channel-based API
    if(!ledcWriteChannel(LED_PWM_CHANNEL, brightness)){
      // PWM failed, fallback to digital control
      digitalWrite(LED, brightness > LED_BRIGHTNESS_LOW ? HIGH : LOW);
    }
  } else {
    // PWM failed, fallback to digital control
    digitalWrite(LED, brightness > LED_BRIGHTNESS_LOW ? HIGH : LOW);
  }
  #else
  // ESP8266 fallback: treat anything above LOW threshold as HIGH
  digitalWrite(LED, brightness > LED_BRIGHTNESS_LOW ? HIGH : LOW);
  #endif
}

void led_on(){
  set_led_brightness(led_brightness_on);
  led_state = true;
}

void led_off(){
  set_led_brightness(led_brightness_off);
  led_state = false;
}

void setup_led(){
  LOG("[LED] Setup on pin %d", LED);

  // LED pin setup
  pinMode(LED, OUTPUT);

  #if defined(SUPPORT_LED_BRIGHTNESS)
  // ESP32 PWM setup
  ledc_clk_cfg_t t = ledcGetClockSource();
  LOG("[LED] LED PWM clock source: %d", t);
  // Setup LED PWM channel
  if (ledcAttachChannel(LED, LED_PWM_FREQUENCY, LED_PWM_RESOLUTION, LED_PWM_CHANNEL)) {
    led_pwm_enabled = true;
    LOG("[LED] PWM control enabled on pin %d, channel %d", LED, LED_PWM_CHANNEL);
  } else {
    // Fallback to digital control if PWM setup fails
    led_pwm_enabled = false;
    LOG("[LED] PWM setup failed, using digital control on pin %d", LED);
  }
  #endif // SUPPORT_LED_BRIGHTNESS

  // Start with LED on, and on/off are normal brightness values
  led_on();

  // setup a LED blink timer, default to 1 second interval -> AFTER pwm setup,
  // use timer 1, as 0 is used by PWM internally? Pick the same as PWM channel,
  // this gets reused internally
  LOG("[LED] setting up LED blink timer");
  led_t = timerBegin(1000);
  if(led_t == NULL){
    LOG("[LED] Failed to initialize timer for LED");
  } else {
    LOG("[LED] Timer initialized successfully");
    timerAttachInterrupt(led_t, &ledBlinkTimer);
    LOG("[LED] Timer interrupt attached");
    timerAlarm(led_t, LED_BLINK_INTERVAL_NORMAL, true, 0);
    LOG("[LED] Timer alarm set to %d ms", LED_BLINK_INTERVAL_NORMAL);
    timerWrite(led_t, 0);
    timerStart(led_t);
    LOG("[LED] Timer started");
    LOG("[LED] LED setup completed successfully");
  }
  LOG("[LED] LED setup done, starting blink loop");
}

int determine_led_state(){
  // Enhanced LED control with new behavior patterns
  int led_interval = 0;
  unsigned long now = millis();
  bool comm_active = (now - last_tcp_activity < COMM_ACTIVITY_LED_DURATION) ||
                     (now - last_udp_activity < COMM_ACTIVITY_LED_DURATION) ||
                     (now - last_uart1_activity < COMM_ACTIVITY_LED_DURATION);

  #ifdef SUPPORT_WIFI
  bool is_wifi_connected = (WiFi.status() == WL_CONNECTED);
  #endif // SUPPORT_WIFI
  uint8_t is_ble_advertising = (ble_advertising_start != 0);
  uint8_t is_ble_connected = (deviceConnected == 1);

  // Determine LED behavior based on priority (highest to lowest):
  if (comm_active) {
    // Data transmission: tiny flicker on top of current state
    led_interval = LED_BLINK_INTERVAL_FLICKER;
    led_brightness_on = LED_BRIGHTNESS_LOW + LED_BRIGHTNESS_FLICKER;
    led_brightness_off = LED_BRIGHTNESS_LOW;
    #ifdef SUPPORT_WIFI
    if (is_wifi_connected) {
      led_brightness_on = LED_BRIGHTNESS_MEDIUM + LED_BRIGHTNESS_FLICKER; // Flicker on top of steady
      led_brightness_off = LED_BRIGHTNESS_MEDIUM; // Return to steady connected state
    }
    #endif // SUPPORT_WIFI
  } else if (is_ble_advertising && !is_ble_connected) {
    // BLE advertising (button pressed, waiting for connection): fast blink
    led_interval = LED_BLINK_INTERVAL_FAST;
    led_brightness_on = LED_BRIGHTNESS_HIGH;
    led_brightness_off = LED_BRIGHTNESS_LOW;
  } else if (is_ble_connected) {
    // BLE device connected: slow blink until disconnected
    led_interval = LED_BLINK_INTERVAL_SLOW;
    led_brightness_on = LED_BRIGHTNESS_MEDIUM;
    led_brightness_off = LED_BRIGHTNESS_LOW;
  #ifdef SUPPORT_WPS
  } else if (wps_running) {
    // WPS active: slow blink
    led_interval = LED_BLINK_INTERVAL_QUICK;
    led_brightness_on = LED_BRIGHTNESS_HIGH;
    led_brightness_off = LED_BRIGHTNESS_DIM;
  #endif // SUPPORT_WPS
  #ifdef SUPPORT_WIFI
  } else if (is_wifi_connected) {
    // WiFi connected: full on at medium brightness (not too bright)
    led_interval = 0; // No blinking, steady on
    led_brightness_on = LED_BRIGHTNESS_MEDIUM;
    led_brightness_off = LED_BRIGHTNESS_MEDIUM; // Same as on = steady
  #endif // SUPPORT_WIFI
  } else {
    // Not connected: slowly blinking
    led_interval = LED_BLINK_INTERVAL_HALF;
    led_brightness_on = LED_BRIGHTNESS_LOW;
    led_brightness_off = LED_BRIGHTNESS_OFF;
  }
  return led_interval;
}

void update_led_state(){
  if(led_state != last_led_state){
    last_led_state = led_state;
    if(led_state) {
      led_on();
    } else {
      led_off();
    }
  }
}

void set_led_blink(int interval_ms){
  if(interval_ms != last_led_interval){
    D("[LED] Setting LED blink interval to %d ms", interval_ms);
    last_led_interval = interval_ms;
    if(interval_ms == 0){
      // Steady on or off
      if(led_brightness_on == led_brightness_off){
        // Same brightness, just set it and stop timer
        timerStop(led_t);
        last_led_state = !led_state; // force update
        if(led_brightness_on > LED_BRIGHTNESS_LOW){
          led_on();
        } else {
          led_off();
        }
      }
    } else {
      timerStop(led_t);
      timerAlarm(led_t, last_led_interval, true, 0);
      D("[LED] Timer alarm set to %d ms", last_led_interval);
      timerWrite(led_t, 0);
      timerStart(led_t);
    }
  }
}
#endif // LED

// button handling settings
#define BUTTON_DEBOUNCE_MS       20
#define BUTTON_SHORT_PRESS_MS    80
#define BUTTON_NORMAL_PRESS_MS 1000
#define BUTTON_LONG_PRESS_MS   2000

// Button configuration, button_changed is volatile as it's set in an ISR, same
// for button_last_time
volatile bool button_changed = false;
volatile unsigned long button_last_time = 0;
uint8_t button_action = 0;
unsigned long button_press_start = 0;
unsigned long press_duration = 0;
portMUX_TYPE button_mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR buttonISR() {
  portENTER_CRITICAL_ISR(&button_mux);
  // millis() is ISR-safe on ESP32, but won't change in the ISR context
  unsigned long current_time = millis();

  // Simple debouncing - ignore button presses within debounce period
  if (current_time - button_last_time > BUTTON_DEBOUNCE_MS) {
    button_changed = true;
    button_last_time = current_time;
  }
  portEXIT_CRITICAL_ISR(&button_mux);
}

void determine_button_state(){
  if(button_changed || button_action != 0){
    if(button_changed){
      D("[BUTTON] Button state changed interrupt detected");
    } else {
      D("[BUTTON] Button action in progress, checking state");
    }
    // reset flag
    button_changed = false;

    // read current state
    bool current_button_state = digitalRead(BUTTON) == LOW;
    D("[BUTTON] Current button state: %s, action: %d, %lu ms", current_button_state ? "PRESSED" : "RELEASED", button_action, press_duration);
    if (current_button_state && button_action == 0) {
      // Button just pressed
      button_press_start = millis();
      button_action = 1;
      press_duration = 0;
      LOG("[BUTTON] Button pressed, timing for short/long press detection");
    } else if (current_button_state && button_action == 1) {
      // Button still pressed, just continue timing
      press_duration = millis() - button_press_start;
      LOG("[BUTTON] Button still pressed, duration: %lu ms", press_duration);

      #ifdef LED
      // during the button pressed check, make the led blink fast in the first
      // section for the BLE advertising, if the button is pressed longer, make
      // the LED blink slower for WPS
      if(press_duration > BUTTON_SHORT_PRESS_MS && press_duration < BUTTON_LONG_PRESS_MS){
        // normal press section
        LOG("[LED] Button held, switching to slow blink for normal press section");
        set_led_blink(LED_BLINK_INTERVAL_SLOW);
      } else if (press_duration >= BUTTON_LONG_PRESS_MS){
        // long press section
        LOG("[LED] Button held, switching to quick blink for long press section");
        set_led_blink(LED_BLINK_INTERVAL_QUICK);
      }
      #endif // LED

    } else if (!current_button_state && button_action == 1) {
      // Button released, use the timed duration to decide action
      press_duration = millis() - button_press_start;
      LOG("[BUTTON] Button released after press, duration: %lu ms", press_duration);
      if(press_duration < BUTTON_DEBOUNCE_MS){
        // reset button pressed flag
        button_action = 0;

        // Ignore very short presses (debounce)
        LOG("[BUTTON] Press duration too short (%lu ms), ignoring", press_duration);
        // Reset button state, not action taken
        button_press_start = 0;
        press_duration = 0;
      } else if (press_duration < BUTTON_SHORT_PRESS_MS ){
        // reset button pressed flag
        button_action = 0;
        LOG("[BUTTON] Short press detected (%lu ms)", press_duration);

        if (ble_advertising_start != 0) {
          LOG("[BUTTON] Short press detected (%lu ms), stopping BLE advertising if active", press_duration);
          // BLE is currently enabled, stop advertising
          stop_advertising_ble();
          LOG("[BUTTON] BLE advertising stopped");
        }

        #ifdef WIFI_WPS
        if(wps_running){
          LOG("[BUTTON] Short press detected (%lu ms), stopping WPS", press_duration);
          // cancel any ongoing WPS
          if(wps_running){
            LOG("[BUTTON] Stopping ongoing WPS due to button press");
            if(stop_wps())
              reset_networking();
          }
        }
        #endif // WIFI_WPS
        // Reset button state, not action taken
        button_press_start = 0;
        press_duration = 0;
      } else if (press_duration < BUTTON_NORMAL_PRESS_MS) {
        // reset button pressed flag
        button_action = 0;
        LOG("[BUTTON] Normal press detected (%lu ms)", press_duration);
        #ifdef SUPPORT_BLE_UART1
        if (cfg.ble_uart1_bridge == 1){
          // BLE UART1 bridge is enabled and in bridge mode
          if(at_mode == BRIDGE_MODE) {
            // Switch to AT mode
            ble_uart1_at_mode(AT_MODE);
            ble_advertising_start = millis();
            LOG("[BUTTON] BLE AT Mode enabled");
          } else {
            // Switch to Bridge mode
            ble_uart1_at_mode(BRIDGE_MODE);
            ble_advertising_start = millis();
            LOG("[BUTTON] BLE Bridge Mode enabled");
          }
        } else {
          // Normal press - toggle BLE advertising as before
          if (ble_advertising_start == 0) {
            start_advertising_ble();
            LOG("[BUTTON] BLE advertising started - will stop on timeout if no connection, or when button pressed again");
          } else {
            stop_advertising_ble();
            LOG("[BUTTON] BLE advertising stopped");
          }
        }
        #else
        // Normal press - toggle BLE advertising as before
        if (ble_advertising_start == 0) {
          start_advertising_ble();
          LOG("[BUTTON] BLE advertising started - will stop on timeout if no connection, or when button pressed again");
        } else {
          stop_advertising_ble();
          LOG("[BUTTON] BLE advertising stopped");
        }
        #endif
        // Reset button state, not action taken
        button_press_start = 0;
        press_duration = 0;
      } else if (press_duration >= BUTTON_LONG_PRESS_MS) {
        // reset button pressed flag
        button_action = 0;

        // Long press - handle WPS
        LOG("[BUTTON] Long press detected (%lu ms), handling WPS", press_duration);
        #ifdef WIFI_WPS
        LOG("[BUTTON] Starting WPS");
        start_wps(NULL);
        #endif // WIFI_WPS

        // Reset button state, not action taken
        button_press_start = 0;
        press_duration = 0;
      }

    } else if(!current_button_state && button_action > 1) {
      // Button released after action taken, reset state
      LOG("[BUTTON] Button released after action taken, resetting state");
      button_action = 0;
      button_press_start = 0;
      press_duration = 0;
    } else {
      // No state change, ignore
      LOG("[BUTTON] No state change detected, ignoring");
    }
  }
}

void setup_button(){
  pinMode(BUTTON, INPUT_PULLUP);
  LOG("[BUTTON] Pin %d configured as INPUT_PULLUP", BUTTON);

  // Attach button interrupt for both press and release
  attachInterrupt(digitalPinToInterrupt(BUTTON), buttonISR, CHANGE);
  LOG("[BUTTON] Interrupt attached to pin %d on CHANGE edge", BUTTON);
}

void setup_nvs(){
  LOG("[NVS] Setting up NVS in partition %s", CFG_PARTITION);
  esp_err_t err = nvs_open_from_partition(CFG_PARTITION, CFG_NAMESPACE, NVS_READWRITE, &nvs_c);
  if (err != ESP_OK) {
    LOG("[NVS] Failed to open NVS partition %s: %s", CFG_PARTITION, esp_err_to_name(err));
    LOG("[NVS] Attempting to initialize NVS partition %s", CFG_PARTITION);
    err = nvs_flash_init_partition(CFG_PARTITION);
    if(err != ESP_OK){
      LOG("[NVS] Failed to initialize NVS partition %s: %s", CFG_PARTITION, esp_err_to_name(err));
      ESP.restart();
    } else {
      LOG("[NVS] initialized successfully");
      err = nvs_open_from_partition(CFG_PARTITION, CFG_NAMESPACE, NVS_READWRITE, &nvs_c);
      if (err) {
        LOG("[NVS] Failed to open NVS partition %s after init: %s", CFG_PARTITION, esp_err_to_name(err));
        ESP.restart();
      } else {
        LOG("[NVS] opened successfully after init");
      }
    }
  } else {
    LOG("[NVS] NVS partition %s opened successfully", CFG_PARTITION);
  }
}

void setup(){
  // Serial setup, init at 115200 8N1
  Serial.setTimeout(0);
  Serial.setTxBufferSize(512);
  Serial.setRxBufferSize(512);
  Serial.begin(115200);

  // enable all ESP32 core logging
  #ifdef DEBUG
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  #endif

  // setup nvs
  setup_nvs();

  // setup cfg
  setup_cfg();

  // Setup AT command handler
  #ifdef UART_AT
  ATSc.SetDefaultHandler(&sc_cmd_handler);
  #endif

  // BlueTooth SPP setup possible?
  #if defined(BLUETOOTH_UART_AT) && defined(BT_BLE)
  setup_ble();
  // Set BLE UART1 bridge as default if enabled
  #ifdef SUPPORT_BLE_UART1
  // Bridge mode by default
  if(cfg.ble_uart1_bridge == 1){
    ble_uart1_at_mode(BRIDGE_MODE);
    start_advertising_ble();
  }
  #endif
  #endif

  #if defined(BLUETOOTH_UART_AT) && defined(BT_CLASSIC)
  LOG("[BT] setting up Bluetooth Classic");
  SerialBT.begin(BLUETOOTH_UART_DEVICE_NAME);
  SerialBT.setPin(BLUETOOTH_UART_DEFAULT_PIN);
  SerialBT.register_callback(BT_EventHandler);
  ATScBT.SetDefaultHandler(&sc_cmd_handler);
  #endif

  // setup WiFi with ssid/pass if set
  #ifdef SUPPORT_WIFI
  start_networking();
  #endif // SUPPORT_WIFI

  #ifdef SUPPORT_UART1
  // use UART1 with configurable parameters
  setup_uart1();
  #endif // SUPPORT_UART1

  #ifdef LED
  // Setup LED with PWM for brightness control
  setup_led();
  #endif // LED

  // Button setup
  setup_button();

  // log info
  #ifdef ESP_LOG_INFO
  log_esp_info();
  #endif // ESP_LOG_INFO
}

#ifdef ESP_LOG_INFO
void do_esp_log(){
  // Log ESP info periodically when DEBUG is enabled
  LOOP_D("[LOOP] ESP info log check");
  if(last_esp_info_log ==0 || millis() - last_esp_info_log > 30000) { // Log every 30 seconds
    log_esp_info();
    last_esp_info_log = millis();
  }
}
#endif // ESP_LOG_INFO

#ifdef SUPPORT_WIFI
INLINE
void do_wifi_check(){
  LOOP_D("[LOOP] WiFi check");
  if(cfg.wifi_enabled && strlen(cfg.wifi_ssid) != 0 && millis() - last_wifi_check > 500){
    last_wifi_check = millis();
    #ifdef VERBOSE
    if(millis() - last_wifi_info_log > 60000){
      last_wifi_info_log = millis();
      if(cfg.do_verbose)
        log_wifi_info();
    }
    #endif
    if(WiFi.status() != WL_CONNECTED && WiFi.status() != WL_IDLE_STATUS){
      // not connected, try to reconnect
      if(last_wifi_reconnect == 0 || millis() - last_wifi_reconnect > 30000){
        last_wifi_reconnect = millis();
        LOG("[WiFi] Not connected, attempting to reconnect, status: %d", WiFi.status());
        reset_networking();
      }
    } else {
      // connected
      last_wifi_reconnect = millis();
    }
  }
}
#endif // SUPPORT_WIFI

#if (defined(SUPPORT_TCP) || defined(SUPPORT_UDP))
INLINE
void do_connections_check(){
  // TCP connection check at configured interval
  LOOP_D("[LOOP] TCP/UDP check");
  if(WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS){
    // connected, check every 500ms
    if(last_tcp_check == 0 || millis() - last_tcp_check > 500){
      last_tcp_check = millis();
      #if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP)
      if(strlen(cfg.tcp_host_ip) != 0 && cfg.tcp_port != 0){
        doYIELD;
        int conn_ok = check_tcp_connection(0);
        if(!conn_ok){
          D("[LOOP] TCP Connection lost");
          connect_tcp();
        }
      }
      #endif // SUPPORT_WIFI && SUPPORT_TCP

      #if defined(SUPPORT_WIFI) && defined(SUPPORT_TLS)
      if(cfg.tls_enabled && strlen(cfg.tcp_host_ip) != 0 && (cfg.tcp_port != 0 || cfg.tls_port != 0)){
        doYIELD;
        int tls_conn_ok = check_tls_connection();
        if(!tls_conn_ok){
          D("[LOOP] TLS Connection lost");
          connect_tls();
        }
      }
      #endif // SUPPORT_WIFI && SUPPORT_TLS
    }
  }
}
#endif // SUPPORT_TCP || SUPPORT_UDP

#ifdef SUPPORT_NTP
INLINE
void do_ntp_check(){
  // NTP check
  LOOP_D("[LOOP] NTP check");
  if(last_ntp_log == 0 || millis() - last_ntp_log > 10000){
    last_ntp_log = millis();
    if((WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS) && cfg.ntp_host[0] != 0 && esp_sntp_enabled()){
      doYIELD;
      // check if synced
      D("[NTP] Checking NTP sync status");
      if(sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED){
        // synced
        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);
        if(timeinfo.tm_hour != last_hour){
          LOG("[NTP] NTP is synced, current time: %s", PT());
          last_hour = timeinfo.tm_hour;
        }
      } else if(sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && last_hour != -1){
        D("[NTP] NTP sync ok");
      } else {
        D("[NTP] not yet synced");
      }
    }
  }
}
#endif // SUPPORT_NTP

#define UART1_READ_SIZE       64 // read bytes at a time from UART1
#define UART1_WRITE_SIZE      64 // write bytes at a time to UART1
#define TCP_READ_SIZE         16 // read bytes at a time from TCP
#define REMOTE_BUFFER_SIZE   512 // max size of REMOTE buffer
#define LOCAL_BUFFER_SIZE    512 // max size of LOCAL buffer

// from "LOCAL", e.g. "UART1", add 1 byte for \0 during buffer prints in debugging/logging
ALIGN(4) uint8_t inbuf[LOCAL_BUFFER_SIZE+1] = {0};
// max size of inbuf, notice the -1, as we will never fill up that byte
const uint8_t *inbuf_max = (uint8_t *)&inbuf + LOCAL_BUFFER_SIZE;

// from "REMOTE", e.g. TCP, UDP
ALIGN(4) uint8_t outbuf[REMOTE_BUFFER_SIZE] = {0};
size_t outlen = 0;

uint8_t sent_ok = 0;

#ifdef SUPPORT_TCP_SERVER
INLINE
void do_tcp_server_check(){
  // if not configured, skip
  if((tcp_server_sock == -1 && tcp6_server_sock == -1) || (cfg.tcp_server_port == 0 && cfg.tcp6_server_port == 0)){
    // no TCP server configured
    return;
  }

  // TCP Server handling
  LOOP_D("[LOOP] Check TCP server connections");
  if(tcp_server_sock != -1) {
    handle_tcp_server();
    // Update last activity time if we have clients
    if(get_tcp_server_client_count() > 0) {
      #ifdef LED
      last_tcp_activity = millis(); // Trigger LED activity for TCP server
      #endif // LED
    }
  }

  // TCP6 Server handling
  LOOP_D("[LOOP] Check TCP6 server connections");
  if(tcp6_server_sock != -1) {
    handle_tcp6_server();
    // Update last activity time if we have clients
    if(get_tcp_server_client_count() > 0) {
      #ifdef LED
      last_tcp_activity = millis(); // Trigger LED activity for TCP6 server
      #endif // LED
    }
  }

  if (tcp_server_sock != -1 || tcp6_server_sock != -1) {
    if(inlen > 0){
      LOOP_D("[LOOP] TCP_SERVER Sending data to clients, len: %d", inlen);
      int clients_sent = send_tcp_server_data((const uint8_t*)inbuf, inlen);
      if (clients_sent > 0) {
        #ifdef LED
        last_tcp_activity = millis(); // Trigger LED activity for TCP server send
        #endif // LED
        D("[TCP_SERVER] Sent %d bytes to %d clients, data: >>%s<<", inlen, clients_sent, inbuf);
        sent_ok |= 1; // mark as sent
      } else {
        LOOP_D("[TCP_SERVER] No clients connected to send data to");
        // Don't mark as error if no clients are connected
      }
    }

    if (outlen + TCP_READ_SIZE >= sizeof(outbuf)) {
      D("[TCP_SERVER] outbuf full, cannot read more data, outlen: %d", outlen);
    } else {
      LOOP_D("[LOOP] TCP_SERVER Checking for incoming data");
      int r = recv_tcp_server_data((uint8_t*)outbuf + outlen, TCP_READ_SIZE);
      if (r > 0) {
        // data received
        #ifdef LED
        last_tcp_activity = millis(); // Trigger LED activity for TCP server receive
        #endif // LED
        D("[TCP_SERVER] Received %d bytes, total: %d, data: >>%s<<", r, outlen + r, outbuf);
        outlen += r;
      } else if (r == 0) {
        // connection closed by remote host
        LOG("[TCP_SERVER] connection closed by remote host");
      } else if (r == -1) {
        // error occurred, check errno
        if(errno && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINPROGRESS){
          // Error occurred, log it
          LOGE("[TCP_SERVER] closing connection");
        } else {
          // No data available, just yield
          LOOP_E("[TCP_SERVER] no data available", errno);
        }
      }
    }
  }

  LOOP_D("[LOOP] Check TCP_SERVER disconnects");
  if(tcp_server_sock != -1 || tcp6_server_sock != -1)
    handle_tcp_server_disconnects();
}
#endif // SUPPORT_TCP_SERVER

#ifdef SUPPORT_UDP
INLINE
void do_udp_check(){
  // no UDP configured?
  if(udp_sock == -1 && udp_out_sock == -1 && udp_listen_sock == -1 && udp6_listen_sock == -1){
    // no UDP configured
    return;
  }

  // UDP send
  LOOP_D("[LOOP] Check for outgoing UDP data fd: %d: inlen: %d", udp_sock, inlen);
  if(inlen > 0){
    if (udp_sock != -1) {
      int sent = send_udp_data(udp_sock, (const uint8_t*)inbuf, inlen, cfg.udp_host_ip, cfg.udp_port, "[UDP]");
      if (sent > 0) {
        #ifdef LED
        last_udp_activity = millis(); // Trigger LED activity for UDP send
        #endif // LED
        D("[UDP] Sent %d bytes, total: %d, data: >>%s<<", sent, inlen, inbuf);
        sent_ok |= 1; // mark as sent
      } else if (sent < 0) {
        LOGE("[UDP] Sent error %d bytes, total: %d", sent, inlen);
      } else if (sent == 0) {
        D("[UDP] Sent 0 bytes, total: %d", inlen);
      }
    }
    if (udp_out_sock != -1){
      int sent = send_udp_data(udp_out_sock, (const uint8_t*)inbuf, inlen, cfg.udp_send_ip, cfg.udp_send_port, "[UDP_SEND]");
      if (sent > 0) {
        #ifdef LED
        last_udp_activity = millis(); // Trigger LED activity for UDP send
        #endif // LED
        D("[UDP_SEND] Sent %d bytes, total: %d, data: >>%s<<", sent, inlen, inbuf);
        sent_ok |= 1; // mark as sent
      } else if (sent < 0) {
        LOGE("[UDP_SEND] Sent error %d bytes, total: %d", sent, inlen);
      } else if (sent == 0) {
        D("[UDP_SEND] Sent 0 bytes, total: %d", inlen);
      }
    }
  }

  // UDP read
  LOOP_D("[LOOP] Check for incoming UDP data");

  // in/out UDP socket read
  udp_read(udp_sock, outbuf, outlen, UDP_READ_MSG_SIZE, REMOTE_BUFFER_SIZE, "[UDP]");

  // in UDP socket read
  udp_read(udp_listen_sock, outbuf, outlen, UDP_READ_MSG_SIZE, REMOTE_BUFFER_SIZE, "[UDP_LISTEN]");

  // in UDP6 socket read
  udp_read(udp6_listen_sock, outbuf, outlen, UDP_READ_MSG_SIZE, REMOTE_BUFFER_SIZE, "[UDP6_LISTEN]");
}
#endif // SUPPORT_UDP

#ifdef SUPPORT_TCP
INLINE
void do_tcp_check(){
  if(tcp_sock == -1){
    // no TCP configured
    return;
  }

  // TCP send
  LOOP_D("[LOOP] Check for outgoing TCP data");
  if (tcp_sock != -1 && inlen > 0) {
    if (!tcp_connection_writable){
      LOOP_D("[TCP] No valid connection, cannot send data");
    } else {
      int sent = send_tcp_data((const uint8_t*)inbuf, inlen);
      if (sent > 0) {
        #ifdef LED
        last_tcp_activity = millis(); // Trigger LED activity for TCP send
        #endif // LED
        D("[TCP] Sent %d bytes, total: %d", sent, inlen);
        sent_ok |= 1; // mark as sent
      } else if (sent == -1) {
        if(errno && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINPROGRESS){
          // Error occurred, log it
          LOGE("[TCP] send error, closing connection");
        } else {
          // Socket not ready for writing, data will be retried on next loop
          E("[TCP] socket not ready for writing, will retry", errno);
        }
      } else if (sent == 0) {
        // Socket not ready for writing, data will be retried on next loop
        LOG("[TCP] connection closed by remote host");
      }
    }
  }

  // TCP read
  LOOP_D("[LOOP] Check for incoming TCP data");
  if (tcp_sock != -1 && tcp_connection_writable) {
    if (outlen + TCP_READ_SIZE >= sizeof(outbuf)) {
      D("[TCP] outbuf full, cannot read more data, outlen: %d", outlen);
      // no space in outbuf, cannot read more data
      // just yield and wait for outbuf to be cleared
    } else {
      // no select(), just read from TCP socket and ignore ENOTCONN etc..
      int os = recv_tcp_data((uint8_t*)outbuf + outlen, TCP_READ_SIZE);
      if (os > 0) {
        // data received
        #ifdef LED
        last_tcp_activity = millis(); // Trigger LED activity for TCP receive
        #endif // LED
        D("[TCP] Received %d bytes, total: %d, data: >>%s<<", os, outlen + os, outbuf);
        outlen += os;
      } else if (os == 0) {
        // connection closed by remote host
        LOG("[TCP] connection closed by remote host");
      } else if (os == -1) {
        // error occurred, check errno
        if(errno && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINPROGRESS){
          // Error occurred, log it
          LOGE("[TCP] closing connection");
        } else {
          // No data available, just yield
          LOOP_E("[TCP] no data available", errno);
        }
      }
    }
  }
}
#endif // SUPPORT_TCP

#ifdef SUPPORT_TLS
INLINE
void do_tls_check(){
  // TLS send (if TLS is enabled and connected)
  LOOP_D("[LOOP] Check for outgoing TLS data");
  if (cfg.tls_enabled && tls_connected && tls_handshake_complete && inlen > 0) {
    int sent = send_tls_data((const uint8_t*)inbuf, inlen);
    if (sent > 0) {
      #ifdef LED
      last_tcp_activity = millis(); // Trigger LED activity for TLS send
      #endif // LED
      D("[TLS] Sent %d bytes, total: %d", sent, inlen);
      sent_ok |= 1; // mark as sent
    } else if (sent == -1) {
      LOG("[TLS] send error or connection lost");
    } else if (sent == 0) {
      LOG("[TLS] connection closed by remote host");
    }
  } else if (cfg.tls_enabled && !tls_connected && inlen > 0) {
    D("[TLS] No valid TLS connection, cannot send data");
  }

  // TLS read
  LOOP_D("[LOOP] Check for incoming TLS data");
  if (cfg.tls_enabled && tls_connected && tls_handshake_complete) {
    if (outlen + TCP_READ_SIZE >= sizeof(outbuf)) {
      D("[TLS] outbuf full, cannot read more data, outlen: %d", outlen);
      // no space in outbuf, cannot read more data
      // just yield and wait for outbuf to be cleared
    } else {
      int os = recv_tls_data((uint8_t*)outbuf + outlen, TCP_READ_SIZE);
      if (os > 0) {
        // data received
        #ifdef LED
        last_tcp_activity = millis(); // Trigger LED activity for TLS receive
        #endif // LED
        D("[TLS] Received %d bytes, total: %d, data: >>%s<<", os, outlen + os, outbuf);
        outlen += os;
      } else if (os == 0) {
        // connection closed by remote host
        LOG("[TLS] connection closed by remote host");
      } else if (os == -1) {
        // no data available or error
        LOOP_D("[TLS] no data available");
      }
    }
  }
}
#endif // SUPPORT_TLS

#ifdef SUPPORT_BLE_UART1
void do_ble_uart1_bridge(){
  // BLE <-> UART1 bridge enabled via AT command?
  if(cfg.ble_uart1_bridge == 0 || at_mode == AT_MODE)
    return; // BLE <-> UART1 bridge disabled

  // Bridge data between UART1 and BLE if connected
  LOOP_D("[LOOP] Check BLE <-> UART1 bridge");
  if(deviceConnected == 1){
    // BLE connected, check if we have data from UART1 to send over BLE
    if(inlen > 0){
      uint8_t ok_send = ble_send_n((const uint8_t *)inbuf, inlen);
      if(ok_send == 1){
        D("[BLE] Sent %d bytes from UART1 to BLE", inlen);
        sent_ok |= 1; // mark as sent
      } else {
        LOG("[BLE] Failed to send %d bytes from UART1 to BLE", inlen);
      }
    }
  }

  // Check if we have data from BLE to send over UART1
  if(ble_rx_len > 0){
    if(outlen + ble_rx_len <= REMOTE_BUFFER_SIZE){
      memcpy(outbuf + outlen, ble_rx_buffer, ble_rx_len);
      D("[BLE] Received %d bytes from BLE to UART1, data: >>%s<<", ble_rx_len, ble_rx_buffer);
      outlen += ble_rx_len;

      // clear BLE buffer
      ble_rx_len = 0;
      memset(ble_rx_buffer, 0, sizeof(ble_rx_buffer));
    } else {
      LOGE("[BLE] Not enough space in outbuf to copy BLE data");
    }
  }
}
#endif // SUPPORT_BLE_UART1

#ifdef LOOP_DELAY
INLINE
void do_loop_delay(){
  // DELAY sleep, we need to pick the lowest amount of delay to not block too
  // long, default to cfg.main_loop_delay if not needed
  int loop_delay = cfg.main_loop_delay;
  if(loop_delay >= 0){
    if((WiFi.status() != WL_CONNECTED && WiFi.status() != WL_IDLE_STATUS) || ble_advertising_start != 0 || inlen > 0){
      loop_delay = 0; // no delay if not connected or BLE enabled
    }
    doYIELD;
    if(loop_delay <= 0){
      // no delay, just yield
      LOOP_D("[LOOP] no delay, len: %d, ble: %s", inlen, ble_advertising_start != 0 ? "y" : "n");
      doYIELD;
    } else {
      // delay and yield, check the loop_start_millis on how long we should still sleep
      loop_start_millis = millis() - loop_start_millis;
      long delay_time = (long)loop_delay - (long)loop_start_millis;
      LOOP_D("[LOOP] delay for tm: %d, wa: %d, wt: %d, len: %d, ble: %s", loop_start_millis, loop_delay, delay_time, inlen, ble_advertising_start != 0 ? "y" : "n");
      if(delay_time > 0){
        power_efficient_sleep(delay_time);
      } else {
        LOOP_D("[LOOP] loop processing took longer than main_loop_delay");
      }
      doYIELD;
    }
  }
}
#endif // LOOP_DELAY

void loop(){
  LOOP_D("[LOOP] Start main loop");
  sent_ok = 0;

  #ifdef LOOP_DELAY
  static unsigned long loop_start_millis = 0;
  loop_start_millis = millis();
  #endif // LOOP_DELAY

  // Handle button press
  LOOP_D("[BUTTON] Checking button state");
  determine_button_state();

  #ifdef LED
  // LED indicator
  LOOP_D("[LED] Checking LED state and updating if needed");
  set_led_blink(determine_led_state());
  update_led_state();
  #endif // LED

  #if defined(SUPPORT_WIFI) && defined(WIFI_WPS)
  // Check WPS timeout
  if (wps_running && (millis() - wps_start_time > WPS_TIMEOUT_MS)) {
    LOG("[WPS] WPS timeout reached, stopping WPS");
    if(stop_wps())
      reset_networking();
  }
  #endif // SUPPORT_WIFI && WIFI_WPS

  #ifdef UART_AT
  // Handle Serial AT commands
  while(ATSc.GetSerial()->available() > 0)
    ATSc.ReadSerial();
  #endif

  #ifdef BT_BLE
  // Check if BLE advertising should be stopped after timeout
  // Only stop on timeout if no device is connected - once connected, wait for remote disconnect or button press
  if (at_mode == AT_MODE && ble_advertising_start != 0 && deviceConnected == 0 && millis() - ble_advertising_start > BLE_ADVERTISING_TIMEOUT){
    stop_advertising_ble();
    #ifdef SUPPORT_WIFI
    reset_networking();
    #endif // SUPPORT_WIFI
  }
  if (at_mode == BRIDGE_MODE && deviceConnected == 0 && cfg.ble_uart1_bridge == 1 && ble_advertising_start == 0){
    #ifdef SUPPORT_WIFI
    if(cfg.wifi_enabled == 1)
      stop_networking();
    #endif // SUPPORT_WIFI
    start_advertising_ble();
  }

  // Handle pending BLE commands
  if(deviceConnected == 1 && at_mode == AT_MODE)
    handle_ble_command();
  #endif // BT_BLE

  #ifdef TIMELOG
  // TIMELOG state send, TODO: implement this instead of a stub/dummy
  LOOP_D("[LOOP] Time logging check");
  if(cfg.do_timelog && (last_time_log == 0 || millis() - last_time_log > 500)){
    #if defined(BT_BLE)
    if(ble_advertising_start != 0)
      ble_send(PT("🍓 [%H:%M:%S]:📡 ⟹  🖫&💾\n"));
    #endif
    #ifdef LOGUART
    if(cfg.do_log)
      LOG("%s", PT("[%H:%M:%S]: UART1 🍓&📡 ⟹  🖫&💾\n"));
    #endif
    last_time_log = millis();
  }
  #endif // TIMELOG

  #ifdef SUPPORT_UART1
  // Read all available bytes from UART, but only for as much data as fits in
  // inbuf, read per X chars to be sure we don't overflow
  LOOP_D("[LOOP] Checking for available data, inlen: %d", inlen);
  uint8_t *b_old = inbuf + inlen;
  uint8_t *b_new = b_old;
  while(b_new < inbuf_max) {
    // read bytes into inbuf
    size_t to_r = Serial1.readBytes(b_new, (size_t)(inbuf_max - b_new));
    if(to_r <= 0)
        break; // nothing read
    inlen += to_r;
    b_new += to_r;
    // slight delay to allow more data to arrive
    //delayMicroseconds(50);
    D("[UART1] READ %04d bytes from UART1, buf:%04d", to_r, inlen);
    doYIELD;
  }
  if(b_old != b_new){
    // null terminate, even if b_new = inbuf_max, we have space for the \0
    *b_new = '\0';
    LOOP_D("[UART1]: Total bytes in inbuf: %d", inlen);
    #ifdef LED
    last_uart1_activity = millis(); // Trigger LED activity for UART1 receive
    #endif // LED
  } else {
    LOOP_D("[UART1]: No new data read from UART1");
    sent_ok |= 1; // nothing read, mark as sent
  }
  doYIELD;
  #endif // SUPPORT_UART1

  #ifdef SUPPORT_UDP
  do_udp_check();
  #endif // SUPPORT_UDP

  #ifdef SUPPORT_TCP
  do_tcp_check();
  #endif // SUPPORT_TCP

  #ifdef SUPPORT_TLS
  do_tls_check();
  #endif // SUPPORT_TLS

  #ifdef SUPPORT_TCP_SERVER
  do_tcp_server_check();
  #endif // SUPPORT_TCP_SERVER

  #ifdef SUPPORT_BLE_UART1
  do_ble_uart1_bridge();
  #endif // SUPPORT_BLE_UART1

  #ifdef SUPPORT_WIFI
  do_wifi_check();
  #endif // SUPPORT_WIFI

  #ifdef ESP_LOG_INFO
  do_esp_log();
  #endif // ESP_LOG_INFO

  #if defined(SUPPORT_WIFI) && (defined(SUPPORT_TCP) || defined(SUPPORT_UDP))
  do_connections_check();
  #endif // SUPPORT_WIFI && (SUPPORT_TCP || SUPPORT_UDP)

  #if defined(SUPPORT_WIFI) && defined(SUPPORT_NTP)
  do_ntp_check();
  #endif // SUPPORT_WIFI && SUPPORT_NTP

  // copy over the inbuf to outbuf for logging if data received
  if(outlen > 0){
    // send outbuf to Serial1 if data received, in chunks of X bytes
    #ifdef SUPPORT_UART1
    uint8_t *o = (uint8_t *)&outbuf;
    uint8_t *m = (uint8_t *)&outbuf + outlen;
    size_t w = 0;
    while(o < m && (w = Serial1.availableForWrite()) > 0){
      doYIELD;
      w = min((size_t)w, (size_t)UART1_WRITE_SIZE);
      w = min((size_t)w, (size_t)(m - o));
      w = Serial1.write(o, w);
      Serial1.flush();
      if(w > 0){
        #ifdef LED
        last_uart1_activity = millis(); // Trigger LED activity for UART1 send
        #endif // LED
        D("[UART1]: Written %d bytes, total: %d, data: >>%s<<", w, outlen, outbuf);
        o += w;
      } else {
        // nothing written, just yield
        D("[UART1]: nothing written, yielding...");
      }
    }
    if(o >= m){
      // all sent
      D("[UART1]: all outbuf data sent");
    } else {
      D("[UART1]: not all outbuf data sent, remaining: %d", m - o);
    }
    outlen = 0;
    #else
    // no UART1 support, just clear the outbuf
    outlen = 0;
    #endif // SUPPORT_UART1
    doYIELD;
  }

  // clear outbuf
  memset(outbuf, 0, sizeof(outbuf));

  // assume the inbuf is sent
  if(inlen && sent_ok){
    inlen = 0;
    memset(inbuf, 0, sizeof(inbuf));
  }
  if(inlen){
    LOOP_D("[LOOP] inbuf not sent, clearing buffer, len: %d", inlen);
    inlen = 0;
    memset(inbuf, 0, sizeof(inbuf));
  }

  #ifdef LOOP_DELAY
  do_loop_delay();
  #endif // LOOP_DELAY

  LOOP_D("[LOOP] End main loop");
}
