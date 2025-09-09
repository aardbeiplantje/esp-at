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
#endif
#include <esp_log.h>
#ifndef SUPPORT_WIFI
#define SUPPORT_WIFI
#endif // SUPPORT_WIFI
#ifdef SUPPORT_WIFI
#include <WiFi.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_wps.h>
#endif // SUPPORT_WIFI
#endif
#ifdef ARDUINO_ARCH_ESP8266
#ifdef DEBUG
#define USE_ESP_IDF_LOG
#define CORE_DEBUG_LEVEL 5
#define LOG_LOCAL_LEVEL 5
#endif
#include <esp_log.h>
#ifdef SUPPORT_WIFI
#include <ESP8266WiFi.h>
#endif // SUPPORT_WIFI
#endif
#include <errno.h>
#include <sys/time.h>
#include "time.h"
#include "EEPROM.h"

#define NOINLINE __attribute__((noinline,noipa))

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
#define SUPPORT_UART1 1
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

#ifndef SUPPORT_UDP
#define SUPPORT_UDP
#endif // SUPPORT_UDP

#ifndef SUPPORT_NTP
#define SUPPORT_NTP
#endif // SUPPORT_NTP

#else

// no WiFi support, disable related features
#undef WIFI_WPS
#undef SUPPORT_TCP_SERVER
#undef SUPPORT_TCP
#undef SUPPORT_UDP
#undef SUPPORT_NTP
#endif // !SUPPORT_WIFI

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
  static char _date_outstr[20] = {0};
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
  static char _buf[256] = {0};
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
 #define D(...)   do_printf(2, DEBUG_TIME_FORMAT, DEBUG_FILE_LINE); do_printf(1, NULL, __VA_ARGS__);
 #define T(...)   do_printf(2, DEBUG_TIME_FORMAT, DEBUG_FILE_LINE); do_printf(1, NULL, __VA_ARGS__);
 #define R(...)   do_printf(0, NULL, __VA_ARGS__);
 #define E(...)   do_printf(2, DEBUG_TIME_FORMAT, DEBUG_FILE_LINE); do_printf(0, NULL, __VA_ARGS__);\
                  do_printf(0, NULL, ", errno: %d (%s)\n", errno, get_errno_string(errno));
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

#ifndef BLUETOOTH_UART_AT
#define BLUETOOTH_UART_AT
#endif // BLUETOOTH_UART_AT

#ifdef BLUETOOTH_UART_AT
#ifndef BLUETOOTH_UART_DEVICE_NAME
#define BLUETOOTH_UART_DEVICE_NAME DEFAULT_HOSTNAME
#endif

#ifdef BT_CLASSIC
#ifndef BLUETOOTH_UART_DEFAULT_PIN
#define BLUETOOTH_UART_DEFAULT_PIN "1234"
#endif
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#warning Bluetooth is not enabled or possible.
#undef BT_CLASSIC
#endif
#if !defined(CONFIG_BT_SPP_ENABLED)
#warning Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#undef BT_CLASSIC
#endif
#endif // BT_CLASSIC

#define BT_BLE
#ifdef BT_BLE
#include <BLEUUID.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEService.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>
#endif
#endif // BT_BLE

#if !defined(BT_BLE) && !defined(BT_CLASSIC)
#undef BLUETOOTH_UART_AT
#endif

#ifdef BT_CLASSIC
/* AT commands over Classic Serial Bluetooth */
BluetoothSerial SerialBT;
char atscbt[128] = {""};
SerialCommands ATScBT(&SerialBT, atscbt, sizeof(atscbt), "\r\n", "\r\n");
#endif

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
char atscbu[128] = {""};
SerialCommands ATSc(&Serial, atscbu, sizeof(atscbu), "\r\n", "\r\n");
#endif // UART_AT

#define CFGVERSION 0x03 // switch between 0x01/0x02 to reinit the config struct change
#define CFGINIT    0x72 // at boot init check flag
#define CFG_EEPROM 0x00

#define IPV4_DHCP    1
#define IPV4_STATIC  2
#define IPV6_SLAAC   4

/* main config */
typedef struct cfg_t {
  uint8_t initialized  = 0;
  uint8_t version      = 0;
  #ifdef VERBOSE
  uint8_t do_verbose   = 0;
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
};
cfg_t cfg;

#if defined(SUPPORT_WIFI) && defined(SUPPORT_NTP)
long last_ntp_log = 0;
uint8_t ntp_is_synced = 0;
int8_t last_hour = -1;
void cb_ntp_synced(struct timeval *tv){
  LOG("[NTP] NTP time synced");
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

/* state flags */
#ifdef SUPPORT_WIFI
long last_wifi_check = 0;
long last_wifi_info_log = 0;
long last_wifi_reconnect = 0;
#endif // SUPPORT_WIFI

#ifdef TIMELOG
long last_time_log = 0;
#endif // TIMELOG

#ifdef DEBUG
long last_esp_info_log = 0;
#endif // DEBUG

#ifdef SUPPORT_UDP
int udp_sock = -1;
int udp_listen_sock = -1;
int udp6_listen_sock = -1;
int udp_out_sock = -1;
#endif // SUPPORT_UDP

#ifdef BT_BLE
long ble_advertising_start = 0;
#define BLE_ADVERTISING_TIMEOUT 10000   // 10 seconds in milliseconds
#endif // BT_BLE

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
uint8_t tcp_server_active = 0;
int tcp_server_sock = -1;
uint8_t tcp6_server_active = 0;
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
  #if defined(SUPPORT_WIFI) && defined(WIFI_WPS)
  if(wps_running){
      LOG("[WiFi] WPS is running, cannot reset networking now");
      return;
  }
  #endif // SUPPORT_WIFI && WIFI_WPS
  LOG("[WiFi] reset networking...");
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

    #if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)
    // TCP server - start or restart if configured
    D("[TCP SERVER] configured port: %hu, active: %d, sock: %d", cfg.tcp_server_port, tcp_server_active, tcp_server_sock);
    if(cfg.tcp_server_port > 0) {
      if(!tcp_server_active || tcp_server_sock < 0) {
        start_tcp_server();
      }
    } else {
      // TCP server port is 0, stop server if running
      if(tcp_server_active || tcp_server_sock >= 0) {
        stop_tcp_server();
      }
    }

    // TCP6 server - start or restart if configured
    D("[TCP6 SERVER] configured port: %hu, active: %d, sock: %d", cfg.tcp6_server_port, tcp6_server_active, tcp6_server_sock);
    if(cfg.tcp6_server_port > 0) {
      if(!tcp6_server_active || tcp6_server_sock < 0) {
        start_tcp6_server();
      }
    } else {
      // TCP6 server port is 0, stop server if running
      if(tcp6_server_active || tcp6_server_sock >= 0) {
        stop_tcp6_server();
      }
    }
    #endif
  } else {
    #if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)
    // WiFi not connected, stop TCP servers
    if(tcp_server_active || tcp_server_sock >= 0) {
      stop_tcp_server();
    }
    if(tcp6_server_active || tcp6_server_sock >= 0) {
      stop_tcp6_server();
    }
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

  #ifdef SUPPORT_UDP
  close_udp_socket(udp_sock, "[UDP]");
  close_udp_socket(udp_listen_sock, "[UDP_LISTEN]");
  close_udp_socket(udp6_listen_sock, "[UDP6_LISTEN]");
  close_udp_socket(udp_out_sock, "[UDP_SEND]");
  #endif // SUPPORT_UDP

  #if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)
  stop_tcp_server();
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
#if defined(SUPPORT_TCP) || defined(SUPPORT_UDP)
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
#endif // SUPPORT_TCP || SUPPORT_UDP


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
    if (tcp_sock < 0) {
      LOGE("[TCP] Failed to create IPv6 TCP socket");
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
    D("[TCP] socket %d writable, connection OK", tcp_sock);
  } else {
    tcp_connection_writable = 0;
    D("[TCP] socket %d not yet writable", tcp_sock);
  }
  #endif
  return 1;
}
#endif // SUPPORT_WIFI && SUPPORT_TCP

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP_SERVER)
// TCP server variables
int tcp_server_clients[8] = {-1, -1, -1, -1, -1, -1, -1, -1}; // support up to 8 clients
int tcp6_server_clients[8] = {-1, -1, -1, -1, -1, -1, -1, -1}; // support up to 8 IPv6-only clients
unsigned long last_tcp_server_activity = 0;

// TCP Server functions
void start_tcp_server() {
  if(cfg.tcp_server_port == 0) {
    D("[TCP_SERVER] TCP server port not configured");
    return;
  }

  if(tcp_server_sock >= 0) {
    LOG("[TCP_SERVER] TCP server already running on port %hu", cfg.tcp_server_port);
    return;
  }

  LOG("[TCP_SERVER] Starting TCP server on port %hu", cfg.tcp_server_port);

  // Create socket (supports both IPv4 and IPv6)
  tcp_server_sock = socket(AF_INET6, SOCK_STREAM, 0);
  if (tcp_server_sock < 0) {
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

  // Configure for dual-stack (IPv4 and IPv6)
  optval = 0;
  if (setsockopt(tcp_server_sock, IPPROTO_IPV6, IPV6_V6ONLY, &optval, sizeof(optval)) < 0) {
    LOGE("[TCP_SERVER] Failed to set IPV6_V6ONLY");
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
  struct sockaddr_in6 server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin6_family = AF_INET6;
  server_addr.sin6_addr = in6addr_any;  // Listen on all interfaces
  server_addr.sin6_port = htons(cfg.tcp_server_port);

  if (bind(tcp_server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    LOGE("[TCP_SERVER] Failed to bind to port %hu", cfg.tcp_server_port);
    close(tcp_server_sock);
    tcp_server_sock = -1;
    return;
  }

  // Start listening
  if (listen(tcp_server_sock, cfg.tcp_server_max_clients) < 0) {
    LOGE("[TCP_SERVER] Failed to listen on port %hu", cfg.tcp_server_port);
    close(tcp_server_sock);
    tcp_server_sock = -1;
    return;
  }

  // Initialize client socket array
  for(int i = 0; i < 8; i++)
    tcp_server_clients[i] = -1;

  tcp_server_active = 1;
  LOG("[TCP_SERVER] TCP server started successfully on port %hu", cfg.tcp_server_port);
}

void stop_tcp_server() {
  if(tcp_server_sock >= 0) {
    LOG("[TCP_SERVER] Stopping TCP server on port %hu", cfg.tcp_server_port);

    // Close all client connections
    for(int i = 0; i < 8; i++) {
      if(tcp_server_clients[i] < 0)
        continue;
      close(tcp_server_clients[i]);
      tcp_server_clients[i] = -1;
    }

    // Close server socket
    close(tcp_server_sock);
    tcp_server_sock = -1;
    tcp_server_active = 0;
  }
}

void handle_tcp_server() {
  if(tcp_server_sock < 0 || !tcp_server_active) {
    return;
  }

  // Accept new connections
  struct sockaddr_in6 client_addr;
  socklen_t client_len = sizeof(client_addr);
  int new_client = accept(tcp_server_sock, (struct sockaddr*)&client_addr, &client_len);
  if(new_client >= 0) {
    // Find empty slot for new client
    int slot = -1;
    for(int i = 0; i < cfg.tcp_server_max_clients && i < 8; i++) {
      if(tcp_server_clients[i] < 0) {
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

      tcp_server_clients[slot] = new_client;

      // Log client connection
      char client_ip[40];
      if(inet_ntop(AF_INET6, &client_addr.sin6_addr, client_ip, sizeof(client_ip))) {
        LOG("[TCP_SERVER] New client connected from %s on slot %d", client_ip, slot);
      } else {
        LOG("[TCP_SERVER] New client connected on slot %d", slot);
      }

      last_tcp_server_activity = millis();
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
  for(int i = 0; i < cfg.tcp_server_max_clients && i < 8; i++) {
    if(tcp_server_clients[i] < 0)
      continue;
    int result = send(tcp_server_clients[i], data, len, 0);
    if(result > 0) {
      clients_sent++;
    } else if(result < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      // Client connection error, disconnect
      LOG("[TCP_SERVER] Error sending to client %d, disconnecting", i);
      close(tcp_server_clients[i]);
      tcp_server_clients[i] = -1;
    }
  }
  return clients_sent;
}

// Receive data from all connected TCP server clients
int recv_tcp_server_data(uint8_t* buf, size_t maxlen) {
  for(int i = 0; i < cfg.tcp_server_max_clients && i < 8; i++) {
    if(tcp_server_clients[i] < 0)
      continue;
    int bytes_received = recv(tcp_server_clients[i], buf, maxlen, 0);
    if(bytes_received > 0) {
      return bytes_received; // Return data from the first client that has data
    } else if(bytes_received == 0 || (bytes_received < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
      // Client disconnected or error
      LOG("[TCP_SERVER] Client %d disconnected", i);
      close(tcp_server_clients[i]);
      tcp_server_clients[i] = -1;
    }
  }
  return -1; // No data received
}

// Get number of connected TCP server clients
int get_tcp_server_client_count() {
  int count = 0;
  for(int i = 0; i < cfg.tcp_server_max_clients && i < 8; i++) {
    if(tcp_server_clients[i] >= 0)
      count++;
  }
  return count;
}

// TCP6 Server functions (IPv6-only)
void start_tcp6_server() {
  if(cfg.tcp6_server_port == 0) {
    D("[TCP6_SERVER] TCP6 server port not configured");
    return;
  }

  if(tcp6_server_sock >= 0) {
    LOG("[TCP6_SERVER] TCP6 server already running on port %hu", cfg.tcp6_server_port);
    return;
  }

  LOG("[TCP6_SERVER] Starting TCP6 server on port %hu", cfg.tcp6_server_port);

  // Create IPv6-only socket
  tcp6_server_sock = socket(AF_INET6, SOCK_STREAM, 0);
  if (tcp6_server_sock < 0) {
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
  server_addr.sin6_port = htons(cfg.tcp6_server_port);

  if (bind(tcp6_server_sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    LOGE("[TCP6_SERVER] Failed to bind to port %hu", cfg.tcp6_server_port);
    close(tcp6_server_sock);
    tcp6_server_sock = -1;
    return;
  }

  // Start listening
  if (listen(tcp6_server_sock, cfg.tcp_server_max_clients) < 0) {
    LOGE("[TCP6_SERVER] Failed to listen on port %hu", cfg.tcp6_server_port);
    close(tcp6_server_sock);
    tcp6_server_sock = -1;
    return;
  }

  // Initialize client socket array
  for(int i = 0; i < 8; i++)
    tcp6_server_clients[i] = -1;

  tcp6_server_active = 1;
  LOG("[TCP6_SERVER] TCP6 server started successfully on port %hu", cfg.tcp6_server_port);
}

void stop_tcp6_server() {
  if(tcp6_server_sock >= 0) {
    LOG("[TCP6_SERVER] Stopping TCP6 server on port %hu", cfg.tcp6_server_port);

    // Close all client connections
    for(int i = 0; i < 8; i++) {
      if(tcp6_server_clients[i] < 0)
        continue;
      close(tcp6_server_clients[i]);
      tcp6_server_clients[i] = -1;
    }

    // Close server socket
    close(tcp6_server_sock);
    tcp6_server_sock = -1;
    tcp6_server_active = 0;
  }
}

void handle_tcp6_server() {
  if(!tcp6_server_active || tcp6_server_sock < 0 || cfg.tcp6_server_port == 0)
    return;

  // Accept new connections
  struct sockaddr_in6 client_addr;
  socklen_t client_len = sizeof(client_addr);
  int new_client = accept(tcp6_server_sock, (struct sockaddr*)&client_addr, &client_len);

  if(new_client >= 0) {
    // Find available slot for new client
    int slot = -1;
    for(int i = 0; i < cfg.tcp_server_max_clients && i < 8; i++) {
      if(tcp6_server_clients[i] < 0) {
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

      tcp6_server_clients[slot] = new_client;
      char client_ip[INET6_ADDRSTRLEN];
      inet_ntop(AF_INET6, &client_addr.sin6_addr, client_ip, INET6_ADDRSTRLEN);
      LOG("[TCP6_SERVER] New client connected from [%s]:%hu in slot %d",
          client_ip, ntohs(client_addr.sin6_port), slot);

      last_tcp_server_activity = millis();
      #ifdef LED
      last_tcp_activity = millis();
      #endif
    } else {
      LOG("[TCP6_SERVER] No available slots for new client, rejecting");
      close(new_client);
    }
  }

  // Handle disconnected clients
  for(int i = 0; i < cfg.tcp_server_max_clients && i < 8; i++) {
    if(tcp6_server_clients[i] < 0)
      continue;

    // Check if client is still connected
    char test_buf[1];
    int result = recv(tcp6_server_clients[i], test_buf, 0, MSG_DONTWAIT);
    if(result == 0 || (result < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
      LOG("[TCP6_SERVER] Client in slot %d disconnected", i);
      close(tcp6_server_clients[i]);
      tcp6_server_clients[i] = -1;
    }
  }
}

void tcp6_server_send(const char* data, int len) {
  if(!tcp6_server_active || tcp6_server_sock < 0)
    return;

  for(int i = 0; i < cfg.tcp_server_max_clients && i < 8; i++) {
    if(tcp6_server_clients[i] < 0)
      continue;
    int result = send(tcp6_server_clients[i], data, len, 0);
    if(result < 0) {
      if(errno != EAGAIN && errno != EWOULDBLOCK) {
        LOG("[TCP6_SERVER] Failed to send to client %d, disconnecting", i);
        close(tcp6_server_clients[i]);
        tcp6_server_clients[i] = -1;
      }
    }
  }
}

int tcp6_server_receive(char* buf, int maxlen) {
  if(!tcp6_server_active || tcp6_server_sock < 0)
    return 0;

  for(int i = 0; i < cfg.tcp_server_max_clients && i < 8; i++) {
    if(tcp6_server_clients[i] < 0)
      continue;
    int bytes_received = recv(tcp6_server_clients[i], buf, maxlen, 0);
    if(bytes_received > 0) {
      return bytes_received;
    } else if(bytes_received == 0 || (bytes_received < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
      LOG("[TCP6_SERVER] Client %d disconnected during receive", i);
      close(tcp6_server_clients[i]);
      tcp6_server_clients[i] = -1;
    }
  }
  return 0;
}

// Get number of connected TCP6 server clients
int get_tcp6_server_client_count() {
  int count = 0;
  for(int i = 0; i < cfg.tcp_server_max_clients && i < 8; i++) {
    if(tcp6_server_clients[i] >= 0)
      count++;
  }
  return count;
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
  char *l_ip = NULL;
  int16_t port = cfg.udp_port;
  LOG("[UDP] setting up UDP to:%s, port:%hu", d_ip, port);
  if(is_ipv6_addr(d_ip)) {
    if(!udp_socket(udp_sock, 1, "[UDP]"))
      return;

    // local IPv6
    struct sockaddr_in6 l_sa6;
    l_ip = "::"; // listen on all interfaces
    memset(&l_sa6, 0, sizeof(l_sa6));
    l_sa6.sin6_family = AF_INET6;
    l_sa6.sin6_port = htons(port);
    if (inet_pton(AF_INET6, l_ip, &l_sa6.sin6_addr) != 1) {
      LOG("[UDP] Invalid IPv6 address:%s", l_ip);
      close_udp_socket(udp_sock, "[UDP]");
      return;
    }
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
    l_ip = "0.0.0.0"; // listen on all interfaces
    memset(&l_sa4, 0, sizeof(l_sa4));
    l_sa4.sin_family = AF_INET;
    l_sa4.sin_port = htons(port);
    if (inet_pton(AF_INET, l_ip, &l_sa4.sin_addr) != 1) {
      LOG("[UDP] Invalid IPv4 address:%s", l_ip);
      close_udp_socket(udp_sock, "[UDP]");
      return;
    }
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

void udp_read(int fd, char *buf, size_t &len, size_t read_size, size_t maxlen, const char *tag) {
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
  int os = recv_udp_data(fd, (uint8_t*)buf + len, read_size);
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
  s->GetSerial()->println(r);
}
#endif // BT_CLASSIC || UART_AT

#if defined(BLUETOOTH_UART_AT) && defined(BT_BLE)
#define AT_R_OK     (const char*)("OK")
#define AT_R(M)     (const char*)(M)
#define AT_R_STR(M) (const char*)String(M).c_str()
#define AT_R_F(M)   (const char*)(M)

const char *AT_short_help_string = R"EOF(Available AT Commands:
AT
AT?
AT+?
AT+HELP?
AT+RESET
AT+ERASE=|1
AT+WIFI_ENABLED=|?
AT+WIFI_SSID=|?
AT+WIFI_PASS=
AT+WIFI_STATUS?
AT+HOSTNAME=
AT+IPV4=
AT+IPV6=
AT+IP_STATUS?
AT+LOOP_DELAY=|?
)EOF"

#ifdef SUPPORT_WIFI

R"EOF(AT+WIFI_SSID=|?
AT+WIFI_PASS=
AT+WIFI_STATUS?
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

#if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP)
R"EOF(AT+TCP_PORT=|?
AT+TCP_HOST_IP=|?
AT+TCP_STATUS?
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

R"EOF(
Use AT+HELP? for detailed help
)EOF";

const char AT_help_string[] = R"EOF(
ESP-AT Command Help:

Basic Commands:
  AT                    - Test AT startup
  AT?                   - Test AT startup
  AT+?                  - Show this help
  AT+HELP?              - Show this help
  AT+RESET              - Restart device
  AT+ERASE              - Erase all configuration, reset to factory defaults
  AT+ERASE=1            - Erase all configuration and restart immediately)EOF"

#ifdef SUPPORT_WIFI
R"EOF(
WiFi Commands:
  AT+WIFI_ENABLED=<1|0> - Enable/Disable WiFi (1=enable, 0=disable)
  AT+WIFI_ENABLED?      - Get WiFi enable status
  AT+WIFI_SSID=<ssid>   - Set WiFi SSID
  AT+WIFI_SSID?         - Get WiFi SSID
  AT+WIFI_PASS=<pass>   - Set WiFi password
  AT+WIFI_STATUS?       - Get WiFi connection status
  AT+HOSTNAME=<name>    - Set device hostname
  AT+HOSTNAME?          - Get device hostname
Network Commands:
  AT+IPV4=<config>      - Set IPv4 config (DHCP/DISABLE/ip,mask,gw[,dns])
  AT+IPV4?              - Get IPv4 configuration
  AT+IPV6=<config>      - Set IPv6 configuration
  AT+IPV6?              - Get IPv6 configuration
  AT+IP_STATUS?         - Get current IP addresses)EOF"
#endif

#ifdef WIFI_WPS
R"EOF(
WPS Commands:
  AT+WPS_PBC            - Start WPS Push Button Configuration
  AT+WPS_PIN=<pin>      - Start WPS PIN method
  AT+WPS_STOP           - Stop WPS
  AT+WPS_STATUS?        - Get WPS status)EOF"
#endif

#if defined(SUPPORT_TCP) || defined(SUPPORT_UDP)
R"EOF(
Network Configuration:
  AT+NETCONF?                       - Get current network configuration
  AT+NETCONF=(protocol,host,port)   - Configure TCP/UDP connection
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
#endif

#ifdef SUPPORT_TCP
R"EOF(
TCP Commands (Legacy):
  AT+TCP_PORT=<port>    - Set TCP port
  AT+TCP_PORT?          - Get TCP port
  AT+TCP_HOST_IP=<ip>   - Set TCP host IP
  AT+TCP_HOST_IP?       - Get TCP host IP
  AT+TCP_STATUS?        - Get TCP connection status)EOF"
#endif

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
#endif

#ifdef SUPPORT_UDP
R"EOF(
UDP Commands (Legacy):
  AT+UDP_PORT=<port>        - Set UDP port
  AT+UDP_PORT?              - Get UDP port
  AT+UDP_LISTEN_PORT=<port> - Set UDP listen port
  AT+UDP_LISTEN_PORT?       - Get UDP listen port
  AT+UDP6_LISTEN_PORT=<port> - Set UDP6 listen port (IPv6 only)
  AT+UDP6_LISTEN_PORT?      - Get UDP6 listen port
  AT+UDP_SEND=<ip:port>     - Set UDP send IP and port
  AT+UDP_SEND?              - Get UDP send IP and port
  AT+UDP_HOST_IP=<ip>       - Set UDP host IP
  AT+UDP_HOST_IP?           - Get UDP host IP)EOF"
#endif

#ifdef SUPPORT_NTP
R"EOF(
NTP Commands:
  AT+NTP_HOST=<host>    - Set NTP server hostname
  AT+NTP_HOST?          - Get NTP server hostname
  AT+NTP_STATUS?        - Get NTP sync status)EOF"
#endif

#ifdef SUPPORT_UART1
R"EOF(
UART1 Commands:
  AT+UART1=baud,data,parity,stop,rx,tx - Configure UART1 parameters
    baud: 300-3000000, data: 5-8 bits, parity: 0=None/1=Even/2=Odd
    stop: 1-2 bits, rx/tx: pin numbers 0-39
  AT+UART1?             - Get current UART1 configuration)EOF"
#endif

R"EOF(
System Commands:
  AT+LOOP_DELAY=<ms>    - Set main loop delay
  AT+LOOP_DELAY?        - Get main loop delay
  AT+RESET              - Restart device)EOF"

#ifdef VERBOSE
R"EOF(
  AT+VERBOSE=<0|1>      - Enable/disable verbose logging
  AT+VERBOSE?           - Get verbose logging status)EOF"
#endif

#ifdef TIMELOG
R"EOF(
  AT+TIMELOG=<0|1>      - Enable/disable time logging
  AT+TIMELOG?           - Get time logging status)EOF"
#endif

#ifdef LOGUART
R"EOF(
  AT+LOG_UART=<0|1>     - Enable/disable UART logging
  AT+LOG_UART?          - Get UART logging status)EOF"
#endif

R"EOF(
Note: Commands with '?' are queries, commands with '=' set values
)EOF";


NOINLINE
void SAVE(){
  EEPROM.put(CFG_EEPROM, cfg);
  EEPROM.commit();
}

const char* at_cmd_handler(const char* atcmdline){
  unsigned int cmd_len = strlen(atcmdline);
  char *p = NULL;
  D("[AT] [%s], size: %d", atcmdline, cmd_len);
  if(cmd_len == 2 && (p = at_cmd_check("AT", atcmdline, cmd_len))){
    return AT_R_OK;
  } else if(cmd_len == 3 && (p = at_cmd_check("AT?", atcmdline, cmd_len))){
    return AT_R_OK;
  #ifdef SUPPORT_WIFI
  } else if(p = at_cmd_check("AT+WIFI_SSID=", atcmdline, cmd_len)){
    if(strlen(p) > 31)
      return AT_R("+ERROR: WiFI SSID max 31 chars");
    if(strlen(p) == 0){
      // Empty SSID, clear it
      memset((char *)&cfg.wifi_ssid, 0, sizeof(cfg.wifi_ssid));
      cfg.wifi_ssid[0] = '\0';
      SAVE();
      reset_networking();
      return AT_R_OK;
    }
    strncpy((char *)&cfg.wifi_ssid, p, sizeof(cfg.wifi_ssid) - 1);
    cfg.wifi_ssid[sizeof(cfg.wifi_ssid) - 1] = '\0';
    SAVE();
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
      SAVE();
      reset_networking();
      return AT_R_OK;
    }
    strncpy((char *)&cfg.wifi_pass, p, sizeof(cfg.wifi_pass) - 1);
    cfg.wifi_pass[sizeof(cfg.wifi_pass) - 1] = '\0';
    SAVE();
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
          return AT_R_STR(wifi_stat);
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
    SAVE();
    reset_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+WIFI_ENABLED=0", atcmdline, cmd_len)){
    cfg.wifi_enabled = 0;
    SAVE();
    stop_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+WIFI_ENABLED?", atcmdline, cmd_len)){
    return AT_R_STR(cfg.wifi_enabled);
  #ifdef TIMELOG
  } else if(p = at_cmd_check("AT+TIMELOG=1", atcmdline, cmd_len)){
    cfg.do_timelog = 1;
    SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TIMELOG=0", atcmdline, cmd_len)){
    cfg.do_timelog = 0;
    SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TIMELOG?", atcmdline, cmd_len)){
    return AT_R_STR(cfg.do_timelog);
  #endif
  #ifdef VERBOSE
  } else if(p = at_cmd_check("AT+VERBOSE=1", atcmdline, cmd_len)){
    cfg.do_verbose = 1;
    SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+VERBOSE=0", atcmdline, cmd_len)){
    cfg.do_verbose = 0;
    SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+VERBOSE?", atcmdline, cmd_len)){
    return AT_R_STR(cfg.do_verbose);
  #endif
  #ifdef LOGUART
  } else if(p = at_cmd_check("AT+LOG_UART=1", atcmdline, cmd_len)){
    cfg.do_log = 1;
    SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+LOG_UART=0", atcmdline, cmd_len)){
    cfg.do_log = 0;
    SAVE();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+LOG_UART?", atcmdline, cmd_len)){
    return AT_R_STR(cfg.do_log);
  #endif
  #ifdef SUPPORT_NTP
  } else if(p = at_cmd_check("AT+NTP_HOST=", atcmdline, cmd_len)){
    if(strlen(p) > 63)
      return AT_R("+ERROR: NTP hostname max 63 chars");
    if(strlen(p) == 0){
      // Empty hostname, clear it
      memset((char *)&cfg.ntp_host, 0, sizeof(cfg.ntp_host));
      cfg.ntp_host[0] = '\0';
      SAVE();
      return AT_R_OK;
    }
    strncpy((char *)&cfg.ntp_host, p, sizeof(cfg.ntp_host) - 1);
    cfg.ntp_host[sizeof(cfg.ntp_host) - 1] = '\0';
    SAVE();
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
    if(cc != 5)
      return AT_R("+ERROR: Format: baud,data,parity,stop,rx_pin,tx_pin");

    // print the strings for debugging
    for(int i = 0; i < 5; i++) {
      if(cp[i] == NULL)
        break;
      D("[AT] UART1 param %d: %s", i, cp[i]);
    }

    // Parse and validate parameters
    errno = 0;
    uint32_t baud = strtoul(p, NULL, 10);
    if(errno != 0)
      return AT_R("+ERROR: Invalid baud rate");
    uint8_t data = strtoul(cp[0], NULL, 10);
    if(errno != 0)
      return AT_R("+ERROR: Invalid data bits");
    uint8_t parity = strtoul(cp[1], NULL, 10);
    if(errno != 0)
      return AT_R("+ERROR: Invalid parity");
    uint8_t stop = strtoul(cp[2], NULL, 10);
    if(errno != 0)
      return AT_R("+ERROR: Invalid stop bits");
    uint8_t rx_pin = strtoul(cp[3], NULL, 10);
    if(errno != 0)
      return AT_R("+ERROR: Invalid RX pin");
    uint8_t tx_pin = strtoul(cp[4], NULL, 10);
    if(errno != 0)
      return AT_R("+ERROR: Invalid TX pin");

    LOG("[AT] UART1 config: baud=%d, data=%d, parity=%d, stop=%d, rx=%d, tx=%d", baud, data, parity, stop, rx_pin, tx_pin);

    // Validate ranges
    if(baud < 300 || baud > 3000000)
      return AT_R("+ERROR: Baud rate must be 300-3000000");
    if(data < 5 || data > 8)
      return AT_R("+ERROR: Data bits must be 5-8");
    if(parity > 2)
      return AT_R("+ERROR: Parity: 0=None, 1=Even, 2=Odd");
    if(stop < 1 || stop > 2)
      return AT_R("+ERROR: Stop bits must be 1 or 2");
    if(rx_pin > 39 || tx_pin > 39)
      return AT_R("+ERROR: Pin numbers must be 0-39");

    // Update configuration
    cfg.uart1_baud = baud;
    cfg.uart1_data = data;
    cfg.uart1_parity = parity;
    cfg.uart1_stop = stop;
    cfg.uart1_rx_pin = rx_pin;
    cfg.uart1_tx_pin = tx_pin;

    SAVE();

    // Apply new configuration
    setup_uart1();

    return AT_R_OK;
  } else if(p = at_cmd_check("AT+UART1?", atcmdline, cmd_len)){
    String response = String(cfg.uart1_baud) + "," +
                     String(cfg.uart1_data) + "," +
                     String(cfg.uart1_parity) + "," +
                     String(cfg.uart1_stop) + "," +
                     String(cfg.uart1_rx_pin) + "," +
                     String(cfg.uart1_tx_pin);
    return AT_R_STR(response);
  #endif // SUPPORT_UART1
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
      if(tcp_server_active) {
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
      if(tcp6_server_active) {
        response += ",status=ACTIVE,clients=";
        response += get_tcp6_server_client_count();
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
    return AT_R_STR(response);
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
      SAVE();
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
        uint16_t server_port = (uint16_t)strtol(host_or_port, NULL, 10);
        if(server_port == 0) {
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
        uint16_t server_port = (uint16_t)strtol(host_or_port, NULL, 10);
        if(server_port == 0) {
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
        uint16_t port = (uint16_t)strtol(port_str, NULL, 10);
        if(port == 0) {
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
    SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  #endif // SUPPORT_UDP || SUPPORT_TCP
  #ifdef SUPPORT_UDP
  } else if(p = at_cmd_check("AT+UDP_SEND?", atcmdline, cmd_len)){
    if(cfg.udp_send_port == 0 || strlen(cfg.udp_send_ip) == 0)
      return AT_R("+ERROR: UDP send not configured");
    String response = String(cfg.udp_send_ip) + ":" + String(cfg.udp_send_port);
    return AT_R_STR(response);
  } else if(p = at_cmd_check("AT+UDP_SEND=", atcmdline, cmd_len)){
    if(strlen(p) == 0){
      // Empty string means disable UDP send
      cfg.udp_send_port = 0;
      memset(cfg.udp_send_ip, 0, sizeof(cfg.udp_send_ip));
      cfg.udp_send_ip[0] = '\0';
      SAVE();
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
    uint16_t port = (uint16_t)strtol(port_str, NULL, 10);
    if(port == 0)
      return AT_R("+ERROR: invalid udp send port");
    IPAddress tst;
    if(!tst.fromString(ip_str))
      return AT_R("+ERROR: invalid udp send ip");
    // Accept IPv4 or IPv6 string
    strncpy(cfg.udp_send_ip, ip_str, 40-1);
    cfg.udp_send_ip[40-1] = '\0';
    cfg.udp_send_port = port;
    SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+UDP_LISTEN_PORT?", atcmdline, cmd_len)){
    return AT_R_STR(cfg.udp_listen_port);
  } else if(p = at_cmd_check("AT+UDP_LISTEN_PORT=", atcmdline, cmd_len)){
    if(strlen(p) == 0){
      // Empty string means disable UDP
      cfg.udp_listen_port = 0;
      SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_udp_port = (uint16_t)strtol(p, NULL, 10);
    if(new_udp_port == 0)
      return AT_R("+ERROR: invalid UDP port");
    if(new_udp_port != cfg.udp_listen_port){
      cfg.udp_listen_port = new_udp_port;
      SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+UDP6_LISTEN_PORT?", atcmdline, cmd_len)){
    return AT_R_STR(cfg.udp6_listen_port);
  } else if(p = at_cmd_check("AT+UDP6_LISTEN_PORT=", atcmdline, cmd_len)){
    if(strlen(p) == 0){
      // Empty string means disable UDP6
      cfg.udp6_listen_port = 0;
      SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_udp6_port = (uint16_t)strtol(p, NULL, 10);
    if(new_udp6_port == 0)
      return AT_R("+ERROR: invalid UDP6 port");
    if(new_udp6_port != cfg.udp6_listen_port){
      cfg.udp6_listen_port = new_udp6_port;
      SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+UDP_PORT?", atcmdline, cmd_len)){
    return AT_R_STR(cfg.udp_port);
  } else if(p = at_cmd_check("AT+UDP_PORT=", atcmdline, cmd_len)){
    if(strlen(p) == 0){
      // Empty string means disable UDP
      cfg.udp_port = 0;
      SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_udp_port = (uint16_t)strtol(p, NULL, 10);
    if(new_udp_port == 0)
      return AT_R("+ERROR: invalid UDP port");
    if(new_udp_port != cfg.udp_port){
      cfg.udp_port = new_udp_port;
      SAVE();
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
    SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  #endif // SUPPORT_UDP
  #ifdef SUPPORT_TCP
  } else if(p = at_cmd_check("AT+TCP_PORT?", atcmdline, cmd_len)){
    return AT_R_STR(cfg.tcp_port);
  } else if(p = at_cmd_check("AT+TCP_PORT=", atcmdline, cmd_len)){
    if(strlen(p) == 0){
      // Empty string means disable TCP
      cfg.tcp_port = 0;
      SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_tcp_port = (uint16_t)strtol(p, NULL, 10);
    if(new_tcp_port == 0)
      return AT_R("+ERROR: invalid TCP port");
    if(new_tcp_port != cfg.tcp_port){
      cfg.tcp_port = new_tcp_port;
      SAVE();
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
    SAVE();
    reconfigure_network_connections();
    return AT_R_OK;
  #endif // SUPPORT_TCP
  #ifdef SUPPORT_TCP_SERVER
  } else if(p = at_cmd_check("AT+TCP_SERVER_PORT?", atcmdline, cmd_len)){
    return AT_R_STR(cfg.tcp_server_port);
  } else if(p = at_cmd_check("AT+TCP_SERVER_PORT=", atcmdline, cmd_len)){
    if(strlen(p) == 0){
      // Empty string means disable TCP server
      cfg.tcp_server_port = 0;
      SAVE();
      reconfigure_network_connections();
      return AT_R_OK;
    }
    uint16_t new_tcp_server_port = (uint16_t)strtol(p, NULL, 10);
    if(new_tcp_server_port == 0)
      return AT_R("+ERROR: invalid TCP server port");
    if(new_tcp_server_port != cfg.tcp_server_port){
      cfg.tcp_server_port = new_tcp_server_port;
      SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TCP_SERVER_MAX_CLIENTS?", atcmdline, cmd_len)){
    return AT_R_STR(cfg.tcp_server_max_clients);
  } else if(p = at_cmd_check("AT+TCP_SERVER_MAX_CLIENTS=", atcmdline, cmd_len)){
    uint8_t new_max_clients = (uint8_t)strtol(p, NULL, 10);
    if(new_max_clients == 0 || new_max_clients > 8)
      return AT_R("+ERROR: invalid max clients (1-8)");
    if(new_max_clients != cfg.tcp_server_max_clients){
      cfg.tcp_server_max_clients = new_max_clients;
      SAVE();
      reconfigure_network_connections();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TCP_SERVER_STATUS?", atcmdline, cmd_len)){
    String response = "";
    if(tcp_server_active && tcp_server_sock >= 0) {
      response += "ACTIVE,port=";
      response += cfg.tcp_server_port;
      response += ",clients=";
      response += get_tcp_server_client_count();
      response += "/";
      response += cfg.tcp_server_max_clients;
    } else {
      response = "INACTIVE";
    }
    return AT_R_STR(response);
  } else if(p = at_cmd_check("AT+TCP_SERVER_START", atcmdline, cmd_len)){
    if(cfg.tcp_server_port == 0)
      return AT_R("+ERROR: TCP server port not configured");
    start_tcp_server();
    if(tcp_server_active)
      return AT_R_OK;
    else
      return AT_R("+ERROR: failed to start TCP server");
  } else if(p = at_cmd_check("AT+TCP_SERVER_STOP", atcmdline, cmd_len)){
    stop_tcp_server();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+TCP_SERVER_SEND=", atcmdline, cmd_len)){
    if(!tcp_server_active || tcp_server_sock < 0)
      return AT_R("+ERROR: TCP server not active");
    int clients_sent = send_tcp_server_data((const uint8_t*)p, strlen(p));
    if(clients_sent > 0) {
      String response = "SENT to ";
      response += clients_sent;
      response += " clients";
      return AT_R_STR(response);
    } else {
      return AT_R("+ERROR: no connected clients");
    }
  #endif // SUPPORT_TCP_SERVER
  #ifdef LOOP_DELAY
  } else if(p = at_cmd_check("AT+LOOP_DELAY=", atcmdline, cmd_len)){
    errno = 0;
    unsigned int new_c = strtoul(p, NULL, 10);
    if(errno != 0)
      return AT_R("+ERROR: invalid loop delay");
    if(new_c != cfg.main_loop_delay){
      cfg.main_loop_delay = new_c;
      SAVE();
    }
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+LOOP_DELAY?", atcmdline, cmd_len)){
    return AT_R_STR(cfg.main_loop_delay);
  #endif // LOOP_DELAY
  } else if(p = at_cmd_check("AT+HOSTNAME=", atcmdline, cmd_len)){
    if(strlen(p) > 63)
      return AT_R("+ERROR: hostname max 63 chars");
    strncpy((char *)&cfg.hostname, p, sizeof(cfg.hostname) - 1);
    cfg.hostname[sizeof(cfg.hostname) - 1] = '\0';
    SAVE();
    reset_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+HOSTNAME?", atcmdline, cmd_len)){
    if(strlen(cfg.hostname) == 0)
      return AT_R_STR(DEFAULT_HOSTNAME);
    else
      return AT_R_STR(cfg.hostname);
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

    SAVE();
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
    return AT_R_STR(response);
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

    SAVE();
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
    return AT_R_STR(response);
  #ifdef SUPPORT_TCP
  } else if(p = at_cmd_check("AT+TCP_STATUS?", atcmdline, cmd_len)){
    String response = "";
    if(strlen(cfg.tcp_host_ip) == 0 || cfg.tcp_port == 0){
      response = "TCP not configured";
    } else {
      response = "TCP Host: " + String(cfg.tcp_host_ip) + ":" + String(cfg.tcp_port);
    }
    return AT_R_STR(response);
  #endif // SUPPORT_WIFI && SUPPORT_TCP
  #endif // SUPPORT_WIFI
  } else if(p = at_cmd_check("AT+ERASE", atcmdline, cmd_len)){
    // Erase all configuration from EEPROM and reset to factory defaults
    LOG("[ERASE] Erasing configuration and resetting to factory defaults");

    // Stop all network connections before erasing config
    stop_networking();

    // Clear the entire EEPROM section used by config
    for(int i = 0; i < sizeof(cfg); i++) {
      EEPROM.write(CFG_EEPROM + i, 0xFF);
    }
    EEPROM.commit();

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
    return AT_R_STR(AT_short_help_string);
  } else {
    return AT_R("+ERROR: unknown command");
  }
  return AT_R("+ERROR: unknown error");
}
#endif // BLUETOOTH_UART_AT && BT_BLE

// BLE UART Service - Nordic UART Service UUID
#if defined(BLUETOOTH_UART_AT) && defined(BT_BLE)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer* pServer = NULL;
BLEService* pService = NULL;
BLECharacteristic* pTxCharacteristic = NULL;
BLECharacteristic* pRxCharacteristic = NULL;
bool deviceConnected = false;

// BLE UART buffer
String bleCommandBuffer = "";
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
      deviceConnected = true;
      LOG("[BLE] connected, MTU: %d", pServer->getPeerMTU(deviceConnected));
    };

    void onDisconnect(BLEServer* pServer) {
      doYIELD;
      deviceConnected = false;
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

// BLE Characteristic Callbacks
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      doYIELD;
      D("[BLE] RX %d>>%s<<", pCharacteristic->getValue().length(), pCharacteristic->getValue().c_str());
      bleCommandBuffer.clear();
      String rxValue = pCharacteristic->getValue().c_str();

      // Process each byte individually to handle command terminators properly
      for (size_t i = 0; i < rxValue.length(); i++) {
        doYIELD;
        if (rxValue[i] == '\n' || rxValue[i] == '\r') {
          // Command terminator found, mark command as ready if buffer is not empty
          if (bleCommandBuffer.length() > 0)
            bleCommandReady = true;
        } else {
          // Add character to command buffer
          bleCommandBuffer.concat(rxValue[i]);
        }

        // Check if command buffer is too long
        if (bleCommandBuffer.length() > 4096) {
          // Reset buffer if it's too long without terminator
          LOG("[BLE] Command buffer overflow, clearing buffer");
          bleCommandBuffer.clear();
          bleCommandReady = false;
        }
      }
      if(bleCommandReady)
        D("[BLE] Command Ready: %d (will be processed in main loop)", bleCommandReady);
      // Don't call handle_ble_command() directly from callback - let main loop handle it
    }
};

void setup_ble() {
  LOG("[BLE] Setup");

  // Create the BLE Device
  BLEDevice::init(BLUETOOTH_UART_DEVICE_NAME);
  BLEDevice::setMTU(ble_mtu); // Request MTU matching AT buffer size

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
  deviceConnected = false;

  LOG("[BLE] Setup complete");
}

void handle_ble_command() {
  // Don't handle commands if BLE is disabled or already processing a command
  if (ble_advertising_start == 0 || bleCommandProcessing)
    return;

  if (bleCommandReady && bleCommandBuffer.length() > 0) {
    bleCommandProcessing = true; // Set flag to prevent re-entrant calls
    // Process the BLE command using the same AT command handler

    #ifdef DEBUG
    // buffer log/debug
    D("[BLE] Handling BLE command: >>%s<<, size: %d", bleCommandBuffer.c_str(), bleCommandBuffer.length());
    // buffer log/debug in hex
    D("[BLE] command buffer in hex: ");
    for (size_t i = 0; i < bleCommandBuffer.length(); i++) {
      R("%02X", (unsigned char)bleCommandBuffer[i]);
    }
    R("\n");
    #endif // DEBUG

    // Check if the command starts with "AT"
    if (bleCommandBuffer.startsWith("AT")) {
      // Handle AT command
      ble_send_response(at_cmd_handler(bleCommandBuffer.c_str()));
    } else {
      ble_send_response((const char*)("+ERROR: invalid command"));
    }

    bleCommandBuffer.clear();
    bleCommandReady = false;
    bleCommandProcessing = false; // Clear flag after processing
  }
}

NOINLINE
void ble_send_response(const char *response) {
  if (ble_advertising_start == 0 || !deviceConnected || !pTxCharacteristic)
    return;

  // Send response with line terminator
  ble_send_n((uint8_t *)response, strlen(response));
  ble_send_n((uint8_t *)("\r\n"), 2);
}

NOINLINE
void ble_send_n(uint8_t *bstr, size_t len) {
  if (ble_advertising_start == 0)
    return;

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
  if (deviceConnected && pTxCharacteristic) {
    // Split response into chunks (BLE characteristic limit), use negotiated MTU
    size_t o = 0;
    uint16_t cs = 0;
    while (o < len) {
      doYIELD;
      cs = ble_mtu - 3; // ATT_MTU-3 for payload
      if(cs > len - o)
        cs = len - o;

      uint8_t chunk[cs] = {0};
      memcpy(chunk, bstr + o, cs);
      D("[BLE] Sending chunk size: %d, >>%s<<", cs, chunk);
      pTxCharacteristic->setValue((uint8_t *)chunk, cs);

      // Check if still connected before notifying
      if (deviceConnected) {
        pTxCharacteristic->notify();
        // Small delay to ensure notification is sent
        delay(10);
      } else {
        // Exit if disconnected during transmission
        break;
      }

      o += cs;
      doYIELD;
    }
  }
}

NOINLINE
void ble_send(const char *dstr) {
  ble_send_n((uint8_t *)dstr, strlen(dstr));
}

void start_advertising_ble(){
  LOG("[BLE] Enabling Bluetooth and starting advertising");
  if (pServer){
    pServer->getAdvertising()->stop();
    pServer->getAdvertising()->start();
  }
  ble_advertising_start = millis();
  LOG("[BLE] Advertising started, waiting for client connection");
}

void stop_advertising_ble() {
  // Mark as disabled
  ble_advertising_start = 0;

  if(deviceConnected) {
    LOG("[BLE] Disconnecting from connected device");
    pServer->disconnect(0);
    deviceConnected = false;
  }

  // Stop advertising
  LOG("[BLE] Stopping advertising and disabling Bluetooth");
  if (pServer)
    pServer->getAdvertising()->stop();

  // don't release memory, ESP-IDF should've handled it, but on BLEDevice::init() it stackdumps
  //BLEDevice::deinit(false);

  LOG("[BLE] Bluetooth disabled");
}
#endif // BT_BLE

void setup_cfg(){
  // EEPROM read
  EEPROM.begin(sizeof(cfg));
  EEPROM.get(CFG_EEPROM, cfg);
  // was (or needs) initialized?
  if(cfg.initialized != CFGINIT || cfg.version != CFGVERSION){
    cfg.do_verbose = 1;
    LOG("reinitializing config");
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
    SAVE();
    LOG("reinitializing config done");
  }
}

#ifdef SUPPORT_UART1
void setup_uart1(){
  // Stop UART1 if already running
  Serial1.end();

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

  // Configure UART1 with new parameters
  Serial1.begin(cfg.uart1_baud, config, cfg.uart1_rx_pin, cfg.uart1_tx_pin);

  LOG("[UART1] Configured: %lu baud, %d%c%d, RX=%d, TX=%d",
      cfg.uart1_baud, cfg.uart1_data,
      (cfg.uart1_parity == 0) ? 'N' : (cfg.uart1_parity == 1) ? 'E' : 'O',
      cfg.uart1_stop, cfg.uart1_rx_pin, cfg.uart1_tx_pin);
}
#endif // SUPPORT_UART1

#if defined(SUPPORT_WIFI) && defined(WIFI_WPS)
/* WPS (WiFi Protected Setup) Functions both PBC and PIN */
bool start_wps(const char *pin) {
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
          }
          break;
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          LOG("[WiFi] STA got IP: %s", WiFi.localIP().toString().c_str());
          reconfigure_network_connections();
          break;
      case ARDUINO_EVENT_WIFI_STA_LOST_IP:
          LOG("[WiFi] STA lost IP");
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
            SAVE();

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
}

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
  static char T_buffer[512] = {""};
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
  bool is_ble_advertising = (ble_advertising_start != 0);
  bool is_ble_connected = (deviceConnected);

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
        LOG("[BUTTON] Normal press detected (%lu ms), toggling BLE advertising", press_duration);
        // Normal press - toggle BLE advertising
        if (ble_advertising_start == 0) {
          // BLE is currently disabled, start advertising
          start_advertising_ble();
          LOG("[BUTTON] BLE advertising started - will stop on timeout if no connection, or when button pressed again");
        } else {
          // BLE is currently enabled, stop advertising
          stop_advertising_ble();
          LOG("[BUTTON] BLE advertising stopped");
        }
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


void setup(){
  // Serial setup, init at 115200 8N1
  Serial.begin(115200);

  // enable all ESP32 core logging
  #ifdef DEBUG
  esp_log_level_set("*", ESP_LOG_VERBOSE);
  #endif

  // setup cfg
  setup_cfg();

  // Setup AT command handler
  #ifdef UART_AT
  ATSc.SetDefaultHandler(&sc_cmd_handler);
  #endif

  // BlueTooth SPP setup possible?
  #if defined(BLUETOOTH_UART_AT) && defined(BT_BLE)
  setup_ble();
  #endif

  #if defined(BLUETOOTH_UART_AT) && defined(BT_CLASSIC)
  LOG("[BT] setting up Bluetooth Classic");
  SerialBT.begin(BLUETOOTH_UART_DEVICE_NAME);
  SerialBT.setPin(BLUETOOTH_UART_DEFAULT_PIN);
  SerialBT.register_callback(BT_EventHandler);
  ATScBT.SetDefaultHandler(&sc_cmd_handler);
  #endif

  // setup WiFi with ssid/pass from EEPROM if set
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
  log_esp_info();
}

#define UART1_READ_SIZE       16 // read bytes at a time from UART1
#define UART1_BUFFER_SIZE    512 // max size of UART1 buffer
#define UART1_WRITE_SIZE      16 // write bytes at a time to UART1
#define TCP_READ_SIZE         16 // read bytes at a time from TCP
#define REMOTE_BUFFER_SIZE  1024 // max size of REMOTE buffer

// from "LOCAL", e.g. "UART1"
char inbuf[UART1_BUFFER_SIZE] = {0};
size_t inlen = 0;
char *inbuf_max = (char *)&inbuf + UART1_BUFFER_SIZE - UART1_READ_SIZE; // max size of inbuf

// from "REMOTE", e.g. TCP, UDP
char outbuf[REMOTE_BUFFER_SIZE] = {0};
size_t outlen = 0;
uint8_t sent_ok = 1;

void loop(){
  LOOP_D("[LOOP] Start main loop");

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
  if (ble_advertising_start != 0 && !deviceConnected && millis() - ble_advertising_start > BLE_ADVERTISING_TIMEOUT){
    stop_advertising_ble();
    #ifdef SUPPORT_WIFI
    reset_networking();
    #endif // SUPPORT_WIFI
  }

  // Handle pending BLE commands
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
  LOOP_D("[LOOP] Checking for available data, inlen: %d, inbuf max: %d", inlen, (int)(inbuf_max - inbuf));
  size_t to_r = 0;
  while((to_r = Serial1.available()) > 0 && inbuf + inlen < inbuf_max) {
    doYIELD;
    // read bytes into inbuf
    to_r = Serial1.read(inbuf + inlen, UART1_READ_SIZE);
    if(to_r <= 0)
        break; // nothing read
    inlen += to_r;
    #ifdef LED
    last_uart1_activity = millis(); // Trigger LED activity for UART1 receive
    #endif // LED
    D("[UART1]: Read %d bytes, total: %d, data: >>%s<<", to_r, inlen, inbuf);
  }
  #endif // SUPPORT_UART1

  #ifdef SUPPORT_UDP
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
        sent_ok = 1; // mark as sent
      } else if (sent < 0) {
        LOGE("[UDP] Sent error %d bytes, total: %d", sent, inlen);
        sent_ok = 0; // mark as not sent
      } else if (sent == 0) {
        D("[UDP] Sent 0 bytes, total: %d", inlen);
        sent_ok = 0; // mark as not sent
      }
    }
    if (udp_out_sock != -1){
      int sent = send_udp_data(udp_out_sock, (const uint8_t*)inbuf, inlen, cfg.udp_send_ip, cfg.udp_send_port, "[UDP_SEND]");
      if (sent > 0) {
        #ifdef LED
        last_udp_activity = millis(); // Trigger LED activity for UDP send
        #endif // LED
        D("[UDP_SEND] Sent %d bytes, total: %d, data: >>%s<<", sent, inlen, inbuf);
        sent_ok = 1; // mark as sent
      } else if (sent < 0) {
        LOGE("[UDP_SEND] Sent error %d bytes, total: %d", sent, inlen);
        sent_ok = 0; // mark as not sent
      } else if (sent == 0) {
        D("[UDP_SEND] Sent 0 bytes, total: %d", inlen);
        sent_ok = 0; // mark as not sent
      }
    }
  }
  #endif // SUPPORT_UDP

  #ifdef SUPPORT_TCP
  // TCP send
  LOOP_D("[LOOP] Check for outgoing TCP data");
  if (tcp_sock != -1 && inlen > 0) {
    if (!tcp_connection_writable){
      D("[TCP] No valid connection, cannot send data");
      sent_ok = 0; // mark as not sent
    } else {
      int sent = send_tcp_data((const uint8_t*)inbuf, inlen);
      if (sent > 0) {
        #ifdef LED
        last_tcp_activity = millis(); // Trigger LED activity for TCP send
        #endif // LED
        D("[TCP] Sent %d bytes, total: %d", sent, inlen);
        sent_ok = 1; // mark as sent
      } else if (sent == -1) {
        if(errno && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINPROGRESS){
          // Error occurred, log it
          LOGE("[TCP] send error, closing connection");
        } else {
          // Socket not ready for writing, data will be retried on next loop
          E("[TCP] socket not ready for writing, will retry", errno);
        }
        sent_ok = 0; // mark as not sent
      } else if (sent == 0) {
        // Socket not ready for writing, data will be retried on next loop
        LOG("[TCP] connection closed by remote host");
        sent_ok = 0; // mark as not sent
      }
    }
  }
  #endif // SUPPORT_TCP

  #ifdef SUPPORT_TCP
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
  #endif // SUPPORT_TCP

  #ifdef SUPPORT_TCP_SERVER
  // TCP Server handling
  LOOP_D("[LOOP] Check TCP server connections");
  if(tcp_server_active && tcp_server_sock >= 0) {
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
  if(tcp6_server_active && tcp6_server_sock >= 0) {
    handle_tcp6_server();
    // Update last activity time if we have clients
    if(get_tcp6_server_client_count() > 0) {
      #ifdef LED
      last_tcp_activity = millis(); // Trigger LED activity for TCP6 server
      #endif // LED
    }
  }

  LOOP_D("[LOOP] TCP_SERVER Check for outgoing TCP Server data");
  if (tcp_server_active && tcp_server_sock >= 0) {
    if(inlen > 0){
      int clients_sent = send_tcp_server_data((const uint8_t*)inbuf, inlen);
      if (clients_sent > 0) {
        #ifdef LED
        last_tcp_activity = millis(); // Trigger LED activity for TCP server send
        #endif // LED
        D("[TCP_SERVER] Sent %d bytes to %d clients, data: >>%s<<", inlen, clients_sent, inbuf);
        sent_ok = 1; // mark as sent
      } else {
        LOOP_D("[TCP_SERVER] No clients connected to send data to");
        // Don't mark as error if no clients are connected
      }
    }

    if (outlen + TCP_READ_SIZE >= sizeof(outbuf)) {
      D("[TCP_SERVER] outbuf full, cannot read more data, outlen: %d", outlen);
    } else {
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

  LOOP_D("[LOOP] TCP6_SERVER Check for outgoing TCP6 Server data");
  if (tcp6_server_active && tcp6_server_sock >= 0) {
    if(inlen > 0){
      tcp6_server_send(inbuf, inlen);
      if (get_tcp6_server_client_count() > 0) {
        #ifdef LED
        last_tcp_activity = millis(); // Trigger LED activity for TCP6 server send
        #endif // LED
        D("[TCP6_SERVER] Sent %d bytes to clients, data: >>%s<<", inlen, inbuf);
        sent_ok = 1; // mark as sent
      } else {
        LOOP_D("[TCP6_SERVER] No clients connected to send data to");
      }
    }

    if (outlen + TCP_READ_SIZE >= sizeof(outbuf)) {
      D("[TCP6_SERVER] outbuf full, cannot read more data, outlen: %d", outlen);
    } else {
      int r = tcp6_server_receive(outbuf + outlen, TCP_READ_SIZE);
      if (r > 0) {
        // data received
        #ifdef LED
        last_tcp_activity = millis(); // Trigger LED activity for TCP6 server receive
        #endif // LED
        D("[TCP6_SERVER] Received %d bytes, total: %d, data: >>%s<<", r, outlen + r, outbuf);
        outlen += r;
      }
    }
  }
  #endif // SUPPORT_TCP_SERVER

  #ifdef SUPPORT_UDP
  // UDP read
  LOOP_D("[LOOP] Check for incoming UDP data");

  // in/out UDP socket read
  udp_read(udp_sock, outbuf, outlen, UDP_READ_MSG_SIZE, REMOTE_BUFFER_SIZE, "[UDP]");

  // in UDP socket read
  udp_read(udp_listen_sock, outbuf, outlen, UDP_READ_MSG_SIZE, REMOTE_BUFFER_SIZE, "[UDP_LISTEN]");

  // in UDP6 socket read
  udp_read(udp6_listen_sock, outbuf, outlen, UDP_READ_MSG_SIZE, REMOTE_BUFFER_SIZE, "[UDP6_LISTEN]");
  #endif // SUPPORT_UDP

  // just wifi check
  #ifdef SUPPORT_WIFI
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
  #endif // SUPPORT_WIFI

  #ifdef DEBUG
  // Log ESP info periodically when DEBUG is enabled
  LOOP_D("[LOOP] ESP info log check");
  if(last_esp_info_log ==0 || millis() - last_esp_info_log > 30000) { // Log every 30 seconds
    log_esp_info();
    last_esp_info_log = millis();
  }
  #endif // DEBUG

  #if defined(SUPPORT_WIFI) && (defined(SUPPORT_TCP) || defined(SUPPORT_UDP))
  // TCP connection check at configured interval
  LOOP_D("[LOOP] TCP/UDP check");
  if(WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS){
    // connected, check every 500ms
    if(last_tcp_check == 0 || millis() - last_tcp_check > 500){
      last_tcp_check = millis();
      #if defined(SUPPORT_WIFI) && defined(SUPPORT_TCP)
      if(strlen(cfg.tcp_host_ip) != 0 && cfg.tcp_port != 0){
        doYIELD;
        int conn_ok = check_tcp_connection(500000);
        if(!conn_ok){
          sent_ok = 0; // mark as not sent
          D("[LOOP] TCP Connection lost");
          connect_tcp();
        }
      }
      #endif // SUPPORT_WIFI && SUPPORT_TCP
    }
  }
  #endif // SUPPORT_WIFI && (SUPPORT_TCP || SUPPORT_UDP)

  #if defined(SUPPORT_WIFI) && defined(SUPPORT_NTP)
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
        LOG("[NTP] NTP is synced");
        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);
        if(timeinfo.tm_hour != last_hour){
          last_hour = timeinfo.tm_hour;
          LOG("[NTP] NTP new time: %s", PT());
        }
      } else if(sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && last_hour != -1){
        D("[NTP] NTP sync ok");
      } else {
        D("[NTP] not yet synced");
      }
    }
  }
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
    sent_ok = 1;
    memset(inbuf, 0, sizeof(inbuf));
  }

  #ifdef LOOP_DELAY
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
  #endif // LOOP_DELAY

  LOOP_D("[LOOP] End main loop");
}
