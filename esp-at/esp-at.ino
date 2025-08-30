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
#include <sys/time.h>
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <esp_wps.h>
#ifdef DEBUG
#define USE_ESP_IDF_LOG
#define CORE_DEBUG_LEVEL 5
#define LOG_LOCAL_LEVEL 5
#endif
#include <esp_log.h>
#endif
#ifdef ARDUINO_ARCH_ESP8266
#ifdef DEBUG
#define USE_ESP_IDF_LOG
#define CORE_DEBUG_LEVEL 5
#define LOG_LOCAL_LEVEL 5
#endif
#include <esp_log.h>
#include <ESP8266WiFi.h>
#endif
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include "time.h"
#include "EEPROM.h"
#include "esp_sntp.h"

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

#define WIFI_WPS 1

#ifndef SUPPORT_UART1
#define SUPPORT_UART1 1
#endif // SUPPORT_UART1

#ifdef SUPPORT_UART1
#define UART1_RX_PIN 0
#define UART1_TX_PIN 1
#endif // SUPPORT_UART1


#ifndef VERBOSE
#define VERBOSE
#endif

#ifndef TIMELOG
#define TIMELOG
#endif

#ifndef DEFAULT_HOSTNAME
#define DEFAULT_HOSTNAME "uart"
#endif

#ifndef UART_AT
#define UART_AT
#endif

#if defined(UART_AT) || defined(BT_CLASSIC)
#include "SerialCommands.h"
#endif

#ifdef VERBOSE
 #define LOG_TIME_FORMAT "[\%H:\%M:\%S][info]: "
#endif
#ifdef DEBUG
 #define DEBUG_TIME_FORMAT "[\%H:\%M:\%S][debug]: "
#endif

#if defined(DEBUG) || defined(VERBOSE)
static char _date_outstr[20] = {0};
static time_t _t;
static struct tm _tm;
void print_time_to_serial(const char *tformat = "[\%H:\%M:\%S]: "){
  time(&_t);
  localtime_r(&_t, &_tm);
  memset(_date_outstr, 0, sizeof(_date_outstr));
  strftime(_date_outstr, sizeof(_date_outstr), tformat, &_tm);
  Serial.print(_date_outstr);
}

static char _buf[256] = {0};
void do_printf(uint8_t t, const char *tf, const char *format, ...) {
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
#endif

#ifdef VERBOSE
 #if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  #define LOG(...)    if(cfg.do_verbose){do_printf(3, LOG_TIME_FORMAT, __VA_ARGS__);}
  #define LOGT(...)   if(cfg.do_verbose){do_printf(2, LOG_TIME_FORMAT, __VA_ARGS__);}
  #define LOGR(...)   if(cfg.do_verbose){do_printf(0, LOG_TIME_FORMAT, __VA_ARGS__);}
  #define LOGE(...)   if(cfg.do_verbose){do_printf(2, LOG_TIME_FORMAT, __VA_ARGS__);do_printf(0, NULL, ", errno: %d (%s)\n", errno, get_errno_string(errno));};
 #else
  #define LOG(...)    if(cfg.do_verbose){do_printf(3, LOG_TIME_FORMAT, __VA_ARGS__);}
  #define LOGT(...)   if(cfg.do_verbose){do_printf(2, LOG_TIME_FORMAT, __VA_ARGS__);}
  #define LOGR(...)   if(cfg.do_verbose){do_printf(0, LOG_TIME_FORMAT, __VA_ARGS__);}
  #define LOGE(...)   if(cfg.do_verbose){do_printf(2, LOG_TIME_FORMAT, __VA_ARGS__);do_printf(0, NULL, ", errno: %d (%s)\n", errno, get_errno_string(errno));};
 #endif
#else
 #define LOG(...)
 #define LOGT(...)
 #define LOGR(...)
 #define LOGE(...)
#endif

#ifdef DEBUG
 #if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  #define D(...)   do_printf(3, DEBUG_TIME_FORMAT, __VA_ARGS__);
  #define T(...)   do_printf(2, DEBUG_TIME_FORMAT, __VA_ARGS__);
  #define R(...)   do_printf(0, DEBUG_TIME_FORMAT, __VA_ARGS__);
  #define E(...)   do_printf(2, DEBUG_TIME_FORMAT, __VA_ARGS__);do_printf(0, NULL, ", errno: %d (%s)\n", errno, get_errno_string(errno));
 #else
  #define D(...)   do_printf(3, DEBUG_TIME_FORMAT, __VA_ARGS__);
  #define T(...)   do_printf(2, DEBUG_TIME_FORMAT, __VA_ARGS__);
  #define R(...)   do_printf(0, DEBUG_TIME_FORMAT, __VA_ARGS__);
  #define E(...)   do_printf(2, DEBUG_TIME_FORMAT, __VA_ARGS__);do_printf(0, NULL, ", errno: %d (%s)\n", errno, get_errno_string(errno));
 #endif
#else
 #define D(...)
 #define T(...)
 #define R(...)
 #define E(...)
#endif

#ifdef LOOP_DEBUG
#define LOOP_D  D
#define LOOP_R  R
#define LOOP_DN T
#define LOOP_DE E
#else
#define LOOP_D(...)
#define LOOP_R(...)
#define LOOP_DN(...)
#define LOOP_DE(...)
#endif

#ifndef BLUETOOTH_UART_AT
#define BLUETOOTH_UART_AT
#endif

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
#endif

#define BT_BLE
#ifdef BT_BLE
#include <BLEUUID.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEService.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>
#endif
#endif // BLUETOOTH_UART_AT

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
 #define doYIELD yield();
#else
 #define doYIELD
#endif

#ifdef UART_AT
/* our AT commands over UART */
char atscbu[128] = {""};
SerialCommands ATSc(&Serial, atscbu, sizeof(atscbu), "\r\n", "\r\n");
#endif

#define CFGVERSION 0x01 // switch between 0x01/0x02 to reinit the config struct change
#define CFGINIT    0x72 // at boot init check flag
#define CFG_EEPROM 0x00

#define IPV4_DHCP    1
#define IPV4_STATIC  2
#define IPV6_DHCP    4

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
  char wifi_ssid[32]   = {0}; // max 31 + 1
  char wifi_pass[64]   = {0}; // nax 63 + 1
  char ntp_host[64]    = {0}; // max hostname + 1
  uint8_t ip_mode      = IPV4_DHCP | IPV6_DHCP;
  char hostname[64]    = {0}; // max hostname + 1
  uint8_t ipv4_addr[4] = {0}; // static IP address
  uint8_t ipv4_gw[4]   = {0}; // static gateway
  uint8_t ipv4_mask[4] = {0}; // static netmask
  uint8_t ipv4_dns[4]  = {0}; // static DNS server

  uint16_t udp_port    = 0;
  char udp_host_ip[15] = {0}; // IPv4 or IPv6 string, TODO: support hostname
  // TCP support
  uint16_t tcp_port    = 0;
  char tcp_host_ip[40] = {0}; // IPv4 or IPv6 string, up to 39 chars for IPv6
};
cfg_t cfg;

#ifndef SUPPORT_NTP
#define SUPPORT_NTP
#endif // SUPPORT_NTP
#ifdef SUPPORT_NTP
long last_ntp_log = 0;
uint8_t ntp_is_synced = 1;
void cb_ntp_synced(struct timeval *tv){
  doYIELD;
  time_t t;
  struct tm gm_new_tm;
  time(&t);
  localtime_r(&t, &gm_new_tm);
  char d_outstr[100];
  strftime(d_outstr, 100, ", sync: %a, %b %d %Y %H:%M:%S%z %Z (%s)", &gm_new_tm);
  LOG("NTP synced, new time: %d %s %s", t, ctime(&t), d_outstr);
  ntp_is_synced = 1;
}

void setup_ntp(){
  // if we have a NTP host configured, sync
  if(strlen(cfg.ntp_host)){
    LOG("[NTP] Setting up NTP with host: %s, interval: %d, timezone: UTC", cfg.ntp_host, 4 * 3600);
    if(esp_sntp_enabled()){
      LOG("NTP already enabled, skipping setup");
      sntp_set_sync_interval(4 * 3600 * 1000UL);
      sntp_setservername(0, (char*)&cfg.ntp_host);
    } else {
      LOG("[NTP] Setting up NTP sync");
      esp_sntp_stop();
      sntp_set_sync_interval(4 * 3600 * 1000UL);
      sntp_setservername(0, (char*)&cfg.ntp_host);
      sntp_set_time_sync_notification_cb(cb_ntp_synced);
      sntp_setoperatingmode(SNTP_OPMODE_POLL);
      sntp_init();
    }
    setenv("TZ", "UTC", 1);
    tzset();
  }
}
#endif // SUPPORT_NTP

/* state flags */
long last_wifi_check = 0;
long last_wifi_info_log = 0;
long last_wifi_reconnect = 0;

#ifdef TIMELOG
long last_time_log = 0;
#endif // TIMELOG

#ifdef DEBUG
long last_esp_info_log = 0;
#endif // DEBUG

#ifdef BT_BLE
long ble_advertising_start = 0;
#define BLE_ADVERTISING_TIMEOUT 10000   // 10 seconds in milliseconds
#endif // BT_BLE

bool button_pressed = false;

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
#ifdef WIFI_WPS
bool wps_running = false;
unsigned long wps_start_time = 0;
#define WPS_TIMEOUT_MS 120000  // 2 minutes timeout for WPS
esp_wps_config_t wps_config;
#endif

void setup_wifi(){
  LOG("[WiFi] setup started");
  LOG("[WiFi] MAC: %s", WiFi.macAddress().c_str());
  LOG("[WiFi] IP Mode configured: %s%s%s",
      (cfg.ip_mode & IPV4_DHCP) ? "IPv4 DHCP " : "",
      (cfg.ip_mode & IPV4_STATIC) ? "IPv4 STATIC " : "",
      (cfg.ip_mode & IPV6_DHCP) ? "IPv6 DHCP " : "");
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
    return;
  }
  if(WiFi.status() == WL_CONNECTED){
    LOG("[WiFi] Already connected, skipping WiFi setup");
    return;
  }

  LOG("[WiFi] setting up WiFi");

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

  WiFi.mode(WIFI_STA);
  WiFi.enableSTA(true);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  if(cfg.hostname){
    WiFi.setHostname(cfg.hostname);
  } else {
    WiFi.setHostname(DEFAULT_HOSTNAME);
  }

  // IPv6 configuration, before WiFi.begin()!
  if(cfg.ip_mode & IPV6_DHCP){
    LOG("[WiFi] Using DHCP for IPv6");
    WiFi.enableIPv6(true);
  }

  // connect to Wi-Fi
  LOG("[WiFi] Connecting to %s", cfg.wifi_ssid);
  WiFi.persistent(false);
  LOG("[WiFi] adding event handler");
  WiFi.removeEvent(WiFiEvent);
  WiFi.onEvent(WiFiEvent);
  // These need to be called before WiFi.begin()!
  WiFi.setMinSecurity(WIFI_AUTH_WPA2_PSK); // require WPA2
  WiFi.setScanMethod(WIFI_FAST_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);

  // after WiFi.config()!
  LOG("[WiFi] Starting connection");
  uint8_t ok = 0;
  if(strlen(cfg.wifi_pass) == 0) {
    if(cfg.do_verbose){
      LOG("[WiFi] No password, connecting to open network");
    }
    ok = WiFi.begin(cfg.wifi_ssid, NULL, 0, NULL, true);
  } else {
    ok = WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass, 0, NULL, true);
  }
  if(ok != WL_CONNECTED && ok != WL_IDLE_STATUS){
    LOG("[WiFi] waiting for connection");
  } else {
    LOG("[WiFi] connected");
  }

  // WiFi.setTxPower(WIFI_POWER_19_5dBm);
  // Lower power to save battery and reduce interference, mostly reflections
  // due to bad antenna design?
  // See https://forum.arduino.cc/t/no-wifi-connect-with-esp32-c3-super-mini/1324046/12
  // See https://roryhay.es/blog/esp32-c3-super-mini-flaw
  WiFi.setTxPower(WIFI_POWER_8_5dBm);

  // setup NTP sync if needed
  #ifdef SUPPORT_NTP
  setup_ntp();
  #endif
}

void stop_networking(){
  LOG("[WiFi] Stop networking");
  // first stop WiFi
  WiFi.disconnect(true);
  while(WiFi.status() == WL_CONNECTED){
    doYIELD;
    LOG("[WiFi] waiting for disconnect, status: %d", WiFi.status());
    delay(100);
  }
  //WiFi.mode(WIFI_MODE_NULL);
  WiFi.enableSTA(false);
  WiFi.enableAP(false);
  LOG("[WiFi] Stop networking done");
}

void start_networking(){
  LOG("[WiFi] Start networking");
  // now reconnect to WiFi
  setup_wifi();
  // and reconfigure network connections
  reconfigure_network_connections();
  LOG("[WiFi] Start networking done");
}

void reset_networking(){
  if(wps_running)
      return;
  LOG("[WiFi] not connected, reset networking...");
  // first stop WiFi
  stop_networking();
  // start networking
  start_networking();
  LOG("[WiFi] not connected, reset networking done");
}

void reconfigure_network_connections(){
  LOG("[WiFi] network connections, wifi status: %s", (WiFi.status() == WL_CONNECTED) ? "connected" : "not connected");
  if(WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS){
    // tcp - attempt both IPv4 and IPv6 connections based on target and available addresses
    if(strlen(cfg.tcp_host_ip) != 0 && cfg.tcp_port != 0){
      if(is_ipv6_addr(cfg.tcp_host_ip) && has_ipv6_address()){
        connections_tcp_ipv6();
      } else if(!is_ipv6_addr(cfg.tcp_host_ip) && has_ipv4_address()) {
        connections_tcp_ipv4();
      } else {
        LOG("[TCP] No matching IP version available for target %s", cfg.tcp_host_ip);
      }
    }

    // udp - attempt both IPv4 and IPv6 connections based on target and available addresses
    if(strlen(cfg.udp_host_ip) != 0 && cfg.udp_port != 0){
      if(is_ipv6_addr(cfg.udp_host_ip) && has_ipv6_address()){
        connections_udp_ipv6();
      } else if(!is_ipv6_addr(cfg.udp_host_ip) && has_ipv4_address()) {
        connections_udp_ipv4();
      } else {
        LOG("[UDP] No matching IP version available for target %s", cfg.udp_host_ip);
      }
    }
  }
  return;
}

#ifndef SUPPORT_TCP
#define SUPPORT_TCP
#endif

// Helper function to get human-readable errno messages
#if defined(SUPPORT_TCP) || defined(SUPPORT_UDP)
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

#ifdef SUPPORT_TCP
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <lwip/netdb.h>

WiFiClient tcp_client;
WiFiUDP udp;
int tcp_sock = -1;
int udp_sock = -1;
uint8_t valid_tcp_host = 0;
uint8_t valid_udp_host = 0;

// Helper: check if string is IPv6
bool is_ipv6_addr(const char* ip) {
  return strchr(ip, ':') != NULL;
}

// Helper: check if we have a valid IPv4 address
bool has_ipv4_address() {
  if(!(cfg.ip_mode & IPV4_DHCP) && !(cfg.ip_mode & IPV4_STATIC)) {
    return false; // IPv4 not enabled
  }
  IPAddress ipv4 = WiFi.localIP();
  return (ipv4 != IPAddress(0,0,0,0) && ipv4 != IPAddress(127,0,0,1));
}

// Helper: check if we have a valid IPv6 address (global or link-local)
bool has_ipv6_address() {
  if(!(cfg.ip_mode & IPV6_DHCP)) {
    return false; // IPv6 not enabled
  }
  IPAddress ipv6_ga = WiFi.globalIPv6();
  IPAddress ipv6_ll = WiFi.linkLocalIPv6();
  D("[IPv6] Global: %s, Link-Local: %s", ipv6_ga.toString().c_str(), ipv6_ll.toString().c_str());
  return (strlen(ipv6_ga.toString().c_str()) > 2 || strlen(ipv6_ll.toString().c_str()) > 2);
}

void connections_tcp_ipv6() {
  if(strlen(cfg.tcp_host_ip) == 0 || cfg.tcp_port == 0) {
    valid_tcp_host = 0;
    D("[TCP] Invalid TCP host IP or port, not setting up TCP");
    return;
  }
  if(!is_ipv6_addr(cfg.tcp_host_ip)) {
    return;
  }
  if(!has_ipv6_address()) {
    valid_tcp_host = 0;
    LOG("[TCP] No IPv6 address available, cannot connect to IPv6 host");
    return;
  }
  // IPv6
  LOG("[TCP] Setting up TCP to: %s, port: %d", cfg.tcp_host_ip, cfg.tcp_port);
  struct sockaddr_in6 sa6;
  memset(&sa6, 0, sizeof(sa6));
  sa6.sin6_family = AF_INET6;
  sa6.sin6_port = htons(cfg.tcp_port);
  if (inet_pton(AF_INET6, cfg.tcp_host_ip, &sa6.sin6_addr) != 1) {
    valid_tcp_host = 0;
    LOG("[TCP] Invalid IPv6 address for TCP: %s", cfg.tcp_host_ip);
    return;
  }
  if(tcp_sock >= 0)
    close_tcp_socket();

  tcp_sock = socket(AF_INET6, SOCK_STREAM, 0);
  if (tcp_sock < 0) {
    valid_tcp_host = 0;
    LOGE("[TCP] Failed to create IPv6 TCP socket");
    return;
  }
  // Set socket to non-blocking mode
  int flags = fcntl(tcp_sock, F_GETFL, 0);
  if (flags >= 0)
    fcntl(tcp_sock, F_SETFL, flags | O_NONBLOCK);
  // connect, this will be non-blocking, so we get a EINPROGRESS
  if (connect(tcp_sock, (struct sockaddr*)&sa6, sizeof(sa6)) == -1) {
    if(errno && errno != EINPROGRESS) {
      // If not EINPROGRESS, connection failed
      LOGE("[TCP] Failed to connect IPv6 TCP socket");
      close_tcp_socket();
      return;
    }
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
    r_bufsize = 8192;
    r_o = setsockopt(tcp_sock, SOL_SOCKET, SO_RCVBUF, &r_bufsize, sizeof(r_bufsize));
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP SO_RCVBUF");
    s_bufsize = 8192;
    r_o = setsockopt(tcp_sock, SOL_SOCKET, SO_SNDBUF, &s_bufsize, sizeof(s_bufsize));
    if (r_o < 0)
      LOGE("[TCP] Failed to set TCP SO_SNDBUF");
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
  }
  valid_tcp_host = 2; // 2 = IPv6
  LOG("[TCP] IPv6 connected to: %s, port: %d, EINPROGRESS, connection in progress", cfg.tcp_host_ip, cfg.tcp_port);
}


void connections_tcp_ipv4() {
  if(strlen(cfg.tcp_host_ip) == 0 || cfg.tcp_port == 0) {
    valid_tcp_host = 0;
    D("[TCP] Invalid host IP or port, not setting up TCP");
    return;
  }
  if(is_ipv6_addr(cfg.tcp_host_ip)) {
    return;
  }
  if(!has_ipv4_address()) {
    valid_tcp_host = 0;
    LOG("[TCP] No IPv4 address available, cannot connect to IPv4 host");
    return;
  }
  // IPv4
  LOG("[TCP] setting up to: %s, port: %d", cfg.tcp_host_ip, cfg.tcp_port);
  IPAddress tcp_tgt;
  if(tcp_tgt.fromString(cfg.tcp_host_ip)) {
    valid_tcp_host = 1;
    LOG("[TCP] Setting up to %s, port:%d", cfg.tcp_host_ip, cfg.tcp_port);
    // WiFiClient will connect on use
  } else {
    valid_tcp_host = 0;
    LOG("[TCP] Invalid host IP or port, not setting up TCP");
  }
}

void close_tcp_socket() {
  int fd_orig = tcp_sock;
  if (tcp_sock >= 0) {
    LOG("[TCP] closing TCP socket %d", fd_orig);
    valid_tcp_host = 0;
    errno = 0;
    if (shutdown(tcp_sock, SHUT_RDWR) == -1) {
        if (errno && errno != ENOTCONN && errno != EBADF && errno != EINVAL)
            LOGE("[TCP] Failed to shutdown %d socket", fd_orig);
    }
    LOG("[TCP] TCP socket %d shutdown", fd_orig);
    errno = 0;
    // now close the socket
    if (close(tcp_sock) == -1)
        if (errno && errno != EBADF && errno != ENOTCONN)
            LOGE("[TCP] Failed to close %d socket", fd_orig);
    tcp_sock = -1;
    LOG("[TCP] TCP socket %d closed", fd_orig);
  }
}

// Helper: send TCP data (IPv4/IPv6)
int send_tcp_data(const uint8_t* data, size_t len) {
  D("[TCP] send_tcp_data len: %d, valid_tcp_host: %d", len, valid_tcp_host);
  if (len == 0 || data == NULL) {
    LOG("[TCP] No data to send");
    return 0; // No data to send
  }
  if (valid_tcp_host == 2 && tcp_sock >= 0) {
    int result = send(tcp_sock, data, len, 0);
    if (result == -1 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINPROGRESS)) {
      // Would block, try again later
      return -1;
    }
    if (result == -1) {
      // Error occurred, close the socket and mark as invalid
      close_tcp_socket();
      return -1;
    }
    if (result == 0) {
      // Connection closed by the remote host
      close_tcp_socket();
      return 0;
    }
    return result;
  } else if (valid_tcp_host == 1) {
    // IPv4 WiFiClient
    if (!tcp_client.connected()) {
      if (!tcp_client.connect(cfg.tcp_host_ip, cfg.tcp_port))
        return -1;
    }
    return tcp_client.write(data, len);
  }
  return -1;
}

// Helper: receive TCP data (IPv4/IPv6)
int recv_tcp_data(uint8_t* buf, size_t maxlen) {
  if (buf == NULL || maxlen == 0) {
    return -1; // Invalid parameters
  }
  
  if (valid_tcp_host == 2 && tcp_sock >= 0) {
    // IPv6 socket (non-blocking)
    int result = recv(tcp_sock, buf, maxlen, 0);
    if (result == -1 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINPROGRESS)) {
      // No data available right now
      return -1;
    }
    if (result == -1) {
      close_tcp_socket(); // Error occurred, close the socket and mark as invalid
      return -1;
    }
    if (result == 0) {
      close_tcp_socket(); // Connection closed by the remote host
      return 0;
    }
    return result;
  } else if (valid_tcp_host == 1) {
    // IPv4 WiFiClient
    if (tcp_client.connected() && tcp_client.available()) {
      return tcp_client.read(buf, maxlen);
    }
  }
  return 0; // Return 0 instead of -1 when no data available
}

// TCP Connection Check: Verify if TCP connection is still alive
void check_tcp_connection(unsigned int tm = 0) {
  if (strlen(cfg.tcp_host_ip) == 0 || cfg.tcp_port == 0) {
    return; // No TCP host configured
  }
  if (WiFi.status() != WL_CONNECTED && WiFi.status() != WL_IDLE_STATUS) {
    return; // No WiFi connection
  }

  // Take a local copy of the socket to avoid race conditions
  int local_tcp_sock = tcp_sock;
  uint8_t local_valid_tcp_host = valid_tcp_host;

  if (local_valid_tcp_host == 2 && local_tcp_sock >= 0) {
    D("[TCP] check_tcp_connection, valid_tcp_host: %d, tcp_sock: %d, tm: %d", local_valid_tcp_host, local_tcp_sock, tm);
    // IPv6 socket: use select() to check if socket is ready for read/write
    fd_set readfds, writefds, errorfds;

    FD_ZERO(&readfds);
    FD_ZERO(&writefds);
    FD_ZERO(&errorfds);
    FD_SET(local_tcp_sock, &readfds);
    FD_SET(local_tcp_sock, &writefds);
    FD_SET(local_tcp_sock, &errorfds);

    // non-blocking select with 0 timeout
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = tm;

    int ready = select(local_tcp_sock + 1, &readfds, &writefds, &errorfds, &timeout);
    if (ready < 0) {
      LOGE("[TCP] select error");
      close_tcp_socket();
      return;
    }

    if (FD_ISSET(local_tcp_sock, &errorfds)) {
      LOG("[TCP] socket %d has error, reconnecting", local_tcp_sock);

      // Check if socket is connected by trying to get socket error
      int socket_error = 0;
      socklen_t len = sizeof(socket_error);
      if (getsockopt(local_tcp_sock, SOL_SOCKET, SO_ERROR, &socket_error, &len) == 0) {
        if (socket_error != 0) {
          LOG("[TCP] socket %d error detected: %d (%s), reconnecting", local_tcp_sock, socket_error, get_errno_string(socket_error));
          close_tcp_socket();
          return;
        }
      } else {
        LOGE("[TCP] getsockopt failed on socket %d", local_tcp_sock);
      }
      close_tcp_socket();
      return;
    }

    #ifdef DEBUG
    if (FD_ISSET(local_tcp_sock, &writefds)) {
      //D("[TCP] socket writable, connection OK");
    }
    #endif

  } else if (local_valid_tcp_host == 1) {
    D("[TCP] check_tcp_connection, valid_tcp_host: %d", local_valid_tcp_host);
    // IPv4 WiFiClient: check if still connected
    if (!tcp_client.connected()) {
      LOG("TCP IPv4 connection lost, reconnecting");
      tcp_client.stop();
      connections_tcp_ipv4(); // Re-validate configuration

      // Try to reconnect
      IPAddress tcp_tgt;
      if (tcp_tgt.fromString(cfg.tcp_host_ip)) {
        if (tcp_client.connect(tcp_tgt, cfg.tcp_port)) {
          LOG("[TCP] IPv4 reconnected successfully");
        } else {
          LOG("[TCP] IPv4 reconnection failed");
          valid_tcp_host = 0;
        }
      }
    } else {
      LOG("[TCP] IPv4 connection OK");
    }
  } else {
    // No valid TCP host or connection needs to be established
    D("[TCP] No valid TCP host, attempting to establish connection");
    if (is_ipv6_addr(cfg.tcp_host_ip) && has_ipv6_address()) {
      connections_tcp_ipv6();
    } else if (!is_ipv6_addr(cfg.tcp_host_ip) && has_ipv4_address()) {
      connections_tcp_ipv4();
    } else {
      D("[TCP] No matching IP version available for target %s", cfg.tcp_host_ip);
    }
  }
  return;
}
#endif // SUPPORT_TCP

#ifndef SUPPORT_UDP
#define SUPPORT_UDP
#endif // SUPPORT_UDP
#ifdef SUPPORT_UDP

void connections_udp_ipv6() {
  if(strlen(cfg.udp_host_ip) == 0 || cfg.udp_port == 0) {
    valid_udp_host = 0;
    LOG("[UDP] Invalid host IP or port, disable");
    return;
  }
  if(!is_ipv6_addr(cfg.udp_host_ip)) {
    return;
  }
  if(!has_ipv6_address()) {
    valid_udp_host = 0;
    LOG("[UDP] No IPv6 address available, cannot connect to IPv6 host");
    return;
  }
  // IPv6
  struct sockaddr_in6 sa6;
  memset(&sa6, 0, sizeof(sa6));
  sa6.sin6_family = AF_INET6;
  sa6.sin6_port = htons(cfg.udp_port);
  if (inet_pton(AF_INET6, cfg.udp_host_ip, &sa6.sin6_addr) != 1) {
    valid_udp_host = 0;
    LOG("[UDP] Invalid IPv6 address:%s", cfg.udp_host_ip);
    return;
  }
  if(udp_sock >= 0) {
    close(udp_sock); // Close any existing socket
    if (errno && errno != EBADF)
      LOGE("[UDP] Failed to close existing socket");
    udp_sock = -1; // Reset socket handle
  }
  udp_sock = socket(AF_INET6, SOCK_DGRAM, 0);
  if (udp_sock < 0) {
    valid_udp_host = 0;
    LOGE("[UDP] Failed to create IPv6 socket");
    return;
  }
  valid_udp_host = 2; // 2 = IPv6
  LOG("[UDP] IPv6 ready to: %s, port: %d", cfg.udp_host_ip, cfg.udp_port);
}

void connections_udp_ipv4() {
  if(strlen(cfg.udp_host_ip) == 0 || cfg.udp_port == 0) {
    valid_udp_host = 0;
    LOG("[UDP] Invalid host IP or port, disable");
    return;
  }
  if(is_ipv6_addr(cfg.udp_host_ip)) {
    return;
  }
  if(!has_ipv4_address()) {
    valid_udp_host = 0;
    LOG("[UDP] No IPv4 address available, cannot connect to IPv4 host");
    return;
  }
  // IPv4
  IPAddress udp_tgt;
  if(udp_tgt.fromString(cfg.udp_host_ip)) {
    valid_udp_host = 1;
    LOG("[UDP] Setting up UDP to %s, port:%d", cfg.udp_host_ip, cfg.udp_port);
    // WiFiUDP will send on use
  } else {
    valid_udp_host = 0;
    LOG("[UDP] Invalid host IP or port, disable");
  }
}
#endif // SUPPORT_UDP

// Helper: send UDP data (IPv4/IPv6)
int send_udp_data(const uint8_t* data, size_t len) {
  if (valid_udp_host == 2 && udp_sock >= 0) {
    // IPv6 socket
    struct sockaddr_in6 sa6;
    memset(&sa6, 0, sizeof(sa6));
    sa6.sin6_family = AF_INET6;
    sa6.sin6_port = htons(cfg.udp_port);
    inet_pton(AF_INET6, cfg.udp_host_ip, &sa6.sin6_addr);
    size_t n = sendto(udp_sock, data, len, 0, (struct sockaddr*)&sa6, sizeof(sa6));
    if (n < 0) {
      LOGE("[UDP] sendto failed to %s:%d", cfg.udp_host_ip, cfg.udp_port);
      close(udp_sock);
      udp_sock = -1;
      valid_udp_host = 0;
      return -1;
    } else if (n == 0) {
      LOG("[UDP] send returned 0 bytes, no data sent");
      return 0; // No data sent
    } else {
      D("[UDP] send_udp_data len: %d, valid_udp_host: %d, sent: %d", len, valid_udp_host, n);
    }
    return n;
  } else if (valid_udp_host == 1) {
    // IPv4 WiFiUDP
    IPAddress udp_tgt;
    udp_tgt.fromString(cfg.udp_host_ip);
    udp.beginPacket(udp_tgt, cfg.udp_port);
    udp.write(data, len);
    return udp.endPacket();
  }
  return -1;
}

// Helper: receive UDP data (IPv4/IPv6)
int recv_udp_data(uint8_t* buf, size_t maxlen) {
  if (valid_udp_host == 2 && udp_sock >= 0) {
    // IPv6 socket
    size_t n = recv(udp_sock, buf, maxlen, 0);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        return 0; // No data available
      } else {
        LOGE("[UDP] recv failed from %s:%d", cfg.udp_host_ip, cfg.udp_port);
        close(udp_sock);
        udp_sock = -1;
        valid_udp_host = 0;
        return -1;
      }
    } else if (n == 0) {
      LOG("[UDP] receive returned 0 bytes, no data received");
      return 0; // No data received
    }
  } else if (valid_udp_host == 1) {
    // IPv4 WiFiUDP
    int psize = udp.parsePacket();
    if (psize > 0) {
      return udp.read(buf, maxlen);
    }
  }
  return -1;
}

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
#define AT_R_OK     (const char*)F("OK")
#define AT_R(M)     (const char*)F(M)
#define AT_R_STR(M) (const char*)String(M).c_str()
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
  } else if(p = at_cmd_check("AT+WIFI_SSID=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 31)
      return AT_R("+ERROR: WiFI SSID max 31 chars");
    if(strlen(p) == 0){
      // Empty SSID, clear it
      memset((char *)&cfg.wifi_ssid, 0, sizeof(cfg.wifi_ssid));
      cfg.wifi_ssid[0] = '\0';
      SAVE();
      reset_networking();
      return AT_R_OK;
    }
    strncpy((char *)&cfg.wifi_ssid, p, sz);
    SAVE();
    reset_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+WIFI_SSID?", atcmdline, cmd_len)){
    if(strlen(cfg.wifi_ssid) == 0)
      return AT_R("+ERROR: WiFi SSID not set");
    else
      return AT_R_STR(cfg.wifi_ssid);
  } else if(p = at_cmd_check("AT+WIFI_PASS=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63)
      return AT_R("+ERROR: WiFi SSID max 63 chars");
    if(strlen(p) == 0){
      // Empty password, clear it
      memset((char *)&cfg.wifi_pass, 0, sizeof(cfg.wifi_pass));
      cfg.wifi_pass[0] = '\0';
      SAVE();
      reset_networking();
      return AT_R_OK;
    }
    strncpy((char *)&cfg.wifi_pass, p, sz);
    SAVE();
    reset_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+WIFI_STATUS?", atcmdline, cmd_len)){
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
      return AT_R_OK;
    } else {
      return AT_R("+ERROR: WPS not running");
    }
  } else if(p = at_cmd_check("AT+WPS_STATUS?", atcmdline, cmd_len)){
    return AT_R(get_wps_status());
  #endif // WIFI_WPS
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
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63)
      return AT_R("+ERROR: NTP hostname max 63 chars");
    if(strlen(p) == 0){
      // Empty hostname, clear it
      memset((char *)&cfg.ntp_host, 0, sizeof(cfg.ntp_host));
      cfg.ntp_host[0] = '\0';
      SAVE();
      return AT_R_OK;
    }
    strncpy((char *)&cfg.ntp_host, p, sz);
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
  #ifdef SUPPORT_UDP
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
    if(strlen(p) >= 15)
      return AT_R("+ERROR: invalid udp host ip (too long)");
    if(strlen(p) == 0){
      // Empty string means disable UDP
      memset(cfg.udp_host_ip, 0, sizeof(cfg.udp_host_ip));
      cfg.udp_host_ip[0] = '\0';
    } else {
      IPAddress tst;
      if(!tst.fromString(p))
        return AT_R("+ERROR: invalid udp host ip");
      // Accept IPv4 or IPv6 string
      strncpy(cfg.udp_host_ip, p, 15-1);
      cfg.udp_host_ip[15-1] = '\0';
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
      return AT_R("+ERROR: invalid tcp host ip (too long)");
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
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63)
      return AT_R("+ERROR: hostname max 63 chars");
    strncpy((char *)&cfg.hostname, p, sz);
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
      cfg.ip_mode |= IPV6_DHCP;
    } else if(params.equalsIgnoreCase("DISABLE")){
      // Disable IPv6
      cfg.ip_mode &= ~IPV6_DHCP;
    } else {
      return AT_R("+ERROR: IPv6 options: DHCP or DISABLE");
    }

    SAVE();
    reset_networking();
    return AT_R_OK;
  } else if(p = at_cmd_check("AT+IPV6?", atcmdline, cmd_len)){
    if(cfg.ip_mode & IPV6_DHCP)
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
    if(cfg.ip_mode & IPV6_DHCP){
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
      if(valid_tcp_host == 1){
        response += "\nTCP IPv4: ";
        if(tcp_client.connected()){
          response += "connected";
        } else {
          response += "disconnected";
        }
      } else if(valid_tcp_host == 2){
        response += "\nTCP IPv6: ";
        if(tcp_sock >= 0){
          response += "connected (socket " + String(tcp_sock) + ")";
        } else {
          response += "disconnected";
        }
      } else {
        response += "\nTCP: not connected";
      }
    }
    return AT_R_STR(response);
  #endif // SUPPORT_TCP
  } else if(p = at_cmd_check("AT+RESET", atcmdline, cmd_len)){
    resetFunc();
  } else if(p = at_cmd_check("AT+HELP?", atcmdline, cmd_len)){
    String help = F("ESP-AT Command Help:\n\n");
    help += F("Basic Commands:\n");
    help += F("  AT                    - Test AT startup\n");
    help += F("  AT?                   - Test AT startup\n");
    help += F("  AT+?                  - Show this help\n");
    help += F("  AT+HELP?              - Show this help\n");
    help += F("  AT+RESET              - Restart device\n\n");

    help += F("WiFi Commands:\n");
    help += F("  AT+WIFI_SSID=<ssid>   - Set WiFi SSID\n");
    help += F("  AT+WIFI_SSID?         - Get WiFi SSID\n");
    help += F("  AT+WIFI_PASS=<pass>   - Set WiFi password\n");
    help += F("  AT+WIFI_STATUS?       - Get WiFi connection status\n");
    help += F("  AT+HOSTNAME=<name>    - Set device hostname\n");
    help += F("  AT+HOSTNAME?          - Get device hostname\n\n");

    help += F("Network Commands:\n");
    help += F("  AT+IPV4=<config>      - Set IPv4 config (DHCP/DISABLE/ip,mask,gw[,dns])\n");
    help += F("  AT+IPV4?              - Get IPv4 configuration\n");
    help += F("  AT+IP_STATUS?         - Get current IP addresses\n\n");

#ifdef SUPPORT_TCP
    help += F("TCP Commands:\n");
    help += F("  AT+TCP_PORT=<port>    - Set TCP port\n");
    help += F("  AT+TCP_PORT?          - Get TCP port\n");
    help += F("  AT+TCP_HOST_IP=<ip>   - Set TCP host IP\n");
    help += F("  AT+TCP_HOST_IP?       - Get TCP host IP\n");
    help += F("  AT+TCP_CHECK_INTERVAL=<ms> - Set TCP check interval\n");
    help += F("  AT+TCP_CHECK_INTERVAL? - Get TCP check interval\n");
    help += F("  AT+TCP_STATUS?        - Get TCP connection status\n\n");
#endif

#ifdef SUPPORT_UDP
    help += F("UDP Commands:\n");
    help += F("  AT+UDP_PORT=<port>    - Set UDP port\n");
    help += F("  AT+UDP_PORT?          - Get UDP port\n");
    help += F("  AT+UDP_HOST_IP=<ip>   - Set UDP host IP\n");
    help += F("  AT+UDP_HOST_IP?       - Get UDP host IP\n\n");
#endif

#ifdef SUPPORT_NTP
    help += F("NTP Commands:\n");
    help += F("  AT+NTP_HOST=<host>    - Set NTP server hostname\n");
    help += F("  AT+NTP_HOST?          - Get NTP server hostname\n");
    help += F("  AT+NTP_STATUS?        - Get NTP sync status\n\n");
#endif

    help += F("System Commands:\n");
    help += F("  AT+LOOP_DELAY=<ms>    - Set main loop delay\n");
    help += F("  AT+LOOP_DELAY?        - Get main loop delay\n");

#ifdef VERBOSE
    help += F("  AT+VERBOSE=<0|1>      - Enable/disable verbose logging\n");
    help += F("  AT+VERBOSE?           - Get verbose logging status\n");
#endif

#ifdef TIMELOG
    help += F("  AT+TIMELOG=<0|1>      - Enable/disable time logging\n");
    help += F("  AT+TIMELOG?           - Get time logging status\n");
#endif

#ifdef LOGUART
    help += F("  AT+LOG_UART=<0|1>     - Enable/disable UART logging\n");
    help += F("  AT+LOG_UART?          - Get UART logging status\n");
#endif

    help += F("\nNote: Commands with '?' are queries, commands with '=' set values");
    return AT_R_STR(help);
  } else if(p = at_cmd_check("AT+?", atcmdline, cmd_len)){
    // Short version of help - just list commands
    String help = F("Available AT Commands:\n");
    help += F("AT, AT?, AT+?, AT+HELP?, AT+RESET\n");
    help += F("AT+WIFI_SSID=|?, AT+WIFI_PASS=, AT+WIFI_STATUS?\n");
    help += F("AT+HOSTNAME=|?, AT+IPV4=|?, AT+IP_STATUS?\n");
    help += F("AT+LOOP_DELAY=|?");

#ifdef SUPPORT_TCP
    help += F(", AT+TCP_PORT=|?, AT+TCP_HOST_IP=|?");
    help += F(", AT+TCP_CHECK_INTERVAL=|?, AT+TCP_STATUS?");
#endif

#ifdef SUPPORT_UDP
    help += F(", AT+UDP_PORT=|?, AT+UDP_HOST_IP=|?");
#endif

#ifdef SUPPORT_NTP
    help += F(", AT+NTP_HOST=|?, AT+NTP_STATUS?");
#endif

#ifdef VERBOSE
    help += F(", AT+VERBOSE=|?");
#endif

#ifdef TIMELOG
    help += F(", AT+TIMELOG=|?");
#endif

#ifdef LOGUART
    help += F(", AT+LOG_UART=|?");
#endif

    help += F("\nUse AT+HELP? for detailed help");
    return AT_R_STR(help);
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
      bleCommandBuffer = "";
      String rxValue = pCharacteristic->getValue().c_str();

      if (rxValue.length() > 0) {
        // Process each byte individually to handle command terminators properly
        for (size_t i = 0; i < rxValue.length(); i++) {
          doYIELD;
          if (rxValue[i] == '\n' || rxValue[i] == '\r') {
            // Command terminator found, mark command as ready if buffer is not empty
            if (bleCommandBuffer.length() > 0)
              bleCommandReady = true;
          } else {
            // Add character to command buffer
            bleCommandBuffer += (char)rxValue[i];
          }

          // Check if command buffer is too long
          if (bleCommandBuffer.length() > 120) {
            // Reset buffer if it's too long without terminator
            bleCommandBuffer = "";
            bleCommandReady = false;
          }
        }
      }
      D("[BLE] Command Ready: %d", bleCommandReady);
      handle_ble_command();
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

  LOG("[BLE] Advertising started, waiting for client connection");
  LOG("[BLE] Advertising will stop after %d seconds if no device connects", BLE_ADVERTISING_TIMEOUT / 1000);
  LOG("[BLE] Once connected, connection will remain until remote disconnects or button is pressed");
}

void handle_ble_command() {
  // Don't handle commands if BLE is disabled
  if (ble_advertising_start == 0)
    return;

  if (bleCommandReady && bleCommandBuffer.length() > 0) {
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
      const char *r = at_cmd_handler(bleCommandBuffer.c_str());
      at_send_response(String(r));
    } else {
      ble_send_response("+ERROR: invalid command");
    }

    bleCommandBuffer = "";
    bleCommandReady = false;
  }
}

void ble_send_response(const String& response) {
  if (ble_advertising_start == 0 || !deviceConnected || !pTxCharacteristic)
    return;

  // Send response with line terminator
  String fr = response + "\r\n";
  ble_send(fr);
}

void ble_send_n(const char& bstr, int len) {
  if (ble_advertising_start == 0)
    return;

  D("[BLE] TX mtu: %d, connected: %d, length: %d >>%s<<", ble_mtu, deviceConnected, len, (const char *)&bstr);
  if (deviceConnected && pTxCharacteristic) {
    // Split response into chunks (BLE characteristic limit), use negotiated MTU
    int o = 0;
    while (o < len) {
      doYIELD;
      int cs = min((int)ble_mtu - 3, len - o); // ATT_MTU-3 for payload
      uint8_t chunk[cs] = {0};
      strncpy((char *)chunk, (const char *)&bstr + o, cs);
      pTxCharacteristic->setValue((uint8_t *)chunk, (size_t)cs);
      pTxCharacteristic->notify();
      o += cs;
      doYIELD;
    }
  }
}

void ble_send(const String& dstr) {
  ble_send_n((const char &)*dstr.c_str(), dstr.length());
}

void start_advertising_ble(){
  LOG("[BLE] Enabling Bluetooth and starting advertising");
  if (pServer)
    pServer->getAdvertising()->start();
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

#if defined(BLUETOOTH_UART_AT) && defined(BT_BLE)
void at_send_response(const String& response) {
  ble_send_response(response);
}
#endif

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
    cfg.do_timelog        = 1;
    #endif
    #ifdef LOGUART
    cfg.do_log            = 1;
    #endif
    #ifdef LOOP_DELAY
    cfg.main_loop_delay   = 100;
    #endif
    strcpy((char *)&cfg.ntp_host, (char *)DEFAULT_NTP_SERVER);
    cfg.ip_mode = IPV4_DHCP | IPV6_DHCP;
    // write config
    SAVE();
    LOG("reinitializing config done");
  }
}

#ifdef WIFI_WPS
/* WPS (WiFi Protected Setup) Functions both PBC and PIN */
bool start_wps(const char *pin) {
  if (wps_running) {
    LOG("[WPS] WPS already running");
    return false;
  }

  LOG("[WPS] Starting WPS Push Button Configuration");

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
#endif // WIFI_WPS

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
          }
          break;
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          LOG("[WiFi] STA got IP: %s", WiFi.localIP().toString().c_str());
          break;
      case ARDUINO_EVENT_WIFI_STA_LOST_IP:
          LOG("[WiFi] STA lost IP");
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
            cfg.ip_mode |= IPV6_DHCP;
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
  LOG("[ESP] MAC Address: %s", WiFi.macAddress().c_str());
  LOG("[ESP] SDK Version: %s", ESP.getSdkVersion());
  LOG("[ESP] Minimum Free Heap: %d bytes", ESP.getMinFreeHeap());
  LOG("[ESP] PSRAM Size: %d bytes", ESP.getPsramSize());
  LOG("[ESP] Free PSRAM: %d bytes", ESP.getFreePsram());
  LOG("[ESP] Minimum Free PSRAM: %d bytes", ESP.getMinFreePsram());
  LOG("[ESP] Uptime: %lu seconds", millis() / 1000);
}

void log_wifi_info(){
  LOG("[WiFi] status: %d: %s", WiFi.status(),
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
    LOGR(", RSSI:%ld", WiFi.RSSI());
    LOGR(", BSSID:%s", WiFi.BSSIDstr().c_str());
    LOGR(", CHANNEL:%d", WiFi.channel());
    LOGR("\n");
    LOGT("[IPV4] ADDR:%s", WiFi.localIP().toString().c_str());
    LOGR(", GW:%s", WiFi.gatewayIP().toString().c_str());
    LOGR(", NM:%s", WiFi.subnetMask().toString().c_str());
    LOGR(", DNS:%s", WiFi.dnsIP().toString().c_str());
    LOGR("\n");
    if(cfg.ip_mode & IPV6_DHCP){
      IPAddress g_ip6 = WiFi.globalIPv6();
      IPAddress l_ip6 = WiFi.linkLocalIPv6();
      LOGT("[IPV6] GA:%s", g_ip6.toString().c_str());
      LOGR(", LL:%s", l_ip6.toString().c_str());
      LOGR("\n");
    }
  }
}

char T_buffer[512] = {""};
char * PT(const char *tformat = "[\%H:\%M:\%S]"){
  time_t t;
  struct tm gm_new_tm;
  time(&t);
  localtime_r(&t, &gm_new_tm);
  strftime(T_buffer, 512, tformat, &gm_new_tm);
  return T_buffer;
}

#ifdef LED

/* LED PWM control */
volatile bool led_state = false;
volatile int last_led_interval = 0;
volatile int8_t last_led_brightness = 0;
volatile int led_interval = 0;
volatile int led_brightness_off = LED_BRIGHTNESS_OFF;
volatile int led_brightness_on  = LED_BRIGHTNESS_LOW;

hw_timer_t *led_t = NULL;
portMUX_TYPE led_timer_mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR ledBlinkTimer() {
  portENTER_CRITICAL_ISR(&led_timer_mux);
  // Note: Don't use ESP_LOG functions in ISR context - they're not ISR-safe
  // Use simple state changes only and no localtime_r/strftime calls
  R("[LED] Timer ISR i:%d\n", led_interval);
  led_state = !led_state;
  if(led_state) {
    led_on();
  } else {
    led_off();
  }
  portEXIT_CRITICAL_ISR(&led_timer_mux);
}

// Helper function to set LED brightness (0-255 on ESP32, digital on/off on ESP8266)
void set_led_brightness(int brightness) {
  R("[LED] Set brightness to %d\n", brightness);
  #if defined(SUPPORT_LED_BRIGHTNESS)
  if (led_pwm_enabled) {
    R("[LED] Using PWM to set brightness\n");
    // Use hardware PWM for smooth brightness control with channel-based API
    if(!ledcWriteChannel(LED_PWM_CHANNEL, brightness)){
      R("[LED] PWM write failed, using digital control\n");
      // PWM failed, fallback to digital control
      digitalWrite(LED, brightness > LED_BRIGHTNESS_LOW ? HIGH : LOW);
    }
  } else {
    R("[LED] PWM not enabled, using digital control\n");
    // PWM failed, fallback to digital control
    digitalWrite(LED, brightness > LED_BRIGHTNESS_LOW ? HIGH : LOW);
  }
  #else
  R("[LED] Using digital control\n");
  // ESP8266 fallback: treat anything above LOW threshold as HIGH
  digitalWrite(LED, brightness > LED_BRIGHTNESS_LOW ? HIGH : LOW);
  #endif
}

void led_on(){
  R("[LED] Turning LED ON\n");
  set_led_brightness(led_brightness_on);
  led_state = true;
}

void led_off(){
  R("[LED] Turning LED OFF\n");
  set_led_brightness(led_brightness_off);
  led_state = false;
}

void setup_led(){
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
  LOG("[LED] Setting up LED blink timer");
  led_t = timerBegin(1000);
  if(led_t == NULL){
    LOG("[LED] Failed to initialize timer for LED");
  } else {
    LOG("[LED] Timer initialized successfully");
    timerAttachInterrupt(led_t, &ledBlinkTimer);
    LOG("[LED] Timer interrupt attached");
    timerAlarm(led_t, LED_BLINK_INTERVAL_NORMAL, true, 0);
    LOG("[LED] Timer alarm set to 1 second");
    timerWrite(led_t, 0);
    timerStart(led_t);
    LOG("[LED] Timer started");
    LOG("[LED] LED setup completed successfully");
  }
  LOG("[LED] LED setup done, starting blink loop");
}
#endif // LED

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
  LOG("[BT] Setting up Bluetooth Classic");
  SerialBT.begin(BLUETOOTH_UART_DEVICE_NAME);
  SerialBT.setPin(BLUETOOTH_UART_DEFAULT_PIN);
  SerialBT.register_callback(BT_EventHandler);
  ATScBT.SetDefaultHandler(&sc_cmd_handler);
  #endif

  // setup WiFi with ssid/pass from EEPROM if set
  reset_networking();

  #ifdef SUPPORT_UART1
  // use UART1
  Serial1.begin(115200, SERIAL_8N1, UART1_RX_PIN, UART1_TX_PIN);
  #endif // SUPPORT_UART1

  #ifdef LED
  // Setup LED with PWM for brightness control
  setup_led();
  #endif // LED

  // button to enable/disable BLE, this will be a toggle
  pinMode(BUTTON, INPUT_PULLUP);

  // log info
  log_esp_info();
}

#define UART1_READ_SIZE       16 // read 16 bytes at a time from UART1
#define UART1_BUFFER_SIZE    512 // max size of UART1 buffer

// from "LOCAL", e.g. "UART1"
char inbuf[UART1_BUFFER_SIZE] = {0};
size_t inlen = 0;
char *inbuf_max = (char *)&inbuf + UART1_BUFFER_SIZE - UART1_READ_SIZE; // max size of inbuf

// from "REMOTE", e.g. TCP, UDP
char outbuf[512] = {0};
size_t outlen = 0;
uint8_t sent_ok = 1;

unsigned long loop_start_millis = 0;

void loop(){
  doYIELD;
  loop_start_millis = millis();

  // Handle button press to enable or disable BLE, toggle style
  // Also track button state for fast LED blinking
  button_pressed = (digitalRead(BUTTON) == LOW);

  static bool button_action_taken = false;
  if (button_pressed && !button_action_taken) {
    LOG("[BUTTON] Pressed, toggling BLE state, currently %s", ble_advertising_start == 0 ? "disabled" : "enabled");
    if (ble_advertising_start == 0) {
      // BLE is currently disabled, start advertising
      start_advertising_ble();
      LOG("[BUTTON] BLE advertising started - will stop on timeout if no connection, or when button pressed again");
    } else {
      // BLE is currently enabled (advertising or connected), stop it
      LOG("[BUTTON] Stopping BLE advertising/connection");
      stop_advertising_ble();
    }
    #ifdef WIFI_WPS
    LOG("[BUTTON] Enable WPS");
    if (!wps_running) {
      start_wps(NULL);
    } else {
      LOG("[BUTTON] WPS already running");
    }
    #endif // WIFI_WPS
    button_action_taken = true;
  } else if (!button_pressed) {
    button_action_taken = false; // Reset when button is released
  }

  doYIELD;

  #ifdef LED
  // Enhanced LED control with new behavior patterns
  unsigned long now = millis();
  bool comm_active = (now - last_tcp_activity < COMM_ACTIVITY_LED_DURATION) ||
                     (now - last_udp_activity < COMM_ACTIVITY_LED_DURATION) ||
                     (now - last_uart1_activity < COMM_ACTIVITY_LED_DURATION);

  bool is_wifi_connected = (WiFi.status() == WL_CONNECTED);
  bool is_ble_advertising = (ble_advertising_start != 0);
  bool is_ble_connected = (deviceConnected);

  // Determine LED behavior based on priority (highest to lowest):
  if (comm_active) {
    // Data transmission: tiny flicker on top of current state
    led_interval = LED_BLINK_INTERVAL_FLICKER;
    if (is_wifi_connected) {
      led_brightness_on = LED_BRIGHTNESS_MEDIUM + LED_BRIGHTNESS_FLICKER; // Flicker on top of steady
      led_brightness_off = LED_BRIGHTNESS_MEDIUM; // Return to steady connected state
    } else {
      led_brightness_on = LED_BRIGHTNESS_LOW + LED_BRIGHTNESS_FLICKER;
      led_brightness_off = LED_BRIGHTNESS_LOW;
    }
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
  } else if (wps_running) {
    // WPS active: slow blink
    led_interval = LED_BLINK_INTERVAL_QUICK;
    led_brightness_on = LED_BRIGHTNESS_HIGH;
    led_brightness_off = LED_BRIGHTNESS_DIM;
  } else if (is_wifi_connected) {
    // WiFi connected: full on at medium brightness (not too bright)
    led_interval = 0; // No blinking, steady on
    led_brightness_on = LED_BRIGHTNESS_MEDIUM;
    led_brightness_off = LED_BRIGHTNESS_MEDIUM; // Same as on = steady
  } else {
    // Not connected: slowly blinking
    led_interval = LED_BLINK_INTERVAL_HALF;
    led_brightness_on = LED_BRIGHTNESS_LOW;
    led_brightness_off = LED_BRIGHTNESS_OFF;
  }
  if(led_interval != last_led_interval){
    D("[LED] New interval, last:%d, i: %d ms, on: %d, off: %d", last_led_interval, led_interval, led_brightness_on, led_brightness_off);
    last_led_interval = led_interval;
    timerStop(led_t);
    timerAlarm(led_t, led_interval, true, 0);
    D("[LED] Timer alarm set to %d ms", led_interval);
    timerWrite(led_t, 0);
    timerStart(led_t);
  }
  #endif // LED

  #ifdef WIFI_WPS
  // Check WPS timeout
  if (wps_running && (millis() - wps_start_time > WPS_TIMEOUT_MS)) {
    LOG("[WPS] WPS timeout reached, stopping WPS");
    stop_wps();
  }
  #endif // WIFI_WPS

  // Handle Serial AT commands
  #ifdef UART_AT
  while(ATSc.GetSerial()->available() > 0)
    ATSc.ReadSerial();
  doYIELD;
  #endif

  #if defined(BT_BLE)
  // Check if BLE advertising should be stopped after timeout
  // Only stop on timeout if no device is connected - once connected, wait for remote disconnect or button press
  if (ble_advertising_start != 0 && !deviceConnected && millis() - ble_advertising_start > BLE_ADVERTISING_TIMEOUT)
    stop_advertising_ble();
  doYIELD;
  #endif

  #ifdef TIMELOG
  if(cfg.do_timelog && (last_time_log == 0 || millis() - last_time_log > 500)){
    #if defined(BT_BLE)
    if(ble_advertising_start != 0)
      ble_send(PT("🍓 [%H:%M:%S]:📡 ⟹  🖫 & 💾\n"));
    #endif
    #ifdef LOGUART
    if(cfg.do_log)
      LOG("%s", PT("[%H:%M:%S]: OK\n"));
    #endif
    last_time_log = millis();
  }
  #endif

  #ifdef SUPPORT_UART1
  // Read all available bytes from UART, but only for as much data as fits in
  // inbuf, read per 16 chars to be sure we don't overflow
  size_t to_r = 0;
  while((to_r = Serial1.available()) > 0 && inbuf + inlen < inbuf_max) {
    doYIELD;
    // read 16 bytes into inbuf
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
  doYIELD;
  if (valid_udp_host && inlen > 0) {
    int sent = send_udp_data((const uint8_t*)inbuf, inlen);
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
  #endif

  #ifdef SUPPORT_TCP
  doYIELD;
  if (valid_tcp_host && inlen > 0) {
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
  #endif

  // TCP read
  #ifdef SUPPORT_TCP
  doYIELD;
  if (valid_tcp_host) {
    if (outlen + 16 >= sizeof(outbuf)) {
      D("[TCP] outbuf full, cannot read more data");
      // no space in outbuf, cannot read more data
      // just yield and wait for outbuf to be cleared
      doYIELD;
    } else {
      // no select(), just read from TCP socket and ignore ENOTCONN etc..
      int os = recv_tcp_data((uint8_t*)outbuf + outlen, 16);
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
          //E("[TCP] no data available", errno);
        }
      }
    }
  }
  #endif

  // UDP read
  #ifdef SUPPORT_UDP
  doYIELD;
  if (valid_udp_host) {
    if (outlen + 16 >= sizeof(outbuf)) {
      D("[UDP] outbuf full, cannot read more data");
      // no space in outbuf, cannot read more data
      // just yield and wait for outbuf to be cleared
      doYIELD;
    } else {
      int os = recv_udp_data((uint8_t*)outbuf + outlen, 16);
      if (os > 0) {
        #ifdef LED
        last_udp_activity = millis(); // Trigger LED activity for UDP receive
        #endif // LED
        D("[UDP] Received %d bytes, total: %d, data: >>%s<<", os, outlen + os, outbuf);
        outlen += os;
      } else if (os < 0) {
        if(errno && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINPROGRESS){
          // Error occurred, log it
          LOGE("[UDP] receive error, closing connection");
        } else {
          // No data available, just yield
          D("[UDP] no data available, yielding...");
        }
      }
    }
  }
  #endif // SUPPORT_UDP

  // just wifi check
  doYIELD;
  if(millis() - last_wifi_check > 500){
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
        reset_networking();
      }
    } else {
      // connected
      last_wifi_reconnect = millis();
    }
  }

  // Log ESP info periodically when DEBUG is enabled
  #ifdef DEBUG
  if(last_esp_info_log ==0 || millis() - last_esp_info_log > 30000) { // Log every 30 seconds
    log_esp_info();
    last_esp_info_log = millis();
  }
  #endif

  // TCP connection check at configured interval
  #ifdef SUPPORT_TCP
  doYIELD;
  check_tcp_connection(100000);
  #endif

  // NTP check
  #ifdef SUPPORT_NTP
  doYIELD;
  if(last_ntp_log == 0 || millis() - last_ntp_log > 10000){
    last_ntp_log = millis();
    if((WiFi.status() == WL_CONNECTED || WiFi.status() == WL_IDLE_STATUS) && cfg.ntp_host[0] != 0 && esp_sntp_enabled()){
      // check if synced
      if(sntp_get_sync_status() == SNTP_SYNC_STATUS_COMPLETED){
        // synced
        static int last_hour = -1;
        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);
        if(timeinfo.tm_hour != last_hour){
          last_hour = timeinfo.tm_hour;
          LOG("NTP synced: %s", PT());
        }
      } else {
        LOG("NTP not yet synced");
      }
    }
  }
  #endif // SUPPORT_NTP

  // copy over the inbuf to outbuf for logging if data received
  doYIELD;
  if(outlen > 0){
    // send outbuf to Serial1 if data received, in chunks of 16 bytes
    #ifdef SUPPORT_UART1
    uint8_t *o = (uint8_t *)&outbuf;
    uint8_t *m = (uint8_t *)&outbuf + outlen;
    size_t w = 0;
    while(o < m && (w = Serial1.availableForWrite()) > 0){
      doYIELD;
      w = min((size_t)w, (size_t)16);
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
  }

  doYIELD;

  // clear outbuf
  memset(outbuf, 0, sizeof(outbuf));

  // assume the inbuf is sent
  if(inlen && sent_ok){
    inlen = 0;
    sent_ok = 1;
    memset(inbuf, 0, sizeof(inbuf));
  }

  // DELAY sleep, we need to pick the lowest amount of delay to not block too
  // long, default to cfg.main_loop_delay if not needed
  #ifdef LOOP_DELAY
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
}
