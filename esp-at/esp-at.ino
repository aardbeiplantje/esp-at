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
#endif
#ifdef ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>
#endif
#include <errno.h>
#include <fcntl.h>
#include "time.h"
#include "SerialCommands.h"
#include "EEPROM.h"
#include "esp_sntp.h"

#define LED           2

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

#if defined(DEBUG) || defined(VERBOSE)
void print_time_to_serial(const char *tformat = "[\%H:\%M:\%S]: "){
  time_t t;
  struct tm gm_new_tm;
  time(&t);
  localtime_r(&t, &gm_new_tm);
  char d_outstr[20];
  strftime(d_outstr, 20, tformat, &gm_new_tm);
  Serial.print(d_outstr);
}
#endif

#ifdef VERBOSE
 #define LOG_TIME_FORMAT "[\%H:\%M:\%S]: "
 #if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  #define DOLOG(L)    if(cfg.do_verbose){Serial.print(L);}
  #define DOLOGLN(L)  if(cfg.do_verbose){Serial.println(L);}
  #define DOLOGT()    print_time_to_serial(LOG_TIME_FORMAT);
 #else
  #define DOLOG(L)    if(cfg.do_verbose){Serial.print(L);}
  #define DOLOGLN(L)  if(cfg.do_verbose){Serial.println(L);}
  #define DOLOGT()    print_time_to_serial(LOG_TIME_FORMAT);
 #endif
#else
 #define DOLOG(L)
 #define DOLOGLN(L)
 #define DOLOGT()
#endif

#ifdef DEBUG
 #define DEBUG_TIME_FORMAT "[\%H:\%M:\%S]: "
 #if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32) || defined(ESP32)
  #define DODEBUG(L)    Serial.print(L);
  #define DODEBUGLN(L)  Serial.println(L);
  #define DODEBUGT()    print_time_to_serial(DEBUG_TIME_FORMAT);
 #else
  #define DODEBUG(L)    Serial.print(L);
  #define DODEBUGLN(L)  Serial.println(L);
  #define DODEBUGT()    print_time_to_serial(DEBUG_TIME_FORMAT);
 #endif
#else
 #define DODEBUG(L)
 #define DODEBUGLN(L)
 #define DODEBUGT()
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
  uint16_t main_loop_delay = 100;
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
uint8_t ntp_is_synced = 1;
void cb_ntp_synced(struct timeval *tv){
  doYIELD;
  time_t t;
  struct tm gm_new_tm;
  time(&t);
  localtime_r(&t, &gm_new_tm);
  DOLOGT();
  DOLOG(F("NTP synced, new time: "));
  DOLOG(t);
  char d_outstr[100];
  strftime(d_outstr, 100, ", sync: %a, %b %d %Y %H:%M:%S%z %Z (%s)", &gm_new_tm);
  DOLOGLN(d_outstr);
  ntp_is_synced = 1;
}

void setup_ntp(){
  // if we have a NTP host configured, sync
  if(strlen(cfg.ntp_host)){
    DOLOGT();
    DOLOG(F("will sync with ntp: "));
    DOLOG(cfg.ntp_host);
    DOLOG(F(", interval: "));
    DOLOG(4 * 3600);
    DOLOG(F(", timezone: "));
    DOLOGLN("UTC");
    if(esp_sntp_enabled()){
      DOLOGT(); DOLOGLN(F("NTP already enabled, skipping setup"));
      sntp_set_sync_interval(4 * 3600 * 1000UL);
      sntp_setservername(0, (char*)&cfg.ntp_host);
    } else {
      DOLOGT(); DOLOGLN(F("Setting up NTP sync"));
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
uint8_t logged_wifi_status = 0;
unsigned long last_wifi_check = 0;

#ifdef TIMELOG
unsigned long last_time_log = 0;
#endif // TIMELOG

void setup_wifi(){
  DOLOGLN(F("WiFi setup"));
  DOLOG(F("WiFi SSID: "));
  DOLOGLN(cfg.wifi_ssid);
  DOLOG(F("WiFi Pass: "));
  if(strlen(cfg.wifi_pass) == 0)
    DOLOGLN(F("none"))
  else
    // print password as stars, even fake the length
    DOLOGLN(F("***********"));
  // are we connecting to WiFi?
  if(strlen(cfg.wifi_ssid) == 0 || strlen(cfg.wifi_pass) == 0)
    return;
  if(WiFi.status() == WL_CONNECTED)
    return;
  logged_wifi_status = 0; // reset logged status

  WiFi.disconnect(); // disconnect from any previous connection
  DOLOGLN(F("WiFi setup"));
  WiFi.onEvent(WiFiEvent);

  // IPv4 configuration
  if(cfg.ip_mode & IPV4_DHCP){
    DOLOGLN(F("WiFi Using DHCP for IPv4"));
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  } else if(cfg.ip_mode & IPV4_STATIC){
    DOLOGLN(F("WiFi Using static IPv4 configuration"));
    WiFi.config(IPAddress(cfg.ipv4_addr[0], cfg.ipv4_addr[1], cfg.ipv4_addr[2], cfg.ipv4_addr[3]),
                IPAddress(cfg.ipv4_gw[0], cfg.ipv4_gw[1], cfg.ipv4_gw[2], cfg.ipv4_gw[3]),
                IPAddress(cfg.ipv4_mask[0], cfg.ipv4_mask[1], cfg.ipv4_mask[2], cfg.ipv4_mask[3]),
                IPAddress(cfg.ipv4_dns[0], cfg.ipv4_dns[1], cfg.ipv4_dns[2], cfg.ipv4_dns[3]));
  } else {
    DOLOGLN(F("WiFi Using no IPv4 configuration, assume loopback address"));
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
  WiFi.setTxPower(WIFI_POWER_19_5dBm);

  // IPv6 configuration
  if(cfg.ip_mode & IPV6_DHCP){
    DOLOGLN(F("WiFi Using DHCP for IPv6"));
    WiFi.enableIPv6(true);
  }

  // connect to Wi-Fi
  DOLOG(F("WiFi Connecting to "));
  DOLOGLN(cfg.wifi_ssid);
  WiFi.persistent(false);
  WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);

  // setup NTP sync if needed
  #ifdef SUPPORT_NTP
  setup_ntp();
  #endif
}


#ifndef SUPPORT_TCP
#define SUPPORT_TCP
#endif
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

void connections_tcp_ipv6() {
  if(strlen(cfg.tcp_host_ip) == 0 || cfg.tcp_port == 0) {
    valid_tcp_host = 0;
    DOLOGLN(F("Invalid TCP host IP or port, not setting up TCP"));
    return;
  }
  if(!is_ipv6_addr(cfg.udp_host_ip)) {
    return;
  }
  // IPv6
  struct sockaddr_in6 sa6;
  memset(&sa6, 0, sizeof(sa6));
  sa6.sin6_family = AF_INET6;
  sa6.sin6_port = htons(cfg.tcp_port);
  if (inet_pton(AF_INET6, cfg.tcp_host_ip, &sa6.sin6_addr) != 1) {
    valid_tcp_host = 0;
    DOLOG(F("Invalid IPv6 address for TCP: "));
    DOLOGLN(cfg.tcp_host_ip);
    return;
  }
  tcp_sock = socket(AF_INET6, SOCK_STREAM, 0);
  if (tcp_sock < 0) {
    valid_tcp_host = 0;
    DOLOG(F("Failed to create IPv6 TCP socket, errno: "));
    DOLOGLN(errno);
    return;
  }
  if (connect(tcp_sock, (struct sockaddr*)&sa6, sizeof(sa6)) < 0) {
    valid_tcp_host = 0;
    DOLOG(F("Failed to connect IPv6 TCP socket"));
    DOLOG(F(" to: "));
    DOLOG(cfg.tcp_host_ip);
    DOLOG(F(", port: "));
    DOLOG(cfg.tcp_port);
    DOLOG(F(", errno: "));
    DOLOGLN(errno);
    close(tcp_sock);
    tcp_sock = -1;
    return;
  }
  // Set socket to non-blocking mode
  int flags = fcntl(tcp_sock, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(tcp_sock, F_SETFL, flags | O_NONBLOCK);
  }
  valid_tcp_host = 2; // 2 = IPv6
  DOLOG(F("TCP IPv6 connected to: "));
  DOLOG(cfg.tcp_host_ip);
  DOLOG(F(", port: "));
  DOLOGLN(cfg.tcp_port);
}

void connections_tcp_ipv4() {
  if(strlen(cfg.tcp_host_ip) == 0 || cfg.tcp_port == 0) {
    valid_tcp_host = 0;
    DOLOGLN(F("Invalid TCP host IP or port, not setting up TCP"));
    return;
  }
  if(is_ipv6_addr(cfg.udp_host_ip)) {
    return;
  }
  // IPv4
  IPAddress tcp_tgt;
  if(tcp_tgt.fromString(cfg.tcp_host_ip)) {
    valid_tcp_host = 1;
    DOLOG(F("Setting up TCP to "));
    DOLOG(cfg.tcp_host_ip);
    DOLOG(F(", port:"));
    DOLOGLN(cfg.tcp_port);
    // WiFiClient will connect on use
  } else {
    valid_tcp_host = 0;
    DOLOGLN(F("Invalid TCP host IP or port, not setting up TCP"));
  }
}


// Helper: send TCP data (IPv4/IPv6)
int send_tcp_data(const uint8_t* data, size_t len) {
  if (valid_tcp_host == 2 && tcp_sock >= 0) {
    // IPv6 socket
    return send(tcp_sock, data, len, 0);
  } else if (valid_tcp_host == 1) {
    // IPv4 WiFiClient
    if (!tcp_client.connected()) {
      if (!tcp_client.connect(cfg.tcp_host_ip, cfg.tcp_port)) return -1;
    }
    return tcp_client.write(data, len);
  }
  return -1;
}

// Helper: receive TCP data (IPv4/IPv6)
int recv_tcp_data(uint8_t* buf, size_t maxlen) {
  if (valid_tcp_host == 2 && tcp_sock >= 0) {
    // IPv6 socket (non-blocking)
    int result = recv(tcp_sock, buf, maxlen, 0);
    if (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      return 0; // No data available
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
#endif // SUPPORT_TCP

#ifndef SUPPORT_UDP
#define SUPPORT_UDP
#endif // SUPPORT_UDP
#ifdef SUPPORT_UDP

void connections_udp_ipv6() {
  if(strlen(cfg.udp_host_ip) == 0 || cfg.udp_port == 0) {
    valid_udp_host = 0;
    DOLOGLN(F("Invalid UDP host IP or port, not setting up UDP"));
    return;
  }
  if(!is_ipv6_addr(cfg.udp_host_ip)) {
    return;
  }
  // IPv6
  struct sockaddr_in6 sa6;
  memset(&sa6, 0, sizeof(sa6));
  sa6.sin6_family = AF_INET6;
  sa6.sin6_port = htons(cfg.udp_port);
  if (inet_pton(AF_INET6, cfg.udp_host_ip, &sa6.sin6_addr) != 1) {
    valid_udp_host = 0;
    DOLOG(F("Invalid IPv6 address for UDP:"));
    DOLOGLN(cfg.udp_host_ip);
    return;
  }
  udp_sock = socket(AF_INET6, SOCK_DGRAM, 0);
  if (udp_sock < 0) {
    valid_udp_host = 0;
    DOLOG(F("Failed to create IPv6 UDP socket"));
    DOLOG(F(", errno: "));
    DOLOGLN(errno);
    return;
  }
  valid_udp_host = 2; // 2 = IPv6
  DOLOG(F("UDP IPv6 ready to: "));
  DOLOG(cfg.udp_host_ip);
  DOLOG(F(", port: "));
  DOLOGLN(cfg.udp_port);
}

void connections_udp_ipv4() {
  if(strlen(cfg.udp_host_ip) == 0 || cfg.udp_port == 0) {
    valid_udp_host = 0;
    DOLOGLN(F("Invalid UDP host IP or port, not setting up UDP"));
    return;
  }
  if(is_ipv6_addr(cfg.udp_host_ip)) {
    return;
  }
  // IPv4
  IPAddress udp_tgt;
  if(udp_tgt.fromString(cfg.udp_host_ip)) {
    valid_udp_host = 1;
    DOLOG(F("Setting up UDP to "));
    DOLOG(cfg.udp_host_ip);
    DOLOG(F(", port:"));
    DOLOGLN(cfg.udp_port);
    // WiFiUDP will send on use
  } else {
    valid_udp_host = 0;
    DOLOGLN(F("Invalid UDP host IP or port, not setting up UDP"));
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
    return sendto(udp_sock, data, len, 0, (struct sockaddr*)&sa6, sizeof(sa6));
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
    return recv(udp_sock, buf, maxlen, 0);
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

void at_cmd_handler(SerialCommands* s, const char* atcmdline){
  unsigned int cmd_len = strlen(atcmdline);
  char *p = NULL;
  DODEBUGT();
  DODEBUG(F("AT: ["));
  DODEBUG(atcmdline);
  DODEBUG(F("], size: "));
  DODEBUGLN(cmd_len);
  if(cmd_len == 2 && (p = at_cmd_check("AT", atcmdline, cmd_len))){
    at_send_response(s, F("OK"));
    return;
  } else if(cmd_len == 3 && (p = at_cmd_check("AT?", atcmdline, cmd_len))){
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+WIFI_SSID=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 31){
      at_send_response(s, F("+ERROR: WiFI SSID max 31 chars"));
      return;
    }
    strncpy((char *)&cfg.wifi_ssid, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    WiFi.disconnect();
    setup_wifi();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+WIFI_SSID?", atcmdline, cmd_len)){
    if(strlen(cfg.wifi_ssid) == 0){
      at_send_response(s, F("+ERROR: WiFi SSID not set"));
    } else {
      at_send_response(s, String(cfg.wifi_ssid));
    }
    return;
  } else if(p = at_cmd_check("AT+WIFI_PASS=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63){
      at_send_response(s, F("+ERROR: WiFi PASS max 63 chars"));
      return;
    }
    strncpy((char *)&cfg.wifi_pass, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    WiFi.disconnect();
    setup_wifi();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+WIFI_STATUS?", atcmdline, cmd_len)){
    uint8_t wifi_stat = WiFi.status();
    String response;
    switch(wifi_stat) {
        case WL_CONNECTED:
          response = "connected";
          break;
        case WL_CONNECT_FAILED:
          response = "failed";
          break;
        case WL_CONNECTION_LOST:
          response = "connection lost";
          break;
        case WL_DISCONNECTED:
          response = "disconnected";
          break;
        case WL_IDLE_STATUS:
          response = "idle";
          break;
        case WL_NO_SSID_AVAIL:
          response = "no SSID configured";
          break;
        default:
          response = String(wifi_stat);
    }
    at_send_response(s, response);
    return;
  #ifdef TIMELOG
  } else if(p = at_cmd_check("AT+TIMELOG=1", atcmdline, cmd_len)){
    cfg.do_timelog = 1;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+TIMELOG=0", atcmdline, cmd_len)){
    cfg.do_timelog = 0;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+TIMELOG?", atcmdline, cmd_len)){
    at_send_response(s, String(cfg.do_timelog));
    return;
  #endif
  #ifdef VERBOSE
  } else if(p = at_cmd_check("AT+VERBOSE=1", atcmdline, cmd_len)){
    cfg.do_verbose = 1;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+VERBOSE=0", atcmdline, cmd_len)){
    cfg.do_verbose = 0;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+VERBOSE?", atcmdline, cmd_len)){
    at_send_response(s, String(cfg.do_verbose));
    return;
  #endif
  #ifdef LOGUART
  } else if(p = at_cmd_check("AT+LOG_UART=1", atcmdline, cmd_len)){
    cfg.do_log = 1;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+LOG_UART=0", atcmdline, cmd_len)){
    cfg.do_log = 0;
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+LOG_UART?", atcmdline, cmd_len)){
    at_send_response(s, String(cfg.do_log));
    return;
  #endif
  #ifdef SUPPORT_NTP
  } else if(p = at_cmd_check("AT+NTP_HOST=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63){
      at_send_response(s, F("+ERROR: NTP hostname max 63 chars"));
      return;
    }
    strncpy((char *)&cfg.ntp_host, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    setup_wifi();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+NTP_HOST?", atcmdline, cmd_len)){
    if(strlen(cfg.ntp_host) == 0){
      at_send_response(s, F("+ERROR: NTP hostname not set"));
      return;
    }
    at_send_response(s, String(cfg.ntp_host));
    return;
  } else if(p = at_cmd_check("AT+NTP_STATUS?", atcmdline, cmd_len)){
    String response;
    if(ntp_is_synced)
      response = "ntp synced";
    else
      response = "not ntp synced";
    at_send_response(s, response);
    return;
  #endif // SUPPORT_NTP
  #ifdef SUPPORT_UDP
  } else if(p = at_cmd_check("AT+UDP_PORT?", atcmdline, cmd_len)){
    at_send_response(s, String(cfg.udp_port));
    return;
  } else if(p = at_cmd_check("AT+UDP_PORT=", atcmdline, cmd_len)){
    uint16_t new_udp_port = (uint16_t)strtol(p, NULL, 10);
    if(new_udp_port == 0){
      at_send_response(s, F("+ERROR: invalid UDP port"));
      return;
    }
    if(new_udp_port != cfg.udp_port){
      cfg.udp_port = new_udp_port;
      EEPROM.put(CFG_EEPROM, cfg);
      EEPROM.commit();
    }
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+UDP_HOST_IP?", atcmdline, cmd_len)){
    at_send_response(s, String(cfg.udp_host_ip));
    return;
  } else if(p = at_cmd_check("AT+UDP_HOST_IP=", atcmdline, cmd_len)){
    if(strlen(p) >= 15){
      at_send_response(s, F("+ERROR: invalid udp host ip (too long)"));
      return;
    }
    IPAddress tst;
    if(!tst.fromString(p)){
      at_send_response(s, F("+ERROR: invalid udp host ip"));
      return;
    }
    // Accept IPv4 or IPv6 string
    strncpy(cfg.udp_host_ip, p, 15-1);
    cfg.udp_host_ip[15-1] = '\0';
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    at_send_response(s, F("OK"));
    return;
  #endif // SUPPORT_UDP
  #ifdef SUPPORT_TCP
  } else if(p = at_cmd_check("AT+TCP_PORT?", atcmdline, cmd_len)){
    at_send_response(s, String(cfg.tcp_port));
    return;
  } else if(p = at_cmd_check("AT+TCP_PORT=", atcmdline, cmd_len)){
    uint16_t new_tcp_port = (uint16_t)strtol(p, NULL, 10);
    if(new_tcp_port == 0){
      at_send_response(s, F("+ERROR: invalid TCP port"));
      return;
    }
    if(new_tcp_port != cfg.tcp_port){
      cfg.tcp_port = new_tcp_port;
      EEPROM.put(CFG_EEPROM, cfg);
      EEPROM.commit();
    }
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+TCP_HOST_IP?", atcmdline, cmd_len)){
    at_send_response(s, String(cfg.tcp_host_ip));
    return;
  } else if(p = at_cmd_check("AT+TCP_HOST_IP=", atcmdline, cmd_len)){
    if(strlen(p) >= 40){
      at_send_response(s, F("+ERROR: invalid tcp host ip (too long)"));
      return;
    }
    IPAddress tst;
    if(!tst.fromString(p)){
      at_send_response(s, F("+ERROR: invalid tcp host ip"));
      return;
    }
    // Accept IPv4 or IPv6 string
    strncpy(cfg.tcp_host_ip, p, 40-1);
    cfg.tcp_host_ip[40-1] = '\0';
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    at_send_response(s, F("OK"));
    return;
  #endif // SUPPORT_TCP
  } else if(p = at_cmd_check("AT+LOOP_DELAY=", atcmdline, cmd_len)){
    errno = 0;
    unsigned int new_c = strtoul(p, NULL, 10);
    if(errno != 0){
      at_send_response(s, F("+ERROR: invalid number"));
      return;
    }
    if(new_c != cfg.main_loop_delay){
      cfg.main_loop_delay = new_c;
      EEPROM.put(CFG_EEPROM, cfg);
      EEPROM.commit();
    }
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+LOOP_DELAY?", atcmdline, cmd_len)){
    at_send_response(s, String(cfg.main_loop_delay));
    return;
  } else if(p = at_cmd_check("AT+HOSTNAME=", atcmdline, cmd_len)){
    size_t sz = (atcmdline+cmd_len)-p+1;
    if(sz > 63){
      at_send_response(s, F("+ERROR: hostname max 63 chars"));
      return;
    }
    strncpy((char *)&cfg.hostname, p, sz);
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    // Apply hostname immediately if WiFi is connected
    if(WiFi.status() == WL_CONNECTED){
      WiFi.setHostname(cfg.hostname);
    }
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+HOSTNAME?", atcmdline, cmd_len)){
    if(strlen(cfg.hostname) == 0){
      at_send_response(s, F(DEFAULT_HOSTNAME)); // default hostname
    } else {
      at_send_response(s, String(cfg.hostname));
    }
    return;
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

      if(commaPos1 == -1 || commaPos2 == -1){
        at_send_response(s, F("+ERROR: IPv4 options: DHCP, DISABLE, or ip,netmask,gateway[,dns]"));
        return;
      }

      String ip = params.substring(0, commaPos1);
      String netmask = params.substring(commaPos1 + 1, commaPos2);
      String gateway = params.substring(commaPos2 + 1, commaPos3 == -1 ? params.length() : commaPos3);
      String dns = commaPos3 == -1 ? DEFAULT_DNS_IPV4 : params.substring(commaPos3 + 1);

      // Parse IP addresses
      if(!ip.length() || !netmask.length() || !gateway.length()){
        at_send_response(s, F("+ERROR: IPv4 format: ip,netmask,gateway[,dns]"));
        return;
      }

      // Parse and validate IP address
      int ip_parts[4], mask_parts[4], gw_parts[4], dns_parts[4];
      if(sscanf(ip.c_str(), "%d.%d.%d.%d", &ip_parts[0], &ip_parts[1], &ip_parts[2], &ip_parts[3]) != 4 ||
         sscanf(netmask.c_str(), "%d.%d.%d.%d", &mask_parts[0], &mask_parts[1], &mask_parts[2], &mask_parts[3]) != 4 ||
         sscanf(gateway.c_str(), "%d.%d.%d.%d", &gw_parts[0], &gw_parts[1], &gw_parts[2], &gw_parts[3]) != 4 ||
         sscanf(dns.c_str(), "%d.%d.%d.%d", &dns_parts[0], &dns_parts[1], &dns_parts[2], &dns_parts[3]) != 4){
        at_send_response(s, F("+ERROR: invalid IP address format"));
        return;
      }

      // Validate IP ranges (0-255)
      for(int i = 0; i < 4; i++){
        if(ip_parts[i] < 0 || ip_parts[i] > 255 || mask_parts[i] < 0 || mask_parts[i] > 255 ||
           gw_parts[i] < 0 || gw_parts[i] > 255 || dns_parts[i] < 0 || dns_parts[i] > 255){
          at_send_response(s, F("+ERROR: IP address parts must be 0-255"));
          return;
        }
        cfg.ipv4_addr[i] = ip_parts[i];
        cfg.ipv4_mask[i] = mask_parts[i];
        cfg.ipv4_gw[i] = gw_parts[i];
        cfg.ipv4_dns[i] = dns_parts[i];
      }

      // Enable static IPv4
      cfg.ip_mode = (cfg.ip_mode & ~IPV4_DHCP) | IPV4_STATIC;
    }

    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    WiFi.disconnect();
    setup_wifi();
    at_send_response(s, F("OK"));
    return;
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
    at_send_response(s, response);
    return;
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
      at_send_response(s, F("+ERROR: IPv6 options: DHCP, DISABLE"));
      return;
    }

    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    WiFi.disconnect();
    setup_wifi();
    at_send_response(s, F("OK"));
    return;
  } else if(p = at_cmd_check("AT+IPV6?", atcmdline, cmd_len)){
    String response;
    if(cfg.ip_mode & IPV6_DHCP){
      response = "DHCP";
    } else {
      response = "DISABLED";
    }
    at_send_response(s, response);
    return;
  } else if(p = at_cmd_check("AT+IP_STATUS?", atcmdline, cmd_len)){
    String response = "";
    bool hasIP = false;

    // Check WiFi connection status first
    if(WiFi.status() != WL_CONNECTED){
      at_send_response(s, F("+ERROR: WiFi not connected"));
      return;
    }

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

    if(!hasIP){
      response = "No IP addresses assigned";
    }

    at_send_response(s, response);
    return;
  } else if(p = at_cmd_check("AT+RESET", atcmdline, cmd_len)){
    at_send_response(s, F("OK"));
    resetFunc();
    return;
  } else {
    at_send_response(s, F("+ERROR: unknown command"));
    return;
  }
}

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
bool oldDeviceConnected = false;

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
      DODEBUGT();
      DODEBUGLN(F("BLE client connected"));
    };

    void onDisconnect(BLEServer* pServer) {
      doYIELD;
      deviceConnected = false;
      DODEBUGT();
      DODEBUGLN(F("BLE client disconnected"));
    }

    // TODO: use/fix once ESP32 BLE MTU negotiation is implemented
    #if defined(ESP32) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
    void onMTU(uint16_t mtu, BLEServer* /*pServer*/) {
      if (mtu < BLE_MTU_MIN) {
        DOLOGT(); DOLOG(F("BLE MTU request too small (")); DOLOG(mtu); DOLOG(F("), keeping ")); DOLOGLN(BLE_MTU_DEFAULT);
        return;
      }
      if (mtu > BLE_MTU_MAX)
          mtu = BLE_MTU_MAX;
      if (mtu > BLE_MTU_MIN) {
        ble_mtu = mtu;
        BLEDevice::setMTU(ble_mtu);
        DOLOGT(); DOLOG(F("BLE MTU set to: ")); DOLOGLN(ble_mtu);
      } else {
        DOLOGT(); DOLOG(F("BLE MTU unchanged (current: ")); DOLOG(ble_mtu); DOLOG(F(", requested: ")); DOLOG(mtu); DOLOGLN(F(")"));
      }
    }
    #endif // ESP32
};

// BLE Characteristic Callbacks
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      doYIELD;
      DODEBUGT();
      DODEBUGLN(F("BLE UART Write Callback"));
      DODEBUGT();
      DODEBUG(F("Characteristic Value: >>"));
      DODEBUG(pCharacteristic->getValue().c_str());
      DODEBUGLN(F("<<"));
      DODEBUGT();
      bleCommandBuffer = "";
      DODEBUG(F("BLE Command Buffer START: "));
      DODEBUGLN(bleCommandBuffer);
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
      DODEBUGT();
      DODEBUG(F("BLE Command Buffer: "));
      DODEBUGLN(bleCommandBuffer);
    }
};

void setup_ble() {
  DOLOGLN(F("Setting up BLE"));

  // Create the BLE Device
  BLEDevice::init(BLUETOOTH_UART_DEVICE_NAME);
  BLEDevice::setMTU(ble_mtu); // Request MTU matching AT buffer size

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic for TX (notifications to client)
  pTxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pTxCharacteristic->addDescriptor(new BLE2902());

  // Create a BLE Characteristic for RX (writes from client)
  pRxCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_RX,
                      BLECharacteristic::PROPERTY_WRITE
                    );

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();

  DOLOGLN(F("BLE Advertising started, waiting for client connection"));
}

void handle_ble_command() {
  if (bleCommandReady && bleCommandBuffer.length() > 0) {
    // Process the BLE command using the same AT command handler

    #ifdef DEBUG
    // buffer log/debug
    DODEBUGT();
    DODEBUG(F("Handling BLE command: >>"));
    DODEBUG(bleCommandBuffer);
    DODEBUG(F("<<, size: "));
    DODEBUGLN(bleCommandBuffer.length());
    // buffer log/debug in hex
    DODEBUGT();
    DODEBUG(F("BLE Command Buffer HEX: "));
    for (size_t i = 0; i < bleCommandBuffer.length(); i++) {
      char hexBuffer[3];
      sprintf(hexBuffer, "%02X", (unsigned char)bleCommandBuffer[i]);
      DODEBUG(hexBuffer);
    }
    DODEBUGLN();
    #endif // DEBUG

    // Check if the command starts with "AT"
    if (bleCommandBuffer.startsWith("AT")) {
      // Handle AT command
      at_cmd_handler(NULL, bleCommandBuffer.c_str());
    } else {
      ble_send_response("+ERROR: invalid command");
    }

    bleCommandBuffer = "";
    bleCommandReady = false;
  }
}

void ble_send_response(const String& response) {
  if (deviceConnected && pTxCharacteristic) {
    // Send response with line terminator
    String fr = response + "\r\n";
    ble_send(fr);
  }
}

void ble_send_n(const char& bstr, int len) {
  DODEBUG(F("Sending BLE data SIZE: "));
  DODEBUG(bstr);
  DODEBUG(F(", size: "));
  DODEBUG(len);
  DODEBUG(F(", MTU: "));
  DODEBUG(ble_mtu);
  DODEBUG(F(", device connected: "));
  DODEBUGLN(deviceConnected);
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
#endif // BT_BLE

void at_send_response(SerialCommands* s, const String& response) {
  if (s != NULL) {
    s->GetSerial()->println(response);
  #if defined(BLUETOOTH_UART_AT) && defined(BT_BLE)
  } else {
    ble_send_response(response);
  #endif
  }
}

void setup_cfg(){
  // EEPROM read
  EEPROM.begin(sizeof(cfg));
  EEPROM.get(CFG_EEPROM, cfg);
  // was (or needs) initialized?
  if(cfg.initialized != CFGINIT || cfg.version != CFGVERSION){
    cfg.do_verbose = 1;
    DOLOGLN(F("reinitializing config"));
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
    cfg.main_loop_delay   = 100;
    strcpy((char *)&cfg.ntp_host, (char *)DEFAULT_NTP_SERVER);
    cfg.ip_mode = IPV4_DHCP | IPV6_DHCP;
    // write to EEPROM
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
    DOLOGLN(F("reinitializing config done"));
  }
}

void WiFiEvent(WiFiEvent_t event){
  doYIELD;
  switch(event) {
      case ARDUINO_EVENT_WIFI_READY:
          DOLOGLN(F("WiFi ready"));
          break;
      case ARDUINO_EVENT_WIFI_STA_START:
          DOLOGLN(F("WiFi STA started"));
          break;
      case ARDUINO_EVENT_WIFI_STA_STOP:
          DOLOGLN(F("WiFi STA stopped"));
          if(esp_sntp_enabled()){
              DOLOGLN(F("Stopping NTP sync"));
              esp_sntp_stop();
          }
          break;
      case ARDUINO_EVENT_WIFI_STA_CONNECTED:
          DOLOG(F("WiFi STA connected to "));
          DOLOGLN(WiFi.SSID());
          break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
          DOLOGLN(F("WiFi STA disconnected"));
          if(esp_sntp_enabled()){
              DOLOGLN(F("Stopping NTP sync"));
              esp_sntp_stop();
          }
          break;
      case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
          DOLOGLN(F("WiFi STA auth mode changed"));
          break;
      case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
          DOLOGLN("WiFi STA got IPv6: ");
          {
              IPAddress g_ip6 = WiFi.globalIPv6();
              DOLOG(F("Global IPv6: "));
              DOLOGLN(g_ip6.toString());
              IPAddress l_ip6 = WiFi.linkLocalIPv6();
              DOLOG(F("LinkLocal IPv6: "));
              DOLOGLN(l_ip6.toString());
          }
          // allow ipv6 TCP/UDP connections both server and client
          connections_tcp_ipv6();
          connections_udp_ipv6();
          break;
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          DOLOG(F("WiFi STA got IP: "));
          DOLOGLN(WiFi.localIP());
          connections_tcp_ipv4();
          connections_udp_ipv4();
          break;
      case ARDUINO_EVENT_WIFI_STA_LOST_IP:
          DOLOGLN(F("WiFi STA lost IP"));
          if(esp_sntp_enabled()){
              DOLOGLN(F("Stopping NTP sync"));
              esp_sntp_stop();
          }
          // TODO: stop TCP/UDP connections
          break;
      case ARDUINO_EVENT_WPS_ER_SUCCESS:
          DOLOGLN(F("WPS succeeded"));
          break;
      case ARDUINO_EVENT_WPS_ER_FAILED:
          DOLOGLN(F("WPS failed"));
          break;
      case ARDUINO_EVENT_WPS_ER_TIMEOUT:
          DOLOGLN(F("WPS timed out"));
          break;
      case ARDUINO_EVENT_WPS_ER_PIN:
          DOLOGLN(F("WPS PIN received"));
          break;
      default:
          break;
  }
}

#ifdef BT_CLASSIC
void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
  doYIELD;
  if(event == ESP_SPP_START_EVT){
    DOLOGLN(F("BlueTooth UART Initialized SPP"));
  } else if(event == ESP_SPP_SRV_OPEN_EVT){
    DOLOGLN(F("BlueTooth UART Client connected"));
  } else if(event == ESP_SPP_CLOSE_EVT){
    DOLOGLN(F("BlueTooth UART Client disconnected"));
  } else if(event == ESP_SPP_DATA_IND_EVT){
    DOLOGLN(F("BlueTooth UART Data received"));
    // any new AT command?
    ATScBT.ReadSerial();
  }
}
#endif

char T_buffer[512] = {""};
char * T(const char *tformat = "[\%H:\%M:\%S]"){
  time_t t;
  struct tm gm_new_tm;
  time(&t);
  localtime_r(&t, &gm_new_tm);
  strftime(T_buffer, 512, tformat, &gm_new_tm);
  return T_buffer;
}

void setup(){
  // Serial setup, init at 115200 8N1
  Serial.begin(115200);

  // setup cfg
  setup_cfg();

  // Setup AT command handler
  #ifdef UART_AT
  ATSc.SetDefaultHandler(&at_cmd_handler);
  #endif

  // BlueTooth SPP setup possible?
  #if defined(BLUETOOTH_UART_AT) && defined(BT_BLE)
  setup_ble();
  #endif

  #if defined(BLUETOOTH_UART_AT) && defined(BT_CLASSIC)
  DOLOG(F("Setting up Bluetooth Classic"));
  SerialBT.begin(BLUETOOTH_UART_DEVICE_NAME);
  SerialBT.setPin(BLUETOOTH_UART_DEFAULT_PIN);
  SerialBT.register_callback(BT_EventHandler);
  ATScBT.SetDefaultHandler(&at_cmd_handler);
  #endif

  // setup WiFi with ssid/pass from EEPROM if set
  setup_wifi();

  // led to show status
  pinMode(LED, OUTPUT);
}

void loop(){
  // DOLOG(F("."));
  doYIELD;

  // Handle Serial AT commands
  #ifdef UART_AT
  if(ATSc.GetSerial()->available())
    ATSc.ReadSerial();
  doYIELD;
  #endif

  #if defined(BLUETOOTH_UART_AT)
  #if defined(BT_BLE)
  #ifdef TIMELOG
  if(cfg.do_timelog && millis() - last_time_log > 500){
    ble_send(T("🍓 [%H:%M:%S]:📡 ⟹  🖫 & 💾\n"));
    #ifdef LOGUART
    if(cfg.do_log){
      DOLOG(T("[%H:%M:%S]: OK\n"));
    }
    #endif
    last_time_log = millis();
  }
  #endif
  #endif
  #endif

  // Handle BLE AT commands
  #ifdef BLUETOOTH_UART_AT
  #ifdef BT_BLE
  handle_ble_command();

  doYIELD;

  // Handle BLE connection changes
  if (!deviceConnected && oldDeviceConnected) {
    // restart advertising
    pServer->startAdvertising();
    DOLOGLN(F("BLE Restart advertising"));
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
  #endif
  #endif

  // Read all available bytes from UART
  char uart_buf[256];
  size_t len = Serial.readBytes(uart_buf, sizeof(uart_buf));
  if (len > 0) {

      // UART read + UDP send
    #ifdef SUPPORT_UDP
    if (valid_udp_host) {
      int sent = send_udp_data((const uint8_t*)uart_buf, len);
      if (sent > 0)
        DOLOG(F("Sent UDP packet with UART data\n"));
    }
    #endif

    // UART read + TCP send
    #ifdef SUPPORT_TCP
    if (valid_tcp_host) {
      int sent = send_tcp_data((const uint8_t*)uart_buf, len);
      if (sent > 0)
        DOLOG(F("Sent TCP packet with UART data\n"));
    }
    #endif
  }

  // TCP read + UART send
  #ifdef SUPPORT_TCP
  if (valid_tcp_host) {
    char tcp_buf[256];
    int tcp_len = recv_tcp_data((uint8_t*)tcp_buf, sizeof(tcp_buf));
    if (tcp_len > 0) {
      Serial.write((const uint8_t*)tcp_buf, tcp_len);
      DOLOG(F("Received TCP data and sent to UART\n"));
    }
  }
  #endif

  //doYIELD;
  delay(cfg.main_loop_delay);

  doYIELD;

  // just wifi check
  if(millis() - last_wifi_check > 500){
    if(!logged_wifi_status){
      #ifdef VERBOSE
      if(cfg.do_verbose){
        if(WiFi.status() == WL_CONNECTED){
          DOLOGLN(F("WiFi connected: "));
          DOLOG(F("WiFi ipv4: "));
          DOLOGLN(WiFi.localIP());
          DOLOG(F("WiFi ipv4 gateway: "));
          DOLOGLN(WiFi.gatewayIP());
          DOLOG(F("WiFi ipv4 netmask: "));
          DOLOGLN(WiFi.subnetMask());
          DOLOG(F("WiFi ipv4 DNS: "));
          DOLOGLN(WiFi.dnsIP());
          DOLOG(F("WiFi MAC: "));
          DOLOGLN(WiFi.macAddress());
          DOLOG(F("WiFi RSSI: "));
          DOLOGLN(WiFi.RSSI());
          DOLOG(F("WiFi SSID: "));
          DOLOGLN(WiFi.SSID());
        } else {
          DOLOGLN(F("WiFi not connected"));
        }
      }
      #endif
      logged_wifi_status = 1;
    }
    last_wifi_check = millis();
  }
}
