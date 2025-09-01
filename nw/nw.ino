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
#define USE_ESP_IDF_LOG
#define CORE_DEBUG_LEVEL 5
#define LOG_LOCAL_LEVEL 5
#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_wps.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>
#include <WiFi.h>

#ifndef DEFAULT_HOSTNAME
#define DEFAULT_HOSTNAME "nw"
#endif

#ifndef DEFAULT_WIFI_SSID
#define DEFAULT_WIFI_SSID ""
#endif
#ifndef DEFAULT_WIFI_PASS
#define DEFAULT_WIFI_PASS ""
#endif

#define LOG_TIME_FORMAT "[\%H:\%M:\%S][info]: "

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

#define LOG(...)    do_printf(3, LOG_TIME_FORMAT, __VA_ARGS__);
#define LOGT(...)   do_printf(2, LOG_TIME_FORMAT, __VA_ARGS__);
#define LOGR(...)   do_printf(0, LOG_TIME_FORMAT, __VA_ARGS__);
#define LOGE(...)   do_printf(2, LOG_TIME_FORMAT, __VA_ARGS__);do_printf(0, NULL, ", errno: %d (%s)\n", errno, get_errno_string(errno));

#define IPV4_DHCP    1
#define IPV4_STATIC  2
#define IPV6_DHCP    4

/* main config */
typedef struct cfg_t {
  char wifi_ssid[32]   = {0}; // max 31 + 1
  char wifi_pass[64]   = {0}; // nax 63 + 1
  uint8_t ip_mode      = IPV4_DHCP | IPV6_DHCP;
  char hostname[64]    = {0}; // max hostname + 1
  uint8_t ipv4_addr[4] = {0}; // static IP address
  uint8_t ipv4_gw[4]   = {0}; // static gateway
  uint8_t ipv4_mask[4] = {0}; // static netmask
  uint8_t ipv4_dns[4]  = {0}; // static DNS server
  uint16_t tcp_port    = 0;
  char tcp_host_ip[40] = {0}; // IPv4 or IPv6 string, up to 39 chars for IPv6
} TCFG;

TCFG cfg = {0};

char WIFI_SSID[] = DEFAULT_WIFI_SSID; // default no SSID, override with your own
char WIFI_PASS[] = DEFAULT_WIFI_PASS; // default no password, override with your own

/* state flags */
long last_wifi_check = 0;
long last_wifi_info_log = 0;
long last_wifi_reconnect = 0;

/* WPS (WiFi Protected Setup) support */
bool wps_running = false;
unsigned long wps_start_time = 0;
#define WPS_TIMEOUT_MS 120000  // 2 minutes timeout for WPS
esp_wps_config_t wps_config;

bool wifi_connected(){
  return (WiFi.status() == WL_CONNECTED);
}

void WiFiEvent(WiFiEvent_t event){
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
          LOGT("[WiFi] STA got IPV6: ga: %s", WiFi.globalIPv6().toString().c_str());
          LOGR(", ll: %s", WiFi.linkLocalIPv6().toString().c_str());
          LOGR("\n");
          break;
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
          LOG("[WiFi] STA got IP: %s", WiFi.localIP().toString().c_str());
          break;
      case ARDUINO_EVENT_WIFI_STA_LOST_IP:
          LOG("[WiFi] STA lost IP");
          break;
      case ARDUINO_EVENT_WPS_ER_SUCCESS:
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
            LOG("[WPS] Saved Pass (clear): %s", cfg.wifi_pass);

            // WPS success, credentials are automatically saved
            // Restart WiFi connection with new credentials
            LOG("[WPS] Restarting WiFi with new credentials");
            reset_networking();
          }
          break;
      case ARDUINO_EVENT_WPS_ER_FAILED:
          LOG("[WPS] failed");
          wps_running = false;
          wps_start_time = 0;
          esp_wifi_wps_disable();
          break;
      case ARDUINO_EVENT_WPS_ER_TIMEOUT:
          LOG("[WPS] timed out");
          wps_running = false;
          wps_start_time = 0;
          esp_wifi_wps_disable();
          break;
      case ARDUINO_EVENT_WPS_ER_PIN:
          LOG("[WPS] PIN received");
          break;
      default:
          break;
  }
}

void setup_wifi(){
  strncpy(cfg.wifi_ssid, WIFI_SSID, sizeof(WIFI_SSID)); // default no password, override with your own
  strncpy(cfg.wifi_pass, WIFI_PASS, sizeof(WIFI_PASS)); // default no password, override with your own

  cfg.ip_mode = IPV4_DHCP | IPV6_DHCP;

  LOG("[WiFi] setup started");
  WiFi.disconnect();
  LOG("[WiFi] setting WiFi mode to STA");
  WiFi.mode(WIFI_MODE_STA);
  LOG("[WiFi] adding event handler");
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
      (cfg.ip_mode & IPV6_DHCP) ? "IPv6 DHCP " : "");
  if(strlen(cfg.wifi_ssid) == 0) {
    LOG("[WiFi] No SSID configured, not connecting to WiFi");
    WiFi.mode(WIFI_MODE_NULL);
    return;
  }
  LOG("[WiFi] SSID: %s", cfg.wifi_ssid);
  if(strlen(cfg.wifi_pass) == 0) {
    LOG("[WiFi] Pass: none");
  } else {
    // print password as stars, even fake the length
    LOG("[WiFi] Pass: %s", cfg.wifi_pass);
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

  // IPv6 configuration, before WiFi.begin()!
  if(cfg.ip_mode & IPV6_DHCP){
    LOG("[WiFi] Using DHCP for IPv6");
    WiFi.enableIPv6(true);
  } else {
    LOG("[WiFi] Not using IPv6");
    WiFi.enableIPv6(false);
  }

  // after WiFi.config()!
  LOG("[WiFi] Starting connection");
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

  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  if(cfg.hostname){
    WiFi.setHostname(cfg.hostname);
  } else {
    WiFi.setHostname(DEFAULT_HOSTNAME);
  }

  // connect to Wi-Fi
  LOG("[WiFi] Connecting to %s", cfg.wifi_ssid);
  WiFi.persistent(false);
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
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
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

  LOG("[WiFi] setup done");
  LOG("[BT] Starting BLE");
  btStart();
  LOG("[BT] BLE started");
}

void stop_networking(){
  LOG("[WiFi] Stop networking");
  // first stop WiFi
  WiFi.removeEvent(WiFiEvent);
  WiFi.disconnect(true);
  while(WiFi.status() == WL_CONNECTED){
    LOG("[WiFi] waiting for disconnect, status: %d", WiFi.status());
    delay(100);
  }
  WiFi.mode(WIFI_MODE_NULL);
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
  LOG("[WiFi] reset networking...");
  // first stop WiFi
  stop_networking();
  // start networking
  start_networking();
  LOG("[WiFi] reset networking done");
}

void reconfigure_network_connections(){
  LOG("[WiFi] network connections, wifi status: %s", (wifi_connected()) ? "connected" : "not connected");
  if(wifi_connected()){
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
  }
  return;
}

// Helper function to get human-readable errno messages
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

#include <WiFiClient.h>
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <lwip/netdb.h>

WiFiClient tcp_client;
int tcp_sock = -1;
uint8_t valid_tcp_host = 0;
long last_tcp_check = 0;

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
  LOG("[IPv6] ip check ipv6? ga: %s, ll: %s", ipv6_ga.toString().c_str(), ipv6_ll.toString().c_str());
  return strlen(ipv6_ga.toString().c_str()) > 2;
}

void connections_tcp_ipv6() {
  if(strlen(cfg.tcp_host_ip) == 0 || cfg.tcp_port == 0) {
    valid_tcp_host = 0;
    LOG("[TCP] Invalid TCP host IP or port, not setting up TCP");
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
  // Set socket to non-blocking mode and read/write
  int flags = fcntl(tcp_sock, F_GETFL, 0);
  if (flags >= 0)
    fcntl(tcp_sock, F_SETFL, flags | O_NONBLOCK | O_RDWR);
  // connect, this will be non-blocking, so we get a EINPROGRESS
  if (connect(tcp_sock, (struct sockaddr*)&sa6, sizeof(sa6)) == -1) {
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
    errno = old_errno; // restore old errno
    LOGE("[TCP] IPv6 connection on fd:%d initiated to: %s, port: %d", tcp_sock, cfg.tcp_host_ip, cfg.tcp_port);
    valid_tcp_host = 2; // 2 = IPv6
    struct sockaddr_in6 l_sa6;
    memset(&l_sa6, 0, sizeof(l_sa6));
    if(getsockname(tcp_sock, (struct sockaddr*)&l_sa6, &optlen) == 0) {
      char local_addr_str[40] = {0};
      if(inet_ntop(AF_INET6, &l_sa6.sin6_addr, local_addr_str, sizeof(local_addr_str))) {
        LOG("[TCP] IPv6 TCP local address: %s, port: %d", local_addr_str, ntohs(l_sa6.sin6_port));
      }
    } else {
      LOGE("[TCP] Failed to get local IPv6 TCP address");
    }
    struct sockaddr_in6 r_sa6;
    memset(&r_sa6, 0, sizeof(r_sa6));
    if(getpeername(tcp_sock, (struct sockaddr*)&r_sa6, &optlen) == 0) {
      char peer_addr_str[40] = {0};
      if(inet_ntop(AF_INET6, &r_sa6.sin6_addr, peer_addr_str, sizeof(peer_addr_str))) {
        LOG("[TCP] IPv6 TCP peer address: %s, port: %d", peer_addr_str, ntohs(r_sa6.sin6_port));
      }
    } else {
      LOGE("[TCP] Failed to get peer IPv6 TCP address");
    }
    return;
  }
  valid_tcp_host = 2; // 2 = IPv6
  LOG("[TCP] IPv6 TCP connected fd:%d to %s, port:%d", tcp_sock, cfg.tcp_host_ip, cfg.tcp_port);
}


void connections_tcp_ipv4() {
  if(strlen(cfg.tcp_host_ip) == 0 || cfg.tcp_port == 0) {
    valid_tcp_host = 0;
    LOG("[TCP] Invalid host IP or port, not setting up TCP");
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
    valid_tcp_host = 0;
    LOG("[TCP] TCP socket %d closed", fd_orig);
  }
}

// Helper: send TCP data (IPv4/IPv6)
int send_tcp_data(const uint8_t* data, size_t len) {
  LOG("[TCP] send_tcp_data len: %d, valid_tcp_host: %d", len, valid_tcp_host);
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
  if (!wifi_connected()) {
    return; // No WiFi connection
  }

  // Take a local copy of the socket to avoid race conditions
  uint8_t local_valid_tcp_host = valid_tcp_host;

  if (local_valid_tcp_host == 2 && tcp_sock >= 0) {
    LOG("[TCP] check_tcp_connection, valid_tcp_host: %d, tcp_sock: %d, tm: %d", local_valid_tcp_host, tcp_sock, tm);
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
      return;
    }

    if (FD_ISSET(tcp_sock, &errorfds)) {
      LOG("[TCP] socket %d has error, reconnecting", tcp_sock);

      // Check if socket is connected by trying to get socket error
      int socket_error = 0;
      socklen_t len = sizeof(socket_error);
      if (getsockopt(tcp_sock, SOL_SOCKET, SO_ERROR, &socket_error, &len) == 0) {
        if (socket_error != 0) {
          LOG("[TCP] socket %d error detected: %d (%s), reconnecting", tcp_sock, socket_error, get_errno_string(socket_error));
          close_tcp_socket();
          return;
        }
      } else {
        LOGE("[TCP] getsockopt failed on socket %d", tcp_sock);
      }
      close_tcp_socket();
      return;
    }

    if (FD_ISSET(tcp_sock, &writefds)) {
      LOG("[TCP] socket %d writable, connection OK", tcp_sock);
    } else {
      LOG("[TCP] socket %d not yet writable", tcp_sock);
    }

  } else if (local_valid_tcp_host == 1) {
    LOG("[TCP] check_tcp_connection, valid_tcp_host: %d", local_valid_tcp_host);
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
    LOG("[TCP] No valid TCP host, attempting to establish connection");
    if (is_ipv6_addr(cfg.tcp_host_ip) && has_ipv6_address()) {
      connections_tcp_ipv6();
    } else if (!is_ipv6_addr(cfg.tcp_host_ip) && has_ipv4_address()) {
      connections_tcp_ipv4();
    } else {
      LOG("[TCP] No matching IP version available for target %s", cfg.tcp_host_ip);
    }
  }
  return;
}

void(* resetFunc)(void) = 0;

/* WPS (WiFi Protected Setup) Functions both PBC and PIN */
bool start_wps(const char *pin) {
  if (wps_running) {
    LOG("[WPS] WPS already running");
    return false;
  }

  LOG("[WPS] Starting WPS Push Button Configuration");

  // Stop any current WiFi connections
  stop_networking();
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

void log_wifi_info(){
  if(wifi_connected()){
    LOG("[WiFi] status: connected");
    LOGT("[WiFi] connected: SSID:%s", WiFi.SSID().c_str());
    LOGR(", MAC:%s", WiFi.macAddress().c_str());
    LOGR(", RSSI:%d", WiFi.RSSI());
    LOGR(", BSSID:%s", WiFi.BSSIDstr().c_str());
    LOGR(", CHANNEL:%d", WiFi.channel());
    LOGR("\n");

    // Get current IP info
    IPAddress ip = WiFi.localIP();
    IPAddress gw = WiFi.gatewayIP();
    IPAddress nm = WiFi.subnetMask();
    IPAddress dns = WiFi.dnsIP();

    if(cfg.ip_mode & (IPV4_STATIC|IPV4_DHCP)){
      LOGT("[IPV4] ADDR:%s", ip.toString().c_str());
      LOGR(", GW:%s", gw.toString().c_str());
      LOGR(", NM:%s", nm.toString().c_str());
      LOGR(", DNS:%s", dns.toString().c_str());
      LOGR("\n");
    }

    if(cfg.ip_mode & IPV6_DHCP){
      IPAddress ipv6_ga = WiFi.globalIPv6();
      IPAddress ipv6_ll = WiFi.linkLocalIPv6();
      LOGT("[IPV6] ADDR GA:%s", ipv6_ga.toString().c_str());
      LOGR(", ADDR LL:%s", ipv6_ll.toString().c_str());
      LOGR("\n");
    }
  } else {
    LOG("[WiFi] status: not connected");
  }
}

void setup(){
  Serial.begin(115200);
  reset_networking();
}

void loop(){

  // Check WPS timeout
  if (wps_running && (millis() - wps_start_time > WPS_TIMEOUT_MS)) {
    LOG("[WPS] WPS timeout reached, stopping WPS");
    stop_wps();
  }

  // just wifi check
  if(millis() - last_wifi_check > 500){
    last_wifi_check = millis();
    if(millis() - last_wifi_info_log > 60000){
      last_wifi_info_log = millis();
      log_wifi_info();
    }
    if(!wifi_connected()){
      // not connected, try to reconnect
      if(last_wifi_reconnect == 0 || millis() - last_wifi_reconnect > 30000){
        last_wifi_reconnect = millis();
        LOG("[WiFi] Not connected, attempting to reconnect...");
        reset_networking();
      }
    } else {
      // connected
      last_wifi_reconnect = millis();
    }
  }

  // TCP connection check at configured interval
  if(wifi_connected() && (cfg.tcp_port != 0 && strlen(cfg.tcp_host_ip) != 0)){
    if(last_tcp_check == 0 || millis() - last_tcp_check > 500){
      last_tcp_check = millis();
      check_tcp_connection(100000);
    }
  }
}
