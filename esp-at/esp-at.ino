#include <Arduino.h>
#include <sys/time.h>
#ifdef ARDUINO_ARCH_ESP32
#include <WiFi.h>
#endif
#ifdef ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>
#endif
#include <errno.h>
#include "time.h"
#include "SerialCommands.h"
#include "EEPROM.h"
#include "sntp.h"

#define LED           2

#ifndef VERBOSE
#define VERBOSE
#endif

#if defined(DEBUG) || defined(VERBOSE)
void print_time_to_serial(const char *tformat = "[%H:%M:%S]: "){
  // new time - set by NTP
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
 #define T()
#endif

#define BLUETOOTH_UART_AT

#ifdef BLUETOOTH_UART_AT
#define BLUETOOTH_UART_DEVICE_NAME "UART"

#ifdef BT_CLASSIC
#define BLUETOOTH_UART_DEFAULT_PIN "1234"
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

/* ESP yield */
#if defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32)
 #define doYIELD yield();
#else
 #define doYIELD
#endif

/* our AT commands over UART to config WiFi */
char atscbu[128] = {""};
SerialCommands ATSc(&Serial, atscbu, sizeof(atscbu), "\r\n", "\r\n");

#define CFGVERSION 0x01 // switch between 0x01/0x02 to reinit the config struct change
#define CFGINIT    0x72 // at boot init check flag
#define CFG_EEPROM 0x00 

/* main config */
typedef struct cfg_t {
  uint8_t initialized  = 0;
  uint8_t version      = 0;
  uint8_t do_verbose   = 0;
  uint8_t do_debug     = 0;
  uint8_t do_log       = 0;
  uint16_t udp_port    = 0;
  uint16_t main_loop_delay = 100;
  char wifi_ssid[32]   = {0};   // max 31 + 1
  char wifi_pass[64]   = {0};   // nax 63 + 1
  char ntp_host[64]    = {0};   // max hostname + 1
};
cfg_t cfg;

/* state flags */
uint8_t ntp_is_synced      = 1;
uint8_t logged_wifi_status = 0;
unsigned long last_wifi_check = 0;

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
    configTime(0, 0, (char *)&cfg.ntp_host);
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
    configTime(0, 0, (char *)&cfg.ntp_host);
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
    configTime(0, 0, (char *)&cfg.ntp_host);
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

// BLE Server Callbacks
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      DODEBUGT();
      DODEBUGLN(F("BLE client connected"));
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      DODEBUGT();
      DODEBUGLN(F("BLE client disconnected"));
    }
};

// BLE Characteristic Callbacks
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
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
  DOLOG(F("Sending BLE response: "));
  DOLOGLN(response);
  if (deviceConnected && pTxCharacteristic) {
    // Send response with line terminator
    String fullResponse = response + "\r\n";

    // Split response into 20-byte chunks (BLE characteristic limit)
    int responseLength = fullResponse.length();
    int offset = 0;

    while (offset < responseLength) {
      int chunkSize = min(20, responseLength - offset);
      String chunk = fullResponse.substring(offset, offset + chunkSize);
      pTxCharacteristic->setValue(chunk.c_str());
      pTxCharacteristic->notify();
      offset += chunkSize;
      delay(10); // Small delay between chunks
    }
  }
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

void setup(){
  // Serial setup, init at 115200 8N1
  Serial.begin(115200);

  // setup cfg
  setup_cfg();

  // Setup AT command handler
  ATSc.SetDefaultHandler(&at_cmd_handler);

  // see http://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
  setenv("TZ", "UTC", 1);
  tzset();

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

  // setup NTP sync to RTC
  if(strlen(cfg.ntp_host) && strlen(cfg.wifi_ssid) && strlen(cfg.wifi_pass)){
    DOLOG(F("will sync with ntp, wifi ssid/pass ok: "));
    DOLOGLN(cfg.ntp_host);
  }
  configTime(0, 0, (char *)&cfg.ntp_host);

  // led to show status
  pinMode(LED, OUTPUT);
}

void loop(){
  // DOLOG(F("."));

  // Handle Serial AT commands
  if(ATSc.GetSerial()->available()){
    ATSc.ReadSerial();
  }

  // Handle BLE AT commands
  #ifdef BLUETOOTH_UART_AT
  #ifdef BT_BLE
  handle_ble_command();

  // Handle BLE connection changes
  if (!deviceConnected && oldDeviceConnected) {
    // restart advertising
    pServer->startAdvertising();
    DOLOG(F("BLE Restart advertising"));
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
  #endif
  #endif

  delay(cfg.main_loop_delay);

  // just wifi check
  if(millis() - last_wifi_check > 500){
    if(!logged_wifi_status){
      #ifdef VERBOSE
      if(cfg.do_verbose){
        if(WiFi.status() == WL_CONNECTED){
          DOLOGLN(F("WiFi connected: "));
          DOLOG(F("ipv4:"));
          DOLOGLN(WiFi.localIP());
          DOLOGLN(WiFi.gatewayIP());
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

void setup_cfg(){
  // EEPROM read
  EEPROM.begin(sizeof(cfg));
  EEPROM.get(CFG_EEPROM, cfg);
  // was (or needs) initialized?
  if(cfg.initialized != CFGINIT || cfg.version != CFGVERSION){
    // clear
    memset(&cfg, 0, sizeof(cfg));
    // reinit
    cfg.initialized       = CFGINIT;
    cfg.version           = CFGVERSION;
    cfg.do_verbose        = 1;
    cfg.do_log            = 1;
    cfg.main_loop_delay   = 100;
    strcpy((char *)&cfg.ntp_host, (char *)DEFAULT_NTP_SERVER);
    // write to EEPROM
    EEPROM.put(CFG_EEPROM, cfg);
    EEPROM.commit();
  }
}

void WiFiEvent(WiFiEvent_t event){
  #ifdef VERBOSE
  if(cfg.do_verbose){
    switch(event) {
        case ARDUINO_EVENT_WIFI_STA_START:
            DOLOGLN(F("WiFi STA started"));
            break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            DOLOG(F("WiFi STA connected to "));
            DOLOGLN(WiFi.SSID());
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP6:
            DOLOGLN("STA IPv6");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            DOLOG(F("WiFi STA got IP: "));
            DOLOGLN(WiFi.localIP());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            DOLOGLN(F("WiFi STA disconnected"));
            break;
        case ARDUINO_EVENT_WIFI_STA_STOP:
            DOLOGLN(F("WiFi STA stopped"));
            break;
        default:
            break;
    }
  }
  #endif
}

#ifdef BT_CLASSIC
void BT_EventHandler(esp_spp_cb_event_t event, esp_spp_cb_param_t *param){
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

void setup_wifi(){
  // are we connecting to WiFi?
  if(strlen(cfg.wifi_ssid) == 0 || strlen(cfg.wifi_pass) == 0)
    return;
  if(WiFi.status() == WL_CONNECTED)
    return;
  logged_wifi_status = 0; // reset logged status

  WiFi.disconnect(); // disconnect from any previous connection
  DOLOGLN(F("Setting up WiFi"));
  #ifdef VERBOSE
  WiFi.onEvent(WiFiEvent);
  #endif
  WiFi.mode(WIFI_STA); // set WiFi mode to Station
  WiFi.setAutoReconnect(true); // enable auto-reconnect
  WiFi.setSleep(false); // disable WiFi sleep mode
  WiFi.setHostname("esp"); // set hostname for the device
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // set WiFi transmit power (optional, adjust as needed)
  WiFi.enableSTA(true); // enable Station mode
  WiFi.enableIPv6(true); // enable IPv6 support

  // connect to Wi-Fi
  DOLOG(F("Connecting to "));
  DOLOGLN(cfg.wifi_ssid);
  WiFi.persistent(false);
  WiFi.begin(cfg.wifi_ssid, cfg.wifi_pass);
}
