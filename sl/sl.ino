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

// Logging setup for esp32c3

#define D Serial.printf

#define UART_LOG_DEV_UART0
#ifdef UART_LOG_DEV_UART0
 #define NO_GLOBAL_INSTANCES
 #define NO_GLOBAL_SERIAL
 #include <HardwareSerial.h>
 extern HardwareSerial Serial0;
 extern HardwareSerial Serial1;
 #define UART0         Serial0
 #define USBSERIAL0    Serial0
 #define Serial        Serial0
 #define UART1         Serial1
#else
 #include <HardwareSerial.h>
 #define UART0         Serial
 #define USBSERIAL0    Serial
 #define UART1         Serial1
#endif // UART_LOG_DEV_UART0

#define USE_ESP_IDF_LOG
#define CORE_DEBUG_LEVEL 5
#define LOG_LOCAL_LEVEL 5
#include <esp_log.h>
#include <WiFi.h>
#include <esp_sleep.h>
#include <esp_wifi.h>
#include <driver/rtc_io.h>

#include <Arduino.h>

#define NOINLINE __attribute__((noinline,noipa))
#define INLINE __attribute__((always_inline))
#define ALIGN(x) __attribute__((aligned(x)))

#define LED    GPIO_NUM_8
#define BUTTON GPIO_NUM_3

uint64_t sleep_ms = 5000;

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR unsigned long start = 0;
RTC_DATA_ATTR uint8_t sleep_is_configured = 0;
RTC_DATA_ATTR uint8_t last_button_state = 0;
RTC_DATA_ATTR unsigned long press_start = 0;

void NOINLINE check_wakeup_reason(){
  D("Checking wakeup reason...\n");
  esp_sleep_wakeup_cause_t wakup_reason = esp_sleep_get_wakeup_cause();
  switch(wakup_reason){
    case ESP_SLEEP_WAKEUP_UART:
      // woke up due to UART
      D("[SLEEP] Woke up due to UART\n");
      break;
    case ESP_SLEEP_WAKEUP_BT:
      // woke up due to BT
      D("[SLEEP] Woke up due to BT\n");
      break;
    case ESP_SLEEP_WAKEUP_WIFI:
      // woke up due to WiFi
      D("[SLEEP] Woke up due to WiFi\n");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      // woke up due to timer
      D("[SLEEP] Woke up due to timer\n");
      break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
      // undefined wakeup, should not happen, probably reset
      D("[SLEEP] Woke up due to undefined reason, probably reset or poweron\n");
      break;
    case ESP_SLEEP_WAKEUP_GPIO:
      // woke up due to GPIO, e.g. button press
      D("[SLEEP] Woke up due to GPIO (button press?)\n");
      break;
    case ESP_SLEEP_WAKEUP_VBAT_UNDER_VOLT:
      // woke up due to VBAT low voltage
      D("[SLEEP] Woke up due to VBAT under voltage\n");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      // woke up due to touchpad, should not happen
      D("[SLEEP] Woke up due to touchpad\n");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      // woke up due to ULP, should not happen
      D("[SLEEP] Woke up due to ULP\n");
      break;
    case ESP_SLEEP_WAKEUP_COCPU:
      // woke up due to COCPU, should not happen
      D("[SLEEP] Woke up due to COCPU\n");
      break;
    case ESP_SLEEP_WAKEUP_EXT0:
      // woke up due to EXT0, should not happen
      D("[SLEEP] Woke up due to EXT0\n");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      // woke up due to EXT1, should not happen
      D("[SLEEP] Woke up due to EXT1\n");
      break;
    default:
      // woke up due to other reason, e.g. button press
      D("[SLEEP] Woke up due to other reason: %d\n", wakup_reason);
      break;
  }
}

void sleep_setup(){
  if(sleep_is_configured){
    D("Sleep already configured\n");
    return;
  }
  D("Configuring sleep\n");
  D("Disabling all wakeup sources\n");
  esp_err_t err = ESP_OK;
  err = esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  if(err != ESP_OK){
    D("Failed to disable all wakeup sources: %s\n", esp_err_to_name(err));
  }

  bool ok_btn = esp_sleep_is_valid_wakeup_gpio((gpio_num_t)BUTTON);
  if(ok_btn){
    D("Enabling button wakeup on pin %d, current state: %d\n", BUTTON, last_button_state);
    err = gpio_wakeup_enable((gpio_num_t)BUTTON, (!last_button_state) ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL);
    if(err != ESP_OK){
      D("[SLEEP] Failed to enable button wakeup on pin %d: %s\n", BUTTON, esp_err_to_name(err));
    } else {
      err = esp_sleep_enable_gpio_wakeup();
      if(err != ESP_OK){
        D("[SLEEP] Failed to enable button wakeup: %s", esp_err_to_name(err));
      } else {
        D("[SLEEP] Button wakeup enabled on pin %d\n", BUTTON);
      }
    }
    //if(!last_button_state){
      err = gpio_pullup_en((gpio_num_t)BUTTON);
      if(err != ESP_OK){
        D("Failed to enable pullup on pin %d: %s\n", BUTTON, esp_err_to_name(err));
      }
      err = gpio_pulldown_dis((gpio_num_t)BUTTON);
      if(err != ESP_OK){
        D("Failed to disable pulldown on pin %d: %s\n", BUTTON, esp_err_to_name(err));
      }
        /*
    } else {
        err = gpio_pullup_dis((gpio_num_t)BUTTON);
        if(err != ESP_OK){
            D("Failed to disable pullup on pin %d: %s\n", BUTTON, esp_err_to_name(err));
        }
        err = gpio_pulldown_en((gpio_num_t)BUTTON);
        if(err != ESP_OK){
            D("Failed to enable pulldown on pin %d: %s\n", BUTTON, esp_err_to_name(err));
        }
    }
        */
  } else {
    D("[SLEEP] Button wakeup not possible on pin %d\n", BUTTON);
  }

  D("Sleep configured\n");
  sleep_is_configured = 0;
}

void do_sleep(){
  esp_err_t err = ESP_OK;
  last_button_state = digitalRead(BUTTON) == LOW;
  if(last_button_state){
    D("Button is currently pressed\n");
    if(press_start == 0){
      D("Button press start time not set, setting to now\n");
      press_start = millis();
    }
    return;
  }
  D("Button is currently not pressed\n");
  if(press_start != 0){
    D("Button was pressed, but now released, clearing press start time, duration: %d ms\n", millis() - press_start);
    press_start = 0;
  }
  D("Going to sleep for %d ms\n", sleep_ms);
  sleep_setup();
  // Configure timer
  D("Enabling timer wakeup for %d ms\n", sleep_ms);
  err = esp_sleep_enable_timer_wakeup((uint64_t)sleep_ms * 1000ULL);
  if(err != ESP_OK){
    D("Failed to enable timer wakeup: %s\n", esp_err_to_name(err));
    return;
  }
  //detachInterrupt(digitalPinToInterrupt(BUTTON));
  if(last_button_state){
    D("Button is currently pressed\n");
    return;
  }
  D("Button is currently not pressed\n");
  D("Flushing Serial\n");
  D("Going to light sleep\n");
  Serial.flush();
  start = millis();
  err = esp_light_sleep_start();
  if(err != ESP_OK){
    Serial.flush();
    D("Failed to enter light sleep: %s\n", esp_err_to_name(err));
  } else {
    check_wakeup_reason();
    D("Woke up from light sleep\n");
  }
  last_button_state = digitalRead(BUTTON) == LOW;
  if(last_button_state){
    D("Button is currently pressed after wakeup\n");
    if(press_start == 0){
      D("Button press start time not set, setting to now\n");
      press_start = millis();
    }
  } else {
    D("Button is currently not pressed after wakeup\n");
    if(press_start != 0){
      D("Button was pressed, but now released, clearing press start time, duration: %d ms\n", millis() - press_start);
      press_start = 0;
    }
  }
  D("Woke up from light sleep, took: %d, pressed: %d\n", millis() - start, last_button_state);
}

void setup(){
  Serial.begin(115200);
  delay(2000);
  pinMode(LED, OUTPUT);
  digitalWrite(LED,LOW);
  pinMode(BUTTON, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(BUTTON), buttonISR, CHANGE);
  D("Boot number: %d\n", ++bootCount);
  check_wakeup_reason();
  sleep_setup();
}

static volatile esp_err_t err = ESP_OK;
void loop(){
  digitalWrite(LED,HIGH);
  D("LED ON\n");
  esp_err_t err = ESP_OK;

  D("Holding LED pin %d\n", LED);
  err = gpio_hold_en(LED);
  if(err != ESP_OK){
    D("Failed to hold LED pin %d: %s\n", LED, esp_err_to_name(err));
  }
  do_sleep();
  //attachInterrupt(digitalPinToInterrupt(BUTTON), buttonISR, CHANGE);

  D("Releasing hold on LED pin %d\n", LED);
  err = gpio_hold_dis(LED);
  if(err != ESP_OK){
    D("Failed to release hold on LED pin %d: %s\n", LED, esp_err_to_name(err));
  }

  digitalWrite(LED,LOW);
  D("LED OFF\n");
  D("Holding LED pin %d\n", LED);
  err = gpio_hold_en(LED);
  if(err != ESP_OK){
    D("Failed to hold LED pin %d: %s\n", LED, esp_err_to_name(err));
  }
  D("Going to deep sleep for %d ms\n", sleep_ms);
  err = gpio_hold_dis(LED);
  if(err != ESP_OK){
    D("Failed to release hold on LED pin %d: %s\n", LED, esp_err_to_name(err));
  }

}

