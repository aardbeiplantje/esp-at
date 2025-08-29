#include <Arduino.h>

// ESP-IDF logging configuration - must be defined BEFORE including esp_log.h
#define CONFIG_LOG_DEFAULT_LEVEL 4          // ESP_LOG_INFO level and above
#define CONFIG_LOG_MAXIMUM_LEVEL 5          // Allow all log levels up to VERBOSE
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE     // Enable all local logging
#define CORE_DEBUG_LEVEL 5                  // Arduino core debug level
#define CONFIG_LOG_COLORS 1                 // Enable colored output

#include <esp_log.h>
#include "driver/gptimer_types.h"
#include "driver/gptimer.h"

typedef void (*voidFuncPtr)(void);
typedef void (*voidFuncPtrArg)(void *);

typedef struct {
  voidFuncPtr fn;
  void *arg;
} interrupt_config_t;

struct timer_struct_t {
  gptimer_handle_t timer_handle;
  interrupt_config_t interrupt_handle;
  bool timer_started;
};

#define LED    8
#define BUTTON 9  // Button pin (usually BOOT button on ESP32-C3)

#define LED_PWM_CHANNEL       2    // Use PWM channel 2
#define LED_PWM_FREQUENCY  1000    // 1000 Hz (low frequency to avoid high CPU usage)
#define LED_PWM_RESOLUTION    8    // 8-bit resolution (0-255 brightness levels)

// note: on ESP32, lower value means brighter LED (0 = full brightness, 255 = off)
#define LED_BRIGHTNESS_OFF  255    // LED completely off
#define LED_BRIGHTNESS_ON     0    // LED at full brightness
#define LED_BRIGHTNESS_LOW  200    // LED at low brightness (dimmed)
#define LED_BRIGHTNESS_MED  100    // LED at medium brightness

// Brightness levels array for cycling
const int brightness_levels[] = {LED_BRIGHTNESS_OFF, LED_BRIGHTNESS_LOW, LED_BRIGHTNESS_MED, LED_BRIGHTNESS_ON};
const int num_brightness_levels = sizeof(brightness_levels) / sizeof(brightness_levels[0]);

static const char *TAG = "TM";     // TAG for your module during ESP_LOGx
#define L(...) do { ESP_LOGI(TAG, __VA_ARGS__); Serial.printf("[INFO][%s] ", TAG); Serial.printf(__VA_ARGS__); Serial.println(); } while(0)
#define LD(...) do { ESP_LOGD(TAG, __VA_ARGS__); Serial.printf("[DEBUG][%s] ", TAG); Serial.printf(__VA_ARGS__); Serial.println(); } while(0)
#define LW(...) do { ESP_LOGW(TAG, __VA_ARGS__); Serial.printf("[WARN][%s] ", TAG); Serial.printf(__VA_ARGS__); Serial.println(); } while(0)
#define LE(...) do { ESP_LOGE(TAG, __VA_ARGS__); Serial.printf("[ERROR][%s] ", TAG); Serial.printf(__VA_ARGS__); Serial.println(); } while(0)
void(* resetFunc)(void) = 0;

volatile bool led_state = false;
volatile int8_t last_led_interval = -1;
volatile int8_t last_led_brightness = 0;
volatile int led_interval = 0;
volatile int led_brightness_on, led_brightness_off;
bool led_pwm_enabled = false;

// Button variables
volatile bool button_pressed = false;
volatile unsigned long button_last_time = 0;
volatile int current_brightness_index = 1; // Start with LED_BRIGHTNESS_LOW
const unsigned long BUTTON_DEBOUNCE_MS = 200; // 200ms debounce

hw_timer_t *led_t = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE buttonMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR ledBlinkTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  led_state = !led_state;
  // Note: Don't use ESP_LOG functions in ISR context - they're not ISR-safe
  // Use simple state changes only
  if(led_state) {
    led_on();
  } else {
    led_off();
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR buttonISR() {
  portENTER_CRITICAL_ISR(&buttonMux);
  unsigned long current_time = millis();

  // Simple debouncing - ignore button presses within debounce period
  if (current_time - button_last_time > BUTTON_DEBOUNCE_MS) {
    button_pressed = true;
    button_last_time = current_time;

    // Cycle to next brightness level
    current_brightness_index = (current_brightness_index + 1) % num_brightness_levels;

    // Update LED brightness immediately
    led_brightness_on = brightness_levels[current_brightness_index];

    // If LED is currently on, update brightness immediately
    if (led_state) {
      // Can't call set_led_brightness from ISR due to logging, use direct PWM call
      if (led_pwm_enabled) {
        ledcWriteChannel(LED_PWM_CHANNEL, led_brightness_on);
      } else {
        digitalWrite(LED, led_brightness_on > LED_BRIGHTNESS_LOW ? HIGH : LOW);
      }
    }
  }
  portEXIT_CRITICAL_ISR(&buttonMux);
}

// Helper function to set LED brightness (0-255 on ESP32, digital on/off on ESP8266)
void set_led_brightness(int brightness) {
  LD("[LED] Setting brightness to %d", brightness);
  if (led_pwm_enabled) {
    LD("[LED] Using PWM to set brightness");
    // Use hardware PWM for smooth brightness control with channel-based API
    if(!ledcWriteChannel(LED_PWM_CHANNEL, brightness)){
      LW("[LED] PWM write failed, using digital control");
      digitalWrite(LED, brightness > LED_BRIGHTNESS_LOW ? HIGH : LOW);
    } else {
      LD("[LED] PWM write successful");
    }
  } else {
    LD("[LED] PWM not enabled, using digital control");
    // PWM failed, fallback to digital control
    digitalWrite(LED, brightness > LED_BRIGHTNESS_LOW ? HIGH : LOW);
  }
}

void led_on(){
  L("[LED] Turning LED ON");
  set_led_brightness(led_brightness_on);
  led_state = true;
}

void led_off(){
  L("[LED] Turning LED OFF");
  set_led_brightness(led_brightness_off);
  led_state = false;
}

void setup_led(){
  L("[LED] Starting LED setup");

  // Initial pin setup
  pinMode(LED, OUTPUT);
  L("[LED] Pin %d configured as OUTPUT", LED);

  // Button setup
  pinMode(BUTTON, INPUT_PULLUP);
  L("[BUTTON] Pin %d configured as INPUT_PULLUP", BUTTON);

  // Attach button interrupt
  attachInterrupt(digitalPinToInterrupt(BUTTON), buttonISR, FALLING);
  L("[BUTTON] Interrupt attached to pin %d on FALLING edge", BUTTON);

  // Initialize brightness levels
  current_brightness_index = 1; // Start with LED_BRIGHTNESS_LOW
  led_brightness_on = brightness_levels[current_brightness_index];
  led_brightness_off = LED_BRIGHTNESS_OFF;
  L("[LED] Initial brightness levels - ON: %d (index %d), OFF: %d",
    led_brightness_on, current_brightness_index, led_brightness_off);

  // ESP32 PWM setup
  ledc_clk_cfg_t t = ledcGetClockSource();
  L("[LED] LEDC clock source: %d", t);

  // Setup LED PWM channel
  if (ledcAttachChannel(LED, LED_PWM_FREQUENCY, LED_PWM_RESOLUTION, LED_PWM_CHANNEL)) {
    led_pwm_enabled = true;
    L("[LED] PWM control enabled on pin %d, channel %d, freq %d Hz, res %d bits",
      LED, LED_PWM_CHANNEL, LED_PWM_FREQUENCY, LED_PWM_RESOLUTION);
  } else {
    // Fallback to digital control if PWM setup fails
    led_pwm_enabled = false;
    pinMode(LED, OUTPUT);
    LE("[LED] PWM setup failed, using digital control on pin %d", LED);
  }

  // Start with LED on
  led_on();

  L("[LED] Setting up hardware timer for LED blinking");
  led_t = timerBegin(1000000);
  if(led_t == NULL){
    LE("[LED] Failed to initialize timer for LED");
  } else {
    L("[LED] Timer initialized successfully");
    timerAttachInterrupt(led_t, &ledBlinkTimer);
    L("[LED] Timer interrupt attached");
    timerAlarm(led_t, 1000 * 1000, true, 0);
    L("[LED] Timer alarm set to 1 second");
    timerWrite(led_t, 0);
    timerStart(led_t);
    L("[LED] Timer started");
    L("[LED] LED setup completed successfully");
  }
}

void setup(){
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize

  // Test basic Serial output first
  L("=== ESP32 TM Module Starting ===");
  L("Testing basic Serial output...");

  // Configure ESP-IDF logging levels
  esp_log_level_set("*", ESP_LOG_INFO);        // Global level
  esp_log_level_set(TAG, ESP_LOG_VERBOSE);     // Our module level

  // Test ESP-IDF logging
  ESP_LOGI(TAG, "Testing ESP-IDF logging...");

  L("[SETUP] Serial initialized at 115200 baud");
  L("[SETUP] ESP-IDF logging configured - global: INFO, %s: VERBOSE", TAG);

  // Show current log levels for diagnostics
  L("ESP-IDF log level for '*': %d\n", esp_log_level_get("*"));
  L("ESP-IDF log level for '%s': %d\n", TAG, esp_log_level_get(TAG));
  L("CONFIG_LOG_DEFAULT_LEVEL: %d\n", CONFIG_LOG_DEFAULT_LEVEL);
  L("LOG_LOCAL_LEVEL: %d\n", LOG_LOCAL_LEVEL);

  /*
  // ESP32c3 only has 2 timers, allocate 2, so we can verify PWM allocation failure
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT, // Select the default clock source
      .direction = GPTIMER_COUNT_UP,      // Counting direction is up
      .resolution_hz = 1 * 1000 * 1000,   // Resolution is 1 MHz, i.e., 1 tick equals 1 microsecond
  };
  L("[TIMER] Setting up system timer 1");
  gptimer_handle_t gptimer1 = NULL;
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer1));
  L("[TIMER] Setting up system timer 2");
  gptimer_handle_t gptimer2 = NULL;
  ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer2));
  */

  L("[SETUP] Starting system initialization");
  setup_led();
  L("[SETUP] System initialization completed");

  L("=== Setup Complete ===");
}

void loop(){
  // Handle button press logging (outside of ISR)
  static bool last_button_state = false;
  if (button_pressed && !last_button_state) {
    L("[BUTTON] Button pressed! New brightness level: %d (index %d)",
      brightness_levels[current_brightness_index], current_brightness_index);
    last_button_state = true;
    button_pressed = false; // Clear the flag
  } else if (!button_pressed) {
    last_button_state = false;
  }

  // Add some periodic logging to show the system is alive
  static unsigned long last_loop_log = 0;
  static int loop_counter = 0;
  unsigned long now = millis();

  if (now - last_loop_log > 3000) { // Log every 3 seconds for testing
    loop_counter++;

    // Log current system state
    L("[LOOP] System running - uptime: %lu ms, LED state: %s, PWM enabled: %s, Current brightness: %d (index %d)",
      now, led_state ? "ON" : "OFF", led_pwm_enabled ? "YES" : "NO",
      brightness_levels[current_brightness_index], current_brightness_index);

    // Log the timers in use
    if (led_t != NULL) {
      // get info about led_t->timer_handle
      uint32_t t_res = 0;
      esp_err_t r = gptimer_get_resolution(led_t->timer_handle, &t_res);
      if(r != ESP_OK){
        LW("[LOOP] Failed to get timer resolution: %d", r);
      }
      uint32_t t_cnt_1 = timerRead(led_t);
      uint64_t t_count = 0;
      r = gptimer_get_raw_count(led_t->timer_handle, &t_count);
      if(r != ESP_OK){
        LW("[LOOP] Failed to get timer info: %d", r);
      }
      L("[LOOP] LED timer handle: %p, resolution: %u, get_resolution result: %d, cnt1: %d, cnt2: %d", led_t->timer_handle, t_res, r, t_cnt_1, t_count);
    } else {
      LW("[LOOP] LED timer is NULL");
    }

    // Log button state
    L("[LOOP] Button state: %s", digitalRead(BUTTON) == LOW ? "PRESSED" : "RELEASED");

    // Test different log levels
    if (loop_counter % 4 == 0) LE("[LOOP] Error level test");
    if (loop_counter % 3 == 0) LW("[LOOP] Warning level test");
    if (loop_counter % 2 == 0) LD("[LOOP] Debug level test");

    last_loop_log = now;
  }

  yield();
}
