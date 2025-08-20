### Supported #define Options and Capabilities

The firmware supports the following compile-time options and capabilities (set via `#define` or build flags):

- `VERBOSE` (default enabled): Enables verbose logging.
- `DEBUG`: Enables debug logging.
- `DEFAULT_HOSTNAME` (default: "uart"): Default device hostname.
- `UART_AT` (default enabled): Enables UART AT command interface.
- `BLUETOOTH_UART_AT` (default enabled): Enables Bluetooth UART AT command interface.
- `BLUETOOTH_UART_DEVICE_NAME` (default: `DEFAULT_HOSTNAME`): Bluetooth device name.
- `BLUETOOTH_UART_DEFAULT_PIN` (default: "1234"): Bluetooth pairing PIN (Classic BT).
- `DEFAULT_NTP_SERVER` (default: "at.pool.ntp.org"): Default NTP server.
- `DEFAULT_DNS_IPV4` (default: "1.1.1.1"): Default DNS for static IPv4.
- `SUPPORT_NTP` (default enabled): Enables NTP support.
- `SUPPORT_UDP` / `SUPPORT_TCP`: Enables UDP/TCP forwarding (if defined).

### Supported AT Commands

The following AT commands are supported via UART, BLE, or Bluetooth (if enabled):

- `AT` / `AT?` — Test if device is responsive.
- `AT+WIFI_SSID=<ssid>` / `AT+WIFI_SSID?` — Set/query WiFi SSID.
- `AT+WIFI_PASS=<pass>` — Set WiFi password.
- `AT+WIFI_STATUS?` — Query WiFi connection status.
- `AT+VERBOSE=1|0` / `AT+VERBOSE?` — Enable/disable/query verbose logging.
- `AT+LOG_UART=1|0` / `AT+LOG_UART?` — Enable/disable/query UART logging.
- `AT+NTP_HOST=<host>` / `AT+NTP_HOST?` — Set/query NTP server.
- `AT+NTP_STATUS?` — Query NTP sync status.
- `AT+UDP_PORT=<port>` / `AT+UDP_PORT?` — Set/query UDP port.
- `AT+UDP_HOST_IP=<ip>` / `AT+UDP_HOST_IP?` — Set/query UDP host IP.
- `AT+TCP_PORT=<port>` / `AT+TCP_PORT?` — Set/query TCP port.
- `AT+TCP_HOST_IP=<ip>` / `AT+TCP_HOST_IP?` — Set/query TCP host IP.
- `AT+LOOP_DELAY=<ms>` / `AT+LOOP_DELAY?` — Set/query main loop delay.
- `AT+HOSTNAME=<name>` / `AT+HOSTNAME?` — Set/query device hostname.
- `AT+IPV4=DHCP|DISABLE|ip,netmask,gateway[,dns]` / `AT+IPV4?` — Set/query IPv4 config.
- `AT+IPV6=DHCP|DISABLE` / `AT+IPV6?` — Set/query IPv6 config.
- `AT+IP_STATUS?` — Query current IP status.
- `AT+RESET` — Reset the device.
## esp-at-template

This project provides a template for building and uploading ESP-AT firmware using Arduino CLI. It includes a flexible `build.sh` script for managing builds, uploads, and serial monitoring, with support for environment variable overrides and custom configuration. The template is designed to simplify development and deployment for ESP32-based AT firmware projects.

### build.sh Commands

The `build.sh` script supports the following commands (pass as the first argument):

- `build` or `compile`: Update platform/libs and build the project.
- `update`: Update platform/libs only (no build).
- `upload`: Upload the compiled firmware to the device.
- `monitor`: Open a serial monitor to the device.
- `deploy`: Build and upload the firmware.
- *(no argument or unknown command)*: Update, build, upload, and monitor in sequence.

Example:

```
bash ./build.sh build
bash ./build.sh upload
bash ./build.sh monitor
```

## BUILD


### Environment Variables for `build.sh`

You can customize the build and upload process using the following environment variables:

- `MODULE` (default: `esp-at`): The Arduino project directory or sketch name.
- `DEV_PLATFORM` (default: `esp32:esp32`): Arduino platform to use.
- `DEV_BOARD` (default: `esp32:esp32:esp32c3`): Board identifier for arduino-cli.
- `DEV_PORT` (default: `/dev/ttyACM0`): Serial port for upload/monitor.
- `DEV_BOARD_BAUDRATE` (default: `460800`): Baudrate for monitor.
- `ARDUINO_DIRECTORIES_DATA` (default: `$HERE/.arduino15`): Arduino data directory.
- `TMPDIR` (default: `/var/tmp`): Temporary directory.
- `DEV_URLS` (default: `https://dl.espressif.com/dl/package_esp32_index.json`): Additional URLs for board manager.
- `DEV_UPDATE` (default: `0`): Set to `1` to update platform/libs.
- `DEBUG`: Set to `1` to add `-DDEBUG` flag.
- `VERBOSE`: Set to any value to add `-DVERBOSE` flag.
- `DEFAULT_NTP_SERVER`: Set to override default NTP server.
- `DEFAULT_HOSTNAME`: Set to override default hostname.
- `DEFAULT_BLUETOOTH_NAME`: Set to override default Bluetooth name.
- `DEFAULT_BLUETOOTH_PIN`: Set to override default Bluetooth PIN.

Example usage:

```
DEV_PORT=/dev/ttyUSB0 DEFAULT_HOSTNAME=myesp DEBUG=1 bash ./build.sh build
```

First update platform/modules:
```
DEV_PORT=/dev/ttyUSB0 bash ./build.sh update
```

Then build:
```
DEV_PORT=/dev/ttyUSB0 bash ./build.sh build
```
