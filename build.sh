#!/bin/bash
#
# build.sh - Arduino CLI wrapper for ESP32/ESP8266 AT firmware
#
# This script provides a convenient wrapper around arduino-cli for building,
# uploading, and monitoring ESP-AT firmware projects. It handles platform
# setup, library installation, and configurable build options through
# environment variables.
#

HERE=$(dirname $(readlink -f $BASH_SOURCE))
MODULE=${MODULE:-esp-at}
DEV_PLATFORM=${DEV_PLATFORM:-esp32:esp32}
DEV_BOARD=${DEV_BOARD:-esp32:esp32:esp32c3}
DEV_PORT=${DEV_PORT:-/dev/ttyACM0}
DEV_BOARD_BAUDRATE=${DEV_BOARD_BAUDRATE:-460800}
export ARDUINO_DIRECTORIES_DATA=${ARDUINO_DIRECTORIES_DATA:-$HERE/.arduino15}
export TMPDIR=/var/tmp

function do_update(){
    DEV_URLS=${DEV_URLS:-https://dl.espressif.com/dl/package_esp32_index.json}
    [ "${DEV_UPDATE:-0}" = 1 ] && {
        arduino-cli core install $DEV_PLATFORM
        arduino-cli --additional-urls "$DEV_URLS" update
        arduino-cli --additional-urls "$DEV_URLS" core install "${DEV_PLATFORM}"
        arduino-cli --additional-urls "$DEV_URLS" lib update-index
        arduino-cli --additional-urls "$DEV_URLS" lib install 'SerialCommands'
        arduino-cli --additional-urls "$DEV_URLS" lib upgrade
        arduino-cli --additional-urls "$DEV_URLS" board list
    }
}

function do_build(){
    DEV_EXTRA_FLAGS="-DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=1 -D_ARDUINO_BLE_H_"
    if [ ! -z "${DEBUG}" -a "${DEBUG:-0}" = "1" ]; then
        DEV_EXTRA_FLAGS="$DEV_EXTRA_FLAGS -DDEBUG"
    fi
    if [ ! -z "${VERBOSE}" ]; then
        DEV_EXTRA_FLAGS="$DEV_EXTRA_FLAGS -DVERBOSE"
    fi
    if [ ! -z "${DEFAULT_NTP_SERVER}" ]; then
        DEV_EXTRA_FLAGS="$DEV_EXTRA_FLAGS -DDEFAULT_NTP_SERVER=\"${DEFAULT_NTP_SERVER}\""
    fi
    if [ ! -z "${DEFAULT_HOSTNAME}" ]; then
        DEV_EXTRA_FLAGS="$DEV_EXTRA_FLAGS -DDEFAULT_HOSTNAME=\"${DEFAULT_HOSTNAME}\""
    fi
    if [ ! -z "${DEFAULT_BLUETOOTH_NAME}" ]; then
        DEV_EXTRA_FLAGS="$DEV_EXTRA_FLAGS -DDEFAULT_BLUETOOTH_NAME=\"${DEFAULT_BLUETOOTH_NAME}\""
    fi
    if [ ! -z "${DEFAULT_BLUETOOTH_PIN}" ]; then
        DEV_EXTRA_FLAGS="$DEV_EXTRA_FLAGS -DDEFAULT_BLUETOOTH_PIN=\"${DEFAULT_BLUETOOTH_PIN}\""
    fi
    if [ ! -z "${CUSTOM_FLAGS}" ]; then
        DEV_EXTRA_FLAGS="$DEV_EXTRA_FLAGS ${CUSTOM_FLAGS}"
    fi
    arduino-cli -b ${DEV_BOARD} compile \
        --log \
        --log-level info \
        --output-dir dist \
        --build-property compiler.cpp.extra_flags="$DEV_EXTRA_FLAGS" \
        --build-property compiler.c.extra_flags="$DEV_EXTRA_FLAGS" \
        --build-property build.extra_flags="$DEV_EXTRA_FLAGS" \
        --build-property build.partitions=min_spiffs \
        --build-property upload.maximum_size=2097152 \
        --board-options PartitionScheme=no_ota \
        $MODULE \
        || exit $?
}

function do_upload(){
    arduino-cli -b ${DEV_BOARD} upload -p ${DEV_PORT} $MODULE
}

function do_monitor(){
    arduino-cli -b ${DEV_BOARD} monitor -p ${DEV_PORT} -c baudrate=${DEV_BOARD_BAUDRATE}
}

function show_usage(){
    echo "Usage: $0 [command]"
    echo ""
    echo "Commands:"
    echo "  build|compile   Update platform/libs and build the project"
    echo "  update          Update platform/libs only (no build)"
    echo "  upload          Upload the compiled firmware to the device"
    echo "  monitor         Open a serial monitor to the device"
    echo "  deploy          Build and upload the firmware"
    echo "  all             Update, build, upload, and monitor (legacy default)"
    echo "  help            Show this usage information"
    echo ""
    echo "Environment variables:"
    echo "  MODULE                    Arduino project directory (default: esp-at)"
    echo "  DEV_PLATFORM              Arduino platform (default: esp32:esp32)"
    echo "  DEV_BOARD                 Board identifier (default: esp32:esp32:esp32c3)"
    echo "  DEV_PORT                  Serial port (default: /dev/ttyACM0)"
    echo "  DEV_BOARD_BAUDRATE        Monitor baudrate (default: 460800)"
    echo "  DEBUG                     Set to 1 to add -DDEBUG flag"
    echo "  VERBOSE                   Set to any value to add -DVERBOSE flag"
    echo "  DEFAULT_HOSTNAME          Override default hostname"
    echo "  DEFAULT_BLUETOOTH_NAME    Override default Bluetooth name"
    echo "  DEFAULT_BLUETOOTH_PIN     Override default Bluetooth PIN"
    echo "  DEFAULT_NTP_SERVER        Override default NTP server"
    echo ""
    echo "Examples:"
    echo "  $0 build"
    echo "  DEV_PORT=/dev/ttyUSB0 $0 deploy"
    echo "  DEBUG=1 DEFAULT_HOSTNAME=myesp $0 build"
}

case $1 in
    deploy)
        do_build
        do_upload
        ;;
    upload)
        do_upload
        ;;
    monitor)
        do_monitor
        ;;
    build|compile)
        DEV_UPDATE=0 do_update
        do_build
        ;;
    update)
        DEV_UPDATE=1 do_update
        ;;
    all)
        DEV_UPDATE=1 do_update
        do_build
        do_upload
        do_monitor
        ;;
    help|--help|-h|"")
        show_usage
        ;;
    *)
        echo "Unknown command: $1"
        echo ""
        show_usage
        exit 1
        ;;
esac
