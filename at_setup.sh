#!/bin/bash

function usage(){
    echo "Usage: $BASH_SOURCE <ble address> <ssid> <password> <remote_host> [remote_port] <metrics KEY>" >>/dev/stderr
    echo "Example: $BASH_SOURCE ec:da:3b:bf:94:02 MySSID MyPassword 2001:db8::1 5665 homeoffice"         >>/dev/stderr
    echo "$*"
    exit 1
}
BLE_ADDR=${1:-${BLE_ADDR?$(usage "Error: BLE_ADDR device is not specified.")}}
SSID=${2:-${SSID?$(usage "Error: SSID is not specified.")}}
PASS=${3:-${PASS?$(usage "Error: Password is not specified.")}}
R_HOST=${4:-${R_HOST?$(usage "Error: Remote host is not specified.")}}
R_PORT=${5:-5775}

cat <<EObleuart | perl ble_uart.pl ${BLE_ADDR}
AT+VERBOSE=0
AT+IPV4=dhcp
AT+IPV6=dhcp
AT+WIFI_SSID=$SSID
AT+WIFI_PASS=$PASS
AT+UDP_PORT=$R_PORT
AT+UDP_HOST_IP=$R_HOST
AT+VERBOSE=0
EObleuart
