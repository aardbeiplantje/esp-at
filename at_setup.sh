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
{
do_uart(){
    local cmd="$1"
    echo -ne "$cmd\n"
}
do_uart "AT+VERBOSE=0"
do_uart "AT+IPV4=dhcp"
do_uart "AT+IPV6=dhcp"
do_uart "AT+WIFI_SSID=$SSID"
do_uart "AT+WIFI_PASS=$PASS"
do_uart "AT+UDP_PORT=$R_PORT"
do_uart "AT+UDP_HOST_IP=$R_HOST"
do_uart "AT+VERBOSE=0"
}|BLE_UART_LOGLEVEL=DEBUG LANG=C perl ble_uart.pl =${BLE_ADDR},uart_at=1
