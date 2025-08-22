# NAME

ble\_uart.pl - BLE UART (Nordic UART Service) bridge in Perl

# SYNOPSIS

**ble\_uart.pl** **\[**OPTIONS**\]** \[XX:XX:XX:XX:XX:XX\[,option=value ...\]\[,...\]\] 

# DESCRIPTION

This script connects to one or more BLE devices implementing the Nordic UART
Service (NUS), discovers the UART RX/TX characteristics, and allows simple
UART-style read/write over BLE. It is intended for use with ESP32/ESP-AT or
similar BLE UART bridges.

Input from the terminal is sent to the BLE device, and data received from the
device is printed to the terminal with a distinct prompt and color. Multiple
connections can be managed interactively.

# NORDIC UART SERVICE (NUS)

This script communicates with BLE devices that implement the Nordic UART Service
(NUS), a standard BLE service for serial-style data transfer over Bluetooth Low
Energy. NUS is commonly used on Nordic Semiconductor devices (such as
nRF52/nRF53 series) and is supported by many ESP32 BLE UART firmware projects,
including ESP-AT.

NUS provides a simple way to send and receive data as if over a UART/serial
port, but using BLE characteristics for RX (write) and TX (notify). This allows
wireless, bidirectional, low-latency communication between a host (like this
script) and a BLE device.

## NUS Service and Characteristics

The Nordic UART Service uses the following UUIDs:

- Service UUID: `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
- TX Characteristic (notify, device to client): `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
- RX Characteristic (write, client to device): `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`

The host writes data to the RX characteristic, and receives notifications from
the TX characteristic.

For more information, see the official Nordic documentation:

[https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/libraries/bluetooth/services/nus.html](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/libraries/bluetooth/services/nus.html)

# ARGUMENTS

The script expects one or more Bluetooth addresses of BLE devices implementing
the Nordic UART Service (NUS). Additional options like \`uart\_at=0\` can be
specified to disable UART AT command mode.

The format is:

    XX:XX:XX:XX:XX:XX[,option=value][,...]

Where:

- XX:XX:XX:XX:XX:XX

    The Bluetooth address of the device to connect to.

- option=value

    An optional configuration option

    - **uart\_at=0|1**

        disable UART AT command mode. If set to 0, the script will not
        send \`AT\` commands and will not expect \`OK\` responses. Default is 1 (enabled).

    - **bt\_listen\_addr=BDADDR\_ANY|BDADDR\_LOCAL|XX:XX:XX:XX:XX:XX**

        local Bluetooth address to listen on (default: BDADDR\_ANY).

# OPTIONS

## Command Line Options

- **--help**

    Show help

- **--man**

    Show full manual

- **--raw**, **-r**

    Enable raw mode - disables colored output, UTF-8 formatting, and fancy prompts.
    Also sets log level to NONE unless explicitly configured. Useful for scripting
    or when piping output.

## COMMANDS

The following commands can be entered at the prompt:

- **/connect XX:XX:XX:XX:XX:XX\[,option=value\]**

    Connect to a new BLE device by address, optionally key=value pairs for config

    Example: 

        /connect 12:34:56:78:9A:BC
        /connect 12:34:56:78:9A:BC,uart_at=0

- **/disconnect \[XX:XX:XX:XX:XX:XX\]**

    Disconnect all BLE connections, or only the specified BLE device if a Bluetooth
    address is given.

    Example:

        /disconnect
        /disconnect 12:34:56:78:9A:BC

- **/script &lt;file>**

    Execute commands from the specified file, one per line, as if entered at the
    prompt. Blank lines and lines starting with '#' are ignored.

    Example:

        /script mycommands.txt

- **/exit**, **/quit**

    Exit the program.

- **/debug on|off**

    Enable or disable debug logging (sets loglevel to DEBUG or INFO).

- **/logging on|off**

    Enable or disable info logging (sets loglevel to INFO or NONE).

- **/loglevel &lt;none|info|warn|error|debug>**

    Set the log level explicitly.

- **/help**

    Show this help message (list of / commands).

- **/usage**

    Show the usage.

- **/man**

    Show the manpage.

- **/switch <XX:XX:XX:XX:XX:XX>**

    Switch the active BLE device for terminal input/output to the specified
    connected device.

    Example:

        /switch 12:34:56:78:9A:BC

    If no address is given, lists all currently connected devices.

# ENVIRONMENT

The following environment variables affect the behavior of this script:

- **BLE\_UART\_DIR**

    Directory for history and config files (default: ~/.ble\_uart). Note that the
    defaulting is done via the HOME and LOGNAME environment variables.

- **BLE\_UART\_HISTORY\_FILE**

    History file location (default: ~/.ble\_uart\_history).

- **BLE\_UART\_RAW**

    Enable raw mode (default: 0). If set to 1, disables colored output, UTF-8
    formatting, and fancy prompts. Also sets log level to NONE unless explicitly
    configured.

- **BLE\_UART\_LOGLEVEL**

    Set the log level (default: info). Can be set to debug, info, error, or none.

- **BLE\_UART\_INTERACTIVE\_COLOR**

    Enable colored output (default: 1).

- **BLE\_UART\_INTERACTIVE\_UTF8**

    Enable UTF-8 output (default: 1).

- **BLE\_UART\_INTERACTIVE\_MULTILINE**

    Enable multiline input (default: 1).

- **BLE\_UART\_NUS\_SERVICE\_UUID**

    Override the Nordic UART Service UUID (default:
    6E400001-B5A3-F393-E0A9-E50E24DCCA9E).

- **BLE\_UART\_NUS\_RX\_CHAR\_UUID**

    Override the NUS RX Characteristic UUID for writing data to the device
    (default: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E).

- **BLE\_UART\_NUS\_TX\_CHAR\_UUID**

    Override the NUS TX Characteristic UUID for receiving notifications from the
    device (default: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E).

- **TERM**

    Terminal type, used to determine if colors are supported, if not set, "vt220"
    is assumed.

- **COLORTERM**

    This is checked for color support, if set to "truecolor" or "24bit", it will
    enable true color support. If not set, "truecolor" is assumed.

- **MANPAGER**

    Used for displaying the manpage, see the manpage of "man". This is usually
    "less". If not set, "less" is used.

# AUTHOR

CowboyTim

# LICENSE

This software is released under the Unlicense. See [https://unlicense.org](https://unlicense.org) for
details.
