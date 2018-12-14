# esp32-ble-custom-serial

A custom BLE service that can be used to implement a simple serial interface.

## Design

This code implements a single BLE GATT Service on the ESP32. The service
contains a Characteristic that can a client, such as a mobile phone, can read
and write to. Additionally, there is a characteristic which depending on how
the code is compiled, can enable indication or notification.

## Requirements

An ESP32 toolchain is required to build the firmware. The project is configured
to find it located in the /opt directory (/opt/xtensa-esp32-elf), but the path
can be altered by running 'make menuconfig'.

## Configuration

Developers using this code should select UUIDs that are unique to their end
application. A Python script "generate-uuid" is included in this repo that can
assist in this regard. The UUIDs to be used should be placed in the
`ble_server_config.c` source file.

Other compile time configuration options and their documentation can be found
in the `ble_server_config.h` header file.
