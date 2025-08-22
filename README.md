# Saj H1-*-S2
This is a ESPHome component that allows to serve Modbus over TCP. Is a bridge between Modbus TCP and BLE. Based on the work on this repo: https://github.com/sgsancho/saj_h1_s2_modbus_esp32

Designed for the SAJ H1-*-S2 with the Bluetooth dongle that you can't connect to any API locally besides Bluetooth.

# Where does it work
Tested on ESPHome with the arduino and esp-idf frameworks
Tested on ESPHome 2025.6.3 and 2025.7.5
NOT tested that using this component breaks any other BLE feature. Provide feedback on the issues.

# How to add to your ESPHome:

### 1. Add the external component

````
external_components:
  - source: github://pr#10368
    components: [logger]
    refresh: 1h
````

### 2. Configure a BLE client to connect the SAJ dongle

````
ble_client:
  - mac_address: !secret ble_device_mac
    id: saj_ble

esp32_ble_tracker:
  scan_parameters:
    interval: 3500ms
    window: 1100ms
    active: true
````

### 3. Configure the component

````
modbus_ble_bridge:
  ble_client_id: saj_ble
  modbus_port: 502
````

### 4. Configure a Modbus client or use NodeRed to fetch the data

NodeRed example flow: TODO