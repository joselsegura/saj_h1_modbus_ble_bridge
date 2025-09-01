#pragma once
#include <esphome.h>
#include <esphome/components/ble_client/ble_client.h>
#include <esphome/components/network/util.h>
#if defined(ARDUINO)
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#else
#include <lwip/sockets.h>
#include <lwip/inet.h>
#include <fcntl.h>
#include <errno.h>
#endif
#include <vector>

#include "modbus_types.h"

namespace esphome {
namespace modbus_ble_bridge {

class ModbusBleBridge : public Component, public ble_client::BLEClientNode {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

  // ESPHome automatic config methods
  void set_modbus_port(uint16_t port) { modbus_port_ = port; }
  void set_ble_response_timeout(uint32_t ms) { ble_response_timeout_ms_ = ms; }

 private:
  uint16_t modbus_port_ = 502;
  uint32_t ble_response_timeout_ms_ = 2000;

  #if defined(ARDUINO)
  WiFiServer *mb_server_ = nullptr;
  WiFiClient client_;
  #else
  int server_fd_ = -1;
  int client_fd_ = -1;
  #endif

  modbus_saj::ModbusTCPRequest modbus_tcp_request;
  int errlen_ = 0;
  int total_calls_ = 0;
  int total_errors_ = 0;
  uint64_t last_wifi_check_ = 0;
  bool waiting_ble_response = false;
  uint64_t waiting_since_ = 0;

  esp32_ble_client::BLECharacteristic *char_read_ = nullptr;

  void startModbusTCPServer();
  bool checkBLETimeout();
  void handleTCPConnection();
  void handleModbusTCP();
  void sendBLERequest(const modbus_saj::ModbusBLERequest &request);
};

}  // namespace modbus_ble_bridge
}  // namespace esphome
