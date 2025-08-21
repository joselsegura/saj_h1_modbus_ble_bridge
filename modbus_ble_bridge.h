#pragma once
#include "esphome.h"
#include "esphome/components/ble_client/ble_client.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <vector>

namespace esphome {
namespace modbus_ble_bridge {

class ModbusBleBridge : public Component, public ble_client::BLEClientNode {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;

  void set_modbus_port(uint16_t port) { modbus_port_ = port; }

 protected:
  // Config
  uint16_t modbus_port_ = 502;

  // BLE UUIDs (fixed)
  static constexpr const char *BLE_SERVICE_UUID = "0000ffff-0000-1000-8000-00805f9b34fb";
  static constexpr const char *BLE_CHAR_READ_UUID = "0000ff02-0000-1000-8000-00805f9b34fb";
  static constexpr const char *BLE_CHAR_WRITE_UUID = "0000ff01-0000-1000-8000-00805f9b34fb";

  // TCP/Modbus state
  WiFiServer *mb_server_ = nullptr;
  WiFiClient client_;
  std::vector<uint8_t> modbus_request_;
  std::vector<uint8_t> modbus_frame_response_;
  int total_registers_ = 0;
  // Number of registers requested over Modbus/TCP (for assembling BLE response)
  uint16_t requested_regs_{0};
  int errlen_ = 0;
  int c_ = 0;
  int total_calls_ = 0;
  int total_errors_ = 0;
  unsigned long last_wifi_check_ = 0;
  unsigned long last_ble_msg_ = 0;
  bool waiting_message_ = false;

  // BLE state
  esp32_ble_client::BLECharacteristic *char_read_ = nullptr;
  esp32_ble_client::BLECharacteristic *char_write_ = nullptr;
  // Buffer for RTU bytes extracted from BLE notifications
  std::vector<uint8_t> ble_response_buffer_;
  // Expected RTU frame length (unit+fn+byte_count+data+CRC)
  size_t expected_frame_len_ = 0;

  // Helpers
  void handle_modbus_tcp();
  void handle_ble_notify(const std::vector<uint8_t> &data);
  void send_ble_request(const std::vector<uint8_t> &request);
  uint16_t modrtu_crc(const uint8_t *buf, int len);
};

}  // namespace modbus_ble_bridge
}  // namespace esphome
