#include <esphome/core/log.h>
#include <algorithm>
#if !defined(ARDUINO)
#include <unistd.h>
#include <sys/ioctl.h>
#endif

#include "modbus_ble_bridge.h"
#include "modbus_types.h"

#define BLE_SERVICE 0xFFFF
#define BLE_CHAR_READ 0xFF02
#define BLE_CHAR_WRITE 0xFF01

namespace esphome {
namespace modbus_ble_bridge {

static const char *const TAG = "modbus_ble_bridge";

void ModbusBleBridge::setup() {
  ESP_LOGI(TAG, "Setting up Modbus BLE Bridge");
  this->total_calls_ = 0;
  this->total_errors_ = 0;
}

void ModbusBleBridge::loop() {
  uint64_t now = millis();
  if (!esphome::network::is_connected()) {
    if (now - this->last_wifi_check_ >= 5000) {
      ESP_LOGW(TAG, "WiFi not connected");
      this->last_wifi_check_ = now;
    }
    return;
  }

  startModbusTCPServer();
  checkBLETimeout();  // Existing client will be disconnected if BLE timeout occurs
  handleTCPConnection();  // If there is no client, it will accept a new one
  handleModbusTCP();
}

void ModbusBleBridge::startModbusTCPServer() {
  #if defined(ARDUINO)
  if (!mb_server_) {
    mb_server_ = new WiFiServer(modbus_port_);
    mb_server_->begin();
    ESP_LOGI(TAG, "Modbus TCP server started on port %d", modbus_port_);
  }
  #else
  if (server_fd_ < 0) {
    server_fd_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (server_fd_ < 0) {
      ESP_LOGE(TAG, "socket() failed: errno=%d", errno);
      return;
    }
    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(modbus_port_);
    if (bind(server_fd_, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
      ESP_LOGE(TAG, "bind() failed: errno=%d", errno);
      ::close(server_fd_);
      server_fd_ = -1;
      return;
    }
    if (listen(server_fd_, 1) != 0) {
      ESP_LOGE(TAG, "listen() failed: errno=%d", errno);
      ::close(server_fd_);
      server_fd_ = -1;
      return;
    }
    int flags = fcntl(server_fd_, F_GETFL, 0);
    fcntl(server_fd_, F_SETFL, flags | O_NONBLOCK);
    ESP_LOGI(TAG, "Modbus TCP server (ESP-IDF) started on port %d", modbus_port_);
  }
  #endif
}

bool ModbusBleBridge::checkBLETimeout() {
  if (waiting_ble_response && waiting_since_ != 0 &&
    (millis() - waiting_since_) > ble_response_timeout_ms_) {

    total_errors_++;
    ESP_LOGW(TAG, "BLE response timeout after %u ms (loop). Resetting state.", ble_response_timeout_ms_);
    waiting_ble_response = false;
    waiting_since_ = 0;
    ESP_LOGW(TAG, "Closing TCP client due to BLE timeout (loop)");
  #if defined(ARDUINO)
    if (client_ && client_.connected())
      client_.stop();
  #else
    if (client_fd_ >= 0) {
      ::close(client_fd_);
      client_fd_ = -1;
    }
  #endif
    return true;
  }

  return false;
}

void ModbusBleBridge::handleTCPConnection() {
  #if defined(ARDUINO)
  if (!this->client_ || !this->client_.connected()) {
    this->client_ = this->mb_server_->accept();
    if (!this->client_)
      return;
    ESP_LOGI(TAG, "Modbus TCP client connected");
  }
  #else
  if (this->client_fd_ < 0) {
    sockaddr_in caddr{};
    socklen_t clen = sizeof(caddr);
    int fd = ::accept(this->server_fd_, (struct sockaddr*)&caddr, &clen);
    if (fd < 0) return;
    int cflags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, cflags | O_NONBLOCK);
    this->client_fd_ = fd;
    ESP_LOGI(TAG, "Modbus TCP client connected (ESP-IDF)");
  }
  #endif
}

void ModbusBleBridge::handleModbusTCP() {
  int avail = 0;
  #if defined(ARDUINO)
  avail = this->client_.available();
  #else
  if (this->client_fd_ >= 0) {
    uint8_t tmp[1];
    int recv_bytes = ::recv(this->client_fd_, reinterpret_cast<char*>(tmp), 1, MSG_PEEK | MSG_DONTWAIT);
    if (recv_bytes > 0) {
      int bytes = 0;
      if (ioctl(this->client_fd_, FIONREAD, &bytes) == 0)
        avail = bytes;
      else
        avail = recv_bytes;
    } else if (recv_bytes == 0) {
      ESP_LOGI(TAG, "TCP client closed by peer");
      ::close(this->client_fd_);
      this->client_fd_ = -1;
      return;
    } else {
      avail = 0;
    }
  }
  #endif

  if (!avail)
    return;

  if (this->waiting_ble_response) {
    checkBLETimeout();
    return;  // If still waiting for BLE response, we don't read the client data
  }

  ESP_LOGD(TAG, "handleModbusTCP: available bytes=%d", avail);
  std::vector<uint8_t> modbus_request_v;

  #if defined(ARDUINO)
  while (this->client_.available())
    modbus_request_v.push_back(this->client_.read());

  #else
  if (this->client_fd_ >= 0) {
    int recv_bytes = ::recv(this->client_fd_, reinterpret_cast<char*>(modbus_request_v.data()), 260, MSG_DONTWAIT);
    if (recv_bytes <= 0)
      return;
  }
  #endif

  ESP_LOGD(TAG, "Read %d bytes from Modbus TCP client", modbus_request_v.size());
  if (modbus_request_v.size() < 12) {
    ESP_LOGW(TAG, "Incomplete Modbus TCP frame: %d bytes, aborting", modbus_request_v.size());
    return;
  }
  #if defined(ARDUINO)
  this->client_.clear();
  #endif

  modbus_tcp_request = modbus_saj::ModbusTCPRequest(modbus_request_v);
  modbus_saj::ModbusBLERequest ble_req(modbus_tcp_request);

  ESP_LOGD(TAG, "Sending BLE request (seq=%d)", ble_req.getBLETransactionId());
  this->sendBLERequest(ble_req);
  this->total_calls_++;
  ESP_LOGD(TAG, "BLE request sent, next seq=%d, total_calls=%d", ble_req.getBLETransactionId(), this->total_calls_);
  this->waiting_ble_response = true;
  this->waiting_since_ = millis();
}

void ModbusBleBridge::sendBLERequest(const modbus_saj::ModbusBLERequest &request) {
  ESP_LOGD(TAG, "sendBLERequest invoked");

  if (!this->parent_ || !this->parent_->connected()) {
    ESP_LOGW(TAG, "BLE not connected when attempting to send request");
    return;
  }

  esp32_ble_client::BLECharacteristic *char_write = this->parent_->get_characteristic(BLE_SERVICE, BLE_CHAR_WRITE);
  if (!char_write) {
    ESP_LOGE(TAG, "BLE write characteristic not found");
    return;
  }
  ESP_LOGD(TAG, "BLE write characteristic acquired");

  std::vector<uint8_t> request_frame = request.toBytes();
  char_write->write_value(request_frame.data(), request_frame.size(), ESP_GATT_WRITE_TYPE_NO_RSP);
  ESP_LOGD(TAG, "BLE request sent successfully");
}

void ModbusBleBridge::dump_config() {
  ESP_LOGCONFIG(TAG, "Modbus BLE Bridge:");
  ESP_LOGCONFIG(TAG, "  Port: %u", this->modbus_port_);
  ESP_LOGCONFIG(TAG, "  BLE response timeout: %u ms", this->ble_response_timeout_ms_);
}

void ModbusBleBridge::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                          esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT: {
      ESP_LOGD(TAG, "[gattc_event_handler] ESP_GATTC_OPEN_EVT");
      if (param->open.status == ESP_GATT_OK) {
        ESP_LOGI(TAG, "BLE device connected successfully!");
        esp_err_t mtu_err = esp_ble_gattc_send_mtu_req(gattc_if, param->open.conn_id);
        if (mtu_err)
          ESP_LOGW(TAG, "MTU request failed, status=%d", mtu_err);
      } else {
        ESP_LOGW(TAG, "Connection failed, status=%d", param->open.status);
      }
      break;
    }
    case ESP_GATTC_CFG_MTU_EVT: {
      ESP_LOGD(TAG, "[gattc_event_handler] ESP_GATTC_CFG_MTU_EVT: MTU updated to %d", param->cfg_mtu.mtu);
      break;
    }
    case ESP_GATTC_DISCONNECT_EVT: {
      ESP_LOGD(TAG, "[gattc_event_handler] ESP_GATTC_DISCONNECT_EVT");
      ESP_LOGI(TAG, "BLE device disconnected");
      this->char_read_ = nullptr;
      this->waiting_ble_response = false;
      this->waiting_since_ = 0;
  #if defined(ARDUINO)
      if (this->client_ && this->client_.connected()) this->client_.stop();
  #else
      if (this->client_fd_ >= 0) {
        ::close(this->client_fd_);
        this->client_fd_ = -1;
      }
  #endif
      break;
    }
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      ESP_LOGD(TAG, "[gattc_event_handler] ESP_GATTC_SEARCH_CMPL_EVT");
      this->char_read_ = this->parent_->get_characteristic(BLE_SERVICE, BLE_CHAR_READ);
      if (this->char_read_ == nullptr) {
        ESP_LOGE(TAG, "Read characteristic not found");
        break;
      }
      ESP_LOGD(TAG, "Found read characteristic");
      auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                      this->char_read_->handle);

      if (status) {
        ESP_LOGW(TAG, "esp_ble_gattc_register_for_notify failed, status=%d", status);
      }
      break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      ESP_LOGD(TAG, "[gattc_event_handler] ESP_GATTC_REG_FOR_NOTIFY_EVT");
      break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
      bool client_ok = false;
  #if defined(ARDUINO)
      client_ok = this->client_.connected();
  #else
      client_ok = this->client_fd_ >= 0;
  #endif
      if (param->notify.handle != this->char_read_->handle || !client_ok) {
        break;
      }
      this->waiting_ble_response = false;
      this->waiting_since_ = 0;
      const uint8_t *pData = param->notify.value;
      size_t length = param->notify.value_len;
      ESP_LOGI(TAG, "BLE notify received");
      ESP_LOGD(
        TAG, "BLE notify: length=%d, expected=%d, registers=%d",
        length, modbus_tcp_request.getNumberOfRegisters() + 7, modbus_tcp_request.getNumberOfRegisters());

      if (modbus_tcp_request.getFunctionCode() == 6) {
        ESP_LOGI(TAG, "Write command response - flushing client. Closing TCP client after write single register response");
  #if defined(ARDUINO)
        this->client_.stop();
  #else
        if (this->client_fd_ >= 0) {
          ::close(this->client_fd_);
          this->client_fd_ = -1;
        }
  #endif
        break;
      }
      if ((static_cast<int>(length) - 7) != modbus_tcp_request.getNumberOfRegisters()) {
        this->errlen_++;
        ESP_LOGW(
          TAG, "Wrong response length (error %d): expected %d registers, got %d bytes",
          this->errlen_, modbus_tcp_request.getNumberOfRegisters(), (int)length - 7);

        if (this->errlen_ > 5) {
          ESP_LOGE(TAG, "Too many length errors (5) - resetting connection");
          ESP_LOGE(TAG, "Closing TCP client due to repeated length errors");
  #if defined(ARDUINO)
          this->client_.stop();
  #else
          if (this->client_fd_ >= 0) {
            ::close(this->client_fd_);
            this->client_fd_ = -1;
          }
  #endif
        }
        break;
      }
      ESP_LOGD(TAG, "Received correct BLE response length");
      this->errlen_ = 0;

      modbus_saj::ModbusBLEResponse ble_resp(pData, length);
      std::vector<uint8_t> data = ble_resp.getData();
      ESP_LOGD(TAG, "Response byte count: %d", data.size() - 1);

      const uint8_t *data_ptr = data.data();

      modbus_saj::ModbusTCPResponse modbus_resp(modbus_tcp_request, ble_resp);
      std::vector<uint8_t> modbus_resp_bytes = modbus_resp.toBytes();
      ESP_LOGI(TAG, "Sending Modbus/TCP response of %d bytes", modbus_resp_bytes.size());

  #if defined(ARDUINO)
      this->client_.write(modbus_resp_bytes.data(), modbus_resp_bytes.size());
      this->client_.stop();
      ESP_LOGD(TAG, "Closed TCP client after sending Modbus/TCP response");
  #else
      if (this->client_fd_ >= 0) {
        ::send(this->client_fd_, modbus_resp_bytes.data(), modbus_resp_bytes.size(), 0);
        ::close(this->client_fd_);
        this->client_fd_ = -1;
        ESP_LOGD(TAG, "Closed TCP client after sending Modbus/TCP response (ESP-IDF)");
      }
  #endif
      break;
    }
    default:
      break;
  }
}
}  // namespace modbus_ble_bridge
}  // namespace esphome
