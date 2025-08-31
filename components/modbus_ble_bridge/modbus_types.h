#ifndef COMPONENTS_MODBUS_BLE_BRIDGE_MODBUS_TYPES_H_
#define COMPONENTS_MODBUS_BLE_BRIDGE_MODBUS_TYPES_H_

#include <array>
#include <cstdint>
#include <vector>

#include "esphome/core/log.h"

namespace modbus_saj {

static const char* const kModbusTypesTag = "modbus_types";

class Modbus {
 protected:
  // Modbus TCP header fields
  uint8_t transaction_identifier[2];
  uint8_t unit_id;
  uint8_t function_code;
  uint8_t address[2];
  uint8_t number_registers[2];

 public:
  // Getter methods for accessing parsed fields
  uint16_t getTransactionId() const;
  uint8_t getFunctionCode() const;
  uint8_t getUnitId() const;
  uint16_t getAddress() const;
  uint16_t getNumberOfRegisters() const;

  // Get raw byte arrays (by reference - no copying!)
  const uint8_t* getTransactionIdentifierBytes() const;
  const uint8_t* getAddressBytes() const;
  const uint8_t* getNumberRegistersBytes() const;
};

class ModbusTCPRequest : public Modbus {
 protected:
  // Modbus TCP header fields
  uint8_t protocol_identifier[2];
  uint8_t length[2];

 public:
  // Default constructor
  ModbusTCPRequest() = default;

  // Constructor that takes a byte array
  explicit ModbusTCPRequest(const std::vector<uint8_t>& request);

  // Get raw byte arrays (by reference - no copying!)
  const uint8_t* getProtocolIdentifierBytes() const;
  const uint8_t* getLengthBytes() const;

  // Debug method using ESPHome logging
  void printDebugInfo() const;
};


class ModbusBLERequest : public Modbus {
 private:
  static uint8_t ble_transaction_id;

  // Modbus BLUE header fields
  uint8_t ble_transaction_identifier;
  uint8_t transaction_identifier[2];
  uint8_t unit_id;
  uint8_t function_code;
  uint8_t address[2];
  uint8_t number_registers[2];

  // Calculate ModRTU CRC for this modbus request
  uint16_t getModRTU_CRC() const;

 public:
  // Default constructor
  ModbusBLERequest() = default;

  // Constructor that takes a ModbusTCP object
  explicit ModbusBLERequest(const ModbusTCPRequest& request);

  // Convert to byte array for BLE transmission
  std::array<uint8_t, 13> toByteArray() const;

  // getters
  uint8_t getBLETransactionId() const;

  // Debug method using ESPHome logging
  void printDebugInfo() const;
};
}  // namespace modbus_saj

#endif  // COMPONENTS_MODBUS_BLE_BRIDGE_MODBUS_TYPES_H_
