#include "modbus_types.h"

namespace modbus_saj {

// Base class implementations
// Getter method implementations
uint16_t Modbus::getTransactionId() const {
  return (transaction_identifier[0] << 8) + transaction_identifier[1] - 1;
}

uint8_t Modbus::getFunctionCode() const {
  return function_code;
}

uint8_t Modbus::getUnitId() const {
  return unit_id;
}

uint16_t Modbus::getAddress() const {
  return (address[0] << 8) | address[1];
}

uint16_t Modbus::getNumberOfRegisters() const {
  return (number_registers[0] << 8) + (2 * number_registers[1]) + 3;
}

const uint8_t* Modbus::getTransactionIdentifierBytes() const {
  return transaction_identifier;
}

const uint8_t* Modbus::getAddressBytes() const {
  return address;
}

const uint8_t* Modbus::getNumberRegistersBytes() const {
  return number_registers;
}

  // Constructor implementation
ModbusTCPRequest::ModbusTCPRequest(const std::vector<uint8_t>& request) {
  // Parse header fields
  transaction_identifier[0] = request[0];
  transaction_identifier[1] = request[1];
  protocol_identifier[0] = request[2];
  protocol_identifier[1] = request[3];
  length[0] = request[4];
  length[1] = request[5];
  unit_id = request[6];
  function_code = request[7];
  address[0] = request[8];
  address[1] = request[9];
  number_registers[0] = request[10];
  number_registers[1] = request[11];
}

const uint8_t* ModbusTCPRequest::getProtocolIdentifierBytes() const {
  return protocol_identifier;
}

const uint8_t* ModbusTCPRequest::getLengthBytes() const {
  return length;
}

void ModbusTCPRequest::printDebugInfo() const {
  ESP_LOGD(kModbusTypesTag, "Modbus TCP Frame Debug Info:");
  ESP_LOGD(kModbusTypesTag, "  Transaction ID: 0x%02X%02X (%d)",
           transaction_identifier[0], transaction_identifier[1],
           getTransactionId());
  ESP_LOGD(kModbusTypesTag, "  Protocol ID: 0x%02X%02X",
           protocol_identifier[0], protocol_identifier[1]);
  ESP_LOGD(kModbusTypesTag, "  Length: 0x%02X%02X (%d bytes)",
           length[0], length[1],
           (length[0] << 8) | length[1]);
  ESP_LOGD(kModbusTypesTag, "  Unit ID: 0x%02X (%d)",
           unit_id, unit_id);
  ESP_LOGD(kModbusTypesTag, "  Function Code: 0x%02X (%d)",
           function_code, function_code);
  ESP_LOGD(kModbusTypesTag, "  Address: 0x%02X%02X (%d)",
           address[0], address[1], getAddress());
  ESP_LOGD(kModbusTypesTag, "  Number of Registers: 0x%02X%02X (%d)",
           number_registers[0], number_registers[1],
           getNumberOfRegisters());
}

// Static member initialization
uint8_t ModbusBLERequest::ble_transaction_id = 0;

// Constructor implementation
ModbusBLERequest::ModbusBLERequest(const ModbusTCPRequest& request) {
  ble_transaction_identifier = ModbusBLERequest::ble_transaction_id;
  transaction_identifier[0] = request.getTransactionIdentifierBytes()[0];
  transaction_identifier[1] = request.getTransactionIdentifierBytes()[1];
  unit_id = request.getUnitId();
  function_code = request.getFunctionCode();
  address[0] = request.getAddressBytes()[0];
  address[1] = request.getAddressBytes()[1];
  number_registers[0] = request.getNumberRegistersBytes()[0];
  number_registers[1] = request.getNumberRegistersBytes()[1];

  ModbusBLERequest::ble_transaction_id++;
}

// Convert to byte array for BLE transmission
std::array<uint8_t, 13> ModbusBLERequest::toByteArray() const {
  uint16_t crc = getModRTU_CRC();
  uint8_t high_byte = crc >> 8;
  uint8_t low_byte = crc & 0xFF;

  std::array<uint8_t, 13> request = {
    77,                           // Magic number
    0,                            // Reserved
    ble_transaction_identifier,   // BLE transaction ID
    9,                            // Length
    50,                           // Command
    unit_id,                      // Modbus unit ID
    function_code,                // Modbus function code
    address[0],                   // Address high byte
    address[1],                   // Address low byte
    number_registers[0],          // Number of registers high byte
    number_registers[1],          // Number of registers low byte
    low_byte,                     // CRC low byte
    high_byte,                    // CRC high byte
  };

  return request;
}

uint16_t ModbusBLERequest::getModRTU_CRC() const {
  uint8_t modbus_message_final[6] = {
    unit_id,
    function_code,
    address[0],
    address[1],
    number_registers[0],
    number_registers[1]
  };

  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < 6; pos++) {
    crc ^= (uint16_t)modbus_message_final[pos];  // XOR uint8_t into least sig. uint8_t of crc

    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      } else {                           // Else LSB is not set
        crc >>= 1;                    // Just shift right
      }
    }
  }

  // Note, this number has low and high uint8_ts swapped, so use it accordingly (or swap uint8_ts)
  return crc;
}

uint8_t ModbusBLERequest::getBLETransactionId() const {
  return ble_transaction_identifier;
}

void ModbusBLERequest::printDebugInfo() const {
  ESP_LOGD(kModbusTypesTag, "Modbus BLE req Frame Debug Info:");
  ESP_LOGD(kModbusTypesTag, "  BLE Transaction ID: 0x%02X (%d)",
           ble_transaction_identifier, ble_transaction_identifier);
  ESP_LOGD(kModbusTypesTag, "  Transaction ID: 0x%02X%02X (%d)",
           transaction_identifier[0], transaction_identifier[1],
           getTransactionId());
  ESP_LOGD(kModbusTypesTag, "  Unit ID: 0x%02X (%d)",
           unit_id, unit_id);
  ESP_LOGD(kModbusTypesTag, "  Function Code: 0x%02X (%d)",
           function_code, function_code);
  ESP_LOGD(kModbusTypesTag, "  Address: 0x%02X%02X (%d)",
           address[0], address[1], getAddress());
  ESP_LOGD(kModbusTypesTag, "  Number of Registers: 0x%02X%02X (%d)",
           number_registers[0], number_registers[1],
           getNumberOfRegisters());
}
}  // namespace modbus_saj
