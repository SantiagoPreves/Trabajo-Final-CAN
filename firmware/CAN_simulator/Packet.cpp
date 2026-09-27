//
//  Packet.cpp
//
//  Estructura de un frame CAN con serialización/deserialización.
//
//  Fixes aplicados:
//    - id: uint16_t → uint32_t (necesario para IDs extendidos de 29 bits)
//    - toBytes(): bits de isExtended e isRTR estaban invertidos respecto a fromBytes()
//    - fromBytes(): memcpy usaba offset incorrecto (buffer + len - dlc en vez de buffer + 3/5)
//    - Constructor inicializa todos los campos
//

#include <Arduino.h>

struct Packet {
  uint32_t id;       // 11 bits (estándar) o 29 bits (extendido)
  bool isRTR;
  bool isExtended;
  uint8_t dlc;
  uint8_t data[8];

  Packet() {
    id = 0;
    isRTR = false;
    isExtended = false;
    dlc = 0;
    memset(data, 0x00, 8);
  }

  // Decodifica desde un arreglo de bytes
  // Formato: [ control | id_bytes | data_bytes ]
  // control: bit7=isExtended, bit6=isRTR, bits3:0=dlc
  bool fromBytes(const uint8_t* buffer, size_t len) {
    if (len < 3) return false;

    uint8_t control = buffer[0];
    isExtended = (control & 0x80) != 0;
    isRTR      = (control & 0x40) != 0;
    dlc        = control & 0x0F;

    if (isExtended) {
      if (len < 5) return false;
      id = ((uint32_t)(buffer[1] & 0xFF) << 24)
         | ((uint32_t)(buffer[2] & 0xFF) << 16)
         | ((uint32_t)(buffer[3] & 0xFF) << 8)
         |  (uint32_t)(buffer[4] & 0xFF);
      if (dlc > 8 || len < (size_t)(5 + dlc)) return false;
      memcpy(data, buffer + 5, dlc);   // FIX: era buffer + len - dlc
    } else {
      id = ((uint16_t)(buffer[1] & 0xFF) << 8)
         |  (uint16_t)(buffer[2] & 0xFF);
      if (dlc > 8 || len < (size_t)(3 + dlc)) return false;
      memcpy(data, buffer + 3, dlc);   // FIX: era buffer + len - dlc
    }
    return true;
  }

  // Codifica a arreglo de bytes
  // Retorna la cantidad de bytes escritos en buffer
  size_t toBytes(uint8_t* buffer) const {
    uint8_t control = 0x00;
    if (isExtended) control |= 0x80;  // FIX: estaban invertidos
    if (isRTR)      control |= 0x40;  // FIX: estaban invertidos
    control |= (dlc & 0x0F);
    buffer[0] = control;

    if (isExtended) {
      buffer[1] = (id >> 24) & 0xFF;
      buffer[2] = (id >> 16) & 0xFF;
      buffer[3] = (id >> 8)  & 0xFF;
      buffer[4] =  id        & 0xFF;
      memcpy(buffer + 5, data, dlc);
      return 5 + dlc;
    } else {
      buffer[1] = (id >> 8) & 0xFF;
      buffer[2] =  id       & 0xFF;
      memcpy(buffer + 3, data, dlc);
      return 3 + dlc;
    }
  }
};
