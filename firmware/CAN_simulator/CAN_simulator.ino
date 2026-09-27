 /*------------------------------------------------------------------------------
-- Project : Sistema de monitoreo de fallas y datos de manejo de vehículos
-------------------------------------------------------------------------------
-- File : CAN_simulator.INO (Compile in Arduino UNO)
-- Author : Preves, Santiago.
-- Date : March, 2026.
-- Rev 1 : 1st Version.
--
-------------------------------------------------------------------------------
-- Description:
// CAN Simulator - ECU OBD-II  (salida Serial RAW hex)
// Los bytes se imprimen tal como viajan por el bus CAN,
// sin etiquetas ni conversión a unidades físicas.
//
// PIDs Mode 01 soportados:
//   0x00, 0x01, 0x04, 0x05, 0x0A, 0x0B, 0x0C, 0x0D,
//   0x0E, 0x0F, 0x10, 0x11, 0x1F, 0x20, 0x21, 0x2F,
//   0x40, 0x5C, 0x5E
// Modos soportados: 01 / 03 / 04
--               
-------------------------------------------------------------------------------*/

#include <mcp_can.h>
#include <SPI.h>
#include "Packet.cpp"

// ── Hardware ──────────────────────────────────────────────────────────────────
#define CAN0_INT 2
MCP_CAN CAN0(10);

// ── Estado global ─────────────────────────────────────────────────────────────
long unsigned int rxId;
unsigned char len = 0;
uint8_t rxBuf[8];
char msgString[128];

uint16_t storedDTCs[] = { 0x0100, 0x5000 };  // P0100 (MAF), U1000 (CAN)
const int numDTCs = sizeof(storedDTCs) / sizeof(storedDTCs[0]);
bool hasDTCs = true;

uint32_t engineStartTime = 0;
uint16_t distanceMIL_km  = 42;

// ── Helpers ───────────────────────────────────────────────────────────────────

void initResponse(Packet& r, uint8_t pid) {
  r.id         = 0x7E8;
  r.isRTR      = false;
  r.isExtended = false;
  r.dlc        = 8;
  memset(r.data, 0x00, 8);
  r.data[1] = 0x41;
  r.data[2] = pid;
}

// Imprime ID + bytes crudos en hex, sin ninguna etiqueta extra
void sendPacketCAN(Packet& packet) {
  Serial.print("TX 0x");
  if (packet.id < 0x100) Serial.print("0");
  if (packet.id < 0x10)  Serial.print("0");
  Serial.print(packet.id, HEX);
  Serial.print(" [");
  Serial.print(packet.dlc);
  Serial.print("]");
  for (int i = 0; i < packet.dlc; i++) {
    Serial.print(" ");
    if (packet.data[i] < 0x10) Serial.print("0");
    Serial.print(packet.data[i], HEX);
  }

  byte sndStat = CAN0.sendMsgBuf(packet.id, packet.isExtended, packet.dlc, packet.data);
  if (sndStat == CAN_OK) Serial.println(" OK");
  else                   Serial.println(" ERR");
}

// ── Mode 01 ───────────────────────────────────────────────────────────────────

void handleMode01(Packet& request, Packet& response) {
  uint8_t pid = request.data[2];

  switch (pid) {

    case 0x00:
      initResponse(response, 0x00);
      response.data[0] = 0x06;
      response.data[3] = 0x98;
      response.data[4] = 0x7F;
      response.data[5] = 0x80;
      response.data[6] = 0x03;
      sendPacketCAN(response);
      break;

    case 0x01: {
      initResponse(response, 0x01);
      response.data[0] = 0x06;
      response.data[3] = (hasDTCs ? 0x80 : 0x00) | (hasDTCs ? (numDTCs & 0x7F) : 0x00);
      response.data[4] = 0xE0;
      response.data[5] = 0xFF;
      response.data[6] = 0x00;
      sendPacketCAN(response);
      break;
    }

    case 0x04:
      initResponse(response, 0x04);
      response.data[0] = 0x03;
      response.data[3] = (uint8_t)random(100, 200);
      sendPacketCAN(response);
      break;

    case 0x05:
      initResponse(response, 0x05);
      response.data[0] = 0x03;
      response.data[3] = (uint8_t)random(140, 190);
      sendPacketCAN(response);
      break;

    case 0x0A:
      initResponse(response, 0x0A);
      response.data[0] = 0x03;
      response.data[3] = (uint8_t)random(70, 120);
      sendPacketCAN(response);
      break;

    case 0x0B:
      initResponse(response, 0x0B);
      response.data[0] = 0x03;
      response.data[3] = (uint8_t)random(80, 120);
      sendPacketCAN(response);
      break;

    case 0x0C: {
      initResponse(response, 0x0C);
      response.data[0] = 0x04;
      uint16_t rpm4 = (uint16_t)random(600, 6000) * 4;
      response.data[3] = (rpm4 >> 8) & 0xFF;
      response.data[4] =  rpm4       & 0xFF;
      sendPacketCAN(response);
      break;
    }

    case 0x0D:
      initResponse(response, 0x0D);
      response.data[0] = 0x03;
      response.data[3] = (uint8_t)random(0, 120);
      sendPacketCAN(response);
      break;

    case 0x0E:
      initResponse(response, 0x0E);
      response.data[0] = 0x03;
      response.data[3] = (uint8_t)random(100, 160);
      sendPacketCAN(response);
      break;

    case 0x0F:
      initResponse(response, 0x0F);
      response.data[0] = 0x03;
      response.data[3] = (uint8_t)random(60, 80);
      sendPacketCAN(response);
      break;

    case 0x10: {
      initResponse(response, 0x10);
      response.data[0] = 0x04;
      uint16_t maf100 = (uint16_t)random(200, 2000);
      response.data[3] = (maf100 >> 8) & 0xFF;
      response.data[4] =  maf100       & 0xFF;
      sendPacketCAN(response);
      break;
    }

    case 0x11:
      initResponse(response, 0x11);
      response.data[0] = 0x03;
      response.data[3] = (uint8_t)random(0, 255);
      sendPacketCAN(response);
      break;

    case 0x1F: {
      initResponse(response, 0x1F);
      response.data[0] = 0x04;
      uint16_t secs = (uint16_t)((millis() - engineStartTime) / 1000UL);
      response.data[3] = (secs >> 8) & 0xFF;
      response.data[4] =  secs       & 0xFF;
      sendPacketCAN(response);
      break;
    }

    case 0x20:
      initResponse(response, 0x20);
      response.data[0] = 0x06;
      response.data[3] = 0x80;
      response.data[4] = 0x02;
      response.data[5] = 0x00;
      response.data[6] = 0x01;
      sendPacketCAN(response);
      break;

    case 0x21: {
      initResponse(response, 0x21);
      response.data[0] = 0x04;
      uint16_t dist = hasDTCs ? distanceMIL_km : 0;
      response.data[3] = (dist >> 8) & 0xFF;
      response.data[4] =  dist       & 0xFF;
      sendPacketCAN(response);
      break;
    }

    case 0x2F:
      initResponse(response, 0x2F);
      response.data[0] = 0x03;
      response.data[3] = (uint8_t)random(50, 200);
      sendPacketCAN(response);
      break;

    case 0x40:
      initResponse(response, 0x40);
      response.data[0] = 0x06;
      response.data[3] = 0x00;
      response.data[4] = 0x00;
      response.data[5] = 0x00;
      response.data[6] = 0x14;
      sendPacketCAN(response);
      break;

    case 0x5C:
      initResponse(response, 0x5C);
      response.data[0] = 0x03;
      response.data[3] = (uint8_t)random(120, 160);
      sendPacketCAN(response);
      break;

    case 0x5E: {
      initResponse(response, 0x5E);
      response.data[0] = 0x04;
      uint16_t rate20 = (uint16_t)random(100, 300);
      response.data[3] = (rate20 >> 8) & 0xFF;
      response.data[4] =  rate20       & 0xFF;
      sendPacketCAN(response);
      break;
    }

    default:
      Serial.print("PID 0x");
      if (pid < 0x10) Serial.print("0");
      Serial.print(pid, HEX);
      Serial.println(" no soportado");
      break;
  }
}

// ── Dispatcher principal ──────────────────────────────────────────────────────

void handlePacket(Packet& request) {
  if (request.id != 0x7DF) return;
  if (request.dlc < 2)     return;

  uint8_t mode = request.data[1];
  Packet response;

  switch (mode) {

    case 0x01:
      handleMode01(request, response);
      break;

    case 0x03:
      // Respuesta a Modo 03 (leer DTCs).
      // FIX: la trama estaba mal armada -> el ELM327 no la parseaba.
      //   - PCI (data[0]) debe ser la cantidad de bytes de PAYLOAD (0x43 + DTCs),
      //     NO incluir el propio byte de PCI. Antes ponía 'offset' (contaba de más).
      //   - dlc debe ser 8 con padding (como el Modo 01), no 'offset'.
      //   - Si no hay DTCs, responder igual con "43" (0 códigos) en vez de callar.
      response.id         = 0x7E8;
      response.isRTR      = false;
      response.isExtended = false;
      memset(response.data, 0x00, 8);
      response.data[1] = 0x43;
      {
        int offset = 2;
        if (hasDTCs) {
          for (int i = 0; i < numDTCs && offset + 1 < 8; i++) {
            uint16_t code = storedDTCs[i];
            uint8_t systemBits = ((code & 0xF000) == 0x5000) ? 0xC0 : 0x00;
            response.data[offset++] = ((code >> 8) & 0x3F) | systemBits;
            response.data[offset++] = code & 0xFF;
          }
        }
        response.data[0] = offset - 1;   // FIX: bytes de payload (43 + DTCs), sin el PCI
        response.dlc     = 8;            // FIX: trama estándar de 8 bytes (padding)
      }
      sendPacketCAN(response);
      break;

    case 0x04:
      hasDTCs = false;
      response.id         = 0x7E8;
      response.isRTR      = false;
      response.isExtended = false;
      response.dlc        = 2;
      response.data[0]    = 0x01;
      response.data[1]    = 0x44;
      sendPacketCAN(response);
      break;

    default:
      break;
  }
}

// ── Setup & Loop ──────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 OK");
  else
    Serial.println("MCP2515 ERR");

  CAN0.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);

  engineStartTime = millis();
  Serial.println("CAN ready");
}

void loop() {
  if (!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    Packet packet;
    packet.dlc = len;

    if ((rxId & 0x80000000) == 0x80000000) {
      packet.id         = rxId & 0x1FFFFFFF;
      packet.isExtended = true;
    } else {
      packet.id         = rxId;
      packet.isExtended = false;
    }

    // Imprimir trama RX cruda
    Serial.print("RX 0x");
    if (packet.id < 0x100) Serial.print("0");
    if (packet.id < 0x10)  Serial.print("0");
    Serial.print(packet.id, HEX);
    Serial.print(" [");
    Serial.print(len);
    Serial.print("]");

    if ((rxId & 0x40000000) == 0x40000000) {
      packet.isRTR = true;
      Serial.println(" RTR");
    } else {
      packet.isRTR = false;
      for (byte i = 0; i < len; i++) {
        Serial.print(" ");
        if (rxBuf[i] < 0x10) Serial.print("0");
        Serial.print(rxBuf[i], HEX);
        packet.data[i] = rxBuf[i];
      }
      Serial.println();
    }

    handlePacket(packet);
  }
}
