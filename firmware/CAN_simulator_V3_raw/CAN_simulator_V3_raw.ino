/*------------------------------------------------------------------------------
-- Project : Sistema de monitoreo de fallas y datos de manejo de vehículos
-------------------------------------------------------------------------------
-- File : CAN_simulator_V3_raw.INO
-- Author : Preves, Santiago.
-- Date : Aug 10, 2026.
-- Rev 0 : Initial release.
--
-------------------------------------------------------------------------------
-- Description:
  CAN Simulator - ECU OBD-II  (salida Serial RAW hex)
  Los bytes se imprimen tal como viajan por el bus CAN,
  sin etiquetas ni conversión a unidades físicas.

  PIDs Mode 01 soportados:
  0x00, 0x01, 0x04, 0x05, 0x0A, 0x0B, 0x0C, 0x0D,
  0x0E, 0x0F, 0x10, 0x11, 0x1F, 0x20, 0x21, 0x2F,
  0x40, 0x5C, 0x5E
  Modos soportados: 01 / 03 / 04
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

// ── DTCs variables (sorteo aleatorio) ──────────────────────────────────────────
// Catálogo de DTCs generales (valor CRUDO de 2 bytes tal como va en el bus,
// justo después del "43"). El decoder los reconstruye a Pxxxx/Cxxxx/Bxxxx/Uxxxx.
const uint16_t catalogoDTC[] = {
  0x0100, // P0100 - sensor de flujo de aire (MAF)          [Motor]
  0x0171, // P0171 - mezcla demasiado pobre                 [Motor]
  0x0300, // P0300 - fallo de encendido varios cilindros    [Motor]
  0x0420, // P0420 - eficiencia del catalizador             [Motor]
  0x0128, // P0128 - termostato de refrigerante             [Motor]
  0x4035, // C0035 - sensor velocidad rueda del. izq.       [Chasis]
  0x4110, // C0110 - motor de bomba de ABS                  [Chasis]
  0x8010, // B0010 - airbag del conductor                   [Carroceria]
  0x9318, // B1318 - tension de bateria baja                [Carroceria]
  0xC100, // U0100 - perdida de comunicacion con la ECU     [Red]
  0xC121, // U0121 - perdida de comunicacion con ABS        [Red]
  0xD000, // U1000 - comunicacion en red CAN                [Red]
};
const int CATALOGO_N = sizeof(catalogoDTC) / sizeof(catalogoDTC[0]);

#define MAX_DTC          2         // máximo de fallas simultáneas sorteadas
#define DTC_INTERVAL_MS  30000UL   // re-sortea cada 30s; poné 0 para sortear solo al arranque

uint16_t activeDTCs[MAX_DTC];
int      numDTCs   = 0;           
bool     hasDTCs   = false;
uint32_t lastSorteo = 0;

// Sortea 0, 1 o 2 DTCs al azar del catálogo (a veces ninguno = auto sano).
void sortearDTCs() {
  int n = random(0, MAX_DTC + 1);   // 0..MAX_DTC (uniforme)
  numDTCs = 0;
  for (int i = 0; i < n; i++) {
    uint16_t code;
    bool dup;
    int intentos = 0;
    do {
      code = catalogoDTC[random(0, CATALOGO_N)];
      dup = false;
      for (int j = 0; j < numDTCs; j++) if (activeDTCs[j] == code) dup = true;
    } while (dup && ++intentos < 10);
    activeDTCs[numDTCs++] = code;
  }
  hasDTCs = (numDTCs > 0);

  Serial.print("DTCs sorteados: ");
  if (numDTCs == 0) {
    Serial.println("(ninguno - auto sano)");
  } else {
    for (int i = 0; i < numDTCs; i++) { Serial.print("0x"); Serial.print(activeDTCs[i], HEX); Serial.print(" "); }
    Serial.println();
  }
}

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
      response.data[3] = (hasDTCs ? 0x80 : 0x00) | (numDTCs & 0x7F);
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
        // activeDTCs ya guarda el valor CRUDO de 2 bytes -> se escribe directo
        for (int i = 0; i < numDTCs && offset + 1 < 8; i++) {
          uint16_t code = activeDTCs[i];
          response.data[offset++] = (code >> 8) & 0xFF;
          response.data[offset++] =  code       & 0xFF;
        }
        response.data[0] = offset - 1;   // FIX: bytes de payload (43 + DTCs), sin el PCI
        response.dlc     = 8;            // FIX: trama estándar de 8 bytes (padding)
      }
      sendPacketCAN(response);
      break;

    case 0x04:
      hasDTCs = false;
      numDTCs = 0;   // [NEW] limpiar también el contador
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

  // [NEW] Semilla random (pin analógico flotante + micros) y primer sorteo de DTCs
  randomSeed(analogRead(A0) ^ micros());
  sortearDTCs();
  lastSorteo = millis();

  Serial.println("CAN ready");
}

void loop() {
  // [NEW] Re-sortear DTCs cada tanto (0 = solo al arranque)
  if (DTC_INTERVAL_MS > 0 && millis() - lastSorteo >= DTC_INTERVAL_MS) {
    lastSorteo = millis();
    sortearDTCs();
  }

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
