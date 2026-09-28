/*------------------------------------------------------------------------------
-- Project : Sistema de monitoreo de fallas y datos de manejo de vehículos
-------------------------------------------------------------------------------
-- File : CAN_simulator.ino
-- Author : Preves, Santiago.
-- Date : Sep 27, 2026.
-- Rev 3 : Código ordenado y estandarizado.
--
-------------------------------------------------------------------------------
-- Description:
  Simulador de una ECU OBD-II sobre bus CAN (MCP2515, 500 kbit/s, ID de 11 bits).
  Responde las consultas que el ELM327 envía al ID funcional 0x7DF con tramas
  desde el ID 0x7E8, y muestra por Serial cada trama recibida (RX) y enviada
  (TX) en hexadecimal crudo, tal como viaja por el bus.

  Modos soportados:
    01 - Datos actuales. PIDs: 00, 01, 04, 05, 0A, 0B, 0C, 0D, 0E, 0F, 10,
         11, 1F, 20, 21, 2F, 40, 5C y 5E (valores aleatorios en rangos
         realistas).
    03 - Lectura de DTC: de 0 a 2 códigos sorteados de un catálogo.
    04 - Borrado de DTC.
--
-------------------------------------------------------------------------------*/

#include <mcp_can.h>
#include <SPI.h>
#include "Packet.cpp"   // estructura Packet: id, dlc, data[8], isRTR, isExtended


// =============================================================================
//  CONFIGURACIÓN
// =============================================================================
#define PIN_CS_CAN   10           // chip select del MCP2515
#define PIN_INT_CAN  2            // interrupción del MCP2515 (activa en bajo)

const uint32_t ID_CONSULTA_OBD  = 0x7DF;   // ID funcional de las consultas OBD-II
const uint32_t ID_RESPUESTA_ECU = 0x7E8;   // ID con el que responde la ECU simulada

#define MAX_DTC              2         // máximo de fallas simultáneas
#define INTERVALO_SORTEO_MS  30000UL   // cada cuánto se re-sortean los DTC (0 = sólo al arrancar)

MCP_CAN CAN0(PIN_CS_CAN);


// =============================================================================
//  CÓDIGOS DE FALLA (DTC)
// =============================================================================
// Valor crudo de 2 bytes de cada DTC, tal como viaja en la respuesta al Modo 03.
const uint16_t CATALOGO_DTC[] = {
  0x0100,   // P0100  Sensor de flujo de aire (MAF)              [Motor]
  0x0171,   // P0171  Mezcla demasiado pobre                     [Motor]
  0x0300,   // P0300  Fallo de encendido en varios cilindros     [Motor]
  0x0420,   // P0420  Eficiencia del catalizador baja            [Motor]
  0x0128,   // P0128  Termostato de refrigerante                 [Motor]
  0x4035,   // C0035  Sensor de velocidad rueda del. izq.        [Chasis]
  0x4110,   // C0110  Motor de bomba de ABS                      [Chasis]
  0x8010,   // B0010  Airbag del conductor                       [Carroceria]
  0x9318,   // B1318  Tension de bateria baja                    [Carroceria]
  0xC100,   // U0100  Perdida de comunicacion con la ECU         [Red]
  0xC121,   // U0121  Perdida de comunicacion con ABS            [Red]
  0xD000,   // U1000  Comunicacion en red CAN                    [Red]
};
const int CANTIDAD_CATALOGO = sizeof(CATALOGO_DTC) / sizeof(CATALOGO_DTC[0]);

uint16_t dtcActivos[MAX_DTC];
int      cantidadDTC        = 0;
uint32_t tiempoUltimoSorteo = 0;


// =============================================================================
//  ESTADO DEL MOTOR SIMULADO
// =============================================================================
uint32_t tiempoArranque     = 0;    // referencia del PID 1F (segundos desde el arranque)
uint16_t distanciaConMIL_km = 42;   // valor del PID 21 cuando hay DTC activos


// =============================================================================
//  SORTEO DE DTC
// =============================================================================
// Elige al azar entre 0 y MAX_DTC códigos del catálogo (0 = auto sano),
// evitando repetir un mismo código.
void sortearDTCs() {
  int n = random(0, MAX_DTC + 1);
  cantidadDTC = 0;
  for (int i = 0; i < n; i++) {
    uint16_t codigo;
    bool repetido;
    int intentos = 0;
    do {
      codigo = CATALOGO_DTC[random(0, CANTIDAD_CATALOGO)];
      repetido = false;
      for (int j = 0; j < cantidadDTC; j++) {
        if (dtcActivos[j] == codigo) repetido = true;
      }
    } while (repetido && ++intentos < 10);
    dtcActivos[cantidadDTC++] = codigo;
  }

  Serial.print("DTCs sorteados: ");
  if (cantidadDTC == 0) {
    Serial.println("(ninguno - auto sano)");
  } else {
    for (int i = 0; i < cantidadDTC; i++) {
      Serial.print("0x");
      Serial.print(dtcActivos[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}


// =============================================================================
//  IMPRESIÓN Y ENVÍO DE TRAMAS
// =============================================================================
// Imprime "<prefijo> 0xIII [n]" con el ID en tres dígitos hexadecimales.
void imprimirEncabezado(const char* prefijo, uint32_t id, uint8_t dlc) {
  Serial.print(prefijo);
  Serial.print(" 0x");
  if (id < 0x100) Serial.print("0");
  if (id < 0x10)  Serial.print("0");
  Serial.print(id, HEX);
  Serial.print(" [");
  Serial.print(dlc);
  Serial.print("]");
}

// Imprime los bytes de datos en hexadecimal: " 06 41 00 98 ...".
void imprimirBytes(const uint8_t* datos, uint8_t cantidad) {
  for (uint8_t i = 0; i < cantidad; i++) {
    Serial.print(" ");
    if (datos[i] < 0x10) Serial.print("0");
    Serial.print(datos[i], HEX);
  }
}

// Envía la trama por el bus y la muestra por Serial con el resultado.
void enviarTrama(Packet& trama) {
  imprimirEncabezado("TX", trama.id, trama.dlc);
  imprimirBytes(trama.data, trama.dlc);

  byte estado = CAN0.sendMsgBuf(trama.id, trama.isExtended, trama.dlc, trama.data);
  if (estado == CAN_OK) Serial.println(" OK");
  else                  Serial.println(" ERR");
}

// Deja una trama de respuesta vacía, con el ID de la ECU y el largo indicado.
void iniciarTrama(Packet& trama, uint8_t dlc) {
  trama.id         = ID_RESPUESTA_ECU;
  trama.isRTR      = false;
  trama.isExtended = false;
  trama.dlc        = dlc;
  memset(trama.data, 0x00, 8);
}


// =============================================================================
//  MODO 01: DATOS ACTUALES
// =============================================================================
// Formato de la respuesta (trama única ISO 15765-4, 8 bytes con relleno):
//   byte 0 = cantidad de bytes útiles (0x41 + PID + datos)
//   byte 1 = 0x41 (respuesta al Modo 01)
//   byte 2 = PID
//   bytes 3.. = datos A, B, C, D

void responderPID1Byte(Packet& r, uint8_t pid, uint8_t a) {
  iniciarTrama(r, 8);
  r.data[0] = 3;
  r.data[1] = 0x41;
  r.data[2] = pid;
  r.data[3] = a;
  enviarTrama(r);
}

void responderPID2Bytes(Packet& r, uint8_t pid, uint16_t valor) {
  iniciarTrama(r, 8);
  r.data[0] = 4;
  r.data[1] = 0x41;
  r.data[2] = pid;
  r.data[3] = (valor >> 8) & 0xFF;
  r.data[4] =  valor       & 0xFF;
  enviarTrama(r);
}

void responderPID4Bytes(Packet& r, uint8_t pid, uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  iniciarTrama(r, 8);
  r.data[0] = 6;
  r.data[1] = 0x41;
  r.data[2] = pid;
  r.data[3] = a;
  r.data[4] = b;
  r.data[5] = c;
  r.data[6] = d;
  enviarTrama(r);
}

void responderModo01(uint8_t pid, Packet& r) {
  switch (pid) {
    // Mapas de PIDs soportados: 01-20, 21-40 y 41-60
    case 0x00: responderPID4Bytes(r, pid, 0x98, 0x7F, 0x80, 0x03); break;
    case 0x20: responderPID4Bytes(r, pid, 0x80, 0x02, 0x00, 0x01); break;
    case 0x40: responderPID4Bytes(r, pid, 0x00, 0x00, 0x00, 0x14); break;

    // Estado de monitores: bit 7 = luz MIL encendida, bits 0-6 = cantidad de DTC
    case 0x01:
      responderPID4Bytes(r, pid, (cantidadDTC > 0 ? 0x80 : 0x00) | (cantidadDTC & 0x7F), 0xE0, 0xFF, 0x00);
      break;

    // PIDs de 1 byte (valor crudo A)
    case 0x04: responderPID1Byte(r, pid, random(100, 200)); break;   // carga del motor
    case 0x05: responderPID1Byte(r, pid, random(140, 190)); break;   // temperatura del refrigerante
    case 0x0A: responderPID1Byte(r, pid, random(70, 120));  break;   // presión de combustible
    case 0x0B: responderPID1Byte(r, pid, random(80, 120));  break;   // presión del múltiple (MAP)
    case 0x0D: responderPID1Byte(r, pid, random(0, 120));   break;   // velocidad
    case 0x0E: responderPID1Byte(r, pid, random(100, 160)); break;   // avance de encendido
    case 0x0F: responderPID1Byte(r, pid, random(60, 80));   break;   // temperatura de admisión
    case 0x11: responderPID1Byte(r, pid, random(0, 255));   break;   // posición del acelerador
    case 0x2F: responderPID1Byte(r, pid, random(50, 200));  break;   // nivel de combustible
    case 0x5C: responderPID1Byte(r, pid, random(120, 160)); break;   // temperatura del aceite

    // PIDs de 2 bytes (valor crudo A*256 + B)
    case 0x0C: responderPID2Bytes(r, pid, random(600, 6000) * 4); break;                   // RPM x 4
    case 0x10: responderPID2Bytes(r, pid, random(200, 2000));     break;                   // MAF x 100
    case 0x1F: responderPID2Bytes(r, pid, (millis() - tiempoArranque) / 1000UL); break;    // s desde el arranque
    case 0x21: responderPID2Bytes(r, pid, cantidadDTC > 0 ? distanciaConMIL_km : 0); break; // km con MIL
    case 0x5E: responderPID2Bytes(r, pid, random(100, 300));      break;                   // consumo x 20

    default:
      Serial.print("PID 0x");
      if (pid < 0x10) Serial.print("0");
      Serial.print(pid, HEX);
      Serial.println(" no soportado");
      break;
  }
}


// =============================================================================
//  MODO 03: LECTURA DE DTC  /  MODO 04: BORRADO DE DTC
// =============================================================================
// Responde 0x43 seguido de los DTC activos (2 bytes cada uno). Si no hay
// ninguno, responde igual con 0x43 solo.
void responderModo03(Packet& r) {
  iniciarTrama(r, 8);
  r.data[1] = 0x43;
  int pos = 2;
  for (int i = 0; i < cantidadDTC && pos + 1 < 8; i++) {
    r.data[pos++] = (dtcActivos[i] >> 8) & 0xFF;
    r.data[pos++] =  dtcActivos[i]       & 0xFF;
  }
  r.data[0] = pos - 1;    // bytes útiles: 0x43 + DTC
  enviarTrama(r);
}

void responderModo04(Packet& r) {
  cantidadDTC = 0;
  iniciarTrama(r, 2);
  r.data[0] = 0x01;
  r.data[1] = 0x44;
  enviarTrama(r);
}


// =============================================================================
//  ATENCIÓN DE CONSULTAS
// =============================================================================
void procesarConsulta(Packet& consulta) {
  if (consulta.id != ID_CONSULTA_OBD) return;
  if (consulta.dlc < 2) return;

  uint8_t modo = consulta.data[1];
  Packet respuesta;

  switch (modo) {
    case 0x01: responderModo01(consulta.data[2], respuesta); break;
    case 0x03: responderModo03(respuesta);                    break;
    case 0x04: responderModo04(respuesta);                    break;
    default:   break;
  }
}


// =============================================================================
//  SETUP Y LOOP
// =============================================================================
void setup() {
  Serial.begin(115200);

  if (CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 OK");
  else
    Serial.println("MCP2515 ERR");

  CAN0.setMode(MCP_NORMAL);
  pinMode(PIN_INT_CAN, INPUT);

  tiempoArranque = millis();

  randomSeed(analogRead(A0) ^ micros());   // semilla: pin analógico flotante + micros()
  sortearDTCs();
  tiempoUltimoSorteo = millis();

  Serial.println("CAN ready");
}

void loop() {
  // Re-sorteo periódico de los DTC
  if (INTERVALO_SORTEO_MS > 0 && millis() - tiempoUltimoSorteo >= INTERVALO_SORTEO_MS) {
    tiempoUltimoSorteo = millis();
    sortearDTCs();
  }

  // El MCP2515 baja su pin de interrupción cuando recibió una trama
  if (digitalRead(PIN_INT_CAN) == LOW) {
    unsigned long idCrudo;
    unsigned char largo = 0;
    unsigned char datos[8];
    CAN0.readMsgBuf(&idCrudo, &largo, datos);

    // La librería marca en el ID: bit 31 = ID extendido (29 bits), bit 30 = trama remota
    Packet consulta;
    consulta.dlc        = largo;
    consulta.isExtended = (idCrudo & 0x80000000) != 0;
    consulta.id         = consulta.isExtended ? (idCrudo & 0x1FFFFFFF) : idCrudo;
    consulta.isRTR      = (idCrudo & 0x40000000) != 0;

    imprimirEncabezado("RX", consulta.id, largo);
    if (consulta.isRTR) {
      Serial.println(" RTR");
    } else {
      for (uint8_t i = 0; i < largo; i++) consulta.data[i] = datos[i];
      imprimirBytes(datos, largo);
      Serial.println();
    }

    procesarConsulta(consulta);
  }
}
