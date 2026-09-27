/*------------------------------------------------------------------------------
-- Project : Sistema de monitoreo de fallas y datos de manejo de vehículos
-------------------------------------------------------------------------------
-- File : monitoreo_sin_gestion.ino (Adafruit Feather nRF52840 Sense)
-- Author : Preves, Santiago.
-- Date : Sept 1, 2026.
-- Rev 8 : Primera versión funcional, sin gestión de energía.
--
-------------------------------------------------------------------------------
-- Description:
  Primera versión completa del sistema, sin gestión de energía: el módem, el
  GNSS y el enlace BLE quedan encendidos todo el tiempo. Cada 15 s lee los
  PIDs y los DTC del ELM327 por BLE, la posición GNSS y el IMU, publica el
  JSON crudo por MQTT (4G, A7670SA) y guarda los valores traducidos en la SD.

  Hardware : Adafruit Feather nRF52840 Sense
  BLE      : ELM327 (servicio FFF0 / característica FFF1)
  UART     : A7670SA en Serial1 (TX = pin 1, RX = pin 0)
  SPI      : tarjeta SD (CS = A0)

  Pines:
    A0 -> CS de la SD
    A1 -> SLEEP_7670 (DTR del A7670SA)
    A3 -> NIVEL_BAT (ADC del divisor de batería)
    A5 -> PWRKEY_7670
    D3 -> CHRG del TP4056 (activo en bajo)
    D4 -> STDBY del TP4056 (activo en bajo)
    D5 -> ACTIVAR_NIVEL (habilita el divisor de batería)
--
-------------------------------------------------------------------------------*/

#include <Arduino.h>
#include <bluefruit.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <Adafruit_LSM6DS33.h>

// ============================================================
//  UART - A7670SA
// ============================================================
#define SIM_SERIAL  Serial1
#define SIM_BAUD    115200

// ============================================================
//  SD
// ============================================================
#define SD_CS_PIN   A0
bool sdLista = false;

// ============================================================
//  NIVEL DE BATERÍA
//  D5 habilita el divisor V_BAT -> R15 -> NIVEL_BAT -> R16 -> GND a
//  través del MOSFET Q2 (canal P, lado alto). A3 lee la tensión.
// ============================================================
#define PIN_ACTIVAR_NIVEL  12     // D5
#define PIN_NIVEL_BAT      A3

// Vbat = Vadc * (R15 + R16) / R16
#define DIV_R_TOP   10000.0f    // R15
#define DIV_R_BOT   10000.0f    // R16
#define DIV_RATIO   ((DIV_R_TOP + DIV_R_BOT) / DIV_R_BOT)

// ADC del nRF52840 con referencia AR_INTERNAL (0.6 V, ganancia 1/6)
#define ADC_VREF        3.6f     // fondo de escala
#define ADC_MAX         4095.0f  // 12 bits
#define BAT_OVERSAMPLE  16       // muestras promediadas
#define BAT_SETTLE_MS   30       // estabilización del divisor
#define CAL_BAT         1.00f    // calibración: Vreal / Vmedido (con tester)

// Q2 (BSS84, canal P): LOW en el gate = divisor habilitado
#define NIVEL_ACTIVO   LOW
#define NIVEL_INACTIVO HIGH

float voltajeBateria = 0.0f;

// ============================================================
//  ESTADO DE CARGA (TP4056: salidas open-drain activas en bajo)
// ============================================================
#define PIN_CHRG    10  // D3
#define PIN_STDBY   11  // D4

bool cargando       = false;
bool cargaCompleta  = false;

// ============================================================
//  A7670SA - PWRKEY y SLEEP
// ============================================================
#define PIN_PWRKEY_7670  A5
#define PIN_SLEEP_7670   A1

// ============================================================
//  MQTT
// ============================================================
#define MQTT_BROKER    "livra-mqtt.fi.mdp.edu.ar"
#define MQTT_PORT      1884
String  mqttClientID   = "feather_sd_";
#define MQTT_USER      ""
#define MQTT_PASS      ""
#define MQTT_TOPIC     "prueba_in"
#define MQTT_QOS       0
#define MQTT_KEEPALIVE 120

// ============================================================
//  APN
// ============================================================
#define APN "internet"

// ============================================================
//  MODO DIAGNÓSTICO (para medir el consumo del módem solo)
//    DIAG_SOLO_MQTT = 1 -> no usa BLE/OBD; sólo publica batería e IMU.
//    DIAG_CON_GNSS  = 0 -> GNSS apagado.
//  Funcionamiento normal: DIAG_SOLO_MQTT = 0, DIAG_CON_GNSS = 1.
// ============================================================
#define DIAG_SOLO_MQTT   0
#define DIAG_CON_GNSS    1

// ============================================================
//  WATCHDOG Y LIVENESS
// ============================================================
#define WDT_TIMEOUT_S   20UL       // reinicio si el firmware se congela más de 20 s
#define LIVENESS_MS     240000UL   // reinicio si pasan 4 min sin publicar
uint32_t tUltimoPublish = 0;

// ============================================================
//  LOG DE DEPURACIÓN EN LA SD
//  /debug.log registra cada intento de transmisión (batería, señal,
//  respuesta del módem) y el motivo de cada reinicio. Cada línea se
//  abre, escribe y cierra para que sobreviva a un corte de energía.
// ============================================================
#define DEBUG_LOG_FILE   "/debug.log"
#define DEBUG_LOG_MAX    1048576UL   // al superar 1 MB se borra
String g_motivoReset = "";

// ============================================================
//  UUIDs ELM327
// ============================================================
static const uint16_t ELM_SERVICE_UUID = 0xFFF0;
static const uint16_t ELM_CHAR_UUID    = 0xFFF1;

BLEClientService        elmService(ELM_SERVICE_UUID);
BLEClientCharacteristic elmChar(ELM_CHAR_UUID);

// ============================================================
//  TABLA DE PIDs
// ============================================================
struct PIDEntry {
  const char* comando;
  const char* nombre;
  uint8_t     pid;
  const char* unidad;
};

const PIDEntry pids[] = {
  { "01 0C", "RPM",          0x0C, "RPM"  },
  { "01 0D", "vel_kph",      0x0D, "km/h" },
  { "01 05", "temp_mot_c",   0x05, "C"    },
  { "01 0F", "temp_adm_c",   0x0F, "C"    },
  { "01 04", "carga_pct",    0x04, "%"    },
  { "01 2F", "comb_pct",     0x2F, "%"    },
  { "01 11", "accel_pct",    0x11, "%"    },
  { "01 0B", "map_kpa",      0x0B, "kPa"  },
  { "01 10", "maf_gs",       0x10, "g/s"  },
  { "01 0A", "pcomb_kpa",    0x0A, "kPa"  },
  { "01 0E", "avance_deg",   0x0E, "deg"  },
  { "01 1F", "ton_s",        0x1F, "s"    },
  { "01 21", "dist_mil_km",  0x21, "km"   },
  { "01 5C", "temp_ace_c",   0x5C, "C"    },
  { "01 5E", "cons_lh",      0x5E, "L/h"  },
};

const uint8_t NUM_PIDS = sizeof(pids) / sizeof(pids[0]);
String rawPID[sizeof(pids) / sizeof(pids[0])];

// ============================================================
//  BUFFER ELM
// ============================================================
#define ELM_BUF_SIZE 80
char          bufELM[ELM_BUF_SIZE];
uint8_t       bufELMIdx = 0;
volatile bool elmListo  = false;

// ============================================================
//  MÁQUINA DE ESTADOS
// ============================================================
enum Estado : uint8_t { IDLE, ESPERANDO_ELM, ESPERANDO_DTC, HACIENDO_GNSS };
volatile Estado estado = IDLE;

uint8_t  pidActual       = 0;
uint32_t tiempoEnvio     = 0;
uint32_t tiempoProxCiclo = 0;

const uint32_t TIMEOUT_ELM     = 3000;
const uint32_t INTERVALO_CICLO = 15000;

// ============================================================
//  GNSS crudo
// ============================================================
String rawGNSS = "NO_FIX";
String rawDTC  = "";   // respuesta cruda del Modo 03, ej. "430133"

// ============================================================
//  FLAGS estado MQTT/4G
// ============================================================
bool simListo   = false;
bool mqttOnline = false;
bool gnssOn     = false;   // evita re-encender el GNSS en cada ciclo

// ============================================================
//  IMU - LSM6DS33
// ============================================================
Adafruit_LSM6DS33 lsm6ds33;
bool  imuListo = false;
float imu_ax = 0, imu_ay = 0, imu_az = 0;  // m/s²
float imu_gx = 0, imu_gy = 0, imu_gz = 0;  // rad/s

// ============================================================
//  PROTOTIPOS
// ============================================================
void scan_callback(ble_gap_evt_adv_report_t* report);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len);
void enviarComando(const char* cmd);
void avanzarPID();
void pedirGNSSCrudo();
void leerIMU();
String construirJSONraw();
void publicarMQTT(const String& payload);
bool iniciarSD();
void guardarCSVtraducido();
float traducirPID(uint8_t pid, const String& hexStr);
String limpiarOBD(const String& bruto, uint8_t pid);
String decodificarDTC(const String& bruto);
String categoriaDTC(char letra);
String descripcionDTC(const String& code);
String alertaDTC(const String& codigos);

float leerNivelBateria();
void  leerEstadoCarga();
void  actualizarLEDCarga();
void  pulsarPWRKEY_7670();

bool    wakeupSIM();
String  simSend(const String& cmd, uint32_t timeout = 3000, const String& waitFor = "OK");
bool    simSendCheck(const String& cmd, const String& expected = "OK", uint32_t timeout = 3000);
bool    iniciarRed();
bool    iniciarMQTT();
void    reconectarMQTT();
void    teardownMQTT();

void    iniciarWatchdog();
void    alimentarWatchdog();

void    logDebug(const String& msg);
String  limpiarResp(String r);
String  motivoReset();
int     leerCSQ();

bool    respuestaTieneReinicio(const String& r);
void    esperarBootModulo();

// ============================================================
//  WATCHDOG (nRF52)
//  Una vez arrancado no se puede detener: hay que recargarlo antes
//  de que venza o el micro se reinicia.
// ============================================================
void iniciarWatchdog()
{
  NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) |
                    (WDT_CONFIG_SLEEP_Run  << WDT_CONFIG_SLEEP_Pos);
  NRF_WDT->CRV    = (WDT_TIMEOUT_S * 32768UL) - 1;   // reloj de 32.768 kHz
  NRF_WDT->RREN   = (WDT_RREN_RR0_Enabled << WDT_RREN_RR0_Pos);
  NRF_WDT->TASKS_START = 1;
}

void alimentarWatchdog()
{
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

// ============================================================
//  LOG DE DEPURACIÓN
// ============================================================

// Deja una respuesta AT en una sola línea legible y acotada.
String limpiarResp(String r)
{
  r.replace("\r", " ");
  r.replace("\n", " ");
  r.trim();
  while (r.indexOf("  ") >= 0) r.replace("  ", " ");
  if (r.length() > 140) r = r.substring(0, 140) + "...";
  if (r.length() == 0) r = "(vacio)";
  return r;
}

// Escribe una línea con marca de tiempo en /debug.log (append + close).
void logDebug(const String& msg)
{
  if (!sdLista) return;
  File f = SD.open(DEBUG_LOG_FILE, FILE_WRITE);   // FILE_WRITE = append (va al final)
  if (!f) return;
  f.print(millis());
  f.print(" | ");
  f.println(msg);
  f.close();
  Serial.print("[LOG] "); Serial.println(msg);
}

// Lee y limpia el registro de motivo de reinicio del nRF52.
String motivoReset()
{
  uint32_t r = NRF_POWER->RESETREAS;
  NRF_POWER->RESETREAS = 0xFFFFFFFF;   // limpiar (write-1-to-clear)
  if (r == 0) return "POWERON/BROWNOUT";
  String s = "";
  if (r & POWER_RESETREAS_RESETPIN_Msk) s += "PIN ";
  if (r & POWER_RESETREAS_DOG_Msk)      s += "WATCHDOG ";
  if (r & POWER_RESETREAS_SREQ_Msk)     s += "SOFT(liveness) ";
  if (r & POWER_RESETREAS_LOCKUP_Msk)   s += "LOCKUP ";
  if (r & POWER_RESETREAS_OFF_Msk)      s += "OFF ";
#ifdef POWER_RESETREAS_VBUS_Msk
  if (r & POWER_RESETREAS_VBUS_Msk)     s += "VBUS ";
#endif
  s.trim();
  if (s.length() == 0) s = "OTRO(0x" + String(r, HEX) + ")";
  return s;
}

// Devuelve el CSQ (0-31, 99=desconocido, -1 sin respuesta). <10 = señal pobre.
int leerCSQ()
{
  String r = simSend("AT+CSQ", 1500, "+CSQ:");
  int i = r.indexOf("+CSQ:");
  if (i < 0) return -1;
  int coma = r.indexOf(',', i);
  if (coma < 0) return -1;
  return r.substring(i + 5, coma).toInt();
}

// ============================================================
//  DETECCIÓN DE REINICIO DEL A7670SA
//  Si una respuesta trae los mensajes de arranque del módem, se
//  reinició en medio de la operación y hay que esperar que termine.
// ============================================================
bool respuestaTieneReinicio(const String& r)
{
  return (r.indexOf("*ATREADY")     >= 0) ||   // A7670 listo tras boot
         (r.indexOf("+CPIN: READY") >= 0) ||   // SIM reinicializada
         (r.indexOf("*ISIMAID")     >= 0);      // banner de arranque
}

// Espera a que el módem termine de arrancar (1.5 s sin datos en el puerto)
// y desactiva el eco, que vuelve a activarse con cada reinicio.
void esperarBootModulo()
{
  logDebug("MODEM reinicio detectado -> esperando boot completo");
  uint32_t t0 = millis();
  uint32_t tUltByte = millis();
  while (millis() - t0 < 12000) {
    alimentarWatchdog();
    while (SIM_SERIAL.available()) { SIM_SERIAL.read(); tUltByte = millis(); }
    if (millis() - tUltByte > 1500) break;
    delay(20);
  }
  simSend("ATE0", 1000, "OK");
  // Se perdió el contexto de red, MQTT y GNSS: se reinicializan después
  simListo   = false;
  mqttOnline = false;
  gnssOn     = false;
  logDebug("MODEM boot completo -> se reinicializará red+MQTT");
}

// ============================================================
//  SETUP
// ============================================================
void setup()
{
  // Leer el motivo del último reinicio antes de que se borre
  g_motivoReset = motivoReset();

  delay(3000);
  Serial.begin(115200);

  Serial.println("=== OBD + GPS -> MQTT (crudo) + SD (traducido) ===");
  Serial.print("Motivo del ultimo reinicio: ");
  Serial.println(g_motivoReset);

  randomSeed(micros());
  mqttClientID += String(random(10000, 99999));
  Serial.print("MQTT Client ID asignado: ");
  Serial.println(mqttClientID);

  // PINES
  pinMode(PIN_ACTIVAR_NIVEL, OUTPUT);
  digitalWrite(PIN_ACTIVAR_NIVEL, NIVEL_INACTIVO);

  pinMode(PIN_CHRG,  INPUT_PULLUP);
  pinMode(PIN_STDBY, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(PIN_SLEEP_7670, OUTPUT);
  digitalWrite(PIN_SLEEP_7670, LOW);   // LOW = despierto

  pinMode(PIN_PWRKEY_7670, OUTPUT);
  digitalWrite(PIN_PWRKEY_7670, HIGH);

  // ADC: referencia y resolución
  analogReference(AR_INTERNAL);   // 0.6V ref + gain 1/6 -> fondo de escala 3.6V
  analogReadResolution(12);

  Serial.println("Encendiendo A7670SA por hardware (PWRKEY)...");
  pulsarPWRKEY_7670();

  SIM_SERIAL.begin(SIM_BAUD);
  delay(1000);
  while (SIM_SERIAL.available()) SIM_SERIAL.read();

  Serial.print("Comprobando estado previo del SIM... ");
  simSend("ATE0", 1000, "OK");
  String checkStr = simSend("AT", 1000, "OK");

  if (checkStr.indexOf("OK") >= 0) {
    Serial.println("RESPONDE.");
    Serial.println("Forzando reinicio (AT+CRESET)...");
    simSend("AT+CRESET", 3000, "OK");
    delay(8000);
    while (SIM_SERIAL.available()) SIM_SERIAL.read();
    Serial.println("SIM reiniciado.");
  } else {
    Serial.println("NO RESPONDE (posiblemente arrancando).");
  }

  if (iniciarRed())  simListo = true;
  else Serial.println("AVISO: red 4G no disponible, se reintentará.");

  if (simListo && iniciarMQTT()) mqttOnline = true;
  else Serial.println("AVISO: MQTT offline, se reintentará.");

  if (iniciarSD()) sdLista = true;

  // Borrar el log si creció demasiado y registrar el arranque
  if (sdLista) {
    File fchk = SD.open(DEBUG_LOG_FILE, FILE_READ);
    if (fchk) {
      uint32_t tam = fchk.size();
      fchk.close();
      if (tam > DEBUG_LOG_MAX) SD.remove(DEBUG_LOG_FILE);
    }
    voltajeBateria = leerNivelBateria();
    leerEstadoCarga();
    logDebug("========== BOOT  reset=" + g_motivoReset +
             "  vbat=" + String(voltajeBateria, 2) +
             "  cargando=" + String(cargando ? 1 : 0) +
             "  simListo=" + String(simListo ? 1 : 0) +
             "  mqtt=" + String(mqttOnline ? 1 : 0) + " ==========");
  }

  // IMU
  if (lsm6ds33.begin_I2C()) {
    imuListo = true;
    lsm6ds33.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
    lsm6ds33.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
    lsm6ds33.setAccelDataRate(LSM6DS_RATE_104_HZ);
    lsm6ds33.setGyroDataRate(LSM6DS_RATE_104_HZ);
    Serial.println("IMU LSM6DS33: OK");
  } else {
    Serial.println("IMU LSM6DS33: FAIL (verificar I2C)");
  }

#if !DIAG_SOLO_MQTT
  // BLE
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Bluefruit-ELM");
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  elmService.begin();
  elmChar.begin();
  elmChar.setNotifyCallback(notify_callback);
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.useActiveScan(true);

  Serial.println("Escaneando BLE...");
  Bluefruit.Scanner.start(0);
#else
  Serial.println("MODO DIAG: BLE/OBD deshabilitados (solo MQTT).");
  logDebug("MODO DIAG solo-MQTT  GNSS=" + String(DIAG_CON_GNSS));
#endif

  // El watchdog arranca al final: la inicialización es larga
  tUltimoPublish = millis();
  iniciarWatchdog();
}

// ============================================================
//  LOOP
// ============================================================
void loop()
{
  alimentarWatchdog();

  // Liveness: si hace demasiado que no se publica, reiniciar el equipo
  if (millis() - tUltimoPublish > LIVENESS_MS) {
    Serial.println("[LIVENESS] Demasiado tiempo sin publicar -> reinicio de recuperacion.");
#if DIAG_SOLO_MQTT
    bool bleConn = false;
#else
    bool bleConn = Bluefruit.Central.connected();
#endif
    logDebug("LIVENESS reset: " + String((millis() - tUltimoPublish) / 1000) +
             "s sin publicar. vbat=" + String(voltajeBateria, 2) +
             " mqtt=" + String(mqttOnline ? 1 : 0) +
             " ble=" + String(bleConn ? 1 : 0));
    Serial.flush();
    delay(50);
    NVIC_SystemReset();
  }

  // Heartbeat MQTT
  static uint32_t tHeartbeat = 0;
  if (!mqttOnline && millis() - tHeartbeat >= 60000) {
    tHeartbeat = millis();
    Serial.println("[HB] MQTT offline - reconectando...");
    reconectarMQTT();
  }

  // Batería y estado de carga cada 5s
  static uint32_t tBateria = 0;
  if (millis() - tBateria >= 5000) {
    tBateria = millis();
    voltajeBateria = leerNivelBateria();
    leerEstadoCarga();
    actualizarLEDCarga();
    Serial.printf("[BAT] %.2fV  cargando=%s  completa=%s\n",
                  voltajeBateria,
                  cargando ? "SI" : "NO",
                  cargaCompleta ? "SI" : "NO");
  }

#if DIAG_SOLO_MQTT
  // Modo diagnóstico: sin BLE/OBD, publica batería e IMU (y GNSS) cada ciclo
  {
    static uint32_t tProxDiag = 0;
    if (millis() >= tProxDiag) {
      tProxDiag = millis() + INTERVALO_CICLO;
      pedirGNSSCrudo();
      leerIMU();
      if (!mqttOnline) reconectarMQTT();
      if (mqttOnline) {
        String json = construirJSONraw();
        Serial.print("JSON: "); Serial.println(json);
        publicarMQTT(json);
      } else {
        Serial.println("MQTT offline - dato no enviado.");
      }
      if (sdLista) guardarCSVtraducido();
    }
  }
  return;
#endif

  if (!Bluefruit.Central.connected() || !elmChar.discovered()) return;

  switch (estado)
  {
    case IDLE:
      if (millis() >= tiempoProxCiclo) {
        pidActual = 0;
        bufELMIdx = 0; bufELM[0] = '\0'; elmListo = false;
        Serial.printf(">> [OBD] %s (%s)\n", pids[pidActual].nombre, pids[pidActual].comando);
        enviarComando(pids[pidActual].comando);
        tiempoEnvio = millis();
        estado = ESPERANDO_ELM;
      }
      break;

    case ESPERANDO_ELM:
      if (elmListo) {
        rawPID[pidActual] = String(bufELM);
        rawPID[pidActual].trim();
        // Quedarse sólo con la trama hex (descarta "SEARCHING...", "OK", etc.)
        {
          String limpio = limpiarOBD(rawPID[pidActual], pids[pidActual].pid);
          if (limpio.length() >= 6) rawPID[pidActual] = limpio;
          else if (rawPID[pidActual].indexOf("TIMEOUT") < 0) rawPID[pidActual] = "NO_DATA";
        }
        Serial.printf("<< %s = %s\n", pids[pidActual].nombre, rawPID[pidActual].c_str());
        elmListo = false;
        avanzarPID();
      } else if (millis() - tiempoEnvio > TIMEOUT_ELM) {
        Serial.printf("!! Timeout: %s\n", pids[pidActual].nombre);
        rawPID[pidActual] = "TIMEOUT";
        elmListo = false;
        avanzarPID();
      }
      break;

    case ESPERANDO_DTC:
      if (elmListo) {
        String d = String(bufELM); d.trim(); d.toUpperCase();
        // Sólo se guarda si es una respuesta de DTC ("43...")
        rawDTC = (d.indexOf("43") >= 0) ? d : "";
        Serial.printf("<< DTC = %s\n", rawDTC.c_str());
        elmListo = false;
        estado = HACIENDO_GNSS;
      } else if (millis() - tiempoEnvio > TIMEOUT_ELM) {
        Serial.println("!! Timeout: DTC");
        rawDTC = "";
        elmListo = false;
        estado = HACIENDO_GNSS;
      }
      break;

    case HACIENDO_GNSS:
      pedirGNSSCrudo();
      leerIMU();

      if (!mqttOnline) reconectarMQTT();

      if (mqttOnline) {
        String json = construirJSONraw();
        Serial.print("JSON: "); Serial.println(json);
        publicarMQTT(json);
      } else {
        Serial.println("MQTT offline - dato no enviado.");
      }

      if (sdLista) guardarCSVtraducido();

      tiempoProxCiclo = millis() + INTERVALO_CICLO;
      estado = IDLE;
      break;
  }
}

// ============================================================
//  Avanzar PID o pasar a GNSS
// ============================================================
void avanzarPID()
{
  pidActual++;
  bufELMIdx = 0; bufELM[0] = '\0';
  if (pidActual < NUM_PIDS) {
    delay(150);
    Serial.printf(">> [OBD] %s (%s)\n", pids[pidActual].nombre, pids[pidActual].comando);
    enviarComando(pids[pidActual].comando);
    tiempoEnvio = millis();
  } else {
    // Terminados los PIDs se piden los DTC (Modo 03)
    rawDTC = "";
    bufELMIdx = 0; bufELM[0] = '\0'; elmListo = false;
    delay(150);
    Serial.println(">> [OBD] DTC (03)");
    enviarComando("03");
    tiempoEnvio = millis();
    estado = ESPERANDO_DTC;
  }
}

// ============================================================
//  BLE callbacks
// ============================================================
void scan_callback(ble_gap_evt_adv_report_t* report)
{
  char name[32] = {0};
  Bluefruit.Scanner.parseReportByType(
    report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, (uint8_t*)name, sizeof(name));
  if (strlen(name) > 0) {
    Serial.printf("Encontrado: %-20s  RSSI: %d\n", name, report->rssi);
    if (strstr(name, "OBD") != NULL) {
      Bluefruit.Scanner.stop();
      Bluefruit.Central.connect(report);
      return;
    }
  }
  Bluefruit.Scanner.resume();
}

void connect_callback(uint16_t conn_handle)
{
  Serial.println("Conectado. Descubriendo servicios...");

  pidActual = 0;
  bufELMIdx = 0; bufELM[0] = '\0'; elmListo = false;
  estado = IDLE;

  if (!elmService.discover(conn_handle)) {
    Serial.println("ERROR: FFF0 - reiniciando scanner...");
    Bluefruit.Scanner.start(0);
    return;
  }
  if (!elmChar.discover()) {
    Serial.println("ERROR: FFF1 - reiniciando scanner...");
    Bluefruit.Scanner.start(0);
    return;
  }
  elmChar.enableNotify();
  delay(500);

  enviarComando("AT Z");
  delay(1000); enviarComando("AT E0");
  delay(300);  enviarComando("AT L0");
  delay(300);  enviarComando("AT S0");
  delay(300);  enviarComando("AT SP 0");
  // Esperar a que el ELM termine de inicializarse y descartar su respuesta
  delay(1200);
  bufELMIdx = 0; bufELM[0] = '\0'; elmListo = false;
  Serial.println("ELM327 listo");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  estado = IDLE;
  pidActual = 0; bufELMIdx = 0; bufELM[0] = '\0'; elmListo = false;
  Serial.printf("BLE desconectado (0x%02X)\n", reason);
  delay(500);
  Bluefruit.Scanner.start(0);
}

void notify_callback(BLEClientCharacteristic*, uint8_t* data, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++) {
    char c = (char)data[i];
    if (c == '>') { bufELM[bufELMIdx] = '\0'; elmListo = true; }
    else if (c != '\r' && c != '\n') {
      if (bufELMIdx < ELM_BUF_SIZE - 1) bufELM[bufELMIdx++] = c;
    }
  }
}

void enviarComando(const char* cmd)
{
  elmChar.write(cmd, strlen(cmd));
  uint8_t cr = 0x0D;
  elmChar.write(&cr, 1);
}

// ============================================================
//  GNSS: AT+CGNSSINFO
//  Se espera el "OK" posterior a la línea de datos, se corta en el
//  primer fin de línea y se guarda la línea completa.
// ============================================================
void pedirGNSSCrudo()
{
#if !DIAG_CON_GNSS
  rawGNSS = "GNSS_OFF";
  return;
#else
  // Encender el GNSS sólo una vez
  if (!gnssOn) {
    if (simSendCheck("AT+CGNSSPWR=1", "OK", 2000)) gnssOn = true;
  }

  rawGNSS = "NO_FIX";
  while (SIM_SERIAL.available()) SIM_SERIAL.read();
  SIM_SERIAL.println("AT+CGNSSINFO");

  String resp = "";
  resp.reserve(256);
  uint32_t t0 = millis();

  while (millis() - t0 < 3000) {
    while (SIM_SERIAL.available()) resp += (char)SIM_SERIAL.read();

    int p = resp.indexOf("+CGNSSINFO:");
    if (p >= 0 && resp.indexOf("OK", p) >= 0) {   // OK posterior a la trama
      delay(50);
      while (SIM_SERIAL.available()) resp += (char)SIM_SERIAL.read();
      break;
    }
    alimentarWatchdog();
    delay(5);
  }

  int ini = resp.indexOf("+CGNSSINFO:");
  if (ini >= 0) {
    String linea = resp.substring(ini + 11);

    // Cortar en el primer fin de línea
    int fin = linea.length();
    for (int i = 0; i < (int)linea.length(); i++) {
      if (linea[i] == '\r' || linea[i] == '\n') { fin = i; break; }
    }
    linea = linea.substring(0, fin);

    // Sin espacios ni "OK" residual
    linea.trim();
    linea.replace(" ", "");
    linea.replace("OK", "");

    // Hay fix si el campo 5 (latitud) no está vacío
    String lat = "";
    int idx = 0, pos = 0;
    for (int i = 0; i <= (int)linea.length(); i++) {
      if (i == (int)linea.length() || linea[i] == ',') {
        if (idx == 5) { lat = linea.substring(pos, i); break; }
        idx++;
        pos = i + 1;
      }
    }
    lat.trim();
    if (lat.length() > 0) rawGNSS = linea;
  }

  Serial.print("<< GNSS: ");
  Serial.println(rawGNSS);
#endif
}

// ============================================================
//  IMU LSM6DS33
// ============================================================
void leerIMU()
{
  if (!imuListo) return;

  sensors_event_t accel, gyro, temp;
  lsm6ds33.getEvent(&accel, &gyro, &temp);

  imu_ax = accel.acceleration.x;
  imu_ay = accel.acceleration.y;
  imu_az = accel.acceleration.z;
  imu_gx = gyro.gyro.x;
  imu_gy = gyro.gyro.y;
  imu_gz = gyro.gyro.z;

  Serial.printf("<< IMU  ax=%.3f ay=%.3f az=%.3f  gx=%.3f gy=%.3f gz=%.3f\n",
                imu_ax, imu_ay, imu_az, imu_gx, imu_gy, imu_gz);
}

// ============================================================
//  JSON crudo para MQTT
// ============================================================
String construirJSONraw()
{
  String j = "{";
  j.reserve(512);
  j += "\"ts\":" + String(millis()) + ",";
  j += "\"gnss\":\"" + rawGNSS + "\"";

  for (uint8_t i = 0; i < NUM_PIDS; i++) {
    j += ",\""; j += pids[i].nombre; j += "\":\""; j += rawPID[i]; j += "\"";
  }

  if (imuListo) {
    j += ",\"imu_ax\":";  j += String(imu_ax, 4);
    j += ",\"imu_ay\":";  j += String(imu_ay, 4);
    j += ",\"imu_az\":";  j += String(imu_az, 4);
    j += ",\"imu_gx\":";  j += String(imu_gx, 4);
    j += ",\"imu_gy\":";  j += String(imu_gy, 4);
    j += ",\"imu_gz\":";  j += String(imu_gz, 4);
  }

  j += ",\"vbat\":";     j += String(voltajeBateria, 2);
  j += ",\"cargando\":"; j += cargando ? "true" : "false";
  j += ",\"carga_ok\":"; j += cargaCompleta ? "true" : "false";
  j += ",\"dtc\":\"";    j += rawDTC; j += "\"";

  j += "}";
  return j;
}

// ============================================================
//  Publicar por MQTT
// ============================================================
void publicarMQTT(const String& payload)
{
  // Descartar mensajes pendientes del módem antes de publicar
  while (SIM_SERIAL.available()) SIM_SERIAL.read();

  String ctx = " vbat=" + String(voltajeBateria, 2);

  String topic = MQTT_TOPIC;
  String r = simSend("AT+CMQTTTOPIC=0," + String(topic.length()), 3000, ">");
  if (respuestaTieneReinicio(r)) {
    logDebug("PUB: reinicio del modem @TOPIC" + ctx);
    esperarBootModulo();
    return;
  }
  if (r.indexOf(">") < 0) {
    mqttOnline = false;
    logDebug("PUB FAIL @TOPIC" + ctx + " resp=" + limpiarResp(r));
    return;
  }
  SIM_SERIAL.print(topic);
  delay(200);

  r = simSend("AT+CMQTTPAYLOAD=0," + String(payload.length()), 3000, ">");
  if (respuestaTieneReinicio(r)) {
    logDebug("PUB: reinicio del modem @PAYLOAD" + ctx);
    esperarBootModulo();
    return;
  }
  if (r.indexOf(">") < 0) {
    mqttOnline = false;
    logDebug("PUB FAIL @PAYLOAD" + ctx + " resp=" + limpiarResp(r));
    return;
  }
  SIM_SERIAL.print(payload);
  delay(200);

  r = simSend("AT+CMQTTPUB=0," + String(MQTT_QOS) + ",60,0", 5000, "+CMQTTPUB:");
  if (respuestaTieneReinicio(r)) {
    logDebug("PUB: reinicio del modem @CMQTTPUB" + ctx);
    esperarBootModulo();
    return;
  }
  if (r.indexOf("+CMQTTPUB: 0,0") >= 0) {
    Serial.println("MQTT: OK");
    tUltimoPublish = millis();
    logDebug("PUB OK" + ctx + " len=" + String(payload.length()));
  } else {
    mqttOnline = false;
    logDebug("PUB FAIL @CMQTTPUB" + ctx + " resp=" + limpiarResp(r));
  }
}

// ============================================================
//  Limpiar la respuesta OBD del ELM327
//  Busca "41"+PID y devuelve la secuencia hex que sigue:
//    "410C40D0OKELM327 v2.1OKOKOK" -> "410C40D0"
//    "SEARCHING...410D02"          -> "410D02"
//    "OK" / "ELM327 v2.1"          -> "" (sin dato)
// ============================================================
String limpiarOBD(const String& bruto, uint8_t pid)
{
  String h = bruto;
  h.toUpperCase();
  h.replace(" ", "");

  char marc[8];
  sprintf(marc, "41%02X", pid);        // ej. "410C"
  int i = h.indexOf(marc);
  if (i < 0) return "";

  String out = "";
  for (int k = i; k < (int)h.length(); k++) {
    char c = h[k];
    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F')) out += c;
    else break;
  }
  return out;
}

// ============================================================
//  Decodificar DTC (Modo 03)
//  Entrada: respuesta cruda "43" + pares de bytes (cada DTC = 2 bytes).
//  Cada par se decodifica al formato estándar Pxxxx/Cxxxx/Bxxxx/Uxxxx.
//  Devuelve los códigos separados por ";" o "" si no hay ninguno.
//    "430133"      -> "P0133"
//    "4301330200"  -> "P0133;C0200"
// ============================================================
String decodificarDTC(const String& bruto)
{
  String h = bruto;
  h.toUpperCase();
  h.replace(" ", "");
  int i = h.indexOf("43");
  if (i < 0) return "";
  h = h.substring(i + 2);   // sacar el "43"

  const char letras[4] = {'P', 'C', 'B', 'U'};
  String codigos = "";
  for (int k = 0; k + 4 <= (int)h.length(); k += 4) {
    bool ok = true;
    for (int j = 0; j < 4; j++) {
      char c = h[k + j];
      if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) { ok = false; break; }
    }
    if (!ok) break;
    // b1/b2 en minúscula: B1 y B2 son macros del core (binary.h)
    uint8_t b1 = (uint8_t)strtol(h.substring(k, k + 2).c_str(), nullptr, 16);
    uint8_t b2 = (uint8_t)strtol(h.substring(k + 2, k + 4).c_str(), nullptr, 16);
    if (b1 == 0 && b2 == 0) continue;   // relleno = sin código
    char cod[6];
    sprintf(cod, "%c%d%X%X%X",
            letras[(b1 >> 6) & 0x03],
            (b1 >> 4) & 0x03,
            b1 & 0x0F,
            (b2 >> 4) & 0x0F,
            b2 & 0x0F);
    if (codigos.length() > 0) codigos += ";";
    codigos += String(cod);
  }
  return codigos;
}

// ============================================================
//  Descripción legible de un DTC (sin comas, por el CSV)
// ============================================================
String categoriaDTC(char letra)
{
  switch (letra) {
    case 'P': return "Motor";
    case 'C': return "Chasis";
    case 'B': return "Carroceria";
    case 'U': return "Red/Comunicacion";
    default:  return "Sistema";
  }
}

String descripcionDTC(const String& code)
{
  if (code == "P0100") return "Sensor de flujo de aire (MAF)";
  if (code == "P0101") return "Sensor MAF - rango/rendimiento";
  if (code == "P0110") return "Sensor de temperatura de admision";
  if (code == "P0115") return "Sensor de temperatura de refrigerante";
  if (code == "P0120") return "Sensor de posicion del acelerador";
  if (code == "P0128") return "Termostato de refrigerante";
  if (code == "P0171") return "Mezcla demasiado pobre";
  if (code == "P0172") return "Mezcla demasiado rica";
  if (code == "P0300") return "Fallo de encendido en varios cilindros";
  if (code == "P0301") return "Fallo de encendido cilindro 1";
  if (code == "P0302") return "Fallo de encendido cilindro 2";
  if (code == "P0303") return "Fallo de encendido cilindro 3";
  if (code == "P0304") return "Fallo de encendido cilindro 4";
  if (code == "P0420") return "Eficiencia del catalizador baja";
  if (code == "P0442") return "Fuga pequena en sistema EVAP";
  if (code == "P0500") return "Sensor de velocidad del vehiculo";
  if (code == "C0035") return "Sensor de velocidad rueda del. izq.";
  if (code == "C0110") return "Motor de bomba de ABS";
  if (code == "B0010") return "Airbag del conductor";
  if (code == "B1318") return "Tension de bateria baja";
  if (code == "U0100") return "Perdida de comunicacion con la ECU";
  if (code == "U0121") return "Perdida de comunicacion con ABS";
  if (code == "U1000") return "Comunicacion en red CAN";
  char l = code.length() > 0 ? code[0] : '?';
  if (l == 'P') return "Falla del motor / tren motriz";
  if (l == 'C') return "Falla del chasis";
  if (l == 'B') return "Falla de carroceria";
  if (l == 'U') return "Falla de red / comunicacion";
  return "Codigo desconocido";
}

// Construye una alerta legible a partir de "P0100;U1000".
String alertaDTC(const String& codigos)
{
  if (codigos.length() == 0) return "sin fallas";
  String alerta = "";
  int start = 0;
  while (start < (int)codigos.length()) {
    int sep = codigos.indexOf(';', start);
    if (sep < 0) sep = codigos.length();
    String c = codigos.substring(start, sep);
    if (c.length() > 0) {
      if (alerta.length() > 0) alerta += " | ";
      alerta += categoriaDTC(c[0]) + " en falla: " + descripcionDTC(c) + " [" + c + "]";
    }
    start = sep + 1;
  }
  return alerta;
}


// ============================================================
//  Traducir PID (para la SD)
// ============================================================
#define PID_ERROR -999999.0f
#define PID_ES_ERROR(v) ((v) == PID_ERROR)

float traducirPID(uint8_t pid, const String& hexStr)
{
  if (hexStr.length() < 6) return PID_ERROR;

  String h = hexStr;
  h.replace(" ", "");
  h.toUpperCase();
  if (h.length() < 6) return PID_ERROR;

  // Sólo tramas hex que empiecen con "41"+PID (descarta TIMEOUT, NO_DATA y ruido)
  char marc[8]; sprintf(marc, "41%02X", pid);
  if (!h.startsWith(marc)) return PID_ERROR;
  for (uint16_t k = 0; k < h.length(); k++) {
    char c = h[k];
    if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) return PID_ERROR;
  }

  uint8_t A = (uint8_t)strtol(h.substring(4, 6).c_str(), nullptr, 16);
  uint8_t B = (h.length() >= 8)
              ? (uint8_t)strtol(h.substring(6, 8).c_str(), nullptr, 16)
              : 0;

  switch (pid) {
    case 0x0C: return ((A * 256.0f) + B) / 4.0f;
    case 0x0D: return (float)A;
    case 0x05: return (float)A - 40.0f;
    case 0x0F: return (float)A - 40.0f;
    case 0x5C: return (float)A - 40.0f;
    case 0x04: return A * 100.0f / 255.0f;
    case 0x2F: return A * 100.0f / 255.0f;
    case 0x11: return A * 100.0f / 255.0f;
    case 0x0B: return (float)A;
    case 0x0A: return (float)A * 3.0f;
    case 0x10: return ((A * 256.0f) + B) / 100.0f;
    case 0x0E: return (float)A / 2.0f - 64.0f;
    case 0x1F: return (float)((A * 256) + B);
    case 0x21: return (float)((A * 256) + B);
    case 0x5E: return ((A * 256.0f) + B) / 20.0f;
    default:   return PID_ERROR;
  }
}

// ============================================================
//  SD
// ============================================================
bool iniciarSD()
{
  if (!SD.begin(SD_CS_PIN)) { Serial.println("SD: FAIL"); return false; }
  Serial.println("SD: OK");
  if (!SD.exists("/trad.csv")) {
    File f = SD.open("/trad.csv", FILE_WRITE);
    if (!f) return false;

    f.print("millis,fecha,hora,lat,lat_dir,lon,lon_dir,alt_m,vel_kph,dir_deg");
    for (uint8_t i = 0; i < NUM_PIDS; i++) {
      f.print(","); f.print(pids[i].nombre);
      f.print("("); f.print(pids[i].unidad); f.print(")");
    }
    f.print(",imu_ax(m/s2),imu_ay(m/s2),imu_az(m/s2)");
    f.print(",imu_gx(rad/s),imu_gy(rad/s),imu_gz(rad/s)");
    f.print(",vbat(V),cargando,carga_completa");
    f.print(",dtc");
    f.print(",dtc_alerta");
    f.println();
    f.close();
    Serial.println("SD: cabecera trad.csv creada");
  }
  return true;
}

void guardarCSVtraducido()
{
  File f = SD.open("/trad.csv", FILE_WRITE);
  if (!f) { Serial.println("SD: ERROR abriendo trad.csv"); return; }

  f.print(millis()); f.print(",");

  if (rawGNSS != "NO_FIX" && rawGNSS != "GNSS_OFF") {
    String g[18];
    int gIdx = 0, pos = 0;
    for (int i = 0; i <= (int)rawGNSS.length() && gIdx < 18; i++) {
      if (i == (int)rawGNSS.length() || rawGNSS[i] == ',') {
        g[gIdx++] = rawGNSS.substring(pos, i);
        pos = i + 1;
      }
    }

    String fecha = (gIdx > 9) ? g[9] : "";
    if (fecha.length() >= 6)
      f.print(fecha.substring(0,2) + "-" + fecha.substring(2,4) + "-" + fecha.substring(4,6));
    f.print(",");

    String hora = (gIdx > 10) ? g[10] : "";
    int puntoIdx = hora.indexOf('.');
    if (puntoIdx > 0) hora = hora.substring(0, puntoIdx);
    // sólo dígitos
    String horaLimpia = "";
    for (uint16_t k = 0; k < hora.length(); k++)
      if (isdigit(hora[k])) horaLimpia += hora[k];
    while (horaLimpia.length() < 6) horaLimpia = horaLimpia + "0";
    if (horaLimpia.length() >= 6)
      f.print(horaLimpia.substring(0,2) + ":" + horaLimpia.substring(2,4) + ":" + horaLimpia.substring(4,6));
    f.print(",");

    f.print((gIdx > 5) ? g[5] : ""); f.print(",");
    f.print((gIdx > 6) ? g[6] : ""); f.print(",");
    f.print((gIdx > 7) ? g[7] : ""); f.print(",");
    f.print((gIdx > 8) ? g[8] : ""); f.print(",");

    f.print((gIdx > 11) ? g[11] : "0"); f.print(",");   // alt

    float velKph = (gIdx > 12) ? g[12].toFloat() * 1.852f : 0.0f;  // vel (nudos->kph)
    f.print(velKph, 2); f.print(",");

    f.print((gIdx > 13) ? g[13] : "0");                 // rumbo
  } else {
    // Sin fix: 9 campos vacíos = 8 comas (la siguiente la agrega el bucle de PIDs)
    f.print(",,,,,,,,");
  }

  for (uint8_t i = 0; i < NUM_PIDS; i++) {
    f.print(",");
    float val = traducirPID(pids[i].pid, rawPID[i]);
    if (PID_ES_ERROR(val)) f.print("ERR");
    else                   f.print(val, 2);
  }

  if (imuListo) {
    f.print(","); f.print(imu_ax, 4);
    f.print(","); f.print(imu_ay, 4);
    f.print(","); f.print(imu_az, 4);
    f.print(","); f.print(imu_gx, 4);
    f.print(","); f.print(imu_gy, 4);
    f.print(","); f.print(imu_gz, 4);
  } else {
    f.print(",,,,,,");
  }

  f.print(","); f.print(voltajeBateria, 2);
  f.print(","); f.print(cargando ? 1 : 0);
  f.print(","); f.print(cargaCompleta ? 1 : 0);

  // DTC decodificados y alerta legible
  {
    String cod = decodificarDTC(rawDTC);
    f.print(","); f.print(cod.length() > 0 ? cod : "none");
    f.print(","); f.print(alertaDTC(cod));
  }

  f.println();
  f.close();
  Serial.println("SD: fila traducida guardada");
}

// ============================================================
//  wakeupSIM
// ============================================================
bool wakeupSIM()
{
  for (int i = 0; i < 5; i++) {
    alimentarWatchdog();
    while (SIM_SERIAL.available()) SIM_SERIAL.read();
    SIM_SERIAL.println("ATE0");
    delay(600);
    String r = "";
    uint32_t t0 = millis();
    while (millis() - t0 < 800) while (SIM_SERIAL.available()) r += (char)SIM_SERIAL.read();

    if (r.indexOf("OK") >= 0 || r.indexOf("ATE0") >= 0 || r.length() > 0) {
      String r2 = simSend("AT", 1500, "OK");
      if (r2.indexOf("OK") >= 0) return true;
    }
    delay(500);
  }
  return false;
}

// ============================================================
//  simSend / simSendCheck
// ============================================================
String simSend(const String& cmd, uint32_t timeout, const String& waitFor)
{
  while (SIM_SERIAL.available()) SIM_SERIAL.read();
  SIM_SERIAL.println(cmd);
  String resp = "";
  resp.reserve(128);
  uint32_t t0 = millis();
  bool matched = false;
  while (millis() - t0 < timeout) {
    while (SIM_SERIAL.available()) resp += (char)SIM_SERIAL.read();
    if (waitFor.length() > 0 && resp.indexOf(waitFor) >= 0) { matched = true; break; }
    alimentarWatchdog();
    delay(10);
  }
  // El token puede ser sólo el comienzo de la línea (ej. "+CMQTTPUB:"):
  // se sigue leyendo 200 ms para completarla
  if (matched) {
    uint32_t tg = millis();
    while (millis() - tg < 200) {
      while (SIM_SERIAL.available()) resp += (char)SIM_SERIAL.read();
      delay(5);
    }
  }
  return resp;
}

bool simSendCheck(const String& cmd, const String& expected, uint32_t timeout)
{
  String r = simSend(cmd, timeout, expected);
  bool ok = (r.indexOf(expected) >= 0);
  if (!ok) Serial.println("  FAIL: " + cmd + " -> " + r);
  return ok;
}

// ============================================================
//  iniciarRed
// ============================================================
bool iniciarRed()
{
  Serial.println("--- Iniciando red 4G ---");
  Serial.print("  [1/5] AT...");
  if (!wakeupSIM()) { Serial.println(" NO RESPONDE"); return false; }
  Serial.println(" OK");

  // Sólo LTE (CNMP=38): en 2G los picos de transmisión (~2 A) hacen
  // caer la tensión de la batería
  simSendCheck("AT+CNMP=38", "OK", 3000);

  Serial.print("  [2/5] Registro...");
  uint32_t t0 = millis();
  bool reg = false;
  while (millis() - t0 < 20000) {
    alimentarWatchdog();
    String r = simSend("AT+CGREG?", 2000, "+CGREG:");
    if (r.indexOf("+CGREG: 0,1") >= 0 || r.indexOf("+CGREG: 0,5") >= 0) { reg = true; break; }
    delay(2000);
    Serial.print(".");
  }
  if (!reg) {
    Serial.println(" SIN SENAL");
    logDebug("RED FAIL registro (sin senal) CSQ=" + String(leerCSQ()));
    return false;
  }
  Serial.println(" OK");
  logDebug("RED registro OK  CSQ=" + String(leerCSQ()));
  // Tecnología de red real (LTE o GSM)
  logDebug("RED CPSI=" + limpiarResp(simSend("AT+CPSI?", 2000, "+CPSI:")));

  Serial.print("  [3/5] APN...");
  if (!simSendCheck("AT+CGDCONT=1,\"IP\",\"" + String(APN) + "\"")) { Serial.println(" FALLO"); return false; }
  Serial.println(" OK");

  Serial.print("  [4/5] PDP...");
  simSendCheck("AT+CGACT=1,1", "OK", 15000);
  Serial.println(" OK");

  Serial.print("  [5/5] IP: ");
  String ip = simSend("AT+CGPADDR=1", 3000, "+CGPADDR:");
  int ini = ip.indexOf("+CGPADDR:");
  if (ini >= 0) { String l = ip.substring(ini); l.trim(); Serial.println(l); logDebug("RED IP=" + limpiarResp(l)); }
  else { Serial.println("(no obtenida)"); logDebug("RED IP no obtenida"); }

#if DIAG_CON_GNSS
  simSendCheck("AT+CGNSSPWR=1");
  gnssOn = true;
#else
  simSendCheck("AT+CGNSSPWR=0");
  gnssOn = false;
#endif
  delay(1000);
  Serial.println("  Red 4G lista");
  return true;
}

// ============================================================
//  teardownMQTT
// ============================================================
void teardownMQTT()
{
  Serial.println("  [MQTT] teardown...");
  simSend("AT+CMQTTDISC=0,10", 3000, "OK");
  simSend("AT+CMQTTRELCLIENT=0", 2000, "OK");
  simSend("AT+CMQTTSTOP", 5000, "+CMQTTSTOP: 0");
  delay(1000);
  while (SIM_SERIAL.available()) SIM_SERIAL.read();
}

// ============================================================
//  iniciarMQTT
// ============================================================
bool iniciarMQTT()
{
  Serial.println("--- Iniciando MQTT ---");
  teardownMQTT();

  Serial.print("  [MQTT] START... ");
  String r = simSend("AT+CMQTTSTART", 5000, "+CMQTTSTART:");
  if (respuestaTieneReinicio(r)) {
    Serial.println("REINICIO MODEM");
    logDebug("MQTT: reinicio del modem @START resp=" + limpiarResp(r));
    esperarBootModulo();
    return false;
  }
  // Si START da ERROR, STOP y un reintento
  if (r.indexOf("+CMQTTSTART:") < 0 && r.indexOf("ERROR") >= 0) {
    logDebug("MQTT @START ERROR -> STOP y reintento");
    simSend("AT+CMQTTSTOP", 5000, "OK");
    delay(500);
    r = simSend("AT+CMQTTSTART", 5000, "+CMQTTSTART:");
  }
  if (r.indexOf("+CMQTTSTART: 0") < 0 && r.indexOf("+CMQTTSTART: 23") < 0) {
    Serial.println("FALLO -> " + r);
    logDebug("MQTT FAIL @START resp=" + limpiarResp(r));
    return false;
  }
  Serial.println("OK");
  delay(500);

  Serial.print("  [MQTT] ACCQ... ");
  r = simSend("AT+CMQTTACCQ=0,\"" + mqttClientID + "\"", 5000, "OK");
  if (respuestaTieneReinicio(r)) {
    Serial.println("REINICIO MODEM");
    logDebug("MQTT: reinicio del modem @ACCQ resp=" + limpiarResp(r));
    esperarBootModulo();
    return false;
  }
  if (r.indexOf("OK") < 0) {
    Serial.println("FALLO -> " + r);
    logDebug("MQTT FAIL @ACCQ resp=" + limpiarResp(r));
    return false;
  }

  Serial.print("  [MQTT] CONNECT... ");
  String connCmd = "AT+CMQTTCONNECT=0,\"tcp://" + String(MQTT_BROKER) + ":" +
                   String(MQTT_PORT) + "\"," + String(MQTT_KEEPALIVE) + ",1";
  if (strlen(MQTT_USER) > 0)
    connCmd += ",\"" + String(MQTT_USER) + "\",\"" + String(MQTT_PASS) + "\"";

  r = simSend(connCmd, 10000, "+CMQTTCONNECT:");
  if (respuestaTieneReinicio(r)) {
    Serial.println("REINICIO MODEM");
    logDebug("MQTT: reinicio del modem @CONNECT resp=" + limpiarResp(r));
    esperarBootModulo();
    return false;
  }
  if (r.indexOf("+CMQTTCONNECT: 0,0") < 0) {
    Serial.println("FALLO -> " + r);
    logDebug("MQTT FAIL @CONNECT vbat=" + String(voltajeBateria, 2) +
             " resp=" + limpiarResp(r));
    return false;
  }
  Serial.println("MQTT: conectado");
  logDebug("MQTT CONNECT OK vbat=" + String(voltajeBateria, 2));
  return true;
}

// ============================================================
//  reconectarMQTT
// ============================================================
void reconectarMQTT()
{
  Serial.println("MQTT: ciclo de reconexion...");
  logDebug("RECONECT inicio vbat=" + String(voltajeBateria, 2));
  teardownMQTT();

  simListo = false;
  if (iniciarRed()) simListo = true;
  else { Serial.println("MQTT: red caida - reintento postergado."); return; }

  if (iniciarMQTT()) mqttOnline = true;
  else Serial.println("MQTT: reintento fallido.");
}

// ============================================================
//  BATERÍA
// ============================================================
float leerNivelBateria()
{
  digitalWrite(PIN_ACTIVAR_NIVEL, NIVEL_ACTIVO);
  delay(BAT_SETTLE_MS);

  uint32_t acum = 0;
  for (int i = 0; i < BAT_OVERSAMPLE; i++) acum += analogRead(PIN_NIVEL_BAT);
  float lectura = (float)acum / (float)BAT_OVERSAMPLE;

  digitalWrite(PIN_ACTIVAR_NIVEL, NIVEL_INACTIVO);

  float vAdc = (lectura / ADC_MAX) * ADC_VREF;
  float vBat = vAdc * DIV_RATIO * CAL_BAT;
  return vBat;
}

// ============================================================
//  ESTADO DE CARGA (TP4056)
// ============================================================
void leerEstadoCarga()
{
  cargando      = (digitalRead(PIN_CHRG)  == LOW);
  cargaCompleta = (digitalRead(PIN_STDBY) == LOW);
}

// ============================================================
//  LED de estado de carga
// ============================================================
void actualizarLEDCarga()
{
  digitalWrite(LED_BUILTIN, cargando ? HIGH : LOW);
}

// ============================================================
//  A7670SA - pulso de encendido en PWRKEY
// ============================================================
void pulsarPWRKEY_7670()
{
  digitalWrite(PIN_PWRKEY_7670, HIGH);
  delay(100);
  digitalWrite(PIN_PWRKEY_7670, LOW);
  delay(1000);
  digitalWrite(PIN_PWRKEY_7670, HIGH);
  delay(3000);
}
