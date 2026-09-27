 /*------------------------------------------------------------------------------
-- Project : Sistema de monitoreo de fallas y datos de manejo de vehículos
-------------------------------------------------------------------------------
-- File : monitoreo-ahorro_bateria.INO (Compile in Adaftruit NRF52840 Sense)
-- Author : Preves, Santiago.
-- Date : Sep 10, 2026.
-- Rev 10 :
--
-------------------------------------------------------------------------------
-- Description:
  Version mejorada de monitoreo_sin_gestion.INO, suma un modo de ahorro de bateria
  en el que el consumo se lleva al minimo posible para mejorar la eficiencia
--               
-------------------------------------------------------------------------------*/
// ============================================================
//  OBD + GPS → MQTT (RAW) + SD (TRADUCIDO)   —   v2 (revisado)
//  Hardware : Adafruit Feather nRF52840 Sense
//  BLE      : ELM327  (servicio FFF0 / característica FFF1)
//  UART     : A7670SA en Serial1  (TX=pin1, RX=pin0)
//  SPI      : SD card  (CS = A0 / CHIPSEL_BLUEF)
//
//  ── Mapa de pines (según esquemático) ──────────────────
//    A0  -> CHIPSEL_BLUEF   (CS de la SD)
//    A1  -> SLEEP_7670      (control de sleep del A7670SA)
//    A3  -> NIVEL_BAT       (lectura ADC del divisor de batería)  
//    A5  -> PWRKEY_7670     (encendido/apagado del A7670SA)        
//    D3  -> CHRG            (open-drain TP4056, activo en LOW)
//    D4  -> STDBY           (open-drain TP4056, activo en LOW)
//    D5  -> ACTIVAR_NIVEL   (habilita el divisor de batería vía Q2)
//
//  ── CAMBIOS v2 ──────────────────────────────────────────────
//  [C1] Comentario de pines corregido (estaba desfasado del código/esquemático).
//  [C2] Batería: el ADC del nRF52 NO es 0-3.3V, su fondo de escala por
//       defecto es 3.6V. El código usaba 3.3 y por eso medía ~9% de menos
//       (4.05V reales -> ~3.6V). Ahora: referencia fija AR_INTERNAL (3.6V),
//       promediado de varias muestras, y factor de calibración CAL_BAT.
//  [C3] GNSS: se esperaba mal el fin de la respuesta y se colaba el "OK"
//       del AT en el campo de hora ("1341OK") y se truncaban vel/rumbo/sats.
//       Ahora se espera el "OK" posterior al +CGNSSINFO, se corta en el
//       primer fin de línea real y se sanitiza. Se pasa la línea COMPLETA.
//  [C4] Watchdog HW del nRF52: si algo se cuelga, reinicia solo.
//  [C5] "Liveness": si pasa demasiado tiempo sin publicar por MQTT,
//       reinicia para recuperar la transmisión (atiende el colgado c/5-7min).
// ============================================================

#include <Arduino.h>
#include <bluefruit.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>
#include <Adafruit_LSM6DS33.h>

// ============================================================
//  UART — A7670SA
// ============================================================
#define SIM_SERIAL  Serial1
#define SIM_BAUD    115200

// ============================================================
//  SD
// ============================================================
#define SD_CS_PIN   A0
bool sdLista = false;

// ============================================================
//  MEDICIÓN DE NIVEL DE BATERÍA
//  ACTIVAR_NIVEL (D5) habilita el divisor V_BAT->R15->NIVEL_BAT->R16->GND
//  a través del MOSFET Q2 (P-channel, high-side). NIVEL_BAT (A3) es la lectura ADC.
// ============================================================
#define PIN_ACTIVAR_NIVEL  12     // D5
#define PIN_NIVEL_BAT      A3      // [C1] NIVEL_BAT está en A3 (no A2)

// Relación del divisor: Vbat = Vadc * (R15+R16)/R16
#define DIV_R_TOP   10000.0f    // R15
#define DIV_R_BOT   10000.0f    // R16
#define DIV_RATIO   ((DIV_R_TOP + DIV_R_BOT) / DIV_R_BOT)   // = 2.0

// [C2] Parámetros del ADC del nRF52840
#define ADC_VREF        3.6f     // fondo de escala real con AR_INTERNAL (0.6V, gain 1/6)
#define ADC_MAX         4095.0f  // 12 bits
#define BAT_OVERSAMPLE  16       // promediado para bajar ruido
#define BAT_SETTLE_MS   30       // estabilización del divisor tras encenderlo
// Factor de calibración
#define CAL_BAT         1.00f

// Nivel del MOSFET Q2
#define NIVEL_ACTIVO   LOW
#define NIVEL_INACTIVO HIGH

float voltajeBateria = 0.0f;

// ============================================================
//  DETECCIÓN DE CARGA Activas en LOW.
// ============================================================
#define PIN_CHRG    10  // D3
#define PIN_STDBY   11  // D4

bool cargando       = false;
bool cargaCompleta  = false;

// ============================================================
//  A7670SA
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
//  MODO DIAGNÓSTICO DE ALIMENTACIÓN
//  Para aislar el consumo y ver si el módem transmite con batería
//  cuando NO compite con el BLE ni el GNSS.
//    DIAG_SOLO_MQTT = 1 -> NO arranca BLE/OBD; solo publica
//                          batería+IMU en un timer (mínimo consumo).
//    DIAG_CON_GNSS  = 0 -> apaga el GNSS (ahorra ~30mA continuos).
//  Combinaciones para probar:
//    (1,0) solo MQTT sin GNSS  = consumo MÍNIMO
//    (1,1) solo MQTT con GNSS
//    (0,1) sistema normal completo (volver a producción)
//    (0,0) normal pero sin GNSS
// ============================================================
#define DIAG_SOLO_MQTT   0
#define DIAG_CON_GNSS    1

// ============================================================
//  MODO DE AHORRO DE ENERGÍA
//  Ciclo "despertar -> adquirir -> transmitir -> dormir": entre
//  transmisiones se APAGA el BLE y se BAJA la parte de red/MQTT
//  (y, opcionalmente, el GNSS), quedando el módem en sleep. Cada
//  transmisión CIERRA la conexión MQTT (DISC+STOP) para que el
//  broker vea la baja y no quede el socket colgado.
//
//    MODO_AHORRO = 1 -> nuevo ciclo con sleep (5 min).
//    MODO_AHORRO = 0 -> comportamiento continuo original (15 s).
// ============================================================
#define MODO_AHORRO   2

// Reintentos de transmisión por ciclo antes de rendirse y dormir
#define AHORRO_MAX_INTENTOS_TX  2

// Ciclos consecutivos con TX fallida antes de forzar
// un reinicio total (NVIC_SystemReset). Reemplaza al liveness (desactivado en
// ahorro): saca al equipo de un cuelgue LÓGICO del módem que el watchdog HW no
// detecta (el firmware corre y alimenta el WDT, pero nunca transmite).
#define AHORRO_MAX_CICLOS_FALLIDOS  3

// ---- Estrategia del MÓDEM entre ciclos --------------------------------
//  Qué hacer con el A7670SA entre transmisiones. En TODOS los
// modos el socket MQTT se cierra (DISC+STOP) al final del ciclo.
//   AHORRO_MODO_MODEM = 0 -> [DEFAULT, ROBUSTO] el módem queda ENCENDIDO y
//        REGISTRADO; sólo se cierra MQTT y se baja el PDP (AT+CGACT=0). El
//        módem SIEMPRE responde a AT, así que despertarlo nunca falla. Idle
//        LTE ~10-20 mA. 
//   AHORRO_MODO_MODEM = 1 -> CSCLK sleep por DTR (menor consumo).
//   AHORRO_MODO_MODEM = 2 -> power-down TOTAL (AT+CPOF). Consumo mínimo, pero
//        cada ciclo rehace el attach de red (~15-25 s), gasta más energía en
//        ese attach y BORRA las efemérides del GNSS.
#define AHORRO_MODO_MODEM   2

// ---- Estrategia del GNSS ----------------------------------------------
//   GNSS_MANTENER_ON = 1 -> el GNSS queda ALIMENTADO siempre. Garantiza fix
//        rápido en cada ventana (hot start "gratis"), a costa de ~30 mA
//        continuos. Es lo más confiable para tener posición siempre.
//   GNSS_MANTENER_ON = 0 -> se apaga el GNSS entre ciclos y se intenta un
//        HOT/WARM START al despertar (AT+CGNSSPWR=1). Ahorra esos ~30 mA,
//        pero el TTFF depende de que el módulo RETENGA las efemérides en su
//        RAM de respaldo (NO combinar con AHORRO_MODO_MODEM=2).
#define GNSS_MANTENER_ON     1
#define GNSS_USAR_AGPS       1          // 1 = pedir efemérides por red (AT+CAGPS) tras levantar el PDP
#define GNSS_FIX_TIMEOUT_MS  45000UL    // máx. espera de un fix al despertar

// ============================================================
//  WATCHDOG / LIVENESS   [C4][C5]
// ============================================================
#define WDT_TIMEOUT_S   20UL       // reinicia si el firmware se congela > 20s
#if MODO_AHORRO
#define LIVENESS_MS     0UL
#else
#define LIVENESS_MS     240000UL   // 4 min sin publicar -> reinicio de recuperación
#endif
uint32_t tUltimoPublish = 0;

// ============================================================
//  LOG DE DEBUG DEL A7670SA EN LA SD
//  Registra en /debug.log cada intento de transmisión con su
//  contexto (batería, señal CSQ, respuesta cruda del módem) y el
//  motivo de cada reinicio.
// ============================================================
#define DEBUG_LOG_FILE   "/debug.log"
#define DEBUG_LOG_MAX    1048576UL   // 1 MB: si supera, se reinicia el archivo
String g_motivoReset = "";           // se completa al arrancar

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
  bool        enSD;     //            true = se traduce y guarda en la SD (caja negra).
                        //            El MQTT sigue enviando TODOS los PID crudos.
};

// Columna en SD: en modo caja negra sólo se traducen/guardan en la
// SD las magnitudes con valor forense/de conducción. El resto (diagnóstico
// fino) se sigue transmitiendo CRUDO por MQTT.
//   SE MANTIENEN en SD:  RPM, vel_kph, temp_mot_c, carga_pct, accel_pct
const PIDEntry pids[] = {
  { "01 0C", "RPM",          0x0C, "RPM",  true  },  // régimen de motor  (SD)
  { "01 0D", "vel_kph",      0x0D, "km/h", true  },  // velocidad         (SD)
  { "01 05", "temp_mot_c",   0x05, "C",    true  },  // temp refrigerante (SD)
  { "01 0F", "temp_adm_c",   0x0F, "C",    false },
  { "01 04", "carga_pct",    0x04, "%",    true  },  // carga del motor   (SD)
  { "01 2F", "comb_pct",     0x2F, "%",    false },
  { "01 11", "accel_pct",    0x11, "%",    true  },  // pos. acelerador   (SD)
  { "01 0B", "map_kpa",      0x0B, "kPa",  false },
  { "01 10", "maf_gs",       0x10, "g/s",  false },
  { "01 0A", "pcomb_kpa",    0x0A, "kPa",  false },
  { "01 0E", "avance_deg",   0x0E, "deg",  false },
  { "01 1F", "ton_s",        0x1F, "s",    false },
  { "01 21", "dist_mil_km",  0x21, "km",   false },
  { "01 5C", "temp_ace_c",   0x5C, "C",    false },
  { "01 5E", "cons_lh",      0x5E, "L/h",  false },
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
// La inicialización del ELM327 (AT Z/E0/L0/S0/SP0)
// ocurre DENTRO de connect_callback, DESPUÉS de que elmChar.discovered() ya
// dio true. Esta bandera se pone en true SÓLO cuando esa init terminó, para
// no empezar a pedir PIDs mientras el callback todavía manda "AT Z"
volatile bool elmInicializado = false;

// Candados para evitar que scan_callback reactive el scanner
// con paquetes rezagados o que connect_callback prenda el LED fuera de ciclo.
volatile bool blePermitido  = false;
volatile bool bleConectando = false;

// Handle del enlace BLE central actual (para desconectar limpio al
// terminar el sondeo OBD). Lo setean connect_callback / disconnect_callback.
volatile uint16_t g_connHandle = BLE_CONN_HANDLE_INVALID;

#if MODO_AHORRO
// Estado del módem: true si quedó apagado (modo 2) o si una recuperación por
// HW lo dejó recién encendido. Evita re-pulsar PWRKEY sobre un módem que ya
// está ON (lo que lo APAGARÍA).
bool g_modemApagado = false;
// true si el ciclo anterior no pudo transmitir. El
// próximo wake hará AT+CRESET del módem para limpiar una pila MQTT trabada
// (SIMCom responde OK al AT pero ERROR al CMQTTSTART) antes de reintentar.
bool g_ultimoCicloFallo = false;
#endif

// ============================================================
//  MÁQUINA DE ESTADOS
// ============================================================
enum Estado : uint8_t { IDLE, ESPERANDO_ELM, ESPERANDO_DTC, HACIENDO_GNSS }; 
volatile Estado estado = IDLE;

uint8_t  pidActual       = 0;
uint32_t tiempoEnvio     = 0;
uint32_t tiempoProxCiclo = 0;

const uint32_t TIMEOUT_ELM     = 3000;
#if MODO_AHORRO
const uint32_t INTERVALO_CICLO = 300000;   // 5 min entre ciclos
#else
const uint32_t INTERVALO_CICLO = 15000;
#endif

// ============================================================
//  GNSS crudo
// ============================================================
String rawGNSS = "NO_FIX";
String rawDTC  = ""; 

// ============================================================
//  FLAGS estado MQTT/4G
// ============================================================
bool simListo   = false;
bool mqttOnline = false;
// Servicio MQTT (CMQTTSTART + CMQTTACCQ) arrancado y vivo. 
bool mqttSvcUp  = false;
bool gnssOn     = false;   // GNSS encendido (para no re-encender cada ciclo)
// Diagnóstico del último intento de fix (satélites vistos)
// y flag de configuración inicial del receptor (READY + modo), para no repetir.
String gnssDiag        = "sats=?";
bool   gnssConfigurado = false;

// ============================================================
//  IMU — LSM6DS33
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
void  apagarPWRKEY_7670();                             

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

#if MODO_AHORRO
bool    ahorroConectarELM(uint32_t timeoutMs); 
void    ahorroApagarBLE();                     
void    ahorroPollPID(uint8_t i);                 
void    ahorroPollDTC();                         
void    ahorroApagarGNSS();                       
void    ahorroPrepararGNSS();                    
bool    ahorroDespertarModem();                   
void    ahorroDormirModem();                      
void    cicloAhorro();                           
#endif

// ============================================================
//  WATCHDOG HW (nRF52)   [C4]
//  Una vez arrancado no se puede parar ni reconfigurar; hay que
//  recargarlo antes de que venza o el chip se reinicia solo.
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
  // Si el WDT todavía no arrancó, escribir acá no tiene efecto (es inofensivo).
  NRF_WDT->RR[0] = WDT_RR_RR_Reload;
}

// ============================================================
//  DEBUG LOG  
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
  Serial.print("[LOG] "); Serial.println(msg);   // eco por serie también
}

// Lee y limpia el registro de motivo de reinicio del nRF52.
// Distingue: WATCHDOG (colgado real), SOFT (reinicio de liveness),
// PIN (reset manual) y POWERON/BROWNOUT (corte de alimentación).
String motivoReset()
{
  uint32_t r = NRF_POWER->RESETREAS;
  NRF_POWER->RESETREAS = 0xFFFFFFFF;   
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
//  DETECCIÓN DE REINICIO DEL A7670SA   [C7]
//  Si una respuesta trae el "banner" de arranque del módem, es que
//  se rebooteó en medio de la operación. En vez de seguir tirando
//  comandos a un módem a medio arrancar, esperamos a que termine.
// ============================================================
bool respuestaTieneReinicio(const String& r)
{
  return (r.indexOf("*ATREADY")     >= 0) ||   // A7670 listo tras boot
         (r.indexOf("+CPIN: READY") >= 0) ||   // SIM reinicializada
         (r.indexOf("*ISIMAID")     >= 0);      // banner de arranque
}

// Espera a que el módem termine de bootear: drena el puerto hasta que
// haya ~1.5s de silencio (o timeout), y vuelve a silenciar el eco.
void esperarBootModulo()
{
  logDebug("MODEM reinicio detectado -> esperando boot completo");   // [C7]
  uint32_t t0 = millis();
  uint32_t tUltByte = millis();
  while (millis() - t0 < 12000) {
    alimentarWatchdog();
    bool hubo = false;
    while (SIM_SERIAL.available()) { SIM_SERIAL.read(); hubo = true; tUltByte = millis(); }
    (void)hubo;
    if (millis() - tUltByte > 1500) break;   // silencio sostenido = boot terminado
    delay(20);
  }
  simSend("ATE0", 1000, "OK");   // re-silenciar eco (se perdió en el reboot)
  // El contexto de red y MQTT quedó perdido: forzar reinicialización limpia.
  simListo   = false;
  mqttOnline = false;
  mqttSvcUp  = false;   // el reboot perdió el servicio MQTT
  gnssOn     = false;   // el reboot apaga el GNSS: hay que reencenderlo
  gnssConfigurado = false;   // re-configurar READY+modo tras el reboot
  logDebug("MODEM boot completo -> se reinicializará red+MQTT");   
}

// ============================================================
//  SETUP
// ============================================================
void setup()
{
  // Leer el motivo del último reinicio ANTES de que algo lo pise.
  g_motivoReset = motivoReset();

  delay(3000);
  Serial.begin(115200);

  Serial.println("=== OBD + GPS → MQTT RAW + SD TRADUCIDO (v2) ===");
  Serial.print("Motivo del último reinicio: ");
  Serial.println(g_motivoReset);

  randomSeed(micros());
  mqttClientID += String(random(10000, 99999));
  Serial.print("MQTT Client ID asignado: ");
  Serial.println(mqttClientID);

  // ── PINES ──
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

  // ADC: fijar referencia y resolución UNA vez.
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

#if !MODO_AHORRO
  if (iniciarRed())  simListo = true;
  else Serial.println("AVISO: red 4G no disponible, se reintentará.");

  if (simListo && iniciarMQTT()) mqttOnline = true;
  else Serial.println("AVISO: MQTT offline, se reintentará.");
#else
  // La red y el MQTT se levantan por ciclo (no en setup) para no
  // tenerlos enganchados. El módem ya quedó encendido por PWRKEY + CRESET.
  Serial.println("[AHORRO] Red/MQTT se levantan por ciclo (no en setup).");
#endif

  if (iniciarSD()) sdLista = true;

  // Rotar el log si quedó muy grande, y dejar la marca de arranque
  // con el motivo del reinicio y la batería medida en ese momento.
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

  // ── IMU ──
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
  // ── BLE ──
  Bluefruit.begin(0, 1);
  Bluefruit.setTxPower(4);
  Bluefruit.setName("Bluefruit-ELM");
  Bluefruit.autoConnLed(false); // Desactivo el manejo automatico del LED
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_BLUE, LOW);
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  elmService.begin();
  elmChar.begin();
  elmChar.setNotifyCallback(notify_callback);
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.useActiveScan(true);

#if MODO_AHORRO
  blePermitido  = false;
  bleConectando = false;
  Serial.println("[AHORRO] BLE configurado; scanner en reposo (se activa por ciclo).");
#else
  blePermitido  = true;
  bleConectando = false;
  Serial.println("Escaneando BLE...");
  Bluefruit.Scanner.start(0);
#endif
#else
  // Modo diagnóstico: BLE apagado para no competir por corriente.
  Serial.println("MODO DIAG: BLE/OBD deshabilitados (solo MQTT).");
  logDebug("MODO DIAG solo-MQTT  GNSS=" + String(DIAG_CON_GNSS));
#endif

  // Arrancar watchdog y contador de liveness al final del setup,
  // para no reiniciar durante la inicialización (que es larga).
  tUltimoPublish = millis();
  iniciarWatchdog();

#if MODO_AHORRO
  tiempoProxCiclo = millis();   // disparar el primer ciclo enseguida
#endif
}

// ============================================================
//  LOOP
// ============================================================
void loop()
{
  alimentarWatchdog();   // [C4]

  // Liveness: si hace demasiado que no publicamos, reiniciar para recuperar.
  // LIVENESS_MS==0 lo desactiva (en ahorro el módem duerme adrede).
  if (LIVENESS_MS != 0 && millis() - tUltimoPublish > LIVENESS_MS) {
    Serial.println("[LIVENESS] Demasiado tiempo sin publicar -> reinicio de recuperación.");
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

#if MODO_AHORRO
  // Ciclo despertar/dormir. Fuera de la ventana de transmisión
  // todo queda apagado/dormido; sólo mantenemos vivo el watchdog. Cuando se
  // cumple el intervalo (5 min) se ejecuta un ciclo completo y se re-agenda.
  if (millis() >= tiempoProxCiclo) {
    cicloAhorro();
    tiempoProxCiclo = millis() + INTERVALO_CICLO;
  } else {
    uint32_t restante = tiempoProxCiclo - millis();
    uint32_t paso = (restante > 2000UL) ? 2000UL : restante;
    alimentarWatchdog();   // mantener el WDT durante el sueño
    delay(paso);           // (dormido en trozos < WDT_TIMEOUT_S)
  }
  return;   // en ahorro NO se corre la máquina de estados continua de abajo
#endif

  // ── Heartbeat MQTT ──
  static uint32_t tHeartbeat = 0;
  if (!mqttOnline && millis() - tHeartbeat >= 60000) {
    tHeartbeat = millis();
    Serial.println("[HB] MQTT offline — reconectando...");
    reconectarMQTT();
  }

  // ── Batería y estado de carga cada 5s ──
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
  // Camino de diagnóstico: sin BLE/OBD. Publica batería+IMU (+GNSS
  // si DIAG_CON_GNSS) en un timer, para medir si el módem transmite con
  // batería cuando es lo único que consume corriente.
  {
    static uint32_t tProxDiag = 0;
    if (millis() >= tProxDiag) {
      tProxDiag = millis() + INTERVALO_CICLO;
      pedirGNSSCrudo();          // respeta DIAG_CON_GNSS internamente
      leerIMU();
      if (!mqttOnline) reconectarMQTT();
      if (mqttOnline) {
        String json = construirJSONraw();
        Serial.print("JSON: "); Serial.println(json);
        publicarMQTT(json);
      } else {
        Serial.println("MQTT offline — dato no enviado.");
      }
      if (sdLista) guardarCSVtraducido();
    }
  }
  return;   // en modo diagnóstico no se ejecuta la máquina de estados OBD
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
        // Limpiar ruido del ELM327 ("ELM327 v2.1", "OK", "SEARCHING...")
        // que se cuela en el primer ciclo. Extrae solo la respuesta hex válida.
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

    case ESPERANDO_DTC:   //respuesta al Modo 03
      if (elmListo) {
        String d = String(bufELM); d.trim(); d.toUpperCase();
        // Guardar solo si es una respuesta de DTC válida ("43..."); si no,
        // queda vacío (sin fallas / sin soporte del simulador).
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
        Serial.println("MQTT offline — dato no enviado.");
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
    // Terminados los PID, pedir DTCs (Modo 03) antes del GNSS.
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
  //Si la ventana BLE ya se cerró o ya estamos conectando al
  // ELM327, ignorar paquetes rezagados para no hacer Scanner.resume().
  if (!blePermitido || bleConectando) return;

  char name[32] = {0};
  Bluefruit.Scanner.parseReportByType(
    report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, (uint8_t*)name, sizeof(name));
  if (strlen(name) > 0) {
    Serial.printf("Encontrado: %-20s  RSSI: %d\n", name, report->rssi);
    if (strstr(name, "OBD") != NULL) {
      bleConectando = true;
      Bluefruit.Scanner.stop();
      Bluefruit.Central.connect(report);
      return;
    }
  }
  if (blePermitido && !bleConectando) {
    Bluefruit.Scanner.resume();
  }
}

void connect_callback(uint16_t conn_handle)
{
  // Si la conexión se concretó cuando ya se había mandado a
  // apagar el BLE, cortar inmediatamente sin prender el LED.
  if (!blePermitido) {
    Bluefruit.disconnect(conn_handle);
    digitalWrite(LED_BLUE, LOW);
    return;
  }

  Serial.println("Conectado. Descubriendo servicios...");

  g_connHandle = conn_handle;   // recordar el handle para desconectar luego

  pidActual = 0;
  bufELMIdx = 0; bufELM[0] = '\0'; elmListo = false;
  elmInicializado = false;   // recién listo al terminar la init
  estado = IDLE;

  if (!elmService.discover(conn_handle)) {
    Serial.println("ERROR: FFF0 — reiniciando scanner...");
    bleConectando = false;
    if (blePermitido) Bluefruit.Scanner.start(0);
    return;
  }
  if (!elmChar.discover()) {
    Serial.println("ERROR: FFF1 — reiniciando scanner...");
    bleConectando = false;
    if (blePermitido) Bluefruit.Scanner.start(0);
    return;
  }
  elmChar.enableNotify();
  delay(500);

  enviarComando("AT Z");
  delay(1000); enviarComando("AT E0");
  delay(300);  enviarComando("AT L0");
  delay(300);  enviarComando("AT S0");
  delay(300);  enviarComando("AT SP 0");
  // Dar tiempo a que el ELM termine su init y descartar el banner
  // ("ELM327 v2.1", "OK", "SEARCHING...") antes de pedir el primer PID.
  delay(1200);
  bufELMIdx = 0; bufELM[0] = '\0'; elmListo = false;

  // Verificar que la ventana BLE no haya expirado durante los
  // 3.6 s de delays de inicialización antes de encender el LED.
  if (!blePermitido) {
    Bluefruit.disconnect(conn_handle);
    digitalWrite(LED_BLUE, LOW);
    return;
  }

  elmInicializado = true;   //ahora sí es seguro pedir PIDs
  digitalWrite(LED_BLUE, HIGH);
  Serial.println("ELM327 listo");
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  g_connHandle    = BLE_CONN_HANDLE_INVALID;   // handle ya no válido
  estado          = IDLE;
  pidActual       = 0; bufELMIdx = 0; bufELM[0] = '\0'; elmListo = false;
  elmInicializado = false;   
  bleConectando   = false;   
  digitalWrite(LED_BLUE, LOW);
  Serial.printf("BLE desconectado (0x%02X)\n", reason);
#if !MODO_AHORRO
  //  En ahorro NO relanzamos el scanner acá: el BLE debe quedar
  // apagado entre ciclos. El scanner lo arranca ahorroConectarELM() cuando toca.
  delay(500);
  digitalWrite(LED_BLUE, LOW);
  Bluefruit.Scanner.start(0);
#endif
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
//  GNSS: pedir AT+CGNSSINFO   [C3]
//  Se espera el "OK" que llega DESPUÉS del +CGNSSINFO (así la línea
//  de datos ya está completa), se corta en el primer fin de línea
//  real y se sanitiza para que no se cuele el "OK" ni espacios.
//  Se guarda la línea COMPLETA (no se fuerza a 14 campos), para no
//  perder altitud / velocidad / rumbo / DOP / sats.
// ============================================================
void pedirGNSSCrudo()
{
#if !DIAG_CON_GNSS
  // GNSS deshabilitado para ahorrar corriente.
  rawGNSS = "GNSS_OFF";
  return;
#else
  // Encender GNSS SOLO si no está ya encendido. Antes se hacía cada
  // ciclo y el URC "+CGNSSPWR: READY" llegaba tarde y contaminaba el
  // siguiente comando MQTT (se veía "@TOPIC ... +CGNSSPWR: READY!").
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
    if (p >= 0 && resp.indexOf("OK", p) >= 0) {   // OK POSTERIOR a la trama
      delay(50);
      while (SIM_SERIAL.available()) resp += (char)SIM_SERIAL.read();
      break;
    }
    alimentarWatchdog();   // [C4]
    delay(5);
  }

  int ini = resp.indexOf("+CGNSSINFO:");
  if (ini >= 0) {
    String linea = resp.substring(ini + 11);

    // Cortar en el primer fin de línea real -> descarta "\r\n...OK"
    int fin = linea.length();
    for (int i = 0; i < (int)linea.length(); i++) {
      if (linea[i] == '\r' || linea[i] == '\n') { fin = i; break; }
    }
    linea = linea.substring(0, fin);

    // Sanitizar: sin espacios y sin "OK" residual pegado (ej. "1341OK")
    linea.trim();
    linea.replace(" ", "");
    linea.replace("OK", "");

    // Separar en campos. Formato A7670 típico:
    //   +CGNSSINFO: <mode>,<GPS_sats>,<GLO_sats>,<BDS_sats>,<lat>,<N/S>,<lon>,...
    // Sin fix llega todo vacío (",,,,,,"). Antes se validaba SOLO el campo 5,
    // que en algunas variantes no es la latitud -> podía leer NO_FIX aun con
    // fix. Ahora: fix si el <mode> es 1/2/3 Y hay latitud (campo 4) no vacía.
    String campos[20];
    int nCampos = 0, pos = 0;
    for (int i = 0; i <= (int)linea.length() && nCampos < 20; i++) {
      if (i == (int)linea.length() || linea[i] == ',') {
        campos[nCampos++] = linea.substring(pos, i);
        pos = i + 1;
      }
    }
    String modo = (nCampos > 0) ? campos[0] : "";
    String lat  = (nCampos > 4) ? campos[4] : "";
    modo.trim(); lat.trim();

    // Diagnóstico: modo + satélites por constelación (campos 1..3). Sirve para
    // distinguir "0 satélites -> antena/cielo" de "ve satélites pero no fija".
    gnssDiag = "modo=" + (modo.length() ? modo : "-") +
               " gps=" + ((nCampos > 1) ? campos[1] : "-") +
               " glo=" + ((nCampos > 2) ? campos[2] : "-") +
               " bds=" + ((nCampos > 3) ? campos[3] : "-");

    // Fix válido si el <mode> indica 2D/3D y hay latitud. No se exige la
    // posición exacta de la longitud para tolerar variantes con distinto nº de
    // campos (el raw completo queda en rawGNSS igual). Ver gnssDiag/logDebug.
    bool hayFix = (modo == "2" || modo == "3") && lat.length() > 0;
    if (hayFix) rawGNSS = linea;
  } else {
    gnssDiag = "sin respuesta CGNSSINFO";
  }

  Serial.print("<< GNSS: ");
  Serial.print(rawGNSS);
  Serial.print("  ["); Serial.print(gnssDiag); Serial.println("]");
#endif   // DIAG_CON_GNSS
}

// ============================================================
//  Leer IMU — LSM6DS33
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
  j += ",\"dtc\":\"";    j += rawDTC; j += "\"";   // crudo Modo 03 (Node-RED decodifica)

  j += "}";
  return j;
}

// ============================================================
//  Publicar por MQTT
// ============================================================
void publicarMQTT(const String& payload)
{
  // Drenar URCs pendientes (GNSS, CGEV, etc.) antes de publicar,
  // para que no se cuelen en la respuesta del primer comando.
  while (SIM_SERIAL.available()) SIM_SERIAL.read();

  // Sacamos el AT+CSQ del camino de publicación (para descartarlo);
  //      el CSQ se sigue registrando en los logs de red.
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
//  Limpiar respuesta OBD del ruido del ELM327
//  Busca el marcador "41"+PID y devuelve solo la corrida hex que
//  sigue (corta en el primer caracter no-hex). Así:
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
  sprintf(marc, "41%02X", pid);      
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
//  Decodificar DTCs (Modo 03)   
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
    // validar 4 hex
    bool ok = true;
    for (int j = 0; j < 4; j++) {
      char c = h[k + j];
      if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) { ok = false; break; }
    }
    if (!ok) break;
    // b1/b2 en minúscula: el core Bluefruit define B1/B2 como macros
    // (binary.h) y chocaban -> "expected unqualified-id before numeric constant".
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
//  Descripción legible de un DTC 
//  Categoría por letra + descripción específica de los códigos más
//  comunes. SIN COMAS (para no romper el CSV).
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

  // Validar que sea una respuesta hex válida "41"+PID. Antes,
  // "NO_DATA" (largo 7) pasaba el filtro y se mal-parseaba como A=0x0A
  // (temp=-30, vel=10, etc.). Ahora se rechaza TIMEOUT/NO_DATA/ruido.
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
#if MODO_AHORRO
      if (!pids[i].enSD) continue;   // caja negra: sólo PIDs relevantes
#endif
      f.print(","); f.print(pids[i].nombre);
      f.print("("); f.print(pids[i].unidad); f.print(")");
    }
    f.print(",imu_ax(m/s2),imu_ay(m/s2),imu_az(m/s2)");
    f.print(",imu_gx(rad/s),imu_gy(rad/s),imu_gz(rad/s)");
    f.print(",vbat(V),cargando,carga_completa");
    f.print(",dtc");        // códigos compactos (P/C/B/U)
#if !MODO_AHORRO
    f.print(",dtc_alerta"); // descripción legible (se omite en ahorro para achicar la fila)
#endif
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
    // sólo dígitos, por las dudas
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
    // FIX: eran 9 comas y sobraba una -> corría todas las columnas de
    // PID/IMU/batería un lugar (filas de 35 columnas). La región GNSS son
    // 9 campos = 8 comas (la última la agrega el loop de PIDs).
    f.print(",,,,,,,,");
  }

  for (uint8_t i = 0; i < NUM_PIDS; i++) {
#if MODO_AHORRO
    if (!pids[i].enSD) continue;   //  igual filtro que la cabecera
#endif
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

  // DTCs decodificados (+ alerta legible sólo fuera de ahorro)
  {
    String cod = decodificarDTC(rawDTC);
    f.print(","); f.print(cod.length() > 0 ? cod : "none");
#if !MODO_AHORRO
    f.print(","); f.print(alertaDTC(cod));   // "sin fallas" o "Motor en falla: ... [P0100]"
#endif
    // En ahorro NO se escribe la columna dtc_alerta (string largo).
    // Los códigos compactos alcanzan para la caja negra; la descripción legible
    // la puede resolver Node-RED / el análisis posterior a partir del código.
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
    alimentarWatchdog();   // [C4]
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
    alimentarWatchdog();   //mantener vivo el WDT durante esperas largas
    delay(10);
  }
  // "grace read": el token puede ser un PREFIJO de la respuesta real.
  // Ej.: esperamos "+CMQTTPUB:" y cortábamos antes de que llegara ", 0,0",
  // reportando FAIL una publicación que en realidad salió. Tras encontrar
  // el token seguimos leyendo una ventana corta para completar la línea.
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
  if (!ok) Serial.println("  FAIL: " + cmd + " → " + r);
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

  // Forzar SOLO LTE (CNMP=38). Evita que el módem caiga a 2G/GSM,
  // cuyos pulsos de TX de ~2A hacen brownear el módem con batería.
  // 2=Automático, 13=GSM only, 38=LTE only, 51=GSM+LTE.
  simSendCheck("AT+CNMP=38", "OK", 3000);
  // (El A7670SA es LTE Cat-1 + 2G; no soporta Cat-M/NB-IoT. Verificar en
  //  el datasheet de tu variante si querés confirmar. Aun así, LTE Cat-1
  //  consume MUCHO menos en picos que 2G, que es lo que importa acá.)

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
    Serial.println(" SIN SEÑAL");
    logDebug("RED FAIL registro (sin señal) CSQ=" + String(leerCSQ()));  
    return false;
  }
  Serial.println(" OK");
  logDebug("RED registro OK  CSQ=" + String(leerCSQ()));                
  // Registrar la tecnología de red real (LTE vs GSM) para confirmar.
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
  simSendCheck("AT+CGNSSPWR=0");   // GNSS apagado para ahorrar corriente
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
  mqttSvcUp  = false;   // el STOP liberó (o intentó) el servicio
  mqttOnline = false;
}

// ============================================================
//  iniciarMQTT
// ============================================================
bool iniciarMQTT()
{
  Serial.println("--- Iniciando MQTT ---");

  // Arrancar el servicio (START + ACCQ) SÓLO si no está ya
  // arrancado. En este A7670 el CMQTTSTOP no libera bien el servicio, así que
  // re-arrancarlo cada ciclo hacía que el próximo CMQTTSTART diera ERROR (el
  // "una sí una no"). Ahora el servicio se mantiene vivo entre ciclos y por
  // ciclo sólo se hace CONNECT/DISC. El servicio se recrea tras teardown total,
  // reboot o CRESET (que ponen mqttSvcUp=false).
  if (!mqttSvcUp) {
    //No llamamos a teardownMQTT() aquí: al final de cada
    // ciclo ahorroDormirModem() ya hizo el teardown, y si el módem viene de un
    // arranque en frío (Modo 2), enviar DISC/RELCLIENT/STOP sin START previo
    // devuelve ERROR y puede bloquear el parser antes de AT+CMQTTSTART.

    Serial.print("  [MQTT] START... ");
    String r = simSend("AT+CMQTTSTART", 5000, "+CMQTTSTART:");
    if (respuestaTieneReinicio(r)) {
      Serial.println("REINICIO MODEM");
      logDebug("MQTT: reinicio del modem @START resp=" + limpiarResp(r));
      esperarBootModulo();
      return false;
    }
    // Si START da ERROR (servicio a medio arrancar), STOP + espera + reintento.
    for (uint8_t k = 0; k < 3 &&
         r.indexOf("+CMQTTSTART: 0") < 0 && r.indexOf("+CMQTTSTART: 23") < 0; k++) {
      logDebug("MQTT @START ERROR -> STOP y reintento " + String(k + 1) + "/3");
      alimentarWatchdog();
      simSend("AT+CMQTTSTOP", 5000, "OK");
      delay(800 + 700 * k);                 // 0.8s, 1.5s, 2.2s
      alimentarWatchdog();
      r = simSend("AT+CMQTTSTART", 6000, "+CMQTTSTART:");
      if (respuestaTieneReinicio(r)) { esperarBootModulo(); return false; }
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
    mqttSvcUp = true;   // servicio arrancado y cliente adquirido
  }

  Serial.print("  [MQTT] CONNECT... ");
  String connCmd = "AT+CMQTTCONNECT=0,\"tcp://" + String(MQTT_BROKER) + ":" +
                   String(MQTT_PORT) + "\"," + String(MQTT_KEEPALIVE) + ",1";
  if (strlen(MQTT_USER) > 0)
    connCmd += ",\"" + String(MQTT_USER) + "\",\"" + String(MQTT_PASS) + "\"";

  String r = simSend(connCmd, 10000, "+CMQTTCONNECT:");
  if (respuestaTieneReinicio(r)) {
    Serial.println("REINICIO MODEM");
    logDebug("MQTT: reinicio del modem @CONNECT resp=" + limpiarResp(r));
    esperarBootModulo();       // esto ya pone mqttSvcUp=false vía flags de reboot
    return false;
  }
  if (r.indexOf("+CMQTTCONNECT: 0,0") < 0) {
    Serial.println("FALLO -> " + r);
    logDebug("MQTT FAIL @CONNECT vbat=" + String(voltajeBateria, 2) +
             " resp=" + limpiarResp(r));
    // CONNECT falló con el servicio arrancado: forzar
    // recreación del servicio en el próximo intento (por las dudas quedó sucio).
    mqttSvcUp = false;
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
  Serial.println("MQTT: ciclo de reconexión...");
  logDebug("RECONECT inicio vbat=" + String(voltajeBateria, 2)); 
  teardownMQTT();

  simListo = false;
  if (iniciarRed()) simListo = true;
  else { Serial.println("MQTT: red caída — reintento postergado."); return; }

  if (iniciarMQTT()) mqttOnline = true;
  else Serial.println("MQTT: reintento fallido.");
}

// ============================================================
//  BATERÍA — leerNivelBateria() 
//  Referencia 3.6V, promediado y factor de calibración.
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
//  CARGA — leerEstadoCarga()  (TP4056 CHRG/STDBY activos en LOW)
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
//  A7670SA — pulsarPWRKEY_7670() (Encendido: pulso de 1s)
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

// ============================================================
//  A7670SA — apagarPWRKEY_7670() 
//  Pulso en LOW de 3000 ms (alimentando el watchdog) y espera
//  de 5000 ms en HIGH para desregistro limpio de la celda LTE.
// ============================================================
void apagarPWRKEY_7670()
{
  digitalWrite(PIN_PWRKEY_7670, HIGH);
  delay(100);
  digitalWrite(PIN_PWRKEY_7670, LOW);
  for (uint8_t i = 0; i < 6; i++) {
    alimentarWatchdog();
    delay(500);   // 6 x 500 ms = 3000 ms en LOW
  }
  digitalWrite(PIN_PWRKEY_7670, HIGH);
  for (uint8_t i = 0; i < 10; i++) {
    alimentarWatchdog();
    delay(500);   // 10 x 500 ms = 5000 ms en HIGH
  }
}

// ============================================================
//  ===============  MODO DE AHORRO DE ENERGÍA  ===============
//  Helpers del ciclo despertar/dormir. 
//  Reutilizan las funciones existentes (iniciarRed, iniciarMQTT,
//  teardownMQTT, pedirGNSSCrudo, publicarMQTT, etc.) sin tocar la
//  filosofía "MQTT crudo / SD traducido".
// ============================================================
#if MODO_AHORRO

bool ahorroConectarELM(uint32_t timeoutMs)
{
  //  Habilitar la ventana BLE antes de chequear o escanear
  blePermitido  = true;
  bleConectando = false;

  if (Bluefruit.Central.connected() && elmInicializado) {
    digitalWrite(LED_BLUE, HIGH);
    return true;
  }

  Serial.println("[AHORRO] BLE ON: buscando ELM327...");
  Bluefruit.Scanner.start(0);              // 0 = sin timeout interno; lo cortamos nosotros
  uint32_t t0 = millis();

  while (millis() - t0 < timeoutMs) {
    alimentarWatchdog();                   //  no dejar vencer el WDT en la espera

    if (Bluefruit.Central.connected() && elmInicializado) {
      Bluefruit.Scanner.stop();
      digitalWrite(LED_BLUE, HIGH);
      return true;
    }
    delay(50);
  }

  // Si dio timeout, cerrar el candado inmediatamente para que
  // un callback tardío en segundo plano no encienda el LED ni reconecte.
  blePermitido  = false;
  bleConectando = false;
  Bluefruit.Scanner.stop();
  digitalWrite(LED_BLUE, LOW);
  Serial.println("[AHORRO] ELM327 no encontrado / no listo (timeout).");
  return false;
}

// ---- BLE: apagar (detener scanner + desconectar) para bajar consumo ----
void ahorroApagarBLE()
{
  //Cerrar candados ANTES de desconectar para bloquear callbacks
  blePermitido  = false;
  bleConectando = false;
  Bluefruit.Scanner.stop();

  if (Bluefruit.Central.connected() && g_connHandle != BLE_CONN_HANDLE_INVALID) {
    Bluefruit.disconnect(g_connHandle);
    uint32_t t0 = millis();
    while (Bluefruit.Central.connected() && millis() - t0 < 1500) {
      alimentarWatchdog();
      delay(20);
    }
  }
  // Forzar apagado del LED y suspender el servicio BLE
  digitalWrite(LED_BLUE, LOW);
  Serial.println("[AHORRO] BLE OFF.");
}

// ---- OBD: poll SÍNCRONO de un PID (llena rawPID[i]) ----
void ahorroPollPID(uint8_t i)
{
  bufELMIdx = 0; bufELM[0] = '\0'; elmListo = false;
  enviarComando(pids[i].comando);
  uint32_t t0 = millis();
  while (millis() - t0 < TIMEOUT_ELM) {
    alimentarWatchdog();
    if (elmListo) break;
    delay(5);
  }
  if (elmListo) {
    rawPID[i] = String(bufELM); rawPID[i].trim();
    String limpio = limpiarOBD(rawPID[i], pids[i].pid);   //descartar banner/ruido
    if (limpio.length() >= 6) rawPID[i] = limpio;
    else if (rawPID[i].indexOf("TIMEOUT") < 0) rawPID[i] = "NO_DATA";
    elmListo = false;
  } else {
    rawPID[i] = "TIMEOUT";
  }
  delay(150);   // guarda entre comandos (igual que avanzarPID)
}

// ---- OBD: poll SÍNCRONO de DTC (Modo 03) ----
void ahorroPollDTC()
{
  bufELMIdx = 0; bufELM[0] = '\0'; elmListo = false;
  enviarComando("03");
  uint32_t t0 = millis();
  while (millis() - t0 < TIMEOUT_ELM) {
    alimentarWatchdog();
    if (elmListo) break;
    delay(5);
  }
  if (elmListo) {
    String d = String(bufELM); d.trim(); d.toUpperCase();
    rawDTC = (d.indexOf("43") >= 0) ? d : "";
    elmListo = false;
  } else {
    rawDTC = "";
  }
  Serial.printf("[AHORRO] DTC = %s\n", rawDTC.c_str());
}

// ---- GNSS: apagar receptor ----
void ahorroApagarGNSS()
{
#if DIAG_CON_GNSS
  if (gnssOn) { simSendCheck("AT+CGNSSPWR=0", "OK", 2000); gnssOn = false; }
#endif
}

// ---- GNSS: preparar/asegurar fix al despertar (hot start o ya encendido) ----
void ahorroPrepararGNSS()
{
#if !DIAG_CON_GNSS
  return;                                  // GNSS deshabilitado por flag de diagnóstico
#else
  if (!gnssOn) {
    // Re-encender el receptor. Si el módulo retuvo efemérides -> hot/warm start.
    if (simSendCheck("AT+CGNSSPWR=1", "OK", 2000)) gnssOn = true;
    delay(500);
  }

  // Configuración inicial del receptor, UNA sola vez por
  // encendido del módem (se re-hace tras un reboot: gnssConfigurado=false).
  if (gnssOn && !gnssConfigurado) {
    // Esperar el URC "+CGNSSPWR: READY" (el receptor tarda en levantar tras
    // CGNSSPWR=1; pedir CGNSSINFO antes devuelve vacío). Tolerante: si no llega
    // en 8 s seguimos igual, pero lo dejamos registrado.
    uint32_t tr = millis(); String rr = "";
    while (millis() - tr < 8000) {
      while (SIM_SERIAL.available()) rr += (char)SIM_SERIAL.read();
      if (rr.indexOf("+CGNSSPWR: READY") >= 0 || rr.indexOf("+CGNSSPWR:READY") >= 0) break;
      alimentarWatchdog();
      delay(50);
    }
    //  Habilitar TODAS las constelaciones (GPS+GLONASS+BDS+
    // Galileo) para maximizar satélites visibles. La sintaxis de CGNSSMODE
    // varía por variante/firmware; se ignora el error si no aplica.
    simSend("AT+CGNSSMODE=15,1", 2000, "OK");
    gnssConfigurado = true;
    logDebug("GNSS config: READY=" +
             String((rr.indexOf("READY") >= 0) ? 1 : 0));
  }

  #if GNSS_USAR_AGPS
    //  AGPS por red para acelerar el TTFF (baja de minutos a
    // segundos). Requiere PDP activo (ya lo está en este punto). El comando
    // típico SIMCom es AT+CAGPS; algunas variantes usan otro nombre. Se pide
    // solo si todavía no hay fix, para no gastar red al pedo.
    if (rawGNSS == "NO_FIX" || rawGNSS == "GNSS_OFF")
      simSend("AT+CAGPS", 10000, "OK");
  #endif

  // Esperar un fix válido hasta el timeout (reutiliza pedirGNSSCrudo()).
  // Con GNSS_MANTENER_ON=1 suele salir en el primer intento (venía encendido).
  uint32_t t0 = millis();
  while (millis() - t0 < GNSS_FIX_TIMEOUT_MS) {
    alimentarWatchdog();
    pedirGNSSCrudo();
    if (rawGNSS != "NO_FIX" && rawGNSS != "GNSS_OFF") break;
    delay(1000);
  }

  // Dejar SIEMPRE registro en el debug.log del resultado y
  // los satélites vistos. Así se puede diagnosticar sin serial: si siempre da
  // "gps=0 glo=0 bds=0" -> antena/cielo; si ve satélites pero no fija -> falta
  // tiempo o AGPS; si fija -> todo OK.
  logDebug("GNSS " + String((rawGNSS == "NO_FIX") ? "NO_FIX" : "FIX") +
           " [" + gnssDiag + "]");
#endif
}

// ---- MÓDEM: despertar y levantar red + MQTT ----
bool ahorroDespertarModem()
{
  digitalWrite(PIN_SLEEP_7670, LOW);       // DTR bajo = módem despierto (inofensivo en modo 0)

#if (AHORRO_MODO_MODEM == 2)
  // El módem venía apagado (power-down total): encender por hardware.
  if (g_modemApagado) {                    // sólo pulsar PWRKEY si estaba apagado
    pulsarPWRKEY_7670();                    // (pulsarlo estando ON lo APAGARÍA)
    SIM_SERIAL.begin(SIM_BAUD);
    // [FIX Modo 2 - Paso 3] Esperar el boot real del A7670SA (6 a 9 s) y
    // limpiar banderas en lugar de sólo delay(1000).
    esperarBootModulo();
    g_modemApagado = false;
  }
  while (SIM_SERIAL.available()) SIM_SERIAL.read();
#elif (AHORRO_MODO_MODEM == 1)
  // Salir del CSCLK sleep por DTR.
  delay(50);
  simSend("AT+CSCLK=0", 1000, "OK");       // deshabilitar sleep mientras trabajamos
#endif
  // Modo 0 (default): el módem quedó encendido y registrado; no hay que hacer nada.

  // Confirmar que responde a AT. Si no, intentar UNA
  // recuperación por hardware (el módem pudo brownoutear o quedar trabado):
  // re-pulsar PWRKEY, re-init de la UART y reintentar. Así el sistema se
  // AUTO-SANA en vez de fallar para siempre (era lo que pasaba: una vez que el
  // módem quedaba mudo, todos los ciclos daban "red/MQTT no levantó").
  if (!wakeupSIM()) {
    Serial.println("[AHORRO] módem no responde; recuperación por HW (PWRKEY)...");
    logDebug("AHORRO wake: modem mudo -> re-PWRKEY");
    pulsarPWRKEY_7670();
    SIM_SERIAL.begin(SIM_BAUD);
    esperarBootModulo();
    g_modemApagado = false;
    if (!wakeupSIM()) { Serial.println("[AHORRO] módem sigue mudo tras recuperación."); return false; }
  }

  // Si el ciclo anterior no pudo transmitir, la pila
  // TCP/MQTT del módem pudo quedar trabada (responde AT pero rechaza
  // CMQTTSTART/ACCQ). Un CRESET la limpia sin llegar al reinicio total del
  // micro; recupera en el ciclo siguiente en vez de esperar el backstop.
  if (g_ultimoCicloFallo) {
    Serial.println("[AHORRO] ciclo previo falló -> AT+CRESET del módem.");
    logDebug("AHORRO wake: CRESET por ciclo previo fallido");
    alimentarWatchdog();
    simSend("AT+CRESET", 3000, "OK");
    alimentarWatchdog();
    delay(8000);                            // < WDT_TIMEOUT_S; el módem re-arranca
    alimentarWatchdog();
    while (SIM_SERIAL.available()) SIM_SERIAL.read();
    if (!wakeupSIM()) { Serial.println("[AHORRO] módem no responde tras CRESET."); return false; }
    // El CRESET reinició el módem: el servicio MQTT y el GNSS
    // quedaron apagados. Marcar el estado como perdido para que se recreen
    // limpios (antes quedaban stale -> el ciclo post-CRESET perdía el fix GNSS
    // y podía arrastrar un servicio MQTT fantasma).
    mqttSvcUp = false;
    gnssOn = false; gnssConfigurado = false;
  }

  simListo = false; mqttOnline = false;
  if (!iniciarRed())  { logDebug("AHORRO wake: red no levantó"); return false; }
  simListo = true;
  if (!iniciarMQTT()) { logDebug("AHORRO wake: MQTT no conectó"); return false; }
  mqttOnline = true;
  return true;
}

// ---- MÓDEM: cerrar el socket MQTT + reposo según AHORRO_MODO_MODEM ----
void ahorroDormirModem()
{
#if (AHORRO_MODO_MODEM == 0)
  // Modo 0 (default): cerrar SOLO el socket ante el broker
  // con CMQTTDISC (el servidor ve la baja limpia), pero MANTENER arrancado el
  // servicio MQTT (mqttSvcUp) y el PDP. Así el próximo ciclo hace sólo CONNECT
  // (sin STOP/START), que es lo que fallaba con ERROR en este A7670. El socket
  // igual queda cerrado, que era el requisito.
  simSend("AT+CMQTTDISC=0,60", 5000, "OK");
  mqttOnline = false;
  #if !GNSS_MANTENER_ON
    ahorroApagarGNSS();                    // opcional: apagar sólo el GNSS
  #endif
  Serial.println("[AHORRO] socket MQTT cerrado (servicio y PDP se mantienen).");
  return;   // el módem queda encendido, registrado, con servicio MQTT vivo
#else
  // Modos 1 y 2: teardown COMPLETO (el módem va a dormir/apagarse, hay que
  // liberar todo y avisar al broker).
  teardownMQTT();                          // DISC + REL + STOP (baja mqttSvcUp)
  simSend("AT+CGACT=0,1", 5000, "OK");     // bajar el PDP
#endif

#if (AHORRO_MODO_MODEM == 2)
  // Power-down TOTAL del módem.
  ahorroApagarGNSS();                      // sin módem no hay GNSS igual

  //Intentar apagado por software nativo con confirmación
  // y, si el módem sigue respondiendo a AT, forzar el apagado físico con el
  // pulso largo de 3 s en LOW + 5 s en HIGH (apagarPWRKEY_7670).
  bool offSW = simSendCheck("AT+CPOF", "OK", 3000);
  if (!offSW) {
    offSW = simSendCheck("AT+CPOWD=1", "OK", 3000);
  }
  if (offSW) {
    for (uint8_t i = 0; i < 6; i++) {
      alimentarWatchdog();
      delay(500);                          // esperar 3 s a que termine el power-down
    }
  }
  // Si no aceptó el comando o sigue vivo respondiendo a AT, apagar por PWRKEY (3s LOW)
  if (!offSW || simSend("AT", 1000, "OK").indexOf("OK") >= 0) {
    Serial.println("[AHORRO] Apagando A7670SA por hardware (PWRKEY 3s)...");
    apagarPWRKEY_7670();
  }

  g_modemApagado  = true;
  // Resetear banderas para que el próximo ciclo sepa que
  // el módem arrancó en frío y vuelva a inicializar GNSS y MQTT.
  gnssOn          = false;
  gnssConfigurado = false;
  mqttSvcUp       = false;
#elif (AHORRO_MODO_MODEM == 1)
  // Sleep por DTR (requiere DTR bien cableado y polaridad correcta).
  #if !GNSS_MANTENER_ON
    ahorroApagarGNSS();                    // apagar GNSS entre ciclos (se re-enciende al despertar)
  #endif
  simSend("AT+CSCLK=1", 1000, "OK");       // habilitar sleep por DTR
  digitalWrite(PIN_SLEEP_7670, HIGH);      // DTR alto = dejar dormir
#endif

  simListo = false; mqttOnline = false;
  Serial.println("[AHORRO] socket MQTT cerrado; PDP abajo.");
}

// ---- Un ciclo completo de ahorro ----
void cicloAhorro()
{
  Serial.println("\n===== [AHORRO] Inicio de ciclo =====");
  alimentarWatchdog();

  // 1) Batería/carga (barato, con todo aún dormido).
  voltajeBateria = leerNivelBateria();
  leerEstadoCarga();
  actualizarLEDCarga();

  // 2) OBD por BLE (encender, pollear, apagar).
  for (uint8_t i = 0; i < NUM_PIDS; i++) rawPID[i] = "TIMEOUT";
  rawDTC = "";
  if (ahorroConectarELM(15000)) {
    for (uint8_t i = 0; i < NUM_PIDS; i++) { alimentarWatchdog(); ahorroPollPID(i); }
    ahorroPollDTC();
  } else {
    logDebug("AHORRO: ELM327 no conectado; PIDs=TIMEOUT");
  }
  ahorroApagarBLE();

  // 3) IMU (barato).
  leerIMU();

  // 4) Despertar módem + red + MQTT, con reintentos. Cada intento levanta todo
  //    desde cero (iniciarMQTT hace teardown antes), así no arrastra un socket
  //    a medio abrir.
  bool txOK = false;
  for (uint8_t intento = 1; intento <= AHORRO_MAX_INTENTOS_TX && !txOK; intento++) {
    alimentarWatchdog();
    Serial.printf("[AHORRO] Intento TX %u/%u\n", (unsigned)intento, (unsigned)AHORRO_MAX_INTENTOS_TX);
    if (!ahorroDespertarModem()) {
      logDebug("AHORRO TX intento " + String(intento) + ": red/MQTT no levantó");
      continue;                            // reintenta (o se rinde si era el último)
    }
    ahorroPrepararGNSS();                   // GNSS: hot start o ya encendido
    pedirGNSSCrudo();                       // lectura final de posición
    String json = construirJSONraw();
    Serial.print("JSON: "); Serial.println(json);
    publicarMQTT(json);                     // pone mqttOnline=false si falla; tUltimoPublish si OK
    if (mqttOnline) txOK = true;
    else logDebug("AHORRO TX intento " + String(intento) + ": publish falló");
  }

  // 5) SD (traducido REDUCIDO) — se guarda siempre, haya o no TX.
  if (sdLista) guardarCSVtraducido();

  // 6) Cerrar socket MQTT y dormir/apagar el módem.
  ahorroDormirModem();

  // Anti-zombificación. Con LIVENESS_MS=0 el watchdog
  // HW no cubre un cuelgue LÓGICO: si la pila MQTT del módem queda trabada,
  // iniciarMQTT falla siempre y el equipo dejaría de transmitir en silencio
  // para siempre. Contamos ciclos fallidos consecutivos: g_ultimoCicloFallo
  // hace que el próximo wake intente un AT+CRESET, y si aun así no se recupera
  // tras AHORRO_MAX_CICLOS_FALLIDOS ciclos, reinicio total (re-corre setup(),
  // que hace PWRKEY+CRESET y deja el módem limpio).
  static uint8_t ciclosFallidos = 0;
  if (txOK) {
    ciclosFallidos = 0;
    g_ultimoCicloFallo = false;
  } else {
    ciclosFallidos++;
    g_ultimoCicloFallo = true;
    logDebug("AHORRO ciclo fallido " + String(ciclosFallidos) + "/" +
             String(AHORRO_MAX_CICLOS_FALLIDOS));
    if (ciclosFallidos >= AHORRO_MAX_CICLOS_FALLIDOS) {
      Serial.println("[AHORRO] Fallos de TX persistentes -> reinicio de recuperación.");
      logDebug("AHORRO reset: " + String(ciclosFallidos) +
               " ciclos sin TX. vbat=" + String(voltajeBateria, 2));
      Serial.flush();
      delay(50);
      NVIC_SystemReset();
    }
  }

  Serial.printf("===== [AHORRO] Fin de ciclo (txOK=%d) =====\n", txOK ? 1 : 0);
}

#endif   // MODO_AHORRO