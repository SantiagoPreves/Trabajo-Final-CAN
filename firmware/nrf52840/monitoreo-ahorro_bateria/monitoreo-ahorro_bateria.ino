/*------------------------------------------------------------------------------
-- Project : Sistema de monitoreo de fallas y datos de manejo de vehículos
-------------------------------------------------------------------------------
-- File : monitoreo-ahorro_bateria.ino (Adafruit Feather nRF52840 Sense)
-- Author : Preves, Santiago.
-- Date : Sep 27, 2026.
-- Rev 4 : Sep 21, 2026. Primera versión con gestión de energía.
-- Rev 5 : Sep 25, 2026. corrección del ciclo con módem apagado (modo 2).
-- Rev 6 : Sep 27, 2026. ajustes a partir de la prueba de 15 ciclos.
-- Rev 7 : Sep 27, 2026. Versión final: comentarios ordenados, mismo código.
--
-------------------------------------------------------------------------------
-- Description:
  Versión final del firmware del sistema, con gestión de energía.
  El equipo trabaja por ciclos de "despertar, adquirir, transmitir y dormir"
  que se repiten cada INTERVALO_CICLO:

    1. Mide la batería y el estado de carga.
    2. Se conecta al ELM327 por BLE, consulta qué PIDs soporta el vehículo
       (01 00, 01 20 y 01 40), lee los 15 PIDs y los DTC (Modo 03) y corta
       la conexión.
    3. Lee el IMU.
    4. Si la lectura OBD está completa, enciende el módem y el GNSS, se
       registra en la red LTE, espera un fix y publica el JSON crudo por MQTT.
    5. Guarda los valores traducidos en la SD, se haya transmitido o no.
    6. Cierra la sesión MQTT y apaga el módem hasta el ciclo siguiente.

  Hardware : Adafruit Feather nRF52840 Sense
  BLE      : ELM327 (servicio FFF0 / característica FFF1)
  UART     : A7670SA en Serial1 (TX = pin 1, RX = pin 0)
  SPI      : tarjeta SD (CS = A0)

  Pines:
    A0 -> CS de la SD
    A1 -> SLEEP_7670 (DTR del A7670SA)
    A3 -> NIVEL_BAT (ADC del divisor de batería)
    A5 -> PWRKEY_7670
    10 -> CHRG del TP4056 (D3 en el esquemático, activo en bajo)
    11 -> STDBY del TP4056 (D4 en el esquemático, activo en bajo)
    12 -> ACTIVAR_NIVEL (D5 en el esquemático, habilita el divisor)

  Configuración:
    La versión final usa MODO_AHORRO = 1 y AHORRO_MODO_MODEM = 2 (módem
    apagado entre ciclos), que es la configuración validada en las pruebas.
    Los modos de AHORRO_MODO_MODEM = 0 (encendido y registrado) y AHORRO_MODO_MODEM = 1 (sleep por DTR) se
    conservan para desarrollo futuro. Con MODO_AHORRO = 0 el equipo vuelve al
    funcionamiento continuo de la primera versión (ciclo de 15 s), y
    DIAG_SOLO_MQTT / DIAG_CON_GNSS permiten hacer pruebas de consumo.

  Historial de versiones
  ----------------------
  Rev 4 (21/09/2026) Versión con gestión de energía.
    Ciclo de ahorro (MODO_AHORRO) con tres estrategias para el módem entre
    ciclos (AHORRO_MODO_MODEM 0, 1 y 2). Incluía además la medición de
    batería con la referencia real del ADC (3.6 V), la lectura completa de
    AT+CGNSSINFO, el watchdog de hardware, el registro de eventos en
    /debug.log, la detección de reinicios del módem, la red sólo LTE, la
    lectura de DTC (Modo 03) y la SD reducida a 5 PIDs.

  Rev 5 (25/09/2026) Corrección del ciclo con módem apagado (modo 2).
    Objetivo: que el modo 2 funcione de forma estable, que cada mensaje
    llegue con todos los PIDs y que aumente la cantidad de fixes GNSS.
    - BLE: el scanner de Bluefruit se relanzaba solo al desconectar
      (restartOnDisconnect) y quedaba pausado; en el ciclo siguiente
      Scanner.start() fallaba y todos los PIDs quedaban en TIMEOUT (un ciclo
      sí y otro no). Ahora el scanner lo maneja sólo bleConectarELM().
    - BLE: connect_callback() bloqueaba la tarea de callbacks durante 3.6 s
      y las respuestas del ELM327 llegaban tarde, mezcladas con los primeros
      PIDs. La inicialización pasó al loop (elmInicializar()) y espera el
      prompt '>' de cada comando.
    - OBD: lectura "todo o nada" (obdLeerCompleto()): reintentos por PID con
      re-sincronización, mapa de PIDs soportados por el vehículo y hasta dos
      conexiones por ciclo. Si falta un PID soportado, el ciclo no se
      publica (PUBLICAR_SOLO_OBD_COMPLETO).
    - UART: el buffer de recepción de Serial1 es de 64 bytes y las lecturas
      cada 5-10 ms perdían bytes en las respuestas largas. Ahora se lee cada
      1 ms.
    - MQTT: el cierre usaba AT+CMQTTRELCLIENT, que no existe en el A76XX. Sin
      liberar el cliente, el servicio no se detenía y el CMQTTSTART del ciclo
      siguiente daba ERROR. teardownMQTT() ahora hace DISC -> REL -> STOP.
    - Módem: encendido y apagado confirmados por AT (modemEncender() y
      modemApagar()), con los tiempos de PWRKEY de la hoja de datos. En modo
      2 se quitó el AT+CRESET después de un ciclo fallido.
    - GNSS: se enciende en paralelo con el registro en la red y usa AP-Flash
      para arrancar en caliente. El fix se detecta por los campos N/S y E/W
      (gnssExtraer()), porque en el formato del A7670SA el campo 4 no es la
      latitud. Una lectura sin fix ya no borra el fix obtenido en el ciclo,
      el AGPS espera la respuesta +AGPS: y se quitó AT+CGNSSMODE=15,1 (valor
      fuera de rango).
    - Red: registro por CEREG además de CGREG; CNMP y APN sólo se escriben
      si cambian.
    - Ciclo anclado a su inicio (el período es INTERVALO_CICLO) y resumen de
      cada ciclo en /debug.log.
    - Modo 1: si el DTR no despierta al módem, se recupera por hardware, y
      el servicio MQTT se mantiene entre ciclos.

  Rev 6 (27/09/2026) Ajustes a partir de la prueba de 15 ciclos.
    En la prueba de la Rev 5 los 15 mensajes llegaron completos, pero el
    registro mostró que AT+CAGPS reiniciaba el módem y que el módem volvía a
    encenderse solo después de AT+CPOF.
    - AGPS desactivado (GNSS_USAR_AGPS = 0).
    - Ante un reinicio inesperado del módem ("*ATREADY") se marca el contexto
      como perdido, se vuelve a encender el GNSS y se verifica la red antes
      de MQTT. La cantidad de reinicios queda en /debug.log.
    - Polaridad de PWRKEY configurable (PWRKEY_ACTIVO_ALTO) y aviso en
      /debug.log (auto_on) cuando el módem aparece encendido tras AT+CPOF.

  Rev 7 (27/09/2026) Versión final.
    Mismo código que la Rev 6: se ordenaron los comentarios, el historial
    de cambios pasó a este encabezado y se renombró el archivo. Los mensajes
    de arranque por Serial y en /debug.log conservan las etiquetas v3 y v3.1.
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
// Calibración: CAL_BAT = Vreal / Vmedido (tensión medida con tester).
#define CAL_BAT         1.00f

// Q2 (BSS84, canal P): LOW en el gate = divisor habilitado.
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

// Polaridad de PWRKEY vista desde A5:
//   0 = A5 conectado directo a PWRKEY: A5 en LOW = presionado.
//   1 = entre A5 y PWRKEY hay un transistor que invierte la señal:
//       A5 en HIGH = presionado.
// Si con 0 el log muestra "MODEM encendido al iniciar el ciclo aunque se
// apagó con CPOF", PWRKEY queda presionado en reposo y el módulo se vuelve
// a encender solo: revisar el esquemático y probar con 1.
#define PWRKEY_ACTIVO_ALTO  0
#if PWRKEY_ACTIVO_ALTO
  #define PWRKEY_PRESIONADO  HIGH
  #define PWRKEY_SUELTO      LOW
#else
  #define PWRKEY_PRESIONADO  LOW
  #define PWRKEY_SUELTO      HIGH
#endif

// Tiempos del A7670 (A7670 Series Hardware Design V1.06, sección 3.2).
// En el módulo, PWRKEY es activo en bajo (pull-up interno a VBAT).
#define PWRKEY_T_ON_MS      1000UL   // pulso de encendido (Ton típ. 50 ms); al ser < Toff no apaga un módem encendido
#define PWRKEY_T_OFF_MS     3000UL   // pulso de apagado (Toff mín. 2.5 s)
#define MODEM_T_BOOT_MS     25000UL  // máx. espera a que responda AT tras encender (Ton(uart) típ. 11.1 s)
#define MODEM_T_OFF_MS      15000UL  // máx. espera a que deje de responder tras AT+CPOF (Toff(uart) típ. 2 s)
#define MODEM_T_OFF_ON_MS   6000UL   // pausa entre apagado y re-encendido por HW (Toff-on mín. 2 s)
#define RED_T_REGISTRO_MS   45000UL  // registro LTE: tras un arranque en frío puede pasar de 20 s

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
//  MODO DE AHORRO DE ENERGÍA
//  Ciclo "despertar -> adquirir -> transmitir -> dormir".
//    MODO_AHORRO = 1 -> ciclo con reposo entre transmisiones (versión final).
//    MODO_AHORRO = 0 -> funcionamiento continuo de la primera versión (15 s).
// ============================================================
#define MODO_AHORRO   1

// Intentos de transmisión por ciclo
#define AHORRO_MAX_INTENTOS_TX  2

// Ciclos seguidos con transmisión fallida antes de reiniciar el micro
// (NVIC_SystemReset). Sólo cuentan los ciclos en los que se intentó
// transmitir: un ciclo sin OBD completo no suma.
#define AHORRO_MAX_CICLOS_FALLIDOS  3

// Pausa mínima entre ciclos cuando uno duró más que INTERVALO_CICLO.
#define AHORRO_PAUSA_MIN_MS  5000UL

// ---- Estrategia del módem entre ciclos --------------------------------
//   AHORRO_MODO_MODEM = 2 -> apagado total (AT+CPOF). Consumo mínimo;
//        cada ciclo implica un arranque (~11 s) y el registro en
//        la red. Con AP-Flash el GNSS recupera las efemérides guardadas y no
//        arranca en frío. Es el modo validado en las pruebas.
//   Los modos 0 y 1 se conservan para desarrollo futuro:
//   AHORRO_MODO_MODEM = 0 -> módem encendido y registrado; sólo se cierra el
//        socket MQTT (CMQTTDISC). Consumo en reposo LTE ~10-20 mA.
//   AHORRO_MODO_MODEM = 1 -> sleep por DTR (AT+CSCLK=1 y SLEEP_7670 en HIGH).
//        Requiere verificar el cableado y la polaridad del DTR. Si al
//        despertar el módem no responde, se recupera por hardware.
#define AHORRO_MODO_MODEM   2

// ---- OBD / ELM327 -----------------------------------------------------
// PUBLICAR_SOLO_OBD_COMPLETO:
//   1 = sólo se publica si se leyeron todos los PIDs que el vehículo soporta
//       (según 01 00, 01 20 y 01 40). Si no, el ciclo no transmite (en modo 2
//       ni siquiera enciende el módem), pero la fila de la SD se guarda igual.
//   0 = se publica aunque haya PIDs en TIMEOUT.
#define PUBLICAR_SOLO_OBD_COMPLETO  1
#define OBD_MAX_SESIONES     2        // sesiones BLE por ciclo (se repite si la lectura falla)
#define OBD_REINTENTOS_PID   2        // reintentos por PID (además del primero)
#define ELM_NOMBRE_FILTRO    "OBD"    // el nombre anunciado del adaptador debe contenerlo
#define BLE_T_ESCANEO_MS     12000UL  // máx. para encontrar el anuncio del ELM327
#define BLE_T_CONEXION_MS    6000UL   // máx. para que se concrete la conexión
#define ELM_T_ATZ_MS         4000UL   // AT Z (reset del ELM)
#define ELM_T_AT_MS          1500UL   // resto de comandos AT del ELM
#define ELM_T_BUSQUEDA_MS    12000UL  // primer 01 00 tras AT SP 0 ("SEARCHING...")
#define ELM_T_PID_MS         2000UL   // cada PID
#define ELM_T_DTC_MS         4000UL   // Modo 03
#define ELM_GAP_MS           40UL     // pausa entre comandos al ELM

// ---- Estrategia del GNSS ----------------------------------------------
//   GNSS_MANTENER_ON (sólo modos 0 y 1): 1 = el GNSS queda encendido entre
//        ciclos; 0 = se apaga y se usa AP-Flash. 
//        En modo 2 no aplica porque se apaga todo el módem.
//   GNSS_FIX_TIMEOUT_MS se mide desde que se enciende el GNSS en el ciclo
//        (el registro en la red ocurre en paralelo). Define el compromiso
//        entre fix y energía: un valor mayor da más fixes en arranques en
//        frío, a costa de más tiempo con el módem encendido.
#define GNSS_MANTENER_ON     1
#define GNSS_USAR_AGPS       0          // 0: en las pruebas, AT+CAGPS reiniciaba el módem
#define GNSS_USAR_APFLASH    1          // 1 = hot start desde la flash del módem (CGNSSPWR=1,1 / 0,1)
#define GNSS_GUARDAR_CADA_MS (30UL * 60UL * 1000UL)   // no escribir la flash del módem más seguido que esto
#define GNSS_FIX_TIMEOUT_MS  90000UL    // máx. espera de fix, desde el encendido del GNSS

// ============================================================
//  WATCHDOG Y LIVENESS
// ============================================================
#define WDT_TIMEOUT_S   20UL       // reinicio si el firmware se congela más de 20 s
#if MODO_AHORRO
// En ahorro el módem pasa apagado la mayor parte del tiempo: el liveness
// se desactiva (0) y la recuperación queda a cargo del contador de ciclos
// fallidos (AHORRO_MAX_CICLOS_FALLIDOS).
#define LIVENESS_MS     0UL
#else
#define LIVENESS_MS     240000UL   // reinicio si pasan 4 min sin publicar
#endif
uint32_t tUltimoPublish = 0;

// ============================================================
//  LOG DE DEPURACIÓN EN LA SD
//  /debug.log registra los eventos de cada ciclo (BLE, OBD, red, GNSS y
//  MQTT), el motivo de cada reinicio y un resumen por ciclo. Cada línea se
//  abre, escribe y cierra para que sobreviva a un corte de energía.
// ============================================================
#define DEBUG_LOG_FILE   "/debug.log"
#define DEBUG_LOG_MAX    1048576UL   // al superar 1 MB se borra
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
  bool        enSD;     // true = se traduce y guarda en la SD
};

// Columna enSD: en modo ahorro la SD funciona como "caja negra" y sólo guarda
// las magnitudes de conducción (RPM, velocidad, temperatura del motor, carga
// y acelerador). Por MQTT se envían siempre todos los PIDs crudos.
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
//  Los '\r' del ELM se guardan como '|' (separador de línea): así una
//  respuesta de varias ECUs o con "SEARCHING..." se puede separar por trama.
// ============================================================
#define ELM_BUF_SIZE 128
char              bufELM[ELM_BUF_SIZE];
volatile uint8_t  bufELMIdx = 0;
volatile bool     elmListo  = false;
// true cuando terminó la inicialización del ELM327 y se pueden pedir PIDs.
volatile bool elmInicializado = false;

// Ventana BLE habilitada: fuera de ella los callbacks no conectan ni
// encienden el LED.
volatile bool blePermitido  = false;
volatile bool bleConectando = false;   // usado por el modo continuo

// Handle de la conexión BLE actual (lo actualizan connect/disconnect_callback).
volatile uint16_t g_connHandle = BLE_CONN_HANDLE_INVALID;
// Estado del enlace y hallazgo del ELM327, informados por los callbacks.
volatile bool     g_bleConectado  = false;
volatile bool     g_elmEncontrado = false;
ble_gap_addr_t    g_elmAddr;             // dirección del ELM327 encontrada al escanear

// Estado del módem entre ciclos.
bool g_modemApagado     = false;   // true si quedó apagado (modo 2)
bool g_ultimoCicloFallo = false;   // modos 0 y 1: al despertar se hace AT+CRESET
bool g_cresetCiclo      = false;   // ya se hizo el CRESET en este ciclo

// ============================================================
//  MÁQUINA DE ESTADOS (modo continuo, MODO_AHORRO = 0)
// ============================================================
enum Estado : uint8_t { IDLE, ESPERANDO_ELM, ESPERANDO_DTC, HACIENDO_GNSS };
volatile Estado estado = IDLE;

uint8_t  pidActual       = 0;
uint32_t tiempoEnvio     = 0;
uint32_t tiempoProxCiclo = 0;

const uint32_t TIMEOUT_ELM     = 3000;
#if MODO_AHORRO
const uint32_t INTERVALO_CICLO = 300000;   // período entre inicios de ciclo (5 min)
#else
const uint32_t INTERVALO_CICLO = 15000;
#endif

// ============================================================
//  GNSS crudo
// ============================================================
String rawGNSS = "NO_FIX";
String rawDTC  = "";   // respuesta cruda del Modo 03, ej. "430133"

// ============================================================
//  FLAGS estado MQTT/4G/GNSS
// ============================================================
bool simListo   = false;   // red registrada y con IP
bool mqttOnline = false;   // cliente MQTT conectado al broker
bool mqttSvcUp  = false;   // servicio MQTT arrancado (CMQTTSTART + CMQTTACCQ)
bool gnssOn     = false;   // GNSS encendido
String gnssDiag = "sin datos";

// Estado del GNSS en el ciclo
bool     g_gnssReady     = false;  // llegó "+CGNSSPWR: READY!" o ya respondió +CGNSSINFO
bool     g_gnssRefFijada = false;  // g_tGnssRef ya se fijó en este ciclo
uint32_t g_tGnssRef      = 0;      // referencia del timeout de fix del ciclo
bool     g_fixCiclo      = false;  // hubo fix válido en este ciclo
bool     g_agpsCiclo     = false;  // ya se pidió AGPS en este ciclo
bool     g_apFlashOK     = true;   // el firmware acepta CGNSSPWR=x,1 (se detecta)
bool     g_gnssGuardado  = false;  // ya se guardaron efemérides desde el arranque
uint32_t g_tGnssGuardado = 0;

// Diagnóstico de reinicios y auto-encendido del módem
uint8_t  g_reiniciosCiclo = 0;     // "*ATREADY" inesperados en el ciclo
bool     g_autoOnCiclo    = false; // estaba encendido al iniciar el ciclo aunque se había apagado
bool     g_autoOnAvisado  = false; // el aviso largo se escribe una vez por arranque

#if MODO_AHORRO
// Mapa de PIDs soportados por el vehículo (01 00, 01 20, 01 40)
uint32_t g_pidSoporte[3]      = {0, 0, 0};
bool     g_pidRangoConocido[3] = {false, false, false};
#endif

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
bool elmEscribir(const char* cmd);
void enviarComando(const char* cmd);
void avanzarPID();
void pedirGNSSCrudo(bool conservarFix = false);
void leerIMU();
String construirJSONraw();
void publicarMQTT(const String& payload);
bool iniciarSD();
void guardarCSVtraducido();
float traducirPID(uint8_t pid, const String& hexStr);
String limpiarOBD(const String& bruto, uint8_t pid);
String limpiarDTC(const String& bruto);
String decodificarDTC(const String& bruto);
String categoriaDTC(char letra);
String descripcionDTC(const String& code);
String alertaDTC(const String& codigos);

float leerNivelBateria();
void  leerEstadoCarga();
void  actualizarLEDCarga();

void    esperarMs(uint32_t ms);
void    revisarURCs(const String& s);
void    drenarSerieModem();
String  simSend(const String& cmd, uint32_t timeout = 3000, const String& waitFor = "OK");
String  esperarRespuesta(const String& token, uint32_t timeout);
bool    simSendCheck(const String& cmd, const String& expected = "OK", uint32_t timeout = 3000);
bool    modemResponde(uint8_t intentos, uint32_t tMs);
bool    modemEsperarListo(uint32_t timeoutMs);
bool    modemEsperarApagado(uint32_t timeoutMs);
void    modemMarcarReiniciado();
void    pwrkeyPulso(uint32_t msBajo);
bool    modemEncender();
bool    modemApagar();
bool    iniciarRed();
int     estadoRegistro(const char* cmd, const char* tag);
bool    esperarRegistro(uint32_t timeoutMs);
bool    tieneIP(const String& r);
bool    iniciarMQTT();
void    reconectarMQTT();
void    teardownMQTT();

struct CamposGNSS { String lat, ns, lon, ew, fecha, hora, alt, vel, rumbo; };
bool    esCoordenada(const String& s);
bool    gnssExtraer(const String& linea, CamposGNSS& c);
bool    gnssEncender();
void    gnssApagar(bool permitirGuardar);

void    iniciarWatchdog();
void    alimentarWatchdog();

void    logDebug(const String& msg);
String  limpiarResp(String r);
String  motivoReset();
int     leerCSQ();

bool    respuestaTieneReinicio(const String& r);
void    esperarBootModulo();

#if MODO_AHORRO
enum ElmRes : uint8_t { ELM_OK = 0, ELM_TIMEOUT, ELM_SIN_ENLACE };
enum ObdRes : uint8_t { OBD_COMPLETO = 0, OBD_PARCIAL, OBD_ECU_NO_RESPONDE, OBD_FALLA_BLE };
void    elmLimpiarBuffer();
ElmRes  elmComando(const char* cmd, uint32_t timeoutMs, String& resp);
void    elmResync();
bool    elmInicializar();
bool    esHex(const String& s);
bool    esRespuestaSinECU(const String& r);
bool    parsearBitmap(const String& resp, uint8_t base, uint32_t& bm);
bool    pidSoportado(uint8_t pid);
bool    pidValido(uint8_t i);
int8_t  obdLeerSoporte();
bool    obdPollPID(uint8_t i, bool& enlaceCaido);
void    obdPollDTC();
bool    bleConectarELM();
void    bleDesconectar();
ObdRes  obdSesion();
bool    obdLeerCompleto();
void    ahorroApagarBLE();
void    gnssObtenerFix();
bool    ahorroDespertarModem();
bool    modemPrepararRed();
void    ahorroDormirModem();
void    cicloAhorro();
#endif

// ============================================================
//  WATCHDOG DE HARDWARE (nRF52)
//  Una vez arrancado no se puede detener ni reconfigurar: hay que
//  recargarlo antes de que venza o el chip se reinicia. Un reinicio por
//  software no lo detiene; por eso setup() también lo alimenta durante
//  sus esperas largas.
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

// Espera 'ms' alimentando el watchdog (en lugar de un delay largo).
void esperarMs(uint32_t ms)
{
  uint32_t t0 = millis();
  for (;;) {
    uint32_t pasado = millis() - t0;
    if (pasado >= ms) break;
    alimentarWatchdog();
    uint32_t resta = ms - pasado;
    delay(resta > 200 ? 200 : resta);
  }
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
  Serial.print("[LOG] "); Serial.println(msg);   // eco por serie también
  if (!sdLista) return;
  File f = SD.open(DEBUG_LOG_FILE, FILE_WRITE);   // FILE_WRITE = append (va al final)
  if (!f) return;
  f.print(millis());
  f.print(" | ");
  f.println(msg);
  f.close();
}

// Lee y limpia el registro de motivo de reinicio del nRF52.
// Distingue: WATCHDOG (cuelgue), SOFT (reinicio por software: liveness o
// ciclos fallidos), PIN (reset manual) y POWERON/BROWNOUT (corte de energía).
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
//  Si una respuesta trae el banner de arranque del módem, es que se
//  reinició en medio de la operación.
// ============================================================
bool respuestaTieneReinicio(const String& r)
{
  return (r.indexOf("*ATREADY")     >= 0) ||   // A7670 listo tras el arranque
         (r.indexOf("+CPIN: READY") >= 0) ||   // SIM reinicializada
         (r.indexOf("*ISIMAID")     >= 0);      // banner de arranque
}

// Espera a que el módem termine de arrancar (responde AT) y marca todo el
// contexto (red, MQTT, GNSS) como perdido.
void esperarBootModulo()
{
  logDebug("MODEM reinicio detectado -> esperando boot completo");
  modemMarcarReiniciado();
  if (modemEsperarListo(MODEM_T_BOOT_MS)) logDebug("MODEM boot completo -> se reinicializará red+MQTT");
  else                                    logDebug("MODEM no respondió tras el reinicio");
}

// ============================================================
//  SETUP
// ============================================================
void setup()
{
  // Leer el motivo del último reinicio antes de que algo lo modifique.
  g_motivoReset = motivoReset();

  delay(3000);
  Serial.begin(115200);

  Serial.println("=== OBD + GPS → MQTT RAW + SD TRADUCIDO (v3) ===");
  Serial.print("Motivo del último reinicio: ");
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

  digitalWrite(PIN_PWRKEY_7670, PWRKEY_SUELTO);   // fijar el nivel antes de habilitar la salida
  pinMode(PIN_PWRKEY_7670, OUTPUT);
  digitalWrite(PIN_PWRKEY_7670, PWRKEY_SUELTO);   // PWRKEY suelto en reposo

  // ADC: referencia y resolución (una sola vez)
  analogReference(AR_INTERNAL);   // 0.6 V con ganancia 1/6 -> fondo de escala 3.6 V
  analogReadResolution(12);

  // SD primero: así el log registra también el arranque del módem.
  if (iniciarSD()) sdLista = true;

  // Borrar el log si superó el tamaño máximo y registrar el arranque.
  if (sdLista) {
    File fchk = SD.open(DEBUG_LOG_FILE, FILE_READ);
    if (fchk) {
      uint32_t tam = fchk.size();
      fchk.close();
      if (tam > DEBUG_LOG_MAX) SD.remove(DEBUG_LOG_FILE);
    }
  }
  voltajeBateria = leerNivelBateria();
  leerEstadoCarga();
  logDebug("========== BOOT v3.1  reset=" + g_motivoReset +
           "  vbat=" + String(voltajeBateria, 2) +
           "  cargando=" + String(cargando ? 1 : 0) +
           "  modo_modem=" + String(AHORRO_MODO_MODEM) +
           "  pwrkey_alto=" + String(PWRKEY_ACTIVO_ALTO) +
           "  agps=" + String(GNSS_USAR_AGPS) + " ==========");

  // MÓDEM: el estado se consulta por AT antes de pulsar PWRKEY.
  SIM_SERIAL.begin(SIM_BAUD);
  delay(200);
  while (SIM_SERIAL.available()) SIM_SERIAL.read();

#if MODO_AHORRO && (AHORRO_MODO_MODEM == 2)
  // Modo 2: se arranca con el módem apagado (estado conocido); el ciclo lo enciende.
  if (modemResponde(2, 500)) {
    Serial.println("Módem encendido -> se apaga para arrancar limpio.");
    modemApagar();
  } else {
    Serial.println("Módem no responde (apagado o arrancando).");
    g_modemApagado = true;
  }
  Serial.println("[AHORRO] Red/MQTT se levantan por ciclo (no en setup).");
#else
  // Modos 0 y 1 (desarrollo futuro) y modo continuo: el módem queda
  // encendido desde el arranque.
  {
    bool yaEncendido = modemResponde(2, 500);
    if (!modemEncender()) {
      Serial.println("AVISO: el módem no responde.");
    } else if (yaEncendido) {
      // Venía encendido (por ejemplo, tras un reinicio del micro): AT+CRESET
      // para arrancar con la pila TCP/MQTT limpia.
      Serial.println("Módem ya encendido -> AT+CRESET.");
      simSend("AT+CRESET", 3000, "OK");
      modemMarcarReiniciado();
      esperarMs(3000);
      modemEsperarListo(MODEM_T_BOOT_MS);
    }
  #if !MODO_AHORRO
    if (iniciarRed())  simListo = true;
    else Serial.println("AVISO: red 4G no disponible, se reintentará.");

    if (simListo && iniciarMQTT()) mqttOnline = true;
    else Serial.println("AVISO: MQTT offline, se reintentará.");
  #endif
  }
#endif

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
  Bluefruit.autoConnLed(false); // el LED azul lo maneja el firmware
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
  // Por defecto el scanner se relanza solo cuando se desconecta el periférico,
  // y eso hacía que el ELM327 se encontrara un ciclo sí y otro no. En ahorro
  // el scanner lo maneja únicamente bleConectarELM().
  Bluefruit.Scanner.restartOnDisconnect(false);
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

  // Watchdog y contador de liveness al final del setup.
  tUltimoPublish = millis();
  iniciarWatchdog();

#if MODO_AHORRO
  tiempoProxCiclo = millis();   // el primer ciclo arranca enseguida
#endif
}

// ============================================================
//  LOOP
// ============================================================
void loop()
{
  alimentarWatchdog();

  // Liveness: reinicio si pasa demasiado tiempo sin publicar.
  // LIVENESS_MS = 0 lo desactiva (modo ahorro).
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
  // Ciclo anclado a su inicio: el período es INTERVALO_CICLO (o la duración
  // del ciclo, si fue mayor). La resta con signo sigue funcionando cuando
  // millis() da la vuelta (~49 días).
  if ((int32_t)(millis() - tiempoProxCiclo) >= 0) {
    uint32_t tInicio = millis();
    cicloAhorro();
    tiempoProxCiclo = tInicio + INTERVALO_CICLO;
    if ((int32_t)(millis() - tiempoProxCiclo) >= 0)
      tiempoProxCiclo = millis() + AHORRO_PAUSA_MIN_MS;
  } else {
    uint32_t restante = tiempoProxCiclo - millis();
    uint32_t paso = (restante > 2000UL) ? 2000UL : restante;
    alimentarWatchdog();   // mantener el WDT durante la espera
    delay(paso);           // esperas cortas (< WDT_TIMEOUT_S)
  }
  return;   // en ahorro no se ejecuta la máquina de estados continua
#endif

  // Modo continuo (MODO_AHORRO = 0): heartbeat MQTT
  static uint32_t tHeartbeat = 0;
  if (!mqttOnline && millis() - tHeartbeat >= 60000) {
    tHeartbeat = millis();
    Serial.println("[HB] MQTT offline — reconectando...");
    reconectarMQTT();
  }

  // Batería y estado de carga cada 5 s
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
  // Camino de diagnóstico: sin BLE/OBD.
  {
    static uint32_t tProxDiag = 0;
    if ((int32_t)(millis() - tProxDiag) >= 0) {
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
      if ((int32_t)(millis() - tiempoProxCiclo) >= 0) {
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
        // Limpiar el ruido del ELM327 y extraer sólo la trama hex válida.
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

    case ESPERANDO_DTC:   // respuesta al Modo 03
      if (elmListo) {
        rawDTC = limpiarDTC(String(bufELM));   // "" si no es una respuesta 43
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
//  Avanzar PID o pasar a GNSS (modo continuo)
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
    // Terminados los PIDs, pedir los DTC (Modo 03) antes del GNSS.
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
//  Bluefruit ejecuta todos estos callbacks en una única tarea ("Callback").
//  Si uno se bloquea, los demás (incluidas las notificaciones del ELM)
//  quedan encolados detrás. Por eso en ahorro son mínimos.
// ============================================================
void scan_callback(ble_gap_evt_adv_report_t* report)
{
#if MODO_AHORRO
  // En ahorro el callback sólo avisa que encontró el ELM327; la conexión la
  // hace bleConectarELM() en el loop. Si no corresponde, el scanner queda
  // pausado (el loop lo detiene con stop()).
  if (!blePermitido || g_elmEncontrado) return;

  char name[32] = {0};
  if (!Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                           (uint8_t*)name, sizeof(name) - 1)) {
    Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
                                        (uint8_t*)name, sizeof(name) - 1);
  }
  if (name[0] != '\0' && strstr(name, ELM_NOMBRE_FILTRO) != NULL) {
    memcpy(&g_elmAddr, &report->peer_addr, sizeof(g_elmAddr));
    __DMB();                         // la dirección queda escrita antes que la bandera
    g_elmEncontrado = true;
    Serial.printf("Encontrado: %-20s  RSSI: %d\n", name, report->rssi);
    return;                          // sin resume(): el loop hace stop() + connect()
  }
  Bluefruit.Scanner.resume();
#else
  // Si ya se está conectando al ELM327, se ignoran los anuncios rezagados.
  if (!blePermitido || bleConectando) return;

  char name[32] = {0};
  Bluefruit.Scanner.parseReportByType(
    report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, (uint8_t*)name, sizeof(name));
  if (strlen(name) > 0) {
    Serial.printf("Encontrado: %-20s  RSSI: %d\n", name, report->rssi);
    if (strstr(name, ELM_NOMBRE_FILTRO) != NULL) {
      bleConectando = true;
      Bluefruit.Scanner.stop();
      Bluefruit.Central.connect(report);
      return;
    }
  }
  if (blePermitido && !bleConectando) {
    Bluefruit.Scanner.resume();
  }
#endif
}

void connect_callback(uint16_t conn_handle)
{
  // Conexión concretada fuera de la ventana BLE: se corta.
  if (!blePermitido) {
    Bluefruit.disconnect(conn_handle);
    digitalWrite(LED_BLUE, LOW);
    return;
  }

  g_connHandle = conn_handle;

#if MODO_AHORRO
  // Nada bloqueante acá: el discovery y la inicialización del ELM los hace el loop.
  g_bleConectado = true;
  Serial.println("BLE conectado (discovery/init en el loop).");
#else
  Serial.println("Conectado. Descubriendo servicios...");

  pidActual = 0;
  bufELMIdx = 0; bufELM[0] = '\0'; elmListo = false;
  elmInicializado = false;
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
  // Dar tiempo a que el ELM termine su inicialización y descartar el banner.
  delay(1200);
  bufELMIdx = 0; bufELM[0] = '\0'; elmListo = false;

  if (!blePermitido) {
    Bluefruit.disconnect(conn_handle);
    digitalWrite(LED_BLUE, LOW);
    return;
  }

  elmInicializado = true;
  digitalWrite(LED_BLUE, HIGH);
  Serial.println("ELM327 listo");
#endif
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  g_connHandle    = BLE_CONN_HANDLE_INVALID;   // el handle ya no es válido
  g_bleConectado  = false;
  estado          = IDLE;
  pidActual       = 0;
  elmInicializado = false;
  bleConectando   = false;
  digitalWrite(LED_BLUE, LOW);
  Serial.printf("BLE desconectado (0x%02X)\n", reason);
#if !MODO_AHORRO
  // En ahorro no se relanza el scanner acá: el BLE queda apagado entre ciclos.
  bufELMIdx = 0; bufELM[0] = '\0'; elmListo = false;
  delay(500);
  digitalWrite(LED_BLUE, LOW);
  Bluefruit.Scanner.start(0);
#endif
}

void notify_callback(BLEClientCharacteristic*, uint8_t* data, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++) {
    char c = (char)data[i];
    if (c == '>') {
      bufELM[bufELMIdx] = '\0';
      elmListo = true;
    } else if (c == '\r' || c == '\n') {
      // Fin de línea -> separador '|' (uno solo por corte de línea)
      if (bufELMIdx > 0 && bufELM[bufELMIdx - 1] != '|' && bufELMIdx < ELM_BUF_SIZE - 1)
        bufELM[bufELMIdx++] = '|';
    } else if (c != 0 && bufELMIdx < ELM_BUF_SIZE - 1) {
      bufELM[bufELMIdx++] = c;
    }
  }
}

// Comando + '\r' en una sola escritura BLE.
bool elmEscribir(const char* cmd)
{
  char tmp[24];
  size_t n = strlen(cmd);
  if (n > sizeof(tmp) - 1) n = sizeof(tmp) - 1;
  memcpy(tmp, cmd, n);
  tmp[n++] = '\r';
  return elmChar.write(tmp, (uint16_t)n) == (uint16_t)n;
}

void enviarComando(const char* cmd)
{
  elmEscribir(cmd);
}

// ============================================================
//  GNSS
// ============================================================

// ¿Parece una coordenada? (dígitos y a lo sumo un punto: "3800.498300" o "38.008305")
bool esCoordenada(const String& s)
{
  if (s.length() < 3) return false;
  uint8_t puntos = 0;
  for (uint16_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '.') { if (++puntos > 1) return false; }
    else if (c < '0' || c > '9') return false;
  }
  return true;
}

// Ubica los campos por los indicadores de hemisferio (N/S y E/W). Así
// funciona con los tres formatos de +CGNSSINFO que documenta SIMCom:
//   "foreign" A7670SA: modo,GPS,BDS,GLONASS,GALILEO,lat,N/S,lon,E/W,fecha,hora,alt,vel,rumbo,...
//   ejemplo del manual: modo,GPS,GLONASS,BDS,lat,N/S,...
//   "domestic":         modo,GPS,BDS,lat,N/S,...
// En el formato del A7670SA el campo 4 es el contador de satélites Galileo,
// no la latitud.
bool gnssExtraer(const String& linea, CamposGNSS& c)
{
  const uint8_t MAXF = 20;
  String f[MAXF];
  uint8_t n = 0;
  int pos = 0;
  for (int i = 0; i <= (int)linea.length() && n < MAXF; i++) {
    if (i == (int)linea.length() || linea[i] == ',') {
      f[n] = linea.substring(pos, i);
      f[n].trim();
      n++;
      pos = i + 1;
    }
  }
  for (uint8_t k = 1; k + 2 < n; k++) {
    if ((f[k] == "N" || f[k] == "S") && (f[k + 2] == "E" || f[k + 2] == "W") &&
        esCoordenada(f[k - 1]) && esCoordenada(f[k + 1])) {
      c.lat   = f[k - 1];
      c.ns    = f[k];
      c.lon   = f[k + 1];
      c.ew    = f[k + 2];
      c.fecha = (k + 3 < n) ? f[k + 3] : String("");
      c.hora  = (k + 4 < n) ? f[k + 4] : String("");
      c.alt   = (k + 5 < n) ? f[k + 5] : String("");
      c.vel   = (k + 6 < n) ? f[k + 6] : String("");
      c.rumbo = (k + 7 < n) ? f[k + 7] : String("");
      return true;
    }
  }
  return false;
}

// Pide AT+CGNSSINFO y, si hay fix válido, deja la línea completa en rawGNSS.
// conservarFix = true (ahorro): una lectura sin fix no borra un fix ya
// obtenido en el ciclo. false (modo continuo): sin fix se informa NO_FIX.
void pedirGNSSCrudo(bool conservarFix)
{
#if !DIAG_CON_GNSS
  // GNSS deshabilitado para ahorrar corriente.
  (void)conservarFix;
  rawGNSS = "GNSS_OFF";
  return;
#else
  if (!gnssOn) gnssEncender();

  drenarSerieModem();
  SIM_SERIAL.println("AT+CGNSSINFO");

  String resp = "";
  resp.reserve(200);
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) {
    while (SIM_SERIAL.available()) resp += (char)SIM_SERIAL.read();
    int p = resp.indexOf("+CGNSSINFO:");
    if (p >= 0 && resp.indexOf("OK", p) >= 0) break;   // OK posterior a la trama
    if (resp.indexOf("ERROR") >= 0) break;
    alimentarWatchdog();
    delay(1);              // buffer RX de 64 bytes: leer seguido
  }
  revisarURCs(resp);

  bool hayFix = false;
  int ini = resp.indexOf("+CGNSSINFO:");
  if (ini >= 0) {
    g_gnssReady = true;    // respondió: el receptor está operativo
    String linea = resp.substring(ini + 11);

    // Cortar en el primer fin de línea real -> descarta "\r\n...OK"
    int fin = linea.length();
    for (int i = 0; i < (int)linea.length(); i++) {
      if (linea[i] == '\r' || linea[i] == '\n') { fin = i; break; }
    }
    linea = linea.substring(0, fin);

    // Sanitizar: sin espacios y sin "OK" residual pegado
    linea.trim();
    linea.replace(" ", "");
    linea.replace("OK", "");

    CamposGNSS c;
    hayFix = gnssExtraer(linea, c);
    int coma = linea.indexOf(',');
    String modo = (coma > 0) ? linea.substring(0, coma) : String("-");
    gnssDiag = "modo=" + modo + (hayFix ? " FIX" : " sin_fix");
    if (hayFix) {
      rawGNSS = linea;
      g_fixCiclo = true;
    }
  } else {
    gnssDiag = "sin respuesta CGNSSINFO";
  }
  if (!hayFix && !conservarFix) rawGNSS = "NO_FIX";

  Serial.print("<< GNSS: ");
  Serial.print(rawGNSS);
  Serial.print("  ["); Serial.print(gnssDiag); Serial.println("]");
#endif   // DIAG_CON_GNSS
}

// Enciende el GNSS (si hace falta) y fija la referencia del timeout de fix
// del ciclo. Con AP-Flash recupera las efemérides guardadas en la flash del
// módem (arranque en caliente aunque el módem haya estado apagado).
bool gnssEncender()
{
#if !DIAG_CON_GNSS
  return false;
#else
  if (!g_gnssRefFijada) { g_tGnssRef = millis(); g_gnssRefFijada = true; }
  if (gnssOn) return true;

  for (uint8_t k = 0; k < 3; k++) {
    // Estado actual. El formato de la respuesta indica si el firmware tiene
    // AP-Flash: "+CGNSSPWR: 0,1,1" (sí) o "+CGNSSPWR: 0" (no). El URC
    // "+CGNSSPWR: READY!" empieza igual, por eso se analiza línea por línea.
    String q = simSend("AT+CGNSSPWR?", 2000, "OK");
    bool vistoEstado = false, encendido = false, conComas = false;
    int desde = 0;
    for (;;) {
      int p = q.indexOf("+CGNSSPWR:", desde);
      if (p < 0) break;
      String v = q.substring(p + 10);
      int eol = v.indexOf('\r');
      if (eol >= 0) v = v.substring(0, eol);
      v.trim();
      if (v.indexOf("READY") >= 0) {
        g_gnssReady = true;
        encendido = true;                  // READY => el receptor está encendido
      } else if (v.length() > 0 && isdigit(v[0])) {
        vistoEstado = true;
        if (v[0] == '1') encendido = true;
        conComas = (v.indexOf(',') >= 0);
      }
      desde = p + 10;
    }
    if (vistoEstado && !conComas) g_apFlashOK = false;   // firmware sin AP-Flash
    if (encendido) {
      gnssOn = true;
      logDebug("GNSS ya estaba encendido");
      return true;
    }

#if GNSS_USAR_APFLASH
    if (g_apFlashOK) {
      String r1 = simSend("AT+CGNSSPWR=1,1", 3000, "OK");
      if (r1.indexOf("OK") >= 0) {
        gnssOn = true;
        if (r1.indexOf("READY") >= 0) g_gnssReady = true;
        g_tGnssRef = millis();            // el timeout corre desde el encendido real
        logDebug("GNSS ON (AP-Flash hot start)");
        return true;
      }
    }
#endif
    String r = simSend("AT+CGNSSPWR=1", 3000, "OK");
    if (r.indexOf("OK") >= 0) {
      gnssOn = true;
      if (r.indexOf("READY") >= 0) g_gnssReady = true;
      g_tGnssRef = millis();
#if GNSS_USAR_APFLASH
      if (g_apFlashOK) {                  // "=1,1" falló y "=1" no: el firmware no tiene AP-Flash
        g_apFlashOK = false;
        logDebug("GNSS: CGNSSPWR=1,1 rechazado -> se sigue sin AP-Flash");
      }
#endif
      logDebug("GNSS ON");
      return true;
    }
    esperarMs(1000);
  }
  logDebug("GNSS: no se pudo encender");
  return false;
#endif
}

// Apaga el GNSS. Si hubo fix en el ciclo (y pasó GNSS_GUARDAR_CADA_MS desde
// el último guardado), guarda las efemérides en la flash del módem
// (AT+CGNSSPWR=0,1) para que el próximo encendido sea en caliente.
void gnssApagar(bool permitirGuardar)
{
#if DIAG_CON_GNSS
  if (!gnssOn) return;
  bool guardar = false;
#if GNSS_USAR_APFLASH
  guardar = permitirGuardar && g_apFlashOK && g_fixCiclo &&
            (!g_gnssGuardado || (millis() - g_tGnssGuardado) >= GNSS_GUARDAR_CADA_MS);
#else
  (void)permitirGuardar;
#endif
  bool ok = false;
  if (guardar) {
    String r = simSend("AT+CGNSSPWR=0,1", 9000, "OK");
    ok = (r.indexOf("OK") >= 0);
    if (ok) {
      g_gnssGuardado  = true;
      g_tGnssGuardado = millis();
      logDebug("GNSS OFF + efemérides guardadas (AP-Flash)");
    }
  }
  if (!ok) simSend("AT+CGNSSPWR=0", 3000, "OK");
  gnssOn = false;
  g_gnssReady = false;
#else
  (void)permitirGuardar;
#endif
}

// ============================================================
//  Leer IMU - LSM6DS33
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
  j += ",\"dtc\":\"";    j += rawDTC; j += "\"";   // Modo 03 crudo (lo decodifica Node-RED)

  j += "}";
  return j;
}

// ============================================================
//  Publicar por MQTT
// ============================================================
void publicarMQTT(const String& payload)
{
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
  esperarRespuesta("OK", 3000);            // espera la confirmación del tópico

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
  esperarRespuesta("OK", 5000);

  r = simSend("AT+CMQTTPUB=0," + String(MQTT_QOS) + ",60,0", 10000, "+CMQTTPUB:");
  if (respuestaTieneReinicio(r)) {
    logDebug("PUB: reinicio del modem @CMQTTPUB" + ctx);
    esperarBootModulo();
    return;
  }
  if (r.indexOf("+CMQTTPUB: 0,0") >= 0) {
    Serial.println("MQTT: OK");
    tUltimoPublish = millis();   // publicación exitosa (liveness)
    logDebug("PUB OK" + ctx + " len=" + String(payload.length()));
  } else {
    mqttOnline = false;
    logDebug("PUB FAIL @CMQTTPUB" + ctx + " resp=" + limpiarResp(r));
  }
}

// ============================================================
//  Limpiar respuesta OBD del ruido del ELM327
//  Busca el marcador "41"+PID y devuelve sólo la corrida hex que sigue
//  (corta en el primer caracter no-hex, incluido el separador '|'):
//    "410C40D0|"                  -> "410C40D0"
//    "SEARCHING...|410D02|"       -> "410D02"
//    "410C1AF8|410C1AF8|" (2 ECU) -> "410C1AF8"
//    "OK" / "NO DATA"             -> "" (sin dato)
// ============================================================
String limpiarOBD(const String& bruto, uint8_t pid)
{
  String h = bruto;
  h.toUpperCase();
  h.replace(" ", "");

  char marc[12];
  snprintf(marc, sizeof(marc), "41%02X", pid);   // ej. "410C"
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

// Respuesta cruda del Modo 03 desde el "43" (sin separadores ni espacios).
// "" si no es una respuesta de DTC (sin soporte o NO DATA).
String limpiarDTC(const String& bruto)
{
  String h = bruto;
  h.toUpperCase();
  h.replace(" ", "");
  h.replace("|", "");
  int i = h.indexOf("43");
  if (i < 0) return "";
  return h.substring(i);
}

// ============================================================
//  Decodificar DTCs (Modo 03)
//  Entrada: respuesta cruda "43" + pares de bytes (cada DTC = 2 bytes).
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
    // b1/b2 en minúscula: el core define B1 y B2 como macros (binary.h).
    uint8_t b1 = (uint8_t)strtol(h.substring(k, k + 2).c_str(), nullptr, 16);
    uint8_t b2 = (uint8_t)strtol(h.substring(k + 2, k + 4).c_str(), nullptr, 16);
    if (b1 == 0 && b2 == 0) continue;   // relleno = sin código
    char cod[8];
    snprintf(cod, sizeof(cod), "%c%d%X%X%X",
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
//  Sin comas, para no romper el CSV.
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

  // Validar que sea una respuesta hex válida "41"+PID.
  char marc[12]; snprintf(marc, sizeof(marc), "41%02X", pid);
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
      if (!pids[i].enSD) continue;   // caja negra: sólo los PIDs marcados en enSD
#endif
      f.print(","); f.print(pids[i].nombre);
      f.print("("); f.print(pids[i].unidad); f.print(")");
    }
    f.print(",imu_ax(m/s2),imu_ay(m/s2),imu_az(m/s2)");
    f.print(",imu_gx(rad/s),imu_gy(rad/s),imu_gz(rad/s)");
    f.print(",vbat(V),cargando,carga_completa");
    f.print(",dtc");        // códigos decodificados (P/C/B/U)
#if !MODO_AHORRO
    f.print(",dtc_alerta"); // descripción legible (sólo en modo continuo)
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

  CamposGNSS c;
  if (rawGNSS != "NO_FIX" && rawGNSS != "GNSS_OFF" && gnssExtraer(rawGNSS, c)) {
    if (c.fecha.length() >= 6)
      f.print(c.fecha.substring(0,2) + "-" + c.fecha.substring(2,4) + "-" + c.fecha.substring(4,6));
    f.print(",");

    String hora = c.hora;
    int puntoIdx = hora.indexOf('.');
    if (puntoIdx > 0) hora = hora.substring(0, puntoIdx);
    String horaLimpia = "";
    for (uint16_t k = 0; k < hora.length(); k++)
      if (isdigit(hora[k])) horaLimpia += hora[k];
    while (horaLimpia.length() < 6) horaLimpia = horaLimpia + "0";
    f.print(horaLimpia.substring(0,2) + ":" + horaLimpia.substring(2,4) + ":" + horaLimpia.substring(4,6));
    f.print(",");

    f.print(c.lat); f.print(",");
    f.print(c.ns);  f.print(",");
    f.print(c.lon); f.print(",");
    f.print(c.ew);  f.print(",");

    f.print(c.alt.length() ? c.alt : String("0")); f.print(",");        // alt

    float velKph = c.vel.length() ? c.vel.toFloat() * 1.852f : 0.0f;    // vel (nudos->kph)
    f.print(velKph, 2); f.print(",");

    f.print(c.rumbo.length() ? c.rumbo : String("0"));                  // rumbo
  } else {
    // Región GNSS = 9 campos = 8 comas (la última la agrega el bucle de PIDs).
    f.print(",,,,,,,,");
  }

  for (uint8_t i = 0; i < NUM_PIDS; i++) {
#if MODO_AHORRO
    if (!pids[i].enSD) continue;   // mismo filtro que la cabecera
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

  {
    String cod = decodificarDTC(rawDTC);
    f.print(","); f.print(cod.length() > 0 ? cod : String("none"));
#if !MODO_AHORRO
    f.print(","); f.print(alertaDTC(cod));
#endif
  }

  f.println();
  f.close();
  Serial.println("SD: fila traducida guardada");
}

// ============================================================
//  UART del módem: lectura, envío y URCs
// ============================================================

// Revisa los URCs que llegan sueltos (fuera de la respuesta de un comando):
// el "+CGNSSPWR: READY!" del GNSS y el banner de arranque del módem.
void revisarURCs(const String& s)
{
  // Banner de arranque con el contexto activo: el módem se reinició solo
  // (en las pruebas pasaba durante AT+CAGPS). Se marca todo como perdido
  // para que el ciclo lo reconstruya (GNSS, red y MQTT).
  if (s.indexOf("*ATREADY") >= 0 && (gnssOn || simListo || mqttSvcUp || mqttOnline)) {
    g_reiniciosCiclo++;
    modemMarcarReiniciado();
  }
  if (s.indexOf("CGNSSPWR: READY") >= 0 || s.indexOf("CGNSSPWR:READY") >= 0) g_gnssReady = true;
}

// Vacía el RX del módem antes de mandar un comando, sin perder los URCs
// de interés.
void drenarSerieModem()
{
  if (!SIM_SERIAL.available()) return;
  String s;
  s.reserve(96);
  uint32_t t0 = millis();
  while (millis() - t0 < 30) {
    while (SIM_SERIAL.available()) {
      char c = (char)SIM_SERIAL.read();
      if (s.length() < 250) s += c;
    }
    delay(1);
  }
  revisarURCs(s);
}

String simSend(const String& cmd, uint32_t timeout, const String& waitFor)
{
  drenarSerieModem();
  SIM_SERIAL.println(cmd);
  String resp = "";
  resp.reserve(160);
  uint32_t t0 = millis();
  bool fin = false;
  while (millis() - t0 < timeout) {
    while (SIM_SERIAL.available()) resp += (char)SIM_SERIAL.read();
    if (waitFor.length() > 0 && resp.indexOf(waitFor) >= 0) { fin = true; break; }
    if (resp.indexOf("ERROR") >= 0) { fin = true; break; }   // no esperar el timeout completo
    alimentarWatchdog();   // mantener vivo el WDT durante esperas largas
    delay(1);              // el buffer RX es de 64 bytes (~5 ms a 115200)
  }
  // El token puede ser un prefijo de la línea (ej. "+CMQTTPUB:"): seguir
  // leyendo hasta que esa línea termine (máx. 150 ms).
  if (fin && waitFor != ">") {
    int p = resp.indexOf(waitFor);
    if (p < 0) p = 0;
    uint32_t tg = millis();
    while (millis() - tg < 150) {
      while (SIM_SERIAL.available()) resp += (char)SIM_SERIAL.read();
      if (resp.indexOf('\n', p) >= 0) break;
      delay(1);
    }
  }
  revisarURCs(resp);
  return resp;
}

// Lee (sin enviar nada) hasta que aparezca 'token' o "ERROR".
String esperarRespuesta(const String& token, uint32_t timeout)
{
  String resp = "";
  resp.reserve(64);
  uint32_t t0 = millis();
  while (millis() - t0 < timeout) {
    while (SIM_SERIAL.available()) resp += (char)SIM_SERIAL.read();
    if (resp.indexOf(token) >= 0 || resp.indexOf("ERROR") >= 0) break;
    alimentarWatchdog();
    delay(1);
  }
  revisarURCs(resp);
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
//  ENERGÍA DEL MÓDEM
//  Regla: el estado se consulta con "AT" antes de tocar PWRKEY. Un pulso
//  corto (1 s < Toff 2.5 s) nunca apaga a un módem encendido, y el pulso
//  largo sólo se usa sobre un módem que se sabe encendido o colgado.
// ============================================================
bool modemResponde(uint8_t intentos, uint32_t tMs)
{
  for (uint8_t i = 0; i < intentos; i++) {
    String r = simSend("AT", tMs, "OK");
    if (r.indexOf("OK") >= 0) return true;
  }
  return false;
}

// Espera a que el módem responda AT (arranque ~11 s según hoja de datos).
bool modemEsperarListo(uint32_t timeoutMs)
{
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (modemResponde(1, 500)) {
      simSend("ATE0", 1000, "OK");
      Serial.printf("[MODEM] responde tras %lu ms\n", (unsigned long)(millis() - t0));
      return true;
    }
    esperarMs(300);
  }
  return false;
}

// Espera a que el módem deje de responder (2 sondeos seguidos sin respuesta).
bool modemEsperarApagado(uint32_t timeoutMs)
{
  uint32_t t0 = millis();
  uint8_t mudo = 0;
  while (millis() - t0 < timeoutMs) {
    esperarMs(700);
    if (modemResponde(1, 400)) mudo = 0;
    else if (++mudo >= 2) return true;
  }
  return false;
}

// Todo el contexto del módem (red, MQTT, GNSS) se perdió.
void modemMarcarReiniciado()
{
  simListo    = false;
  mqttOnline  = false;
  mqttSvcUp   = false;
  gnssOn      = false;
  g_gnssReady = false;
}

// Pulso en PWRKEY (respeta PWRKEY_ACTIVO_ALTO), alimentando el watchdog.
void pwrkeyPulso(uint32_t msBajo)
{
  digitalWrite(PIN_PWRKEY_7670, PWRKEY_SUELTO);
  delay(20);
  digitalWrite(PIN_PWRKEY_7670, PWRKEY_PRESIONADO);
  esperarMs(msBajo);
  digitalWrite(PIN_PWRKEY_7670, PWRKEY_SUELTO);
}

// Deja el módem encendido y respondiendo AT. Resuelve los casos "ya estaba
// encendido", "apagado" y "encendido pero mudo o colgado" sin invertir el estado.
bool modemEncender()
{
  digitalWrite(PIN_SLEEP_7670, LOW);            // DTR bajo = despierto
  if (modemResponde(2, 500)) {                  // ya estaba encendido
    if (g_modemApagado) {
      // Se lo apagó con CPOF en el ciclo anterior y volvió a encenderse solo.
      g_autoOnCiclo = true;
      if (!g_autoOnAvisado) {
        g_autoOnAvisado = true;
        logDebug("MODEM encendido al iniciar el ciclo aunque se apagó con CPOF: se "
                 "re-enciende solo (PWRKEY queda presionado en reposo?). Revisar PWRKEY_ACTIVO_ALTO.");
      }
    }
    simSend("ATE0", 800, "OK");
    g_modemApagado = false;
    return true;
  }

  Serial.println("[MODEM] encendiendo (PWRKEY)...");
  modemMarcarReiniciado();
  pwrkeyPulso(PWRKEY_T_ON_MS);
  if (modemEsperarListo(MODEM_T_BOOT_MS)) {
    g_modemApagado = false;
    return true;
  }

  // No respondió: estaba encendido pero colgado/dormido (el pulso corto no lo
  // afecta). Ciclo por HW: pulso largo (apaga), pausa, y encendido.
  logDebug("MODEM mudo tras PWRKEY -> pulso largo (apagar) + encender");
  pwrkeyPulso(PWRKEY_T_OFF_MS);
  esperarMs(MODEM_T_OFF_ON_MS);
  if (modemEsperarListo(4000)) {                // estaba apagado y el pulso largo lo encendió
    g_modemApagado = false;
    return true;
  }
  pwrkeyPulso(PWRKEY_T_ON_MS);
  if (modemEsperarListo(MODEM_T_BOOT_MS)) {
    g_modemApagado = false;
    return true;
  }
  logDebug("MODEM sigue mudo tras la recuperación por HW");
  return false;
}

// Apaga el módem (AT+CPOF) y confirma que dejó de responder. Sólo si sigue
// vivo se usa el pulso largo de PWRKEY. Nunca se pulsa sobre un módem que
// no responde: si estaba apagado, el pulso lo encendería.
bool modemApagar()
{
  if (!modemResponde(2, 500)) {
    // Mudo: ya apagado o colgado. Si está colgado, el próximo modemEncender()
    // lo detecta y hace el ciclo por HW.
    g_modemApagado = true;
    modemMarcarReiniciado();
    Serial.println("[MODEM] no responde: se asume apagado.");
    return true;
  }

  gnssApagar(true);                             // guarda efemérides si corresponde
  uint32_t t0 = millis();
  simSend("AT+CPOF", 3000, "OK");
  if (modemEsperarApagado(MODEM_T_OFF_MS)) {
    Serial.printf("[MODEM] apagado por CPOF en %lu ms\n", (unsigned long)(millis() - t0));
    g_modemApagado = true;
    modemMarcarReiniciado();
    return true;
  }

  logDebug("MODEM sigue vivo tras CPOF -> PWRKEY largo");
  pwrkeyPulso(PWRKEY_T_OFF_MS);
  if (modemEsperarApagado(8000)) {
    g_modemApagado = true;
    modemMarcarReiniciado();
    return true;
  }
  logDebug("MODEM no se apagó (se reintenta el próximo ciclo)");
  g_modemApagado = false;
  return false;
}

// ============================================================
//  RED
// ============================================================
int estadoRegistro(const char* cmd, const char* tag)
{
  String r = simSend(cmd, 2000, tag);
  int p = r.indexOf(tag);
  if (p < 0) return -1;
  int c = r.indexOf(',', p);        // "+CEREG: <n>,<stat>[,...]"
  if (c < 0) return -1;
  return r.substring(c + 1).toInt();
}

// Registrado si CEREG (LTE/EPS) o CGREG dan 1 (local) o 5 (roaming).
bool esperarRegistro(uint32_t timeoutMs)
{
  uint32_t t0 = millis();
  bool avisoDenegado = false;
  while (millis() - t0 < timeoutMs) {
    int e = estadoRegistro("AT+CEREG?", "+CEREG:");
    if (e == 1 || e == 5) return true;
    int g = estadoRegistro("AT+CGREG?", "+CGREG:");
    if (g == 1 || g == 5) return true;
    if ((e == 3 || g == 3) && !avisoDenegado) {
      logDebug("RED registro DENEGADO (CEREG=" + String(e) + " CGREG=" + String(g) + ")");
      avisoDenegado = true;
    }
    Serial.print(".");
    esperarMs(1500);
  }
  return false;
}

bool tieneIP(const String& r)
{
  int p = r.indexOf("+CGPADDR:");
  if (p < 0) return false;
  int c = r.indexOf(',', p);
  if (c < 0) return false;
  String ip = r.substring(c + 1);
  int eol = ip.indexOf('\r');
  if (eol >= 0) ip = ip.substring(0, eol);
  ip.trim();
  ip.replace("\"", "");
  return ip.length() >= 7 && ip != "0.0.0.0" && isdigit(ip[0]);
}

bool iniciarRed()
{
  Serial.println("--- Iniciando red 4G ---");
  if (!modemResponde(3, 800)) { Serial.println("  módem NO RESPONDE"); return false; }
  simSend("ATE0", 1000, "OK");

  // Sólo LTE (CNMP=38): evita caer a 2G, cuyos picos de ~2 A provocan caídas
  // de tensión con batería. CNMP queda guardado en el módem: se escribe sólo
  // si difiere (reescribirlo en cada ciclo podía forzar una nueva búsqueda).
  String r = simSend("AT+CNMP?", 2000, "+CNMP:");
  if (r.indexOf("+CNMP: 38") < 0) {
    simSendCheck("AT+CNMP=38", "OK", 3000);
    logDebug("RED CNMP=38 escrito (antes: " + limpiarResp(r) + ")");
  }

  // APN sólo si difiere. Un ERROR acá no aborta la conexión.
  r = simSend("AT+CGDCONT?", 2000, "OK");
  if (r.indexOf("\"" APN "\"") < 0) {
    if (!simSendCheck("AT+CGDCONT=1,\"IP\",\"" APN "\"", "OK", 3000))
      logDebug("RED CGDCONT rechazado (se sigue)");
  }

  Serial.print("  Registro...");
  uint32_t t0 = millis();
  if (!esperarRegistro(RED_T_REGISTRO_MS)) {
    Serial.println(" SIN SEÑAL");
    logDebug("RED FAIL registro (" + String(RED_T_REGISTRO_MS / 1000) + " s) CSQ=" + String(leerCSQ()));
    return false;
  }
  Serial.println(" OK");
  logDebug("RED registro OK en " + String((millis() - t0) / 1000) + " s  CSQ=" + String(leerCSQ()));
  logDebug("RED CPSI=" + limpiarResp(simSend("AT+CPSI?", 2000, "+CPSI:")));

  // PDP + IP (verificada, con un reintento)
  bool ip = false;
  for (uint8_t k = 0; k < 2 && !ip; k++) {
    simSend("AT+CGACT=1,1", 15000, "OK");
    String a = simSend("AT+CGPADDR=1", 3000, "+CGPADDR:");
    ip = tieneIP(a);
    if (ip) logDebug("RED IP=" + limpiarResp(a.substring(a.indexOf("+CGPADDR:"))));
    else    esperarMs(2000);
  }
  if (!ip) { logDebug("RED sin IP tras CGACT"); return false; }

#if DIAG_CON_GNSS
  gnssEncender();                     // (en ahorro ya viene encendido: no hace nada)
#else
  if (gnssOn) { simSend("AT+CGNSSPWR=0", 3000, "OK"); gnssOn = false; }
#endif
  Serial.println("  Red 4G lista");
  return true;
}

// ============================================================
//  teardownMQTT
//  Orden del manual A76XX: DISC -> REL -> STOP. Sin el REL, el STOP no
//  libera el servicio.
// ============================================================
void teardownMQTT()
{
  Serial.println("  [MQTT] teardown...");
  simSend("AT+CMQTTDISC=0,10", 12000, "+CMQTTDISC:");
  simSend("AT+CMQTTREL=0", 3000, "OK");
  simSend("AT+CMQTTSTOP", 6000, "+CMQTTSTOP:");
  mqttSvcUp  = false;
  mqttOnline = false;
}

// ============================================================
//  iniciarMQTT
// ============================================================
bool iniciarMQTT()
{
  Serial.println("--- Iniciando MQTT ---");

  // Servicio (START + ACCQ) sólo si no está arrancado. Por ciclo alcanza con
  // CONNECT/DISC; el servicio se recrea tras teardown, reboot o CRESET.
  if (!mqttSvcUp) {
    String r = "";
    for (uint8_t k = 0; k < 2; k++) {
      Serial.print("  [MQTT] START... ");
      r = simSend("AT+CMQTTSTART", 12000, "+CMQTTSTART:");
      if (respuestaTieneReinicio(r)) {
        Serial.println("REINICIO MODEM");
        logDebug("MQTT: reinicio del modem @START resp=" + limpiarResp(r));
        esperarBootModulo();
        return false;
      }
      if (r.indexOf("+CMQTTSTART: 0") >= 0 || r.indexOf("+CMQTTSTART: 23") >= 0) break;
      // Según el manual, ERROR = el servicio ya estaba arrancado: liberar y reintentar.
      logDebug("MQTT @START " + limpiarResp(r) + " -> teardown y reintento");
      teardownMQTT();
      esperarMs(1000);
    }
    if (r.indexOf("+CMQTTSTART: 0") < 0 && r.indexOf("+CMQTTSTART: 23") < 0) {
      Serial.println("FALLO -> " + r);
      logDebug("MQTT FAIL @START resp=" + limpiarResp(r));
      return false;
    }
    Serial.println("OK");

    Serial.print("  [MQTT] ACCQ... ");
    String accq = "AT+CMQTTACCQ=0,\"" + mqttClientID + "\"";
    r = simSend(accq, 5000, "OK");
    if (respuestaTieneReinicio(r)) {
      Serial.println("REINICIO MODEM");
      logDebug("MQTT: reinicio del modem @ACCQ resp=" + limpiarResp(r));
      esperarBootModulo();
      return false;
    }
    if (r.indexOf("OK") < 0) {
      // Cliente ya adquirido (de un intento previo): liberarlo y reintentar.
      simSend("AT+CMQTTREL=0", 3000, "OK");
      r = simSend(accq, 5000, "OK");
      if (r.indexOf("OK") < 0) {
        Serial.println("FALLO -> " + r);
        logDebug("MQTT FAIL @ACCQ resp=" + limpiarResp(r));
        teardownMQTT();
        return false;
      }
    }
    Serial.println("OK");
    mqttSvcUp = true;   // servicio arrancado y cliente adquirido
  }

  Serial.print("  [MQTT] CONNECT... ");
  String connCmd = "AT+CMQTTCONNECT=0,\"tcp://" + String(MQTT_BROKER) + ":" +
                   String(MQTT_PORT) + "\"," + String(MQTT_KEEPALIVE) + ",1";
  if (strlen(MQTT_USER) > 0)
    connCmd += ",\"" + String(MQTT_USER) + "\",\"" + String(MQTT_PASS) + "\"";

  String r = simSend(connCmd, 15000, "+CMQTTCONNECT:");
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
    teardownMQTT();          // limpio: el próximo intento arranca de cero
    return false;
  }
  Serial.println("MQTT: conectado");
  logDebug("MQTT CONNECT OK vbat=" + String(voltajeBateria, 2));
  return true;
}

// ============================================================
//  reconectarMQTT (modo continuo)
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
//  BATERÍA - leerNivelBateria()
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
//  CARGA - leerEstadoCarga() (TP4056: CHRG y STDBY activos en bajo)
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
//  ===============  MODO DE AHORRO DE ENERGÍA  ===============
// ============================================================
#if MODO_AHORRO

// ---------------- ELM327: comandos síncronos ----------------

void elmLimpiarBuffer()
{
  taskENTER_CRITICAL();              // evita que la tarea de callbacks escriba en medio
  bufELMIdx = 0;
  bufELM[0] = '\0';
  elmListo  = false;
  taskEXIT_CRITICAL();
}

// Envía un comando y espera el prompt '>' del ELM. Deja la respuesta en 'resp'.
ElmRes elmComando(const char* cmd, uint32_t timeoutMs, String& resp)
{
  resp = "";
  if (!g_bleConectado) return ELM_SIN_ENLACE;
  elmLimpiarBuffer();
  if (!elmEscribir(cmd)) return g_bleConectado ? ELM_TIMEOUT : ELM_SIN_ENLACE;

  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (elmListo) {
      char copia[ELM_BUF_SIZE];
      taskENTER_CRITICAL();
      memcpy(copia, bufELM, ELM_BUF_SIZE);
      taskEXIT_CRITICAL();
      copia[ELM_BUF_SIZE - 1] = '\0';
      resp = String(copia);
      return ELM_OK;
    }
    if (!g_bleConectado) return ELM_SIN_ENLACE;
    alimentarWatchdog();
    delay(2);
  }
  return ELM_TIMEOUT;
}

// Re-sincroniza tras un timeout o una respuesta que no corresponde: espera
// un prompt tardío y, si no llega, manda un '\r' (interrumpe al ELM si seguía
// ocupado -> "STOPPED", o repite el último comando) y descarta todo.
void elmResync()
{
  if (!g_bleConectado) return;
  uint32_t t0 = millis();
  while (!elmListo && millis() - t0 < 800) { alimentarWatchdog(); delay(5); }
  if (!elmListo) {
    elmLimpiarBuffer();
    uint8_t cr = '\r';
    elmChar.write(&cr, 1);
    t0 = millis();
    while (!elmListo && g_bleConectado && millis() - t0 < 2500) { alimentarWatchdog(); delay(5); }
  }
  esperarMs(60);
  elmLimpiarBuffer();
}

// Inicialización del ELM327, esperando el '>' de cada comando.
bool elmInicializar()
{
  String r;
  esperarMs(150);
  elmLimpiarBuffer();

  if (elmComando("AT Z", ELM_T_ATZ_MS, r) != ELM_OK) {
    elmResync();
    if (elmComando("AT Z", ELM_T_ATZ_MS, r) != ELM_OK) {
      logDebug("ELM init: AT Z sin prompt");
      return false;
    }
  }
  esperarMs(100);

  // H0 = sin headers (una trama por ECU sin prefijo); SP 0 = protocolo automático
  const char* const cmds[] = { "AT E0", "AT L0", "AT S0", "AT H0", "AT SP 0" };
  for (uint8_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
    ElmRes e = elmComando(cmds[i], ELM_T_AT_MS, r);
    if (e != ELM_OK) { elmResync(); e = elmComando(cmds[i], ELM_T_AT_MS, r); }
    if (e != ELM_OK) {
      logDebug(String("ELM init: sin prompt en ") + cmds[i]);
      return false;
    }
    // Algunos clones responden "?" a comandos opcionales: se registra y se sigue.
    if (r.indexOf("OK") < 0) logDebug(String("ELM init: ") + cmds[i] + " -> " + limpiarResp(r));
    esperarMs(ELM_GAP_MS);
  }
  return true;
}

// ---------------- OBD: soporte de PIDs y lectura ----------------

bool esHex(const String& s)
{
  if (s.length() == 0) return false;
  for (uint16_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) return false;
  }
  return true;
}

// La ECU no contesta (motor apagado, sin protocolo): no es un problema del BLE.
bool esRespuestaSinECU(const String& r)
{
  String h = r;
  h.toUpperCase();
  h.replace(" ", "");
  return h.indexOf("UNABLETOCONNECT") >= 0 || h.indexOf("NODATA") >= 0 ||
         h.indexOf("BUSINIT") >= 0 || h.indexOf("ERROR") >= 0;
}

// Lee los 4 bytes del mapa "41"+base (01 00 / 01 20 / 01 40). Si responden
// varias ECUs, se combinan (OR) sus mapas, trama por trama ('|').
bool parsearBitmap(const String& resp, uint8_t base, uint32_t& bm)
{
  String h = resp;
  h.toUpperCase();
  h.replace(" ", "");
  char marc[12];
  snprintf(marc, sizeof(marc), "41%02X", base);
  bm = 0;
  bool ok = false;
  int ini = 0;
  while (ini <= (int)h.length()) {
    int fin = h.indexOf('|', ini);
    if (fin < 0) fin = h.length();
    String seg = h.substring(ini, fin);
    int m = seg.indexOf(marc);
    if (m >= 0 && m + 12 <= (int)seg.length()) {
      String d = seg.substring(m + 4, m + 12);
      if (esHex(d)) {
        bm |= (uint32_t)strtoul(d.c_str(), nullptr, 16);
        ok = true;
      }
    }
    ini = fin + 1;
  }
  return ok;
}

// ¿El vehículo declara soportar este PID? Si el rango no se pudo leer, se
// asume que sí (criterio estricto: se le exige respuesta).
bool pidSoportado(uint8_t pid)
{
  if (pid == 0) return true;
  uint8_t idx = (uint8_t)((pid - 1) / 0x20);      // 01-20 -> 0, 21-40 -> 1, 41-60 -> 2
  if (idx > 2 || !g_pidRangoConocido[idx]) return true;
  uint8_t off = (uint8_t)(pid - idx * 0x20);      // 1..32 (bit 31 = primer PID del rango)
  return ((g_pidSoporte[idx] >> (32 - off)) & 1UL) != 0;
}

bool pidValido(uint8_t i)
{
  return rawPID[i].length() >= 6 && rawPID[i].startsWith("41");
}

// 01 00 (+ 01 20 / 01 40 si corresponde). La primera consulta tras AT SP 0
// dispara la búsqueda de protocolo ("SEARCHING...") y puede tardar varios s.
// Devuelve 1 = OK, 0 = la ECU no responde, -1 = falla del BLE/ELM.
int8_t obdLeerSoporte()
{
  for (uint8_t k = 0; k < 3; k++) { g_pidSoporte[k] = 0; g_pidRangoConocido[k] = false; }

  String r;
  bool ok = false;
  for (uint8_t intento = 0; intento < 2 && !ok; intento++) {
    ElmRes e = elmComando("0100", ELM_T_BUSQUEDA_MS, r);
    if (e == ELM_SIN_ENLACE) return -1;
    if (e == ELM_OK) {
      uint32_t bm = 0;
      if (parsearBitmap(r, 0x00, bm)) { g_pidSoporte[0] = bm; ok = true; break; }
      if (esRespuestaSinECU(r)) {
        logDebug("OBD 0100: la ECU no responde -> " + limpiarResp(r));
        return 0;
      }
    }
    elmResync();
  }
  if (!ok) {
    logDebug("OBD 0100 sin respuesta válida -> " + limpiarResp(r));
    return -1;
  }
  g_pidRangoConocido[0] = true;

  // Rangos siguientes: el bit del PID 0x20 (0x40) indica si existe 01 20 (01 40).
  for (uint8_t idx = 1; idx <= 2; idx++) {
    uint8_t basePrev = (uint8_t)((idx - 1) * 0x20);
    if (!g_pidRangoConocido[idx - 1]) break;                 // rango previo desconocido
    if ((g_pidSoporte[idx - 1] & 1UL) == 0) {                // no hay rango siguiente
      for (uint8_t j = idx; j <= 2; j++) { g_pidSoporte[j] = 0; g_pidRangoConocido[j] = true; }
      break;
    }
    char cmd[8];
    snprintf(cmd, sizeof(cmd), "01%02X", (unsigned)(basePrev + 0x20));
    for (uint8_t intento = 0; intento < 2; intento++) {
      esperarMs(ELM_GAP_MS);
      ElmRes e = elmComando(cmd, ELM_T_PID_MS, r);
      if (e == ELM_SIN_ENLACE) return -1;
      uint32_t bm = 0;
      if (e == ELM_OK && parsearBitmap(r, (uint8_t)(basePrev + 0x20), bm)) {
        g_pidSoporte[idx] = bm;
        g_pidRangoConocido[idx] = true;
        break;
      }
      if (e != ELM_OK) elmResync();
    }
  }
  logDebug("OBD soporte 01-20=" + String(g_pidSoporte[0], HEX) +
           " 21-40=" + (g_pidRangoConocido[1] ? String(g_pidSoporte[1], HEX) : String("?")) +
           " 41-60=" + (g_pidRangoConocido[2] ? String(g_pidSoporte[2], HEX) : String("?")));
  return 1;
}

// Pide un PID con reintentos. true si quedó una trama válida en rawPID[i].
bool obdPollPID(uint8_t i, bool& enlaceCaido)
{
  bool soportado = pidSoportado(pids[i].pid);
  uint8_t intentos = soportado ? (uint8_t)(1 + OBD_REINTENTOS_PID) : 1;
  rawPID[i] = "TIMEOUT";
  for (uint8_t k = 0; k < intentos; k++) {
    String r;
    ElmRes e = elmComando(pids[i].comando, ELM_T_PID_MS, r);
    if (e == ELM_SIN_ENLACE) { enlaceCaido = true; return false; }
    if (e == ELM_OK) {
      String limpio = limpiarOBD(r, pids[i].pid);   // valida el "41"+PID
      if (limpio.length() >= 6) {
        rawPID[i] = limpio;
        Serial.printf("<< %s = %s\n", pids[i].nombre, rawPID[i].c_str());
        return true;
      }
      rawPID[i] = "NO_DATA";
      // Una respuesta que no es "sin dato" de la ECU (p.ej. la de otro PID,
      // "?" o "STOPPED") indica desincronización: re-sincronizar.
      if (!esRespuestaSinECU(r)) elmResync();
    } else {
      rawPID[i] = "TIMEOUT";
      elmResync();
    }
    esperarMs(ELM_GAP_MS);
  }
  // Un PID que el vehículo declara NO soportado se informa como "sin dato",
  // nunca como TIMEOUT (no es una falla de lectura).
  if (!soportado) rawPID[i] = "NO_DATA";
  Serial.printf("<< %s = %s%s\n", pids[i].nombre, rawPID[i].c_str(), soportado ? "" : " (no soportado)");
  return false;
}

// Modo 03 (no forma parte del criterio de "completo").
void obdPollDTC()
{
  rawDTC = "";
  for (uint8_t k = 0; k < 2; k++) {
    String r;
    ElmRes e = elmComando("03", ELM_T_DTC_MS, r);
    if (e == ELM_SIN_ENLACE) return;
    if (e == ELM_OK) { rawDTC = limpiarDTC(r); break; }
    elmResync();
  }
  Serial.printf("[AHORRO] DTC = %s\n", rawDTC.c_str());
}

// ---------------- BLE: conexión con el ELM327 ----------------

bool bleConectarELM()
{
  blePermitido    = true;
  g_elmEncontrado = false;

  // Estado limpio: detener un escaneo que haya quedado pausado, cancelar una
  // conexión pendiente y soltar un enlace viejo.
  Bluefruit.Scanner.stop();
  sd_ble_gap_connect_cancel();
  if (g_bleConectado) bleDesconectar();

  Serial.println("[AHORRO] BLE ON: buscando ELM327...");
  if (!Bluefruit.Scanner.start((uint16_t)(BLE_T_ESCANEO_MS / 10))) {   // unidad: 10 ms
    logDebug("BLE: Scanner.start() rechazado");
    return false;
  }
  uint32_t t0 = millis();
  while (!g_elmEncontrado && millis() - t0 < BLE_T_ESCANEO_MS) { alimentarWatchdog(); delay(20); }
  Bluefruit.Scanner.stop();
  if (!g_elmEncontrado) {
    logDebug("BLE: ELM327 no encontrado en " + String(BLE_T_ESCANEO_MS / 1000) + " s");
    return false;
  }

  g_bleConectado = false;
  if (!Bluefruit.Central.connect(&g_elmAddr)) {
    logDebug("BLE: connect() rechazado");
    return false;
  }
  t0 = millis();
  while (!g_bleConectado && millis() - t0 < BLE_T_CONEXION_MS) { alimentarWatchdog(); delay(20); }
  if (!g_bleConectado) {
    sd_ble_gap_connect_cancel();
    logDebug("BLE: timeout de conexión");
    return false;
  }

  // Discovery (bloqueante): acá en el loop, no en el callback.
  if (!elmService.discover(g_connHandle) || !elmChar.discover()) {
    logDebug("BLE: servicio FFF0/FFF1 no encontrado");
    bleDesconectar();
    return false;
  }
  if (!elmChar.enableNotify()) {
    logDebug("BLE: enableNotify falló");
    bleDesconectar();
    return false;
  }
  if (!elmInicializar()) {
    bleDesconectar();
    return false;
  }
  elmInicializado = true;
  digitalWrite(LED_BLUE, HIGH);
  Serial.println("ELM327 listo");
  return true;
}

void bleDesconectar()
{
  elmInicializado = false;
  uint16_t h = g_connHandle;
  if (g_bleConectado && h != BLE_CONN_HANDLE_INVALID) {
    Bluefruit.disconnect(h);
    uint32_t t0 = millis();
    while (g_bleConectado && millis() - t0 < 2000) { alimentarWatchdog(); delay(20); }
  }
  digitalWrite(LED_BLUE, LOW);
}

// Una sesión completa: conectar, init, soporte, PIDs y DTC.
ObdRes obdSesion()
{
  if (!bleConectarELM()) return OBD_FALLA_BLE;

  int8_t s = obdLeerSoporte();
  if (s < 0) return OBD_FALLA_BLE;
  if (s == 0) return OBD_ECU_NO_RESPONDE;

  bool enlaceCaido = false;
  for (uint8_t i = 0; i < NUM_PIDS && !enlaceCaido; i++) {
    alimentarWatchdog();
    obdPollPID(i, enlaceCaido);
    esperarMs(ELM_GAP_MS);
  }
  if (enlaceCaido) {
    logDebug("OBD: se cayó el enlace BLE en medio de la lectura");
    return OBD_FALLA_BLE;
  }
  obdPollDTC();

  String faltan = "";
  for (uint8_t i = 0; i < NUM_PIDS; i++) {
    if (!pidValido(i) && pidSoportado(pids[i].pid)) {
      faltan += pids[i].nombre;
      faltan += " ";
    }
  }
  if (faltan.length() > 0) {
    logDebug("OBD incompleto, faltan: " + faltan);
    return OBD_PARCIAL;
  }
  return OBD_COMPLETO;
}

// Lee el OBD con hasta OBD_MAX_SESIONES sesiones. true = completo.
bool obdLeerCompleto()
{
  for (uint8_t i = 0; i < NUM_PIDS; i++) rawPID[i] = "TIMEOUT";
  rawDTC = "";

  bool completo = false;
  for (uint8_t s = 1; s <= OBD_MAX_SESIONES; s++) {
    ObdRes r = obdSesion();
    if (r == OBD_COMPLETO) { completo = true; break; }
    const char* motivo = (r == OBD_PARCIAL) ? "parcial" :
                         (r == OBD_ECU_NO_RESPONDE) ? "ECU no responde" : "falla BLE/ELM";
    logDebug("OBD sesión " + String(s) + "/" + String(OBD_MAX_SESIONES) + ": " + motivo);
    if (r == OBD_ECU_NO_RESPONDE) break;       // motor apagado: no insistir
    if (s < OBD_MAX_SESIONES) {
      bleDesconectar();
      esperarMs(1500);                         // que el ELM vuelva a anunciarse
    }
  }
  ahorroApagarBLE();
  return completo;
}

// ---- BLE: apagar (detener scanner + cancelar + desconectar) ----
void ahorroApagarBLE()
{
  blePermitido    = false;   // cerrar la ventana BLE antes de desconectar
  bleConectando   = false;
  g_elmEncontrado = false;
  Bluefruit.Scanner.stop();
  sd_ble_gap_connect_cancel();
  bleDesconectar();
  digitalWrite(LED_BLUE, LOW);
  Serial.println("[AHORRO] BLE OFF.");
}

// ---------------- GNSS del ciclo ----------------
// Espera un fix hasta g_tGnssRef + GNSS_FIX_TIMEOUT_MS. Con GNSS_USAR_AGPS = 1
// pide AGPS una vez (con el receptor operativo y la red lista) si todavía no
// hay fix.
void gnssObtenerFix()
{
#if !DIAG_CON_GNSS
  rawGNSS = "GNSS_OFF";
#else
  if (!gnssOn && !gnssEncender()) { logDebug("GNSS no disponible: sin fix"); return; }
  uint32_t deadline = g_tGnssRef + GNSS_FIX_TIMEOUT_MS;
  uint32_t t0 = millis();
  String agps = "no";
  uint8_t reiniciosVistos = g_reiniciosCiclo;
  bool extendido = false;

  while (!g_fixCiclo) {
    if (g_reiniciosCiclo != reiniciosVistos) {
      // El módem se reinició en plena espera y el GNSS quedó apagado.
      // pedirGNSSCrudo() lo vuelve a encender; se le da una ventana mínima.
      reiniciosVistos = g_reiniciosCiclo;
      logDebug("GNSS: el módem se reinició durante la espera -> se re-enciende el GNSS");
      if (!extendido) {
        uint32_t minimo = millis() + GNSS_FIX_TIMEOUT_MS / 2;
        if ((int32_t)(minimo - deadline) > 0) deadline = minimo;
        extendido = true;
      }
    }
    pedirGNSSCrudo(true);                        // (re-enciende el GNSS si hace falta)
    if (g_fixCiclo) break;
#if GNSS_USAR_AGPS
    if (!g_agpsCiclo && g_gnssReady && simListo) {
      g_agpsCiclo = true;
      String r = simSend("AT+CAGPS", 12000, "+AGPS:");
      agps = limpiarResp(r);
      continue;                                  // leer enseguida tras inyectar
    }
#endif
    if ((int32_t)(millis() - deadline) >= 0) break;
    esperarMs(1000);
  }
  logDebug("GNSS " + String(g_fixCiclo ? "FIX" : "NO_FIX") +
           " (espera " + String((millis() - t0) / 1000) + " s, desde encendido " +
           String((millis() - g_tGnssRef) / 1000) + " s) [" + gnssDiag + "] agps=" + agps +
           " reinicios_modem=" + String(g_reiniciosCiclo));
#endif
}

// ---------------- MÓDEM: despertar / preparar / dormir ----------------

bool ahorroDespertarModem()
{
#if (AHORRO_MODO_MODEM == 1)
  // Modo 1 (se conserva para desarrollo futuro): salir del sleep por DTR.
  // Si el módem no responde, se recupera por hardware.
  digitalWrite(PIN_SLEEP_7670, LOW);
  esperarMs(200);
  if (modemResponde(3, 500)) {
    simSend("AT+CSCLK=0", 1000, "OK");         // sin sleep durante el ciclo
  } else {
    logDebug("AHORRO wake: el DTR no despertó al módem -> recuperación por HW");
    if (!modemEncender()) return false;
  }
#else
  // Modo 2: enciende el módem (confirmado por AT). Modo 0: ya responde.
  if (!modemEncender()) {
    logDebug("AHORRO wake: módem no responde");
    return false;
  }
#endif

#if (AHORRO_MODO_MODEM != 2)
  // Modos 0 y 1: si el ciclo anterior no pudo transmitir, un CRESET limpia
  // la pila TCP/MQTT. En modo 2 no hace falta: cada ciclo es un arranque.
  if (g_ultimoCicloFallo && !g_cresetCiclo) {
    g_cresetCiclo = true;
    logDebug("AHORRO wake: CRESET por ciclo previo fallido");
    simSend("AT+CRESET", 3000, "OK");
    modemMarcarReiniciado();
    esperarMs(3000);
    if (!modemEsperarListo(MODEM_T_BOOT_MS)) {
      logDebug("AHORRO wake: módem no responde tras CRESET");
      return false;
    }
  }
#endif
  return true;
}

// Módem despierto + GNSS encendido lo antes posible + red lista.
bool modemPrepararRed()
{
  if (!ahorroDespertarModem()) return false;
#if DIAG_CON_GNSS
  gnssEncender();   // el receptor adquiere mientras se registra la red
#endif
  if (!iniciarRed()) return false;
  simListo = true;
  return true;
}

void ahorroDormirModem()
{
#if (AHORRO_MODO_MODEM == 0)
  // Modo 0 (se conserva para desarrollo futuro): se cierra sólo el socket
  // ante el broker; el servicio MQTT y el PDP se mantienen.
  if (mqttOnline) simSend("AT+CMQTTDISC=0,60", 8000, "+CMQTTDISC:");
  mqttOnline = false;
  #if !GNSS_MANTENER_ON
    gnssApagar(true);
  #endif
  Serial.println("[AHORRO] socket MQTT cerrado (servicio y PDP se mantienen).");
#elif (AHORRO_MODO_MODEM == 1)
  // Modo 1 (se conserva para desarrollo futuro): igual que el modo 0, más
  // sleep por DTR.
  if (mqttOnline) simSend("AT+CMQTTDISC=0,60", 8000, "+CMQTTDISC:");
  mqttOnline = false;
  #if !GNSS_MANTENER_ON
    gnssApagar(true);
  #endif
  simSend("AT+CSCLK=1", 1000, "OK");       // habilitar sleep por DTR
  digitalWrite(PIN_SLEEP_7670, HIGH);      // DTR alto = dejar dormir
  simListo = false;
  Serial.println("[AHORRO] módem en sleep por DTR.");
#else
  // Modo 2: avisar al broker, guardar efemérides y apagar el módem con
  // confirmación. Si ya estaba apagado (ciclo sin TX), modemApagar() sólo
  // lo verifica.
  if (mqttSvcUp || mqttOnline) teardownMQTT();
  modemApagar();
  Serial.println("[AHORRO] módem apagado.");
#endif
}

// ---- Un ciclo completo de ahorro ----
void cicloAhorro()
{
  static uint32_t nCiclo = 0;
  static uint8_t  ciclosFallidos = 0;
  nCiclo++;
  uint32_t tIni = millis();
  Serial.println("\n===== [AHORRO] Inicio de ciclo " + String(nCiclo) + " =====");
  alimentarWatchdog();

  // Estado por ciclo
  g_cresetCiclo   = false;
  g_gnssRefFijada = false;
  g_fixCiclo      = false;
  g_agpsCiclo     = false;
  g_reiniciosCiclo = 0;
  g_autoOnCiclo   = false;
  rawGNSS         = DIAG_CON_GNSS ? "NO_FIX" : "GNSS_OFF";

  // 1) Batería/carga (barato, con todo aún dormido).
  voltajeBateria = leerNivelBateria();
  leerEstadoCarga();
  actualizarLEDCarga();

  // 2) OBD por BLE: todo o nada.
#if DIAG_SOLO_MQTT
  for (uint8_t i = 0; i < NUM_PIDS; i++) rawPID[i] = "TIMEOUT";
  rawDTC = "";
  bool obdOK = true;          // diagnóstico: se publica sin OBD
#else
  bool obdOK = obdLeerCompleto();
#endif

  // 3) IMU (barato).
  leerIMU();

  // 4) Transmisión: sólo con OBD completo (o si la compuerta está apagada).
  bool intentarTX = obdOK || !PUBLICAR_SOLO_OBD_COMPLETO;
  bool txOK = false;
  bool modemUsado = false;
  if (intentarTX) {
    bool redOK = false;
    bool gnssHecho = false;
    for (uint8_t intento = 1; intento <= AHORRO_MAX_INTENTOS_TX && !txOK; intento++) {
      alimentarWatchdog();
      modemUsado = true;
      Serial.printf("[AHORRO] Intento TX %u/%u\n", (unsigned)intento, (unsigned)AHORRO_MAX_INTENTOS_TX);
      if (!redOK) {
        if (!modemPrepararRed()) {
          logDebug("AHORRO TX intento " + String(intento) + ": red no levantó");
          continue;
        }
        redOK = true;
      }
      if (!gnssHecho) {                        // una sola espera de fix por ciclo
        gnssObtenerFix();
        gnssHecho = true;
      }
      if (!simListo) {
        // El módem se reinició durante la espera del GNSS: verificar el registro
        // y la IP antes de MQTT (tras el reinicio no hay servicio MQTT).
        logDebug("AHORRO: módem reiniciado en el ciclo -> se re-verifica la red");
        if (!iniciarRed()) {
          logDebug("AHORRO TX intento " + String(intento) + ": red no volvió tras el reinicio");
          redOK = false;
          continue;
        }
        simListo = true;
      }
      if (!mqttOnline) {
        if (!iniciarMQTT()) {
          logDebug("AHORRO TX intento " + String(intento) + ": MQTT no conectó");
          redOK = false;                       // el próximo intento re-verifica la red
          continue;
        }
        mqttOnline = true;
      }
      String json = construirJSONraw();
      Serial.print("JSON: "); Serial.println(json);
      publicarMQTT(json);                      // pone mqttOnline=false si falla
      if (mqttOnline) {
        txOK = true;
      } else {
        logDebug("AHORRO TX intento " + String(intento) + ": publish falló");
        teardownMQTT();
        redOK = false;
      }
    }
  } else {
    logDebug("AHORRO: OBD incompleto -> este ciclo NO se publica");
  }

  // 5) SD (valores traducidos): se guarda siempre, haya o no TX.
  if (sdLista) guardarCSVtraducido();

  // 6) Módem a reposo. En modo 2 siempre (confirma que quede apagado).
#if (AHORRO_MODO_MODEM == 2)
  (void)modemUsado;
  ahorroDormirModem();
#else
  if (modemUsado) ahorroDormirModem();
#endif

  // 7) Contador de ciclos fallidos: sólo cuentan los que intentaron transmitir.
  if (intentarTX) {
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
  }

  logDebug("CICLO " + String(nCiclo) + " fin: obd=" + String(obdOK ? 1 : 0) +
           " tx=" + String(txOK ? 1 : 0) + " fix=" + String(g_fixCiclo ? 1 : 0) +
           " reinicios_modem=" + String(g_reiniciosCiclo) +
           " auto_on=" + String(g_autoOnCiclo ? 1 : 0) +
           " dur=" + String((millis() - tIni) / 1000) + " s vbat=" + String(voltajeBateria, 2));
  Serial.printf("===== [AHORRO] Fin de ciclo (txOK=%d) =====\n", txOK ? 1 : 0);
}

#endif   // MODO_AHORRO
