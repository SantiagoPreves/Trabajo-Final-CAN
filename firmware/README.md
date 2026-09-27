# Código del sistema de monitoreo de fallas y datos de manejo de vehículos

Este directorio reúne el software de los tres componentes del sistema:

- **nrf52840**: firmware del equipo embarcado. Lee los datos del vehículo por OBD-II, la posición GNSS, la IMU y la batería. Los transmite por 4G/MQTT y los guarda en una tarjeta SD.
- **CAN_simulator**: simulador de una ECU OBD-II sobre bus CAN, usado para las pruebas de banco.
- **node-red**: función de Node-RED que traduce los datos recibidos a magnitudes físicas y genera un reporte legible.

## Arquitectura

```
+----------------+  CAN  +--------+  BLE  +---------------------------+
| ECU (vehículo  |<----->| ELM327 |<----->| Feather nRF52840 Sense    |
| o simulador)   |       +--------+       | A7670SA, SD, IMU, batería |
+----------------+                        +-------------+-------------+
                                                        | 4G / MQTT
                                                        | tópico prueba_in
                                                        v
                                                 +-------------+
                                                 | Broker MQTT |
                                                 +------+------+
                                                        |
                                                        v
                                               +-----------------+
                                               | Node-RED        |
                                               | (parser)        |
                                               +--------+--------+
                                                        |
                                    tópicos prueba_out y prueba_out_texto
```

El sistema sigue el criterio **"MQTT crudo / SD traducido"**:

- Por MQTT se envían las tramas OBD-II tal como las devuelve el ELM327 (hexadecimal) y la respuesta del GNSS sin procesar. La decodificación se hace en Node-RED.
- En la tarjeta SD se guardan los valores ya traducidos a magnitudes físicas, como registro local tipo "caja negra".

## Estructura

```
CAN_simulator/
├── CAN_simulator.ino          Simulador de ECU OBD-II (MCP2515)
└── Packet.cpp                 Estructura de una trama CAN
node-red/
└── Node-red-parser.js         Código del nodo "function"
nrf52840/
├── monitoreo_sin_gestion/
│   └── monitoreo_sin_gestion.ino   Primera versión, sin gestión de energía
└── monitoreo_con_gestion/
    └── monitoreo_con_gestion.ino   Versión final, con gestión de energía
```

El IDE de Arduino exige que cada archivo `.ino` esté en una carpeta con su mismo nombre. Por eso cada versión del firmware tiene su propia subcarpeta.

---

## nrf52840: firmware del equipo embarcado

### Hardware

| Componente | Función |
|---|---|
| Adafruit Feather nRF52840 Sense | Microcontrolador. Incluye BLE y la IMU LSM6DS33 |
| Adaptador ELM327 BLE | Interfaz OBD-II (servicio `0xFFF0`, característica `0xFFF1`) |
| SIMCom A7670SA | Módem LTE Cat-1 con GNSS, conectado a `Serial1` a 115200 bit/s |
| Tarjeta SD (SPI) | Registro local de datos y de eventos |
| TP4056 + batería LiPo | Alimentación y carga |

| Pin en el código | Señal (esquemático) | Descripción |
|---|---|---|
| A0 | CHIPSEL_BLUEF | Chip select de la SD |
| A1 | SLEEP_7670 | DTR del A7670SA (modo sleep) |
| A3 | NIVEL_BAT | Lectura ADC del divisor de batería |
| A5 | PWRKEY_7670 | Encendido y apagado del A7670SA |
| 10 | CHRG (D3) | Salida del TP4056, activa en bajo (cargando) |
| 11 | STDBY (D4) | Salida del TP4056, activa en bajo (carga completa) |
| 12 | ACTIVAR_NIVEL (D5) | Habilita el divisor de batería mediante un MOSFET P, activo en bajo |
| TX / RX | Serial1 | UART del A7670SA |

### Versiones

**`monitoreo_sin_gestion`**: primera versión funcional.
- El módem, el GNSS y el enlace BLE permanecen encendidos todo el tiempo.
- Cada 15 s lee los 15 PIDs y los DTC, la posición y la IMU, publica por MQTT y guarda una fila en la SD.
- Si pasan 4 minutos sin poder publicar, se reinicia. El watchdog de hardware (20 s) cubre los cuelgues.

**`monitoreo_con_gestion`**: versión final, pensada para funcionar con batería. Ejecuta un ciclo de *despertar, adquirir, transmitir y dormir* cada `INTERVALO_CICLO`:

1. Mide la batería y el estado de carga.
2. Se conecta al ELM327 por BLE. Lee qué PIDs soporta el vehículo (`01 00`, `01 20`, `01 40`), los 15 PIDs y los DTC (Modo 03). Luego corta la conexión BLE. Si la lectura queda incompleta o falla el enlace, lo intenta una vez más con una conexión nueva.
3. Lee la IMU.
4. Si la lectura OBD está completa:
   - enciende el módem y el GNSS;
   - se registra en la red LTE;
   - espera un fix;
   - publica el mensaje por MQTT.
5. Guarda la fila traducida en la SD. Esto ocurre siempre, se haya transmitido o no.
6. Cierra la sesión MQTT y apaga el módem. Si hubo fix, antes de apagarlo guarda los datos del GNSS en la memoria del módem (AP-Flash), como máximo una vez cada 30 min. Así el próximo arranque del GNSS es más rápido.

Si falla la transmisión en tres ciclos seguidos, el microcontrolador se reinicia.

### Parámetros principales (`monitoreo_con_gestion`)

| Parámetro | Valor | Descripción |
|---|---|---|
| `INTERVALO_CICLO` | 300000 ms | Período entre inicios de ciclo |
| `AHORRO_MODO_MODEM` | 2 | Reposo del módem: 0 = encendido y registrado, 1 = sleep por DTR, 2 = apagado total |
| `PUBLICAR_SOLO_OBD_COMPLETO` | 1 | Sólo publica si se leyeron todos los PIDs que soporta el vehículo |
| `GNSS_FIX_TIMEOUT_MS` | 90000 ms | Espera máxima de fix, desde que se enciende el GNSS |
| `GNSS_USAR_APFLASH` | 1 | Arranque rápido del GNSS con datos guardados en el módem |
| `GNSS_USAR_AGPS` | 0 | Asistencia por red (desactivada: en las pruebas reiniciaba el módem) |
| `PWRKEY_ACTIVO_ALTO` | 0 | Polaridad de PWRKEY: 0 = A5 directo, 1 = con transistor inversor |
| `MQTT_BROKER` / `MQTT_PORT` | `livra-mqtt.fi.mdp.edu.ar` / 1884 | Servidor MQTT |
| `MQTT_TOPIC` | `prueba_in` | Tópico de publicación |
| `APN` | `internet` | APN de la SIM |

Si en `/debug.log` las líneas `CICLO` muestran `auto_on=1`, el módem se vuelve a encender solo después de apagarlo. En ese caso hay que revisar `PWRKEY_ACTIVO_ALTO` contra el circuito.

### Compilación

1. En el IDE de Arduino, agregar en *Preferencias → URLs adicionales de gestor de placas*:
   `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json`
2. Instalar el paquete **Adafruit nRF52** y elegir la placa **Adafruit Feather nRF52840 Sense**. El paquete incluye la librería Bluefruit.
3. Instalar desde el gestor de librerías:
   - **SD** (Arduino)
   - **Adafruit LSM6DS**, que instala además *Adafruit Unified Sensor* y *Adafruit BusIO*.

### Archivos en la tarjeta SD

- **`/trad.csv`**: una fila por ciclo con los valores traducidos. Columnas:
  - tiempo desde el arranque (`millis`);
  - posición (`fecha`, `hora` en UTC, `lat`, `lat_dir`, `lon`, `lon_dir`, `alt_m`, `vel_kph`, `dir_deg`);
  - PIDs;
  - IMU;
  - batería (`vbat`, `cargando`, `carga_completa`);
  - códigos de falla (`dtc`).

  La versión con gestión guarda sólo 5 PIDs: RPM, velocidad, temperatura del motor, carga y acelerador. El resto viaja completo por MQTT. La versión sin gestión guarda los 15 PIDs y una descripción legible de cada falla.
- **`/debug.log`**: eventos con marca de tiempo (arranques y su causa, registro en la red, fix GNSS, publicaciones). En la versión con gestión agrega un resumen por ciclo con este formato:
  ```
  1708215 | CICLO 9 fin: obd=1 tx=1 fix=1 reinicios_modem=0 auto_on=0 dur=98 s vbat=4.14
  ```
  El primer número es el tiempo desde el arranque, en ms. `obd`, `tx` y `fix` indican si el ciclo logró una lectura OBD completa, la transmisión y la posición.

---

## CAN_simulator: simulador de ECU

Simula una ECU OBD-II para probar el sistema sin un vehículo. Responde las consultas que el ELM327 envía al ID funcional `0x7DF` con tramas desde el ID `0x7E8` (CAN a 500 kbit/s, identificadores de 11 bits).

### Hardware y librerías

- Placa Arduino (UNO o compatible) con un módulo **MCP2515** con cristal de 8 MHz:
  - CS en el pin 10;
  - INT en el pin 2.
- El ELM327 se conecta a un conector OBD-II hembra cableado al bus: pin 6 CAN_H, pin 14 CAN_L, pin 16 +12 V, pines 4 y 5 GND.
- Librería **mcp_can** (MCP_CAN_lib, de coryjfowler).
- `Packet.cpp` define la estructura de una trama. Va en la misma carpeta que el `.ino`.

### Qué responde

| Modo | Descripción |
|---|---|
| 01 | Datos actuales. PIDs `00 01 04 05 0A 0B 0C 0D 0E 0F 10 11 1F 20 21 2F 40 5C 5E`, con valores aleatorios en rangos realistas |
| 03 | Lectura de DTC: de 0 a 2 códigos sorteados de un catálogo de 12, que se vuelve a sortear cada 30 s |
| 04 | Borrado de DTC. Los códigos vuelven a aparecer en el sorteo siguiente |

El monitor serie (115200 bit/s) muestra cada trama recibida y enviada en hexadecimal. Por ejemplo, la respuesta a un pedido de RPM:

```
TX 0x7E8 [8] 04 41 0C 2A E4 00 00 00 OK
```

---

## node-red: traducción de los datos

`Node-red-parser.js` es el código de un nodo **function**.

### Flujo

```
[mqtt in: prueba_in] --> [function: Node-red-parser.js] --> [mqtt out: tópico vacío]
```

El nodo *mqtt out* debe tener el tópico vacío. La función envía dos mensajes con `msg.topic` ya asignado:

- **`prueba_out`**: JSON con los valores traducidos.
  - Posición con signo y hora argentina (UTC−3).
  - PIDs según las fórmulas de SAE J1979.
  - Aceleraciones en g y giros en grados por segundo.
  - Estado de la batería.
  - Lista de DTC con su descripción.
- **`prueba_out_texto`**: el mismo contenido como reporte de texto.

Los PIDs que llegan como `TIMEOUT` o `NO_DATA` se omiten.

### Ejemplo de reporte (`prueba_out_texto`)

```
========== TELEMETRIA  (ts 1658863) ==========

[DTC / FALLAS]
  Carroceria en falla: Tension de bateria baja [B1318] | Motor en falla: Eficiencia del catalizador baja [P0420]
  Cantidad: 2

[GNSS]
  Fecha/Hora: 27-09-26 14:11:22
  Posicion: -38.008202, -57.54427
  Alt: 72 m   Vel: 1.33 km/h   Rumbo: 0 deg   Sats: 6

[OBD / MOTOR]
  RPM: 2745   Vel: 114 km/h
  Temp motor: 114 C   Temp adm: 21 C   Temp aceite: 98 C
  Carga: 49.8 %   Acelerador: 61.6 %   Combustible: 34.1 %
  MAP: 90 kPa   MAF: 17.81 g/s   Pres comb: 243 kPa   Avance: -12.5 deg
  Consumo: 13.1 L/h   Tiempo motor: 1723 s   Dist MIL: 42 km

[IMU]
  Accel (g): x=0.0254 y=0.0523 z=1.0136   |total|=1.0153
  Accel horizontal: 0.5705 m/s2 (0.0582 g)
  Giro (deg/s): x=3.7873 y=-4.979 z=-7.4141

[BATERIA]
  Tension: 4.14 V   Estado: cargando
```

---

## Formato del mensaje MQTT (`prueba_in`)

Las dos versiones del firmware publican el mismo formato:

| Campo | Contenido |
|---|---|
| `ts` | Tiempo desde el arranque del equipo, en ms |
| `gnss` | Respuesta de `AT+CGNSSINFO` sin el prefijo. Vale `NO_FIX` si no hay posición, o `GNSS_OFF` si el GNSS está deshabilitado |
| PIDs | Trama cruda `41` + PID + datos, por ejemplo `"410C2AE4"` |
| `imu_ax`, `imu_ay`, `imu_az` | Aceleración en m/s² |
| `imu_gx`, `imu_gy`, `imu_gz` | Velocidad angular en rad/s |
| `vbat` | Tensión de la batería, en V |
| `cargando`, `carga_ok` | Estado del cargador |
| `dtc` | Respuesta cruda al Modo 03 (`43` + códigos) |

Los PIDs que no pudieron leerse se envían como `TIMEOUT` o `NO_DATA`.

| Campo | PID | Magnitud | Fórmula (A, B: bytes de datos) |
|---|---|---|---|
| `RPM` | 0C | Régimen del motor (rpm) | (256·A + B) / 4 |
| `vel_kph` | 0D | Velocidad (km/h) | A |
| `temp_mot_c` | 05 | Temperatura del refrigerante (°C) | A − 40 |
| `temp_adm_c` | 0F | Temperatura de admisión (°C) | A − 40 |
| `carga_pct` | 04 | Carga del motor (%) | 100·A / 255 |
| `comb_pct` | 2F | Nivel de combustible (%) | 100·A / 255 |
| `accel_pct` | 11 | Posición del acelerador (%) | 100·A / 255 |
| `map_kpa` | 0B | Presión del múltiple de admisión (kPa) | A |
| `maf_gs` | 10 | Flujo de aire (g/s) | (256·A + B) / 100 |
| `pcomb_kpa` | 0A | Presión de combustible (kPa) | 3·A |
| `avance_deg` | 0E | Avance de encendido (°) | A / 2 − 64 |
| `ton_s` | 1F | Tiempo desde el arranque del motor (s) | 256·A + B |
| `dist_mil_km` | 21 | Distancia recorrida con la luz MIL encendida (km) | 256·A + B |
| `temp_ace_c` | 5C | Temperatura del aceite (°C) | A − 40 |
| `cons_lh` | 5E | Consumo de combustible (L/h) | (256·A + B) / 20 |

Ejemplo real:

```json
{
  "ts": 1658863,
  "gnss": "3,06,,01,00,38.0082016,S,57.5442696,W,270926,171122.00,72.0,0.719,,2.92,2.41,1.65,06",
  "RPM": "410C2AE4", "vel_kph": "410D72", "temp_mot_c": "41059A", "temp_adm_c": "410F3D",
  "carga_pct": "41047F", "comb_pct": "412F57", "accel_pct": "41119D", "map_kpa": "410B5A",
  "maf_gs": "411006F5", "pcomb_kpa": "410A51", "avance_deg": "410E67", "ton_s": "411F06BB",
  "dist_mil_km": "4121002A", "temp_ace_c": "415C8A", "cons_lh": "415E0106",
  "imu_ax": 0.2489, "imu_ay": 0.5133, "imu_az": 9.9398,
  "imu_gx": 0.0661, "imu_gy": -0.0869, "imu_gz": -0.1294,
  "vbat": 4.14, "cargando": true, "carga_ok": false,
  "dtc": "4393180420"
}
```

---

## Limitaciones conocidas

- En los vehículos con CAN, la respuesta al Modo 03 lleva después del `43` un byte con la cantidad de códigos. El simulador no lo envía y los decodificadores (firmware y Node-RED) están hechos para ese formato. Para usar el sistema en un vehículo real hay que ignorar ese byte.
- La hora guardada en la SD está en UTC. La conversión a hora argentina se hace en Node-RED.

## Referencias

1. SIMCom, *A76XX Series AT Command Manual*, V1.12.
2. SIMCom, *A7670 Series Hardware Design*, V1.06.
3. SIMCom, *A76XX Series GNSS Application Note*.
4. ELM Electronics, *ELM327 OBD to RS232 Interpreter* (hoja de datos).
5. SAE J1979 / ISO 15031-5, servicios y PIDs de diagnóstico OBD-II.
6. Adafruit, *Feather nRF52840 Sense* y *Bluefruit nRF52 Arduino API*.
