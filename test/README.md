# Pruebas del sistema

Registros de las pruebas hechas durante el desarrollo, como constancia del funcionamiento de cada versión del firmware. Las pruebas de funcionamiento se hicieron con el simulador de ECU ([`firmware/CAN_simulator`](../firmware/CAN_simulator)) en lugar de un vehículo, publicando en el broker MQTT de la facultad y traduciendo los datos en Node-RED.

Los números de revisión (Rev 4, Rev 5, Rev 6) corresponden al historial de versiones del encabezado de [`monitoreo_version_final.ino`](../firmware/nrf52840/monitoreo_version_final/monitoreo_version_final.ino).

## Estructura

```
test/
├── prueba_final/                   Prueba de la versión final
├── mediciones_corriente_24-09/     Mediciones de consumo de corriente
└── pruebas_anteriores/             Pruebas de versiones previas, ordenadas por fecha
    ├── 2026-03-30_primer_logger/
    ├── 2026-04-07_15_pids_y_gnss/
    ├── 2026-04-29_prueba_con_sd/
    ├── 2026-09-10_sin_gestion/
    ├── 2026-09-13_dtc/
    ├── 2026-09-14_reporte_texto/
    ├── 2026-09-22_rev4_timeouts/
    ├── 2026-09-27_rev5_15_ciclos/
    └── historial_debug_sd/
```

## Tipos de archivo

| Archivo | Origen |
|---|---|
| `mqttx.json` | Exportación del cliente MQTTX: configuración de la conexión y mensajes recibidos en `prueba_in`, `prueba_out` y `prueba_out_texto` |
| `debug.log` | Registro de eventos de la tarjeta SD del equipo (`/debug.log`) |
| `trad.csv` | Valores traducidos guardados en la tarjeta SD (`/trad.csv`) |
| `monitor_serie.txt` | Salida del monitor serie del IDE de Arduino |
| `notas.txt` | Notas tomadas durante la prueba |

El formato de los mensajes MQTT, de `debug.log` y de `trad.csv` está descrito en [`firmware/README.md`](../firmware/README.md). En `debug.log`, el primer número de cada línea es el tiempo desde el arranque del equipo, en ms.

---

## Prueba final

**`prueba_final/`**: prueba de la versión final del firmware (`monitoreo_version_final.ino`, mismo código que la Rev 6), hecha el 27/09/2026. Incluye la exportación del debug de Node-RED con los mensajes de `prueba_in`, `prueba_out` y `prueba_out_texto`.

## Mediciones de corriente

**`mediciones_corriente_24-09/`**: mediciones del consumo de corriente del equipo, hechas el 24/09/2026.

---

## Pruebas anteriores

### 2026-03-30_primer_logger

Registro más antiguo, con los sketches de prueba iniciales ("TEST 4G + MQTT" y "OBD + GPS MQTT Logger"). La fecha se tomó de la respuesta del GNSS.

- Al principio el módulo no responde a `AT`; con el código corregido se registra en la red 4G y conecta al broker MQTT.
- El logger lee un solo PID (RPM) y la posición GNSS cada 2,5 s aproximadamente.
- La primera respuesta del ELM327 llega mezclada con las respuestas de su inicialización (`OKELM327 v2.1OKOKOK`).
- Después de una falla de publicación, la reconexión da ERROR en `AT+CMQTTACCQ`.

### 2026-04-07_15_pids_y_gnss

Logger con los 15 PIDs, con los nombres de esa etapa (`Vel_kmh`, `Temp_Motor`, etc.).

- Un ciclo completo cada 35 s aproximadamente, con posición GNSS.
- En el primer ciclo después de conectarse al ELM327, las respuestas quedan desfasadas: cada PID recibe la respuesta del anterior, porque el ELM327 todavía está respondiendo a la inicialización.
- Después de un reinicio, la reconexión MQTT vuelve a fallar con ERROR en `AT+CMQTTACCQ`.

### 2026-04-29_prueba_con_sd

Prueba con la tarjeta SD y con Node-RED traduciendo en el servidor. Es el primero de estos registros que usa la SD. `notas.txt` tiene los mensajes observados y las desconexiones y reconexiones hechas durante la prueba. `monitor_serie.txt` es la salida de la misma sesión, entre las 12:34 y las 12:37.

- Se repite el desfasaje de PIDs en el primer ciclo (`SEARCHING...` y `OK` en lugar de datos).
- El parser de Node-RED traducía sólo RPM y los datos del GNSS, con errores en la hora y en la cantidad de satélites.
- Al reconectar la placa, MQTT no vuelve a conectar (ERROR en `AT+CMQTTACCQ`).

### 2026-09-10_sin_gestion

Versión sin gestión de energía, todavía sin lectura de DTC, alimentada por la batería (alrededor de 3,8 V).

- Unos 70 minutos de funcionamiento: 70 mensajes en MQTTX, uno con TIMEOUT.
- En `debug.log` hay una publicación cada 20 s aproximadamente, con fallas periódicas en `AT+CMQTTTOPIC`, reinicios del módem y reconexiones.
- Al final, un bucle de "MQTT @START ERROR" termina en un reinicio por liveness.
- `trad.csv` tiene las filas de la SD de la misma prueba: 127 ciclos de tres arranques, con los 15 PIDs y la posición GNSS. Los valores de `millis` coinciden con los de `debug.log`.

### 2026-09-13_dtc

Versión sin gestión de energía, ya con lectura de DTC (Modo 03). Es la primera prueba registrada con DTC.

- `mqttx.json` tiene 6 ciclos. En `prueba_out` aparecen los códigos P0100 y U1000 decodificados, con su descripción.
- `trad.csv` tiene las filas de la SD de la misma prueba: 8 ciclos, con los 15 PIDs y las columnas `dtc` y `dtc_alerta`.
- En el primer ciclo, RPM llegó en TIMEOUT (`ERR` en la SD).

### 2026-09-14_reporte_texto

Versión sin gestión de energía, con lectura de DTC. Mensajes del 13/09 (desde las 18 h) y del 14/09 (de 10 h a 11:32 h).

- 146 ciclos, uno cada 20 s aproximadamente, con varios arranques del equipo en el medio: 7 mensajes con algún PID en TIMEOUT y 9 con posición GNSS.
- A partir de las 11:06 del 14/09 aparece el tópico `prueba_out_texto`. Es la primera prueba registrada con el reporte de texto de Node-RED.
- Los DTC informados cambian a lo largo de la prueba.

### 2026-09-22_rev4_timeouts

Rev 4, versión con gestión de energía. Mensajes del 22, 23 y 26/09.

- 84 mensajes en `prueba_in`, 26 de ellos con algún PID en TIMEOUT.
- El 26/09 aparece la alternancia de un mensaje completo y otro con TIMEOUT. Este problema motivó la Rev 5.

### 2026-09-27_rev5_15_ciclos

Rev 5, con el módem apagado entre ciclos (modo 2) y un ciclo cada 200 s.

- Los 15 mensajes llegaron completos.
- En 12 de los 15 ciclos el módem se reinició durante `AT+CAGPS` (aparece `*ATREADY` en `debug.log`) y el GNSS quedó apagado el resto del ciclo. Los dos fix de la prueba llegaron en ciclos en que el AGPS falló (error 106).
- En el ciclo 8 falló la primera conexión BLE y la segunda sesión completó la lectura.
- Estos resultados motivaron la Rev 6: AGPS desactivado y recuperación ante reinicios del módem.

### historial_debug_sd

Registro acumulado de la tarjeta SD de varias sesiones anteriores a la Rev 4, sin fecha exacta. Incluye sesiones de la versión sin gestión y las primeras pruebas del ciclo de ahorro (líneas `DORMIR` y `DESPERTAR`).

- Fallas en `AT+CMQTTTOPIC` seguidas de bucles de "MQTT @START ERROR" que terminaban en un reinicio por liveness. Se explica porque el cierre de MQTT usaba `AT+CMQTTRELCLIENT`, que no existe en el A76XX: sin liberar el cliente, el servicio no se detenía. Se corrigió en la Rev 5.
- Reinicios del módem y sesiones sin señal.
