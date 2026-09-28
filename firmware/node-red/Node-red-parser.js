/*------------------------------------------------------------------------------
-- Project : Sistema de monitoreo de fallas y datos de manejo de vehículos
-------------------------------------------------------------------------------
-- File : Node-red-parser.js
-- Author : Preves, Santiago.
-- Date : Sep 27, 2026.
-- Rev 13 : Código ordenado y estandarizado.
--
-------------------------------------------------------------------------------
-- Description:
  Nodo "function" de Node-RED que traduce el JSON crudo publicado por el
  equipo a magnitudes físicas.

  Entrada : tópico "prueba_in"  (JSON con tramas OBD-II en hexadecimal,
            línea de AT+CGNSSINFO, IMU, batería y respuesta del Modo 03).
  Salidas : tópico "prueba_out"        -> JSON con los valores traducidos.
            tópico "prueba_out_texto"  -> reporte de texto legible.

  El nodo "mqtt out" debe tener el tópico vacío para que use msg.topic.
--
-------------------------------------------------------------------------------*/


// =============================================================================
//  CONSTANTES
// =============================================================================
const M_S2_POR_G        = 9.80665;      // 1 g expresado en m/s2
const GRADOS_POR_RADIAN = 180 / Math.PI;
const KMH_POR_NUDO      = 1.852;
const MINUTOS_UTC_A_ARG = -180;         // hora argentina = UTC - 3 h

// Valores que el equipo envía cuando un PID no tiene dato válido
const VALORES_SIN_DATO = new Set(["", "TIMEOUT", "NO_DATA"]);

// Valores del campo "gnss" que indican que no hay posición
const GNSS_SIN_POSICION = new Set(["", "NO_FIX", "GNSS_OFF"]);

// Tabla de PIDs del Modo 01 (fórmulas de la norma SAE J1979).
//   Clave : nombre del campo en el JSON de entrada (mismo orden que el firmware).
//   pid   : número de PID, para validar que la trama empiece con "41" + PID.
//   clave : nombre del campo en el JSON de salida.
//   f     : fórmula con los bytes de datos A y B.
const TABLA_OBD = {
    RPM:         { pid: 0x0C, clave: "rpm",            f: (A, B) => Math.round(((A * 256 + B) / 4) * 10) / 10 },
    vel_kph:     { pid: 0x0D, clave: "vel_obd_kph",    f: (A, B) => A },
    temp_mot_c:  { pid: 0x05, clave: "temp_motor_c",   f: (A, B) => A - 40 },
    temp_adm_c:  { pid: 0x0F, clave: "temp_adm_c",     f: (A, B) => A - 40 },
    carga_pct:   { pid: 0x04, clave: "carga_mot_pct",  f: (A, B) => redondear(A * 100 / 255, 1) },
    comb_pct:    { pid: 0x2F, clave: "nivel_comb_pct", f: (A, B) => redondear(A * 100 / 255, 1) },
    accel_pct:   { pid: 0x11, clave: "acelerador_pct", f: (A, B) => redondear(A * 100 / 255, 1) },
    map_kpa:     { pid: 0x0B, clave: "pres_map_kpa",   f: (A, B) => A },
    maf_gs:      { pid: 0x10, clave: "maf_gs",         f: (A, B) => redondear((A * 256 + B) / 100, 2) },
    pcomb_kpa:   { pid: 0x0A, clave: "pres_comb_kpa",  f: (A, B) => A * 3 },
    avance_deg:  { pid: 0x0E, clave: "avance_enc_deg", f: (A, B) => redondear(A / 2 - 64, 1) },
    ton_s:       { pid: 0x1F, clave: "tiempo_motor_s", f: (A, B) => A * 256 + B },
    dist_mil_km: { pid: 0x21, clave: "dist_mil_km",    f: (A, B) => A * 256 + B },
    temp_ace_c:  { pid: 0x5C, clave: "temp_aceite_c",  f: (A, B) => A - 40 },
    cons_lh:     { pid: 0x5E, clave: "consumo_lh",     f: (A, B) => redondear((A * 256 + B) / 20, 2) },
};

// Códigos de falla (DTC)
const LETRA_DTC = ["P", "C", "B", "U"];

const CATEGORIA_DTC = {
    P: "Motor",
    C: "Chasis",
    B: "Carroceria",
    U: "Red/Comunicacion",
};

const DESCRIPCION_DTC = {
    "P0100": "Sensor de flujo de aire (MAF)",
    "P0101": "Sensor MAF - rango/rendimiento",
    "P0110": "Sensor de temperatura de admision",
    "P0115": "Sensor de temperatura de refrigerante",
    "P0120": "Sensor de posicion del acelerador",
    "P0128": "Termostato de refrigerante",
    "P0171": "Mezcla demasiado pobre",
    "P0172": "Mezcla demasiado rica",
    "P0300": "Fallo de encendido en varios cilindros",
    "P0301": "Fallo de encendido cilindro 1",
    "P0302": "Fallo de encendido cilindro 2",
    "P0303": "Fallo de encendido cilindro 3",
    "P0304": "Fallo de encendido cilindro 4",
    "P0420": "Eficiencia del catalizador baja",
    "P0442": "Fuga pequena en sistema EVAP",
    "P0500": "Sensor de velocidad del vehiculo",
    "C0035": "Sensor de velocidad rueda del. izq.",
    "C0110": "Motor de bomba de ABS",
    "B0010": "Airbag del conductor",
    "B1318": "Tension de bateria baja",
    "U0100": "Perdida de comunicacion con la ECU",
    "U0121": "Perdida de comunicacion con ABS",
    "U1000": "Comunicacion en red CAN",
};

// Descripción por categoría, para los códigos que no están en la tabla anterior
const DESCRIPCION_GENERICA_DTC = {
    P: "Falla del motor / tren motriz",
    C: "Falla del chasis (frenos/suspension/direccion)",
    B: "Falla de carroceria / confort",
    U: "Falla de red / comunicacion",
};


// =============================================================================
//  FUNCIONES AUXILIARES
// =============================================================================

// Redondea 'valor' a la cantidad de decimales indicada.
function redondear(valor, decimales) {
    return parseFloat(valor.toFixed(decimales));
}

// Completa con un cero a la izquierda: 7 -> "07".
function dosDigitos(n) {
    return n.toString().padStart(2, "0");
}

// true si el texto es un número decimal sin signo (ej. "38.008305").
function esCoordenada(texto) {
    return /^\d+(\.\d+)?$/.test(texto || "");
}

// Ubica los campos de la línea de AT+CGNSSINFO a partir de los indicadores de
// hemisferio (N/S y E/W), igual que el firmware. Así no depende de cuántos
// contadores de satélites informe el módulo antes de la latitud.
// Devuelve null si la línea no tiene coordenadas válidas.
function extraerCamposGNSS(linea) {
    let c = linea.split(",").map(campo => campo.trim());
    for (let k = 1; k + 2 < c.length; k++) {
        let hemisferios = (c[k] === "N" || c[k] === "S") && (c[k + 2] === "E" || c[k + 2] === "W");
        if (hemisferios && esCoordenada(c[k - 1]) && esCoordenada(c[k + 1])) {
            return {
                lat: c[k - 1], latDir: c[k], lon: c[k + 1], lonDir: c[k + 2],
                fecha: c[k + 3], hora: c[k + 4], alt: c[k + 5], vel: c[k + 6],
                rumbo: c[k + 7], pdop: c[k + 8], hdop: c[k + 9], vdop: c[k + 10],
                sats: c[k + 11],
            };
        }
    }
    return null;
}

// Resta un día a una fecha "dd-mm-aa".
function restarUnDia(fecha) {
    let p = fecha.split("-");
    let d = new Date(parseInt("20" + p[2]), parseInt(p[1]) - 1, parseInt(p[0]));
    d.setDate(d.getDate() - 1);
    return dosDigitos(d.getDate()) + "-" + dosDigitos(d.getMonth() + 1) + "-" +
           d.getFullYear().toString().substring(2);
}

// Convierte la hora UTC del GNSS (hhmmss.ss) a hora argentina y la guarda en
// salida.hora. Si al restar 3 h se pasa al día anterior, corrige salida.fecha.
function convertirHoraArgentina(salida, horaUTC) {
    let partes = (horaUTC || "").split(".");
    let entero = (partes[0] || "").replace(/\D/g, "");
    let hh = "00", mm = "00", ss = "00";

    if (entero.length === 6) {                // hhmmss
        hh = entero.substring(0, 2);
        mm = entero.substring(2, 4);
        ss = entero.substring(4, 6);
    } else if (entero.length === 4) {         // hhmm.mmmm (minutos con decimales)
        hh = entero.substring(0, 2);
        mm = entero.substring(2, 4);
        let fraccion = partes[1] ? parseFloat("0." + partes[1].replace(/\D/g, "")) : 0;
        ss = dosDigitos(Math.round(fraccion * 60));
    } else {
        node.warn("GNSS: formato de hora inesperado -> " + horaUTC);
    }

    let minutos = parseInt(hh) * 60 + parseInt(mm) + MINUTOS_UTC_A_ARG;
    if (minutos < 0) {
        minutos += 24 * 60;
        if (salida.fecha) salida.fecha = restarUnDia(salida.fecha);
    }
    salida.hora = dosDigitos(Math.floor(minutos / 60)) + ":" + dosDigitos(minutos % 60) + ":" + ss;
}

// Traduce una trama OBD-II. Devuelve null si no es una trama válida "41"+PID
// (por ejemplo "TIMEOUT", "NO_DATA" o una respuesta incompleta).
function traducirPID(definicion, valorCrudo) {
    if (typeof valorCrudo !== "string") return null;
    let hex   = valorCrudo.replace(/\s+/g, "").toUpperCase();
    let marca = "41" + dosDigitos(definicion.pid.toString(16).toUpperCase());
    if (!/^[0-9A-F]+$/.test(hex) || !hex.startsWith(marca) || hex.length < 6) return null;

    let A = parseInt(hex.substring(4, 6), 16);
    let B = hex.length >= 8 ? parseInt(hex.substring(6, 8), 16) : 0;
    return definicion.f(A, B);
}

// Decodifica la respuesta cruda del Modo 03 ("43" + pares de bytes) a una
// lista de códigos, por ejemplo "430133" -> ["P0133"].
function decodificarDTC(bruto) {
    if (typeof bruto !== "string") return [];
    let h = bruto.toUpperCase().replace(/[^0-9A-F]/g, "");
    let i = h.indexOf("43");
    if (i < 0) return [];
    h = h.substring(i + 2);

    let codigos = [];
    for (let k = 0; k + 4 <= h.length; k += 4) {
        let b1 = parseInt(h.substring(k, k + 2), 16);
        let b2 = parseInt(h.substring(k + 2, k + 4), 16);
        if (b1 === 0 && b2 === 0) continue;            // relleno: no es un código
        codigos.push(LETRA_DTC[(b1 >> 6) & 0x03] +
                     ((b1 >> 4) & 0x03).toString() +
                     (b1 & 0x0F).toString(16).toUpperCase() +
                     ((b2 >> 4) & 0x0F).toString(16).toUpperCase() +
                     (b2 & 0x0F).toString(16).toUpperCase());
    }
    return codigos;
}

function categoriaDTC(codigo) {
    return CATEGORIA_DTC[codigo[0]] || "Sistema";
}

function descripcionDTC(codigo) {
    return DESCRIPCION_DTC[codigo] || DESCRIPCION_GENERICA_DTC[codigo[0]] || "Codigo desconocido";
}

// Arma una línea "Etiqueta: valor unidad   Etiqueta: valor unidad" con los
// campos que tengan valor. Cada campo es [etiqueta, valor, unidad].
function lineaDeCampos(campos) {
    return campos
        .filter(([, valor]) => valor !== undefined)
        .map(([etiqueta, valor, unidad]) => etiqueta + ": " + valor + (unidad ? " " + unidad : ""))
        .join("   ");
}


// =============================================================================
//  1. ENTRADA (Buffer, texto u objeto)
// =============================================================================
let entrada;
try {
    let p = msg.payload;
    if (Buffer.isBuffer(p)) p = p.toString("utf8");
    entrada = (typeof p === "string") ? JSON.parse(p) : p;
} catch (e) {
    node.error("JSON invalido: " + e.message, msg);
    return null;
}
if (entrada === null || typeof entrada !== "object" || Array.isArray(entrada)) {
    node.error("El payload no es un objeto JSON (" + typeof entrada + ")", msg);
    return null;
}

let salida = { ts: entrada.ts || 0 };


// =============================================================================
//  2. GNSS (línea de AT+CGNSSINFO)
// =============================================================================
if (typeof entrada.gnss === "string" && !GNSS_SIN_POSICION.has(entrada.gnss)) {
    let g = extraerCamposGNSS(entrada.gnss);
    if (g === null) {
        node.warn("GNSS: linea sin coordenadas validas -> " + entrada.gnss);
    } else {
        // Coordenadas con signo: sur y oeste son negativas
        salida.lat_dir = g.latDir;
        salida.lon_dir = g.lonDir;
        salida.lat = redondear(g.latDir === "S" ? -parseFloat(g.lat) : parseFloat(g.lat), 6);
        salida.lon = redondear(g.lonDir === "W" ? -parseFloat(g.lon) : parseFloat(g.lon), 6);

        // Fecha ddmmaa -> dd-mm-aa y hora UTC -> hora argentina
        let fecha = g.fecha || "";
        if (fecha.length >= 6) {
            salida.fecha = fecha.substring(0, 2) + "-" + fecha.substring(2, 4) + "-" + fecha.substring(4, 6);
        }
        convertirHoraArgentina(salida, g.hora);

        // Campos opcionales (la velocidad llega en nudos)
        if (g.alt   !== undefined) salida.alt_m        = parseFloat(g.alt) || 0;
        if (g.vel   !== undefined) salida.vel_gnss_kph = redondear(parseFloat(g.vel || 0) * KMH_POR_NUDO, 2);
        if (g.rumbo !== undefined) salida.dir_deg      = parseFloat(g.rumbo) || 0;
        if (g.pdop  !== undefined) salida.pdop         = parseFloat(g.pdop) || 0;
        if (g.hdop  !== undefined) salida.hdop         = parseFloat(g.hdop) || 0;
        if (g.vdop  !== undefined) salida.vdop         = parseFloat(g.vdop) || 0;
        if (g.sats  !== undefined) salida.sats_fix     = parseInt(g.sats) || 0;
    }
}


// =============================================================================
//  3. IMU LSM6DS33
// =============================================================================
const CAMPOS_IMU = ["imu_ax", "imu_ay", "imu_az", "imu_gx", "imu_gy", "imu_gz"];

if (CAMPOS_IMU.some(campo => entrada[campo] !== undefined)) {
    // Acelerómetro: m/s2 -> g
    if (entrada.imu_ax !== undefined) salida.accel_x_g = redondear(entrada.imu_ax / M_S2_POR_G, 4);
    if (entrada.imu_ay !== undefined) salida.accel_y_g = redondear(entrada.imu_ay / M_S2_POR_G, 4);
    if (entrada.imu_az !== undefined) salida.accel_z_g = redondear(entrada.imu_az / M_S2_POR_G, 4);

    // Giróscopo: rad/s -> grados/s
    if (entrada.imu_gx !== undefined) salida.gyro_x_dps = redondear(entrada.imu_gx * GRADOS_POR_RADIAN, 4);
    if (entrada.imu_gy !== undefined) salida.gyro_y_dps = redondear(entrada.imu_gy * GRADOS_POR_RADIAN, 4);
    if (entrada.imu_gz !== undefined) salida.gyro_z_dps = redondear(entrada.imu_gz * GRADOS_POR_RADIAN, 4);

    // Módulo de la aceleración total (incluye la gravedad: ~1 g en reposo)
    if (salida.accel_x_g !== undefined && salida.accel_y_g !== undefined && salida.accel_z_g !== undefined) {
        salida.accel_total_g = redondear(
            Math.sqrt(salida.accel_x_g ** 2 + salida.accel_y_g ** 2 + salida.accel_z_g ** 2), 4);
    }

    // Aceleración del vehículo (frenadas y giros): módulo en el plano X-Y.
    // Supone la placa montada con el eje Z hacia arriba, de modo que la
    // gravedad queda sobre Z y el plano X-Y sólo ve la dinámica (~0 en reposo).
    if (entrada.imu_ax !== undefined && entrada.imu_ay !== undefined) {
        let horizontal = Math.sqrt(entrada.imu_ax ** 2 + entrada.imu_ay ** 2);
        salida.accel_horizontal_ms2 = redondear(horizontal, 4);
        salida.accel_horizontal_g   = redondear(horizontal / M_S2_POR_G, 4);
    }
} else {
    node.warn("IMU: el payload no trae ningun campo imu_*");
}


// =============================================================================
//  4. BATERÍA
// =============================================================================
if (entrada.vbat !== undefined) {
    let v = parseFloat(entrada.vbat);
    if (!isNaN(v)) salida.bat_v = redondear(v, 2);
}
if (entrada.cargando !== undefined) salida.cargando       = Boolean(entrada.cargando);
if (entrada.carga_ok !== undefined) salida.carga_completa = Boolean(entrada.carga_ok);

if (salida.carga_completa === true)  salida.estado_carga = "completa";
else if (salida.cargando === true)   salida.estado_carga = "cargando";
else if (salida.cargando === false)  salida.estado_carga = "en_bateria";


// =============================================================================
//  5. CÓDIGOS DE FALLA (Modo 03)
// =============================================================================
if (entrada.dtc !== undefined) {
    let dtcs = decodificarDTC(entrada.dtc);
    salida.dtc          = dtcs;
    salida.dtc_cantidad = dtcs.length;
    salida.dtc_presente = dtcs.length > 0;
    salida.dtc_desc     = dtcs.map(descripcionDTC);
    salida.dtc_alerta   = dtcs.length > 0
        ? dtcs.map(c => categoriaDTC(c) + " en falla: " + descripcionDTC(c) + " [" + c + "]").join(" | ")
        : "sin fallas";
}


// =============================================================================
//  6. PIDs OBD-II (Modo 01)
// =============================================================================
for (let nombre of Object.keys(TABLA_OBD)) {
    let crudo = entrada[nombre];
    if (crudo === undefined) continue;

    let valor = traducirPID(TABLA_OBD[nombre], crudo);
    if (valor !== null) {
        salida[TABLA_OBD[nombre].clave] = valor;
    } else if (!VALORES_SIN_DATO.has(crudo)) {
        node.warn("PID " + nombre + ": trama invalida -> " + crudo);
    }
}


// =============================================================================
//  7. REPORTE DE TEXTO
// =============================================================================
let L = [];
L.push("========== TELEMETRIA  (ts " + (salida.ts || 0) + ") ==========");

L.push("");
L.push("[DTC / FALLAS]");
if (salida.dtc_alerta !== undefined) {
    L.push("  " + salida.dtc_alerta);
    if (salida.dtc_cantidad) L.push("  Cantidad: " + salida.dtc_cantidad);
} else {
    L.push("  (no reportado)");
}

L.push("");
L.push("[GNSS]");
if (salida.lat !== undefined || salida.hora !== undefined) {
    if (salida.fecha !== undefined || salida.hora !== undefined)
        L.push("  Fecha/Hora: " + (salida.fecha || "?") + " " + (salida.hora || "?"));
    if (salida.lat !== undefined && salida.lon !== undefined)
        L.push("  Posicion: " + salida.lat + ", " + salida.lon);
    let extra = lineaDeCampos([
        ["Alt", salida.alt_m, "m"], ["Vel", salida.vel_gnss_kph, "km/h"],
        ["Rumbo", salida.dir_deg, "deg"], ["Sats", salida.sats_fix],
    ]);
    if (extra) L.push("  " + extra);
} else {
    L.push("  (sin fix)");
}

L.push("");
L.push("[OBD / MOTOR]");
let filasOBD = [
    lineaDeCampos([["RPM", salida.rpm], ["Vel", salida.vel_obd_kph, "km/h"]]),
    lineaDeCampos([["Temp motor", salida.temp_motor_c, "C"], ["Temp adm", salida.temp_adm_c, "C"],
                   ["Temp aceite", salida.temp_aceite_c, "C"]]),
    lineaDeCampos([["Carga", salida.carga_mot_pct, "%"], ["Acelerador", salida.acelerador_pct, "%"],
                   ["Combustible", salida.nivel_comb_pct, "%"]]),
    lineaDeCampos([["MAP", salida.pres_map_kpa, "kPa"], ["MAF", salida.maf_gs, "g/s"],
                   ["Pres comb", salida.pres_comb_kpa, "kPa"], ["Avance", salida.avance_enc_deg, "deg"]]),
    lineaDeCampos([["Consumo", salida.consumo_lh, "L/h"], ["Tiempo motor", salida.tiempo_motor_s, "s"],
                   ["Dist MIL", salida.dist_mil_km, "km"]]),
].filter(fila => fila.length > 0);
if (filasOBD.length > 0) filasOBD.forEach(fila => L.push("  " + fila));
else L.push("  (sin datos)");

L.push("");
L.push("[IMU]");
let hayIMU = false;
if (salida.accel_x_g !== undefined) {
    L.push("  Accel (g): x=" + salida.accel_x_g + " y=" + salida.accel_y_g + " z=" + salida.accel_z_g +
           (salida.accel_total_g !== undefined ? "   |total|=" + salida.accel_total_g : ""));
    hayIMU = true;
}
if (salida.accel_horizontal_ms2 !== undefined) {
    L.push("  Accel horizontal: " + salida.accel_horizontal_ms2 + " m/s2 (" + salida.accel_horizontal_g + " g)");
    hayIMU = true;
}
if (salida.gyro_x_dps !== undefined) {
    L.push("  Giro (deg/s): x=" + salida.gyro_x_dps + " y=" + salida.gyro_y_dps + " z=" + salida.gyro_z_dps);
    hayIMU = true;
}
if (!hayIMU) L.push("  (sin datos)");

L.push("");
L.push("[BATERIA]");
let bateria = lineaDeCampos([["Tension", salida.bat_v, "V"], ["Estado", salida.estado_carga]]);
L.push("  " + (bateria || "(sin datos)"));


// =============================================================================
//  8. ENVÍO
// =============================================================================
// El nodo tiene una sola salida: con node.send() se emiten los dos mensajes por
// ella y el nodo "mqtt out" publica cada uno en su msg.topic.
node.send(Object.assign({}, msg, { payload: JSON.stringify(salida), topic: "prueba_out" }));
node.send(Object.assign({}, msg, { payload: L.join("\n"),           topic: "prueba_out_texto" }));
return null;
