/*------------------------------------------------------------------------------
-- Project : Sistema de monitoreo de fallas y datos de manejo de vehículos
-------------------------------------------------------------------------------
-- File : Node-red-parser.js
-- Author : Preves, Santiago.
-- Date : Sep 13, 2026.
-- Rev 13 : Final release.
--
-------------------------------------------------------------------------------
-- Description:
	Codigo de traducción de datos MQTT en Node Red
	Topic entrada : prueba_in   |   Topic salida : prueba_out & prueba_out_texto
--               
-------------------------------------------------------------------------------*/

const G_TO_MS2   = 9.80665;
const RAD_TO_DEG = 180 / Math.PI;


// ════════ PASO 1 — Parsear entrada (Buffer / string / objeto) ════════
let raw;
try {
    let p = msg.payload;
    if (Buffer.isBuffer(p)) p = p.toString("utf8");
    raw = (typeof p === "string") ? JSON.parse(p) : p;
} catch (e) {
    node.error("JSON inválido: " + e.message, msg);
    return null;
}
if (raw === null || typeof raw !== "object" || Array.isArray(raw)) {
    node.error("Payload no es un objeto JSON (" + typeof raw + ")", msg);
    return null;
}


// ════════ PASO 2 — Objeto de salida ════════
let out = { ts: raw.ts || 0 };


// ════════ PASO 3 — GNSS (AT+CGNSSINFO) ════════
if (raw.gnss && raw.gnss !== "NO_FIX" && typeof raw.gnss === "string") {

    let g = raw.gnss.split(",");

    try {
        // ── Coordenadas SIGNADAS (S y W = negativo) ──────────────
        let latNum = parseFloat(g[5]);
        let lonNum = parseFloat(g[7]);
        out.lat_dir = (g[6] || "").trim();
        out.lon_dir = (g[8] || "").trim();
        if (!isNaN(latNum))
            out.lat = parseFloat((out.lat_dir === "S" ? -latNum : latNum).toFixed(6));
        if (!isNaN(lonNum))
            out.lon = parseFloat((out.lon_dir === "W" ? -lonNum : lonNum).toFixed(6));

        // ── Fecha: "200826" -> "20-08-26" ───────────────────────
        let fecha = (g[9] || "").trim();
        if (fecha.length >= 6) {
            out.fecha = fecha.substring(0, 2) + "-" +
                        fecha.substring(2, 4) + "-" +
                        fecha.substring(4, 6);
        }

        // ── Hora UTC -> Argentina (UTC-3) ───────────────────────
        let horaRaw     = (g[10] || "").toString().trim();
        let partes      = horaRaw.split(".");
        let parteEntera = (partes[0] || "").replace(/\D/g, "");  // [FIX] sólo dígitos
        let hh, mm, ss;

        if (parteEntera.length === 6) {
            hh = parteEntera.substring(0, 2);
            mm = parteEntera.substring(2, 4);
            ss = parteEntera.substring(4, 6);
        } else if (parteEntera.length === 4) {
            hh = parteEntera.substring(0, 2);
            mm = parteEntera.substring(2, 4);
            let fracMin = partes[1] ? parseFloat("0." + partes[1].replace(/\D/g, "")) : 0;
            ss = Math.round(fracMin * 60).toString().padStart(2, "0");
        } else {
            node.warn("GNSS: formato de hora inesperado -> " + horaRaw);
            hh = "00"; mm = "00"; ss = "00";
        }

        let totalMin = parseInt(hh) * 60 + parseInt(mm) - 180;

        if (totalMin < 0) {
            totalMin += 24 * 60;
            if (out.fecha) {
                let pf = out.fecha.split("-");
                let d  = new Date(parseInt("20" + pf[2]),
                                  parseInt(pf[1]) - 1,
                                  parseInt(pf[0]));
                d.setDate(d.getDate() - 1);
                let dd = d.getDate().toString().padStart(2, "0");
                let mo = (d.getMonth() + 1).toString().padStart(2, "0");
                let yy = d.getFullYear().toString().substring(2);
                out.fecha = dd + "-" + mo + "-" + yy;
            }
        }

        let hhAR = Math.floor(totalMin / 60).toString().padStart(2, "0");
        let mmAR = (totalMin % 60).toString().padStart(2, "0");
        out.hora = hhAR + ":" + mmAR + ":" + ss;

        // ── Campos opcionales según cantidad de campos ──────────
        let total = g.length;

        if (total >= 18) {
            out.alt_m        = parseFloat(g[11]) || 0;
            out.vel_gnss_kph = parseFloat((parseFloat(g[12] || 0) * 1.852).toFixed(2));
            out.dir_deg      = parseFloat(g[13]) || 0;
            out.pdop         = parseFloat(g[14]) || 0;
            out.hdop         = parseFloat(g[15]) || 0;
            out.vdop         = parseFloat(g[16]) || 0;
            out.sats_fix     = parseInt(g[17])   || 0;
        } else if (total >= 17) {
            out.alt_m        = parseFloat(g[11]) || 0;
            out.vel_gnss_kph = parseFloat((parseFloat(g[12] || 0) * 1.852).toFixed(2));
            out.dir_deg      = parseFloat(g[13]) || 0;
            out.pdop         = parseFloat(g[14]) || 0;
            out.hdop         = parseFloat(g[15]) || 0;
            out.vdop         = parseFloat(g[16]) || 0;
        } else if (total >= 14) {
            out.vel_gnss_kph = parseFloat((parseFloat(g[11] || 0) * 1.852).toFixed(2));
            out.dir_deg      = parseFloat(g[12]) || 0;
            out.sats_fix     = parseInt(g[13])   || 0;
        } else {
            node.warn("GNSS: cantidad de campos inesperada -> " + total);
        }

    } catch (e) {
        node.warn("GNSS parse error: " + e.message + " | raw: " + raw.gnss);
    }
}


// ════════ PASO 4 — Tabla de PIDs OBD-II ════════
const OBD_TABLA = {
    "RPM":         { out: "rpm",            f: (A, B) => Math.round(((A * 256 + B) / 4) * 10) / 10 },
    "vel_kph":     { out: "vel_obd_kph",    f: (A, B) => A },
    "temp_mot_c":  { out: "temp_motor_c",   f: (A, B) => A - 40 },
    "temp_adm_c":  { out: "temp_adm_c",     f: (A, B) => A - 40 },
    "temp_ace_c":  { out: "temp_aceite_c",  f: (A, B) => A - 40 },
    "carga_pct":   { out: "carga_mot_pct",  f: (A, B) => parseFloat((A * 100 / 255).toFixed(1)) },
    "comb_pct":    { out: "nivel_comb_pct", f: (A, B) => parseFloat((A * 100 / 255).toFixed(1)) },
    "accel_pct":   { out: "acelerador_pct", f: (A, B) => parseFloat((A * 100 / 255).toFixed(1)) },
    "map_kpa":     { out: "pres_map_kpa",   f: (A, B) => A },
    "pcomb_kpa":   { out: "pres_comb_kpa",  f: (A, B) => A * 3 },
    "maf_gs":      { out: "maf_gs",         f: (A, B) => parseFloat(((A * 256 + B) / 100).toFixed(2)) },
    "avance_deg":  { out: "avance_enc_deg", f: (A, B) => parseFloat((A / 2 - 64).toFixed(1)) },
    "ton_s":       { out: "tiempo_motor_s", f: (A, B) => (A * 256) + B },
    "dist_mil_km": { out: "dist_mil_km",    f: (A, B) => (A * 256) + B },
    "cons_lh":     { out: "consumo_lh",     f: (A, B) => parseFloat(((A * 256 + B) / 20).toFixed(2)) },
};


// ════════ PASO 5 — IMU LSM6DS33 + aceleración del vehículo ════════
const IMU_CAMPOS = ["imu_ax", "imu_ay", "imu_az", "imu_gx", "imu_gy", "imu_gz"];

if (IMU_CAMPOS.some(c => raw[c] !== undefined)) {

    // Acelerómetro m/s² -> g
    if (raw.imu_ax !== undefined)
        out.accel_x_g = parseFloat((raw.imu_ax / G_TO_MS2).toFixed(4));
    if (raw.imu_ay !== undefined)
        out.accel_y_g = parseFloat((raw.imu_ay / G_TO_MS2).toFixed(4));
    if (raw.imu_az !== undefined)
        out.accel_z_g = parseFloat((raw.imu_az / G_TO_MS2).toFixed(4));

    // Giróscopo rad/s -> °/s
    if (raw.imu_gx !== undefined)
        out.gyro_x_dps = parseFloat((raw.imu_gx * RAD_TO_DEG).toFixed(4));
    if (raw.imu_gy !== undefined)
        out.gyro_y_dps = parseFloat((raw.imu_gy * RAD_TO_DEG).toFixed(4));
    if (raw.imu_gz !== undefined)
        out.gyro_z_dps = parseFloat((raw.imu_gz * RAD_TO_DEG).toFixed(4));

    // Módulo de aceleración TOTAL (incluye gravedad; ~1 g en reposo)
    if (out.accel_x_g !== undefined &&
        out.accel_y_g !== undefined &&
        out.accel_z_g !== undefined) {
        out.accel_total_g = parseFloat(
            Math.sqrt(out.accel_x_g ** 2 + out.accel_y_g ** 2 + out.accel_z_g ** 2).toFixed(4)
        );
    }

    // ── [NEW] Aceleración del vehículo (frenada + giro) ──────────
    //   Magnitud en el plano horizontal X-Y.
    //   Supone la placa montada con el eje Z hacia ARRIBA, de modo
    //   que la gravedad cae toda sobre Z y X-Y captura sólo la
    //   dinámica del vehículo. En reposo ~0.
    //     a_horizontal = sqrt(ax² + ay²)   [se usa el crudo en m/s²]
    if (raw.imu_ax !== undefined && raw.imu_ay !== undefined) {
        let hor = Math.sqrt(raw.imu_ax ** 2 + raw.imu_ay ** 2);
        out.accel_horizontal_ms2 = parseFloat(hor.toFixed(4));
        out.accel_horizontal_g   = parseFloat((hor / G_TO_MS2).toFixed(4));
    }

} else {
    node.warn("IMU: ningún campo imu_* presente en el payload.");
}


// ════════ PASO 5B — [NEW] Batería / estado de carga ════════
//   Entrada del Feather:  vbat (V) , cargando (bool) , carga_ok (bool)
if (raw.vbat !== undefined) {
    let v = parseFloat(raw.vbat);
    if (!isNaN(v)) out.bat_v = parseFloat(v.toFixed(2));
}
if (raw.cargando !== undefined)  out.cargando       = Boolean(raw.cargando);
if (raw.carga_ok !== undefined)  out.carga_completa = Boolean(raw.carga_ok);

// Estado combinado, cómodo para el dashboard
if (out.carga_completa === true)      out.estado_carga = "completa";
else if (out.cargando === true)       out.estado_carga = "cargando";
else if (out.cargando === false)      out.estado_carga = "en_bateria";


// ════════ PASO 5C — [NEW v11] DTCs (Modo 03) ════════
//   Entrada: raw.dtc = respuesta cruda "43"+pares de bytes (o "").
//   Salida:  dtc (array de códigos, ej ["P0133"]), dtc_cantidad, dtc_presente.
function decodificarDTC(bruto) {
    if (typeof bruto !== "string") return [];
    let h = bruto.toUpperCase().replace(/[^0-9A-F]/g, "");
    let i = h.indexOf("43");
    if (i < 0) return [];
    h = h.substring(i + 2);
    const letras = ["P", "C", "B", "U"];
    let codigos = [];
    for (let k = 0; k + 4 <= h.length; k += 4) {
        let B1 = parseInt(h.substring(k, k + 2), 16);
        let B2 = parseInt(h.substring(k + 2, k + 4), 16);
        if (isNaN(B1) || isNaN(B2)) break;
        if (B1 === 0 && B2 === 0) continue;        // relleno = sin código
        let cod = letras[(B1 >> 6) & 0x03] +
                  ((B1 >> 4) & 0x03).toString() +
                  (B1 & 0x0F).toString(16).toUpperCase() +
                  ((B2 >> 4) & 0x0F).toString(16).toUpperCase() +
                  (B2 & 0x0F).toString(16).toUpperCase();
        codigos.push(cod);
    }
    return codigos;
}

// [v12] Categoría legible por letra del código
function categoriaDTC(code) {
    return ({ P: "Motor", C: "Chasis", B: "Carroceria", U: "Red/Comunicacion" })[code[0]] || "Sistema";
}

// [v12] Descripción legible del DTC (tabla de comunes + fallback por categoría)
function descripcionDTC(code) {
    const T = {
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
    if (T[code]) return T[code];
    const cat = {
        P: "Falla del motor / tren motriz",
        C: "Falla del chasis (frenos/suspension/direccion)",
        B: "Falla de carroceria / confort",
        U: "Falla de red / comunicacion",
    };
    return cat[code[0]] || "Codigo desconocido";
}

if (raw.dtc !== undefined) {
    let dtcs = decodificarDTC(raw.dtc);
    out.dtc          = dtcs;              // ej ["P0133","C0200"]  o  []
    out.dtc_cantidad = dtcs.length;
    out.dtc_presente = dtcs.length > 0;

    // [v12] Descripción legible + alerta para un usuario cualquiera
    out.dtc_desc   = dtcs.map(descripcionDTC);
    out.dtc_alerta = dtcs.length
        ? dtcs.map(c => categoriaDTC(c) + " en falla: " + descripcionDTC(c) + " [" + c + "]").join(" | ")
        : "sin fallas";
}


// ════════ PASO 6 — Decodificar un PID ════════
function parsearPID(nombre, hexStr) {
    let entrada = OBD_TABLA[nombre];
    if (!entrada) return null;
    if (typeof hexStr !== "string") return null;
    if (hexStr === "" || hexStr === "TIMEOUT") return null;

    let limpio = hexStr.replace(/\s+/g, "").toUpperCase();
    if (limpio.length < 6) {
        node.warn("PID " + nombre + ": respuesta demasiado corta -> " + hexStr);
        return null;
    }
    let A = parseInt(limpio.substring(4, 6), 16);
    let B = limpio.length >= 8 ? parseInt(limpio.substring(6, 8), 16) : 0;
    try {
        return { clave: entrada.out, valor: entrada.f(A, B) };
    } catch (e) {
        node.warn("PID " + nombre + ": error en fórmula -> " + e.message);
        return null;
    }
}


// ════════ PASO 7 — Procesar todos los PIDs ════════
const CAMPOS_RESERVADOS = new Set([
    "ts", "gnss",
    "imu_ax", "imu_ay", "imu_az",
    "imu_gx", "imu_gy", "imu_gz",
    "vbat", "cargando", "carga_ok",     // [NEW] ya procesados arriba
    "dtc",                              // [NEW v11] ya procesado arriba
]);

for (let campo of Object.keys(raw)) {
    if (CAMPOS_RESERVADOS.has(campo)) continue;
    try {
        let resultado = parsearPID(campo, raw[campo]);
        if (resultado !== null) out[resultado.clave] = resultado.valor;
    } catch (e) {
        node.warn("Campo " + campo + ": excepción ignorada -> " + e.message);
    }
}


// ════════ PASO 8 — Salidas: JSON (máquina) + TEXTO legible (humano) ════════
//  msg1 -> prueba_out       : JSON igual que antes (no romper aguas abajo)
//  msg2 -> prueba_out_texto : reporte legible, bloques DTC/GNSS/OBD/IMU/batería
//  El nodo "mqtt out" debe tener el Topic VACÍO para que use msg.topic.

let L = [];
L.push("========== TELEMETRIA  (ts " + (out.ts || 0) + ") ==========");

// ── DTC / FALLAS ──
L.push("");
L.push("[DTC / FALLAS]");
if (out.dtc_alerta !== undefined) {
    L.push("  " + out.dtc_alerta);
    if (out.dtc_cantidad) L.push("  Cantidad: " + out.dtc_cantidad);
} else {
    L.push("  (no reportado)");
}

// ── GNSS ──
L.push("");
L.push("[GNSS]");
if (out.lat !== undefined || out.hora !== undefined) {
    if (out.fecha !== undefined || out.hora !== undefined)
        L.push("  Fecha/Hora: " + (out.fecha || "?") + " " + (out.hora || "?"));
    if (out.lat !== undefined && out.lon !== undefined)
        L.push("  Posicion: " + out.lat + ", " + out.lon);
    let l3 = [];
    if (out.alt_m !== undefined)        l3.push("Alt: " + out.alt_m + " m");
    if (out.vel_gnss_kph !== undefined) l3.push("Vel: " + out.vel_gnss_kph + " km/h");
    if (out.dir_deg !== undefined)      l3.push("Rumbo: " + out.dir_deg + " deg");
    if (out.sats_fix !== undefined)     l3.push("Sats: " + out.sats_fix);
    if (l3.length) L.push("  " + l3.join("   "));
} else {
    L.push("  (sin fix)");
}

// ── OBD / MOTOR ──
L.push("");
L.push("[OBD / MOTOR]");
{
    let filas = [];
    let r1 = [];
    if (out.rpm !== undefined)          r1.push("RPM: " + out.rpm);
    if (out.vel_obd_kph !== undefined)  r1.push("Vel: " + out.vel_obd_kph + " km/h");
    if (r1.length) filas.push(r1.join("   "));
    let r2 = [];
    if (out.temp_motor_c !== undefined)  r2.push("Temp motor: " + out.temp_motor_c + " C");
    if (out.temp_adm_c !== undefined)    r2.push("Temp adm: " + out.temp_adm_c + " C");
    if (out.temp_aceite_c !== undefined) r2.push("Temp aceite: " + out.temp_aceite_c + " C");
    if (r2.length) filas.push(r2.join("   "));
    let r3 = [];
    if (out.carga_mot_pct !== undefined)  r3.push("Carga: " + out.carga_mot_pct + " %");
    if (out.acelerador_pct !== undefined) r3.push("Acelerador: " + out.acelerador_pct + " %");
    if (out.nivel_comb_pct !== undefined) r3.push("Combustible: " + out.nivel_comb_pct + " %");
    if (r3.length) filas.push(r3.join("   "));
    let r4 = [];
    if (out.pres_map_kpa !== undefined)   r4.push("MAP: " + out.pres_map_kpa + " kPa");
    if (out.maf_gs !== undefined)         r4.push("MAF: " + out.maf_gs + " g/s");
    if (out.pres_comb_kpa !== undefined)  r4.push("Pres comb: " + out.pres_comb_kpa + " kPa");
    if (out.avance_enc_deg !== undefined) r4.push("Avance: " + out.avance_enc_deg + " deg");
    if (r4.length) filas.push(r4.join("   "));
    let r5 = [];
    if (out.consumo_lh !== undefined)     r5.push("Consumo: " + out.consumo_lh + " L/h");
    if (out.tiempo_motor_s !== undefined) r5.push("Tiempo motor: " + out.tiempo_motor_s + " s");
    if (out.dist_mil_km !== undefined)    r5.push("Dist MIL: " + out.dist_mil_km + " km");
    if (r5.length) filas.push(r5.join("   "));
    if (filas.length) filas.forEach(f => L.push("  " + f));
    else L.push("  (sin datos)");
}

// ── IMU ──
L.push("");
L.push("[IMU]");
{
    let hay = false;
    if (out.accel_x_g !== undefined) {
        L.push("  Accel (g): x=" + out.accel_x_g + " y=" + out.accel_y_g + " z=" + out.accel_z_g +
               (out.accel_total_g !== undefined ? "   |total|=" + out.accel_total_g : ""));
        hay = true;
    }
    if (out.accel_horizontal_ms2 !== undefined) {
        L.push("  Accel horizontal: " + out.accel_horizontal_ms2 + " m/s2 (" + out.accel_horizontal_g + " g)");
        hay = true;
    }
    if (out.gyro_x_dps !== undefined) {
        L.push("  Giro (deg/s): x=" + out.gyro_x_dps + " y=" + out.gyro_y_dps + " z=" + out.gyro_z_dps);
        hay = true;
    }
    if (!hay) L.push("  (sin datos)");
}

// ── BATERIA ──
L.push("");
L.push("[BATERIA]");
{
    let b = [];
    if (out.bat_v !== undefined)        b.push("Tension: " + out.bat_v + " V");
    if (out.estado_carga !== undefined) b.push("Estado: " + out.estado_carga);
    if (b.length) L.push("  " + b.join("   "));
    else L.push("  (sin datos)");
}

let textoLegible = L.join("\n");

let m1 = Object.assign({}, msg, { payload: JSON.stringify(out), topic: "prueba_out" });
let m2 = Object.assign({}, msg, { payload: textoLegible,        topic: "prueba_out_texto" });

// La Function tiene UNA sola salida. Con return [m1,m2] Node-RED mandaría
// solo el primero. Con node.send() emitimos los DOS por esa única salida
// (el nodo "mqtt out" publica cada uno según su msg.topic).
node.send(m1);
node.send(m2);
return null;
