# CAN3 Map

Mapa de las tramas que la ECU publica hacia el dashboard por `CAN_BUS_DASH`
(`FDCAN3`).

Importante:

- `telemetry_snapshot_t` es una estructura **interna** de la ECU
- no se envia "todo el snapshot" por CAN3
- el dash recibe solo un **subconjunto** de campos, serializado en
  `0x510..0x521`

Origen en codigo:

- construccion del snapshot: [Core/Src/telemetry.c](C:/Users/info/OneDrive/Documentos/4/ACU+ECU/IFS08-CE-ECU/Core/Src/telemetry.c:105)
- publicacion CAN3: [Core/Src/freertos.c](C:/Users/info/OneDrive/Documentos/4/ACU+ECU/IFS08-CE-ECU/Core/Src/freertos.c:1397)

## Flujo

1. `TelemetryTask` hace snapshot de `g_in`
2. se construye `telemetry_frame_t`
3. el frame se encola a `telemetryDashQueueHandle`
4. `DashTask` consume ese frame
5. `DashTask` lo convierte en paquetes CAN3 rapidos y lentos

Cadencia:

- periodica base: cada `100 ms`
- variables rapidas: cada `100 ms`
- variables lentas: cada `500 ms`
- adicionalmente puede salir una publicacion extra ante eventos de telemetria
- cada publicacion rapida genera `0x510..0x517`
- cada quinta publicacion genera ademas `0x518..0x521`

## IDs

### `0x510` - estado general

Bus: `FDCAN3`
DLC: `8`

| Byte | Campo | Origen snapshot |
|---|---|---|
| 0 | `inv_state` | `snapshot.inverter.inv_state` |
| 1 | `torque_total` | `snapshot.ecu.torque_total` |
| 2 | `fault_bits` | derivado local |
| 3 | `ok_precarga` | `snapshot.ams.ok_precarga` |
| 4 | `boton_arranque` | `snapshot.ecu.boton_arranque` |
| 5 | `kind` | `telemetry_frame.kind` |
| 6 | `sequence_lo` | `telemetry_frame.sequence` |
| 7 | `sequence_hi` | `telemetry_frame.sequence` |

`fault_bits`:

- bit 0: `flag_ev_2_3`
- bit 1: `flag_t11_8_9`
- bit 2: `inv_error != 0`

### `0x511` - pedales y freno

Bus: `FDCAN3`
DLC: `6`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-1 | `s1_aceleracion` LE | `snapshot.ecu.s1_aceleracion` |
| 2-3 | `s2_aceleracion` LE | `snapshot.ecu.s2_aceleracion` |
| 4-5 | `s_freno` LE | `snapshot.ecu.s_freno` |

### `0x512` - tensiones y estado inversor

Bus: `FDCAN3`
DLC: `6`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-1 | `inv_dc_bus_voltage` LE | `snapshot.inverter.inv_dc_bus_voltage` |
| 2-3 | `v_celda_min` LE | `snapshot.ams.v_celda_min` |
| 4 | `inv_error` | `snapshot.inverter.inv_error` |
| 5 | `inv_vdc_ready` | `snapshot.inverter.inv_vdc_ready` |

### `0x513` - temperaturas inversor

Bus: `FDCAN3`
DLC: `6`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-1 | `inv_motor_temp` LE | `snapshot.inverter.inv_motor_temp` |
| 2-3 | `inv_igbt_temp` LE | `snapshot.inverter.inv_igbt_temp` |
| 4-5 | `inv_air_temp` LE | `snapshot.inverter.inv_air_temp` |

### `0x514` - rpm inversor

Bus: `FDCAN3`
DLC: `4`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-3 | `inv_rpm` LE | `snapshot.inverter.inv_rpm` |

### `0x515` - velocidad inversor

Bus: `FDCAN3`
DLC: `4`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-3 | `inv_speed_actual` LE | `snapshot.inverter.inv_speed_actual` |

### `0x516` - corriente inversor

Bus: `FDCAN3`
DLC: `4`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-3 | `inv_current_actual` LE | `snapshot.inverter.inv_current_actual` |

### `0x517` - estados ECU y AMS

Bus: `FDCAN3`
DLC: `2`

| Byte | Campo | Origen snapshot |
|---|---|---|
| 0 | `ecu_fsm_state` | `snapshot.ecu.fsm_state` |
| 1 | `ams_state` | `snapshot.ams.ams_state` |

### `0x518` - AMS lento

Bus: `FDCAN3`
DLC: `7`
Cadencia: `500 ms`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0 | `soc` | `snapshot.ams.soc` |
| 1-2 | `corriente_accu` LE | `snapshot.ams.corriente_accu` |
| 3-4 | `corriente_dcdc` LE | `snapshot.ams.corriente_dcdc` |
| 5-6 | `temp_dcdc` LE | `snapshot.ams.temp_dcdc` |

### `0x519` - GPS A

Bus: `FDCAN3`
DLC: `8`
Cadencia: `500 ms`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-1 | `gps_speed` LE | `snapshot.gps.speed` |
| 2-3 | `gps_course_deg` LE | `snapshot.gps.course_deg` |
| 4-7 | `gps_altitude` LE | `snapshot.gps.altitude` |

### `0x51A` - GPS B

Bus: `FDCAN3`
DLC: `8`
Cadencia: `500 ms`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0 | `gps_fix_type` | `snapshot.gps.fix_type` |
| 1 | `gps_sat_count` | `snapshot.gps.sat_count` |
| 2-3 | `gps_hdop` LE | `snapshot.gps.hdop` |
| 4-7 | `gps_latitude` LE | `snapshot.gps.latitude` |

### `0x51B` - GPS C + tiempo

Bus: `FDCAN3`
DLC: `8`
Cadencia: `500 ms`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-3 | `gps_longitude` LE | `snapshot.gps.longitude` |
| 4-7 | `tick_ms` LE | `snapshot.tick_ms` |

### `0x51C` - AMS vmin modulos 0..2

Bus: `FDCAN3`
DLC: `6`
Cadencia: `500 ms`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-1 | `vmin_modulo[0]` LE | `snapshot.ams.vmin_modulo[0]` |
| 2-3 | `vmin_modulo[1]` LE | `snapshot.ams.vmin_modulo[1]` |
| 4-5 | `vmin_modulo[2]` LE | `snapshot.ams.vmin_modulo[2]` |

### `0x51D` - AMS vmin modulos 3..4

Bus: `FDCAN3`
DLC: `4`
Cadencia: `500 ms`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-1 | `vmin_modulo[3]` LE | `snapshot.ams.vmin_modulo[3]` |
| 2-3 | `vmin_modulo[4]` LE | `snapshot.ams.vmin_modulo[4]` |

### `0x51E` - AMS vmax modulos 0..2

Bus: `FDCAN3`
DLC: `6`
Cadencia: `500 ms`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-1 | `vmax_modulo[0]` LE | `snapshot.ams.vmax_modulo[0]` |
| 2-3 | `vmax_modulo[1]` LE | `snapshot.ams.vmax_modulo[1]` |
| 4-5 | `vmax_modulo[2]` LE | `snapshot.ams.vmax_modulo[2]` |

### `0x51F` - AMS vmax modulos 3..4

Bus: `FDCAN3`
DLC: `4`
Cadencia: `500 ms`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-1 | `vmax_modulo[3]` LE | `snapshot.ams.vmax_modulo[3]` |
| 2-3 | `vmax_modulo[4]` LE | `snapshot.ams.vmax_modulo[4]` |

### `0x520` - AMS temperaturas maximas 0..2

Bus: `FDCAN3`
DLC: `6`
Cadencia: `500 ms`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-1 | `temp_max_modulo[0]` LE | `snapshot.ams.temp_max_modulo[0]` |
| 2-3 | `temp_max_modulo[1]` LE | `snapshot.ams.temp_max_modulo[1]` |
| 4-5 | `temp_max_modulo[2]` LE | `snapshot.ams.temp_max_modulo[2]` |

### `0x521` - AMS temperaturas maximas 3..4 + DC/DC

Bus: `FDCAN3`
DLC: `6`
Cadencia: `500 ms`

| Bytes | Campo | Origen snapshot |
|---|---|---|
| 0-1 | `temp_max_modulo[3]` LE | `snapshot.ams.temp_max_modulo[3]` |
| 2-3 | `temp_max_modulo[4]` LE | `snapshot.ams.temp_max_modulo[4]` |
| 4-5 | `temp_dcdc` LE | `snapshot.ams.temp_dcdc` |

## Que NO se envia al dash

Aunque exista dentro de `telemetry_snapshot_t`, por CAN3 ahora mismo no se envia:

- `ams_session_id`
- `ams_session_valid`

Eso confirma que el snapshot es una base comun interna y que el contrato CAN3
del dash sigue siendo un empaquetado por grupos y cadencias.
