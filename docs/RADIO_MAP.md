# Radio Map

Contrato actual de radio entre la ECU y `IFS08-TE-main`.

La ECU no envia floats ni IDs `0x600/0x610/...` por `nRF24`.
Ahora envia paquetes binarios de `32` bytes con cabecera fija y payload
troceado por cadencia.

Origen en codigo:

- serializacion radio: [Core/Src/telemetry.c](C:/Users/info/OneDrive/Documentos/4/ACU+ECU/IFS08-CE-ECU/Core/Src/telemetry.c:154)
- estructura base: [Core/Inc/telemetry.h](C:/Users/info/OneDrive/Documentos/4/ACU+ECU/IFS08-CE-ECU/Core/Inc/telemetry.h:33)

## Cabecera comun

Todos los paquetes radio ocupan `32` bytes.

| Byte | Campo | Valor |
|---|---|---|
| 0 | `magic` | `0xEC` |
| 1 | `version` | `0x02` |
| 2 | `fragment_index` | fragmento actual |
| 3 | `fragment_count` | numero total de fragmentos del frame |
| 4-5 | `sequence` LE | secuencia del frame |
| 6 | `kind` | `3=RF_FAST`, `4=RF_SLOW`, `5=RF_EVENT` |
| 7 | `event_type` | `0..3` segun `telemetry_event_type_t` |

Bytes `8..31`: payload del fragmento.

## Cadencia

- `RF_FAST`: cada `100 ms`
- `RF_SLOW`: cada `500 ms`
- `RF_EVENT`: solo cuando hay evento

## RF_FAST

Fragmentos: `2`

### Fragmento 0

| Byte | Campo |
|---|---|
| 8 | `ecu_fsm_state` |
| 9 | `inv_state` |
| 10 | `ams_state` |
| 11 | `boton_arranque` |
| 12 | `ok_precarga` |
| 13 | `flag_ev_2_3` |
| 14 | `flag_t11_8_9` |
| 15 | `inv_vdc_ready` |
| 16 | `inv_error` |
| 17-18 | `torque_total` LE |
| 19-20 | `inv_dc_bus_voltage` LE |
| 21-22 | `v_celda_min` LE |
| 23-24 | `s1_aceleracion` LE |
| 25-26 | `s2_aceleracion` LE |
| 27-28 | `s_freno` LE |

### Fragmento 1

| Bytes | Campo |
|---|---|
| 8-9 | `inv_motor_temp` LE |
| 10-11 | `inv_igbt_temp` LE |
| 12-13 | `inv_air_temp` LE |
| 14-17 | `inv_rpm` LE |
| 18-21 | `inv_speed_actual` LE |
| 22-25 | `inv_current_actual` LE |

## RF_SLOW

Fragmentos: `5`

### Fragmento 0

| Bytes | Campo |
|---|---|
| 8 | `soc` |
| 9-10 | `corriente_accu` LE |
| 11-12 | `corriente_dcdc` LE |
| 13-14 | `temp_dcdc` LE |
| 15-16 | `gps_speed` LE |
| 17-18 | `gps_course_deg` LE |
| 19-22 | `gps_altitude` LE |
| 23-26 | `tick_ms` LE |
| 27 | `gps_fix_type` |
| 28 | `gps_sat_count` |
| 29-30 | `gps_hdop` LE |

### Fragmento 1

| Bytes | Campo |
|---|---|
| 8-11 | `gps_latitude` LE |
| 12-15 | `gps_longitude` LE |

### Fragmento 2

| Bytes | Campo |
|---|---|
| 8-9 | `vmin_modulo[0]` LE |
| 10-11 | `vmin_modulo[1]` LE |
| 12-13 | `vmin_modulo[2]` LE |
| 14-15 | `vmin_modulo[3]` LE |
| 16-17 | `vmin_modulo[4]` LE |

### Fragmento 3

| Bytes | Campo |
|---|---|
| 8-9 | `vmax_modulo[0]` LE |
| 10-11 | `vmax_modulo[1]` LE |
| 12-13 | `vmax_modulo[2]` LE |
| 14-15 | `vmax_modulo[3]` LE |
| 16-17 | `vmax_modulo[4]` LE |

### Fragmento 4

| Bytes | Campo |
|---|---|
| 8-9 | `temp_max_modulo[0]` LE |
| 10-11 | `temp_max_modulo[1]` LE |
| 12-13 | `temp_max_modulo[2]` LE |
| 14-15 | `temp_max_modulo[3]` LE |
| 16-17 | `temp_max_modulo[4]` LE |

## RF_EVENT

Fragmentos: `1`

| Byte | Campo |
|---|---|
| 8 | `inv_state` |
| 9 | `flag_ev_2_3` |
| 10 | `flag_t11_8_9` |
| 11 | `inv_error` |
| 12 | `previous_inv_state` |
| 13 | `current_inv_state` |
| 14 | `event_inv_error` |
| 15 | `ok_precarga` |
| 16 | `inv_vdc_ready` |
| 17-18 | `torque_total` LE |
| 19-20 | `inv_dc_bus_voltage` LE |
| 21-22 | `v_celda_min` LE |
| 23-26 | `event_tick_ms` LE |
| 27 | `ecu_fsm_state` |
| 28 | `ams_state` |

## Nota

Este contrato radio no incluye:

- tensiones por celda individuales
- temperaturas por celda individuales
- `stack_total_mv`
- `cell_max_mv`

Si `IFS08-TE-main` necesita esas variables, hay que anadirlas explicitamente al
`telemetry_snapshot_t` y volver a trocear el protocolo.
