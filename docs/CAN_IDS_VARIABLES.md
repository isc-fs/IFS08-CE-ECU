# Mapa CAN actual — IDs y variables

Fecha: 18-may-2026

---

## 1. Alcance

Este documento resume los IDs CAN usados por el firmware actual y qué variable viaja en cada uno.

Se separan dos grupos:

- CAN crítico de funcionamiento de ECU
- CAN de visualización para dash

Cuando un valor ocupa más de un byte, se indica el orden usado.

---

## 2. CAN crítico

## 2.1 ECU -> Inversor

### ID `0x360` — `RX_SETPOINT_1`

- Bus: `CAN_BUS_INV` (`FDCAN1`)
- Dirección: ECU -> inversor
- DLC: `3`
- Uso: selección de modo del inversor

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[0]` | `0x00` | no usado |
| `data[1]` | `0x00` | no usado |
| `data[2]` | modo | `INV_MODE_STANDBY`, `INV_MODE_READY`, `INV_MODE_TORQUE`, `INV_MODE_RESET` |

### ID `0x362` — `RX_SETPOINT_3`

- Bus: `CAN_BUS_INV` (`FDCAN1`)
- Dirección: ECU -> inversor
- DLC: `4`
- Uso: consigna legacy de par

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[0]` | `0x00` | no usado |
| `data[1]` | `0x00` | no usado |
| `data[2]` | byte bajo | `legacy_torque & 0xFF` |
| `data[3]` | byte alto | `(legacy_torque >> 8) & 0xFF` |

Nota: `legacy_torque` sale de `torque_pct_to_legacy_command()`.

## 2.2 ECU -> ACU / acumulador

### ID `0x100` — `ID_DC_BUS_VOLTAGE`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Dirección: ECU -> ACU
- DLC: `2`
- IDE: estándar
- Uso: reenvío de tensión de bus DC al sistema de acumulador

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[0]` | byte bajo | `inv_dc_bus_voltage` |
| `data[1]` | byte alto | `inv_dc_bus_voltage` |

### ID `0x600` — `ID_PRECHARGE_CMD`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Dirección: ECU -> ACU
- DLC: `2`
- IDE: estándar
- Uso: orden de precarga

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[0]` | `0` o `1` | `boton_arranque ? 1 : 0` |
| `data[1]` | `0x00` | no usado |

## 2.3 Inversor -> ECU

### ID `0x461` — `TX_STATE_2`

- Bus: `CAN_BUS_INV` (`FDCAN1`)
- Dirección: inversor -> ECU
- DLC esperado: variable
- Uso: estado principal del inversor

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[2]` | error | `inv_error` cuando `inv_state == 10` o `11` |
| `data[4]` nibble bajo | estado | `inv_state` |

### ID `0x463` — `TX_STATE_4`

- Bus: `CAN_BUS_INV` (`FDCAN1`)
- Dirección: inversor -> ECU
- DLC esperado: `8`
- Uso: rpm del inversor

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[5]` | byte bajo | `inv_rpm` |
| `data[6]` | byte medio | `inv_rpm` |
| `data[7]` nibble bajo | nibble alto | `inv_rpm` |

Nota: el firmware recompone un valor de 20 bits con signo.

### ID `0x464` — `TX_STATE_5`

- Bus: `CAN_BUS_INV` (`FDCAN1`)
- Dirección: inversor -> ECU
- DLC esperado: `8`
- Uso: temperaturas

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[0]` | temperatura | `inv_motor_temp` |
| `data[1]` | temperatura | `inv_igbt_temp` |
| `data[2]` | temperatura | `inv_air_temp` |

### ID `0x465` — `TX_STATE_6`

- Bus: `CAN_BUS_INV` (`FDCAN1`)
- Dirección: inversor -> ECU
- DLC esperado: `8`
- Uso: velocidad y corriente medidas

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[2]` | byte bajo | `inv_speed_actual` |
| `data[3]` | byte alto | `inv_speed_actual` |
| `data[4]` | byte bajo | `inv_current_actual` |
| `data[5]` | byte alto | `inv_current_actual` |

Nota: ambos valores se leen en little-endian de 16 bits.

### ID `0x466` — `TX_STATE_7`

- Bus: `CAN_BUS_INV` (`FDCAN1`)
- Dirección: inversor -> ECU
- DLC esperado: `6`
- Uso: tensión de bus DC y habilitación lógica de arranque

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[2]` | byte bajo | `inv_dc_bus_voltage` |
| `data[3]` | byte alto | `inv_dc_bus_voltage` |

Además, al recibir esta trama válida el firmware pone:

- `inv_vdc_ready = 1`

## 2.4 ACU / acumulador -> ECU

### ID `0x20` — `ID_ACK_PRECARGA`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Dirección: ACU -> ECU
- DLC esperado: variable
- Uso: confirmación de precarga

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[0]` | `1` | `ok_precarga = 1` |
| `data[0]` | `0` | `ok_precarga = 0` |

### ID `0x12C` — `ID_V_CELDA_MIN`

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Dirección: ACU -> ECU
- DLC esperado: variable
- Uso: tensión mínima de celda

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[0]` | byte alto | `v_celda_min` |
| `data[1]` | byte bajo | `v_celda_min` |

Nota: esta trama se interpreta en big-endian.

### IDs `0x136` y `0x137` — temperaturas AMS

- Bus: `CAN_BUS_ACU` (`FDCAN2`)
- Dirección: ACU -> ECU
- DLC esperado: `6`
- Uso: temperaturas máximas por módulo y temperatura DCDC

Nota: el firmware actual las interpreta como enteros con signo de 16 bits en big-endian. En `0x137`, `temp_dcdc = -32768` actúa como sentinel de “no disponible”.

---

## 3. CAN dash

Estas tramas se generan en `DashTask` a partir del `telemetry_frame_t` recibido desde `TelemetryTask`.

La transmisión al cuadro se hace directamente desde `DashTask` por `CAN_BUS_DASH` (`FDCAN3`), no pasando por la tarea general `CanTxTask`.

## 3.1 ECU -> Dash

### ID `0x510`

- Bus: `CAN_BUS_DASH`
- Dirección: ECU -> dash
- DLC: `8`
- Uso: estado general, flags y secuencia

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[0]` | estado inversor | `inv_state` |
| `data[1]` | par total | `torque_total` truncado a `uint8_t` |
| `data[2]` | flags | `bit0=flag_EV_2_3`, `bit1=flag_T11_8_9`, `bit2=inv_error!=0` |
| `data[3]` | precarga | `ok_precarga` |
| `data[4]` | botón | `boton_arranque` |
| `data[5]` | tipo de frame | `frame->kind` |
| `data[6]` | secuencia LSB | `sequence` byte bajo |
| `data[7]` | secuencia MSB | `sequence` byte alto |

### ID `0x511`

- Bus: `CAN_BUS_DASH`
- Dirección: ECU -> dash
- DLC: `6`
- Uso: entradas del conductor

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[0]` | byte bajo | `s1_aceleracion` |
| `data[1]` | byte alto | `s1_aceleracion` |
| `data[2]` | byte bajo | `s2_aceleracion` |
| `data[3]` | byte alto | `s2_aceleracion` |
| `data[4]` | byte bajo | `s_freno` |
| `data[5]` | byte alto | `s_freno` |

Nota: todos los valores van en little-endian de 16 bits.

### ID `0x512`

- Bus: `CAN_BUS_DASH`
- Dirección: ECU -> dash
- DLC: `6`
- Uso: variables eléctricas y de estado

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[0]` | byte bajo | `inv_dc_bus_voltage` |
| `data[1]` | byte alto | `inv_dc_bus_voltage` |
| `data[2]` | byte bajo | `v_celda_min` |
| `data[3]` | byte alto | `v_celda_min` |
| `data[4]` | error | `inv_error` |
| `data[5]` | Vdc ready | `inv_vdc_ready` |

### ID `0x513`

- Bus: `CAN_BUS_DASH`
- Dirección: ECU -> dash
- DLC: `6`
- Uso: temperaturas del inversor

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[0]` | byte bajo | `inv_motor_temp` |
| `data[1]` | byte alto | `inv_motor_temp` |
| `data[2]` | byte bajo | `inv_igbt_temp` |
| `data[3]` | byte alto | `inv_igbt_temp` |
| `data[4]` | byte bajo | `inv_air_temp` |
| `data[5]` | byte alto | `inv_air_temp` |

Nota: aunque las temperaturas sean `int16_t`, se transmiten como sus 16 bits crudos en little-endian.

### ID `0x514`

- Bus: `CAN_BUS_DASH`
- Dirección: ECU -> dash
- DLC: `4`
- Uso: rpm del inversor completas

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[0]` | byte 0 | `inv_rpm` |
| `data[1]` | byte 1 | `inv_rpm` |
| `data[2]` | byte 2 | `inv_rpm` |
| `data[3]` | byte 3 | `inv_rpm` |

Nota: se manda el valor completo de `inv_rpm` como 32 bits little-endian.

### ID `0x515`

- Bus: `CAN_BUS_DASH`
- Dirección: ECU -> dash
- DLC: `4`
- Uso: velocidad actual del inversor

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[0]` | byte 0 | `inv_speed_actual` |
| `data[1]` | byte 1 | `inv_speed_actual` |
| `data[2]` | byte 2 | `inv_speed_actual` |
| `data[3]` | byte 3 | `inv_speed_actual` |

### ID `0x516`

- Bus: `CAN_BUS_DASH`
- Dirección: ECU -> dash
- DLC: `4`
- Uso: corriente actual del inversor

| Byte | Contenido | Variable / significado |
|---|---|---|
| `data[0]` | byte 0 | `inv_current_actual` |
| `data[1]` | byte 1 | `inv_current_actual` |
| `data[2]` | byte 2 | `inv_current_actual` |
| `data[3]` | byte 3 | `inv_current_actual` |

---

## 4. Observaciones

- `FDCAN1` queda para inversor.
- `FDCAN2` queda para acumulador / ACU.
- `CAN_BUS_DASH` existe en software para el cuadro y no forma parte del lazo crítico de control.
- El dash no recibe el `telemetry_frame_t` serializado tal cual: la `DashTask` lo adapta a estas tres tramas CAN específicas.

---

## 5. Contenido completo del snapshot

El `snapshot` que viaja dentro de `telemetry_frame_t` contiene un `app_inputs_t` completo. Es decir, la telemetría interna no guarda solo lo que luego se manda al dash, sino todo este bloque de estado:

## 5.1 Entradas físicas

| Campo | Tipo | Significado |
|---|---|---|
| `s1_aceleracion` | `uint16_t` | lectura ADC cruda de APPS1 |
| `s2_aceleracion` | `uint16_t` | lectura ADC cruda de APPS2 |
| `s_freno` | `uint16_t` | lectura ADC cruda de freno |
| `boton_arranque` | `uint8_t` | botón de arranque `0/1` |

## 5.2 Feedback del inversor

| Campo | Tipo | Significado |
|---|---|---|
| `inv_state` | `uint8_t` | estado reportado por el inversor |
| `inv_dc_bus_voltage` | `uint16_t` | tensión de bus DC |
| `inv_vdc_ready` | `uint8_t` | indica que ya se observó `TX_STATE_7` válido |
| `inv_motor_temp` | `int16_t` | temperatura de motor |
| `inv_igbt_temp` | `int16_t` | temperatura de inversor / IGBT |
| `inv_air_temp` | `int16_t` | temperatura de aire interno del inversor |
| `inv_rpm` | `int32_t` | rpm del inversor |
| `inv_speed_actual` | `int32_t` | velocidad medida reportada por inversor |
| `inv_current_actual` | `int32_t` | corriente medida reportada por inversor |
| `inv_error` | `uint8_t` | código de error del inversor |

## 5.3 Batería / acumulador

| Campo | Tipo | Significado |
|---|---|---|
| `v_celda_min` | `uint16_t` | tensión mínima de celda recibida desde ACU |
| `ok_precarga` | `uint8_t` | confirmación lógica de precarga |

## 5.4 Seguridad y plausibilidad

| Campo | Tipo | Significado |
|---|---|---|
| `flag_EV_2_3` | `uint8_t` | flag de plausibilidad EV 2.3 |
| `flag_T11_8_9` | `uint8_t` | flag de plausibilidad / seguridad T11.8.9 |

## 5.5 Variables derivadas

| Campo | Tipo | Significado |
|---|---|---|
| `torque_total` | `uint16_t` | torque total calculado en porcentaje |

## 5.6 Nota importante

- Que un campo exista en el `snapshot` no significa que ya se esté transmitiendo por CAN al dash.
- Ahora mismo el dash publica prácticamente todo el snapshot disponible mediante `0x510` a `0x516`.
- Lo que sigue sin existir en el snapshot actual es, por ejemplo, una temperatura propia del acumulador; por tanto tampoco puede mapearse todavía al dash.
