# CLAUDE.md — IFS08-CE-ECU

Guía de contexto para sesiones futuras. Leer antes de tocar código.

---

## Qué es este proyecto

ECU de competición FSAE (fórmula SAE eléctrico). Se está migrando el firmware de arranque
bloqueante (`main_polling.c`) a FreeRTOS con arquitectura cooperativa. La referencia funcional
es `main_polling.c` — el objetivo es equivalencia 1:1 en comportamiento observable.

**Rama activa de desarrollo:** `feat/1`
**Rama principal:** `main`

---

## Arquitectura en una línea

```
IoSignals_InputStep()  →  g_in (mutex)  →  Control_Step10ms()  →  g_out  →  CanTx / IoSignals_ApplyOutputs()
```

- **`io_signals.c`** — ADC3 (freno, APPS1, APPS2) + GPIO (botón arranque, RTDS)
- **`control.c`** — FSM de arranque + cálculo de torque
- **`can.c`** — parser RX y builder TX
- **`freertos.c`** — tareas: ControlTask (10 ms), CanTxTask (10 ms), CanRxTask (5 ms), TelemetryTask
- **`app_state.h`** — struct `app_inputs_t` / `control_out_t` compartidos

---

## FSM de arranque (control.c)

```
WAIT_INV_VDC_CONFIG  →  (inv_vdc_ready)
BOOT                 →  (precharge_complete || timeout 10 s → retry BOOT)
WAIT_PRECHARGE_ACK   →  (ok_precarga || inv_dc_bus_voltage >= 300)
WAIT_START_BRAKE     →  (boton_arranque && s_freno > UMBRAL_FRENO_ARRANQUE)
R2D_DELAY            →  (2000 ms RTDS activo)
WAIT_INV_STANDBY     →  (inv_state == 3)
ACTIVE               →  torque runtime
```

---

## CAN buses

| Bus    | FDCAN | Rol                  |
|--------|-------|----------------------|
| INV    | FDCAN1| Inversor (RX 0x461–0x466, TX 0x360/0x362) |
| ACU    | FDCAN2| AMS/precarga (RX 0x020/0x12C, TX 0x100/0x600) |
| DASH   | FDCAN3| Reservado (no fuente de pedales) |

Tramas TX 0x100 y 0x600: **FDCAN_EXTENDED_ID**, bus ACU, verificadas correctas vs legacy.

---

## Constantes que requieren calibración en banco

**Actualizar cuando el auto esté montado con sensores reales:**

```c
// Core/Src/control.c — líneas 6-9
#define APPS1_ADC_MIN  2050u   // ADC en reposo, sensor 1  ← PENDING
#define APPS1_ADC_MAX  2950u   // ADC a fondo,   sensor 1  ← PENDING
#define APPS2_ADC_MIN  1915u   // ADC en reposo, sensor 2  ← PENDING
#define APPS2_ADC_MAX  2570u   // ADC a fondo,   sensor 2  ← PENDING

#define UMBRAL_FRENO_ARRANQUE  900u   // raw ADC (≈0.72 V)  ← PENDING
#define UMBRAL_FRENO_APPS     3000u   // raw ADC            ← PENDING
```

**Procedimiento:** leer `g_in.s1_aceleracion`, `g_in.s2_aceleracion`, `g_in.s_freno` con debugger
SWD en reposo y a fondo. El umbral de freno debe ser ≈10 % del recorrido del sensor.

---

## Qué se hizo en feat/1 (esta rama)

| Fix | Archivo | Descripción |
|-----|---------|-------------|
| ADC safe fail | `io_signals.c:130` | Error ADC → 0 en lugar de valor viejo |
| Precharge timeout | `control.c:334` | 10 s en WAIT_PRECHARGE_ACK → retry BOOT |
| Debounce botón | `io_signals.c:108` | 5 muestras × 10 ms = 50 ms |
| APPS defines | `control.c:6–9` | Constantes nombradas MIN/MAX por sensor |

---

## Qué falta para equivalencia completa con main_polling.c

### Bloqueado hasta montaje físico
- [ ] Calibrar APPS1_ADC_MIN/MAX, APPS2_ADC_MIN/MAX, UMBRAL_FRENO_* con sensor real
- [ ] Validar IO física: niveles ADC de freno y APPS, sentido GPIO botón
- [ ] Validar RTDS: polaridad y pin final (`PB4 → D1`)

### Pendiente de decisión / implementación
- [ ] Tests SIL para los 3 fixes de esta rama (debounce, precharge timeout, ADC safe fail)
- [ ] Telemetría: decidir entre formato nuevo (heartbeat+eventos) o modo legacy (`TelFrame` + nRF24)
  - Si legacy: implementar `tel_build_packet()` + `Telemetry_Send32()` real + ruta nRF24/UART
- [ ] Tests golden: comparar trazas CAN de `main_polling.c` vs implementación nueva con misma secuencia de entradas
- [ ] Validar bring-up real FDCAN/TIM16 en hardware (filtros, ISR, cadencia)
- [ ] Vendorizar Unity (mirror local) para `tests/unit` sin red

### Baja prioridad
- [ ] Revisar si `0x100 RX compat` debe retirarse cuando haya cobertura golden
- [ ] Revisar `app_tasks.c` que duplica ruta de runtime
- [ ] Limpiar IDs legacy huérfanos en `VCU.h`

---

## Cómo compilar y correr SIL

```bash
cmake --build build-sil
.\build-sil\tests\sil\ecu08_sil.exe --test-integration   # 134/134 esperado
.\build-sil\tests\sil\ecu08_sil.exe --test-full-cycle
.\build-sil\tests\sil\ecu08_sil.exe --test-legacy-compat
```

SIL define `SIL_BUILD` — el bloque `#ifndef SIL_BUILD` de `io_signals.c` queda excluido
y se usa `sil_can_simulator.c` + `sil_hal_mocks.c` como fuente de señales.

---

## Archivos clave

| Archivo | Qué hace |
|---------|----------|
| `Core/Src/control.c` | FSM arranque + torque. Constantes de calibración aquí. |
| `Core/Src/io_signals.c` | ADC3 + GPIO → g_in. Debounce aquí. |
| `Core/Src/can.c` | Parser RX y TX. Buses por switch anidado. |
| `Core/Src/freertos.c` | Tareas RTOS. Punto de entrada real de control. |
| `Core/Inc/app_state.h` | Structs compartidos `app_inputs_t` / `control_out_t`. |
| `main_polling.c` | Referencia legacy. No modificar. |
| `docs/MAIN_POLLING_MIGRATION_COMPARISON.md` | Comparativa detallada con legacy. |
