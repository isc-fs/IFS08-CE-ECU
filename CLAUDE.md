# CLAUDE.md — IFS08-CE-ECU

Guía de contexto para sesiones futuras. Leer antes de tocar código.

---

## Qué es este proyecto

ECU de competición FSAE (fórmula SAE eléctrico) sobre **STM32H733ZG**. Es una app
**FreeRTOS lean en C++17**: un núcleo de control puro y host-testable (`ecu::Controller`)
rodeado de una fina capa de tareas RTOS y drivers HAL. Enlaza **sobre el
stm32-can-bootloader** (app en `0x08020000`), así que toda la recuperación y el
flasheo van por CAN.

**Rama principal:** `dev` (protegida: PR + checks de firmware/SIL).
**Plataforma:** STM32H733ZG · HAL FW_H7 · CubeMX (`ECU.ioc`).

---

## Arquitectura en una línea

```
FDCAN RX ISR → can_rx_queue → CanRxTask → VehicleService (estado RX compartido)
                                               │ (snapshot)
io_signals (ADC3 + GPIO) → ControlTask 10 ms → ecu::Controller::step()
                                               │
                          salidas: RTDS/LEDs · setpoints inversor · can_tx_post()
                                               ↓
                              can_tx_queue → CanTxTask → FDCAN TX (FDCAN1 INV / FDCAN2 ACU)
```

- **`ecu::Controller`** (núcleo puro, sin HAL): FSM de arranque + torque + cortes de
  plausibilidad FSAE. Compila en host → SIL.
- **VehicleService**: estado recibido por CAN. `CanRxTask` es su **único escritor**;
  `ControlTask` lo *snapshot*-ea cada ciclo. Sustituye al viejo `g_in` con mutex.
- **`can_tx_post()`**: cola productora; `CanTxTask` es el **único** punto que toca el
  TX del FDCAN, así que ninguna otra tarea bloquea en el FIFO de transmisión.

### Tareas RTOS (`Core/Src/freertos.c` + `Core/Src/app/*.cpp`)

| Tarea          | Prioridad     | Período | Rol |
|----------------|---------------|---------|-----|
| `App_InitTask` | High (one-shot)| —      | Levanta FDCAN1/2 y se autodestruye. Corazón del fix #48. |
| `ControlTask`  | **Realtime**  | 10 ms   | Único actor de seguridad. Lee IO, snapshot, `step()`, salidas, `0x100` **en todos los estados**, stream pit-diag opcional, **único que patea el IWDG**. |
| `CanRxTask`    | AboveNormal   | ~RX     | Drena `can_rx_queue`. Despacha: trigger BL → cmd pit-diag → VehicleService. |
| `CanTxTask`    | AboveNormal   | ~TX     | Drena `can_tx_queue` → `HAL_FDCAN_AddMessageToTxFifoQ`. |
| `DiagTask`     | Low           | 1000 ms | Emite `0x704` (health) **aparte de ControlTask**, para sobrevivir a un cuelgue de éste. |
| `TelemetryTask`| —             | ~ms     | Construye el frame de telemetría (io + veh + ctrl), lo manda al dash y fragmenta un snapshot a `telemetry_radio_queue`. |
| `RadioTxTask`  | —             | ~cola   | Único dueño del bus nRF24 (`ecu_radio_tx_task_run`, `telemetry_task.cpp`). Drena `telemetry_radio_queue` → `Telemetry_RadioSend()`. Corre el diag de banco del nRF24 una vez al arrancar (ver sección nRF24 más abajo). |

---

## FSM de arranque (`ecu::CtrlState`, `Core/Src/app/control.cpp`)

```
WaitInvVdcConfig  →  (inv_vconfig_ready, 0x466)
Precharge         →  stream 0x100; espera AMS ok_precharge (0x020); timeout 10 s → reintenta
WaitStartBrake    →  (start_button && brake_raw > BrakeArmRaw)
R2dDelay          →  (RTDS R2dSoundMs = 2000 ms)
WaitInvStandby    →  (inv_state == InvReadyState=4)
Active            →  torque runtime; si !ok_precharge → vuelve a Precharge
AmsError          →  entrada desde CUALQUIER estado si in.ams_error (latcheado);
                     sale a WaitInvVdcConfig cuando !ams_error
```

`AmsError` distingue el **Start re-armable** de la AMS del **Error latcheado**
(`AMS_status` 0x4A0 byte0 == 5): en Error la ECU **inhibe** en vez de reintentar
precarga.

---

## Buses CAN

| Bus       | FDCAN  | Rol |
|-----------|--------|-----|
| **INV**   | FDCAN1 | Inversor NX/EMC (IDs estándar). RX 0x461/0x463/0x466 · TX 0x360/0x362. |
| **ACU**   | FDCAN2 | AMS + Pit-Tool (compartido). RX 0x020/0x12C/0x4A0/0x002/0x7E0 · TX 0x100 + stream pit-diag. |

**No hay FDCAN3** (sólo quedan los handlers débiles del vector de arranque). El `0x600`
está **retirado** (la AMS auto-dispara precarga). El offset de MessageRAM de FDCAN2 es
**387 words** (no solapa con FDCAN1) — esto fue la raíz del TX-dead #48; se aplica en el
`MX_FDCAN1/2_Init` de `fdcan.c`.

### Contrato de mensajes (generado del DSL)

**TX propias de la ECU**

- **`0x100` VCU_heartbeat** (DC-bus V, LE u16) → bus ACU, cada ciclo, **en todos los
  estados**. Contrato cargado ECU↔AMS: la AMS arma un watchdog `VcuStale` (200 ms) en Car
  mode y abre los AIRs si para.
- **Setpoints inversor** (`inverter.cpp` / `ecu_config.hpp`, aún **fuera del DSL** por E2E):
  - `0x360` EMC_RX_SETPOINT_1 — mode word (`App_State_Req` @ byte2, DLC 3).
  - `0x362` EMC_RX_SETPOINT_3 — `Torque_Nm_Req` (s16 LE @ bytes 2-3, DLC 4).
  - Se envían **planos** (bytes 0-1 = 0, sin E2E, byte-por-byte como la VCU original). El
    par va **NEGADO**: restricción mecánica del motor (drive adelante = par negativo), **no
    opcional**.

**Stream pit-diag** (bus ACU, gated por `0x7E0` = `DEADBEEF`, deshabilita con `0`, ACK `0x7E1`):

| ID    | Frame            | Contenido |
|-------|------------------|-----------|
| 0x700 | PitDiag_status   | FSM · inv_state · flags de control (bits) · torque · v_cell_min |
| 0x701 | PitDiag_pedals   | APPS1/2 raw+%, brake raw |
| 0x702 | PitDiag_inverter | DC-bus V · rpm (signed) · error |
| 0x703 | PitDiag_fwinfo   | semver + 4 bytes de git hash |
| 0x704 | PitDiag_health   | heap · liveness por tarea (bits) · reset_cause · uptime · last_fault |
| 0x705 | PitDiag_brake    | presión (bar) + % (depende de la calibración de freno PENDING) |

> **`0x704` se emite siempre (ungated)** desde `DiagTask`, fuera del gate `0x7E0`, para que
> `reset_cause` y la liveness por tarea sean visibles en CAN nada más arrancar y sobrevivan
> a un cuelgue de ControlTask. Paridad con la AMS `0x4A2/0x4A3`.

**RX consumidas** (el sender es dueño del layout)

- `0x020` ACU_ok_precharge — precarga OK / HV viva (la FSM gatea aquí).
- `0x12C` ACU_v_cell_min — mín. tensión de celda (mV, BE) → derate de torque por celda baja.
- `0x4A0` AMS_status — `fsm_state` (5=Error) / `ams_ok` → distingue Start vs Error latcheado.
- `0x461`/`0x463`/`0x466` — inversor: App_State / rpm / DC-bus V.
- `0x002` BL_boot_trigger — magic `0xB007AD12` → escribe magic en RTC backup + reset.
- `0x7E0` PitDiag_cmd — magic `0xDEADBEEF` enable / `0` disable.

---

## CAN DSL code-first (fuente de verdad)

El mapa CAN se define una sola vez en un DSL de macros-X y de ahí se generan structs,
encoders/decoders, descriptores y el DBC:

- **`Core/Inc/can/messages/*.def`** — un frame por archivo (`CAN_MSG` / `FIELD_LE` /
  `FIELD_BE` / `FIELD_BE_S` / `FIELD_LE_BITS` para bits con nombre / `CAN_VAL` para tablas
  enum). `all_messages.inc` los registra (añadir un frame = una línea + su `.def`).
- **`Core/Inc/can/can_dsl.hpp` + `can_codecs.hpp`** — el motor de macros; expande el `.inc`
  en varias pasadas (struct / encode / decode / descriptor / guarda de solape). De aquí
  salen `ecu::<Msg>_ID` / `_DLC`.
- **`tools/dbc_dump.cpp` → `docs/dbc/ecu.dbc`** — el DBC generado, con tablas `VAL_` de enum
  (`CAN_VAL`, gated por `ECU_DSL_VALUES_PASS`) y señales de bit con nombre (`FIELD_LE_BITS`).
  El bot **dbcinator** (`.github/workflows/dbc-bot.yml`) lo regenera y fuerza-pushea en cada
  PR, así que el DBC nunca diverge del DSL.

```bash
# regenerar el DBC a mano:
c++ -std=c++17 -I Core/Inc tools/dbc_dump.cpp -o /tmp/dbc_dump && /tmp/dbc_dump
```

---

## Constantes que requieren calibración en banco

Todas viven ahora en **`Core/Inc/app/ecu_config.hpp`** (HAL-free, host-testable), tagueadas
`COMMISSION` — son placeholders heredados de la VCU legacy (placa IFS06) y **deben
re-medirse en el coche montado con sensores reales antes de cualquier marcha**:

```cpp
// Core/Inc/app/ecu_config.hpp
Apps1AdcMin = 2050;  Apps1AdcMax = 2950;   // COMMISSION (APPS1, ADC 12-bit)
Apps2AdcMin = 1915;  Apps2AdcMax = 2570;   // COMMISSION (APPS2)
BrakeArmRaw     = 900;    // COMMISSION: freno-para-armar (R2D)
BrakePressedRaw = 3000;   // COMMISSION: EV.2.3 "freno pisado"
```

**Procedimiento:** leer `apps1_raw` / `apps2_raw` / `brake_raw` del stream pit-diag `0x701`
(o por SWD) en reposo y a fondo. El umbral de armado de freno debe ser ≈10 % del recorrido.
`pct = clamp((raw - min) * 100 / (max - min), 0, 100)`.

---

## Arranque y layout de flash (fix #48 — handoff del bootloader)

El stm32-can-bootloader entrega la app con **IRQs enmascaradas** (`__disable_irq`) y SysTick
parado (su `Bootloader_JumpToApplication`, su issue #59). La app debe rearmar todo:

1. **`main.c` USER CODE BEGIN 1** — lo PRIMERO: limpiar SysTick pendiente y `__enable_irq()`.
   Sin esto `HAL_GetTick()` queda congelado, las esperas de HAL giran para siempre y el IWDG
   heredado resetea antes del init (era el crash de boot intermitente — confirmado por SWD).
2. **SysInit** — refresca el IWDG heredado y fuerza-resetea el periférico FDCAN para
   configurarlo desde un estado limpio (la BL lo deja arrancado).
3. **USER CODE 2** — habilita el reloj del RTC (`__HAL_RCC_RTC_ENABLE`) para que `BKPxR`
   (fault latch + boot magic) no haga bus-fault tras un reset de backup domain.

**Flash** (`STM32H733ZGTX_FLASH.ld`): app en `0x08020000` (768K); registro firmware-info
fijado en `0x08020400`. Node id de la ECU = `0x01` (multi-nodo BL: ECU 0x01 / AMS 0x02 /
uDV 0x03).

---

## Telemetría radio (nRF24) — SPI **por software**, no periférico SPI1

**El periférico SPI1 hardware de este board no lee MISO correctamente** (bench-confirmado
2026-07: `SPI1->SR`/HAL reportaban transferencia OK pero MISO leía `0xFF` fijo, en TODAS las
condiciones probadas — prescaler /64 y /256, dos módulos nRF24 distintos, cableado
revisado). Un bit-bang GPIO puro en los mismos pines, en cambio, lee el STATUS real del
chip (`0x0E`) de forma perfecta y repetible. Módulo y cableado quedaron descartados como
causa; el fallo está aislado al *capture path* del propio periférico SPI1 en este
MCU/board, sin causa raíz confirmada (habría que instrumentar con osciloscopio/analizador
lógico, no disponible en el bench).

**Consecuencia: `Core/Src/spi.c` y `Core/Inc/spi.h` fueron eliminados.** SPI1 no se usa en
absoluto en el firmware actual. Todo el protocolo nRF24 se clockea a mano:

- `NRF24_BusInit()` (`Core/Src/nrf24.c`) toma PA5/PA6/PA7 (SCK/MOSI/MISO) como **GPIO puro
  permanente** — nunca vuelven a modo AF de SPI1.
- `NRF24_BitBangTransfer()` (`Core/Src/nrf24.c`) es el transporte real: reemplaza a los
  antiguos `SPI1_Transfer`/`SPI1_Transfer8` en los tres call-sites (`NRF24_SpiTransfer`,
  `NRF24_ReadRawNop4`, `NRF24_ExecuteCommand`). Throughput de sobra para telemetría (unos
  pocos ms por payload de 32 bytes).
- Pines sin cambios respecto al diseño original: `NRF24_CS`=PB0, `NRF24_CE`=PC5,
  `NRF24_IRQ`=PC4.
- `NRF24_DiagnoseSerial()` corre **una sola vez**, al arrancar `ecu_radio_tx_task_run`
  (`Core/Src/app/telemetry_task.cpp`, única tarea dueña del bus nRF24). Imprime por
  USART10 (PG12 TX, 115200 8N1) un veredicto de banco: NOP/REG/write-readback de RF_CH
  con dos patrones (`0x2A`/`0x15`). Si el nRF24 alguna vez vuelve a fallar (módulo,
  cableado, alimentación), este es el primer sitio a mirar — el mensaje `VERDICT:`
  distingue MISO-siempre-alto, MISO-siempre-bajo e inestable.
- `g_spi_diag`/`ecu_spi_diag_t` (campos SPI del log `TASK` por UART) fueron retirados de
  `app_globals.*` y `diag_task.cpp` — quedaban congelados en 0 al no usarse ya el
  periférico.

**Si en el futuro se quiere retomar el SPI1 hardware:** el punto de partida sería revisar
`hspi1.Init.MasterSSIdleness`/`MasterInterDataIdleness` (ambos en 0 ciclos actualmente) o
comparar formas de onda SCK/MOSI/MISO en HW-SPI vs bit-bang con un osciloscopio — nunca se
llegó a esa instrumentación. Mientras tanto, **no reintroducir `spi.c`/`spi.h` sin repetir
esta comparación**; si se regenera código desde `ECU.ioc` en CubeMX, es probable que
vuelvan a aparecer con SPI1 habilitado y haya que borrarlos de nuevo.

---

## Cómo compilar y correr SIL

```bash
# SIL host (núcleo de control puro). Unity offline → deshabilitar unit tests:
cmake -S . -B build-sil -DBUILD_SIL_TESTS=ON -DBUILD_UNIT_TESTS=OFF
cmake --build build-sil
ctest --test-dir build-sil --output-on-failure        # o:
./build-sil/tests/sil/ecu08_sil --test-all
```

El target SIL `ecu08_sil` define `SIL_BUILD=1` y apunta directamente al núcleo
`ecu::Controller` (FSM + cada corte de plausibilidad FSAE). El firmware ARM se compila desde
`firmware/CMakeLists.txt` (lista las fuentes de app explícitamente; quitar un periférico de
CubeMX exige editar ese CMakeLists a mano).

---

## Archivos clave

| Archivo | Qué hace |
|---------|----------|
| `Core/Src/app/control.cpp` · `Core/Inc/app/control.hpp` | Núcleo `ecu::Controller`: FSM + torque + plausibilidad. Sin HAL. |
| `Core/Inc/app/ecu_config.hpp` | **Todas** las constantes tunables. Calibración `COMMISSION` aquí. |
| `Core/Src/app/control_task.cpp` | Tarea realtime 10 ms. `0x100` en todo estado + único kicker IWDG. |
| `Core/Src/app/can_rx_task.cpp` | Drena RX, despacha, único escritor de VehicleService. |
| `Core/Src/app/can_tx_task.cpp` | Único punto TX del FDCAN; selecciona bus por `frame.bus`. |
| `Core/Src/app/diag_task.cpp` | `0x704` health cada 1 s, separado de control. |
| `Core/Src/app/app_init_task.cpp` | Bring-up FDCAN1/2 one-shot (fix #48). |
| `Core/Src/app/io_signals.cpp` | ADC3 (freno, APPS1/2) + GPIO (botón, RTDS, LEDs). |
| `Core/Src/app/inverter.cpp` | Adaptador inversor NX/EMC: setpoints 0x360/0x362, decode 0x461/63/66. |
| `Core/Src/app/vehicle_service.cpp` | Estado RX compartido (snapshot por control). |
| `Core/Src/app/{bootloader,error_latch,reset_cause,firmware_info,pit_diag,watchdog}.cpp` | Trigger BL (0x002), latch de fault en BKPxR, reset cause, fwinfo (0x703), builders pit-diag, helpers IWDG. |
| `Core/Src/app/telemetry_task.cpp` | `TelemetryTask` + `RadioTxTask` (único dueño del bus nRF24). Corre `NRF24_DiagnoseSerial()` al arrancar. |
| `Core/Src/nrf24.c` · `Core/Inc/nrf24.h` | Driver nRF24 **por bit-bang GPIO** (sin SPI1 hardware, ver sección arriba). `NRF24_BitBangTransfer` es el transporte real. |
| `Core/Inc/can/messages/*.def` + `all_messages.inc` | DSL CAN (fuente de verdad). |
| `Core/Src/{main,fdcan,freertos}.c` | Handoff BL (#48), MX FDCAN + offset 387w, attrs de tareas. |
| `docs/dbc/ecu.dbc` | DBC generado del DSL (dbcinator lo mantiene). |
| `docs/ECU_ACU_MENSAJES.md` · `docs/CAN_IDS_VARIABLES.md` · `docs/PINES_RUTEADOS_IOC.md` | Contrato AMS, mapa de IDs/variables, pines ruteados. |

> Histórico: `docs/MAIN_POLLING_MIGRATION_COMPARISON.md` documenta el firmware bloqueante
> anterior. Es referencia histórica, **no** describe el `dev` actual.
