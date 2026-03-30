# Comparativa de Migracion 1:1 vs `main_polling.c`

Fecha: 2026-03-31

Objetivo: comparar el proyecto actual contra `main_polling.c`, tratando `main_polling.c` como referencia funcional legacy para una migracion 1:1.

## 1. Resumen ejecutivo

La migracion actual es **parcial**, no 1:1.

Estimacion de paridad funcional:
- **55-60%** si medimos parser CAN + control principal.
- Menos si incluimos hardware real, telemetria legacy, IO fisica y validacion HIL.

Conclusion corta:
- El parser CAN legacy principal esta bastante bien migrado.
- La logica base de torque, EV2.3 y T11.8.9 esta portada en buena parte.
- El SIL es ahora mas fiel al runtime real porque compila `freertos.c`, llama `MX_FREERTOS_Init()` y ejecuta las tareas reales con un scheduler cooperativo mock. Referencias: `tests/sil/CMakeLists.txt:22`, `tests/sil/CMakeLists.txt:71`, `tests/sil/sil_main.c:70`, `tests/sil/sil_main.c:75`, `tests/sil/mocks/cmsis_os2_impl.c:171`, `tests/sil/mocks/cmsis_os2_impl.c:343`.
- Aun asi, la secuencia real de arranque del inversor, la telemetria legacy, la adquisicion fisica de IO y el bring-up real del hardware no estan en equivalencia.

Los 5 gaps mas importantes siguen siendo:

1. **La FSM actual no espera los estados reales del inversor `3 -> 4 -> 6`.**
   El legacy bloquea hasta `state == 3` y luego `state == 4` antes de entrar en torque.
   Referencias: `main_polling.c:682`, `main_polling.c:696`, `Core/Src/control.c:200`, `Core/Src/control.c:207`, `Core/Src/control.c:218`.

2. **La telemetria legacy no esta migrada.**
   El legacy rota `TelFrame` `0x600/0x610/0x620/0x630` y lo manda por `nrf24_tx32()` con logging UART.
   Hoy solo hay un buffer simple de 32 bytes y `Telemetry_Send32()` es `weak`/no-op en firmware real.
   Referencias: `main_polling.c:119`, `main_polling.c:2326`, `main_polling.c:2440`, `Core/Src/telemetry.c:4`, `Core/Src/telemetry.c:33`.

3. **La IO fisica de arranque/freno no esta portada.**
   El legacy lee freno por ADC y boton por GPIO en la secuencia de arranque.
   Hoy `boton_arranque` y `s_freno` son senales en estado compartido.
   Referencias: `main_polling.c:625`, `main_polling.c:635`, `Core/Inc/app_state.h:11`, `Core/Src/adc.c:29`, `Core/Src/gpio.c:42`.

4. **La limitacion por `v_celda_min` falta en control.**
   El legacy la calcula en `setTorque()`.
   `Control_ComputeTorque()` no usa `v_celda_min`, `inv_dc_bus_voltage`, temperaturas ni `inv_state`.
   Referencias: `main_polling.c:1994`, `Core/Src/control.c:101`.

5. **El bring-up real de CAN/TIM no esta en paridad.**
   El legacy configura filtros, arranca FDCAN, activa notificaciones y arranca TIM16.
   Hoy `main.c` solo inicializa perifericos y RTOS; `fdcan.c` y `tim.c` no reproducen ese bring-up.
   Referencias: `main_polling.c:430`, `main_polling.c:438`, `main_polling.c:508`, `main_polling.c:1115`, `Core/Src/main.c:100`, `Core/Src/fdcan.c:32`, `Core/Src/tim.c:109`.

## 2. Mapa de migracion

| Bloque legacy en `main_polling.c` | Equivalente actual | Estado | Comentario |
|---|---|---|---|
| `HAL_FDCAN_RxFifo0Callback` monolitico | `CanRx_ParseAndUpdate()` + `Can_ISR_PushRxFifo0()` | Migrado | El parser principal `0x20/0x12C/0x101/0x461..0x466` si esta portado. Referencias: `main_polling.c:1738`, `Core/Src/can.c:73`, `Core/Src/can.c:203`, `Core/Src/main_rx_callback_snippet.c:6`. |
| Bucle de precarga `while (precarga_inv == 0 && inv_dc_bus_voltage < 300)` | `CTRL_ST_BOOT` y `CTRL_ST_WAIT_PRECHARGE_ACK` | Parcial | Faltan la condicion `inv_dc_bus_voltage < 300` y la periodicidad exacta del legacy. Referencias: `main_polling.c:534`, `Core/Src/control.c:153`, `Core/Src/control.c:173`. |
| Espera boton + freno fisico | `CTRL_ST_WAIT_START_BRAKE` | Parcial | Existe la condicion, pero usa senales logicas y umbral `3000`, no lectura fisica ni umbral `900` del legacy. Referencias: `main_polling.c:622`, `main_polling.c:645`, `Core/Src/control.c:192`, `Core/Src/control.c:193`. |
| RTDS 2 s por GPIO | `CTRL_ST_R2D_DELAY` | Parcial | Se mantiene el retardo de 2 s, pero no la salida GPIO `RTDS`. Referencias: `main_polling.c:662`, `Core/Src/control.c:200`, `Core/Src/control.c:203`. |
| Espera estados del inversor `3/4`, y ramas `0/10/11/13` | `CTRL_ST_READY` y `CTRL_ST_RUN` | Desviado | La FSM actual no usa `inv_state` para transicionar ni maneja `soft fault/hard fault/shutdown` como el legacy. Referencias: `main_polling.c:682`, `main_polling.c:696`, `main_polling.c:2135`, `Core/Src/control.c:207`, `Core/Src/control.c:214`, `Core/Src/control.c:218`. |
| `setTorque()` | `Control_ComputeTorque()` | Parcial | EV2.3, T11.8.9 y el escalado base si; falta la rama de `v_celda_min`. Referencias: `main_polling.c:1876`, `main_polling.c:1994`, `Core/Src/control.c:101`, `Core/Src/control.c:115`. |
| ISR periodica TIM16 que emite CAN | `StartControlTask` + `StartCanTxTask` | Parcial | La logica existe y SIL ya ejecuta esas tareas reales mediante `MX_FREERTOS_Init()` + scheduler cooperativo mock, pero el timing observable sigue siendo 10 ms de control y 20 ms de TX, no los 10 ms efectivos del legacy. Referencias: `main_polling.c:2041`, `Core/Src/freertos.c:193`, `Core/Src/freertos.c:199`, `Core/Src/freertos.c:266`, `Core/Src/freertos.c:304`, `tests/sil/sil_main.c:75`, `tests/sil/mocks/cmsis_os2_impl.c:343`. |
| `TelFrame` + `tel_build_packet()` + `nrf24_tx32()` | `Telemetry_Build32()` + `Telemetry_Send32()` | No migrado | Formato, cadencia y transporte no coinciden. Referencias: `main_polling.c:119`, `main_polling.c:2326`, `main_polling.c:2440`, `Core/Src/telemetry.c:4`, `Core/Src/telemetry.c:33`. |
| ADC1/ADC2/DMA + GPIO start/brake | Solo `ADC3` configurado y GPIO basicos | No migrado | No hay adquisicion runtime equivalente. Referencias: `main_polling.c:417`, `main_polling.c:625`, `main_polling.c:635`, `Core/Src/adc.c:29`, `Core/Src/gpio.c:42`. |
| Bring-up FDCAN/TIM real | `main.c` + `fdcan.c` + `tim.c` | No migrado | Configuran perifericos, pero no reproducen `Start/Filter/Notification/Start_IT`. Referencias: `main_polling.c:430`, `main_polling.c:438`, `main_polling.c:508`, `main_polling.c:1115`, `Core/Src/main.c:100`, `Core/Src/fdcan.c:32`, `Core/Src/tim.c:109`. |

## 3. Diferencias funcionales

### Alta

1. **La FSM nueva no espera al inversor real antes de entrar en torque.**
   - Legacy:
     espera `state == 3`, luego manda `0x360` modo `0x04` hasta `state == 4`, y solo despues entra en torque.
     Referencias: `main_polling.c:682`, `main_polling.c:696`, `main_polling.c:2145`, `main_polling.c:2198`.
   - Actual:
     pasa de `R2D_DELAY` a `READY` y luego a `RUN` por tiempo local, sin usar `in->inv_state`.
     Referencias: `Core/Src/control.c:200`, `Core/Src/control.c:207`, `Core/Src/control.c:214`, `Core/Src/control.c:218`.
   - Riesgo:
     puede mandar par fuera de secuencia real.
   - Para igualarlo:
     gatear la FSM con `in->inv_state` y portar ramas `0/10/11/13`.

2. **El bring-up de hardware real no esta en equivalencia.**
   - Legacy:
     configura filtros, hace `HAL_FDCAN_Start`, activa `FDCAN_IT_RX_FIFO0_NEW_MESSAGE` y arranca TIM16.
     Referencias: `main_polling.c:430`, `main_polling.c:438`, `main_polling.c:456`, `main_polling.c:477`, `main_polling.c:508`, `main_polling.c:1115`, `main_polling.c:1178`, `main_polling.c:1241`.
   - Actual:
     `main.c` solo inicializa perifericos y RTOS; `fdcan.c` y `tim.c` solo hacen init.
     Referencias: `Core/Src/main.c:100`, `Core/Src/fdcan.c:32`, `Core/Src/tim.c:109`.
   - Riesgo:
     en placa puede no entrar ningun frame ni existir tick periodico equivalente.
   - Para igualarlo:
     portar explicitamente `HAL_FDCAN_ConfigFilter`, `HAL_FDCAN_Start`, `HAL_FDCAN_ActivateNotification` y `HAL_TIM_Base_Start_IT`.

3. **Falta la limitacion por `v_celda_min`.**
   - Legacy:
     calcula `torque_limitado` segun tension minima de celda.
     Referencia: `main_polling.c:1994`.
   - Actual:
     `Control_ComputeTorque()` no usa `v_celda_min`.
     Referencia: `Core/Src/control.c:101`.
   - Riesgo:
     perdida de proteccion funcional frente a bateria baja.
   - Para igualarlo:
     portar esa rama exacta y decidir si tambien se preserva el bug legacy de calcular `torque_limitado` pero luego escalar `torque_total`.

4. **La telemetria legacy no existe en el runtime actual.**
   - Legacy:
     empaqueta `TelFrame` y lo transmite por nRF24 con apoyo UART.
     Referencias: `main_polling.c:2326`, `main_polling.c:2440`, `main_polling.c:2454`.
   - Actual:
     solo empaqueta un buffer simple y `Telemetry_Send32()` es vacio en firmware real.
     Referencias: `Core/Src/telemetry.c:4`, `Core/Src/telemetry.c:33`.
   - Riesgo:
     no hay equivalencia operativa ni observabilidad real.
   - Para igualarlo:
     portar `tel_build_packet()` y `tel_send_now()` como modo legacy.

5. **La IO fisica de arranque/freno no esta portada.**
   - Legacy:
     usa ADC y GPIO reales.
     Referencias: `main_polling.c:625`, `main_polling.c:635`.
   - Actual:
     usa solo senales almacenadas en `g_in`.
     Referencias: `Core/Inc/app_state.h:11`.
   - Riesgo:
     el firmware no puede comportarse 1:1 en hardware real.
   - Para igualarlo:
     crear tarea o ruta HAL que lea ADC/GPIO reales y escriba `g_in`.

### Media

1. **La temporizacion observable no coincide con el legacy.**
   - Legacy:
     emite desde TIM16 cada 10 ms.
     Referencia: `main_polling.c:2041`.
   - Actual:
     RX a 5 ms, control a 10 ms y TX a 20 ms.
     Referencias: `Core/Src/freertos.c:285`, `Core/Src/freertos.c:266`, `Core/Src/freertos.c:304`.
   - Riesgo:
     cambios de fase, jitter y periodicidad hacia inversor/ACU.
   - Para igualarlo:
     mantener 10 ms efectivos de emision para las tramas legacy.

2. **La salida de precarga no replica la pauta legacy completa.**
   - Legacy:
     emite `0x100` y `0x600` tanto en el bucle de precarga como en la ISR de TIM16.
     Referencias: `main_polling.c:543`, `main_polling.c:564`, `main_polling.c:2046`, `main_polling.c:2066`.
   - Actual:
     solo las manda en `BOOT` y `WAIT_PRECHARGE_ACK`.
     Referencias: `Core/Src/control.c:153`, `Core/Src/control.c:173`.
   - Riesgo:
     puede cambiar como ve el ACU la secuencia de precarga.
   - Para igualarlo:
     portar la pauta temporal exacta.

3. **La condicion de salida de precarga diverge.**
   - Legacy:
     sale del while si `precarga_inv == 1` o si `inv_dc_bus_voltage >= 300`.
     Referencia: `main_polling.c:534`.
   - Actual:
     solo usa `ok_precarga`.
     Referencias: `Core/Src/control.c:156`, `Core/Src/control.c:176`.
   - Riesgo:
     divergencia en ramp-up HV.
   - Para igualarlo:
     anadir la condicion de `inv_dc_bus_voltage`.

4. **El umbral de freno para arrancar no coincide.**
   - Legacy:
     `s_freno > 900`.
     Referencia: `main_polling.c:645`.
   - Actual:
     `s_freno > 3000`.
     Referencias: `Core/Src/control.c:5`, `Core/Src/control.c:193`.
   - Riesgo:
     cambio de usabilidad y gating funcional.
   - Para igualarlo:
     separar umbral de arranque del umbral EV2.3.

5. **La telemetria tiene otra cadencia.**
   - Legacy:
     500 ms.
     Referencias: `main_polling.c:101`, `main_polling.c:762`.
   - Actual:
     100 ms.
     Referencias: `Core/Src/freertos.c:323`, `Core/Src/freertos.c:329`.
   - Riesgo:
     no rompe control, pero si integracion y ancho de banda.
   - Para igualarlo:
     restaurar 500 ms o hacer modo `legacy_telemetry_period`.

### Baja

1. **`app_tasks.c` duplica una ruta de runtime distinta de `freertos.c`.**
   - Riesgo:
     que firmware, SIL y codigo de referencia diverjan.
   - Referencia:
     `Core/Src/app_tasks.c:35`.

2. **`VCU.h` conserva IDs legacy no cableados al runtime actual.**
   - Riesgo:
     confusion de mantenimiento.
   - Referencia:
     `Core/Inc/VCU.h:97`.

3. **El parser actual mantiene rutas compat no canonicas para la migracion 1:1.**
   - Ejemplos:
     `0x100` como RX compat y `0x102`/`0x103` parseados de forma simplificada.
   - Referencias:
     `Core/Src/can.c:84`, `Core/Src/can.c:102`, `Core/Src/can.c:106`.

## 4. Protocolo CAN comparado

| ID | Bus | Dir | Payload legacy esperado | Implementacion actual | Estado |
|---|---|---|---|---|---|
| `0x020` | ACU / FDCAN2 | RX | `data[0] == 0` => precarga OK. Referencia: `main_polling.c:1822`. | Igual en `Core/Src/can.c:79`. | OK |
| `0x12C` | ACU / FDCAN2 | RX | `v_celda_min` big-endian. Referencia: `main_polling.c:1829`. | Igual en `Core/Src/can.c:110`. | OK |
| `0x101` | DASH / FDCAN3 | RX | `S1` y `S2` juntos, big-endian, 4 bytes. Referencia: `main_polling.c:1842`. | Soportado tal cual si `bus == DASH` y `dlc >= 4`. Referencia: `Core/Src/can.c:89`. | OK |
| `0x102` | DASH / FDCAN3 | RX | Definido en `VCU.h`, pero no usado por el callback legacy principal. Referencia: `Core/Inc/VCU.h:144`. | Parse compat LE en `Core/Src/can.c:102`. | Parcial |
| `0x103` | DASH / FDCAN3 | RX | Definido en `VCU.h`; en arranque legacy el freno se lee por ADC. Referencias: `Core/Inc/VCU.h:145`, `main_polling.c:628`. | Parse compat LE en `Core/Src/can.c:106`. | Parcial |
| `0x461` | INV / FDCAN1 | RX | `state` en `data[4] & 0xF`, `error` en `data[2]`. Referencia: `main_polling.c:1751`. | Igual en `Core/Src/can.c:115`. | OK |
| `0x463` | INV / FDCAN1 | RX | RPM con sign-extend desde bytes `5..7`. Referencia: `main_polling.c:1761`. | Igual en `Core/Src/can.c:123`. | OK |
| `0x464` | INV / FDCAN1 | RX | Temperaturas. Referencia: `main_polling.c:1772`. | Igual en `Core/Src/can.c:137`. | OK |
| `0x465` | INV / FDCAN1 | RX | Velocidad/corriente LE bytes `2..5`. Referencia: `main_polling.c:1780`. | Igual en `Core/Src/can.c:146`. | OK |
| `0x466` | INV / FDCAN1 | RX | Vdc LE bytes `2..3`, `dlc == 6`. Referencia: `main_polling.c:1787`. | Igual en `Core/Src/can.c:154`. | OK |
| `0x100` | ACU / FDCAN2 | TX | Vdc al AMS, ext ID, LE, 2 bytes. Referencia: `main_polling.c:543`. | Emitido en `Core/Src/control.c:79`, pero ademas aceptado como RX compat en `Core/Src/can.c:84`. | Parcial |
| `0x600` | ACU / FDCAN2 | TX | Boton de precarga, ext ID, 2 bytes. Referencia: `main_polling.c:564`. | Emitido solo en `BOOT`/`WAIT_PRECHARGE_ACK`. Referencia: `Core/Src/control.c:90`. | Parcial |
| `0x360` | INV / FDCAN1 | TX | Modo `0x01/0x04/0x06/0x13` segun estado. Referencia: `main_polling.c:2135`. | Solo `0x04` y `0x06`. Referencias: `Core/Src/control.c:207`, `Core/Src/control.c:224`. | Parcial |
| `0x362` | INV / FDCAN1 | TX | Torque cero o comando legacy bytes `2..3`. Referencias: `main_polling.c:2172`, `main_polling.c:2207`. | Formato portado, pero faltan ramas de fallo/reactivacion. Referencia: `Core/Src/control.c:69`. | Parcial |

Nota:
- Los IDs `0x600/0x610/0x620/0x630` del legacy son IDs internos del `TelFrame` de telemetria por nRF24, no tramas CAN.
- Referencias: `main_polling.c:157`, `main_polling.c:2326`.

## 5. Estado de tests

### Que si demuestran equivalencia util

- El parser CAN principal y la tuberia mock `HAL -> ISR -> queue -> parser` funcionan en SIL.
  Referencias: `tests/sil/mocks/hal_impl.c:197`, `Core/Src/test_integration.c:215`.

- SIL ya compila y ejecuta `freertos.c`, no solo la logica pura.
  El build incluye `Core/Src/freertos.c`, el runner llama `MX_FREERTOS_Init()` y el kernel mock planifica las tareas creadas por `osThreadNew()`.
  Referencias: `tests/sil/CMakeLists.txt:22`, `tests/sil/sil_main.c:28`, `tests/sil/sil_main.c:70`, `tests/sil/sil_main.c:75`, `tests/sil/mocks/cmsis_os2_impl.c:171`, `tests/sil/mocks/cmsis_os2_impl.c:343`.

- La topologia real de tareas ya queda cubierta por SIL.
  `StartCanRxTask`, `StartControlTask`, `StartCanTxTask` y `StartTelemetryTask` se crean desde `MX_FREERTOS_Init()` y ceden con `osDelay()` bajo el scheduler cooperativo mock.
  Referencias: `Core/Src/freertos.c:193`, `Core/Src/freertos.c:196`, `Core/Src/freertos.c:199`, `Core/Src/freertos.c:202`, `Core/Src/freertos.c:266`, `Core/Src/freertos.c:285`, `Core/Src/freertos.c:304`, `Core/Src/freertos.c:323`, `tests/sil/mocks/cmsis_os2_impl.c:119`, `tests/sil/mocks/cmsis_os2_impl.c:126`, `tests/sil/mocks/cmsis_os2_impl.c:343`.

- La logica EV2.3/T11.8.9 y la emision `0x360`/`0x362` estan cubiertas en tests.
  Referencias: `tests/unit/test_control_logic.c:29`, `tests/unit/test_control_logic.c:137`, `Core/Src/test_integration.c:461`.

- El scheduler SIL ya no usa solo ticks sinteticos por helper.
  Ahora el runner avanza el tiempo, procesa CAN fisico mock y drena las tareas listas del RTOS mock en cada ms.
  Referencias: `tests/sil/sil_main.c:86`, `tests/sil/sil_main.c:91`, `tests/sil/mocks/cmsis_os2_impl.c:304`, `tests/sil/mocks/cmsis_os2_impl.c:343`.

### Que no demuestran

- No validan el firmware completo ni el bring-up real de placa.
  Aunque SIL ahora si compila y ejecuta `freertos.c`, sigue sin compilar ni recorrer `main.c`, `fdcan.c`, `tim.c`, `adc.c`, `gpio.c` o `stm32h7xx_it.c`.
  Referencias: `tests/sil/CMakeLists.txt:18`, `tests/sil/CMakeLists.txt:22`, `Core/Src/main.c:100`, `Core/Src/fdcan.c:32`, `Core/Src/tim.c:109`.

- La topologia de tareas en SIL aun no es 100% identica al firmware final.
  Se omite `IntegrationTestTask` a proposito para que el runner SIL no autoarranque la suite de integracion durante un ciclo funcional normal.
  Referencias: `Core/Src/freertos.c:208`, `Core/Src/freertos.c:211`.

- No prueban la secuencia real del inversor.
  El full-cycle inyecta `inv_state = 0x02` y aun asi pasa porque la FSM actual no mira `inv_state` para entrar en `RUN`.
  Referencias: `tests/sil/sil_can_simulator.c:20`, `tests/sil/sil_can_simulator.c:97`, `Core/Src/control.c:192`.

- Algunos tests validan rutas compat, no el legacy 1:1.
  Ejemplos:
  `S4.2` inyecta `0x101` como frame de 2 bytes LE.
  `S2.7` y el unit test de DC bus usan `0x100` como RX.
  Referencias: `Core/Src/test_integration.c:381`, `Core/Src/test_integration.c:503`, `tests/unit/test_can_parsing.c:38`.

- La telemetria testada es la `Telemetry_Build32` simple, no el `TelFrame` legacy ni `nrf24_tx32()`.
  Referencias: `tests/unit/test_telemetry.c:20`, `main_polling.c:2440`.

- `test_error_handling.c` no es prueba fiable de paridad 1:1.
  Espera limitaciones por tension, temperatura y faults, pero `Control_ComputeTorque()` no usa esos campos.
  Referencias: `tests/unit/test_error_handling.c:42`, `Core/Src/control.c:101`.

### Casos golden que faltan

1. Reproducir el mismo rastro de entradas en `main_polling.c` y en la version modular, y comparar salidas CAN y timing.
2. Casos `state 0/3/4/6/10/11/13` del inversor.
3. Cadencia exacta de `0x100` y `0x600` a 10 ms.
4. Telemetria legacy completa `0x600/0x610/0x620/0x630` con nRF24/UART.
5. Arranque real con ADC/GPIO y RTDS.
6. Bring-up real con filtros FDCAN, notificaciones e ISR de TIM16.

### Estado actual de ejecucion

Verificado localmente en SIL:
- `cmake --build build-sil`
- `.\build-sil\tests\sil\ecu08_sil.exe --test-integration`
- `.\build-sil\tests\sil\ecu08_sil.exe --test-full-cycle`

Resultado:
- `120/120` tests de integracion OK.
- `full-cycle` OK.

Limite importante:
- Los unit tests no se pudieron ejecutar aqui porque `tests/unit/CMakeLists.txt` descarga Unity desde GitHub y el entorno actual no tiene acceso de red.
- Referencia: `tests/unit/CMakeLists.txt:4`.

## 6. Plan de cierre

### Fase 1: cambios minimos para paridad funcional

1. Hacer que la FSM nueva dependa de `inv_state` exactamente como el legacy.
2. Portar estados `0/10/11/13`, `flag_r2d`, `flag_react` y la reactivacion.
3. Portar exacto el arranque: condicion de precarga, umbral de freno de arranque, separacion `precharge_button` y `start_button_act`.
4. Portar `v_celda_min` y decidir explicitamente si la migracion 1:1 preserva tambien el bug legacy de `torque_limitado`.

### Fase 2: cierre de perifericos, telemetria y hardware

1. Rehabilitar el bring-up real de FDCAN y TIM16.
2. Portar lectura fisica ADC/GPIO del arranque y RTDS.
3. Sustituir `Telemetry_Build32` por el `TelFrame` legacy y dar implementacion real a `Telemetry_Send32`.
4. Recuperar, si forma parte del 1:1 esperado, UART de diagnostico y ruta nRF24/SD.

### Fase 3: validacion final

1. Anadir tests golden contra `main_polling.c`.
2. Ejecutar HIL en placa con trazas reales de CAN.
3. Retirar rutas compat que hoy hacen pasar tests pero no son legacy 1:1.

## 7. Checklist accionable

1. Anadir a `app_inputs_t` los campos `precharge_button`, `start_button_act`, `flag_r2d` y `flag_react`.
2. Reescribir `Control_Step10ms()` para esperar `inv_state == 3` antes de `READY` y `inv_state == 4` antes de `RUN`.
3. Portar ramas `state 0/10/11/13` desde `main_polling.c`.
4. Portar la salida de precarga exacta `0x100/0x600` con su cadencia real.
5. Portar la limitacion por `v_celda_min` y documentar si se conserva el bug legacy.
6. Meter lectura real de freno y boton en runtime y usar el umbral de arranque legacy.
7. Implementar RTDS por GPIO durante el retardo de 2 s.
8. Portar `tel_build_packet()` y `tel_send_now()` al modulo nuevo y dar implementacion real a `Telemetry_Send32()`.
9. Anadir `HAL_FDCAN_ConfigFilter`, `HAL_FDCAN_Start`, `HAL_FDCAN_ActivateNotification` y `HAL_TIM_Base_Start_IT` al arranque real.
10. Crear tests golden que comparen la nueva ruta con `main_polling.c` usando la misma secuencia de entradas.
11. Separar o eliminar las rutas compat `0x100 RX`, `0x102` y `0x103` si el objetivo es 1:1 estricta.
12. Vendorizar Unity o anadir mirror local para ejecutar `tests/unit` sin red y limpiar los tests aspiracionales de `test_error_handling.c`.

## Referencias clave

- `main_polling.c:533`
- `main_polling.c:1738`
- `main_polling.c:1876`
- `main_polling.c:2041`
- `Core/Src/can.c:73`
- `Core/Src/control.c:101`
- `Core/Src/control.c:192`
- `Core/Src/freertos.c:193`
- `Core/Src/freertos.c:266`
- `Core/Src/freertos.c:395`
- `Core/Src/telemetry.c:4`
- `Core/Inc/app_state.h:8`
- `Core/Inc/VCU.h:97`
- `tests/sil/CMakeLists.txt:22`
- `tests/sil/CMakeLists.txt:71`
- `tests/sil/sil_main.c:70`
- `tests/sil/sil_main.c:86`
- `tests/sil/mocks/cmsis_os2_impl.c:343`
- `Core/Src/test_integration.c:398`
