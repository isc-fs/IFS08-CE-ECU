# Comparativa de Migracion 1:1 vs `main_polling.c`

Fecha: 2026-03-31

Objetivo: comparar el proyecto actual contra `main_polling.c`, tratando `main_polling.c` como referencia funcional legacy para una migracion 1:1.


## 1. Resumen ejecutivo

La migracion actual sigue siendo **parcial**, no 1:1, pero ya no esta en el mismo punto que al inicio del dia.

Estimacion actual de paridad funcional:
- **68-72%** si medimos parser CAN + control principal + secuencia cooperativa de arranque del inversor.
- Menos si incluimos telemetria legacy, IO fisica, bring-up real de hardware y validacion HIL.

Esta estimacion es una inferencia a partir del codigo y de la cobertura SIL disponible; no es una medida formal en placa.

Conclusion corta:
- El parser CAN legacy principal esta bien migrado.
- La FSM de arranque ya espera la secuencia real del inversor `3 -> 4 -> 6` en un modelo compatible con FreeRTOS.
- Las ramas legacy de runtime `0/3/4/6/10/11/13` ya existen en la logica cooperativa.
- EV2.3, T11.8.9 y la limitacion por `v_celda_min` ya estan portadas en `Control_ComputeTorque()`.
- La cadencia observable de TX en runtime ya esta alineada a 10 ms en la tarea de CAN TX.
- Aun asi, la telemetria legacy y la validacion en placa de IO fisica, RTDS y bring-up FDCAN/TIM siguen fuera de equivalencia completa.

### Si dejamos telemetria fuera por ahora

Los gaps principales que quedan respecto a `main_polling.c` son estos:

1. Validar en placa el puente de IO fisica (`START_FIL`, `S_BRAKE_FIL`, `APPS_1`, `APPS_2`, `RTDS`).
2. Cerrar la pauta exacta de `0x100/0x600` respecto al legacy.
3. Validar en placa el bring-up real de FDCAN/TIM ya portado.
4. Validar en placa que pedales y freno queden alimentados solo por la IO fisica, ya sin rutas CAN `0x101/0x102/0x103`.
5. Ejecutar golden/HIL para demostrar equivalencia temporal y de tramas.

Los 5 gaps mas importantes ahora son:

1. **La telemetria legacy no esta migrada.**
   El legacy rota `TelFrame` `0x600/0x610/0x620/0x630` y lo manda por `nrf24_tx32()` con logging UART.
   Hoy solo hay un buffer simple de 32 bytes y `Telemetry_Send32()` es `weak`/no-op en firmware real.
   Referencias: `main_polling.c:2326`, `main_polling.c:2440`, `Core/Src/telemetry.c:4`, `Core/Src/telemetry.c:33`.

2. **La IO fisica de arranque/freno ya esta puenteada, pero falta validarla en placa.**
   El legacy lee freno por ADC y boton por GPIO en la secuencia de arranque.
   La ruta nueva ya actualiza `g_in` desde ADC/GPIO reales mediante `io_signals.c`, pero el mapeo actual sigue dependiendo de la configuracion final de placa y del `.ioc`.
   Referencias: `main_polling.c:625`, `main_polling.c:635`, `Core/Src/io_signals.c:91`, `Core/Src/freertos.c:422`.

3. **Falta validar en placa el bring-up real de CAN/TIM.**
   El legacy configura filtros, arranca FDCAN, activa notificaciones y arranca TIM16.
   La ruta nueva ya hace `HAL_FDCAN_ConfigFilter()` en init y `HAL_FDCAN_Start()`, `HAL_FDCAN_ActivateNotification()` y `HAL_TIM_Base_Start_IT()` desde la `InitTask`, pero aun no esta validada en hardware real.
   Referencias: `main_polling.c:430`, `main_polling.c:438`, `main_polling.c:508`, `main_polling.c:1115`, `Core/Src/fdcan.c:32`, `Core/Src/fdcan.c:249`, `Core/Src/tim.c:228`, `Core/Src/freertos.c:433`.

4. **La salida RTDS ya esta portada a GPIO real, pero falta validarla en placa.**
   En el legacy el RTDS se activa durante 2 s por GPIO antes de entrar en la secuencia del inversor.
   La ruta nueva ya expone `rtds_active` desde la FSM y conmuta el GPIO fisico desde `io_signals.c`.
   Referencias: `main_polling.c:662`, `Core/Src/control.c:315`, `Core/Src/io_signals.c:129`, `Core/Src/freertos.c:436`.

5. **Falta validacion golden/HIL de la ruta nueva.**
   SIL ya cubre la secuencia nominal `3 -> 4 -> 6`, pero aun no hay comparacion de trazas contra `main_polling.c` ni validacion en placa.
   Referencias: `Core/Src/test_integration.c:447`, `Core/Src/test_integration.c:782`, `tests/sil/sil_main.c:279`, `tests/sil/sil_main.c:297`.

## 2. Mapa de migracion

| Bloque legacy en `main_polling.c` | Equivalente actual | Estado | Comentario |
|---|---|---|---|
| `HAL_FDCAN_RxFifo0Callback` monolitico | `CanRx_ParseAndUpdate()` + `Can_ISR_PushRxFifo0()` | Migrado | El parser principal activo `0x20/0x12C/0x461..0x466` esta portado y las rutas legacy de pedales por CAN ya se retiraron del runtime nuevo. Referencias: `main_polling.c:1738`, `Core/Src/can.c:73`. |
| Bucle de precarga `while (precarga_inv == 0 && inv_dc_bus_voltage < 300)` | `CTRL_ST_BOOT` y `CTRL_ST_WAIT_PRECHARGE_ACK` | Parcial | Ya replica la salida por `ok_precarga || inv_dc_bus_voltage >= 300`, pero no la pauta temporal exacta del legacy. Referencias: `main_polling.c:534`, `Core/Src/control.c:112`, `Core/Src/control.c:272`, `Core/Src/control.c:292`. |
| Espera boton + freno fisico | `CTRL_ST_WAIT_START_BRAKE` + `io_signals.c` | Parcial avanzado | Ya usa el umbral legacy `>900` y la ruta runtime alimenta `g_in` desde ADC/GPIO reales; falta validacion en placa y ajuste final del `.ioc` si algun pin cambia. Referencias: `main_polling.c:622`, `main_polling.c:645`, `Core/Src/control.c:6`, `Core/Src/io_signals.c:91`, `Core/Src/freertos.c:422`. |
| RTDS 2 s por GPIO | `CTRL_ST_R2D_DELAY` + `io_signals.c` | Parcial avanzado | Ya se mantiene el retardo de 2 s y se conmuta la salida fisica a traves de `rtds_active`; falta validarla en hardware real. Referencias: `main_polling.c:662`, `Core/Src/control.c:315`, `Core/Src/io_signals.c:129`, `Core/Src/freertos.c:436`. |
| Espera estados del inversor `3/4` y ramas `0/10/11/13` | FSM cooperativa en `Control_Step10ms()` | Migrado en logica | La FSM ya espera `inv_state == 3` antes de activar runtime y maneja `0/3/4/6/10/11/13` sin bloqueos. Referencias: `main_polling.c:682`, `main_polling.c:696`, `main_polling.c:2135`, `Core/Src/control.c:163`, `Core/Src/control.c:326`, `Core/Src/control.c:334`. |
| `setTorque()` | `Control_ComputeTorque()` | Parcial avanzado | EV2.3, T11.8.9 y `v_celda_min` ya estan portados; siguen fuera del calculo otras protecciones aspiracionales no equivalentes al legacy. Referencias: `main_polling.c:1876`, `main_polling.c:1995`, `Core/Src/control.c:137`, `Core/Src/control.c:216`, `Core/Src/control.c:251`. |
| ISR periodica TIM16 que emite CAN | `StartControlTask` + `StartCanTxTask` | Parcial avanzado | La logica existe, SIL ejecuta las tareas reales y TX ya va a 10 ms, pero sigue sin ser una ISR TIM16 real. Referencias: `main_polling.c:2041`, `Core/Src/freertos.c:307`, `Core/Src/freertos.c:311`. |
| `TelFrame` + `tel_build_packet()` + `nrf24_tx32()` | `Telemetry_Build32()` + `Telemetry_Send32()` | No migrado | Formato, cadencia y transporte no coinciden. Referencias: `main_polling.c:2326`, `main_polling.c:2440`, `Core/Src/telemetry.c:4`, `Core/Src/telemetry.c:33`. |
| ADC1/ADC2/DMA + GPIO start/brake | `io_signals.c` + `g_in` | Parcial avanzado | Ya existe adquisicion runtime equivalente basica sobre ADC3 y GPIO, desacoplada del `.ioc`; falta validarla en placa y decidir si hace falta DMA/filtrado adicional para paridad fina. Referencias: `main_polling.c:417`, `main_polling.c:625`, `main_polling.c:635`, `Core/Src/io_signals.c:24`, `Core/Src/io_signals.c:91`. |
| Bring-up FDCAN/TIM real | `fdcan.c` + `tim.c` + `AppRuntime_InitStep()` | Parcial avanzado | Ya configura filtros en init y hace `Start/Notification/Start_IT` desde la `InitTask`; falta validarlo en placa y decidir si la emision periodica final debe quedar solo en tareas RTOS o tambien colgar de TIM16. Referencias: `main_polling.c:430`, `main_polling.c:438`, `main_polling.c:508`, `main_polling.c:1115`, `Core/Src/fdcan.c:32`, `Core/Src/fdcan.c:249`, `Core/Src/tim.c:228`, `Core/Src/freertos.c:433`. |

## 3. Diferencias funcionales

### Alta

1. **El bring-up de hardware real ya esta portado en codigo, pero no validado en placa.**
   - Legacy:
     configura filtros, hace `HAL_FDCAN_Start`, activa `FDCAN_IT_RX_FIFO0_NEW_MESSAGE` y arranca TIM16.
   - Actual:
     `fdcan.c` configura filtros en `USER CODE`, y la `InitTask` arranca FDCAN y TIM16 cuando ya existen colas y mutex de FreeRTOS.
   - Riesgo:
     falta comprobar en placa que entren frames, que las IRQ encolen bien y que TIM16 no interfiera con la cadencia cooperativa actual.
   - Para igualarlo:
     validar en hardware real la secuencia ya portada y decidir el rol final de TIM16 en runtime.

2. **La telemetria legacy no existe en el runtime actual.**
   - Legacy:
     empaqueta `TelFrame` y lo transmite por nRF24 con apoyo UART.
   - Actual:
     solo empaqueta un buffer simple y `Telemetry_Send32()` es vacio en firmware real.
   - Riesgo:
     no hay equivalencia operativa ni observabilidad real.
   - Para igualarlo:
     portar `tel_build_packet()` y `tel_send_now()` como modo legacy.

3. **La IO fisica de arranque/freno ya esta portada en una ruta base, pero falta validarla en placa.**
   - Legacy:
     usa ADC y GPIO reales.
   - Actual:
     `io_signals.c` lee ADC3 y GPIO, y actualiza `g_in` antes del paso de control.
   - Riesgo:
     falta confirmar niveles electricos, sentido de la entrada de arranque y canal efectivo de `APPS_1`.
   - Para igualarlo:
     validar en placa y ajustar el mapeo final cuando el `.ioc` quede definitivo.

4. **La salida RTDS real ya esta cableada en firmware, pero falta validarla en placa.**
   - Legacy:
     activa RTDS por GPIO durante 2 s.
   - Actual:
     la FSM expone `rtds_active` y `io_signals.c` conmuta el GPIO asociado a `RTDS`.
   - Riesgo:
     falta comprobar polaridad, pin final y comportamiento audible en hardware.
   - Para igualarlo:
     validar la salida real y, si hace falta, corregir el mapeo final tras ajustar el `.ioc`.

5. **Falta validacion golden/HIL contra `main_polling.c`.**
   - Actual:
     la logica nueva ya cubre la secuencia nominal en SIL y la limitacion por `v_celda_min`.
   - Riesgo:
     podemos tener equivalencia "de intencion" sin demostrar equivalencia temporal y de frames en hardware.
   - Para igualarlo:
     anadir tests golden y ejecutar HIL con trazas reales.

### Media

1. **La pauta exacta de precarga no coincide aun con el legacy.**
   - Legacy:
     emite `0x100` y `0x600` tanto en el bucle de precarga como en la ISR de TIM16.
   - Actual:
     las manda solo desde `BOOT` y `WAIT_PRECHARGE_ACK`.
   - Riesgo:
     puede cambiar como ve el ACU la secuencia de precarga.
   - Para igualarlo:
     portar la pauta temporal exacta.

2. **La temporizacion ya mejoro, pero sigue sin ser una ISR real.**
   - Legacy:
     emite desde TIM16 cada 10 ms.
   - Actual:
     control y TX ya van a 10 ms en FreeRTOS, con RX a 5 ms.
   - Riesgo:
     sigue habiendo diferencia entre scheduler cooperativo y tick hardware/ISR.
   - Para igualarlo:
     validar en placa o mover la emision critica a la ruta hardware esperada.

3. **La telemetria tiene otra cadencia.**
   - Legacy:
     500 ms.
   - Actual:
     100 ms.
   - Riesgo:
     no rompe control, pero si integracion y ancho de banda.
   - Para igualarlo:
     restaurar 500 ms o introducir un modo `legacy_telemetry_period`.

4. **Sigue existiendo al menos una ruta compat CAN que no es 1:1 estricta.**
   - Ejemplo:
     `0x100` todavia se acepta como RX compat para `inv_dc_bus_voltage`.
   - Riesgo:
     algun test puede seguir pasando por compatibilidad y no por equivalencia estricta.
   - Para igualarlo:
     decidir mas adelante si `0x100` debe mantenerse o retirarse cuando exista cobertura golden.

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

## 4. Protocolo CAN comparado

| ID | Bus | Dir | Payload legacy esperado | Implementacion actual | Estado |
|---|---|---|---|---|---|
| `0x020` | ACU / FDCAN2 | RX | `data[0] == 0` => precarga OK | Igual en `Core/Src/can.c:79` | OK |
| `0x12C` | ACU / FDCAN2 | RX | `v_celda_min` big-endian | Igual en `Core/Src/can.c:110` | OK |
| `0x101` | DASH / FDCAN3 | RX | `S1` y `S2` juntos, big-endian, 4 bytes | Retirado del runtime nuevo; pedales pasan por ADC | Retirado |
| `0x102` | DASH / FDCAN3 | RX | Definido en `VCU.h`, pero no usado por el callback legacy principal | Retirado del runtime nuevo | Retirado |
| `0x103` | DASH / FDCAN3 | RX | En arranque legacy el freno se lee por ADC | Retirado del runtime nuevo; freno pasa por ADC | Retirado |
| `0x461` | INV / FDCAN1 | RX | `state` en `data[4] & 0xF`, `error` en `data[2]` | Igual en `Core/Src/can.c:115` | OK |
| `0x463` | INV / FDCAN1 | RX | RPM con sign-extend desde bytes `5..7` | Igual en `Core/Src/can.c:123` | OK |
| `0x464` | INV / FDCAN1 | RX | Temperaturas | Igual en `Core/Src/can.c:137` | OK |
| `0x465` | INV / FDCAN1 | RX | Velocidad/corriente LE bytes `2..5` | Igual en `Core/Src/can.c:146` | OK |
| `0x466` | INV / FDCAN1 | RX | Vdc LE bytes `2..3`, `dlc == 6` | Igual en `Core/Src/can.c:154` | OK |
| `0x100` | ACU / FDCAN2 | TX | Vdc al AMS, ext ID, LE, 2 bytes | Emitido desde control, pero aun aceptado como RX compat | Parcial |
| `0x600` | ACU / FDCAN2 | TX | Boton de precarga, ext ID, 2 bytes | Emitido en `BOOT`/`WAIT_PRECHARGE_ACK`; falta pauta exacta legacy | Parcial |
| `0x360` | INV / FDCAN1 | TX | Modo `0x01/0x04/0x06/0x13` segun estado | Ya emite `0x01/0x04/0x06/0x13` segun la FSM cooperativa | Parcial avanzado |
| `0x362` | INV / FDCAN1 | TX | Torque cero o comando legacy bytes `2..3` | Formato portado; ahora el torque ya sale limitado por `v_celda_min` | Parcial avanzado |

Nota:
- Los IDs `0x600/0x610/0x620/0x630` del legacy son IDs internos del `TelFrame` de telemetria por nRF24, no tramas CAN.

## 5. Estado de tests

### Que si demuestran equivalencia util

- El parser CAN principal y la tuberia mock `HAL -> ISR -> queue -> parser` funcionan en SIL.
- SIL compila y ejecuta `freertos.c`, no solo la logica pura.
- La topologia real de tareas queda cubierta por SIL.
- En SIL, `START_FIL`, `APPS_1`, `APPS_2` y `S_BRAKE_FIL` ya entran por la capa `io_signals.c` simulada, no por las tramas CAN retiradas.
- La FSM nominal del inversor ya se prueba con secuencia explicita `3 -> 4 -> 6`.
  Referencias: `Core/Src/test_integration.c:447`, `tests/sil/sil_main.c:279`, `tests/sil/sil_main.c:289`, `tests/sil/sil_main.c:297`.
- La logica EV2.3, T11.8.9 y la emision `0x360`/`0x362` estan cubiertas en tests.
- La limitacion por `v_celda_min` ya tiene cobertura en zona lineal y zona critica.
  Referencias: `Core/Src/test_integration.c:782`, `Core/Src/test_integration.c:786`.

### Que no demuestran

- No validan el firmware completo ni el bring-up real de placa.
- No prueban telemetria legacy `TelFrame` ni `nrf24_tx32()`.
- No validan en placa la IO fisica real de arranque/freno ni RTDS por GPIO.
- No existe aun comparacion golden de trazas contra `main_polling.c`.
- Las ramas de fault `10/11/13` existen en la logica, pero aun no tienen la misma profundidad de validacion que la secuencia nominal `3 -> 4 -> 6`.

### Casos golden que faltan

1. Reproducir el mismo rastro de entradas en `main_polling.c` y en la version modular, y comparar salidas CAN y timing.
2. Casos `state 0/10/11/13` del inversor con expectativas de salida exacta.
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
- `130/130` tests de integracion OK.
- `full-cycle` OK.

Limite importante:
- Los unit tests no se pudieron ejecutar aqui porque `tests/unit/CMakeLists.txt` descarga Unity desde GitHub y el entorno actual no tiene acceso de red.

## 6. Plan de cierre

### Fase 1: cerrar equivalencia funcional pendiente

1. Portar la pauta exacta de precarga `0x100/0x600`.
2. Validar RTDS por GPIO durante el retardo de 2 s.
3. Validar en placa la lectura real de freno, `START_FIL`, `APPS_1` y `APPS_2`.

### Fase 2: cerrar perifericos, telemetria y hardware

1. Validar en placa el bring-up real de FDCAN y TIM16 ya portado.
2. Sustituir `Telemetry_Build32` por el `TelFrame` legacy y dar implementacion real a `Telemetry_Send32`.
3. Recuperar, si forma parte del 1:1 esperado, UART de diagnostico y ruta nRF24/SD.

### Fase 3: validacion final

1. Anadir tests golden contra `main_polling.c`.
2. Ejecutar HIL en placa con trazas reales de CAN.
3. Endurecer tests y HIL para que la IO fisica sea la unica fuente de pedales y freno.

## 7. Checklist accionable

### Completado recientemente

1. Hacer que la FSM nueva dependa de `inv_state` y espere `3 -> 4 -> 6`.
2. Portar ramas `state 0/10/11/13` al modelo cooperativo de FreeRTOS.
3. Portar la condicion de salida de precarga por `inv_dc_bus_voltage >= 300`.
4. Separar el umbral de arranque y usar `s_freno > 900`.
5. Portar la limitacion por `v_celda_min` y hacerla efectiva en el runtime nuevo.
6. Alinear la tarea de CAN TX a 10 ms.
7. Portar `HAL_FDCAN_ConfigFilter`, `HAL_FDCAN_Start`, `HAL_FDCAN_ActivateNotification` y `HAL_TIM_Base_Start_IT` dentro de zonas protegidas.
8. Portar un puente fisico `ADC/GPIO -> g_in` y `rtds_active -> RTDS` sin tocar el `.ioc`.

### Pendiente

1. Portar la salida de precarga exacta `0x100/0x600` con su cadencia legacy.
2. Validar en placa la lectura real de freno, `START_FIL`, `APPS_1` y `APPS_2`.
3. Validar en placa la salida RTDS y su polaridad.
4. Portar `tel_build_packet()` y `tel_send_now()` al modulo nuevo y dar implementacion real a `Telemetry_Send32()`.
5. Crear tests golden que comparen la nueva ruta con `main_polling.c` usando la misma secuencia de entradas.
6. Revisar si `0x100 RX` debe seguir existiendo como compat o retirarse cuando haya cobertura golden.
7. Vendorizar Unity o anadir mirror local para ejecutar `tests/unit` sin red.

## Referencias clave

- `main_polling.c:533`
- `main_polling.c:682`
- `main_polling.c:1876`
- `main_polling.c:1995`
- `main_polling.c:2041`
- `Core/Src/can.c:73`
- `Core/Src/control.c:112`
- `Core/Src/control.c:137`
- `Core/Src/control.c:163`
- `Core/Src/control.c:251`
- `Core/Src/control.c:326`
- `Core/Src/freertos.c:307`
- `Core/Src/app_state.c:11`
- `Core/Src/telemetry.c:4`
- `Core/Src/main.c:102`
- `Core/Src/fdcan.c:32`
- `Core/Src/tim.c:110`
- `Core/Src/test_integration.c:447`
- `Core/Src/test_integration.c:782`
- `Core/Src/test_integration.c:786`
- `tests/sil/sil_main.c:279`
- `tests/sil/sil_main.c:297`
