# Mapa de pines ECU08

Mapa de referencia entre:

- pin del conector de la PCB inferior,
- senal funcional en placa,
- pin fisico del STM32,
- y nombre/configuracion observados en `ECU08 NSIL.ioc`.

Este documento no es solo un volcado de CubeMX. Tambien fija la asociacion
funcional que vamos a usar en el proyecto, aunque en el `.ioc` algunos pines
aparezcan con alias genericos como `D1`, `D2`, `A1` o `A2`.

## Fuentes usadas

- `ECU08 NSIL.ioc`
- `Core/Inc/main.h`
- `Core/Src/gpio.c`
- descripcion de ruteo micro -> placa facilitada por el usuario

## Regla de lectura

- La columna "Senal funcional" representa la funcion real en placa.
- La columna "Label CubeMX" refleja el `GPIO_Label` del `.ioc`, cuando existe.
- La columna "Signal `.ioc`" refleja la senal exacta asignada en CubeMX.
- Si una senal funcional no coincide con el nombre de CubeMX, prevalece este
  documento como mapa de asociacion funcional para firmware y migracion.

## Lado izquierdo del conector

| Pin | Senal funcional | Tipo | Pin STM32 | Label CubeMX | Signal `.ioc` | Modo `.ioc` | Estado |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | GND | Alimentacion | - | - | - | - | OK |
| 2 | GND | Alimentacion | - | - | - | - | OK |
| 3 | S_TEMP_REFRI | Funcional | PD5 | `DS18B20_REFRI` | `GPIO_Input` | - | OK |
| 4 | GPS_TX | Funcional | PG11 | - | `USART10_RX` | `Asynchronous` | OK |
| 5 | GPS_RX | Funcional | PG12 | - | `USART10_TX` | `Asynchronous` | OK |
| 6 | RTDS | Funcional | PB4 | `D1` | `GPIO_Output` | - | OK |
| 7 | START_FIL | Funcional | PB5 | `D2` | `GPIO_Output` | - | OK |
| 8 | GPIO3 | Generico | PB6 | `D3` | `GPIO_Output` | - | OK |
| 9 | GPIO4 | Generico | PB7 | `D4` | `GPIO_Output` | - | OK |
| 10 | GPIO5 | Generico | PB8 | `D5` | `GPIO_Output` | - | OK |
| 11 | GPIO6 | Generico | PB9 | `D6` | `GPIO_Output` | - | OK |
| 12 | S_BRAKE_FIL | Funcional | PF7 | `A1` | `ADC3_INP3` | `IN3-Single-Ended` | OK |
| 13 | APPS_1 | Funcional | PF8 | `A2` | `SharedAnalog_PF8` | - | Revisar ADC |
| 14 | APPS_2 | Funcional | PF9 | `A3` | `ADC3_INP2` | `IN2-Single-Ended` | OK |
| 15 | GPIO10 | Generico | PF10 | `A4` | `ADC3_INP6` | `IN6-Single-Ended` | OK |
| 16 | GPIO11 | Generico | PC0 | `A5` | `ADCx_INP10` | - | OK |
| 17 | GPIO12 | Generico | PC1 | `A6` | `ADCx_INP11` | - | OK |
| 18 | GPIO13 | Generico | PC2 | - | - | - | No aparece en `.ioc` |

## Lado derecho del conector

| Pin | Senal funcional | Tipo | Pin STM32 | Label CubeMX | Signal `.ioc` | Modo `.ioc` | Estado |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 19 | GND | Alimentacion | - | - | - | - | OK |
| 20 | GND | Alimentacion | - | - | - | - | OK |
| 21 | CAN_L_INV | Funcional | PD0 | - | `FDCAN1_RX` | `FDCAN_Activate` | OK |
| 22 | CAN_H_INV | Funcional | PD1 | - | `FDCAN1_TX` | `FDCAN_Activate` | OK |
| 23 | CAN_H_TEL | Funcional | PB13 | - | `FDCAN2_TX` | `FDCAN_Activate` | OK |
| 24 | CAN_L_TEL | Funcional | PB12 | - | `FDCAN2_RX` | `FDCAN_Activate` | OK |
| 25 | CAN_H_SENS | Funcional | PG9 | - | `FDCAN3_TX` | `FDCAN_Activate` | OK |
| 26 | CAN_L_SENS | Funcional | PG10 | - | `FDCAN3_RX` | `FDCAN_Activate` | OK |
| 27 | NRF_CS | Funcional | PB0 | `NRF24_CS` | `GPIO_Output` | - | OK |
| 28 | NRF24_CE | Funcional | PC5 | `NRF24_CE` | `GPIO_Output` | - | OK |
| 29 | NRF24_IRQ | Funcional | PC4 | `NRF24_IRQ` | `GPIO_Input` | - | OK |
| 30 | SPI1_MOSI | Funcional | PA7 | - | `SPI1_MOSI` | `Full_Duplex_Master` | OK |
| 31 | SPI1_MISO | Funcional | PA6 | - | `SPI1_MISO` | `Full_Duplex_Master` | OK |
| 32 | SPI1_SCK | Funcional | PA5 | - | `SPI1_SCK` | `Full_Duplex_Master` | OK |
| 33 | +3V3 | Alimentacion | - | - | - | - | OK |
| 34 | +3V3 | Alimentacion | - | - | - | - | OK |
| 35 | +5V | Alimentacion | - | - | - | - | OK |
| 36 | +5V | Alimentacion | - | - | - | - | OK |

## Asociaciones funcionales clave

Estas asociaciones son las que conviene usar cuando hablemos de firmware:

- `RTDS` -> `PB4` -> label CubeMX `D1`
- `START_FIL` -> `PB5` -> label CubeMX `D2`
- `S_BRAKE_FIL` -> `PF7` -> label CubeMX `A1`
- `APPS_1` -> `PF8` -> label CubeMX `A2`
- `APPS_2` -> `PF9` -> label CubeMX `A3`
- `GPIO10` -> `PF10` -> label CubeMX `A4`
- `GPIO11` -> `PC0` -> label CubeMX `A5`
- `GPIO12` -> `PC1` -> label CubeMX `A6`

## Hallazgos

### 1. `PC2 / GPIO13` no esta configurado en el `.ioc`

- No aparece en `ECU08 NSIL.ioc`.
- No aparece definido en `Core/Inc/main.h`.
- No aparece inicializado en `Core/Src/gpio.c`.

Implicacion:

- A dia de hoy no hay configuracion firmware asociada a esa linea.

### 2. `PF8 / APPS_1` esta en modo analogico compartido

En el `.ioc`:

- `PF8.Signal = SharedAnalog_PF8`
- `SH.SharedAnalog_PF8.0 = ADC3_INN3`
- `SH.SharedAnalog_PF8.1 = ADC3_INP7,IN7-Single-Ended`

Implicacion:

- El pin si esta asociado a recursos ADC en CubeMX, pero no aparece con un
  `Signal` simple del tipo `ADC3_INPx`.
- Conviene revisar en firmware si la configuracion runtime de ADC realmente
  esta usando ese canal para `APPS_1`.

### 3. Convencion UART

El naming de placa queda asi:

- `GPS_TX` -> `PG11` -> `USART10_RX`
- `GPS_RX` -> `PG12` -> `USART10_TX`

Esto es electricamente coherente y corresponde al cruce habitual TX/RX entre
dispositivos.

### 4. Diferencia entre nombre funcional y nombre CubeMX

En el proyecto actual, CubeMX usa etiquetas genericas:

- `D1..D6` para PB4..PB9
- `A1..A6` para PF7, PF8, PF9, PF10, PC0, PC1

Para la migracion y el control conviene pensar en sus nombres funcionales de
placa (`RTDS`, `START_FIL`, `S_BRAKE_FIL`, `APPS_1`, `APPS_2`, etc.), no en
los alias genericos.

## Resumen operativo

El ruteo funcional relevante queda fijado asi:

- `RTDS` en `PB4`
- `START_FIL` en `PB5`
- `S_BRAKE_FIL` en `PF7`
- `APPS_1` en `PF8`
- `APPS_2` en `PF9`
- `CAN_INV` en `PD0/PD1`
- `CAN_TEL` en `PB12/PB13`
- `CAN_SENS` en `PG10/PG9`
- `NRF24` en `PB0`, `PC5`, `PC4`
- `SPI1` en `PA5`, `PA6`, `PA7`

## Pendiente de confirmacion final

- `PC2 / GPIO13`
- uso ADC efectivo de `PF8 / APPS_1` en firmware

