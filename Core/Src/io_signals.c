#include "io_signals.h"

#if (IO_DRIVER_INPUTS_SOURCE != IO_DRIVER_INPUTS_SOURCE_LOCAL_IO)
#error "Production firmware currently supports only local ADC/GPIO driver inputs."
#endif

#ifndef SIL_BUILD
#include "adc.h"
#include "app_state.h"
#include "main.h"

/* Current functional-to-generated mapping:
 * RTDS      -> PB4 -> D1
 * START_FIL -> PB5 -> D2
 * S_BRAKE   -> PF7 -> A1 -> ADC3_INP3
 * APPS_1    -> PF8 -> A2 -> ADC3_INP7
 * APPS_2    -> PF9 -> A3 -> ADC3_INP2
 */
#define IO_RTDS_GPIO_Port     D1_GPIO_Port
#define IO_RTDS_Pin           D1_Pin
#define IO_START_GPIO_Port    D2_GPIO_Port
#define IO_START_Pin          D2_Pin

#define IO_ADC_TIMEOUT_MS     2u
#define IO_ADC_BRAKE_CH       ADC_CHANNEL_3
#define IO_ADC_APPS1_CH       ADC_CHANNEL_7
#define IO_ADC_APPS2_CH       ADC_CHANNEL_2

/* Debounce: require this many consecutive identical samples (each 10ms) */
#define IO_DEBOUNCE_SAMPLES   5u   /* 5 × 10ms = 50ms */

static uint8_t s_start_debounced  = 0u;
static uint8_t s_start_debounce_cnt = 0u;

static HAL_StatusTypeDef io_adc_select_channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC3_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSign = ADC3_OFFSET_SIGN_NEGATIVE;

  return HAL_ADC_ConfigChannel(&hadc3, &sConfig);
}

static HAL_StatusTypeDef io_adc_read_channel(uint32_t channel, uint16_t *value)
{
  if (!value) return HAL_ERROR;

  if (io_adc_select_channel(channel) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_ADC_Start(&hadc3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_ADC_PollForConversion(&hadc3, IO_ADC_TIMEOUT_MS) != HAL_OK)
  {
    (void)HAL_ADC_Stop(&hadc3);
    return HAL_ERROR;
  }

  *value = (uint16_t)HAL_ADC_GetValue(&hadc3);

  if (HAL_ADC_Stop(&hadc3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

void IoSignals_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Force the semantic meaning of RTDS and START_FIL at runtime without
   * relying on the current CubeMX labels or direction.
   */
  HAL_GPIO_WritePin(IO_RTDS_GPIO_Port, IO_RTDS_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = IO_RTDS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IO_RTDS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = IO_START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(IO_START_GPIO_Port, &GPIO_InitStruct);
}

void IoSignals_InputStep(void)
{
  uint16_t brake_raw = 0u;
  uint16_t apps1_raw = 0u;
  uint16_t apps2_raw = 0u;
  uint8_t start_pressed = 0u;

  {
    uint8_t raw = (HAL_GPIO_ReadPin(IO_START_GPIO_Port, IO_START_Pin) == GPIO_PIN_SET) ? 1u : 0u;

    if (raw == s_start_debounced)
    {
      s_start_debounce_cnt = 0u;
    }
    else
    {
      s_start_debounce_cnt++;
      if (s_start_debounce_cnt >= IO_DEBOUNCE_SAMPLES)
      {
        s_start_debounced    = raw;
        s_start_debounce_cnt = 0u;
      }
    }
    start_pressed = s_start_debounced;
  }

  /* Production definition: driver inputs come from local IO, not CAN3.
   * On ADC error, default to 0 (safe fail: no brake, no throttle) instead of
   * keeping a stale value that could mask a sensor fault. */
  if (io_adc_read_channel(IO_ADC_BRAKE_CH, &brake_raw) != HAL_OK)
  {
    brake_raw = 0u;
  }

  if (io_adc_read_channel(IO_ADC_APPS1_CH, &apps1_raw) != HAL_OK)
  {
    apps1_raw = 0u;
  }

  if (io_adc_read_channel(IO_ADC_APPS2_CH, &apps2_raw) != HAL_OK)
  {
    apps2_raw = 0u;
  }

  if (g_inMutex) {
    (void)osMutexAcquire(g_inMutex, osWaitForever);
  }

  g_in.boton_arranque = start_pressed;
  g_in.s_freno = brake_raw;
  g_in.s1_aceleracion = apps1_raw;
  g_in.s2_aceleracion = apps2_raw;

  if (g_inMutex) {
    (void)osMutexRelease(g_inMutex);
  }
}

void IoSignals_ApplyOutputs(const control_out_t *out)
{
  GPIO_PinState level = GPIO_PIN_RESET;

  if (out && out->rtds_active)
  {
    level = GPIO_PIN_SET;
  }

  HAL_GPIO_WritePin(IO_RTDS_GPIO_Port, IO_RTDS_Pin, level);
}

#else

#include "app_state.h"
#include "sil_can_simulator.h"
#include "sil_hal_mocks.h"

#define SIL_IO_RTDS_PIN        1u

void IoSignals_Init(void)
{
  SIL_GPIO_Write(SIL_IO_RTDS_PIN, 0);
}

void IoSignals_InputStep(void)
{
  uint8_t start_pressed = SIL_IO_GetStartButton();
  uint16_t brake_raw = SIL_IO_GetBrakeRaw();
  uint16_t apps1_raw = SIL_IO_GetApps1Raw();
  uint16_t apps2_raw = SIL_IO_GetApps2Raw();

  /* SIL mirrors the production architecture: local driver inputs only. */
  if (g_inMutex) {
    (void)osMutexAcquire(g_inMutex, osWaitForever);
  }

  g_in.boton_arranque = start_pressed;
  g_in.s_freno = brake_raw;
  g_in.s1_aceleracion = apps1_raw;
  g_in.s2_aceleracion = apps2_raw;

  if (g_inMutex) {
    (void)osMutexRelease(g_inMutex);
  }
}

void IoSignals_ApplyOutputs(const control_out_t *out)
{
  SIL_GPIO_Write(SIL_IO_RTDS_PIN, (out && out->rtds_active) ? 1 : 0);
}

#endif
