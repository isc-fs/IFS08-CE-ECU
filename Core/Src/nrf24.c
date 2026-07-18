#include "telemetry.h"

#if !defined(SIL_BUILD) && !defined(UNIT_TEST)

#include "diag.h"
#include "main.h"
#include "spi.h"

#include <string.h>

/* nRF24 transport backend for telemetry payloads and optional SD mirroring. */

extern SPI_HandleTypeDef hspi1;

#ifndef APP_SD_USE_FATFS
#define APP_SD_USE_FATFS 0
#endif

#define APP_NRF24_ECU_CHANNEL 0x4Cu
#define APP_NRF24_ECU_ADDR_LEN 5u
#define APP_NRF24_ECU_PAYLOAD_SIZE 32u
#define APP_NRF24_ECU_TX_TIMEOUT_MS 40u

static const uint8_t s_nrf24_addr[APP_NRF24_ECU_ADDR_LEN] = {'E', 'C', 'U', '0', '1'};

static uint8_t s_nrf24_ready;
static uint8_t s_nrf24_diag_reported;
static uint8_t s_sd_diag_reported;
static uint32_t s_nrf24_tx_ok_count;
static uint32_t s_nrf24_uart_trace_count;

#define NRF24_CMD_R_REGISTER   0x00u
#define NRF24_CMD_W_REGISTER   0x20u
#define NRF24_CMD_W_TX_PAYLOAD 0xA0u
#define NRF24_CMD_FLUSH_TX     0xE1u
#define NRF24_CMD_NOP          0xFFu

#define NRF24_REG_CONFIG       0x00u
#define NRF24_REG_EN_AA        0x01u
#define NRF24_REG_EN_RXADDR    0x02u
#define NRF24_REG_SETUP_AW     0x03u
#define NRF24_REG_SETUP_RETR   0x04u
#define NRF24_REG_RF_CH        0x05u
#define NRF24_REG_RF_SETUP     0x06u
#define NRF24_REG_STATUS       0x07u
#define NRF24_REG_RX_ADDR_P0   0x0Au
#define NRF24_REG_TX_ADDR      0x10u
#define NRF24_REG_RX_PW_P0     0x11u
#define NRF24_REG_FIFO_STATUS  0x17u
#define NRF24_REG_DYNPD        0x1Cu
#define NRF24_REG_FEATURE      0x1Du

#define NRF24_CONFIG_EN_CRC    0x08u
#define NRF24_CONFIG_PWR_UP    0x02u
#define NRF24_CONFIG_PRIM_RX   0x01u

#define NRF24_STATUS_RX_DR     0x40u
#define NRF24_STATUS_TX_DS     0x20u
#define NRF24_STATUS_MAX_RT    0x10u

#define NRF24_FIFO_TX_FULL     0x20u

static void nrf24_csn_low(void)
{
  HAL_GPIO_WritePin(NRF24_CS_GPIO_Port, NRF24_CS_Pin, GPIO_PIN_RESET);
}

static void nrf24_csn_high(void)
{
  HAL_GPIO_WritePin(NRF24_CS_GPIO_Port, NRF24_CS_Pin, GPIO_PIN_SET);
}

static void nrf24_ce_low(void)
{
  HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_RESET);
}

static void nrf24_ce_high(void)
{
  HAL_GPIO_WritePin(NRF24_CE_GPIO_Port, NRF24_CE_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef nrf24_write_reg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = { (uint8_t)(NRF24_CMD_W_REGISTER | (reg & 0x1Fu)), value };
  uint8_t rx[2] = {0u, 0u};
  HAL_StatusTypeDef hal_status;

  nrf24_csn_low();
  hal_status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2u, 100u);
  nrf24_csn_high();
  return hal_status;
}

static HAL_StatusTypeDef nrf24_read_reg(uint8_t reg, uint8_t *value)
{
  uint8_t tx[2] = { (uint8_t)(NRF24_CMD_R_REGISTER | (reg & 0x1Fu)), NRF24_CMD_NOP };
  uint8_t rx[2] = {0u, 0u};
  HAL_StatusTypeDef hal_status;

  if (!value) return HAL_ERROR;

  nrf24_csn_low();
  hal_status = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2u, 100u);
  nrf24_csn_high();
  *value = rx[1];
  return hal_status;
}

static void nrf24_log_probe(const char *stage)
{
  uint8_t cfg = 0xFFu;
  uint8_t status = 0xFFu;
  uint8_t rf_ch = 0xFFu;
  uint8_t fifo = 0xFFu;
  HAL_StatusTypeDef cfg_ok;
  HAL_StatusTypeDef status_ok;
  HAL_StatusTypeDef ch_ok;
  HAL_StatusTypeDef fifo_ok;
  GPIO_PinState irq_state = HAL_GPIO_ReadPin(NRF24_IRQ_GPIO_Port, NRF24_IRQ_Pin);

  cfg_ok = nrf24_read_reg(NRF24_REG_CONFIG, &cfg);
  status_ok = nrf24_read_reg(NRF24_REG_STATUS, &status);
  ch_ok = nrf24_read_reg(NRF24_REG_RF_CH, &rf_ch);
  fifo_ok = nrf24_read_reg(NRF24_REG_FIFO_STATUS, &fifo);

  Diag_Log("NRF24 %s: irq=%u cfg=%02X(%u) st=%02X(%u) ch=%02X(%u) fifo=%02X(%u)",
           stage ? stage : "probe",
           (unsigned)((irq_state == GPIO_PIN_SET) ? 1u : 0u),
           (unsigned)cfg, (unsigned)(cfg_ok == HAL_OK),
           (unsigned)status, (unsigned)(status_ok == HAL_OK),
           (unsigned)rf_ch, (unsigned)(ch_ok == HAL_OK),
           (unsigned)fifo, (unsigned)(fifo_ok == HAL_OK));
}

static HAL_StatusTypeDef nrf24_write_buf(uint8_t reg, const uint8_t *data, uint8_t len)
{
  uint8_t cmd = (uint8_t)(NRF24_CMD_W_REGISTER | (reg & 0x1Fu));
  HAL_StatusTypeDef hal_status = HAL_OK;

  if (!data || len == 0u) return HAL_ERROR;

  nrf24_csn_low();
  hal_status = HAL_SPI_Transmit(&hspi1, &cmd, 1u, 100u);
  if (hal_status == HAL_OK)
  {
    hal_status = HAL_SPI_Transmit(&hspi1, (uint8_t *)data, len, 100u);
  }
  nrf24_csn_high();
  return hal_status;
}

static HAL_StatusTypeDef nrf24_flush_tx(void)
{
  uint8_t cmd = NRF24_CMD_FLUSH_TX;
  HAL_StatusTypeDef hal_status;

  nrf24_csn_low();
  hal_status = HAL_SPI_Transmit(&hspi1, &cmd, 1u, 100u);
  nrf24_csn_high();
  return hal_status;
}

static HAL_StatusTypeDef nrf24_wait_tx_complete(uint32_t timeout_ms, uint8_t *status_out)
{
  uint32_t start_tick = HAL_GetTick();
  uint8_t status = 0u;

  do
  {
    if (nrf24_read_reg(NRF24_REG_STATUS, &status) != HAL_OK)
    {
      return HAL_ERROR;
    }

    if ((status & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)) != 0u)
    {
      if (status_out)
      {
        *status_out = status;
      }
      return HAL_OK;
    }

    if (HAL_GPIO_ReadPin(NRF24_IRQ_GPIO_Port, NRF24_IRQ_Pin) == GPIO_PIN_RESET)
    {
      if (nrf24_read_reg(NRF24_REG_STATUS, &status) != HAL_OK)
      {
        return HAL_ERROR;
      }

      if (status_out)
      {
        *status_out = status;
      }

      if ((status & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)) != 0u)
      {
        return HAL_OK;
      }
    }
  } while ((HAL_GetTick() - start_tick) < timeout_ms);

  if (status_out)
  {
    *status_out = status;
  }

  return HAL_TIMEOUT;
}

static void nrf24_init_once(void)
{
  uint8_t cfg = 0u;
  uint8_t status = 0u;

  if (s_nrf24_ready) return;

  nrf24_ce_low();
  nrf24_csn_high();
  HAL_Delay(5u);

  if (nrf24_write_reg(NRF24_REG_CONFIG, (uint8_t)(NRF24_CONFIG_EN_CRC | NRF24_CONFIG_PWR_UP)) != HAL_OK)
  {
    if (!s_nrf24_diag_reported)
    {
      Diag_Log("NRF24 init SPI write failed at CONFIG");
      nrf24_log_probe("init-fail-config");
      s_nrf24_diag_reported = 1u;
    }
    return;
  }
  if (nrf24_write_reg(NRF24_REG_EN_AA, 0x00u) != HAL_OK)
  {
    if (!s_nrf24_diag_reported)
    {
      Diag_Log("NRF24 init SPI write failed at EN_AA");
      nrf24_log_probe("init-fail-en_aa");
      s_nrf24_diag_reported = 1u;
    }
    return;
  }
  if (nrf24_write_reg(NRF24_REG_EN_RXADDR, 0x01u) != HAL_OK)
  {
    if (!s_nrf24_diag_reported)
    {
      Diag_Log("NRF24 init SPI write failed at EN_RXADDR");
      nrf24_log_probe("init-fail-en_rxaddr");
      s_nrf24_diag_reported = 1u;
    }
    return;
  }
  if (nrf24_write_reg(NRF24_REG_SETUP_AW, 0x03u) != HAL_OK)
  {
    if (!s_nrf24_diag_reported)
    {
      Diag_Log("NRF24 init SPI write failed at SETUP_AW");
      nrf24_log_probe("init-fail-setup_aw");
      s_nrf24_diag_reported = 1u;
    }
    return;
  }
  if (nrf24_write_reg(NRF24_REG_SETUP_RETR, 0x00u) != HAL_OK)
  {
    if (!s_nrf24_diag_reported)
    {
      Diag_Log("NRF24 init SPI write failed at SETUP_RETR");
      nrf24_log_probe("init-fail-setup_retr");
      s_nrf24_diag_reported = 1u;
    }
    return;
  }
  if (nrf24_write_reg(NRF24_REG_RF_CH, APP_NRF24_ECU_CHANNEL) != HAL_OK)
  {
    if (!s_nrf24_diag_reported)
    {
      Diag_Log("NRF24 init SPI write failed at RF_CH");
      nrf24_log_probe("init-fail-rf_ch");
      s_nrf24_diag_reported = 1u;
    }
    return;
  }
  if (nrf24_write_reg(NRF24_REG_RF_SETUP, 0x06u) != HAL_OK)
  {
    if (!s_nrf24_diag_reported)
    {
      Diag_Log("NRF24 init SPI write failed at RF_SETUP");
      nrf24_log_probe("init-fail-rf_setup");
      s_nrf24_diag_reported = 1u;
    }
    return;
  }
  if (nrf24_write_reg(NRF24_REG_DYNPD, 0x00u) != HAL_OK)
  {
    if (!s_nrf24_diag_reported)
    {
      Diag_Log("NRF24 init SPI write failed at DYNPD");
      nrf24_log_probe("init-fail-dynpd");
      s_nrf24_diag_reported = 1u;
    }
    return;
  }
  if (nrf24_write_reg(NRF24_REG_FEATURE, 0x00u) != HAL_OK)
  {
    if (!s_nrf24_diag_reported)
    {
      Diag_Log("NRF24 init SPI write failed at FEATURE");
      nrf24_log_probe("init-fail-feature");
      s_nrf24_diag_reported = 1u;
    }
    return;
  }
  if (nrf24_write_reg(NRF24_REG_RX_PW_P0, APP_NRF24_ECU_PAYLOAD_SIZE) != HAL_OK)
  {
    if (!s_nrf24_diag_reported)
    {
      Diag_Log("NRF24 init SPI write failed at RX_PW_P0");
      nrf24_log_probe("init-fail-rx_pw_p0");
      s_nrf24_diag_reported = 1u;
    }
    return;
  }
  if (nrf24_write_buf(NRF24_REG_TX_ADDR, s_nrf24_addr, sizeof(s_nrf24_addr)) != HAL_OK)
  {
    if (!s_nrf24_diag_reported)
    {
      Diag_Log("NRF24 init SPI write failed at TX_ADDR");
      nrf24_log_probe("init-fail-tx_addr");
      s_nrf24_diag_reported = 1u;
    }
    return;
  }
  if (nrf24_write_buf(NRF24_REG_RX_ADDR_P0, s_nrf24_addr, sizeof(s_nrf24_addr)) != HAL_OK)
  {
    if (!s_nrf24_diag_reported)
    {
      Diag_Log("NRF24 init SPI write failed at RX_ADDR_P0");
      nrf24_log_probe("init-fail-rx_addr_p0");
      s_nrf24_diag_reported = 1u;
    }
    return;
  }
  if (nrf24_write_reg(NRF24_REG_STATUS, (uint8_t)(NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)) != HAL_OK)
  {
    if (!s_nrf24_diag_reported)
    {
      Diag_Log("NRF24 init SPI write failed at STATUS");
      nrf24_log_probe("init-fail-status");
      s_nrf24_diag_reported = 1u;
    }
    return;
  }
  if (nrf24_flush_tx() != HAL_OK)
  {
    if (!s_nrf24_diag_reported)
    {
      Diag_Log("NRF24 init SPI write failed at FLUSH_TX");
      nrf24_log_probe("init-fail-flush_tx");
      s_nrf24_diag_reported = 1u;
    }
    return;
  }
  HAL_Delay(5u);

  if (nrf24_read_reg(NRF24_REG_CONFIG, &cfg) == HAL_OK &&
      nrf24_read_reg(NRF24_REG_STATUS, &status) == HAL_OK &&
      (cfg & (NRF24_CONFIG_EN_CRC | NRF24_CONFIG_PWR_UP | NRF24_CONFIG_PRIM_RX)) ==
      (NRF24_CONFIG_EN_CRC | NRF24_CONFIG_PWR_UP))
  {
    s_nrf24_ready = 1u;
    Diag_Log("NRF24 init OK: cfg=%02X status=%02X ch=%02X addr=ECU01",
             (unsigned)cfg, (unsigned)status, (unsigned)APP_NRF24_ECU_CHANNEL);
  }
  else if (!s_nrf24_diag_reported)
  {
    Diag_Log("NRF24 init verify failed: cfg=%02X expected_mask=%02X",
             (unsigned)cfg,
             (unsigned)(NRF24_CONFIG_EN_CRC | NRF24_CONFIG_PWR_UP));
    nrf24_log_probe("init-verify-fail");
    s_nrf24_diag_reported = 1u;
  }
}

static uint8_t nrf24_tx32(const uint8_t payload[32])
{
  uint8_t cmd = NRF24_CMD_W_TX_PAYLOAD;
  uint8_t status = 0u;
  uint8_t fifo_status = 0u;
  HAL_StatusTypeDef hal_status;

  if (!payload) return 0u;

  if (nrf24_write_reg(NRF24_REG_STATUS,
                      (uint8_t)(NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT)) != HAL_OK ||
      nrf24_flush_tx() != HAL_OK ||
      nrf24_read_reg(NRF24_REG_FIFO_STATUS, &fifo_status) != HAL_OK)
  {
    return 0u;
  }

  if ((fifo_status & NRF24_FIFO_TX_FULL) != 0u)
  {
    return 0u;
  }

  nrf24_csn_low();
  hal_status = HAL_SPI_Transmit(&hspi1, &cmd, 1u, 100u);
  if (hal_status == HAL_OK)
  {
    hal_status = HAL_SPI_Transmit(&hspi1, (uint8_t *)payload, APP_NRF24_ECU_PAYLOAD_SIZE, 100u);
  }
  nrf24_csn_high();
  if (hal_status != HAL_OK)
  {
    return 0u;
  }

  nrf24_ce_high();
  for (volatile uint32_t i = 0u; i < 800u; ++i)
  {
    __NOP();
  }
  nrf24_ce_low();

  hal_status = nrf24_wait_tx_complete(APP_NRF24_ECU_TX_TIMEOUT_MS, &status);
  (void)nrf24_write_reg(NRF24_REG_STATUS,
                        (uint8_t)(NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT));

  if (hal_status != HAL_OK)
  {
    (void)nrf24_flush_tx();
    return 0u;
  }

  if ((status & NRF24_STATUS_TX_DS) != 0u)
  {
    return 1u;
  }

  if ((status & NRF24_STATUS_MAX_RT) != 0u)
  {
    (void)nrf24_flush_tx();
    return 0u;
  }

  return 0u;
}

#if APP_SD_USE_FATFS
#include "ff.h"

static FATFS s_sd_fs;
static uint8_t s_sd_mounted;

static uint8_t sd_mount_once(void)
{
  FRESULT fr;

  if (s_sd_mounted) return 1u;

  fr = f_mount(&s_sd_fs, "", 1u);
  if (fr == FR_OK)
  {
    s_sd_mounted = 1u;
    return 1u;
  }

  if (!s_sd_diag_reported)
  {
    Diag_Log("SD FatFs mount failed\n");
    s_sd_diag_reported = 1u;
  }

  return 0u;
}
#endif

void Telemetry_Send32(const uint8_t payload[32])
{
  uint16_t seq;

  nrf24_init_once();
  if (!s_nrf24_ready) return;

  seq = (uint16_t)payload[4] | ((uint16_t)payload[5] << 8);

  if (nrf24_tx32(payload))
  {
    s_nrf24_tx_ok_count++;
    s_nrf24_uart_trace_count++;
    if ((s_nrf24_uart_trace_count <= 40u) || ((s_nrf24_uart_trace_count % 40u) == 0u))
    {
      Diag_Log("BOT UART NRF TX #%lu magic=%02X ver=%02X frag=%u/%u seq=%u kind=%u p8=%02X p9=%02X p10=%02X p11=%02X",
               (unsigned long)s_nrf24_uart_trace_count,
               (unsigned)payload[0],
               (unsigned)payload[1],
               (unsigned)payload[2],
               (unsigned)payload[3],
               (unsigned)seq,
               (unsigned)payload[6],
               (unsigned)payload[8],
               (unsigned)payload[9],
               (unsigned)payload[10],
               (unsigned)payload[11]);
    }
    if ((s_nrf24_tx_ok_count <= 3u) || ((s_nrf24_tx_ok_count % 10u) == 0u))
    {
      Diag_Log("BOT NRF24 TX ok #%lu magic=%02X frag=%u/%u seq=%u kind=%u",
               (unsigned long)s_nrf24_tx_ok_count,
               (unsigned)payload[0],
               (unsigned)payload[2],
               (unsigned)payload[3],
               (unsigned)((uint16_t)payload[4] | ((uint16_t)payload[5] << 8)),
               (unsigned)payload[6]);
    }
    return;
  }

  if (!s_nrf24_diag_reported)
  {
    uint8_t status = 0xFFu;
    uint8_t fifo = 0xFFu;
    (void)nrf24_read_reg(NRF24_REG_STATUS, &status);
    (void)nrf24_read_reg(NRF24_REG_FIFO_STATUS, &fifo);
    Diag_Log("BOT NRF24 telemetry TX failed status=%02X fifo=%02X frag=%u/%u seq=%u kind=%u",
             (unsigned)status,
             (unsigned)fifo,
             (unsigned)payload[2],
             (unsigned)payload[3],
             (unsigned)((uint16_t)payload[4] | ((uint16_t)payload[5] << 8)),
             (unsigned)payload[6]);
    s_nrf24_diag_reported = 1u;
  }
}

void Telemetry_SdStore32(const uint8_t payload[32])
{
  if (!payload) return;

#if APP_SD_USE_FATFS
  {
    FIL file;
    FRESULT fr;
    UINT written = 0u;

    if (!sd_mount_once()) return;

    fr = f_open(&file, "telemetry.bin", FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK)
    {
      if (!s_sd_diag_reported)
      {
        Diag_Log("SD open telemetry.bin failed\n");
        s_sd_diag_reported = 1u;
      }
      return;
    }

    fr = f_write(&file, payload, 32u, &written);
    (void)f_close(&file);

    if (fr != FR_OK || written != 32u)
    {
      if (!s_sd_diag_reported)
      {
        Diag_Log("SD write telemetry.bin failed\n");
        s_sd_diag_reported = 1u;
      }
    }
  }
#else
  if (!s_sd_diag_reported)
  {
    Diag_Log("SD backend pending FatFs integration\n");
    s_sd_diag_reported = 1u;
  }
#endif
}

#endif
