#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <nrfx.h>

#include "app_util_platform.h"
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "arm_const_structs.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_gpio.h"

#include "nrf_pdm.h"
#include "nrfx_pdm.h"
#include "timers.h"
#include "draw.h"
#include "gpio.h"
#include "event.h"
#include "main.h"
#include "audio.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"
#include "time_sync.h"

int16_t pdmBuffer[2][PDM_BUFFER_LENGTH] = {0};
int16_t releasedPdmBuffer[PDM_DECIMATION_BUFFER_LENGTH] = {0};
static bool fftInputBufferReady = false;
static int pdmBufferIndex = 0;
static uint32_t bufferReleasedTime = 0;

static void decimate(int16_t* outputBuffer, int16_t* inputBuffer, uint8_t decimationFactor)
{
  for (int i = 0; i < PDM_DECIMATION_BUFFER_LENGTH; i++) {
    outputBuffer[i] = inputBuffer[i*decimationFactor];
  }
}

static void pdmEventHandler(nrfx_pdm_evt_t *event)
{
  nrfx_err_t errorStatus;
  static bool pdmBufferSwitchFlag = false;

  if (event->error != NRFX_PDM_NO_ERROR) {
    NRF_LOG_RAW_INFO("[audio] pdm error\n");
    ASSERT(0);
  }

  if (event->buffer_released) {
    CRITICAL_REGION_ENTER();
    // bufferReleasedTime = ts_timestamp_get_ticks_u32(NRF_PPI_CHANNEL10);
    bufferReleasedTime = systemTimeGetTicks();
    decimate(releasedPdmBuffer, event->buffer_released, PDM_DECIMATION_FACTOR);
    eventQueuePush(EVENT_AUDIO_MIC_DATA_READY);
    CRITICAL_REGION_EXIT();
  }

  if (event->buffer_requested) {
    pdmBufferIndex = (pdmBufferIndex == 0) ? 1 : 0;
    errorStatus = nrfx_pdm_buffer_set(pdmBuffer[pdmBufferIndex], PDM_BUFFER_LENGTH);
    ASSERT(errorStatus == NRFX_SUCCESS);
  }

  return;
}

int16_t* audioGetMicData(void)
{
  return releasedPdmBuffer;
}

uint32_t audioGetBufferReleasedTime(void)
{
  return bufferReleasedTime;
}

void audioInit(void)
{
  gpioOutputEnable(MIC_EN_PIN);
  gpioWrite(MIC_EN_PIN, 1);
  delayMs(1);

  NRF_LOG_RAW_INFO("%08d [audio] initialized\n", systemTimeGetMs());
}

void audioStart(void)
{
  nrfx_err_t errorStatus;

  nrfx_pdm_config_t pdmConfig = {
    .mode               = (nrf_pdm_mode_t)NRFX_PDM_CONFIG_MODE,
    .edge               = (nrf_pdm_edge_t)NRFX_PDM_CONFIG_EDGE,
    .pin_clk            = PDM_CLK_PIN,
    .pin_din            = PDM_DATA_PIN,
    .clock_freq         = (nrf_pdm_freq_t)NRFX_PDM_CONFIG_CLOCK_FREQ,
    .gain_l             = 0x3C,
    .gain_r             = 0x3C,
    .interrupt_priority = NRFX_PDM_CONFIG_IRQ_PRIORITY
  };

  errorStatus = nrfx_pdm_init(&pdmConfig, (nrfx_pdm_event_handler_t) pdmEventHandler);
  ASSERT(errorStatus == NRFX_SUCCESS);

  nrfx_pdm_buffer_set(pdmBuffer[0], PDM_BUFFER_LENGTH);
  ASSERT(errorStatus == NRFX_SUCCESS);

  errorStatus = nrfx_pdm_start();
  ASSERT(errorStatus == NRFX_SUCCESS);

  NRF_LOG_RAW_INFO("%08d [audio] pdm start\n", systemTimeGetMs());
}

void audioDeInit(void)
{
  nrfx_pdm_stop();
  nrfx_pdm_uninit();
  gpioWrite(MIC_EN_PIN, 0);
  NRF_LOG_RAW_INFO("%08d [audio] deinitialized\n", systemTimeGetMs());
}