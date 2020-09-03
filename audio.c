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

#include "time_sync.h"

typedef enum {
  OFFSET_NONE = 0,
  OFFSET_OVERSAMPLE,
  OFFSET_UNDERSAMPLE
} offsetEvent_t;

int16_t pdmBuffer[2][PDM_BUFFER_LENGTH] = {0};
int16_t releasedPdmBuffer[PDM_DECIMATION_BUFFER_LENGTH] = {0};
static bool fftInputBufferReady = false;
static int pdmBufferIndex = 0;
static volatile int32_t m_offset = 0;
offsetEvent_t offsetEvent;
int64_t samplesSkipped;
int64_t timeOffsetMs;

void audioRefreshOffset(void)
{
  uint32_t peer_timer = ts_get_peer_timer();
  uint32_t local_timer = ts_get_local_timer();
  uint32_t timer_offset = ts_get_timer_offset();

  if (local_timer > peer_timer)
  {
      m_offset += (int)timer_offset;
  }
  else
  {
      m_offset -= (int)timer_offset;
  }
}

static int32_t getOffset(void)
{
  return m_offset;
}

static void getCompensateData(void)
{
    int32_t c_lower_bound   = TIME_SYNC_TIMER_MAX_VAL*(samplesSkipped - 1);
    int32_t c_upper_bound   = TIME_SYNC_TIMER_MAX_VAL*(samplesSkipped + 1);

    if (ts_is_master()) {
        timeOffsetMs = 0;
    } else {
        timeOffsetMs = getOffset();
    }

    if (timeOffsetMs > c_upper_bound) {
        samplesSkipped++;
        offsetEvent = OFFSET_UNDERSAMPLE;
    } else if (timeOffsetMs < c_lower_bound) {
        samplesSkipped--;
        offsetEvent = OFFSET_OVERSAMPLE;
    } else {
        offsetEvent = OFFSET_NONE;
    }
}

static void decimate(int16_t* outputBuffer, int16_t* inputBuffer, uint8_t decimationFactor)
{
  int decimation_index = 0;

  for (int i = 0; i < PDM_DECIMATION_BUFFER_LENGTH; i++) {
    getCompensateData();

    // Abort decimation and proceed to next mic sample
    if (offsetEvent == OFFSET_UNDERSAMPLE && decimation_index > (PDM_BUFFER_LENGTH - 2 * decimationFactor - 1)) {
      return;
    }

    // Drift compensate event handler
    switch (offsetEvent) {
      case OFFSET_UNDERSAMPLE:
        decimation_index += (i == 0) ? 0 : decimationFactor * 2;
        break;

      case OFFSET_OVERSAMPLE:
        decimation_index += (i == 0) ? 0 : decimationFactor / 2;
        break;

      case OFFSET_NONE:
        decimation_index += (i == 0) ? 0 : decimationFactor;
        break;

      default:
        break;
    }

    outputBuffer[i] = inputBuffer[decimation_index];
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

void audioInit(void)
{
  gpioOutputEnable(MIC_EN_PIN);
  gpioWrite(MIC_EN_PIN, 1);
  delayMs(1);

  // Initialize compensation variables
  offsetEvent = OFFSET_NONE;
  samplesSkipped = 0;
  timeOffsetMs = 0;

  NRF_LOG_RAW_INFO("%08d [audio] initialized\n", systemTimeGetMs());
}

void audioStart(void)
{
  nrfx_err_t errorStatus;
  nrfx_pdm_config_t pdmConfig = NRFX_PDM_DEFAULT_CONFIG(PDM_CLK_PIN, PDM_DATA_PIN);
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