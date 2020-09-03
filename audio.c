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

#define COMPENSATE_SKEW_TICK_CONSTANT         320                                       // Tick skew interval that indicates a sample offset

static int32_t getOffset(void);

int16_t pdmBuffer[2][PDM_BUFFER_LENGTH] = {0};
int16_t releasedPdmBuffer[PDM_DECIMATION_BUFFER_LENGTH] = {0};
static bool fftInputBufferReady = false;
static int pdmBufferIndex = 0;
static compensate_data_s c_data;
static volatile compensate_data_s * c_data_p;
static volatile int32_t m_offset = 0;

static compensate_data_s * getCompensateData(void)
{
    int32_t c_lower_bound   = TIME_SYNC_TIMER_MAX_VAL*((&c_data)->c_sample_skew - 1);
    int32_t c_upper_bound   = TIME_SYNC_TIMER_MAX_VAL*((&c_data)->c_sample_skew + 1);

    if (ts_is_master()) {
        (&c_data)->c_offset = 0;
    } else {
        (&c_data)->c_offset = getOffset();
    }

    if ((&c_data)->c_offset > c_upper_bound) {
        (&c_data)->c_sample_skew++;
        (&c_data)->c_evt = EVENT_COMP_UNDER_SAMPLE;
    } else if ((&c_data)->c_offset < c_lower_bound) {
        (&c_data)->c_sample_skew--;
        (&c_data)->c_evt = EVENT_COMP_OVER_SAMPLE;
    } else {
        (&c_data)->c_evt = EVENT_COMP_NONE;
    }

    return(&c_data);
}

static void decimate(int16_t* outputBuffer, int16_t* inputBuffer, uint8_t decimationFactor)
{
  int decimation_index = 0;

  for (int i = 0; i < PDM_DECIMATION_BUFFER_LENGTH; i++) {
    c_data_p = getCompensateData();

    // Abort decimation and proceed to next mic sample
    if (c_data_p->c_evt == EVENT_COMP_UNDER_SAMPLE && decimation_index > (PDM_BUFFER_LENGTH - 2 * decimationFactor - 1)) {
      break;
    }

    // Drift compensate event handler
    switch (c_data_p->c_evt) {
      case EVENT_COMP_UNDER_SAMPLE:
        decimation_index += (i == 0) ? 0 : decimationFactor * 2;
        outputBuffer[i] = inputBuffer[decimation_index];
        break;

      case EVENT_COMP_OVER_SAMPLE:
        decimation_index += (i == 0) ? 0 : decimationFactor / 2;
        outputBuffer[i] = inputBuffer[decimation_index];
        break;

      case EVENT_COMP_NONE:
        decimation_index += (i == 0) ? 0 : decimationFactor;
        outputBuffer[i] = inputBuffer[decimation_index];
        break;

      default:
        break;
    }
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

void adjustOffset(void)
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
  (&c_data)->c_evt            = EVENT_COMP_NONE;
  (&c_data)->c_sample_skew    = 0;
  (&c_data)->c_offset         = 0;

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