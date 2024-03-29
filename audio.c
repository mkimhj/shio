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
#include "time_sync.h"
#include "timers.h"
#include "draw.h"
#include "gpio.h"
#include "event.h"
#include "main.h"
#include "audio.h"

// #define AUDIO_SYNC_DEBUG
#define TIME_SYNC_DISABLED 0

#define TICKS_THRESHOLD 512
#define PDM_EDGE_RISING  1
#define PDM_EDGE_FALLING 0

int16_t releasedPdmBuffer[PDM_DECIMATION_BUFFER_LENGTH] = {0};
int16_t pdmBuffer[2][PDM_BUFFER_LENGTH+2] = {0}; // add two to the buffer for sample compensationReady         = false;
static int pdmBufferIndex               = 0;
static int64_t samplesCompensated       = 0;
static int64_t ticksAhead               = 0;
static bool streamStarted = false;
uint16_t fakeData = 0;
uint32_t pdmBufferCount = 0;

static void decimate(int16_t* outputBuffer, int16_t* inputBuffer, uint8_t decimationFactor)
{
  for (int i = 0; i < PDM_DECIMATION_BUFFER_LENGTH; i++) {

    if (fakeData < 32768) {
      outputBuffer[i] = fakeData++;
    } else {
#ifdef AUDIO_SYNC_DEBUG
      outputBuffer[i] = fakeData++;
#else
    outputBuffer[i] = inputBuffer[i*decimationFactor];
#endif
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
    pdmBufferCount++;
    gpioWrite(GPIO_3_PIN, (pdmBufferIndex == 0) ? 1 : 0);
    decimate(releasedPdmBuffer, event->buffer_released, PDM_DECIMATION_FACTOR);
    eventQueuePush(EVENT_AUDIO_MIC_DATA_READY);
  }

  if (event->buffer_requested) {
    // if ticksAhead > TICKS_THRESHOLD, increase pdm buffer size to slow down
    // if ticksAhead < -TICKS_THRESHOLD, decrease pdm buffer size to catch up
    int bufferTweakAmount = 0;

    if (!ts_master() && streamStarted) {
      if (samplesCompensated == 0) {
        if (ticksAhead > TICKS_THRESHOLD) {
          bufferTweakAmount = 1;
          samplesCompensated++;
        } else if (ticksAhead < -TICKS_THRESHOLD) {
          bufferTweakAmount = -1;
          samplesCompensated--;
        }
      } else if (samplesCompensated > 0) {
        if (ticksAhead > TICKS_THRESHOLD * (samplesCompensated + 1)) {
          bufferTweakAmount = 1;
          samplesCompensated++;
        } else if (ticksAhead < TICKS_THRESHOLD * (samplesCompensated - 1)) {
          bufferTweakAmount = -1;
          samplesCompensated--;
        }
      } else if (samplesCompensated < 0) {
        if (ticksAhead < -TICKS_THRESHOLD * abs((samplesCompensated - 1))) {
          bufferTweakAmount = -1;
          samplesCompensated--;
        } else if (ticksAhead > -TICKS_THRESHOLD * abs((samplesCompensated + 1))) {
          bufferTweakAmount = 1;
          samplesCompensated++;
        }
      }

      if (bufferTweakAmount != 0) {
        NRF_LOG_RAW_INFO("%08d [audio] samplesCompensated:%d bufferTweakAmount:%d\n",
          systemTimeGetMs(), samplesCompensated, bufferTweakAmount);

#if TIME_SYNC_DISABLED
        bufferTweakAmount = 0;
        eventQueuePush(EVENT_METADATA_SAVE_TIMESTAMP);
#endif
      }
    }

    pdmBufferIndex = (pdmBufferIndex == 0) ? 1 : 0;
    errorStatus = nrfx_pdm_buffer_set(pdmBuffer[pdmBufferIndex], PDM_BUFFER_LENGTH + bufferTweakAmount);
    ASSERT(errorStatus == NRFX_SUCCESS);
  }

  return;
}

int16_t* audioGetMicData(void)
{
  return releasedPdmBuffer;
}

uint32_t audioGetPdmBufferCount(void)
{
  return pdmBufferCount;
}

void audioUpdateTicksAhead(void)
{
  // 64 pdm clockes edges equates to 1 sample of PCM audio data
  // After 64 pdm clock edges a sample needs to be skipped

  // With f_pdm of 3.2MHz:
  // (64 clock cycles) / 3.2MHz = 20us
  // 20us on the 16MHz time sync clock is 320 ticks
  // Skip a 50khz sample after 320 ticks have accumulated

  // With f_pdm of 1MHz:
  // (64 clock cycles) / 1MHz = 64us
  // 64us on the 16MHz time sync clock is 1024 ticks
  // Skip a 15.625khz sample after 1024 ticks have accumulated

  // With f_pdm of 1MHz:
  // (64 clock cycles) / 2MHz = 32us
  // 32us on the 16MHz time sync clock is 512 ticks
  // Skip a 31250khz sample after 512 ticks have accumulated

  static bool biasInitialized     = false;
  static uint64_t systemTimeBias  = 0;
  static uint64_t syncTimeBias    = 0;
  static int32_t prevTimerOffset  = 0;
  static uint32_t offsetTolerance = 100;
  static uint32_t tolerancePassed = 0;

  if (!ts_master() && streamStarted) {
    uint64_t systemTimeTicks = systemTimeGetTicks();
    uint64_t syncTimeTicks   = ts_timestamp_get_ticks_u64(6);

    if (!biasInitialized) {
      biasInitialized = true;
      systemTimeBias  = systemTimeTicks;
      syncTimeBias    = syncTimeTicks;
#ifdef AUDIO_SYNC_DEBUG
      NRF_LOG_RAW_INFO("%08d [audio] pBias:%u lBias:%u\n", systemTimeGetMs(), syncTimeBias, systemTimeBias);
#endif
    }

    // Subtract off bias
    int64_t relativeSystemTime = systemTimeTicks - systemTimeBias;
    int64_t relativeSyncTime   = syncTimeTicks - syncTimeBias;

    // Calculate offset
    int32_t timerOffset = (relativeSystemTime - relativeSyncTime) % TIME_SYNC_TIMER_MAX_VAL;

    if (prevTimerOffset == 0) {
      prevTimerOffset = timerOffset;
    }

    // If there's an erroneous jump, then don't update ticksAhead
    // Likely hit this function as one timer was recently updated and the other hasn't
    if (abs(timerOffset - prevTimerOffset) < offsetTolerance) {
      // Only update ticksAhead if we are within tolerance for at LEAST 3 time sync packets
      // This gives the timers a chance to stabilize before updating ticksAhead
      if (tolerancePassed++ < 3) {
        offsetTolerance += 5;
      } else {
        ticksAhead      = timerOffset;
        prevTimerOffset = timerOffset;
        offsetTolerance = 100;
      }
    } else {
      tolerancePassed = 0;
      offsetTolerance += 5; // Each time we fail to update, increase our acceptable tolerance
#ifdef AUDIO_SYNC_DEBUG_2
      NRF_LOG_RAW_INFO("%08d [audio] offset:%d prevOffset:%d delta:%d\n",
        systemTimeGetMs(), timerOffset, prevTimerOffset, abs(timerOffset - prevTimerOffset));
#endif
    }

#ifdef AUDIO_SYNC_DEBUG_2
    NRF_LOG_RAW_INFO("%08d [audio] p:%u l:%u o:%d t:%d\n",
      systemTimeGetMs(), relativeSyncTime, relativeSystemTime, timerOffset, ticksAhead);
#endif
  }
}

bool audioStreamStarted(void)
{
  return streamStarted;
}

void audioSetStreamStarted(bool started)
{
  streamStarted = started;
}

uint32_t audioGetPdmStartTaskAddress(void)
{
  return nrfx_pdm_task_address_get(NRF_PDM_TASK_START);
}

void audioInit(void)
{
  nrfx_err_t errorStatus;

  // Enable Mic
  gpioOutputEnable(MIC_EN_PIN);
  gpioWrite(MIC_EN_PIN, 1);
  delayMs(1);

  // Setup PDM
  nrfx_pdm_config_t pdmConfig = {
    .mode               = (nrf_pdm_mode_t)NRFX_PDM_CONFIG_MODE,
    .edge               = (nrf_pdm_edge_t)PDM_EDGE_FALLING,
    .pin_clk            = PDM_CLK_PIN,
    .pin_din            = PDM_DATA_PIN,

    // DIV32: 0x08000000 -> CLK: 1.000 MHz -> SR: 15625 Hz
    // DIV31: 0x08400000 -> CLK: 1.032 MHz -> SR: 16125 Hz
    // DIV30: 0x08800000 -> CLK: 1.067 MHz -> SR: 16667 Hz
    // DIV25: 0x0A000000 -> CLK: 1.280 MHz -> SR: 20000 Hz
    // DIV16: 0x10000000 -> CLK: 2.000 MHz -> SR: 31250 Hz
    // DIV12: 0x15000000 -> CLK: 2.667 MHz -> SR: 41667 Hz
    // DIV10: 0x19000000 -> CLK: 3.200 MHz -> SR: 50000 Hz
    // DIV08: 0x20000000 -> CLK: 4.000 MHz -> SR: 62500 Hz

    .clock_freq         = (nrf_pdm_freq_t) 0x10000000,
    .gain_l             = 0x2e, // 3dB gain
    .gain_r             = 0x2e, // 3dB gain
    .interrupt_priority = NRFX_PDM_CONFIG_IRQ_PRIORITY
  };

  errorStatus = nrfx_pdm_init(&pdmConfig, (nrfx_pdm_event_handler_t) pdmEventHandler);
  ASSERT(errorStatus == NRFX_SUCCESS);

  nrfx_pdm_buffer_set(pdmBuffer[0], PDM_BUFFER_LENGTH);
  ASSERT(errorStatus == NRFX_SUCCESS);

  NRF_LOG_RAW_INFO("%08d [audio] initialized\n", systemTimeGetMs());
}

void audioStart(void)
{
  if (!streamStarted) {
    nrfx_err_t errorStatus;
    streamStarted = true;
    errorStatus = nrfx_pdm_start();
    ASSERT(errorStatus == NRFX_SUCCESS);
    NRF_LOG_RAW_INFO("%08d [audio] pdm start\n", systemTimeGetMs());
  }
}

void audioStop(void)
{
  streamStarted = false;
  nrfx_pdm_stop();
}

void audioDeInit(void)
{
  nrfx_pdm_uninit();
  gpioWrite(MIC_EN_PIN, 0);
  NRF_LOG_RAW_INFO("%08d [audio] deinitialized\n", systemTimeGetMs());
}
