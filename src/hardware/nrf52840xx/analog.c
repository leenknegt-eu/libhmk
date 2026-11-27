/*
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "hardware/hardware.h"

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#endif

#include "nrf.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

// SAADC PSELP values for AIN0..AIN7 (P0.02, P0.03, P0.04, P0.05, P0.28..P0.31)
static const uint32_t channel_pselp[] = {
    SAADC_CH_PSELP_PSELP_AnalogInput0, SAADC_CH_PSELP_PSELP_AnalogInput1,
    SAADC_CH_PSELP_PSELP_AnalogInput2, SAADC_CH_PSELP_PSELP_AnalogInput3,
    SAADC_CH_PSELP_PSELP_AnalogInput4, SAADC_CH_PSELP_PSELP_AnalogInput5,
    SAADC_CH_PSELP_PSELP_AnalogInput6, SAADC_CH_PSELP_PSELP_AnalogInput7,
};

_Static_assert(M_ARRAY_SIZE(channel_pselp) == ADC_NUM_CHANNELS,
               "Invalid number of ADC channels");

#if ADC_NUM_MUX_INPUTS > 0
static const uint8_t mux_input_channels[] = ADC_MUX_INPUT_CHANNELS;

_Static_assert(M_ARRAY_SIZE(mux_input_channels) == ADC_NUM_MUX_INPUTS,
               "Invalid number of ADC multiplexer inputs");

static NRF_GPIO_Type *const mux_select_ports[] = ADC_MUX_SELECT_PORTS;

_Static_assert(M_ARRAY_SIZE(mux_select_ports) == ADC_NUM_MUX_SELECT_PINS,
               "Invalid number of multiplexer select pins");

static const uint8_t mux_select_pins[] = ADC_MUX_SELECT_PINS;

_Static_assert(M_ARRAY_SIZE(mux_select_pins) == ADC_NUM_MUX_SELECT_PINS,
               "Invalid number of multiplexer select pins");

static const uint16_t mux_input_matrix[][ADC_NUM_MUX_INPUTS] =
    ADC_MUX_INPUT_MATRIX;

_Static_assert(M_ARRAY_SIZE(mux_input_matrix) == (1 << ADC_NUM_MUX_SELECT_PINS),
               "Invalid number of multiplexer select pins");
#endif

#if ADC_NUM_RAW_INPUTS > 0
static const uint8_t raw_input_channels[] = ADC_RAW_INPUT_CHANNELS;

_Static_assert(M_ARRAY_SIZE(raw_input_channels) == ADC_NUM_RAW_INPUTS,
               "Invalid number of ADC raw inputs");

static const uint16_t raw_input_vector[] = ADC_RAW_INPUT_VECTOR;

_Static_assert(M_ARRAY_SIZE(raw_input_vector) == ADC_NUM_RAW_INPUTS,
               "Invalid number of ADC raw inputs");
#endif

#define ADC_TOTAL_INPUTS (ADC_NUM_MUX_INPUTS + ADC_NUM_RAW_INPUTS)

static volatile bool adc_initialized = false;
static volatile uint32_t adc_scan_interval_us = 0;

// EasyDMA requires RAM-resident, word-aligned signed 16-bit result buffer
__attribute__((aligned(4))) static volatile int16_t
    adc_buffer[ADC_TOTAL_INPUTS];
static volatile uint16_t adc_values[NUM_KEYS];

static void saadc_configure_matrix_channels(void) {
  // Disable all 8 SAADC channels first
  for (uint32_t ch = 0; ch < 8U; ch++) {
    NRF_SAADC->CH[ch].PSELP = SAADC_CH_PSELP_PSELP_NC;
    NRF_SAADC->CH[ch].PSELN = SAADC_CH_PSELN_PSELN_NC;
  }

  const uint32_t ch_cfg =
      (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos) |
      (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |
      (ADC_GAIN << SAADC_CH_CONFIG_GAIN_Pos) |
      (ADC_REFSEL << SAADC_CH_CONFIG_REFSEL_Pos) |
      (ADC_ACQTIME << SAADC_CH_CONFIG_TACQ_Pos) |
      (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |
      (SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos);

  uint32_t slot = 0;

#if ADC_NUM_MUX_INPUTS > 0
  for (uint32_t i = 0; i < ADC_NUM_MUX_INPUTS; i++) {
    NRF_SAADC->CH[slot].CONFIG = ch_cfg;
    NRF_SAADC->CH[slot].PSELP = channel_pselp[mux_input_channels[i]];
    NRF_SAADC->CH[slot].PSELN = SAADC_CH_PSELN_PSELN_NC;
    slot++;
  }
#endif

#if ADC_NUM_RAW_INPUTS > 0
  for (uint32_t i = 0; i < ADC_NUM_RAW_INPUTS; i++) {
    NRF_SAADC->CH[slot].CONFIG = ch_cfg;
    NRF_SAADC->CH[slot].PSELP = channel_pselp[raw_input_channels[i]];
    NRF_SAADC->CH[slot].PSELN = SAADC_CH_PSELN_PSELN_NC;
    slot++;
  }
#endif

  NRF_SAADC->RESOLUTION = ADC_RESOLUTION_HAL << SAADC_RESOLUTION_VAL_Pos;
  NRF_SAADC->OVERSAMPLE = SAADC_OVERSAMPLE_OVERSAMPLE_Bypass
                          << SAADC_OVERSAMPLE_OVERSAMPLE_Pos;
  NRF_SAADC->SAMPLERATE = SAADC_SAMPLERATE_MODE_Task
                          << SAADC_SAMPLERATE_MODE_Pos;
  NRF_SAADC->RESULT.PTR = (uint32_t)(uintptr_t)adc_buffer;
  NRF_SAADC->RESULT.MAXCNT = ADC_TOTAL_INPUTS;
}

static void saadc_start_scan(void) {
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;
  NRF_SAADC->RESULT.PTR = (uint32_t)(uintptr_t)adc_buffer;
  NRF_SAADC->RESULT.MAXCNT = ADC_TOTAL_INPUTS;

  NRF_SAADC->EVENTS_STARTED = 0UL;
  NRF_SAADC->EVENTS_END = 0UL;
  NRF_SAADC->TASKS_START = 1UL;
  while (NRF_SAADC->EVENTS_STARTED == 0UL)
    ;
  NRF_SAADC->TASKS_SAMPLE = 1UL;
}

static void timer1_start_oneshot_us(uint32_t delay_us) {
  NRF_TIMER1->TASKS_STOP = 1UL;
  NRF_TIMER1->TASKS_CLEAR = 1UL;
  NRF_TIMER1->EVENTS_COMPARE[0] = 0UL;
  NRF_TIMER1->CC[0] = delay_us > 0UL ? delay_us : 1UL;
  NRF_TIMER1->TASKS_START = 1UL;
}

__attribute__((always_inline)) static inline uint16_t
saadc_clamp(int16_t raw_val) {
  if (raw_val <= 0)
    return 0;
  const uint16_t val = (uint16_t)raw_val;
  return val > ADC_MAX_VALUE ? ADC_MAX_VALUE : val;
}

void analog_init(void) {
#if ADC_NUM_MUX_INPUTS > 0
  // Configure multiplexer select pins as high-drive digital outputs
  for (uint32_t i = 0; i < ADC_NUM_MUX_SELECT_PINS; i++) {
    mux_select_ports[i]->PIN_CNF[mux_select_pins[i]] =
        (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
        (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
        (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
        (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
        (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
    mux_select_ports[i]->OUTCLR = 1UL << mux_select_pins[i];
  }
#endif

  // Configure TIMER1 as 1 MHz (1 us/tick) 32-bit one-shot timer for mux
  // settling delay and low-power scan throttling
  NRF_TIMER1->TASKS_STOP = 1UL;
  NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer << TIMER_MODE_MODE_Pos;
  NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_32Bit << TIMER_BITMODE_BITMODE_Pos;
  NRF_TIMER1->PRESCALER = 4UL; // 16 MHz / 2^4 = 1 MHz
  NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_STOP_Msk;
  NRF_TIMER1->EVENTS_COMPARE[0] = 0UL;
  NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE0_Msk;

  NVIC_SetPriority(TIMER1_IRQn, NRF_IRQ_PRIO_MID);
  NVIC_ClearPendingIRQ(TIMER1_IRQn);
  NVIC_EnableIRQ(TIMER1_IRQn);

  // Configure SAADC channels and run offset calibration
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;
  saadc_configure_matrix_channels();

  NRF_SAADC->EVENTS_CALIBRATEDONE = 0UL;
  NRF_SAADC->TASKS_CALIBRATEOFFSET = 1UL;
  while (NRF_SAADC->EVENTS_CALIBRATEDONE == 0UL)
    ;
  NRF_SAADC->EVENTS_CALIBRATEDONE = 0UL;

  NRF_SAADC->INTENCLR = 0xFFFFFFFFUL;
  NRF_SAADC->INTENSET = SAADC_INTENSET_END_Msk;

  NVIC_SetPriority(SAADC_IRQn, NRF_IRQ_PRIO_MID);
  NVIC_ClearPendingIRQ(SAADC_IRQn);
  NVIC_EnableIRQ(SAADC_IRQn);

  // Kick off the asynchronous SAADC scan loop
  saadc_start_scan();

  // Wait until every key has been sampled at least once
  while (!adc_initialized)
    ;
}

void analog_task(void) {}

uint16_t analog_read(uint8_t key) { return adc_values[key]; }

#if defined(BLUETOOTH_ENABLED)
void analog_set_scan_interval(uint32_t interval_us) {
  adc_scan_interval_us = interval_us;
}

uint16_t analog_read_battery_mv(void) {
  // Pause asynchronous matrix scanning safely
  NVIC_DisableIRQ(SAADC_IRQn);
  NVIC_DisableIRQ(TIMER1_IRQn);
  NRF_TIMER1->TASKS_STOP = 1UL;

  // Wait for any in-flight conversion to finish
  if ((NRF_SAADC->ENABLE & SAADC_ENABLE_ENABLE_Msk) != 0UL) {
    while ((NRF_SAADC->STATUS & SAADC_STATUS_STATUS_Msk) != 0UL)
      ;
    NRF_SAADC->TASKS_STOP = 1UL;
    while (NRF_SAADC->EVENTS_STOPPED == 0UL)
      ;
    NRF_SAADC->EVENTS_STOPPED = 0UL;
  }

  // Configure channel 0 for absolute battery voltage measurement:
  // Internal 0.6V reference + 1/6 gain => 0..3.6V full scale at 12-bit (4096)
  NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos;
  for (uint32_t ch = 0; ch < 8U; ch++) {
    NRF_SAADC->CH[ch].PSELP = SAADC_CH_PSELP_PSELP_NC;
    NRF_SAADC->CH[ch].PSELN = SAADC_CH_PSELN_PSELN_NC;
  }

  NRF_SAADC->CH[0].CONFIG =
      (SAADC_CH_CONFIG_RESP_Bypass << SAADC_CH_CONFIG_RESP_Pos) |
      (SAADC_CH_CONFIG_RESN_Bypass << SAADC_CH_CONFIG_RESN_Pos) |
      (SAADC_CH_CONFIG_GAIN_Gain1_6 << SAADC_CH_CONFIG_GAIN_Pos) |
      (SAADC_CH_CONFIG_REFSEL_Internal << SAADC_CH_CONFIG_REFSEL_Pos) |
      (SAADC_CH_CONFIG_TACQ_10us << SAADC_CH_CONFIG_TACQ_Pos) |
      (SAADC_CH_CONFIG_MODE_SE << SAADC_CH_CONFIG_MODE_Pos) |
      (SAADC_CH_CONFIG_BURST_Disabled << SAADC_CH_CONFIG_BURST_Pos);

#if defined(BLE_BATTERY_ADC_CHANNEL)
  NRF_SAADC->CH[0].PSELP = channel_pselp[BLE_BATTERY_ADC_CHANNEL];
#else
  NRF_SAADC->CH[0].PSELP = SAADC_CH_PSELP_PSELP_VDD;
#endif
  NRF_SAADC->CH[0].PSELN = SAADC_CH_PSELN_PSELN_NC;
  NRF_SAADC->RESOLUTION = SAADC_RESOLUTION_VAL_12bit
                          << SAADC_RESOLUTION_VAL_Pos;

  volatile int16_t raw_sample = 0;
  NRF_SAADC->RESULT.PTR = (uint32_t)(uintptr_t)&raw_sample;
  NRF_SAADC->RESULT.MAXCNT = 1UL;

  NRF_SAADC->EVENTS_STARTED = 0UL;
  NRF_SAADC->EVENTS_END = 0UL;
  NRF_SAADC->TASKS_START = 1UL;
  while (NRF_SAADC->EVENTS_STARTED == 0UL)
    ;
  NRF_SAADC->TASKS_SAMPLE = 1UL;
  while (NRF_SAADC->EVENTS_END == 0UL)
    ;
  NRF_SAADC->EVENTS_END = 0UL;

  // Convert raw 12-bit sample to millivolts:
  // mV = (sample * 3600 / 4096) * (BLE_BATTERY_DIVIDER_MILLIS / 1000)
  uint32_t mv = 0;
  if (raw_sample > 0) {
    const uint32_t adc_mv = ((uint32_t)raw_sample * 3600UL) >> 12;
    mv = (adc_mv * (uint32_t)BLE_BATTERY_DIVIDER_MILLIS) / 1000UL;
  }

  // Restore matrix channels and resume scanning
  saadc_configure_matrix_channels();
  NRF_SAADC->EVENTS_END = 0UL;
  NVIC_ClearPendingIRQ(SAADC_IRQn);
  NVIC_ClearPendingIRQ(TIMER1_IRQn);
  NVIC_EnableIRQ(SAADC_IRQn);
  NVIC_EnableIRQ(TIMER1_IRQn);
  saadc_start_scan();

  return (uint16_t)mv;
}
#endif

//--------------------------------------------------------------------+
// Interrupt Handlers
//--------------------------------------------------------------------+

void SAADC_IRQHandler(void) {
#if ADC_NUM_MUX_INPUTS > 0
  static uint8_t current_mux_channel = 0;
#endif

  if (NRF_SAADC->EVENTS_END != 0UL) {
    NRF_SAADC->EVENTS_END = 0UL;

#if ADC_NUM_MUX_INPUTS > 0
    for (uint32_t i = 0; i < ADC_NUM_MUX_INPUTS; i++) {
      const uint16_t key = mux_input_matrix[current_mux_channel][i];
      if (key != 0U)
        adc_values[key - 1U] = saadc_clamp(adc_buffer[i]);
    }
#endif

#if ADC_NUM_RAW_INPUTS > 0
    for (uint32_t i = 0; i < ADC_NUM_RAW_INPUTS; i++) {
      const uint16_t key = raw_input_vector[i];
      if (key != 0U)
        adc_values[key - 1U] =
            saadc_clamp(adc_buffer[ADC_NUM_MUX_INPUTS + i]);
    }
#endif

#if ADC_NUM_MUX_INPUTS > 0
    current_mux_channel =
        (uint8_t)((current_mux_channel + 1U) &
                  ((1U << ADC_NUM_MUX_SELECT_PINS) - 1U));
    const bool full_scan_done = (current_mux_channel == 0U);
    adc_initialized |= full_scan_done;

    // Advance multiplexer select lines
    for (uint32_t i = 0; i < ADC_NUM_MUX_SELECT_PINS; i++) {
      if (((current_mux_channel >> i) & 1U) != 0U)
        mux_select_ports[i]->OUTSET = 1UL << mux_select_pins[i];
      else
        mux_select_ports[i]->OUTCLR = 1UL << mux_select_pins[i];
    }

    uint32_t delay_us = ADC_SAMPLE_DELAY;
    if (full_scan_done && adc_scan_interval_us > delay_us) {
      delay_us = adc_scan_interval_us;
      // Disable SAADC between throttled scans to save power (~1 mA)
      NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled
                          << SAADC_ENABLE_ENABLE_Pos;
    }

    timer1_start_oneshot_us(delay_us);
#else
    adc_initialized = true;
    if (adc_scan_interval_us > 0UL) {
      NRF_SAADC->ENABLE = SAADC_ENABLE_ENABLE_Disabled
                          << SAADC_ENABLE_ENABLE_Pos;
      timer1_start_oneshot_us(adc_scan_interval_us);
    } else {
      saadc_start_scan();
    }
#endif
  }
}

void TIMER1_IRQHandler(void) {
  if (NRF_TIMER1->EVENTS_COMPARE[0] != 0UL) {
    NRF_TIMER1->EVENTS_COMPARE[0] = 0UL;
    saadc_start_scan();
  }
}
