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

#pragma once

#include <stdbool.h>
#include <stdint.h>

//--------------------------------------------------------------------+
// Memory & Interrupt Priority Configuration
//--------------------------------------------------------------------+

#if !defined(FLASH_BASE)
#define FLASH_BASE 0x00000000UL
#endif

// SoftDevice S140 v7.3.0 flash footprint (156 KB = 39 x 4 KB pages)
#if defined(SOFTDEVICE_PRESENT)
#define NRF_SD_FLASH_SIZE 0x00027000UL
#else
#define NRF_SD_FLASH_SIZE 0x00000000UL
#endif

// On nRF52840 with SoftDevice S140, NVIC priorities 0, 1, and 4 are reserved
// for the SoftDevice. Application interrupts must use 2, 3, 5, 6, or 7.
#define NRF_IRQ_PRIO_HIGH 2U
#define NRF_IRQ_PRIO_MID 3U
#define NRF_IRQ_PRIO_LOW 6U

//--------------------------------------------------------------------+
// ADC (SAADC) Configuration
//--------------------------------------------------------------------+

#if !defined(ADC_ACQTIME)
// Fast 3us acquisition time for Hall-effect sensors / analog multiplexers
#define ADC_ACQTIME SAADC_CH_CONFIG_TACQ_3us
#endif

#if !defined(ADC_GAIN)
// Ratiometric VDD/4 reference with 1/4 gain gives full 0..VDD input range
#define ADC_GAIN SAADC_CH_CONFIG_GAIN_Gain1_4
#endif

#if !defined(ADC_REFSEL)
#define ADC_REFSEL SAADC_CH_CONFIG_REFSEL_VDD1_4
#endif

// SAADC resolution in bits, set by `scripts/make.py`
#if ADC_RESOLUTION == 14
#define ADC_RESOLUTION_HAL SAADC_RESOLUTION_VAL_14bit
#elif ADC_RESOLUTION == 12
#define ADC_RESOLUTION_HAL SAADC_RESOLUTION_VAL_12bit
#elif ADC_RESOLUTION == 10
#define ADC_RESOLUTION_HAL SAADC_RESOLUTION_VAL_10bit
#elif ADC_RESOLUTION == 8
#define ADC_RESOLUTION_HAL SAADC_RESOLUTION_VAL_8bit
#else
#error "Unsupported ADC resolution for nRF52840 SAADC (supported: 8, 10, 12, 14)"
#endif

//--------------------------------------------------------------------+
// Internal nRF52840 Driver Hooks
//--------------------------------------------------------------------+

bool nrf52_is_sd_enabled(void);
bool nrf52_is_usb_vbus_present(void);
void nrf52_soc_evt_poll(void);
void nrf52_flash_sd_evt_handler(uint32_t soc_evt);
void nrf52_ble_sd_evt_poll(void);
