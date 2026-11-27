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

// Minimal nrfx configuration for bare-metal nRF52840 + TinyUSB builds.
// TinyUSB's dcd_nrf5x.c includes nrfx_clock.h for inline HAL clock helpers.

#define NRFX_CLOCK_ENABLED 0
#define NRFX_POWER_ENABLED 0
#define NRFX_SAADC_ENABLED 0
#define NRFX_TIMER_ENABLED 0
#define NRFX_RTC_ENABLED 0
#define NRFX_GPIOTE_ENABLED 0

#include <templates/nrfx_config_nrf52840.h>
