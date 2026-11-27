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

// RTC1 runs from 32768 Hz LFCLK and continues ticking during System ON sleep
// (unlike SysTick, which halts on WFE/sd_app_evt_wait on nRF52840).
// RTC0 is reserved for SoftDevice S140, so we use RTC1.
//
// Exact 1 ms tick progression: 32768 / 1000 = 32 + 96/125 ticks per ms.
#define RTC1_TICKS_PER_MS_INT 32UL
#define RTC1_TICKS_PER_MS_NUM 96UL
#define RTC1_TICKS_PER_MS_DEN 125UL
#define RTC1_COUNTER_MASK 0x00FFFFFFUL

static volatile uint32_t ms_counter;
static uint32_t bresenham_acc;

void timer_init(void) {
  NRF_RTC1->TASKS_STOP = 1UL;
  NRF_RTC1->TASKS_CLEAR = 1UL;
  NRF_RTC1->PRESCALER = 0UL; // 32768 Hz

  ms_counter = 0;
  bresenham_acc = RTC1_TICKS_PER_MS_NUM;

  NRF_RTC1->EVENTS_COMPARE[0] = 0UL;
  NRF_RTC1->CC[0] = RTC1_TICKS_PER_MS_INT;
  NRF_RTC1->EVTENSET = RTC_EVTENSET_COMPARE0_Msk;
  NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk;

  NVIC_SetPriority(RTC1_IRQn, NRF_IRQ_PRIO_LOW);
  NVIC_ClearPendingIRQ(RTC1_IRQn);
  NVIC_EnableIRQ(RTC1_IRQn);

  NRF_RTC1->TASKS_START = 1UL;
}

uint32_t timer_read(void) { return ms_counter; }

//--------------------------------------------------------------------+
// Interrupt Handlers
//--------------------------------------------------------------------+

void RTC1_IRQHandler(void) {
  if (NRF_RTC1->EVENTS_COMPARE[0] != 0UL) {
    NRF_RTC1->EVENTS_COMPARE[0] = 0UL;

    uint32_t step = RTC1_TICKS_PER_MS_INT;
    bresenham_acc += RTC1_TICKS_PER_MS_NUM;
    if (bresenham_acc >= RTC1_TICKS_PER_MS_DEN) {
      bresenham_acc -= RTC1_TICKS_PER_MS_DEN;
      step += 1UL;
    }

    NRF_RTC1->CC[0] = (NRF_RTC1->CC[0] + step) & RTC1_COUNTER_MASK;
    ms_counter++;
  }
}
