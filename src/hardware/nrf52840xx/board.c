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
#pragma GCC diagnostic ignored "-Wcast-qual"
#endif

#include "nrf.h"

#if defined(SOFTDEVICE_PRESENT)
#include "nrf_error.h"
#include "nrf_mbr.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"
#endif

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include "tusb.h"

// TinyUSB nRF5x USB power event values (must match dcd_nrf5x.c)
enum {
  NRF_USB_EVT_DETECTED = 0,
  NRF_USB_EVT_REMOVED = 1,
  NRF_USB_EVT_READY = 2,
};

extern void tusb_hal_nrf_power_event(uint32_t event);

static volatile bool usb_vbus_present = false;
static volatile bool sd_enabled = false;

#if defined(SOFTDEVICE_PRESENT)
static void sd_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {
  (void)id;
  (void)pc;
  (void)info;
  board_error_handler();
}

static void board_softdevice_init(void) {
  nrf_clock_lf_cfg_t lf_cfg = {0};

#if defined(BLE_LFCLK_SOURCE_RC)
  lf_cfg.source = NRF_CLOCK_LF_SRC_RC;
  lf_cfg.rc_ctiv = 16;
  lf_cfg.rc_temp_cal_iv = 2;
  lf_cfg.accuracy = NRF_CLOCK_LF_ACCURACY_500_PPM;
#elif defined(BLE_LFCLK_SOURCE_SYNTH)
  lf_cfg.source = NRF_CLOCK_LF_SRC_SYNTH;
  lf_cfg.rc_ctiv = 0;
  lf_cfg.rc_temp_cal_iv = 0;
  lf_cfg.accuracy = NRF_CLOCK_LF_ACCURACY_50_PPM;
#else
  lf_cfg.source = NRF_CLOCK_LF_SRC_XTAL;
  lf_cfg.rc_ctiv = 0;
  lf_cfg.rc_temp_cal_iv = 0;
  lf_cfg.accuracy = NRF_CLOCK_LF_ACCURACY_20_PPM;
#endif

  const uint32_t err = sd_softdevice_enable(&lf_cfg, sd_fault_handler);
  if (err != NRF_SUCCESS)
    board_error_handler();

  sd_enabled = true;

#if !defined(NRF_DISABLE_DCDC)
  // Enable REG1 DC/DC converter for lower BLE radio and CPU power consumption
  (void)sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
#endif

  // Enable SoftDevice event interrupt (SWI2 / EGU2) at safe application priority
  NVIC_SetPriority(SWI2_EGU2_IRQn, NRF_IRQ_PRIO_LOW);
  NVIC_ClearPendingIRQ(SWI2_EGU2_IRQn);
  NVIC_EnableIRQ(SWI2_EGU2_IRQn);
}
#endif

static void board_clock_init(void) {
#if !defined(SOFTDEVICE_PRESENT)
  // Start internal or external LFCLK for RTC1 when SoftDevice is not present
  NRF_CLOCK->TASKS_LFCLKSTOP = 1UL;
  NRF_CLOCK->EVENTS_LFCLKSTARTED = 0UL;
#if defined(BLE_LFCLK_SOURCE_RC)
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_RC << CLOCK_LFCLKSRC_SRC_Pos;
#elif defined(BLE_LFCLK_SOURCE_SYNTH)
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Synth << CLOCK_LFCLKSRC_SRC_Pos;
#else
  NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
#endif
  NRF_CLOCK->TASKS_LFCLKSTART = 1UL;
  while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0UL)
    ;

#if !defined(NRF_DISABLE_DCDC)
  NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Enabled << POWER_DCDCEN_DCDCEN_Pos;
#endif
#endif
}

static void board_usb_init(void) {
  // Set USBD IRQ priority to highest application-safe level for SoftDevice
  NVIC_SetPriority(USBD_IRQn, NRF_IRQ_PRIO_HIGH);

  uint32_t usb_reg = 0;
#if defined(SOFTDEVICE_PRESENT)
  if (sd_enabled) {
    (void)sd_power_usbdetected_enable(1U);
    (void)sd_power_usbpwrrdy_enable(1U);
    (void)sd_power_usbremoved_enable(1U);
    (void)sd_power_usbregstatus_get(&usb_reg);
  } else
#endif
  {
    NRF_POWER->INTENSET = POWER_INTENSET_USBDETECTED_Msk |
                          POWER_INTENSET_USBPWRRDY_Msk |
                          POWER_INTENSET_USBREMOVED_Msk;
    NVIC_SetPriority(POWER_CLOCK_IRQn, NRF_IRQ_PRIO_LOW);
    NVIC_EnableIRQ(POWER_CLOCK_IRQn);
    usb_reg = NRF_POWER->USBREGSTATUS;
  }

  // Handle USB cable already connected before boot
  if ((usb_reg & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0UL) {
    usb_vbus_present = true;
    tusb_hal_nrf_power_event(NRF_USB_EVT_DETECTED);
  }
  if ((usb_reg & POWER_USBREGSTATUS_OUTPUTRDY_Msk) != 0UL) {
    tusb_hal_nrf_power_event(NRF_USB_EVT_READY);
  }
}

bool nrf52_is_sd_enabled(void) { return sd_enabled; }

bool nrf52_is_usb_vbus_present(void) { return usb_vbus_present; }

__attribute__((weak)) void nrf52_ble_sd_evt_poll(void) {}

void nrf52_soc_evt_poll(void) {
#if defined(SOFTDEVICE_PRESENT)
  if (!sd_enabled)
    return;

  uint32_t soc_evt = 0;
  while (sd_evt_get(&soc_evt) == NRF_SUCCESS) {
    switch (soc_evt) {
    case NRF_EVT_POWER_USB_DETECTED:
      usb_vbus_present = true;
      tusb_hal_nrf_power_event(NRF_USB_EVT_DETECTED);
      break;

    case NRF_EVT_POWER_USB_POWER_READY:
      tusb_hal_nrf_power_event(NRF_USB_EVT_READY);
      break;

    case NRF_EVT_POWER_USB_REMOVED:
      usb_vbus_present = false;
      tusb_hal_nrf_power_event(NRF_USB_EVT_REMOVED);
      break;

    case NRF_EVT_FLASH_OPERATION_SUCCESS:
    case NRF_EVT_FLASH_OPERATION_ERROR:
      nrf52_flash_sd_evt_handler(soc_evt);
      break;

    default:
      break;
    }
  }
#endif
}

void board_init(void) {
#if defined(SOFTDEVICE_PRESENT)
  board_softdevice_init();
#endif
  board_clock_init();
  board_usb_init();

  // Enable Cortex-M4 DWT cycle counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;
}

void board_error_handler(void) {
  __disable_irq();
  while (1)
    ;
}

void board_reset(void) { NVIC_SystemReset(); }

void board_enter_bootloader(void) {
#if defined(SOFTDEVICE_PRESENT)
  if (sd_enabled) {
    // Write Nordic DFU bootloader magic (0xB1) into GPREGRET via SoftDevice
    (void)sd_power_gpregret_clr(0, 0xFFUL);
    (void)sd_power_gpregret_set(0, BOOTLOADER_MAGIC);
  } else
#endif
  {
    NRF_POWER->GPREGRET = BOOTLOADER_MAGIC;
  }

  NVIC_SystemReset();
}

uint32_t board_serial(char *buf) {
  // Use the 64-bit factory device ID (NRF_FICR->DEVICEID) as serial number
  const uint32_t id[2] = {NRF_FICR->DEVICEID[1], NRF_FICR->DEVICEID[0]};
  const uint8_t *bytes = (const uint8_t *)id;

  for (uint32_t i = 0; i < 8; i++) {
    buf[i * 2] = (char)M_HEX(bytes[i] >> 4);
    buf[i * 2 + 1] = (char)M_HEX(bytes[i] & 0x0FU);
  }

  return 16;
}

uint32_t board_cycle_count(void) { return DWT->CYCCNT; }

//--------------------------------------------------------------------+
// Interrupt Handlers
//--------------------------------------------------------------------+

void USBD_IRQHandler(void) { tud_int_handler(0); }

#if defined(SOFTDEVICE_PRESENT)
void SWI2_EGU2_IRQHandler(void) {
  nrf52_soc_evt_poll();
  nrf52_ble_sd_evt_poll();
}
#else
void POWER_CLOCK_IRQHandler(void) {
  if (NRF_POWER->EVENTS_USBDETECTED != 0UL) {
    NRF_POWER->EVENTS_USBDETECTED = 0UL;
    usb_vbus_present = true;
    tusb_hal_nrf_power_event(NRF_USB_EVT_DETECTED);
  }
  if (NRF_POWER->EVENTS_USBPWRRDY != 0UL) {
    NRF_POWER->EVENTS_USBPWRRDY = 0UL;
    tusb_hal_nrf_power_event(NRF_USB_EVT_READY);
  }
  if (NRF_POWER->EVENTS_USBREMOVED != 0UL) {
    NRF_POWER->EVENTS_USBREMOVED = 0UL;
    usb_vbus_present = false;
    tusb_hal_nrf_power_event(NRF_USB_EVT_REMOVED);
  }
}
#endif
