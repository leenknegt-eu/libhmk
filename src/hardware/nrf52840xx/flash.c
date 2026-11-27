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

#if defined(SOFTDEVICE_PRESENT)
#include "nrf_error.h"
#include "nrf_soc.h"
#endif

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

typedef enum {
  FLASH_OP_IDLE = 0,
  FLASH_OP_PENDING,
  FLASH_OP_SUCCESS,
  FLASH_OP_ERROR,
} flash_op_state_t;

// Write in bounded word-aligned chunks so unaligned caller buffers work safely
// and SoftDevice can interleave BLE radio events between flash chunks.
#define FLASH_WRITE_CHUNK_WORDS 64U

static volatile flash_op_state_t flash_op_state = FLASH_OP_IDLE;
static uint32_t flash_write_chunk[FLASH_WRITE_CHUNK_WORDS];

void nrf52_flash_sd_evt_handler(uint32_t soc_evt) {
#if defined(SOFTDEVICE_PRESENT)
  if (soc_evt == NRF_EVT_FLASH_OPERATION_SUCCESS)
    flash_op_state = FLASH_OP_SUCCESS;
  else if (soc_evt == NRF_EVT_FLASH_OPERATION_ERROR)
    flash_op_state = FLASH_OP_ERROR;
#else
  (void)soc_evt;
#endif
}

#if defined(SOFTDEVICE_PRESENT)
static bool flash_sd_wait(void) {
  while (flash_op_state == FLASH_OP_PENDING) {
    nrf52_soc_evt_poll();
    if (flash_op_state == FLASH_OP_PENDING)
      (void)sd_app_evt_wait();
  }

  const bool ok = (flash_op_state == FLASH_OP_SUCCESS);
  flash_op_state = FLASH_OP_IDLE;
  return ok;
}
#endif

static void nvmc_wait_ready(void) {
  while ((NRF_NVMC->READY & NVMC_READY_READY_Msk) ==
         (NVMC_READY_READY_Busy << NVMC_READY_READY_Pos))
    ;
}

void flash_init(void) { flash_op_state = FLASH_OP_IDLE; }

bool flash_erase(uint32_t sector) {
  if (sector >= FLASH_NUM_SECTORS)
    return false;

  // Never allow erasing the MBR or SoftDevice flash pages
  if (sector * FLASH_SECTOR_SIZE < NRF_SD_FLASH_SIZE)
    return false;

#if defined(SOFTDEVICE_PRESENT)
  if (nrf52_is_sd_enabled()) {
    uint32_t err;
    do {
      flash_op_state = FLASH_OP_PENDING;
      err = sd_flash_page_erase(sector);
      if (err == NRF_ERROR_BUSY) {
        nrf52_soc_evt_poll();
        (void)sd_app_evt_wait();
      }
    } while (err == NRF_ERROR_BUSY);

    if (err != NRF_SUCCESS) {
      flash_op_state = FLASH_OP_IDLE;
      return false;
    }

    return flash_sd_wait();
  }
#endif

  nvmc_wait_ready();
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos;
  nvmc_wait_ready();
  NRF_NVMC->ERASEPAGE = FLASH_BASE + sector * FLASH_SECTOR_SIZE;
  nvmc_wait_ready();
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
  nvmc_wait_ready();

  return true;
}

bool flash_read(uint32_t addr, void *buf, uint32_t len) {
  if (addr + len * 4UL > FLASH_SIZE)
    return false;

  memcpy(buf, (const void *)(uintptr_t)(FLASH_BASE + addr), len * 4UL);
  return true;
}

bool flash_write(uint32_t addr, const void *buf, uint32_t len) {
  if ((addr & 3UL) != 0UL || addr < NRF_SD_FLASH_SIZE ||
      addr + len * 4UL > FLASH_SIZE)
    return false;

  const uint8_t *src = (const uint8_t *)buf;
  uint32_t offset_words = 0;

#if defined(SOFTDEVICE_PRESENT)
  if (nrf52_is_sd_enabled()) {
    while (offset_words < len) {
      const uint32_t chunk = M_MIN(len - offset_words, FLASH_WRITE_CHUNK_WORDS);
      memcpy(flash_write_chunk, src + offset_words * 4UL, chunk * 4UL);

      uint32_t *dst =
          (uint32_t *)(uintptr_t)(FLASH_BASE + addr + offset_words * 4UL);
      uint32_t err;
      do {
        flash_op_state = FLASH_OP_PENDING;
        err = sd_flash_write(dst, flash_write_chunk, chunk);
        if (err == NRF_ERROR_BUSY) {
          nrf52_soc_evt_poll();
          (void)sd_app_evt_wait();
        }
      } while (err == NRF_ERROR_BUSY);

      if (err != NRF_SUCCESS) {
        flash_op_state = FLASH_OP_IDLE;
        return false;
      }

      if (!flash_sd_wait())
        return false;

      offset_words += chunk;
    }
    return true;
  }
#endif

  nvmc_wait_ready();
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
  nvmc_wait_ready();

  volatile uint32_t *dst = (volatile uint32_t *)(uintptr_t)(FLASH_BASE + addr);
  for (uint32_t i = 0; i < len; i++) {
    uint32_t word = 0;
    memcpy(&word, src + i * 4UL, sizeof(word));
    dst[i] = word;
    nvmc_wait_ready();
  }

  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
  nvmc_wait_ready();

  return true;
}
