# Performance: Rust rewrite vs. C original

Status of the comparison between this Rust firmware and the C original (`ac063bb`, the
commit before the rewrite), plus what remains to be measured once hardware is available.

## 1. Measured so far (no hardware)

All numbers are from the 67-key **he60** configuration. Lower is better.

**How they were produced**

- **Cortex-M4 counts:** the shared firmware core (matrix scan, layout, advanced keys, HID,
  XInput, eeconfig) was cross-compiled for both firmwares, at C `-Os` and Rust `opt-level = "s"`.
  It runs in an instruction-level emulator (Unicorn). An approximate Cortex-M4 cycle model is
  applied on top: zero-wait-state memory, early-terminating `UDIV` from the real operands, and
  load/store pipelining. Use the counts for relative comparison, not as absolute timings.
- **ADC input:** samples come from a per-key array, as in the firmware.
- **Firmware struct:** the Rust `Firmware` sits behind an opaque pointer, as with `StaticCell`.

### 1.1 Cycles per tick (Cortex-M4 model)

A **full tick** is matrix scan + layout + HID + XInput, i.e. one main-loop iteration
without USB servicing.

| Scenario | Metric | C | Rust | Δ |
|---|---|---|---|---|
| Idle | Scan, cycles | 5,451 | 3,866 | −29% |
| | Full tick, cycles | 11,747 | 6,093 | −48% |
| Keys moving, Rapid Trigger off | Scan, cycles | 5,770 | 5,204 | −10% |
| | Full tick, cycles | 12,808 | 8,673 | −32% |
| Keys moving, Rapid Trigger 20 | Scan, cycles | 7,608 | 6,397 | −16% |
| | Full tick, cycles | 14,414 | 9,647 | −33% |
| Keys moving, continuous RT 10/30 | Scan, cycles | 7,803 | 6,616 | −15% |
| | Full tick, cycles | 14,694 | 10,071 | −31% |

### 1.2 Full-tick time at core clock (µs, derived from 1.1)

| Scenario | STM32F446 @ 180 MHz (C / Rust) | AT32F405 @ 216 MHz (C / Rust) | nRF52840 @ 64 MHz (C / Rust) |
|---|---|---|---|
| Idle | 65.3 / 33.9 | 54.4 / 28.2 | 183.5 / 95.2 |
| Moving, RT 20 | 80.1 / 53.6 | 66.7 / 44.7 | 225.2 / 150.7 |

### 1.3 Memory (shared core only, bytes)

| Metric | C | Rust |
|---|---|---|
| Flash (text + data) | 13,939 | 18,884 (≈2,300 B of it is `compiler_builtins` `memcpy`/`memmove`) |
| RAM | 12,487 | 12,204 |

### 1.4 Behaviour equivalence (host, 2,000,000 ticks)

- **Pressed key-samples:** identical in both, 103,200,000.
- **Matrix scan:** `matrix_scan_matches_c_reference` checks it bit-exactly against a port of
  the C `matrix_scan()` over 20k random ticks, covering all Rapid Trigger modes.

## 2. Not yet measured: ADC sampling rate

The full-matrix sample rate is the end-to-end quantity that matters for latency. So far it is
only **estimated from datasheet timings**:

| MCU (keyboard) | C | Rust |
|---|---|---|
| STM32F446 (he60, 9 inputs × 8 mux channels) | ≈26 µs per mux step, ≈4.8 kHz | ≈27 µs per mux step, ≈4.6 kHz |
| nRF52840 (2 inputs × 8) | ≈30 µs per mux step, ≈4.2 kHz | ≈30–31 µs per mux step, ≈4.1 kHz |
| AT32F405 (he16, 2 inputs × 8) | ≈21.5 µs per mux step, ≈5.8 kHz | same pipeline, ≈5.8 kHz |

The small Rust deficit on STM32 and nRF is an **assumed** ≈1 µs per step, not a measurement:

- **C** runs the whole settle → convert → store → mux-switch chain inside interrupt handlers.
- **Rust (STM32, nRF)** has an async task in the chain. Each step adds a time-driver alarm or
  timer IRQ wake-up, an executor poll, and a DMA/SAADC completion wake-up.
- **Rust (AT32)** uses the same interrupt chain as C.

## 3. Future investigation (needs hardware)

### 3.1 Measure with the `scan-probe` GPIO instrumentation

All three firmware crates have an opt-in `scan-probe` feature. Without it the probes compile
to nothing (binary sizes are byte-identical).

| Probe | Variable | Waveform |
|---|---|---|
| Scan | `LIBHMK_PROBE_SCAN_PIN` (required with the feature) | High when the mux select lines change, low once that step's samples are stored. **Period** = time per mux step; **high time** ≈ settle + conversion; **low time** ≈ software overhead between steps. |
| Main loop | `LIBHMK_PROBE_LOOP_PIN` (optional) | Toggles once per main-loop iteration: loop rate = 2 × toggle frequency. |

Pins use `keyboard.json` naming. The build fails if a pin is used by the keyboard's
analog/mux setup or is reserved by the driver (USB, SWD, crystal, reset). Check the board
schematic as well: a pin can be free in `keyboard.json` but wired to something on the PCB.

```bash
LIBHMK_KEYBOARD=he60 LIBHMK_PROBE_SCAN_PIN=B5 LIBHMK_PROBE_LOOP_PIN=B6 cargo build -p libhmk-stm32f446 --features scan-probe --target thumbv7em-none-eabihf --release
```

```bash
LIBHMK_KEYBOARD=he16 LIBHMK_PROBE_SCAN_PIN=B5 LIBHMK_PROBE_LOOP_PIN=B6 cargo build -p libhmk-at32f405 --features scan-probe --target thumbv7em-none-eabihf --release
```

```bash
LIBHMK_KEYBOARD=nrf52840-dk-he16 LIBHMK_PROBE_SCAN_PIN=P0.26 LIBHMK_PROBE_LOOP_PIN=P0.27 cargo build -p libhmk-nrf52840 --features scan-probe --target thumbv7em-none-eabihf --release
```

**Procedure** (logic analyzer at ≥ 10 MS/s, or a scope):

1. **Rust:** flash the probe build, let calibration finish (500 ms), and capture ≥ 100 ms with
   keys at rest, then while pressing keys.
2. **Rust results:** record the mux-step period, high time and low time (mean and max). Full
   matrix rate = 1 / (8 × step period) for 3 select pins. Also record the loop rate.
3. **C reference:** build the C firmware from `ac063bb` with an equivalent probe:
   - **STM32F446**, `src/hardware/stm32f446xx/analog.c`, `HAL_ADC_ConvCpltCallback`: drive
     the pin low after the `adc_values[]` store loop and high right after the mux
     `HAL_GPIO_WritePin` loop.
   - **nRF52840**, `src/hardware/nrf52840xx/analog.c`, `SAADC_IRQHandler`: low after the
     `adc_values[]` store, high after the mux select pins are written.
   - **AT32F405**, `src/hardware/at32f405xx/analog.c`, `DMA1_Channel1_IRQHandler`: low after
     the store, high after `gpio_bits_write` of the select pins.
   - **Main loop:** toggle the loop pin once per `while (1)` iteration in `src/main.c`.
4. **Compare** both firmwares on the same board and with the same `ADC_SAMPLE_DELAY` (default
   20 µs). On nRF, also check the high time is never below the configured settle delay; the
   old RTC-based delay could be as short as 0 µs.

**Decision criteria.** Consider option 3.2 only if:

- the Rust mux-step period exceeds C's by more than ≈1 µs (≈4%) on the same board, or
- the Rust low time (software overhead) is clearly above C's.

Otherwise the async design costs nothing measurable and should stay.

### 3.2 Option: hardware-triggered conversions (only if 3.1 shows a gap)

This moves the settle → convert hand-off out of software. It removes one wake-up per mux step.

| MCU | Mechanism | Notes |
|---|---|---|
| nRF52840 | PPI channel: TIMER1 `COMPARE[0]` → SAADC `TASKS_SAMPLE` (SAADC started beforehand) | Cleanest option; also saves one IRQ and wake-up per step on battery. The BLE stack uses PPI 17–31, so 0–16 are free. |
| STM32F446 | One-pulse timer whose TRGO is the ADC1 regular external trigger (`EXTSEL`/`EXTEN`), replacing `SWSTART` | ADC1 can only be triggered by TIM1/2/3/4/5/8 events. Pin the embassy time driver to a different timer (e.g. `time-driver-tim5` instead of `time-driver-any`). |
| AT32F405 | Already interrupt driven like C | Only worth doing if 3.1 shows AT32 itself is slower. |

Costs and risks:

- **Gain is small:** at most ≈1 µs of ≈21–30 µs per step, ≈8 µs per full-matrix pass. That's
  negligible against the 1 ms full-speed USB poll interval, and only marginal against 125 µs
  high-speed polling.
- **Only half the path goes away:** storing the samples, switching the mux and re-arming the
  timer still run in software. Moving those to hardware needs DMA writing GPIO BSRR, which is
  much more complex.
- **More chip-specific unsafe code:** register-level code outside the HAL, more sensitive to
  HAL upgrades.
- **Silent failure modes:** wrong trigger edges, DMA re-arm races and ADC overrun only show
  up on hardware. A misconfigured trigger stops or mistimes conversions without an error.

### 3.3 Other hardware checks pending

- **AT32F405 USB:** `embassy-usb-synopsys-otg` with the AT32-specific GCCFG setup on OTGHS and
  OTGFS1. Check enumeration and polling interval: 1 = 8 kHz, 4 = 1 kHz with high polling
  rate off.
- **AT32F405 ADC:** the TMR6 + DMA1 scanner. Check all mux channels read the expected keys.
- **STM32F446 ADC DMA sequence:** check no channel shift and no overrun under USB load.
- **nRF52840 TIMER1 settle delay and battery sampling:** check the 10 s battery interval.
  Also check SAADC recalibration on each battery measurement doesn't cause a visible gap
  (expected < 1 ms every 10 s).
