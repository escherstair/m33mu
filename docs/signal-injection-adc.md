# m33mu — External Signal Injection Design: ADC

Status: **design agreed, not implemented.** Register-level facts are checked
against RM0517 Rev 1. Values marked *(confirm)* need the STM32H5F4 datasheet;
§9 lists them.
Depends on: [signal-injection-gpio.md](signal-injection-gpio.md). This phase
reuses the `.sigc` container, `--signal-bind`, `--signal-sync` and the replay
clock of `src/signal.c` without changing their shape. Read that doc first; this
one only covers what is ADC-specific.

Scope: a new shared ADC model, `cpu/stm32_adc.c`, that converts the voltage of
an injected pin into a digital code on the same time axis as GPIO injection.
One capture can drive GPIO pins and ADC inputs at the same time. The goal is
that unmodified STM32CubeH5 HAL code (`HAL_ADC_Init`, `HAL_ADCEx_Calibration_Start`,
`HAL_ADC_ConfigChannel`, `HAL_ADC_Start`, `HAL_ADC_PollForConversion`,
`HAL_ADC_Start_IT`, `HAL_ADC_IRQHandler`, `HAL_ADC_Stop`) runs without hanging
or taking an error path.

---

## 1. Decisions

| # | Topic | Decision | Rationale |
|---|---|---|---|
| 1 | First target | **STM32H5F4** (RM0517). H563/H533 (RM0481) follow on the same `cpu/stm32_adc.c`, with per-SoC glue for base addresses, IRQ numbers and the channel table | H5F4 is what the production firmware runs on. The ADC IP is the same across the H5 family |
| 2 | Instances | Map **all three ADCs**, all with the full model. ADC1 (master) and ADC2 (slave) share one block at 0x4202 8000: ADC1 at +0x000, ADC2 at +0x100, common registers at +0x300. ADC3 is independent, at 0x4202 D800 with the same layout and its own common registers. The secure aliases are at 0x5202 xxxx. IRQs: ADC1 37, ADC2 69, ADC3 149. Dual mode (`CCR.DUAL`) is stored but not modelled | HAL reads the other instance's `CR` when it decides whether it may write `CCR`. An unmapped instance faults |
| 3 | Conversion modes | **Software start, polled**, then **EOC/EOS interrupts**, then **continuous mode** (`CFGR.CONT`). **Continuous + DMA is a separate PR** | These cover the production firmware. DMA needs GPDMA peripheral requests (`REQSEL`, one beat per request, circular mode), which `cpu/stm32_gpdma.c` lacks today |
| 4 | Channels | **Regular scan sequence**: `SQR1.L` + 1 conversions, channels from `SQR1..SQR4`. The injected group is out of scope: `JADSTART` writes are ignored | The production firmware scans several channels |
| 5 | Calibration and ready | **Fixed delays**: `ADCAL` clears after t_CAL, and `ADRDY` sets t_STAB after `ADEN` *(confirm both)*. The full `CR` state machine is in §2.2 | HAL polls both flags with timeouts. Fixed delays keep start-up timing realistic |
| 6 | ADC clock | **HCLK synchronous only**: `ADC_CCR.CKMODE` = 01/10/11 → HCLK/1, /2, /4. `CKMODE = 00` prints a one-time "unsupported" warning and runs as HCLK/1. A one-time warning also fires when f_ADC exceeds the datasheet maximum *(confirm)*. `RCC_CCIPR5.ADCDACSEL`, `CCR.PRESC` and the HPRE = 1 requirement of HCLK/1 (RM0517 §30.4.3) are not modelled | The production firmware uses the synchronous clock, which gives an exact integer ratio to CPU cycles |
| 7 | Conversion timing | **Sampled at the end of the sampling time**, when `EOSMP` sets. `EOC` follows after the resolution time. §3 has the arithmetic | Matches silicon. With long sampling times, sampling at slot start would be about 10 µs early |
| 8 | Quantizer | `code = clamp(round(mV · (2^res − 1) / vref_mv), 0, 2^res − 1)`, then the offset stage and alignment of §2.5. Out-of-range input **saturates**, it never wraps | Ideal by default, so tests assert exact codes |
| 9 | Noise | Off by default. **`--adc-noise-lsb=<n>`** adds uniform ±n LSB before the clamp, from a PRNG seeded by **`--adc-seed=<s>`** (default 0). No offset or gain error model | Robustness tests that stay reproducible |
| 10 | Reference voltage | **`--adc-vref-mv`**, defaulting to `--vdd-mv` | On H5, full scale is `VREF+`, which may differ from `VDD` |
| 11 | Binding | **By pin**: `--signal-bind:trace=ain0:target=PA0`, with no new role. The per-SoC table maps **(instance, channel) → pin**, because some channel numbers sit on different pins on ADC1 and ADC2. Binding **two traces to the same pin is rejected** | Firmware and schematics name pins. One pad voltage feeds both the digital input and the ADC |
| 12 | Pin mode | The ADC sees the injected voltage only when the pin's `MODER` is analog (`11`). A channel with no pin in the table, an unbound pin, or a pin not in analog mode converts as 0 mV. An analog pin reads 0 in `IDR`, as on silicon | Matches the hardware, and catches firmware that forgets analog mode |
| 13 | Sampling API | `mm_signal_sample_mv(bank, pin, cycles_ahead, &mv)`; §4 | The ADC pulls one value per conversion. GPIO keeps its precomputed crossings |
| 14 | `full_scale_mv` | **Metadata only.** The emulator never reads it | It records the capture's range, not the emulated ADC's |
| 15 | Trigger fragility | **Already handled** by `--signal-sync`: an unreached sync PC fails the run with exit code 1. Test firmware anchors on a `noinline` marker | Same mechanism as GPIO |
| 16 | Internal channels | **Out of scope.** VREFINT, VSENSE (temperature), VBAT/4 and VDDCORE convert as 0 mV, and no calibration values are provided. The first conversion of an internal channel prints a **one-time warning**. RM0517 §30.4.4 places them at ADC1 IN17, ADC2 IN16/IN17 and ADC3 IN14–IN17; the per-SoC table marks them | Firmware that derives VDDA from VREFINT would divide by zero, which gives 0 on M33 without a fault. The warning makes that visible |
| 17 | Snapshots | **Debugger reads must be side-effect free**: a `peek` of `DR` does not clear `EOC`. Save/load and reverse stepping are **deferred**, as for GPIO injection | `core_interfaces.c` falls back to `read` when there is no `peek`. The GPIO doc already defers replay-position snapshots |
| 18 | Clock gating | While the clock enable is clear (`RCC_AHB2ENR` bit 10 `ADCEN` for ADC1/2, bit 24 `ADC3EN` for ADC3), register reads return 0, writes are ignored and the ADC does not advance | Same rule as `stm32_gpio.c` |

## 2. Register model (`cpu/stm32_adc.c`)

### 2.1 Storage

Each instance's registers (offsets 0x000–0x0FC) and the common registers are
backed by storage, so every register HAL writes reads back: `CFGR2` (0x10),
`DIFSEL`, `CALFACT`, `TRx`, `AWDxCR`, `OR` (0xC8), and every `CCR` field
(`VBATEN`, `TSEN`, `VREFEN`, `PRESC`, `CKMODE`, `MDMA`, `DMACFG`, `DELAY`, `DUAL`).
Reset values follow RM0517 Table 303: `CR` = 0x2000 0000 (`DEEPPWD` set) and
`CFGR` = 0x8000 0000 (`JQDIS` set). `CSR` is a read-only live mirror of the
master and slave `ISR` bits. Only the bits below have behaviour.

### 2.2 `CR`

`CR` is not a plain register. LL writes it with
`MODIFY_REG(CR, ADC_CR_BITS_PROPERTY_RS, bit)`, so a write that sets one bit
writes 0 to the others.

| Bits | Behaviour |
|---|---|
| `ADVREGEN`, `DEEPPWD`, `ADCALDIF` | Read/write, not checked. Writing `DEEPPWD = 1` also clears `ADVREGEN` |
| `ADEN` | Set-only; writing 0 is ignored. Accepted only when `ADCAL`, `ADSTART`, `JADSTART`, `ADSTP`, `JADSTP` and `ADDIS` are all 0, and not during the 4 ADC clocks after `ADCAL` clears. Sets `ISR.ADRDY` t_STAB later. Rewriting 1 while waiting does not restart the delay. Cleared only by `ADDIS` |
| `ADDIS` | Set-only. Accepted only when `ADEN = 1` and no conversion is ongoing. Clears `ADEN` and `ADDIS` immediately. `ADRDY` stays set; software clears it (RM0517 §30.4.9) |
| `ADCAL` | Set-only. Accepted only when `ADEN = 0`. Cleared by hardware after t_CAL. `CALFACT` keeps its stored value |
| `ADSTART` | Set-only. Accepted only when `ADEN = 1`, `ADDIS = 0` and `ADSTP = 0`. Starts the sequence. Cleared by hardware at `EOS` when `CONT = 0`, and when `ADSTP` completes |
| `ADSTP` | Set-only. Accepted only when `ADSTART = 1`. Aborts the conversion in progress without updating `DR`, then clears `ADSTP` and `ADSTART` immediately |
| `JADSTART`, `JADSTP` | Writes ignored; always read 0 |

### 2.3 `ISR`, `IER` and the interrupt

- `ISR`: `ADRDY`, `EOSMP`, `EOC`, `EOS`, `OVR` have behaviour. All `ISR` bits are write-1-to-clear. Reading `DR` clears `EOC`.
- The IRQ is level-like. After any change to `ISR` or `IER`, the instance pends its NVIC line when `ISR & IER` is non-zero. That includes enabling an interrupt while its flag is already set.

### 2.4 Sequence and overrun

- `SMPR1`/`SMPR2` give each channel's sampling time. `SQR1..SQR4` give the order.
- `CFGR` fields with behaviour: `RES`, `ALIGN`, `CONT`, `OVRMOD`. `EXTEN`/`EXTSEL`, `DMAEN`, `DMACFG`, `DISCEN` and `AUTDLY` are stored but ignored.
- Overrun (RM0517 §30.4.26): `DR` is backed by a three-stage FIFO.
  - With `OVRMOD = 0`, results queue in the FIFO. Reading `DR` pops the oldest one. `EOC` stays set while the FIFO is not empty *(interpretation: the RM text is ambiguous here)*. A result that arrives with the FIFO full sets `OVR` and is discarded. The sequence continues.
  - With `OVRMOD = 1`, the FIFO is off. Each result overwrites `DR`, and `OVR` sets when the previous result was unread.

### 2.5 Offset and alignment

- `OFR1..4` (0x60–0x6C) each have `OFFSET_EN` (31), `OFFSET_CH` (30:26), `SATEN` (25), `OFFSETPOS` (24) and `OFFSET` (11:0). An enabled `OFRx` whose `OFFSET_CH` matches the converted channel applies to the result.
  - `OFFSET` is on the 12-bit scale at every resolution. At lower resolutions the result is `code − (OFFSET >> (12 − res))` (RM0517 Table 294). `OFFSETPOS = 1` adds instead of subtracting.
  - With `SATEN = 1`, the result saturates to `[0, 2^res − 1]` and is unsigned. With `SATEN = 0`, it is signed.
- Alignment and sign extension follow RM0517 Figures 270–273 exactly, and the unit tests take their expected bit patterns from those figures.
  - `ALIGN = 0` right-aligns.
  - `ALIGN = 1` left-aligns to the half-word, except at 6-bit resolution, which aligns to the byte.
  - Signed results put the sign extension in the upper bits.
- Oversampling (`ROVSE`, `JOVSE`) is out of scope. When it is enabled, RM0517 ignores `ALIGN` and the offsets; the model warns once and converts without oversampling.

## 3. Timing and clock

- **ADC clock:** f_ADC = HCLK / {1, 2, 4} from `CKMODE`. All ADC delays count in half ADC clocks, then convert to CPU cycles: `cycles = half_clocks · ckdiv / 2`, rounded up.
- **Sampling time:** `SMPx` ∈ {2.5, 6.5, 12.5, 24.5, 47.5, 92.5, 247.5, 640.5} ADC clocks. `SMPR1.SMPPLUS` (bit 31) replaces 2.5 with 3.5 (RM0517 §30.4.12).
  - RM0517 shifts the real sampling instant by 0.5 clock in scan and continuous mode without changing the total time. The model ignores that shift.
  - `BULB` and `SMPTRIG` are stored but not modelled.
- **Resolution time:** 12.5 / 10.5 / 8.5 / 6.5 ADC clocks for 12/10/8/6 bit (RM0517 Table 293).
- **Each conversion slot:**
  1. The slot starts.
  2. After the sampling time, the pin is sampled and `EOSMP` sets.
  3. After the resolution time, the code is written to `DR` and `EOC` sets.
  4. After the last channel, `EOS` sets.
  5. In continuous mode the next slot starts straight away.
- **Start granularity:** the ADC advances in the timer tick at the end of each executed batch, at most one translation block. A start written during a batch therefore takes effect at the end of that batch, so flags are never early relative to the write. This is the same granularity as GPIO edges.

## 4. Signal API

```c
mm_bool mm_signal_sample_mv(int bank, int pin, mm_u64 cycles_ahead, mm_i32 *mv_out);
```

- **Return value:** the zero-order-hold value of the trace bound to the pin, at `cycles_ahead` CPU cycles after the replay clock's current position. It returns `MM_FALSE` when the pin has no binding.
- **Why relative:** the ADC tick runs inside `mm_timer_tick`, which comes before `mm_signal_advance` at every time site in `main.c`. The replay clock therefore still stands at the start of the batch the ADC is processing. The conversion from cycles to ns uses the current CPU clock, like the existing replay clock.
- **Before the trigger:** a time before the trigger returns sample 0, including when the trigger fires later in the same batch.
- **After the end:** the last sample is held after the trace ends.
- **Empty traces:** a trace with no samples returns 0 mV.
- **Lookup:** uniform timebases are indexed directly; explicit timestamps use a binary search.

## 5. Integration

- **Tick:** `mm_stm32h5f4_timers_tick` (`STM32H5F4_TIMER_TICK`) calls the ADC tick. That covers every time site without touching `main.c` or `mm_target_cfg`.
- **Idle bound:** the WFI fast-forward in `main.c` sizes each jump from SysTick and `mm_signal_cycles_until_next_event()` only. It gains one more bound, `mm_target_cycles_until_next_event()` in `src/target_hal.c`. The ADC registers its next-deadline callback there from its init.
  - A registered callback avoids a new `mm_target_cfg` field, which would mean editing every `cpu_db.c` entry.
  - Without this bound, an `EOC` interrupt during WFI would arrive up to one SysTick period late.
- **Fast path when idle:** the ADC caches its next deadline. The tick costs one comparison while nothing is converting.
- **Binding:** `--signal-bind` stays as it is. ADC glue looks up the bound pin through the channel table at conversion time. The duplicate-pin check goes into `mm_signal_bind`.

## 6. Implementation order

1. `mm_signal_sample_mv` and the duplicate-pin check in `src/signal.c`, with unit tests.
2. `cpu/stm32_adc.c/.h`: storage, the `CR` state machine, calibration/ready delays, single software conversion, polled, peek. `tests/stm32_adc_test.c`.
3. H5F4 glue: instances, base addresses, IRQ numbers and channel table from RM0517 and the datasheet; MMIO registration; clock gating; tick call; idle-bound callback.
4. CLI: `--adc-vref-mv`, `--adc-noise-lsb`, `--adc-seed`; `docs/cli-usage.md`, `m33mu.1`.
5. Scan sequence, `EOC`/`EOS`/`OVR` interrupts, continuous mode, offset and alignment.
6. H563/H533 glue.

## 7. Testing

Host unit tests (`tests/stm32_adc_test.c`, `tests/signal_test.c`):

- `CR` state machine driven with HAL's write patterns (`MODIFY_REG` with the RS mask): enable, calibrate, start, stop, disable, re-enable; writes that must be ignored.
- Conversion timing in CPU cycles for each `CKMODE`, `SMPx` and resolution; `EOSMP` at the sampling instant.
- Quantizer: resolutions, saturation, offset (both signs, `SATEN` on and off), alignment; seeded noise gives identical codes for the same seed.
- Scan order, `EOC`/`EOS`, overrun under both `OVRMOD` values; level-like IRQ when `IER` is written with a flag already set.
- A `peek` of `DR` leaves `EOC` set.
- `mm_signal_sample_mv`: before trigger, ZOH, last-sample hold, empty trace, explicit timebase; duplicate-pin binding rejected.

Firmware tests on H5F4, each anchored on a `--signal-sync` marker. They are bare
metal but reproduce HAL's register access sequences. All assert **exact codes**,
using traces that are flat around each sampling instant.

- `adc-single`: polled single conversions, back to back, with a channel reconfiguration between them.
- `adc-scan-irq`: three channels on different traces, read in the `EOC` handler; checks order and values.
- `adc-wfi`: firmware sleeps in WFI until the `EOC` interrupt; checks the wake-up time against the expected conversion time.
- `adc-continuous`: conversion rate matches `SMPR` and `CKMODE`; an overrun with each `OVRMOD`.
- `adc-stop-disable`: stop mid-sequence, disable, recalibrate, re-enable, convert again.
- `gpio-adc-shared-trace`: one trace on two pins, one analog and one digital. Each is read on a flat segment; the expected tolerance is one batch.
- Negative: a pin not in analog mode converts 0; `CKMODE = 00` prints its warning; two traces on one pin are rejected.

Running the production image is the final HAL compatibility check.

## 8. Deferred

- Continuous + DMA, with GPDMA peripheral requests (separate PR).
- Timer-triggered conversion (`EXTSEL`/`EXTEN`), injected group, analog watchdogs, oversampling, the asynchronous kernel clock.
- Internal channels (VREFINT, temperature sensor, VBAT) with their calibration values.
- Save/load and reverse stepping of ADC state.
- STM32U5/L5 ADC.

## 9. To confirm from ST documents before implementing

RM0517 Rev 1 answered the register-level questions: instances, addresses,
IRQs, clock enables, `ADRDY` after `ADDIS`, `SMPPLUS`, alignment and the `OFRx`
layout. The rest needs the STM32H5F4 datasheet:

- The (instance, channel) → pin table.
- t_CAL and t_STAB.
- The maximum f_ADC. RM0517 gives 5 Msps at 12 bit, which with 15 clocks per conversion implies about 75 MHz.

To check in the CMSIS device header: whether ADC3's common registers are named
`ADC3_COMMON` at 0x4202 DB00.
