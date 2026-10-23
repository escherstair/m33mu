# m33mu — External Signal Injection Design: ADC

Status: **design agreed, not implemented.**
Depends on: [signal-injection-gpio.md](signal-injection-gpio.md). This phase
reuses the `.sigc` container, `--signal-bind`, `--signal-sync` and the replay
clock of `src/signal.c` without changing them. Read that doc first; this one
only covers what is ADC-specific.

Scope: a new shared ADC model, `cpu/stm32_adc.c`, that converts the voltage of
an injected pin into a digital code on the same time axis as GPIO injection.
One capture can drive GPIO pins and ADC inputs at the same time.

---

## 1. Decisions

| # | Topic | Decision | Rationale |
|---|---|---|---|
| 1 | First target | **STM32H5F4** (RM0517). H563/H533 (RM0481) follow on the same `cpu/stm32_adc.c`, with per-SoC glue for base addresses, IRQ numbers and the pin-to-channel table | H5F4 is what the production firmware runs on. The ADC IP is the same across the H5 family, so the shared model carries over |
| 2 | Conversion modes | **Software start, polled**, then **EOC/EOS interrupts**, then **continuous mode** (`CFGR.CONT`). **Continuous + DMA is a separate PR** | These cover the production firmware. DMA needs GPDMA peripheral requests (`REQSEL`, one beat per request, circular mode), which `cpu/stm32_gpdma.c` lacks today: it copies the whole block as soon as a channel is enabled. That work also benefits other peripherals |
| 3 | Channels | **Regular scan sequence**: `SQR1.L` + 1 conversions, channels from `SQR1..SQR4`, `EOC` after each, `EOS` after the last | The production firmware scans several channels. The injected group (`JSQR`, `JADSTART`) is out of scope |
| 4 | Calibration and ready | **Fixed delays.** Writing `ADCAL` starts a calibration that clears `ADCAL` after t_CAL ADC clocks. Writing `ADEN` sets `ISR.ADRDY` after t_STAB ADC clocks. Take both values from the H5F4 datasheet before implementing. `DEEPPWD` and `ADVREGEN` are stored but not checked | HAL polls `ADCAL` and `ADRDY`, so without them it spins forever. Fixed delays keep HAL timeouts and start-up timing realistic. The regulator start-up wait in HAL is a software delay, so it needs no model |
| 5 | ADC clock | **HCLK synchronous only**: `ADC_CCR.CKMODE` = HCLK/1, /2 or /4, derived from the existing CPU clock. `CKMODE = 00` (asynchronous kernel clock) prints a one-time "unsupported" warning and falls back to HCLK/1. `RCC_CCIPR5.ADCDACSEL` and `ADC_CCR.PRESC` are not modelled | The production firmware uses the synchronous clock. With 1 instruction = 1 cycle, HCLK/n gives an exact integer ratio to CPU cycles |
| 6 | Conversion timing | Each channel is **sampled at the start of its slot**. `EOC` follows after the `SMPRx` sampling time plus the resolution time (12.5 / 10.5 / 8.5 / 6.5 ADC clocks for 12/10/8/6 bit). Time is counted in half ADC clocks so the .5 values stay integer. In continuous mode the next sequence starts immediately | Deterministic, and close enough to silicon for IRQ timing and software that depends on sample rate. An instant conversion would make continuous mode a busy loop |
| 7 | Quantizer | `code = clamp(round(mV · (2^res − 1) / vref_mv), 0, 2^res − 1)`, with `res` from `CFGR.RES`, then `OFFSETx` subtraction and `CFGR.ALIGN`. Out-of-range input **saturates**, it never wraps | Matches real ADC behaviour. Ideal by default, so tests can assert exact codes |
| 8 | Noise | Off by default. **`--adc-noise-lsb=<n>`** adds uniform ±n LSB noise before the clamp, from a PRNG seeded by **`--adc-seed=<s>`** (default 0). No offset or gain error model | Lets you test robustness against noise while keeping every run reproducible |
| 9 | Reference voltage | **`--adc-vref-mv`**. Defaults to `--vdd-mv` if omitted, independently overridable | On H5, full scale is `VREF+`, which can be external, from `VREFBUF`, or bonded to `VDDA`. Defaulting to `VDD` covers the common case without hardcoding it |
| 10 | Binding | **By pin**: `--signal-bind:trace=ain0:target=PA0`, with no new role. The pin's ADC channel comes from a per-SoC table taken from the datasheet. One trace can be bound to several pins | Firmware and schematics name pins, not channels. A single pad voltage feeds both the digital input and the ADC |
| 11 | Pin mode | The ADC sees the injected voltage only when the pin's `MODER` is analog (`11`). An unbound channel, or a pin not in analog mode, converts as 0 mV. An analog pin reads 0 in `IDR`, as on silicon | Matches the hardware, and catches firmware that forgets to put the pin in analog mode |
| 12 | Sampling API | New `mm_signal_sample_mv(bank, pin, t_cycles)` returns the zero-order-hold value of the trace bound to that pin at `t_cycles`. Before the trigger fires it returns the trace's first sample | The ADC pulls a value at slot start, while GPIO keeps its precomputed crossings. Both use the same replay clock |
| 13 | `full_scale_mv` | **Metadata only.** The emulator never reads it; quantization uses `--adc-vref-mv` alone. The format section of the GPIO doc says so | It records the capture's range, not the emulated ADC's. Using it for scaling would tie the result to the recording instead of the board |
| 14 | Trigger fragility | **Already handled** by `--signal-sync`: a sync PC that is never reached fails the run with exit code 1. Test firmware anchors on a `noinline` marker, not on a HAL entry point that LTO could inline | Same mechanism as GPIO |
| 15 | Internal channels | **Out of scope**: VREFINT, temperature sensor and VBAT convert as 0 mV, and no `VREFINT_CAL`/`TS_CAL` values are provided | Not needed yet. Add them when firmware computes VDDA from VREFINT |

## 2. Register model (`cpu/stm32_adc.c`)

Per ADC instance:

- `ISR`: `ADRDY`, `EOSMP`, `EOC`, `EOS`, `OVR`, write-1-to-clear.
- `IER`: same bits; an enabled flag raises the instance's IRQ.
- `CR`: `ADEN`, `ADDIS`, `ADSTART`, `ADSTP`, `ADCAL`, `ADVREGEN`, `DEEPPWD`.
- `CFGR`: `RES`, `ALIGN`, `CONT`, `OVRMOD`. `EXTEN`/`EXTSEL`, `DMAEN`, `DISCEN` and `AUTDLY` are stored but ignored.
- `SMPR1`/`SMPR2`, `SQR1..SQR4`, `OFFSET1..4`, `DR`.
- Common: `ADC_CCR.CKMODE`.

Overrun: if `EOC` is still set when the next conversion ends, `OVR` is set.
With `OVRMOD = 0` `DR` keeps the old value; with `OVRMOD = 1` it is overwritten.
Reading `DR` clears `EOC`.

The model implements the `mmio_bus` `peek`, `save` and `load` hooks, like the
other stateful peripherals.

## 3. Runtime architecture

```
src/signal.c  ── mm_signal_sample_mv(bank, pin, t) ──►  cpu/stm32_adc.c
   (replay clock, ZOH lookup)                           slot start: sample, quantize
                                                        slot end:   DR, EOC/EOS, IRQ
```

The ADC advances through a tick hook next to `mm_timer_tick` at every time site
in `main.c`, so it keeps running across WFI fast-forward and the TB fast path.
A pending conversion deadline is cached so the hook costs one comparison when
nothing is converting.

## 4. Implementation order

1. `mm_signal_sample_mv` in `src/signal.c`, with unit tests (before trigger, ZOH, explicit timebase).
2. `cpu/stm32_adc.c/.h`: register model, calibration/ready delays, single software conversion, polled. `tests/stm32_adc_test.c` for the state machine and quantizer.
3. H5F4 glue: base addresses, `ADC_CCR`, IRQ number and pin-to-channel table from RM0517 and the datasheet; MMIO registration; `adc_init`/`adc_reset`/`adc_tick` in `mm_target_cfg`.
4. CLI: `--adc-vref-mv`, `--adc-noise-lsb`, `--adc-seed`; `docs/cli-usage.md`, `m33mu.1`.
5. Scan sequence, `EOC`/`EOS`/`OVR` interrupts, continuous mode.
6. H563/H533 glue.

## 5. Testing

Host unit tests: quantizer (resolutions, saturation, offset, alignment, seeded
noise), conversion and calibration timing in CPU cycles for each `CKMODE`,
scan order and flags, overrun, pin-mode check.

Firmware tests on H5F4, each anchored on a `--signal-sync` marker:

- `adc-single`: a ramp sampled by polled single conversions; codes checked within ±1 LSB of the expected value.
- `adc-scan-irq`: three channels on different traces, read in the EOC interrupt; checks order and values.
- `adc-continuous`: conversion rate matches `SMPR` and `CKMODE`.
- `gpio-adc-shared-trace`: one trace bound to two pins, one analog and one digital, read back to back; both reflect the same instant.
- Negative: pin not in analog mode converts 0.

## 6. Deferred

- Continuous + DMA, with GPDMA peripheral requests (separate PR).
- Timer-triggered conversion (`EXTSEL`/`EXTEN`), injected group, watchdogs, oversampling, the asynchronous kernel clock.
- Internal channels (VREFINT, temperature sensor, VBAT) with their calibration values.
- STM32U5/L5 ADC.
