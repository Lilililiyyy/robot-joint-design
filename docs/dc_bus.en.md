# DC-bus regenerative energy and overvoltage screening

[中文](dc_bus.md) · [English](dc_bus.en.md)

This module advances the regenerative-bus item in the design review. It converts the computed motor electrical-power history into DC-side energy, capacitor voltage and buffering requirements for specified limits. It is a new, independently callable energy model without commercial-drive dependencies. The [maxon regeneration/overvoltage explanation](https://support.maxongroup.com/hc/en-us/articles/360006243333-Measures-in-case-of-energy-recovery-and-overvoltage-error-states) and [Kollmorgen DC-link capacitor note](https://www.kollmorgen.com/en-us/developer-network/akd2g-low-voltage-dc-operation), accessed 2026-11-02, explain why returned energy can raise bus voltage and why buffering/absorption capacity needs explicit design. This implementation is not calibrated to a particular drive.

## Input power and energy conventions

Each segment supplies `duration_s` in seconds and `electrical_power_w` in watts. Use the signed motor-side electrical power from `motor_duty`:

`Pe = Tmotor·ωmotor + Pcu + Pother`.

`Tmotor` already includes rotor inertia torque. The input therefore includes rotor inertia power, copper loss and other motor losses. **Negative output mechanical power does not necessarily imply DC regeneration.** Do not construct negative electrical power merely from `Tout·ωout<0`.

With explicit inverter efficiencies, `Pdc=Pe/ηm` during consumption and `Pdc=Pe·ηr` during regeneration. Positive `Pdc` draws energy from the bus; negative `Pdc` returns energy. Inverter loss is `Pdc−Pe≥0`. Existing copper/gear losses and rotor energy are not counted a second time.

## Exact piecewise model

For source voltage `Vs` and capacitance `C`, define usable capacitor energy above the source voltage:

`U = C(V²−Vs²)/2`, `V = sqrt(Vs²+2U/C)`.

The ideal source has unlimited delivery power and maintains a voltage floor of `Vs`. Its ability to absorb returned power is separately specified as `absorption_power_limit_w=A`. During motoring, stored `U` is used first; the source supplies the remaining demand once `U=0`. During regeneration, the external sink absorbs up to `A` while the capacitor is above source voltage, giving `Udot=−Pdc−A`. At `U=0`, absorption reduces to the returned power so that the bus cannot be pulled below the source. The external sink is enabled only in regenerative segments, where `Pdc<0`.

Each segment advances `U` with a constant slope, clipped at zero, retaining exact within-segment upper-limit crossings and source-floor events. The full-history energy residual is:

`Uend−Uinitial + ∑Pdc·dt + Eabsorbed−Esource`.

This is not a brake-chopper, resistor, battery-charging or finite-bandwidth supply-control model. Without a resistor branch, an overvoltage result is not artificially clamped. The trajectory after crossing the upper limit is a **free energy-balance extrapolation assuming the prescribed power continues**. Real protection may disable the drive earlier, so the computed peak is not a predicted hardware voltage.

## API and reproduction

```python
from robot_joint.dc_bus import dc_bus_screen

bus = dc_bus_screen(
    result.motor["segments"],
    capacitance_f=470e-6,
    source_voltage_v=36,
    minimum_voltage_v=30,
    maximum_voltage_v=48,
    inverter_motoring_efficiency=0.96,
    inverter_regenerating_efficiency=0.96,
    absorption_power_limit_w=0,
)
```

`initial_voltage_v` defaults to the source voltage. Inputs must satisfy `0<Vmin≤Vs≤Vinitial≤Vmax`, with `Vmax>Vs`, `C>0`, `A≥0`, and efficiencies in `(0,1]`. Invalid states and missing power fields raise explicit errors. Input objects are not modified.

```bash
# Recompute only the motor history from the public case, without rerunning LTCA
python -m examples.check_dc_bus --output output/dc_bus
# Or read a motor history already exported by export_joint
python -m examples.check_dc_bus --results output/pendulum --output output/dc_bus
```

Outputs are `study.json`, `comparison.csv`, `segments.csv` and `voltage.csv`, using SI units. Selected public outputs are in the [result directory](../examples/results/dc_bus/). `study.source` records the input-file hash without publishing private absolute paths.

## New case assumptions and results

The 36V source, 30/48V voltage limits, 96% directional inverter efficiencies, 470/4700μF capacitances and 0/15W absorption capacities below are **new design assumptions**, not DFKI or original-drive specifications. All candidates use the same 34 representative motor states.

|Candidate|Capacitance / μF|Absorption limit / W|Computed peak / V|48V constraint|
|---|---:|---:|---:|---|
|Small capacitor, no sink|470|0|83.624|Not met|
|Larger capacitor, no sink|4700|0|43.194|Met within this model|
|Small capacitor, external sink|470|15|36.000|Met within this model|

There are 10 motor-side electrical regeneration segments. After the assumed inverter efficiency, DC regeneration peaks at 13.3983W and returns 2.67755J over the complete input history. With no external sink and a 36V start, at least 2656.30μF is needed to remain below 48V in this model; this excludes capacitance tolerance, ageing, temperature and component voltage margin. The 470μF candidate has a peak additional buffering deficit of 1.10190J. This is a capacity deficit at the observed peak, not full-cycle brake-resistor dissipation. The 15W sink candidate actually absorbs 1.48753W on average and 13.3983W at its peak.

## Interpreting outputs and limits

- `maximum_voltage_v` and `events` retain analytical boundary/threshold results. `time_s/voltage_v` contains segment boundaries and events only. To draw exact within-segment curves, use `segments.net_capacitor_power_before_floor_w` and initial `U` in the square-root relation; straight connections are not a physical high-frequency voltage waveform.
- `minimum_capacitance_f_for_nominal_start` is the energy capacitance required for one history starting at source voltage under the same power/sink model. It does not cover arbitrary initial voltage, control faults or unknown repeated-duty initial states.
- `within_minimum_voltage_by_ideal_source_assumption` follows from the ideal voltage floor. It **does not verify finite supply capability, wiring drop or hardware undervoltage performance**.
- `within_maximum_voltage` and `physical_validation` separately report model constraints and hardware validation. The standalone function returns results; configuring top-level `dc_bus` in `analyze_joint` adds maximum bus voltage to `summary.configured_checks_pass`. Minimum voltage guaranteed by the ideal source remains `not_evaluated`, without claiming hardware undervoltage validation. See the [complete API](api.en.md).

PWM, phase-inductance energy, wiring ESR/ESL, voltage ripple, drive current/voltage limits and peak duration, protective shutdown and resistor thermal capacity are outside scope. No Ke is inferred from Kt across current conventions. The static mechanical brake and regenerative bus are separate functions. Nine independent tests cover capacitor energy, directional efficiency, regeneration versus mechanical-power signs, floor/upper-limit events, limited absorption, subdivision invariance, conservation and invalid inputs.
