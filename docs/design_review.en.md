# Joint-module design review and improvements

[中文](design_review.md) · [English](design_review.en.md)

This review examines the parameterized kernel and public case through joint requirements, power/load transfer, stiffness and precision, failure modes and verification evidence. The findings support use for **quasi-static design calculation and candidate screening of a single-stage spur NGW joint**. Complete physical design, manufacturing acceptance and hardware performance still require the work listed below.

Version 0.3.0 advances the earlier recommendations with planet-pin/bearing and regenerative-bus screens and full tooth-passage contact patterns. Implemented calculations, input assumptions and hardware-validation states are recorded separately. Missing optional inputs do not produce a pass for the corresponding failure mode.

## Findings and changes

|Design issue|Consequence|Improvement|
|---|---|---|
|Supplied-hotspot damage did not affect design acceptance|A record explicitly producing `D>1` could still receive an overall screening pass|Added the `fatigue_damage_per_input_history` summary and limit; supplied records default to `D<=1`, with stricter targets configurable|
|Stiffness and sharing targets were missing|Candidate comparison had no direct constraint for excessive compliance or unequal loading|Added minimum output torsional stiffness and maximum planetary load-share factor; feasible Pareto eligibility requires configured constraints to pass|
|Boolean results hid acceptance evidence|Missing inputs or targets could be mistaken for completed checks|Added `check_details` with actual values, limits, comparison directions, scope and four states; unevaluated/unconfigured items are explicit|
|The stiffness reference point could not differ from the force application point|An overhung force could be evaluated without its contribution to structural tilt|Added `support.reference_offset_m` and work-conjugate wrench/displacement transformations; default zero preserves the port-condensed K interface|
|Engineering metadata conflated coordinate origins|Bearing reactions, structural motion and misalignment maps could use inconsistent moment arms|Separately identify the bearing midpoint, load port, stiffness reference and unspecified physical gear-plane axial position|
|Assembly checks gave no external-gear undercut warning|An assemblable small-tooth-count layout may not be manufacturable with the assumed cutter|Added a zero-shift standard full-depth rack undercut-risk screen; actual cutters, internal-gear interference and peaking remain unassessed|
|The scope of a configured-check pass was unclear|Equivalent stiffness, basic L10 and power balance could be misread as component strength, hardware life or inverter certification|Added `assessment_coverage` and uncovered failure modes to exported reports|
|Display discretization was coarse and evidence levels were scattered|A single plot could not establish discretization error or benchmark scope|The production case now uses 64 phases ×26 strips, with consolidated single-mesh software comparisons, published hardware-parameter checks and independent equation/discretization evidence in [validation](validation.en.md)|
|Contact displays contained only a few lines/nodes at the peak-pressure phase|The original plot could not show full-passage pressure or differences between flanks and individual planets|Added multi-phase finite Hertz bands, full-passage maximum pressure and three-planet/flank/low-load maps, retaining original line loads, half-widths and clipping/display differences; see [contact patterns](contact_patterns.en.md)|
|Equivalent pin stiffness had no component-strength or planet-bearing check|A load-sharing pass did not establish pin bending/shear/hole-pressure or bearing suitability|Added an explicitly supported solid-pin bending–shear/deflection/average-pressure model and basic bearing ratings using actual pin reactions and carrier-relative planet speed, without silently replacing system stiffness|
|Backdriving power had no bus-absorption requirement|Mechanical-power signs could be confused with actual electrical regeneration, omitting bus overvoltage|Added directional inverter losses, capacitor energy, threshold events and limited external absorption from motor electrical power including copper loss and rotor inertia; see [the bus model](dc_bus.en.md)|

The structural transformation uses `uport=B uref`, `Wref=Bᵀ Wport` and `Cport=B K^-1 Bᵀ`, retaining `r×F` before solving the reference-point response. `bearings.load_offset_m` is measured from the bearing midpoint; the structural offset is measured from K's reference point. They must not be automatically identified. Output loads use a design-demand convention; measured action/reaction wrenches require a consistent sign conversion. The [model](model.en.md) and [API](api.en.md) specify the contract.

The external-gear risk screen uses the standard full-depth rack relation `z sin²α>=2`, documented in the [QTC/KHK technical reference](https://www.qtcgears.com/tools/catalogs/PDF_Q420/Tech.pdf). It indicates theoretical risk under those assumptions and does not automatically reject a manufacturing design.

## Current case choices

DFKI's published masses, dimensions and original actuator parameters provide physical requirements; the tooth-count combination follows a public gearset. Motion, modifications, errors, stiffness, efficiency, thermal inputs, rating factors and S–N data added here remain design inputs. The supplier's 30 mm center-distance statement differs from the zero-shift relationship implied by its tooth counts and module. The model uses 15 mm and retains the discrepancy for confirmation; manufacturing requires physical dimensions or an approved drawing.

This revision explicitly selects `Kout>=20000 N·m/rad`, `Kγ<=2` and supplied-hotspot `D<=1`. These are **case assumptions, not DFKI hardware specifications**. K is condensed at the output load port, so `reference_offset_m=0`; the port's offset from the bearing midpoint is independently 0.012 m. Candidates are recalculated for the same duty, without presenting factors from an old geometry as automatic rating of a new geometry.

The 64×26 discretization is a case-specific choice. Agreement with a finer grid at one representative state does not establish convergence of all loads, edge contacts or tooth counts. New geometry, modifications, errors or materials require the corresponding discretization study.

## Design conclusions from the continued work

**Full contact patterns.** The original display correctly retained instantaneous loaded nodes, but a few lines could not represent the whole tooth passage. Added patterns use 512 phases ×41 strips at selected fixed local loads and map finite-width Hertz bands into material coordinates. The instantaneous comparison uses the same phase/tooth pair with both bands inside their valid flank domains, rather than the peak-pressure phase. Full-passage maps take the maximum experienced pressure under identical-tooth periodicity, not the maximum across all states of the 1.8 s duty. Flanks, all three planets and four member surfaces retain separate material coordinates; unloaded edges and unresolved domains keep their data states. This is not a nonlocal 2D or FE solve and does not smooth sparse lines into a whole patch. Original sampling, local-Hertz assumptions, source peaks, integration/edge differences and coverage remain explicit in [the method and raw data](contact_patterns.en.md).

**Pins and planet bearings.** Current loads give a maximum pin reaction of 118.889 N. With new 4/6 mm pin, 12 mm simply supported span and 5 mm loaded-width assumptions, both candidates pass the supplied static criteria. Their work-conjugate mean bending–shear stiffnesses are 71.225/297.707 MN/m, about 0.2374/0.9924 of the original 300 MN/m. Passing stress does not justify retaining that original stiffness for a 4 mm pin; explicitly feed an appropriate structural stiffness back into a new load-sharing solve. Numerical proximity for the 6 mm pin is not calibration either. Span/diameter ratios are only 3 and 2: transverse-shear flexibility is included, but actual 3D pin-hole/support behaviour still needs review. Basic L10 from assumed capacities is a formula screen, not oscillatory-life validation or catalog selection; see [planet support](planet_support.en.md).

**Regenerative bus.** With added assumptions of a 36 V source, 48 V limit and 96% directional inverter efficiency, the same motor history gives a free-energy extrapolated peak of 83.624 V for 470 μF without a sink, exceeding the limit. A 4700 μF capacitor gives 43.194 V, and 470 μF with a 15 W sink stays at 36 V. Real protection can disable a drive at the limit, so the beyond-limit curve is not a hardware-voltage prediction. DC regeneration peaks at 13.398 W and returns 2.678 J; the nominal-start no-sink minimum capacitance is about 2656 μF. This supplies an energy screen, without validating finite-source capability, the actual inverter voltage/current envelope, resistor thermal capacity or protection; see [the bus documentation](dc_bus.en.md).

## Verification of the changes

|Object|Checks|Reproduction entry|
|---|---|---|
|Design acceptance|A `D>1` counterexample, failed stiffness/sharing/damage targets, missing inputs and unconfigured states, actual/threshold consistency|`tests/test_analysis.py`|
|Reference-point transformation|Analytical moment/tilt/port translation from an overhung force, both offset signs, virtual work, reciprocal positive-definite compliance, invalid parameters|`tests/test_support.py`|
|Structural-to-mesh interface|A small actual LTCA solve verifies that offset-induced motion reaches each planet's misalignment map|`tests/test_analysis.py`|
|Engineering coordinates|Separate bearing-midpoint, port and stiffness-reference positions; unspecified dimensions remain unknown|`tests/test_engineering.py`|
|Geometry risk|Standard rack equation, ordinary/small tooth-count cases and explicit unassessed internal-gear checks|`tests/test_planetary.py`|
|Full-passage contact|Material coordinates, single-tooth/periodic folding, finite Hertz width, original pressure, unloaded/unresolved regions and display differences|[Contact-pattern reproduction and verification](contact_patterns.en.md)|
|Planet pins and bearings|Bending–shear energy, unit-load/scaling relations, reversed loads, relative speed, pin reactions, invalid inputs and source hashes|`tests/test_planet_support.py`|
|Bus energy|Analytical capacitance energy, within-segment thresholds, directional efficiency, limited absorption, mechanical backdrive with positive electrical power, conservation and subdivision invariance|`tests/test_dc_bus.py`|
|Existing physical chain|Signed equilibrium, zero-speed holding copper loss, rotor-inertia power, basic L10, rainflow, thermal energy and periodic closure|Corresponding `tests/` modules|

```bash
python -m unittest discover -s tests -v
python -m examples.build_showcase --output output/showcase --raw-output output/raw
python -m examples.run_benchmarks --output output/benchmarks
python -m examples.build_contact_patterns --phases 512 --strips 41
python -m examples.check_planet_support --results output/pendulum --output output/planet_support
python -m examples.check_dc_bus --results output/pendulum --output output/dc_bus
python -m examples.plot_design_screens
```

Ordinary interface integration tests use small actual solves to verify the contracts and physical relationships. The case workflow reproduces all 34 duty states at the production discretization. Repository CI records final test status. Single-mesh software agreement, published hardware-parameter checks, equation checks and physical testing remain separate evidence levels.

## Remaining priorities

|Priority|Work|Required inputs and acceptance evidence|
|---|---|---|
|High|Calibrate bidirectional stiffness, backlash and friction|Torque–angle loading/unloading records, mounting boundaries and encoder calibration; validate held-out loads rather than only fitting a curve|
|High|Identify loss and thermal parameters|Input/output power across speed/load, no-load losses and heating/cooling records; validate temperature on an independent duty|
|High|Apply pin/bearing screens to physical components|Confirm dimensions/material, 3D pin-hole contact and actual bearing selection; feed appropriate stiffness back into load sharing; basic ratings do not establish oscillatory life|
|High|Check manufacturing and physical structures|Actual profiles/cutters, tip thickness, internal interference, rims/housing/carrier, bolts and fits; approved drawings and corresponding structural models are required|
|Medium|Assess oscillating output bearings|Preload/clearance, swing/contact coverage, lubrication, fretting and false brinelling; very large basic L10 is not hardware service life|
|Medium|Validate the actual inverter and bus-absorption hardware|Extend the energy screen with finite-source capability, current/voltage envelopes, peak duration, ripple, protection and resistor thermal capacity using selected hardware and tests|
|Medium|Add synchronous dynamics and control|Phase-synchronous meshes, component inertia, modes, controller and measured frequency response; quasi-static stiffness is not closed-loop bandwidth|

The table lists physical/manufacturing work that remains open; implemented pin/bearing and bus screens retain their own inputs and states. Output-angle budgets are uncompensated deterministic bounds, and static two-flank figures do not model measured hysteresis. Supplied-hotspot damage and basic L10 do not certify whole-joint life. Full-passage pressure maps are not measured hardware contact imprints.
