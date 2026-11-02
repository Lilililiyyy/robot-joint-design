# Joint-module design review and improvements

[中文](design_review.md) · [English](design_review.en.md)

This review examines the parameterized kernel and public case through joint requirements, power/load transfer, stiffness and precision, failure modes and verification evidence. The findings support use for **quasi-static design calculation and candidate screening of a single-stage spur NGW joint**. Complete physical design, manufacturing acceptance and hardware performance still require the work listed below.

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

The structural transformation uses `uport=B uref`, `Wref=Bᵀ Wport` and `Cport=B K^-1 Bᵀ`, retaining `r×F` before solving the reference-point response. `bearings.load_offset_m` is measured from the bearing midpoint; the structural offset is measured from K's reference point. They must not be automatically identified. Output loads use a design-demand convention; measured action/reaction wrenches require a consistent sign conversion. The [model](model.en.md) and [API](api.en.md) specify the contract.

The external-gear risk screen uses the standard full-depth rack relation `z sin²α>=2`, documented in the [QTC/KHK technical reference](https://www.qtcgears.com/tools/catalogs/PDF_Q420/Tech.pdf). It indicates theoretical risk under those assumptions and does not automatically reject a manufacturing design.

## Current case choices

DFKI's published masses, dimensions and original actuator parameters provide physical requirements; the tooth-count combination follows a public gearset. Motion, modifications, errors, stiffness, efficiency, thermal inputs, rating factors and S–N data added here remain design inputs. The supplier's 30 mm center-distance statement differs from the zero-shift relationship implied by its tooth counts and module. The model uses 15 mm and retains the discrepancy for confirmation; manufacturing requires physical dimensions or an approved drawing.

This revision explicitly selects `Kout>=20000 N·m/rad`, `Kγ<=2` and supplied-hotspot `D<=1`. These are **case assumptions, not DFKI hardware specifications**. K is condensed at the output load port, so `reference_offset_m=0`; the port's offset from the bearing midpoint is independently 0.012 m. Candidates are recalculated for the same duty, without presenting factors from an old geometry as automatic rating of a new geometry.

The 64×26 discretization is a case-specific choice. Agreement with a finer grid at one representative state does not establish convergence of all loads, edge contacts or tooth counts. New geometry, modifications, errors or materials require the corresponding discretization study.

## Verification of the changes

|Object|Checks|Reproduction entry|
|---|---|---|
|Design acceptance|A `D>1` counterexample, failed stiffness/sharing/damage targets, missing inputs and unconfigured states, actual/threshold consistency|`tests/test_analysis.py`|
|Reference-point transformation|Analytical moment/tilt/port translation from an overhung force, both offset signs, virtual work, reciprocal positive-definite compliance, invalid parameters|`tests/test_support.py`|
|Structural-to-mesh interface|A small actual LTCA solve verifies that offset-induced motion reaches each planet's misalignment map|`tests/test_analysis.py`|
|Engineering coordinates|Separate bearing-midpoint, port and stiffness-reference positions; unspecified dimensions remain unknown|`tests/test_engineering.py`|
|Geometry risk|Standard rack equation, ordinary/small tooth-count cases and explicit unassessed internal-gear checks|`tests/test_planetary.py`|
|Existing physical chain|Signed equilibrium, zero-speed holding copper loss, rotor-inertia power, basic L10, rainflow, thermal energy and periodic closure|Corresponding `tests/` modules|

```bash
python -m unittest discover -s tests -v
python -m examples.build_showcase --output output/showcase --raw-output output/raw
python -m examples.run_benchmarks --output output/benchmarks
```

Ordinary interface integration tests use small actual solves to verify the contracts and physical relationships. The case workflow reproduces all 34 duty states at the production discretization. Repository CI records final test status. Single-mesh software agreement, published hardware-parameter checks, equation checks and physical testing remain separate evidence levels.

## Remaining priorities

|Priority|Work|Required inputs and acceptance evidence|
|---|---|---|
|High|Calibrate bidirectional stiffness, backlash and friction|Torque–angle loading/unloading records, mounting boundaries and encoder calibration; validate held-out loads rather than only fitting a curve|
|High|Identify loss and thermal parameters|Input/output power across speed/load, no-load losses and heating/cooling records; validate temperature on an independent duty|
|High|Rate planet pins and planet bearings|Pin diameter/span/material, pin-hole contact, bearing selection and relative speed/load; equivalent pin stiffness does not establish strength or life|
|High|Check manufacturing and physical structures|Actual profiles/cutters, tip thickness, internal interference, rims/housing/carrier, bolts and fits; approved drawings and corresponding structural models are required|
|Medium|Assess oscillating output bearings|Preload/clearance, swing/contact coverage, lubrication, fretting and false brinelling; very large basic L10 is not hardware service life|
|Medium|Assess inverter and regenerative bus|Current/voltage envelopes, peak duration, bus capacitance/absorption or braking resistor; motor power balance does not establish these capabilities|
|Medium|Add synchronous dynamics and control|Phase-synchronous meshes, component inertia, modes, controller and measured frequency response; quasi-static stiffness is not closed-loop bandwidth|

These tasks are not assumed complete in this revision. Output-angle budgets are uncompensated deterministic bounds, and static two-flank figures do not model measured hysteresis. Supplied-hotspot damage and basic output-bearing L10 do not certify whole-joint life.
