# Robot Joint Design

[中文](README.md) · [English](README.en.md)

[![Python](https://img.shields.io/badge/Python-3.10%2B-3776ab?logo=python&logoColor=white)](https://www.python.org/) [![License: AGPL-3.0-only](https://img.shields.io/badge/License-AGPL--3.0--only-blue.svg)](LICENSE)

An independent Python kernel for parameterized design and quasi-static assessment of a single-stage NGW robot joint. It connects duty loads, planetary load sharing, per-planet LTCA, rating stresses, output support, motor matching, precision budgets and periodic thermal response through callable functions and reproducible raw results. No commercial software or vendor DLL is required at runtime.

## Contents

- [Installation and use](#installation-and-use)
- [Calculation capabilities](#calculation-capabilities)
- [Design review and improvements](#design-review-and-improvements)
- [Design case and candidate comparison](#design-case-and-candidate-comparison)
- [Professional-software and public-reference comparisons](#professional-software-and-public-reference-comparisons)
- [Verification and reproduction](#verification-and-reproduction)
- [Scope](#scope)
- [Release history and next steps](#release-history-and-next-steps)
- [Documentation and repository layout](#documentation-and-repository-layout)

## Installation and use

Requires Python 3.10 or later. Run from the project root:

```bash
python -m pip install -e ".[plots]"
python -m robot_joint examples/dfki_pendulum_case.json --output output/pendulum
```

For the calculation kernel alone, use `python -m pip install -e .`. The command entry point is `robot-joint`; functions can also be called directly:

```python
import json
from pathlib import Path
from robot_joint import analyze_joint, export_joint

config = json.loads(Path("examples/dfki_pendulum_case.json").read_text(encoding="utf-8"))
result = analyze_joint(config)
export_joint(result, "output/pendulum")
```

Interfaces use SI units: m, rad, N, N·m, Pa, s and K. Each state explicitly provides a six-component output wrench; axial moment `Mz` and required output torque share the same value and sign. See the [API contract](docs/api.en.md) for fields, coordinates and array layouts.

## Calculation capabilities

|Module|Implemented functions|Required inputs or boundary|
|---|---|---|
|NGW geometry and kinematics|Tooth assembly, adjacent-planet clearance, ratio, speeds and mesh-force relations|Fixed ring, sun input, carrier output; zero-shift spur gears|
|Planetary load sharing|Flank-specific path errors, pin compliance, shared sun translation and nonnegative path forces with unloading/re-entry|Supplied support stiffness and additional condensed compliance|
|Per-planet LTCA|Two-mesh cyclic TE, tangent stiffness, contact-node forces and local Hertz pressure; displacement-compatible iteration|Quasi-static, cycle-mean coupling; raw contact arrays retained|
|Rating stresses|ISO 6336:2006 explicit-factor contact/bending stresses and safety factors|External rating factors and material limits supplied by the user|
|Structure and bearings|Six-DOF condensed stiffness, two-support reactions, revolution-weighted basic L10 and static safety|No actual bearing contact, preload, clearance or small-oscillation damage|
|Motor and brake|Motoring/backdriving efficiency, rotor inertia, torque–speed envelopes, RMS current and static brake screening|Supplied motor envelope, efficiencies, current convention and brake safety factor|
|Precision and thermal response|Loaded angle, backlash, encoder and assembly budget; two-node transient/periodic temperature|Deterministic angular bound; thermal capacity/conductance and losses require calibration|
|Fatigue and candidate screening|Rainflow, supplied S–N, Goodman/Miner; explicit design limits and finite-candidate Pareto comparison|Applies to a supplied scalar hotspot history; does not infer whole-joint life|
|Engineering exports|JSON/CSV/NPZ, calculation report, CAD dimensions, parameter-level BOM and reference layout|No manufacturing solids, procurement part numbers or tolerance chain|

Rating stresses, local contact pressure, output angle budgets and controlled positioning accuracy have separate definitions. See the [model documentation](docs/model.en.md) for equations, coupling and omitted physics.

## Design review and improvements

This revision adds minimum output-stiffness, maximum load-sharing and supplied-hotspot damage limits. Results report actual values, thresholds and coverage through four states: `passed`, `failed`, `not_configured` and `not_evaluated`. The structural stiffness reference and load port can be defined separately, with offset loads handled by work-conjugate wrench/displacement transformations. External gears also receive an undercut-risk screen under the standard full-depth rack assumption.

The current case assumes `Kout≥20000 N·m/rad`, `Kγ≤2` and supplied-history `D≤1` for same-duty candidate screening. These are not DFKI hardware specifications. See the [design review](docs/design_review.en.md) for findings, fixes and remaining pin/bearing strength, manufacturing and hardware-validation needs.

## Design case and candidate comparison

A 6:1 redesign uses [published DFKI physical-pendulum requirements](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/11a946d4e6db58d001fdeff89be269eb61461757/hardware/testbench_description.md), with geometry referenced to a [mjbots steel gearset](https://mjbots.com/products/m05-6x-planetary-gearset). It demonstrates the parameterized workflow; added support, modifications, thermal parameters and rating limits are recorded separately as assumptions.

![Requirements and NGW layout](examples/results/dfki_pendulum/en/01_design.png)

|Parameter|Value|
|---|---:|
|Point mass / rod and mounts / length|0.546 kg / 0.130 kg / 0.5 m|
|Sun / planet / ring|20 / 40 / 100 teeth|
|Module / planet count / effective face width|0.5 mm / 3 / 5 mm|
|Ratio / pressure angle / shift|6 / 20° / zero|
|New motion duty|±60°; 0.5 s move, 0.4 s hold; 1.8 s period|

The supplier states a 30 mm center distance, while the listed zero-shift tooth counts and module give 15 mm. The model uses 15 mm and retains that source conflict for clarification. This is a source-grounded new design, without reconstruction of AK80-6 internals or measured-performance reproduction.

**Load sharing and tooth contact**

Thirty-four representative states connect three planets, both meshes and system compliance. The current case uses 64 phases ×26 strips. Supplied errors produce unequal sharing. The figure shows cyclic normal TE and raw loaded nodes at the peak-pressure phase.

![Per-planet LTCA and raw contact nodes](examples/results/dfki_pendulum/en/04_contact.png)

**Motor matching and periodic temperature**

The same duty checks motoring/backdriving, motor envelopes and static brake requirements, then feeds segment losses into the two-node thermal model. Temperatures are conditional on the assumed thermal inputs.

![Motor matching and periodic thermal response](examples/results/dfki_pendulum/en/07_motor_thermal.png)

**Candidate comparison under the same duty**

With geometry, materials and motion fixed, compare the baseline, increased path errors and flexible pins to expose trade-offs among pressure, angle budget and output stiffness.

![Three-candidate comparison](examples/results/dfki_pendulum/en/09_comparison.png)

The [complete case guide](docs/examples.en.md) contains all nine stages, input provenance and key numbers. Shared chart data are in [metrics.json](examples/results/dfki_pendulum/metrics.json) and [duty.csv](examples/results/dfki_pendulum/duty.csv); editable SVG/PDF files accompany the images. See the [figure guide](docs/figures.en.md) for the blue/red styling, source data, dimensions and English-only reproduction. CAD/BOM files provide parameter interfaces.

## Professional-software and public-reference comparisons

Evidence is reported at its actual calculation level:

|Evidence|Comparison target|Supported conclusion|
|---|---|---|
|Retained MASTA single-mesh reference|Cylindrical geometry, rating stresses and LTCA with matching input, flank, load and discretization|Numerical agreement of the reused contact kernel for the listed states|
|Public software references and independent equations|Declared algorithm tables, equilibrium relations and numerical solutions|Correctness and regression coverage of the corresponding module/equation|
|DFKI hardware and mjbots geometry documentation|Published masses, length, ratio, tooth counts and related parameters|Traceability of case inputs and geometric consistency|

![MASTA single-mesh numerical comparison](benchmarks/results/en/01_masta_comparison.png)

The 18/77 helical pair is recalculated with matching inputs. All 12 geometry and 14 rating-stress common fields pass their declared tolerances. Cycle summaries are below; ppm expresses the relative difference in parts per million:

|Cycle metric|Saved MASTA reference|Current kernel|Relative difference / ppm|
|---|---:|---:|---:|
|Mean normal TE / μm|1.106704585|1.106704539|0.0414|
|TE peak-to-peak / μm|0.034133914|0.034133913|0.0400|
|Maximum endpoint contact pressure / MPa|186.674352|186.674382|0.160|
|Mean normal stiffness / N/μm|220.932698401|220.932698268|0.000603|

Endpoint pressure follows the reference endpoint definition and is saved separately from strip-centre peak pressure. In addition, all 48 scalar fields from two published MathWorks rainflow tables match; reproduction does not execute MATLAB.

See the [validation record](docs/validation.en.md) for reference inputs, retained results, absolute/relative differences and reproduction commands. Single-mesh comparisons do not cover the NGW assembly; hardware-parameter checks do not validate actual joint temperature, stiffness, precision or life. No assembly-level KISSsoft, JYCD or ETAGear comparison is currently available.

## Verification and reproduction

```bash
python -m examples.build_showcase --output output/showcase --raw-output output/raw
python -m examples.compare_designs --output output/comparison.json
python -m examples.run_benchmarks --output output/benchmarks
python -m examples.check_joint_resolution --output output/benchmarks/resolution
python -m examples.plot_validation --source output/benchmarks --output output/benchmarks
python -m unittest discover -s tests -v
```

Tests cover planetary torque/path compatibility, reverse loading and unloading, six-DOF support, electromechanical power, rainflow, analytical thermal response and interface constraints. Results retain input, units and diagnostics. Convergence, vendor-component numerical comparisons and physical experiments are separate verification states. Passing configured design limits is a screening result for the chosen model and thresholds.

The resolution study compares seven actual solver grids at one peak-pressure state. It does not establish asymptotic convergence across all states or parameters. Figure exports include PNG and editable SVG/PDF. Add `--language en` to `build_showcase` and `plot_validation` for English-only figures; see the [figure guide](docs/figures.en.md).

## Scope

The system layer currently covers one fixed-ring, sun-input, carrier-output, zero-shift spur NGW stage with small deformation, linear elasticity and quasi-static cycle-mean response. The independent contact kernel supports additional single-mesh configurations, but the system interface has not been extended to helical, multistage, RV or harmonic drives.

Full housing/shaft FE, synchronized transient mesh dynamics, friction-history hysteresis, planet needle bearings and small-angle oscillatory wear, lubrication, electromagnetics, control precision and manufacturing tolerance chains are not included. Light-load basic L10, assumed-hotspot damage and deterministic angle budgets are interpreted separately, without whole-joint life or positioning certification.

## Release history and next steps

- **0.1.0**: NGW system, support, fatigue, thermal, electromechanical, precision and parameter-level engineering exports.
- **0.1.2**: DFKI physical-requirement reference, complete bilingual case and figures, and a cross-platform floating-point tolerance fix for saved duty comparison.
- **0.2.0**: Project-level bilingual documentation, blue/red publication-style figures, expert design review, explicit assessment coverage and reproducible comparison evidence; a single default branch, `main`.
- **Near-term priorities**: Extend the resolution study to all critical duty states, add measured torsional stiffness/backlash and friction calibration, identify thermal networks and efficiency maps, and assess bearing suitability under actual oscillatory duty.
- **Later extensions**: Multistage/helical systems, physical gear-body/support compliance, manufacturing tolerance statistics and prototype comparisons after separate validation. These remain planned work.

## Documentation and repository layout

- [Model and equations](docs/model.en.md) · [API and units](docs/api.en.md) · [Complete case](docs/examples.en.md) · [Comparison evidence](docs/validation.en.md)
- [Design review and further validation](docs/design_review.en.md) · [Blue/red figures and source data](docs/figures.en.md)
- `robot_joint/`: new NGW system/performance modules; `gear_contact/`: reused cylindrical-contact kernel.
- `examples/`: inputs, runnable workflows and selected results; `benchmarks/`: offline references and comparison outputs.
- `tests/` and `.github/`: automated verification; `docs/`: equations, integration and engineering guidance.

The project uses [AGPL-3.0-only](LICENSE). See the [source and license notices](THIRD_PARTY_NOTICES.md) for reused code, source commits and dependencies.
