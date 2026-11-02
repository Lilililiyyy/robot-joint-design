# Robot Joint Design

[中文](README.md) · [English](README.en.md)

[![Python](https://img.shields.io/badge/Python-3.10%2B-3776ab?logo=python&logoColor=white)](https://www.python.org/) [![License](https://img.shields.io/badge/License-AGPL--3.0--only-blue.svg)](LICENSE) [![Version](https://img.shields.io/badge/version-0.3.3-blue)](https://github.com/Lilililiyyy/robot-joint-design/tree/v0.3.3)

An independently developed robot joint design project for single-stage NGW planetary actuators. It connects task loads, gear contact, structural support, motor sizing, precision and thermal performance through parameterized calculations. Complete Python source, bilingual worked examples and engineering data exports support standalone use and integration into a design workflow.

## Contents

- [Design capabilities](#design-capabilities)
- [Installation and use](#installation-and-use)
- [Design example](#design-example)
- [Result benchmarks](#result-benchmarks)
- [Engineering scope](#engineering-scope)
- [Version history](#version-history)
- [Roadmap](#roadmap)
- [Documentation and structure](#documentation-and-structure)

## Design capabilities

|Module|Capabilities|
|---|---|
|Requirements and loads|Motion profiles, load inertia, gravity and acceleration torque, segmented duty histories|
|Planetary transmission|Assembly conditions, ratio, clearances, individual planet loads, path errors and compliance coupling|
|Gear contact|Internal/external LTCA, transmission error, tangent stiffness, local Hertz pressure, loaded flanks and full tooth-passage pressure maps|
|Strength and support|Explicit-factor flank/root rating, six-degree-of-freedom support, bearing reactions, planet-pin bending/shear and basic planet-bearing rating|
|Motor and energy|Driving/backdriving efficiency, rotor inertia, torque–speed envelopes, RMS current, braking and regenerative-bus screening|
|Precision and thermal|Loaded angle, backlash/encoder budgets, segmented temperature response and periodic thermal equilibrium|
|Life and selection|Rainflow cycles, cumulative damage with supplied S-N curves, constraint screening and finite-candidate Pareto comparison|
|Engineering outputs|Reports, JSON/CSV/NPZ, parameter-level CAD dimensions and BOM, bilingual publication-style PNG/SVG/PDF figures|

Modules share SI units and callable interfaces. See the [model](docs/model.en.md) and [API](docs/api.en.md) for equations and input/output definitions.

## Installation and use

Python 3.10 or newer is required. From the repository root:

```bash
python -m pip install -e ".[plots]"
python -m robot_joint examples/dfki_pendulum_case.json --output output/pendulum
```

The source API can also be used directly:

```python
import json
from pathlib import Path
from robot_joint import analyze_joint, export_joint

config = json.loads(Path("examples/dfki_pendulum_case.json").read_text(encoding="utf-8"))
result = analyze_joint(config)
export_joint(result, "output/pendulum")
```

Length, angle, force, torque and pressure use m, rad, N, N·m and Pa. For calculation modules only, install with `python -m pip install -e .`.

## Design example

A 6:1 robot joint is designed around [published pendulum requirements](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/11a946d4e6db58d001fdeff89be269eb61461757/hardware/testbench_description.md), using tooth counts from a [published steel gearset](https://mjbots.com/products/m05-6x-planetary-gearset). Support, modifications, allowable stresses, thermal and electrical parameters are explicit project design inputs.

![Joint transmission layout](examples/results/dfki_pendulum/en/01_design.png)

|Parameter|Value|
|---|---:|
|Point mass / rod and attachments / length|0.546 kg / 0.130 kg / 0.5 m|
|Sun / planet / ring|20 / 40 / 100 teeth|
|Module / planets / effective face width|0.5 mm / 3 / 5 mm|
|Ratio / model centre distance|6 / 15 mm|
|Motion task|±60°, 0.5 s move and 0.4 s hold, 1.8 s period|

**Load sharing and gear contact**

Thirty-four representative duty states connect three planets, both gear meshes and system compliance. Tooth-passage maps show local peak pressure experienced at material positions. Instantaneous bands use separate local coordinates along the contact line in mm and normal to it in μm. The two displays represent different quantities.

![Full tooth-passage pressure](examples/results/contact_patterns/en/01_contact_overview.png)

![Local instantaneous contact](examples/results/contact_patterns/en/06_instantaneous_contact.png)

See [contact analysis](docs/contact_patterns.en.md) for positive/negative flanks, low-load states and individual member surfaces.

**Motor, thermal performance and design selection**

The same duty drives motor-envelope, current, braking and periodic-temperature calculations. Candidate comparisons show how path errors and pin compliance affect output stiffness, contact pressure and precision budgets.

![Motor and thermal performance](examples/results/dfki_pendulum/en/07_motor_thermal.png)

![Candidate design comparison](examples/results/dfki_pendulum/en/09_comparison.png)

An integrated candidate assumes a 6 mm planet pin, feeds its 297.707 MN/m bending/shear stiffness into the system, and includes planet bearings and a 4700 μF bus. Maximum pin load is about 118.889 N, minimum planet-bearing static safety about 4.206, and maximum bus voltage about 43.194 V. These results depend on the declared geometry, capacities, limits and electrical conditions.

The [worked example](docs/examples.en.md) explains module inputs, results and design tradeoffs. [Design criteria](docs/design_review.en.md) summarize requirements and engineering boundaries.

## Result benchmarks

An 18/77 helical mesh is compared with saved MASTA results for geometry, explicit-factor strength, transmission error, stiffness and contact pressure.

|Metric|MASTA|This project|
|---|---:|---:|
|Mean normal TE / μm|1.106704585|1.106704539|
|Mean normal tangent stiffness / N/μm|220.932698401|220.932698268|
|Peak endpoint pressure / MPa|186.674351800|186.674381661|
|Tooth-passage display peak / MPa|186.522329|186.524300|

![MASTA result benchmark](benchmarks/results/en/01_masta_comparison.png)

On the same 30×30 display grid, pressure MAE is 0.040641 MPa, RMSE is 0.140090 MPa, and maximum absolute difference is 2.994763 MPa. The benchmark covers the stated single-mesh condition and does not establish measured joint-assembly performance. See [result benchmarks](docs/validation.en.md) for parameters and pressure maps.

## Engineering scope

The system models a fixed-ring, sun-input, carrier-output NGW stage with zero-shift spur gears, small deformation, linear elasticity and quasi-static cycle-averaged response. Additional stiffness, rating factors, allowable stresses, motor and thermal parameters are supplied inputs.

Outputs support design calculation and candidate selection. Manufacturing requires detailed housing/rim/carrier structures, fits and preload, tolerances, lubrication and oscillating-bearing design, followed by torsional, thermal and endurance validation. CAD/BOM outputs are currently parameter-level.

## Version history

|Version|Main updates|
|---|---|
|**0.3.3**|Unified contact-pressure colour scale and figure style; version badge, bilingual history and roadmap|
|0.3.2|Refined standalone project documentation and focused public presentation on design capabilities and results|
|0.3.1|Completed tooth-passage and instantaneous contact maps and added a single-mesh pressure benchmark|
|0.3.0|Added planet-pin, planet-bearing and regenerative-bus checks to the integrated design workflow|
|0.2.0|Added design constraints, assessment coverage, strength benchmarks and bilingual publication-style figures|
|0.1.2|Added the complete bilingual design example driven by published pendulum requirements|
|0.1.0|Established NGW transmission, support, motor, precision, fatigue, thermal and parameter-level engineering outputs|

## Roadmap

The following are planned directions rather than fully implemented capabilities.

|Direction|Planned work|
|---|---|
|Contact and modification|Parameterized modification search and multi-objective selection using pressure, TE and stiffness|
|Structure and assembly|Condensed rim, carrier and housing compliance; fits, preload and tolerance-chain analysis|
|Engineering calibration|Measured bidirectional stiffness, backlash, friction, efficiency and thermal-parameter inputs|
|Drive and service|Finite-source, absorption and protection boundaries; actual oscillating-bearing duty assessment|
|Model extensions|Helical gears, multi-stage transmissions and broader system coupling after appropriate verification|

## Documentation and structure

- [Model and equations](docs/model.en.md) · [API and units](docs/api.en.md) · [Worked example](docs/examples.en.md)
- [Contact analysis](docs/contact_patterns.en.md) · [Planet support](docs/planet_support.en.md) · [Regenerative bus](docs/dc_bus.en.md)
- [Design criteria](docs/design_review.en.md) · [Result benchmarks](docs/validation.en.md) · [Figures](docs/figures.en.md)
- `robot_joint/`: joint system and performance modules; `gear_contact/`: gear geometry and contact modules.
- `examples/`: parameterized cases and design results; `benchmarks/`: benchmark inputs and results.

Licensed under [AGPL-3.0-only](LICENSE). See [license and dependencies](THIRD_PARTY_NOTICES.md).
