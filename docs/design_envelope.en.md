# Design margins and finite assembly scenarios

[中文](design_envelope.md) · [English](design_envelope.en.md)

A nominal pass does not show which limit assembly or support changes will reach first. This workflow recalculates every explicitly listed scenario through the complete duty, system sharing and both meshes' LTCA. It reports signed margins and traceable governing states. It does not predict failure probability or bound a continuous tolerance domain.

```bash
python -m examples.design_envelope --output output/design_envelope
```

The [base input](../examples/dfki_pin_geometry.json) and [scenario specification](../examples/dfki_assembly_scenarios.json) are editable. `--input` and `--scenarios` replace them. The nominal design is always included. Outputs retain full adopted inputs, `study.json`, `margins.csv`, bilingual reports and SHA-256 hashes. Completed results replace the previous comparison only after all calculations succeed. See the [full 34-state result](../examples/results/design_envelope/report.en.md), using 64 phases and 26 strips.

```python
import json
from pathlib import Path
from robot_joint import analyze_joint_envelope, export_joint_envelope

config = json.loads(Path("examples/dfki_pin_geometry.json").read_text(encoding="utf-8"))
spec = json.loads(Path("examples/dfki_assembly_scenarios.json").read_text(encoding="utf-8"))
study = analyze_joint_envelope(config, spec["scenarios"], required_checks=spec["required_checks"])
export_joint_envelope(study, "output/design_envelope")
```

## Scenario contract

Each scenario has a unique ASCII `id`, optional `description` and nonempty `overrides`. `nominal` is reserved. Allowed changes are:

- `load_sharing`: positive/negative path gaps, sun radial stiffness, extra condensed compliance, and numeric pin stiffness in specified mode.
- `planet_support.pins`: diameter, span, loaded width, E, ν and shear coefficient. Shared or per-pin partial objects merge with the base parameters.
- `support.stiffness_matrix`, retaining the same reference frame.
- `mesh.sun_planet_misalignment_m/planet_ring_misalignment_m`.

Arrays are replaced as complete arrays. Pin geometry changes require explicit `pin_geometry`; geometry mode rejects overrides of unused numeric stiffness. Gear geometry, duty, allowables, rating factors, bearing capacities, discretization and acceptance limits cannot change within this study. Use complete candidates with `compare_joint_designs` for broader design comparisons.

Path gaps are combined tangential-equivalent path errors in m, not direct dimensional tolerances. Map real manufacturing errors through an assembly/structural model first. Condensed compliance must exclude already modeled meshes, pins and sun support.

## Margins and acceptance

`constraint_margins(result)` also works on a single existing result. Standard `export_joint` adds `constraint_margins.json` and relative margins to the report.

For upper limits, margin is `threshold−actual`; for lower limits, `actual−threshold`. Relative margin divides by `|threshold|`. Negative means violation. Absolute margin retains the criterion's unit. Unconfigured or unevaluated checks have no margin; zero limits have no percentage.

Temperature percentages use SI thresholds in K, not allowable temperature rise. Percentages from different criteria do not form a common risk score; interpret absolute margins and model scope separately.

`required_checks` must contain distinct existing criterion names. Missing required evaluations block scenario acceptance even when `configured_checks_pass=true`. Any failed configured criterion also blocks it. `all_scenarios_accepted` means only that every listed point meets these requirements.

`worst_checks` identifies each criterion's lowest signed margin and scenario. Unevaluated scenarios remain explicit. Traceable extrema include zero-based state, planet, mesh, role or stress-type locations; ties select the first. Duty RMS, thermal peaks, fatigue and L10 are not assigned a fabricated single LTCA state, and their location may be null.

## Engineering interpretation

The example assumes selected 1 μm path changes, 5.98/6.00/6.02 mm pin diameters, 50% sun radial stiffness and one combined point. These are sensitivity assumptions without manufacturing distributions or measurement calibration. Bearing capacities remain illustrative. Counts of passing scenarios are neither yield nor reliability, and the combined point is not a global worst case.

Use the governing criterion/state to revise assembly errors, support design or requirements, then recalculate the full duty. Manufacturing acceptance still needs dimensional chains, fits/preload, structural compliance, material/rating evidence and torsional, thermal and service tests. See [planet support](planet_support.en.md) for short-pin beam limitations.
