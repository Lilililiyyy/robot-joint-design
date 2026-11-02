# API and data contract

[中文](api.md) · [English](api.en.md)

## Optional pin, planet-bearing and bus checks

`planet_support={pins,bearing_ratings}` runs only when explicitly configured. See [pins and planet bearings](planet_support.en.md) for parameters and formulas. Pin utilization is the maximum `actual stress×required safety factor/allowable`; `planet_pin_static` requires a value ≤1 and enters overall screening. Calculated mean bending/shear pin stiffness only diagnoses consistency with system input: static strength does not establish stiffness agreement or update load sharing. Configured planet-bearing static safety defaults to ≥1. Basic L10 enters acceptance only with an explicit life requirement.

`dc_bus` is passed to `dc_bus_screen` using `motor.segments[*].electrical_power_w`, retaining motor losses and rotor inertia rather than treating negative mechanical power as regenerated electrical power. `dc_bus_maximum_voltage` compares the calculated peak with the configured upper limit and enters screening. `dc_bus_minimum_voltage` stays `not_evaluated` because the ideal unlimited supply enforces the floor; it is not a real undervoltage check. See the [bus model](dc_bus.en.md).

```python
import json
from pathlib import Path
from examples.check_planet_support import default_assumptions
from robot_joint import analyze_joint

config = json.loads(Path("examples/dfki_pendulum_case.json").read_text(encoding="utf-8"))
candidate = default_assumptions()["candidates"][1]
config["planet_support"] = {key: candidate[key] for key in ("pins", "bearing_ratings")}
config["dc_bus"] = {
    "capacitance_f": 0.01, "source_voltage_v": 48,
    "minimum_voltage_v": 42, "maximum_voltage_v": 60,
    "inverter_motoring_efficiency": 0.95,
    "inverter_regenerating_efficiency": 0.95,
    "absorption_power_limit_w": 10,
}
config.setdefault("design_limits", {})["maximum_pin_stiffness_relative_difference"] = 0.1
result = analyze_joint(config)
```

These pin and bus inputs are new design assumptions, not physical-product specifications. `JointResult` adds `planet_support=None` and `dc_bus=None` after its original nine positional fields, preserving old construction. Configured options return complete result objects. Export writes `planet_support.json` and `dc_bus.json`; missing options produce JSON null, preventing stale optional results. Summary adds pin static utilization, relative stiffness difference, planet-bearing basic static safety/L10 hours and peak bus voltage. Missing inputs yield null metrics and `not_evaluated` check details.

Additional `design_limits` are `maximum_pin_stiffness_relative_difference` (upper bound, diagnostic unless explicitly limited), `minimum_planet_bearing_static_safety_factor` (lower bound, configured bearings default to 1), and `minimum_planet_bearing_basic_l10_hours` (explicit lower bound only). None establishes oscillatory bearing service life or physical stiffness calibration.

## Entry points

`analyze_joint(config) -> JointResult` evaluates one complete design and its duty states without modifying the input. `export_joint(result, output_dir, plots=False)` exports raw arrays, diagnostics, engineering parameters and a Chinese report. `compare_joint_designs(configs)` recalculates complete candidates; `results` retains calculation objects and `candidates` provides a serializable comparison. Pareto eligibility requires the configured checks to pass.

The complete current example is [dfki_pendulum_case.json](../examples/dfki_pendulum_case.json); [requirements](../examples/dfki_pendulum_requirements.json) identify published values, derived quantities, design assumptions and the unresolved supplier dimension.

## Main inputs

|Field|Meaning|
|---|---|
|`schema_version`, `case_id`|Schema 1 and a nonempty design identifier|
|`stage`|`sun_teeth, planet_teeth, ring_teeth, planet_count, module_m, face_width_m, pressure_angle_rad`|
|`duty_segments`|Duration, signed output torque/speed, supplied acceleration and a six-component output wrench; optional label|
|`load_sharing`|Initial mesh stiffness, pin stiffness, sun radial stiffness, positive/negative-flank path gaps and optional extra compliance|
|`mesh`|Materials, phases/strips, elastic mode, both flanks' modifications, misalignment and optional structural transfer maps|
|`support.stiffness_matrix`|Reciprocal positive-definite 6×6 structural stiffness, excluding already modeled planetary-path compliance|
|`support.reference_offset_m`|Optional, default 0; signed z distance in m from the stiffness reference point to the output load port|
|`bearings`|Two-support span, load offset, axial split and two rating dictionaries|
|`motor`, `drive`|Consistently defined Kt/resistance, continuous/peak torque, speeds/inertia, efficiencies and static brake inputs|
|`thermal`, `thermal_mode`|Two capacities, ambient/coupling conductances, ambient temperature; `single_duty` or `periodic_steady`|
|`precision`|Total free backlash, encoder count and location, assembly-error bound|
|`rating`|Optional explicit factors and limits for `sun_planet` and `planet_ring`|
|`fatigue`|Optional scalar stress history in Pa and explicit S–N/mean-stress parameters|
|`design_limits`|Optional upper limits for pressure/angle/temperature/sharing/hotspot damage and lower limits for stiffness, rating/bearing safety or life|

SI units are m, rad, N, N·m, Pa, s and K. Unknown fields and unsupported branches raise errors.

Saved duty values are checked against the generated finite SI values with rtol=2e-12 and atol=2e-12, allowing last-bit differences between system math libraries. Fields, labels, state order and six-component layouts must match exactly. Actual input changes beyond these tolerances raise errors.

## Coordinates, signs and units

The wrench order is `[Fx,Fy,Fz,Mx,My,Mz]`; displacement is `[x,y,z,θx,θy,θz]`. Mixed stiffness blocks carry their corresponding force/displacement units and must satisfy energy reciprocity. `output_wrench` is specified at the output load port in the design-demand convention; its Mz equals the signed `output_torque_n_m`. Torque selects the tooth flank; `T·ω` selects motoring or backdriving. Before importing load-cell data, distinguish the wrench applied by the joint to the load from its action/reaction counterpart, and convert it to the stated demand convention.

The ring is fixed and the carrier is the output, with `ratio=1+zr/zs`. In `sun_planet`, the sun is `pinion` and planet is `wheel`; in `planet_ring`, the planet is `pinion` and ring is `wheel`. All tooth counts remain positive; the wheel role carries the internal-gear sign.

`positive_path_gaps_m` and `negative_path_gaps_m` are combined tangential-equivalent path errors, one per planet. Positive values delay engagement. They are not vendor matrix fields. Global output backlash is a separate input; do not count the same clearance twice.

Mesh stiffness is normal N/m; pin stiffness is tangential N/m; output torsional stiffness is N·m/rad. Initial stiffness is an iteration seed, not a calibrated result. Additional path compliance must not repeat mesh, pin or sun-support compliance.

Each `alignment_maps` matrix has shape `planet_count×6`. It maps output-load-port structural displacement to a normal gap difference across the face width. Translation columns have units m/m and rotation columns m/rad. The map must use the same port coordinates and signs; the case uses an assumed carrier-tilt mapping.

Positive tooth modification means material removal. Parameter, measured-curve, measured-surface and composite inputs are supported by the reused gear module. Positive/negative flanks are configured separately; there is no implicit mirroring.

## Structural reference point and return fields

`support_response(wrench, stiffness_matrix, *, reference_offset_m=0.0)` transfers the port wrench to the stiffness reference point. With `h=reference_offset_m` and `r=[0,0,h]`, `Mref=Mport+r×F`; port translation is `uref+θref×r` and rotation is unchanged. In matrix form, `uport=B uref` and `Wref=Bᵀ Wport`, where `B[0,4]=h`, `B[1,3]=-h` and the remaining entries form the identity. Port compliance is `Cport=B K^-1 Bᵀ`, preserving virtual work.

|Return field|Reference and meaning|
|---|---|
|`generalized_displacement`|Six-component displacement at the output load port, m/rad|
|`displacement_m`, `rotation_rad`|Translation and rotation at that same port|
|`reference_offset_m`|Signed z offset used, m|
|`reference_wrench`|Six-component wrench at the stiffness reference point, N/N·m|
|`reference_generalized_displacement`|Six-component displacement at the stiffness reference point, m/rad|
|`load_point_compliance`|6×6 port compliance, with the corresponding displacement/load units in each mixed block|
|`diagnostics.virtual_work_residual_j`|Difference between the two reference-point virtual works, J|

`bearings.load_offset_m` uses the midpoint of the two output bearings as its origin and is independent of the structural offset. If K is already condensed at the load port, the structural offset is zero even when the port overhangs the bearing midpoint. The current case explicitly uses `support.reference_offset_m=0` and `bearings.load_offset_m=0.012`. Exported `coordinate_references` distinguish the bearing midpoint, load port and stiffness reference point. No physical gear-plane axial position is invented when absent from the inputs.

## Design limits

`design_limits` retains the existing pressure/angle/temperature upper bounds and rating/bearing safety or bearing-life lower bounds, and adds:

|Field|Comparison|Scope|
|---|---|---|
|`minimum_output_torsional_stiffness_n_m_per_rad`|Actual ≥ limit|Minimum local output torsional stiffness across duty states, N·m/rad|
|`maximum_load_share_factor`|Actual ≤ limit|Largest planetary load-share factor among loaded states; undefined for an entirely unloaded duty|
|`fatigue_damage_per_input_history`|Actual ≤ limit|Miner damage for the supplied scalar hotspot record, without a mapping to target service hours|

The case selects stiffness ≥20000 N·m/rad, load-share factor ≤2 and supplied-record damage ≤1. These are **new case-specific design assumptions, not DFKI hardware specifications**. The example system uses 64 phases ×26 strips; local contact calculations may be refined separately.

## Results and checks

`result.cases[j]` contains `sharing, contacts, ratings, support, bearing_reactions, precision, speeds, kinematic_exposure`. `contacts` has two groups of N `gear_contact.AnalysisResult` objects, ordered by planet position. Each phase retains contact-line/strip geometry, gaps, forces, active masks, pressures and stiffness.

`sharing.coupling` records iteration history and physical path residuals. Failure raises `CouplingConvergenceError`. Cycle-mean displacement uses mean compliance, hence harmonic-mean phase stiffness. Local tangent stiffness freezes final nominal rating inputs; it is not a finite difference between rerated operating points. Zero-load angular lag is defined as zero, while first-contact position and the right-hand stiffness are retained separately.

`summary.configured_checks_pass` aggregates evaluated constraints only. When rating is configured, default minimum SH and SF are 1; default bearing static safety is 1. A supplied hotspot record defaults to damage ≤1, with an explicit stricter bound available. Bearing hours, minimum stiffness and maximum load-share factor enter acceptance only when their limits are specified. Fatigue evaluates only the supplied record and does not automatically map it to target service hours.

`summary.checks` retains evaluated Boolean results. Each `summary.check_details[name]` provides `status,actual,threshold,relation,scope`. Status is `passed`, `failed`, `not_configured` (a calculated actual value without a limit), or `not_evaluated` (missing inputs or an undefined quantity). The last two states do not count as passes; requesting a limit for a missing quantity raises an error. The motor peak check reports maximum segment peak-envelope utilization; the continuous screen reports speed-normalized torque RMS. Both have a limit of 1.

`summary.assessment_coverage` records basic output-bearing L10 and hotspot fatigue, the configured scope of optional pin/planet-bearing and bus models, and uncovered ring/carrier/housing strength, synchronous dynamics, manufacturing interference and hardware validation. Passing configured constraints does not replace these remaining design tasks.

`geometry.undercut_screen` reports the continuous tooth-count threshold and sun/planet risk for a zero-shift standard full-depth rack, together with unassessed items; `automatic_design_rejection=false`. Actual cutter tips, internal-gear interference and tooth peaking are not evaluated. This screen is not manufacturing acceptance.

Missing rating produces empty per-state `ratings` and null safety summaries. Missing fatigue returns an explicit unconfigured status. Undefined zero-load safety factors or zero-damage life are null, not proof that every failure mode has passed.

JSON uses null for undefined nonfinite statistics. NPZ contains numeric arrays without pickle/object data. `manifest.json` hashes exported files. Raw nodes, interpolated displays and aggregate metrics remain distinct.


## Contact pressure display API

Passage display uses `robot_joint.contact_display` for passage displays. It reads an original `AnalysisResult` without feeding back loads or stiffness:

```python
from robot_joint.contact_display import contact_pass_data, instantaneous_contact_data

passage = contact_pass_data(contact, samples=(201, 241))
instant = instantaneous_contact_data(contact, normal_points=301)
```

`contact_pass_data(result, *, samples=(201,241), face_coordinate_m=None, roll_distance_m=None)` returns a pinion-material-coordinate display dictionary. `face_coordinate_m` starts at the pinion's left edge and `roll_distance_m` is pinion roll, both 1D SI coordinates; `pressure_pa_x_y` is [face points, roll points]. Supply original reference coordinates for pointwise checks. `samples` retains original pressure points; `ordinary_samples` excludes extension branches. `boundary_display_samples` contains only outer-strip boundary display support for centre-only models, not newly solved independent endpoints. Points outside the convex hull are NaN. `member_contact_pass_data(result,member="wheel",**kwargs)` applies the exact conjugate transformation to wheel material coordinates, with same/opposite roll directions for internal/external meshes. `draw_contact_pass(ax,data,maximum_pa=...,title=None,language="zh",member="pinion")` draws pressure and contours using this definition.

`instantaneous_contact_data(result, *, phase_index=None, tooth_pair_index=None, normal_points=301)` selects one pair at one phase and returns `s_edges_m`, `normal_edges_m`, `pressure_pa_s_n` and `source_*` values. The matrix is [along-line cells, normal cells]. Functions use SI; plots convert along-line coordinates to mm and normal coordinates to μm, with different axis display scales. `normal_points` must be an integer of at least 21. Automatic selection uses `interior_snapshot` within its supported scope. With an explicit phase but no pair index, the pair containing that phase's peak-pressure node is selected. Centre-only models preserve each original strip's constants; independent endpoint models use only piecewise-linear endpoint subdivision for display. Instantaneous and passage grids cannot be compared pixelwise.

The local function also returns `source_total_force_n`, `display_integrated_force_n` and `display_relative_force_error`. The latter is the signed, dimensionless `(display integral − original force) / original force`, defined as zero when original force is zero. Constant-centre analytical Hertz integrals satisfy the original strip-force identity. Independent endpoint interpolation treats pressure peaks and half-widths separately, so its displayed integral need not equal the original centre-strip force. Retaining `source_*` is not a display-force-conservation claim. The API does not renormalize pressure; see [contact definitions and limits](contact_patterns.en.md).

Batch `pattern.npz` files retain original fields and add `pass_face_coordinate_m`, `pass_roll_distance_m`, `pass_pressure_pa_x_y`, `pass_samples`, `pass_ordinary_samples`, `pass_boundary_display_samples`, `local_s_edges_m`, `local_normal_edges_m` and `local_pressure_pa_s_n`. `--replot` verifies SHA manifests before drawing. Display interpolation adds no solver points or contact-boundary solution.

`interior_snapshot`, `build_contact_pattern` and `export_contact_pattern` provide additional finite-Hertz-band analysis. Their envelope field `sweep_pressure_pa` differs from the material-coordinate peak-pressure display. See [contact analysis](contact_patterns.en.md).
