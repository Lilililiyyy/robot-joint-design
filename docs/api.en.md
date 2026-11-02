# API and data contract

[中文](api.md) · [English](api.en.md)

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

The case selects stiffness ≥20000 N·m/rad, load-share factor ≤2 and supplied-record damage ≤1. These are **new case-specific design assumptions, not DFKI hardware specifications**. The production example uses 64 phases ×26 strips; ordinary interface tests use small solves, while the dedicated workflows reproduce the full case and discretization study.

## Results and checks

`result.cases[j]` contains `sharing, contacts, ratings, support, bearing_reactions, precision, speeds, kinematic_exposure`. `contacts` has two groups of N `gear_contact.AnalysisResult` objects, ordered by planet position. Each phase retains contact-line/strip geometry, gaps, forces, active masks, pressures and stiffness.

`sharing.coupling` records iteration history and physical path residuals. Failure raises `CouplingConvergenceError`. Cycle-mean displacement uses mean compliance, hence harmonic-mean phase stiffness. Local tangent stiffness freezes final nominal rating inputs; it is not a finite difference between rerated operating points. Zero-load angular lag is defined as zero, while first-contact position and the right-hand stiffness are retained separately.

`summary.configured_checks_pass` aggregates evaluated constraints only. When rating is configured, default minimum SH and SF are 1; default bearing static safety is 1. A supplied hotspot record defaults to damage ≤1, with an explicit stricter bound available. Bearing hours, minimum stiffness and maximum load-share factor enter acceptance only when their limits are specified. Fatigue evaluates only the supplied record and does not automatically map it to target service hours.

`summary.checks` retains evaluated Boolean results. Each `summary.check_details[name]` provides `status,actual,threshold,relation,scope`. Status is `passed`, `failed`, `not_configured` (a calculated actual value without a limit), or `not_evaluated` (missing inputs or an undefined quantity). The last two states do not count as passes; requesting a limit for a missing quantity raises an error. The motor peak check reports maximum segment peak-envelope utilization; the continuous screen reports speed-normalized torque RMS. Both have a limit of 1.

`summary.assessment_coverage` records basic output-bearing L10 and hotspot fatigue, plus the uncovered planet-pin strength, planet bearings, ring/carrier/housing strength, regenerative bus/inverter, synchronous dynamics, manufacturing interference and hardware validation. Passing configured constraints does not replace these remaining design tasks.

`geometry.undercut_screen` reports the continuous tooth-count threshold and sun/planet risk for a zero-shift standard full-depth rack, together with unassessed items; `automatic_design_rejection=false`. Actual cutter tips, internal-gear interference and tooth peaking are not evaluated. This screen is not manufacturing acceptance.

Missing rating produces empty per-state `ratings` and null safety summaries. Missing fatigue returns an explicit unconfigured status. Undefined zero-load safety factors or zero-damage life are null, not proof that every failure mode has passed.

JSON uses null for undefined nonfinite statistics. NPZ contains numeric arrays without pickle/object data. `manifest.json` hashes exported files. Raw nodes, interpolated displays and aggregate metrics remain distinct.
