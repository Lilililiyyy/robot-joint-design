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
|`bearings`|Two-support span, load offset, axial split and two rating dictionaries|
|`motor`, `drive`|Consistently defined Kt/resistance, continuous/peak torque, speeds/inertia, efficiencies and static brake inputs|
|`thermal`, `thermal_mode`|Two capacities, ambient/coupling conductances, ambient temperature; `single_duty` or `periodic_steady`|
|`precision`|Total free backlash, encoder count and location, assembly-error bound|
|`rating`|Optional explicit factors and limits for `sun_planet` and `planet_ring`|
|`fatigue`|Optional scalar stress history in Pa and explicit S–N/mean-stress parameters|
|`design_limits`|Optional upper limits for pressure/angle/temperature and lower limits for rating/bearing safety or life|

SI units are m, rad, N, N·m, Pa, s and K. Unknown fields and unsupported branches raise errors.

## Coordinates, signs and units

The wrench order is `[Fx,Fy,Fz,Mx,My,Mz]`; displacement is `[x,y,z,θx,θy,θz]`. Mixed stiffness blocks carry their corresponding force/displacement units and must satisfy energy reciprocity. `output_wrench.Mz` and `output_torque_n_m` express the same signed load demand. Torque selects the tooth flank; `T·ω` selects motoring or backdriving.

The ring is fixed and the carrier is the output, with `ratio=1+zr/zs`. In `sun_planet`, the sun is `pinion` and planet is `wheel`; in `planet_ring`, the planet is `pinion` and ring is `wheel`. All tooth counts remain positive; the wheel role carries the internal-gear sign.

`positive_path_gaps_m` and `negative_path_gaps_m` are combined tangential-equivalent path errors, one per planet. Positive values delay engagement. They are not vendor matrix fields. Global output backlash is a separate input; do not count the same clearance twice.

Mesh stiffness is normal N/m; pin stiffness is tangential N/m; output torsional stiffness is N·m/rad. Initial stiffness is an iteration seed, not a calibrated result. Additional path compliance must not repeat mesh, pin or sun-support compliance.

Each `alignment_maps` matrix has shape `planet_count×6`. It maps structural displacement to a normal gap difference across the face width. Translation columns have units m/m and rotation columns m/rad. The map is supplied explicitly; the case uses an assumed carrier-tilt mapping.

Positive tooth modification means material removal. Parameter, measured-curve, measured-surface and composite inputs are supported by the reused gear module. Positive/negative flanks are configured separately; there is no implicit mirroring.

## Results and checks

`result.cases[j]` contains `sharing, contacts, ratings, support, bearing_reactions, precision, speeds, kinematic_exposure`. `contacts` has two groups of N `gear_contact.AnalysisResult` objects, ordered by planet position. Each phase retains contact-line/strip geometry, gaps, forces, active masks, pressures and stiffness.

`sharing.coupling` records iteration history and physical path residuals. Failure raises `CouplingConvergenceError`. Cycle-mean displacement uses mean compliance, hence harmonic-mean phase stiffness. Local tangent stiffness freezes final nominal rating inputs; it is not a finite difference between rerated operating points. Zero-load angular lag is defined as zero, while first-contact position and the right-hand stiffness are retained separately.

`configured_checks_pass` aggregates calculated/configured checks only. When rating is configured, default minimum SH and SF are 1; default bearing static safety is 1. Higher values can be supplied. Bearing life enters acceptance only when a lower life limit is explicitly requested. Fatigue evaluates the supplied stress record without automatically mapping it to target service hours.

Missing rating produces empty per-state `ratings` and null safety summaries. Missing fatigue returns an explicit unconfigured status. Undefined zero-load safety factors or zero-damage life are null, not proof that every failure mode has passed.

JSON uses null for undefined nonfinite statistics. NPZ contains numeric arrays without pickle/object data. `manifest.json` hashes exported files. Raw nodes, interpolated displays and aggregate metrics remain distinct.
