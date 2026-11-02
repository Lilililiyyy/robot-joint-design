# Full contact patterns, raw data and model limits

[中文](contact_patterns.md) · [English](contact_patterns.en.md)

Version 0.3.0 adds the reusable `robot_joint/contact_pattern.py`. The old node plot represented one phase. New outputs distinguish a **finite-band snapshot of one tooth pair**, the **sampled pressure envelope of a complete periodic-equivalent tooth passage**, positive/negative and low-load patterns for all planets, and each of the four member flanks in its own material coordinates.

![Snapshot and full passage](../examples/results/contact_patterns/en/01_contact_overview.png)

## What constitutes a complete passage

The geometry solves several neighbouring tooth pairs within one base-pitch period. `tooth_pair_index` identifies a neighbour at the wrapped phase; it is not a permanent physical-tooth identity. Joining **all tooth pairs** in one period by their material roll coordinates is equivalent to one full tooth passage under identical-tooth, fixed local load and periodic-boundary assumptions. Tracking one fixed index is insufficient. Summing simultaneous neighbouring-tooth contact areas does not produce a one-pair footprint.

Default data cover positive peak `forward_04`, negative peak `reverse_04` and low load `forward_01`: three planets and both meshes per state, or 18 independent mesh solves and 36 member surfaces. Loads and misalignments come from the coupled 64×26 system solution; local contact is refined to 512 phases×41 strips. This refinement **does not feed back into the original system load sharing or stiffness**. A full pattern is one tooth passage at the frozen state load, not the maximum pressure field over the 1.8 s motion duty.

Low-load `forward_01` has positive speed but negative demanded torque, so it uses the negative flank. Torque sign selects the flank; motion direction cannot replace it.

[Positive planets](../examples/results/contact_patterns/en/02_positive_planets.png) · [Negative planets](../examples/results/contact_patterns/en/03_negative_planets.png) · [Low-load edges](../examples/results/contact_patterns/en/04_low_load_planets.png) · [Four material flanks](../examples/results/contact_patterns/en/05_members.png)

## Finite Hertz bands and force checks

Every original strip retains force `Fi`, actual strip length `wi`, peak pressure `p0` and half-width `b`. Spur contact lines run across the face; pressure is piecewise constant across the original strips. Profile mapping uses involute material arc length `s=ρ²/(2rb)`, with this member's positive involute roll `ρ` and base radius `rb`. The local line-contact pressure is:

`p(s)=p0·sqrt(max(0,1−((s−s0)/b)²))`, nonzero only for `|s−s0|≤b`.

The original strip satisfies `Fi=wi·πp0b/2`. The maximum strip-integral discrepancy here is 4.441e-16 N: numerical consistency of the half-ellipse identity, not physical accuracy. The full map takes the maximum of solved finite bands at each material position. No Delaunay interpolation, smoothing or coloured filling of unloaded edges is used. Integrating the swept maximum does not give a simultaneously acting resultant force.

The snapshot selects **the same interior phase and tooth pair** for both members, with both Hertz bands inside the effective form limits. It uses its own adaptive narrow-band arc grid so a whole-flank pixel grid cannot miss most of a low-load band. The maximum snapshot raster-integral difference across the 36 surfaces is 0.000342%. Full maps retain the entire effective involute and use each member's own roll coordinate. External meshes use `ρw=span−ρp`; internal meshes use `ρw=span+ρp`, where `span=a·sin(αw)`. The internal-wheel coordinate must not inherit the external sign.

The face coordinate `face_m/face_edges_m` retains the source geometry's common axial reference: zero is the pinion face midpoint. It is not silently recentered on the wheel. The example has zero axial offset and equal widths. Only the common overlapping face is mapped; member-specific coordinates refer to the vertical material roll.

## Blank regions, sampling gaps and tip edges

- Grey: effective flank outside the theoretical centre-contact path. A finite band can cross a path endpoint, but cannot be shown beyond the actual form limit as physical material pressure.
- White: no positive pressure recorded at saved phases; this does not prove permanent separation in continuous time.
- Hatching: gaps between two adjacent positive-load material samples, excluding any position covered by another solved band. This diagnoses only part of sampling uncertainty; **entry/exit and transitions through zero load are not fully classified**.

From 256×41 to 512×41, the maximum gap-area fraction above decreases from 1.6439% to 0.0755%. Its denominator is theoretical-path material arc length times common face width. This does not establish convergence for every load, error or boundary. The peak-pressure phase can fall at a tooth tip, where clipping a locally infinite band to the finite flank can remove about half the pair force. The API records original pair force, analytically clipped force and raster integral; **pressure is not inflated to restore the removed load**. Interior snapshots avoid that edge, but full-passage source peaks still belong to the local Hertz model, without an edge-corrected 2D elastic equilibrium solution.

## Reproduction, API and data layout

```bash
python -m examples.build_contact_patterns --phases 512 --strips 41
python -m examples.build_contact_patterns --replot
python -m examples.build_contact_patterns --joint-results output/pendulum --output output/contact
```

Add `--language en` for English only. `--raw-output output/contact_raw` saves additional complete LTCA arrays, diagnostics and CSV. The [default local inputs](../examples/contact_pattern_inputs.json) contain explicit planet torques, material, modifications, misalignment and system-source SHA values; pressure responses are not solver inputs. A custom `export_joint` directory is verified against its manifest. For one state, call the functions directly:

```python
from robot_joint import build_contact_pattern, export_contact_pattern, interior_snapshot

# contact is a gear_contact.AnalysisResult or result.cases[j]['contacts'][mesh][planet]
phase, tooth = interior_snapshot(contact)
pattern = build_contact_pattern(contact, member="wheel", snapshot_index=phase,
    snapshot_tooth_pair_index=tooth, profile_points=1601)
export_contact_pattern(pattern, "output/pattern")
```

Each `data/<case>/<mesh>/planet_XX/<member>/` contains `pattern.npz`, `summary.json` and a SHA manifest. Lengths are m, pressures Pa and forces N; plots alone convert to mm/MPa. Original strips retain `phase_index, phase, tooth_pair_index, face_m, roll_pinion_m, roll_wheel_m, strip_width_m, force_n, pressure_pa, half_width_m`. Matrices are [profile samples, original face strips]:

|Array|Coordinates and meaning|
|---|---|
|`sweep_pressure_pa`|`profile_roll_edges_m`×`face_edges_m`; maximum finite-band pressure over solved phases|
|`instant_pressure_pa`|Independent `snapshot_profile_roll_edges_m`×the same `face_edges_m`; narrow-band pressure of one pair|
|`unresolved_sampling_mask`|Partial sampling-gap diagnostic on the sweep grid|
|`theoretical_path_mask`|Whether the 1D sweep profile coordinate lies in the theoretical centre-contact path|

The two pressure maps use **different profile grids** and cannot be subtracted pixelwise. Source and displayed pressure peaks are recorded separately. Pressure layers are raster images; SVG/PDF labels and axes remain editable, and NPZ permits replotting. Per-surface 256/512-phase differences are in [resolution_comparison.csv](../examples/results/contact_patterns/resolution_comparison.csv). See the global [summary.json](../examples/results/contact_patterns/summary.json).

The API currently supports non-extended spur internal/external meshes with equal strips over the common face width. Helical geometry, tip extension, missing phases and external callbacks that may introduce nonperiodic tooth differences are rejected explicitly. This is local Hertz-pattern postprocessing of LTCA results, **not 2D/3D FE contact, a marking-compound test, lubricant-film pressure or hardware validation**.
