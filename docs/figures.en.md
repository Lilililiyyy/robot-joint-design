# Figures and source data

[中文](figures.md) · [English](figures.en.md)

Figures use Python/Matplotlib with blue `#2166ac` and red `#b2182b` as the main series, lighter blue/red for secondary series, and additional line styles, markers or hatching. White backgrounds, fine axes, unit-bearing labels and lowercase panel letters provide a scientific-paper layout. Pressure is nonnegative: its linear scale increases from blue to red, and the middle colour does not denote zero. Dimensions follow the [public Nature guide](https://research-figure-guide.nature.com/figures/building-and-exporting-figure-panels/); this is not a journal-submission compliance claim.

Each figure is 183 mm wide and at most 170 mm high. Exports include 300 dpi PNG previews and SVG/PDF with editable text. Chinese and English figures share the same numerical results. These are deterministic calculations; no experimental replicates, confidence intervals or simulated measurement error bars are added.

|Figure|Data and interpretation|
|---|---|
|Case 01–03|Published inputs, analytical quintic motion and statewise path solutions; continuous motion and representative states are marked separately|
|Case 04|Original phasewise LTCA TE and loaded nodes at the peak-pressure phase; no smoothed pressure interpolation|
|Contact display 01|[Full tooth-passage pressure and contours for both meshes](../examples/results/contact_patterns/en/01_contact_overview.png); material-coordinate interpolation of raw local peak pressures; coloured area is not simultaneous loaded area|
|Full contact 02–03|[Positive-flank planets](../examples/results/contact_patterns/en/02_positive_planets.png) and [negative-flank planets](../examples/results/contact_patterns/en/03_negative_planets.png); retain flank and planet identity instead of mixing their errors/loads into an unattributed map|
|Full contact 04|[Low-load pressure display](../examples/results/contact_patterns/en/04_low_load_planets.png); the same method as other states, with NaN outside the support convex hull|
|Full contact 05|[Four member surfaces](../examples/results/contact_patterns/en/05_members.png); the sun, sun-/ring-facing planet flanks and internal ring use their own material coordinates|
|Contact display 06|[Local instantaneous Hertz bands](../examples/results/contact_patterns/en/06_instantaneous_contact.png); one pair at one phase, mm along the line and μm normal to it, with different axis display scales|
|Contact display 07|[Control baseline and explicit-modification diagnostic](../examples/results/contact_patterns/en/07_model_comparison.png) and [comparison summary](../examples/results/contact_patterns/model_comparison.json); actual ISO control, Love only and crown replacement with 3 μm per member plus Love at 128×21, with source input and forced controls recorded; the default 512×41 main workflow is unchanged|
|Contact display 08|[18/77 reference-display reproduction](../examples/results/contact_patterns/en/08_reference_reproduction.png) and [check summary](../examples/results/contact_patterns/reference_reproduction.json); retained MASTA and current recalculated fields compared pointwise on the original 30×30 grid with one blue/red scale; upstream-display reproduction error is separate|
|Case 05–06|Explicit-factor rating stresses, support reactions, structural displacement and deterministic angle budget; the static envelope is not a hysteresis test|
|Case 07|Motor power and exact two-node thermal propagation/periodic solution; material and thermal inputs are not hardware-calibrated|
|Case 08–09|Explicitly assumed scalar hotspot history, rainflow damage and same-duty candidates; hatching marks candidates that fail configured constraints|
|Benchmark 01|[comparison.csv](../benchmarks/results/comparison.csv), [cycle.csv](../benchmarks/results/cycle.csv), [strip_forces.csv](../benchmarks/results/strip_forces.csv); sorting for display does not alter saved arrays|
|Benchmark 02|[resolution.json](../benchmarks/results/resolution/resolution.json); every point is an independent solve, and connecting lines add no solver samples|
|Design screen 01|[planet_support/summary.json](../examples/results/planet_support/summary.json), [statewise loads](../examples/results/planet_support/history.csv) and [pin-design figure](../examples/results/design_screens/en/01_planet_support.png); actual pin reactions with new geometry/material assumptions, separating stresses, deflection and mean stiffness|
|Design screen 02|[dc_bus/study.json](../examples/results/dc_bus/study.json), [segment power/energy](../examples/results/dc_bus/segments.csv) and [bus figure](../examples/results/design_screens/en/02_dc_bus.png); within-segment voltage follows the exact square-root energy relation; beyond-limit values extrapolate continued prescribed power|

## Quantities shown in full contact maps

Version 0.3.1 follows the cylindrical-LTCA source project's passage display: assemble raw local peak pressures from all pairs in one base-pitch period in material coordinates, exclude extension branches for the ordinary profile, then apply triangular linear interpolation and pressure contours. Independently solved endpoint pressures are used when available. With centre-only data, the first/last centre values are extended to actual strip boundaries as display support, not claimed as newly solved endpoint pressures. Duplicate coordinates retain the maximum as in the source method. NaN outside the convex hull is grey; zero pressure is blue. The convex hull is not a physical contact boundary, and interpolated points are not new solver samples.

Passage maps use face position in mm × member material roll in mm. They represent one periodic-equivalent passage of identical teeth at fixed local load, not instantaneous whole-flank pressure or maxima over the 1.8 s duty. Local instantaneous bands separately use `s / mm` × `n / μm` with different axis scales. Centre branches retain constant original strip peaks/half-widths; independent-endpoint branches interpolate those quantities separately, so the displayed area integral need not equal original strip force. Original forces and display integrals are reported separately without pressure renormalization. Displayed aspect ratio or coloured area cannot determine physical dimensions or an instantaneous resultant force.

The 0.3.0 finite-band maximum envelope and the current source-style display are different quantities; the previous “no interpolation” description is withdrawn. Baseline pressure zones caused by single-/double-pair switching are retained, while diagnostic modifications identify their changed inputs and computational trade-offs. Default local solves use 512 phases ×41 strips; a finer display grid adds no solver samples. Visual continuity, passing tests and Hertz integral identities cannot independently establish contact boundaries or physical accuracy. See [contact patterns](contact_patterns.en.md) for raw solutions, display support/interpolation and evidence limits. These figures add no 2D or FE contact solve. Pressure layers are rasterized; text and coordinates remain editable in SVG/PDF.

```bash
python -m examples.build_showcase --output output/showcase --raw-output output/raw
python -m examples.build_contact_patterns --phases 512 --strips 41
python -m examples.check_planet_support --results output/pendulum --output output/planet_support
python -m examples.check_dc_bus --results output/pendulum --output output/dc_bus
python -m examples.plot_design_screens
python -m examples.run_benchmarks --output output/benchmarks
python -m examples.check_joint_resolution --output output/benchmarks/resolution
python -m examples.plot_validation --source output/benchmarks --output output/benchmarks
```

The pin and bus commands read `output/pendulum` exported in the README installation/API steps; `output/raw` from the showcase command can also be used. Shared style is in `robot_joint/figure_style.py`, case plotting in `examples/build_showcase.py`, contact maps in `examples/build_contact_patterns.py`, design screens in `examples/plot_design_screens.py`, and benchmarks in `examples/plot_validation.py`. Changing fonts can change layout, so labels and legends need another visual check. See [validation](validation.en.md) for the limits of numerical comparisons.

Chinese plots require Microsoft YaHei, SimHei or Noto Sans CJK SC. On Linux, install the system package `fonts-noto-cjk` and rebuild Matplotlib's font cache. Missing CJK fonts raise an explicit error instead of exporting unreadable glyphs. Add `--language en` to the plotting commands for English-only output without a CJK font.
