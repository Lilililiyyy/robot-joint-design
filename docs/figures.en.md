# Figures and source data

[中文](figures.md) · [English](figures.en.md)

Figures use Python/Matplotlib with blue `#2166ac` and red `#b2182b` as the main series, lighter blue/red for secondary series, and additional line styles, markers or hatching. White backgrounds, fine axes, unit-bearing labels and lowercase panel letters provide a scientific-paper layout. Pressure is nonnegative: its linear scale increases from blue to red, and the middle colour does not denote zero. Dimensions follow the [public Nature guide](https://research-figure-guide.nature.com/figures/building-and-exporting-figure-panels/); this is not a journal-submission compliance claim.

Each figure is 183 mm wide and at most 170 mm high. Exports include 300 dpi PNG previews and SVG/PDF with editable text. Chinese and English figures share the same numerical results. These are deterministic calculations; no experimental replicates, confidence intervals or simulated measurement error bars are added.

|Figure|Data and interpretation|
|---|---|
|Case 01–03|Published inputs, analytical quintic motion and statewise path solutions; continuous motion and representative states are marked separately|
|Case 04|Original phasewise LTCA TE and loaded nodes at the peak-pressure phase; no smoothed pressure interpolation|
|Full contact 01|[Instantaneous single-tooth Hertz bands and full-passage maximum pressure](../examples/results/contact_patterns/en/01_contact_overview.png) for both meshes; swept coloured area is not simultaneous loaded area|
|Full contact 02–03|[Positive-flank planets](../examples/results/contact_patterns/en/02_positive_planets.png) and [negative-flank planets](../examples/results/contact_patterns/en/03_negative_planets.png); retain flank and planet identity instead of mixing their errors/loads into an unattributed map|
|Full contact 04|[Low-load patterns](../examples/results/contact_patterns/en/04_low_load_planets.png); retain unloaded edges and domain states; unresolved regions are not automatically painted as contact or zero pressure|
|Full contact 05|[Four member surfaces](../examples/results/contact_patterns/en/05_members.png); the sun, sun-/ring-facing planet flanks and internal ring use their own material coordinates|
|Case 05–06|Explicit-factor rating stresses, support reactions, structural displacement and deterministic angle budget; the static envelope is not a hysteresis test|
|Case 07|Motor power and exact two-node thermal propagation/periodic solution; material and thermal inputs are not hardware-calibrated|
|Case 08–09|Explicitly assumed scalar hotspot history, rainflow damage and same-duty candidates; hatching marks candidates that fail configured constraints|
|Benchmark 01|[comparison.csv](../benchmarks/results/comparison.csv), [cycle.csv](../benchmarks/results/cycle.csv), [strip_forces.csv](../benchmarks/results/strip_forces.csv); sorting for display does not alter saved arrays|
|Benchmark 02|[resolution.json](../benchmarks/results/resolution/resolution.json); every point is an independent solve, and connecting lines add no solver samples|
|Design screen 01|[planet_support/summary.json](../examples/results/planet_support/summary.json), [statewise loads](../examples/results/planet_support/history.csv) and [pin-design figure](../examples/results/design_screens/en/01_planet_support.png); actual pin reactions with new geometry/material assumptions, separating stresses, deflection and mean stiffness|
|Design screen 02|[dc_bus/study.json](../examples/results/dc_bus/study.json), [segment power/energy](../examples/results/dc_bus/segments.csv) and [bus figure](../examples/results/design_screens/en/02_dc_bus.png); within-segment voltage follows the exact square-root energy relation; beyond-limit values extrapolate continued prescribed power|

## Quantities shown in full contact maps

Actual phasewise strip line loads and Hertz half-widths form finite pressure bands, mapped into material coordinates under identical-tooth, fixed-load periodicity. Instantaneous comparisons use the same phase/tooth pair with both bands inside their valid flank domains, rather than the peak-pressure phase. Full-passage maps take the maximum experienced pressure in one periodic-equivalent tooth passage at that fixed state, not across the complete 1.8 s duty. Their time meanings differ: swept coloured area does not determine instantaneous loaded area.

Across-face strips, finite Hertz bands, tooth-boundary clipping and display grids are model/postprocessing definitions, not a new 2D or FE contact solution. Zero pressure, non-working regions and domains unresolved by original samples keep distinct states. Sparse lines are not smoothed into an entire patch. Raw line loads, half-widths, phases, material coordinates and pressure NPZs, plus source peaks and integration/edge differences, are documented in [contact patterns](contact_patterns.en.md). The default uses 512 phases ×41 strips; visual continuity does not establish convergence. Pressure is rasterized; text and coordinates remain editable in SVG/PDF.

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
