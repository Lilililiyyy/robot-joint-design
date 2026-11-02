# Figures and source data

[中文](figures.md) · [English](figures.en.md)

Figures use Python/Matplotlib with blue `#2166ac` and red `#b2182b` as the main series, lighter blue/red for secondary series, and additional line styles, markers or hatching. White backgrounds, fine axes, unit-bearing labels and lowercase panel letters provide a scientific-paper layout. Pressure is nonnegative: its linear scale increases from blue to red, and the middle colour does not denote zero. Dimensions follow the [public Nature guide](https://research-figure-guide.nature.com/figures/building-and-exporting-figure-panels/); this is not a journal-submission compliance claim.

Each figure is 183 mm wide and at most 170 mm high. Exports include 300 dpi PNG previews and SVG/PDF with editable text. Chinese and English figures share the same numerical results. These are deterministic calculations; no experimental replicates, confidence intervals or simulated measurement error bars are added.

|Figure|Data and interpretation|
|---|---|
|Case 01–03|Published inputs, analytical quintic motion and statewise path solutions; continuous motion and representative states are marked separately|
|Case 04|Original phasewise LTCA TE and loaded nodes at the peak-pressure phase; no smoothed pressure interpolation|
|Case 05–06|Explicit-factor rating stresses, support reactions, structural displacement and deterministic angle budget; the static envelope is not a hysteresis test|
|Case 07|Motor power and exact two-node thermal propagation/periodic solution; material and thermal inputs are not hardware-calibrated|
|Case 08–09|Explicitly assumed scalar hotspot history, rainflow damage and same-duty candidates; hatching marks candidates that fail configured constraints|
|Benchmark 01|[comparison.csv](../benchmarks/results/comparison.csv), [cycle.csv](../benchmarks/results/cycle.csv), [strip_forces.csv](../benchmarks/results/strip_forces.csv); sorting for display does not alter saved arrays|
|Benchmark 02|[resolution.json](../benchmarks/results/resolution/resolution.json); every point is an independent solve, and connecting lines add no solver samples|

```bash
python -m examples.build_showcase --output output/showcase --raw-output output/raw
python -m examples.run_benchmarks --output output/benchmarks
python -m examples.check_joint_resolution --output output/benchmarks/resolution
python -m examples.plot_validation --source output/benchmarks --output output/benchmarks
```

The shared style is in `robot_joint/figure_style.py`, case plotting in `examples/build_showcase.py`, and benchmark plotting in `examples/plot_validation.py`. Changing fonts can change layout, so labels and legends need another visual check. See [validation](validation.en.md) for the limits of numerical comparisons.

Chinese plots require Microsoft YaHei, SimHei or Noto Sans CJK SC. On Linux, install the system package `fonts-noto-cjk` and rebuild Matplotlib's font cache. Missing CJK fonts raise an explicit error instead of exporting unreadable glyphs. Add `--language en` to the plotting commands for English-only output without a CJK font.
