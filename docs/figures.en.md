# Figures

[中文](figures.md) · [English](figures.en.md)

Figures use white backgrounds and fine axes. Line plots use blue `#2166ac` and red `#b2182b`, supported by line and marker styles. Nonnegative contact pressure uses a blue→cyan→green→yellow→red scale with black contours, starting at zero; grey denotes unsupported regions. Figures are 183 mm wide and no more than 170 mm high, with 300 dpi PNG and editable-text SVG/PDF exports. Both languages use identical numerical results.

|Figure group|Design meaning|
|---|---|
|Example 01–03|Transmission layout, motion loads and individual planet sharing|
|Example 04–06|Cycle TE/contact nodes, standard strength, support and precision budgets|
|Example 07–09|Motor/thermal response, supplied-history damage and candidate comparison|
|Contact 01–05|Both mesh passages, positive/negative/low loads, planets and member flanks|
|Contact 06|One-pair instantaneous local Hertz band|
|Design screens 01–02|Pin strength/stiffness and bus capacitance/absorption|
|Benchmarks 01–02|Single-mesh TE, stiffness and passage-pressure comparisons; see [result benchmarks](validation.en.md)|

Passage maps use face/material-roll coordinates in mm. Instantaneous maps use mm along the line and μm across it. Display interpolation adds no solver samples. Grey denotes unsupported regions; coloured area is not simultaneously loaded area. See [contact analysis](contact_patterns.en.md).

```bash
python -m examples.build_showcase --output output/showcase
python -m examples.build_contact_patterns --output output/contact
python -m examples.plot_design_screens
```

Add `--language en` for English-only figures. Chinese figures require Microsoft YaHei, SimHei or Noto Sans CJK SC. Figures use parameterized inputs and calculated results without fabricated experimental replicates or statistical error bars.
