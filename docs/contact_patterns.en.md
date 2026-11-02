# Gear contact analysis

[中文](contact_patterns.md) · [English](contact_patterns.en.md)

Contact analysis connects individual planet loads, modifications and misalignment to local pressure, transmission error and stiffness. Figures distinguish a complete tooth passage from one tooth pair at one phase.

## Full tooth-passage pressure

![Both mesh passages](../examples/results/contact_patterns/en/01_contact_overview.png)

The horizontal coordinate starts at the member's left face edge. The vertical coordinate is that member's involute roll; both use mm. Local peak-pressure samples from one base-pitch cycle form a complete passage under identical-tooth, fixed-load and periodic-boundary assumptions.

Endpoint models use independent endpoint pressures. Centre models use strip-centre pressures, extending the first and last values to their original integration-cell edges as display support. Coincident material samples take the maximum; linear triangular interpolation and pressure contours form the display. Ordinary-flank maps exclude tip-extension samples. Regions outside the support hull remain NaN and appear grey. Pressure follows a blue→cyan→green→yellow→red scale, with zero in blue and higher values toward red.

Interpolation is for display. Coloured area is not the simultaneously loaded area, and the hull is not an independently solved contact boundary. Each member uses the proper internal/external conjugate mapping, including face widths and axial offset.

## Flanks and individual planets

The default example includes positive peak, negative peak and low-load states, each with three planets and two meshes: 18 local analyses and 36 member surfaces. Local calculations use 512 phases ×41 strips, with loads and boundaries from the system solution. Local refinement does not feed back into system load sharing.

[Positive peak](../examples/results/contact_patterns/en/02_positive_planets.png) · [Negative peak](../examples/results/contact_patterns/en/03_negative_planets.png) · [Low load](../examples/results/contact_patterns/en/04_low_load_planets.png) · [Four member flanks](../examples/results/contact_patterns/en/05_members.png)

Torque sign selects the loaded flank. The default low-load state has positive speed and negative torque and uses the negative flank.

## Local instantaneous contact

![Local instantaneous Hertz band](../examples/results/contact_patterns/en/06_instantaneous_contact.png)

At a selected phase and tooth pair, `s / mm` follows the contact line and `n / μm` is normal to it. Axis scales differ; the apparent aspect ratio is not a physical dimension ratio.

The local Hertz band uses `p(n)=p0 sqrt(max(0,1−(n/b)²))`. A constant centre strip has analytical force `Fi=wi π p0 b/2`, where `wi` is its contact-line length. Centre values remain piecewise constant. Endpoint displays interpolate peak pressure and half-width separately, so their area integral need not equal centre-strip force exactly. Original force, display integral and difference remain separate; pressure is not rescaled.

The default spur ISO model has no profile crown. Single/double-pair load transitions therefore produce pressure regions, while lead modification affects the face distribution. Changes to profile modification or compliance require explicit inputs and a new solve.

## Use

```bash
python -m examples.build_contact_patterns --language both
python -m examples.build_contact_patterns --replot
python -m examples.build_contact_patterns --joint-results output/pendulum --output output/contact
```

```python
from robot_joint import contact_pass_data, instantaneous_contact_data

passage = contact_pass_data(contact, samples=(201, 241))
instant = instantaneous_contact_data(contact, normal_points=301)
```

`contact` is a single-mesh result. Display grids and solver samples are stored separately. See the [API](api.en.md) for SI matrices, units and signatures. Figures can be regenerated from saved results without solving again.

The joint system currently uses non-extended spur internal/external meshes. These maps use reduced contact calculations and local Hertz approximations, rather than 2D/3D FE contact or marking-compound measurements. Single-mesh pressure comparisons are in [result benchmarks](validation.en.md).
