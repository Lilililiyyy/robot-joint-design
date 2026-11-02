# Planet-pin and planet-bearing screening

[中文](planet_support.md) · [English](planet_support.en.md)

This module implements the pin-strength and planet-bearing items from the design review. It reads already-converged planet-pin reactions and evaluates static stress, bending/shear deflection and basic planet-bearing ratings under an explicit support model. **This post-processing does not overwrite the NGW pin stiffness or automatically recalculate load sharing.**

Alternatively, explicitly add `planet_support={pins,bearing_ratings}` to the main configuration. After load sharing, `analyze_joint` calls the same interface; pin static failure and planet-bearing static failure enter overall screening. Stiffness consistency and basic L10 enter acceptance only with their explicit limits. See the [optional main-workflow API](api.en.md#optional-pin-planet-bearing-and-bus-checks).

## Loads and reference frames

The pin axis is z; transverse input is signed `[Fx,Fy]` in N. The joint's `sharing.carrier_pin_forces_xy_n` retains each pin-force vector. For the current spur NGW, mesh radial components cancel and pin-force magnitude is `2Ft`, not a single mesh's normal force. Torque reversal changes the force direction; stress magnitudes use its resultant. Each support reaction is `−F/2`.

The planet-bearing inner ring follows the pin/carrier and its outer ring follows the planet. Relative speed is therefore `planet_relative_to_carrier_rad_s`, not absolute planet or output speed. The present spur model has zero axial mesh load; helical or externally axially loaded planets need another interface.

## Explicit pin model

Only a solid circular pin with simple supports, centered uniform transverse load over width `w`, linear elasticity and small displacement is supported. Other constraints, cantilevers, eccentric or nonuniform loading raise an explicit error. `L` is support span, `d` is diameter and `0<w≤L`.

`A=πd²/4`, `I=πd⁴/64`, `G=E/[2(1+ν)]`. Effective shear area `κA` uses the explicit `shear_coefficient`. The example's κ=0.9 is an assumption, not a calibration of a physical pin/bore assembly. The [OpenSees Timoshenko-beam documentation](https://opensees.berkeley.edu/wiki/index.php/Elastic_Timoshenko_Beam_Column_Element) identifies E, G, section inertia and effective shear area as separate inputs. This implementation evaluates independently derived closed-form integrals without calling OpenSees.

Maximum moment and transverse shear are `Mmax=F(2L−w)/8` and `Vmax=F/2`. Separate quantities are:

- Maximum bending normal stress `σb=32Mmax/(πd³)`.
- Maximum circular-section transverse shear stress `τmax=4Vmax/(3A)`.
- Average pin/bore projected pressure `pavg=F/(dw)`, which is not rolling-contact Hertz pressure.

Each is compared with its supplied allowable: `actual×required_safety_factor≤allowable`. Peak bending and peak transverse shear occur at different spatial locations; they are not combined into a purported peak von Mises stress. At zero load, safety-factor ratios are null and stresses are zero.

## Deflection and work-conjugate stiffness

On the left half-span, let `a=(L−w)/2` and `<x−a>=max(x−a,0)`:

`M(x)=Fx/2−F<x−a>²/(2w)`, `V(x)=F/2−F<x−a>/w`.

Symmetric integration of bending energy `M²/(2EI)` and transverse-shear energy `V²/(2κGA)` gives load-weighted mean compliances:

`Cb,avg=(5L³−5Lw²+2w³)/(240EI)`, `Cs,avg=(L/4−w/6)/(κGA)`.

Center-deflection compliances are:

`Cb,center=(8L³−4Lw²+w³)/(384EI)`, `Cs,center=(2L−w)/(8κGA)`.

The mean displacement `δavg=F(Cb,avg+Cs,avg)` is work-conjugate to total load. Thus `kpin=1/(Cb,avg+Cs,avg)` is compared with the system's tangential pin stiffness; the reciprocal of maximum center displacement is a different quantity. Outputs retain center displacement, separate bending/shear compliances, strain energy and signed 2D displacement.

Independent piecewise Gauss integration verifies strain energy and unit-load virtual work. Full-span uniform loading, the point-load limit, diameter scaling and load reversal are separately checked. Basic beam boundaries and standard whole-span cases are available in the [Purdue mechanics-of-materials course tables](https://www.purdue.edu/freeform/me323/wp-content/uploads/sites/2/2018/10/ME323_F18_Hw7_final.pdf); the partial central-load expressions above are derived from their stated energy integrals.

Including shear deformation does not turn a short pin into a validated 3D pin/bore assembly. `L/d<10` triggers a conservative project warning, **not a standard acceptance limit**. Bore indentation, support-plate bending, clearance/interference fits, edge contact, local stress concentrations and pin fatigue remain outside the model.

## Basic planet-bearing rating

Each planet's pin-load magnitude and relative speed feed the existing `bearing_duty_life` function for basic L10 damage and static safety. Dynamic/static capacities, life exponent and `X/Y/X0/Y0` must all be explicit; the caller must select catalog parameters for the rotating ring and current load branch. The example assumes a purely radial branch and selects no purchasable bearing.

`n=|ωrelative|dt/(2π)`, `D=Σ(P/C)^p n/10⁶`. A stationary segment adds no damage in this rolling-fatigue model, while static load is still evaluated. See the [official NSK explanation](https://www.nskamericas.com/am-en/tools-resources/knowledge-center/bearing-abcs/dynamic-load-ratings-and-fatigue-life/) for the basic rating formula. Oscillatory contact coverage, fretting/false brinelling, lubrication, contamination, preload and reliability corrections are excluded, so the result is not physical service life.

## API and reproduction

`pin_response(force_xy_n, **pin_options)` returns pin statics, stress, deflection, stiffness, individual criteria and applicability notes. `assess_planet_support(cases, duty_segments, *, pins, bearing_ratings, specified_pin_stiffness_n_per_m=None)` aggregates a complete duty by planet. `pins` and `bearing_ratings` accept a shared object or per-planet list. All fields are demonstrated in the [reproduction script](../examples/check_planet_support.py).

```bash
python -m examples.check_planet_support --results output/pendulum --output output/planet_support
python -m examples.check_planet_support --input examples/dfki_pendulum_case.json --output output/planet_support
python -m unittest discover -s tests -p test_planet_support.py -v
```

The first command directly reuses a full `export_joint` directory after checking input, pin-load and speed SHA-256 values. The second reruns the joint analysis first. `--assumptions custom.json` replaces the candidate pin and bearing parameters. Source modules/functions are directly reusable without the REA workspace or vendor components.

## Two new design assumptions under identical solved loads

The comparison uses 4 mm and 6 mm pins, each with 12 mm support span, 5 mm loaded width, E=210 GPa, ν=0.3 and κ=0.9. Bending/shear/projected-pressure allowables are 300/180/60 MPa with required safety factor 1.5. Illustrative bearing C/C0=800/500 N, p=10/3 and purely radial coefficients 1 are assumed. **These are neither DFKI/mjbots product specifications nor catalog selections.**

|Screening quantity over the current 34 states|4 mm pin|6 mm pin|
|---|---:|---:|
|Maximum pin load / N|118.889|118.889|
|Maximum bending stress / MPa|44.939|13.315|
|Maximum transverse shear stress / MPa|6.307|2.803|
|Maximum average projected pressure / MPa|5.944|3.963|
|Maximum center deflection / μm|1.805|0.433|
|Mean equivalent bending/shear stiffness / MN/m|71.225|297.707|
|Ratio to system input of 300 MN/m|0.2374|0.9924|
|Static criteria using supplied allowables|Pass|Pass|

The 6 mm value is close to the originally assumed system stiffness, but this does not calibrate real support compliance. Both candidates have short spans, L/d=3 and 2, requiring 3D contact/support review. Adopting the 4 mm pin requires explicitly feeding an appropriate structural stiffness into the system and recalculating sharing; retaining 300 MN/m without reconciliation is insufficient.

Illustrative bearing capacities are deliberately held equal to isolate pin-diameter sensitivity, so basic bearing outputs coincide: minimum static safety is about 4.206 and basic L10 about 6.32×10⁵ h. This large value belongs only to the specified formula/capacity/load duty, not physical oscillatory-bearing life. A real diameter change requires bearing reselection and recalculation.

Data: [summary.json](../examples/results/planet_support/summary.json), [per-planet comparison](../examples/results/planet_support/comparison.csv), [state history](../examples/results/planet_support/history.csv). Source-input and pin-load hashes are retained in provenance; original results are unchanged.
