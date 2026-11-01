# Models, equations and verification limits

[中文](model.md) · [English](model.en.md)

The system modules are newly written reduced engineering models. Independent execution, equation checks, vendor numerical agreement and hardware accuracy are separate verification states.

## Kinematics and load coordinates

The fixed-ring, sun-input, carrier-output, zero-shift spur stage satisfies `zr=zs+2zp` and `i=1+zr/zs`. Equally spaced assembly requires `(zs+zr)/N` to be an integer and positive clearance between neighboring planet tip circles.

Let `R=rs+rp=(rs+rr)/2`. Planet rotational equilibrium makes the two reference-circle tangential force magnitudes equal to ft; each pin carries `2ft`. Therefore `Tout=2RΣft`, with sun and ring torques `rsΣft` and `rrΣft`. Each mesh's normal force is `Fn=ft/cosα`.

Virtual work gives a path displacement `2Rθout`. A normal tooth displacement δn maps to `δn/cosα`, so mesh path compliance is `1/(kn cos²α)`. A pin displacement `2ft/kpin` maps by another factor of 2, giving `4/kpin` in path compliance.

For sun radial stiffness Ksun, radial/tangential unit vectors rj/tj and torque direction s=±1, define `Dj=tj+s tanα rj`. Shared compliance is `D Ksun^-1 Dᵀ`; the sun force is `-s Dᵀf`. The ring's radial term changes sign. These forces and carrier-pin forces satisfy total force and torque equilibrium.

## Unilateral sharing and LTCA coupling

The path problem minimizes complementary energy `fᵀCf/2+gᵀf`, subject to `f>=0` and `Σf=|Tout|/(2R)`. Loaded paths share a common approach; unloaded paths must not penetrate. The active set permits unloading and re-entry.

Compliance includes both meshes, pins, shared sun translation and optional externally condensed terms. Output tangent stiffness is `(2R)²·1ᵀCactive^-1·1`. A separate reciprocal six-DOF structural matrix represents series compliance; its inverse C66 adds to `1/Kout`. Already included flexibility must not be counted again.

LTCA evaluates every planet's two meshes over a full mesh cycle. Mean displacement uses the average phase compliance. Each mesh contributes the next linearization intercept `δmean/cosα−ft/(kn cos²α)`. The outer iteration checks both force change and actual path compatibility, rather than torque balance alone.

The reused gear kernel supplies contact geometry, structural approximations, unilateral loading and local Hertz pressure. ISO mode does not add Hertz indentation twice. Local incremental stiffness freezes nominal rating parameters. The system uses cycle-mean coupling, not synchronous whole-drivetrain dynamics. Loaded angular lag includes the datum of path errors and modifications; global free backlash is evaluated separately.

## Support and bearings

`K u=W` assumes a reciprocal positive-definite matrix after rigid modes have been removed. Bearings lie at z=±L/2; a load at h is shifted through `M*=M+[0,0,h]×F`. Two-support reactions balance force and overturning moment. Axial torque is carried by the drivetrain, not the radial support pair.

Basic bearing life is `L10=(C/P)^p` million revolutions, with p=3 for balls or 10/3 for rollers. Segment revolutions are `n=|ω|dt/(2π)`; basic duty damage is `D=Σ(P/C)^p n/10^6`. Repeated-duty hours are duty duration divided by `3600D`. Dynamic/static coefficients in `P=XFr+YFa` and `P0=X0Fr+Y0Fa` are supplied catalogue-branch inputs. See [NSK](https://www.nsk.com/tools-resources/abc-bearings/dynamic-load-ratings-and-fatigue-life/) and [Schaeffler](https://www.schaeffler.com/remotemedien/media/_shared_media/08_media_library/01_publications/schaeffler_2/catalogue_1/downloads_6/stt_de_en.pdf).

Preload, internal clearance, lubrication/contamination/reliability corrections, small-angle contact coverage, fretting and false brinelling are excluded. Zero speed adds no rolling-fatigue damage in this formula but still requires a static check. Very large light-load life values are not a physical service-life claim.

## Rainflow and finite-life damage

The new turning-point stack retains residual half cycles for open records. Two [MathWorks reference tables](https://www.mathworks.com/help/signal/ref/rainflow.html) supply independent numerical checks without invoking MATLAB. Stress amplitude is `range/2`; the supplied S–N power law is `N=Nref(σref/σa)^m`. Miner accumulation is `Σcount/N`.

Optional Goodman correction is `σa,eq=σa/[1−max(σmean,0)/Su]`. Giving no beneficial correction for compressive mean stress is an explicit engineering convention; see [COMSOL](https://doc.comsol.com/6.3/doc/com.comsol.help.fatigue/fatigue_ug_sme.4.28.html).

Inputs must be stresses in Pa. No multiaxial reduction, material database, endurance limit or actual tooth-by-tooth history generation is provided. Kinematic contact opportunities do not equal fatigue cycles. Truncated repeated records must preserve real ordering and residual-cycle handling.

## Motor, precision and heat

Motor speed is `iωout`. Reflected load torque is `Tout/(iηf)` in motoring, `Toutηb/i` in backdriving and ideal `Tout/i` at zero speed, plus `Jm iαout`. Losses follow power balance, with copper loss `I²R` using consistent Kt/current/resistance conventions. The brake screen uses ideal input load torque and a supplied safety factor, without crediting frictional self-locking; it is not dynamic brake certification.

The uncompensated deterministic angle bound adds absolute loaded offset, half total backlash, output-referred half-count and absolute assembly error. Two-flank static envelopes have no friction history and do not represent measured hysteresis.

The two-node thermal network is `Cθdot=P−Gθ`. G contains ambient and node-to-node conductances. Constant-loss segments use exact matrix-exponential propagation. Each temperature is a sum of two real exponentials, so endpoints and internal derivative zeros are checked for peaks. Periodic mode composes `θend=Aθ0+b`, solves `(I−A)θ0=b`, then checks cycle closure and energy. [SciPy matrix exponential](https://docs.scipy.org/doc/scipy/reference/generated/scipy.linalg.expm.html) supplies an independent numerical reference.

Thermal capacities/conductances, efficiency and loss parameters are supplied. Temperature-dependent resistance feedback, lubrication prediction and spatial temperature fields are excluded.

## Current verification state

The modules have analytical, equilibrium, public-reference-table, numerical and interface tests. The reused single-mesh kernel has its own bounded vendor comparison history. The new system layer has no whole-system commercial benchmark or hardware accuracy validation. The pendulum case is a reproducible source-grounded redesign, not a calibrated replica of the physical testbed.
