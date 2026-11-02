# Design criteria and engineering boundaries

[中文](design_review.md) · [English](design_review.en.md)

Joint design starts from the task and evaluates transmission loads, gear/support response, drive energy, precision and thermal performance. Each decision uses explicit inputs and limits; unconfigured and unevaluated items remain distinct.

## Design criteria

|Object|Requirement|
|---|---|
|Planetary stage|Assembly and clearance conditions, nonnegative path forces, individual-planet torque and displacement compatibility|
|Gear strength|Required flank/root safety with explicit factors; external undercut risk under the declared standard-rack assumption|
|Stiffness and precision|Output stiffness, load-share factor, loaded angle and backlash meet specified targets|
|Structural support|Explicit load and stiffness reference points with a virtual-work-preserving transformation|
|Planet pins|Separate bending, shear, mean pressure and deflection checks; geometric stiffness consistent with system input|
|Bearings|Basic rating from actual reactions and relative speeds; oscillating duty evaluated separately|
|Motor and brake|Torque–speed envelope, RMS current, zero-speed holding loss and static-brake requirements|
|Regeneration|Electrical-power-based return energy, capacitor buffering, absorption and voltage limits|
|Thermal and fatigue|Periodic temperatures and cumulative damage under supplied thermal, loss and S-N inputs|

Results use `passed`, `failed`, `not_configured` and `not_evaluated`, retaining actual values and limits. Passing configured checks means meeting those inputs within the current model.

## Example design choices

The 6:1 example selects `Kout≥20000 N·m/rad`, `Kγ≤2` and supplied-history `D≤1`. These are case-specific targets. Modification, stiffness, thermal parameters and allowable stresses are explicit assumptions.

Assumed 4/6 mm planet pins both meet the supplied static-strength criteria under identical loads, but their mean bending/shear stiffnesses are 71.225/297.707 MN/m. The integrated candidate selects 6 mm and feeds calculated stiffness back into the system for a new load-sharing solve.

The bus compares capacitance and absorption with a 36 V source, 48 V upper limit and supplied inverter efficiency. The integrated candidate uses 4700 μF and reaches about 43.194 V. See [planet support](planet_support.en.md) and the [bus model](dc_bus.en.md).

## Interpreting contact and precision

Passage peak-pressure maps, instantaneous local bands, standard rating stresses and output-angle budgets are distinct quantities. Single/double-pair transitions can produce pressure regions in unrelieved spur models. Changes to modification or compliance require a new solve. Parameter-level CAD/BOM supports detailed structural work rather than serving as manufacturing drawings.

## Further engineering work

|Work|Required inputs or physical evidence|
|---|---|
|Bidirectional stiffness, backlash and friction|Reversing torque–angle measurements and assembly conditions|
|Pin holes, rims, carrier and housing|Detailed geometry, fits, preload and 3D structural analysis|
|Bearing selection and oscillation|Supplier ratings, swing amplitude, lubrication and wear requirements|
|Efficiency and temperature calibration|Current, speed, temperature and loss measurements|
|Drive and absorption hardware|Source capability, voltage/current envelopes, protection and resistor thermal capacity|
|Manufacturing and assembly|Tolerance chains, heat treatment, flank quality and acceptance requirements|

See the [model](model.en.md) for equations and the [worked example](examples.en.md) for use.
