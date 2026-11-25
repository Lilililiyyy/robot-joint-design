# Using the project for a real joint design

[中文](practical_workflow.md) · [English](practical_workflow.en.md)

The current project supports parameter calculation, duty screening and candidate comparison for a single-stage spur NGW joint. A product-specific workflow needs traceable inputs, structural analysis and physical tests. Public example constants are assumptions, not product specifications.

## Minimum input package

|Input|Actual evidence needed|Interface|
|---|---|---|
|Duty and installation|Mass/inertia, gravity, trajectory, stops/holding, six-component load reference|Duty, support and bearing inputs|
|Gears and materials|Geometry, heat treatment, quality, modifications and rating-factor basis|Stage, mesh and rating|
|Pins and carrier|Diameters/spans, loaded width, constraints, holes/fits and plate deformation|Planet support and condensed compliance|
|Bearings|Catalog model, C/C0, rotating-ring factors, preload, lubrication and oscillation|Output/planet bearing ratings; oscillatory service remains separate|
|Motor and drive|Consistent Kt/R definitions, speed/torque and thermal derating, regeneration/protection|Motor, drive and bus|
|Requirements|Stiffness, angle, pressure/stress, temperature, static safety and life targets|Design limits and scenario required checks|

Retain each value's source, units and conditions. Treat unknowns as explicit assumptions and study sensitivity rather than copying example constants.

## Working sequence

1. Calculate the complete duty and check signed torque, action/reaction, SI units and references. Inspect missing and unconfigured evaluations first.
2. Review signed margins and governing states in `constraint_margins.json`. Check phase/strip convergence separately for critical discrete quantities such as local peak pressure.
3. Map actual assembly errors or support bounds into [finite scenarios](design_envelope.en.md), then recalculate the duty. A nominal pass does not establish acceptance under variations.
4. Use final reactions for hole, carrier, ring and housing structural analysis. Define references, DOFs and units before adding condensed compliance; exclude already modeled flexibility.
5. Confirm procurement bearings, material allowables, motor/drive and bus capability. Resizing pins requires bearing reselection.
6. Measure torsional response, efficiency and temperature using consistent load/angle/speed references. Retain original forward/reverse, holding and cyclic data before revising support, loss or thermal parameters.

## Next priorities

|Area|Available|Still needed|
|---|---|---|
|Duty/contact|Complete duty, both LTCA meshes, sharing, reactions and raw arrays|Real boundary data and critical-state discretization checks|
|Decisions|Margins, governing locations and finite support/assembly studies|Tolerance projection; continuous-domain and statistical methods are not implemented|
|Measurement loop|[Bidirectional torsion CSV, fit, held-out verification and model comparison](torsion_calibration.en.md)|Use real runs to assess torsion, then import efficiency and temperature data|
|Structure|Six-DOF and path-compliance interfaces|Real carrier/rim/housing models and reference verification|
|Procurement|Parameter CAD/BOM and explicit ratings|Catalog parts, fits/preload, manufacturing and assembly drawings|
|Service/dynamics|Basic L10, supplied hotspot fatigue and quasi-static stiffness|Oscillatory bearing damage, torsional dynamics and assembly endurance|

Measurement feedback, actual structural compliance and procurement-level selection take priority next. They determine whether the model represents the assembled module. Additional charts or broader claims from a single-mesh benchmark do not replace that evidence.
