# Pin geometry, sharing and final checks

All four complete designs are recalculated through system/mesh coordination; final pin reactions drive pin and bearing checks. Inputs, actual stiffnesses and hashes are retained.

Pin diameters / mm: Manual equivalent input: 6 / 6 / 6; Geometry source: 6 / 6 / 6; All pins smaller: 4 / 4 / 4; First pin smaller: 4 / 6 / 6.

|Design|Adopted stiffness / MN/m|Maximum sharing factor|Peak pressure / MPa|Minimum output stiffness / N·m/rad|Maximum pin bending / MPa|Minimum bearing static safety|Configured checks|
|---|---|---:|---:|---:|---:|---:|---|
|Manual equivalent input|297.707 / 297.707 / 297.707|1.843442|827.188|23412.890|13.314|4.205881|Pass|
|Geometry source|297.707 / 297.707 / 297.707|1.843442|827.188|23412.890|13.314|4.205881|Pass|
|All pins smaller|71.225 / 71.225 / 71.225|1.683598|819.837|17643.769|44.029|4.292559|Fail|
|First pin smaller|71.225 / 297.707 / 297.707|1.901956|852.199|21141.332|35.073|3.930154|Pass|

Manual input and the geometry source produce 714 elementwise-identical response arrays.

Every design converged in all 34 duty states. Maximum output-torque residual: 8.88e-16 N·m; maximum path residual: 1.63e-09 m.

Failed configured checks: All pins smaller: minimum_output_torsional_stiffness_n_m_per_rad.

The short-pin beam model includes bending and transverse shear, but excludes pin-hole indentation, fits, carrier-plate deformation and local 3D contact. Bearing capacities are held fixed for this comparison; actual diameter changes require bearing reselection.
Input consistency, equilibrium and convergence are numerical integration checks, not physical validation of the joint assembly.
