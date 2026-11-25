# Finite assembly and support scenario study

Some scenarios fail or lack required checks

|Scenario|Failed configured checks|Required checks missing|Decision|
|---|---|---|---|
|nominal|—|—|Pass|
|path_offsets|maximum_load_share_factor, minimum_output_torsional_stiffness_n_m_per_rad|—|Fail|
|sun_support_soft|—|—|Pass|
|pin_sizes|—|—|Pass|
|combined|maximum_load_share_factor, minimum_output_torsional_stiffness_n_m_per_rad|—|Fail|

|Criterion|Governing scenario|Actual|Limit|Relative margin / %|Governing location|
|---|---|---:|---:|---:|---|
|maximum_load_share_factor|path_offsets|2.775018|2|-38.75091|{"case_index": 0, "label": "forward_01"}|
|minimum_output_torsional_stiffness_n_m_per_rad|combined|11146.43|20000|-44.26784|{"case_index": 17, "label": "reverse_01"}|
|contact_pressure_pa|path_offsets|8.663164e+08|1.2e+09|27.80697|{"mesh": "sun_planet", "planet_index": 1, "case_index": 20, "label": "reverse_04"}|
|planet_pin_static|path_offsets|0.1100396|1|88.99604|{"case_index": 20, "label": "reverse_04", "planet_index": 1, "stress_kind": "projected_pressure"}|
|minimum_planet_bearing_static_safety_factor|path_offsets|3.786515|2|89.32573|{"case_index": 20, "label": "reverse_04", "planet_index": 1}|

Margin is limit minus actual for upper limits, and actual minus limit for lower limits, normalized by the absolute limit. Negative values exceed the limit. Unevaluated, unconfigured and zero-limit checks do not receive a percentage. Full inputs and every criterion are retained.

This covers only the listed finite deterministic scenarios, not a continuous-domain worst case or failure probability. Scenario assumptions need tolerances, measurements or structural evidence. Local pin-hole, fit, carrier-plate response and physical assembly validation remain separate.
