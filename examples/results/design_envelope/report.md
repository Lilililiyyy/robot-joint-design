# 有限装配与支承情景设计校核

存在未通过或缺少必需校核的情景

|情景|已配置失败项|必需但未完成项|判定|
|---|---|---|---|
|nominal|—|—|通过|
|path_offsets|maximum_load_share_factor, minimum_output_torsional_stiffness_n_m_per_rad|—|未通过|
|sun_support_soft|—|—|通过|
|pin_sizes|—|—|通过|
|combined|maximum_load_share_factor, minimum_output_torsional_stiffness_n_m_per_rad|—|未通过|

|判据|最不利情景|实际值|限值|相对裕量 / %|危险位置|
|---|---|---:|---:|---:|---|
|maximum_load_share_factor|path_offsets|2.775018|2|-38.75091|{"case_index": 0, "label": "forward_01"}|
|minimum_output_torsional_stiffness_n_m_per_rad|combined|11146.43|20000|-44.26784|{"case_index": 17, "label": "reverse_01"}|
|contact_pressure_pa|path_offsets|8.663164e+08|1.2e+09|27.80697|{"mesh": "sun_planet", "planet_index": 1, "case_index": 20, "label": "reverse_04"}|
|planet_pin_static|path_offsets|0.1100396|1|88.99604|{"case_index": 20, "label": "reverse_04", "planet_index": 1, "stress_kind": "projected_pressure"}|
|minimum_planet_bearing_static_safety_factor|path_offsets|3.786515|2|89.32573|{"case_index": 20, "label": "reverse_04", "planet_index": 1}|

裕量：上限取限值−实际，下限取实际−限值，再除以限值绝对值；负值表示超限。未评价、未配置和零限值不构造百分比。所有完整输入保存在本目录，margins.csv保留每项判据及状态。

结果仅覆盖明确列出的有限确定性情景，不代表连续公差域最坏情况或失效概率。情景假设需来自实际公差、测量或结构分析；销孔、配合、架板三维响应和总成实物验证仍需单独完成。
