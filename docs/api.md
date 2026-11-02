# 接口与数据契约

[中文](api.md) · [English](api.en.md)

## 调用流程

`analyze_joint(config) -> JointResult`：解析JSON兼容的命名配置，计算所有工况。原配置不修改。`export_joint(result, output_dir, plots=False)`：导出输入、诊断、全数组与报告。`compare_joint_designs(configs)`：重算每个完整候选；`results`保留原结果对象，`candidates`给出可序列化比较表，只在已配置约束通过的候选内判定Pareto非支配。

## 主要输入

|字段|含义|
|---|---|
|`schema_version`|当前为1|
|`case_id`|方案标识，输出目录由调用者单独指定|
|`stage`|`sun_teeth,planet_teeth,ring_teeth,planet_count,module_m,face_width_m,pressure_angle_rad`|
|`duty_segments`|每段`duration_s,output_torque_n_m,output_speed_rad_s,output_acceleration_rad_s2,output_wrench`；可附`label`|
|`load_sharing`|初始两副刚度、逐销刚度、太阳径向刚度、两工作面的路径误差，可附额外凝聚柔度|
|`mesh`|材料、相位/条带数、弹性模型、正反工作面两副修形、给定失准与可选传递矩阵|
|`support.stiffness_matrix`|6×6互易正定结构刚度；须排除齿面和已经进入行星路径的柔度|
|`support.reference_offset_m`|可选，缺省0；结构K参考点到输出载荷端口的有符号z距离，m|
|`bearings`|两支承跨距、载荷点偏置、轴向分担和两组评级参数|
|`motor`|同一电流基准的Kt/R、连续/峰值转矩、基速/最大速度、转子惯量和损耗系数|
|`drive`|正向/回驱效率、输入侧静态制动转矩与安全系数|
|`thermal`|两个热容、各对环境热导、节点耦合热导、环境温度；单次模式可给初温|
|`thermal_mode`|`single_duty`或`periodic_steady`；缺省前者，示例选择周期稳态|
|`precision`|总自由侧隙、编码器计数/安装侧、装配误差上界|
|`rating`|可选；`sun_planet`与`planet_ring`分别给定mesh/gear因子及限值|
|`fatigue`|可选；`stress_history_pa`、S-N参考应力/循环数/斜率及可选抗拉强度|
|`design_limits`|可选的压力/角度/温度/均载/热点谱损伤上限，以及刚度、强度安全系数、轴承静安全/小时寿命下限|

完整可编辑字段示例见`examples/dfki_pendulum_case.json`，需求与来源见`examples/dfki_pendulum_requirements.json`。嵌套工作面、修形和载荷分配中的未知字段报错，防止拼写错误静默失效。

案例重建时，保存工况与公式生成的有限SI数值采用rtol=2e-12、atol=2e-12校验，允许不同系统数学库的末位差异；字段、标签、工况顺序和六分量布局保持精确匹配。超过容差的实际输入变更明确报错。

## 单位、坐标和分支

- 六分量广义载荷顺序为`[Fx,Fy,Fz,Mx,My,Mz]`，位移为`[x,y,z,θx,θy,θz]`。长度m、角度rad、力N、力矩N·m。刚度的混合块具有对应的力/位移单位，必须满足虚功互易。
- `output_wrench`在输出载荷端口给定，是设计需求坐标；`Mz`与`output_torque_n_m`同值同号。正扭矩选positive工作面，负扭矩选negative。电机驱动/回驱按`T·ω`判别，工作面按T判别。接入测力仪数据时先区分“关节施加给负载”和“负载施加给关节”的作用/反作用，再统一为本接口符号，不能直接混用反力。
- 固定环NGW输出为行星架，`ratio=1+zr/zs`。太阳轮—行星轮副以太阳轮为pinion，行星轮为wheel；行星轮—内齿圈副以行星轮为pinion，内齿圈为wheel。内齿圈齿数保持正整数。
- `positive_path_gaps_m/negative_path_gaps_m`是每颗行星两副合并的切向等效路径误差，正值推迟承载，不是厂商原矩阵协议。全局自由侧隙另由`precision.output_backlash_rad`给定，不应在路径中重复计入同一项。
- `sun_mesh/ring_mesh`刚度为法向N/m；销刚度为切向N/m；最终输出刚度为N·m/rad。初始刚度仅为协调迭代初值。
- `alignment_maps`各为`planet_count×6`，由输出载荷端口的结构位移得到每副齿宽两端的法向间隙差m。转动列单位m/rad，平移列m/m。映射须与端口位移同坐标同符号，示例采用行星架倾转的降阶假设。
- 修形遵循复用模块的正去除量约定。单副`pinion/wheel`修形可采用参数、测量曲线、测量曲面或组合；正反工作面分别配置，不隐含镜像。
- 额外路径柔度矩阵允许半正定，总柔度须正定；不得再次包含已经计入的齿面、销和太阳支承。

## 结构参考点与返回值

`support_response(wrench, stiffness_matrix, *, reference_offset_m=0.0)`把端口载荷转换到K的参考点求解。令`h=reference_offset_m`、`r=[0,0,h]`，则`Mref=Mport+r×F`；端口平移为`uref+θref×r`，转角保持不变。矩阵形式为`uport=B uref`、`Wref=Bᵀ Wport`，其中`B[0,4]=h`、`B[1,3]=-h`，其余为单位阵。端口柔度为`Cport=B K^-1 Bᵀ`，虚功保持一致。

|返回字段|参考点与含义|
|---|---|
|`generalized_displacement`|输出载荷端口的6分量位移，m/rad|
|`displacement_m`, `rotation_rad`|同一端口的平移和转角|
|`reference_offset_m`|采用的有符号z偏置，m|
|`reference_wrench`|K参考点的6分量载荷，N/N·m|
|`reference_generalized_displacement`|K参考点的6分量位移，m/rad|
|`load_point_compliance`|端口6×6柔度；混合块按对应位移/载荷单位|
|`diagnostics.virtual_work_residual_j`|两参考点虚功差，J|

`bearings.load_offset_m`以双输出轴承中点为原点；它独立于结构偏置。K已经凝聚在端口时，结构偏置应为0，即使端口相对双轴承中点悬伸。当前案例显式使用`support.reference_offset_m=0`与`bearings.load_offset_m=0.012`。工程导出的`coordinate_references`分别登记轴承中点、载荷端口和K参考点；齿轮平面的实际轴向位置没有输入时不补造。

## 方案限值

`design_limits`保留原有压力/角度/温度上限和强度/轴承安全系数、轴承小时寿命下限，并增加：

|字段|比较方向|评价范围|
|---|---|---|
|`minimum_output_torsional_stiffness_n_m_per_rad`|实际值≥限值|全部工况的最小局部输出扭转刚度，N·m/rad|
|`maximum_load_share_factor`|实际值≤限值|有载工况最大行星均载系数；全零载时未定义|
|`fatigue_damage_per_input_history`|实际值≤限值|给定热点标量谱的Miner损伤；无目标服役小时数映射|

当前案例选择刚度≥20000 N·m/rad、均载系数≤2、给定谱损伤≤1，均是**本例新增设计假设，不是DFKI实物规格**。生产示例采用64相位×26条带；普通接口测试使用小型求解，完整案例和离散研究由相应复算入口承担。

## 输出与诊断

`result.cases[j]`包含`sharing,contacts,ratings,support,bearing_reactions,precision,speeds,kinematic_exposure`。`contacts`为两副各N个`gear_contact.AnalysisResult`，按行星角度顺序排列。每个相位保留接触线、条带、间隙、载荷、活动掩码、压力和刚度。

`sharing.coupling`登记迭代次数、路径残差和历史；协调未收敛抛出`CouplingConvergenceError`。最终局部切线冻结评级参数，周期平均位移对应各相位柔度的平均；不等同重新评级工况的有限差分刚度。无载时滞后角约定0，首次接触路径量及右切线另存。

`summary.configured_checks_pass`只汇总已评价的约束。配置强度时，最低接触/齿根安全系数缺省要求1；轴承静安全缺省要求1。给定热点疲劳谱时，其损伤缺省要求≤1，可显式指定更严格上限。轴承小时寿命、最小刚度和最大均载系数只在给出相应限值时参与判定。疲劳损伤仅评价输入谱，不自动关联目标服役小时数。

`summary.checks`保留已评价项目的布尔结果；`summary.check_details[name]`提供`status,actual,threshold,relation,scope`。状态为`passed`（通过）、`failed`（未通过）、`not_configured`（已算实际值但未设限值）、`not_evaluated`（缺少输入或当前量未定义）。后两者不算作通过；对缺失评价量强行配置限值会报错。电机峰值检查的实际值是逐工况最大峰值包络利用率，连续初筛为按转速包络归一化的转矩RMS；两者限值均为1。

`summary.assessment_coverage`登记输出轴承基本L10、热点疲劳和未覆盖的行星销强度、行星轴承寿命、内齿圈/行星架/壳体强度、母线吸能/驱动器、同步动力学、制造干涉及实物验证。已配置约束通过不能替代这些缺失的设计工作。

`geometry.undercut_screen`给出标准全齿高零变位齿条的连续齿数阈值、太阳/行星外齿的根切风险与未覆盖项；`automatic_design_rejection=false`。它不评价实际刀尖、内齿干涉和尖顶，不能当作制造验收。

未配置`rating`时各工况ratings为空，汇总安全系数为null；未配置fatigue时返回未配置状态。零载未定义安全系数/无损伤寿命返回null，不能读为已通过所有物理失效模式。

`manifest.json`保存导出文件SHA256。JSON非有限的未定义统计量输出null；NPZ不用对象数组或pickle。原始计算节点与展示图分开保存。
