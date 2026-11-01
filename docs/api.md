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
|`bearings`|两支承跨距、载荷点偏置、轴向分担和两组评级参数|
|`motor`|同一电流基准的Kt/R、连续/峰值转矩、基速/最大速度、转子惯量和损耗系数|
|`drive`|正向/回驱效率、输入侧静态制动转矩与安全系数|
|`thermal`|两个热容、各对环境热导、节点耦合热导、环境温度；单次模式可给初温|
|`thermal_mode`|`single_duty`或`periodic_steady`；缺省前者，示例选择周期稳态|
|`precision`|总自由侧隙、编码器计数/安装侧、装配误差上界|
|`rating`|可选；`sun_planet`与`planet_ring`分别给定mesh/gear因子及限值|
|`fatigue`|可选；`stress_history_pa`、S-N参考应力/循环数/斜率及可选抗拉强度|
|`design_limits`|可选的压力/角度/温度上限，及强度安全系数、轴承静安全/小时寿命下限|

完整可编辑字段示例见`examples/dfki_pendulum_case.json`，需求与来源见`examples/dfki_pendulum_requirements.json`。嵌套工作面、修形和载荷分配中的未知字段报错，防止拼写错误静默失效。

案例重建时，保存工况与公式生成的有限SI数值采用rtol=2e-12、atol=2e-12校验，允许不同系统数学库的末位差异；字段、标签、工况顺序和六分量布局保持精确匹配。超过容差的实际输入变更明确报错。

## 单位、坐标和分支

- 六分量广义载荷顺序为`[Fx,Fy,Fz,Mx,My,Mz]`，位移为`[x,y,z,θx,θy,θz]`。长度m、角度rad、力N、力矩N·m。刚度的混合块具有对应的力/位移单位，必须满足虚功互易。
- `output_wrench.Mz`与`output_torque_n_m`使用同一个载荷需求值和符号；正扭矩选positive工作面，负扭矩选negative。电机驱动/回驱按`T·ω`判别，工作面按T判别。
- 固定环NGW输出为行星架，`ratio=1+zr/zs`。太阳轮—行星轮副以太阳轮为pinion，行星轮为wheel；行星轮—内齿圈副以行星轮为pinion，内齿圈为wheel。内齿圈齿数保持正整数。
- `positive_path_gaps_m/negative_path_gaps_m`是每颗行星两副合并的切向等效路径误差，正值推迟承载，不是厂商原矩阵协议。全局自由侧隙另由`precision.output_backlash_rad`给定，不应在路径中重复计入同一项。
- `sun_mesh/ring_mesh`刚度为法向N/m；销刚度为切向N/m；最终输出刚度为N·m/rad。初始刚度仅为协调迭代初值。
- `alignment_maps`各为`planet_count×6`，由结构位移得到每副齿宽两端的法向间隙差m。转动列单位m/rad，平移列m/m。映射由调用者明确给出，示例采用行星架倾转的降阶假设。
- 修形遵循复用模块的正去除量约定。单副`pinion/wheel`修形可采用参数、测量曲线、测量曲面或组合；正反工作面分别配置，不隐含镜像。
- 额外路径柔度矩阵允许半正定，总柔度须正定；不得再次包含已经计入的齿面、销和太阳支承。

## 输出与诊断

`result.cases[j]`包含`sharing,contacts,ratings,support,bearing_reactions,precision,speeds,kinematic_exposure`。`contacts`为两副各N个`gear_contact.AnalysisResult`，按行星角度顺序排列。每个相位保留接触线、条带、间隙、载荷、活动掩码、压力和刚度。

`sharing.coupling`登记迭代次数、路径残差和历史；协调未收敛抛出`CouplingConvergenceError`。最终局部切线冻结评级参数，周期平均位移对应各相位柔度的平均；不等同重新评级工况的有限差分刚度。无载时滞后角约定0，首次接触路径量及右切线另存。

`summary.configured_checks_pass`只汇总已计算及已配置约束。配置强度时，最低接触/齿根安全系数缺省要求1；轴承静安全缺省要求1。更高目标可在`design_limits`指定。轴承小时寿命只在明确给出下限时参与判定。疲劳结果只评价输入历程，未自动关联目标服役小时数。

未配置`rating`时各工况ratings为空，汇总安全系数为null；未配置fatigue时返回未配置状态。零载未定义安全系数/无损伤寿命返回null，不能读为已通过所有物理失效模式。

`manifest.json`保存导出文件SHA256。JSON非有限的未定义统计量输出null；NPZ不用对象数组或pickle。原始计算节点与展示图分开保存。
