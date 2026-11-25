# 设计裕量与有限装配情景

[中文](design_envelope.md) · [English](design_envelope.en.md)

名义方案通过以后，还需要判断装配误差和支承变化会先触及哪项限值。本模块复用完整任务、系统分载与两副LTCA，逐个重算用户明确列出的情景，报告最差约束、裕量和可追溯的危险工况。它不生成失效概率，也不声称覆盖整个连续公差域。

## 直接使用

```bash
python -m examples.design_envelope --output output/design_envelope
```

默认使用[几何刚度输入](../examples/dfki_pin_geometry.json)和[可编辑情景定义](../examples/dfki_assembly_scenarios.json)，自动加入名义方案。修改`--input`、`--scenarios`可替换设计；输出保留全部采用输入、`study.json`、`margins.csv`、双语报告及SHA-256清单。所有情景完成后才替换正式比较文件，未收敛不返回通过结论。[本例正式结果](../examples/results/design_envelope/report.md)使用原64相位、26条带及34个任务状态。

```python
import json
from pathlib import Path
from robot_joint import analyze_joint_envelope, export_joint_envelope

config = json.loads(Path("examples/dfki_pin_geometry.json").read_text(encoding="utf-8"))
spec = json.loads(Path("examples/dfki_assembly_scenarios.json").read_text(encoding="utf-8"))
study = analyze_joint_envelope(config, spec["scenarios"], required_checks=spec["required_checks"])
export_joint_envelope(study, "output/design_envelope")
print(study["worst_checks"]["maximum_load_share_factor"])
```

## 情景输入

每个情景包含唯一ASCII `id`、可选`description`和非空`overrides`。`nominal`保留给自动加入的原设计。支持以下原接口字段，数组整体替换，原配置不修改：

|模块|允许变化的量|
|---|---|
|`load_sharing`|正/反工作面`positive_path_gaps_m/negative_path_gaps_m`、`sun_radial_stiffness_n_per_m`、`extra_compliance_m_per_n`、显式模式的`pin_stiffness_n_per_m`|
|`planet_support.pins`|直径、支承跨距、加载宽度、E、ν、剪切系数；共享局部对象或逐销局部对象列表，与基准销参数合并|
|`support`|`stiffness_matrix`，坐标参考点保持基准定义|
|`mesh`|`sun_planet_misalignment_m/planet_ring_misalignment_m`|

销参数变化要求基准显式选择`pin_geometry`；几何模式不允许覆盖未采用的数值刚度。其他模式继续使用既有直接刚度路径。情景不能改变齿轮几何、载荷任务、材料许用值、评级因子、轴承容量、离散参数或验收限值，以保持同一假设下的比较。超出此范围的方案选型可分别构造完整设计并调用`compare_joint_designs`。

路径误差是两副合并的切向等效路径量，单位m，不是某个孔位尺寸公差本身；实际尺寸误差应先按结构/装配模型投影，不能直接等同。所有额外凝聚柔度须排除已计算的销、齿面和太阳支承，以免重复计入。

## 裕量与判定

`constraint_margins(result)`也可用于一个普通`analyze_joint`结果，且`export_joint`默认保存`constraint_margins.json`并在报告约束表显示相对裕量：

- 上限判据：`margin=threshold−actual`。
- 下限判据：`margin=actual−threshold`。
- `relative_margin=margin/|threshold|`，负值表示超限，0表示恰好达到边界。裕量单位与原判据一致；百分比表示相对输入限值的余量，不是失效概率。
- 未评价或未配置限值时不构造裕量；零限值保留绝对裕量，相对裕量为null。

温度判据沿用SI的K阈值；其百分比不是相对允许温升的比例。不同种类的百分比不组成统一风险评分，应分别结合绝对裕量和模型边界解释。

`required_checks`必须是非空、无重复的已有判据名。任一必需项为`not_evaluated/not_configured`，该情景就不能通过，即使原`configured_checks_pass=true`。任一已配置判据失败也使情景失败。`all_scenarios_accepted`仅表示所有列出点的已配置及必需判据通过。

`worst_checks`逐判据给出最小有符号裕量和情景ID；缺少评价的情景另列，不用已评价点掩盖缺失项。可定位的极值附`governing_location`：任务段索引/标签、行星、齿轮副、构件角色或应力类别，索引均从0开始；相等极值保留首个。电机RMS、热极值、疲劳和L10等累计指标本版不伪造单个LTCA工况，位置可为null。

## 本例含义与下一步设计

本例设路径部分偏差1 μm、三销直径5.98/6.00/6.02 mm、太阳径向刚度50%及一个组合点。它们是敏感性假设，没有制造公差、统计分布或实测标定依据；轴承容量仍沿用演示值。不能把情景通过数量解释为良率或可靠度，也不能称组合点为全域最坏点。

若出现失败，先查看具体判据及危险工况，再缩小相应装配路径误差、调整支承方案或修改设计目标，并使用完整任务重算。若要形成制造验收要求，还需提供实际尺寸链、配合/预紧、结构凝聚柔度、材料与评级依据，以及扭转、温升和服役试验数据。短粗销梁模型的适用范围见[行星支承](planet_support.md)。
