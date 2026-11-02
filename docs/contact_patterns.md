# 完整接触斑点、原始数据与模型边界

[中文](contact_patterns.md) · [English](contact_patterns.en.md)

0.3.0新增可复用的`robot_joint/contact_pattern.py`。原主页的节点图只对应一个相位，不是完整通过程；现在提供**瞬时一对齿有限接触带**、**周期等价完整单齿通过程最大压力**、三行星正反面与低载斑点，以及四个构件齿面的各自材料坐标。

![瞬时与完整通过程](../examples/results/contact_patterns/zh/01_contact_overview.png)

## 哪些数据构成完整通过程

当前几何在一个基节周期内同时求多对相邻齿。`tooth_pair_index`是当前包装相位中的相邻齿编号，不是固定物理齿的永久编号。将一个周期中**所有齿对**的同种材料展长拼接，在相同齿、恒定局部载荷与周期边界假设下，等价于一个齿从进入到退出的完整通过程。不能只跟踪固定编号，也不能把相邻齿对的瞬时接触面积相加后称一对齿斑点。

默认数据包含正向峰载`forward_04`、反向峰载`reverse_04`和低载`forward_01`，每工况3颗行星×2副，独立重算18副、36个构件齿面。系统载荷和失准来自已协调的64×26系统结果；局部接触加密到512相位×41条带。该加密结果**没有反向修改原系统分载/刚度**。完整斑点是冻结本工况载荷的一次齿通过程，不是1.8秒运动任务的最大压力场。

低载`forward_01`虽然角速度为正，但需求扭矩为负，因此采用negative工作面；工作面由扭矩符号决定，不能按运动方向代替。

[正面三行星](../examples/results/contact_patterns/zh/02_positive_planets.png) · [反面三行星](../examples/results/contact_patterns/zh/03_negative_planets.png) · [低载边缘](../examples/results/contact_patterns/zh/04_low_load_planets.png) · [四构件材料坐标](../examples/results/contact_patterns/zh/05_members.png)

## 有限Hertz带及力的核对

每个原条带保留`Fi`、实际条带长度`wi`、峰压`p0`与半宽`b`。直齿接触线沿齿宽，横向以原条带分片常值展开；齿廓沿渐开线材料弧长` s=ρ²/(2rb) `映射，`ρ`为本轮正渐开线展长、`rb`为本轮基圆半径。局部线接触压力为：

`p(s)=p0·sqrt(max(0,1−((s−s0)/b)²))`，只在`|s−s0|≤b`内非零。

原条带满足`Fi=wi·πp0b/2`。本案例最大条带积分绝对差为4.441e-16 N，这是原半椭圆积分恒等式的数值一致性，不是物理精度。完整图在每个材料位置取已求解相位有限带的最大值；没有Delaunay、平滑拟合或向未承载边缘补色。扫掠最大压力的面积积分没有同时受载合力的含义。

瞬时图单独选择**同一个内域相位、同一对齿**，两轮Hertz带均未越过有效形径。其窄带使用独立的自适应弧长栅格，避免全齿面像素漏掉低载窄接触；当前36份图的最大瞬时栅格积分差为0.000342%。单轮整齿图保留完整有效形径，并按各轮自身展长显示。外啮合`ρw=span−ρp`，内啮合`ρw=span+ρp`，其中`span=a·sin(αw)`；不能把内齿圈坐标照搬外啮合反号。

横向`face_m/face_edges_m`沿用源几何的共同轴向坐标，小轮齿宽中面为0；不暗中将大轮重置为自己的中面。样例轴向偏置为0、两轮齿宽相同。接口仅绘制共同重叠齿宽；各构件自身坐标的区别在于纵向材料展长。

## 空白、间隙与齿顶边缘

- 灰色：理论啮合中心路径外的有效齿面；有限Hertz带可跨过该路径端点，但不能跨越实际形径后仍冒充实体压力。
- 白色：在保存的采样相位中未记录到正压，不等价于证明连续时间永不接触。
- 网纹：相邻两个正载材料样本的Hertz带之间、且未被任何其他已解带覆盖的空隙。这只是一项采样不足诊断，**不包含全部端部或零载过渡未知域**。

256×41至512×41的上述最大间隙面积比例由1.6439%降至0.0755%。其分母为理论路径材料弧长×共同齿宽；不能据此宣称其他载荷、误差或所有边界均已收敛。默认峰压相位有时恰在齿顶，局部无限带被实际齿面裁掉后可能只保留约半个齿对力。接口保存原齿对力、解析裁剪后力和栅格积分，**不放大压力来补回被裁掉的合力**。内域瞬时展示避开了该边缘，但整个通过程源峰压仍属于当前局部Hertz模型，尚未得到边缘修正后的二维弹性平衡。

## 复算、接入和数据布局

```bash
python -m examples.build_contact_patterns --phases 512 --strips 41
python -m examples.build_contact_patterns --replot
python -m examples.build_contact_patterns --joint-results output/pendulum --output output/contact
```

只需英文时加`--language en`；`--raw-output output/contact_raw`保存额外完整LTCA原数组/诊断/CSV。默认[完整局部输入](../examples/contact_pattern_inputs.json)含各行星明确扭矩、材料、修形、失准及系统来源SHA，压力响应不作为求解输入。可传自己的`export_joint`目录，由清单绑定输入；仅单工况时直接调用函数：

```python
from robot_joint import build_contact_pattern, export_contact_pattern, interior_snapshot

# contact为现有gear_contact.AnalysisResult，或result.cases[j]['contacts'][mesh][planet]
phase, tooth = interior_snapshot(contact)
pattern = build_contact_pattern(contact, member="wheel", snapshot_index=phase,
    snapshot_tooth_pair_index=tooth, profile_points=1601)
export_contact_pattern(pattern, "output/pattern")
```

每份`data/<case>/<mesh>/planet_XX/<member>/`包含`pattern.npz`、`summary.json`和SHA清单。所有长度m、压力Pa、力N；图中才转mm/MPa。原条带以`phase_index, phase, tooth_pair_index, face_m, roll_pinion_m, roll_wheel_m, strip_width_m, force_n, pressure_pa, half_width_m`保存。矩阵按[齿廓采样, 原齿宽条带]排列：

|数组|坐标/含义|
|---|---|
|`sweep_pressure_pa`|`profile_roll_edges_m`×`face_edges_m`；已求解相位最大有限带压力|
|`instant_pressure_pa`|独立` snapshot_profile_roll_edges_m `×同一`face_edges_m`；同一对齿的窄带压力|
|`unresolved_sampling_mask`|扫掠网格中的部分采样空隙诊断|
|`theoretical_path_mask`|一维扫掠齿廓轴是否在理论啮合中心路径中|

两种压力图使用**不同齿廓网格**，不能逐像素相减。源峰压与显示峰压分别记录。压力图层为栅格，SVG/PDF文字和坐标可编辑；NPZ可重新绘制。逐齿面256/512相位比较另见[resolution_comparison.csv](../examples/results/contact_patterns/resolution_comparison.csv)。全局索引与摘要见[summary.json](../examples/results/contact_patterns/summary.json)。

当前接口只实现共同齿宽等分条带的无延伸直齿内/外啮合。斜齿、齿顶延伸、缺相位或可能含逐齿非周期差异的外部回调明确报错。此功能是既有LTCA结果的局部Hertz斑点后处理，**不是二维/三维FE齿面接触、涂色试验、润滑膜压力或样机验证**。
