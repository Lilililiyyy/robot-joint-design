# 图表与源数据

[中文](figures.md) · [English](figures.en.md)

图表采用Python/Matplotlib，蓝`#2166ac`、红`#b2182b`为主要系列，浅蓝/浅红为次系列，同时使用线型、点形或网纹。白底、细坐标轴、带单位的标签及小写分图字母用于论文式排版。压力是非负量，色标从蓝到红线性增加；中间颜色不代表零点。尺寸参照[Nature公开指南](https://research-figure-guide.nature.com/figures/building-and-exporting-figure-panels/)，并非特定期刊的投稿合规声明。

每张图宽183 mm、高度不超过170 mm，输出300 dpi PNG网页预览及保留可编辑文字的SVG/PDF。中英图使用同一份计算结果。确定性计算没有添加实验重复、统计区间或模拟实测误差条。

|图|源数据及含义|
|---|---|
|案例01—03|公开输入、五次轨迹解析式及逐工况路径解；连续运动轨迹与代表工况点分别标记|
|案例04|逐相位原始LTCA TE及峰压相位的原承载节点；无平滑压力插值|
|完整接触01|[瞬时单齿Hertz带与完整通过程最大压力](../examples/results/contact_patterns/zh/01_contact_overview.png)；内外两副分别比较，彩色扫掠面积不是同时承载面积|
|完整接触02—03|[正面三行星](../examples/results/contact_patterns/zh/02_positive_planets.png)、[反面三行星](../examples/results/contact_patterns/zh/03_negative_planets.png)；保持工作面与行星身份，不能将其误差/载荷混合成一张无来源图|
|完整接触04|[低载斑点](../examples/results/contact_patterns/zh/04_low_load_planets.png)；保留未承载边缘和区域状态，未解析域不自动涂成接触或零压|
|完整接触05|[四个构件齿面](../examples/results/contact_patterns/zh/05_members.png)；太阳轮、太阳侧/内齿侧行星齿面与内齿圈采用各自材料坐标|
|案例05—06|显式评级因子应力、支承反力、结构位移与确定性角度预算；静态包络不是迟滞试验|
|案例07|机电功率、两节点热网络精确传播与周期稳态；材料/热参数未经样机标定|
|案例08—09|明确假定的标量热点谱、雨流损伤与同任务方案比较；网纹表示未通过给定约束|
|对标01|[comparison.csv](../benchmarks/results/comparison.csv)、[cycle.csv](../benchmarks/results/cycle.csv)、[strip_forces.csv](../benchmarks/results/strip_forces.csv)；显示排序不改变原数组|
|对标02|[resolution.json](../benchmarks/results/resolution/resolution.json)；每个点是独立求解，连线不增加求解网格|
|设计初筛01|[planet_support/summary.json](../examples/results/planet_support/summary.json)、[逐工况载荷](../examples/results/planet_support/history.csv)及[销设计图](../examples/results/design_screens/zh/01_planet_support.png)；真实销反力配新增几何/材料假设，分别显示应力、挠度与平均刚度|
|设计初筛02|[dc_bus/study.json](../examples/results/dc_bus/study.json)、[段功率与能量](../examples/results/dc_bus/segments.csv)及[母线图](../examples/results/design_screens/zh/02_dc_bus.png)；段内电压由精确能量平方根关系得到，越限后为规定功率继续时的外推|

## 完整接触图的量定义

完整接触图由多个实际求解相位的条带线载荷和Hertz半宽构造有限宽带，再按相同齿、固定载荷的周期假设映射到齿面材料坐标。瞬时对照采用同相位同齿对，且两轮Hertz带均在有效齿面内域，不是峰压相位；完整通过程图在各材料点取该固定工况下一次周期等价齿通过程的最大压力，不是1.8 s任务最大压力。两者使用不同的时间含义，不能据扫掠彩色面积推断瞬时承载面积。

条带沿齿宽的离散、有限Hertz带、齿面边界裁剪及显示栅格均属于模型/后处理定义，不能将其描述为新求得的二维或FE接触场。零压、非作用区域、原采样未解析域分别处理；没有用平滑插值把少量线变成完整斑点。原线力、半宽、相位、材料坐标与压力NPZ，以及源峰压、积分/边缘误差见[接触斑点说明](contact_patterns.md)。默认完整图示例使用512相位×41条带，不能据图面连续自动认定离散收敛。压力栅格显示，SVG/PDF中的文字与坐标可编辑。

```bash
python -m examples.build_showcase --output output/showcase --raw-output output/raw
python -m examples.build_contact_patterns --phases 512 --strips 41
python -m examples.check_planet_support --results output/pendulum --output output/planet_support
python -m examples.check_dc_bus --results output/pendulum --output output/dc_bus
python -m examples.plot_design_screens
python -m examples.run_benchmarks --output output/benchmarks
python -m examples.check_joint_resolution --output output/benchmarks/resolution
python -m examples.plot_validation --source output/benchmarks --output output/benchmarks
```

销与母线命令读取README安装/调用步骤已导出的`output/pendulum`；也可改用上述`output/raw`。版式代码为`robot_joint/figure_style.py`，案例绘图为`examples/build_showcase.py`，接触图为`examples/build_contact_patterns.py`，设计初筛为`examples/plot_design_screens.py`，对标图为`examples/plot_validation.py`。更换字体会改变排版，应重新检查标签和图例。图表数值比较边界见[验证说明](validation.md)。

中文绘图需要微软雅黑、黑体或Noto Sans CJK SC；Linux可安装系统字体包`fonts-noto-cjk`并重建Matplotlib字体缓存。缺少中文字体会明确报错，防止导出方块字。只需英文图时在上述绘图命令添加`--language en`，不需要中文字体。
