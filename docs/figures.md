# 图表与源数据

[中文](figures.md) · [English](figures.en.md)

图表采用Python/Matplotlib，蓝`#2166ac`、红`#b2182b`为主要系列，浅蓝/浅红为次系列，同时使用线型、点形或网纹。白底、细坐标轴、带单位的标签及小写分图字母用于论文式排版。压力是非负量，色标从蓝到红线性增加；中间颜色不代表零点。尺寸参照[Nature公开指南](https://research-figure-guide.nature.com/figures/building-and-exporting-figure-panels/)，并非特定期刊的投稿合规声明。

每张图宽183 mm、高度不超过170 mm，输出300 dpi PNG网页预览及保留可编辑文字的SVG/PDF。中英图使用同一份计算结果。确定性计算没有添加实验重复、统计区间或模拟实测误差条。

|图|源数据及含义|
|---|---|
|案例01—03|公开输入、五次轨迹解析式及逐工况路径解；连续运动轨迹与代表工况点分别标记|
|案例04|逐相位原始LTCA TE及峰压相位的原承载节点；无平滑压力插值|
|接触显示01|[两副完整齿通过程压力与等压线](../examples/results/contact_patterns/zh/01_contact_overview.png)；原始局部峰压的材料坐标插值显示，彩色面积不是同时承载面积|
|完整接触02—03|[正面三行星](../examples/results/contact_patterns/zh/02_positive_planets.png)、[反面三行星](../examples/results/contact_patterns/zh/03_negative_planets.png)；保持工作面与行星身份，不能将其误差/载荷混合成一张无来源图|
|完整接触04|[低载压力显示](../examples/results/contact_patterns/zh/04_low_load_planets.png)；采用与其他工况相同的方法，支撑凸包外保留NaN|
|完整接触05|[四个构件齿面](../examples/results/contact_patterns/zh/05_members.png)；太阳轮、太阳侧/内齿侧行星齿面与内齿圈采用各自材料坐标|
|接触显示06|[局部瞬时Hertz带](../examples/results/contact_patterns/zh/06_instantaneous_contact.png)；同相位一对齿，沿接触线mm、垂直接触线μm，两轴显示比例不同|
|接触显示07|[控制基准与显式修形诊断](../examples/results/contact_patterns/zh/07_model_comparison.png)及[比较摘要](../examples/results/contact_patterns/model_comparison.json)；128×21下实际重算ISO控制基准、仅Love及各轮鼓形替换为3 μm+Love，来源输入及强制控制设置单列，默认512×41主流程不变|
|接触显示08|[18/77参考显示复现](../examples/results/contact_patterns/zh/08_reference_reproduction.png)及[核对摘要](../examples/results/contact_patterns/reference_reproduction.json)；MASTA保存场与本次重算场在原30×30网格逐点比较，同一蓝红色标；上游显示再现误差另列|
|案例05—06|显式评级因子应力、支承反力、结构位移与确定性角度预算；静态包络不是迟滞试验|
|案例07|机电功率、两节点热网络精确传播与周期稳态；材料/热参数未经样机标定|
|案例08—09|明确假定的标量热点谱、雨流损伤与同任务方案比较；网纹表示未通过给定约束|
|对标01|[comparison.csv](../benchmarks/results/comparison.csv)、[cycle.csv](../benchmarks/results/cycle.csv)、[strip_forces.csv](../benchmarks/results/strip_forces.csv)；显示排序不改变原数组|
|对标02|[resolution.json](../benchmarks/results/resolution/resolution.json)；每个点是独立求解，连线不增加求解网格|
|设计初筛01|[planet_support/summary.json](../examples/results/planet_support/summary.json)、[逐工况载荷](../examples/results/planet_support/history.csv)及[销设计图](../examples/results/design_screens/zh/01_planet_support.png)；真实销反力配新增几何/材料假设，分别显示应力、挠度与平均刚度|
|设计初筛02|[dc_bus/study.json](../examples/results/dc_bus/study.json)、[段功率与能量](../examples/results/dc_bus/segments.csv)及[母线图](../examples/results/design_screens/zh/02_dc_bus.png)；段内电压由精确能量平方根关系得到，越限后为规定功率继续时的外推|

## 完整接触图的量定义

0.3.1采用圆柱齿轮LTCA源项目的通过程显示方法：将一个基节周期内所有齿对的原始局部峰压按材料坐标拼接，普通齿廓排除延伸分支，再作三角线性插值并叠加等压线。存在独立端点压力时使用原端点；仅有中心值时，将首末中心值延至实际条带边界作为显示支撑，不声称新增求得端点压力。重合坐标按源方法取最大值，凸包外保留NaN并显示灰色，色标蓝端表示零压力。凸包不是实际接触边界，插值点不是新增求解点。

通过程图采用齿宽mm×构件材料展长mm，表示相同齿、固定局部载荷下的一次周期等价通过程，不是瞬时全齿面压力或1.8 s任务最大值。局部瞬时带另用`s / mm`×`n / μm`，两轴比例不同。中心分支保持原条带峰压/半宽常数；独立端点分支分别线性插值二者，图面积积分未必等于原条带合力，原力与显示积分另报且不重新归一化压力。不能据图面长宽比或彩色面积推断实体尺寸或瞬时合力。

0.3.0有限带最大值包络与本版源项目显示不是同一个量；旧“无插值”说明已撤回。原基准中单/双齿切换造成的压力分区保留，另行标注修形诊断的输入改变和计算代价。默认局部解为512相位×41条带，更密显示网格不增加求解样本；图面连续、测试通过或Hertz积分恒等式均不能独自证明接触边界及物理准确性。原解、显示支撑、显示插值与证据范围见[接触斑点说明](contact_patterns.md)。这些图不属于新增二维或FE接触求解。压力层为栅格，SVG/PDF文字与坐标可编辑。

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
