# 图表与源数据

[中文](figures.md) · [English](figures.en.md)

图表采用Python/Matplotlib，蓝`#2166ac`、红`#b2182b`为主要系列，浅蓝/浅红为次系列，同时使用线型、点形或网纹。白底、细坐标轴、带单位的标签及小写分图字母用于论文式排版。压力是非负量，色标从蓝到红线性增加；中间颜色不代表零点。尺寸参照[Nature公开指南](https://research-figure-guide.nature.com/figures/building-and-exporting-figure-panels/)，并非特定期刊的投稿合规声明。

每张图宽183 mm、高度不超过170 mm，输出300 dpi PNG网页预览及保留可编辑文字的SVG/PDF。中英图使用同一份计算结果。确定性计算没有添加实验重复、统计区间或模拟实测误差条。

|图|源数据及含义|
|---|---|
|案例01—03|公开输入、五次轨迹解析式及逐工况路径解；连续运动轨迹与代表工况点分别标记|
|案例04|逐相位原始LTCA TE及峰压相位的原承载节点；无平滑压力插值|
|案例05—06|显式评级因子应力、支承反力、结构位移与确定性角度预算；静态包络不是迟滞试验|
|案例07|机电功率、两节点热网络精确传播与周期稳态；材料/热参数未经样机标定|
|案例08—09|明确假定的标量热点谱、雨流损伤与同任务方案比较；网纹表示未通过给定约束|
|对标01|[comparison.csv](../benchmarks/results/comparison.csv)、[cycle.csv](../benchmarks/results/cycle.csv)、[strip_forces.csv](../benchmarks/results/strip_forces.csv)；显示排序不改变原数组|
|对标02|[resolution.json](../benchmarks/results/resolution/resolution.json)；每个点是独立求解，连线不增加求解网格|

```bash
python -m examples.build_showcase --output output/showcase --raw-output output/raw
python -m examples.run_benchmarks --output output/benchmarks
python -m examples.check_joint_resolution --output output/benchmarks/resolution
python -m examples.plot_validation --source output/benchmarks --output output/benchmarks
```

版式代码为`robot_joint/figure_style.py`，案例绘图为`examples/build_showcase.py`，对标绘图为`examples/plot_validation.py`。更换字体会改变排版，应重新检查标签和图例。图表数值比较边界见[验证说明](validation.md)。

中文绘图需要微软雅黑、黑体或Noto Sans CJK SC；Linux可安装系统字体包`fonts-noto-cjk`并重建Matplotlib字体缓存。缺少中文字体会明确报错，防止导出方块字。只需英文图时在上述绘图命令添加`--language en`，不需要中文字体。
