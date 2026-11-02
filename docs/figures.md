# 图表说明

[中文](figures.md) · [English](figures.en.md)

正式图表采用白底、细坐标轴；线图以蓝`#2166ac`与红`#b2182b`为主，并辅以线型及点形。非负接触压力统一采用蓝→青→绿→黄→红色阶和黑色等压线，色标从0开始，灰色表示支撑域外。图宽183 mm、高度不超过170 mm，输出300 dpi PNG及保留文字的SVG/PDF；中英图读取相同计算结果。

|图组|设计含义|
|---|---|
|案例01—03|传动布局、运动载荷及逐行星分载|
|案例04—06|周期TE与接触节点、标准强度、支承及精度预算|
|案例07—09|机电和周期温升、给定谱损伤及候选方案比较|
|接触01—05|两副通过程压力、正反/低载、各行星与各构件齿面|
|接触06|同相位同齿对的瞬时局部Hertz带|
|设计初筛01—02|行星销强度与刚度、母线电容与吸收能力|
|结果对标01—02|单副TE、刚度及通过程接触压力比较，见[结果对标](validation.md)|

通过程图采用齿宽—材料展长mm坐标，瞬时图采用沿线mm与垂线μm坐标。显示插值不新增求解点；灰色表示支撑域外，彩色面积不代表同一时刻的承载面积。详细定义见[接触分析](contact_patterns.md)。

```bash
python -m examples.build_showcase --output output/showcase
python -m examples.build_contact_patterns --output output/contact
python -m examples.plot_design_screens
```

绘图命令可加`--language en`仅生成英文。中文需微软雅黑、黑体或Noto Sans CJK SC。图表使用参数化输入和正式计算结果；未添加模拟实验重复或统计误差条。
