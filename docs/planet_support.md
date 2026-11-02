# 行星销与行星轴承初筛

[中文](planet_support.md) · [English](planet_support.en.md)

该模块继续落实设计审查中的销强度与行星轴承检查。它读取当前系统已经收敛的逐行星销反力，计算一个明确支承模型下的静应力、弯剪挠度及行星轴承基本评级。**该后处理不回写NGW中的销刚度，也不自动重新分载。**

也可在主配置中显式添加`planet_support={pins,bearing_ratings}`，由`analyze_joint`在分载完成后调用同一接口；销静强度失败及行星轴承静安全失败进入总体初筛。刚度一致性和基本L10只有明确配置相应限值时参与判定。见[可选主流程接口](api.md#可选销行星轴承与母线校核)。

## 载荷与参考坐标

销轴沿z，输入横向力`[Fx,Fy]`，单位N。关节接口的`sharing.carrier_pin_forces_xy_n`保留各销反力向量；当前直齿NGW两副径向分力相消，销力幅值为`2Ft`，不是单副法向力。反向扭矩改变力的方向，静应力幅值按合力计算。支承反力各为`−F/2`。

行星轴承内圈随销/行星架运动、外圈随行星轮运动，其相对转速取`planet_relative_to_carrier_rad_s`，不能使用行星绝对转速或输出转速。当前直齿模型的轴向啮合力为0；变为斜齿或有其他轴向载荷时需另建接口。

## 显式销模型

只支持圆实销、两端简支、中央宽度`w`内均布横载、线弹性小位移。其他约束、悬臂、偏心或非均匀加载明确报错。`L`为支承跨距，`d`为直径，`0<w≤L`。

`A=πd²/4`，`I=πd⁴/64`，`G=E/[2(1+ν)]`。剪切有效面积`κA`由显式`shear_coefficient`给定；示例κ=0.9是设计假设，不是经过实际销孔结构标定的值。[OpenSees Timoshenko梁说明](https://opensees.berkeley.edu/wiki/index.php/Elastic_Timoshenko_Beam_Column_Element)说明E、G、截面惯性矩与有效剪切面积是分别输入的模型量；这里使用自主闭式积分，没有调用该软件。

最大弯矩和横剪力为`Mmax=F(2L−w)/8`、`Vmax=F/2`。分别计算：

- 最大弯曲正应力`σb=32Mmax/(πd³)`。
- 圆截面最大横剪应力`τmax=4Vmax/(3A)`。
- 销/孔投影平均面压`pavg=F/(dw)`；它不是滚动接触Hertz应力。

三者各自与输入许用值比较，`actual×required_safety_factor≤allowable`为该项静校核通过。最大弯曲与最大横剪不在同一空间点，结果不将其直接拼成“最大von Mises应力”。零载安全系数比值返回null，各实际应力为0。

## 挠度与功共轭刚度

在左半跨，令`a=(L−w)/2`，`<x−a>=max(x−a,0)`：

`M(x)=Fx/2−F<x−a>²/(2w)`，`V(x)=F/2−F<x−a>/w`。

以对称性积分弯曲能`M²/(2EI)`和横剪能`V²/(2κGA)`，得到载荷加权平均柔度：

`Cb,avg=(5L³−5Lw²+2w³)/(240EI)`；`Cs,avg=(L/4−w/6)/(κGA)`。

中心最大挠度柔度分别为：

`Cb,center=(8L³−4Lw²+w³)/(384EI)`；`Cs,center=(2L−w)/(8κGA)`。

平均挠度`δavg=F(Cb,avg+Cs,avg)`与总载荷功共轭，因此`kpin=1/(Cb,avg+Cs,avg)`用于和系统中的销切向刚度比较；不能用中心最大挠度倒数代替这一广义刚度。输出还保留中心位移、弯剪柔度分量、应变能及有符号二维位移。

公式通过分段Gauss积分的独立应变能与单位载荷虚功测试；全跨均布、集中力极限、直径尺度律与载荷反转分别验证。基本梁边界与常见全跨载荷的公式可参照[Purdue材料力学公开课程表](https://www.purdue.edu/freeform/me323/wp-content/uploads/sites/2/2018/10/ME323_F18_Hw7_final.pdf)，中央局部载荷公式由上述能量积分得到。

短粗销虽已计横剪柔度，仍不等于真实三维销孔/支承结构。`L/d<10`触发本项目保守提示，**它不是标准验收界限**。销孔压陷、支承板弯曲、间隙/过盈、边缘接触、局部应力集中和销疲劳尚未计算。

## 行星轴承基本评级

每颗行星使用其销载荷幅值和相对转速，调用现有`bearing_duty_life`累积基本L10损伤及静安全。动/静容量、寿命指数、`X/Y/X0/Y0`必须全部显式给定，调用方须选取符合旋转圈和当前载荷分支的目录参数。示例仅采用假设的纯径向分支，未选择真实采购型号。

`n=|ωrelative|dt/(2π)`，`D=Σ(P/C)^p n/10⁶`；零速仅不产生该滚动疲劳损伤，仍计算静载。基本公式见[NSK官方说明](https://www.nskamericas.com/am-en/tools-resources/knowledge-center/bearing-abcs/dynamic-load-ratings-and-fatigue-life/)。该输出不含摆动接触覆盖、微动/假压痕、润滑、污染、预紧和可靠度修正，不能当作实物服役寿命。

## 接口与复算

`pin_response(force_xy_n, **pin_options)`返回单销静力、应力、挠度、刚度、逐项判据和模型提示。`assess_planet_support(cases, duty_segments, *, pins, bearing_ratings, specified_pin_stiffness_n_per_m=None)`对完整任务逐行星汇总；`pins`和`bearing_ratings`可为共享对象或逐行星列表。完整字段可见[复算脚本](../examples/check_planet_support.py)。

```bash
python -m examples.check_planet_support --results output/pendulum --output output/planet_support
python -m examples.check_planet_support --input examples/dfki_pendulum_case.json --output output/planet_support
python -m unittest discover -s tests -p test_planet_support.py -v
```

第一条命令复用`export_joint`的完整目录，核对输入/销反力/转速的SHA-256后直接计算；第二条先重新运行关节分析。`--assumptions custom.json`允许替换`candidates`中的全部显式设计参数。源码和函数可直接复制到其他程序，不依赖REA目录或厂商组件。

## 同一分载下的两组新设计假设

比较4 mm及6 mm销，均假设12 mm简支跨距、5 mm加载宽度、E=210 GPa、ν=0.3、κ=0.9。弯曲/横剪/平均面压许用值为300/180/60 MPa，要求安全系数1.5；示例轴承C/C0=800/500 N、p=10/3、纯径向系数1。**这些不是DFKI或mjbots的原产品参数，也不是采购选型结果。**

|当前34工况下的初筛量|4 mm销|6 mm销|
|---|---:|---:|
|最大销载 / N|118.889|118.889|
|最大弯曲应力 / MPa|44.939|13.315|
|最大横剪应力 / MPa|6.307|2.803|
|最大投影平均面压 / MPa|5.944|3.963|
|最大中心挠度 / μm|1.805|0.433|
|平均等效弯剪刚度 / MN/m|71.225|297.707|
|与系统给定300 MN/m之比|0.2374|0.9924|
|给定许用值的静校核|通过|通过|

6 mm方案的数值接近原系统假定刚度，仍不能证明实际支承刚度已标定；两方案的L/d仅为3和2，需要三维接触与支承检查。若采用4 mm销，应将适用结构模型得到的刚度显式输入系统并重新求解分载，不能仅保留原300 MN/m。

两方案故意保持演示轴承容量相同以隔离销径影响，因此基本轴承输出相同：最小静安全约4.206，基本L10约6.32×10⁵ h。这个大数值只属于所给公式/容量/载荷任务，不能解释为实际摆动轴承寿命；现实改销径须重选轴承并重算。

数据见[summary.json](../examples/results/planet_support/summary.json)、[逐行星比较](../examples/results/planet_support/comparison.csv)和[逐工况历程](../examples/results/planet_support/history.csv)。原输入与销反力文件哈希保存在输出来源记录中，未更改原始结果。
