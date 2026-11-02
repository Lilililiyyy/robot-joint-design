# 齿轮接触分析

[中文](contact_patterns.md) · [English](contact_patterns.en.md)

接触分析连接逐行星载荷、齿面修形及失准，输出局部压力、传动误差和刚度。图表区分完整齿通过程与同相位的一对齿接触。

## 完整齿通过程压力

![两副完整齿通过程压力](../examples/results/contact_patterns/zh/01_contact_overview.png)

横坐标是从构件齿宽左端起算的位置，纵坐标是该构件的渐开线展长，图中单位均为mm。各材料位置经历的局部峰压由一个基节周期内的接触样本组成，在相同齿、固定局部载荷和周期边界下表示一次完整通过程。

独立端点模型采用端点压力；条带中心模型采用中心压力，并将首末中心值延至原积分单元外边界作为显示支撑。重合材料点取最大值，通过三角线性插值和等压线形成图面。普通齿廓图排除齿顶延伸分支；支撑凸包外保留NaN，显示为灰色。接触压力采用蓝→青→绿→黄→红色阶，零压为蓝色，较高压力向红色递增。

插值只用于显示。彩色面积不是同一时刻的承载面积，凸包也不是独立求得的接触边界。内外啮合按各自共轭关系转换材料坐标，并计入齿宽与轴向偏置。

## 工作面与各行星

默认案例选取正向峰载、反向峰载和低载三种状态，各含3颗行星与内外两副，共18次局部分析、36个构件齿面结果。局部计算采用512相位×41条带，载荷和边界来自系统协调结果；局部加密不回写系统分载。

[正向峰载](../examples/results/contact_patterns/zh/02_positive_planets.png) · [反向峰载](../examples/results/contact_patterns/zh/03_negative_planets.png) · [低载](../examples/results/contact_patterns/zh/04_low_load_planets.png) · [四构件齿面](../examples/results/contact_patterns/zh/05_members.png)

工作面由转矩符号决定。默认低载状态速度为正、转矩为负，采用负工作面。

## 瞬时局部接触

![瞬时局部Hertz带](../examples/results/contact_patterns/zh/06_instantaneous_contact.png)

选定相位与齿对后，沿接触线用`s / mm`，垂直接触线用`n / μm`。两轴显示比例不同，图面长宽比不代表实体尺寸比。

局部Hertz带采用`p(n)=p0 sqrt(max(0,1−(n/b)²))`。中心条带的解析合力为`Fi=wi π p0 b/2`，其中`wi`为接触线单元长度。中心模型保留分片常值；独立端点模型对峰压与半宽分别作线性显示，因此显示面积积分不一定严格等于中心条带合力。原力、显示积分及差值分别记录，压力不作归一化。

默认直齿ISO模型没有齿廓鼓形，单/双齿分担切换会形成压力分区；齿向修形影响宽度方向分布。需要更改齿廓修形或柔度模型时，应明确输入并重新计算。

## 使用方式

```bash
python -m examples.build_contact_patterns --language both
python -m examples.build_contact_patterns --replot
python -m examples.build_contact_patterns --joint-results output/pendulum --output output/contact
```

```python
from robot_joint import contact_pass_data, instantaneous_contact_data

passage = contact_pass_data(contact, samples=(201, 241))
instant = instantaneous_contact_data(contact, normal_points=301)
```

`contact`为单副分析结果。显示网格与原求解采样分别保存，SI矩阵、单位及函数签名见[API](api.md)。图表读取输入和结果清单后生成，可仅重绘已保存数据。

当前关节系统采用无延伸直齿内外啮合。这些图基于降阶接触计算和局部Hertz近似，不是二维/三维FE接触或涂色试验。单副压力结果对标见[结果对标](validation.md)。
