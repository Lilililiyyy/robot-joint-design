# 直流母线回馈能量与过压初筛

[中文](dc_bus.md) · [English](dc_bus.en.md)

此模块继续设计审查中的再生母线工作，将已计算的电机电功率历程转换为DC侧能量、电容电压及给定上限下的缓冲需求。它是可独立调用的新写能量模型，不依赖商业驱动软件。相关背景见[maxon回馈与过压说明](https://support.maxongroup.com/hc/en-us/articles/360006243333-Measures-in-case-of-energy-recovery-and-overvoltage-error-states)及[Kollmorgen母线电容说明](https://www.kollmorgen.com/en-us/developer-network/akd2g-low-voltage-dc-operation)（2026-11-02访问）：回馈能量可能使母线电压上升，电容及吸收装置能力需分别设计。此处未按任何具体驱动器进行标定。

## 输入功率与能量约定

输入每段的`duration_s`（s）和`electrical_power_w`（W）。应直接采用`motor_duty`的电机侧有符号电功率：

`Pe = Tmotor·ωmotor + Pcu + Pother`。

`Tmotor`已经包含转子惯性力矩，因此该输入包含转子惯性功率、铜耗及其他电机损耗。**负的输出机械功率不一定意味着母线回馈**；不能以`Tout·ωout<0`直接构造负电功率。

给定双向逆变效率后，驱动时`Pdc=Pe/ηm`，回馈时`Pdc=Pe·ηr`。`Pdc>0`表示从DC母线取能，`Pdc<0`表示向其回馈。逆变损耗为`Pdc−Pe≥0`；不重复计入已有铜损、齿轮损耗或转子能量。

## 分段解析模型

定义源电压`Vs`、电容`C`以及高于源电压的可用能量：

`U = C(V²−Vs²)/2`，`V = sqrt(Vs²+2U/C)`。

在本初筛模型中，理想电源可无限供能，使电压不低于`Vs`；它是否可以吸收回馈由另一个显式参数`absorption_power_limit_w=A`给定。驱动时，电容先释放已有`U`，到0后由电源供能。回馈时，外部吸收器在电容高于源电压时最多以`A`吸能，`Udot=−Pdc−A`；到`U=0`后，吸收只等于当时回馈，避免把母线抽至源电压以下。外部吸收器只在`Pdc<0`的回馈段启用。

每段`U`按常斜率推进并在0处截断，准确记录上限跨越和回落至源电压的段内时刻。全历程的能量残差为：

`Uend−Uinitial + ∑Pdc·dt + Eabsorbed−Esource`。

这不是制动斩波器、电阻、电池充电或有限带宽的电源控制模型。未配置电阻支路时不会把过压电压人为钳位。越过上限后的曲线是**假定规定功率继续施加时的自由能量推算**；真实驱动器可能在此前保护关断，因此不能将其解释为实物必达电压。

## 调用与复算

```python
from robot_joint.dc_bus import dc_bus_screen

bus = dc_bus_screen(
    result.motor["segments"],
    capacitance_f=470e-6,
    source_voltage_v=36,
    minimum_voltage_v=30,
    maximum_voltage_v=48,
    inverter_motoring_efficiency=0.96,
    inverter_regenerating_efficiency=0.96,
    absorption_power_limit_w=0,
)
```

`initial_voltage_v`可省略，缺省等于源电压。参数须满足`0<Vmin≤Vs≤Vinitial≤Vmax`，且`Vmax>Vs`；`C>0`、`A≥0`、效率位于`(0,1]`。无效工况与不完整功率输入明确报错。函数不修改输入。

```bash
# 只重算机电历程，读取同一个公开案例配置，不重跑LTCA
python -m examples.check_dc_bus --output output/dc_bus
# 或使用export_joint已经导出的电机结果
python -m examples.check_dc_bus --results output/pendulum --output output/dc_bus
```

输出`study.json`、`comparison.csv`、`segments.csv`与`voltage.csv`。全部数值采用SI单位；公开精选输出见[结果目录](../examples/results/dc_bus/)。来源文件哈希写入`study.source`，未将绝对私有路径写入公开数据。

## 案例新增设计假设及结果

以下36V源、30/48V上下限、96%双向逆变效率、470/4700μF电容及0/15W外部吸收能力，全部是**新设计假设**，不是DFKI样机或原驱动器规格。保持34个电机代表工况不变。

|候选|电容 / μF|回馈吸收上限 / W|推算峰值电压 / V|48V上限|
|---|---:|---:|---:|---|
|小电容、无吸收|470|0|83.624|未满足|
|增大电容、无吸收|4700|0|43.194|满足本模型约束|
|小电容、外部吸收|470|15|36.000|满足本模型约束|

本任务电机侧电功率有10个回馈段。经给定逆变效率后，DC侧回馈峰值为13.3983W，完整输入历程的回馈能量为2.67755J。无外部吸收、从36V起始时，需要至少2656.30μF才能在本模型下不超过48V；该数值不含电容容差、老化、温度及器件电压裕量。470μF方案的最大额外缓冲能量缺口为1.10190J；这是所观察峰值处的缓冲容量差，不是制动电阻整周期耗散能量。15W吸收方案实际平均吸收1.48753W，峰值13.3983W。

## 输出解释与边界

- `maximum_voltage_v`、`events`：解析端点/阈值结果；`time_s/voltage_v`仅保存段端及事件点。若绘制段内精确曲线，应使用`segments.net_capacitor_power_before_floor_w`与初始`U`计算平方根曲线，不能将直线连接点称为真实高频电压波形。
- `minimum_capacitance_f_for_nominal_start`：同一功率/吸收模型、从源电压开始的一次历程所需能量电容。它不声称覆盖任意初始电压、控制故障或重复历程的未知初态。
- `within_minimum_voltage_by_ideal_source_assumption`：源电压下限假设保证的数值结果，**不是有限电源、线路压降或实物欠压能力检查**。
- `within_maximum_voltage`与`physical_validation`分别记录模型约束和实物验证状态。独立函数只返回结果；在`analyze_joint`配置顶层`dc_bus`后，最高母线电压会纳入`summary.configured_checks_pass`。理想源保证的最低电压仍标记`not_evaluated`，不能视为实物欠压通过。见[完整API](api.md)。

未覆盖PWM、相电感能量、线路ESR/ESL、母线纹波、驱动器电流/电压/峰值持续时间、保护关断与电阻热容量；也未由Kt推断跨电流基准的Ke。静态机械制动器和再生母线是不同功能。9项独立测试覆盖电容解析能量、双向效率、回馈/机械功率区别、源电压及上限阈值事件、有限吸收、分段不变性、守恒和无效输入。
