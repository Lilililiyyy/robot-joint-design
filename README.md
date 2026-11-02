# Robot Joint Design

[中文](README.md) · [English](README.en.md)

[![Python](https://img.shields.io/badge/Python-3.10%2B-3776ab?logo=python&logoColor=white)](https://www.python.org/) [![License: AGPL-3.0-only](https://img.shields.io/badge/License-AGPL--3.0--only-blue.svg)](LICENSE)

单级NGW机器人关节模组的独立 Python 设计与准静态校核计算内核。连接载荷谱、行星均载、逐行星LTCA、标准强度、输出支承、机电匹配、精度预算与周期热响应，提供函数接口和可复算的原始数据。运行不依赖商业软件或厂商DLL。

## 目录

- [安装与调用](#安装与调用)
- [计算能力](#计算能力)
- [设计审查与修订](#设计审查与修订)
- [设计案例与方案比较](#设计案例与方案比较)
- [专业软件与公开参考对标](#专业软件与公开参考对标)
- [验证与复算](#验证与复算)
- [适用范围](#适用范围)
- [更新记录与后续计划](#更新记录与后续计划)
- [文档与文件组织](#文档与文件组织)

## 安装与调用

需要 Python 3.10 或更新版本。在项目根目录执行：

```bash
python -m pip install -e ".[plots]"
python -m robot_joint examples/dfki_pendulum_case.json --output output/pendulum
```

仅使用计算内核时，执行 `python -m pip install -e .`。命令入口为 `robot-joint`，也可直接调用函数：

```python
import json
from pathlib import Path
from robot_joint import analyze_joint, export_joint

config = json.loads(Path("examples/dfki_pendulum_case.json").read_text(encoding="utf-8"))
result = analyze_joint(config)
export_joint(result, "output/pendulum")
```

接口采用SI单位：长度 m、角度 rad、力 N、扭矩 N·m、压力 Pa、时间 s、温度 K。每个工况显式提供输出端六分量载荷；轴向力矩 `Mz` 与输出扭矩保持同值、同符号。字段、坐标和数组布局见[接口说明](docs/api.md)。

## 计算能力

|模块|已实现功能|主要输入或边界|
|---|---|---|
|NGW几何与运动学|齿数装配、相邻行星间隙、速比、转速与啮合载荷关系|固定内齿圈、太阳输入、行星架输出；零变位直齿|
|行星载荷分配|双工作面路径误差、销柔度、太阳轮共享平移、非负路径力与脱开/重入|支承刚度及额外凝聚柔度由输入给定|
|逐行星LTCA|内外两副周期TE、切线刚度、接触节点力与局部Hertz压力；位移协调迭代|准静态、周期平均耦合；保留原始接触数组|
|标准强度|ISO 6336:2006显式因子接触/齿根应力及安全系数|外部评级因子、材料限值由用户提供|
|结构与轴承|六自由度凝聚刚度、双支承反力、转数加权基本L10与静安全|不含实际轴承接触、预紧、游隙和小角摆动损伤|
|机电与制动|驱动/回驱效率、转子惯量、转矩—转速包络、RMS电流、静态制动初筛|电机包络、效率、电流基准与制动安全系数由输入给定|
|精度与热|承载角、侧隙、编码器及装配误差预算；双节点瞬态/周期热稳态|确定性角度上界；热容、热导及损耗参数需标定|
|疲劳与方案筛选|雨流、给定S-N、Goodman/Miner；显式设计限值与有限候选Pareto比较|对给定标量热点应力谱有效，不自动预测整机寿命|
|工程交付|JSON/CSV/NPZ、计算报告、CAD尺寸参数、参数级BOM与参考布局|未生成制造实体、采购型号或公差链|

标准评级应力、局部接触压力、输出角度预算和控制定位精度分别定义。模型公式、耦合关系和未覆盖物理见[模型说明](docs/model.md)。

## 设计审查与修订

本轮补入最小输出刚度、最大均载系数与给定热点谱损伤限值，并以 `passed`、`failed`、`not_configured`、`not_evaluated` 四种状态报告实际值、阈值和覆盖范围。结构刚度参考点与载荷端口可以独立定义，偏置载荷通过保持虚功的力/位移变换计入；外齿轮新增标准全齿高齿条假设下的根切风险提示。

当前案例假定 `Kout≥20000 N·m/rad`、`Kγ≤2`、给定谱 `D≤1`，用于同任务方案筛选，不是DFKI公布的实物指标。完整问题、修复和仍需补充的销/轴承强度、制造及样机验证见[设计审查](docs/design_review.md)。

## 设计案例与方案比较

以[DFKI公开实物摆台需求](https://github.com/dfki-ric-underactuated-lab/torque_limited_simple_pendulum/blob/11a946d4e6db58d001fdeff89be269eb61461757/hardware/testbench_description.md)构建6:1关节再设计，几何参照[mjbots钢齿轮组](https://mjbots.com/products/m05-6x-planetary-gearset)。案例用于展示参数化设计流程；新增支承、修形、热参数和评级限值均单独登记为假设。

![需求参照与NGW布局](examples/results/dfki_pendulum/zh/01_design.png)

|参数|取值|
|---|---:|
|摆端质量 / 杆与连接件质量 / 摆长|0.546 kg / 0.130 kg / 0.5 m|
|太阳轮 / 行星轮 / 内齿圈|20 / 40 / 100齿|
|模数 / 行星数 / 有效齿宽|0.5 mm / 3 / 5 mm|
|速比 / 压力角 / 变位|6 / 20° / 零变位|
|新增运动任务|±60°；移动0.5 s、保持0.4 s；周期1.8 s|

供应页中心距表述为30 mm，而上述零变位齿数与模数给出15 mm；模型使用15 mm，并保留来源冲突待确认。本案例是公开需求驱动的新设计，未复原AK80-6内部结构，也未复现其实测性能。

**载荷分配与齿面接触**

34个代表工况连接三颗行星、内外两副接触与系统柔度，当前案例采用64相位×26条带。给定误差使载荷偏离三等分；图中显示周期法向TE及峰压相位原始承载节点。

![逐行星LTCA与原始接触节点](examples/results/dfki_pendulum/zh/04_contact.png)

**机电匹配与周期温升**

同一载荷谱检查驱动/回驱、电机包络与静态制动要求，再将分段损耗输入双节点热网络。温度结果以当前假定热参数为条件。

![机电匹配与周期热响应](examples/results/dfki_pendulum/zh/07_motor_thermal.png)

**同一任务下的方案比较**

保持几何、材料与运动任务相同，比较基准、路径误差放大与柔性行星销方案，观察局部压力、角度预算和输出刚度的取舍。

![三候选方案比较](examples/results/dfki_pendulum/zh/09_comparison.png)

完整九步图文案例、输入来源与所有关键数值见[案例复现指南](docs/examples.md)。共享图表数据见[metrics.json](examples/results/dfki_pendulum/metrics.json)和[duty.csv](examples/results/dfki_pendulum/duty.csv)，图像旁保存可编辑SVG/PDF。蓝红配色、数据来源、输出尺寸及英文图复算方法见[图表说明](docs/figures.md)。CAD/BOM导出用于参数接入。

## 专业软件与公开参考对标

对标按计算层级分别报告：

|证据|核验对象|能支持的结论|
|---|---|---|
|MASTA已保存单副参照|相同输入、工作面、载荷及离散下的圆柱齿轮几何、标准强度与LTCA|当前复用齿面内核在所列工况的数值一致性|
|公开软件参考及独立方程|已列明的算法表值、平衡关系和数值解|相应模块/方程的正确性与回归覆盖|
|DFKI硬件与mjbots几何资料|质量、长度、速比、齿数等公开参数|案例输入可追溯性与几何一致性|

![MASTA单副数值对比](benchmarks/results/zh/01_masta_comparison.png)

18/77斜齿单副在相同输入下重新求解，12项几何与14项强度共同量在声明容差内通过。周期汇总如下，ppm为相对差的百万分数：

|周期指标|MASTA保存参照|当前内核|相对差 / ppm|
|---|---:|---:|---:|
|平均法向TE / μm|1.106704585|1.106704539|0.0414|
|TE峰峰值 / μm|0.034133914|0.034133913|0.0400|
|最大端点接触压力 / MPa|186.674352|186.674382|0.160|
|平均法向刚度 / N/μm|220.932698401|220.932698268|0.000603|

端点压力按参照的端点定义比较，与条带中心峰值分别保存。此外，2组MathWorks公开雨流表的48个标量字段全部一致；复算不运行MATLAB。

参照输入、原始结果、绝对/相对误差与复算入口见[验证说明](docs/validation.md)。单副对标未覆盖NGW总成，公开硬件参数核验也未验证关节的实际温升、刚度、精度或寿命。当前没有KISSsoft、精益传动或ETAGear的总成对标结果。

## 验证与复算

```bash
python -m examples.build_showcase --output output/showcase --raw-output output/raw
python -m examples.compare_designs --output output/comparison.json
python -m examples.run_benchmarks --output output/benchmarks
python -m examples.check_joint_resolution --output output/benchmarks/resolution
python -m examples.plot_validation --source output/benchmarks --output output/benchmarks
python -m unittest discover -s tests -v
```

测试覆盖行星力矩/路径协调、反向载荷与脱开、六自由度支承、机电功率、雨流、解析热响应和接口约束。每个结果保留输入、单位及诊断；收敛性、原组件数值对标与物理试验按不同状态记录。设计限值通过表示所配置模型及阈值下的初筛结果。

离散研究在一个峰压工况比较7组实际求解网格，不证明所有工况或参数的渐近收敛。图表输出为PNG及可编辑SVG/PDF；只生成英文图时，对 `build_showcase` 和 `plot_validation` 添加 `--language en`，详见[图表说明](docs/figures.md)。

## 适用范围

当前系统层限于单级、固定环、太阳输入、行星架输出、零变位直齿NGW，小变形、线弹性与准静态周期平均响应。独立齿面内核具备更多单副能力，但系统接口尚未扩展至斜齿、多级、RV或谐波结构。

尚未覆盖完整箱体/轴系FE、瞬态同步啮合动力学、摩擦历史迟滞、行星针轴承与小角摆动磨损、润滑、电磁、控制精度和制造公差链。轻载基本L10、假定热点谱损伤及确定性角度预算分别解释，不合并为整关节寿命或定位性能认证。

## 更新记录与后续计划

- **0.1.0**：建立NGW系统、支承、疲劳、热、机电、精度与参数级工程交付。
- **0.1.2**：加入DFKI实物需求参照、完整中英案例与图表，修正跨平台案例输入比较的浮点容差。
- **0.2.0**：重组项目级中英说明，统一蓝红论文式图表，补充专家设计审查、显式校核覆盖与可复算对标证据；默认分支统一为 `main`。
- **近期优先**：将离散研究扩大至所有关键工况，补充实测扭转刚度/回差与摩擦标定，识别热网络和效率图谱，并在实际摆动工况评价轴承适用性。
- **后续扩展**：在独立验证后增加多级与斜齿系统、真实轮体/支承柔度、制造公差统计及样机性能比较。以上为计划。

## 文档与文件组织

- [模型与公式](docs/model.md) · [接口与单位](docs/api.md) · [完整案例](docs/examples.md) · [对标与证据边界](docs/validation.md)
- [设计审查与后续验证](docs/design_review.md) · [蓝红图表与源数据](docs/figures.md)
- `robot_joint/`：新写NGW系统与性能模块；`gear_contact/`：复用圆柱齿轮接触内核。
- `examples/`：案例输入、运行脚本及精选结果；`benchmarks/`：离线参照与对标输出。
- `tests/` 与 `.github/`：自动验证；`docs/`：公式、接入和工程说明。

项目采用[AGPL-3.0-only](LICENSE)，源码复用、来源提交及依赖说明见[来源与许可证](THIRD_PARTY_NOTICES.md)。
