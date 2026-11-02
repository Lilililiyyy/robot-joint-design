# 离线对标数据

[中文验证说明](../docs/validation.md) · [English validation guide](../docs/validation.en.md)

`reference/`保存独立输入、公开参考输出及身份清单；`results/`保存当前内核复算的数据。参照响应不进入数学求解。运行：

```bash
python -m examples.run_benchmarks --output output/benchmarks
```

|文件|内容|
|---|---|
|`reference/masta_18_77/case.json`|18/77齿轮副完整输入、载荷与显式评级因子|
|`reference/masta_18_77/reference.json`|12个几何/载荷、14个强度和4个LTCA参考标量|
|`reference/masta_18_77/contact.npz`|原参考32相位、5040中心、5166端点的原始数组|
|`reference/masta_18_77/contact.json`|单位、坐标、索引、相位配准和端点定义|
|`reference/masta_18_77/provenance.json`|上游提交、文件哈希及原组件身份|
|`reference/mathworks_rainflow.json`|MathWorks两组公开雨流参考表|
|`results/comparison.csv`|当前内核与MASTA的标量、绝对差、相对差及适用数值判据|
|`results/cycle.csv`|双方32个几何相位的法向TE与切线刚度|
|`results/strip_forces.csv`|双方同相位、同齿对的5040条带力|
|`results/rainflow_comparison.csv`|两组16行、48个标量逐项比较|
|`results/hardware_parameters.csv`|实物参数一致性、简化COM近似差与未解决来源冲突|
|`results/summary.json`|分层状态、残差、逐点差异与模型边界|

全部原始量为SI单位；CSV列名保留单位后缀。NPZ的字段布局由相邻元数据解释，不允许把端点压力与条带中心力直接混用。目录不包含厂商程序、私有工程或反编译源码。上述MASTA证据只覆盖选定单副，不扩展为NGW总成对标；实物参数一致性不是样机精度或寿命验证。

`contact.json`保留上游原字节和其中的历史文件名`showcase_contact.npz`；本仓库将同一数组文件命名为`contact.npz`，映射与SHA-256由`provenance.json`明确绑定。
