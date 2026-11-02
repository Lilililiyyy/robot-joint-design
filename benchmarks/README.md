# 结果对标数据 / Benchmark data

[中文说明](../docs/validation.md) · [English guide](../docs/validation.en.md)

本目录包含MASTA单副结果对标的独立输入、已保存参考数据及本项目计算结果。全部原始量采用SI单位，参照响应不作为求解输入。

This directory contains independent inputs, saved MASTA reference data and calculated results for the single-mesh benchmark. Raw quantities use SI units. Reference responses are not solver inputs.

|目录或文件 / Path|内容 / Contents|
|---|---|
|`reference/masta_18_77/`|几何、材料、载荷、修形、评级输入与参考响应 / Inputs and saved responses|
|`results/comparison.csv`|几何、强度及LTCA标量结果 / Scalar comparison|
|`results/cycle.csv`|周期TE及切线刚度 / Cycle TE and tangent stiffness|
|`results/contact_pressure.json`|通过程压力差异 / Tooth-passage pressure differences|
|`results/zh/`、`results/en/`|正式中英对标图 / Bilingual benchmark figures|

```bash
python -m examples.run_benchmarks --output output/benchmarks
python -m examples.benchmark_contact --output output/contact_benchmark
```

以上结果限定于所列单副工况，不扩展为机器人关节总成或实物验证。

These results cover the stated single-mesh case, rather than joint-assembly or physical validation.
