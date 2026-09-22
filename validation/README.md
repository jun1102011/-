# 验收证据说明

`formal25/raw.mat` 是已验收正式模型的30 s原始Simulink输出，包含变量out。它不是平滑后的曲线或仅有末段统计的汇总。MATLAB读取：

```matlab
s = load('validation/formal25/raw.mat','out');
out = s.out;
out.who
```

各日志的时间轴可能位于数组首维或末维，见 `formal25/log_dimensions.json`；禁止不核对维度就reshape。信号单位和顺序见 [信号索引](../docs/信号与文件索引.md)。

- `formal25/`：正式25 m入漂30 s原始MAT、状态摘要、CarSim echo与运行日志。
- `checks/`：交付指标、14次接管扰动摘要、25/50 m比较、代数/分配/周期/移植核验及源文件追溯。
- `case_summaries/`：局部维持与两项扰动、原生25/50 m、转向接口、普通模式摘要。

本Git版本只上传关键验收证据。文档提及的完整results、source和internal目录未全部上传；完整求解过程、失败对照、各扰动试验原始数据、重复CSV保留原机：

`E:\00硕士毕业论文\第四章_赵选铭漂移复现_维修版\分步维修记录\step19_actual_load_entry_20260922`

原报告按原实验目录撰写，里面的绝对路径用于追溯，仓库内位置以本说明为准。新运行不会自动复现这些已归档文件。原始记录中的绝对路径、模型名和配置哈希保持原样。
