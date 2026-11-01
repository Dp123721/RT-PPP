# BIE 算法说明

本文说明实时 PPP 中 BIE 模糊度固定的输入条件、固定模式、持久化门限和验证范围。

## 范围

- `LAMBDA (MW + NL)` 保持原有参数和处理流程。
- `BIE` 只处理以周表示的 `AMB_L1/AMB_L2` 状态；以米表示的 `AMB_IF` 不进入整数搜索。
- BIE 需要 `Pi&Li` 或 `P1&L1` 组合，并要求参与固定的频点匹配整数相位偏差。
- BIE 参数通过 PPP 设置界面和 `ppp/arBie*` 配置项管理。

## 处理步骤

1. 按系统和频点对模糊度分组，检查历元数、高度角和整数相位偏差指示。
2. 在每组中选择双差方差最小的参考卫星。
3. 使用 Cholesky 与平方根 QR Kalman 更新联合条件化参考模糊度。
4. 对条件化后的协方差进行 LDL 分解和 decorrelation，再执行候选搜索及 BIE 加权。
5. 按 fractional part 与 sigma 阈值筛选候选，并保留可接受约束及后验状态量。

当一组达到最少可固定卫星数量后，约束结果包含该组所有逐项可固定模糊度，包括参考项。非正的筛选阈值会同时关闭 fractional part 和 sigma 两项筛选。

## 状态更新模式

默认 `ppp/arBiePersistConstraints=false`。程序复制浮点状态和协方差，在副本上生成当前历元固定结果与诊断记录，随后继续使用主滤波器的浮点状态。GUI、PPP JSON、ZTD 和下一历元预测都读取主状态。

启用持久化后，程序按 `系统|频点|卫星` 跟踪连续相位弧。只有连续确认通过、达到组内稳定数量、后验协方差收敛，且垂向坐标与 ZTD 改变量均未超过门限的候选子集，才会通过矩匹配写回主滤波器。参考星重选和无关卫星变化不会清除其他卫星的连续状态；周跳、相位粗差、锁定/不连续计数或 SSR 相位偏差连续性变化只重置受影响的卫星弧。

## 配置

| 设置 | 配置键 | 默认值 |
| --- | --- | --- |
| AR algorithm | `ppp/arAlgorithm` | `LAMBDA` |
| BIE minimum epochs | `ppp/arBieMinEpochs` | 10 |
| BIE minimum ambiguities | `ppp/arBieMinSat` | 4 |
| BIE max fractional part | `ppp/arBieMaxFrac` | 0.01 |
| BIE max sigma | `ppp/arBieMaxSig` | 0.001 cycle |
| BIE constraint sigma | `ppp/arBieConstraintSigma` | `1e-4` cycle |
| Persist BIE constraints | `ppp/arBiePersistConstraints` | `false` |
| BIE persist confirmation epochs | `ppp/arBiePersistConfirmEpochs` | 3 |
| BIE persist max U correction | `ppp/arBiePersistMaxVerticalM` | 0.10 m |
| BIE persist max ZTD correction | `ppp/arBiePersistMaxZtdM` | 0.03 m |

当前配置读取器会将旧算法标识归一化为 `BIE`；保存时只写入当前标识。其余 BIE 参数键保持不变。

## 文件职责

- `ambbie.*`：分组、参考项选择、联合候选搜索、候选筛选和后验约束结果。
- `ambfix.*`：MW 宽巷与窄巷 LAMBDA 固定流程。
- `ppp_engine.*`：模糊度状态生命周期、观测筛选、固定调用和受控状态更新。
- `tests/ambbie_smoke.cpp`：搜索结果、筛选、联合约束和后验状态对齐检查。
- `tests/postppp_wtzr_integration.cpp`：记录产品输入下的处理与持久化分支集成检查。

## 验证范围

单元测试覆盖候选搜索、参考项约束、多组固定、筛选拒绝、奇异协方差和后验状态索引一致性。集成测试使用记录的 WTZR 产品覆盖持久化提交及输出收尾；测试中放宽的筛选门限只用于触发该分支，不代表实时默认值。
