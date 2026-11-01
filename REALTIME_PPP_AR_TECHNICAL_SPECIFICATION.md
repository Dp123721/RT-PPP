# RT-PPP 实时 PPP-AR 技术说明

## 1. 文档定位

本文档描述仓库根目录当前 C++/Qt 主程序的实时 PPP 和 PPP-AR 实现。它是实现说明，不是一般 PPP 教材，也不承诺代码中没有实现的产品、模型或质量控制能力。

主要实现位置：

- 实时调度：`pppruntimeservice.*`、`ppp_worker.*`
- PPP 核心：`ppp_engine.*`
- LAMBDA PPP-AR：`ambfix.*`
- BIE PPP-AR：`ambbie.*`
- RTCM 解码和预处理：`rtcmdecoder.*`、`rtcmprocessingservice.*`
- 输出：`outputworker.*`、`pppjsonwriter.*`
- GUI 与 `QSettings`：`mainwindow.*`

本地参考源码树和 `research/` 不进入主程序构建或运行时依赖。实时算法只依赖当前工程中的服务、状态和产品接口。

## 2. 运行结构与数据所有权

```text
NTRIP / TCP / MQTT 字节流
  -> StreamClient
  -> RtcmProcessingService  (RTCM 线程)
  -> MainWindow 编排
  -> PppRuntimeService / PppWorker  (PPP 线程)
  -> PppEngine
  -> SolutionEvent
  -> MainWindow 显示快照
  -> OutputWorker  (输出线程)
```

运行期算法状态不由 GUI 保存。`PppWorker` 独占 `PppEngine`，因此星历、SSR、相位偏差、滤波状态、周跳状态和 AR 历史都只在 PPP 线程内访问。GUI 通过 `PppRuntimeService` 将配置、广播星历、SSR、VTEC 和 MSM 事件排入该线程；结果再异步返回 GUI。

`OutputWorker` 是 SP3/CLK、RINEX、PPP JSON、ZTD、telemetry 和 PPP-AR 日志的唯一运行期文件写入者。它使用有界队列串行写入，避免 PPP 线程因文件 I/O 阻塞，也避免多个线程同时追加同一 JSON/ZTD 文件。

## 3. 时间系统与实时历元调度

### 3.1 内部时间基准

PPP 内部时间是 GPS week/SOW。UTC 仅用于日志、用户显示和文件时间标签。`GnssUtils` 提供 MSM、SSR、UTC 和 GPS 时间之间的统一转换。

- GPS、Galileo 的 RTCM 历元按 GPS SOW 处理。
- GLONASS 的日内时间结合当前 GPS 周定位。
- BeiDou RTCM 时间转换到 GPS 时间时应用系统时间差；当前实现按系统时间偏移将 BDS 时间增加 14 秒。
- GPS 到 UTC 的展示转换扣除闰秒。

这些规则意味着滤波器、SSR 匹配、采样网格和输出历元均以 GPS 时间对齐，不能用计算机本地时钟替代。

### 3.2 采样网格

`PppWorker` 使用绝对 GPS 时间网格筛选 MSM 历元。若处理间隔为 5 秒，仅 SOW 尾数为 `...00`、`...05`、`...10` 的完整 MSM 历元进入 PPP。原始 RTCM、广播星历和 SSR 仍连续接收和解析，以保持产品状态最新。

因此，采样设置不是“每 5 秒挑一个刚到的数据包”，而是“仅处理落在 GPS 网格上的观测历元”。RTCM 预处理层与 PPP worker 都执行相同的网格检查，防止一个层按到达时间、另一个层按观测时间造成错位。

### 3.3 多系统 MSM 合并

`PppEngine::processMsm()` 维护按数据源和系统分开的待处理历元。各系统相同 GPS 历元的 MSM 会先合并；当更晚历元到达时，前一历元被送入标准化观测处理。这样 GPS、GLONASS、Galileo、BeiDou 不会在同一滤波更新中混用相邻但不同的观测时刻。

## 4. 输入、产品与卫星状态

### 4.1 必需实时输入

实时 PPP 至少需要：

1. 一个观测 MSM 数据源；
2. 一个广播星历数据源；
3. 如配置 SSR 模式，则同时配置轨道和钟差源。

若只选择轨道或只选择钟差，`processMsm()` 直接拒绝该历元。SSR 码偏差和相位偏差是可选输入，但 PPP-AR 所需频点没有可用整数相位偏差时，该卫星不会进入 AR；浮点 PPP 仍可继续。

### 4.2 广播星历与 SSR

广播星历按卫星和来源保存。`computeSatState()` 在信号发射时刻计算广播位置、速度和钟差，并处理地球自转。SSR 模式在此基础上应用轨道和钟差改正。产品选择受下列条件约束：

- 卫星系统与 PRN 必须匹配；
- IOD 必须兼容；
- SSR 更新间隔和有效期必须满足；
- 轨道与钟差必须成对可用；
- 码/相位偏差按信号标识匹配，不能仅按“同一颗卫星”替代。

不满足条件的卫星在进入滤波前被跳过，并记录广播匹配、SSR 匹配、IOD 拒绝和残差拒绝等阶段诊断。

### 4.3 精密轨道接入

实时程序也可接收外部提供的精密卫星状态。`PppPreciseSatelliteState` 含 GPS 时刻、位置、速度和钟差；当精密状态在当前历元可用时，核心使用它代替广播加 SSR 的轨道/钟差状态。此处的精密状态插值和文件读取属于上游适配器或事后模块；`PppEngine` 只消费标准化卫星状态。

### 4.4 SSR 相位偏差与 yaw

相位偏差输入包含信号偏差、整数指示、宽巷指示、连续性/锁定信息和必要的 yaw 信息。核心会：

- 依据观测的实际信号 token 匹配偏差；
- 将偏差保持为米，并在形成观测方程或 MW 组合时转换到相应单位；
- 对 BIE 检查所用频点是否具备整数相位偏差；
- 在 SSR 提供 yaw 时供载波相位 wind-up 模型使用。

“有相位偏差消息”不等于“该卫星、该频点、该历元的整数相位偏差可用”。日志中的 `phaseState`、匹配信号列表和来源字段用于区分这两种情况。

## 5. 标准化观测与信号选择

实时 MSM 先转换为 `PppStandardizedObservation`：双频码观测、双频相位观测、频率、波长、信号 ID、GLONASS 频率槽、LLI/锁定/不连续计数、偏差以及 yaw 信息。值的单位约定如下：

- `C1/C2`：米；
- 原始 `L1/L2`：周；
- 进入相位观测方程的 `L1*lambda1`、`L2*lambda2`：米；
- `AMB_L1/AMB_L2` 状态：周；
- 电离层和湿对流层残差状态：米。

信号优先级由 `signalPriorities` 控制。每个系统最多选定两条工作频带；无法同时构成所选观测组合的卫星不参与相应方程。GLONASS 使用频率槽计算实际频率和波长，不能套用 GPS 固定频率。

## 6. 观测组合

每个系统可独立选择下列组合模式：

| 模式 | 加入滤波器的观测 |
| --- | --- |
| `Pi&Li` 或 `L1L2` | `C1`、`C2`、`L1`、`L2`，可选 GIM 伪观测 |
| `Pi` | `C1`、`C2`，可选 GIM |
| `P1&L1` | `C1`、`L1`，可选 GIM |
| `P1` | `C1`，可选 GIM |
| `P3` | 无电离层码组合 `C_IF` |
| `L3` | 无电离层相位组合 `L_IF` |
| 其他/默认 | `C_IF` 与 `L_IF` |

无电离层组合使用代码中计算的系数 `lkA`、`lkB`：

\[
P_{IF}=aP_1+bP_2,\qquad \Phi_{IF}=a\Phi_1+b\Phi_2
\]

其中系数由两频率计算。`AMB_IF` 的单位是米，因而不参与当前 BIE 整数搜索；BIE 要求采用 `Pi&Li` 或 `P1&L1`，使 `AMB_L1/AMB_L2` 保持周单位。LAMBDA PPP-AR 也要求启用系统使用 `Pi&Li`。

## 7. 状态向量与随机模型

状态由 `PppFilter::Param` 动态维护。实际存在的分量取决于启用系统、观测组合和当前可见卫星：

\[
x=[dX,dY,dZ,dt_G,dt_R,dt_E,dt_C,T_w,I_1,\ldots,N_1,\ldots,N_2,\ldots,B^C,B^L]^T
\]

含义如下：

- `CRD_X/Y/Z`：相对 `_baseXyz` 的 ECEF 修正，米；
- `CLK_G/R/E/C`：每个系统接收机钟差，米；
- `TROPO`：Saastamoinen 天顶延迟之外的湿延迟残差，米；
- `IONO_L1`：逐卫星 L1 标度的斜路径电离层状态，米；
- `AMB_L1/L2`：逐卫星、逐频点模糊度，周；
- `AMB_IF`：无电离层相位模糊度，米；
- `CBIAS_*`、`PBIAS_*`：系统/频点偏差状态，在相应组合和模型路径需要时加入。

预测步骤依据时间间隔和 PPP 设置给状态加入过程噪声。坐标、接收机钟差、湿延迟、模糊度等初始方差与随机游走参数均来自 PPP Settings。状态重建会保持仍有效的状态及其协方差；新卫星或新相位弧的模糊度以观测模型给出的整数附近种子初始化。

## 8. 观测方程和改正模型

### 8.1 未组合双频方程

对频点 \(j\)，代码中的线性化模型可概括为：

\[
P_j=\rho+c\delta t_r-c\delta t_s+m_wT_w+
I_j+D^P_j+M_j+\epsilon_P
\]

\[
\lambda_jL_j=\rho+c\delta t_r-c\delta t_s+m_wT_w-
I_j+
\lambda_jN_j+D^L_j+M_j+W_j+\epsilon_L
\]

其中电离层系数为

\[
\mu_j=\frac{f_{L1}^2}{f_j^2}
\]

所以码观测使用 \(+\mu_j I\)，相位观测使用 \(-\mu_j I\)。无电离层组合的电离层系数为零。GIM 伪观测直接约束 `IONO_L1`，仅在启用 GIM 且并非无电离层模式时使用。

### 8.2 几何与非色散项

`cmpModel()` 和 `cmpValue()` 形成每颗卫星的计算观测值。当前实现包含：

- 发射时刻卫星位置、速度和卫星钟；
- 几何距离；
- 地球自转 Sagnac 项；
- 接收机钟初值（Bancroft）；
- Saastamoinen 对流层斜路径延迟；
- 固体潮位移；
- 可选 BLQ 海潮负荷位移；
- Shapiro 相对论项；
- 接收机 NEU 天线偏心；
- ANTEX 卫星/接收机天线改正；
- 载波相位 wind-up；
- SSR 码偏差和相位偏差。

卫星高度角、方位角和几何距离由当前基准接收机坐标计算。低于 `minEleRad` 的卫星在模型阶段移除。

### 8.3 对流层与 ZTD

`delay_saast(ele)` 使用站高相关的压强、温度、湿度近似和高度分段系数，得到 Saastamoinen 斜路径延迟。`TROPO` 状态的设计系数为

\[
m_w=\frac{1}{\sin(e)}
\]

输出中：

\[
ZTD=ZTD_{Saastamoinen}(e=90^\circ)+T_w
\]

`trp` 是估计的完整 ZWD，`zhdModel`/`ztdModel` 是天顶 Saastamoinen ZHD，`ztdStdev` 当前沿用 `TROPO` 状态标准差。JSON compact v1 中对应字段名为 `zwdM`。

### 8.4 电离层/VTEC

VTEC 文件或实时 VTEC 产品被转换为多层球壳模型。满足“启用 VTEC，且选择伪观测电离层或 PPP-RTK，且未使用无电离层组合”时，模型计算 L1 斜路径电离层延迟。PPP-RTK 分支会把模型延迟加入观测计算值；普通未组合 PPP 可将其作为 GIM 伪观测信息。

### 8.5 天线与 wind-up

若 ANTEX 已加载，按系统、频带和卫星姿态计算卫星天线改正，并可叠加接收机天线改正。`isApc` 影响卫星参考频点处理。缺失的卫星/接收机天线条目记录诊断，但不会虚构改正值。wind-up 使用卫星位置、速度、太阳方向和接收机几何；SSR yaw 可参与卫星姿态方向的计算。

## 9. 初始化、滤波更新和质量控制

### 9.1 Bancroft 初始化

每个 PPP 历元首先进行两轮 Bancroft：第一轮用当前码观测估计接收机位置和钟差，第二轮在已有模型基础上复算。每轮都可剔除码残差超过 1500 m 的卫星。若最终可用卫星少于 `minObs`、解算坐标不合理或模型建立失败，历元拒绝。

若设置了先验 ECEF，则以其作为基准；否则以 Bancroft 坐标初始化 `_baseXyz`。PPP 状态中的 `CRD_X/Y/Z` 代表对该基准的修正。

### 9.2 Kalman 过程

在每个历元：

1. 建立/预测状态和协方差；
2. 先处理码观测；若存在相位，再处理相位观测；
3. 按 GPS、GLONASS、Galileo、BeiDou 顺序分系统更新；
4. 每个系统保留更新前副本，发现超限残差时回滚该系统并剔除最大粗差；
5. 通过平方根 QR Kalman 更新计算状态和协方差；
6. 全部系统成功后进入 PPP-AR 和 DOP 计算。

测量标准差由 `sigmaC1/sigmaL1` 给出。GLONASS 使用 5 倍权重因子、BeiDou使用 2 倍权重因子。若启用高度角加权，标准差再乘：

\[
1+4\times10^{-6}(90^\circ-e)^3
\]

码和相位粗差门限分别为 `maxResC1` 和 `maxResL1`；无电离层组合用传播后的组合门限。相位粗差会重置受影响模糊度状态和其协方差行/列。

### 9.3 周跳与弧段

周跳检测使用观测中的 slip 标志、锁定时间倒退、不连续计数变化和历元间隔。发生周跳或相位粗差时：

- 相关 `AMB_L1`、`AMB_L2` 或 `AMB_IF` 重置；
- 重置模糊度种子、历元数和协方差；
- 清除该卫星的 MW/AR 历史；
- 清除该卫星两个频点上与当前相位弧绑定的 BIE 持久候选；其他卫星和频点的候选不受影响。

这保证不会把旧相位弧的固定信息施加到新相位弧。

## 10. PPP-AR 共用前提

启用 PPP-AR 后，核心仍先完成完整浮点 PPP 更新。AR 是其后的附加状态处理，不替代观测质量控制。

共同条件包括：

- 系统已启用且 AR 系统开关允许；
- 卫星未被标为粗差；
- 高度角、弧段长度和观测组合满足算法要求；
- 对需要相位偏差的路径，SSR 整数相位偏差必须按卫星/信号匹配；
- 固定结果、候选数、比率、组信息和固定卫星集写入 `PppSolution` 与 AR 日志。

`arStatus` 的含义取决于最终可用固定约束数量；不应把“收到相位偏差”或“BIE 搜索成功”单独解释为可靠固定。

## 11. LAMBDA (MW + NL) 分支

### 11.1 使用范围

GUI 的 `AR algorithm=LAMBDA` 对应 `ambfix.*`。此分支要求已启用系统采用 `Pi&Li`，并使用双频观测的 MW 宽巷和窄巷整数固定流程。历史辅助仅用于该分支，不用于 BIE 分支。

### 11.2 宽巷与窄巷

每颗卫星维护 Melbourne-Wubbena 统计量。代码在应用码/相位偏差之后形成 MW 周值，维护均值、样本方差和有效样本数。满足 `arMinMwSamples`、高度角、锁定和 MW 偏差/标准差门限的卫星进入宽巷候选。

以每个系统的参考卫星形成双差，随后构造窄巷浮点量和协方差。`ambfix.cpp` 使用 LAMBDA 搜索多个整数候选，并以：

- `arNlRatio`：候选距离比率；
- `arNlBoot`：bootstrap 门限；
- `arNlMaxDev`：浮点到固定窄巷的偏差；
- `arMinFixPairs`：最少双差对数；

筛选固定集。若部分双差不合格，代码允许在规定条件下生成 partial fix，而不是将所有卫星强行固定。

### 11.3 LAMBDA 约束写回

LAMBDA 约束是双差线性关系，设计矩阵包含卫星和参考卫星对应的模糊度状态系数。`applyConstraint()` 以设置的 `arConstraintSigma` 作为伪观测标准差进行平方根 Kalman 更新，并记录更新前后残差、方差范围和最大状态改变量。固定签名、参考星和历史状态按系统维护，日志标记 `fresh_fix`、`hold`、`reacquire` 或 `lost`。

## 12. BIE 分支

### 12.1 使用范围和默认模式

`AR algorithm=BIE` 只处理以周表示的 `AMB_L1/AMB_L2`。它要求 `Pi&Li` 或 `P1&L1`，且每个用于 AR 的频点都必须匹配整数相位偏差。

默认 `arBiePersistConstraints=false`。此时复制浮点状态 `xFlt/QFlt`，在副本上生成当前历元 BIE 固定结果和日志，然后恢复主滤波器浮点状态。GUI、PPP JSON、ZTD 和下一历元预测不消费该固定副本。

### 12.2 分组、参考模糊度和联合搜索

`BieAlgorithm::run()` 的步骤为：

1. 收集满足最少历元、最小高度角和整数相位偏差条件的 `AMB_L1/L2`；
2. 按 `系统|L1`、`系统|L2` 分组；
3. 每组选择双差方差和最小的参考模糊度；
4. 用平方根 Kalman 参考约束把每个参考模糊度条件到其最近整数；
5. 对全部活动组的模糊度和协方差作一次联合 decorrelation/LAMBDA 候选搜索；
6. 对候选整数向量按归一化权重求 BIE 均值和候选离散协方差；
7. 对每个模糊度以 BIE fractional part 与 BIE sigma 筛选；
8. 至少一组达到最少可固定数量后，输出所有逐项可固定模糊度（包括参考项）。

候选权重为：

\[
w_i=\frac{\exp[-\tfrac12(d_i-d_1)]}
{\sum_j\exp[-\tfrac12(d_j-d_1)]}
\]

其中 \(d_i\) 是 decorrelated 整数候选的二次型距离。BIE 均值和候选离散协方差为：

\[
\hat a_{BIE}=\sum_iw_i n_i
\]

\[
Q_{BIE}=Z^{-T}\left(\sum_iw_i(z_i-\bar z)(z_i-\bar z)^T\right)Z^{-1}
\]

实现中的候选协方差以 decorrelated 空间对角离散方差形成，再转换回原模糊度空间。`effectiveCandidates=1/\sum_iw_i^2` 只描述候选权重集中程度，不是独立的完整性检验。

### 12.3 BIE 当历元固定

在默认模式下，BIE 约束以 `arBieConstraintSigma` 应用于状态副本。该参数既用于参考项条件化，也用于当历元固定副本的伪观测标准差。固定副本仅供 `BIE FIXED_SOLUTION` 与 AR 诊断输出使用。

### 12.4 可选 BIE 持久化：当前实现

打开 `Persist BIE constraints to next epoch` 后，程序不再把每历元 BIE 均值以固定 `1e-4 cycle` 硬写入主滤波器。它执行以下受控流程：

1. `BieAlgorithm::Result` 返回已筛选状态索引、BIE 后验均值和对应完整子协方差；BIE 均值本身可以是非整数，程序不会把它先取整再伪装成零方差观测；
2. 对每个 `系统|频点|卫星` 建立独立的相位弧假设，身份是该卫星 BIE 均值的最近整数候选。相邻 BIE 历元保持同一候选才累计确认；卫星没有获得 BIE 约束、出现历元间断或候选改变时，下一次必须从一次确认重新开始；
3. 一个 `系统|频点` 组中至少 `arBieMinSat-1` 个卫星弧达到 `arBiePersistConfirmEpochs`（默认 3）后，该稳定子集才允许写回。可见星集合变化或 BIE 的参考星重选不会清空无关卫星的确认；当前可用的稳定子集仍使用本历元联合 BIE 后验的对应边缘均值和子协方差；
4. 周跳、相位粗差、锁定时间倒退、SSR phase-bias discontinuity 或观测弧重置只清除受影响卫星的候选。主滤波器中已经完成的贝叶斯更新不会在没有新观测的情况下重复注入旧约束；
5. 对全部已就绪组的稳定子集执行一次 BIE 后验矩匹配；
6. 若本次状态更新的绝对 U 修正超过 `arBiePersistMaxVerticalM`（默认 0.10 m），或 ZTD 修正超过 `arBiePersistMaxZtdM`（默认 0.03 m），拒绝主状态提交，并回退为当历元 BIE 副本输出；
7. 任一矩阵维度、状态索引、有限性，或协方差缩减 `Qaa-QBIE` 不具半正定性时，也只保留当历元输出。若理论上半正定的更新因不同量级状态相减而无法 Cholesky 分解，程序最多以全状态最大方差的 `1e-7` 进行对角加载；该加载只增大协方差、不会增加固定信息，仍不能分解则拒绝。

设当前浮点状态和协方差为 \(x_f,P_f\)，本批 BIE 选择的模糊度为 \(a=Sx\)，浮点子协方差为 \(Q_{aa}=SP_fS^T\)，交叉协方差为 \(P_{xa}=P_fS^T\)。BIE 返回 \(\hat a_{BIE},Q_{BIE}\) 后，程序用：

\[
x_{BIE}=x_f+P_{xa}Q_{aa}^{-1}(\hat a_{BIE}-a_f)
\]

\[
P_{BIE}=P_f-P_{xa}Q_{aa}^{-1}(Q_{aa}-Q_{BIE})Q_{aa}^{-1}P_{ax}
\]

更新完整状态。该形式保留 BIE 候选离散性，不把非整数的 BIE 后验均值伪装成零方差整数观测。

持久化不是全局、不可撤销的整数锁定。它是“与连续相位弧绑定的、带后验协方差的可撤销假设”：浮点模糊度状态始终跨历元递推；只有通过确认和转换门限的 BIE 后验才改变该递推先验。参考星重选只影响本历元的 BIE 条件化，不会自动释放其他连续卫星弧。它与传统静态站 LAMBDA 的永久保持策略不同，也保留了 BIE 面向实时多候选估计的特性。

### 12.5 BIE 日志解释

典型 BIE 日志字段：

- `BIE search`：活动模糊度数、组数、候选数、前两候选距离、第二权重、有效候选数；
- `BIE group`：系统/频点、参考星、组大小、逐项可固定数、最大 fractional part 与 sigma；
- `BIE constraintBatch`：约束数及状态，可能为 `epoch_local`、`epoch_local_pending`、`committed_posterior` 或 `epoch_local_rejected`；
- `BIE PERSISTENCE group`：每个系统/频点组的 active/stable 数、确认要求和 ready/pending 状态；
- `BIE PERSISTENCE status`：稳定弧子集的矩匹配维度、`dU`、`dZTD`、最大状态修正、`covarianceLoading`、保持原因或拒绝原因；
- `BIE CONSTRAINT update=epoch_local`：历元副本上的伪观测 Kalman 残差；`update=posterior`：该项由联合矩匹配提交，逐项 Kalman 残差不适用，字段记为 `-`；
- `BIE FIXED_SOLUTION`：当历元 BIE 副本或已提交后验状态对应的坐标/ZTD。

`BIE FIXED_SOLUTION` 在默认历元副本模式下不代表 GUI/JSON 使用的主状态。必须结合 `constraintBatch state` 判断它是否已持久化。

## 13. 配置项

PPP Settings 中与 BIE 直接相关的配置及其 `QSettings` 键为：

| UI 含义 | 键 | 默认值 |
| --- | --- | --- |
| AR algorithm | `ppp/arAlgorithm` | `LAMBDA` |
| Enable PPP-AR | `ppp/arEnable` | `false` |
| BIE minimum epochs | `ppp/arBieMinEpochs` | 10 |
| BIE minimum ambiguities | `ppp/arBieMinSat` | 4 |
| BIE max fractional part | `ppp/arBieMaxFrac` | 0.01 |
| BIE max sigma | `ppp/arBieMaxSig` | 0.001 cycle |
| BIE constraint sigma | `ppp/arBieConstraintSigma` | `1e-4` cycle |
| Persist BIE constraints | `ppp/arBiePersistConstraints` | `false` |
| BIE persist confirmation epochs | `ppp/arBiePersistConfirmEpochs` | 3 |
| BIE persist max U correction | `ppp/arBiePersistMaxVerticalM` | 0.10 m |
| BIE persist max ZTD correction | `ppp/arBiePersistMaxZtdM` | 0.03 m |

当未启用 PPP-AR、未选择 BIE，或未勾选持久化时，后 3 个持久化门限控件在 GUI 中禁用。运行中的任务应使用初始化时刻的完整 PPP 配置；配置改变会通过 runtime service 有序发送到 PPP 线程。

## 14. 结果、JSON、ZTD 与诊断

`PppSolution` 输出：GPS/UTC 时刻、ECEF、NEU、ECEF 协方差、湿延迟残差、ZTD 与标准差、可用卫星数、HDOP、AR 状态、AR 算法、固定卫星、ratio/bootstrap、BIE 候选统计、已应用约束数、分组固定信息和逐观测残差。

PPP JSON compact v1 的 `enuM` 顺序固定为 `[N,E,U]`，不是 `[E,N,U]`。结果文件由 `PppJsonWriter` 以 `meta + solutions` 结构输出；停止流程负责封闭 JSON 数组和对象。ZTD 文件写入同一 `OutputWorker` 管理的输出链路。

定位问题时建议按以下顺序检查：

1. `PPP solution`：卫星数、HDOP、坐标、ZTD、码/相位预拟合与后拟合残差；
2. 阶段诊断：观测、广播、SSR、IOD、残差各阶段是否淘汰卫星；
3. `PPP-AR`：算法、可用卫星、相位偏差匹配、LAMBDA/BIE 具体拒绝原因；
4. `BIE constraintBatch` 与 `BIE PERSISTENCE`：区分当历元副本、等待确认、正式提交和门限拒绝；
5. JSON 的 `arData`、`satelliteSummary` 和 `troposphere`：与图形时刻逐点对齐。

## 15. 已实现边界与正确使用方式

- PPP-AR 不能纠正错误的卫星轨道、钟差、码偏差、相位偏差或天线模型；AR 只在浮点模型已通过残差检验后执行。
- BIE 的候选权重集中不是独立完整性证明；持久化还需要逐弧连续确认、协方差缩减、U/ZTD 转换门限和周跳/SSR 连续性检查。
- BIE 默认模式的固定结果是当历元副本，不能据此声称主滤波器已经连续固定。
- 参考星变化本身不释放连续卫星弧；新进入或整数候选改变的卫星仍须独立确认，且只有满足最小稳定数量的子集可以写回。
- 当前持久化门限只检查 BIE 提议的状态转换和协方差契约；它不替代外部完整性监测、参考站约束或长期统计质量评估。
- 观测缺失本身不能证明网络丢包、SSR 拒绝或算法失败；应使用阶段诊断和产品时效日志判断。

## 16. 验证覆盖

当前仓库的相关验证包括：

- `tests/ambbie_smoke.cpp`：Lambda/BIE 参考向量、多组 BIE、筛选拒绝、奇异协方差，以及 BIE 后验均值/协方差与约束状态索引的一致性；
- `tests/postppp_smoke.cpp`：共享 PPP 核心的事后适配器烟雾测试；
- `tests/postppp_wtzr_integration.cpp`：使用 `output/data` 中的 WTZR OBS/NAV/SSR 实际产品，以 `Pi&Li`、两历元逐弧确认运行到第一次 BIE 后验提交，再取消任务并检查 JSON/manifest 收尾。该夹具仅为覆盖提交分支而关闭 fractional/sigma 与 U/ZTD 转换门限；这些设置不是实时默认值；
- `tests/architecture_smoke.ps1`：线程/输出边界、BIE 默认当历元模式、后验矩匹配、批次确认、转换门限、弧段清除和 GUI 配置持久化契约；
- `tests/run_smoke.ps1`：Visual Studio 2022 + Qt 5.15.2 MSVC 构建、各 C++ 烟雾测试、架构检查和可用时的无界面生命周期测试。

这些测试证明接口、数学数据契约和构建链路；它们不等同于对所有实时网络、所有 SSR 产品或所有站点的长期精度验收。生产部署仍应使用已知坐标站、完整产品记录和独立 ZTD/坐标参考进行回放评估。
