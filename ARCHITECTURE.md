# RT-PPP 架构与运行链路

## 范围

当前主程序是仓库根目录下的 Qt Widgets/C++17 工程，正式构建入口为
`RT-PPP.sln`（Visual Studio 2022）和 `RT-PPP.pro`（qmake）。

- 本地参考源码树只用于算法和行为对照，不作为运行时依赖。
- `research/` 是独立的科研绘图与分析项目，不参与主程序构建、重构和 smoke test。
- `SettingsStore` 负责配置持久化；算法输入文件（CRD、ANTEX、BLQ、VTEC）保持只读。

## 模块边界

| 层 | 主要模块 | 职责 |
| --- | --- | --- |
| UI/编排 | `MainWindow` | 控件绑定、用户命令、状态快照展示、数据准备策略和结果绘图 |
| 数据接入 | `streamclients.*` | NTRIP、TCP、MQTT 连接、重连和字节流接收 |
| RTCM 处理 | `RtcmProcessingService`, `rtcmdecoder.*` | 独立线程解码、按源维护帧缓冲、MSM 采样、分片合并和状态摘要 |
| PPP runtime | `PppRuntimeService`, `PppWorker` | 独占 PPP 线程，统一配置、启停、清空、消息排队和结果回传 |
| PPP 算法 | `PppEngine`, `ambfix.*`, `ambbie.*` | 星历/SSR/观测建模、滤波、PPP-AR 和解算诊断，不写运行结果文件 |
| GNSS 公共服务 | `gnss_utils.*`, `gnss_signals.*`, `gnss_ephemeris.*` | 时间、信号、坐标、星历和改正计算 |
| 输出 | `OutputWorker` | 单一有界异步队列，串行写 SP3/CLK、RINEX、PPP JSON、ZTD、telemetry 和日志 |

## 线程模型

```text
GUI thread
  StreamClient dataFrame
    -> RtcmProcessingService queue

RTCM thread
  frame buffering -> CRC -> decode -> MSM sampling/merge/status snapshot
    -> GUI result queue

GUI thread
  apply decoded state -> update views
  broadcast/SSR/MSM/VTEC -> PppRuntimeService

PPP thread
  PppWorker queue -> PppEngine -> SolutionEvent
    -> GUI result queue

GUI thread
  update result views -> OutputWorker bounded queue

Output thread
  serialize all runtime result files
```

`OutputWorker::enqueue*()` 可以跨线程调用，数据先进入同一个受互斥锁保护的有界队列，
再由输出线程 drain。配置更新也是队列任务，因此配置与后续数据的先后关系确定。
Qt 框架诊断写标准错误流，不再绕过输出边界直接创建日志文件。

## GNSS 时间规则

统一入口位于 `GnssUtils::GpsTime` 及以下函数：

- `currentGpsTime()`：UTC 到 GPS week/SOW。
- `gpsTimeFromMsm()`：RTCM MSM 历元转换。
- `gpsTimeFromSsr()`：SSR 历元转换。
- `normalizeGpsTime()`：周界归一化并生成 UTC。
- `gpsTimeDifferenceSeconds()`：跨周时间差。

规则按 GNSS 时间系统定义和产品时间标记处理：GPS/Galileo 使用 GPS SOW，
GLONASS 按当前 GPS 周定位日内毫秒，BDS 转 GPS 时增加 14 秒。GPS 到 UTC 的逆转换会扣除闰秒，
与 UTC 到 GPS 的转换保持对称。

## 生命周期

### Start All

1. 将 stream selection、runtime policy 和 PPP options 同步配置到 PPP 线程。
2. 清空旧 filter/队列状态并启动 PPP runtime。
3. 启动各网络数据源。

### Stop All

1. 立即设置 PPP stop 标志，阻止长计算继续发布结果。
2. 停止网络数据源。
3. 同步停止并清空 PPP runtime。
4. 同步清空 RTCM decoder、采样和摘要状态。
5. 清空 UI 的运行快照和历史图数据。

### Reload

Reload 先完整执行 Stop All，再销毁旧 client，加载并应用新配置。重载完成后保持
Stopped，不隐式重启数据源。

### Shutdown

关闭输入后，依次停止 RTCM 服务、PPP runtime，最后关闭 `OutputWorker`。
输出线程在退出前排空已接受任务并关闭所有 writer，避免 PPP 结果晚于输出线程退出。

## 当前约束

`MainWindow` 仍负责“改正采用/数据准备状态”到 UI 视图模型的映射，这是后续继续拆分
展示层时的候选边界。新增算法、协议或文件格式不应继续直接加入 `MainWindow`；优先放入
对应 service、engine 或 writer，并通过快照/任务接口连接。

## 当前后台线程处理边界

`RtcmProcessingService` 独占所有可变 RTCM 数据准备状态，包括广播星历、按数据源隔离的
SSR 轨道/钟差/码偏差/相位偏差、GLONASS 频道、数据准备快照、改正后 RINEX 观测值、
改正后 SP3 记录以及卫星方位角/高度角计算。`MainWindow` 只提供最新接收机 ECEF 坐标，
并消费不可变结果用于显示、转发 PPP 和提交 `OutputWorker`。

`MainWindow`、`PppWorker` 和 `PppEngine` 均不写运行结果文件。`tests/architecture_smoke.ps1`
持续检查线程、时间、输出边界以及 `research/`、legacy 目录不进入主程序构建清单。

“Processing epoch interval” 使用绝对 GPS 时间网格。例如选择 5 秒时，仅
GPS SOW 为 `...00、...05、...10` 的 MSM 历元进入状态整理、改正观测、RINEX、PPP
计算和 PPP 输出。原始 RTCM 字节、广播星历和 SSR 仍连续接收与解码，以保持改正状态
最新；`RtcmProcessingService` 和 `PppWorker` 分别执行同一网格校验。
