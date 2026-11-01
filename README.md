# RT-PPP

## Headless realtime operation

See `HEADLESS_OPERATION.md` for cross-platform command-line startup using a
saved INI configuration.

RT-PPP 是一个基于 Qt Widgets 和 C++17 的实时 GNSS 精密单点定位应用。项目当前主线代码位于仓库根目录，提供实时数据源接入、RTCM3 解码、SSR 改正管理、PPP/PPP-AR 解算、观测状态展示，以及 SP3、RINEX OBS、ZTD 等结果文件输出。

本说明根据当前工作区代码和项目文件整理；本地参考源码树不属于当前 Qt 工程的直接编译单元。

## 主要功能

- 实时流接入：支持 NTRIP Caster、普通 TCP、MQTT 数据流。
- RTCM3 解码：解析 MSM 观测、广播星历、SSR 轨道/钟差/码偏差/相位偏差、高频钟差、VTEC 等消息。
- 数据准备：按数据源选择广播星历、轨道、钟差、码偏差、相位偏差和观测流，并生成改正后的中间状态。
- 实时 PPP：多系统 GPS/GLONASS/Galileo/BDS PPP 解算，支持电离层/对流层估计、CRD/ANTEX/BLQ/VTEC 等外部模型输入。
- PPP-AR：支持 MW + NL LAMBDA 和可切换的 BIE 两种固定后端；包含历史辅助、部分固定、约束注入与日志输出。
- 结果展示：流状态表、RTCM 统计、观测数量趋势、SNR 图、天空图、NEU 轨迹、ZTD 曲线和测站结果表。
- 文件输出：支持 SP3、CLK、RINEX OBS、ZTD 文本、按测站/UTC 日组织的紧凑 PPP JSON，以及 PPP-AR 诊断日志。

## 技术栈

- GUI 框架：Qt Widgets
- 网络模块：Qt Network
- 语言标准：C++17
- 构建入口：`RT-PPP.pro`
- 辅助脚本：Python 3，用于 PPP-AR 日志统计分析

## 现有说明文档

- `GREAT_legacy/FILE_USAGE_INDEX.md`：GREAT 参考源码树的文件用途索引，共 58 个文件，覆盖模糊度固定、偏差产品、PPP/PVT 滤波、姿态/潮汐/坐标转换等模块。

## 目录与文件结构

```text
.
|-- RT-PPP.pro                  # qmake 工程文件
|-- main.cpp                    # 应用入口、命令行模式、Qt 日志和退出处理
|-- mainwindow.*                # 主窗口、UI 绑定、数据源管理、结果展示和业务调度
|-- ppp_engine.*                # PPP/PPP-AR 核心解算状态与算法入口
|-- ppp_worker.*                # 后台 PPP 处理队列，避免阻塞 UI 线程
|-- pppjsonwriter.*             # 单测站单日紧凑 PPP JSON 输出、轮转与恢复
|-- telemetrywriter.*           # MSM/SSR/PPP 逐观测残差结构化遥测
|-- rtcmdecoder.*               # RTCM3 帧缓存、CRC、消息类型识别和内容解析
|-- streamclients.*             # NTRIP/TCP/MQTT 客户端
|-- gnss_ephemeris.*            # 广播星历模型与 SSR 轨道钟差改正应用
|-- gnss_signals.*              # GNSS 信号、频率与组合相关工具
|-- gnss_utils.*                # 时间系统、坐标、RSW/XYZ 等通用工具
|-- ambfix.*                    # PPP-AR 模糊度固定逻辑
|-- ambbie.*                    # BIE 分组搜索、筛选和约束生成
|-- sp3writer.*                 # SP3/CLK 输出
|-- rinexobswriter.*            # RINEX OBS 输出
|-- settingsstore.*             # 本地配置读写
|-- ui/                         # Qt Designer .ui 文件
|-- analyze_pppar_log.py        # PPP-AR 日志汇总脚本
|-- output/                     # 本地运行输出，已在 .gitignore 中忽略
|-- build/                      # 全部构建产物与生成文件，已忽略
|-- GREAT_legacy/               # GREAT 参考源码树，已忽略
`-- newmat(legacy)/             # newmat 参考源码树，已忽略
```

## 运行时数据流

1. 用户在界面中添加 NTRIP、TCP 或 MQTT 数据源。
2. `streamclients.*` 接收二进制数据帧，并把数据交给主窗口。
3. `Rtcm3Decoder` 按数据源维护解码状态，输出广播星历、SSR、MSM、VTEC 等结构化消息。
4. 主窗口更新状态表、观测统计和改正准备状态，并把 PPP 所需消息投递给 `PppWorker`。
5. `PppWorker` 在后台排队处理观测历元，调用 `PppEngine` 完成解算。
6. 解算结果返回 UI，用于 NEU/ZTD 曲线、测站表、日志和结果文件输出。

## 构建与运行

### Visual Studio

1. 使用 Visual Studio 2022 打开 `RT-PPP.sln`。
2. 在 Qt VS Tools 中为项目选择 `5.15.2_msvc2019_64`，平台选择 `x64`。
3. 执行“重新生成解决方案”。`ambbie.cpp` 已作为 `ClCompile` 项加入主项目，不需要手工添加源文件。

### Qt Creator

1. 使用 Qt Creator 打开 `RT-PPP.pro`。
2. 选择安装了 Qt Widgets 和 Qt Network 的 Kit。
3. 执行 qmake，然后构建并运行。

### 命令行

Windows/MinGW 示例：

```powershell
New-Item -ItemType Directory -Force build\mingw | Out-Null
Set-Location build\mingw
qmake ..\..\RT-PPP.pro
mingw32-make
```

Visual Studio 工程必须使用 MSVC 版 Qt；不要把 `msvc2019_64` Qt 与 MinGW 编译器混用。

Linux/macOS 示例：

```sh
mkdir -p build/linux
cd build/linux
qmake ../../RT-PPP.pro
make
```

Visual Studio 产物位于 `build/vs/debug` 或 `build/vs/release`；qmake 产物应放在 `build/<kit>` 下，并自动分为 `obj/`、`generated/`、`bin/`。构建生成的 `Makefile*`、对象文件、moc/uic 文件和 `.qmake.stash` 均不得写入项目根目录。

## 配置与输出

- 默认本地配置文件：`rtppp_settings.ini`
- Qt 调试日志：`build/logs/rtppp_qt_debug.log`
- SP3/CLK 输出：`output/sp3`
- RINEX OBS 输出：`output/rinex`
- ZTD 输出：`output/ztd`
- PPP JSON 输出：`output/ppp_json/<station>/ppp_<station>_YYYYMMDD.json`
- 结构化遥测：MSM、接收状态和 PPP residual 按类型/UTC 日写入 CSV；SSR 使用 ASCII `.ssr` 分块格式按 GPS 日写入，文件名为 `<mountpoint>_S_<YYYY><DDD>0000_01D_<MC|ION>.ssr`，例如 `SSRC00WHU0_S_20262820000_01D_MC.ssr`。为避免相同挂载点的多连接或不同 provider/solution 发生混写，目录按 `ssr/<mountpoint>/<source-id-hash>/p<provider>_s<solution>/` 隔离；常规改正写入 `_MC.ssr`，VTEC 写入 `_ION.ssr`。各类文件共用遥测开关和目录。
- 旧 PPP 输出示例：`output/ppp`

PPP JSON 的数组位置、AR 数据和运行期恢复规则见 `PPP_JSON_OUTPUT.md`。

这些文件通常是本机运行产物，已在 `.gitignore` 中忽略，不建议作为源码提交。

## PPP 相关输入

界面和 `PppOptions` 支持以下外部输入或模型配置：

- CRD 测站坐标文件
- ANTEX 天线相位中心文件
- BLQ 海潮负荷文件
- VTEC 电离层模型文件
- 流来源选择：广播星历、SSR 轨道、SSR 钟差、码偏差、相位偏差、观测流
- GNSS 系统选择：GPS、GLONASS、Galileo、BDS
- 组合与信号优先级配置
- PPP-AR 参数和日志文件路径

PPP 算法在 GUI 的 PPP 设置中通过 `AR algorithm` 下拉框选择。`LAMBDA (MW + NL)` 保持现有实现；`BIE` 使用相位模糊度分组、最小方差参考星、平方根 Kalman 参考条件化、联合候选搜索和 sigma 筛选。BIE 只处理以周表示的 `L1/L2` 模糊度状态，因此组合模式需选择 `Pi&Li` 或 `P1&L1`，不会把米单位的 `L3/AMB_IF` 状态误作整数。最少历元、最少模糊度数、最大 fractional part、最大 sigma 和参考条件化 sigma 可在同一设置区调整。`Persist BIE constraints to next epoch` 默认关闭，此时固定结果只用于当历元诊断；打开后，程序按 `系统|频点|卫星` 的连续相位弧确认 BIE 候选，并对达到组内最少稳定数量的子集使用 BIE 后验均值和协方差作矩匹配。参考星与无关可见星变化不会使连续卫星弧失效；相位周跳、粗差、锁定/不连续计数或 SSR phase-bias discontinuity 只清除受影响卫星的候选。若协方差缩减、U/ZTD 状态修正门限不通过，则不写回主状态，仅保留当历元副本输出。

算法设计与验证说明见 `BIE_ALGORITHM.md`；完整实时 PPP-AR 算法与运行说明见 `REALTIME_PPP_AR_TECHNICAL_SPECIFICATION.md`。

## 辅助脚本

`analyze_pppar_log.py` 用于汇总 PPP-AR 日志中的固定状态、系统统计、历史辅助、周跳重置和历元状态。

```powershell
python analyze_pppar_log.py path\to\pppar.log
```

## 开发注意事项

- 当前工程直接编译的源码以 `RT-PPP.pro` 中 `SOURCES`、`HEADERS`、`FORMS` 为准。
- `ui_*.h` 是由 Qt uic 生成的文件，根目录和 `ui/` 下均可能出现生成副本；修改界面时应优先编辑 `ui/*.ui` 或 `mainwindow.*`。
- `mainwindow.cpp` 体量较大，新增功能时建议尽量把可独立测试或复用的逻辑放到专门模块。
- `PppWorker` 负责后台处理，UI 线程与解算线程之间的状态同步需要继续保持信号/槽和队列边界清晰。
- 仓库当前包含未提交/未跟踪的源码与生成文件，提交前建议先用 `git status --short` 核对需要纳入版本控制的文件范围。
