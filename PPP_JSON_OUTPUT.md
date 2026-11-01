# PPP 紧凑 JSON 输出

## 文件组织

GUI 的 `PPP Outputs` 区域可以启用输出并设置根目录，默认目录为：

```text
output/ppp_json
```

每个测站每天生成一个独立文件：

```text
output/ppp_json/<station>/ppp_<station>_YYYYMMDD.json
```

例如：

```text
output/ppp_json/BJFS/ppp_BJFS_20260727.json
output/ppp_json/WHU2/ppp_WHU2_20260727.json
```

测站名优先采用 CRD 匹配结果，否则采用观测流挂载点。文件名中的非法字符会被替换，并附加短哈希以避免名称冲突；原始测站名保留在 `meta.station.id`。

## 顶层结构

```json
{
  "meta": {
    "schema": "rtppp.compact",
    "version": 1,
    "station": {},
    "timeBase": {},
    "algorithms": ["NONE", "LAMBDA", "BIE"],
    "arStatus": ["disabled", "float", "partial_fix", "fixed"],
    "systems": ["?", "G", "R", "E", "C", "J", "S", "I"],
    "ambiguityGroups": ["", "L1", "L2", "NL", "WL"],
    "satelliteIdEncoding": {},
    "layout": {}
  },
  "solutions": []
}
```

`meta.timeBase.epoch` 是当天 UTC 00:00:00。每个结果只保存相对午夜的毫秒偏移，完整 UTC 可由 `epoch + timeOffsetMs` 恢复。GPS 周和周内秒基准同时保存在 `timeBase` 中。

## 解数组

`solutions` 中每个元素依次为：

```text
[
  timeOffsetMs,
  algorithm,
  arStatus,
  [x, y, z],
  [n, e, u],
  [qxx, qxy, qxz, qyy, qyz, qzz],
  [ztdM, zwdM, ztdStdM],
  [usedSatelliteCount, arEligibleCount, fixedCount, hdop],
  [usedSatelliteIds, arEligibleSatelliteIds, fixedSatelliteIds],
  arData,
  flags
]
```

数组位置也完整写入 `meta.layout`，读取程序应以文件内 layout 和 version 为准。ECEF/ENU/ZTD 单位为米，ECEF 协方差单位为平方米。

当前写入器固定输出严格的 `rtppp.compact` v1。为兼容该版本契约，布局字段名仍为 `enuM`，实际数组顺序固定为内部 `[N, E, U]`，且 metadata 不包含 `enuOrder`。科研分析会独立从 ECEF 和外部参考坐标重算 NEU，并输出“文件内 NEU - 独立重算 NEU”用于检查顺序或参考点错误。

卫星使用整数压缩：GPS、GLONASS、Galileo、BDS 的基数分别为 0、100、200、300，例如 `G01 -> 1`、`R02 -> 102`、`C03 -> 303`。不能识别的卫星编号保留原字符串。

## AR 数据

LAMBDA 的 `arData`：

```text
[ratio, bootstrap, constraintsApplied, groups]
```

BIE 的 `arData`：

```text
[candidateCount, effectiveCandidates, constraintsApplied, groups]
```

每个 group 为：

```text
[system, ambiguityGroup, referenceSatellite,
 fixedCount, eligibleCount, maxFractionCycle, maxSigmaCycle]
```

无有效 ratio、bootstrap 或 BIE 候选统计时使用 JSON `null`，不会用数值 0 冒充有效指标。所有非有限浮点值，包括 `NaN` 和 `Inf`，同样写为 `null`。

## 高频写入与恢复

- 每个测站维护独立写入状态，互不阻塞。
- 结果按 `timeOffsetMs` 排序，使用 2 秒缓冲吸收短时乱序。
- 同一测站、同一时间偏移在写盘前重复出现时保留最后一个结果。
- UTC 日期变化时关闭旧日文件并创建新日文件。
- 运行期间直接写入 `.json`；正常关闭或跨日后补全 JSON 尾部。文件在活跃写入期间可能尚未是完整 JSON，关闭后即可作为完整结果使用。
- 同日重启时，已有 `.json` 会恢复为可追加状态，关闭后仍生成同一个日文件。
- 异常退出后会截断 `.json` 末尾未完成的记录，再从最后一个完整数组继续写入。仅当发现旧版遗留的 `.json.part` 且同名 `.json` 不存在时，程序会自动迁移并恢复它；两者同时存在会报错，避免覆盖任何数据。

当前文件保存成功产生的 PPP 解；滤波失败原因仍进入运行日志和 PPP-AR 诊断日志，不作为 solution 数组成员。
