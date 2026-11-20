# 空地任务 V2

本文定义空地任务 V2 的命令、执行语义和状态字段。UAV 的 `ag_uav_client` 发送命令和飞机状态，UGV 的 `ag_ugv_task` 保存路线并驱动已有导航。两个节点各上各自的 ROS master。协议按状态快照工作，不需要查询服务、命令前读取状态或逐条等待回执。

## 话题

| 话题 | 类型 | 流向 |
|---|---|---|
| `/air_ground/v2/uav_intent` | `std_msgs/String` JSON | UAV 本机的调用方 → UAV gateway |
| `/air_ground/v2/command` | `std_msgs/String` JSON | UAV gateway → UGV |
| `/air_ground/v2/state` | `std_msgs/String` JSON | UGV → UAV |
| `/air_ground/v2/uav_state` | `std_msgs/String` JSON | UAV → UGV |

默认 FKIE 只同步后三项。本地 intent 留在 UAV。部署和话题验收见 [实现与操作指南](implementation_guide.md)。

## 命令

所有本地 intent 都带 `op` 和由任务发起方创建的 `task_id`。gateway 加 `version: 2`、递增整数 `seq` 和 `sent_at`（ROS 时间秒），一次发送。车端当前只接受一个逻辑命令源，按 `seq` 忽略旧消息。正常运行时使用同一 gateway 序列；系统时钟回拨或重启后要避免序号倒退。命令非锁存，断线不补发，没有自动重试或重放。

| `op` | 额外字段 | 行为 |
|---|---|---|
| `SET_TARGET` | `target: {lat, lon}`，可选 `target.image_id` | 首次建任务；同任务后续只更换终点，保留待经过 MVP 和暂停状态。 |
| `REPORT_QR` | `qr: {qr_id, lat, lon, measured_at, revision}`，可选 `altitude_m` 和 `image_id` | 上报一个二维码展示点；可在导航任务建立前发送。 |
| `INSERT_MVP` | `mvp_id`、`point: {lat, lon}`，可选 `before_id` | 默认追加在最终目标前。有 `before_id` 时插到指定待经过 MVP 前。 |
| `DEL_MVP` | `mvp_id` | 删除还未经过的点。 |
| `UPDATE_MVP` | `mvp_id`、`point: {lat, lon}` | 改坐标，保留顺序。 |
| `PAUSE` | — | 请求停车，保留路线。暂停期间仍可编辑。 |
| `RESUME` | — | 从暂停或阻塞恢复，按剩余路线执行。 |
| `STOP` | — | 请求停车，确认停稳后结束任务并清空路线。 |

坐标是 WGS84 地面点。MVP ID 在同一任务内不重复使用；已经过或已删除的 ID 不能再修改。用 SET_TARGET 创建导航任务。STOP 或完成后用新的任务 ID；STOPPING 时不接受改路线或恢复。若丢了一条增量命令，以 UGV 快照为准，由上层决定下一步；第一版无路线快照替换和历史查询。

`REPORT_QR` 的 `task_id` 使用巡检任务 ID，`qr_id` 使用二维码原文，`measured_at` 是采样的 ROS 时间（秒），`altitude_m` 是 WGS84 椭球高（米）。车端保存最近一次有效上报所对应的巡检任务内的展示点，按 `qr_id` 更新。相同版本且字段相同的再次上报会受理，旧版本或同版本内容冲突会拒绝。更新测量值或图片标识时递增 `revision`。

`image_id` 是可选的非空字符串，最长 128 字符，用于关联后续独立传输的图片。本协议只传标识。更新导航终点的图片标识会更新 `target` 状态，坐标相同时继续执行当前导航段。

## 执行与状态

路线为 `remaining_mvps → target`。途经点进入通过半径就完成该段，默认 0.25 m。终点复用车端的导航成功、停车和地理到达复核。只在当前点的 ID 或坐标变化时中断当前段；取消未确认时不派发新 Action。导航与定位、停车、外部 hold 门禁都在 UGV 本地处理。只有 UGV 才决定是否接受并执行。

两种状态共用以下形状，发送方不同。

| 字段 | 含义 |
|---|---|
| `version`、`vehicle_id`、`updated_at` | 协议版本、`ugv` 或 `uav`、本帧时间（ROS 时间秒）。 |
| `vehicle_position` | `lat`、`lon`、`measured_at`、`valid`。无有效新鲜坐标时为 null 坐标。UGV 读车端已补偿的 RTK 观测，UAV 读配置的 NavSatFix。 |
| `task_id`、`phase`、`reason` | 任务 ID、车型自己的状态及原因。UGV 有 IDLE、READY、SWITCHING、EXECUTING、PAUSING、PAUSED、STOPPING、STOPPED、COMPLETED、BLOCKED。UAV phase 来自可选状态源，缺省为 null。 |
| `active_point` | UGV 当前实际派发的段，含 `kind: MVP/TARGET`、`mvp_id`（终点时 null）、`lat`、`lon`，无活动段为 null。切换时可暂时显示正在取消的旧点。 |
| `remaining_mvps`、`target` | 有序的 `{mvp_id,lat,lon}` 和 `{lat,lon}` 终点。终点可能带有 `image_id`。UAV 不拥有车端路线，其相应字段为空。 |
| `last_command` | `task_id`、`seq`、`op`、`sent_at`、`received_at`、`accepted`、`reason`。UGV 显示最近处理的一条，UAV 只显示最后发布的一条，接受与接收时间在 UAV 上为 null。 |
| `qr_reports` | UGV 最近一轮巡检的展示点列表，每条含 `task_id` 和 `qr` 的字段。UGV 显示程序可订阅 `/air_ground/v2/state` 读取。 |
| `last_qr_report` | 最近一次 `REPORT_QR` 的状态，字段同 `last_command`，另含 `qr_id`。UGV 的 `accepted` 表示已更新展示数据。 |

状态变化时发布，运行平稳时每秒一帧。一次状态快照可能覆盖较早的命令结果。`accepted` 表示车端接受意图；执行进度和完成看 `phase`。两端时钟同步后才可比较收发时间。断网不会让车自动停车。
