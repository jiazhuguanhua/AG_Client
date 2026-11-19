# 空地任务 V2

UAV 和 UGV 都从 AG_Client 构建。UAV 的 `ag_client` 发送命令和飞机状态，UGV 的 `ag_ugv_task` 保存路线并驱动已有导航。两个节点各上各自的 ROS master。协议按状态快照工作，不需要查询服务、命令前读取状态或逐条等待回执。

## 话题

| 话题 | 类型 | 流向 |
|---|---|---|
| `/air_ground/v2/uav_intent` | `std_msgs/String` JSON | UAV 本机的调用方 → UAV gateway |
| `/air_ground/v2/command` | `std_msgs/String` JSON | UAV gateway → UGV |
| `/air_ground/v2/state` | `std_msgs/String` JSON | UGV → UAV |
| `/air_ground/v2/uav_state` | `std_msgs/String` JSON | UAV → UGV |

两端 FKIE 加 `protocol_v2:=true`，只同步后三项。已运行 FKIE 时在两边一起切换现有实例的配置，不叠加第二套。V1 与 V2 话题隔离，正常联调只运行一个车端任务接口，避免两个执行器竞争 `move_base`。旧 maxt 和 `test_menu.py` 仍发 V1，当前不会自动转换为 V2。

## 命令

所有本地 intent 都带 `op` 和由任务发起方创建的 `task_id`。gateway 加 `version: 2`、递增整数 `seq` 和 `sent_at`（ROS 时间秒），一次发送。车端当前只接受一个逻辑命令源，按 `seq` 忽略旧消息。正常运行时使用同一 gateway 序列；系统时钟回拨或重启后要避免序号倒退。命令非锁存，断线不补发，没有自动重试或重放。

| `op` | 额外字段 | 行为 |
|---|---|---|
| `SET_TARGET` | `target: {lat, lon}` | 首次建任务；同任务后续只更换终点，保留待经过 MVP 和暂停状态。 |
| `INSERT_MVP` | `mvp_id`、`point: {lat, lon}`，可选 `before_id` | 默认追加在最终目标前。有 `before_id` 时插到指定待经过 MVP 前。 |
| `DEL_MVP` | `mvp_id` | 删除还未经过的点。 |
| `UPDATE_MVP` | `mvp_id`、`point: {lat, lon}` | 改坐标，保留顺序。 |
| `PAUSE` | — | 请求停车，保留路线。暂停期间仍可编辑。 |
| `RESUME` | — | 从暂停或阻塞恢复，按剩余路线执行。 |
| `STOP` | — | 请求停车，确认停稳后结束任务并清空路线。 |

坐标是 WGS84 地面点。MVP ID 在同一任务内不重复使用；已经过或已删除的 ID 不能再修改。没有任务时先发 SET。STOP 或完成后用新的任务 ID；STOPPING 时不接受改路线或恢复。若丢了一条增量命令，以 UGV 快照为准，由上层决定下一步；第一版无路线快照替换和历史查询。

## 执行与状态

路线为 `remaining_mvps → target`。途经点进入通过半径就完成该段，默认 0.25 m。终点复用车端的导航成功、停车和地理到达复核。只在当前点的 ID 或坐标变化时中断当前段；取消未确认时不派发新 Action。导航与定位、停车、外部 hold 门禁都在 UGV 本地处理。只有 UGV 才决定是否接受并执行。

两种状态共用以下形状，发送方不同。

| 字段 | 含义 |
|---|---|
| `version`、`vehicle_id`、`updated_at` | 协议版本、`ugv` 或 `uav`、本帧时间（ROS 时间秒）。 |
| `vehicle_position` | `lat`、`lon`、`measured_at`、`valid`。无有效新鲜坐标时为 null 坐标。UGV 读车端已补偿的 RTK 观测，UAV 读配置的 NavSatFix。 |
| `task_id`、`phase`、`reason` | 任务 ID、车型自己的状态及原因。UGV 有 IDLE、READY、SWITCHING、EXECUTING、PAUSING、PAUSED、STOPPING、STOPPED、COMPLETED、BLOCKED。UAV phase 来自可选状态源，缺省为 null。 |
| `active_point` | UGV 当前实际派发的段，含 `kind: MVP/TARGET`、`mvp_id`（终点时 null）、`lat`、`lon`，无活动段为 null。切换时可暂时显示正在取消的旧点。 |
| `remaining_mvps`、`target` | 有序的 `{mvp_id,lat,lon}` 和 `{lat,lon}` 终点。UAV 不拥有车端路线，其相应字段为空。 |
| `last_command` | `task_id`、`seq`、`op`、`sent_at`、`received_at`、`accepted`、`reason`。UGV 显示最近处理的一条，UAV 只显示最后发布的一条，接受与接收时间在 UAV 上为 null。 |

状态变化时发布，运行平稳时每秒一帧。一次状态快照可能覆盖较早的命令结果。`accepted` 表示车端接受意图；执行进度和完成看 `phase`。两端时钟同步后才可比较收发时间。断网不会让车自动停车。

## 启动与依赖

1. UAV：构建并 source 含 `ag_client` 的工作区。配置现有 FKIE 后运行 `roslaunch ag_client uav.launch`。可通过 `fix_topic:=...` 绑定已有 NavSatFix、`phase_topic:=...` 绑定已有 String 状态。未提供数据源也可发送路线，UAV 状态如实显示未知。launch 仅启动 gateway。
2. UGV：在**原车工作空间之上的独立 catkin overlay**构建 `ag_ugv_task`，避免在同一个 src 树放两份 `slicer_air_ground_msgs`。运行环境先有原车的 `ugv_navigation`、`ugv_slicer_control`，再 source V2 overlay。保留原来的定位、导航和底盘启动流程。启动 `roslaunch ag_ugv_task ugv.launch` 替代 `./run.sh tasks`。如果车端依赖在 Docker 中构建，就在能访问同一 overlay、原车安装空间和 UGV master 的 host 网络容器里运行新 launch。
3. 车端 V2 通过原车已安装的 `RosNavigation`、`TaskExecutor` 接现有系统。该适配层在 `/geographic/status` 取配准与测量，使用现有 TF/里程计，向 `/ugv/move_base` 发 Action，通过原 Mux 状态和 `control.py` 管理停车。无需把这些接口同步给 UAV。依赖原车 ZIP 里已有的模块及其安装布局，无需复制车端源码到本仓库。

使用车端原包自带的 Docker 包装脚本时，可以将 V2 overlay 放在原项目根目录下的 `v2_ws/src/AG_Client`（不放进原项目的 `src/`）。在 UGV 宿主机的原项目根目录执行：

```bash
./run.sh shell bash -c 'set -e
  cd /workspace/v2_ws
  catkin_make --only-pkg-with-deps ag_ugv_task -DPYTHON_EXECUTABLE=/usr/bin/python3
  catkin_make install'
./run.sh shell bash -c 'source /workspace/v2_ws/install/setup.bash && exec roslaunch ag_ugv_task ugv.launch'
```

此方式复用原车镜像、host 网络和 UGV master。只运行一个车端任务节点。车端若也在该容器环境内启动 FKIE，使用同一个 V2 overlay；原镜像需要能够找到 FKIE 包。

UAV 手工发送示例。下面变量应是现场实测可通行地点，测试运行命令可能让 UGV 行驶。

```bash
rosrun ag_client v2_send.py SET_TARGET run-01 --lat "$TARGET_LAT" --lon "$TARGET_LON"
rosrun ag_client v2_send.py INSERT_MVP run-01 --mvp-id p1 --lat "$MVP_LAT" --lon "$MVP_LON"
rosrun ag_client v2_send.py PAUSE run-01
rostopic echo -n 1 /air_ground/v2/state
```

更新终点时重复 `SET_TARGET run-01`。程序调用可以直接向 UAV 本地 intent 话题发同一 JSON，例如 `{"op":"SET_TARGET","task_id":"run-01","target":{"lat":31.94,"lon":118.78}}`。

## 验证

`catkin_make run_tests_ag_ugv_task` 执行纯路线测试。另有 `ag_ugv_task/test/check_v2_ros.py`，接受原车 `ugv_navigation/scripts` 目录，启动独立 loopback master，用原车 TaskExecutor 和模拟导航验证完整流程。无需接入现场 master 或硬件。

```bash
source ~/catkin_ws/devel/setup.bash
/usr/bin/python3 ag_ugv_task/test/check_v2_ros.py /path/to/ugv_navigation/scripts
```
