# 自组网电台与 FKIE

本页示例默认 V1。使用 [V2](protocol_v2.md) 时，两端都加 `protocol_v2:=true`，同步 V2 命令及 UAV、UGV 状态，UAV 本地 intent 不跨机。

## 选型

电台若提供双向 IP 网络，FKIE 适合保留 UAV/UGV 各自 ROS master，并只共享本任务的两个 topic；它同步 ROS 注册信息，消息仍经 TCPROS 直接传输，不会改善电台丢包或自动压缩。若只需当前空地任务接口，先用原说明的**同一车端 master**最快；已在用双 master 时再用 FKIE。串口透明数传、NAT 后仅开放单端口、频繁断连或高丢包窄带链路，应改用专用网关。

FKIE 上游已标记 deprecated，适合现有 Noetic 系统短期集成；新项目应同时评估维护成本。依据：[上游仓库](https://github.com/fkie/multimaster_fkie)、[同步机制](https://fkie.github.io/multimaster_fkie/master_sync.html)。

## 先按接口 V1：同一个 master

将下面 `UGV_IP`、`UAV_IP` 替换为两端电台网络的实际固定 IP。车端继续原任务节点和导航链；飞机只启动本客户端。每个终端设置一致，source 两端同版消息包。

```bash
# UGV 各终端
export ROS_MASTER_URI=http://UGV_IP:11311 ROS_IP=UGV_IP
unset ROS_HOSTNAME

# UAV 各终端：连接车端 master
export ROS_MASTER_URI=http://UGV_IP:11311 ROS_IP=UAV_IP
unset ROS_HOSTNAME
```

无需 FKIE。保留唯一串行命令出口，不启动飞行任务，不同步其他导航或控制接口。

## FKIE：两个 master

本仓库提供 `ag_client/launch/fkie_radio.launch` 和 `ag_client/config/fkie_sync.yaml`。在两端安装 `ros-noetic-fkie-master-discovery`、`ros-noetic-fkie-master-sync`，构建并 source 本仓库的消息包和客户端。通信容器也须 source 相同消息环境；使用 host 网络或保证其公布地址可双向访问。

两端分别使用**自己的** master：UGV 为 `http://UGV_IP:11311`、UAV 为 `http://UAV_IP:11311`，各自 `ROS_IP` 为本机电台 IP，`unset ROS_HOSTNAME`。已有 master 就沿用，不再启动第二个；各端仅允许一套 discovery/sync，已有 FKIE 应修改原实例配置，避免并行启动本例。

```bash
# UGV，source 本工作区后，在车端本地 master 下运行
roslaunch ag_client fkie_radio.launch local_name:=ugv peer_name:=uav peer_ip:=UAV_IP

# UAV，source 本工作区后，在飞机本地 master 下运行
roslaunch ag_client fkie_radio.launch local_name:=uav peer_name:=ugv peer_ip:=UGV_IP
```

默认关闭多播，以 `robot_hosts` 对端 IP 单播发现，适合不转发多播的电台；两端 UDP discovery 端口须一致，默认 11511。若确认支持多播，可两端加 `multicast:=true`。`local_name` 是本例显式设置的 master 名，`peer_name` 必须与对端一致；沿用原实例时核实其真实发现名称。

白名单只有 `/air_ground/command` 和 `/air_ground/state`，保持原名及正式类型。命令发布者非锁存、有限队列、一次发布；状态锁存队列 1，每 0.2 s 最新快照。FKIE 不添加重发器。**`sync_services: []` 本身不表示禁用服务**，所以配置显式 `ignore_services: ['*']`；`sync_nodes` 保持空，否则可能放开整节点；`sync_topics_on_demand: false` 防止本地订阅意外扩大白名单。不上电台传 `/tf`、图像、点云、Action、`/cmd_vel`。

默认 `sync_remote_nodes: false` 要求接口节点与其 master 在同一主机。若接口节点实际注册到另一台主机的 master，先核对节点 URI，再按需两端增加 `sync_remote_nodes:=true`；topic 白名单仍生效。不要用它掩盖错误的 ROS_IP。配置语义核对本机 FKIE 1.3.2 源码及[官方默认 launch](https://github.com/fkie/multimaster_fkie/blob/master/fkie_master_sync/launch/master_sync.launch)、[过滤实现](https://github.com/fkie/multimaster_fkie/blob/master/fkie_master_discovery/src/fkie_master_discovery/filter_interface.py)。

## 连通与验收

1. 两端 IP、路由、主机名解析双向可达；允许可信电台网段内 master TCP 11311、discovery UDP 11511、discovery XML-RPC TCP 11611，以及 ROS 节点动态 XML-RPC/TCPROS 端口。仅 ping 或开放 11311 不够；跨 NAT 不建议直接 FKIE。不得将 ROS1 接口暴露到公网。
2. 两端用 chrony/NTP 同步系统时钟；检查 ROS `/use_sim_time`。真实设备使用同一时间基准，状态未来时间或超龄应修复时钟，不放宽保护。
3. 只读检查以下接口；命令 topic 应只有一个逻辑发送出口，状态只有车端任务发布者。FKIE `get_sync_info` 只列允许的两个 topic，无业务服务同步。

```bash
rostopic type /air_ground/command
rostopic type /air_ground/state
rostopic info /air_ground/command
rostopic info /air_ground/state
rosmsg md5 slicer_air_ground_msgs/Command
rosservice call /ag_master_sync/get_sync_info
```

预期类型为 `slicer_air_ground_msgs/Command`、`std_msgs/String`，Command MD5 为 `b574bb9bb1240c0d2ae262d6c3feb2bf`。读取客户端新鲜状态后，再由现场人员组织地面联调；本包本地夹具测试不能代表电台或车辆验收。电台实测应记录 RTT、丢包、吞吐、状态年龄、重连耗时和命令回执；窄带时首先测状态 JSON 字节数 × 5 Hz 的实际负载。

**断链不会自动停车**：V1 明确允许车端继续最后已受理任务。FKIE 离线发现也不是停车保障。回执超时表示结果未知；先看新鲜状态，必要时原样 replay 保存请求，保留身份与负载，不自动生成新 SET 或重新排队。物理急停/接管沿用原车端方案。

## 其他方案

以下是针对本接口的工程建议，当前版本仍使用原 ROS1 topic 方法。

| 方案 | 适用条件 | 代价 / 注意点 |
| --- | --- | --- |
| 同一 ROS master | 两台设备双向 IP 可达、链路稳定、接口很少 | 实现最少；动态端口和远端 master 依赖仍存在 |
| FKIE 白名单双 master | 两端各自独立运行，已有 Noetic 多 master 系统 | 保留原消息；仍需双向动态端口，上游已弃用 |
| [rosbridge WebSocket](https://github.com/RobotWebTools/rosbridge_suite/tree/ros1) 专用网关 | 只能放行固定端口、跨语言客户端 | 增加网关及 JSON 编解码；限制两个 topic，并保持一次发布、回执关联及重连策略；`uint64 sequence` 不得经 JavaScript Number 丢精度 |
| [MQTT 5](https://mqtt.org/mqtt-specification/) + 小型 ROS 网关 | 窄带、频繁断连，需要显式消息过期/会话控制 | 需自行实现协议映射；命令禁用 retain 与离线积压，QoS 可能重复，必须沿用 request_id 去重，不能把 broker ACK 当受理回执 |
| 自定义 TCP/QUIC + Protobuf 网关 | 需要紧凑编码、严格带宽预算或网络拓扑控制 | 开发测试成本最高；必须补齐边界、uint64、身份、重复、过期及断线语义，不能越过原任务层直控底盘 |

建议顺序：现有同 master 接口跑通 → 必要时 FKIE 白名单 → 实测证明链路/拓扑不满足，再做固定端口网关。更换传输不改变终点 Action、停车窗口及地理复核三项完成条件。
