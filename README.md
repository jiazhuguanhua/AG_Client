# AG_Client

AG_Client 是 UAV 与 UGV 之间的 ROS Noetic 任务桥。默认使用 V2，支持更新终点、按 ID 编辑途经点（MVP）、暂停、恢复、停止和上报二维码展示点。两端入口会同时启动本地任务节点和 FKIE，状态由两端主动发布。

| 设备 | ROS 包 | 一键启动入口 | 默认电台 IP |
|---|---|---|---|
| UAV | `ag_uav_client` | `uav.launch` | `192.168.10.32` |
| UGV | `ag_ugv_task` | `ugv.launch` | `192.168.10.33` |

UGV 任务节点把路线拆成单个地图目标，交给原有 `move_base`。车端定位、规划和底盘仍由原系统启动。

- [实现与操作指南](docs/implementation_guide.md) · 构建、两端启动、发命令、验收与测试。
- [V2 协议](docs/protocol_v2.md) · 话题、动作和状态字段。
