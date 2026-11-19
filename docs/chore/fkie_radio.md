# FKIE 网络排查

本页供一键启动后发现不到对端或读不到状态时使用。日常启动看 [实现与操作指南](../implementation_guide.md)。

1. 两端用各自的 ROS master。确认电台 IP 可双向访问，`ROS_IP` 指向本端电台网口，launch 中的 peer IP 指向对端。
2. 默认通过 UDP 11511 单播发现，使用 TCP 11611 读取发现信息。还需允许 master TCP 11311 及 ROS 节点动态 XML-RPC/TCPROS 端口。仅 ping 或开放 11311 不足以验证话题通信。
3. 在任一端用 `rosservice call /ag_master_sync/get_sync_info` 查看发现结果，再用 `rostopic info /air_ground/v2/state` 和 `rostopic echo -n 1 /air_ground/v2/state` 分别核对注册与实际数据。
4. 白名单应只有 V2 的 `command`、`state`、`uav_state`；不要同步底盘控制、导航、TF 和传感器数据。若状态可读但时间过旧，检查两端系统时钟和 `/use_sim_time`。
5. 两端各保持一套 discovery 和 sync。断链期间的新命令不会补发，UGV 可能继续执行最后已受理的路线。
