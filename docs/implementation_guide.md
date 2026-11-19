# V2 实现与操作指南

本文说明如何部署 UAV 和 UGV 两端、检查 FKIE，以及发送一条 V2 任务。两端的 launch 默认启动各自的 V2 节点和一套 FKIE，使用各自的 ROS master。

## 构建

两端安装 ROS Noetic 和 FKIE 的 discovery、sync 包，将本仓库放进 catkin 工作区的 `src/`。

- UAV 构建 `ag_uav_client`，例如在 `~/catkin_ws` 执行 `catkin_make --only-pkg-with-deps ag_uav_client -DPYTHON_EXECUTABLE=/usr/bin/python3`。
- UGV 在已 source 原车工作区的环境里，用独立 overlay 构建 `ag_ugv_task`。它自带任务执行器，并复用原车的 `ugv_navigation` 地理定位和 `ugv_slicer_control`，无需旧 `air_ground_task` 包装脚本。不要把本仓库放进已有同名 `slicer_air_ground_msgs` 的车端 `src/`。

车端使用原包的 Docker 包装脚本时，可把 overlay 放在原项目根目录下的 `v2_ws/src/AG_Client`。由宿主机在原项目目录构建，原镜像需要有 FKIE：

```bash
./run.sh shell bash -c 'set -e
  cd /workspace/v2_ws
  catkin_make --only-pkg-with-deps ag_ugv_task -DPYTHON_EXECUTABLE=/usr/bin/python3
  catkin_make install'
```

## 两端启动

先让 UGV 原有定位和 RTK 导航正常运行。实车行驶还需原有底盘链路就绪。V2 车端入口替代旧的 `./run.sh tasks`，同一台车只运行一个任务执行器。

**UAV** · 在飞机电脑上：

```bash
source ~/catkin_ws/devel/setup.bash
unset ROS_HOSTNAME
ROS_MASTER_URI=http://192.168.10.32:11311 ROS_IP=192.168.10.32 \
  roslaunch ag_uav_client uav.launch
```

可用 `fix_topic:=...` 和 `phase_topic:=...` 接入本机现有 `NavSatFix` 和 `std_msgs/String` 数据。默认订阅 `/uav/fix` 和 `/uav/phase`，无输入就显示未知。

**UGV** · 在小车能访问原车和 V2 两层工作区的环境里，source V2 overlay 后运行：

```bash
unset ROS_HOSTNAME
ROS_MASTER_URI=http://192.168.10.33:11311 ROS_IP=192.168.10.33 \
  roslaunch ag_ugv_task ugv.launch
```

如果使用上面的 Docker overlay，从 UGV 宿主机运行：

```bash
ROS_MASTER_URI=http://192.168.10.33:11311 ROS_IP=192.168.10.33 \
  ./run.sh shell bash -c 'source /workspace/v2_ws/install/setup.bash && exec roslaunch ag_ugv_task ugv.launch'
```

已有 FKIE 时，先停旧实例再使用一键入口。若要保留已经配置好 V2 的 FKIE 实例，可给对应一键入口加 `start_fkie:=false`。更换电台 IP 时，设置本端的 `ROS_MASTER_URI` 和 `ROS_IP`，并用 UAV 侧 `ugv_ip:=...` 或 UGV 侧 `uav_ip:=...` 改对端地址。两端要使用各自 master。补充网络排查见 [FKIE 排查](chore/fkie_radio.md)。

## 只读验收和下发

在 UAV 本机 ROS 环境检查两端状态。用新鲜的 `updated_at` 确认链路，查看 UGV 的当前位置、活动点和剩余路线：

```bash
rosservice call /ag_master_sync/get_sync_info
rostopic echo -n 1 /air_ground/v2/state
rostopic echo -n 1 /air_ground/v2/uav_state
```

车端具备行驶条件后，在 UAV 端发任务。经纬度变量填现场实测的可通行点：

```bash
rosrun ag_uav_client v2_send.py SET_TARGET run-01 --lat "$TARGET_LAT" --lon "$TARGET_LON"
rosrun ag_uav_client v2_send.py INSERT_MVP run-01 --mvp-id p1 --lat "$MVP_LAT" --lon "$MVP_LON"
rosrun ag_uav_client v2_send.py PAUSE run-01
```

重复 `SET_TARGET run-01` 更新同一任务的终点。程序可向 UAV 本机 `/air_ground/v2/uav_intent` 发布 JSON，例如 `{"op":"SET_TARGET","task_id":"run-01","target":{"lat":31.94,"lon":118.78}}`。在 UGV 的 `last_command` 看是否受理，在 `phase` 和 `active_point` 看是否执行。现有 maxt 的二维码下发仍使用 V1，不能作为 V2 发送入口。

## 实现边界与离线测试

UGV 在任务层维护路线，复用车端定位配准、地图转换、`/ugv/move_base` 和 Mux 执行。FKIE 只同步 V2 命令及两端状态。本地 intent、导航 Action、TF 和控制话题留在各自设备上。

```bash
catkin_make run_tests_ag_ugv_task
source devel/setup.bash
/usr/bin/python3 src/AG_Client/ag_ugv_task/test/check_v2_ros.py
```

隔离联调自建 loopback ROS master，用车端执行器和模拟导航验证，不启动飞行或底盘节点。

## UGV Compose 服务

`deploy/ugv/compose.yaml` 将 V2 任务节点、FKIE discovery 和 sync 放进一个 host 网络服务，复用 UGV 原 master。使用前设置 `AG_UID`、`AG_GID`、`AG_RTK_ROOT`、`AG_NORTK_ROOT`、`AG_OVERLAY_ROOT`。overlay 须已安装，并建好 `runtime/`。服务的原车目录和安装产物只读挂载，ROS 日志落到 overlay 内的 `runtime/.ros`。

Compose 默认只发布状态，`reason: MONITOR_ONLY`，不订阅任务命令，也不连接导航和停车控制。需要正式接收车辆任务时，由现场单独决定启用执行模式。旧 FKIE 停用后可启动此服务，避免端口冲突。

```bash
docker compose -f deploy/ugv/compose.yaml up -d
docker logs -f ugv-ag-v2
```
