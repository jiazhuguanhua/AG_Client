# V1 兼容入口

本页供仍使用旧车端任务接口或 maxt V1 的系统维护参考。当前部署入口统一使用 [V2](../implementation_guide.md)，V1 使用独立话题，不应与 V2 车端执行器同时运行。

## 构建

已对照用户提供的车端 ZIP 源码完成适配；[小车架构、适配结论与车端建议](vehicle_architecture.md)。

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps ag_uav_client -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
rosmsg md5 slicer_air_ground_msgs/Command
```

MD5 必须为 `b574bb9bb1240c0d2ae262d6c3feb2bf`。同一工作区不得重复安装另一个同名消息源码包。原工作区构建其他包时，使用 `catkin_make -DCATKIN_WHITELIST_PACKAGES=""` 清除选择列表。

## 连接及命令

使用 V1 时按原文档连接同一个车端 master。当前默认 FKIE 白名单仅支持 V2。运行客户端不会启动硬件、解锁、起飞、启动导航或解除 hold；`set` 等任务命令可能驱动车辆，只由现场人员在原车端准备完成后执行。

在 UAV 电脑上沿用已打通的 FKIE 环境，运行 `rosrun ag_uav_client test_menu.py`。选 **1** 原样输出下一帧新鲜 `/air_ground/state`。选 **2** 输入 WGS84 纬度和经度，回车直接发布一次 V1 目标，随后打印收到的一帧状态。选 **3/4/5** 发布暂停、恢复、取消，可填写 task_id 或用车端状态里的当前任务。菜单只从状态提取 V1 封包需要的身份和序号，不检查导航是否就绪，不等待匹配回执。车端仍按自己的规则决定是否执行。

```bash
rosrun ag_uav_client ag_client_cli.py status
rosrun ag_uav_client ag_client_cli.py health
rosrun ag_uav_client ag_client_cli.py task-status "$TASK_ID"  # 只读检查指定任务的完成证据
rosrun ag_uav_client ag_client_cli.py health   # 只读：执行线程/启动检查/取消屏障
rosrun ag_uav_client ag_client_cli.py task-status TASK_ID  # 只读：该任务的完成证据

# TARGET_FILE 为真实 geographic.py capture 文件的本机路径
rosrun ag_uav_client ag_client_cli.py set --target-file "$TARGET_FILE" \
  --save-request /tmp/mission.request.json
rosrun ag_uav_client ag_client_cli.py replace-via --via-file "$VIA_FILE"
rosrun ag_uav_client ag_client_cli.py replace-via   # 清空途经点，保留终点
rosrun ag_uav_client ag_client_cli.py pause
rosrun ag_uav_client ag_client_cli.py resume
rosrun ag_uav_client ag_client_cli.py cancel
rosrun ag_uav_client ag_client_cli.py replay /tmp/mission.request.json
```

显式坐标：`set --latitude LAT --longitude LON [--altitude-ellipsoid HEIGHT] [--yaw RAD]`。坐标是车体 `base_link` 水平地面投影的 WGS84 位置；高度为椭球高，yaw 为 ENU 东零逆时针弧度。不要传飞机位置/飞行高度或重复做杆臂补偿。`--via-file` 可重复，文件顺序即途经顺序；途经点禁止 yaw。

所有等待选项放在子命令后：`--state-timeout 5 --state-max-age 2 --connect-timeout 3 --receipt-timeout 3`，单位秒，必须为有限正数。`--command-topic`、`--state-topic` 支持隔离测试的重命名，实际使用保持默认正式 topic。

新 SET/REPLACE/RESUME 发布前还检查执行快照、启动状态及取消屏障，`--execution-max-age` 默认2秒；不会将 worker 卡住时持续发布的新鲜反馈当作可执行。PAUSE/CANCEL 不增加这些门禁；replay 保持准确旧请求语义。

退出码：任务命令 `0` 收到明确受理，`1` 车端明确拒绝，`2` 本地输入/传输/回执异常；只读 status/health/task-status 的 `0` 表示获取结果，是否健康/完成须看输出字段。**回执超时可能已执行**，会打印原请求；先读状态，再按需 replay 保存文件。保存必须使用新路径，拒绝覆盖；不要编辑已保存请求后重放。

## Python 接入

```python
import rospy
from ag_uav_client import load_target
from ag_uav_client.client import AirGroundClient, CommandRejected, OutcomeUnknown

rospy.init_node('air_ground_client')
with AirGroundClient() as client:
    state = client.status()  # 完整最新快照，保持车端执行证据
    goal = load_target(target_file)  # 调用者传入真实采点路径
    try:
        receipt = client.set_task(goal, save_path=new_request_path)
    except CommandRejected as exc:
        handle_rejection(exc.receipt)
    except OutcomeUnknown as exc:
        record_unknown(exc.request)  # 不自动重试、取消或生成新 SET
```

`set_task(final_goal, via_points=None, save_path=None)`、`replace_via_points(points, save_path=None)`、`pause/resume/cancel(save_path=None)`、`replay(request)` 均返回匹配身份的回执字典；`status()` 返回状态字典。用 `point(latitude, longitude, altitude_ellipsoid_m=None, yaw=None)` 构造显式地面坐标，`load_request(path)` 读取保存请求。异常区分见 [协议与限制](protocol.md)。

命令非锁存、队列 1、每次调用只发布一次。新命令读取新鲜状态分配全局序号；相同主机、用户、master/topic 的发送操作互斥。跨主机无法由本地锁仲裁，飞机端仍必须保留唯一串行出口，人工命令也通过该出口协调。断网不自动取消，车端可继续最后任务。

`accepted=true` 表示受理。最终完成看同任务 `phase=COMPLETED`、`last_task_result`，以及 Action 成功、停车窗口和地理复核全部通过；客户端不把受理当作到达。

Python 可调用 `client.health()` 和 `client.task_status(task_id)`，后者仅对匹配任务且三项证据与静止窗口均有效时返回 `verified_complete=true`；它是对车端软件证据的检查，不是独立物理测量。

## 验证

```bash
cd ~/catkin_ws
source devel/setup.bash
catkin_make run_tests_ag_uav_client
catkin_test_results build/test_results/ag_uav_client
```

测试只启动假车端、独立 ROS master 和测试 topic；不启动飞行或真实车辆。范围、结果和未验证项见 [测试记录](testing.md)。
