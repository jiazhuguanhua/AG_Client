# 历史验证记录

本文记录 V1 客户端和 maxt 接入时的测试结果。当前 V2 的验证步骤见 [实现与操作指南](../implementation_guide.md)。

2026-11-18 C++ 接入增量：完整 maxt/AG_Client 构建通过，既有52项客户端测试仍全部通过；`maxt_pkg/test/qr_ugv_dispatch.test` 的18项跨仓库集成测试通过（0错误/0失败）。真实 QR context+C++ SDK 与假 UGV 通信，BT 仅运行无副作用心跳，不构造 MavKit、不启动生产任务、MAVROS或飞控。覆盖中文日志、坐标/角色/任务门禁、重复心跳、uint64身份/回执、异常JSON、错误任务结果、完整到达证据、超时/重启/无连接、Python/C++共用发送锁及等待期间BT tick持续推进。独立临时工作区 build/install、仅使用 install 的外部 C++ 消费者编译/链接和Python入口检查均通过，未覆盖 maxt 的既有 install 配置。

2026-11-17，ROS Noetic / Python 3.8。未启动硬件、飞控、解锁或起飞，未向真实车辆发送任务。

| 范围 | 结果 |
| --- | --- |
| 两包 Catkin 定向构建及 install | 通过 |
| 协议单元测试 | 18 项通过 |
| 执行健康/任务证据单元测试 | 12 项通过 |
| 独立 master、`/test_ag/*` 假车端集成测试 | 22 项通过，0 errors / 0 failures |
| 对照用户车端 ZIP 真实纯逻辑代码 | 30 项兼容检查通过，结果见 [兼容基线](validation/vehicle_source_compatibility.json) |
| Command MD5 | `b574bb9bb1240c0d2ae262d6c3feb2bf`，与接口 V1 一致 |
| install 环境 Python 导入、CLI `--help` | 通过，确认导入 install 路径 |
| Python 编译、Git 空白检查 | 通过 |
| 本机 FKIE 1.3.2 真实 FilterInterface 离线检查 | 允许两个任务 topic，拦截七个其他 topic 的发布/订阅及三个服务；launch XML 解析通过 |

集成测试覆盖五类命令和空途经点、消息序列化及默认负载、请求保存且不覆盖、原样 replay/duplicate、非锁存且重连不重放、旧 boot 拒绝、未来/过期/坏 JSON/停止反馈、无订阅者不发布、回执身份匹配、明确拒绝、回执超时后仅发一次且不取消、发布后接收端重启、跨客户端本地发送锁、CLI 的 0/1/2 退出码及无 master 有界退出。

复现时使用独立环境，避免继承真实车端 master：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps ag_uav_client -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11379 ROS_IP=127.0.0.1
unset ROS_HOSTNAME
catkin_make run_tests_ag_uav_client -j2
catkin_test_results build/test_results/ag_uav_client
```

有效自动测试数是 18 + 12 + 22 = **52**；Catkin 汇总还可能计入 rostest 外层测试进程，不把它重复计为业务用例。测试 XML 在 `~/catkin_ws/build/test_results/ag_client/`。新增覆盖取消中阶段、本端执行健康门禁与停止请求通道、旧/新transport_error、只读CLI及任务归属/静止窗口证据。

30项独立兼容检查通过动态导入外部源码运行，不复制车端实现到本包；检查正式消息定义、实际 capture_target 输出、双方请求文件保存/加载、五类命令映射/序列化、真实 TaskCore 受理/去重及取消阶段。复现：

```bash
source ~/catkin_ws/devel/setup.bash
python3 ~/catkin_ws/src/AG_Client/ag_uav_client/test/check_vehicle_compat.py \
  --vehicle-root /absolute/path/to/extracted/Online_Navigation_with_RTK
```

该脚本不初始化 ROS，不访问网络或硬件，不写车端源码。52项客户端测试和30项源码兼容检查分别报告，不冒充车端全回归。尚未验证：实机采点、真实车端 ROS 进程及部署版本、电台双机 FKIE 链路与射频工况、底盘运动和到达精度；也未在本机运行原车端整套 ROS 集成测试。不复用原接口文档中车端429项结论。
