# 本地验证

2026-11-17，ROS Noetic / Python 3.8。未启动硬件、飞控、解锁或起飞，未向真实车辆发送任务。

| 范围 | 结果 |
| --- | --- |
| 两包 Catkin 定向构建及 install | 通过 |
| 协议单元测试 | 18 项通过 |
| 独立 master、`/test_ag/*` 假车端集成测试 | 17 项通过，0 errors / 0 failures |
| Command MD5 | `b574bb9bb1240c0d2ae262d6c3feb2bf`，与接口 V1 一致 |
| install 环境 Python 导入、CLI `--help` | 通过，确认导入 install 路径 |
| Python 编译、Git 空白检查 | 通过 |
| 本机 FKIE 1.3.2 真实 FilterInterface 离线检查 | 允许两个任务 topic，拦截七个其他 topic 的发布/订阅及三个服务；launch XML 解析通过 |

集成测试覆盖五类命令和空途经点、消息序列化及默认负载、请求保存且不覆盖、原样 replay/duplicate、非锁存且重连不重放、旧 boot 拒绝、未来/过期/坏 JSON/停止反馈、无订阅者不发布、回执身份匹配、明确拒绝、回执超时后仅发一次且不取消、发布后接收端重启、跨客户端本地发送锁、CLI 的 0/1/2 退出码及无 master 有界退出。

复现时使用独立环境，避免继承真实车端 master：

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps ag_client -DPYTHON_EXECUTABLE=/usr/bin/python3 -j2
source devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11379 ROS_IP=127.0.0.1
unset ROS_HOSTNAME
catkin_make run_tests_ag_client -j2
catkin_test_results build/test_results/ag_client
```

有效用例数是 18 + 17 = **35**；Catkin 汇总还可能计入 rostest 外层测试进程，不把它重复计为业务用例。测试 XML 在 `~/catkin_ws/build/test_results/ag_client/`，结果不提交为源码。

尚未验证：真实采点文件/原 SDK 保存请求格式、真实车端状态机、电台双机 FKIE 链路与射频工况、底盘运动和到达精度。测试仅验证本客户端，不复用原接口文档中车端的 429 项结论。最终实车完成仍须原车端的 Action、停车窗口和地理复核证据。
