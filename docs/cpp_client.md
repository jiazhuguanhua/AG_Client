# C++ 单目标派车客户端

`ag_client_cpp` 是可链接的公共库，不是额外代理节点。当前只封装“提交一个地面目标 SET_TASK＋异步观察”，Python CLI/API 的暂停、恢复、取消、保存重放能力继续保留。

```text
maxt 当前 QR 任务结果
  → 选定 target 的首次有效 WGS84 坐标
  → Client::submitTarget（仅入队）
  → 私有 CallbackQueue / AsyncSpinner / 50 ms WallTimer
  → /air_ground/command → UGV
  ← /air_ground/state ← UGV
  → 受理、执行、阻塞、完成或未知结果日志
```

## 调用

在正常 ROS 初始化后构造共享客户端，并让它存活到任务观察结束：

```cpp
#include <ag_client/client.hpp>

ag_client::Client client;
ag_client::Target target;
target.mission_id = current_mission_id;
target.label = qr_content;
target.latitude = measured_ground_latitude;
target.longitude = measured_ground_longitude;
std::string reason;
bool queued = client.submitTarget(target, &reason);
```

`queued=true` 只代表首次入队或该 mission/label 已入队，不代表已发送/受理。调用不等待 ROS 连接、回执或小车到达；内部忙时立即返回 false，可在下一次 QR 心跳尝试入队。队列只接纳一个正在观察的目标，重复 mission/label 不重复发令。无额外线程由调用方管理。

发送前检查反馈与执行快照、WGS84/base_link_horizontal、startup_checked、取消屏障和命令订阅连接；按最新 server_id 分配 uint64 序号及 UUID，一次非锁存发布。仅发送目标纬经度，高度/yaw 未指定；不发送飞机高度。发送锁与 Python 客户端兼容，但不能代替跨主机的唯一逻辑出口约定。

默认等待状态与连接总上限8秒（state_timeout+connect_timeout），受理回执3秒，反馈/执行年龄上限各2秒。未发送超时不会自行重试；发布后超时记录“结果未知”，继续被动观察，不重发、不取消。车端重启后停止观察旧任务，不自动创建新任务。此薄封装没有跨进程持久化或自动恢复功能。

## 日志

中文日志要求在应用启动最前、ROS/log4cxx 初始化前执行 `std::setlocale(LC_CTYPE, "C.UTF-8")`（`<clocale>`）。maxt 入口已设置，仅影响字符编码，不改变小数点等数字格式；避免默认 C locale 将中文转成问号。

统一使用 ROS 命名 logger `ag_client`，前缀 `[AG_Client]`；输出到节点终端、`/rosout` 和标准 ROS 日志目录，不另外创建散落日志文件。经纬度输出9位小数，便于关联实际目标，不表示保证该精度。

- `已发送【QR001】：经纬度 <实际纬度>, <实际经度> 到 UGV！`，带 request_id/task_id。
- `UGV 已收到【QR001】，任务已受理：经纬度 ...`：匹配正式 accepted 回执。
- `UGV 开始执行任务【QR001】：经纬度 ...`：仅观察到同 task_id 的 EXECUTING 后输出；READY 或 BLOCKED 不冒充开始执行。
- `UGV 任务完成！【QR001】：经纬度 ...`：仅同任务最终结果的 Action=3、停车确认、合格RTK/残差与同一停车窗口均通过。
- 拒绝、外部 hold、回执超时、接收端重启、证据不完整分别记录；不因这些情况自动 release 或重发。

BT 节点 halt/飞行任务中止不会暗中发送 UGV CANCEL。小车通信中断继续原任务的V1行为不变；需要取消时由唯一命令出口明确操作。

## 构建依赖

消费者 `find_package(catkin REQUIRED COMPONENTS ag_client ...)`，并链接 `${catkin_LIBRARIES}`；本包导出 `ag_client_cpp` 与头文件。依赖 roscpp、正式消息、JsonCpp 和 OpenSSL Crypto；应用不需要复制 JSON/序号/回执逻辑。
