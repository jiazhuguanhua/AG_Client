# 协议与边界

- 默认命令 `/air_ground/command`，正式 `slicer_air_ground_msgs/Command`，非锁存队列 1；反馈 `/air_ground/state`，`std_msgs/String` JSON。
- V1、当前 `server_id`、正 uint64 全局递增序号、UUID 请求身份。SET 分配新任务身份，其余操作指向当前未结束任务。暂停期间更新仍由车端保持暂停。
- 新请求严格校验整批点；不提供任意原始命令透传。replay 保留所有已保存字段，旧 boot 在发布前拒绝；旧序号是否为重复请求由车端判定。
- 严格 JSON 拒绝重复键和非有限数字；纬经度有限且范围合法；未提供高度/yaw 用 JSON null，转成 ROS `has_* = false` 及默认值。非 SET `final_goal=null` 转完整默认 GeoPoint。
- 状态新鲜度同时检查本机 monotonic 接收年龄和车端 ROS `updated_stamp`；拒绝未来时间。等待采用 monotonic 时间，不会因 `/clock` 停止而无限阻塞。`execution_updated_stamp` 等执行证据原样保留，发布新鲜不代表执行线程正常。
- 回执匹配 server、request、sequence、task、operation，必须显式布尔 accepted。超时、发布异常、发布后重启、入口传输异常归为未知结果，不自动重试或取消。

## 已按车端源码验证的文件格式

已对照用户提供的 `Online_Navigation_with_RTK.zip`，直接调用其 `capture_target()` 生成采点文件，并用双方 loader 交叉验证；正式消息定义与 MD5 相同。仍未采集实车数据。

车端正式采点文件根层包含 `kind=slicer_geographic_target`、整数 `version=1`、`datum=WGS84`、`physical_reference=base_link_horizontal`，以及有限纬经度/椭球高。`load_target()` 直接读取且不再次补偿；额外支持 `point` 对象是本包扩展，不能称为车端标准格式。文件内非空 yaw 拒绝；最终朝向用 API/CLI 明确提供。车端高度注释说明 Up 可能仍是天线值，不是实测 base_link 三维高度。

保存请求是八个 Command 字段组成的 JSON（非 SET final_goal 为 null），通过独占新建、0600 权限和 fsync 写入后才发布。已验证与本次 ZIP 车端 CLI 的文件可双向互换。replay 不校验“文件是否人为改过”，调用者必须保存原件。

## 执行与完成观察

新 SET/REPLACE/RESUME 发布前要求 `execution_updated_stamp` 新鲜、启动已检查、无取消屏障且参考语义一致，默认2秒。它不证明 GNSS/Mux/规划条件满足，最终仍以车端回执及运行状态为准。PAUSE/CANCEL 保留尽力停止请求的通道；准确 replay 不加执行门禁，以便获取旧请求结果。CANCELING 阶段不再生成非 SET 新请求。

`health()` 返回执行健康摘要；`task_status(task_id)` 只解释对应任务的 `last_task_result`，核对 Action=3、停车、合格RTK、残差、地理观测属于相同停车窗口。完成证据不完整时报告 `COMPLETION_UNCONFIRMED`，快照没有该任务时报告 `UNKNOWN`；不会使用其他任务遗留 arrival，也不假装具有完整历史。

## 异常

| 类型 | 含义 |
| --- | --- |
| `ProtocolError` / `ValueError` / `OSError` | 无效输入、文件或配置；未发送 |
| `ClientError` | 未发布的本地错误，如无订阅者、无新鲜状态、并发发送、旧 boot |
| `CommandRejected` | 已发布且明确拒绝，查看 `.receipt` 和 `.request` |
| `OutcomeUnknown` | 已尝试发布但结果未知，查看 `.request` 并核查车端状态 |

`CommandRejected` 与 `OutcomeUnknown` 都继承 `ClientError`；捕获时应先捕获二者，再捕获基类。`.published` 是发送是否已经发生/尝试的标记，不证明车端已收到。

客户端不实现失联运动租约、车端状态机、控制 hold 解除或导航坐标转换。实际执行及停车/到达测量仍由车端完成，客户端只核查公开证据。ROS1 网络连通及唯一命令出口由部署保证。
