# 协议与边界

- 默认命令 `/air_ground/command`，正式 `slicer_air_ground_msgs/Command`，非锁存队列 1；反馈 `/air_ground/state`，`std_msgs/String` JSON。
- V1、当前 `server_id`、正 uint64 全局递增序号、UUID 请求身份。SET 分配新任务身份，其余操作指向当前未结束任务。暂停期间更新仍由车端保持暂停。
- 新请求严格校验整批点；不提供任意原始命令透传。replay 保留所有已保存字段，旧 boot 在发布前拒绝；旧序号是否为重复请求由车端判定。
- 严格 JSON 拒绝重复键和非有限数字；纬经度有限且范围合法；未提供高度/yaw 用 JSON null，转成 ROS `has_* = false` 及默认值。非 SET `final_goal=null` 转完整默认 GeoPoint。
- 状态新鲜度同时检查本机 monotonic 接收年龄和车端 ROS `updated_stamp`；拒绝未来时间。等待采用 monotonic 时间，不会因 `/clock` 停止而无限阻塞。`execution_updated_stamp` 等执行证据原样保留，发布新鲜不代表执行线程正常。
- 回执匹配 server、request、sequence、task、operation，必须显式布尔 accepted。超时、发布异常、发布后重启、入口传输异常归为未知结果，不自动重试或取消。

## 文件格式的已知限制

用户暂时没有 SDK 或真实 `geographic.py capture` 示例。消息定义按字段表实现并核对官方给出的 MD5；**采点文件格式尚不能宣称与实际 capture 输出兼容**。

当前 `load_target()` 要求根层元数据 `kind=slicer_geographic_target`、整数 `version=1`、`datum=WGS84`、`physical_reference=base_link_horizontal`；有限纬经度和椭球高位于根层或唯一 `point` 对象，拒绝二义结构。文件内非空 yaw 拒绝；最终朝向用 API/CLI 明确提供，途经点不可提供朝向。拿到真实样例后应核对字段位置；在此之前可用显式 WGS84 API/CLI。

保存请求是八个 Command 字段组成的 JSON（非 SET final_goal 为 null），通过独占新建、0600 权限和 fsync 写入后才发布。此文件供本客户端 replay；未承诺与队友 CLI 的保存文件封装格式互换。replay 不校验“文件是否人为改过”，调用者必须保存原件。

## 异常

| 类型 | 含义 |
| --- | --- |
| `ProtocolError` / `ValueError` / `OSError` | 无效输入、文件或配置；未发送 |
| `ClientError` | 未发布的本地错误，如无订阅者、无新鲜状态、并发发送、旧 boot |
| `CommandRejected` | 已发布且明确拒绝，查看 `.receipt` 和 `.request` |
| `OutcomeUnknown` | 已尝试发布但结果未知，查看 `.request` 并核查车端状态 |

`CommandRejected` 与 `OutcomeUnknown` 都继承 `ClientError`；捕获时应先捕获二者，再捕获基类。`.published` 是发送是否已经发生/尝试的标记，不证明车端已收到。

客户端不实现失联运动租约、车端状态机、控制 hold 解除、导航坐标转换或最终到达判断。这些继续由原车端执行。ROS1 网络连通及唯一命令出口由部署保证。
