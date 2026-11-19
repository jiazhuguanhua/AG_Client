# V1 退役记录

本文说明旧 V1 工具的退役范围。日常联调和部署全部使用 [V2 操作指南](../implementation_guide.md)。

V1 的命令行入口已退役，`test_menu.py` 已改为 V2。旧 C++ 库与消息定义暂留供尚未迁移的 maxt 构建使用，其通信接口与 V2 不互通。协议细节仅存档在 [V1 协议](protocol.md)。
