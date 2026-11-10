# 验证记录

构建：Windows x64，Qt 6.12.0，MinGW GCC 13.1，CMake Release。

自动验证分三层：

1. C++ QtTest：12 个实质测试加初始化/清理共 14 项通过。包括 V1/V2 golden vectors、布尔结果与 error_t 区别、SET→GET 应答别名、ASIX 升级 Demo 状态机、环形覆盖、预触发、故障前后窗口、FFT 幅值、SN 版本/回读、失能门禁和急停。
2. 原生应用 Demo 集成：启动窗口、1kHz 采集、30Hz 重绘、SaveParam → 七项参数回读 → SQLite 归档 → 31 项测试清单默认集合 → 正常退出。该流程要求结果中无 FAIL；SKIP 不算 PASS。
3. Python 扩展：统计表达式、危险语法拒绝、内嵌运行时、Virtual CAN FD 兼容进程打开/发送/关闭。

本机采样吞吐测量：100,000 次 append（含环形和故障前窗口维护）约 5ms。仅为核心缓冲微基准，不代表 USB/CAN 实物端到端吞吐或急停延迟。

详细 C++ 输出保存在 core-tests.txt。原始旧版目录 203 个文件的 SHA-256 指纹在 legacy-hashes-before.json；部署后再次核对，结果写入 delivery-check.json。

未执行任何真机使能、运动、Flash 写入或 IAP。真实硬件验收仍需按 ACCEPTANCE.md 的台架清单完成。

独立部署复核：移除 PATH 中的 Qt/MinGW，使用 release/MotorTune.exe 在 offscreen 与 Windows 原生平台各运行 Demo 集成流程，退出码均为 0。Windows 原生截图位于 screens-native，中文和曲线已人工目视检查。
