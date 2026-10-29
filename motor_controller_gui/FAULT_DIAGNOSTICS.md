# 周期故障字节（V2）

V2 固件不再使用 0xB7 主动诊断帧。故障信息仅通过 **周期上报 `payload[15]` error 字节** 传递：

- **BIT7=0**：保护子码（与 V1 线格式一致，0x00~0x0F 见协议表，0x10~ 为 V2 扩展）
- **BIT7=1**：IAP 子码（与 V1 相同，一般仅在 IAP 场景出现）

固件 `can_fd_codec_fault_to_error` 对 `fault_t` 按固定优先级取 **主因** 映射为一条保护子码；上位机用 `parse_error_byte` / `V2_PROTECTION_WIRE_ERROR_NAMES` 解析。

多 fault 同时置位时线上只体现优先级最高的一类。
