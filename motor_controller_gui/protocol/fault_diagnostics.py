"""Versioned full firmware fault masks; do not infer chip-specific causes."""
import struct

FAULT_NAMES = {
    0: '母线欠压', 1: '母线过压', 2: '软件过流', 3: '硬件过流',
    4: '电机过温', 5: '驱动过温', 6: '编码器异常', 7: '位置跟随误差',
    8: '电机轴硬超速', 9: '电角对齐失败', 10: '校准失败', 11: '控制饱和',
    12: '控制看门狗', 13: '参数非法', 14: '缺相', 15: '短路', 16: '堵转',
    17: '三相电流不平衡', 18: '电流采样异常', 19: '通信超时', 20: '存储异常',
    21: '制动异常', 22: '去饱和保护', 23: '驱动nFAULT有效（具体芯片原因未区分）',
    24: '逻辑电源欠压', 27: 'W相开路', 28: '用户故障指令',
}


def describe_faults(bits):
    return '、'.join(FAULT_NAMES.get(i, f'未知故障位{i}') for i in range(32)
                    if bits & (1 << i)) or '正常'


def parse_fault_diagnostics(payload):
    if len(payload) < 12 or payload[0] != 1:
        return None
    version, enabled, mode, state, status, latched = struct.unpack_from('>BBBBII', payload)
    bits = status | latched
    return dict(enable=bool(enabled), mode=mode, fault_fsm=state,
                fault_status=status, fault_latched=latched, error_code=bits,
                error_name=describe_faults(bits), fault_detail=True)
