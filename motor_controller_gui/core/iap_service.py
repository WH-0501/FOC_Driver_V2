"""IAP 固件升级（单电机，遵循 CAN-FD 协议与流程图时序）。"""



from __future__ import annotations



import threading

import time

from dataclasses import dataclass

from pathlib import Path

from typing import Callable, Optional



from core.motor_client import MotorClient

from protocol import (

    CMD,

    IAP_MAX_FILE_SIZE,

    IAP_V5_CHUNK_SIZE,

    format_asix_header_log,

    prepare_iap_firmware_image,

    parse_iap_status_payload,

    parse_fw_version_payload,

    split_iap_v5_image_chunks,

)

from protocol.constants import (

    IAP_DATA_FRAME_INTERVAL_S,

    IAP_POST_VERIFY_QUERY_S,

    IAP_START_ERASE_POLL_S,

    IAP_START_ERASE_WAIT_S,

    IAP_FIRMWARE_TIMEOUT_MS,

    IAP_START_DATA_BURST_FRAMES,

    IAP_START_DATA_DEADLINE_S,

    IAP_START_FC_POLL_S,

    IAP_START_MIN_RESPONSE_WAIT_S,

    IAP_STATUS_QUERY_TIMEOUT_S,

    IAP_V5_IDLE_WAIT_S,

    IAP_V5_RECOVERY_WAIT_S,

)

from protocol.time_util import format_mono_delta_ms
from protocol.version import (

    iap_is_complete_state,

    iap_is_dex_v5_success,

    iap_is_error_state,

    iap_is_receiving_state,

    iap_wire_uses_dex_v5_format,

)





@dataclass

class IapProgress:

    phase: str = ""

    sent: int = 0

    total: int = 0

    message: str = ""





class IapUpgradeService:

    """

    流程（CAN-FD通信协议.md + dex-con iap.c）：

    ENTER → START(同步擦除) → 等 FC(state=2,err=0x80) → DATA → END → VERIFY → RESET

    """



    def __init__(

        self,

        client: MotorClient,

        motor_id: int,

        log_fn: Optional[Callable[[str], None]] = None,

    ) -> None:

        self.client = client

        self.motor_id = motor_id

        self._log = log_fn or (lambda _msg: None)

        self._cancel = threading.Event()

        self._thread: Optional[threading.Thread] = None

        self._progress_cb: Optional[Callable[[IapProgress], None]] = None

        self._done_cb: Optional[Callable[[bool, str], None]] = None



    @property

    def running(self) -> bool:

        return self._thread is not None and self._thread.is_alive()



    def cancel(self) -> None:

        self._cancel.set()



    def start(

        self,

        firmware_path: Path,

        progress_cb: Callable[[IapProgress], None],

        done_cb: Callable[[bool, str], None],

    ) -> None:

        if self.running:

            done_cb(False, "升级正在进行中")

            return

        self._cancel.clear()

        self._progress_cb = progress_cb

        self._done_cb = done_cb

        self._thread = threading.Thread(

            target=self._run,

            args=(firmware_path,),

            daemon=True,

        )

        self._thread.start()



    def _emit(self, phase: str, sent: int, total: int, message: str) -> None:

        if self._progress_cb:

            self._progress_cb(IapProgress(phase=phase, sent=sent, total=total, message=message))



    def _log_step(self, msg: str) -> None:

        self._log(msg)



    def _check_cancel(self) -> None:

        if self._cancel.is_set():

            raise RuntimeError("已取消")



    def _drain_iap_responses(self, timeout: float = 0.2) -> None:

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:

            payload = self.client.wait_iap_response(

                self.motor_id,

                CMD.IAP_STATUS,

                min(0.05, max(0.0, deadline - time.monotonic())),

            )

            if payload is None:

                break

        self.client.clear_iap_wait()



    @staticmethod

    def _dex_v5_idle(parsed: dict, payload: bytes) -> bool:

        if len(payload) < 2:

            return False

        state = int(parsed.get("iap_state", -1))

        raw_err = payload[1]

        return (

            iap_wire_uses_dex_v5_format(state, raw_err)

            and state == 0

            and iap_is_dex_v5_success(raw_err)

        )



    def _fetch_iap_status(self, timeout: float = IAP_STATUS_QUERY_TIMEOUT_S) -> Optional[tuple[dict, bytes]]:

        if not self.client.iap_query_status(self.motor_id):

            return None

        raw = self.client.wait_iap_response(self.motor_id, CMD.IAP_STATUS, timeout)

        if raw is None:

            return None

        parsed = parse_iap_status_payload(self.client.protocol_version, raw)

        return parsed, raw



    def _wait_iap_idle(self, timeout: float = IAP_V5_IDLE_WAIT_S) -> None:
        """失败后等待从机回到 IDLE；被动监听，不主动 FC 查询（避免 0x8F/0x8C 风暴）。"""
        deadline = time.monotonic() + timeout
        last_log = 0.0
        self._log_step(f"IAP：等待从机恢复空闲（最长 {int(timeout)}s）…")
        while time.monotonic() < deadline:
            self._check_cancel()
            payload = self.client.wait_iap_response(
                self.motor_id,
                CMD.IAP_STATUS,
                min(0.5, max(0.05, deadline - time.monotonic())),
            )
            if payload is not None:
                parsed = parse_iap_status_payload(self.client.protocol_version, payload)
                if self._dex_v5_idle(parsed, payload):
                    self._log_step("IAP：从机已恢复空闲")
                    return
                now = time.monotonic()
                if now - last_log >= 5.0:
                    err = payload[1] if len(payload) >= 2 else 0
                    hint = "仍在恢复" if err in (0x8F, 0x8C, 0x87) else "未就绪"
                    self._log_step(
                        f"IAP：{hint} — {parsed.get('iap_state_name', '?')} / "
                        f"{parsed.get('error_name', '?')}"
                    )
                    last_log = now
            else:
                time.sleep(0.2)
        raise RuntimeError(f"等待 IAP 空闲超时（{int(timeout)}s），请断电重启电机后再试")



    def _status_failed(self, parsed: dict, step: str) -> None:

        raise RuntimeError(

            f"{step}：{parsed.get('iap_state_name', '?')} — {parsed.get('error_name', '?')}"

        )



    @staticmethod

    def _is_receiving_ok(state: int, raw_err: int) -> bool:

        return (

            iap_wire_uses_dex_v5_format(state, raw_err)

            and iap_is_dex_v5_success(raw_err)

            and state != 6

            and iap_is_receiving_state(state)

        )



    def _peek_iap_status(self) -> Optional[tuple[dict, bytes]]:
        payload = self.client.wait_iap_response(self.motor_id, CMD.IAP_STATUS, 0.0)
        if payload is None:
            return None
        parsed = parse_iap_status_payload(self.client.protocol_version, payload)
        return parsed, payload

    def _fire_data_on_start_fc(self, start_sent_at: float, data_chunks: list[bytes]) -> int:
        """
        收到擦除 FC 后零延迟连发 DATA（先于日志），并丢弃队列中滞后的错误 FC。
        返回已连发 DATA 帧数。
        """
        burst_n = min(IAP_START_DATA_BURST_FRAMES, len(data_chunks))
        t_send = time.monotonic()
        sent_burst = 0
        if burst_n > 0:
            sent_burst = self.client.iap_send_data_burst(
                data_chunks[:burst_n], log_first=True, log_async=True
            )
        send_ms = format_mono_delta_ms(t_send)
        fc_ms = format_mono_delta_ms(start_sent_at, t_send)
        drained = self.client.drain_iap_status(self.motor_id)
        self._log_step(
            f"IAP：FC→DATA 连发 {sent_burst} 帧（START→FC {fc_ms} ms，CAN 提交 {send_ms} ms）"
        )
        if drained:
            self._log_step(f"IAP：已丢弃滞后 FC {drained} 条（含可能的 0x87）")
        fc_elapsed_ms = (t_send - start_sent_at) * 1000
        if fc_elapsed_ms > IAP_FIRMWARE_TIMEOUT_MS:
            self._log_step(
                f"IAP：擦除 FC 在 {fc_elapsed_ms:.3f} ms 到达，已超过固件 IAP_TIMEOUT="
                f"{IAP_FIRMWARE_TIMEOUT_MS} ms；擦除耗时过长，需增大 iap.c 中 IAP_TIMEOUT 或优化擦除"
            )
        if sent_burst <= 0 and burst_n > 0:
            raise RuntimeError("IAP_DATA：收到 FC 后首批连发失败")
        return sent_burst

    def _wait_start_erase_fc(
        self, start_sent_at: float, total: int, data_chunks: list[bytes]
    ) -> int:
        """
        等 START FC(state=2,err=0x80) 后立即连发 DATA。
        iap.c：IAP_TIMEOUT 从 START 起算且含擦除；FC 若 >3s 则设备可能同时上报 0x87。
        """
        deadline = start_sent_at + IAP_START_ERASE_WAIT_S
        min_ok_at = start_sent_at + IAP_START_MIN_RESPONSE_WAIT_S
        last_hint = start_sent_at
        version = self.client.protocol_version

        self._emit("start", 0, total, "等待 Flash 擦除…")

        while True:
            peek = self._peek_iap_status()
            if peek is None:
                break
            parsed, payload = peek
            state = int(parsed.get("iap_state", -1))
            raw_err = payload[1] if len(payload) >= 2 else 0
            if self._is_receiving_ok(state, raw_err):
                return self._fire_data_on_start_fc(start_sent_at, data_chunks)

        while time.monotonic() < deadline:
            self._check_cancel()
            now = time.monotonic()
            remaining = deadline - now
            payload = self.client.wait_iap_response(
                self.motor_id,
                CMD.IAP_STATUS,
                min(IAP_START_FC_POLL_S, max(0.005, remaining)),
            )
            if payload is not None:
                parsed = parse_iap_status_payload(version, payload)
                state = int(parsed.get("iap_state", -1))
                raw_err = payload[1] if len(payload) >= 2 else 0
                if self._is_receiving_ok(state, raw_err):
                    return self._fire_data_on_start_fc(start_sent_at, data_chunks)
                if now >= min_ok_at and state == 6 and iap_is_error_state(version, state, raw_err):
                    if raw_err == 0x87:
                        fc_ms = format_mono_delta_ms(start_sent_at, now)
                        raise RuntimeError(
                            f"IAP_START：超时(0x87)，START→错误 FC {fc_ms} ms。"
                            f"固件 IAP_TIMEOUT={IAP_FIRMWARE_TIMEOUT_MS} ms 含擦除时间；"
                            f"本次擦除可能≥3s，需增大 iap.c IAP_TIMEOUT。"
                        )
                    self._status_failed(parsed, "IAP_START")

            if now - last_hint >= 2.0:
                elapsed = format_mono_delta_ms(start_sent_at, now)
                self._emit("start", 0, total, f"Flash 擦除中…已等待 {elapsed} ms")
                last_hint = now

        raise RuntimeError(f"IAP_START：等待擦除 FC 超时（{int(IAP_START_ERASE_WAIT_S)}s）")



    def _check_iap_error(self) -> None:

        payload = self.client.poll_iap_error(self.motor_id, timeout=0.0)

        if payload is None:

            return

        parsed = parse_iap_status_payload(self.client.protocol_version, payload)

        raise RuntimeError(f"IAP 失败：{parsed.get('error_name', '?')}")



    def _transfer(self, image: bytes, total: int) -> None:

        chunks = split_iap_v5_image_chunks(image, IAP_V5_CHUNK_SIZE)

        if not chunks or len(chunks[0]) < 32:

            raise RuntimeError("IAP_START 首帧须含完整固件头（≥32B）及数据")



        data_frames = len(chunks) - 1

        self._emit("enter", 0, total, "进入 IAP 模式…")

        self._log_step(

            f"IAP：Node-ID={self.motor_id}，ASIX 整包 {total} B，"

            f"START 1 帧 + DATA {data_frames} 帧（每帧 {IAP_V5_CHUNK_SIZE}B）"

        )



        self._log_step(format_asix_header_log(chunks[0]))

        enter_cnt, _start_cnt, start_sent_at, can_elapsed = self.client.iap_enter_start_burst(
            self.motor_id, chunks[0]
        )
        if enter_cnt < 0:
            raise RuntimeError("发送 IAP_ENTER / IAP_START 失败")
        self._log_step(
            f"IAP：ENTER→START 已发送（ZCAN 提交 {can_elapsed * 1000:.3f} ms）"
        )
        self._log_step(
            f"IAP：等待 START FC（擦除完成 state=2 err=0x80），最长 {IAP_START_ERASE_WAIT_S:.0f}s…"
        )

        data_chunks = chunks[1:]
        sent_burst = self._wait_start_erase_fc(start_sent_at, total, data_chunks)
        burst_n = sent_burst

        sent = len(chunks[0])
        if sent_burst > 0:
            sent += sum(len(c) for c in data_chunks[:sent_burst])
            data_chunks = data_chunks[sent_burst:]
        self._emit("data", sent, total, "传输固件 DATA…")

        for idx, chunk in enumerate(data_chunks, start=1 + burst_n):

            self._check_cancel()

            self._check_iap_error()

            log_frame = idx == (1 + burst_n) or idx == data_frames

            if not self.client.iap_data(self.motor_id, chunk, log=log_frame):

                raise RuntimeError(f"IAP_DATA 发送失败（第 {idx}/{data_frames} 帧）")

            sent += len(chunk)

            if idx % 64 == 0 or idx == data_frames:

                self._emit("data", sent, total, f"传输中 {sent}/{total} B")

            if IAP_DATA_FRAME_INTERVAL_S > 0:

                time.sleep(IAP_DATA_FRAME_INTERVAL_S)



        self._check_iap_error()

        self._emit("end", total, total, "结束传输…")

        if not self.client.iap_end(self.motor_id):

            raise RuntimeError("发送 IAP_END 失败")



        err_payload = self.client.poll_iap_error(self.motor_id, timeout=2.0)

        if err_payload is not None:

            parsed = parse_iap_status_payload(self.client.protocol_version, err_payload)

            raise RuntimeError(f"IAP_END 失败：{parsed.get('error_name', '?')}")



        self._emit("verify", total, total, "校验固件…")

        if not self.client.iap_verify(self.motor_id):

            raise RuntimeError("发送 IAP_VERIFY 失败")



        deadline = time.monotonic() + IAP_POST_VERIFY_QUERY_S

        verified = False

        while time.monotonic() < deadline:

            self._check_cancel()

            fetched = self._fetch_iap_status(IAP_STATUS_QUERY_TIMEOUT_S)

            if fetched:

                parsed, raw = fetched

                state = int(parsed.get("iap_state", -1))

                err = raw[1] if len(raw) >= 2 else 0

                version = self.client.protocol_version

                if iap_is_error_state(version, state, err):

                    self._status_failed(parsed, "IAP_VERIFY")

                if (

                    iap_wire_uses_dex_v5_format(state, err)

                    and iap_is_complete_state(state)

                    and iap_is_dex_v5_success(err)

                ):

                    verified = True

                    break

            time.sleep(0.2)

        if verified:

            self._log_step("IAP：校验通过")

        else:

            self._log_step("IAP：未收到校验 FC，继续 RESET")



        self._emit("reset", total, total, "复位 MCU…")

        if not self.client.iap_reset(self.motor_id):

            raise RuntimeError("发送 IAP_RESET 失败")



    def _run(self, firmware_path: Path) -> None:

        ok = False

        msg = ""

        try:

            if not self.client.can.manager.is_running:

                raise RuntimeError("请先打开设备并启动 CAN 通道")



            raw = firmware_path.read_bytes()

            image, prep_note = prepare_iap_firmware_image(raw)

            total = len(image)

            if total <= 0:

                raise RuntimeError("固件文件为空")

            if total > IAP_MAX_FILE_SIZE:

                raise RuntimeError(f"固件过大: {total} B（最大 {IAP_MAX_FILE_SIZE} B）")

            if prep_note:

                self._log_step(f"IAP：{prep_note}")

            if image[:4] != b"ASIX":

                raise RuntimeError("IAP 镜像缺少 ASIX 固件头")



            self._drain_iap_responses(0.15)

            self.client.begin_iap_session(self.motor_id)

            try:

                self._transfer(image, total)

            except RuntimeError as exc:

                err_text = str(exc)

                retry = any(k in err_text for k in ("0x87", "超时", "0x8F", "丢帧"))

                if not retry:

                    raise

                self._log_step(

                    f"IAP：失败（{err_text}），等待 {IAP_V5_RECOVERY_WAIT_S:.0f}s 后重试…"

                )

                self.client.clear_iap_wait()

                time.sleep(IAP_V5_RECOVERY_WAIT_S)

                self._wait_iap_idle()

                self.client.begin_iap_session(self.motor_id)

                self._transfer(image, total)



            ok = True

            msg = "固件升级完成，设备正在复位"

            self._log_step(msg)

        except Exception as exc:

            msg = str(exc)

            self._log_step(f"IAP 失败: {msg}")

        finally:

            self.client.end_iap_session()

            self.client.clear_iap_wait()

            if self._done_cb:

                self._done_cb(ok, msg)



    @staticmethod

    def query_firmware_version(

        client: MotorClient,

        motor_id: int,

        timeout: float = 3.0,

    ) -> str:

        client.clear_iap_wait()

        if not client.iap_get_version(motor_id):

            raise RuntimeError("发送 IAP_FW_VERSION 失败")

        payload = client.wait_iap_response(motor_id, CMD.IAP_FW_VERSION, timeout)

        if payload is None:

            raise RuntimeError("读取固件版本超时")

        return parse_fw_version_payload(payload)


