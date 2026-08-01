"""STM32回传遥测的TF卡二进制记录器。

用途：
    STM32把视觉、控制、舵机和车辆运动状态组合成固定64/106字节遥测帧，
    K230在UART主循环中非阻塞接收，CRC校验通过后批量写入TF卡。

CanMV/K230 API：
    - UART.any() / UART.read()：非阻塞串口读取；
    - open()、file.write()、file.close()：MicroPython文件接口；
    - os.sync()：固件提供时定期提交文件系统。

硬件：
    - Yahboom K230 12Pin，插入可写TF卡；
    - STM32 PA9/USART1_TX -> K230 IO10/UART1_RX；
    - 两块板必须共地，不连接两端VCC。
"""

import os


HEADER_0 = 0x54
HEADER_1 = 0x4D
VER_V2 = 2
VER_V3 = 3
PKT_V2 = 64
PKT_V3 = 106
CRC_V2 = 62
CRC_V3 = 104


def _crc16(data, offset, length):
    """CRC-16/CCITT-FALSE，多项式0x1021，初值0xFFFF。"""
    crc = 0xFFFF
    end = offset + length
    i = offset
    while i < end:
        crc ^= data[i] << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        i += 1
    return crc


class TelemetryLogger:
    """非阻塞接收、校验并批量记录STM32遥测帧（V2=64B / V3=106B）。"""

    def __init__(
        self,
        path,
        rx_chunk=256,
        write_block=1024,
        sync_interval=4,
        enabled=False,
    ):
        self.enabled = bool(enabled)
        self.active = False
        self.path = path
        self._file = None
        self._rx = bytearray(int(rx_chunk))
        self._pkt = bytearray(PKT_V3)       # 解析缓冲区，按最大包分配
        self._pkt_idx = 0
        self._pkt_size = 0
        self._blk = bytearray(int(write_block))
        self._blk_off = 0
        self._blk_size = int(write_block)
        self._sync_int = max(int(sync_interval), 1)

        self.valid = 0
        self.crc_err = 0
        self.fmt_err = 0
        self.write_err = 0
        self.blocks = 0
        self.syncs = 0
        self.bytes_out = 0
        self.last_seq = -1

        if self.enabled:
            self._open()

    def _open(self):
        try:
            self._file = open(self.path, "wb")
            self.active = True
            print("telemetry_log=ON path=" + str(self.path))
        except BaseException as e:
            self._file = None
            self.active = False
            print("telemetry_log=OFF err=" + str(e))

    def _flush(self):
        """写出整个写块；块内未使用的尾部字节也一并写出（解码器会跳过）。"""
        if self._file is None:
            return
        try:
            n = self._file.write(self._blk)
            if n is None:
                n = self._blk_size
            self.bytes_out += int(n)
            self.blocks += 1
            self._blk_off = 0
            if self.blocks % self._sync_int == 0:
                self._sync()
        except BaseException as e:
            self.write_err += 1
            self.active = False
            print("telemetry_write_err=" + str(e))

    def _sync(self):
        try:
            m = getattr(self._file, "flush", None)
            if m is not None:
                m()
            s = getattr(os, "sync", None)
            if s is not None:
                s()
            else:
                self._file.close()
                self._file = open(self.path, "ab")
            self.syncs += 1
        except BaseException:
            pass

    def _save(self):
        """把已校验通过的帧放入写缓冲区。"""
        sz = self._pkt_size
        if self._blk_off + sz > self._blk_size:
            self._flush()
        end = self._blk_off + sz
        self._blk[self._blk_off:end] = self._pkt[:sz]
        self._blk_off = end
        self.valid += 1
        self.last_seq = self._pkt[4] | (self._pkt[5] << 8)

        if self._blk_off + PKT_V3 > self._blk_size:
            self._flush()

    # ── 逐字节帧同步与校验 ──────────────────────────────────
    def _feed(self, b):
        """喂入一个字节；完成一帧后自动调用 _save()。"""
        b = int(b) & 0xFF

        # 状态 0：等待帧头 0x54
        if self._pkt_idx == 0:
            if b == HEADER_0:
                self._pkt[0] = b
                self._pkt_idx = 1
            return

        # 状态 1：等待帧头 0x4D
        if self._pkt_idx == 1:
            if b == HEADER_1:
                self._pkt[1] = b
                self._pkt_idx = 2
            else:
                self._pkt_idx = 0
                if b == HEADER_0:
                    self._pkt[0] = b
                    self._pkt_idx = 1
            return

        # 状态 2+：填入包体
        self._pkt[self._pkt_idx] = b
        self._pkt_idx += 1

        # 第 3 字节：版本 → 确定包长
        if self._pkt_idx == 3:
            v = self._pkt[2]
            if v == VER_V2:
                self._pkt_size = PKT_V2
            elif v == VER_V3:
                self._pkt_size = PKT_V3
            else:
                self.fmt_err += 1
                self._pkt_idx = 0
                if b == HEADER_0:
                    self._pkt[0] = b
                    self._pkt_idx = 1
            return

        # 第 4 字节：长度字段须与版本匹配
        if self._pkt_idx == 4:
            if self._pkt[3] != self._pkt_size:
                self.fmt_err += 1
                self._pkt_idx = 0
                if b == HEADER_0:
                    self._pkt[0] = b
                    self._pkt_idx = 1
            return

        # 等收满一帧
        if self._pkt_idx < self._pkt_size:
            return

        # CRC 校验
        crc_off = CRC_V3 if self._pkt_size == PKT_V3 else CRC_V2
        expect = self._pkt[crc_off] | (self._pkt[crc_off + 1] << 8)
        actual = _crc16(self._pkt, 2, crc_off - 2)
        if actual == expect:
            self._save()
        else:
            self.crc_err += 1
        self._pkt_idx = 0

    def poll(self, uart):
        """读取UART中已到达的全部字节；返回本次新增有效帧数。"""
        if not self.active:
            return 0

        before = self.valid

        # 优先使用 readinto（CanMV K230 已验证）
        m = getattr(uart, "readinto", None)
        if m is not None:
            received = m(self._rx)
            if received is None or received <= 0:
                return 0
            i = 0
            while i < received:
                self._feed(self._rx[i])
                i += 1
            return self.valid - before

        # 回退：any() + read()
        try:
            n = uart.any()
        except BaseException:
            return 0
        if not n or n <= 0:
            return 0
        if n > len(self._rx):
            n = len(self._rx)
        try:
            data = uart.read(n)
        except BaseException:
            return 0
        if data is None:
            return 0
        for b in data:
            self._feed(b)
        return self.valid - before

    def status(self):
        return (
            "telemetry active={} frames={} seq={} blk={} sync={} bytes={} "
            "crc={} fmt={} wr={}"
        ).format(
            1 if self.active else 0,
            self.valid,
            self.last_seq,
            self.blocks,
            self.syncs,
            self.bytes_out,
            self.crc_err,
            self.fmt_err,
            self.write_err,
        )

    def close(self):
        if self._file is None:
            self.active = False
            return
        try:
            if self._blk_off > 0:
                n = self._file.write(self._blk[:self._blk_off])
                if n is None:
                    n = self._blk_off
                self.bytes_out += int(n)
                self._blk_off = 0
            m = getattr(self._file, "flush", None)
            if m is not None:
                m()
            self._file.close()
            s = getattr(os, "sync", None)
            if s is not None:
                s()
        except BaseException as e:
            self.write_err += 1
            print("telemetry_close_err=" + str(e))
        finally:
            self._file = None
            self.active = False
            print(
                "telemetry_closed frames={} bytes={} err={}".format(
                    self.valid, self.bytes_out,
                    self.crc_err + self.fmt_err + self.write_err,
                )
            )
