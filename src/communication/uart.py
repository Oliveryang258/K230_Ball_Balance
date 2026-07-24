"""K230视觉测量UART发送器。

CanMV API：machine.FPIOA、FPIOA.set_function()、machine.UART、
          UART.write()、UART.deinit()。
硬件：Yahboom K230 12Pin，IO9/UART1_TXD接STM32 PA10/USART1_RX，
      K230与STM32必须共地，不连接两端VCC。
兼容性：UART1、IO9/IO10复用和固定11字节发送已在Yahboom CanMV
        v1.8.0上完成实机验证；真实视觉数据集成仍需本轮联调确认。
"""


FRAME_HEADER_0 = 0xAA
FRAME_HEADER_1 = 0x55
FRAME_VERSION = 1
FRAME_SIZE = 11

FLAG_BALL_VALID = 0x01
FLAG_BALL_SAFE = 0x02


def _write_u16_le(buffer, offset, value):
    """将有符号或无符号16位整数按小端序写入缓冲区。"""
    value &= 0xFFFF
    buffer[offset] = value & 0xFF
    buffer[offset + 1] = (value >> 8) & 0xFF


def encode_measurement(
    frame_id,
    ball_valid,
    ball_safe,
    error_px,
    ball_x,
    buffer=None,
):
    """编码STM32 V1协议帧；传入buffer时不产生新的帧缓冲区分配。

    无效检测必须编码为valid=0、safe=0、error_px=0、ball_x=-1，避免
    下位机把上一帧坐标或零误差误认为新的有效测量。
    """
    if buffer is None:
        buffer = bytearray(FRAME_SIZE)
    elif len(buffer) != FRAME_SIZE:
        raise ValueError("UART frame buffer must be 11 bytes")

    ball_valid = bool(ball_valid)
    ball_safe = bool(ball_safe) and ball_valid

    if not ball_valid:
        error_px = 0
        ball_x = -1

    flags = 0
    if ball_valid:
        flags |= FLAG_BALL_VALID
    if ball_safe:
        flags |= FLAG_BALL_SAFE

    buffer[0] = FRAME_HEADER_0
    buffer[1] = FRAME_HEADER_1
    buffer[2] = FRAME_VERSION
    buffer[3] = flags
    _write_u16_le(buffer, 4, int(frame_id))
    _write_u16_le(buffer, 6, int(error_px))
    _write_u16_le(buffer, 8, int(ball_x))

    checksum = 0
    for index in range(2, 10):
        checksum ^= buffer[index]
    buffer[10] = checksum
    return buffer


class VisionUart:
    """预分配单帧缓冲区并通过UART1发送视觉测量。"""

    def __init__(self, uart_id, baudrate, tx_pin, rx_pin, enabled=False):
        self.enabled = bool(enabled)
        self._uart = None
        self._frame_id = 0
        self._frame = bytearray(FRAME_SIZE)

        if not self.enabled:
            return

        # 延迟导入machine，保证PC端可以单独测试encode_measurement()。
        from machine import FPIOA, UART

        uart_name = "UART{}".format(int(uart_id))
        tx_name = "UART{}_TXD".format(int(uart_id))
        rx_name = "UART{}_RXD".format(int(uart_id))
        if not hasattr(UART, uart_name):
            raise RuntimeError("UART channel not found: {}".format(uart_name))
        if not hasattr(FPIOA, tx_name) or not hasattr(FPIOA, rx_name):
            raise RuntimeError("FPIOA UART function missing: {}/{}".format(tx_name, rx_name))

        fpioa = FPIOA()
        fpioa.set_function(int(tx_pin), getattr(FPIOA, tx_name))
        # CanMV UART构造函数会同时检查RX映射；即使当前只发送也必须配置。
        fpioa.set_function(int(rx_pin), getattr(FPIOA, rx_name))

        self._uart = UART(
            getattr(UART, uart_name),
            baudrate=int(baudrate),
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )

    @property
    def frame_id(self):
        return self._frame_id

    def send_measurement(self, ball_valid, ball_safe, error_px, ball_x):
        """发送一帧；返回UART.write()写入的字节数，禁用时返回0。"""
        if not self.enabled or self._uart is None:
            return 0

        encode_measurement(
            self._frame_id,
            ball_valid,
            ball_safe,
            error_px,
            ball_x,
            self._frame,
        )
        written = self._uart.write(self._frame)
        self._frame_id = (self._frame_id + 1) & 0xFFFF
        return written

    def deinit(self):
        if self._uart is not None:
            self._uart.deinit()
            self._uart = None
