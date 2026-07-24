"""K230 -> STM32 单向UART物理链路测试。

用途：
    在接入摄像头主循环之前，单独验证K230 UART1发送、接线、STM32
    USART1中断接收以及11字节V1协议解析。

CanMV API：machine.FPIOA、FPIOA.set_function()、machine.UART、
          UART.write()、UART.deinit()。
硬件：Yahboom K230 12Pin的IO9(UART1_TXD)连接STM32 PA10(USART1_RX)，
      两块板必须共地；不要连接K230 4Pin接口的5V脚。
兼容性：构造方式依据CanMV K230 UART官方API；已在Yahboom v1.8.0上
        验证UART1初始化、11字节帧发送和STM32端持续解析。
"""

import time
from machine import FPIOA, UART


BAUDRATE = 115200
SEND_PERIOD_MS = 50       # 20 Hz，与当前视觉帧率接近
PRINT_INTERVAL_FRAMES = 20

FRAME_HEADER_0 = 0xAA
FRAME_HEADER_1 = 0x55
FRAME_VERSION = 1
FLAG_BALL_VALID = 0x01
FLAG_BALL_SAFE = 0x02

# 固定测试数据：error_px = 361 - ball_x = 123。
# STM32成功解析后，应看到error_px=123、ball_x=238、valid=1、safe=1。
TEST_ERROR_PX = 123
TEST_BALL_X = 238


def _write_u16_le(buffer, offset, value):
    """把16位整数按小端序写入bytearray，不依赖struct模块。"""
    value &= 0xFFFF
    buffer[offset] = value & 0xFF
    buffer[offset + 1] = (value >> 8) & 0xFF


def build_test_frame(frame_id):
    """创建STM32当前V1解析器需要的固定11字节测试帧。"""
    frame = bytearray(11)
    frame[0] = FRAME_HEADER_0
    frame[1] = FRAME_HEADER_1
    frame[2] = FRAME_VERSION
    frame[3] = FLAG_BALL_VALID | FLAG_BALL_SAFE
    _write_u16_le(frame, 4, frame_id)
    _write_u16_le(frame, 6, TEST_ERROR_PX)
    _write_u16_le(frame, 8, TEST_BALL_X)

    checksum = 0
    for index in range(2, 10):
        checksum ^= frame[index]
    frame[10] = checksum
    return frame


def main():
    uart = None
    frame_id = 0

    try:
        # UART构造函数会同时检查TX和RX是否已完成引脚复用。
        # Yahboom默认已把IO9设为UART1_TXD，但IO10仍可能处于GPIO模式，
        # 所以这里显式配置两根内部信号，避免出现“UART(1) rx not configured”。
        # 本次物理接线仍然只使用IO9/TX；IO10/RX不需要接到STM32。
        fpioa = FPIOA()
        fpioa.set_function(9, FPIOA.UART1_TXD)
        fpioa.set_function(10, FPIOA.UART1_RXD)

        uart = UART(
            UART.UART1,
            baudrate=BAUDRATE,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )

        print("uart_link_test_started")
        print("uart=UART1 baud=115200 format=8N1 period_ms=50")
        print("expected: valid=1 safe=1 error_px=123 ball_x=238")

        while True:
            frame = build_test_frame(frame_id)
            written = uart.write(frame)

            if frame_id % PRINT_INTERVAL_FRAMES == 0:
                print("tx frame_id={} bytes={}".format(frame_id, written))

            frame_id = (frame_id + 1) & 0xFFFF
            time.sleep_ms(SEND_PERIOD_MS)

    except KeyboardInterrupt:
        print("uart_link_test_stopped")
    except Exception as exc:
        print("uart_link_test_error: {}".format(exc))
        raise
    finally:
        if uart is not None:
            uart.deinit()


main()
