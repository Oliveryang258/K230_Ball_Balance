"""UART output interface; disabled during the visual-only milestone.

APIs: machine.UART only when explicitly enabled.
Hardware: Yahboom K230 12Pin UART connected to a 3.3 V-compatible MCU.
Runtime: CanMV K230 Yahboom v1.8.0; UART ID and pin mapping are unverified.
"""


def format_measurement(normalized_error, position=None, valid=True):
    """Build the temporary integer CSV payload without accessing hardware."""
    if valid and normalized_error is not None:
        valid_int = 1
        error_milli = int(round(normalized_error * 1000.0))
        if position is None:
            ball_x, ball_y = -1, -1
        else:
            ball_x, ball_y = int(position[0]), int(position[1])
    else:
        valid_int = 0
        error_milli = 0
        ball_x, ball_y = -1, -1

    return "BALL,{},{},{},{}\n".format(valid_int, error_milli, ball_x, ball_y)


class VisionUart:
    """Small wrapper that keeps machine-specific UART code out of main.py."""

    def __init__(self, uart_id, baudrate, enabled=False):
        self.enabled = enabled
        self._uart = None
        if enabled:
            from machine import UART
            self._uart = UART(uart_id, baudrate=baudrate)

    def send_measurement(self, normalized_error, position=None, valid=True):
        """Send a readable temporary integer protocol line.

        The final framed/binary protocol will be designed after visual validation.
        """
        if not self.enabled or self._uart is None:
            return False
        self._uart.write(format_measurement(normalized_error, position, valid))
        return True

    def deinit(self):
        if self._uart is not None and hasattr(self._uart, "deinit"):
            self._uart.deinit()
