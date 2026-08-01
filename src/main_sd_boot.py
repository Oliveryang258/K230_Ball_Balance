# -*- coding: utf-8 -*-
"""无CanMV IDE时使用的K230 SD卡实验启动器。

API：MicroPython open()、sys.print_exception()（存在时）。
硬件：插入可写SD卡的Yahboom K230 12Pin。
兼容性：不初始化摄像头、LCD或网络，只导入并启动main_stream_test；任何启动
        异常都会尽量记录到/sdcard/k230_stream_boot.log。

部署时把本文件复制并改名为main.py；原视觉main.py改名为vision_main.py。
"""

import sys


BOOT_LOG_PATH = "/sdcard/k230_stream_boot.log"


def _write_log(text, reset=False):
    try:
        mode = "w" if reset else "a"
        log_file = open(BOOT_LOG_PATH, mode)
        log_file.write(str(text) + "\n")
        log_file.close()
    except BaseException:
        pass


def _write_exception(error):
    try:
        log_file = open(BOOT_LOG_PATH, "a")
        log_file.write("fatal_type={}\n".format(type(error).__name__))
        log_file.write("fatal_repr={}\n".format(repr(error)))
        printer = getattr(sys, "print_exception", None)
        if printer is not None:
            try:
                printer(error, log_file)
            except BaseException:
                pass
        log_file.close()
    except BaseException:
        pass


_write_log("boot_stage=wrapper_start", reset=True)

try:
    import vision_main  # noqa: F401

    _write_log("boot_stage=vision_main_import_ok")

    import main_stream_test

    _write_log("boot_stage=stream_test_import_ok")
    _write_log("boot_stage=stream_test_run")
    main_stream_test.run()
    _write_log("boot_stage=stream_test_returned")
except BaseException as boot_error:
    _write_log("boot_stage=fatal")
    _write_exception(boot_error)
