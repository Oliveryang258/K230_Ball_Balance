# K230 Ball Balance Vision Control

Yahboom K230 12Pin + CanMV v1.8.0 MicroPython 小球平衡视觉控制项目。

当前阶段已进入钢球动态视觉测试：K230获取RGB888图像，通过已实机验证的`cv_lite.rgb888_find_circles()`检测钢球，并增加连续候选选择、轻量指数滤波和软件安全区状态，输出相对于实测物理中心`x=361`的整数像素误差。当前不启用UART、PID或舵机。

## 仓库入口

- [开发与部署说明](docs/README.md)
- [黄色轨道上板测试指南](docs/track-static-test.md)
- [钢球静态上板测试指南](docs/ball-static-test.md)
- [机械结构与初步模型](docs/mechanical_model.md)
- [已验证 API 记录](docs/verified-api-notes.md)
- [项目长期规则](AGENTS.md)
- [K230 项目 Skill](.agents/skills/k230-canmv-vision-control/SKILL.md)
- [K230 运行代码](src/main.py)
- [PC 端纯逻辑测试](tests/pc/test_geometry.py)
- [PC 端机械建模工具](tools/mechanical_model.py)
- [舵机—轨道标定数据模板](data/servo_rail_calibration.csv)

## 协作流程

```text
电脑 A：编辑 -> PC 静态/纯逻辑检查 -> git commit -> git push
电脑 B：git pull -> CanMV IDE 上传 -> K230 实测 -> 回填验证记录
```

GitHub 远程仓库配置完成后，以 Git 仓库内容为协作事实来源。
