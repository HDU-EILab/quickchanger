# Quickchanger FR10 Web Teach Release

本仓库是 FR10 + Jetson 现场交付代码，用于网页示教器、Quickchanger ArUco 自动定位演示、手眼标定配置、外设电机/供电控制和 RH56 灵巧手控制。

仓库已设置为 private，面向 HDU-EILab 组织内成员查看。

## 主要功能

- FR10 网页示教器：读取 TCP、MoveL 位姿移动、相对移出/移入、回 home。
- Quickchanger ArUco 自动定位：识别 ArUco 4x4 marker，默认 ID 为 `1`。
- Smooth 演示流程：机械臂连续平滑扫描，扫描中检测 ArUco，回到最佳视角后执行两阶段对接移动。
- 外设控制：快换电机正反转/停止/状态/诊断、末端供电切换、HEPG 夹爪、RH56 灵巧手。
- 标定支持：包含当前 active 手眼标定矩阵、RealSense 内参和现场补偿文件。

## 目录结构

```text
.
├── README_RELEASE.md          # 发布包运行说明
├── DELIVERY_CONTENTS.md       # 交付内容清单和功能说明
├── requirements.txt           # 环境依赖说明
├── run_web_teach.sh           # 直接启动网页示教器
├── web_panel.sh               # conda 方式启动网页示教器
└── web_teach_192.168.58.2/
    ├── fr10_web_teach.py      # 网页示教器主程序
    ├── DEVELOPMENT_DOC.md     # 系统设计、HTTP API、机械臂接口文档
    ├── WEB_TEACH_RULES.md     # 页面互锁和操作规则
    ├── motor_power_bus.py     # 电机/供电总线控制
    ├── calibration/           # 手眼标定、选点移动和 active 标定结果
    └── calibration_hd_test/   # Quickchanger ArUco HD 和 smooth 演示脚本
```

## 启动网页示教器

在 Jetson 或能访问 FR10 控制器的主机上：

```bash
chmod +x *.sh
./run_web_teach.sh
```

默认访问：

```text
http://<Jetson IP>:8890
```

默认控制器 IP 在 `web_teach_192.168.58.2/fr10_web_teach.py` 中：

```python
FR10_HOST = "192.168.58.2"
```

## Quickchanger 入口

```bash
cd web_teach_192.168.58.2/calibration_hd_test

# 当前网页示教器默认调用的 smooth 演示流程
./run_quickchanger_smooth
./run_quickchanger_smooth --aruco-id 2

# 单次 ArUco 定位移动
./run_quickchanger
./run_quickchanger --aruco-id 1

# 保留的分段扫描备用入口
./run_quickchanger_demo
```

网页示教器中的 Quickchanger 演示按钮默认执行：

```text
quickchanger_smooth_sweep_hd.py
```

不是旧的分段扫描脚本。

## 当前 Quickchanger 默认参数

配置文件：

```text
web_teach_192.168.58.2/calibration_hd_test/quickchanger_config.json
```

当前交付默认值：

```text
smooth_scan_vel = 30.0
lost_after_visible = 2
smooth_scan_detect_scale = 0.5
smooth_frame_time_offset_s = 0.12
stage1_deadzone_mm = [-2.0, -2.0, 37.0]
stage2_deadzone_mm = [-2.0, -2.0, -3.0]
```

说明：

- `smooth_scan_vel`：smooth 连续扫描 MoveL 速度百分比。
- `lost_after_visible`：首次识别后连续多少帧未识别即结束扫描。
- `smooth_scan_detect_scale`：扫描阶段 ArUco 检测缩放比例，`0.5` 表示半分辨率检测后映射回原图。
- `smooth_frame_time_offset_s`：图像帧和 TCP 位姿匹配的时间补偿。
- `stage1_deadzone_mm` / `stage2_deadzone_mm`：两阶段最终对接补偿。

## 标定与接口文档

关键文件：

```text
web_teach_192.168.58.2/calibration/sessions/active/handeye_result.json
web_teach_192.168.58.2/calibration/sessions/active/intrinsics.npz
web_teach_192.168.58.2/calibration/sessions/active/field_offset.json
web_teach_192.168.58.2/DEVELOPMENT_DOC.md
web_teach_192.168.58.2/calibration/HANDEYE_CALIB.md
web_teach_192.168.58.2/calibration/PICK_POINT.md
web_teach_192.168.58.2/calibration_hd_test/README.md
```

## 现场注意事项

- 运行前确认 Jetson 与 FR10 控制器 `192.168.58.2` 网络互通。
- Quickchanger/RealSense 脚本依赖现场 conda 环境，默认配置见 `fr10_web_teach.py` 顶部 `QUICKCHANGER_*` 常量。
- 网页示教器和 Quickchanger 脚本不会同时使用机械臂 SDK；脚本内包含对 `fr10_web_teach.py` 的抢占暂停逻辑。
- 串口设备建议使用固定 udev 名：`/dev/fr10-motor-serial`、`/dev/fr10-hand-serial`。
