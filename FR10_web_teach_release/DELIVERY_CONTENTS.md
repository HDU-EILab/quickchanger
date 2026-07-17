# FR10 交付包内容说明

本交付包用于 Jetson + FR10 控制器现场运行，默认控制器 IP 为 `192.168.58.2`。包内包含网页示教器、外设控制、RH56 灵巧手控制、Quickchanger ArUco 自动定位演示，以及 Quickchanger 所需的手眼标定与相机配置文件。

## 1. 可实现功能

### 1.1 FR10 网页示教器

入口：

```bash
cd FR10_web_teach_release
./run_web_teach.sh
```

默认访问地址：

```text
http://<Jetson IP>:8890
```

网页示教器支持：

- 读取当前 TCP 位姿。
- 执行 MoveL 绝对位姿移动。
- 执行相对移动：`Z+=80`、`X+=80`、`X-=80`。
- 执行 `#home`。
- 快换电机控制：正转、反转、停止、状态、锁紧诊断、复位诊断。
- 末端电源控制：切换灵巧手 8.4V、切换夹爪 24V、关闭输出、电源状态、末端监测。
- HEPG 夹爪打开/闭合。
- RH56 灵巧手抓紧/打开。
- 从网页触发 Quickchanger 平滑演示流程，并指定 ArUco ID。

### 1.2 Quickchanger ArUco 自动定位

入口目录：

```bash
cd FR10_web_teach_release/web_teach_192.168.58.2/calibration_hd_test
```

直接定位运行：

```bash
./run_quickchanger
./run_quickchanger --aruco-id 2
./run_quickchanger --headless --aruco-id 1
```

平滑演示扫描运行（网页示教器默认调用这一版）：

```bash
./run_quickchanger_smooth
./run_quickchanger_smooth 2
./run_quickchanger_smooth --aruco-id 2
```

分段演示扫描备用入口：

```bash
./run_quickchanger_demo
./run_quickchanger_demo 2
./run_quickchanger_demo --aruco-id 2
```

Quickchanger 支持：

- 默认识别 ArUco `DICT_4X4_50`、ID `1`。
- 支持通过参数指定不同 ArUco ID。
- 支持窗口模式与 headless 模式。
- RealSense HD 取流。
- 自动检测 ArUco 中心点。
- 使用手眼标定矩阵将相机坐标转换到机械臂基座坐标。
- 使用两阶段 deadzone 补偿移动到目标点。
- 平滑演示模式先沿固定扫描轨迹连续移动，扫描阶段用半分辨率彩色帧识别 ArUco，并按 RealSense 帧时间匹配 TCP 位姿；回到最佳视角后再采集深度并执行 Quickchanger 定位移动。
- 分段演示模式保留为备用脚本。

## 2. 关键代码入口

网页示教器：

```text
web_teach_192.168.58.2/fr10_web_teach.py
web_teach_192.168.58.2/motor_power_bus.py
web_teach_192.168.58.2/rh56_hand.py
```

Quickchanger：

```text
web_teach_192.168.58.2/calibration_hd_test/quickchanger_aruco_hd.py
web_teach_192.168.58.2/calibration_hd_test/quickchanger_demo_sweep_hd.py
web_teach_192.168.58.2/calibration_hd_test/quickchanger_smooth_sweep_hd.py
web_teach_192.168.58.2/calibration_hd_test/pick_point_aruco_move_hd.py
web_teach_192.168.58.2/calibration/pick_point_move.py
```

## 3. 手眼标定矩阵与相机配置

包内已包含当前 active 标定结果：

```text
web_teach_192.168.58.2/calibration/sessions/active/handeye_result.json
web_teach_192.168.58.2/calibration/sessions/active/intrinsics.npz
web_teach_192.168.58.2/calibration/sessions/active/field_offset.json
web_teach_192.168.58.2/calibration/sessions/active/meta.json
```

其中：

- `handeye_result.json`：手眼标定矩阵 `T_cam2gripper`。
- `intrinsics.npz`：RealSense 相机内参。
- `field_offset.json`：现场手眼/落点补偿。
- `meta.json`：标定会话元信息。

Quickchanger 运行参数集中在：

```text
web_teach_192.168.58.2/calibration_hd_test/quickchanger_config.json
```

其中包含两阶段 deadzone、演示扫描起点/终点、失配次数、扫描分段数、平滑扫描速度、平滑扫描检测缩放比例等现场参数。当前交付默认值：

```text
smooth_scan_vel = 30.0
lost_after_visible = 2
smooth_scan_detect_scale = 0.5
smooth_frame_time_offset_s = 0.12
```

## 4. 机械臂相关接口文档

包内包含机械臂接口、网页 API、互锁规则和标定说明：

```text
web_teach_192.168.58.2/DEVELOPMENT_DOC.md
web_teach_192.168.58.2/WEB_TEACH_RULES.md
web_teach_192.168.58.2/calibration/PICK_POINT.md
web_teach_192.168.58.2/calibration/HANDEYE_CALIB.md
web_teach_192.168.58.2/calibration_hd_test/README.md
```

重点说明：

- `DEVELOPMENT_DOC.md`：网页示教器架构、HTTP API、机械臂通道、外设总线、Quickchanger demo 入口。
- `WEB_TEACH_RULES.md`：示教点、挂起状态、二次确认和互锁规则。
- `PICK_POINT.md`：选点/移动链路说明。
- `HANDEYE_CALIB.md`：手眼标定说明。
- `calibration_hd_test/README.md`：Quickchanger ArUco HD 运行说明。

## 5. 依赖与环境

运行环境需要：

- Python 3.9+。
- Fairino/FR10 Python SDK，可导入 `fairino` 或 `Robot`。
- `pyserial`。
- Quickchanger/RealSense 运行需要 `opencv-python`、`numpy`、`pyrealsense2`，现场脚本默认使用 conda 环境 `rmc-yolo`。
- Jetson 能访问 FR10 控制器 `192.168.58.2`。
- 串口设备建议使用 udev 固定名：
  - `/dev/fr10-motor-serial`
  - `/dev/fr10-hand-serial`

## 6. 未包含内容

交付包未包含中间调试和历史归档代码，例如：

- 旧 HD 选点脚本。
- 圆拟合、椭圆拟合、模板匹配测试脚本。
- 内参对比脚本。
- `archive_hd_test_*` 归档目录。
- `__pycache__`、`.pyc`、`.DS_Store` 等临时文件。
