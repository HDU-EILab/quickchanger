# Quickchanger ArUco HD

当前目录只保留 quickchanger 运行入口及其依赖。旧的 HD 选点/圆拟合/椭圆/模板匹配/内参对比测试脚本已归档到：

```bash
archive_hd_test_20260716_quickchanger_cleanup/
```

## 保留文件

| 文件 | 作用 |
|------|------|
| `run_quickchanger` | 当前 quickchanger 主入口 |
| `run_quickchanger.sh` | 运行环境、DISPLAY、SDK 抢占包装 |
| `quickchanger_aruco_hd.py` | ArUco 检测 + quickchanger 两阶段移动逻辑 |
| `quickchanger_config.json` | quickchanger 现场参数：两阶段 deadzone、移动速度、演示扫描起终点、失配次数、分段数 |
| `run_quickchanger_demo` | 演示扫描入口：先平移找无遮挡视角，再调用 quickchanger |
| `run_quickchanger_demo.sh` | 演示扫描运行环境、DISPLAY、SDK 抢占包装 |
| `quickchanger_demo_sweep_hd.py` | 演示扫描逻辑 |
| `run_quickchanger_smooth` | 平滑演示扫描入口：连续 MoveL 平移时检测 ArUco |
| `run_quickchanger_smooth.sh` | 平滑演示扫描运行环境、DISPLAY、SDK 抢占包装 |
| `quickchanger_smooth_sweep_hd.py` | 平滑扫描逻辑：子进程连续 MoveL，主进程取流检测 |
| `pick_point_aruco_move_hd.py` | ArUco 检测函数依赖 |
| `_paths.py` | 定位主 calibration 目录 |
| `hd_config.py` | HD/legacy 取流配置 |
| `hd_pick_args.py` | HD 选点公共参数 |
| `hd_realsense.py` | RealSense 取流封装 |
| `robot_sdk_preempt.sh` | 暂停/恢复 `fr10_web_teach.py`，避免 SDK 端口占用 |

## 运行

```bash
./run_quickchanger
```

## 配置文件

现场参数集中在同目录：

```bash
quickchanger_config.json
```

当前默认值：

```json
{
  "stage1_deadzone_mm": [-2.0, -2.0, 37.0],
  "stage2_deadzone_mm": [-2.0, -2.0, -3.0],
  "quickchanger_move_vel": 25.0,
  "demo_start": [-769.0, -82.0, 1118.0, -159.0, 16.0, 45.0],
  "demo_end": [-769.0, 487.0, 1118.0, -159.0, 16.0, 45.0],
  "lost_after_visible": 2,
  "scan_steps": 56,
  "demo_scan_vel": 35.0,
  "demo_settle_s": 0.02,
  "min_perimeter_px": 120.0,
  "smooth_scan_vel": 35.0,
  "smooth_sample_hz": 30.0,
  "smooth_stop_on_lost": true,
  "smooth_pose_sync_mode": "buffer",
  "smooth_pose_sample_hz": 40.0,
  "smooth_frame_time_offset_s": 0.12,
  "smooth_scan_detect_scale": 0.5,
  "_comments": {
    "...": "配置文件内含完整字段说明；JSON 不支持 // 注释，因此使用 _comments 保持文件可解析。"
  }
}
```

可以用 `--config <path>` 指定其他配置文件；命令行参数仍可临时覆盖配置。

### 配置参数说明

| 参数 | 单位 / 类型 | 作用 | 建议取值 |
|------|-------------|------|----------|
| `stage1_deadzone_mm` | `[mm, mm, mm]` | Quickchanger 阶段1预对准补偿，基座坐标系 `+X/+Y/+Z`。当前 `[-2,-2,37]` 会先到最终点上方预对准。 | 现场实测后修改；不要随意调 |
| `stage2_deadzone_mm` | `[mm, mm, mm]` | Quickchanger 阶段2最终点补偿，直接影响最终插入/对接点。 | 现场实测后修改；不要随意调 |
| `quickchanger_move_vel` | `%` | 最终两阶段 MoveL 速度，影响 `run_quickchanger`、分段 demo、smooth demo 的最终定位阶段。 | `10-40` |
| `demo_start` | `[mm,mm,mm,deg,deg,deg]` | 演示扫描起点 TCP 位姿 `[X,Y,Z,Rx,Ry,Rz]`。 | 根据现场安全轨迹设定 |
| `demo_end` | `[mm,mm,mm,deg,deg,deg]` | 演示扫描终点 TCP 位姿 `[X,Y,Z,Rx,Ry,Rz]`。 | 根据现场安全轨迹设定 |
| `lost_after_visible` | 次 / 帧 | 首次识别到 ArUco 后，连续多少次未识别就结束扫描。分段 demo 按扫描点计数，smooth demo 按检测帧计数。 | `1-5`，越小触发越快 |
| `scan_steps` | 段数 | `run_quickchanger_demo` 分段扫描数量，只影响分段 demo。 | `20-80` |
| `demo_scan_vel` | `%` | `run_quickchanger_demo` 分段扫描 MoveL 速度。 | `10-60` |
| `demo_settle_s` | 秒 | 分段 demo 每个扫描点到位后等待相机稳定的时间。 | `0-0.1` |
| `min_perimeter_px` | 像素 | ArUco 参与最佳视角评分的最小周长阈值。 | `80-200` |
| `smooth_scan_vel` | `%` | `run_quickchanger_smooth` 连续扫描 MoveL 速度。 | `20-80` |
| `smooth_sample_hz` | Hz | smooth 连续移动时 ArUco 检测采样频率。高速扫描时应尽量接近相机帧率。 | `12-30` |
| `smooth_stop_on_lost` | bool | smooth 首次识别后连续丢失目标时是否调用 `StopMotion` 提前结束扫描。 | `true` 更快；`false` 更保守 |
| `smooth_pose_sync_mode` | string | smooth 图像帧与机械臂 TCP 位姿同步方式。`buffer` 后台高频采 TCP，不阻塞识别；`bracket` 取图前后读 TCP；`after` 为旧逻辑。 | 高速扫描用 `buffer` |
| `smooth_pose_sample_hz` | Hz | `buffer` 模式下后台 TCP 位姿采样频率。 | `30-60` |
| `smooth_frame_time_offset_s` | 秒 | RealSense 图像帧时间提前校正。脚本先用 RealSense 帧时间戳定位图像时刻，再减去该值去匹配 TCP 缓冲。 | `0.05-0.25` |
| `smooth_scan_detect_scale` | 倍率 | smooth 扫描阶段 ArUco 检测图像缩放比例。`0.5` 用半分辨率检测后映射回原图，能提高扫描 FPS；最终定位仍使用原始 1920x1080 图和深度。 | `0.5-1.0` |
| `_comments` | object | 字段说明。脚本不会读取该字段；用于保持 JSON 合法且可注释。 | 保留即可 |

常用参数：

```bash
./run_quickchanger --aruco-id 1
./run_quickchanger --headless
./run_quickchanger --skip-stage1
./run_quickchanger --dry-run
```

## 演示扫描

```bash
./run_quickchanger_demo
./run_quickchanger_demo 2
./run_quickchanger_demo --aruco-id 2
```

默认流程：

1. 移动到演示扫描起点 `[-769, -82, 1118, -159, 16, 45]`
2. 分段扫描到终点 `[-769, 487, 1118, -159, 16, 45]`
3. 每个扫描点检测指定 ArUco，使用周长和四边一致性给无遮挡程度打分
4. 第一次识别到目标后，如果连续 2 个扫描点未识别到目标，结束扫描
5. 回到扫描过程中评分最高点对应的 TCP 视角
6. 调用 `quickchanger_aruco_hd.py` 的两阶段移动逻辑

调试参数：

```bash
./run_quickchanger_demo --dry-run
./run_quickchanger_demo --complete-scan
./run_quickchanger_demo --scan-steps 56 --scan-vel 35
./run_quickchanger_demo --lost-after-visible 2
./run_quickchanger_demo --config quickchanger_config.json
```

默认逻辑：

- ArUco dictionary: `4x4_50`
- ArUco ID: `1`
- 固定 RPY: `-180 0 45`
- 阶段1 deadzone: `-2 -2 37`
- 阶段2 deadzone: `-2 -2 -3`

阶段2目标生成复用 `pick_point_move.compute_tcp_target()`，与 `run_pick_point_aruco_move_hd.sh` 原移动逻辑保持一致；quickchanger 只覆盖固定 RPY 和两阶段移动。

## 平滑演示扫描

```bash
./run_quickchanger_smooth
./run_quickchanger_smooth 2
./run_quickchanger_smooth --aruco-id 2
./run_quickchanger_smooth --smooth-scan-vel 30 --smooth-sample-hz 30
```

平滑扫描与分段 demo 的区别：

1. 先移动到演示扫描起点。
2. 子进程下发一条从起点到终点的高速连续 `MoveL`。
3. 主进程在连续移动过程中按 `smooth_sample_hz` 只取彩色帧识别 ArUco；扫描阶段可用 `smooth_scan_detect_scale=0.5` 半分辨率检测并映射回原图，同时用 `smooth_pose_sync_mode=buffer` 从后台 TCP 位姿缓冲中按 RealSense 图像帧时间插值同步；深度只在回到最佳视角后的最终定位阶段采集。
4. 首次有效识别后若连续 `lost_after_visible` 帧未识别到目标，默认调用 `StopMotion` 结束扫描，再回到历史最佳视角。
5. 复查 ArUco 稳定后，调用与 `run_quickchanger` 相同的两阶段最终定位逻辑。

调试参数：

```bash
./run_quickchanger_smooth --dry-run
./run_quickchanger_smooth --complete-scan
./run_quickchanger_smooth --no-stop-on-lost
./run_quickchanger_smooth --smooth-scan-vel 10 --smooth-sample-hz 30
./run_quickchanger_smooth --frame-time-offset-s 0.16
./run_quickchanger_smooth --scan-detect-scale 1.0
```

如果高速扫描后“回到最佳视角”仍有一点错位：

- 回到的位置偏扫描方向后方/已经越过目标：调大 `smooth_frame_time_offset_s`，例如 `0.16`、`0.20`。
- 回到的位置偏扫描方向前方/还没到目标：调小 `smooth_frame_time_offset_s`，例如 `0.08`。
