# FR10 手眼标定说明（当前生效数据）

## 生效标定

| 项目 | 值 |
|------|-----|
| 会话目录 | `sessions/active/`（原 `handeye_session_20260608_163137`） |
| 标定时间 | 2026-06-08 16:33:28 |
| 模式 | **眼在手上**（`eye_in_hand`） |
| 样本数 | 14 |
| 一致性误差 | **4.32 mm**（`mean_board_origin_error_m`） |
| 算法 | OpenCV `CALIB_HAND_EYE_TSAI` |
| 结果文件 | `sessions/active/handeye_result.json` |

其余历史会话已删除，仅保留上述目录。

---

## 坐标系对照（实际引用的是哪一套）

标定脚本采集机械臂位姿时调用 `GetActualTCPPose`，并结合控制器当前 **工具号 / 工件号**。标定当时及现场查询结果如下。

### 1. 世界坐标系（World / Base）

| 项目 | 实际含义 |
|------|----------|
| **名称** | `robot_base`（FR10 **基坐标系**） |
| **工件号** | `user = 0` |
| **工件偏移** | `[0, 0, 0, 0, 0, 0]` mm/°（与基座无偏移） |
| **结论** | 标定中的「基座 / 世界」= **法奥 FR10 基坐标系**，不是自定义工件坐标系。 |

机械臂每个采样位姿 `pose_mm_deg` 与 `T_gripper2base`，均为 **Tool#0 在基坐标系下** 的位姿。

### 2. 工具坐标系（Gripper / TCP）

| 项目 | 实际含义 |
|------|----------|
| **名称** | `tool0_tcp` |
| **工具号** | `tool = 0`（`GetActualTCPNum` 返回 0） |
| **工具偏移** | `GetToolCoordWithID(0) = [0,0,0,0,0,0]` |
| **SDK 接口** | `GetActualTCPPose`（`meta.robot_frame = "tcp"`） |
| **结论** | 标定中的「gripper」= **当前激活的 Tool#0 TCP**。因 Tool0 无相对法兰偏置，**与末端法兰坐标系实质重合**（位姿数值与 `GetActualToolFlangePose` 仅亚毫米级差异）。 |

若后续在示教器或控制器中 **切换工具号** 或 **修改 Tool0 偏置**，本标定外参将不再直接适用，需重新标定或做 TCP 偏置换算。

### 3. 相机坐标系

| 项目 | 实际含义 |
|------|----------|
| **名称** | `realsense_color_optical` |
| **设备** | Intel RealSense D435 彩色光学系 |
| **轴向** | OpenCV 相机光学系：**+Z 朝前，+X 向右，+Y 向下** |
| **内参** | `sessions/active/intrinsics.npz`（取自 D435 彩色流出厂内参） |
| **深度** | 采集时深度已 **align 到彩色** |

### 4. 标定板坐标系

| 项目 | 实际含义 |
|------|----------|
| **名称** | `chessboard` |
| **规格** | 12×9 格，内角点 11×8，单格 **15 mm** |
| **原点** | 棋盘某一内角点（OpenCV `findChessboardCorners` 角点网格原点） |
| **平面** | Z = 0，格点沿 X/Y 展开 |

---

## 外参结果含义

`handeye_result.json` 中主结果：

```
T_cam2gripper   # 4×4 齐次矩阵，单位：旋转无量纲，平移 m
```

**物理含义：**

> 将 **相机彩色光学坐标系** 下的点变换到 **Tool#0 TCP 坐标系**：
>
> `P_tcp = R_cam2gripper @ P_cam + t_cam2gripper`

**眼在手上**完整链路（棋盘固定在工作台）：

```
T_target2base = T_gripper2base @ T_cam2gripper @ T_target2cam
```

其中：

- `T_gripper2base`：采样时刻 `GetActualTCPPose` 得到（Tool#0 → Base）
- `T_target2cam`：该帧 `solvePnP` 得到（棋盘 → 相机）
- `T_cam2gripper`：标定结果（相机 → Tool#0）

### 当前数值摘要

平移 `t_cam2gripper`（m）≈ `[-0.029, -0.130, 0.019]`  
即相机光心相对 Tool0 大约在：**X −29 mm，Y −130 mm，Z +19 mm**（在 TCP 坐标系下表达）。

---

## 位姿表示约定

| 项目 | 约定 |
|------|------|
| 机械臂位姿来源 | `GetActualTCPPose` → `[x,y,z,rx,ry,rz]` |
| 长度单位 | mm |
| 角度单位 | ° |
| 旋转→矩阵 | 外旋欧拉 `xyz`（`scipy Rotation.from_euler('xyz', ..., degrees=True)`） |
| 齐次变换进 OpenCV | 平移先换算为 **m** |

---

## 与示教器的关系

`fr10_web_teach.py` 默认 `tool=0`、`user=0`，与本标定一致。使用本外参时：

1. 保持 **Tool#0** 为当前工具，且偏置勿改（或按偏置重新换算）。
2. 保持 **User#0** 与基座对齐（或先将位姿变换到基座再使用）。
3. 相机需仍安装在标定时的同一刚性位置（眼在手上）。

---

## 文件索引

```
calibration/
├── HANDEYE_CALIB.md          # 本文档
├── fr10_handeye_calibrate.py
├── run_handeye_calibrate.sh
└── sessions/
    └── active/               # 当前唯一生效标定
        ├── handeye_result.json
        ├── meta.json         # 含 coordinate_frames 字段
        ├── samples.json
        ├── intrinsics.npz
        └── *.png             # 14 组彩色/深度原图
```
