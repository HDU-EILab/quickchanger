# RGB 点击选点 → 机械臂移到物体上方

## 思路（眼在手上 + 已有手眼标定）

**默认 `--approach tcp_above`（TCP 在点击点正上方 H mm）：**

```
像素 (u,v) + 深度 → 物体表面点 S（基座系）
目标 TCP = [S_x, S_y, S_z + H]   （基座 +Z 方向抬高，姿态保持当前）
MoveL(目标 TCP)
```

**旧模式 `--approach standoff`：** 沿法向 n 抬起，见 `--height-ref`。

| 坐标系 | 来源 |
|--------|------|
| 相机光学系 | RealSense 彩色流内参 + 对齐深度 `deproject` |
| T_cam2gripper | `sessions/active/handeye_result.json` |
| T_gripper2base | 实时 `GetActualTCPPose`（Tool#0，基座/user#0） |

**不需要棋盘格**：任意物体只要在 RGBD 视野内、该像素有有效深度即可。

## 运行

```bash
cd web_teach_192.168.58.2/calibration

# GUI：左键选点，滚轮/双指捏合缩放，右键拖动平移，m 移动，r 刷新，q 退出
./run_pick_point_move.sh

# 命令行指定像素（无 GUI）
./run_pick_point_move.sh --pixel 640 360 --height-mm 50

# 只算不移动
./run_pick_point_move.sh --pixel 640 360 --dry-run
```

## 高度 `--height-mm` 与 TCP/相机偏差

**默认总高度 100 mm**；`--height-mm` 为在其之上的**增量**（例：不传 → 100 mm，`--height-mm 50` → 150 mm）。

| `--height-ref` | 含义 |
|----------------|------|
| `camera`（**默认**） | `--height-mm` = **相机光心**到物体表面的距离（与深度读数同一参考） |
| `tcp` | `--height-mm` = **Tool0 TCP**到物体表面的距离 |

眼在手上时相机与 TCP 相差约 **13 cm**（见标定 `t_cam2gripper`）。若把深度/高度都当成相机距离，请用默认 `camera`；旧版按 TCP 算会偏高。

| 参数 | 用途 |
|------|------|
| `sessions/active/field_offset.json` | **基座系现场偏移**（不改 handeye）；当前 X -5、Y +5 mm |
| `--no-field-offset` | 本次运行不应用 `field_offset.json` |
| `--height-bias-mm 25` | 沿法向再降低 TCP 25 mm（现场微调） |
| `--assume-depth-mm 100` | 深度显示 0（距离 &lt; 约 20 cm）时，手动指定相机到物体距离 |

**深度稳定（默认已开）**：RealSense spatial/temporal 滤波；GUI 显示 9 帧时序平滑；按 `m` 移动前 7 帧采样取中值。可调：`--depth-frames`、`--depth-smooth-frames`、`--depth-radius`；关闭硬件滤波：`--no-rs-depth-filter`。

D435 可靠深度通常 **≥ 200 mm**；100 mm 时界面常显示 `depth=0`，需拉远或使用 `--assume-depth-mm`。

若「预计位移」&lt; 3 mm，脚本默认**不下发运动**（加 `--force-move` 可强制）。

## 「上方」方向 `--normal`

| 模式 | 含义 |
|------|------|
| `plane`（默认） | 点击点附近深度拟合局部平面，沿法向抬起 |
| `base_z` | 沿基坐标系 ±Z |
| `camera` | 沿「物体点 → 当前 TCP」方向抬起 |

## 使用注意

1. **必须先完成眼在手上标定**（`T_cam2gripper`）。
2. 点击处及周围需有**可靠深度**（避免反光、纯黑、过远 >3m）。
3. 移动后若物体不在画面内，无法再次点击；需先移回能看见物体的姿态。
4. 默认**保持当前 TCP 姿态**，只改位置；若需垂直朝下需另做姿态规划。
5. 结果图保存在 `sessions/active/pick_point_last.jpg`。

## 与棋盘格方案对比

| | 棋盘格 `go_above_board` | 像素选点 `pick_point_move` |
|--|-------------------------|----------------------------|
| 目标 | 棋盘中心 | 任意可见物体表面一点 |
| 深度 | 可选 / 缓存 | **必须** |
| 适用 | 固定棋盘重复定位 | 通用物体 |
