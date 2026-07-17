# 点击选点逼近（与 Jetson 现场脚本一致）

## 手眼标定矩阵（已核对）

| 来源 | 时间 | 样本数 | t_cam2gripper (mm) |
|------|------|--------|---------------------|
| `sessions/active/handeye_result.json` | 2026-07-07 14:12 | 11 | (-102.1, -51.1, 99.0) |
| `quickchanger/config/hand_eye_T_FC.json` | 同上 | 11 | 同上 |
| Jetson `sessions/active/` | 同上 | 11 | 同上 |

**结论：已采用 2026-07-07 相机移位后的最新手眼矩阵。**

手眼外参 `T_cam2gripper` 与图像分辨率无关；高分辨率模式使用 RealSense **当前流**的内参做反投影。

## 分辨率

| 脚本 | 彩色 | 深度 | 是否高分辨率 |
|------|------|------|--------------|
| `pick_point/run_pick_point_move.sh` | 1280×720 | 848×480 | 否（与手眼标定采图一致） |
| `pick_point_hd/run_pick_point_move_hd.sh` | **1920×1080** | **1280×720** | **是（推荐选点）** |

手眼标定在 **1280×720** 下完成；高分辨率选点用设备实时内参，外参矩阵不变。

## 在 Jetson 上运行（推荐）

```bash
ssh jetson-orin-nx
# 关闭网页示教器后再跑

# 高分辨率（默认 profile=hd）
cd ~/FR10_web_teach_release/web_teach_192.168.58.2/calibration_hd_test
./run_pick_point_move_hd.sh
./run_pick_point_move_hd.sh --height-mm 50    # 总高度 150 mm（100 基准 + 50）

# 标准 720p（与标定同分辨率）
cd ../calibration
./run_pick_point_move.sh
```

GUI：**左键选点** → **`m` 移动** → **`r` 刷新** → **`q` 退出**

高度：**硬编码基准 100 mm** + `--height-mm` 增量。

## 本地仓库路径

| 路径 | 说明 |
|------|------|
| `quickchanger/calibration/pick_point/` | 720p 主线 |
| `quickchanger/calibration/pick_point_hd/` | **1080p 高分辨率**（已修复路径） |
| `pick_point/sessions/active/handeye_result.json` | 最新手眼 |

同步 HD 修复到 Jetson：

```bash
rsync -av quickchanger/calibration/pick_point_hd/ \
  jetson-orin-nx:~/FR10_web_teach_release/web_teach_192.168.58.2/calibration_hd_test/
```

## 常用参数

| 参数 | 默认 | 含义 |
|------|------|------|
| `--profile hd` | hd | 1920×1080（HD 脚本默认） |
| `--profile legacy` | - | 1280×720 |
| `--height-mm` | 0 | 在 **100 mm 基准**之上的增量 |
| `--approach tcp_above` | 默认 | 基座 +Z 正上方 |
| `--dry-run` | off | 只算不运动 |

