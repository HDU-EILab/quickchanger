# FR10 网页示教器 — 独立发布包

本压缩包内含 **网页示教器**、**关节程序参考 `step`**、**规则说明**、**启动脚本** 与 **环境说明**，解压后即可按下列步骤在能访问机械臂控制器的电脑上运行。

当前版本已集成 Quickchanger ArUco 平滑演示流程。网页示教器里的 Quickchanger 演示按钮默认调用 `calibration_hd_test/quickchanger_smooth_sweep_hd.py`，不是旧的分段扫描脚本。

## 包内结构

```
FR10_web_teach_release/
├── README_RELEASE.md          # 本说明
├── requirements.txt          # 环境 / 依赖说明（非纯 pip 列表）
├── verify_fairino_env.py     # 检查 fairino / Robot 是否可导入
├── web_panel.sh              # 使用 conda 环境启动（与原版习惯一致）
├── run_web_teach.sh          # 不依赖 conda，直接用 python3 启动
└── web_teach_192.168.58.2/
    ├── fr10_web_teach.py     # 示教器主程序
    ├── WEB_TEACH_RULES.md    # 业务规则汇总
    ├── calibration/          # 手眼标定、选点移动、active 标定结果
    ├── calibration_hd_test/  # Quickchanger ArUco HD 与 smooth 演示脚本
    └── step                   # 关节程序点参考（度）；与脚本内常量应对齐
```

## 环境要求

1. **Python 3.9+**（推荐 3.10+），系统已安装 `python3`。
2. **FAIRINO 法奥机器人 Python SDK**（与控制器固件版本匹配）。安装方式见 `requirements.txt` 注释。配置完成后可在本目录执行：
   ```bash
   python3 verify_fairino_env.py
   ```
   打印 `OK` 即表示当前解释器能找到 SDK。
3. 运行电脑与机械臂控制器 **网络互通**（默认脚本内 `FR10_HOST = "192.168.58.2"`，请按现场 IP 修改 `fr10_web_teach.py` 顶部常量）。

## 启动方式

在解压后的 **`FR10_web_teach_release` 目录** 下执行（先 `chmod +x *.sh` 若需要）：

### 方式 A：`run_web_teach.sh`（推荐先试）

不强制 conda，使用当前环境中的 `python3`（需已能 `import fairino` 或 `import Robot`）：

```bash
./run_web_teach.sh
```

指定端口：

```bash
PORT=9000 ./run_web_teach.sh
```

### 方式 B：`web_panel.sh`（与原仓库习惯一致）

需已安装 **conda**，并已创建/配置好含 FAIRINO SDK 的环境（默认名为 `fr10-fairino`，可在脚本内修改）。若在非交互 shell 中 `conda activate` 报错，可先执行 `source <conda安装路径>/etc/profile.d/conda.sh`，或改用 **`run_web_teach.sh`**。

```bash
./web_panel.sh
```

脚本会切换到 `web_teach_192.168.58.2` 子目录并启动服务。

## 访问

浏览器打开：`http://<运行示教器的电脑IP>:8890`（默认端口 8890，与 `PORT` 环境变量一致）。

## 修改机器人 IP

编辑 `web_teach_192.168.58.2/fr10_web_teach.py` 中的 `FR10_HOST` 常量后重新启动。

## Quickchanger 演示

网页内 Quickchanger 演示流程支持填写 ArUco ID，默认 ID 为 `1`。后端默认执行：

```text
web_teach_192.168.58.2/calibration_hd_test/quickchanger_smooth_sweep_hd.py
```

当前交付默认参数在 `quickchanger_config.json` 中：

```text
smooth_scan_vel = 30.0
lost_after_visible = 2
smooth_scan_detect_scale = 0.5
smooth_frame_time_offset_s = 0.12
```

命令行备用入口：

```bash
cd web_teach_192.168.58.2/calibration_hd_test
./run_quickchanger_smooth
./run_quickchanger --aruco-id 1
```

## 规则与程序点

- 示教逻辑、二次确认、挂起状态等见 **`web_teach_192.168.58.2/WEB_TEACH_RULES.md`**。
- 关节角度参考见同目录 **`step`**；若现场程序变更，请同时更新脚本内对应 `*_JOINT` 常量与 `step` 文件。

## 防火墙

若手机或其它电脑无法访问，请在本机放行监听端口（默认 8890）的入站 TCP。

## 电机串口控制（已集成）

- 页面内已包含电机 `正转/反转/停止` 控制。
- 首次使用请填写串口端口（默认留空），Linux 常见如 `/dev/ttyUSB0`。
- 运行用户需要具备串口访问权限（通常加入 `dialout` 组）。
