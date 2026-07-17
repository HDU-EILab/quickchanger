# FR10 网页示教器 — 系统设计与开发文档

**文档类型：** 系统设计 / 接口实现说明  
**最后更新：** 2026-07-17  
**运行环境：** NVIDIA Jetson + FR10 控制器（默认 `192.168.58.2`）  
**访问地址：** `http://<Jetson IP>:8890`  
**现场部署目录：** `/opt/fr10_web_teach/web_teach_192.168.58.2/`  
**systemd 服务：** `fr10-web-teach.service`

> 业务规则细则见 `web_teach_192.168.58.2/WEB_TEACH_RULES.md`。  
> 若本文与代码冲突，以 `fr10_web_teach.py` / `motor_power_bus.py` / `rh56_hand.py` 当前实现为准。

---

## 1. 系统目标与边界

### 1.1 要解决什么

在 Jetson 上提供统一的浏览器示教入口，让现场人员无需反复开终端脚本，即可完成：

| 能力 | 说明 |
|------|------|
| 关节程序点 | MoveJ：`#home`、`#A/#B#C` 的 pre / 工作位 / out |
| TCP 微调 | MoveL：绝对位姿、相对位移（向上/向后/向前） |
| 快换电机 | 正转锁紧 / 反转复位 / 停止 / 状态 / 诊断 |
| 末端电源 | CL-200W：电压、限流、输出开关、设备档案切换（8.4V 手 / 24V 夹爪） |
| HEPG 夹爪 | 张开 / 闭合 / 演示 |
| RH56 灵巧手 | 抓紧 / 打开（独立 EB90 协议） |
| 可选演示 | Quickchanger 演示流程（ArUco + 两段运动，依赖标定目录） |

### 1.2 明确不做 / 边界

- **不替代** 控制器 Web 后台（8080/8082 等）做固件级诊断。
- **视觉抓点主链路** 仍在 `quickchanger/calibration/`，网页仅挂可选 demo 入口，未形成全闭环。
- **挂起状态** 仅存在于示教器进程内存；重启服务后与真实关节可能不一致，需人工确认安全位。
- 实验室 Windows 单 COM 联调脚本（`motor_control_with_power_debug.py`）与现场 **双 RS485** 拓扑不同，不可照搬端口/从站假设。

---

## 2. 总体架构

### 2.1 逻辑分层

```
浏览器 (HTML/JS 内嵌于 fr10_web_teach.py)
        │  HTTP GET / 、POST /api/fr10
        ▼
┌───────────────────────────────────────────────────────────┐
│  fr10_web_teach.py                                        │
│  ThreadingHTTPServer :8890                                │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────────┐ │
│  │ 机械臂通道   │  │ 外设总线门面  │  │ RH56 独立通道     │ │
│  │ _arm_lock   │  │ motor_power  │  │ _rh56_lock       │ │
│  │ SDK 长连接  │  │ _bus.py      │  │ rh56_hand.py     │ │
│  │ 位姿轻量RPC │  │ 双总线锁     │  │                  │ │
│  └──────┬──────┘  └──────┬───────┘  └────────┬─────────┘ │
└─────────┼────────────────┼───────────────────┼───────────┘
          │                │                   │
          ▼                ▼                   ▼
   FR10 控制器      /dev/fr10-motor-serial   /dev/fr10-hand-serial
   192.168.58.2     快换电机 Modbus 0x01     电源0x01 + 夹爪0x09
   :20003 / :20004                           (+ RH56 EB90 同物理口或 ttyUSB1)
```

### 2.2 为何拆成三路

| 通道 | 协议 | 设计动机 |
|------|------|----------|
| 机械臂 | Fairino SDK（XML-RPC + 实时口） | 运动与状态读分离：轮询不抢 20004 |
| 快换电机 | Modbus RTU | 独立 FTDI；从站 **0x01**（现场与实验室 0x02 不同） |
| 末端 + RH56 | Modbus（电源/夹爪）+ EB90（手） | 电源与夹爪共享末端总线；RH56 协议栈独立，避免与 Modbus 会话混用 |

核心原则：**按物理总线拆分串口与会话锁**，承认双 FTDI / 双 USB 现场拓扑。

### 2.3 进程与并发模型

- **单进程、多线程 HTTP**：`ThreadingHTTPServer`，避免慢外设请求堵死页面加载。
- **锁**：
  - `_arm_lock`：所有 MoveJ/MoveL 与 SDK 长连接互斥。
  - `_motor_bus_lock` / `_ee_bus_lock`：分别保护电机总线与末端总线（在 `motor_power_bus.py`）。
  - `_rh56_lock`：RH56 开串口序列。
- **外设会话**：每次动作 `open → 执行 → close`（contextmanager），避免长期占用串口；代价是每次有开端口开销（相对串口等待仍可接受）。

---

## 3. 目录与模块职责

### 3.1 源码与备份

| 路径 | 职责 |
|------|------|
| `web_teach_192.168.58.2/fr10_web_teach.py` | 主程序：HTTP、API 路由、机械臂规则、内嵌前端 |
| `web_teach_192.168.58.2/motor_power_bus.py` | 双总线门面：电机 / 电源 / 夹爪 |
| `web_teach_192.168.58.2/rh56_hand.py` | RH56 EB90 协议与抓紧/打开序列 |
| `web_teach_192.168.58.2/set.py` / `reset.py` | 命令行入口（复用 `rh56_hand`） |
| `web_teach_192.168.58.2/WEB_TEACH_RULES.md` | A/B/C 挂起与二次确认规则 |
| `step`（项目根或同目录） | 关节程序点权威数值（度） |
| `FR10_web_teach_release/` | 发布/备份包（改完需同步） |
| Jetson `/opt/fr10_web_teach/web_teach_192.168.58.2/` | 现场运行副本 |

### 3.2 导入约定

`fr10_web_teach.py` 通过同目录动态导入：

- `_bus_import()`：优先 `BusActionHandler`，否则 `MotorPowerBus`（二者应互为别名）。
- `_rh56_import()`：`rh56_grip_sequence` / `rh56_open_sequence`。

部署时须保证上述 `.py` 与主程序同目录。

---

## 4. 机械臂通道设计

### 4.1 控制器端口角色

| 端口 | 用途 | 示教器用法 |
|------|------|------------|
| **20003** | XML-RPC 命令/部分状态 | `get_pose` 轻量读 TCP；SDK 内部也用 |
| **20004** | 实时状态/运动通道 | MoveJ/MoveL 依赖；**长连接复用** |
| 20005 等 | 其它实时流 | 诊断用；网页不直接连 |
| 80/8080/8082 | 控制器 Web | 与示教器独立 |

历史上「突然不能动」常见根因：

1. 控制器 20004 未监听 / 拒绝连接 → SDK 报 **-4**；
2. 示教器对每次 `get_pose` 都 `Robot.RPC()` 再 `CloseRPC()`，反复折腾 20004，导致通道不稳。

### 4.2 连接策略（关键设计）

**位姿轮询与运动分离：**

```
get_pose  →  _get_pose_light()
            仅 xmlrpc.client → http://FR10_HOST:20003 GetActualTCPPose
            不打开 SDK 全量 RPC，不占 20004

MoveJ/L   →  _rpc_ensure() 进程级长连接 _rpc_rob
            通道异常（GetSDKComState / 20004 不通）→ 关闭并重连
            单次运动失败若判定通道类错误 → force 重连并重试一次
```

进程退出时 `atexit` 关闭 `_rpc_rob`，避免僵尸占用。

### 4.3 关节程序点与数据源

常量定义在 `fr10_web_teach.py` 顶部（`HOME_JOINT`、`A_*`、`B_*`、`C_*`），须与 `step` 文件一致。  
到位判定：`GetActualJointPosDegree`，六轴最大单轴误差 ≤ `JOINT_AT_PRE_TOL_DEG`（默认 **0.8°**）。

### 4.4 挂起状态 `_deferred_pre`

进程内：`None | "A" | "B" | "C"`。

| 事件 | 状态变化 |
|------|----------|
| MoveJ 成功到达 `#A` / `#B` / `#C` | 进入对应挂起 |
| 挂起期间 | 仅允许 `get_pose` + 对应 `pre` / `out` |
| 对应 `pre` 或 `out` MoveJ 成功 | 清除挂起 |

目的：防止在工作位未退出时误发 `#home`、其它工位或 MoveL。  
详细允许表与入口路径见 `WEB_TEACH_RULES.md`（文档若仍只写 A/B，以代码中 C 对称实现为准）。

### 4.5 二次确认（服务端强制）

| `confirm_kind` | 场景 | 重试字段 |
|----------------|------|----------|
| `gripper_clear` | 去 `pre` 且当前在 out / 对应工作位 | `confirm_gripper_clear` |
| `no_fixture_at_target` | 从 `out` 进工作位 | `confirm_no_fixture_at_target` |
| `power_on`（外设） | 开电源 / 切换设备档案 | `confirm_power_on` |

前端用 `window.confirm`；取消则不带确认字段二次请求。规则在服务端执行，仅改 HTML 无法绕过关节校验与挂起。

### 4.6 MoveL / 相对位移

- `movel_pose`：body 带 `pose[6]`，可选 `v` / `ovl` / `tool` / `user`。
- `move_tcp_z_plus_80` / `move_tcp_x_plus_80` / `move_tcp_x_minus_80`：读当前 TCP → 加偏移 → MoveL（快换相关进出路径）。

受 `_deferred_pre` 约束：挂起时非白名单动作一律拒绝。

---

## 5. 外设总线设计（`motor_power_bus.py`）

### 5.1 双总线默认端口

| 总线 | 默认设备节点 | 从站 / ID |
|------|--------------|-----------|
| 电机 | `/dev/fr10-motor-serial`（亦可页面填 `/dev/ttyUSB0`） | 电机 **0x01** |
| 末端 | `/dev/fr10-hand-serial`（或 `/dev/ttyUSB1`） | 电源 **0x01**、夹爪 **0x09** |

推荐用 udev 按 USB 序列号固定 `fr10-*-serial`，避免重启后 `ttyUSB0/1` 对调。

### 5.2 门面入口

```python
MotorPowerBus.handle_action(action, payload)
# BusActionHandler = MotorPowerBus  # 兼容别名
```

`fr10_web_teach._fr10_request` 将一组 `bus_actions` 直接转交门面，示教器本身不拼 Modbus 帧。

### 5.3 设备档案（电源）

```text
DEVICE_PROFILES:
  "1" 灵巧手  8.4 V / 2.0 A
  "2" 夹爪    24.0 V / 2.0 A
```

切换流程（`_switch_device`）：

1. `output_off`
2. `set_voltage` / `set_current`
3. 需 `confirm_power_on` 后才 `output_on`
4. 更新内存态 `_device_manager_state`

夹爪动作前 `_ensure_device(..., "夹爪", auto_switch)`：当前档案不是夹爪时可自动切到 24V（仍走上述安全流程）。

### 5.4 夹爪控制时序与性能

**动作路径：** `gripper_open` / `gripper_close` → 串口会话 →（可选确保设备）→ 电源输出检查 → `move_to` → **固定行程等待 1.2s** → `read_status`。

**已做优化（保留行程等待）：**

| 项 | 原状 | 现况 |
|----|------|------|
| 动作前电源预检 | `read_basic_status`（3 读）+ `read_output_setting` | 仅 `read_output_setting`，`out_on==0` 则失败 |
| `_send_and_read` | 无条件 sleep 0.25 + 固定 sleep 0.15 再读 | 按 `last_command_time` 自适应间隔；读满 `expect_len` 即返回 |

单次开/闭预期约 **1.4–1.6s**（含 1.2s 行程），较优化前约 3.3–3.5s。

若仍要更快：可将 1.2s 改为轮询 `read_status().position` 到位提前返回（需单独评审）。

### 5.5 电机诊断

`motor_lock_diagnosis` / `motor_reset_diagnosis`：下发动作后按 `ACTION_MONITOR_SECONDS`（默认 3.5s）采样状态，用于现场判断是否真在转、是否堵转。正常网页正/反转默认可不带 monitor，以缩短反馈时间。

---

## 6. RH56 通道设计（`rh56_hand.py`）

- 协议：EB90 帧，`115200 8N1`，device ID 默认 1。
- 业务封装：`rh56_grip_sequence` / `rh56_open_sequence`（清错 → 力/速 → 角度）。
- 示教器：`rh56_grip` / `rh56_open`，默认串口 `RH56_DEFAULT_PORT = "/dev/ttyUSB1"`；payload 可覆盖 `rh56_port`、`hand_id`。
- **不**走 `motor_power_bus` 的 Modbus 会话，避免与电源/夹爪帧交错。

供电注意：手需 8.4V 档案；夹爪需 24V。换子头必须先关输出再改电压，网页「切换灵巧手 / 切换夹爪」强制确认。

---

## 7. HTTP 接口

### 7.1 约定

| 项 | 说明 |
|----|------|
| 页面 | `GET /` → 返回内嵌 HTML |
| API | `POST /api/fr10`，`Content-Type: application/json` |
| Body | `{ "action": "<字符串>", ...其它字段 }` |
| 响应 | JSON，至少含 `ok: bool`、`msg: str`；失败可含 `needs_confirm`、`confirm_kind` |

统一入口便于前端一个 `api(action, payload)` 函数；扩展新动作只需在 `_fr10_request`（或总线门面）加分支。

### 7.2 机械臂 action 一览

| action | 功能 | 主要参数 |
|--------|------|----------|
| `get_pose` | 读 TCP（轻量 XML-RPC） | 无 |
| `movel_pose` | MoveL 绝对 | `pose[6]`, `v`, `ovl`, `tool`, `user` |
| `move_tcp_z_plus_80` | TCP Z+=80 | 可选 tool/user/v/ovl |
| `move_tcp_x_plus_80` | TCP X+=80 | 同上 |
| `move_tcp_x_minus_80` | TCP X-=80 | 同上 |
| `movej_home` | `#home` | — |
| `movej_a_pre` / `movej_a` / `movej_a_out` | A 序列 | 确认字段见 §4.5 |
| `movej_b_*` / `movej_c_*` | B/C 序列 | 同 A |

### 7.3 外设 action 一览（总线门面）

| action | 功能 |
|--------|------|
| `motor_forward` / `motor_reverse` / `motor_stop` | 电机锁紧 / 复位 / 停止 |
| `motor_status` | 读电机状态寄存器 |
| `motor_lock_diagnosis` / `motor_reset_diagnosis` | 动作 + 监测采样 |
| `power_set_voltage` / `power_set_current` | 设电压 / 限流 |
| `power_output_on` / `power_output_off` | 输出开关（开需确认） |
| `power_status` | 电源状态 |
| `device_switch_hand` / `device_switch_gripper` | 切 8.4V / 24V 档案 |
| `gripper_open` / `gripper_close` / `gripper_demo` | 夹爪 |
| `bus_monitor` | 末端电源连续监测 |

常用 payload 字段：`motor_port` / `ee_port`（或兼容 `bus_port`）、`baud`、`timeout`、从站地址、`confirm_power_on`、`auto_switch_device`、`with_monitor`。

### 7.4 RH56 / 演示

| action | 功能 | 参数 |
|--------|------|------|
| `rh56_grip` | 抓紧 | `rh56_port`, `hand_id` |
| `rh56_open` | 打开 | 同上 |
| `quickchanger_demo` | 演示流程 | 如 ArUco id 等（依赖标定脚本目录） |

### 7.5 响应示例

成功：

```json
{ "ok": true, "msg": "夹爪张开完成", "gripper_status": { "...": "..." }, "ee_port": "/dev/..." }
```

需确认：

```json
{
  "ok": false,
  "needs_confirm": true,
  "confirm_kind": "gripper_clear",
  "msg": "请确定快换工具当前未装配！"
}
```

---

## 8. 前端实现思路

- HTML/CSS/JS **内嵌**于 `_HTML` 字符串，单文件部署，无构建步骤。
- 分区：机械臂关节点、TCP MoveL、相对移出、电机、末端供电/夹爪、RH56、可选 Quickchanger。
- **实时位姿**：勾选「实时同步机械臂」后对 `get_pose` 轮询（周期约 800ms 量级）；关闭后停止请求，减轻 20003 与页面负载。
- 二次确认：收到 `needs_confirm` → `confirm(msg)` → 原 action + 对应 confirm 字段重发。
- 按钮禁用：请求进行中防连点；注意末端总线锁会排队，连点夹爪仍会串行等待。

---

## 9. 安全与互锁设计小结

```
用户点击
   → 前端可选 confirm
   → POST /api/fr10
   → 服务端：
        挂起白名单？
        关节入口合法？
        需服务端二次确认字段？
        外设：电源档案 / confirm_power_on / 24V 输出已开？
   → 通过后才下发 MoveJ/MoveL 或写串口
```

原则：**安全约束放在服务端**；前端弹窗只是体验层。伪造 confirm 字段等同操作员书面确认责任。

---

## 10. 部署与运维

### 10.1 现场路径

```bash
# 开发机改完后同步示例
scp web_teach_192.168.58.2/fr10_web_teach.py \
    web_teach_192.168.58.2/motor_power_bus.py \
    web_teach_192.168.58.2/rh56_hand.py \
    jetson-orin-nx:/opt/fr10_web_teach/web_teach_192.168.58.2/

ssh jetson-orin-nx 'sudo systemctl restart fr10-web-teach.service'
# 校验
curl -s -o /dev/null -w "%{http_code}\n" http://127.0.0.1:8890/
```

同时更新本地备份：`FR10_web_teach_release/web_teach_192.168.58.2/`。

### 10.2 常见故障对照

| 现象 | 优先排查 |
|------|----------|
| 网页能开、臂不能动，SDK -4 | 控制器 20004；是否被其它脚本占；重启示教服务释放坏连接 |
| 网页卡住 / Address already in use | 旧进程占 8890：`fuser -k 8890/tcp` 后 restart |
| 电机无动作但返回 ok | 串口是否接错总线；地址是否为 0x01；是否独立供电 |
| 夹爪超时 | 是否已切 24V 且输出开；夹爪地址 9；RS485 接线 |
| RH56 无反应 | 端口、8.4V、conda/系统 Python 的 pyserial |
| ImportError BusActionHandler | `motor_power_bus` 需 `BusActionHandler = MotorPowerBus` |

### 10.3 环境依赖

- Python 3.9+，Fairino SDK（`fairino` 或 `Robot`）
- `pyserial`
- 推荐 conda 环境 `fr10-fairino`（与 systemd unit 中 ExecStart 一致）

---

## 11. 扩展指南

### 11.1 新增关节程序点

1. 在 `step` 写明关节角；
2. 在 `fr10_web_teach.py` 增加 `*_JOINT` 常量与 `movej_*` 分支；
3. 若有挂起语义，扩展 `_deferred_pre` 白名单与清除逻辑；
4. 更新 `WEB_TEACH_RULES.md` 与前端按钮；
5. 同步 release + Jetson。

### 11.2 新增外设动作

1. 在 `motor_power_bus.py`（或 RH56 模块）实现原子操作；
2. 挂入 `handle_action` / `_EE_ACTIONS` 或 `_MOTOR_ACTIONS`；
3. 在 `_fr10_request` 的 `bus_actions`（或独立分支）注册；
4. 前端加按钮与端口字段。

### 11.3 修改默认串口

改 `fr10_web_teach.py` 中 `MOTOR_DEFAULT_PORT` / `RH56_DEFAULT_PORT`，以及 HTML input 的 `value`；`motor_power_bus` 内还有 udev 逻辑名默认值，页面传参可覆盖。

---

## 12. 设计演进要点（便于回顾）

1. **单文件示教 → 双总线门面**：现场拓扑迫使电机与末端拆锁、拆端口。  
2. **位姿与运动分通道**：消灭 get_pose 轮询对 20004 的反复开关。  
3. **ThreadingHTTPServer**：慢串口不再拖死整站。  
4. **夹爪路径减负**：预检与 Modbus 节流优化，保留 1.2s 机械行程等待。  
5. **规则服务端化**：挂起 + 二次确认，避免「只改前端」的假安全。

---

## 13. 相关文档索引

| 文档 | 内容 |
|------|------|
| `web_teach_192.168.58.2/WEB_TEACH_RULES.md` | A/B（及代码中 C）挂起、确认字段 |
| `FR10_网页示教器_研究进展文档（nvidia）.md` | 进展汇报体裁、背景与成果 |
| `FR10_web_teach_release/README_RELEASE.md` | 发布包启动方式 |
| `研究进展文档_汇总.md` | 视觉线 + 示教线总览 |
| `quickchanger/HARDWARE.md` 等 | 快换硬件与标定上下文 |

---

## 14. 修订记录

| 日期 | 说明 |
|------|------|
| 2026-07-17 | 初版：架构、三通道、API、挂起/确认、夹爪性能优化、部署运维 |
