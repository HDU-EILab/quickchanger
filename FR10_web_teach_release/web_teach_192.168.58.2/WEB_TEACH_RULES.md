# FR10 网页示教器规则说明

本文档描述 `fr10_web_teach.py` 中与 **step 关节程序点**、**硬性依赖**、**二次确认** 相关的全部规则。关节角度数值以项目根目录 `step` 文件为准，代码内常量需与 `step` 保持一致。

---

## 1. 程序点与数据来源

| 名称 | 说明 |
|------|------|
| `#home` | 固定 HOME（MoveJ） |
| `#A pre` / `#A` / `#A out` | A 序列预置、工作、退出 |
| `#B pre` / `#B` / `#B out` | B 序列预置、工作、退出 |

示教器通过 `POST /api/fr10`，`body.action` 为下表之一时下发对应 `MoveJ`：

| `action` | 目标点 |
|----------|--------|
| `movej_home` | `#home` |
| `movej_a_pre` | `#A pre` |
| `movej_a` | `#A` |
| `movej_a_out` | `#A out` |
| `movej_b_pre` | `#B pre` |
| `movej_b` | `#B` |
| `movej_b_out` | `#B out` |

另有：`get_pose`（读 TCP）、`movel_pose`（MoveL），不受本节「挂起状态」约束（但若处于挂起，仍可能被拦截，见第 2 节）。

---

## 2. 关节到位判定

- 使用 `GetActualJointPosDegree` 读取当前关节（度）。
- 与某程序点是否「一致」：六轴中**最大单轴误差** ≤ `JOINT_AT_PRE_TOL_DEG`（默认 **0.8°**）即视为到位。该阈值在 `fr10_web_teach.py` 中常量 `JOINT_AT_PRE_TOL_DEG` 定义。

---

## 3. 挂起状态 `_deferred_pre`（硬性依赖）

示教器进程内维护状态：`None` | `"A"` | `"B"`。

### 3.1 何时进入挂起

- 成功执行 **`movej_a`**（到达 `#A`）后 → `_deferred_pre = "A"`。
- 成功执行 **`movej_b`**（到达 `#B`）后 → `_deferred_pre = "B"`。

### 3.2 挂起期间允许的动作

当 `_deferred_pre == "A"` 时，仅允许：

- `get_pose`
- `movej_a_pre`
- `movej_a_out`

当 `_deferred_pre == "B"` 时，仅允许：

- `get_pose`
- `movej_b_pre`
- `movej_b_out`

其余运动类指令（含 `movej_home`、`movel_pose`、`movej_b` 系、`movej_a` 等）一律拒绝，并返回提示：须先完成对应 **`#A pre` 或 `#A out`** / **`#B pre` 或 `#B out`**。

### 3.3 何时清除挂起

仅在**对应**动作 **MoveJ 成功** 后清除：

| 挂起 | 清除条件 |
|------|----------|
| `"A"` | 成功执行 `movej_a_pre`（且此前挂起为 A）或成功执行 `movej_a_out`（且此前挂起为 A） |
| `"B"` | 成功执行 `movej_b_pre` 或 `movej_b_out`（且此前挂起为 B） |

若动作失败，挂起状态不变。

### 3.4 注意

- 状态保存在**示教器服务端进程内存**中；**重启进程**后状态丢失，与真实机械臂姿态可能不一致，需谨慎操作或先移动到安全已知位。

---

## 4. 进入 `#A` / `#B` 的路径（入口校验）

### 4.1 进入 `#A`（`movej_a`）

当前关节须满足以下**之一**：

1. 已在 **`#A pre`**（在阈值内）→ 直接允许执行。
2. 已在 **`#A out`** → 须带 **`confirm_no_fixture_at_target`**（见第 6 节）；否则返回需确认，不执行移动。
3. 其它位置 → 拒绝，提示须从 `#A pre` 或经确认的 `#A out` 进入。

### 4.2 进入 `#B`（`movej_b`）

与 4.1 对称：允许从 **`#B pre`**，或从 **`#B out`** 且带 **`confirm_no_fixture_at_target`**。

---

## 5. 进入 `#A pre` / `#B pre` 时的快换确认（`confirm_gripper_clear`）

下发 **`movej_a_pre`** 或 **`movej_b_pre`** 时，若满足下列**任一**条件，则必须先通过「快换工具未装配」确认（见第 6 节），否则不执行移动：

| 条件 | 说明 |
|------|------|
| A | 当前关节落在 **`#A out` 或 `#B out`**（任一在阈值内），且本次为去 **`#A pre` 或 `#B pre`**（out 对两条 pre 都触发确认） |
| B | 当前关节落在 **`#A`**，且本次为 **`movej_a_pre`** |
| C | 当前关节落在 **`#B`**，且本次为 **`movej_b_pre`** |

确认文案（服务端 `msg` 与网页弹窗一致）：**「请确定快换工具当前未装配！」**

---

## 6. 二次确认与 API 字段

当服务端需要用户确认时，返回示例字段：

```json
{
  "ok": false,
  "needs_confirm": true,
  "confirm_kind": "<类型>",
  "msg": "<提示文案>",
  "sdk": "..."
}
```

### 6.1 `confirm_kind: "gripper_clear"`（快换工具）

- 适用场景：见 **第 5 节**。
- 用户确认后，**再次**请求同一 `action`，并在 JSON 中增加：
  - **`confirm_gripper_clear`**：`true`，或 `1`，或字符串 `"1"` / `"true"` / `"yes"` / `"y"`（大小写不敏感）。

### 6.2 `confirm_kind: "no_fixture_at_target"`（目标位无快换装置）

- 适用场景：从 **`#A out`** 去 **`#A`**，或从 **`#B out`** 去 **`#B`**（见 **第 4 节**）。
- 文案：**「请确定目标位置没有快换装置！」**
- 确认后再次请求须带：
  - **`confirm_no_fixture_at_target`**：真值规则同 `confirm_gripper_clear`。

网页端对上述两类均使用 `window.confirm`；若用户取消，不发起带确认的第二次请求。

---

## 7. 网页端与脚本的关系

- 规则在 **服务端** 强制执行；仅用浏览器改请求无法绕过关节校验与挂起逻辑（除非伪造确认字段，仍属人为承担责任）。
- 非浏览器客户端调用 `POST /api/fr10` 时，须自行处理 `needs_confirm` / `confirm_kind`，并在用户确认后附带对应布尔字段重试。

---

## 8. 相关文件

| 文件 | 作用 |
|------|------|
| `fr10_web_teach.py` | 示教器实现（规则以代码为准） |
| 项目根目录 `step` | 各程序点关节角（度）来源 |
| `WEB_TEACH_RULES.md` | 本文档（规则汇总） |

若代码与文档冲突，以 **`fr10_web_teach.py` 当前实现** 为准。
