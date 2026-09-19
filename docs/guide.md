# 操作说明

[English](guide.en.md) · **简体中文**

本文说明 layout-kernel 对外开放的全部接口：怎么调用、传什么、返回什么、出错时怎么表现，并配有可运行的示例。

- 任务书的完整字段、目标函数、约束注册表：见 [contract.md](contract.md)。
- 数学模型与求解方法：见 [model.md](model.md)。

**目录**

1. [选哪个入口](#1-选哪个入口)
2. [安装](#2-安装)
3. [通用约定](#3-通用约定)
4. [入口 A：三维场景接口（接入已有项目）](#4-入口-a三维场景接口接入已有项目)
5. [入口 B：任务书（摆放 + 布管）](#5-入口-b任务书摆放--布管)
6. [入口 C：Python 模块](#6-入口-cpython-模块)
7. [约束配置](#7-约束配置)
8. [调参](#8-调参)
9. [出错与排查](#9-出错与排查)

---

## 1. 选哪个入口

| 你的情况 | 用哪个入口 | 调用方式 |
|---|---|---|
| 已有项目，自己管理设备实例、姿态、管路，想让内核重布管、挪设备 | **A 场景接口** | 命令行 `layout-kernel-scene`（任何语言都能以子进程调用），或 `scene.optimize_scene()` |
| 只有设备清单和连接关系，要内核决定设备放哪、管怎么走 | **B 任务书** | `solve()` 或命令行 `layout-kernel` |
| 要在自己的算法里单独用布管器、校验器、下界 | **C Python 模块** | `routing` / `constraints` / `scene` 各函数 |

三个入口共用同一套布管器、约束注册表和独立校验器。

## 2. 安装

需要 Python 3.11 或更高版本。请装在独立的虚拟环境里，因为 `ortools` 会升级 `protobuf`，可能和其他项目冲突：

```bash
python -m venv .venv
.venv/Scripts/python -m pip install "layout-kernel @ git+https://github.com/66-zhimeng/layout-kernel"
```

安装后得到两个命令：`layout-kernel`（任务书）、`layout-kernel-scene`（场景）。

**调用方不一定要装内核的依赖。** 另一个项目可以只记住内核虚拟环境里 Python 的路径，以子进程方式调用 `python -m layout_kernel.scene_cli`，自己的环境不需要 numpy、numba 这些依赖（写法见 4.6 节）。

开发时这样安装：

```bash
git clone https://github.com/66-zhimeng/layout-kernel && cd layout-kernel
python -m venv .venv && .venv/Scripts/python -m pip install -e ".[test,plot]"
.venv/Scripts/python -m pytest -q
```

## 3. 通用约定

1. **参数不设默认值。** 缺少任何必填项都会报错，并指出缺的是哪一项。这是有意的：每个数值都对应一条工艺规定，默认值会悄悄改变结果。
2. **约束全部由调用方开关。** 13 条约束中的每一条都必须写 `"enabled": true/false`；打开时要给全参数；写了不认识的名字也报错（第 7 节）。
3. **结果以独立校验器为准。** 返回的 `ok`、`violations`、指标都来自只看几何的校验器 `routing.check_routes`，不依赖求解器内部数据。
4. **失败是正常返回。** 布不通、有违规时返回 `ok: false` 和违规原文；只有输入本身不合法才抛异常（命令行则返回错误 JSON 和非 0 退出码）。
5. **启发式，附下界。** 方案是可行的较优解，不是最优性证明；`lower_bound.gap` 说明离下界最多差多少（4.3 节）。
6. **单位**：场景接口使用米，**Y 轴向上**（常见三维引擎的约定）；任务书和 Python 模块使用毫米，**Z 轴向上**。场景接口在内部自动换算。
7. **多进程**：`route_workers > 1` 或任务书摆放会用多进程（spawn 方式）。在 Python 里直接调用时，入口代码必须放在 `if __name__ == "__main__":` 之下。

---

## 4. 入口 A：三维场景接口（接入已有项目）

### 4.1 调用

**命令行**：标准输入读请求 JSON，标准输出写结果 JSON，过程日志写到标准错误。

```bash
layout-kernel-scene < request.json > result.json
# 或：<内核虚拟环境的 python> -m layout_kernel.scene_cli < request.json > result.json
```

| 情况 | 标准输出 | 退出码 |
|---|---|---|
| 正常（包括 `ok: false`，即没找到无违规方案） | 结果 JSON（4.3 节） | 0 |
| 请求不合法，或内核内部出错 | `{"ok": false, "error": "异常类型: 说明"}` | 1 |

**Python**：

```python
from layout_kernel import scene

if __name__ == "__main__":
    out = scene.optimize_scene(inp, settings, log=print)   # 布管 + 平移 / 旋转 / 换口 + 下界
    out = scene.route_scene(inp, settings)                  # 只重布管，设备不动
```

- `route_scene(inp, settings, fixed_ids=(), log=None)`：`fixed_ids` 中的管保持原路径，作为障碍。它返回的字段比 `optimize_scene` 少：没有平移、朝向和下界。

**可运行示例**：[examples/scene/request.json](../examples/scene/request.json)，脚本是 [examples/scene/run_scene.py](../examples/scene/run_scene.py)。

- **场景**：泵出口和冷水机组进口不在同一条线上。中间的在线传感器位置不好，两根管各拐两个弯。
- **结果**：内核把传感器沿 z 方向平移 −0.3 m，弯头 4 → 2，离下界差距 0%，用时约 0.2 s。
- 这个示例同时是测试用例（`tests/test_scene.py`），保证它一直能跑通。

```bash
python examples/scene/run_scene.py
```

### 4.2 请求

```jsonc
{
  "input":    { /* 场景，见下表 */ },
  "settings": { /* 设置，见 4.2.4 */ }
}
```

#### 4.2.1 `input.nodes`：节点（设备、三通、阀门、传感器……）

| 字段 | 类型 | 说明 |
|---|---|---|
| `id` | 字符串 | 节点 id，全局唯一 |
| `move` | 布尔 | 能否平移。平移范围见 `input.radius` |
| `orientations` | 数组，至少 1 个 | **第 0 个是当前姿态**，其余是允许换成的姿态（旋转、三通换向、同向口交换）。每个姿态的端口都要由调用方按该姿态算好 |
| `orientations[].ports` | `{端口 key: {position, normal}}` | 端口 key 在整个场景里唯一；`position` 为 `[x, y, z]`（米）；`normal` 为出管方向的单位向量。斜向（非轴向）法向表示斜支口，见 `oblique_stub_mm` |
| `orientations[].box` | `{min, max}` 或 `null` | 包围盒（米）。`null` 表示没有实体盒的节点（如三通），它不挡管，实体用 `spools` 表示 |
| `orientations[].spools` | `[[p, q], ...]` | 节点内部短管（如三通内的接管），每段两个端点，单位米。对不接在这个节点上的管是障碍 |
| `orientations[].angle`、`swap` | 任意 | 内核不读取，原样留给调用方识别这是哪个姿态 |

只有一个姿态且 `move: false` 的节点不会被改动。

#### 4.2.2 `input.routes`：管道（每根两个端点）

| 字段 | 类型 | 说明 |
|---|---|---|
| `id` | 字符串 | 管 id |
| `code` | 字符串 | 显示用编号，会原样带回结果 |
| `from.key` / `to.key` | 字符串 | 两端的端口 key |
| `points` | `[[x,y,z], ...]` | 现有折线（米），**包括两端端口**。内核把它作为比较基准；固定管则按它当障碍 |
| `segments` | `[{r}, ...]` | 管外半径（米），取最大值作为这根管的外径 |
| `leadA` / `leadB` | 数值（米） | 起端、末端的直颈加变径长度，用来推算端口到第一个弯头所需的直管（见 `pipe_rules`） |
| `fixed` | 布尔 | `true` 表示不改这根管（锁定管、局部优化范围外的管）；它作为硬障碍，并参与净距校验 |
| `low` | 布尔 | 低位管，受 `low_pipes` 约束 |
| `weight_length` / `weight_bends` / `weight_height_changes` | 数值，可选 | 这根管在目标里的权重倍数，不写时为 1。例如主管弯头更贵，可以设 `weight_bends: 3` |

#### 4.2.3 `input` 的其他字段

| 字段 | 说明 |
|---|---|
| `radius` | 每轴平移范围（米），相对原始位置。设为 0 表示不平移 |
| `seconds` | 时间上限（秒），见 4.5 节 |
| `equipment_keepout` | 可选。设备禁区 `[[x0,y0,z0,x1,y1,z1], ...]`（米），在 `equipment_keepout` 约束打开时生效 |
| `pipe_keepout` | 可选。管道禁区，格式同上，在 `pipe_keepout` 约束打开时生效 |

#### 4.2.4 `settings`（全部必填）

| 键 | 说明 |
|---|---|
| `routing` | 求解参数（毫米）加 `constraints`，逐项说明见 [contract.md 第 6 节](contract.md)；约束见第 7 节 |
| `weights` | `{area, length, bends, height_changes}`，目标里各项的权重 |
| `scale` | `{A0, L0, B0, C0, kappa, l_min_mm}`，归一化尺度（占地 mm²、管长 mm、弯头数、高度变化数）、长宽比上限、最小直管 |
| `oblique_stub_mm` | 斜支口在基线里找不到斜段时，沿法向伸出的斜管长度 |
| `pipe_rules` | 调用方的直管和弯头规则，内核换算成直管长度要求：`{trim_ratio, radius_margin_mm, port_margin_mm, safety, rule_D_mm, rule_R_mm}`，见 [model.md 第 3.2 节](model.md#32-管路几何) |
| `rotation_candidates` | 每个可转节点每轮最多真实重布几个朝向（先按估计排序） |
| `lower_bound` | `{enabled, max_expansions}`。打开时报告下界和差距 |
| `candidate_routing` | 评估候选时覆盖 `routing` 的轻量参数，如 `{max_iters, stall_iters, max_expansions, cleanup_max_expansions}`，另外**必须有 `window_mm`**：候选只在相关管和被移动节点周围、外扩这么多的窗口内布管 |
| `pose_negotiation` | `{enabled, max_poses, pres_bends, hist_bends}`，姿态协商（4.4 节）；`enabled: false` 时其余三项仍要写 |

一套完整可用的设置，可以直接复制 [examples/scene/request.json](../examples/scene/request.json) 里的 `settings` 再按需修改。

### 4.3 结果

```jsonc
{
  "ok": true,                         // 全部布通，且独立校验无违规
  "violations": [],                   // 违规原文
  "routes": [{"id": "pipe_1", "code": "供水 1", "points": [[1.2,0.5,0.4], [3.0,0.5,0.4]]}],
  "offsets": {"sensor": [0, 0, -0.3]},        // 被平移的节点 → 平移量（米，Y 向上，相对原始位置）
  "orientations": {},                          // 换了姿态的节点 → 输入 orientations 里的下标
  "moves": [{"move": "串联拉直 1 个设备到直线 (0.5, 0.4)", "gain": 6.0}],   // 接受的改进
  "base_cost": 17.0, "cost": 11.0,             // 基准代价与最终代价（内核目标口径）
  "metrics": {"W_m": 8.0, "H_m": 2.5, "area_m2": 20.0, "L_m": 5.0, "bends": 2, "height_changes": 0, "J": 11.0},
  "lower_bound": {"valid": true, "value": 11.0, "cost": 11.0, "gap": 0.0, "nets": 2, "unresolved": {}, "seconds": 0.0, "note": "…"},
  "candidates_tried": 5, "iterations": 1, "grid_nodes": null,
  "timing": {"route_s": 0.2}
}
```

**调用方怎么施加结果**：

1. 对 `orientations` 里的每个节点，换成输入里对应下标的姿态（旋转、换向或换口，调用方自己知道 `angle`、`swap` 的含义）。
2. 对 `offsets` 里的每个节点，整体平移。
3. 用 `routes[].points` 替换管路。**点列包括两端端口**；斜支口的斜段也在其中（端口到斜段终点）。
4. 结果中没有出现的管，保持原路径不变。
5. **建议调用方用自己的规则再校验一遍**。内核的校验口径已经尽量与调用方对齐，但几何细节（例如弯头扫掠体）以调用方为准。

**`lower_bound` 的含义**：把设备位置和朝向固定为最终方案，每根两端点管单独求精确最短路（不考虑其他待布管，放宽同一根管自身的净距），各管加起来就是 `value`。`gap = (cost − value) / cost`，表示当前方案最多比这个下界差多少。

- 它**不是**设备还能移动时的全局下界。
- 多端点管网不给下界。
- `unresolved` 列出单独求解没完成的管，这时 `valid: false`。

### 4.4 优化过程

1. **基准**：把输入里的现有路径用内核校验器检查。通过就直接作为基准；只有少数管不符合时，只重布这几根；不符合的太多则整网重布。
2. **姿态协商**（可选，`pose_negotiation.enabled`）：把每个可动节点的候选姿态并入协商布线，一次选定各节点姿态（原理见 [model.md 第 5.11 节](model.md#511-姿态协商)）。
3. **逐轮候选改进**：
   - 候选有三类：串联拉直（相连的在线设备整串平移到一条线上）、单个对齐、换朝向；
   - 用 `route_workers` 个进程并行评估，每个候选只在局部窗口内重布相关的管；
   - 有改进的候选里，挑出互不相干的一批，复核后一次接受。
4. **收尾**：没有改进或时间到了以后，时间还有剩余就按最终姿态完整重布一次，取更好的；否则把逐步结果在全场景完整校验一次（不通过则强制重布）。
5. **下界**：按最终姿态计算。

### 4.5 时间上限

`seconds` 限制第 3 步：时间一到就停止收集候选结果，只在已评估的候选里挑，并跳过第 4 步的完整重布。以下几项不在限时内：

- 第 1 步基准（通常几秒）；
- 第 2 步姿态协商（大场景可能要几十秒，时间紧时建议关掉）；
- 第 5 步下界（大场景约 5 s）。

参考数据（230 根管、12 进程）：

| `seconds` | 实际用时 | 结果 |
|---|---|---|
| 60 | 约 70 s | 离下界 1.9% |
| 120 | 约 130 s | 离下界 1.2%，约 110 s 时收敛 |

### 4.6 从其他语言调用

**Python 子进程**（调用方不装内核依赖）：

```python
import json, subprocess
KERNEL_PY = r"D:/path/to/layout-kernel/.venv/Scripts/python.exe"
done = subprocess.run([KERNEL_PY, "-m", "layout_kernel.scene_cli"],
                      input=json.dumps({"input": inp, "settings": settings}).encode("utf-8"),
                      capture_output=True, timeout=900)
out = json.loads(done.stdout.decode("utf-8"))
if done.returncode != 0:
    raise RuntimeError(out["error"])
```

**Node.js**：

```js
import {spawnSync} from 'node:child_process';
const p = spawnSync(KERNEL_PY, ['-m', 'layout_kernel.scene_cli'], {input: JSON.stringify({input, settings}), maxBuffer: 1 << 28});
const out = JSON.parse(p.stdout.toString('utf8'));
if (p.status !== 0) throw new Error(out.error);
```

**浏览器**：不能直接起进程。需要由本机的一个小服务接收请求（例如 `POST /api/layout`），用上面的子进程写法转发给内核，再把结果返回浏览器。

---

## 5. 入口 B：任务书（摆放 + 布管）

```python
from layout_kernel import solve, validate_case, CaseError

if __name__ == "__main__":
    try:
        result = solve("examples/case_small_plant.json", log=print)   # 也可以传 dict
    except CaseError as ex:
        print("任务书不合法：", ex)
```

```bash
layout-kernel examples/case_small_plant.json -o plan.json      # -q 不输出过程日志
```

| 接口 | 说明 |
|---|---|
| `solve(case, log=None, work_dir=None)` | `case` 是路径或 dict；`log(line)` 接收日志；`work_dir` 放中间文件（默认临时目录）。返回方案 dict |
| `validate_case(case)` | 只检查任务书，不求解；不合法时抛 `CaseError`（指出哪一项） |
| 命令行退出码 | 0 = 成功；1 = 有违规或未布通；2 = 任务书不合法 |

- **两种模式**：`task.mode = "place_and_route"` 由内核决定设备位置；`"route_only"` 表示每台设备都已经给了 `placed`，只布管。
- **字段说明**：任务书和方案的完整字段见 [contract.md 第 2–3 节](contract.md)。
- **示例**：[examples/case_small_plant.json](../examples/case_small_plant.json)；[examples/generate_cases.py](../examples/generate_cases.py) 可以生成不同规模的算例。

---

## 6. 入口 C：Python 模块

所有函数都用**毫米、Z 轴向上**。

### 6.1 `layout_kernel.routing`：布管与校验

```python
from layout_kernel import routing as rt

sc = rt.Scene(devices, nets, routing_params, weights, scale,
              fixed_routes=None, spools=None, pipe_keepout=None)
routes, history, G = rt.negotiate(sc, log=print)
viol, metrics = rt.check_routes(sc, routes)
```

| 接口 | 输入 | 返回 |
|---|---|---|
| `Scene(devices, nets, rp, weights, scale, fixed_routes, spools, pipe_keepout)` | 见下表 | 场景对象；参数缺失时抛 `KeyError`，约束配置不对时抛 `ConstraintError` |
| `negotiate(sc, log=print, cleanup_enabled=True)` | 场景 | `(routes, history, G)`：`routes = {管网 id: [支路, ...]}`；`history` 是每轮的统计；`G` 是网格 |
| `check_routes(sc, routes)` | 场景、路径 | `(violations, metrics)`：违规原文列表；`metrics = {W_m, H_m, area_m2, L_m, bends, height_changes, J, per_net}` |
| `net_cost(sc, nid, per_net[nid])` | | 单根管的目标代价（含单管权重） |
| `lower_bounds(sc, max_expansions, workers)` | | `{管网 id: (下界代价或 None, 说明)}` |
| `cleanup(G, sc, routes, log)` | | `(routes, records)`：其他管作为硬障碍，逐个消除冲突 |
| `Grid(sc)`、`route_net(G, sc, net, ctx)`、`astar(...)` | | 底层，一般不需要直接调用 |

**`devices`**：`{设备 id: {...}}`，各字段如下：

| 字段 | 说明 |
|---|---|
| `box` | `(x0,y0,z0,x1,y1,z1)`，或 `None` 表示没有实体盒 |
| `zones` | `[(x0,y0,x1,y1), ...]` 检修区平面矩形 |
| `ports` | `{端口名: (x,y,z,ux,uy,uz)}`，方向必须是轴向单位向量 |
| `fitting_ports` | 可选。端口外已经有管件（按弯头计算直管）的端口名列表 |

**`nets`**：`[{...}, ...]`，各字段如下：

| 字段 | 说明 |
|---|---|
| `id`、`terms` | `terms = [(设备, 端口), ...]`，端点多于 2 个时自动接三通 |
| `D_mm`、`rho_mm`、`lead_mm` | 可选。外径、弯曲半径、最小直管 |
| `zc_max_mm` | 可选。中心线高度上限 |
| `port_straight_mm` | 可选。`{端口名: 端口到第一个弯所需直管}` |
| `weight_length` / `weight_bends` / `weight_height_changes` | 可选。单管权重倍数 |

**`fixed_routes`**：`{id: {"points": [[x,y,z], ...], "D_mm", 可选 "trim_nodes": (起端三通 id 或 None, 末端 …)}}`，不参与布管，作为硬障碍。

**`spools`**：`[{"owner": 节点 id, "points": [...], "D_mm"}]`，节点内部短管。

**支路格式**：`{"start": (设备, 端口), "points": [(x,y,z), ...], "end": ("port", (设备, 端口)) 或 ("tee", None)}`，相邻两点只沿一个轴变化。

**完整示例**：[examples/minimal_routing.py](../examples/minimal_routing.py)，每个参数都有注释。

### 6.2 `layout_kernel.constraints`：约束注册表

| 接口 | 说明 |
|---|---|
| `REGISTRY` | `{约束名: {"doc", "params"}}` |
| `validate(cfg)` | 检查约束配置，返回规范化后的 dict；缺项、多项、类型不对都抛 `ConstraintError` |
| `describe()` | 各约束的说明和参数，可以直接用来生成配置界面 |

### 6.3 `layout_kernel.scene`：场景

| 接口 | 说明 |
|---|---|
| `optimize_scene(inp, settings, log=None)` | 入口 A 的函数形式 |
| `route_scene(inp, settings, fixed_ids=(), log=None)` | 设备不动，只重布 |
| `build_scene(inp, settings, fixed_ids=(), deltas=None)` | 场景输入转换成 `(routing.Scene, 元数据)`，可以接着用 6.1 的函数 |
| `to_k(p)` / `to_w(p)` | 场景坐标（米、Y 向上）与内核坐标（毫米、Z 向上）互相换算 |

---

## 7. 约束配置

约束写在 `routing.constraints` 下，注册表里的 13 条每一条都要出现：

```jsonc
"constraints": {
  "pipe_pipe_clearance":      {"enabled": true,  "gap_mm": 25},
  "pipe_equipment_clearance": {"enabled": true,  "gap_mm": 25},
  "self_clearance":           {"enabled": true,  "skip_along_mm": 550},
  "height_change_limit":      {"enabled": false},
  "ceiling":                  {"enabled": true,  "z_max_mm": 10000},
  "service_zones":            {"enabled": false},
  "straight_lengths":         {"enabled": true},
  "low_pipes":                {"enabled": true,  "zc_max_mm": 3199.9},
  "junction_merge_exemption": {"enabled": true},
  "internal_spools":          {"enabled": true},
  "equipment_spacing":        {"enabled": true,  "gap_mm": 120},
  "equipment_keepout":        {"enabled": false},
  "pipe_keepout":             {"enabled": false}
}
```

- 每条约束的含义、参数、关闭时的效果：见 [contract.md 第 5 节](contract.md)，或调用 `constraints.describe()`。
- 规则：关闭时只写 `enabled`；打开时参数必须写全；`self_clearance` 依赖 `pipe_pipe_clearance`，两者要同时打开。

---

## 8. 调参

| 想要 | 调什么 |
|---|---|
| 更快 | 调小 `seconds`；关掉 `pose_negotiation`；调小 `candidate_routing.max_iters`、`max_expansions`；调大 `routing.pitch_mm`（网格更粗）；关掉 `port_side_lines` |
| 更好 | 调大 `seconds`（120–300 s 通常能收敛）；调大 `radius`；把 `rotation_candidates` 调大一点 |
| 候选能找到更远的绕行 | 调大 `candidate_routing.window_mm`（窗口越大越慢） |
| 更少弯头、宁可多走点管 | 调大 `weights.bends`；个别主管单独设 `weight_bends` |
| 大场景布不通或太慢 | 调大 `max_expansions`；`astar_weight` 设为 1.2–1.5（更快，但单管不保证最优）；`route_workers` 设为 CPU 核数 |

`weights` 与 `scale` 一起决定“1 个弯头相当于多少米管”。例如 `bends: 3, B0: 1, length: 1, L0: 1000` 表示 1 个弯头相当于 3 m 管长。

## 9. 出错与排查

| 现象 | 原因 | 怎么办 |
|---|---|---|
| `KeyError: 布管缺少必填参数：…`、`场景布管缺少设置：…` | 缺少必填项（不设默认值） | 按提示补上 |
| `ConstraintError: 约束配置缺少 …` / `未知的约束 …` | 约束表不完整或拼错 | 13 条都写出来，名字对照第 7 节 |
| `violations` 里有 `端口正前方不足 ℓ_min` | 端口前方被设备或其他管堵住 | 检查端口法向和周围的设备、固定管 |
| `搜索未找到路径（或超过扩展上限）` | 真的无路，或单管搜索规模超过 `max_expansions` | 先调大 `max_expansions`；清理记录里会区分“确实无路”和“超过上限” |
| `管–管净距不足` 残留 | 协商和清理都没消掉 | 调大 `max_iters`；放宽 `radius` 让设备可以挪开；检查固定管是否挡住 |
| 日志出现 `到达时间上限：本轮只评估了 x/y 个候选` | 时间不够 | 调大 `seconds` |
| `端口超出布管范围` | 候选把端口移到网格之外（例如高过顶棚） | 正常现象，这个候选会被判为不可行 |
| Windows 上多进程报错或卡住 | 调用代码没放在 `if __name__ == "__main__":` 之下 | 放进去；命令行入口不受影响 |
| 调用方校验不通过，但内核 `ok: true` | 两边口径有差异 | 以调用方为准。把请求和结果存下来，对比违规的管和那段几何；需要时调整 `pipe_rules`、净距参数 |
