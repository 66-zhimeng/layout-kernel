# 计算内核契约

外部只通过这一份契约调用内核：一份**任务书**（JSON / dict）进，一份**方案**（JSON / dict）出。

```python
from layout_kernel import solve, validate_case, CaseError

result = solve("任务书.json")        # 或 solve(case_dict)
```

命令行：

```bash
.venv/Scripts/python -m layout_kernel 任务书.json -o 方案.json
```

> `place_and_route` 模式用多进程（spawn），调用方必须放在 `if __name__ == "__main__":` 里。

---

## 1. 原则

1. **参数不设默认值。** 缺任何一项都会抛 `CaseError`，并指出缺的是哪一项。默认值会悄悄改变结果，而这里每个数值都对应工艺规定。
2. **长度单位一律 mm，角度为度**（只允许 0 / 90 / 180 / 270）。坐标都必须是 `params.grid_mm` 的整数倍。
3. **结果里的 J、指标、违规全部来自独立校验器**（`routing.check_routes`、`placement_cpsat.validate`），不依赖求解器内部数据。求解器说自己做到了不算数。
4. **失败是正常返回，不是异常。** `result["ok"] = false` 并在 `violations` 里给出原因；只有任务书不合法才抛 `CaseError`。

## 2. 任务书

```jsonc
{
  "params": {
    "grid_mm": 100,                      // 摆放网格：所有坐标的最小单位
    "delta_ee_mm": 800,                  // 设备—设备最小间距
    "l_min_mm": 300,                     // 两个管件之间的最小直管长度 ℓ_min
    "kappa": 2,                          // 占地矩形允许的最大长宽比
    "service_zones_inside_footprint": false,  // 检修区是否必须落在占地矩形内
    "weights": {"area": 1.0, "length": 1.0, "bends": 0.3, "height_changes": 0.3},
    "routing": { /* 21 项布管参数，见第 5 节 */ },
    "blocking": { /* 9 项分块参数，见第 5 节 */ }
  },

  "device_types": {
    "泵": {
      "height": 1000,                    // 设备高度
      "size": [1200, 800],               // 平面尺寸（未旋转时 x, y）
      "rotations": [0, 90, 180, 270],    // 允许的旋转角
      "service_zones": [[0, -800, 1200, 0]],   // 检修区（相对设备原点的 x0,y0,x1,y1）
      "ports": {
        "in":  {"pos": [0, 400],    "dir": [-1, 0], "z": 500},   // 位置、必须先直出的方向、标高
        "out": {"pos": [1200, 400], "dir": [1, 0],  "z": 500}
      }
    }
  },

  "devices": [
    {"id": "泵1", "type": "泵"},
    {"id": "泵2", "type": "泵", "fixed":  [2000, 2000]},        // 可选：钉死平面位置（mm）
    {"id": "泵3", "type": "泵", "placed": [5000, 3000, 90]}     // route_only 模式必填：位置 + 旋转角
  ],

  "nets": [
    {"id": "供水", "terminals": ["泵1.out", "机组1.in"]}         // 端点 ≥ 3 时自动接三通
  ],

  "keepout": [[x0, y0, z0, x1, y1, z1]],   // 可选：管道禁区（设备可以占，管道不能进）

  "task": {
    "mode": "place_and_route",   // 或 "route_only"
    "time_budget_s": 60,         // 每个候选摆放的搜索秒数
    "candidates": 3,             // 候选摆放数（种子 seed, seed+1, …），逐个完整布管后按真实 J 选
    "seed": 0,
    "workers": 12                // 摆放搜索的进程数（布管的进程数是 params.routing.route_workers）
  }
}
```

`route_only` 模式下 `task` 只需要 `mode`，但每台设备都要给 `placed`。

**注意 `blocking.auto_module_policy`**：设为 `accept` 会把自动识别的重复结构（如冷却塔组、主机—泵链）合并成刚性模块再摆放。实测这样**布管明显更难**：小冷站上 6 个候选全部有违规（5–7 处）；设为 `report_only`（只报告、不合并）时 3 个候选中 2 个零违规，最终方案零违规（见 `分块策略对比_log.txt`）。原因推测是模块内部排法只按设备间距紧凑排列，没有为出管留空间；出管留空只作用在块的外边界。在修好之前建议用 `report_only`。

**注意 `fixed` 的坐标系**：摆放会在设备四周为出管预留空间，所以整个布局不会贴着原点。固定坐标太靠近原点（例如 `[0, 0]`）会与预留空间冲突而无解——这时 `violations` 会直接说明原因。

## 3. 方案（返回值）

```jsonc
{
  "ok": true,                       // 全部布通且独立校验无违规
  "mode": "place_and_route",
  "violations": [],                 // 校验器报出的每一条违规（原文）
  "unrouted": {},                   // 未布通的管网 → 原因
  "metrics": {"W_m": 19.8, "H_m": 16.7, "area_m2": 330.7, "L_m": 383.3,
              "bends": 146, "height_changes": 53, "J": 6.3521},   // 含管道的实际指标
  "per_net": [{"net": "供水", "L_mm": 12345, "bends": 6, "height_changes": 2, "branches": 1}],
  "lower_bound": {"area_m2": ..., "L_lb_m": ..., "bends_lb": ..., "J_full_weights": ...},
  "devices": {"泵1": {"x_mm": 2000, "y_mm": 2000, "rot_deg": 90,
                      "box_mm": [x0,y0,z0,x1,y1,z1],
                      "ports_mm": {"out": [x, y, z]}}},
  "routes": {"供水": [{"start": ["泵1", "out"],
                       "end": ["port", ["机组1", "in"]],      // 或 ["tee", null]：接在本管网的三通上
                       "points_mm": [[x,y,z], ...]}]},        // 折线，相邻点只沿一个轴变化
  "candidates": [{"seed": 0, "ok": true, "J": 6.35, "area_m2": ..., "L_m": ...,
                  "violations": 0, "route_s": 15.2, "placement_s": 60.3}],
  "chosen_seed": 0,
  "route_s": 15.2, "route_iters": 9, "total_s": 91.4
}
```

一个管网可能有多条支路：第一条连接两个端点，其余每条从一个端点接到已布管道上的三通（`end = ["tee", null]`）。

## 4. 目标函数

```
J = w_A · 占地/A₀ + w_L · 管长/L₀ + w_B · 弯头/B₀ + w_C · 高度变化/C₀
```

权重在 `params.weights` 里调：更在意占地就调大 `area`，不想多拐弯就调大 `bends`。当前算例下 1 个弯头相当于约 3.4 m 管长，调权重就是在改这个兑换率。

## 5. 约束（全部由调用方配置）

约束写在 `params.routing.constraints` 下。**注册表里的每一条都必须写出 `enabled`**；打开时必须给出全部参数；写了不认识的约束名直接报错。内核不写死任何约束，也不设默认值。说明可用 `layout_kernel.constraints.describe()` 取得（可直接生成配置界面）。

| 约束 | 参数 | 含义 | 关闭时 |
|---|---|---|---|
| `pipe_pipe_clearance` | `gap_mm` | 不同管外壁之间的最小净距 | 管道之间不避让、不校验 |
| `pipe_equipment_clearance` | `gap_mm` | 管外壁与设备包围盒的最小净距 | 按 0 处理（可贴、不可穿） |
| `self_clearance` | `skip_along_mm` | 同一根管上沿管长相隔超过此值的两段也要满足管间净距（防回绕、自交）；依赖 `pipe_pipe_clearance` | 不查 |
| `height_change_limit` | `max_changes` | 每条支路高度变化次数上限 | 不限 |
| `ceiling` | `z_max_mm` | 管顶最高标高 | 只受场景范围限制 |
| `service_zones` | `height_mm` | 设备检修区在此高度以下不得走管 | 忽略检修区 |
| `straight_lengths` | — | 管件之间的最短直管（弯曲半径 + ℓ_min，或调用方给出的每管、每端规则） | 只要求正交 |
| `low_pipes` | `zc_max_mm` | 标为 low 的管的中心线最高标高 | 忽略 low 标记 |
| `junction_merge_exemption` | — | 汇合于同一无盒节点（三通）的几根管，在口附近不算彼此冲突 | 口附近也按管间净距判 |
| `internal_spools` | — | 节点内部短管是其他管的障碍（接在该节点上的管豁免首末段） | 忽略内部短管 |
| `equipment_spacing` | `gap_mm` | 移动设备时：设备间距；被移动设备与固定管道之间满足管—设备净距 | 不查 |

示例（与 拆件做网页 的校验口径一致）：

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
  "equipment_spacing":        {"enabled": true,  "gap_mm": 120}
}
```

## 6. 求解参数（全部必填）

`params.routing` 中除 `constraints` 外是求解器本身的参数，不是约束：

| 参数 | 含义 |
|---|---|
| `D_default_mm` | 缺省管径（每根管可在输入里单独给出） |
| `c_rho` | 弯曲半径 ρ = c_rho × D（`straight_lengths` 打开时使用） |
| `eps_z_mm` | 高度容差（校验用） |
| `pitch_mm` / `margin_mm` | 布管网格基础间距 / 布管区域在场景外接矩形外的扩展 |
| `port_side_lines` | 是否在端口坐标两侧加密网格线（大场景可关，控制网格规模） |
| `max_iters` / `stall_iters` | 协商最多轮数 / 连续多少轮无改进就转入清理 |
| `pres_fac_init` / `pres_fac_mult` / `hist_fac` | 协商的拥堵与历史代价系数 |
| `max_expansions` / `astar_weight` | 单次 A* 扩展上限 / 启发式放大系数（1 = 单管最优） |
| `cleanup_trigger_nets` / `cleanup_max_expansions` | 中途试清理的冲突管网上限 / 清理时的扩展上限 |
| `freeze_after_exhausted` | 连续几次搜到上限后，协商中不再重搜该管网 |
| `route_workers` | 并行布管进程数 |

`params.blocking`（全部必填）：`auto_module_policy`（`accept` / `report_only`）、`min_copies`、`min_members`、`module_rotations`、`module_aspect_bands`、`module_solve_s`、`cluster_max_units`、`cluster_resolution`、`seed`。

## 7. 三维场景接口（接入已有项目）

已有项目自己管理设备实例、姿态和管路时，不必写任务书，直接用场景接口：

```bash
layout-kernel-scene < 请求.json > 结果.json        # 或 python -m layout_kernel.scene_cli
```

**请求** `{"input": 场景, "settings": 设置}`

- 场景（米制、Y 向上）：
  - `nodes`：每个节点 `{id, move, box, orientations: [{angle, swap, ports: {key: {position, normal}}, box, spools}]}`。`orientations[0]` 是当前姿态，其余是允许换成的朝向（设备旋转、三通换向与换口），每个朝向的端口已按该朝向算好；`move` 表示可以平移。
  - `routes`：每根管 `{id, code, points, segments: [{r}], from: {key}, to: {key}, leadA, leadB, fixed, low}`，可选 `weight_length / weight_bends / weight_height_changes`（本管权重倍数）。
  - `radius`（每轴移动范围，米）、`seconds`（时间上限）。
  - 可选区域：`equipment_keepout`、`pipe_keepout`（三维盒 `[x0,y0,z0,x1,y1,z1]`，米；是否生效由同名约束决定）。
- 设置：`routing`（求解参数 + `constraints`）、`weights`、`scale`、`oblique_stub_mm`、`pipe_rules`、
  `rotation_candidates`（每个可转节点每轮真实重布几种朝向）、`lower_bound`（`{enabled, max_expansions}`）、
  `candidate_routing`（评估候选时覆盖 `routing` 的轻量参数，如 `{max_iters, stall_iters, max_expansions, cleanup_max_expansions}`；最终联合重布仍用 `routing`）。

**结果** `{ok, violations, metrics, routes, offsets, orientations, moves, cost, base_cost, lower_bound, timing}`

- `offsets`：被移动节点的平移（米，Y 向上）；`orientations`：换了朝向的节点 → 输入 `orientations` 里的下标（调用方据此施加旋转 / 换口）。
- `lower_bound`：`{valid, value, cost, gap, nets, unresolved, note}`。设备位置与朝向固定为最终方案时，各两端点管单独求精确最短路（不考虑其他待布管、放宽同管自身净距）之和；`gap = (cost − value) / cost` 是当前方案离这一下界的最大相对差距。它**不是**设备也能移动时的全局下界；多端点管网不给下界。
- `pipe_rules` 用来把调用方自己的直管 / 弯头规则换算成内核的直管长度要求（见 `scene._straight_rules`），只在 `straight_lengths` 打开时生效。

**优化过程**：先按当前姿态重布；每一轮生成全部候选（串联拉直、单个对齐、换朝向），用 `routing.route_workers` 个进程并行评估（每个候选只重布相连的管、其余固定，用 `candidate_routing` 参数）；有改进的候选按改进量排序，挑出互不相干（移动对象与相关管都不重叠）的一批，合起来复核仍有改进就一次接受（否则只接受最好的一个）；没有改进或到时后，按最终姿态用完整参数联合重布一次取更好者。换朝向的候选先按“端口间曼哈顿距离 + 弯头下界”估计排序，只真实重布前 `rotation_candidates` 个。

## 8. 还没有的能力

| 需求 | 现状 |
|---|---|
| 设备之间的相对关系约束（必须相邻、必须靠墙） | 未实现 |
| 设备也能移动时的全局下界 / 最优性证明 | 未实现；当前下界只针对最终设备位置 |
| 多端点管网的下界 | 未实现（树为贪心构造） |
| 任务书摆放中的设备禁区 | 已实现，但方式是“碰到禁区的候选判为不可行”，禁区靠近布局原点时搜索效率低 |
| 很大的单管权重倍数 | 可用，但倍数过大（实测弯头 ×50）时，A* 对弯头数的估计相对偏弱，单管搜索可能撞上 `max_expansions` 而布不通；实测 ×5 正常。需要时同时调大 `max_expansions` |
