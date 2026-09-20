# User guide

**English** · [简体中文](guide.md)

This guide covers every public interface of layout-kernel: how to call it, what to send, what comes back, and how errors are reported. Runnable examples are included.

- Field-level details of the task file, the objective and the constraint registry: [contract.md](contract.md) (in Chinese).
- The mathematical model and solution method: [model.en.md](model.en.md).

**Contents**

1. [Which entry point](#1-which-entry-point)
2. [Install](#2-install)
3. [Conventions](#3-conventions)
4. [Entry A: 3D scene interface (integrate with an existing project)](#4-entry-a-3d-scene-interface)
5. [Entry B: task file (placement + routing)](#5-entry-b-task-file)
6. [Entry C: Python modules](#6-entry-c-python-modules)
7. [Constraint configuration](#7-constraint-configuration)
8. [Tuning](#8-tuning)
9. [Errors and troubleshooting](#9-errors-and-troubleshooting)

---

## 1. Which entry point

| Your situation | Entry | How |
|---|---|---|
| An existing app owns the equipment, poses and routes, and you want the kernel to re-route pipes and move equipment | **A scene** | CLI `layout-kernel-scene` (any language, via a subprocess) or `scene.optimize_scene()` |
| You have only an equipment list and connections, and the kernel should decide positions and routes | **B task file** | `solve()` or CLI `layout-kernel` |
| You want the router, validator or lower bound inside your own algorithm | **C modules** | functions in `routing` / `constraints` / `scene` |

All three entries share the same router, constraint registry and independent validator.

## 2. Install

Python 3.11+. Use a dedicated virtual environment, because `ortools` upgrades `protobuf`:

```bash
python -m venv .venv
.venv/Scripts/python -m pip install "layout-kernel @ git+https://github.com/66-zhimeng/layout-kernel"
```

This installs two commands: `layout-kernel` (task file) and `layout-kernel-scene` (scene).

**Callers don't need the kernel's dependencies.** Another project can keep the path to the kernel venv's Python and run `python -m layout_kernel.scene_cli` as a subprocess, so its own environment needs no numpy or numba (see §4.6).

For development:

```bash
git clone https://github.com/66-zhimeng/layout-kernel && cd layout-kernel
python -m venv .venv && .venv/Scripts/python -m pip install -e ".[test,plot]"
.venv/Scripts/python -m pytest -q
```

## 3. Conventions

1. **No defaults.** A missing required setting is an error that names the missing key. This is deliberate: every number stands for an engineering rule, and a silent default would change results.
2. **Every constraint is switched by the caller.** Each of the 13 constraints must state `"enabled": true/false`. An enabled constraint needs all its parameters, and an unknown name is an error (§7).
3. **The validator is the source of truth.** `ok`, `violations` and all metrics come from the geometry-only validator `routing.check_routes`, not from solver internals.
4. **Failure is a normal return.** When routing fails or leaves violations, the result is `ok: false` with the violations spelled out. Only invalid input raises; on the command line it prints an error JSON and exits non-zero.
5. **Heuristic, with a lower bound.** Results are feasible and good, but not proven optimal. `lower_bound.gap` gives the maximum distance to a bound (§4.3).
6. **Units**: the scene interface uses meters with **Y up** (the usual 3D-engine convention). The task file and the modules use millimeters with **Z up**. The scene interface converts internally.
7. **Multiprocessing**: `route_workers > 1` and task-file placement use spawn-based processes. When calling from Python, put the entry code under `if __name__ == "__main__":`.

---

## 4. Entry A: 3D scene interface

### 4.1 Calling it

**CLI**: the request JSON goes to stdin, the result JSON comes out on stdout, and progress logs go to stderr.

```bash
layout-kernel-scene < request.json > result.json
# or: <kernel venv python> -m layout_kernel.scene_cli < request.json > result.json
```

| Case | stdout | exit code |
|---|---|---|
| Normal, including `ok: false` (no violation-free layout found) | result JSON (§4.3) | 0 |
| Invalid request or internal error | `{"ok": false, "error": "Type: message"}` | 1 |

**Python**:

```python
from layout_kernel import scene

if __name__ == "__main__":
    out = scene.optimize_scene(inp, settings, log=print)   # routing + move / rotate / swap + lower bound
    out = scene.route_scene(inp, settings)                  # re-route only; equipment stays put
```

- `route_scene(inp, settings, fixed_ids=(), log=None)`: pipes listed in `fixed_ids` keep their paths and act as obstacles. Its result has fewer fields than `optimize_scene`: no offsets, orientations or lower bound.

**Runnable example**: [examples/scene/request.json](../examples/scene/request.json), driven by [examples/scene/run_scene.py](../examples/scene/run_scene.py).

- **Scene**: a pump outlet and a chiller inlet are not on one line, and the in-line sensor between them sits badly, so both pipes need two bends each.
- **Result**: the kernel shifts the sensor by −0.3 m along z. Bends go from 4 to 2, the gap to the lower bound is 0%, and the run takes about 0.2 s.
- The example is also a test case (`tests/test_scene.py`), so it keeps working.

```bash
python examples/scene/run_scene.py
```

### 4.2 Request

```jsonc
{"input": { /* scene, tables below */ }, "settings": { /* §4.2.4 */ }}
```

#### 4.2.1 `input.nodes`: equipment, tees, valves, sensors, …

| Field | Type | Meaning |
|---|---|---|
| `id` | string | Unique node id |
| `move` | bool | Whether the node may translate (range: `input.radius`) |
| `move_axes` | `[bool, bool, bool]`, optional | Whether it may move along the scene's X, Y (up) and Z axes. Omitted means all three are free; use it to lock only one or two axes |
| `orientations` | array, at least 1 | **Index 0 is the current pose.** The others are poses it may switch to (rotation, tee re-orientation, swapping same-direction ports). The caller computes the ports for each pose |
| `orientations[].ports` | `{port key: {position, normal}}` | Port keys are unique across the scene. `position` is `[x, y, z]` in meters. `normal` is the unit vector a pipe must leave along; an oblique (non-axis) normal marks an oblique branch port (see `oblique_stub_mm`) |
| `orientations[].box` | `{min, max}` or `null` | Bounding box in meters. `null` means a node without a solid box (such as a tee): it does not block pipes, and its solid parts are given as `spools` |
| `orientations[].spools` | `[[p, q], ...]` | Internal short pipes (such as tee internals), in meters. They block pipes that are not attached to this node |
| `orientations[].angle`, `swap` | any | Not read by the kernel; left for the caller to identify each pose |

A node with a single pose and `move: false` is never changed.

#### 4.2.2 `input.routes`: two-terminal pipes

| Field | Type | Meaning |
|---|---|---|
| `id` | string | Pipe id |
| `code` | string | Display code, echoed back in the result |
| `from.key` / `to.key` | string | Port keys at both ends |
| `points` | `[[x,y,z], ...]` | Current polyline in meters, **including both port ends**. It is the baseline; for a fixed pipe, it is the obstacle |
| `segments` | `[{r}, ...]` | Outer radii in meters; the largest one sets the pipe's diameter |
| `leadA` / `leadB` | number (m) | Straight neck plus reducer length at each end. Together with `pipe_rules`, it sets the straight run needed before the first bend |
| `fixed` | bool | `true` means the pipe is not changed (locked, or outside the local scope). It is a hard obstacle and still counts in clearance checks |
| `low` | bool | A low-level pipe, subject to `low_pipes` |
| `weight_length` / `weight_bends` / `weight_height_changes` | number, optional | Per-pipe weight multipliers (default 1). For example, `weight_bends: 3` makes bends on a main header more expensive |

#### 4.2.3 Other `input` fields

| Field | Meaning |
|---|---|
| `radius` | Translation range per axis in meters, relative to the original position. 0 means no translation |
| `seconds` | Time limit in seconds (§4.5) |
| `equipment_keepout` | Optional. Equipment keep-out boxes `[[x0,y0,z0,x1,y1,z1], ...]` in meters; active when the `equipment_keepout` constraint is enabled |
| `pipe_keepout` | Optional. Pipe keep-out boxes, same format; active when `pipe_keepout` is enabled |

#### 4.2.4 `settings` (all required)

| Key | Meaning |
|---|---|
| `routing` | Solver parameters in millimeters, plus `constraints`. See [contract.md §6](contract.md) for each parameter and §7 below for constraints |
| `weights` | `{area, length, bends, height_changes}`: objective weights |
| `scale` | `{A0, L0, B0, C0, kappa, l_min_mm}`: normalization scales (area in mm², length in mm, bend count, elevation-change count), maximum aspect ratio, and minimum straight run |
| `oblique_stub_mm` | Length of the oblique stub pushed out along the normal when the baseline has none |
| `pipe_rules` | The caller's straight and bend rules, which the kernel converts into straight-run requirements: `{trim_ratio, radius_margin_mm, port_margin_mm, safety, rule_D_mm, rule_R_mm}`; see [model §3.2](model.en.md#32-route-geometry) |
| `rotation_candidates` | How many orientations per rotatable node are actually re-routed each round (after ranking by estimate) |
| `lower_bound` | `{enabled, max_expansions}`: whether to report the lower bound and gap |
| `candidate_routing` | Lighter solver parameters used when evaluating candidates, such as `{max_iters, stall_iters, max_expansions, cleanup_max_expansions}`. It **must include `window_mm`**: each candidate is routed only inside a window around the affected pipes and moved nodes, expanded by this margin |
| `pose_negotiation` | `{enabled, max_poses, pres_bends, hist_bends}` (§4.4). The other three keys are required even when `enabled` is false |

For a complete working `settings` block, copy the one in [examples/scene/request.json](../examples/scene/request.json) and adjust it.

### 4.3 Result

```jsonc
{
  "ok": true,                         // everything routed and the independent validator found no violations
  "violations": [],
  "routes": [{"id": "pipe_1", "code": "供水 1", "points": [[1.2,0.5,0.4], [3.0,0.5,0.4]]}],
  "offsets": {"sensor": [0, 0, -0.3]},        // translated node → offset (m, Y up, relative to the original position)
  "orientations": {},                          // re-oriented node → index into its input orientations
  "moves": [{"move": "…", "gain": 6.0}],       // accepted improvements
  "base_cost": 17.0, "cost": 11.0,             // baseline and final cost (kernel objective)
  "metrics": {"W_m": 8.0, "H_m": 2.5, "area_m2": 20.0, "L_m": 5.0, "bends": 2, "height_changes": 0, "J": 11.0},
  "lower_bound": {"valid": true, "value": 11.0, "cost": 11.0, "gap": 0.0, "nets": 2, "unresolved": {}, "seconds": 0.0, "note": "…"},
  "candidates_tried": 5, "iterations": 1, "grid_nodes": null,
  "timing": {"route_s": 0.2}
}
```

**Applying the result on the caller's side:**

1. For each node in `orientations`, switch to the pose at that index. The caller knows what its `angle` and `swap` mean.
2. For each node in `offsets`, translate the whole node.
3. Replace pipe paths with `routes[].points`. **The points include both port ends**, and oblique stubs (port to stub end) are part of the path.
4. Pipes that are not in the result keep their paths.
5. **Re-validate with your own rules.** The kernel's rules are aligned with the caller's as closely as possible, but the caller's geometry, such as swept elbow bodies, has the final say.

**What `lower_bound` means**: fix the equipment at its final positions and orientations, and find the exact shortest route of each two-terminal pipe on its own (ignoring the other pipes being routed, and relaxing each pipe's clearance to itself). The sum is `value`. `gap = (cost − value) / cost` bounds how far the result can be from that value.

- It is **not** a global bound over equipment that can still move.
- Multi-terminal nets get no bound.
- Pipes whose single-pipe search didn't finish are listed in `unresolved`, and then `valid` is false.

### 4.4 Optimization pipeline

1. **Baseline**: check the current paths with the kernel's validator. If they pass, use them directly. If only a few pipes fail, re-route just those; if many fail, re-route everything.
2. **Pose negotiation** (optional, `pose_negotiation.enabled`): candidate poses of movable nodes join negotiated routing, and all poses are chosen in one pass (see [model §5.11](model.en.md#511-pose-negotiation)).
3. **Candidate rounds**:
   - Candidates come in four kinds: line-straightening (a chain of connected in-line devices shifts onto one line), single alignment, chain compaction (every movable device on one side of a cut plane is translated towards the other side, shortening straight runs), and re-orientation.
   - They are evaluated in parallel on `route_workers` processes, each re-routing only the affected pipes inside a local window.
   - Among the improving candidates, a batch of mutually independent ones is re-checked and accepted at once.
4. **Finish**: when no candidate improves, or time runs out: if time remains, do one full joint re-route at the final poses and keep the better result; otherwise check the step-by-step result once on the full scene, and re-route if that check fails.
5. **Lower bound** at the final poses.

### 4.5 Time limit

`seconds` limits step 3. When time is up, the kernel stops collecting candidate results, picks among those already evaluated, and skips the full re-route in step 4. These parts are not counted against the limit:

- step 1, the baseline (usually a few seconds);
- step 2, pose negotiation (tens of seconds on large scenes; turn it off when time is tight);
- step 5, the lower bound (about 5 s on large scenes).

Reference (230 pipes, 12 processes):

| `seconds` | Actual time | Result |
|---|---|---|
| 60 | ~70 s | 1.9% from the lower bound |
| 120 | ~130 s | 1.2% from the lower bound; converged at ~110 s |

### 4.6 Calling from other languages

**Python subprocess** (the caller doesn't need the kernel's dependencies):

```python
import json, subprocess
KERNEL_PY = r"/path/to/layout-kernel/.venv/Scripts/python.exe"
done = subprocess.run([KERNEL_PY, "-m", "layout_kernel.scene_cli"],
                      input=json.dumps({"input": inp, "settings": settings}).encode("utf-8"),
                      capture_output=True, timeout=900)
out = json.loads(done.stdout.decode("utf-8"))
if done.returncode != 0:
    raise RuntimeError(out["error"])
```

**Node.js**:

```js
import {spawnSync} from 'node:child_process';
const p = spawnSync(KERNEL_PY, ['-m', 'layout_kernel.scene_cli'], {input: JSON.stringify({input, settings}), maxBuffer: 1 << 28});
const out = JSON.parse(p.stdout.toString('utf8'));
if (p.status !== 0) throw new Error(out.error);
```

**Browser**: a browser can't start a process. Run a small local service that accepts the request (for example `POST /api/layout`), forwards it to the kernel with the subprocess pattern above, and returns the result.

---

## 5. Entry B: task file

```python
from layout_kernel import solve, validate_case, CaseError

if __name__ == "__main__":
    try:
        result = solve("examples/case_small_plant.json", log=print)   # a dict works too
    except CaseError as ex:
        print("invalid task file:", ex)
```

```bash
layout-kernel examples/case_small_plant.json -o plan.json      # -q: no progress log
```

| Interface | Meaning |
|---|---|
| `solve(case, log=None, work_dir=None)` | `case` is a path or a dict; `log(line)` receives progress; `work_dir` holds intermediate files (a temp dir by default). Returns the plan dict |
| `validate_case(case)` | Checks the task file without solving; raises `CaseError` naming the problem |
| CLI exit codes | 0 = success, 1 = violations or unrouted nets, 2 = invalid task file |

- **Modes**: `task.mode = "place_and_route"` lets the kernel choose equipment positions; `"route_only"` requires every device to have `placed`, and only routes pipes.
- **Fields**: the full task-file and result fields are in [contract.md §2–3](contract.md).
- **Examples**: [examples/case_small_plant.json](../examples/case_small_plant.json); [examples/generate_cases.py](../examples/generate_cases.py) generates cases of other sizes.

---

## 6. Entry C: Python modules

All functions here use **millimeters, Z up**.

### 6.1 `layout_kernel.routing`: routing and validation

```python
from layout_kernel import routing as rt

sc = rt.Scene(devices, nets, routing_params, weights, scale,
              fixed_routes=None, spools=None, pipe_keepout=None)
routes, history, G = rt.negotiate(sc, log=print)
viol, metrics = rt.check_routes(sc, routes)
```

| Interface | Input | Returns |
|---|---|---|
| `Scene(devices, nets, rp, weights, scale, fixed_routes, spools, pipe_keepout)` | see below | Scene object. Raises `KeyError` for missing parameters and `ConstraintError` for a bad constraint config |
| `negotiate(sc, log=print, cleanup_enabled=True)` | scene | `(routes, history, G)`: `routes = {net id: [branch, ...]}`, per-round stats, and the grid |
| `check_routes(sc, routes)` | scene, routes | `(violations, metrics)`, with `metrics = {W_m, H_m, area_m2, L_m, bends, height_changes, J, per_net}` |
| `net_cost(sc, nid, per_net[nid])` | | One pipe's objective cost, including per-pipe weights |
| `lower_bounds(sc, max_expansions, workers)` | | `{net id: (bound or None, note)}` |
| `cleanup(G, sc, routes, log)` | | `(routes, records)`: clears clashes one by one, holding the other pipes as hard obstacles |
| `Grid(sc)`, `route_net(G, sc, net, ctx)`, `astar(...)` | | Low level; rarely needed |

**`devices`**: `{device id: {...}}` with these fields:

| Field | Meaning |
|---|---|
| `box` | `(x0,y0,z0,x1,y1,z1)`, or `None` for a node without a solid box |
| `zones` | `[(x0,y0,x1,y1), ...]`: maintenance-zone rectangles in plan |
| `ports` | `{port name: (x,y,z,ux,uy,uz)}`; the direction must be an axis-aligned unit vector |
| `fitting_ports` | Optional. Ports that already have a fitting outside them (straight runs are counted as from an elbow) |

**`nets`**: `[{...}, ...]` with these fields:

| Field | Meaning |
|---|---|
| `id`, `terms` | `terms = [(device, port), ...]`; with more than 2 terminals, tees are added automatically |
| `D_mm`, `rho_mm`, `lead_mm` | Optional. Outer diameter, bend radius, minimum straight run |
| `zc_max_mm` | Optional. Centerline height limit |
| `port_straight_mm` | Optional. `{port name: straight run needed before the first bend}` |
| `weight_length` / `weight_bends` / `weight_height_changes` | Optional. Per-pipe weight multipliers |

**`fixed_routes`**: `{id: {"points": [[x,y,z], ...], "D_mm", optional "trim_nodes": (tee id or None at the start, … at the end)}}`. These pipes are not routed; they are hard obstacles.

**`spools`**: `[{"owner": node id, "points": [...], "D_mm"}]`: internal short pipes of a node.

**Branch format**: `{"start": (device, port), "points": [(x,y,z), ...], "end": ("port", (device, port)) or ("tee", None)}`. Consecutive points differ along exactly one axis.

**Complete example**: [examples/minimal_routing.py](../examples/minimal_routing.py), with every parameter commented.

### 6.2 `layout_kernel.constraints`

| Interface | Meaning |
|---|---|
| `REGISTRY` | `{name: {"doc", "params"}}` |
| `validate(cfg)` | Checks a constraint config and returns it normalized. Missing, extra or mistyped entries raise `ConstraintError` |
| `describe()` | Description and parameters of each constraint, ready for generating a settings UI |

### 6.3 `layout_kernel.scene`

| Interface | Meaning |
|---|---|
| `optimize_scene(inp, settings, log=None)` | Entry A as a function |
| `route_scene(inp, settings, fixed_ids=(), log=None)` | Re-route only; equipment stays put |
| `build_scene(inp, settings, fixed_ids=(), deltas=None)` | Converts a scene input to `(routing.Scene, metadata)`, for use with the §6.1 functions |
| `to_k(p)` / `to_w(p)` | Convert between web coordinates (m, Y up) and kernel coordinates (mm, Z up) |

---

## 7. Constraint configuration

Constraints go under `routing.constraints`, and all 13 must be present:

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

- The meaning, parameters and "off" behavior of each constraint are in the README table, [contract.md §5](contract.md), and `constraints.describe()`.
- Rules: a disabled constraint needs only `enabled`; an enabled one needs all its parameters; `self_clearance` requires `pipe_pipe_clearance`.

## 8. Tuning

| Goal | Adjust |
|---|---|
| Faster | Lower `seconds`; turn off `pose_negotiation`; lower `candidate_routing.max_iters` / `max_expansions`; raise `routing.pitch_mm` (coarser grid); turn off `port_side_lines` |
| Better | Raise `seconds` (120–300 s usually converges); raise `radius`; raise `rotation_candidates` a little |
| Let candidates find longer detours | Raise `candidate_routing.window_mm` (larger windows are slower) |
| Fewer bends, even at the cost of more pipe | Raise `weights.bends`, or set `weight_bends` on selected headers |
| Large scene fails to route or is slow | Raise `max_expansions`; set `astar_weight` to 1.2–1.5 (faster, but a single pipe is no longer guaranteed optimal); set `route_workers` to your core count |

`weights` and `scale` together set how much pipe one bend is worth. For example, `bends: 3, B0: 1, length: 1, L0: 1000` makes one bend worth 3 m of pipe.

## 9. Errors and troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| `KeyError: 布管缺少必填参数…` / `场景布管缺少设置…` | A required setting is missing (no defaults) | Add the key named in the message |
| `ConstraintError: 约束配置缺少…` / `未知的约束…` | The constraint table is incomplete or has a typo | List all 13 constraints, spelled as in §7 |
| `端口正前方不足 ℓ_min` in `violations` | Something blocks the space in front of a port | Check the port normal and the nearby equipment or fixed pipes |
| `搜索未找到路径（或超过扩展上限）` | Either no path exists or the search exceeded `max_expansions` | Raise `max_expansions`; the cleanup records tell "truly no path" apart from "hit the limit" |
| `管–管净距不足` remains | Negotiation and cleanup couldn't clear it | Raise `max_iters`; allow a larger `radius` so equipment can move; check for blocking fixed pipes |
| Log shows `到达时间上限：本轮只评估了 x/y 个候选` | Not enough time | Raise `seconds` |
| `端口超出布管范围` | A candidate moved a port off the grid (for example, above the ceiling) | Expected; that candidate counts as infeasible |
| Multiprocessing errors or hangs on Windows | The calling code is not under `if __name__ == "__main__":` | Move it there; the CLIs are not affected |
| The caller's validation fails while the kernel says `ok: true` | The two sets of rules differ | The caller's rules win. Save the request and result, compare the failing pipe's geometry, and adjust `pipe_rules` or the clearances if needed |

Messages are currently in Chinese; the table above maps the main ones.
