<div align="center">

# layout-kernel

**A computational kernel for automatic equipment and piping layout**

Equipment placement · 3D orthogonal pipe routing · negotiated congestion resolution · equipment move / rotate / tee-port-swap optimization · lower bounds & gaps · independent validator

**English** · [简体中文](README.zh-CN.md)

[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.11%2B-3776AB.svg?logo=python&logoColor=white)](pyproject.toml)
[![Tests](https://img.shields.io/badge/tests-66%20passed-brightgreen.svg)](tests)
[![Docs](https://img.shields.io/badge/docs-user%20guide-orange.svg)](docs/guide.en.md)
[![Numba](https://img.shields.io/badge/A*-numba%20compiled-00A3E0.svg)](src/layout_kernel/astar_fast.py)

</div>

---

## Documentation

| Document | Contents |
|---|---|
| 📘 [**User guide**](docs/guide.en.md) ([简体中文](docs/guide.md)) | Every public interface: the three entry points, each request and result field, errors and exit codes, calling from Python / Node / a browser, applying results, tuning, troubleshooting, and a case study |
| 📐 [Contract](docs/contract.md) (Chinese) | Field-level definition of the task file and result, the objective, the constraint registry, solver parameters |
| 🧮 [Model and method](docs/model.en.md) ([简体中文](docs/model.md)) | Problem definition, constraints, objective, and the method and proof boundaries of placement, routing, negotiation and bounds |
| ▶️ [Scene example](examples/scene/) | A runnable `request.json` plus `run_scene.py` (moving a sensor cuts bends from 4 to 2) |
| ▶️ [Routing example](examples/minimal_routing.py) | Calls the router and validator directly, with every parameter commented |
| ▶️ [Task-file example](examples/case_small_plant.json) | A complete place-and-route task file |

## Why

| | |
|---|---|
| 🎛️ **Every constraint is caller-configured** | 13 constraints, each with an explicit on/off switch and parameters. Nothing is hard-coded and there are no defaults: a missing setting is an error. |
| ✅ **The validator is the source of truth** | Reported violations and metrics come from a geometry-only validator that is independent of the solver's internal state. |
| 📉 **Heuristic, with a lower bound** | Returns a feasible, good layout plus the maximum gap to a lower bound at the final equipment positions. This is not an optimality proof, and the docs say so. |
| 🔌 **Plugs into existing projects** | The scene interface takes the host project's equipment instances, ports, poses and current routes, and returns route polylines, equipment offsets and orientations. |

## Mathematical model and method

The full derivation and every step in detail are in [docs/model.en.md](docs/model.en.md). The essentials:

### Problem

Given equipment with volume and ports, and the connections between them, the kernel decides two things: where each piece of equipment goes and how it is oriented, and an orthogonal route for each connection (only ±X, ±Y, ±Z, only 90° elbows, with tees for multi-terminal connections).

**Decision variables**:

| Variable | Task-file form | Scene form |
|---|---|---|
| Position | A plan grid point (x_i, y_i) | t_i = t_i⁰ + Δ_i, \|Δ_i\| ≤ r on each axis |
| Orientation | 0° / 90° / 180° / 270° about the vertical axis | One of the caller's candidate poses (rotation, tee re-orientation, same-direction port swap) |
| Route Γ_e | An orthogonal polyline between ports; an orthogonal Steiner tree for multi-terminal nets | same |

**Objective** (computed from actual geometry by the validator):

$$
J = w_A\frac{A}{A_0} + w_L\frac{L}{L_0} + w_B\frac{N_{\text{bend}}}{B_0} + w_C\frac{N_{\text{layer}}}{C_0}
$$

- A: XY bounding-rectangle area of all equipment and pipes, padded to aspect ratio κ;
- L: total centerline length;
- N_bend: number of 90° elbows;
- N_layer: number of elevation changes, counted when two consecutive horizontal segments differ in height by more than ε_z.

Each pipe can have its own weight multipliers.

**Constraints** (13, each switchable):

- orthogonal routing; ports entered and left along their normals;
- minimum straight runs: elbow–elbow ≥ 2ρ + ℓ_min, port–elbow ≥ ρ + ℓ_min, with ρ = c_ρ·D;
- clearances: pipe–pipe ≥ δ_pp, pipe–equipment ≥ δ_ep, distant segments of the same pipe ≥ δ_pp;
- elevation changes ≤ K, ceiling, low-pipe height, maintenance zones, keep-outs, equipment spacing;
- tee internals block other pipes. Pipes meeting at the same tee are exempt from each other, and only from each other, near its ports.

### Method

| Step | Method | Status |
|---|---|---|
| Blocking | Detect rigid modules (parallel arrays, chains); CP-SAT for internal module layouts; cluster by connectivity | Heuristic |
| Placement | **Sequence pair** (Γ⁺, Γ⁻) for relative positions → longest path for the minimal bounding size → **LP (HiGHS)** for coordinates, minimizing the area's tangent plane plus pipe lower bounds → parallel **simulated annealing**. An exact **CP-SAT** model is also available | Heuristic; pipe bounds proven |
| Pipe bounds during placement | Two terminals: length ≥ max(\|p−q\|₁, 2ℓ_min + \|s_p−s_q\|₁), bends ≥ 0/1/2 by direction; multi-terminal: k·ℓ_min + the sum of stub-point spans | Proven lower bounds |
| Routing grid | Non-uniform 3D track grid: base pitch ∪ port coordinates ∪ inflated obstacle boundaries | — |
| Single pipe | **A\*** over (node, direction, last fitting type, elevation changes, has-horizontal, straight run). Admissible heuristic c_L·Manhattan + c_B·minimum bends still needed; dominance pruning; numba-compiled; multi-start / multi-goal | Exact single-pipe optimum on the grid (unweighted, self clearance relaxed) |
| Multi-terminal | Connect the nearest pair, then attach each remaining terminal to the tree, nearest first: inside a straight segment, or on an elbow leg's extension (the elbow becomes a tee) | Heuristic |
| Multiple pipes | **PathFinder negotiation**: edge cost c_L·len·(1+h_e)·(1+π·n_e); history h_e accumulates on conflicted edges and pressure π grows each round; optimistic parallel search over a shared occupancy map | Heuristic |
| Cleanup | For each clash X–Y try: X only / Y only / X then Y / Y then X, with all other pipes as hard obstacles | Heuristic; tells "no path" apart from "hit the limit" |
| Moving equipment | Start from the current layout as the baseline. Candidates are line-straightening, single alignment and pose changes. Each one re-routes the affected pipes inside a **local window**, evaluated in parallel; a batch of independent improvements is accepted at once | Heuristic |
| Pose negotiation (optional) | Candidate poses become virtual nodes, and multi-start / multi-goal A\* picks both end poses at once. Disagreement at a node is priced like congestion, round after round, until all pipes agree | Heuristic |
| Lower bound and gap | At the final poses, LB = sum of each two-terminal pipe's exact shortest route on its own; gap = (cost − LB)/cost | A valid lower bound at the final poses |

The **independent validator** looks only at polyline geometry, checks every constraint and computes J. Every `ok`, violation and metric in a result comes from it.

```mermaid
flowchart LR
    A[Scene / task file] --> B[Baseline or placement]
    B --> C{Pose negotiation<br/>optional}
    C --> D[Candidate moves<br/>local windows, parallel]
    D --> E[Joint re-route<br/>full parameters]
    E --> F[Independent validator]
    F --> G[Result + lower bound / gap]
```

## Install

Python 3.11+. Use a dedicated virtual environment (`ortools` upgrades `protobuf`, which can clash with other projects):

```bash
python -m venv .venv
.venv/Scripts/python -m pip install "layout-kernel @ git+https://github.com/66-zhimeng/layout-kernel"
# development: git clone, then pip install -e ".[test,plot]"
```

Dependencies: numpy, scipy, numba, ortools (CP-SAT), networkx.

## Three ways to use it

<details open>
<summary><b>1. Task file: place + route, or route only</b></summary>

```python
from layout_kernel import solve

if __name__ == "__main__":                       # placement uses multiprocessing
    result = solve("examples/case_small_plant.json")
    print(result["ok"], result["metrics"])
```

```bash
layout-kernel examples/case_small_plant.json -o plan.json
```

Set `task.mode` to `place_and_route` (find equipment positions and pipes) or `route_only` (positions are given). The fields are described in [docs/contract.md](docs/contract.md) (in Chinese).
</details>

<details>
<summary><b>2. 3D scene: integrate with an existing project</b></summary>

```bash
layout-kernel-scene < examples/scene/request.json > result.json
python examples/scene/run_scene.py            # the same example from Python
```

The request holds nodes (bounding boxes, ports for each candidate pose), routes (ends, current bends, outer diameter, straight necks) and settings (constraints, weights, time limit, and so on). The result holds new route polylines, equipment `offsets`, re-oriented nodes in `orientations`, and the lower bound and gap. Every field, and how to call the kernel from other languages, is in [user guide §4](docs/guide.en.md#4-entry-a-3d-scene-interface).
</details>

<details>
<summary><b>3. Call the router directly</b></summary>

```python
from layout_kernel import routing as rt

sc = rt.Scene(DEVICES, NETS, ROUTING, WEIGHTS, SCALE)   # ROUTING includes "constraints"
routes, history, G = rt.negotiate(sc)
viol, metrics = rt.check_routes(sc, routes)
```

A runnable, fully commented example is in [examples/minimal_routing.py](examples/minimal_routing.py).
</details>

## Constraint switches

| Constraint | Meaning |
|---|---|
| `pipe_pipe_clearance` | Clearance between the outer walls of different pipes |
| `pipe_equipment_clearance` | Clearance between pipes and equipment boxes |
| `self_clearance` | Clearance between distant segments of the same pipe |
| `height_change_limit` | Maximum number of elevation changes per branch |
| `ceiling` | Maximum top-of-pipe elevation |
| `service_zones` | No pipes below maintenance zones |
| `straight_lengths` | Minimum straight runs between fittings |
| `low_pipes` | Centerline height limit for low-level pipes |
| `junction_merge_exemption` | Two pipes meeting at the same tee don't clash near its ports (applies only to that pair) |
| `internal_spools` | Tee internals and oblique stubs block other pipes |
| `equipment_spacing` | Equipment spacing when moving equipment |
| `equipment_keepout` | Equipment keep-out zones |
| `pipe_keepout` | Pipe keep-out zones |

Every constraint must state `"enabled": true/false`. Parameters and the meaning of "off" are in the contract doc.

## On a large instance

A real plant-room piping network: 230 pipes, 183 nodes (equipment, tees, valves, sensors), 12 processes:

| Scenario | Result | Time |
|---|---|---|
| Global, `seconds` = 120 | bends 454 → 330, length 2173 → 2089 m, 1.2% from the lower bound | ≈ 130 s |
| Global, `seconds` = 60 | bends 454 → 380, 1.9% from the lower bound | ≈ 70 s |
| Local (3 sensors) | bends 464 → 458, gap 0% | ≈ 7 s |

> The gap is computed at the final equipment positions and orientations: each pipe's shortest route on its own, summed. It is not a global bound over equipment that can still move. Parallel negotiation varies slightly from run to run.

## Modules

| Module | Role |
|---|---|
| `constraints.py` | Constraint registry and config validation |
| `routing.py` | Grid, A\*, tees, negotiated routing, pose negotiation, cleanup, independent validator |
| `astar_fast.py` | numba A\* for a single pipe (multi-start / multi-goal) |
| `scene.py` / `scene_cli.py` | 3D scene interface: per-pipe diameter and necks, oblique stubs, fixed routes, move / rotate / swap optimization, lower bound |
| `placement_sp.py` / `placement_cpsat.py` | Block placement (sequence pair + LP + parallel annealing / CP-SAT) and placement validation |
| `blocking.py` | Equipment → blocks: module detection, intra-module arrangement, clustering |
| `api.py` / `contract.py` / `build.py` | Task-file API, contract validation, glue |
| `coarse_route.py` | Coarse-grid routing (feasibility in seconds) |

## Tests

```bash
.venv/Scripts/python -m pytest -q        # 66 tests
```

## Limitations

- Pipes follow grid lines (the spacing is configurable). With mixed diameters, occupancy conservatively uses the largest diameter, and pipe sections are treated as squares.
- Placement is planar. In scene optimization equipment only translates, and orientations are picked from the candidates the caller supplies.
- Heuristic. The lower bound holds only for the final equipment positions, and none is given for multi-terminal nets.

## License

[Apache-2.0](LICENSE)
