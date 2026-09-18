# layout-kernel

设备与管道自动排布的计算内核：设备摆放、三维布管、多管协商消冲突、设备平移优化，以及只看几何的独立校验器。

- **约束全部由调用方配置。** 每条约束都有开关和参数（见 [docs/contract.md 第 5 节](docs/contract.md)），内核不写死约束，也不设默认值。
- **结果以校验器为准。** 返回的违规和指标都来自独立校验器，不依赖求解器内部数据。
- **启发式求解。** 给出可行且较优的方案，不附带最优性证明。

研究过程（数学模型推导、各轮实验与失败记录）在 [math-problem-discussions](https://github.com/66-zhimeng/math-problem-discussions)，本仓库保留了其中内核部分的完整提交历史。

---

## 安装

需要 Python 3.11+。建议装在独立虚拟环境里（`ortools` 会升级 `protobuf`，可能与其他项目冲突）：

```bash
python -m venv .venv
.venv/Scripts/python -m pip install "layout-kernel @ git+https://github.com/66-zhimeng/layout-kernel"
# 开发：git clone 后 pip install -e ".[test,plot]"
```

依赖：numpy、scipy、numba（单管 A* 的编译实现）、ortools（CP-SAT）、networkx。

## 三种用法

### 1. 任务书：摆放 + 布管，或只布管

```python
from layout_kernel import solve

if __name__ == "__main__":                       # 摆放用多进程，必须放在这里
    result = solve("examples/case_small_plant.json")
    print(result["ok"], result["metrics"])
```

```bash
layout-kernel examples/case_small_plant.json -o 方案.json
```

任务书给设备库、设备清单、管网拓扑和全部参数。`task.mode` 选 `place_and_route`（求设备位置 + 管道；多个候选摆放各布一次管，按真实指标选）或 `route_only`（设备位置已给定）。字段见 [docs/contract.md](docs/contract.md)。

### 2. 三维场景：接入已有项目

已有项目自己管理设备实例、姿态、管路时，把当前场景交给内核，拿回管路折点和设备平移：

```bash
layout-kernel-scene < 请求.json > 结果.json
```

实际接入例子：拆件做网页（Three.js 系统管网页）的“布局内核”引擎，由其本机服务以子进程调用本命令；内核结果还要再过那个项目自己的实体校验才会被应用。

### 3. 直接调用布管器

```python
from layout_kernel import routing as rt

sc = rt.Scene(DEVICES, NETS, ROUTING, WEIGHTS, SCALE)   # ROUTING 含 constraints
routes, history, G = rt.negotiate(sc)
viol, metrics = rt.check_routes(sc, routes)
```

完整可运行示例见 [examples/minimal_routing.py](examples/minimal_routing.py)，每个参数都有注释。

## 约束开关

| 约束 | 含义 |
|---|---|
| `pipe_pipe_clearance` | 不同管外壁之间的净距 |
| `pipe_equipment_clearance` | 管与设备包围盒的净距 |
| `self_clearance` | 同一根管沿管长相隔较远的两段之间的净距（防回绕、自交） |
| `height_change_limit` | 每条支路高度变化次数上限 |
| `ceiling` | 管顶最高标高 |
| `service_zones` | 检修区下方不得走管 |
| `straight_lengths` | 管件之间的最短直管 |
| `low_pipes` | 低位管的中心线高度上限 |
| `junction_merge_exemption` | 汇合于同一三通的管在口附近不算冲突 |
| `internal_spools` | 三通内部短管是其他管的障碍 |
| `equipment_spacing` | 移动设备时的设备间距 |

每一条都要在配置里显式写 `"enabled": true/false`；参数与关闭时的含义见契约文档。

## 模块

| 模块 | 作用 |
|---|---|
| `constraints.py` | 约束注册表与配置校验 |
| `routing.py` | 网格、A*、三通、协商布线、清理、独立校验器 |
| `astar_fast.py` | 单管 A* 的 numba 实现（与 `routing.astar_py` 逐例一致，有测试保证） |
| `scene.py` / `scene_cli.py` | 三维场景接口：每管管径与直颈、斜支口、固定管路、设备平移优化 |
| `placement_sp.py` / `placement_cpsat.py` | 块级摆放（序列对 + LP + 并行退火 / CP-SAT）与摆放校验 |
| `blocking.py` | 设备 → 块：模块识别、模块内部排法、聚簇 |
| `api.py` / `contract.py` / `build.py` | 任务书接口、契约校验、模块间的粘合 |
| `coarse_route.py` | 粗网格快速布管（秒级判断能否布下） |

## 测试

```bash
.venv/Scripts/python -m pytest -q        # 51 个
```

## 已知限制

- 管道只走网格线（间距可配置）；管径不同时，占用计算按最大管径保守处理。
- 设备优化只做平移，不改朝向；摆放只在平面内。
- 启发式求解，不给最优性证明或下界。
- 示例算例为自拟数据；真实项目的接入例子见上文。

## 许可

Apache-2.0，见 [LICENSE](LICENSE)。
