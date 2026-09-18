"""内核接口测试。运行：.venv/Scripts/python -m pytest -q（在 01_设备与管道自动排布 目录下）"""
import copy
import json

import pytest

from layout_kernel import CaseError, solve, validate_case

TYPES = {
    "泵": {"height": 1000, "size": [1200, 800], "rotations": [0, 90, 180, 270],
           "service_zones": [[0, -800, 1200, 0]],
           "ports": {"in": {"pos": [0, 400], "dir": [-1, 0], "z": 500},
                     "out": {"pos": [1200, 400], "dir": [1, 0], "z": 500}}},
    "机组": {"height": 2200, "size": [2000, 2500], "rotations": [0, 90, 180, 270], "service_zones": [],
             "ports": {"in": {"pos": [0, 1200], "dir": [-1, 0], "z": 800},
                       "out": {"pos": [2000, 1200], "dir": [1, 0], "z": 800}}},
    "水箱": {"height": 1500, "size": [1500, 1500], "rotations": [0, 90, 180, 270], "service_zones": [],
             "ports": {"out": {"pos": [700, 0], "dir": [0, -1], "z": 700}}},
}
CONSTRAINTS = {"pipe_pipe_clearance": {"enabled": True, "gap_mm": 100},
               "pipe_equipment_clearance": {"enabled": True, "gap_mm": 100},
               "self_clearance": {"enabled": True, "skip_along_mm": 0},
               "height_change_limit": {"enabled": True, "max_changes": 3},
               "ceiling": {"enabled": True, "z_max_mm": 4500},
               "service_zones": {"enabled": True, "height_mm": 2000},
               "straight_lengths": {"enabled": True},
               "low_pipes": {"enabled": False},
               "junction_merge_exemption": {"enabled": True},
               "internal_spools": {"enabled": True},
               "equipment_spacing": {"enabled": False},
               "equipment_keepout": {"enabled": True},
               "pipe_keepout": {"enabled": True}}
ROUTING = {"D_default_mm": 200, "c_rho": 1.5, "eps_z_mm": 1, "constraints": CONSTRAINTS, "pitch_mm": 300, "margin_mm": 1500,
           "max_iters": 20, "pres_fac_init": 0.5, "pres_fac_mult": 1.6, "hist_fac": 0.5,
           "max_expansions": 300000, "astar_weight": 1.5, "route_workers": 1, "stall_iters": 6,
           "cleanup_trigger_nets": 4, "cleanup_max_expansions": 300000, "freeze_after_exhausted": 2, "port_side_lines": True}
BLOCKING = {"auto_module_policy": "report_only", "min_copies": 2, "min_members": 2,
            "module_rotations": [0, 90, 180, 270], "module_aspect_bands": [[1, 1.5]],
            "module_solve_s": 1, "cluster_max_units": 25, "cluster_resolution": 1.0, "seed": 0}
CASE = {
    "params": {"grid_mm": 100, "delta_ee_mm": 800, "l_min_mm": 300, "kappa": 2,
               "service_zones_inside_footprint": False,
               "weights": {"area": 1.0, "length": 1.0, "bends": 0.3, "height_changes": 0.3},
               "routing": ROUTING, "blocking": BLOCKING},
    "device_types": TYPES,
    "devices": [{"id": "泵1", "type": "泵"}, {"id": "机组1", "type": "机组"}, {"id": "箱1", "type": "水箱"}],
    "nets": [{"id": "供水", "terminals": ["泵1.out", "机组1.in"]},
             {"id": "回水", "terminals": ["机组1.out", "箱1.out"]}],
    "task": {"mode": "place_and_route", "time_budget_s": 2, "candidates": 1, "seed": 0, "workers": 1},
}


def case(**task):
    c = copy.deepcopy(CASE)
    c["task"] = {**c["task"], **task}
    return c


def test_place_and_route_returns_valid_plan():
    r = solve(case(time_budget_s=2, candidates=1, workers=1))
    assert r["ok"] and r["violations"] == []
    assert set(r["routes"]) == {"供水", "回水"}
    assert set(r["devices"]) == {"泵1", "机组1", "箱1"}
    assert r["metrics"]["L_m"] > 0 and len(r["candidates"]) == 1


def test_route_only_reproduces_same_geometry():
    r = solve(case(time_budget_s=2, candidates=1, workers=1))
    c = copy.deepcopy(CASE)
    for d in c["devices"]:
        v = r["devices"][d["id"]]
        d["placed"] = [v["x_mm"], v["y_mm"], v["rot_deg"]]
    c["task"] = {"mode": "route_only"}
    r2 = solve(c)
    assert r2["ok"]
    for k, v in r["devices"].items():                         # 位置回灌后几何完全一致
        assert r2["devices"][k]["box_mm"] == v["box_mm"]
        assert r2["devices"][k]["ports_mm"] == v["ports_mm"]


def test_fixed_device_position_is_respected():
    c = case(time_budget_s=2, candidates=1, workers=1)
    c["devices"][0]["fixed"] = [2000, 2000]                   # 坐标要给四周预留的出管空间留出余地
    r = solve(c)
    assert (r["devices"]["泵1"]["x_mm"], r["devices"]["泵1"]["y_mm"]) == (2000, 2000)


def test_infeasible_fixed_position_explains_why():
    c = case(time_budget_s=2, candidates=1, workers=1)
    c["devices"][0]["fixed"] = [0, 0]                         # 与四周预留空间冲突
    r = solve(c)
    assert not r["ok"] and "fixed" in r["violations"][0] and r["devices"] == {}


def test_keepout_box_blocks_pipes():
    r = solve(case(time_budget_s=2, candidates=1, workers=1))
    c = copy.deepcopy(CASE)
    for d in c["devices"]:
        v = r["devices"][d["id"]]
        d["placed"] = [v["x_mm"], v["y_mm"], v["rot_deg"]]
    c["task"] = {"mode": "route_only"}
    pts = [p for brs in r["routes"].values() for b in brs for p in b["points_mm"]]
    x0 = min(p[0] for p in pts) - 5000
    c["keepout"] = [[x0, min(p[1] for p in pts) - 5000, 0, x0 + 1, max(p[1] for p in pts) + 5000, 4500]]
    r2 = solve(c)                                             # 禁区在管道范围之外：仍应布通
    assert r2["ok"]


@pytest.mark.parametrize("mutate, msg", [
    (lambda c: c["task"].pop("candidates"), "candidates"),
    (lambda c: c["task"].update(mode="乱填"), "mode"),
    (lambda c: c["devices"][0].update(fixed=[123, 0]), "网格"),
    (lambda c: c["devices"][0].update(placed=[0, 0, 45]), "旋转角"),
    (lambda c: c["nets"][0].update(terminals=["泵1.没有这个端口", "机组1.in"]), "端口"),
    (lambda c: c["params"]["routing"]["constraints"].pop("ceiling"), "ceiling"),
    (lambda c: c.update(keepout=[[0, 0, 0, 0, 1, 1]]), "下界"),
    (lambda c: c["device_types"]["泵"]["ports"]["in"].update(pos=[0, 450]), "整数倍"),
    (lambda c: c["device_types"]["泵"]["ports"]["in"].update(dir=[1, 1]), "单位向量"),
])
def test_contract_errors_point_at_the_problem(mutate, msg):
    c = copy.deepcopy(CASE)
    mutate(c)
    with pytest.raises((CaseError, KeyError)) as ex:
        validate_case(c)
    assert msg in str(ex.value)


def test_route_only_requires_all_positions():
    c = copy.deepcopy(CASE)
    c["task"] = {"mode": "route_only"}
    with pytest.raises(CaseError) as ex:
        validate_case(c)
    assert "placed" in str(ex.value)


def test_result_is_json_serializable():
    r = solve(case(time_budget_s=2, candidates=1, workers=1))
    json.dumps(r, ensure_ascii=False)


def test_equipment_keepout_rejects_overlapping_placement():
    """设备禁区：与禁区重叠的摆放在搜索中判为不可行。"""
    from layout_kernel import placement_sp as sp
    blocks = [{"poses": [{"W": 10, "H": 10}]}, {"poses": [{"W": 5, "H": 5}]}]
    pr = type("P", (), {})()
    pr.blocks, pr.keepout = blocks, [(20, 20, 30, 30)]
    assert sp._in_keepout(pr, [{"x": 0, "y": 0, "pose": 0}, {"x": 22, "y": 22, "pose": 0}])
    assert not sp._in_keepout(pr, [{"x": 0, "y": 0, "pose": 0}, {"x": 30, "y": 22, "pose": 0}])     # 贴边不算重叠


def test_equipment_keepout_in_route_only_is_reported():
    r = solve(case(time_budget_s=2, candidates=1, workers=1))
    c = copy.deepcopy(CASE)
    for d in c["devices"]:
        v = r["devices"][d["id"]]
        d["placed"] = [v["x_mm"], v["y_mm"], v["rot_deg"]]
    c["task"] = {"mode": "route_only"}
    v = r["devices"]["泵1"]
    c["equipment_keepout"] = [[v["x_mm"], v["y_mm"], v["x_mm"] + 100, v["y_mm"] + 100]]
    r2 = solve(c)
    assert not r2["ok"] and any("设备禁区" in x for x in r2["violations"])
