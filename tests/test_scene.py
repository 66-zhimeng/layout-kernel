"""三维场景接口测试：以文档里的示例请求为准（examples/scene/request.json），保证示例始终可运行、结论不变。"""
import copy
import json
import subprocess
import sys
from pathlib import Path

import pytest

from layout_kernel import scene

REQ = json.loads((Path(__file__).parents[1] / "examples/scene/request.json").read_text(encoding="utf-8"))


def test_example_moves_sensor_onto_line():
    """示例：传感器平移到泵出口直线上，弯头 4 → 2，达到下界。"""
    out = scene.optimize_scene(copy.deepcopy(REQ["input"]), REQ["settings"])
    assert out["ok"] and out["violations"] == []
    assert out["metrics"]["bends"] == 2 and out["base_cost"] > out["cost"]
    assert list(out["offsets"]) == ["sensor"] and abs(out["offsets"]["sensor"][2] + 0.3) < 1e-9
    pts = {r["id"]: r["points"] for r in out["routes"]}
    assert pts["pipe_1"][0] == [1.2, 0.5, 0.4] and len(pts["pipe_1"]) == 2          # 两端含端口，供水 1 为直管
    assert out["lower_bound"]["valid"] and out["lower_bound"]["gap"] == 0


def test_locked_node_is_not_moved():
    """move=false 且只有一个朝向的节点不会被移动或旋转。"""
    inp = copy.deepcopy(REQ["input"])
    s = next(n for n in inp["nodes"] if n["id"] == "sensor")
    s["move"] = False
    s["orientations"] = s["orientations"][:1]
    out = scene.optimize_scene(inp, REQ["settings"])
    assert out["ok"] and out["offsets"] == {} and out["orientations"] == {}


def test_missing_setting_is_an_error():
    settings = {k: v for k, v in REQ["settings"].items() if k != "lower_bound"}
    with pytest.raises(KeyError, match="lower_bound"):
        scene.optimize_scene(copy.deepcopy(REQ["input"]), settings)


def test_cli_reports_errors_as_json():
    """命令行：出错时标准输出为 {"ok": false, "error": ...}，退出码 1。"""
    bad = json.dumps({"input": REQ["input"]}).encode("utf-8")
    p = subprocess.run([sys.executable, "-m", "layout_kernel.scene_cli"], input=bad, capture_output=True)
    assert p.returncode == 1
    out = json.loads(p.stdout.decode("utf-8"))
    assert out["ok"] is False and "settings" in out["error"]


def test_high_port_under_ceiling_is_on_grid():
    """顶棚余量按包络（最大）管径算，不能把细管的高位端口裁出网格：
    否则布管已通过、算下界时才抛 OffGrid，整个优化结果被丢掉（网页端表现为“优化未应用”）。"""
    inp, st = copy.deepcopy(REQ["input"]), copy.deepcopy(REQ["settings"])
    st["routing"]["constraints"]["ceiling"] = {"enabled": True, "z_max_mm": 560}   # 端口标高 500
    for k in ("pipe_pipe_clearance", "self_clearance", "pipe_equipment_clearance", "equipment_spacing"):
        st["routing"]["constraints"][k]["enabled"] = False
    inp["routes"][1].update(segments=[{"r": 0.5}], fixed=True)   # 固定粗管：只把包络管径抬到 1000
    inp["radius"] = 0
    out = scene.optimize_scene(inp, st)
    assert out["ok"] and out["violations"] == []
    assert out["lower_bound"]["valid"]


def _chain(gaps_m, radius_m):
    """一串沿 X 排开的设备：第一台固定（泵），其余可动，相邻两台的端口相距 gaps_m[i]，中间一根直管。
    设备盒长 1 m，端口在盒的两个端面上。"""
    xs, x = [], 0.0                                               # xs[i] = 第 i 台设备盒的左端
    for g in [0.0] + list(gaps_m):
        x += g
        xs.append(x)
        x += 1.0
    nodes, routes = [], []
    for i, x0 in enumerate(xs):
        ports = {}
        if i:
            ports[f"d{i}/in"] = {"position": [x0, 0.5, 0.5], "normal": [-1, 0, 0]}
        if i < len(xs) - 1:
            ports[f"d{i}/out"] = {"position": [x0 + 1.0, 0.5, 0.5], "normal": [1, 0, 0]}
        nodes.append({"id": f"d{i}", "move": i > 0, "orientations": [{
            "angle": 0, "swap": False, "box": {"min": [x0, 0.0, 0.0], "max": [x0 + 1.0, 1.0, 1.0]},
            "ports": ports, "spools": []}]})
        if i:
            routes.append({"id": f"pipe_{i}", "code": f"管 {i}", "from": {"key": f"d{i-1}/out"},
                           "to": {"key": f"d{i}/in"},
                           "points": [[xs[i - 1] + 1.0, 0.5, 0.5], [x0, 0.5, 0.5]], "segments": [{"r": 0.05}],
                           "leadA": 0, "leadB": 0, "fixed": False, "low": False})
    return {"nodes": nodes, "routes": routes, "radius": radius_m, "seconds": 60,
            "equipment_keepout": [], "pipe_keepout": []}


def test_compaction_pulls_a_device_along_the_pipe():
    """整串靠拢：可动设备沿管道轴向靠近固定设备，直管段真正变短。
    对齐类候选的位移永远垂直于管道走向，单靠它们管长一毫米都不会减。"""
    out = scene.optimize_scene(_chain([10.0], 4.0), copy.deepcopy(REQ["settings"]))
    assert out["ok"] and out["violations"] == []
    assert out["offsets"]["d1"][0] < -3.5                          # 移动范围 ±4 m，应当用满
    assert out["metrics"]["L_m"] < 6.5                             # 原来 10 m


def test_compaction_moves_a_whole_chain_together():
    """下游的一串设备一起平移：只动 d1 的话，管 1 缩短多少管 2 就拉长多少，总长不变；
    只有 d1、d2 整串一起靠拢才真的把总管长降下来。"""
    out = scene.optimize_scene(_chain([10.0, 2.0], 4.0), copy.deepcopy(REQ["settings"]))
    assert out["ok"] and out["violations"] == []
    assert out["offsets"]["d1"][0] < -3.5 and out["offsets"]["d2"][0] < -3.5
    assert out["metrics"]["L_m"] <= 8.5                            # 原来 10 + 2 = 12 m


def test_compaction_squeezes_every_gap_not_just_a_few():
    """切面按余量自适应挑选：挤紧过的地方余量变小，下一轮自然轮到别处。
    一串 7 台设备、每段空 5 m，移动范围放开后应当每一段都被挤到只剩最小直管长度。"""
    out = scene.optimize_scene(_chain([5.0] * 6, 100.0), copy.deepcopy(REQ["settings"]))
    assert out["ok"] and out["violations"] == []
    assert out["metrics"]["L_m"] < 3.0                             # 原来 6 × 5 = 30 m
    assert all(out["offsets"][f"d{i}"][0] < out["offsets"][f"d{i-1}"][0] for i in range(2, 7))
