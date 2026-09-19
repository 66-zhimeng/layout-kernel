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
