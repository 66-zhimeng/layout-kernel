"""三维场景接口示例：读取 request.json → 优化（布管 + 设备平移 / 换朝向）→ 打印结果并写出 result.json。

    python examples/scene/run_scene.py                 # 先 pip install -e .

场景：泵出口与冷水机组进口不在同一条线上，中间的在线传感器现在位置不好，两根管各拐两个弯（共 4 个）。
内核把传感器平移到与泵出口同一条直线上，供水 1 变成直管，只剩供水 2 的两个弯。

命令行等价写法（其他语言的项目以子进程调用时就用这个）：

    layout-kernel-scene < examples/scene/request.json > result.json
"""
import json
import sys
from pathlib import Path

from layout_kernel import scene

HERE = Path(__file__).parent


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    req = json.loads((HERE / "request.json").read_text(encoding="utf-8"))
    out = scene.optimize_scene(req["input"], req["settings"], log=lambda line: print("  ·", line))
    print(f"\n通过校验：{out['ok']}  违规：{out['violations'] or '无'}")
    print(f"代价 {out['base_cost']} → {out['cost']}；弯头 {out['metrics']['bends']}，管长 {out['metrics']['L_m']} m")
    print(f"设备平移（米，Y 向上）：{out['offsets']}")
    print(f"换了朝向的节点（输入 orientations 的下标）：{out['orientations'] or '无'}")
    for r in out["routes"]:
        print(f"  {r['code']}：{r['points']}")
    lb = out["lower_bound"]
    if lb and lb["valid"]:
        print(f"下界 {lb['value']}，差距 ≤ {lb['gap']:.1%}（在最终设备位置下）")
    (HERE / "result.json").write_text(json.dumps(out, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    return 0 if out["ok"] else 1


if __name__ == "__main__":                       # route_workers > 1 时用多进程，必须放在这里
    sys.exit(main())
