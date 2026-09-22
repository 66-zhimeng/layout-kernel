"""三维场景任务：节点（设备 / 三通）直接给世界坐标下的包围盒与端口，每根管给两端端口。

这是给已有项目接入用的入口：调用方已经有设备实例、姿态与现有管路，
不需要设备库和摆放，只需要在当前（或候选）姿态下重新布管、比较真实指标。

输入（场景坐标：米制、Y 向上）：
  nodes:  [{id, box: bool, orientations: [{ports: {key: {position, normal}}, box: {min, max} | null,
            spools: [[p, q], ...]}]}]          —— orientations[0] 是当前姿态，其余是允许换成的朝向
  routes: [{id, code, points, segments: [{r}], from: {key}, to: {key}, leadA, leadB, fixed, low}]
settings（全部必填）：
  routing:   布管参数（毫米，同 routing.REQUIRED；D_default_mm 仅作缺省，实际按每根管的外径）
  weights / scale：目标权重与归一化尺度（同 routing.Scene）
  约束开关与参数一律在 routing.constraints 下（见 constraints.py）；本模块用到 low_pipes、equipment_spacing
  oblique_stub_mm：斜向端口在基线里找不到斜段时，沿法向伸出的短管长度
  rotation_candidates：每个可转节点每轮最多真实重布几种朝向（先按端口距离与弯头下界估计排序）
  lower_bound：{"enabled": bool, "max_expansions": 单管精确搜索的扩展上限}；打开时报告下界与差距（见 _attach_bound）
  candidate_routing：评估候选移动时覆盖的求解参数（只需快速判断能否布下与代价，不必用出最终方案的完整强度），
    例如 {"max_iters": 15, "stall_iters": 4, "max_expansions": 300000, "cleanup_max_expansions": 200000}；
    基准与最终方案仍用 routing 里的完整参数
  pose_negotiation：姿态协商（见 _pose_negotiate）{"enabled": bool, "max_poses": 每个节点除当前外最多几个候选姿态,
    "pres_bends": 每有一根同节点的管选了别的姿态，附加多少个弯头当量（随协商轮次按 pres_fac_mult 增长）,
    "hist_bends": 姿态不一致时每轮累积的历史价（弯头当量）}
  pipe_rules：调用方的直管规则，用来推算每根管的 rho / lead（见 _straight_rules）：
    {"trim_ratio": 弯头最多占相邻直管的比例, "radius_margin_mm": 有效弯曲半径须超过管半径的量,
     "port_margin_mm": 端口直颈之外的余量, "safety": 放大系数,
     "rule_D_mm": 调用方直管规则所用的管径（调用方可能统一按某个代理管径计算，不按实际外径）,
     "rule_R_mm": 调用方的名义弯曲半径}
  管间净距仍按每根管的实际外径。
返回：{"ok", "violations", "metrics", "routes": [{id, code, points}]（米制、Y 向上，含两端端口）, "timing"}
"""
import math
import re
import time

from . import constraints
from . import routing as rt
SETTINGS_REQUIRED = ["routing", "weights", "scale", "oblique_stub_mm", "pipe_rules", "rotation_candidates", "lower_bound",
                     "candidate_routing", "pose_negotiation"]
POSE_REQUIRED = ["enabled", "max_poses", "pres_bends", "hist_bends"]
SERIAL_NETS = 8                                                 # 每个并行进程至少分到的管数
REACH_M = 1.0                                                   # 局部窗口外的东西离窗口多远以上才去掉（米；须大于各类净距）
RULES_REQUIRED = ["trim_ratio", "radius_margin_mm", "port_margin_mm", "safety", "rule_D_mm", "rule_R_mm"]
COMPACT_AXES = (2, 0)                                           # 整串靠拢只压水平两轴（场景坐标 Z、X）；竖向受顶棚与低位管约束
COMPACT_ORDERS = ((2, 0), (0, 2))                               # 两个轴的先手顺序：谁先压结果差别很大，且事先判断不出来，
                                                                # 所以两种都跑一遍、按校验器口径取更好的那个（见 _optimize 的 axes）
COMPACT_MOVES = 16                                              # 每轮每个轴真正评估的切面数上限（按估计收益取前几个）
COMPACT_LOOKAHEAD = 4                                           # 算设备行程时多看几倍的切面，再按修正后的收益挑（算行程比算管道余量贵）
COMPACT_STEPS = (1.0, 0.5, 0.25)                                # 每个切面试的行程比例（满行程常常布不通，留退路）
COMPACT_MIN_M = 0.05                                            # 行程小于这个值不值得重布一次（米）


def _straight_rules(leads, rules):
    """把调用方的直管规则换算成内核的 (rho, lmin)。内核要求：两弯之间直管 ≥ 2·rho + lmin，端口到弯 ≥ rho + lmin。
    调用方的规则形式：弯头让位 trim = min(R, trim_ratio·相邻直管)，有效半径 trim 须
    > D/2 + margin；端口处去掉让位后的直管 ≥ 直颈 + 变径（lead）+ port_margin。于是
      两弯之间最短  S_ee = (D/2 + margin) / trim_ratio
      端口到弯最短  S_pe = lead + R + port_margin          （若此时 trim_ratio·S_pe ≥ R）
                          (lead + port_margin)/(1 − trim_ratio)  （否则）
    U 形回弯两边是否相碰不在这里加严，由自身净距检查处理（routing 的 self_clearance.skip_along_mm）。"""
    k, t, D, R = rules["safety"], rules["trim_ratio"], rules["rule_D_mm"], rules["rule_R_mm"]
    s_rad = (D / 2 + rules["radius_margin_mm"]) / t                  # 每个弯头两侧直管都要满足（含端口后的第一个弯）
    s_ee = k * s_rad

    def s_pe(lead):
        v = lead + R + rules["port_margin_mm"]
        if t * v < R:
            v = (lead + rules["port_margin_mm"]) / (1 - t)
        return round(k * max(v, s_rad), 1)
    rho = s_ee / 2
    return round(rho, 1), 0.0, [s_pe(x) for x in leads]


def to_k(p):
    """场景坐标（m，Y 上）→ 内核坐标（mm，Z 上），取整到 0.1 mm。"""
    return (round(p[0] * 1000, 1), round(p[2] * 1000, 1), round(p[1] * 1000, 1))


def to_w(p):
    return [p[0] / 1000, p[2] / 1000, p[1] / 1000]


def _axis_dir(v, where):
    """法向量 → 轴向单位向量（内核坐标）；斜向返回 None。"""
    k = (v[0], v[2], v[1])
    big = [i for i in range(3) if abs(k[i]) > 1e-6]
    if len(big) != 1:
        return None
    i = big[0]
    if abs(abs(k[i]) - 1) > 1e-6:
        raise ValueError(f"{where}：法向量不是单位向量 {v}")
    out = [0, 0, 0]
    out[i] = 1 if k[i] > 0 else -1
    return tuple(out)


def _box_k(b):
    lo, hi = to_k(b["min"]), to_k(b["max"])
    return tuple(min(lo[i], hi[i]) for i in range(3)) + tuple(max(lo[i], hi[i]) for i in range(3))


def _oblique_stub(port_w, normal_w, baseline_points, stub_mm):
    """斜向端口：沿法向伸出一段斜直管，之后接正交布管。
    优先沿用基线里的斜段（长度与之后的正交方向都照旧）；没有则伸出 stub_mm，之后方向取法向的最大水平分量。"""
    P = to_k(port_w)
    n = (normal_w[0], normal_w[2], normal_w[1])
    if baseline_points and len(baseline_points) >= 3:
        a, b, c = (to_k(q) for q in baseline_points[:3])
        if a == P and sum(1 for i in range(3) if abs(a[i] - b[i]) > 0.5) > 1:
            d = [c[i] - b[i] for i in range(3)]
            i = max(range(3), key=lambda k: abs(d[k]))
            out = [0, 0, 0]
            out[i] = 1 if d[i] > 0 else -1
            return b, tuple(out), list(baseline_points[1])
    exact = [port_w[i] + normal_w[i] * stub_mm / 1000 for i in range(3)]   # 场景坐标下精确沿法向，不经取整
    i = max((0, 1), key=lambda k: abs(n[k]))
    out = [0, 0, 0]
    out[i] = 1 if n[i] > 0 else -1
    return to_k(exact), tuple(out), exact


def build_scene(inp, settings, fixed_ids=(), deltas=None, ghosts=None, alts=None, window=None):
    """场景输入 → routing.Scene。返回 (Scene, 元数据)。fixed_ids 中的管保持原路径，作为障碍。
    deltas = {节点 id: [dX, dY, dZ]}（场景坐标，米、Y 向上）：该节点的端口与包围盒整体平移。平移过的节点，
    其斜向端口不再沿用基线里的斜段，按法向重新伸出。
    姿态协商用：ghosts = {虚拟节点: 真实节点}（候选姿态，只提供端口，不是障碍，其端口可穿过真实节点当前的盒）；
    alts = {端口 key: [(虚拟端口 key, 姿态序号), ...]}：接在该端口上的管两端可改接这些候选端口。
    window = {"min", "max"}（场景坐标）：网格只建在这个范围内（候选评估用，见 _crop）。"""
    ghosts, alts = ghosts or {}, alts or {}
    deltas = deltas or {}
    miss = [k for k in SETTINGS_REQUIRED if k not in settings]
    if miss:
        raise KeyError(f"场景布管缺少设置：{miss}（不设默认值）")
    rp = dict(settings["routing"])
    cons_cfg = constraints.validate(rp.get("constraints"))
    miss = [k for k in RULES_REQUIRED if k not in settings["pipe_rules"]]
    if miss:
        raise KeyError(f"pipe_rules 缺少：{miss}")
    port_owner, port_w = {}, {}
    devices, fixed = {}, {}
    for n in inp["nodes"]:
        o = n["orientations"][0]
        d = deltas.get(n["id"])
        dw = tuple(d) if d else (0.0, 0.0, 0.0)
        box = None
        if o.get("box"):
            box = _box_k({"min": [o["box"]["min"][i] + dw[i] for i in range(3)],
                          "max": [o["box"]["max"][i] + dw[i] for i in range(3)]})
        devices[n["id"]] = {"box": box, "zones": [], "ports": {}}
        if n["id"] in ghosts:
            devices[n["id"]].update(ghost=True, pass_owner=ghosts[n["id"]])
        for key, p in o["ports"].items():
            port_owner[key] = n["id"]
            port_w[key] = {"position": [p["position"][i] + dw[i] for i in range(3)], "normal": p["normal"]} if d else p
    by_key = {}
    for r in inp["routes"]:
        for end, pts in (("from", r["points"]), ("to", r["points"][::-1])):
            by_key.setdefault(r[end]["key"], pts)
    stubs = {}                                                  # 端口 key → (斜段终点, 其后方向)
    for key, p in port_w.items():
        d = _axis_dir(p["normal"], f"端口 {key}")
        owner = port_owner[key]
        if d is None:
            hint = None if owner in deltas else by_key.get(key)
            end, d2, exact = _oblique_stub(p["position"], p["normal"], hint, settings["oblique_stub_mm"])
            stubs[key] = exact
            devices[owner]["ports"][key] = (*end, *d2)
            devices[owner].setdefault("fitting_ports", []).append(key)   # 斜段末端是 45° 弯，按弯头计直管
        else:
            devices[owner]["ports"][key] = (*to_k(p["position"]), *d)
    nets, meta = [], {}
    for r in inp["routes"]:
        D = 2000 * max(s["r"] for s in r["segments"])
        if r["id"] in fixed_ids or r.get("fixed"):
            ends = [port_owner[r[k]["key"]] if devices[port_owner[r[k]["key"]]]["box"] is None else None
                    for k in ("from", "to")]
            fixed[r["id"]] = {"points": [to_k(q) for q in r["points"]], "D_mm": round(D, 1), "trim_nodes": ends,
                              "ends": [port_owner[r[k]["key"]] for k in ("from", "to")]}
            continue
        ka, kb = r["from"]["key"], r["to"]["key"]

        def port_straight(k, lead):                             # 布管器另行加上盒内长度，这里扣掉避免重复
            i = _inside_mm(devices[port_owner[k]], k) if k not in stubs else 0.0
            return _straight_rules([_lead_with_inside(lead, i, D, settings["pipe_rules"], cons_cfg)],
                                   settings["pipe_rules"])[2][0] - i
        rho, lmin, _ps = _straight_rules([0.0], settings["pipe_rules"])
        la, lb = 1000 * r["leadA"], 1000 * r["leadB"]
        pst = {k_: port_straight(k_, ld) for k_, ld in ((ka, la), (kb, lb)) if k_ not in stubs}
        net = {"id": r["id"], "terms": [(port_owner[ka], ka), (port_owner[kb], kb)],
               "D_mm": round(D, 1), "rho_mm": rho, "lead_mm": lmin,
               **{k: r[k] for k in ("weight_length", "weight_bends", "weight_height_changes") if k in r},
               "port_straight_mm": pst}
        if ka in alts or kb in alts:
            sides = []
            for k_, ld in ((ka, la), (kb, lb)):
                side = [((port_owner[k_], k_), (port_owner[k_], 0))]
                for vk, pose in alts.get(k_, ()):
                    side.append(((port_owner[vk], vk), (port_owner[k_], pose)))
                    if vk not in stubs:
                        pst[vk] = port_straight(vk, ld)
                sides.append(side)
            net["alts"] = sides
        if r.get("low") and cons_cfg["low_pipes"]["enabled"]:
            net["zc_max_mm"] = cons_cfg["low_pipes"]["zc_max_mm"]
        nets.append(net)
        meta[r["id"]] = {"code": r.get("code"), "from": r["from"]["key"], "to": r["to"]["key"]}
    # 节点内部短管（三通内的接管等）：对不接在该节点上的管是障碍；随节点平移
    spools = []
    attached = {}
    for r in inp["routes"]:
        for end in ("from", "to"):
            attached.setdefault(port_owner[r[end]["key"]], []).append(2000 * max(s["r"] for s in r["segments"]))
    for n in inp["nodes"]:
        d = deltas.get(n["id"]) or (0.0, 0.0, 0.0)
        for p_, q_ in n["orientations"][0].get("spools") or []:
            spools.append({"owner": n["id"], "D_mm": round(max(attached.get(n["id"], [rp["D_default_mm"]])), 1),
                           "points": [to_k([p_[i] + d[i] for i in range(3)]), to_k([q_[i] + d[i] for i in range(3)])]})
    # 斜支口的斜段（端口 → 斜段终点）不在布管折线里，但它是实体管：按所属节点的内部短管处理（其他管不得靠近，
    # 接在该节点上的管豁免首末段）。斜段按管半径分成小段，避免整段的包围盒过于保守
    for r in inp["routes"]:
        if r["id"] in fixed_ids or r.get("fixed"):
            continue                                            # 固定管路的折线里已含斜段
        D = 2000 * max(s_["r"] for s_ in r["segments"])
        for end in ("from", "to"):
            key = r[end]["key"]
            if key in stubs:
                a, b = port_w[key]["position"], stubs[key]
                n = max(1, math.ceil(1000 * math.dist(a, b) / (D / 2)))
                spools.append({"owner": port_owner[key], "pipe": r["id"], "D_mm": round(D, 1),
                               "points": [to_k([a[i] + (b[i] - a[i]) * k / n for i in range(3)]) for k in range(n + 1)]})
    # 并行进程各自要建一遍网格：管少时并行不划算，按每进程至少 SERIAL_NETS 根管分配
    rp["route_workers"] = max(1, min(rp["route_workers"], len(nets) // SERIAL_NETS))
    pipe_keepout = [_box_k({"min": b[:3], "max": b[3:]}) for b in inp.get("pipe_keepout", [])]
    sc = rt.Scene(devices, nets, rp, settings["weights"], settings["scale"], fixed, spools, pipe_keepout)
    sc.equipment_keepout = [_box_k({"min": b[:3], "max": b[3:]}) for b in inp.get("equipment_keepout", [])]
    if window is not None:
        sc.window = _box_k(window)
    return sc, {"stubs": stubs, "port_w": port_w, "meta": meta}


def _inside_mm(dev, key):
    """端口在自身设备盒内时，沿法向到盒面的距离（mm）；否则 0。"""
    b = dev["box"]
    x = dev["ports"][key]
    if b is None or not all(b[k] - 1e-9 <= x[k] <= b[k + 3] + 1e-9 for k in range(3)):
        return 0.0
    a = [k for k in range(3) if x[3 + k]][0]
    return (b[a + 3] - x[a]) if x[3 + a] > 0 else (x[a] - b[a])


def _lead_with_inside(lead, inside, D, rules, cons_cfg):
    """端口在设备盒内：调用方的弯头让位按整段直管（含盒内部分）的比例计，弯头圆弧须整段在盒外并与设备留净距。
    因此把盒内长度并入端口直颈再套用调用方的直管规则；盒外至少留 管半径 + 管—设备净距 − 端口余量。"""
    if inside <= 0:
        return lead
    gap = cons_cfg["pipe_equipment_clearance"]["gap_mm"] if cons_cfg["pipe_equipment_clearance"]["enabled"] else 0.0
    return inside + max(lead, D / 2 + gap - rules["port_margin_mm"])


def _moved_violations(sc, deltas):
    """equipment_spacing：被移动设备的包围盒与其他设备的间距，及与固定管道（不参与本次布管）的净距。"""
    if not deltas:
        return []
    out = []
    if sc.cons["equipment_keepout"]["enabled"]:
        for nid in deltas:                                              # 设备禁区：无盒节点按端口外接盒
            d = sc.dev[nid]
            b = d["box"] or tuple(f(v[i] for v in d["ports"].values()) for f in (min, max) for i in range(3))
            if any(rt._gap(b, k) < -1e-6 for k in sc.equipment_keepout):
                out.append(f"移动后 {nid} 进入设备禁区")
    c = sc.cons["equipment_spacing"]
    if not c["enabled"]:
        return out
    boxes = {did: d["box"] for did, d in sc.dev.items() if d["box"] is not None}
    for nid in deltas:
        b = boxes.get(nid)
        if b is None:
            continue
        for oid, ob in boxes.items():
            if oid != nid and rt._gap(b, ob) < c["gap_mm"] - 1e-6:
                out.append(f"移动后 {nid} 与 {oid} 设备间距不足")
        for fid, f in sc.fixed_routes.items():
            if nid in f.get("ends", ()):                                # 接在本设备上的管：布它时已查过与设备的净距
                continue
            pts = [tuple(q) for q in f["points"]]
            for p, q in zip(pts, pts[1:]):
                if rt._gap(rt._box(p, q, f["D_mm"] / 2), b) < sc.gap_ep - 1e-6:
                    out.append(f"移动后 {nid} 与固定管道 {fid} 净距不足")
                    break
    return out


def _route_once(inp, settings, fixed_ids=(), deltas=None, log=None, window=None):
    sc, meta = build_scene(inp, settings, fixed_ids, deltas, window=window)
    moved_viol = _moved_violations(sc, deltas)
    if moved_viol:                                                   # 移动本身不合法：不必布管
        return {"sc": sc, "meta": meta, "routes": {}, "viol": moved_viol, "met": {"per_net": {}}, "ok": False,
                "cost": math.inf, "history": [{}], "G": None}
    try:
        routes, history, G = rt.negotiate(sc, log=log or (lambda _l: None))
    except rt.OffGrid as ex:                                         # 移动后端口超出布管空间（如高过顶棚）：不可行
        return {"sc": sc, "meta": meta, "routes": {}, "viol": [f"端口超出布管范围：{ex}"], "met": {"per_net": {}},
                "ok": False, "cost": math.inf, "history": [{}], "G": None}
    viol, met = rt.check_routes(sc, routes)
    ok = not viol and len(routes) == len(sc.nets)
    cost = sum(rt.net_cost(sc, nid, v) for nid, v in met["per_net"].items())
    return {"sc": sc, "meta": meta, "routes": routes, "viol": viol, "met": met, "ok": ok,
            "cost": cost if ok else math.inf, "history": history, "G": G}


def _baseline(inp, settings, deltas=None):
    """把输入里范围内各管的现有路径换算成内核折线，用内核校验器检查并计算代价，作为比较基准（省去一次整网重布）。
    斜口的斜段不在内核折线里：沿用基线斜段时去掉它。任何一根不符合内核规则（非轴向、端口不符、违规）就返回
    (None, 原因)，由调用方改为整网重布。返回的结果带 "web"：原样的场景路径（不经取整）。"""
    sc, meta = build_scene(inp, settings, (), deltas)
    owner = {k: n["id"] for n in inp["nodes"] for k in n["orientations"][0]["ports"]}
    routes, web = {}, {}
    for r in inp["routes"]:
        if r.get("fixed"):
            continue
        a, z = r["from"]["key"], r["to"]["key"]
        pts = [to_k(q) for q in r["points"]]
        if a in meta["stubs"]:
            pts = pts[1:]
        if z in meta["stubs"]:
            pts = pts[:-1]
        pts = [q for i, q in enumerate(pts) if i == 0 or q != pts[i - 1]]
        if len(pts) < 2:
            return None, (f"{r.get('code')} 路径过短", frozenset())
        routes[r["id"]] = [{"start": (owner[a], a), "end": ("port", (owner[z], z)), "points": pts}]
        web[r["id"]] = r["points"]
    try:
        viol, met = rt.check_routes(sc, routes)
    except ValueError as ex:
        return None, (str(ex), frozenset())
    if viol:
        bad = frozenset(t for v in viol for t in re.findall(r"[\w\-]+", v) if t in routes)
        if len(bad) > max(3, len(routes) // 5):                      # 大面积不符：不如整网重布
            bad = frozenset()
        return None, (f"{len(viol)} 条不符合内核规则，如 {viol[0]}", bad)
    cost = sum(rt.net_cost(sc, nid, v) for nid, v in met["per_net"].items())
    return {"sc": sc, "meta": meta, "routes": routes, "viol": [], "met": met, "ok": True, "cost": cost,
            "history": [{}], "G": None, "web": web}, None


def _to_web_routes(meta, routes):
    out = []
    for nid, brs in routes.items():
        m = meta["meta"][nid]
        pts = [tuple(q) for q in brs[0]["points"]]
        a, b = brs[0]["start"][1], brs[0]["end"][1][1]
        if a != m["from"]:                                      # 统一成原管道的 from → to 方向
            pts, a, b = pts[::-1], b, a
        A, B = list(meta["port_w"][a]["position"]), list(meta["port_w"][b]["position"])
        SA, SB = meta["stubs"].get(a), meta["stubs"].get(b)
        inner = [to_w(q) for q in pts[1:-1]]
        # 内核坐标取整到 0.1 mm；与端口（或斜段终点）共线的坐标改回精确值，避免调用方看到微斜的管段
        refs = [c for c in (A, B, SA, SB) if c is not None]
        for q in inner:
            for k in range(3):
                for c in refs:
                    if abs(q[k] - c[k]) < 6e-4:
                        q[k] = c[k]
                        break
        full = [A] + ([SA] if SA else []) + inner + ([SB] if SB else []) + [B]
        out.append({"id": nid, "code": m["code"], "points": full})
    return out


def route_scene(inp, settings, fixed_ids=(), log=None):
    """设备不动，重布（未固定的）全部管道。返回场景坐标下的结果。"""
    t0 = time.time()
    r = _route_once(inp, settings, fixed_ids, None, log)
    return {"ok": r["ok"], "violations": r["viol"],
            "metrics": {k: v for k, v in r["met"].items() if k != "per_net"},
            "routes": _to_web_routes(r["meta"], r["routes"]), "offsets": {}, "grid_nodes": r["G"].size if r["G"] is not None else None,
            "iterations": len(r["history"]), "timing": {"route_s": round(time.time() - t0, 1)}}


# ================================================================== 设备移动
def _k_ports(node):
    """节点当前姿态下的端口（内核坐标）：{key: (位置, 轴向法向或 None)}。"""
    return {k: (to_k(p["position"]), _axis_dir(p["normal"], k)) for k, p in node["orientations"][0]["ports"].items()}


def _inline_axis(node):
    """两口、法向相反且沿同一坐标轴的节点（在线传感器、阀门等）→ 该轴；否则 None。"""
    ps = list(_k_ports(node).values())
    if len(ps) != 2 or ps[0][1] is None or ps[1][1] is None:
        return None
    a = [i for i in range(3) if ps[0][1][i]]
    return a[0] if a and ps[1][1][a[0]] == -ps[0][1][a[0]] else None


def _free_axes(inp):
    """节点 → (能否沿 X, 能否沿 Y, 能否沿 Z 移动)。取自输入的 move_axes（场景坐标，Y 向上），缺省三轴都能动。"""
    out = {}
    for n in inp["nodes"]:
        ax = n.get("move_axes")
        if ax is None:
            out[n["id"]] = (True, True, True)
            continue
        if not isinstance(ax, (list, tuple)) or len(ax) != 3:
            raise ValueError(f"节点 {n['id']} 的 move_axes 必须是三个布尔值（场景坐标 X, Y, Z）")
        out[n["id"]] = tuple(bool(v) for v in ax)
    return out


def _candidates(inp, meta0, movable, radius_m):
    """候选移动：[(说明, {节点: delta})]。
    1. 串联拉直：相连的在线设备（同一轴向）整串平移到同一条直线上。候选直线取自串外端点（斜口取斜段终点）
       与串内各端口的横向坐标。
    2. 单个对齐：每个可移动节点对齐到相连管道另一端的端口直线（两个横向坐标都对齐，或只对齐其一）。"""
    nodes = {n["id"]: n for n in inp["nodes"]}
    owner = {k: n["id"] for n in inp["nodes"] for k in n["orientations"][0]["ports"]}
    kdir = {}                                                     # 端口 → 内核里的出管方向（斜口为斜段之后的方向）
    for nid, dev in meta0["sc"].dev.items():
        for k, v in dev["ports"].items():
            kdir[k] = v[3:]
    exact = {k: (meta0["stubs"].get(k) or p["position"]) for k, p in meta0["port_w"].items()}   # 场景坐标（米）

    def wdir(k):                                                  # 内核方向 → 场景坐标轴下标
        d = kdir[k]
        return {0: 0, 1: 2, 2: 1}[[i for i in range(3) if d[i]][0]]
    pipes = [r for r in inp["routes"] if not r.get("fixed")]
    out = []
    # ---------------- 串联拉直
    inline = {nid: _inline_axis(nodes[nid]) for nid in movable}
    inline = {k: {0: 0, 1: 2, 2: 1}[v] for k, v in inline.items() if v is not None}    # 转成场景坐标轴
    adj = {nid: set() for nid in inline}
    for r in pipes:
        a, b = owner[r["from"]["key"]], owner[r["to"]["key"]]
        if a in inline and b in inline and inline[a] == inline[b]:
            adj[a].add(b); adj[b].add(a)
    seen = set()
    for start in inline:
        if start in seen:
            continue
        comp, stack = set(), [start]
        while stack:
            x = stack.pop()
            if x not in comp:
                comp.add(x); stack.extend(adj[x] - comp)
        seen |= comp
        ax = inline[start]
        perp = [i for i in range(3) if i != ax]
        lines = set()
        for r in pipes:
            for mine, other in (("from", "to"), ("to", "from")):
                if owner[r[mine]["key"]] in comp and owner[r[other]["key"]] not in comp:
                    k = r[other]["key"]
                    if wdir(k) == ax:                                # 串外端点朝向与串同轴：可以直接对齐
                        lines.add(tuple(exact[k][i] for i in perp))
        for nid in comp:
            for k in nodes[nid]["orientations"][0]["ports"]:
                lines.add(tuple(exact[k][i] for i in perp))
        for line in sorted(lines):
            move = {}
            for nid in comp:
                pos = exact[next(iter(nodes[nid]["orientations"][0]["ports"]))]
                d = [0.0, 0.0, 0.0]
                for j, i in enumerate(perp):
                    d[i] = line[j] - pos[i]
                if any(abs(v) > radius_m + 1e-9 for v in d):
                    break
                if any(abs(v) > 1e-7 for v in d):
                    move[nid] = tuple(d)
            else:
                if move:
                    out.append((f"串联拉直 {len(comp)} 个设备到直线 {line}", move))
    # ---------------- 单个对齐
    for nid in movable:
        for r in pipes:
            for mine, other in (("from", "to"), ("to", "from")):
                if owner[r[mine]["key"]] != nid or owner[r[other]["key"]] == nid:
                    continue
                k_me, k_ot = r[mine]["key"], r[other]["key"]
                if _axis_dir(nodes[nid]["orientations"][0]["ports"][k_me]["normal"], k_me) is None:
                    continue
                ax = wdir(k_me)
                perp = [i for i in range(3) if i != ax]
                for use in (perp, perp[:1], perp[1:]):
                    d = [0.0, 0.0, 0.0]
                    for i in use:
                        d[i] = exact[k_ot][i] - exact[k_me][i]
                    if any(abs(v) > 1e-7 for v in d) and all(abs(v) <= radius_m + 1e-9 for v in d):
                        out.append((f"对齐 {nid} 到 {r.get('code')} 另一端", {nid: tuple(d)}))
    uniq, keys = [], set()
    for name, move in out:
        key = tuple(sorted(move.items()))
        if key not in keys:
            keys.add(key); uniq.append((name, move))
    return uniq


def _dev_coord(node, exact, ax):
    """节点在场景坐标某轴上的位置：有盒取盒心，无盒取各端口的平均；既无盒又无端口返回 None。"""
    o = node["orientations"][0]
    if o.get("box"):
        return (o["box"]["min"][ax] + o["box"]["max"][ax]) / 2
    ps = [exact[k][ax] for k in o["ports"]]
    return sum(ps) / len(ps) if ps else None


def _cut_room(group, ax, side, pipes, owner, exact, np_):
    """切面一侧的 group 整体朝另一侧平移时，(可走的行程, 会变短的管数)。
    行程取各跨切面管道在该轴上可压缩余量（两端口在该轴上的间距减去最小直管长度）的最小值。"""
    room, shorter = math.inf, 0
    for r in pipes:
        ka, kb = r["from"]["key"], r["to"]["key"]
        in_a, in_b = owner[ka] in group, owner[kb] in group
        if in_a == in_b:                                          # 两端同侧：整串平移，这根管原样跟着走
            continue
        mine, other = (ka, kb) if in_a else (kb, ka)
        toward = side * (exact[mine][ax] - exact[other][ax])
        if toward <= 1e-9:                                        # 这根管反而会被拉长：不限制行程，由评估定优劣
            continue
        npar = np_.get(r["id"])
        reserve = (2 * npar["rho"] + npar["lmin"]) / 1000 if npar else 0.0   # 弯头圆弧 + 最小直管，单位米
        room = min(room, toward - reserve)
        shorter += 1
    return room, shorter


def _node_box(node):
    b = node["orientations"][0].get("box")
    return (list(b["min"]), list(b["max"])) if b else None


def _obstacles(inp, group, owner, gap_ep, gap_dev, routes_w):
    """组整体平移时撞得上的组外东西：组外设备的盒（按设备间距），以及两端都不在组里的管的管段盒
    （按管—设备净距）。两端有一端在组里的管会跟着重布，不算障碍。返回 [(min, max, 需留净距)]。
    管的几何取 routes_w 里的当前路径——inp 里的 points 是本次优化开始时的，接受过候选后就过时了。"""
    out = []
    for n in inp["nodes"]:
        if n["id"] in group:
            continue
        b = _node_box(n)
        if b:
            out.append((b[0], b[1], gap_dev))
    for r in inp["routes"]:
        if owner[r["from"]["key"]] in group or owner[r["to"]["key"]] in group:
            continue
        rad = max(s["r"] for s in r["segments"])
        pts = routes_w.get(r["id"]) or r["points"]
        for a, b in zip(pts, pts[1:]):
            out.append(([min(a[i], b[i]) - rad for i in range(3)],
                        [max(a[i], b[i]) + rad for i in range(3)], gap_ep))
    return out


def _travel_limit(boxes, obstacles, ax, side):
    """组沿 −side 方向整体平移，撞上组外障碍之前还能走多远（米）。
    只有在另外两个轴上（含净距）重叠的障碍才拦得住；已经在身后的障碍越走越远，不限制。"""
    perp = [i for i in range(3) if i != ax]
    limit = math.inf
    for lo, hi in boxes:
        for olo, ohi, gap in obstacles:
            if any(lo[i] > ohi[i] + gap or hi[i] < olo[i] - gap for i in perp):
                continue
            ahead = (lo[ax] - ohi[ax]) if side > 0 else (olo[ax] - hi[ax])
            if ahead >= -1e-9:
                limit = min(limit, ahead - gap)
    return max(0.0, limit)


def _compaction_candidates(inp, meta0, movable, radius_m, free=None, routes_w=None, axes=None):
    """整串靠拢：沿 X / Z 取一个切面，把切面一侧的可动设备整体朝另一侧平移，压缩整体占地。
    与“串联拉直”“单个对齐”互补——那两族的位移永远垂直于管道走向，只消弯头，不会缩短直管段。

    切面取自各设备在该轴上坐标的相邻中点（全部设备，可动的一串也要能和固定设备分开）。所有切面都估一遍
    收益（行程 × 会变短的管数），只把收益最大的 COMPACT_MOVES 个按 COMPACT_STEPS 分档发出去：挤紧过的
    切面余量变小，下一轮自然轮到别处，不会总压同几个地方。行程只是估计，靠拢会不会撞设备、够不够绕，
    由候选评估的重布与校验器判定。free 给出各节点哪些轴能动（见 _free_axes）：沿某轴靠拢时，该轴被锁的节点不进组。"""
    free, routes_w, axes = free or {}, routes_w or {}, axes or COMPACT_AXES
    nodes = {n["id"]: n for n in inp["nodes"]}
    owner = {k: n["id"] for n in inp["nodes"] for k in n["orientations"][0]["ports"]}
    exact = {k: (meta0["stubs"].get(k) or p["position"]) for k, p in meta0["port_w"].items()}
    np_ = meta0["sc"].np
    pipes = [r for r in inp["routes"] if not r.get("fixed")]
    scored, seen = [], set()
    for ax in axes:
        pos = {nid: c for nid, n in nodes.items() if (c := _dev_coord(n, exact, ax)) is not None}
        vals = sorted(set(pos.values()))
        for cut in [(a + b) / 2 for a, b in zip(vals, vals[1:])]:
            for side in (1, -1):                                  # side = +1：切面正侧的一串朝负向靠；−1 反之
                group = frozenset(nid for nid in movable if nid in pos and (pos[nid] - cut) * side > 0
                                  and free.get(nid, (True, True, True))[ax])
                if not group or (group, ax, side) in seen:
                    continue
                seen.add((group, ax, side))
                room, shorter = _cut_room(group, ax, side, pipes, owner, exact, np_)
                if shorter and room > COMPACT_MIN_M:
                    scored.append((room * shorter, room, shorter, ax, side, group))
    scored.sort(key=lambda t: -t[0])
    cons = meta0["sc"].cons
    gap_ep = meta0["sc"].gap_ep / 1000
    gap_dev = cons["equipment_spacing"]["gap_mm"] / 1000 if cons["equipment_spacing"]["enabled"] else 0.0
    # 管道余量只说明管子能压多短，设备还可能先撞上组外的管或设备。行程算起来比余量贵，所以只对排在
    # 前面的若干切面算，再按修正后的收益重排——否则"余量大但走不动"的切面会一直霸占每轮的名额。
    # 名额按轴分配。估计收益是"行程 × 会变短的管数"，管多的那个轴会把名额全占掉：实测 X 的切面
    # 平均让 16 根管变短、Z 只有 10 根，于是 Z 的候选虽然可行、收益也有 +54，却从来排不进名额，
    # 占地的另一边就一直压不下去。
    usable = []
    for ax in axes:
        same = [t for t in scored if t[3] == ax][:COMPACT_MOVES * COMPACT_LOOKAHEAD]
        fixed = []
        for _est, room, shorter, _ax, side, group in same:
            boxes = [b for nid in group if (b := _node_box(nodes[nid])) is not None]
            room = min(room, _travel_limit(boxes, _obstacles(inp, group, owner, gap_ep, gap_dev, routes_w), ax, side))
            if room > COMPACT_MIN_M:
                fixed.append((room * shorter, room, ax, side, group))
        fixed.sort(key=lambda t: -t[0])
        usable += fixed[:COMPACT_MOVES]
    usable.sort(key=lambda t: -t[0])
    out = []
    for _est, room, ax, side, group in usable:
        for frac in COMPACT_STEPS:
            step = min(room * frac, radius_m)
            if step <= COMPACT_MIN_M:
                continue
            d = [0.0, 0.0, 0.0]
            d[ax] = -side * step
            out.append((f"整串靠拢：{len(group)} 个设备沿轴 {ax} 移动 {d[ax]:.2f} m",
                        {nid: tuple(d) for nid in group}))
    return out


def _net_costs(r):
    """每根重布管的代价（与 _route_once 的总代价同一口径）。"""
    return {nid: rt.net_cost(r["sc"], nid, v) for nid, v in r["met"]["per_net"].items()}


def optimize_scene(inp, settings, log=None):
    """场景优化的对外入口：布管 + 设备平移 / 旋转 / 换口，附下界。输入、设置、结果见 docs/guide.md 第 4 节。
    log(line) 接收过程日志（可不给）。

    整串靠拢按轴轮流独占整轮，而**哪个轴先手**对结果影响很大：先手那个轴压完会重布一大片管，
    后手常常发现机会已经没了。两个轴的估计收益往往接近，事先判断不出该谁先，所以 COMPACT_ORDERS
    里的每种顺序都跑一遍，按校验器口径的代价取更好的那个；`seconds` 由各趟平分，总时长不变。"""
    log = log or (lambda _l: None)
    movable = any(n.get("move") for n in inp["nodes"])
    orders = COMPACT_ORDERS if movable and float(inp.get("radius") or 0) > 0 and len(COMPACT_ORDERS) > 1 else (None,)
    limit = inp.get("seconds")
    best = None
    for i, order in enumerate(orders):
        one = inp if limit is None or len(orders) == 1 else {**inp, "seconds": float(limit) / len(orders)}
        tag = "" if order is None else f"[先压轴 {order[0]}] "
        out, final_inp, deltas = _optimize(one, settings, lambda line: log(tag + line), axes=order)
        cost = out.get("cost")
        if order is not None:
            log(f"{tag}这一趟：{'通过' if out['ok'] else '有违规'}，代价 {cost if cost is not None else '—'}")
        better = best is None or (out["ok"] and cost is not None
                                  and (not best[0]["ok"] or best[0]["cost"] is None or cost < best[0]["cost"]))
        if better:
            best = (out, final_inp, deltas, order)
    if len(orders) > 1 and best[3] is not None:
        log(f"两种先手顺序都跑过，采用先压轴 {best[3][0]} 的结果（代价 {best[0]['cost']}）")
    return _attach_bound(best[0], best[1], settings, best[2], log)


def _attach_bound(out, final_inp, settings, deltas, log):
    """在最终位置 / 朝向下计算下界与差距（lower_bound.enabled 时）。代价与下界都是内核目标口径（含单管权重）。"""
    cfg = settings["lower_bound"]
    if not isinstance(cfg, dict) or not isinstance(cfg.get("enabled"), bool):
        raise KeyError("lower_bound 需要 {enabled: true/false, max_expansions}")
    if not cfg["enabled"] or not out.get("ok") or out.get("cost") is None:
        out["lower_bound"] = None
        return out
    t0 = time.time()
    sc, _meta = build_scene(final_inp, settings, (), deltas)
    per = rt.lower_bounds(sc, int(cfg["max_expansions"]), sc.rp["route_workers"] if len(sc.nets) >= 2 * SERIAL_NETS
                          else 1)
    missing = {k: why for k, (v, why) in per.items() if v is None}
    lb = sum(v for v, _ in per.values() if v is not None)
    valid = not missing
    out["lower_bound"] = {
        "valid": valid, "value": round(lb, 4) if valid else None, "cost": out["cost"],
        "gap": round((out["cost"] - lb) / out["cost"], 4) if valid and out["cost"] > 0 else None,
        "nets": len(per), "unresolved": dict(list(missing.items())[:20]), "seconds": round(time.time() - t0, 1),
        "note": ("设备位置与朝向固定为最终方案时的下界：各管单独求精确最短路（不考虑其他待布管、放宽同管自身净距）之和；"
                 "不是设备也能移动时的全局下界。代价为内核目标口径（管长、弯头、高度变化加权，含单管权重）。")}
    if valid:
        log(f"下界 {lb:.3f}，当前 {out['cost']:.3f}，差距 ≤ {out['lower_bound']['gap']:.1%}（{time.time() - t0:.1f} s）")
    else:
        log(f"下界不完整：{len(missing)} 根管没有得到精确最短路")
    return out


_EV = {}


def _ev_init(inp, light):
    _EV.update(inp=inp, light=light)


def _sub_input(inp, orient, inc, routes_w):
    """候选评估用的输入：只放开 inc 中的管，范围内其余的管按当前路径固定为障碍。"""
    return {**_posed(inp, orient),
            "routes": [r if r["id"] in inc or r.get("fixed") else {**r, "points": routes_w[r["id"]], "fixed": True}
                       for r in inp["routes"]]}


def _window(inp, deltas, inc, routes_w, margin_m):
    """候选评估的局部窗口（场景坐标）：要重布的管的现有路径、被移动节点（新姿态）的端口与盒，外扩 margin_m。"""
    pts = [q for rid in inc for q in routes_w[rid]]
    owner = {k: n["id"] for n in inp["nodes"] for k in n["orientations"][0]["ports"]}
    for r in inp["routes"]:
        if r["id"] in inc:
            for end in ("from", "to"):
                nid = owner[r[end]["key"]]
                o = next(n for n in inp["nodes"] if n["id"] == nid)["orientations"][0]
                d = deltas.get(nid, (0.0, 0.0, 0.0))
                pts += [[p["position"][i] + d[i] for i in range(3)] for p in o["ports"].values()]
                if o.get("box"):
                    pts += [[o["box"][c][i] + d[i] for i in range(3)] for c in ("min", "max")]
    return {"min": [min(q[i] for q in pts) - margin_m for i in range(3)],
            "max": [max(q[i] for q in pts) + margin_m for i in range(3)]}


def _crop(inp, window, reach_m, deltas):
    """去掉与窗口无关的部分：包围盒外扩 reach_m 后不与窗口相交的固定管路，以及不连任何保留管、自身也不靠近窗口的节点。
    新路径只在窗口内，被去掉的东西与它相距都超过 reach_m（大于管—管、管—设备净距与设备间距），不会影响可行性判断。"""
    lo, hi = window["min"], window["max"]

    def near(a, b):
        return all(a[i] - reach_m <= hi[i] and b[i] + reach_m >= lo[i] for i in range(3))
    routes = [r for r in inp["routes"] if not r.get("fixed")
              or near([min(q[i] for q in r["points"]) for i in range(3)], [max(q[i] for q in r["points"]) for i in range(3)])]
    keys = {r[e]["key"] for r in routes for e in ("from", "to")}
    nodes = []
    for n in inp["nodes"]:
        o = n["orientations"][0]
        d = deltas.get(n["id"], (0.0, 0.0, 0.0))                       # 按移动后的位置判断远近
        ps = [p["position"] for p in o["ports"].values()]
        if o.get("box"):
            ps += [o["box"]["min"], o["box"]["max"]]
        ps = [[q[i] + d[i] for i in range(3)] for q in ps]
        if keys & set(o["ports"]) or (ps and near([min(q[i] for q in ps) for i in range(3)],
                                                   [max(q[i] for q in ps) for i in range(3)])):
            nodes.append(n)
    return {**inp, "nodes": nodes, "routes": routes}


def _ev_one(task):
    """评估一个候选（可在工作进程中运行）。返回 (序号, 是否可行, 相关管的代价, 相关管的新路径)。
    只在相关管与被移动节点周围的局部窗口内布管（candidate_routing.window_mm）。"""
    k, trial, trial_orient, inc, routes_w = task
    sub = _sub_input(_EV["inp"], trial_orient, inc, routes_w)
    margin = _EV["light"]["candidate_routing"]["window_mm"] / 1000
    win = _window(sub, trial, inc, routes_w, margin)
    crop = _crop(sub, win, REACH_M, trial)
    kept = {n["id"] for n in crop["nodes"]}
    r = _route_once(crop, _EV["light"], (), {nid: d for nid, d in trial.items() if nid in kept}, window=win)
    if not r["ok"]:
        return k, False, None, None
    return k, True, _net_costs(r), {x["id"]: x["points"] for x in _to_web_routes(r["meta"], r["routes"])}


def _optimize(inp, settings, log=None, axes=None):
    """布管 + 设备移动 / 旋转 / 换向（局部或全局，范围由输入的 move 标记、候选朝向与 fixed 管决定）。
      1. 按当前姿态重布范围内全部管，得到基准；
      2. 每一轮生成候选（串联拉直、单个对齐、换朝向），并行评估：每个候选只重布与被移动对象相连的管，
         其余范围内的管按当前路径固定为障碍，用 candidate_routing 的轻量参数；
         取有改进的候选，按改进量从大到小挑出互不相干（移动对象与相关管都不重叠）的一批，
         合在一起再评估一次确认仍有改进后一次接受（确认不通过则只接受最好的一个）；
      3. 没有改进或到达时间上限后，按最终姿态用完整参数把范围内全部管联合重布一次，取两者中更好的。
    所有比较都用内核自己的指标（校验器口径）；最终是否采用由调用方的校验决定。"""
    log = log or (lambda _l: None)
    axes = axes or COMPACT_AXES
    t0 = time.time()
    limit = float(inp.get("seconds") or math.inf)
    radius = float(inp.get("radius") or 0)                        # 各轴移动范围（米，场景坐标）
    movable = [n["id"] for n in inp["nodes"] if n.get("move")]
    free = _free_axes(inp)                                        # 节点 → 三个场景轴能不能动（move_axes，缺省三轴都能动）
    rotatable = [n["id"] for n in inp["nodes"] if len(n["orientations"]) > 1]
    owner = {k: n["id"] for n in inp["nodes"] for k in (x for o in n["orientations"] for x in o["ports"])}
    scope = [r for r in inp["routes"] if not r.get("fixed")]
    first, why = _baseline(inp, settings)
    if first is None and why[1]:                                   # 只有少数管不符合：只重布这几根，其余沿用现有路径
        bad = why[1]
        cur = {r["id"]: r["points"] for r in inp["routes"]}
        r = _route_once(_sub_input(inp, {}, bad, cur), settings, (), None)
        if r["ok"]:
            cur.update({x["id"]: x["points"] for x in _to_web_routes(r["meta"], r["routes"])})
            first, why2 = _baseline({**inp, "routes": [{**x, "points": cur[x["id"]]} for x in inp["routes"]]}, settings)
            if first is not None:
                log(f"当前布局有 {len(bad)} 根管不符合内核规则，只重布这几根（{time.time() - t0:.1f} s）")
            else:
                why = why2
    if first is not None:
        log(f"以当前布局为基准（内核校验通过），代价 {first['cost']:.3f}（{time.time() - t0:.1f} s）；可移动节点 {len(movable)} 个，"
            f"可转节点 {len(rotatable)} 个，移动范围 ±{radius} m")
    else:
        first = _route_once(inp, settings, (), None)
        log(f"当前布局不能直接作基准（{why[0]}），按当前姿态重布：{'通过' if first['ok'] else '有违规'}，代价 {first['cost']:.3f}；"
            f"可移动节点 {len(movable)} 个，可转节点 {len(rotatable)} 个，移动范围 ±{radius} m")
    if not first["ok"]:
        return _result(first, {}, [], 0, first["cost"], log), inp, {}
    if "window_mm" not in settings["candidate_routing"]:
        raise KeyError("candidate_routing 缺少 window_mm（候选评估的局部窗口外扩量，不设默认值）")
    light = {**settings, "routing": {**settings["routing"],
                                     **{k: v for k, v in settings["candidate_routing"].items() if k != "window_mm"},
                                     "route_workers": 1}}
    routes_w = dict(first["web"]) if first.get("web") else         {r["id"]: r["points"] for r in _to_web_routes(first["meta"], first["routes"])}
    cost = _net_costs(first)
    orient, deltas, tried, accepted = {}, {}, 0, []
    info = first["meta"] | {"sc": first["sc"]}
    full = None                                                    # 与当前姿态一致的完整重布结果（有则最终不必再重布）
    pcfg = settings["pose_negotiation"]
    miss = [k for k in POSE_REQUIRED if not isinstance(pcfg, dict) or k not in pcfg]
    if miss:
        raise KeyError(f"pose_negotiation 缺少 {miss}（不设默认值）")
    if pcfg["enabled"] and (movable and radius > 0 or rotatable):
        now = _shifted(_posed(inp, orient), deltas)
        poses = _pose_sets(now, info, movable, rotatable, radius, orient, deltas, settings)
        picked = _pose_negotiate(inp, settings, orient, deltas, poses, log) if poses else {}
        if picked:
            n_orient = {**orient, **{nid: o for nid, (o, _d) in picked.items()}}
            n_deltas = {**deltas, **{nid: d for nid, (_o, d) in picked.items()}}
            n_deltas = {k: v for k, v in n_deltas.items() if any(abs(x) > 1e-9 for x in v)}
            r = _route_once(_posed(inp, n_orient), settings, (), n_deltas)
            for _retry in range(2):                                # 个别节点按正式规则布不通：退回原姿态再试
                if r["ok"] or not picked:
                    break
                bad = _violating_nodes(inp, r["viol"]) & set(picked)
                if not bad:
                    break
                log(f"姿态协商方案有 {len(r['viol'])} 条违规，{len(bad)} 个相关节点退回原姿态后重试")
                picked = {k: v for k, v in picked.items() if k not in bad}
                n_orient = {**orient, **{nid: o for nid, (o, _d) in picked.items()}}
                n_deltas = {k: v for k, v in {**deltas, **{nid: d for nid, (_o, d) in picked.items()}}.items()
                            if any(abs(x) > 1e-9 for x in v)}
                r = _route_once(_posed(inp, n_orient), settings, (), n_deltas)
            if picked and r["ok"] and r["cost"] < first["cost"] - 1e-9:
                orient, deltas, full = n_orient, n_deltas, r
                routes_w = {x["id"]: x["points"] for x in _to_web_routes(r["meta"], r["routes"])}
                cost = _net_costs(r)
                accepted.append({"move": f"姿态协商：{len(picked)} 个节点换姿态", "gain": round(first["cost"] - r["cost"], 4)})
                info = r["meta"] | {"sc": r["sc"]}
                log(f"姿态协商方案按正式规则重布通过，代价 {first['cost']:.3f} → {r['cost']:.3f}（{time.time() - t0:.1f} s）")
            else:
                log("姿态协商方案未采用：" + (f"代价 {r['cost']:.3f} 不优于 {first['cost']:.3f}" if r["ok"]
                                         else "按正式规则重布有违规：" + "；".join(r["viol"][:3])))
    workers = int(settings["routing"]["route_workers"])
    pool, rounds, stale = None, 0, 0
    try:
        while (movable and radius > 0 or rotatable) and time.time() - t0 < limit:
            now = _shifted(_posed(inp, orient), deltas)
            # 靠拢按轴轮流独占整轮。同轮里两个轴的候选会互相挤：管多的那个轴收益总是更大，而两边的
            # 设备组大面积重叠、不可能同批接受，于是小的那个轴每轮都排第二，等大的压完机会也没了。
            axis_now = (axes[rounds % len(axes)],)
            rounds += 1
            moves = []
            if movable and radius > 0:                             # 先对齐（局部、便宜），再整串靠拢（范围大、按估计收益排序）
                moves = (_candidates(now, info, movable, radius)
                         + _compaction_candidates(now, info, movable, radius, free, routes_w, axis_now))
            cands = [(nm, mv, {}) for nm, mv in moves]
            cands += _rotation_candidates(now, info, rotatable, orient, deltas, settings, settings["rotation_candidates"])
            tasks, meta = [], []
            for name, move, rot in cands:
                trial = dict(deltas)
                for nid, d in move.items():
                    acc = tuple(trial.get(nid, (0.0, 0.0, 0.0))[i] + d[i] for i in range(3))
                    if any(abs(v) > radius + 1e-9 for v in acc):
                        break
                    if any(abs(d[i]) > 1e-9 and not free[nid][i] for i in range(3)):   # 该轴被调用方锁住
                        break
                    trial[nid] = acc
                else:
                    moved = set(move)
                    inc = frozenset(r["id"] for r in scope if owner[r["from"]["key"]] in moved or owner[r["to"]["key"]] in moved)
                    if inc:
                        tasks.append((len(tasks), trial, {**orient, **rot}, inc, routes_w))
                        meta.append((name, moved, inc, move, rot))
            if not tasks:                                          # 本轮轮到的那个轴可能没有任何候选：换下一个轴再看
                stale += 1
                if stale >= len(axes):
                    break
                continue
            tried += len(tasks)
            deadline = t0 + limit
            if workers > 1 and len(tasks) >= 4:
                if pool is None:
                    import multiprocessing as mp
                    from concurrent.futures import ProcessPoolExecutor
                    pool = ProcessPoolExecutor(workers, mp_context=mp.get_context("spawn"), initializer=_ev_init,
                                               initargs=(inp, light))
                results = _collect(pool, tasks, deadline)
            else:
                _ev_init(inp, light)
                results = []
                for t_ in tasks:                                       # 串行：到时间就停，只用已评估的候选
                    if time.time() > deadline:
                        break
                    results.append(_ev_one(t_))
            if len(results) < len(tasks):
                log(f"到达时间上限：本轮只评估了 {len(results)}/{len(tasks)} 个候选")
            good = []
            for k, ok, new, rw in results:
                if ok:
                    inc = meta[k][2]
                    gain = sum(cost[x] for x in inc) - sum(new[x] for x in inc)
                    if gain > 1e-9:
                        good.append((gain, k, new, rw))
            if not good:
                stale += 1                                         # 轮换时某个轴本轮可能压根没有候选，要把所有轴都试过才算收敛
                log(f"本轮 {len(results)} 个候选都没有改进（其中可布通 {sum(1 for x in results if x[1])} 个）")
                if stale >= len(axes):
                    break
                continue
            stale = 0
            good.sort(key=lambda g: -g[0])
            batch, used_n, used_p = [], set(), set()
            for g in good:                                             # 互不相干的一批
                name, moved, inc, move, rot = meta[g[1]]
                if moved & used_n or inc & used_p:
                    continue
                batch.append(g); used_n |= moved; used_p |= inc
            if len(batch) > 1:                                         # 合在一起再确认一次
                trial, trial_orient, inc_all = dict(deltas), dict(orient), set()
                for g in batch:
                    _n, moved, inc, move, rot = meta[g[1]]
                    t_trial = tasks[g[1]][1]
                    for nid in moved:
                        trial[nid] = t_trial[nid]
                    trial_orient.update(rot)
                    inc_all |= inc
                _ev_init(inp, light)
                _k, ok, new, rw = _ev_one((0, trial, trial_orient, frozenset(inc_all), routes_w))
                gain = sum(cost[x] for x in inc_all) - sum(new[x] for x in inc_all) if ok else -math.inf
                if ok and gain > batch[0][0] - 1e-9:
                    full = None
                    deltas, orient = trial, trial_orient
                    routes_w.update(rw); cost.update(new)
                    for g in batch:
                        accepted.append({"move": meta[g[1]][0], "gain": round(g[0], 4)})
                    log(f"本轮并行接受 {len(batch)} 个候选，代价 -{gain:.3f}")
                    info = _info(inp, settings, orient, deltas)
                    continue
                batch = batch[:1]
            gain, k, new, rw = batch[0]
            full = None
            name = meta[k][0]
            deltas, orient = tasks[k][1], tasks[k][2]
            routes_w.update(rw); cost.update(new)
            accepted.append({"move": name, "gain": round(gain, 4)})
            log(f"接受：{name}，代价 -{gain:.3f}")
            info = _info(inp, settings, orient, deltas)
    finally:
        if pool is not None:
            pool.shutdown(wait=True, cancel_futures=True)
    total_inc = sum(cost.values())
    if full is not None:                                           # 最后一次接受的就是完整重布的结果
        return _result(full, deltas, accepted, tried, first["cost"], log, time.time() - t0, orient), _posed(inp, orient), deltas
    if accepted and time.time() - t0 > limit:
        log("到达时间上限，跳过最终联合重布，采用逐步移动的结果")
    if accepted and time.time() - t0 <= limit:
        final = _route_once(_posed(inp, orient), settings, (), deltas)   # 最终姿态下用完整参数联合重布
        if final["ok"] and final["cost"] <= total_inc + 1e-9:
            return _result(final, deltas, accepted, tried, first["cost"], log, time.time() - t0, orient), _posed(inp, orient), deltas
        log(f"联合重布未更好（{final['cost']:.3f} ≥ {total_inc:.3f}），采用逐步移动的结果")
    if accepted:
        # 逐步移动的结果是各候选分别在局部窗口里验证的：在全场景里再完整校验一次（约 1 s）
        posed = _posed(inp, orient)
        chk, why = _baseline({**posed, "routes": [{**r, "points": routes_w.get(r["id"], r["points"])} for r in posed["routes"]]},
                             settings, deltas)
        if chk is not None:
            out = _result(chk, deltas, accepted, tried, first["cost"], log, time.time() - t0, orient)
            return out, posed, deltas
        log(f"逐步移动的结果全场景校验未通过（{why[0]}），按最终姿态联合重布")
        final = _route_once(posed, settings, (), deltas)
        if final["ok"]:
            return _result(final, deltas, accepted, tried, first["cost"], log, time.time() - t0, orient), posed, deltas
        log("联合重布也未通过，保留原布局")
    return _result(first, deltas, accepted, tried, first["cost"], log, time.time() - t0), inp, {}


def _collect(pool, tasks, deadline):
    """并行评估候选，到 deadline 为止：返回已完成的结果，未开始的取消（已在运行的算完后丢弃）。"""
    from concurrent.futures import FIRST_COMPLETED, wait
    futs = [pool.submit(_ev_one, t_) for t_ in tasks]
    done, pending = set(), set(futs)
    while pending:
        left = deadline - time.time()
        if left <= 0:
            break
        d, pending = wait(pending, timeout=left, return_when=FIRST_COMPLETED)
        done |= d
    for f in pending:
        f.cancel()
    return [f.result() for f in futs if f in done]


def _info(inp, settings, orient, deltas):
    """当前姿态下的端口信息（生成候选用；不布管）。"""
    sc, meta = build_scene(_posed(inp, orient), settings, (), deltas)
    return meta | {"sc": sc}


def _result(r, deltas, accepted, tried, base_cost, log, seconds=0.0, orient=None):
    offsets = {nid: list(d) for nid, d in deltas.items() if any(abs(v) > 1e-9 for v in d)}
    return {"ok": r["ok"], "violations": r["viol"],
            "orientations": {k: v for k, v in (orient or {}).items() if v},   # 节点 → 输入 orientations 里的下标
            "metrics": {k: v for k, v in r["met"].items() if k != "per_net"},
            "routes": ([{"id": k, "code": r["meta"]["meta"][k]["code"], "points": v} for k, v in r["web"].items()]
                       if r.get("web") else _to_web_routes(r["meta"], r["routes"])), "offsets": offsets,
            "moves": accepted, "candidates_tried": tried,
            "base_cost": round(base_cost, 4) if base_cost < math.inf else None,
            "cost": round(r["cost"], 4) if r["cost"] < math.inf else None,
            "iterations": len(r["history"]), "grid_nodes": r["G"].size if r["G"] is not None else None,
            "timing": {"route_s": round(seconds, 1)}}


def _posed(inp, orient):
    """按朝向选择 {节点: 原始朝向下标} 改写节点：选中的朝向放到 orientations[0]；原始列表保存在 _all。"""
    nodes = []
    for n in inp["nodes"]:
        allo = n.get("_all", n["orientations"])
        o = orient.get(n["id"], 0)
        nodes.append({**n, "_all": allo, "orientations": [allo[o]] + [x for i, x in enumerate(allo) if i != o]})
    return {**inp, "nodes": nodes}


def _pose_est(ori, d, pipes, exact, kdir, w, sc):
    """节点取朝向 ori、平移 d（场景坐标，相对原始位置）时，相连各管的代价估计：端口间曼哈顿距离 + 至少几个弯。
    pipes = [(本节点端口, 对端端口)]；对端按当前位置（exact：场景坐标，斜口取斜段终点；kdir：内核出管方向）。"""
    est = 0.0
    for mine, other in pipes:
        if other not in exact or mine not in ori["ports"]:
            return math.inf
        pw = [ori["ports"][mine]["position"][i] + d[i] for i in range(3)]
        q = exact[other]
        man = 1000 * sum(abs(pw[i] - q[i]) for i in range(3))
        dp = _axis_dir(ori["ports"][mine]["normal"], mine)
        dq = kdir.get(other)
        if dp is None or dq is None:
            b = 1
        else:
            dpk, dqk = tuple(dp), tuple(dq)
            ax = [i for i in range(3) if dpk[i]][0]
            pk, qk = to_k(pw), to_k(q)
            facing = dpk == tuple(-v for v in dqk) and all(abs(pk[i] - qk[i]) < 0.5 for i in range(3) if i != ax) \
                and (qk[ax] - pk[ax]) * dpk[ax] > 0
            b = 0 if facing else (1 if dpk[ax] == 0 or dqk[ax] == 0 else 2)
        est += w["length"] * man / sc["L0"] + w["bends"] * b / sc["B0"]
    return est


def _rotation_candidates(inp, meta0, rotatable, orient, deltas, settings, top):
    """旋转 / 换向候选：[(说明, {节点: 0 平移}, {节点: 朝向下标})]。每个节点按“端口间曼哈顿距离 + 弯头下界”
    估计各朝向下相连管道的代价，取最好的 top 个（不含当前朝向）。"""
    owner = {k: n["id"] for n in inp["nodes"] for k in n["orientations"][0]["ports"]}
    exact = {k: (meta0["stubs"].get(k) or p["position"]) for k, p in meta0["port_w"].items()}
    kdir = {k: v[3:] for dev in meta0["sc"].dev.values() for k, v in dev["ports"].items()}
    w, sc = settings["weights"], settings["scale"]
    nodes = {n["id"]: n for n in inp["nodes"]}
    out = []
    for nid in rotatable:
        n = nodes[nid]
        d = deltas.get(nid, (0.0, 0.0, 0.0))
        pipes = [(r["from"]["key"], r["to"]["key"]) if owner[r["from"]["key"]] == nid else (r["to"]["key"], r["from"]["key"])
                 for r in inp["routes"] if not r.get("fixed") and nid in (owner[r["from"]["key"]], owner[r["to"]["key"]])]
        if not pipes:
            continue
        ests = [(_pose_est(ori, d, pipes, exact, kdir, w, sc), o) for o, ori in enumerate(n["_all"])]
        cur = orient.get(nid, 0)
        for est, o in sorted(ests)[:top + 1]:
            if o != cur and est < math.inf:
                out.append((f"{nid} 换成朝向 {o}（估计 {est:.2f}）", {nid: (0.0, 0.0, 0.0)}, {nid: o}))
    return out


def _pose_sets(now, info, movable, rotatable, radius, orient, deltas, settings):
    """每个可动节点的候选姿态 [(朝向下标, 平移)]，第 0 个是当前姿态。候选来自换朝向（原位）与平移候选
    （串联拉直、单个对齐拆到各节点），按 _pose_est 估计排序取前 max_poses 个；单独看就违反设备间距 / 设备禁区的去掉。"""
    cfg = settings["pose_negotiation"]
    cons_cfg = constraints.validate(settings["routing"]["constraints"])
    sp = cons_cfg["equipment_spacing"]
    boxes = {did: d["box"] for did, d in info["sc"].dev.items() if d["box"] is not None}
    keepout = [_box_k({"min": b[:3], "max": b[3:]}) for b in now.get("equipment_keepout", [])] \
        if cons_cfg["equipment_keepout"]["enabled"] else []
    owner = {k: n["id"] for n in now["nodes"] for k in n["orientations"][0]["ports"]}
    exact = {k: (info["stubs"].get(k) or p["position"]) for k, p in info["port_w"].items()}
    kdir = {k: v[3:] for dev in info["sc"].dev.values() for k, v in dev["ports"].items()}
    w, scl = settings["weights"], settings["scale"]
    nodes = {n["id"]: n for n in now["nodes"]}
    trans = {}
    for _name, move in (_candidates(now, info, movable, radius) if movable and radius > 0 else []):
        for nid, d in move.items():
            trans.setdefault(nid, set()).add(tuple(d))

    top = info["sc"].z_ceiling - info["sc"].D / 2 if info["sc"].z_ceiling is not None else math.inf

    def fits(nid, ori, d):
        if any(1000 * (p_["position"][1] + d[1]) > top + 1e-6 for p_ in ori["ports"].values()):
            return False                                           # 端口高过顶棚：不在布管空间内
        if not ori.get("box"):
            return True
        b = _box_k({"min": [ori["box"]["min"][i] + d[i] for i in range(3)], "max": [ori["box"]["max"][i] + d[i] for i in range(3)]})
        if any(rt._gap(b, k) < -1e-6 for k in keepout):
            return False
        return not sp["enabled"] or all(rt._gap(b, ob) >= sp["gap_mm"] - 1e-6 for oid, ob in boxes.items() if oid != nid)
    out = {}
    for nid in dict.fromkeys(list(movable) + list(rotatable)):
        n = nodes[nid]
        pipes = [(r["from"]["key"], r["to"]["key"]) if owner[r["from"]["key"]] == nid else (r["to"]["key"], r["from"]["key"])
                 for r in now["routes"] if not r.get("fixed") and nid in (owner[r["from"]["key"]], owner[r["to"]["key"]])]
        if not pipes:
            continue
        o0, d0 = orient.get(nid, 0), tuple(deltas.get(nid, (0.0, 0.0, 0.0)))
        cands = [(o, d0) for o in range(len(n["_all"])) if o != o0] if nid in rotatable else []
        for dd in trans.get(nid, ()):
            d = tuple(d0[i] + dd[i] for i in range(3))
            if all(abs(v) <= radius + 1e-9 for v in d):
                cands.append((o0, d))
        scored = sorted((_pose_est(n["_all"][o], d, pipes, exact, kdir, w, scl), o, d) for o, d in cands)
        keep = [(o, d) for est, o, d in scored if est < math.inf and fits(nid, n["_all"][o], d)][:cfg["max_poses"]]
        if keep:
            out[nid] = [(o0, d0)] + keep
    return out


def _pose_negotiate(inp, settings, orient, deltas, poses, log):
    """姿态协商：把每个节点的候选姿态做成虚拟节点（端口 + 内部短管；包围盒不是障碍），每根管在两端的候选端口间一次搜索，
    同一节点各管的姿态分歧按拥堵的办法逐轮加价（routing.negotiate），直到一致。返回 {节点: (朝向, 平移)}（只含换了的）。
    只用来选姿态：选定后由调用方按正式规则（设备盒、内部短管、完整参数）重布并校验。"""
    cfg = settings["pose_negotiation"]
    base = _posed(inp, orient)
    byid = {n["id"]: n for n in base["nodes"]}
    nodes, ghosts, alts = list(base["nodes"]), {}, {}
    for nid, plist in poses.items():
        for k, (o, d) in enumerate(plist):
            if k == 0:
                continue
            ori, vid = byid[nid]["_all"][o], f"{nid}#{k}"
            box = ({"min": [ori["box"]["min"][i] + d[i] for i in range(3)], "max": [ori["box"]["max"][i] + d[i] for i in range(3)]}
                   if ori.get("box") else None)
            nodes.append({"id": vid, "move": False, "orientations": [{
                "ports": {f"{key}#{k}": {"position": [p["position"][i] + d[i] for i in range(3)], "normal": p["normal"]}
                          for key, p in ori["ports"].items()}, "box": box,
                "spools": [[[q[i] + d[i] for i in range(3)] for q in seg] for seg in ori.get("spools") or []]}]})
            ghosts[vid] = nid
            for key in ori["ports"]:
                alts.setdefault(key, []).append((f"{key}#{k}", k))
    pose_settings = {**settings, "routing": {**settings["routing"], **settings["candidate_routing"]}}
    sc, _meta = build_scene({**base, "nodes": nodes}, pose_settings, (), deltas, ghosts, alts)
    sc.pose_cfg = cfg
    try:
        routes, history, _G = rt.negotiate(sc, log=lambda _l: None, cleanup_enabled=False)
    except rt.OffGrid as ex:
        log(f"姿态协商跳过：候选姿态的端口超出布管范围（{ex}）")
        return {}
    alt = {n["id"]: {tuple(t): pose for side in n.get("alts", ()) for t, pose in side} for n in sc.nets}
    votes = {}
    for nid, brs in routes.items():
        if not alt.get(nid):
            continue
        for t in (tuple(brs[0]["start"]), tuple(brs[0]["end"][1])):
            node, k = alt[nid][t]
            if node in poses:
                votes.setdefault(node, {}).setdefault(k, 0)
                votes[node][k] += 1
    picked = {}
    for node, v in votes.items():
        k = max(v, key=lambda x: (v[x], x == 0))                        # 多数；平票取当前姿态
        if k != 0:
            picked[node] = poses[node][k]
    split = sum(1 for v in votes.values() if len(v) > 1)
    log(f"姿态协商：{len(poses)} 个节点、每个最多 {cfg['max_poses']} 个候选姿态，{len(history)} 轮，"
        f"换姿态 {len(picked)} 个，仍不一致 {split} 个（按多数取）")
    return picked


def _violating_nodes(inp, viol):
    """违规说明里提到的节点：直接点名的节点，以及点名的管两端的节点。"""
    owner = {k: n["id"] for n in inp["nodes"] for o in n["orientations"] for k in o["ports"]}
    ends = {r["id"]: (owner[r["from"]["key"]], owner[r["to"]["key"]]) for r in inp["routes"]}
    ids = {n["id"] for n in inp["nodes"]}
    out = set()
    for v in viol:
        for tok in re.findall(r"[\w\-]+", v):
            if tok in ids:
                out.add(tok)
            elif tok in ends:
                out.update(ends[tok])
    return out


def _shifted(inp, deltas):
    """按已接受的平移更新节点端口 / 包围盒（场景坐标），供生成下一轮候选。"""
    if not deltas:
        return inp
    nodes = []
    for n in inp["nodes"]:
        d = deltas.get(n["id"])
        if not d:
            nodes.append(n)
            continue
        dw = tuple(d)
        o = n["orientations"][0]
        ports = {k: {"position": [p["position"][i] + dw[i] for i in range(3)], "normal": p["normal"]} for k, p in o["ports"].items()}
        box = {"min": [o["box"]["min"][i] + dw[i] for i in range(3)], "max": [o["box"]["max"][i] + dw[i] for i in range(3)]} if o.get("box") else None
        nodes.append({**n, "orientations": [{**o, "ports": ports, "box": box}] + n["orientations"][1:]})
    return {**inp, "nodes": nodes}
