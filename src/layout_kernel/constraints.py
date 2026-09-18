"""约束注册表：布管的每一条约束都由调用方显式开关与配置，内核不写死任何约束，也不设默认值。

配置写在布管参数 routing 的 "constraints" 下，注册表里的每一条都必须出现：

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

关闭（enabled=false）时只需写 enabled；打开时必须给出该约束的全部参数。未知的约束名直接报错，防止拼写错误被静默忽略。
"""

REGISTRY = {
    "pipe_pipe_clearance": {
        "doc": "不同管之间的最小净距（外壁之间）。关闭后管道之间不再互相避让，也不做管—管校验。",
        "params": {"gap_mm": "外壁净距 / mm"},
    },
    "pipe_equipment_clearance": {
        "doc": "管外壁与设备包围盒之间的最小净距。关闭后按 0 处理（可贴着设备，但不能穿过）。",
        "params": {"gap_mm": "净距 / mm"},
    },
    "self_clearance": {
        "doc": "同一根管上，沿管长相隔超过 skip_along_mm 的两段之间也要满足管—管净距（防止回绕贴近或自交）。"
               "需要 pipe_pipe_clearance 同时打开。",
        "params": {"skip_along_mm": "沿管长相隔多远以上才检查 / mm"},
    },
    "height_change_limit": {
        "doc": "每条支路高度变化次数上限（两个水平段之间的一段连续竖直管计 1 次）。",
        "params": {"max_changes": "上限（整数）"},
    },
    "ceiling": {
        "doc": "管顶最高标高。关闭后只受场景范围限制（端口、设备、固定管路的最高点之上再留 margin_mm）。",
        "params": {"z_max_mm": "管顶最高标高 / mm"},
    },
    "service_zones": {
        "doc": "设备检修区（平面矩形）在给定高度以下不得走管。",
        "params": {"height_mm": "检修区高度 / mm"},
    },
    "straight_lengths": {
        "doc": "管件之间的最短直管：弯头—弯头 ≥ 2ρ + ℓ_min，端口—弯头 ≥ ρ + ℓ_min，或调用方给出的每管、每端规则。"
               "关闭后只要求正交走管，不限直管长度。",
        "params": {},
    },
    "low_pipes": {
        "doc": "被标为 low 的管，中心线最高标高（例如机房低位支管须低于某高度）。",
        "params": {"zc_max_mm": "中心线最高标高 / mm"},
    },
    "junction_merge_exemption": {
        "doc": "几根管汇合于同一个无包围盒节点（三通等）时，节点口附近一段不算彼此冲突（汇合处的几何由管件本身决定）。",
        "params": {},
    },
    "internal_spools": {
        "doc": "节点内部短管（三通内的接管等）对其他管是障碍；接在该节点上的管只豁免首段与末段。",
        "params": {},
    },
    "equipment_keepout": {
        "doc": "设备不得进入的区域（区域本身作为输入数据给出：任务书 equipment_keepout 为平面矩形，场景接口 "
               "equipment_keepout 为三维盒）。摆放搜索中碰到禁区的方案判为不可行；场景接口移动 / 旋转设备时同样检查。",
        "params": {},
    },
    "pipe_keepout": {
        "doc": "管道不得进入的区域（区域作为输入数据给出：任务书 keepout、场景接口 pipe_keepout，均为三维盒）。"
               "布管时作为障碍，校验时检查。",
        "params": {},
    },
    "equipment_spacing": {
        "doc": "移动设备时：设备包围盒之间的最小间距；被移动设备与不参与布管的管道之间满足管—设备净距。",
        "params": {"gap_mm": "设备间距 / mm"},
    },
}


class ConstraintError(KeyError):
    """约束配置不完整或不合法。"""


def validate(cfg):
    """检查约束配置，返回规范化后的 {名称: {"enabled": bool, 参数...}}。"""
    if not isinstance(cfg, dict):
        raise ConstraintError("constraints 必须是对象，逐条列出注册表中的约束")
    unknown = sorted(set(cfg) - set(REGISTRY))
    if unknown:
        raise ConstraintError(f"未知的约束：{unknown}（可用：{sorted(REGISTRY)}）")
    missing = sorted(set(REGISTRY) - set(cfg))
    if missing:
        raise ConstraintError(f"约束配置缺少：{missing}（每一条都要显式写 enabled，不设默认值）")
    out = {}
    for name, spec in REGISTRY.items():
        c = cfg[name]
        if not isinstance(c, dict) or not isinstance(c.get("enabled"), bool):
            raise ConstraintError(f"约束 {name} 需要 enabled: true / false")
        if c["enabled"]:
            miss = [p for p in spec["params"] if p not in c]
            if miss:
                raise ConstraintError(f"约束 {name} 已启用，缺少参数 {miss}（{spec['doc']}）")
            for p in spec["params"]:
                if not isinstance(c[p], (int, float)) or isinstance(c[p], bool) or c[p] < 0:
                    raise ConstraintError(f"约束 {name}.{p} 必须是非负数，实际为 {c[p]!r}")
        extra = sorted(set(c) - {"enabled"} - set(spec["params"]))
        if extra:
            raise ConstraintError(f"约束 {name} 有未知参数 {extra}")
        out[name] = dict(c)
    if out["self_clearance"]["enabled"] and not out["pipe_pipe_clearance"]["enabled"]:
        raise ConstraintError("self_clearance 依赖 pipe_pipe_clearance，请同时打开或同时关闭")
    return out


def describe():
    """注册表的可读说明（给调用方生成界面或文档用）。"""
    return {name: {"doc": s["doc"], "params": dict(s["params"])} for name, s in REGISTRY.items()}
