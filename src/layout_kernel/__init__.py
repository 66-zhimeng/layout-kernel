"""设备与管道自动排布：计算内核。

    from layout_kernel import solve, validate_case, CaseError     # 任务书接口
    from layout_kernel import routing, scene, constraints          # 各模块

契约见 docs/contract.md。子模块按需导入（只用布管 / 场景接口时不加载摆放与分块的依赖）。
"""
__all__ = ["solve", "validate_case", "CaseError"]


def __getattr__(name):
    if name == "solve":
        from .api import solve
        return solve
    if name in ("validate_case", "CaseError"):
        from . import contract
        return getattr(contract, name)
    raise AttributeError(name)
