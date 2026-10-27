from arena_robots.bringup import BRINGUPS, Bringup


@BRINGUPS["mobile"].register("nav2")
def _load_nav2() -> type[Bringup]:
    from .nav2 import Nav2Bringup

    return Nav2Bringup


@BRINGUPS["mobile"].register("meshnav")
def _load_meshnav() -> type[Bringup]:
    from .meshnav import MeshNavBringup

    return MeshNavBringup


@BRINGUPS["mobile"].register("test-collision")
def _load_test_collision() -> type[Bringup]:
    from .test_collision import TestCollisionBringup

    return TestCollisionBringup


@BRINGUPS["mobile"].register("none")
def _load_none() -> type[Bringup]:
    from .none import NoneBringup

    return NoneBringup


@BRINGUPS["mobile"].register("external")
def _load_external() -> type[Bringup]:
    from .external import ExternalBringup

    return ExternalBringup


@BRINGUPS["mobile"].register("manual")
def _load_manual() -> type[Bringup]:
    from .manual import ManualBringup

    return ManualBringup


@BRINGUPS["mobile"].register("rosnav_rl")
def _load_rosnav_rl() -> type[Bringup]:
    from .rosnav_rl import RosnavRlBringup

    return RosnavRlBringup


@BRINGUPS["mobile"].register("drl")
def _load_drl() -> type[Bringup]:
    from .drl import DrlBringup

    return DrlBringup
