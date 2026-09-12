"""Tests for arena_robots.assembly, the pure mount-resolution engine that matches a
robot's typed mounts + declared defaults against a per-type part request."""

from __future__ import annotations

import pytest
from arena_robots.assembly import AssemblySpec, Mount


def _mount(
    name: str,
    accepts: list[str],
    parent: str = "base_link",
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
    frame: str | None = None,
) -> Mount:
    return Mount(name=name, parent=parent, xyz=xyz, rpy=rpy, accepts=tuple(accepts), frame=frame)


class TestResolveDefaults:
    def test_default_only_resolve_uses_declared_defaults(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="top")]},
        )
        resolved = resolve(spec, {})
        assert len(resolved.placements) == 1
        p = resolved.placements[0]
        assert (p.type, p.variant, p.mount.name) == ("lidar", "sick_s300", "top")
        assert resolved.warnings == []

    def test_untouched_type_keeps_defaults(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, RequestPart, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"]), "front": _mount("front", ["camera"])},
            defaults={
                "lidar": [DefaultPart(variant="sick_s300", mount="top")],
                "camera": [DefaultPart(variant="d435", mount="front")],
            },
        )
        resolved = resolve(spec, {"lidar": [RequestPart(variant="vlp16")]})
        by_type = {p.type: p.variant for p in resolved.placements}
        assert by_type["lidar"] == "vlp16"
        assert by_type["camera"] == "d435"

    def test_touched_type_discards_defaults(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, RequestPart, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="top")]},
        )
        resolved = resolve(spec, {"lidar": [RequestPart(variant="vlp16")]})
        variants = [p.variant for p in resolved.placements]
        assert variants == ["vlp16"]

    def test_resolve_empty_request_and_omitted_clear_kwargs_are_equivalent(self):
        """``resolve(spec, {})`` (defaults-only) is unaffected by the new clear kwargs
        existing at all: omitting them must reproduce byte-identical output to passing
        them explicitly empty (replace-on-touch untouched-defaults path)."""
        from arena_robots.assembly import AssemblySpec, DefaultPart, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="top")]},
        )
        a = resolve(spec, {})
        b = resolve(spec, {}, cleared_sockets=frozenset(), cleared_types=frozenset())
        assert [(p.type, p.variant, p.mount.name) for p in a.placements] == [(p.type, p.variant, p.mount.name) for p in b.placements]


class TestResolveClear:
    def test_explicit_clear_removes_all_instances_of_type(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="top")]},
        )
        resolved = resolve(spec, {"lidar": []})
        assert resolved.placements == []
        assert resolved.warnings == []

    def test_clear_of_mountless_type_warns_not_errors(self):
        from arena_robots.assembly import AssemblySpec, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"])})
        resolved = resolve(spec, {"arm": []})
        assert resolved.placements == []
        assert len(resolved.warnings) == 1
        assert "arm" in resolved.warnings[0]

    def test_nonempty_request_of_mountless_type_errors(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"])})
        with pytest.raises(AssemblyError, match="arm"):
            resolve(spec, {"arm": [RequestPart(variant="panda")]})


class TestResolveClearKwargs:
    """Mount-centric clear model: a default part is dropped when its socket is
    in ``cleared_sockets``, its type is in ``cleared_types``, its socket is filled by a
    mount-centric request item, or its (unpinned) type has a shorthand request item.
    Clearing/touching is per-socket, not all-or-nothing across a type's every mount."""

    def _two_lidar_defaults_spec(self) -> AssemblySpec:
        from arena_robots.assembly import AssemblySpec, DefaultPart

        return AssemblySpec(
            mounts={"A": _mount("A", ["lidar"]), "B": _mount("B", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="a", mount="A"), DefaultPart(variant="b", mount="B")]},
        )

    def test_cleared_socket_drops_only_that_sockets_default(self):
        from arena_robots.assembly import resolve

        spec = self._two_lidar_defaults_spec()
        resolved = resolve(spec, {}, cleared_sockets=frozenset({"A"}))
        by_mount = {p.mount.name: p.variant for p in resolved.placements}
        assert by_mount == {"B": "b"}

    def test_cleared_type_drops_every_default_of_that_type(self):
        from arena_robots.assembly import resolve

        spec = self._two_lidar_defaults_spec()
        resolved = resolve(spec, {}, cleared_types=frozenset({"lidar"}))
        assert resolved.placements == []

    def test_socket_scoped_request_replaces_only_that_sockets_default(self):
        """Filling mount A via a mount-centric request item drops only A's default;
        B's default (same type) survives untouched."""
        from arena_robots.assembly import RequestPart, resolve

        spec = self._two_lidar_defaults_spec()
        resolved = resolve(spec, {"lidar": [RequestPart(variant="new", mount="A")]})
        by_mount = {p.mount.name: p.variant for p in resolved.placements}
        assert by_mount == {"A": "new", "B": "b"}

    def test_type_scoped_request_replaces_every_default_of_that_type(self):
        """An unpinned (shorthand) request item for a type discards ALL of that
        type's defaults, not just the mount it ends up landing on."""
        from arena_robots.assembly import RequestPart, resolve

        spec = self._two_lidar_defaults_spec()
        resolved = resolve(spec, {"lidar": [RequestPart(variant="new")]})
        assert [(p.mount.name, p.variant) for p in resolved.placements] == [("A", "new")]


class TestResolveAppend:
    def test_two_request_parts_accumulate(self):
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"]), "front": _mount("front", ["lidar"])})
        resolved = resolve(spec, {"lidar": [RequestPart(variant="sick"), RequestPart(variant="sick")]})
        assert len(resolved.placements) == 2
        assert {p.mount.name for p in resolved.placements} == {"top", "front"}


class TestResolvePins:
    def test_pin_success(self):
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"]), "front": _mount("front", ["lidar"])})
        resolved = resolve(spec, {"lidar": [RequestPart(variant="sick", mount="front")]})
        assert resolved.placements[0].mount.name == "front"

    def test_pin_to_unknown_mount_raises_and_lists_declared(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"])})
        with pytest.raises(AssemblyError, match="top"):
            resolve(spec, {"lidar": [RequestPart(variant="sick", mount="nope")]})

    def test_pin_to_mount_not_accepting_type_raises(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"front": _mount("front", ["camera"]), "top": _mount("top", ["lidar"])})
        with pytest.raises(AssemblyError, match="does not accept"):
            resolve(spec, {"lidar": [RequestPart(variant="sick", mount="front")]})

    def test_two_parts_pinned_to_one_mount_raises(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"])})
        with pytest.raises(AssemblyError, match="two parts pinned"):
            resolve(spec, {"lidar": [RequestPart(variant="a", mount="top"), RequestPart(variant="b", mount="top")]})


class TestResolveMatching:
    def test_greedy_trap_resolved_by_augmenting_paths(self):
        """camera only fits A; lidar fits A or B. Processed camera-then-lidar, a
        first-come allocator would burn A on camera's greedy pick and strand lidar;
        matching must still land both."""
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"A": _mount("A", ["lidar", "camera"]), "B": _mount("B", ["lidar"])})
        request = {"camera": [RequestPart(variant="d435")], "lidar": [RequestPart(variant="sick")]}
        resolved = resolve(spec, request)
        by_type = {p.type: p.mount.name for p in resolved.placements}
        assert by_type["camera"] == "A"
        assert by_type["lidar"] == "B"

    def test_matching_reassigns_earlier_pick_when_needed(self):
        """Reverse order: lidar processed first, greedily prefers A; matcher must
        reassign it to B once camera (A-only) needs the mount."""
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"A": _mount("A", ["lidar", "camera"]), "B": _mount("B", ["lidar"])})
        request = {"lidar": [RequestPart(variant="sick")], "camera": [RequestPart(variant="d435")]}
        resolved = resolve(spec, request)
        by_type = {p.type: p.mount.name for p in resolved.placements}
        assert by_type == {"lidar": "B", "camera": "A"}

    def test_declaration_order_preference_respected_when_slack_exists(self):
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"front": _mount("front", ["lidar"]), "top": _mount("top", ["lidar"])})
        resolved = resolve(spec, {"lidar": [RequestPart(variant="sick")]})
        assert resolved.placements[0].mount.name == "front"

    def test_cross_type_contention_over_single_shared_mount(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"A": _mount("A", ["lidar", "camera"])})
        with pytest.raises(AssemblyError):
            resolve(spec, {"lidar": [RequestPart(variant="sick")], "camera": [RequestPart(variant="d435")]})

    def test_over_capacity_failure_message_content(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"]), "front": _mount("front", ["lidar"])})
        request = {"lidar": [RequestPart(variant="sick") for _ in range(3)]}
        with pytest.raises(AssemblyError) as excinfo:
            resolve(spec, request)
        msg = str(excinfo.value)
        assert "3x lidar" in msg
        assert "2 lidar-mount" in msg
        assert "top" in msg and "front" in msg
        assert "drop one or add a mount" in msg


class TestPreferenceRanking:
    """Among maximum-cardinality assignments, ``_match`` must pick the unique
    minimum-summed-preference-rank one, not just whatever a naive first-declared-mount
    fit produces. accepts order is per-socket preference, consulted only under
    contention between mounts that could each take either item."""

    def test_min_rank_pairing_overrides_naive_first_fit(self):
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        # A prefers camera over lidar; B prefers lidar over camera. A naive
        # declaration-order-first-fit matcher (old algorithm) would burn A on
        # whichever item is processed first, landing camera->A/lidar->B OR the
        # reverse depending on processing order; the rank-optimal pairing is
        # always camera->A, lidar->B (summed rank 0), never the alternative
        # (summed rank 2).
        spec = AssemblySpec(mounts={"A": _mount("A", ["camera", "lidar"]), "B": _mount("B", ["lidar", "camera"])})
        request = {"lidar": [RequestPart(variant="sick")], "camera": [RequestPart(variant="d435")]}
        resolved = resolve(spec, request)
        by_type = {p.type: p.mount.name for p in resolved.placements}
        assert by_type == {"camera": "A", "lidar": "B"}

    def test_two_shorthands_contend_over_two_multitype_sockets(self):
        """End-to-end via build_request: two shorthand fills resolved against two
        sockets that each accept both types, in opposite preference order."""
        from arena_robots.assembly import AssemblySpec, build_request, resolve

        spec = AssemblySpec(mounts={"A": _mount("A", ["camera", "lidar"]), "B": _mount("B", ["lidar", "camera"])})
        request, cleared_sockets, cleared_types = build_request(spec, {"lidar": ["sick"], "camera": ["d435"]})
        resolved = resolve(spec, request, cleared_sockets=cleared_sockets, cleared_types=cleared_types)
        by_type = {p.type: p.mount.name for p in resolved.placements}
        assert by_type == {"camera": "A", "lidar": "B"}


class TestResolveParams:
    def test_default_params_flow_into_placement(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="top", params={"max_angle": 2.1})]},
        )
        resolved = resolve(spec, {})
        assert resolved.placements[0].params == {"max_angle": 2.1}

    def test_request_params_are_not_propagated(self):
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"])})
        resolved = resolve(spec, {"lidar": [RequestPart(variant="sick", params={"max_angle": 1.0})]})
        assert resolved.placements[0].params == {}


class TestWarnIfBlind:
    def test_warns_when_required_type_absent(self):
        from arena_robots.assembly import ResolvedAssembly, warn_if_blind

        resolved = ResolvedAssembly(placements=[])
        warnings = warn_if_blind(resolved, {"lidar"})
        assert len(warnings) == 1
        assert "lidar" in warnings[0]

    def test_no_warning_when_required_type_present(self):
        from arena_robots.assembly import Placement, ResolvedAssembly, warn_if_blind

        resolved = ResolvedAssembly(placements=[Placement(type="lidar", variant="sick", mount=_mount("top", ["lidar"]))])
        assert warn_if_blind(resolved, {"lidar"}) == []


class TestAssemblySpecParse:
    def test_parse_builds_mounts_defaults(self):
        from arena_robots.assembly import AssemblySpec

        spec = AssemblySpec.parse(
            {
                "mounts": {"top": {"parent": "base_link", "xyz": [0, 0, 0.2], "accepts": ["lidar"]}},
                "defaults": {"lidar": [{"variant": "sick_s300", "mount": "top", "params": {"max_angle": 2.1}}]},
            }
        )
        assert spec.mounts["top"].accepts == ("lidar",)
        assert spec.defaults["lidar"][0].params == {"max_angle": 2.1}

    def test_parse_accepts_preserves_declared_order(self):
        from arena_robots.assembly import AssemblySpec

        spec = AssemblySpec.parse({"mounts": {"top": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["camera", "lidar", "imu"]}}})
        assert spec.mounts["top"].accepts == ("camera", "lidar", "imu")

    def test_parse_declaration_order_is_allocation_preference(self):
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec.parse(
            {
                "mounts": {
                    "front": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lidar"]},
                    "rear": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lidar"]},
                }
            }
        )
        resolved = resolve(spec, {"lidar": [RequestPart(variant="sick")]})
        assert resolved.placements[0].mount.name == "front"

    def test_parse_rejects_default_mount_not_accepting_type(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec

        with pytest.raises(AssemblyError, match="does not accept"):
            AssemblySpec.parse(
                {
                    "mounts": {"top": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["camera"]}},
                    "defaults": {"lidar": [{"variant": "sick_s300", "mount": "top"}]},
                }
            )

    def test_parse_passes_overrides_onto_default_part(self):
        from arena_robots.assembly import AssemblySpec

        spec = AssemblySpec.parse(
            {
                "mounts": {"rear": {"parent": "base_link", "xyz": [0, 0, 0.2], "accepts": ["lidar"]}},
                "defaults": {"lidar": [{"variant": "sick_s300", "mount": "rear", "overrides": {"name": "lidar_rear", "topic": "scan/rear"}}]},
            }
        )
        assert spec.defaults["lidar"][0].overrides == {"name": "lidar_rear", "topic": "scan/rear"}

    def test_parse_default_overrides_defaults_to_empty(self):
        from arena_robots.assembly import AssemblySpec

        spec = AssemblySpec.parse(
            {
                "mounts": {"top": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lidar"]}},
                "defaults": {"lidar": [{"variant": "sick_s300", "mount": "top"}]},
            }
        )
        assert spec.defaults["lidar"][0].overrides == {}


class TestDefaultPartOptionalMount:
    """DefaultPart.mount is optional: an unpinned default joins the same
    matching pool as an unpinned request item, forced onto the sole accepting mount
    when there is exactly one."""

    def test_default_mount_omitted_in_yaml_parses_to_none(self):
        from arena_robots.assembly import AssemblySpec

        spec = AssemblySpec.parse(
            {
                "mounts": {"top": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lidar"]}},
                "defaults": {"lidar": [{"variant": "sick_s300"}]},
            }
        )
        assert spec.defaults["lidar"][0].mount is None

    def test_unpinned_default_forced_onto_sole_accepting_mount(self):
        from arena_robots.assembly import AssemblySpec, resolve

        spec = AssemblySpec.parse(
            {
                "mounts": {"top": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lidar"]}},
                "defaults": {"lidar": [{"variant": "sick_s300"}]},
            }
        )
        resolved = resolve(spec, {})
        assert len(resolved.placements) == 1
        assert resolved.placements[0].mount.name == "top"

    def test_unpinned_default_participates_in_matching_alongside_pinned(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, resolve

        spec = AssemblySpec(
            mounts={"A": _mount("A", ["lidar"]), "B": _mount("B", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="pinned", mount="A"), DefaultPart(variant="floating", mount=None)]},
        )
        resolved = resolve(spec, {})
        by_mount = {p.mount.name: p.variant for p in resolved.placements}
        assert by_mount == {"A": "pinned", "B": "floating"}


class TestMountFrame:
    """Mount.frame: identity stem decoupling addressing (name) from the
    sim2real frame/joint/sensor identity contract."""

    def test_frame_parses_when_declared(self):
        from arena_robots.assembly import AssemblySpec

        spec = AssemblySpec.parse(
            {
                "mounts": {
                    "front_laser": {
                        "parent": "base_link",
                        "xyz": [0, 0, 0],
                        "accepts": ["lidar"],
                        "frame": "front_laser_link",
                    }
                }
            }
        )
        assert spec.mounts["front_laser"].frame == "front_laser_link"

    def test_frame_defaults_to_none_when_omitted(self):
        from arena_robots.assembly import AssemblySpec

        spec = AssemblySpec.parse({"mounts": {"top": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lidar"]}}})
        assert spec.mounts["top"].frame is None

    def test_frame_field_directly_on_mount(self):
        m = _mount("top", ["lidar"], frame="custom_stem")
        assert m.frame == "custom_stem"


class TestBuildRequest:
    """LHS disambiguation (mount-centric addressing): a declared mount name is
    MOUNT-CENTRIC (mount wins over a same-named type on collision); otherwise a known
    accepted type is SHORTHAND. ``/`` only separates type from variant."""

    def _spec(self, accepts: list[str] | None = None) -> AssemblySpec:
        from arena_robots.assembly import AssemblySpec

        return AssemblySpec(mounts={"top": _mount("top", accepts if accepts is not None else ["lidar"])})

    def test_mount_centric_fill_with_type_slash_variant(self):
        from arena_robots.assembly import RequestPart, build_request

        spec = self._spec()
        request, cleared_sockets, cleared_types = build_request(spec, {"top": ["lidar/sick"]})
        assert request == {"lidar": [RequestPart(variant="sick", mount="top")]}
        assert cleared_sockets == frozenset()
        assert cleared_types == frozenset()

    def test_mount_centric_bare_variant_inferred_when_single_accept(self):
        from arena_robots.assembly import RequestPart, build_request

        spec = self._spec()
        request, _, _ = build_request(spec, {"top": ["sick"]})
        assert request == {"lidar": [RequestPart(variant="sick", mount="top")]}

    def test_mount_centric_bare_variant_errors_on_multi_accept_socket(self):
        from arena_robots.assembly import AssemblyError, build_request

        spec = self._spec(["lidar", "camera"])
        with pytest.raises(AssemblyError, match="accepts multiple types"):
            build_request(spec, {"top": ["sick"]})

    def test_mount_centric_type_not_accepted_errors(self):
        from arena_robots.assembly import AssemblyError, build_request

        spec = self._spec(["lidar"])
        with pytest.raises(AssemblyError, match="does not accept"):
            build_request(spec, {"top": ["camera/d435"]})

    def test_shorthand_fill_leaves_mount_unpinned(self):
        from arena_robots.assembly import RequestPart, build_request

        spec = self._spec()
        request, _, _ = build_request(spec, {"lidar": ["sick"]})
        assert request == {"lidar": [RequestPart(variant="sick", mount=None)]}

    def test_shorthand_multi_value_produces_multiple_instances(self):
        from arena_robots.assembly import RequestPart, build_request

        spec = self._spec()
        request, _, _ = build_request(spec, {"lidar": ["sick", "vlp16"]})
        assert request == {"lidar": [RequestPart(variant="sick", mount=None), RequestPart(variant="vlp16", mount=None)]}

    def test_shorthand_none_sets_cleared_types(self):
        from arena_robots.assembly import build_request

        spec = self._spec()
        request, cleared_sockets, cleared_types = build_request(spec, {"lidar": ["none"]})
        assert request == {}
        assert cleared_types == frozenset({"lidar"})
        assert cleared_sockets == frozenset()

    def test_socket_none_sets_cleared_sockets(self):
        from arena_robots.assembly import build_request

        spec = self._spec()
        request, cleared_sockets, cleared_types = build_request(spec, {"top": ["none"]})
        assert request == {}
        assert cleared_sockets == frozenset({"top"})
        assert cleared_types == frozenset()

    def test_mount_wins_name_collision(self):
        """A mount literally named after its own accepted type resolves mount-centric,
        not shorthand: the collision is won by the mount."""
        from arena_robots.assembly import AssemblySpec, RequestPart, build_request

        spec = AssemblySpec(mounts={"lidar": _mount("lidar", ["lidar"])})
        request, _, _ = build_request(spec, {"lidar": ["sick"]})
        assert request == {"lidar": [RequestPart(variant="sick", mount="lidar")]}

    def test_unknown_lhs_errors(self):
        from arena_robots.assembly import AssemblyError, build_request

        spec = self._spec()
        with pytest.raises(AssemblyError, match="unknown 'bogus'"):
            build_request(spec, {"bogus": ["x"]})

    def test_socket_targeted_more_than_once_errors(self):
        from arena_robots.assembly import AssemblyError, build_request

        spec = self._spec(["lidar", "camera"])
        with pytest.raises(AssemblyError, match="targeted more than once"):
            build_request(spec, {"top": ["lidar/sick", "camera/d435"]})

    def test_socket_both_cleared_and_filled_errors(self):
        from arena_robots.assembly import AssemblyError, build_request

        spec = self._spec()
        with pytest.raises(AssemblyError, match="none"):
            build_request(spec, {"top": ["none", "lidar/sick"]})

    def test_shorthand_none_combined_with_value_errors(self):
        from arena_robots.assembly import AssemblyError, build_request

        spec = self._spec()
        with pytest.raises(AssemblyError, match="none"):
            build_request(spec, {"lidar": ["sick", "none"]})

    def test_build_request_output_feeds_resolve(self):
        from arena_robots.assembly import build_request, resolve

        spec = self._spec()
        request, cleared_sockets, cleared_types = build_request(spec, {"top": ["lidar/sick"]})
        resolved = resolve(spec, request, cleared_sockets=cleared_sockets, cleared_types=cleared_types)
        assert len(resolved.placements) == 1
        assert (resolved.placements[0].variant, resolved.placements[0].mount.name) == ("sick", "top")


class TestChainedMounts:
    """A mount's ``parent`` may be ``"@<mount>:<frame>"``, chaining
    through another mount's placed component's exported frame."""

    def test_parse_chained_parent(self):
        from arena_robots.assembly import parse_chained_parent

        assert parse_chained_parent("@lift0:top") == ("lift0", "top")
        assert parse_chained_parent("base_link") is None

    def test_unchained_mount_has_no_chained_parent(self):
        m = _mount("top", ["lidar"])
        assert m.chained_parent is None

    def test_chained_mount_parses_ref(self):
        m = _mount("arm0", ["arm"], parent="@lift0:top")
        assert m.chained_parent == ("lift0", "top")

    def test_parse_rejects_chained_parent_to_unknown_mount(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec

        with pytest.raises(AssemblyError, match="unknown mount 'lift0'"):
            AssemblySpec.parse(
                {
                    "mounts": {
                        "arm0": {"parent": "@lift0:top", "xyz": [0, 0, 0], "accepts": ["arm"]},
                    }
                }
            )

    def test_parse_rejects_two_mount_cycle(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec

        with pytest.raises(AssemblyError, match="cycle"):
            AssemblySpec.parse(
                {
                    "mounts": {
                        "a": {"parent": "@b:top", "xyz": [0, 0, 0], "accepts": ["x"]},
                        "b": {"parent": "@a:top", "xyz": [0, 0, 0], "accepts": ["y"]},
                    }
                }
            )

    def test_parse_rejects_self_referencing_mount(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec

        with pytest.raises(AssemblyError, match="cycle"):
            AssemblySpec.parse({"mounts": {"a": {"parent": "@a:top", "xyz": [0, 0, 0], "accepts": ["x"]}}})

    def test_parse_accepts_valid_chain(self):
        from arena_robots.assembly import AssemblySpec

        spec = AssemblySpec.parse(
            {
                "mounts": {
                    "lift0": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lift"]},
                    "arm0": {"parent": "@lift0:top", "xyz": [0, 0, 0], "accepts": ["arm"]},
                }
            }
        )
        assert spec.mounts["arm0"].chained_parent == ("lift0", "top")

    def test_empty_chained_mount_is_fine(self):
        """An unpopulated chained mount that nothing is placed on is not an error."""
        from arena_robots.assembly import AssemblySpec, resolve

        spec = AssemblySpec.parse(
            {
                "mounts": {
                    "lift0": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lift"]},
                    "arm0": {"parent": "@lift0:top", "xyz": [0, 0, 0], "accepts": ["arm"]},
                }
            }
        )
        resolved = resolve(spec, {})
        assert resolved.placements == []

    def test_placed_part_stranded_on_unpopulated_chain_raises(self):
        from arena_robots.assembly import AssemblyError, AssemblySpec, RequestPart, resolve

        spec = AssemblySpec.parse(
            {
                "mounts": {
                    "lift0": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lift"]},
                    "arm0": {"parent": "@lift0:top", "xyz": [0, 0, 0], "accepts": ["arm"]},
                }
            }
        )
        with pytest.raises(AssemblyError, match="'arm0' requires 'lift0'"):
            resolve(spec, {"arm": [RequestPart(variant="ur10e")]})

    def test_placed_part_on_populated_chain_succeeds(self):
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec.parse(
            {
                "mounts": {
                    "lift0": {"parent": "base_link", "xyz": [0, 0, 0], "accepts": ["lift"]},
                    "arm0": {"parent": "@lift0:top", "xyz": [0, 0, 0], "accepts": ["arm"]},
                }
            }
        )
        resolved = resolve(spec, {"lift": [RequestPart(variant="ewellix_900mm")], "arm": [RequestPart(variant="ur10e")]})
        by_type = {p.type: p.mount.name for p in resolved.placements}
        assert by_type == {"lift": "lift0", "arm": "arm0"}


class TestResolveOverrides:
    def test_default_overrides_flow_onto_placement(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, resolve

        spec = AssemblySpec(
            mounts={"rear": _mount("rear", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="rear", overrides={"name": "lidar_rear", "topic": "scan/rear"})]},
        )
        resolved = resolve(spec, {})
        assert resolved.placements[0].overrides == {"name": "lidar_rear", "topic": "scan/rear"}

    def test_request_placements_carry_no_overrides(self):
        from arena_robots.assembly import AssemblySpec, RequestPart, resolve

        spec = AssemblySpec(mounts={"top": _mount("top", ["lidar"])})
        resolved = resolve(spec, {"lidar": [RequestPart(variant="sick")]})
        assert resolved.placements[0].overrides == {}


class TestApplyFrameOverrides:
    """apply_frame_overrides (sim2real frames block): a deployment override on the
    mount's ``frame`` wins over both the mount's declared ``frame`` and its addressing
    ``name`` at ``_frame_stem`` time."""

    def test_override_sets_frame_on_matching_placement(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, apply_frame_overrides, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="top")]},
        )
        resolved = resolve(spec, {})
        overridden = apply_frame_overrides(resolved, {"top": "custom_link"})
        assert overridden.placements[0].mount.frame == "custom_link"

    def test_key_naming_no_placed_mount_is_inert(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, apply_frame_overrides, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="top")]},
        )
        resolved = resolve(spec, {})
        overridden = apply_frame_overrides(resolved, {"bogus": "custom_link"})
        assert overridden.placements[0].mount.frame is None

    def test_empty_frames_returns_equivalent_assembly(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, apply_frame_overrides, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="top")]},
        )
        resolved = resolve(spec, {})
        overridden = apply_frame_overrides(resolved, {})
        before = [(p.type, p.variant, p.mount.name) for p in resolved.placements]
        after = [(p.type, p.variant, p.mount.name) for p in overridden.placements]
        assert before == after

    def test_override_wins_over_mount_declared_frame(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, apply_frame_overrides, resolve

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"], frame="declared")},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="top")]},
        )
        resolved = resolve(spec, {})
        overridden = apply_frame_overrides(resolved, {"top": "deploy"})
        assert overridden.placements[0].mount.frame == "deploy"

    def test_frame_stem_reflects_override(self):
        from arena_robots.assembly import AssemblySpec, DefaultPart, apply_frame_overrides, resolve
        from arena_robots.catalog import _frame_stem

        spec = AssemblySpec(
            mounts={"top": _mount("top", ["lidar"])},
            defaults={"lidar": [DefaultPart(variant="sick_s300", mount="top")]},
        )
        resolved = resolve(spec, {})
        overridden = apply_frame_overrides(resolved, {"top": "custom_link"})
        assert _frame_stem(overridden.placements[0].mount) == "custom_link"
