"""Pure, launch-free parametrized-robot assembler. Resolves a robot's typed mounts +
declared defaults against a per-type part request into a concrete placement list via
maximum bipartite matching. No xacro/yaml rendering, no ROS: this module only decides
*what goes where*.
"""

from __future__ import annotations

import re
import typing

import attrs


class AssemblyError(RuntimeError):
    """Infeasible or malformed assembly request. The message is the product: name the
    offending type/mount and the robot's actual inventory, never a generic failure."""


_CHAINED_PARENT_RE = re.compile(r'^@(?P<mount>[^:]+):(?P<frame>.+)$')


def parse_chained_parent(parent: str) -> tuple[str, str] | None:
    """Parse a ``Mount.parent`` of the form ``"@<mount>:<frame>"``:
    a parent chained through another mount's component-exported frame, rather than
    a literal chassis link name. Returns ``(mount_name, frame_name)``, or ``None``
    for an ordinary literal-frame parent."""
    m = _CHAINED_PARENT_RE.match(parent)
    return (m.group('mount'), m.group('frame')) if m else None


@attrs.define
class Mount:
    """A typed attachment slot declared by the chassis."""

    name: str
    parent: str
    xyz: tuple[float, float, float]
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)
    accepts: tuple[str, ...] = attrs.field(factory=tuple)
    """Accepted component types, in declared order: membership validates a placement,
    order is this socket's own preference among its accepted types, consulted only
    under contention."""
    frame: str | None = None
    """Identity stem substituted for ${mount} at render (frame+joint+sensor frame_id);
    None -> falls back to name. Decouples addressing (name) from the sim2real contract."""

    @property
    def chained_parent(self) -> tuple[str, str] | None:
        return parse_chained_parent(self.parent)


@attrs.define
class DefaultPart:
    """One entry of a robot's declared default assembly."""

    variant: str
    mount: str | None = None
    """Pinned socket, or ``None`` to join the unpinned matching pool like a shorthand
    request item (forced onto the sole accepting mount when only one exists)."""
    params: dict[str, object] = attrs.field(factory=dict)
    overrides: dict[str, str] = attrs.field(factory=dict)
    """Per-placement sensor-template overrides (catalog.py), e.g. rbtheron's rear
    lidar ``{name: lidar_rear, topic: scan/rear}``, authored in a robot's own
    ``assembly.yaml defaults:`` and kept by a request pinned to the same mount and variant."""


@attrs.frozen
class PowerSpec:
    """Chassis-intrinsic power model from ``assembly.yaml power:``."""

    compute_core_w: float
    idle_motors_w: float
    drivetrain_efficiency: float
    heating_coefficient_ch: float
    battery_capacity_wh: float
    drivetrain_damping: float = 0.0
    drivetrain_friction: float = 0.0
    rolling_resistance_crr: float = 0.0
    robot_mass_kg: float = 0.0
    wheel_radius_m: float = 0.0
    num_wheels: int = 1

    @classmethod
    def parse(cls, data: dict[str, typing.Any]) -> PowerSpec:
        required = {
            "compute_core_w",
            "idle_motors_w",
            "drivetrain_efficiency",
            "heating_coefficient_ch",
            "battery_capacity_wh",
        }
        fields = {f.name for f in attrs.fields(cls)}
        unknown = set(data) - fields
        if unknown:
            raise AssemblyError(f"assembly.yaml 'power' has unknown keys {sorted(unknown)}; expected {sorted(fields)}")
        missing = required - set(data)
        if missing:
            raise AssemblyError(f"assembly.yaml 'power' is missing keys {sorted(missing)}")
        values = {k: float(v) for k, v in data.items() if k != "num_wheels"}
        return cls(**values, num_wheels=int(data.get("num_wheels", 1)))


@attrs.define
class RequestPart:
    """One requested part instance from a fleet-def morphology item; ``mount`` set only
    for an explicit pin. ``params`` is reserved for later bracket-level tuning and is
    accepted but not propagated: a part pinned to a default's own mount and variant takes
    that default's params and overrides, its own ``overrides`` winning."""

    variant: str
    mount: str | None = None
    params: dict[str, object] = attrs.field(factory=dict)
    overrides: dict[str, str] = attrs.field(factory=dict)


def _validate_mount_dag(mounts: dict[str, Mount]) -> None:
    """Chained-parent references must resolve to declared mounts
    and form a DAG; raises :class:`AssemblyError` naming the unknown mount or the
    cycle otherwise. Purely structural: independent of which mounts end up placed."""
    state: dict[str, int] = {}

    def visit(name: str, path: list[str]) -> None:
        if state.get(name) == 2:
            return
        if state.get(name) == 1:
            cycle = ' -> '.join([*path[path.index(name) :], name])
            raise AssemblyError(f"chained mount parents form a cycle: {cycle}")
        state[name] = 1
        chained = mounts[name].chained_parent
        if chained is not None:
            ref, _frame = chained
            if ref not in mounts:
                raise AssemblyError(f"mount '{name}' parent references unknown mount '{ref}'; declared mounts: {sorted(mounts)}")
            visit(ref, [*path, name])
        state[name] = 2

    for name in mounts:
        visit(name, [])


@attrs.define
class AssemblySpec:
    """Parsed ``assembly.yaml``: mounts + declared defaults."""

    mounts: dict[str, Mount] = attrs.field(factory=dict)
    defaults: dict[str, list[DefaultPart]] = attrs.field(factory=dict)
    power: PowerSpec | None = None
    prefix: str = 'robot_'
    """Frame-templating prefix for ``catalog.render_effective_sensors`` (mount frames
    render as ``${prefix}${mount}_link``). Defaults to ``robot_`` (the Robotnik-family
    convention rbtheron/rbrobout/rbvogui all share); a chassis with no such convention
    (mpo700) declares ``prefix: ""`` here to match its own xacro's ``prefix`` arg default."""

    @classmethod
    def parse(cls, data: dict[str, typing.Any]) -> AssemblySpec:
        """Parse the ``assembly.yaml`` shape (mounts/defaults keys).
        Validates that a pinned default references a declared mount that accepts its
        type; raises :class:`AssemblyError` otherwise. Allocation preference among
        mounts accepting the same type follows mount declaration order (first-declared
        wins); preference among types at one mount follows ``accepts`` declaration
        order."""
        mounts_raw = data.get('mounts', {})
        if not isinstance(mounts_raw, dict):
            raise AssemblyError(f"assembly.yaml 'mounts' must be a mapping; got {type(mounts_raw).__name__}")

        mounts: dict[str, Mount] = {}
        for name, m in mounts_raw.items():
            xyz = m.get('xyz', [0.0, 0.0, 0.0])
            rpy = m.get('rpy', [0.0, 0.0, 0.0])
            frame_raw = m.get('frame')
            mounts[name] = Mount(
                name=str(name),
                parent=str(m['parent']),
                xyz=(float(xyz[0]), float(xyz[1]), float(xyz[2])),
                rpy=(float(rpy[0]), float(rpy[1]), float(rpy[2])),
                accepts=tuple(str(a) for a in m.get('accepts', [])),
                frame=str(frame_raw) if frame_raw is not None else None,
            )
        _validate_mount_dag(mounts)

        defaults: dict[str, list[DefaultPart]] = {}
        for t, entries in data.get('defaults', {}).items():
            parts: list[DefaultPart] = []
            for entry in entries:
                mount_raw = entry.get('mount')
                mount_name = str(mount_raw) if mount_raw is not None else None
                if mount_name is not None:
                    if mount_name not in mounts:
                        raise AssemblyError(f"defaults['{t}']: unknown mount '{mount_name}'; declared mounts: {sorted(mounts)}")
                    if t not in mounts[mount_name].accepts:
                        raise AssemblyError(f"defaults['{t}']: mount '{mount_name}' does not accept '{t}' (accepts: {sorted(mounts[mount_name].accepts)})")
                parts.append(
                    DefaultPart(
                        variant=str(entry['variant']),
                        mount=mount_name,
                        params=dict(entry.get('params', {})),
                        overrides=dict(entry.get('overrides', {})),
                    )
                )
            defaults[str(t)] = parts

        power_raw = data.get('power')
        power = PowerSpec.parse(power_raw) if power_raw is not None else None
        return cls(mounts=mounts, defaults=defaults, power=power, prefix=str(data.get('prefix', 'robot_')))


@attrs.define
class Placement:
    """One resolved (type, variant) instance bound to a concrete mount."""

    type: str
    variant: str
    mount: Mount
    params: dict[str, object] = attrs.field(factory=dict)
    overrides: dict[str, str] = attrs.field(factory=dict)
    """Copied verbatim from the originating ``DefaultPart`` or ``RequestPart``; consumed
    only by ``catalog.render_effective_sensors``."""


@attrs.define
class ResolvedAssembly:
    placements: list[Placement] = attrs.field(factory=list)
    warnings: list[str] = attrs.field(factory=list)


@attrs.define(slots=True)
class _Item:
    """Internal: one part instance flattened out of defaults/request, pre-matching."""

    type: str
    variant: str
    mount: str | None
    params: dict[str, object]
    local_index: int
    overrides: dict[str, str] = attrs.field(factory=dict)


def build_request(spec: AssemblySpec, directives: dict[str, list[str]]) -> tuple[dict[str, list[RequestPart]], frozenset[str], frozenset[str]]:
    """Disambiguate raw setup-grammar directives (``lhs -> raw value strings``) into a
    type-keyed request plus cleared-socket/cleared-type sets (mount-centric addressing).

    LHS is resolved by name against declared mounts first: a declared mount name is
    MOUNT-CENTRIC (``mount=type/variant``, or bare ``mount=variant`` when the socket
    accepts exactly one type); otherwise a known accepted type is SHORTHAND
    (``type=variant``, resolver places one of the type); a name matching both wins as
    a mount (mount beats type on collision). ``/`` is only the type/variant separator,
    consulted only for mount-centric fills. Returns ``(type_keyed_request,
    cleared_sockets, cleared_types)`` for :func:`resolve`.
    """
    known_types = {t for m in spec.mounts.values() for t in m.accepts}
    request: dict[str, list[RequestPart]] = {}
    cleared_sockets: set[str] = set()
    cleared_types: set[str] = set()

    for lhs, values in directives.items():
        if lhs in spec.mounts:
            mount = spec.mounts[lhs]
            has_clear = 'none' in values
            fills = [v for v in values if v != 'none']
            if has_clear and fills:
                raise AssemblyError(f"socket '{lhs}': cannot combine 'none' with a fill value")
            if len(fills) > 1:
                raise AssemblyError(f"socket '{lhs}' targeted more than once")
            if has_clear:
                cleared_sockets.add(lhs)
                continue
            for value in fills:
                if '/' in value:
                    typ, _, variant = value.partition('/')
                    if typ not in mount.accepts:
                        raise AssemblyError(f"socket '{lhs}' does not accept '{typ}' (accepts: {sorted(mount.accepts)})")
                else:
                    if len(mount.accepts) != 1:
                        raise AssemblyError(f"socket '{lhs}' accepts multiple types {sorted(mount.accepts)}; use type/variant")
                    typ = mount.accepts[0]
                    variant = value
                request.setdefault(typ, []).append(RequestPart(variant=variant, mount=lhs))
        elif lhs in known_types:
            has_clear = 'none' in values
            fills = [v for v in values if v != 'none']
            if has_clear and fills:
                raise AssemblyError(f"'{lhs}': 'none' cannot be combined with other value(s) for the same type")
            if has_clear:
                cleared_types.add(lhs)
                continue
            for value in fills:
                request.setdefault(lhs, []).append(RequestPart(variant=value, mount=None))
        else:
            raise AssemblyError(f"unknown '{lhs}': not a mount {sorted(spec.mounts)} nor an accepted type {sorted(known_types)}")

    return request, frozenset(cleared_sockets), frozenset(cleared_types)


def _mounts_accepting(spec: AssemblySpec, t: str) -> list[str]:
    return [m.name for m in spec.mounts.values() if t in m.accepts]


def _candidates(item: _Item, mounts_pool: list[str], mounts: dict[str, Mount]) -> list[str]:
    """Mounts accepting ``item.type``, in mount declaration order (the allocation
    preference: first-declared wins)."""
    return [m for m in mounts_pool if item.type in mounts[m].accepts]


def _match(items: list[_Item], mounts_pool: list[str], mounts: dict[str, Mount]) -> tuple[dict[int, str], list[int]]:
    """Optimal bipartite assignment via backtracking: MAXIMUM cardinality first, then
    minimum summed preference-rank (index of ``item.type`` in the assigned mount's
    ``accepts``), then the lexicographically-earliest mount-declaration-order tuple (in
    item-index order) as the deterministic uniqueness tie-break. Returns (item-index ->
    mount-name assignment, indices left unmatched); a perfect assignment of all
    ``items`` exists iff the returned unmatched list is empty.
    """
    n = len(items)
    mount_index = {name: i for i, name in enumerate(mounts)}
    unmatched_rank = len(mounts)  # sentinel: sorts after every real declaration index
    candidates = [_candidates(item, mounts_pool, mounts) for item in items]

    best: dict[str, typing.Any] = {'key': None, 'assignment': {}, 'unmatched': list(range(n))}

    def recurse(idx: int, used: set[str], assignment: dict[int, str], placed: int, rank_sum: int, order: tuple[int, ...]) -> None:
        if best['key'] is not None and -(placed + (n - idx)) > best['key'][0]:
            return  # even matching everything remaining can't beat the current best's cardinality
        if idx == n:
            key = (-placed, rank_sum, order)
            if best['key'] is None or key < best['key']:
                best['key'] = key
                best['assignment'] = dict(assignment)
                best['unmatched'] = [i for i in range(n) if i not in assignment]
            return

        for mount_name in candidates[idx]:
            if mount_name in used:
                continue
            used.add(mount_name)
            assignment[idx] = mount_name
            rank = mounts[mount_name].accepts.index(items[idx].type)
            recurse(idx + 1, used, assignment, placed + 1, rank_sum + rank, (*order, mount_index[mount_name]))
            del assignment[idx]
            used.remove(mount_name)

        recurse(idx + 1, used, assignment, placed, rank_sum, (*order, unmatched_rank))

    recurse(0, set(), {}, 0, 0, ())
    return best['assignment'], best['unmatched']


def _pinned_item(part_type: str, part: RequestPart, default: DefaultPart | None, index: int) -> _Item:
    if default is None:
        return _Item(type=part_type, variant=part.variant, mount=part.mount, params={}, overrides=part.overrides, local_index=index)
    return _Item(type=part_type, variant=part.variant, mount=part.mount, params=dict(default.params), overrides={**default.overrides, **part.overrides}, local_index=index)


def resolve(
    spec: AssemblySpec,
    request: dict[str, list[RequestPart]],
    *,
    cleared_sockets: frozenset[str] = frozenset(),
    cleared_types: frozenset[str] = frozenset(),
) -> ResolvedAssembly:
    """Resolve a per-type part request against ``spec`` into concrete placements.

    Replace-on-touch: a default part ``d`` is dropped when ANY of: its socket
    is in ``cleared_sockets``; its type is in ``cleared_types``; its socket is filled by
    a mount-centric request item (socket-scoped replace); or its type is named in the
    request by an unpinned (shorthand) item or an empty fill list (type-scoped replace).
    Surviving defaults keep their optional mount, joining the unpinned matching pool
    alongside shorthand request items when ``mount`` is ``None``. Per-type gating: touching a type with no
    accepting mount anywhere is an error, except clearing it, which is a warning.
    Allocation is maximum bipartite matching over ALL parts of ALL types against ALL
    mounts jointly: pins are fixed edges, checked first. ``cleared_sockets``/
    ``cleared_types`` default to empty so callers that already hold a type-keyed
    ``request`` (e.g. a fully-pinned reconstruction, or a direct fleet-def morphology
    dict) keep working unchanged.
    """
    warnings: list[str] = []
    effective: dict[str, list[_Item]] = {}

    socket_touched = {r.mount for items in request.values() for r in items if r.mount is not None}
    type_touched = {t for t, items in request.items() if not items or any(r.mount is None for r in items)}

    for t, default_parts in spec.defaults.items():
        survivors = [p for p in default_parts if p.mount not in cleared_sockets and t not in cleared_types and p.mount not in socket_touched and t not in type_touched]
        if survivors:
            effective.setdefault(t, []).extend(_Item(type=t, variant=p.variant, mount=p.mount, params=p.params, overrides=p.overrides, local_index=i) for i, p in enumerate(survivors))

    for t in cleared_types:
        if t not in request and not _mounts_accepting(spec, t):
            warnings.append(f"'{t}' cleared but robot declares no '{t}' mounts (no-op)")

    for t, requested in request.items():
        mounts_for_type = _mounts_accepting(spec, t)
        if not mounts_for_type:
            if len(requested) == 0:
                warnings.append(f"'{t}' cleared but robot declares no '{t}' mounts (no-op)")
                continue
            inventory = ", ".join(f"{m.name} accepts {sorted(m.accepts)}" for m in spec.mounts.values()) or "(no mounts declared)"
            raise AssemblyError(f"robot declares no '{t}' mounts; mount inventory: {inventory}")
        pinned_defaults: dict[tuple[str | None, str], DefaultPart] = {(d.mount, d.variant): d for d in spec.defaults.get(t, []) if d.mount is not None}
        effective.setdefault(t, []).extend(_pinned_item(t, p, pinned_defaults.get((p.mount, p.variant)), i) for i, p in enumerate(requested))

    flat: list[_Item] = [item for parts in effective.values() for item in parts]

    occupied: dict[str, _Item] = {}
    for item in flat:
        if item.mount is None:
            continue
        if item.mount not in spec.mounts:
            raise AssemblyError(f"unknown mount '{item.mount}' pinned for '{item.type}'; declared mounts: {sorted(spec.mounts)}")
        target = spec.mounts[item.mount]
        if item.type not in target.accepts:
            raise AssemblyError(f"mount '{item.mount}' does not accept '{item.type}' (accepts: {sorted(target.accepts)})")
        other = occupied.get(item.mount)
        if other is not None:
            raise AssemblyError(f"mount '{item.mount}' has two parts pinned to it: '{other.type}={other.variant}' and '{item.type}={item.variant}'")
        occupied[item.mount] = item

    pinned = [item for item in flat if item.mount is not None]
    unpinned = [item for item in flat if item.mount is None]
    mounts_pool = [name for name in spec.mounts if name not in occupied]

    assignment, unmatched = _match(unpinned, mounts_pool, spec.mounts)

    if unmatched:
        by_type: dict[str, list[int]] = {}
        for idx in unmatched:
            by_type.setdefault(unpinned[idx].type, []).append(idx)

        lines: list[str] = []
        for t in by_type:
            n_req = len(effective.get(t, []))
            inv = _mounts_accepting(spec, t)
            plural = "s" if len(inv) != 1 else ""
            lines.append(f"requested {n_req}x {t}, only {len(inv)} {t}-mount{plural} ({', '.join(inv)}); drop one or add a mount")
        for idx in unmatched:
            item = unpinned[idx]
            cands = _candidates(item, mounts_pool, spec.mounts)
            lines.append(f"{item.type}#{item.local_index} candidates: [{', '.join(cands)}]" if cands else f"{item.type}#{item.local_index} candidates: none")
        raise AssemblyError("; ".join(lines))

    unpinned_mounts = iter(assignment[i] for i in range(len(unpinned)))
    placements: list[Placement] = []
    for item in flat:
        mount_name = item.mount if item.mount is not None else next(unpinned_mounts)
        placements.append(
            Placement(
                type=item.type,
                variant=item.variant,
                mount=spec.mounts[mount_name],
                params=dict(item.params),
                overrides=dict(item.overrides),
            )
        )

    placed_mounts = {p.mount.name for p in placements}
    for p in placements:
        chained = p.mount.chained_parent
        if chained is not None:
            ref_mount, _frame = chained
            if ref_mount not in placed_mounts:
                raise AssemblyError(f"'{p.mount.name}' requires '{ref_mount}'")

    return ResolvedAssembly(placements=placements, warnings=warnings)


def apply_frame_overrides(resolved: ResolvedAssembly, frames: dict[str, str]) -> ResolvedAssembly:
    """Bake a per-deployment frame override (mount name -> identity stem) onto a
    resolved assembly (sim2real frames block). Each matching placement's mount
    gets its ``frame`` set to the override, winning over both the mount's declared
    ``frame`` and its addressing ``name``: ``catalog._frame_stem`` then substitutes the
    override for ``${mount}`` in every frame/joint/sensor/controller template. Keys
    naming no placed mount are inert. Returns ``resolved`` unchanged when ``frames`` is
    empty."""
    if not frames:
        return resolved
    placements = [attrs.evolve(p, mount=attrs.evolve(p.mount, frame=frames[p.mount.name])) if p.mount.name in frames else p for p in resolved.placements]
    return ResolvedAssembly(placements=placements, warnings=list(resolved.warnings))


def warn_if_blind(resolved: ResolvedAssembly, required_types: set[str]) -> list[str]:
    """Caller-side helper: the assembler itself never hard-errors on "no
    lidar" (localization is ground-truth in Arena sim), but a bound adapter that needs an
    observation source can ask for a warning here."""
    present = {p.type for p in resolved.placements}
    return [f"no {t} placed: nav2 costmap will run without observation sources" for t in required_types if t not in present]
