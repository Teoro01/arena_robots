from __future__ import annotations

import copy
import itertools
import typing
from collections.abc import Sequence
from pathlib import Path

import attrs
import yaml
from arena_simulation_setup.tree import Identifier, ResolverBase

from arena_robots import ARENA_ROBOTS_DIR

# Config fields a morphology directive must not shadow.
_RESERVED_TYPES = frozenset({'robot', 'name', 'count', 'parts', 'frames', 'adapters', 'extra'})
# Bare keys routed straight to extra (identity-lane, consumed by Robot.parse).
_EXTRA_KEYS = frozenset({'pos'})


@attrs.define()
class Config:
    """Configuration for setting up a robot instance."""

    robot: str  # name of robot
    name: str | None = None  # name or name prefix
    parts: dict[str, list[str]] = attrs.field(factory=dict)  # lhs (mount or type) -> raw value strings
    frames: dict[str, str] = attrs.field(factory=dict)  # mount -> frame stem override
    adapters: dict[str, str] = attrs.field(factory=dict)  # cap -> adapter kind
    extra: dict[str, typing.Any] = attrs.field(factory=dict)  # extra arbitrary data

    @classmethod
    def parse(cls, data: str | dict[str, typing.Any]) -> Sequence[Config]:
        """Parse a configuration from the given data.

        Dict keys route by grammar, not a registry: ``robot``/``name`` are fields,
        ``count`` expands instances, ``pos`` is an identity extra,
        ``adapters`` (dict value) and ``<cap>.adapter`` set adapter kinds, ``frames``
        (dict value) sets mount frame-stem overrides, any other dotted key is rejected
        (the dot is the adapter lane), and remaining bare keys are morphology
        directives (``mount=type/variant``, ``mount=variant``, ``type=variant``,
        ``lhs=none``). Directives are grammar-collected here as raw strings only;
        mount-vs-type disambiguation and realization happen later, per-robot, at
        ``Robot.parse``/``assembly.build_request`` resolution time.
        """
        if isinstance(data, str):
            return (cls(robot=data, name=data),)

        count = data.get('count', 1)
        fields: dict[str, typing.Any] = {}
        extra: dict[str, typing.Any] = {}
        adapters: dict[str, str] = {}
        frames: dict[str, str] = {}
        parts: dict[str, list[str]] = {}

        for k, v in data.items():
            if k == 'count':
                continue
            if k in ('robot', 'name'):
                fields[k] = v
                continue
            if k in _EXTRA_KEYS:
                extra[k] = v
                continue
            if k == 'adapters':
                if not isinstance(v, dict):
                    raise RuntimeError(f"'adapters': expected a mapping of cap -> adapter kind, got {v!r}")
                adapters.update({str(ck): str(cv) for ck, cv in v.items()})
                continue
            if k == 'frames':
                if not isinstance(v, dict):
                    raise RuntimeError(f"'frames': expected a mapping of mount -> frame, got {v!r}")
                frames.update({str(ck): str(cv) for ck, cv in v.items()})
                continue
            if k == 'extra':
                if not isinstance(v, dict):
                    raise RuntimeError(f"'extra': expected a mapping, got {v!r}")
                extra.update(v)
                continue
            if '.' in k:
                cap, _, tail = k.partition('.')
                if tail != 'adapter':
                    raise RuntimeError(f"'{k}': the only dotted key is '<cap>.adapter=<kind>'; tuning belongs on the robot.planner:= / task-config channel")
                adapters[cap] = str(v)
                continue

            # remaining bare keys are morphology directives, collected as raw values
            if k in _RESERVED_TYPES:
                raise RuntimeError(f"'{k}' is a reserved Config field name; cannot be used as a morphology key")
            parts[k] = [str(x) for x in v] if isinstance(v, list) else [str(v)]

        fields['parts'] = parts
        fields['frames'] = frames
        fields['adapters'] = adapters
        if extra:
            fields['extra'] = extra
        return tuple(cls(**copy.deepcopy(fields)) for _ in range(count))


class RobotSetupResolver(ResolverBase):
    base_path = ARENA_ROBOTS_DIR / 'config' / 'setup'

    async def resolve(self, identifier: RobotSetupIdentifier) -> Path | None:
        target_path = self.base_path / f'{identifier.name}.yaml'
        if target_path.exists():
            return target_path
        return None


class RobotSetupIdentifier(Identifier[list[Config]]):
    def load(self, path: Path, /, **kwargs: object) -> list[Config]:
        del kwargs  # unused
        with open(path) as f:
            configuration = yaml.safe_load(f)

        if not isinstance(configuration, list):
            raise ValueError(f"{path}: robot_setup.yaml must be a list")

        return list(itertools.chain.from_iterable(map(Config.parse, configuration)))
