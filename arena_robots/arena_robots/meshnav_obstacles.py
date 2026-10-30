"""Obstacle points of range sensors classified against the walkable mesh of mesh navigation."""

from __future__ import annotations

import itertools
import math
from pathlib import Path

import attrs
import numpy as np

_PLY_TYPES = {
    "char": "i1",
    "int8": "i1",
    "uchar": "u1",
    "uint8": "u1",
    "short": "i2",
    "int16": "i2",
    "ushort": "u2",
    "uint16": "u2",
    "int": "i4",
    "int32": "i4",
    "uint": "u4",
    "uint32": "u4",
    "float": "f4",
    "float32": "f4",
    "double": "f8",
    "float64": "f8",
}
_EPS = 1e-9
_STEP_TRAVEL = 0.2


def read_ply(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Vertices and triangle faces of a binary little-endian PLY mesh."""
    elements: list[tuple[str, int, list[tuple[str, str, str | None]]]] = []
    form = ""
    with Path(path).open("rb") as file:
        if file.readline().strip() != b"ply":
            raise ValueError(f"{path}: not a PLY file")
        while line := file.readline():
            words = line.decode("ascii").split()
            if not words:
                continue
            if words[0] == "format":
                form = words[1]
            elif words[0] == "element":
                elements.append((words[1], int(words[2]), []))
            elif words[0] == "property" and words[1] == "list":
                elements[-1][2].append((words[4], _PLY_TYPES[words[3]], _PLY_TYPES[words[2]]))
            elif words[0] == "property":
                elements[-1][2].append((words[2], _PLY_TYPES[words[1]], None))
            elif words[0] == "end_header":
                break
        data = file.read()
    if form != "binary_little_endian":
        raise ValueError(f"{path}: PLY format {form!r} is not binary_little_endian")
    vertices, faces = np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64)
    offset = 0
    for name, count, properties in elements:
        fields: list[tuple] = []
        for prop, kind, count_kind in properties:
            fields += [(prop, f"<{kind}")] if count_kind is None else [(f"{prop}__count", f"<{count_kind}"), (prop, f"<{kind}", (3,))]
        dtype = np.dtype(fields)
        block = np.frombuffer(data, dtype=dtype, count=count, offset=offset)
        offset += dtype.itemsize * count
        if name == "vertex":
            vertices = np.stack([block["x"], block["y"], block["z"]], axis=1).astype(np.float64)
        elif name == "face":
            listed = next(prop for prop, _, count_kind in properties if count_kind is not None)
            if np.any(block[f"{listed}__count"] != 3):
                raise ValueError(f"{path}: only triangle faces are supported")
            faces = block[listed].astype(np.int64)
    return vertices, faces


def _surface_samples(vertices: np.ndarray, faces: np.ndarray, spacing: float) -> np.ndarray:
    """The used vertices plus points inside every face whose edges are longer than `spacing`."""
    parts = [vertices[np.unique(faces)]]
    triangles = vertices[faces]
    edges = np.linalg.norm(triangles - np.roll(triangles, 1, axis=1), axis=2).max(axis=1)
    splits = np.maximum(1, np.ceil(edges / spacing - _EPS)).astype(np.int64)
    for n in np.unique(splits[splits > 1]):
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing="ij")
        inner = (i + j <= n) & (i < n) & (j < n) & (i + j > 0)
        weights = np.stack([i[inner], j[inner], n - i[inner] - j[inner]], axis=1) / n
        parts.append(np.einsum("mk,fkd->fmd", weights, triangles[splits == n]).reshape(-1, 3))
    return np.concatenate(parts)


class WalkingSurface:
    """Height samples of a walkable mesh binned into square cells of `resolution` centered on its grid points."""

    def __init__(self, vertices: np.ndarray, faces: np.ndarray, resolution: float) -> None:
        self.resolution = float(resolution)
        faces = np.asarray(faces, dtype=np.int64).reshape(-1, 3)
        samples = _surface_samples(np.asarray(vertices, dtype=np.float64), faces, self.resolution * math.sqrt(2.0)) if len(faces) else np.zeros((1, 3))
        cells = np.rint(samples[:, :2] / self.resolution).astype(np.int64)
        self._origin = cells.min(axis=0) - 1
        self._shape = cells.max(axis=0) - self._origin + 2
        self._floor = float(samples[:, 2].min()) - 1.0
        self._span = float(samples[:, 2].max()) - self._floor + 1.0
        keys, _ = self._keys(cells)
        composite = keys * self._span + self._offset(samples[:, 2])
        order = np.argsort(composite, kind="stable")
        self._sorted = composite[order] if len(faces) else np.full(1, np.inf)
        self._z = samples[order, 2]

    @classmethod
    def from_ply(cls, path: str | Path, resolution: float) -> WalkingSurface:
        """The surface of the mesh in a binary little-endian PLY file."""
        return cls(*read_ply(path), resolution)

    def _keys(self, cells: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        shifted = cells - self._origin
        inside = np.all((shifted >= 0) & (shifted < self._shape), axis=1)
        return (shifted[:, 0] * self._shape[1] + shifted[:, 1]).astype(np.float64), inside

    def _offset(self, z: np.ndarray) -> np.ndarray:
        return np.clip(z - self._floor, 0.0, self._span - 0.5)

    def _highest(self, cells: np.ndarray, z: np.ndarray) -> np.ndarray:
        keys, inside = self._keys(cells)
        base = keys * self._span
        index = np.searchsorted(self._sorted, base + self._offset(z), side="right") - 1
        safe = np.maximum(index, 0)
        return np.where(inside & (index >= 0) & (self._sorted[safe] >= base), self._z[safe], np.nan)

    def _any_between(self, cells: np.ndarray, low: np.ndarray, high: np.ndarray) -> np.ndarray:
        keys, inside = self._keys(cells)
        base = keys * self._span
        index = np.minimum(np.searchsorted(self._sorted, base + self._offset(low), side="left"), len(self._sorted) - 1)
        return inside & (self._sorted[index] >= base + self._offset(low)) & (self._sorted[index] <= base + self._offset(high))

    def _neighbors(self, points: np.ndarray, reach: int = 1) -> list[np.ndarray]:
        cells = np.rint(points[:, :2] / self.resolution).astype(np.int64)
        return [cells + np.array(step) for step in itertools.product(range(-reach, reach + 1), repeat=2)]

    def below(self, points: np.ndarray) -> np.ndarray:
        """Height of the highest surface at or below each point in its cell, else in the cells around, NaN where there is none."""
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        own = self._highest(np.rint(points[:, :2] / self.resolution).astype(np.int64), points[:, 2])
        around = np.full(len(points), np.nan)
        for cells in self._neighbors(points):
            around = np.fmax(around, self._highest(cells, points[:, 2]))
        return np.where(np.isnan(own), around, own)

    def explained(self, points: np.ndarray, below: float, above: float) -> np.ndarray:
        """Whether a surface within a moving stair step's travel of each point lies between `below` under and `above` over it."""
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        hit = np.zeros(len(points), dtype=bool)
        for cells in self._neighbors(points, max(1, round(_STEP_TRAVEL / self.resolution))):
            hit |= self._any_between(cells, points[:, 2] - below, points[:, 2] + above)
        return hit

    def obstacles(self, points: np.ndarray, *, min_height: float, max_height: float, max_step: float) -> np.ndarray:
        """Whether each point stands `min_height` to `max_height` above the surface below it, unexplained by a surface `min_height` under to `max_step` over it."""
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        height = points[:, 2] - self.below(points)
        with np.errstate(invalid="ignore"):
            raised = (height >= min_height) & (height <= max_height)
        raised[raised] = ~self.explained(points[raised], min_height, max_step)
        return raised


@attrs.frozen
class ObstacleFilter:
    """Obstacle points of one scan in the odom frame: finite, in range, off the robot body and standing on the walkable mesh."""

    surface: WalkingSurface
    min_height: float
    max_height: float
    max_step: float
    obstacle_range: float
    self_radius: float

    def __call__(self, points: np.ndarray, sensor_in_base: np.ndarray, base_in_odom: np.ndarray) -> np.ndarray:
        """The obstacles among sensor-frame `points`, given the sensor pose in the base frame and the base pose in odom as 4x4 matrices."""
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        points = points[np.all(np.isfinite(points), axis=1)]
        points = points[np.linalg.norm(points, axis=1) <= self.obstacle_range]
        in_base = points @ sensor_in_base[:3, :3].T + sensor_in_base[:3, 3]
        in_base = in_base[np.hypot(in_base[:, 0], in_base[:, 1]) > self.self_radius]
        in_odom = in_base @ base_in_odom[:3, :3].T + base_in_odom[:3, 3]
        return in_odom[self.surface.obstacles(in_odom, min_height=self.min_height, max_height=self.max_height, max_step=self.max_step)]
