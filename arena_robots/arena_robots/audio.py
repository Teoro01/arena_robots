"""Robot audio contract: microphone arrays, level math, TDOA and audio topic names."""

from __future__ import annotations

import enum
import math
import typing
from collections.abc import Iterable, Mapping, Sequence
from pathlib import Path

import attrs
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_path
from arena_simulation_setup.tree.assets.sound_level import active_rms as active_rms
from arena_simulation_setup.tree.assets.sound_level import dbfs_from_rms as dbfs_from_rms
from arena_simulation_setup.tree.assets.sound_level import rms_from_dbfs as rms_from_dbfs
from numpy.typing import NDArray

type Vec3 = tuple[float, float, float]

NS_PER_S = 1_000_000_000
SPEED_OF_SOUND_MPS = 343.0


def _vec3(value: Iterable[float]) -> Vec3:
    x, y, z = (float(v) for v in value)
    return (x, y, z)


@attrs.frozen(kw_only=True)
class MicSpec:
    name: str
    position_m: Vec3 = attrs.field(converter=_vec3)
    yaw_rad: float = 0.0
    side: typing.Literal["left", "right", "center"] = attrs.field(default="center", validator=attrs.validators.in_(("left", "right", "center")))
    group: typing.Literal["front", "rear", ""] = attrs.field(default="", validator=attrs.validators.in_(("front", "rear", "")))


PRESETS: tuple[str, ...] = ("mono", "stereo", "four_mic")


@attrs.frozen(kw_only=True)
class ArraySpec:
    """Microphone array geometry, positions in the robot mount frame."""

    name: str
    sample_rate_hz: int
    block_size: int
    sensitivity_dbfs_at_94_dbspl: float
    mics: tuple[MicSpec, ...] = attrs.field(converter=tuple)

    @property
    def channels(self) -> int:
        return len(self.mics)

    @property
    def channel_names(self) -> tuple[str, ...]:
        return tuple(mic.name for mic in self.mics)

    @property
    def centroid_m(self) -> Vec3:
        return _vec3(np.mean(np.asarray([mic.position_m for mic in self.mics], dtype=np.float64), axis=0))

    @classmethod
    def from_dict(cls, data: Mapping[str, typing.Any]) -> ArraySpec:
        """Raises ValueError on an invalid layout."""
        if "rectangular" in data:
            mics = rectangular(**{key: float(value) for key, value in data["rectangular"].items()})
        else:
            mics = tuple(
                MicSpec(
                    name=str(mic["name"]),
                    position_m=mic.get("position_m", (0.0, 0.0, 0.0)),
                    yaw_rad=math.radians(float(mic.get("yaw_deg", 0.0))),
                    side=mic.get("side", "center"),
                    group=mic.get("group", ""),
                )
                for mic in data["mics"]
            )
        if not mics:
            raise ValueError("an array needs at least one microphone")
        names = [mic.name for mic in mics]
        if len(set(names)) != len(names):
            raise ValueError(f"duplicate microphone names {names}")
        return cls(
            name=str(data["name"]),
            sample_rate_hz=int(data["sample_rate_hz"]),
            block_size=int(data["block_size"]),
            sensitivity_dbfs_at_94_dbspl=float(data["sensitivity_dbfs_at_94_dbspl"]),
            mics=mics,
        )


def rectangular(*, width_m: float, length_m: float, height_m: float, corner_inset_m: float) -> tuple[MicSpec, ...]:
    """REP-103 four-microphone rectangle in channel order FL FR RL RR."""
    values = (width_m, length_m, height_m, corner_inset_m)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("microphone geometry must be finite")
    if width_m <= 0.0 or length_m <= 0.0 or height_m < 0.0:
        raise ValueError("width/length must be positive and height non-negative")
    if corner_inset_m < 0.0 or 2.0 * corner_inset_m >= min(width_m, length_m):
        raise ValueError("corner inset must leave a positive rectangular aperture")
    x = length_m / 2.0 - corner_inset_m
    y = width_m / 2.0 - corner_inset_m
    return (
        MicSpec(name="front_left", position_m=(x, y, height_m), yaw_rad=math.radians(45.0), side="left", group="front"),
        MicSpec(name="front_right", position_m=(x, -y, height_m), yaw_rad=math.radians(-45.0), side="right", group="front"),
        MicSpec(name="rear_left", position_m=(-x, y, height_m), yaw_rad=math.radians(135.0), side="left", group="rear"),
        MicSpec(name="rear_right", position_m=(-x, -y, height_m), yaw_rad=math.radians(-135.0), side="right", group="rear"),
    )


def presets_dir() -> Path:
    return get_package_share_path("arena_robots") / "config" / "audio" / "arrays"


def load_array_spec(ref: str) -> ArraySpec:
    """A preset name from PRESETS or a yaml path. Raises FileNotFoundError or ValueError."""
    path = presets_dir() / f"{ref}.yaml" if ref in PRESETS else Path(ref).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"array spec {ref!r} is neither a preset {PRESETS} nor a file")
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, Mapping):
        raise ValueError(f"array spec {path} must be a mapping")
    return ArraySpec.from_dict(data)


def transform(mics: Sequence[MicSpec], *, position_m: Vec3, yaw_rad: float) -> tuple[Vec3, ...]:
    """Rigidly transform mount-frame microphone positions into the frame of position_m."""
    cosine, sine = math.cos(yaw_rad), math.sin(yaw_rad)
    rx, ry, rz = position_m
    return tuple(
        (
            rx + cosine * mic.position_m[0] - sine * mic.position_m[1],
            ry + sine * mic.position_m[0] + cosine * mic.position_m[1],
            rz + mic.position_m[2],
        )
        for mic in mics
    )


def geometric_delays_s(source_m: Vec3, mics_m: Sequence[Vec3], *, speed_of_sound_mps: float = SPEED_OF_SOUND_MPS) -> NDArray[np.float64]:
    if speed_of_sound_mps <= 0.0 or not math.isfinite(speed_of_sound_mps):
        raise ValueError("speed of sound must be finite and positive")
    source = np.asarray(source_m, dtype=np.float64)
    positions = np.asarray(mics_m, dtype=np.float64)
    return np.linalg.norm(positions - source[None, :], axis=1) / speed_of_sound_mps


REFERENCE_SPL_DB = 94.0


def rms(samples: NDArray[np.floating], *, axis: int | None = None) -> NDArray[np.float64] | float:
    values = np.asarray(samples, dtype=np.float64)
    return np.sqrt(np.mean(values * values, axis=axis)) if values.size else 0.0


def spl_to_dbfs(spl_db: float, sensitivity_dbfs_at_94_dbspl: float) -> float:
    """Digital level of a sound pressure level through a fixed MEMS sensitivity."""
    return float(spl_db) - REFERENCE_SPL_DB + float(sensitivity_dbfs_at_94_dbspl)


def dbfs_to_spl(level_dbfs: float, sensitivity_dbfs_at_94_dbspl: float) -> float:
    return float(level_dbfs) + REFERENCE_SPL_DB - float(sensitivity_dbfs_at_94_dbspl)


def gcc_phat(
    signal: NDArray[np.floating],
    reference: NDArray[np.floating],
    *,
    sample_rate_hz: int,
    max_tau_s: float | None = None,
    interpolation: int = 8,
) -> tuple[float, float]:
    """Signal-minus-reference delay in seconds and normalized peak confidence."""
    sig = np.asarray(signal, dtype=np.float64)
    ref = np.asarray(reference, dtype=np.float64)
    if sig.size == 0 or ref.size == 0 or sample_rate_hz <= 0:
        return 0.0, 0.0
    n = sig.size + ref.size
    spectrum = np.fft.rfft(sig, n=n) * np.conj(np.fft.rfft(ref, n=n))
    magnitude = np.abs(spectrum)
    spectrum /= np.maximum(magnitude, 1e-15)
    correlation = np.fft.irfft(spectrum, n=interpolation * n)
    maximum_shift = interpolation * n // 2
    if max_tau_s is not None:
        maximum_shift = min(
            maximum_shift,
            int(interpolation * sample_rate_hz * max_tau_s),
        )
    correlation = np.concatenate((correlation[-maximum_shift:], correlation[: maximum_shift + 1]))
    peak_index = int(np.argmax(np.abs(correlation)))
    shift = peak_index - maximum_shift
    confidence = float(np.abs(correlation[peak_index]))
    return shift / float(interpolation * sample_rate_hz), confidence


class ArrayStream(enum.StrEnum):
    RAW = "raw_array"
    STEM_MOTOR = "stem_motor"
    STEM_PEDESTRIAN = "stem_pedestrian"
    STEM_AMBIENT = "stem_ambient"
    MONITOR = "headphones/stereo"
    HEARING_MONO = "hearing/mono"
    ENERGY = "hearing/energy"
    TDOA = "diagnostics/tdoa"
    RENDER_INPUTS = "diagnostics/render_inputs"
    ACTIVITY = "rendered_sound_activity"
    LEVELS = "diagnostics/levels"


def array_stream(robot: str, stream: ArrayStream) -> str:
    return f"{robot}/audio/{stream}"
