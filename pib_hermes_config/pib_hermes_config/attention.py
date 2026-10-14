"""Attention and presence before an answer (concept section 6.4).

The head turn is geometry: a detected person or frontal face, and the
microphone array's direction of arrival when several people are in view.
It does not ask a language model where to look.

Person re-identification is the 256-d embedding from
person-reidentification-retail-0031 (cosine distance, as the model card
states). A match is addressed by name and read from that person's own
memory file. Personality MEMORY.md is not read or written here.

The camera pipeline does not yet publish those embeddings. Matching runs
when a detection carries the vector; without one, nobody is given a name.

A frontal face that stays put is looking at the robot. A face or body
that crosses the frame is walking past. Only a recognised person who is
looking is greeted, and not again until the cooldown has elapsed.
"""

from __future__ import annotations

import json
import logging
import math
import os
import re
import tempfile
import threading
from dataclasses import dataclass

from pib_hermes_config import align_profile_ownership, profiles_dir

logger = logging.getLogger(__name__)

# OAK-D Lite colour camera, the preview this repository publishes (1280x720).
# Horizontal field of view is the Luxonis figure for that sensor. Motor
# positions are centidegrees, the same scale Write.move uses (15 degrees
# is 1500). turn_head_motor is the yaw joint; its seeded range is ±90 degrees.
COLOR_FRAME_WIDTH = 1280
COLOR_FRAME_HEIGHT = 720
COLOR_HFOV_DEG = 69.0
HEAD_MOTOR = "turn_head_motor"
HEAD_POSITION_MIN = -9000
HEAD_POSITION_MAX = 9000

# person-reidentification-retail-0031: 256-d embedding, cosine distance.
REID_DIMENSION = 256
MAX_COSINE_DISTANCE = 0.5

# A face that drifts slower than this is holding still. Faster, it is crossing.
PASSING_SPEED_PX_PER_S = 200.0
MOTION_WINDOW_SECONDS = 1.0
STABLE_LOOK_SECONDS = 0.4
# Head-pose or gaze yaw, degrees. Near zero means the face is toward the camera.
LOOK_YAW_DEG = 25.0

# Long enough that a person met at an event is not greeted on every pass.
GREETING_COOLDOWN_SECONDS = 600.0

AUDIENCE_LOOKING = "looking"
AUDIENCE_PASSING = "passing"
AUDIENCE_NONE = "none"

# Sound from behind the camera is not a person in the picture.
_BEHIND_DEG = 90.0

_SLUG = re.compile(r"[^a-z0-9_-]")
_lock = threading.Lock()


@dataclass(frozen=True)
class Body:
    """One person or face in a camera frame. Coordinates are pixels."""

    kind: str
    x_min: float
    y_min: float
    x_max: float
    y_max: float
    frame_width: int
    frame_height: int
    score: float = 1.0
    embedding: tuple[float, ...] | None = None
    gaze_yaw: float | None = None

    @property
    def cx(self) -> float:
        return (float(self.x_min) + float(self.x_max)) / 2.0

    @property
    def area(self) -> float:
        width = max(0.0, float(self.x_max) - float(self.x_min))
        height = max(0.0, float(self.y_max) - float(self.y_min))
        return width * height


@dataclass(frozen=True)
class AttentionPlan:
    """What to do before the model speaks. ``preface`` is for this turn only."""

    head_motor: str | None = None
    head_position: int | None = None
    person_id: str | None = None
    name: str | None = None
    audience: str = AUDIENCE_NONE
    greeting: str | None = None
    preface: str = ""
    now: float = 0.0

    def for_hermes(self, content: str) -> str:
        """User text for a Smart turn. The stored chat line is not this string."""
        if not self.preface:
            return content
        return self.preface + "\n\n" + content

    def for_direct(self, system_prompt: str) -> str:
        """System prompt for a Direct turn. SOUL.md is not rewritten."""
        if not self.preface:
            return system_prompt
        return system_prompt.rstrip() + "\n\n" + self.preface


class SceneBuffer:
    """Latest camera and microphone observations. Safe to update from ROS threads."""

    def __init__(
        self,
        frame_width: int = COLOR_FRAME_WIDTH,
        frame_height: int = COLOR_FRAME_HEIGHT,
    ) -> None:
        self.frame_width = int(frame_width)
        self.frame_height = int(frame_height)
        self.doa_deg: int | None = None
        self._sources: dict[str, list[Body]] = {}
        self._haar_face: Body | None = None
        self.samples: list[tuple[float, float | None]] = []
        self._speech = False
        self._lock = threading.Lock()

    def note_doa(self, angle: int | None) -> None:
        with self._lock:
            if angle is None:
                self.doa_deg = None
                return
            self.doa_deg = int(angle) % 360

    def note_speech(self, active: bool) -> bool:
        """Return True on the rising edge, when a speaker has just started."""
        with self._lock:
            started = bool(active) and not self._speech
            self._speech = bool(active)
            return started

    def note_face_center(self, x: float, y: float, now: float) -> None:
        """Haar frontal-face offset published on ``face_center``.

        The camera publishes ``[0, 0]`` when it finds no face. That sentinel
        is not a face at the centre of the frame.
        """
        with self._lock:
            if float(x) == 0.0 and float(y) == 0.0:
                self._haar_face = None
            else:
                self._haar_face = _face_at_offset(
                    float(x),
                    float(y),
                    self.frame_width,
                    self.frame_height,
                )
            self._observe_locked(now)

    def note_bodies(
        self,
        source: str,
        bodies: list[Body],
        now: float,
        frame_width: int | None = None,
        frame_height: int | None = None,
    ) -> None:
        with self._lock:
            if frame_width:
                self.frame_width = int(frame_width)
            if frame_height:
                self.frame_height = int(frame_height)
            self._sources[str(source)] = list(bodies)
            self._observe_locked(now)

    def snapshot(
        self,
    ) -> tuple[list[Body], int | None, list[tuple[float, float | None]]]:
        with self._lock:
            return (
                list(self._visible_locked()),
                self.doa_deg,
                list(self.samples),
            )

    def _visible_locked(self) -> list[Body]:
        bodies: list[Body] = []
        for group in self._sources.values():
            bodies.extend(group)
        if bodies:
            return bodies
        if self._haar_face is not None:
            return [self._haar_face]
        return []

    def _observe_locked(self, now: float) -> None:
        speaker = select_speaker(self._visible_locked(), self.doa_deg)
        center = None if speaker is None else speaker.cx
        self.samples.append((float(now), center))
        cutoff = float(now) - 3.0
        self.samples = [(stamp, x) for stamp, x in self.samples if stamp >= cutoff]


def assess(buffer: SceneBuffer, now: float) -> AttentionPlan:
    """Decide the head target, who is speaking, and whether a greeting is due.

    Does not write the cooldown. The caller records a greeting it actually uses.
    """
    bodies, doa_deg, samples = buffer.snapshot()
    speaker = select_speaker(bodies, doa_deg)
    audience = audience_of(
        samples,
        now,
        None if speaker is None else speaker.gaze_yaw,
    )
    matched = None
    if speaker is not None and speaker.embedding is not None:
        matched = match_person(speaker.embedding)
    name = None if matched is None else str(matched["name"])
    person_id = None if matched is None else str(matched["person_id"])
    memory = "" if person_id is None else read_person_memory(person_id)
    already = False
    if person_id is not None:
        already = greeted_recently(matched, now)
    greeting = None
    if audience == AUDIENCE_LOOKING and name and not already:
        greeting = greeting_line(name, None)
    return AttentionPlan(
        head_motor=None if speaker is None else HEAD_MOTOR,
        head_position=None if speaker is None else head_position(speaker),
        person_id=person_id,
        name=name,
        audience=audience,
        greeting=greeting,
        preface=_preface(name, memory, audience, already),
        now=float(now),
    )


def select_speaker(bodies: list[Body], doa_deg: int | None) -> Body | None:
    """The person the robot should face.

    Faces win over body boxes. With several candidates, direction of arrival
    picks the one whose image bearing is closest. Zero degrees is straight
    ahead along the camera axis; positive angles are toward image right
    (the viewer's right; the colour image is not mirrored). Sound from
    behind the camera does not select a person in front of it.
    """
    faces = [body for body in bodies if body.kind == "face"]
    people = [body for body in bodies if body.kind == "person"]
    candidates = faces or people
    if not candidates:
        return None
    if doa_deg is not None and abs(_signed_doa(doa_deg)) > _BEHIND_DEG:
        return None
    if doa_deg is None or len(candidates) == 1:
        return max(candidates, key=lambda body: (body.area, body.score))
    signed = _signed_doa(doa_deg)
    return min(candidates, key=lambda body: abs(_bearing_deg(body) - signed))


def head_position(body: Body) -> int:
    """``turn_head_motor`` centidegrees that bring this body toward the centre."""
    half = float(body.frame_width) / 2.0
    if half <= 0.0:
        return 0
    offset = body.cx - half
    degrees = (offset / half) * (COLOR_HFOV_DEG / 2.0)
    centidegrees = int(round(degrees * 100.0))
    return max(HEAD_POSITION_MIN, min(HEAD_POSITION_MAX, centidegrees))


def audience_of(
    samples: list[tuple[float, float | None]],
    now: float,
    gaze_yaw: float | None,
) -> str:
    """``looking`` when a face holds still toward the camera, else ``passing`` or neither."""
    recent = [
        (stamp, center)
        for stamp, center in samples
        if center is not None
        and 0.0 <= float(now) - float(stamp) <= MOTION_WINDOW_SECONDS
    ]
    recent.sort(key=lambda item: float(item[0]))
    if len(recent) < 2:
        return AUDIENCE_NONE
    span = float(recent[-1][0]) - float(recent[0][0])
    if span < STABLE_LOOK_SECONDS:
        return AUDIENCE_NONE
    speed = abs(float(recent[-1][1]) - float(recent[0][1])) / span
    if speed >= PASSING_SPEED_PX_PER_S:
        return AUDIENCE_PASSING
    if gaze_yaw is not None and abs(float(gaze_yaw)) > LOOK_YAW_DEG:
        return AUDIENCE_NONE
    return AUDIENCE_LOOKING


def body_from_detection(detection, frame_width: int, frame_height: int) -> Body | None:
    """A person or face box from a ``Detection`` message.

    The re-identification model is not on the camera pipeline, so this does
    not invent an embedding from unrelated scalars. ``gaze_yaw`` (or head-pose
    ``yaw``) is copied when that model published it.
    """
    label = str(getattr(detection, "label", "")).strip().lower()
    if label == "face":
        kind = "face"
    elif label == "person":
        kind = "person"
    else:
        return None
    names = [str(name) for name in (getattr(detection, "scalar_names", []) or [])]
    values = list(getattr(detection, "scalar_values", []) or [])
    gaze = _named_scalar(names, values, "gaze_yaw")
    if gaze is None:
        gaze = _named_scalar(names, values, "yaw")
    width = int(frame_width) if frame_width else COLOR_FRAME_WIDTH
    height = int(frame_height) if frame_height else COLOR_FRAME_HEIGHT
    return Body(
        kind=kind,
        x_min=float(detection.x_min),
        y_min=float(detection.y_min),
        x_max=float(detection.x_max),
        y_max=float(detection.y_max),
        frame_width=width,
        frame_height=height,
        score=float(getattr(detection, "score", 1.0) or 0.0),
        gaze_yaw=gaze,
    )


def greeting_line(name: str, language: str | None) -> str:
    """One spoken greeting. The personality's language is German unless it says English."""
    if isinstance(language, str) and language.strip().lower().startswith("en"):
        return f"Hello {name}."
    return f"Hallo {name}."


def enroll_person(name: str, embedding: list[float] | tuple[float, ...]) -> dict:
    """Store a named embedding. An existing slug keeps its memory file."""
    cleaned = _clean_name(name)
    vector = _validate_embedding(embedding)
    person_id = _person_id(cleaned)
    path = _identity_path(person_id)
    with _lock:
        previous = _read_json(path)
        greeted = None if previous is None else previous.get("last_greeted_at")
        record = {
            "person_id": person_id,
            "name": cleaned,
            "embedding": vector,
            "last_greeted_at": greeted,
        }
        _write_json(path, record)
    _own(os.path.dirname(path))
    return record


def match_person(embedding: list[float] | tuple[float, ...]) -> dict | None:
    """The enrolled person within the cosine-distance limit, or None."""
    vector = _validate_embedding(embedding)
    best: dict | None = None
    best_distance = MAX_COSINE_DISTANCE
    root = people_root()
    if not os.path.isdir(root):
        return None
    with _lock:
        names = list(os.listdir(root))
    for name in names:
        record = _read_json(os.path.join(root, name, "identity.json"))
        if record is None:
            continue
        stored = record.get("embedding")
        if not isinstance(stored, list) or len(stored) != REID_DIMENSION:
            continue
        try:
            distance = cosine_distance(vector, stored)
        except ValueError:
            continue
        if distance <= best_distance:
            best_distance = distance
            best = record
    return best


def cosine_distance(
    left: list[float] | tuple[float, ...],
    right: list[float] | tuple[float, ...] | list,
) -> float:
    """``1 - cosine similarity``. Zero is the same direction."""
    if len(left) != len(right):
        raise ValueError("embeddings differ in length")
    dot = 0.0
    left_norm = 0.0
    right_norm = 0.0
    for a, b in zip(left, right):
        fa = float(a)
        fb = float(b)
        dot += fa * fb
        left_norm += fa * fa
        right_norm += fb * fb
    if left_norm <= 0.0 or right_norm <= 0.0:
        raise ValueError("embedding has no direction")
    similarity = dot / math.sqrt(left_norm * right_norm)
    similarity = max(-1.0, min(1.0, similarity))
    return 1.0 - similarity


def read_person_memory(person_id: str) -> str:
    path = _memory_path(_safe_id(person_id))
    try:
        with open(path, encoding="utf-8") as handle:
            return handle.read()
    except FileNotFoundError:
        return ""
    except OSError as exc:
        logger.warning("could not read %s: %s", path, exc)
        return ""


def write_person_memory(person_id: str, text: str) -> None:
    """Replace one person's memory. Does not touch MEMORY.md or SOUL.md."""
    safe = _safe_id(person_id)
    identity = _identity_path(safe)
    if not os.path.isfile(identity):
        raise ValueError(f"unknown person {safe}")
    path = _memory_path(safe)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    _atomic_write(path, text)
    _own(os.path.dirname(path))


def greeted_recently(record: dict | None, now: float) -> bool:
    if not record:
        return False
    stamp = record.get("last_greeted_at")
    if stamp is None:
        return False
    try:
        elapsed = float(now) - float(stamp)
    except (TypeError, ValueError):
        return False
    return 0.0 <= elapsed < GREETING_COOLDOWN_SECONDS


def remember_greeting(person_id: str, now: float) -> None:
    """Start the cooldown for this person. Other people's stamps stay put."""
    safe = _safe_id(person_id)
    path = _identity_path(safe)
    with _lock:
        record = _read_json(path)
        if record is None:
            return
        record["last_greeted_at"] = float(now)
        _write_json(path, record)


def people_root() -> str:
    """Directory of per-person files, beside the Hermes profiles directory."""
    parent = os.path.dirname(profiles_dir().rstrip(os.sep))
    return os.path.join(parent, "people")


def _preface(name: str | None, memory: str, audience: str, already: bool) -> str:
    parts: list[str] = []
    if name:
        parts.append(f"You are speaking with {name}. Address them by that name.")
        text = memory.strip()
        if text:
            parts.append(
                "What you remember about this person, and no one else:\n" + text
            )
        else:
            parts.append(
                "You have no stored memory for this person. "
                "Do not mix in another person's memory."
            )
    if audience == AUDIENCE_PASSING:
        parts.append("They are walking past, not looking at you. Do not greet them.")
    elif already and name:
        parts.append("You already greeted them recently. Do not greet them again.")
    return "\n".join(parts)


def _face_at_offset(x: float, y: float, frame_width: int, frame_height: int) -> Body:
    # face_center y is (frame_height / 2) - face_y, so up is positive.
    cx = (frame_width / 2.0) + x
    cy = (frame_height / 2.0) - y
    half = 20.0
    return Body(
        kind="face",
        x_min=cx - half,
        y_min=cy - half,
        x_max=cx + half,
        y_max=cy + half,
        frame_width=frame_width,
        frame_height=frame_height,
    )


def _named_scalar(names: list[str], values: list, wanted: str) -> float | None:
    for name, value in zip(names, values):
        if name != wanted:
            continue
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(number):
            return None
        return number
    return None


def _bearing_deg(body: Body) -> float:
    half = float(body.frame_width) / 2.0
    if half <= 0.0:
        return 0.0
    offset = body.cx - half
    return (offset / half) * (COLOR_HFOV_DEG / 2.0)


def _signed_doa(angle: int) -> float:
    signed = int(angle) % 360
    if signed > 180:
        signed -= 360
    return float(signed)


def _clean_name(name: object) -> str:
    text = " ".join(str(name).split())
    if not text:
        raise ValueError("a person needs a name")
    return text


def _person_id(name: str) -> str:
    slug = _SLUG.sub("", name.lower().replace(" ", "_"))
    if not slug:
        raise ValueError("a person needs a name")
    return slug


def _safe_id(person_id: str) -> str:
    slug = _SLUG.sub("", str(person_id).lower())
    if not slug or slug != str(person_id).lower():
        raise ValueError("unknown person")
    return slug


def _validate_embedding(embedding: list[float] | tuple[float, ...]) -> list[float]:
    values = [float(item) for item in embedding]
    if len(values) != REID_DIMENSION:
        raise ValueError(
            f"embedding has {len(values)} values, expected {REID_DIMENSION}"
        )
    if not all(math.isfinite(item) for item in values):
        raise ValueError("embedding contains a non-finite value")
    if sum(item * item for item in values) <= 0.0:
        raise ValueError("embedding has no direction")
    return values


def _identity_path(person_id: str) -> str:
    return os.path.join(people_root(), person_id, "identity.json")


def _memory_path(person_id: str) -> str:
    return os.path.join(people_root(), person_id, "memory.md")


def _read_json(path: str) -> dict | None:
    try:
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
    except FileNotFoundError:
        return None
    except (OSError, json.JSONDecodeError) as exc:
        logger.warning("could not read %s: %s", path, exc)
        return None
    if not isinstance(data, dict):
        return None
    return data


def _write_json(path: str, record: dict) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    _atomic_write(path, json.dumps(record))


def _atomic_write(path: str, text: str) -> None:
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(dir=directory, prefix=".tmp-")
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(text)
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _own(directory: str) -> None:
    try:
        align_profile_ownership(directory)
    except Exception:
        logger.debug("could not align ownership of %s", directory, exc_info=True)
