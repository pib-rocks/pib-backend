import time
from typing import Dict, Any, Callable, List
from .ros_bridge import start as ros_start, publish_right_pose

def _pose(targets: Dict[str, float]):
    ros_start()
    publish_right_pose(targets)

def gesture_wave(
    *,
    cycles: int = 3,
    amplitude: float = 1.0,
    settle: float = 0.5,
    interval: float = 0.4,
    shoulder: float = 1.3,
    shoulder_swing: float = 0.15,
) -> str:
    base = {
        "shoulder_vertical_right": shoulder,
        "shoulder_horizontal_right": 0.0,
        "elbow_right": 120.0,
        "wrist_right": 0.0,
        "thumb_right_opposition": 30.0,
        "thumb_right_stretch": 80.0,
        "index_right_stretch": 80.0,
        "middle_right_stretch": 80.0,
        "ring_right_stretch": 80.0,
        "pinky_right_stretch": 80.0,
    }

    _pose(base); time.sleep(settle)

    plus = dict(base, **{
        "wrist_right":  +2000.0 * amplitude,
        "elbow_right":   60.0,
        "shoulder_horizontal_right": +400.0 * shoulder_swing,
    })
    minus = dict(base, **{
        "wrist_right":  -2000.0 * amplitude,
        "elbow_right":  130.0,
        "shoulder_horizontal_right": -400.0 * shoulder_swing,
    })

    for _ in range(max(1, cycles)):
        _pose(plus);  time.sleep(interval)
        _pose(minus); time.sleep(interval)

    _pose(base)
    return "ok"

_GESTURES: Dict[str, Callable[..., str]] = {
    "wave": gesture_wave,
}

def available_gestures() -> List[str]:
    return sorted(_GESTURES.keys())

def run_gesture(name: str, **kwargs: Any) -> Any:
    if name not in _GESTURES:
        raise ValueError(f"Unbekannte Geste: {name}. Verfügbar: {', '.join(available_gestures())}")
    return _GESTURES[name](**kwargs)