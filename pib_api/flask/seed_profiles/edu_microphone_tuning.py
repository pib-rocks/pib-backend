"""Shared microphone defaults for educational hardware profiles."""

MICROPHONE_TUNING = {
    "AGCONOFF": 1,
    "AGCMAXGAIN": 31.6,
    "AGCDESIREDLEVEL": 0.005,
    "AGCTIME": 1.0,
    "STATNOISEONOFF": 1,
    "NONSTATNOISEONOFF": 1,
    "ECHOONOFF": 1,
    "HPFONOFF": 1,
    "STATNOISEONOFF_SR": 1,
    "NONSTATNOISEONOFF_SR": 1,
    # firmware default; the motion guard raises it, a profile does not
    "GAMMAVAD_SR": 3.5,
}

MICROPHONE_LED_RING = {
    "mode": "off",
    "brightness": 16,
    "color": "#000000",
    "vad_led": 0,
}
