# ros_audio_io

Audio nodes for pib: raw audio streaming, output volume, and the ReSpeaker Mic
Array v2.0 (XVF3000) control surface. The `doa_publisher` node owns the USB
device: it publishes `/doa_angle`, `/voice_activity` and `/speech_detected`, and
exposes the tuning registers, the presets and the LED ring as ROS parameters.
No other process may open the microphone array.

## Setting microphone parameters

```bash
source /opt/ros/jazzy/setup.bash
source /app/ros2_ws/install/setup.bash
ros2 param list /doa_publisher
ros2 param set /doa_publisher HPFONOFF 2
ros2 param set /doa_publisher preset 'Noisy Environment / ASR'
```

Every tuning write is read back from the device and verified. When a read-back
leaves the tolerance of its parameter, the whole parameter set is rejected, the
node logs the parameter with the requested and the actual value, and the ROS
parameters are re-synchronised with what the device really holds.

### Read-back tolerances

The XVF3000 does not return every float exactly as it was written, so
`microphone_parameters.py` keeps a documented per-parameter tolerance
(`READBACK_TOLERANCES`, relative plus absolute) next to `PARAMETER_SPECS`.
Measured on 192.168.1.172 with a ReSpeaker attached:

| write | read-back | deviation |
| --- | --- | --- |
| `AGCTIME 1.0` | `0.9841422392055392` | 1.59 % |
| `AGCTIME 0.5` | `0.9685218567028642` | 93.7 % |

`AGCTIME` therefore gets a 2 % relative tolerance, everything else keeps the
strict default of `1e-5` relative / `1e-8` absolute. The 0.5 s case is not
rounding: the device does not hold that value, so the
"Noisy Environment / ASR" preset asks for 1.0 s instead (AGC is off in that
preset, which makes the ramp time-constant inert there). Add a tolerance only
with a measured read-back next to it.

## Trap: `ros2 param set <node> led_mode off` fails

```bash
$ ros2 param set /doa_publisher led_mode off
Setting parameter failed: Wrong parameter type, expected 'Type.STRING' got 'Type.BOOL'
```

The CLI parses the value as YAML, and YAML 1.1 reads bare `off` (like `on`,
`no`, `yes`, `true`, `false`) as a boolean, so the node is handed a bool where
it declared a string. Force the string:

```bash
ros2 param set /doa_publisher led_mode '!!str off'
```

The quotes are for the shell, `!!str` is the YAML tag that keeps the value a
string. Other LED modes (`listen`, `speak`, `think`, `spin`, `trace`, `mono`)
are unaffected. Clients that speak rosbridge send typed JSON and are not
affected at all; this is a CLI-only trap.
