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

### `AGCTIME` is read back as a coefficient, not as seconds

`AGCTIME` is written in seconds, but the register holds the one-pole
coefficient of that ramp at the processing block rate of the device:

```
coefficient = exp(-1 / (62.5 Hz * seconds))      62.5 Hz = 16000 / 256
```

Measured on 192.168.1.172 with a ReSpeaker attached:

| write | model coefficient | read-back | residual | read-back in seconds |
| --- | --- | --- | --- | --- |
| `AGCTIME 1.0` | `0.9841273201` | `0.9841422392055392` | `1.49e-05` | 1.00095 s |
| `AGCTIME 0.5` | `0.9685065821` | `0.9685218567028642` | `1.53e-05` | 0.50025 s |

Both values follow the model, so neither is a value the device refused.
`microphone_parameters.py` converts every read-back into the unit of its
parameter (`parameter_from_readback`) before it is compared or reported, so
comparison, the ROS parameters and the UI all speak seconds.

Converting the 1.5e-05 residual into seconds amplifies it to at most 9.6e-04 s
at the top of the range, which is what the absolute epsilon
`AGCTIME_READBACK_EPSILON_SECONDS` (2e-03 s) covers. Apart from that epsilon
every parameter keeps the strict default tolerance of `1e-5` relative /
`1e-8` absolute, and a write the device cannot store is still rejected with the
parameter name, the requested seconds and the value the device reported. Widen
a tolerance only with a measured read-back next to it.

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
