# PR-1516 — audio_recorder ROS node crashes in ros-voice-assistant container

Jira: https://pib-rocks.atlassian.net/browse/PR-1516
Repo: `pib-backend` (ros_packages/voice_assistant)
Target: Raspberry Pi 5 (192.168.1.28)

## Current state

- Container `multirepo-ros-voice-assistant-1` runs image built **2026-08-01T15:52:56Z**
- `audio_recorder` node crashes on startup:
  - `ioctl( devHandle, SNDCTL_DSP_CHANNELS, &numChannels )` failed in `pa_unix_oss.c`
  - `TypeError: 'NoneType' object does not support the context manager protocol`
  - `AttributeError: 'MultiThreadedExecutor' object has no attribute '_sigint_gc'`
- Result: `/send_chat_message` and `/set_voice_assistant_state` ROS services NOT advertised
- E2E test `test_voice_assistant_hermes_persists_reply_and_recalls_prior_fact` times out

## Step 1 — Rebuild container on Pi 5 (do this FIRST)

The image predates the Hermes fixes (PR-1528/1535: hermes-agent baked in, Gemini keys, timeouts). A fresh build may already resolve the audio_recorder crash.

On the Pi 5 (192.168.1.28):
```bash
cd /home/pib/app/pib-backend
git pull origin develop
docker compose build ros-voice-assistant
docker compose up -d ros-voice-assistant
```

Then verify:
```bash
# Wait ~15s for container to stabilize
docker exec multirepo-ros-voice-assistant-1 bash -lc 'source /opt/ros/*/setup.bash; source /app/ros2_ws/install/setup.bash; timeout 20 ros2 node list'
docker exec multirepo-ros-voice-assistant-1 bash -lc 'source /opt/ros/*/setup.bash; source /app/ros2_ws/install/setup.bash; timeout 20 ros2 service list | grep -E "send_chat_message|set_voice_assistant_state"'
docker logs multirepo-ros-voice-assistant-1 2>&1 | grep -iE "audio_recorder|error|exception" | tail -20
```

## Step 2 — If rebuild doesn't fix it, investigate

Only if the crash persists after rebuild:

1. Check if `/dev/dsp` or ALSA/OSS devices exist in the container and have correct permissions
2. Check `ros_packages/voice_assistant/Dockerfile` for:
   - `audio_recorder` entrypoint/command
   - PortAudio / OSS dependencies
   - `--device` or `--privileged` flags in docker-compose.yaml for audio
3. Check `ros_packages/voice_assistant/voice_assistant/audio_recorder.cpp` (or .py) for the executor shutdown race
4. Consider adding `--device /dev/snd:/dev/snd` or similar to docker-compose.yaml

## Step 3 — Run the E2E test

Once the node is alive and services advertised, run from dev machine:
```bash
/home/pib/.hermes/hermes-agent/venv/bin/pytest tests/e2e/test_voice_assistant_hermes_e2e.py -k persists_reply -v
```

## Acceptance criteria

- [ ] `audio_recorder` node appears in `ros2 node list` and stays alive
- [ ] `/send_chat_message` and `/set_voice_assistant_state` services advertised
- [ ] E2E test `persists_reply_and_recalls_prior_fact` passes
- [ ] No executor/audio errors in logs
- [ ] Other tests in the suite remain green