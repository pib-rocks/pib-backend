# Test Basis: Stereo Camera CPU Optimization

**Repository:** `pib-backend`  
**Requirement:** Jira PR-1507  
**Components:** `ros-camera`, `stereo.py`, DepthAI pipeline, ROS 2 camera node

## Requirement

`stereo.py` (DepthAI camera pipeline node) on the Raspberry Pi 5 exhibits high CPU utilization (>100%).
This story covers the root cause analysis (identifying why the sudden spike occurred) and implementing optimizations to bring CPU load down while maintaining camera frame delivery and stereo depth functionality.

## Acceptance Criteria Traceability

| AC | Acceptance criterion | Coverage | Status |
|---|---|---|---|
| AC1 | Root cause analysis documented for the sudden CPU load spike in `stereo.py`. | Documented in PR-1507 ticket / release notes / commit messages | Planned |
| AC2 | CPU usage of `stereo.py` on Pi 5 reduced significantly under sustained operation. | CPU load measurement on target hardware (`top` / `pidstat`) | Planned |
| AC3 | Camera and stereo depth functionality remain fully operational without frame drops. | `tests/integration/` camera tests & frame verification | Planned |
| AC4 | Automated regression tests & test basis updated to cover the optimization. | This document + pytest suite in `tests/` | Documented |
