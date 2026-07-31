# Implementation Plan: PR-1507 Stereo Camera CPU Optimization

**Repository:** `pib-backend`  
**Requirement:** Jira PR-1507  
**Branch:** `PR-1507`  

## Root Cause Analysis
In `ros_packages/camera/oak_d_lite/stereo.py`:
1. **Unthrottled OpenCV Haar Cascade Face Detection:** `publish_face_center()` runs `cv2.CascadeClassifier.detectMultiScale()` on CPU for every incoming camera frame at 10 Hz, regardless of whether any node is subscribed to `face_center` or whether face tracking is actively needed.
2. **Unconditional Frame Processing:** ISP output frames are grabbed, converted (`getCvFrame()`), resized, face-detected, JPEG-encoded (`cv2.imencode`), and base64-encoded every 100ms (10 Hz), even if no subscribers exist for `camera_topic` or `face_center`.
3. **Full-Resolution Haar Cascade Input:** Haar cascade detection runs on full `1280x720` gray frames without downscaling first, causing massive CPU cycles on Pi 5 ARM cores.

## Optimizations & Fixes

### Task 1: Downscale frame for Face Detection & Skip when no subscribers
- Downscale the grayscale image for `detectMultiScale` (e.g. to `320x180` or `160x90` scale) before face detection, then scale center coordinates back to full resolution.
- Only execute `publish_face_center` when `self.face_center_publisher_.get_subscription_count() > 0` or run face detection at a lower frequency (e.g., every N frames or 2 Hz).
- Only perform JPEG/base64 encoding when `self.publisher_.get_subscription_count() > 0` or when `get_camera_image` service / subscriber is active.

### Task 2: Unit / Integration Test Coverage
- Add / update unit and integration tests in `tests/integration/` or `tests/unit/` verifying frame handling, subscription-awareness, and face center coordinate scaling.
