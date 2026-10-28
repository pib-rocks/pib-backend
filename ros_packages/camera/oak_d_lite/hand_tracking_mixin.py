"""Hand tracking stages extracted from CameraNode."""

from ros_packages.camera.oak_d_lite import stereo as stereo

from datatypes.msg import Detection, DetectionArray

from .face_crop import packet_timestamp
from .hand_tracking import (
    FAST_MODEL_ID,
    HAND_KEYPOINT_NAMES,
    LANDMARK_HANDEDNESS_LAYER,
    LANDMARK_SCORE_LAYER,
    LANDMARK_SCORE_THRESHOLD,
    LANDMARK_VALUE_COUNT,
    LANDMARK_XYZ_LAYERS,
    MANIP_CROP_INSET_PIXELS,
    build_fast_hand_graph,
    decode_palm_result,
    fit_manip_crop,
    landmark_score,
    landmarks_in_crop_pixels,
    map_fast_hand,
    map_landmarks_to_frame,
    parse_fast_script_result,
    relative_landmark_z,
    PalmRegion,
)
from .imitation import (
    LANDMARK_COUNT as IMITATION_LANDMARK_COUNT,
    PALM_PADDING as IMITATION_PALM_PADDING,
    ProcessDetections,
    box_from_points,
    world_landmark_scalars,
)
from .imitation_archive import create_landmark_archive, create_palm_archive


class HandTrackingMixin:
    def _warn_hand_once(self, message):
        if message not in self._hand_warnings:
            self._hand_warnings.add(message)
            self.get_logger().warning(message)

    def _hand_fingerprint_due(self, name, interval=5.0):
        """Rate-limit diagnostic fingerprints without changing hand processing."""
        now = stereo.time.monotonic()
        last_logged = getattr(self, "_hand_fingerprint_last_logged", {}).get(name, 0.0)
        if now - last_logged < interval:
            return False
        if not hasattr(self, "_hand_fingerprint_last_logged"):
            self._hand_fingerprint_last_logged = {}
        self._hand_fingerprint_last_logged[name] = now
        return True

    @staticmethod
    def _hand_tensor_text(values):
        return stereo.np.array2string(
            stereo.np.asarray(values),
            separator=",",
            threshold=stereo.np.inf,
            max_line_width=1000000,
        )

    def _log_palm_fingerprint(self, values, palms):
        if not self._hand_fingerprint_due("palm"):
            return
        records = values.reshape(-1, 8)
        finite = stereo.np.all(stereo.np.isfinite(records), axis=1)
        positive_size = records[:, 3] > 0
        above_threshold = records[:, 0] >= 0.5
        finite_scores = records[finite, 0]
        best_score = (
            float(stereo.np.max(finite_scores)) if finite_scores.size else float("nan")
        )
        self.get_logger().info(
            "HAND_FP DEC "
            f"shape={values.shape} raw={self._hand_tensor_text(values)}; "
            f"decoded_candidates={len(palms)} best_score={best_score:.9g} "
            "score_threshold=0.5 "
            f"dropped_nonfinite={int(stereo.np.count_nonzero(~finite))} "
            f"dropped_low_score={int(stereo.np.count_nonzero(finite & ~above_threshold))} "
            "dropped_nonpositive_size="
            f"{int(stereo.np.count_nonzero(finite & above_threshold & ~positive_size))}"
        )

    def _log_landmark_fingerprint(
        self,
        packet,
        score_tensor,
        landmarks_tensor,
        transformation,
        layer_name=None,
        layer_probe=(),
    ):
        if not self._hand_fingerprint_due("landmark"):
            return
        available_layers = self._packet_layer_names(packet)
        layer_parts = [
            f"{LANDMARK_SCORE_LAYER} shape={score_tensor.shape} "
            f"raw={self._hand_tensor_text(score_tensor)}"
        ]
        if "Identity_2" not in available_layers:
            layer_parts.append("Identity_2 unavailable")
        else:
            try:
                handedness = self._nn_layer(packet, "Identity_2")
                layer_parts.append(
                    f"Identity_2 shape={handedness.shape} "
                    f"raw={self._hand_tensor_text(handedness)}"
                )
            except Exception:
                layer_parts.append("Identity_2 unavailable")
        layer_parts.append(
            f"{layer_name or 'no landmark layer'} "
            f"shape={landmarks_tensor.shape} "
            f"raw={self._hand_tensor_text(landmarks_tensor)}"
        )
        xyz = stereo.np.asarray(landmarks_tensor, dtype=stereo.np.float32).reshape(
            -1, 3
        )
        xy = xyz[:, :2]
        peak = float(stereo.np.max(stereo.np.abs(xy))) if xy.size else 0.0
        xy_space = "normalized" if peak <= 1.5 else "input_pixels"
        self.get_logger().info(
            "HAND_FP LAND "
            + " / ".join(layer_parts)
            + f"; packet_layers={available_layers} "
            f"landmark_layer_probe={list(layer_probe)} "
            f"input_size={self.hand_landmark_input_size} "
            f"xy_space={xy_space} xy_raw={self._hand_tensor_text(xy)} "
            f"TRANSFORM_available={transformation is not None}"
        )

    def _reset_hand_stage_counters(self):
        self.hand_stage_counters = {stage: 0 for stage in stereo.HAND_STAGE_NAMES}
        self._hand_stage_last_logged = dict(self.hand_stage_counters)

    def _reset_imitation_stage_counters(self):
        self.imitation_stage_counters = {
            stage: 0 for stage in stereo.IMITATION_STAGE_NAMES
        }
        self._imitation_stage_last_logged = dict(self.imitation_stage_counters)

    def _count_hand_stage(self, stage, count=1):
        self.hand_stage_counters[stage] += count

    def _count_imitation_stage(self, stage, count=1):
        self.imitation_stage_counters[stage] += count

    def _log_hand_stage_counters(self):
        if not any(
            active.model.model_id == "hand_tracking"
            for active in getattr(self, "_pipeline_models", ())
        ):
            return
        interval = {
            stage: self.hand_stage_counters[stage] - self._hand_stage_last_logged[stage]
            for stage in stereo.HAND_STAGE_NAMES
        }
        last_flowing = "none"
        for stage in stereo.HAND_STAGE_NAMES:
            if interval[stage] > 0:
                last_flowing = stage
        raw = " ".join(
            f"{stage}={self.hand_stage_counters[stage]}"
            for stage in stereo.HAND_STAGE_NAMES
        )
        interval_raw = " ".join(
            f"{stage}={interval[stage]}" for stage in stereo.HAND_STAGE_NAMES
        )
        source_width, source_height = getattr(self, "hand_source_size", (0, 0))
        self.get_logger().info(
            f"hand_tracking stage packets total: {raw}; "
            f"interval: {interval_raw}; last_flowing={last_flowing}; "
            f"branch={source_width}x{source_height}"
        )
        self._hand_stage_last_logged = dict(self.hand_stage_counters)

    def _log_imitation_stage_counters(self):
        if not any(
            active.model.model_id == "imitation"
            for active in getattr(self, "_pipeline_models", ())
        ):
            return
        interval = {
            stage: (
                self.imitation_stage_counters[stage]
                - self._imitation_stage_last_logged[stage]
            )
            for stage in stereo.IMITATION_STAGE_NAMES
        }
        last_flowing = "none"
        for stage in stereo.IMITATION_STAGE_NAMES:
            if interval[stage] > 0:
                last_flowing = stage
        raw = " ".join(
            f"{stage}={self.imitation_stage_counters[stage]}"
            for stage in stereo.IMITATION_STAGE_NAMES
        )
        interval_raw = " ".join(
            f"{stage}={interval[stage]}" for stage in stereo.IMITATION_STAGE_NAMES
        )
        source_width, source_height = self.imitation_source_size
        depths = (
            f"colour_queue={self._queue_state(getattr(self, 'queue', None))} "
            f"imitation_queue={self._queue_state(getattr(self, 'imitation_queue', None))} "
            f"pending={1 if getattr(self, '_pending_imitation_packet', None) is not None else 0}"
        )
        self.get_logger().info(
            f"imitation stage packets total: {raw}; "
            f"interval: {interval_raw}; last_flowing={last_flowing}; "
            f"branch={source_width}x{source_height}; {depths}"
        )

        # Mark the moment the pipeline stops, not just the aftermath: without
        # this the counters only ever show that something froze, and the state
        # at that instant - which queue was full - is gone by the stereo.time anyone
        # looks. Log once per stall, and re-arm as soon as stages move again.
        if all(interval[stage] == 0 for stage in stereo.IMITATION_STAGE_NAMES):
            if not getattr(self, "_imitation_stall_logged", False):
                self._imitation_stall_logged = True
                self.get_logger().warning(
                    f"IMIT_STALL no imitation stage advanced in the last "
                    f"interval; last_flowing={last_flowing}; {depths}"
                )
        else:
            self._imitation_stall_logged = False
        self._imitation_stage_last_logged = dict(self.imitation_stage_counters)

    @staticmethod
    def _nn_layer(packet, name):
        return stereo.np.asarray(packet.getTensor(name), dtype=stereo.np.float32)

    @staticmethod
    def _packet_layer_names(packet):
        getter = getattr(packet, "getAllLayerNames", None)
        if getter is None:
            return []
        try:
            return sorted(str(name) for name in getter())
        except Exception:
            return []

    def _hand_landmark_score(self, packet):
        """Read the landmark presence score that decides whether a hand is kept.

        A good crop scores near 1.0 and an unusable one scores below 0.02, so
        the same value the reference gates on is usable here.  A missing or
        malformed head returns NaN, which the caller treats as a drop rather
        than letting it pass silently.
        """
        try:
            tensor = self._nn_layer(packet, LANDMARK_SCORE_LAYER)
            return tensor, landmark_score(tensor)
        except Exception:
            return stereo.np.zeros(0, dtype=stereo.np.float32), float("nan")

    def _hand_landmark_handedness(self, packet):
        """Read the hand's handedness, or NaN when the head is absent."""
        try:
            tensor = self._nn_layer(packet, LANDMARK_HANDEDNESS_LAYER)
            values = stereo.np.asarray(tensor, dtype=stereo.np.float32).reshape(-1)
            if values.size != 1:
                raise ValueError("handedness must contain one value")
            return float(values[0])
        except Exception:
            return float("nan")

    def _hand_landmark_tensor(self, packet):
        """Return the first landmark head that actually carries 21 XYZ triples.

        A tensor name that the blob does not expose yields either an exception
        or an empty array, and an empty array maps to zero keypoints without
        raising anything - the landmark result is then silently dropped.  Every
        rejected candidate is reported so the chosen head is visible in the log.
        """
        probe = []
        for name in LANDMARK_XYZ_LAYERS:
            try:
                values = self._nn_layer(packet, name)
            except Exception as exc:
                probe.append(f"{name}=absent({type(exc).__name__})")
                continue
            if values.size == LANDMARK_VALUE_COUNT:
                probe.append(f"{name}=used(size={values.size})")
                return name, values, probe
            probe.append(f"{name}=rejected(size={values.size})")
        return None, stereo.np.zeros(0, dtype=stereo.np.float32), probe

    def _map_hand_landmarks(
        self,
        packet,
        tensor,
        palm,
        frame_width,
        frame_height,
        source_width,
        source_height,
        score_tensor=None,
        layer_name=None,
        layer_probe=(),
    ):
        """Map landmark-crop coordinates onto the published preview frame."""
        values = landmarks_in_crop_pixels(tensor, self.hand_landmark_input_size)

        def via_palm_roi():
            return map_landmarks_to_frame(
                values,
                palm,
                frame_width,
                frame_height,
                self.hand_landmark_input_size,
                source_width,
                source_height,
            )

        transformation = self._packet_transformation(packet)
        if score_tensor is not None:
            self._log_landmark_fingerprint(
                packet,
                score_tensor,
                tensor,
                transformation,
                layer_name,
                layer_probe,
            )
        if transformation is None:
            return via_palm_roi()

        try:
            scale_x = frame_width / float(source_width)
            scale_y = frame_height / float(source_height)
            points = []
            for crop_x, crop_y, _ in values:
                crop_point = stereo.dai.Point2f(float(crop_x), float(crop_y))
                source_point = transformation.invTransformPoint(crop_point)
                points.append(
                    (
                        min(
                            float(frame_width),
                            max(0.0, float(source_point.x) * scale_x),
                        ),
                        min(
                            float(frame_height),
                            max(0.0, float(source_point.y) * scale_y),
                        ),
                    )
                )
            return points
        except Exception:
            return via_palm_roi()

    def _publish_hand_detections(self, frame_width, frame_height, detections):
        message = DetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.model_id = "hand_tracking"
        message.frame_width = frame_width
        message.frame_height = frame_height
        message.detections = detections
        self.last_detections["hand_tracking"] = message
        publisher = self.detection_publishers.get("hand_tracking")
        if publisher is not None:
            publisher.publish(message)
        self._count_hand_stage("publish")
        self.pipeline_manager.record_packet("hand_tracking")

    def _imitation_detection(self, hand, frame_width, frame_height):
        landmarks = hand.get("landmarks", ())
        if len(landmarks) != IMITATION_LANDMARK_COUNT:
            raise ValueError("imitation result must contain 21 landmarks")
        # Decision (a): the published box encloses the 21 landmarks. The
        # detector's own palm box covers the palm only and would leave the
        # fingers outside the rectangle drawn in Cerebra.
        bbox = box_from_points(landmarks, frame_width, frame_height)
        world_names, world_values = world_landmark_scalars(hand.get("world", ()))

        detection = Detection()
        detection.label = "hand"
        detection.score = float(hand["landmark_score"])
        (
            detection.x_min,
            detection.y_min,
            detection.x_max,
            detection.y_max,
        ) = bbox
        detection.keypoint_names = list(HAND_KEYPOINT_NAMES)
        detection.keypoint_x = [float(point[0]) for point in landmarks]
        detection.keypoint_y = [float(point[1]) for point in landmarks]
        detection.keypoint_z = [0.0] * len(HAND_KEYPOINT_NAMES)
        detection.scalar_names = [
            "handedness",
            "palm_score",
            "landmark_score",
        ] + world_names
        detection.scalar_values = [
            float(hand["handedness"]),
            float(hand["palm_score"]),
            float(hand["landmark_score"]),
        ] + world_values
        return detection

    def _publish_imitation_detections(self, frame_width, frame_height, detections):
        message = DetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.model_id = "imitation"
        message.frame_width = frame_width
        message.frame_height = frame_height
        message.detections = detections
        self.last_detections["imitation"] = message
        publisher = self.detection_publishers.get("imitation")
        if publisher is not None:
            publisher.publish(message)
        self._count_imitation_stage("publish")
        self.pipeline_manager.record_packet("imitation")

    def _publish_face_crop_detection(self, result_packet, face):
        if self.current_frame is None:
            frame_width = self.preview_width
            frame_height = self.preview_height
        else:
            frame_height, frame_width = self.current_frame.shape[:2]
        if self.face_crop_model_id is None or self.face_crop_translator is None:
            raise RuntimeError("face-crop classifier metadata is unavailable")
        detection = self.face_crop_translator(
            result_packet,
            face,
            frame_width,
            frame_height,
        )

        message = DetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.model_id = self.face_crop_model_id
        message.frame_width = frame_width
        message.frame_height = frame_height
        message.detections = [detection]
        self.last_detections[self.face_crop_model_id] = message
        publisher = self.detection_publishers.get(self.face_crop_model_id)
        if publisher is not None:
            publisher.publish(message)
        self.pipeline_manager.record_packet(self.face_crop_model_id)

    def _publish_hand_mp_detections(self, frame_width, frame_height, detections):
        message = DetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.model_id = "hand_tracking_mp"
        message.frame_width = frame_width
        message.frame_height = frame_height
        message.detections = detections
        self.last_detections["hand_tracking_mp"] = message
        publisher = self.detection_publishers.get("hand_tracking_mp")
        if publisher is not None:
            publisher.publish(message)
        self._count_hand_stage("publish")
        self.pipeline_manager.record_packet("hand_tracking_mp")

    def _consume_imitation_packet(self, packet):
        frame_height, frame_width = self.current_frame.shape[:2]
        palm_count = len(packet.reference_data.detections)
        result_count = len(packet.items)
        self._count_imitation_stage("palm_detector_nn")
        self._count_imitation_stage("decoding_nn")
        self._count_imitation_stage("decoding_result")
        self._count_imitation_stage("image_manip_config", palm_count)
        self._count_imitation_stage("image_manip_roi", result_count)
        self._count_imitation_stage("hand_landmark_nn", result_count)
        self._count_imitation_stage("post_processing", result_count)

        # The trace runs BEFORE the conversion on purpose: when stereo.gathered_hands
        # raises, the trace is the only evidence left, and a trace that sits
        # after it disappears exactly when it is needed most.
        for palm_score, landmark_score, crop in stereo.gathered_result_trace_values(
            packet
        ):
            score_text = (
                f"{landmark_score:.9g}" if landmark_score is not None else "unavailable"
            )
            self.get_logger().info(
                "IMIT_TRACE "
                f"palm_score={palm_score:.9g} landmark_score={score_text} "
                f"crop=({crop[0]:.9g},{crop[1]:.9g},"
                f"{crop[2]:.9g},{crop[3]:.9g})"
            )

        hands = stereo.gathered_hands(packet, frame_width, frame_height)
        detections = []
        for hand in hands:
            try:
                detections.append(
                    self._imitation_detection(hand, frame_width, frame_height)
                )
            except (KeyError, TypeError, ValueError) as exc:
                self._warn_hand_once(f"Invalid imitation gathered result: {exc}")
        self._publish_imitation_detections(frame_width, frame_height, detections)

    def _process_imitation(self):
        if self.imitation_queue is None or self.current_frame is None:
            return
        packet = self._pending_imitation_packet
        self._pending_imitation_packet = None
        for _ in range(32):
            if packet is None:
                packet = self.imitation_queue.tryGet()
            if packet is None:
                break
            try:
                self._consume_imitation_packet(packet)
            except (AttributeError, IndexError, TypeError, ValueError) as exc:
                # NOT _warn_hand_once: that logs a single line for the whole
                # process lifetime. A repeating failure then looks like "no hand
                # detected" while 8 packets per second are silently discarded,
                # which is exactly how this bug hid. Log the type and a running
                # count, throttled so it cannot flood the log.
                self._imitation_error_count = (
                    getattr(self, "_imitation_error_count", 0) + 1
                )
                if self._imitation_error_count % 25 == 1:
                    self.get_logger().error(
                        "IMIT_DROP "
                        f"count={self._imitation_error_count} "
                        f"type={type(exc).__name__} message={exc}"
                    )
            packet = None

    def _log_hand_assembly_fingerprint(self, batch):
        if not batch.get("log_details"):
            return
        reasons = batch.get("drop_reasons", [])
        reason_counts = {
            reason: reasons.count(reason) for reason in sorted(set(reasons))
        }
        self.get_logger().info(
            "HAND_FP ASSEMBLY "
            f"candidates={batch.get('candidates', 0)} "
            f"landmark_results={batch.get('landmark_results', 0)} "
            f"keypoints_built={batch.get('keypoints_built', 0)} "
            f"landmark_layer={batch.get('landmark_layer')} "
            f"appended={len(batch['detections'])} "
            f"dropped={len(reasons)} reasons={reason_counts or {}}"
        )

    def _log_hand_keypoint_fingerprint(self, batch, detail):
        """Report one landmark result and the exact reason it was or was not kept."""
        if not batch.get("log_details"):
            return
        self.get_logger().info(
            "HAND_FP KP "
            f"candidates={batch.get('candidates', 0)} "
            f"landmark_results={batch.get('landmark_results', 0)} "
            f"keypoints_built={batch.get('keypoints_built', 0)} "
            f"detections_pending={len(batch['detections'])} "
            f"{detail}"
        )

    def _hand_detection_message(
        self,
        palm,
        landmarks,
        frame_width,
        frame_height,
        source_width,
        source_height,
        landmark_score_value=float("nan"),
        handedness=float("nan"),
        relative_z=(),
    ):
        """Assemble one hand the way the reference reports it.

        The reference carries three things this message has to hold: the
        landmark presence score as the result's confidence, the handedness that
        tells the two hands apart, and the landmark head's third component as a
        hand-relative, unitless depth - all three components of a landmark share
        one scale, as they do in ``rrn_lms``.  ``z_source`` names where the z
        came from, because ``keypoint_z`` is otherwise reserved for depth in
        millimetres.
        """
        detection = Detection()
        detection.label = "hand"
        detection.score = float(landmark_score_value)
        (
            detection.x_min,
            detection.y_min,
            detection.x_max,
            detection.y_max,
        ) = palm.bbox_pixels(frame_width, frame_height, source_width, source_height)
        detection.keypoint_names = list(HAND_KEYPOINT_NAMES)
        detection.keypoint_x = [float(point[0]) for point in landmarks]
        detection.keypoint_y = [float(point[1]) for point in landmarks]
        z_values = [float(value) for value in relative_z]
        if len(z_values) != len(HAND_KEYPOINT_NAMES):
            z_values = [0.0] * len(HAND_KEYPOINT_NAMES)
        detection.keypoint_z = z_values
        detection.scalar_names = [
            "handedness",
            "palm_score",
            "landmark_score",
            "z_source",
        ]
        detection.scalar_values = [
            float(handedness),
            float(palm.score),
            float(landmark_score_value),
            1.0,
        ]
        return detection

    def _queue_landmark_crops(
        self,
        palms,
        frame_width,
        frame_height,
        source_width,
        source_height,
    ):
        batch = {
            "remaining": len(palms),
            "candidates": len(palms),
            "landmark_results": 0,
            "keypoints_built": 0,
            "landmark_layer": None,
            "detections": [],
            "drop_reasons": [],
            "frame_width": frame_width,
            "frame_height": frame_height,
            "source_width": source_width,
            "source_height": source_height,
        }
        for index, palm in enumerate(palms):
            config = self._landmark_crop_config(
                palm,
                source_width,
                source_height,
                reuse_previous=index + 1 < len(palms),
            )
            self.hand_landmark_config_queue.send(config)
            self._count_hand_stage("image_manip_config")
            self._pending_hands.append((palm, batch))

    def _queue_empty_landmark_frame(
        self,
        frame_width,
        frame_height,
        source_width,
        source_height,
    ):
        """Advance the config-gated ROI branch for a frame with no palms."""
        batch = {
            "remaining": 1,
            "candidates": 0,
            "landmark_results": 0,
            "keypoints_built": 0,
            "landmark_layer": None,
            "detections": [],
            "drop_reasons": ["no decoded palm candidates"],
            "frame_width": frame_width,
            "frame_height": frame_height,
            "source_width": source_width,
            "source_height": source_height,
        }
        config = self._landmark_crop_config(
            None,
            source_width,
            source_height,
        )
        self.hand_landmark_config_queue.send(config)
        self._count_hand_stage("image_manip_config")
        # The full-frame landmark result is deliberately discarded. Keeping a
        # sentinel in the pairing queue prevents its packet from being matched
        # with a palm from a later decoder frame.
        self._pending_hands.append((None, batch))

    def _landmark_crop_config(
        self, palm, source_width, source_height, reuse_previous=False
    ):
        """Build a complete dynamic warp config for one decoded palm ROI."""
        if palm is None:
            roi_x, roi_y, roi_width, roi_height, rotation = 0.5, 0.5, 1.0, 1.0, 0.0
        else:
            roi_x, roi_y, roi_width, roi_height = palm.roi_for_frame(
                source_width, source_height
            )
            rotation = palm.rotation
        geometry = (roi_x, roi_y, roi_width, roi_height, rotation)
        if not all(stereo.math.isfinite(value) for value in geometry):
            raise ValueError("hand ROI contains non-finite geometry")
        if roi_width <= 0 or roi_height <= 0:
            raise ValueError("hand ROI must have positive dimensions")
        if source_width <= 0 or source_height <= 0:
            raise ValueError("hand ROI source must have positive dimensions")

        crop = fit_manip_crop(
            source_width,
            source_height,
            self.hand_landmark_input_size,
            self.hand_landmark_input_size,
        )
        if palm is None:
            # The sentinel keeps the config-gated branch alive on frames without
            # a palm. Its result is discarded, so it reuses the build-stereo.time crop:
            # the complete measured source, inset from the validation boundary.
            roi_x, roi_y = crop.center_x, crop.center_y
            roi_width, roi_height = crop.width, crop.height

        # DepthAI rejects a rotated crop unless its complete bounding box is
        # inside the source image; warp border replication happens only after
        # that validation. Fit the decoded square in actual branch pixels,
        # preserving its aspect and rotation, then normalize it again.
        cos_rotation = abs(stereo.math.cos(rotation))
        sin_rotation = abs(stereo.math.sin(rotation))
        crop_width = roi_width * source_width
        crop_height = roi_height * source_height
        extent_x = (crop_width * cos_rotation + crop_height * sin_rotation) / 2.0
        extent_y = (crop_width * sin_rotation + crop_height * cos_rotation) / 2.0
        # Keep every corner at least half a pixel inside the frame. Maintaining
        # the decoded center is more important than retaining the full ROI near
        # an edge, so shrink around that center instead of moving off the hand.
        inset = MANIP_CROP_INSET_PIXELS
        center_x = min(
            source_width - inset,
            max(inset, roi_x * source_width),
        )
        center_y = min(
            source_height - inset,
            max(inset, roi_y * source_height),
        )
        available_x = max(inset, min(center_x, source_width - center_x) - inset)
        available_y = max(inset, min(center_y, source_height - center_y) - inset)
        scale = min(
            1.0,
            available_x / extent_x,
            available_y / extent_y,
        )
        crop_width *= scale
        crop_height *= scale
        roi_x = center_x / source_width
        roi_y = center_y / source_height
        roi_width = crop_width / source_width
        roi_height = crop_height / source_height

        rotated = stereo.dai.RotatedRect()
        rotated.center.x = roi_x
        rotated.center.y = roi_y
        rotated.size.width = roi_width
        rotated.size.height = roi_height
        rotated.angle = stereo.math.degrees(rotation)

        config = stereo.dai.ImageManipConfig()
        config.setOutputSize(crop.output_width, crop.output_height)
        config.setFrameType(stereo.dai.ImgFrame.Type.BGR888p)
        # roi_for_frame converts decoder coordinates to normalized coordinates
        # of the hand tap as the branch actually delivers it.
        config.addCropRotatedRect(rotated, True)
        # Palm ROIs routinely cross an image edge. Explicit warp border handling
        # keeps those valid instead of making ImageManip skip the frame.
        border_replicate = getattr(config, "setWarpBorderReplicatePixels", None)
        if border_replicate is not None:
            border_replicate()
        config.setReusePreviousImage(reuse_previous)
        return config

    def _drop_landmark_result(self, batch, check, reason):
        """Record and report the exact check that skipped one landmark append."""
        batch.setdefault("drop_reasons", []).append(reason)
        self._log_hand_keypoint_fingerprint(batch, f"skipped={check} reason={reason}")

    def _consume_landmark_result(self, packet, palm, batch):
        """Assemble one landmark result, naming the check that skips the append."""
        batch["landmark_results"] = batch.get("landmark_results", 0) + 1
        if palm is None:
            self._log_hand_keypoint_fingerprint(
                batch, "skipped=sentinel reason=crop carries no paired palm"
            )
            return

        score_tensor, score = self._hand_landmark_score(packet)
        if not stereo.math.isfinite(score) or score < LANDMARK_SCORE_THRESHOLD:
            self._drop_landmark_result(
                batch,
                "score",
                f"landmark score {score:.6g} below {LANDMARK_SCORE_THRESHOLD}",
            )
            return
        layer_name, landmarks_tensor, layer_probe = self._hand_landmark_tensor(packet)
        batch["landmark_layer"] = layer_name

        if layer_name is None:
            self._log_landmark_fingerprint(
                packet,
                score_tensor,
                landmarks_tensor,
                self._packet_transformation(packet),
                layer_name,
                layer_probe,
            )
            reason = (
                f"no landmark layer carries {LANDMARK_VALUE_COUNT} values; "
                f"probe={list(layer_probe)}"
            )
            self._drop_landmark_result(batch, "layer", reason)
            self._warn_hand_once(f"Hand landmark output unusable: {reason}")
            return

        landmarks = self._map_hand_landmarks(
            packet,
            landmarks_tensor,
            palm,
            batch["frame_width"],
            batch["frame_height"],
            batch["source_width"],
            batch["source_height"],
            score_tensor,
            layer_name,
            layer_probe,
        )
        batch["keypoints_built"] = batch.get("keypoints_built", 0) + len(landmarks)
        if not landmarks:
            self._drop_landmark_result(batch, "mapping", "missing mapped keypoints")
            return

        batch["detections"].append(
            self._hand_detection_message(
                palm,
                landmarks,
                batch["frame_width"],
                batch["frame_height"],
                batch["source_width"],
                batch["source_height"],
                score,
                self._hand_landmark_handedness(packet),
                relative_landmark_z(landmarks_tensor, self.hand_landmark_input_size),
            )
        )
        self._log_hand_keypoint_fingerprint(
            batch,
            f"appended=1 score={score:.9g} layer={layer_name} "
            f"keypoints={len(landmarks)}",
        )

    def _process_hand_tracking(self):
        if self.hand_decoder_queue is None or self.current_frame is None:
            return

        while self._pending_hands:
            packet = self.hand_landmark_queue.tryGet()
            if packet is None:
                break
            self._count_hand_stage("hand_landmark_nn")
            palm, batch = self._pending_hands.popleft()
            if "log_details" not in batch:
                batch["log_details"] = self._hand_fingerprint_due("assembly", 1.0)
            try:
                self._consume_landmark_result(packet, palm, batch)
            except (RuntimeError, ValueError) as exc:
                reason = f"invalid landmark output: {type(exc).__name__}: {exc}"
                self._drop_landmark_result(batch, "exception", reason)
                self._warn_hand_once(f"Invalid hand landmark output: {exc}")
            self._count_hand_stage("post_processing")
            batch["remaining"] -= 1
            if batch["remaining"] == 0:
                self._log_hand_assembly_fingerprint(batch)
                self._publish_hand_detections(
                    batch["frame_width"],
                    batch["frame_height"],
                    batch["detections"],
                )

        # Keep decoder and landmark packets paired; accept the next palm frame
        # only after all landmark crops from the previous one have completed.
        if self._pending_hands:
            return
        packet = self._pending_hand_decoder_packet
        self._pending_hand_decoder_packet = None
        if packet is None:
            packet = self.hand_decoder_queue.tryGet()
        if packet is None:
            return
        self._count_hand_stage("decoding_nn")
        frame_height, frame_width = self.current_frame.shape[:2]
        source_width, source_height = self.hand_source_size
        if not source_width or not source_height:
            source_width, source_height = self.current_source_size
        if not source_width or not source_height:
            source_width, source_height = frame_width, frame_height
        try:
            palm_values = self._nn_layer(packet, "result")
            palms = decode_palm_result(palm_values)
            self._log_palm_fingerprint(palm_values, palms)
        except (RuntimeError, ValueError) as exc:
            self._warn_hand_once(f"Invalid palm decoder output: {exc}")
            palms = []
        self._count_hand_stage("decoding_result")
        if not palms:
            self._queue_empty_landmark_frame(
                frame_width,
                frame_height,
                source_width,
                source_height,
            )
            return
        self._queue_landmark_crops(
            palms,
            frame_width,
            frame_height,
            source_width,
            source_height,
        )

    def _fast_detection_message(self, hand):
        detection = Detection()
        detection.label = "hand"
        detection.score = float(hand.landmark_score)
        detection.x_min = int(hand.x_min)
        detection.y_min = int(hand.y_min)
        detection.x_max = int(hand.x_max)
        detection.y_max = int(hand.y_max)
        detection.keypoint_names = list(HAND_KEYPOINT_NAMES)
        detection.keypoint_x = [float(value) for value in hand.keypoint_x]
        detection.keypoint_y = [float(value) for value in hand.keypoint_y]
        detection.keypoint_z = [float(value) for value in hand.keypoint_z]
        detection.scalar_names = [
            "handedness",
            "palm_score",
            "landmark_score",
            "z_source",
        ]
        detection.scalar_values = [
            float(hand.handedness),
            float(hand.palm_score),
            float(hand.landmark_score),
            1.0,
        ]
        return detection

    def _publish_hand_fast_detections(self, frame_width, frame_height, detections):
        message = DetectionArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.model_id = FAST_MODEL_ID
        message.frame_width = frame_width
        message.frame_height = frame_height
        message.detections = detections
        self.last_detections[FAST_MODEL_ID] = message
        publisher = self.detection_publishers.get(FAST_MODEL_ID)
        if publisher is not None:
            publisher.publish(message)
        self.pipeline_manager.record_packet(FAST_MODEL_ID)

    def _process_hand_tracking_fast(self):
        queue = getattr(self, "hand_fast_queue", None)
        if queue is None or self.current_frame is None:
            return
        frame_height, frame_width = self.current_frame.shape[:2]
        source_width, source_height = getattr(self, "hand_fast_source_size", (0, 0))
        if source_width <= 0 or source_height <= 0:
            return
        packet = getattr(self, "_pending_hand_fast_packet", None)
        self._pending_hand_fast_packet = None
        for _ in range(32):
            if packet is None:
                packet = queue.tryGet()
            if packet is None:
                break
            try:
                detections = [
                    self._fast_detection_message(
                        map_fast_hand(
                            hand,
                            frame_width,
                            frame_height,
                            source_width,
                            source_height,
                        )
                    )
                    for hand in parse_fast_script_result(packet)
                ]
            except (KeyError, TypeError, ValueError) as exc:
                self._hand_fast_error_count = (
                    getattr(self, "_hand_fast_error_count", 0) + 1
                )
                if self._hand_fast_error_count % 25 == 1:
                    self.get_logger().error(
                        "HAND_FAST_DROP "
                        f"count={self._hand_fast_error_count} "
                        f"type={type(exc).__name__} message={exc}"
                    )
                packet = None
                continue
            self._publish_hand_fast_detections(frame_width, frame_height, detections)
            packet = None

    def _publish_ready_face_crops(self, stamp):
        entry = self.face_crop_pairs.get(stamp)
        if entry is None or entry["faces"] is None:
            return
        # Stop at the shorter side. In particular, YuNet may report more faces
        # than the three-input gaze graph manages to complete for this frame;
        # unmatched indices are retained for a late packet and eventually
        # evicted with the bounded timestamp buffer instead of being indexed.
        while entry["faces"] and entry["results"]:
            face = entry["faces"].popleft()
            result = entry["results"].popleft()
            try:
                self._publish_face_crop_detection(result, face)
            except (
                AttributeError,
                KeyError,
                RuntimeError,
                TypeError,
                ValueError,
            ) as exc:
                self.get_logger().warning(f"Invalid face-crop result: {exc}")
        if not entry["faces"] and not entry["results"]:
            self.face_crop_pairs.pop(stamp, None)

    def _process_face_crop(self):
        """Pair parsed face boxes and raw classifier packets by timestamp."""
        if (
            self.face_crop_detection_queue is None
            or self.face_crop_result_queue is None
        ):
            return
        for _ in range(stereo.FACE_CROP_PAIR_WINDOW):
            packet = self.face_crop_detection_queue.tryGet()
            if packet is None:
                break
            faces = tuple(getattr(packet, "detections", ()) or ())
            if not faces:
                continue
            stamp = packet_timestamp(packet)
            entry = self.face_crop_pairs.setdefault(
                stamp,
                {"faces": None, "results": stereo.deque()},
            )
            if entry["faces"] is None:
                entry["faces"] = stereo.deque(faces)
            self._publish_ready_face_crops(stamp)

        packet = self._pending_face_crop_packet
        self._pending_face_crop_packet = None
        for _ in range(stereo.FACE_CROP_PAIR_WINDOW):
            if packet is None:
                packet = self.face_crop_result_queue.tryGet()
            if packet is None:
                break
            stamp = packet_timestamp(packet)
            entry = self.face_crop_pairs.setdefault(
                stamp,
                {"faces": None, "results": stereo.deque()},
            )
            entry["results"].append(packet)
            self._publish_ready_face_crops(stamp)
            packet = None

        while len(self.face_crop_pairs) > stereo.MAX_FACE_CROP_BUFFER:
            self.face_crop_pairs.pop(min(self.face_crop_pairs), None)

    @staticmethod
    def _hand_mp_stamp(packet):
        """Normalise a packet timestamp to a (seconds, nanoseconds) key.

        depthai hands out datetime.timedelta for device packets, the replay
        helpers use an object with sec/nanosec; both spellings are read here
        instead of assuming one of them.
        """
        stamp = packet.getTimestamp()
        seconds = getattr(stamp, "sec", None)
        if seconds is None:
            seconds = getattr(stamp, "seconds", 0)
        nanos = getattr(stamp, "nanosec", None)
        if nanos is None:
            nanos = getattr(stamp, "microseconds", 0) * 1000
        return (int(seconds), int(nanos))

    def _process_hand_mp(self):
        """Drain both streams and publish the hands they agree on."""
        # Both queues and the frame are required. Checking only one of them left
        # this called every loop iteration with a None queue whenever the chain
        # was not the active one, and an AttributeError there kills the whole
        # publish path without a traceback.
        if (
            self.hand_mp_landmark_queue is None
            or self.hand_mp_detection_queue is None
            or self.current_frame is None
        ):
            return
        for _ in range(stereo.HAND_MP_PAIR_WINDOW):
            packet = self.hand_mp_detection_queue.tryGet()
            if packet is None:
                break
            stamp = self._hand_mp_stamp(packet)
            entry = self.hand_mp_pairs.pop(stamp, None)
            if entry is not None:
                entry["detection"] = packet
            else:
                self.hand_mp_pairs[stamp] = {"detection": packet, "landmark": None}
        for _ in range(stereo.HAND_MP_PAIR_WINDOW):
            packet = self.hand_mp_landmark_queue.tryGet()
            if packet is None:
                break
            stamp = self._hand_mp_stamp(packet)
            entry = self.hand_mp_pairs.pop(stamp, None)
            if entry is None:
                self.hand_mp_pairs[stamp] = {"detection": None, "landmark": packet}
                continue
            detection = entry.get("detection")
            if detection is None:
                self.hand_mp_pairs[stamp] = {"detection": None, "landmark": packet}
                continue
            try:
                self._publish_hand_mp_pair(detection, packet)
            except (KeyError, TypeError, ValueError) as exc:
                self._warn_hand_once(f"Invalid hand_mp pair: {exc}")
        # Keep the buffer bounded: drop the oldest unpaired entries.
        while len(self.hand_mp_pairs) > stereo.MAX_HAND_MP_BUFFER:
            oldest = min(self.hand_mp_pairs)
            self.hand_mp_pairs.pop(oldest, None)

    def _publish_hand_mp_pair(self, detection_packet, landmark_packet):
        """Turn one matched (palm detection, landmark result) pair into detections."""
        frame_height, frame_width = self.current_frame.shape[:2]
        source_width, source_height = self.hand_mp_source_size
        score_tensor, landmark_score_value = self._hand_landmark_score(landmark_packet)
        layer_name, values, layer_probe = self._hand_landmark_tensor(landmark_packet)
        detections = []
        if (
            layer_name is not None
            and stereo.math.isfinite(landmark_score_value)
            and landmark_score_value >= LANDMARK_SCORE_THRESHOLD
        ):
            crop_points = landmarks_in_crop_pixels(
                values, self.hand_landmark_input_size
            )
            relative_z = relative_landmark_z(values, self.hand_landmark_input_size)
            palm = self._hand_mp_palm(detection_packet, frame_width, frame_height)
            mapped = self._map_hand_landmarks(
                landmark_packet,
                values,
                palm,
                frame_width,
                frame_height,
                source_width,
                source_height,
                score_tensor,
                layer_name,
                layer_probe,
            )
            if mapped:
                detections.append(
                    self._hand_mp_detection(
                        palm,
                        mapped,
                        relative_z,
                        landmark_score_value,
                        landmark_packet,
                        frame_width,
                        frame_height,
                        source_width,
                        source_height,
                    )
                )
        self._publish_hand_mp_detections(frame_width, frame_height, detections)

    def _hand_mp_palm(self, detection_packet, frame_width, frame_height):
        """Build the PalmRegion the mapping path needs from the parsed detection.

        The palm parser hands out stereo.dai-style detections whose geometry lives in a
        RotatedRect (``getBoundingBox()``), not in x_min/y_min/x_max/y_max: reading
        those attributes silently yielded zeros and an "empty box" for every hand.
        """
        detections = getattr(detection_packet, "detections", None) or []
        if not detections:
            raise ValueError("palm detection packet carries no detection")
        first = detections[0]
        rect = first.getBoundingBox()
        center_x = float(rect.center.x)
        center_y = float(rect.center.y)
        width = float(rect.size.width)
        height = float(rect.size.height)
        rotation = float(getattr(rect, "angle", 0.0) or 0.0)
        box_size = max(width, height)
        if box_size <= 0.0:
            raise ValueError("palm detection has an empty box")
        # The parser fills the detection's confidence (stereo.dai's ImgDetection field);
        # there is no "score" attribute, so reading it yielded NaN for every hand.
        palm_score = float("nan")
        for attribute in ("confidence", "score"):
            try:
                palm_score = float(getattr(first, attribute))
                break
            except (AttributeError, TypeError, ValueError):
                continue
        if abs(rotation) < 1e-6:
            # The reference computes the rotation from the wrist and middle-finger
            # anchors; the parser may leave the rect axis aligned.
            keypoints = list(getattr(first, "keypoints", None) or [])
            if len(keypoints) >= 3:
                try:
                    delta_x = float(keypoints[2].x) - float(keypoints[0].x)
                    delta_y = float(keypoints[2].y) - float(keypoints[0].y)
                    rotation = 0.5 * stereo.math.pi - stereo.math.atan2(
                        -delta_y, delta_x
                    )
                    rotation -= (
                        2
                        * stereo.math.pi
                        * stereo.math.floor(
                            (rotation + stereo.math.pi) / (2 * stereo.math.pi)
                        )
                    )
                except (AttributeError, TypeError):
                    rotation = 0.0
        # The crop the device makes is the square box plus padding and is
        # letterboxed to the landmark input, so the region is a little larger.
        roi_size = box_size * (1.0 + 2.0 * float(IMITATION_PALM_PADDING))
        return PalmRegion(
            score=palm_score,
            box_x=center_x - 0.5 * box_size * stereo.math.cos(rotation),
            box_y=center_y - 0.5 * box_size * stereo.math.sin(rotation),
            box_size=box_size,
            roi_x=center_x,
            roi_y=center_y,
            roi_size=roi_size,
            rotation=rotation,
        )

    def _hand_mp_detection(
        self,
        palm,
        mapped,
        relative_z,
        landmark_score_value,
        landmark_packet,
        frame_width,
        frame_height,
        source_width,
        source_height,
    ):
        """Publish one hand with the box enclosing its keypoints, as the example does."""
        xs = [float(point[0]) for point in mapped]
        ys = [float(point[1]) for point in mapped]
        box_x = min(xs) / float(frame_width)
        box_y = min(ys) / float(frame_height)
        box_size = max(
            (max(xs) - min(xs)) / float(frame_width),
            (max(ys) - min(ys)) / float(frame_height),
        )
        enclosing = PalmRegion(
            score=palm.score,
            box_x=box_x,
            box_y=box_y,
            box_size=box_size,
            roi_x=box_x,
            roi_y=box_y,
            roi_size=box_size,
            rotation=0.0,
        )
        # Same reader the three-blob chain uses: it reshapes the head and returns
        # NaN when the blob exposes no usable handedness, where reading values[0]
        # by hand failed on the head's shape and silently published 0.0.
        handedness = float(self._hand_landmark_handedness(landmark_packet))
        if not stereo.math.isfinite(handedness):
            self._warn_hand_once(
                "hand_mp: landmark packet carries no usable handedness head"
            )
        return self._hand_detection_message(
            enclosing,
            [(float(p[0]), float(p[1])) for p in mapped],
            frame_width,
            frame_height,
            source_width,
            source_height,
            landmark_score_value=landmark_score_value,
            handedness=handedness,
            relative_z=relative_z,
        )
