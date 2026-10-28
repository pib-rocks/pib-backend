"""DepthAI pipeline builders extracted from CameraNode."""

from . import stereo as stereo

from depthai_nodes.node import FrameCropper, GatherData, ParsingNeuralNetwork

from .face_crop import (
    FACE_CROP_EXCLUSIVITY_RULE,
    FACE_CROP_TRANSLATORS,
    FACE_CROP_PADDING,
    FACE_DETECTOR_MODEL_ID,
    GAZE_MODEL_ID,
    HEAD_POSE_MODEL_ID,
    face_crop_classifier_id,
    is_face_crop_composite,
    is_gaze_composite,
    packet_timestamp,
    translate_gaze,
)
from .hand_tracking import (
    FAST_MODEL_ID,
    MANIP_CROP_INSET_PIXELS,
    build_fast_hand_graph,
    fit_manip_crop,
)
from .imitation import (
    LANDMARK_COUNT as IMITATION_LANDMARK_COUNT,
    PALM_PADDING as IMITATION_PALM_PADDING,
    ProcessDetections,
    ProcessGazeEyes,
    box_from_points,
)
from .parsed_detections import published_field, translate_detections
from .task_archives import labels_for_model


class PipelineBuildersMixin:
    def _read_stereo_mode(self):
        mode = stereo.os.environ.get("PIB_CAMERA_STEREO", "auto").strip().lower()
        if mode not in stereo.STEREO_MODES:
            self.get_logger().warning(
                f"Invalid PIB_CAMERA_STEREO={mode!r}; using 'off'."
            )
            return "off"
        return mode

    def _read_stereo_timeout(self):
        value = stereo.os.environ.get(
            "PIB_CAMERA_STEREO_TIMEOUT", str(stereo.DEFAULT_STEREO_TIMEOUT)
        )
        try:
            timeout = float(value)
            if timeout < 0 or not stereo.math.isfinite(timeout):
                raise ValueError
            return timeout
        except ValueError:
            self.get_logger().warning(
                "Invalid PIB_CAMERA_STEREO_TIMEOUT; using 5.0 seconds."
            )
            return stereo.DEFAULT_STEREO_TIMEOUT

    def _read_hand_startup_grace(self):
        value = stereo.os.environ.get(
            "PIB_HAND_STARTUP_GRACE", str(stereo.DEFAULT_HAND_STARTUP_GRACE)
        )
        try:
            grace = float(value)
            if grace < 0 or not stereo.math.isfinite(grace):
                raise ValueError
            return grace
        except ValueError:
            self.get_logger().warning(
                "Invalid PIB_HAND_STARTUP_GRACE; using 5.0 seconds."
            )
            return stereo.DEFAULT_HAND_STARTUP_GRACE

    def _build_pipeline(self, include_stereo):
        """Build one colour pipeline, optionally including the stereo path."""
        self.pipeline = stereo.dai.Pipeline()
        self.camRgb = self.pipeline.create(stereo.dai.node.Camera)
        imitation_active = any(
            active.model.model_id == "imitation"
            for active in getattr(self, "_pipeline_models", ())
        )
        if imitation_active:
            self.camRgb.build(
                stereo.dai.CameraBoardSocket.CAM_A,
                sensorFps=stereo.IMITATION_FPS,
            )
        else:
            self.camRgb.build(stereo.dai.CameraBoardSocket.CAM_A)
        self.isp_out = self.camRgb.requestIspOutput()
        self.queue = self.isp_out.createOutputQueue(
            maxSize=stereo.COLOR_OUTPUT_QUEUE_DEPTH, blocking=False
        )
        self.depth_queue = None
        self.imu_queue = None
        self.imu_available = False
        self._imu_last_received_monotonic = None
        self._imu_last_sequence = None
        self._imu_last_device_stamp_ns = None
        # A new pipeline is a new device session, so the device clock restarts
        # near zero. Both the throttle deadline and the offset window describe
        # the previous session: keeping the deadline would drop every report
        # until the restarted device clock caught up, and keeping the offsets
        # would stamp with an offset measured against a clock that no longer
        # exists.
        self._imu_throttle = stereo.PublishRateThrottle()
        self._imu_clock_offset = stereo.ClockOffsetEstimator()
        self._imu_publish_times = stereo.deque(maxlen=stereo.IMU_RATE_WINDOW)
        self.nn_queues = {}
        self._pending_nn_packets = {}
        self.parsed_model_ids = set()
        self._parsed_error_counts = {}
        self.hand_decoder_queue = None
        self.hand_palm_queue = None
        self.hand_roi_queue = None
        self.hand_landmark_queue = None
        self.hand_landmark_config_queue = None
        self.hand_landmark_input_size = 0
        self.hand_source_size = (0, 0)
        self._pending_hand_decoder_packet = None
        self.imitation_queue = None
        self.hand_mp_pairs = {}
        self._pending_imitation_packet = None
        self.imitation_source_size = (0, 0)
        self.face_crop_detection_queue = None
        self.face_crop_result_queue = None
        self.face_crop_pairs = {}
        self.face_crop_source_size = (0, 0)
        self._pending_face_crop_packet = None
        self.face_crop_model_id = None
        self.face_crop_translator = None
        self.hand_mp_detection_queue = None
        self.hand_mp_landmark_queue = None
        self.hand_mp_source_size = (0, 0)
        self.hand_fast_queue = None
        self.hand_fast_source_size = (0, 0)
        self._pending_hand_fast_packet = None
        self._colour_branch_capture_queues = {}
        self._pending_colour_branch_packet = None
        self._colour_branch_field = None
        if hasattr(self, "_pending_hands"):
            self._pending_hands.clear()
        else:
            self._pending_hands = stereo.deque()
        self._reset_hand_stage_counters()
        self._reset_imitation_stage_counters()

        if include_stereo:
            self._init_stereo_depth()

        for active_model in getattr(self, "_pipeline_models", []):
            model = active_model.model
            if model.model_id == "hand_tracking":
                self._build_hand_pipeline(model)
                continue
            if model.model_id == "hand_tracking_mp":
                self._build_hand_mp_pipeline(model)
                continue
            if model.model_id == FAST_MODEL_ID:
                self._build_hand_fast_pipeline(model)
                continue
            if model.model_id == "imitation":
                self._build_imitation_pipeline(model)
                continue
            if is_face_crop_composite(model):
                if is_gaze_composite(model.artifact_ids):
                    self._build_gaze_pipeline(model)
                else:
                    self._build_face_crop_pipeline(model)
                continue
            self._build_single_network_pipeline(model)

    def _published_frame_size(self):
        """Width and height of the frame the UI displays."""
        width = getattr(self, "preview_width", None)
        height = getattr(self, "preview_height", None)
        if (
            isinstance(width, bool)
            or isinstance(height, bool)
            or not isinstance(width, (int, float))
            or not isinstance(height, (int, float))
            or width <= 0
            or height <= 0
        ):
            return (stereo.PUBLISHED_FRAME_WIDTH, stereo.PUBLISHED_FRAME_HEIGHT)
        return (int(width), int(height))

    def _colour_branch_size(self, model_id):
        """Chosen colour-branch size for one model.

        The hand chains keep a short branch so the palm warp stays within 2:1.
        hand_tracking_fast uses that 256x144 output rather than the 2104x1560
        ISP frame, which cannot be warped to the palm input in one step.
        Every other model shares the full-field branch. Both sizes have the
        published frame's aspect; the model's own input is a later stretch.
        """
        if model_id in ("hand_tracking", FAST_MODEL_ID):
            return (stereo.HAND_NN_WIDTH, stereo.HAND_NN_HEIGHT)
        return (stereo.COLOUR_BRANCH_WIDTH, stereo.COLOUR_BRANCH_HEIGHT)

    def _require_published_aspect(self, width, height):
        """Refuse a branch whose aspect is not the published frame's."""
        published_width, published_height = self._published_frame_size()
        if (
            width <= 0
            or height <= 0
            or width * published_height != height * published_width
        ):
            message = (
                f"Refusing camera branch {width}x{height}: its aspect does not "
                f"match the published frame {published_width}x{published_height}"
            )
            self.get_logger().error(message)
            raise ValueError(message)

    def _request_camera_branch(self, size, frame_type=None, fps=None):
        """Request a colour branch in the published frame's aspect.

        ``size`` is a chosen resolution. A size whose aspect differs from
        ``preview_width`` / ``preview_height`` is refused: the camera would
        deliver a centred crop, and a later per-axis scale cannot undo it.
        The model's own input size is produced afterwards by an ImageManip
        with ResizeMode.STRETCH.
        """
        width, height = int(size[0]), int(size[1])
        self._require_published_aspect(width, height)
        if frame_type is None:
            frame_type = stereo.dai.ImgFrame.Type.BGR888p
        request = {"type": frame_type}
        if fps is not None:
            request["fps"] = fps
        output = self.camRgb.requestOutput((width, height), **request)
        if output is None:
            raise RuntimeError(
                f"Camera cannot provide a {width}x{height} {frame_type} branch output"
            )
        return output

    def _stretch_to_network_input(self, source, width, height, frame_type):
        """Stretch a published-aspect branch to a model's own input size."""
        manip = self.pipeline.create(stereo.dai.node.ImageManip)
        channels = 1 if frame_type == stereo.dai.ImgFrame.Type.GRAY8 else 3
        manip.setMaxOutputFrameSize(int(width) * int(height) * channels)
        manip.initialConfig.setOutputSize(
            int(width),
            int(height),
            mode=stereo.dai.ImageManipConfig.ResizeMode.STRETCH,
        )
        manip.initialConfig.setFrameType(frame_type)
        self._relax_branch_input(manip.inputImage)
        source.link(manip.inputImage)
        return manip

    def _build_single_network_pipeline(self, model):
        """Build a parsed task when registered, preserving the plain fallback."""
        archive = stereo.create_archive(model.model_id, model.blob_path)
        frame_type = stereo.dai.ImgFrame.Type.BGR888p
        if model.model_id == stereo.QR_MODEL_ID:
            stereo.validate_qr_blob(stereo.dai.OpenVINO.Blob(model.blob_path))
            frame_type = stereo.dai.ImgFrame.Type.GRAY8
        camera_branch = self._request_camera_branch(
            self._colour_branch_size(model.model_id), frame_type=frame_type
        )
        if getattr(self, "_colour_branch_capture", False):
            queues = getattr(self, "_colour_branch_capture_queues", None)
            if queues is None:
                queues = {}
                self._colour_branch_capture_queues = queues
            queues[model.model_id] = camera_branch.createOutputQueue(
                maxSize=1, blocking=False
            )
        nn_input = self._stretch_to_network_input(
            camera_branch,
            model.input_width,
            model.input_height,
            frame_type,
        ).out
        if archive is not None:
            neural_network = self.pipeline.create(stereo.ParsingNeuralNetwork).build(
                nn_input, archive
            )
            self.parsed_model_ids.add(model.model_id)
        else:
            neural_network = self.pipeline.create(stereo.dai.node.NeuralNetwork)
            neural_network.setBlobPath(model.blob_path)
            neural_network.setNumShavesPerInferenceThread(model.shaves)
            self._relax_branch_input(neural_network.input)
            nn_input.link(neural_network.input)
        self.nn_queues[model.model_id] = neural_network.out.createOutputQueue(
            maxSize=stereo.BRANCH_OUTPUT_QUEUE_DEPTH, blocking=False
        )

    def _configure_hand_manip(
        self,
        manip,
        output_width,
        output_height,
        source_width,
        source_height,
    ):
        """Configure one hand ImageManip from the branch's real dimensions."""
        crop = fit_manip_crop(source_width, source_height, output_width, output_height)
        manip.setMaxOutputFrameSize(crop.output_width * crop.output_height * 3)
        manip.initialConfig.setOutputSize(crop.output_width, crop.output_height)
        manip.initialConfig.setFrameType(stereo.dai.ImgFrame.Type.BGR888p)
        # An ImageManip without an explicit initial crop validates the default
        # rect it carries against each incoming frame, and rejects the frame
        # whenever that rect does not fit - including while the landmark manip
        # waits for its first per-frame config.  The rect is centred, inside the
        # measured source, and it is never rotated: only the per-frame landmark
        # config carries an angle, because only the landmark model needs its
        # crop aligned to the palm.
        rotated = stereo.dai.RotatedRect()
        rotated.center.x = crop.center_x
        rotated.center.y = crop.center_y
        rotated.size.width = crop.width
        rotated.size.height = crop.height
        rotated.angle = 0.0
        manip.initialConfig.addCropRotatedRect(rotated, True)
        return crop

    def _hand_chain_is_built(self):
        requested = any(
            active.model.model_id == "hand_tracking"
            for active in getattr(self, "_pipeline_models", ())
        )
        if not requested:
            return False
        return all(
            queue is not None
            for queue in (
                self.hand_palm_queue,
                self.hand_decoder_queue,
                self.hand_roi_queue,
                self.hand_landmark_queue,
                self.hand_landmark_config_queue,
            )
        )

    def _hand_chain_is_flowing(self):
        return self._hand_chain_is_built() and all(
            self.hand_stage_counters[stage] > 0
            for stage in (
                "colour_isp",
                "palm_detector_nn",
                "decoding_nn",
                "decoding_result",
                "image_manip_config",
                "image_manip_roi",
                "hand_landmark_nn",
                "post_processing",
                "publish",
            )
        )

    def _hand_mp_chain_is_built(self):
        requested = any(
            active.model.model_id == "hand_tracking_mp"
            for active in getattr(self, "_pipeline_models", ())
        )
        if not requested:
            return False
        return getattr(self, "hand_mp_landmark_queue", None) is not None

    def _imitation_chain_is_built(self):
        requested = any(
            active.model.model_id == "imitation"
            for active in getattr(self, "_pipeline_models", ())
        )
        if not requested:
            return False
        return getattr(self, "imitation_queue", None) is not None

    def _face_crop_chain_is_built(self):
        face_crop_id = getattr(self, "face_crop_model_id", None)
        requested = any(
            active.model.model_id == face_crop_id
            for active in getattr(self, "_pipeline_models", ())
        )
        if not requested:
            return False
        return all(
            queue is not None
            for queue in (
                getattr(self, "face_crop_detection_queue", None),
                getattr(self, "face_crop_result_queue", None),
            )
        )

    def _imitation_chain_is_flowing(self):
        return self._imitation_chain_is_built() and all(
            self.imitation_stage_counters[stage] > 0
            for stage in (
                "palm_detector_nn",
                "decoding_nn",
                "decoding_result",
                "publish",
            )
        )

    def _relax_branch_input(self, node_input):
        """Drop frames on a branch input instead of back-pressuring the camera."""
        try:
            node_input.setBlocking(False)
        except Exception:
            self._warn_hand_once("Branch input blocking mode is not configurable.")
        for setter_name in ("setMaxSize", "setQueueSize"):
            setter = getattr(node_input, setter_name, None)
            if setter is None:
                continue
            try:
                setter(stereo.BRANCH_INPUT_QUEUE_DEPTH)
                return
            except Exception:
                continue
        self._warn_hand_once("Branch input queue depth is not configurable.")

    def _build_hand_pipeline(self, composite):
        """Add palm resize/detect/decode and dynamic hand ROI landmarks."""
        artifacts = {
            model_id: self.model_registry.get(model_id)
            for model_id in composite.artifact_ids
        }
        # The reference's models (pib-rocks/imitation).  The post-processing and
        # the host-side decoding below belong together: this blob emits the
        # ``result`` records the decoder parses, and it caps them at top-2 on the
        # device, which is also why the decoder expects exactly two records.
        # The zoo's own three-blob set.  The decoding head is compiled without
        # -ip U8 so its float tensors match the detector's FP16 outputs; with the
        # image-input default it emitted a constant score and unusable geometry.
        palm = artifacts["palm_detection_128x128"]
        decoder = artifacts["palm_detection_128x128_decoding"]
        landmark = artifacts["hand_landmark_224x224"]

        # One downscaled camera stream feeds the palm chain and the landmark
        # manip, both non-blocking. Requesting two identical Camera outputs
        # exceeds the OAK-D Lite camera-output budget once the colour output is
        # present.
        hand_size = self._colour_branch_size("hand_tracking")
        hand_tap = self._request_camera_branch(hand_size)
        # Every manip crops the frames this branch really carries, so the crops
        # follow the delivered size instead of the requested one.
        self.hand_source_size = self._branch_output_size(hand_tap, hand_size)
        source_width, source_height = self.hand_source_size

        palm_manip = self.pipeline.create(stereo.dai.node.ImageManip)
        self._configure_hand_manip(
            palm_manip,
            palm.input_width,
            palm.input_height,
            source_width,
            source_height,
        )
        self._relax_branch_input(palm_manip.inputImage)
        hand_tap.link(palm_manip.inputImage)

        palm_nn = self.pipeline.create(stereo.dai.node.NeuralNetwork)
        palm_nn.setBlobPath(palm.blob_path)
        palm_nn.setNumShavesPerInferenceThread(palm.shaves)
        self._relax_branch_input(palm_nn.input)
        palm_manip.out.link(palm_nn.input)
        self.hand_palm_queue = palm_nn.out.createOutputQueue(
            maxSize=stereo.BRANCH_OUTPUT_QUEUE_DEPTH, blocking=False
        )

        decoder_nn = self.pipeline.create(stereo.dai.node.NeuralNetwork)
        decoder_nn.setBlobPath(decoder.blob_path)
        decoder_nn.setNumShavesPerInferenceThread(decoder.shaves)
        # Preserve the palm NNData packet so DepthAI can map its named output
        # tensors to the decoder blob's named inputs.
        palm_nn.out.link(decoder_nn.input)
        self.hand_decoder_queue = decoder_nn.out.createOutputQueue(
            maxSize=stereo.BRANCH_OUTPUT_QUEUE_DEPTH, blocking=False
        )

        landmark_manip = self.pipeline.create(stereo.dai.node.ImageManip)
        landmark_crop = self._configure_hand_manip(
            landmark_manip,
            landmark.input_width,
            landmark.input_height,
            source_width,
            source_height,
        )
        # Landmark tensors are normalized against the size the crop really
        # delivers, so the mapping back to the frame follows it too.
        self.hand_landmark_input_size = landmark_crop.output_width
        # This manip only consumes an image once the host has sent a crop
        # config. Its non-blocking input discards images while idle, so sharing
        # the hand tap cannot back-pressure the palm or colour paths.
        self._relax_branch_input(landmark_manip.inputImage)
        hand_tap.link(landmark_manip.inputImage)
        landmark_manip.inputConfig.setWaitForMessage(True)
        self.hand_landmark_config_queue = landmark_manip.inputConfig.createInputQueue(
            maxSize=16, blocking=False
        )
        self.hand_roi_queue = landmark_manip.out.createOutputQueue(
            maxSize=stereo.BRANCH_OUTPUT_QUEUE_DEPTH, blocking=False
        )

        landmark_nn = self.pipeline.create(stereo.dai.node.NeuralNetwork)
        landmark_nn.setBlobPath(landmark.blob_path)
        landmark_nn.setNumShavesPerInferenceThread(landmark.shaves)
        landmark_manip.out.link(landmark_nn.input)
        # Stays blocking: _pending_hands expects one landmark result per crop
        # config, and a dropped result would stall the pairing permanently.
        self.hand_landmark_queue = landmark_nn.out.createOutputQueue()

    def _build_imitation_pipeline(self, composite):
        """Add the official parsed palm, full-frame crop, and landmark graph."""
        artifact_ids = set(composite.artifact_ids)
        required_ids = {"palm_detection_sh4", "hand_landmark_full_sh4"}
        if not required_ids.issubset(artifact_ids):
            raise ValueError("imitation composite is missing its detector or landmark")
        # The composite's decoder is for the legacy raw-NN path. The parsed
        # archive graph decodes palms itself and must not allocate that blob.
        palm = self.model_registry.get("palm_detection_sh4")
        landmark = self.model_registry.get("hand_landmark_full_sh4")
        detection_archive = stereo.create_palm_archive(palm.blob_path)
        landmark_archive = stereo.create_landmark_archive(landmark.blob_path)

        detector_width = detection_archive.getInputWidth()
        detector_height = detection_archive.getInputHeight()
        landmark_width = landmark_archive.getInputWidth()
        landmark_height = landmark_archive.getInputHeight()

        detector_resize = self.pipeline.create(stereo.dai.node.ImageManip)
        detector_resize.setMaxOutputFrameSize(detector_width * detector_height * 3)
        detector_resize.initialConfig.setOutputSize(
            detector_width,
            detector_height,
            mode=stereo.dai.ImageManipConfig.ResizeMode.STRETCH,
        )
        detector_resize.initialConfig.setFrameType(stereo.dai.ImgFrame.Type.BGR888p)
        # The full 16:9 field of view is intentionally stretched to square. This
        # squeezes hands horizontally by about 1.78 and may reduce palm score,
        # but avoids the field-of-view loss of a square camera crop.
        #
        # The neural branches must NOT tap requestIspOutput(): that is the raw,
        # full-resolution ISP stream which also feeds the host queue publishing
        # /camera_topic. Hanging two device consumers on it froze the whole
        # pipeline (last_flowing=none) and took the camera image in Cerebra with
        # it. The Luxonis reference asks the camera for its own sized, rate
        # limited output instead - three consumers on THAT are fine, the running
        # example has two device consumers plus a host node on one output.
        imitation_source = self._request_camera_branch(
            self._colour_branch_size("imitation"),
            fps=stereo.IMITATION_FPS,
        )
        self.imitation_source_size = (
            stereo.IMITATION_SOURCE_WIDTH,
            stereo.IMITATION_SOURCE_HEIGHT,
        )
        imitation_source.link(detector_resize.inputImage)

        detection_nn = self.pipeline.create(stereo.ParsingNeuralNetwork).build(
            detector_resize.out,
            detection_archive,
        )
        detections_processor = self.pipeline.create(ProcessDetections).build(
            detections_input=detection_nn.out,
            padding=IMITATION_PALM_PADDING,
            target_size=(landmark_width, landmark_height),
        )

        cropper = (
            self.pipeline.create(stereo.FrameCropper)
            .fromManipConfigs(
                inputManipConfigs=detections_processor.config_output,
                maxOutputFrameSize=landmark_width * landmark_height * 3,
                waitForConfig=True,
            )
            .build(imitation_source)
        )
        # The crop runs in a device Script node whose ImageManip is built with
        # ``inputConfig.setWaitForMessage(True)``: while no palm detection
        # arrives there is no config, and the node keeps every frame it is
        # handed. Those frames belong to the camera's shared frame pool, so a
        # long stretch without a detection exhausts the pool and stops the WHOLE
        # device - the palm branch, the preview and /camera_topic with it. That
        # is the freeze named in PR-1778: measured 1208 frames at 8 Hz with zero
        # detections, then every stage stood still. Dropping frames instead of
        # blocking keeps the camera alive while nothing is detected, and a held
        # frame could not have produced a crop anyway.
        cropper_input = getattr(
            getattr(cropper, "_cropper_image_manip", None), "inputImage", None
        )
        if cropper_input is None:
            self.get_logger().warning(
                "stereo.FrameCropper internals changed: cannot stop it from holding "
                "frames while no detection arrives"
            )
        else:
            cropper_input.setMaxSize(1)
            cropper_input.setBlocking(False)
        pose_nn = self.pipeline.create(stereo.ParsingNeuralNetwork).build(
            cropper.out,
            landmark_archive,
        )
        gather_data = self.pipeline.create(stereo.GatherData).build(
            cameraFps=stereo.IMITATION_FPS,
            inputData=pose_nn.outputs,
            inputReference=detection_nn.out,
        )
        self.imitation_queue = gather_data.out.createOutputQueue(
            maxSize=stereo.BRANCH_OUTPUT_QUEUE_DEPTH, blocking=False
        )
        self.imitation_source_size = (detector_width, detector_height)

    def _build_face_crop_pipeline(self, composite):
        """Add parsed YuNet detection, timestamped crops, and a raw classifier."""
        classifier_id = face_crop_classifier_id(composite.artifact_ids)
        if self.face_crop_model_id is not None:
            raise ValueError(FACE_CROP_EXCLUSIVITY_RULE)
        self.face_crop_model_id = composite.model_id
        self.face_crop_translator = FACE_CROP_TRANSLATORS[classifier_id]
        detector = self.model_registry.get(FACE_DETECTOR_MODEL_ID)
        classifier = self.model_registry.get(classifier_id)
        detection_archive = stereo.create_archive(detector.model_id, detector.blob_path)
        if detection_archive is None:
            raise ValueError("YuNet parser archive is unavailable")

        detector_width = detection_archive.getInputWidth()
        detector_height = detection_archive.getInputHeight()
        classifier_width = classifier.input_width
        classifier_height = classifier.input_height

        detector_resize = self.pipeline.create(stereo.dai.node.ImageManip)
        detector_resize.setMaxOutputFrameSize(detector_width * detector_height * 3)
        detector_resize.initialConfig.setOutputSize(
            detector_width,
            detector_height,
            mode=stereo.dai.ImageManipConfig.ResizeMode.STRETCH,
        )
        detector_resize.initialConfig.setFrameType(stereo.dai.ImgFrame.Type.BGR888p)

        # Keep the full 16:9 field of view and the sensor's existing rate. A
        # second fps request on this camera leaves the raw ISP colour queue empty.
        face_source = self._request_camera_branch(
            self._colour_branch_size(composite.model_id)
        )
        self.face_crop_source_size = (
            stereo.FACE_CROP_SOURCE_WIDTH,
            stereo.FACE_CROP_SOURCE_HEIGHT,
        )
        face_source.link(detector_resize.inputImage)

        detection_nn = self.pipeline.create(stereo.ParsingNeuralNetwork).build(
            detector_resize.out,
            detection_archive,
        )
        detections_processor = self.pipeline.create(ProcessDetections).build(
            detections_input=detection_nn.out,
            padding=FACE_CROP_PADDING,
            target_size=(classifier_width, classifier_height),
            # Square in pixels with a fractional padding, so the face fills the crop
            # the way the classifier expects. The hand chains keep the normalised
            # variant deliberately: switching them is a separate, measured change.
            square_in_pixels=True,
            source_size=self.face_crop_source_size,
        )
        cropper = (
            self.pipeline.create(stereo.FrameCropper)
            .fromManipConfigs(
                inputManipConfigs=detections_processor.config_output,
                maxOutputFrameSize=classifier_width * classifier_height * 3,
                waitForConfig=True,
            )
            .build(face_source)
        )
        cropper_input = getattr(
            getattr(cropper, "_cropper_image_manip", None), "inputImage", None
        )
        if cropper_input is None:
            self.get_logger().warning(
                "stereo.FrameCropper internals changed: cannot bound face-crop frames"
            )
        else:
            cropper_input.setMaxSize(stereo.FACE_CROPPER_QUEUE)
            cropper_input.setBlocking(False)

        classifier_nn = self.pipeline.create(stereo.dai.node.NeuralNetwork)
        classifier_nn.setBlobPath(classifier.blob_path)
        classifier_nn.setNumShavesPerInferenceThread(classifier.shaves)
        cropper.out.link(classifier_nn.input)

        self.face_crop_detection_queue = detection_nn.out.createOutputQueue(
            maxSize=stereo.FACE_CROP_PAIR_WINDOW,
            blocking=False,
        )
        self.face_crop_result_queue = classifier_nn.out.createOutputQueue(
            maxSize=stereo.FACE_CROP_PAIR_WINDOW,
            blocking=False,
        )

    def _build_gaze_pipeline(self, composite):
        """Add YuNet, geometric eye crops, head pose, and named-input gaze."""
        if not is_gaze_composite(composite.artifact_ids):
            raise ValueError("gaze composite is missing a required artifact")
        if self.face_crop_model_id is not None:
            raise ValueError(FACE_CROP_EXCLUSIVITY_RULE)
        self.face_crop_model_id = composite.model_id
        self.face_crop_translator = translate_gaze

        detector = self.model_registry.get(FACE_DETECTOR_MODEL_ID)
        head_pose = self.model_registry.get(HEAD_POSE_MODEL_ID)
        gaze = self.model_registry.get(GAZE_MODEL_ID)
        detection_archive = stereo.create_archive(detector.model_id, detector.blob_path)
        if detection_archive is None:
            raise ValueError("YuNet parser archive is unavailable")

        detector_width = detection_archive.getInputWidth()
        detector_height = detection_archive.getInputHeight()
        detector_resize = self.pipeline.create(stereo.dai.node.ImageManip)
        detector_resize.setMaxOutputFrameSize(detector_width * detector_height * 3)
        detector_resize.initialConfig.setOutputSize(
            detector_width,
            detector_height,
            mode=stereo.dai.ImageManipConfig.ResizeMode.STRETCH,
        )
        detector_resize.initialConfig.setFrameType(stereo.dai.ImgFrame.Type.BGR888p)
        self._relax_branch_input(detector_resize.inputImage)

        # The camera must already be built before requesting this branch. It feeds
        # YuNet and all three croppers, so every consumer is bounded and
        # non-blocking to keep one slow path from back-pressuring the camera.
        face_source = self._request_camera_branch(
            self._colour_branch_size(composite.model_id)
        )
        self.face_crop_source_size = (
            stereo.FACE_CROP_SOURCE_WIDTH,
            stereo.FACE_CROP_SOURCE_HEIGHT,
        )
        face_source.link(detector_resize.inputImage)

        detection_nn = self.pipeline.create(stereo.ParsingNeuralNetwork).build(
            detector_resize.out,
            detection_archive,
        )
        face_processor = self.pipeline.create(ProcessDetections).build(
            detections_input=detection_nn.out,
            padding=FACE_CROP_PADDING,
            target_size=(head_pose.input_width, head_pose.input_height),
            square_in_pixels=True,
            source_size=self.face_crop_source_size,
        )
        eye_processor = self.pipeline.create(ProcessGazeEyes).build(
            detections_input=detection_nn.out,
            target_size=(gaze.input_width, gaze.input_height),
        )
        self._relax_branch_input(face_processor.detections_input)
        self._relax_branch_input(eye_processor.detections_input)

        def build_cropper(config_output, width, height):
            cropper = (
                self.pipeline.create(stereo.FrameCropper)
                .fromManipConfigs(
                    inputManipConfigs=config_output,
                    maxOutputFrameSize=width * height * 3,
                    waitForConfig=True,
                )
                .build(face_source)
            )
            cropper_input = getattr(
                getattr(cropper, "_cropper_image_manip", None),
                "inputImage",
                None,
            )
            if cropper_input is None:
                self.get_logger().warning(
                    "stereo.FrameCropper internals changed: cannot bound gaze-crop frames"
                )
            else:
                self._relax_branch_input(cropper_input)
            return cropper

        face_cropper = build_cropper(
            face_processor.config_output,
            head_pose.input_width,
            head_pose.input_height,
        )
        left_eye_cropper = build_cropper(
            eye_processor.left_config_output,
            gaze.input_width,
            gaze.input_height,
        )
        right_eye_cropper = build_cropper(
            eye_processor.right_config_output,
            gaze.input_width,
            gaze.input_height,
        )

        head_pose_nn = self.pipeline.create(stereo.dai.node.NeuralNetwork)
        head_pose_nn.setBlobPath(head_pose.blob_path)
        head_pose_nn.setNumShavesPerInferenceThread(head_pose.shaves)
        self._relax_branch_input(head_pose_nn.input)
        face_cropper.out.link(head_pose_nn.input)

        gaze_nn = self.pipeline.create(stereo.dai.node.NeuralNetwork)
        gaze_nn.setBlobPath(gaze.blob_path)
        gaze_nn.setNumShavesPerInferenceThread(gaze.shaves)
        left_input = gaze_nn.inputs["left_eye_image"]
        right_input = gaze_nn.inputs["right_eye_image"]
        head_pose_input = gaze_nn.inputs["head_pose_angles"]
        for node_input in (left_input, right_input, head_pose_input):
            self._relax_branch_input(node_input)
        left_eye_cropper.out.link(left_input)
        right_eye_cropper.out.link(right_input)
        # DepthAI 3.6.1 maps the head-pose NNData output tensors directly to the
        # gaze blob's named vector input; no host decoder or Script node belongs
        # between these networks.
        head_pose_nn.out.link(head_pose_input)

        self.face_crop_detection_queue = detection_nn.out.createOutputQueue(
            maxSize=stereo.FACE_CROP_PAIR_WINDOW,
            blocking=False,
        )
        self.face_crop_result_queue = gaze_nn.out.createOutputQueue(
            maxSize=stereo.FACE_CROP_PAIR_WINDOW,
            blocking=False,
        )

    def _build_hand_mp_pipeline(self, composite):
        """Add the Luxonis hand-pose reference pipeline (PR-1791).

        Palm detector through a parsing network (the anchors are decoded host-side,
        so no decoding blob is needed), a host node that turns every palm detection
        into a timestamped crop config, the device-side stereo.FrameCropper, and the hand
        landmarker. The landmark stage deliberately runs as a plain NeuralNetwork
        rather than through the parsing network: depthai-nodes' KeypointParser clips
        the landmark components into 0..1, and the third component is the
        hand-relative depth this chain has to publish.
        """
        artifact_ids = set(composite.artifact_ids)
        required = {"palm_detection_128x128", "hand_landmark_224x224"}
        if not required.issubset(artifact_ids):
            raise ValueError(
                "hand_tracking_mp composite is missing its palm detector or landmarker"
            )
        palm = self.model_registry.get("palm_detection_128x128")
        landmark = self.model_registry.get("hand_landmark_224x224")
        detection_archive = stereo.create_palm_archive(palm.blob_path)

        detector_width = detection_archive.getInputWidth()
        detector_height = detection_archive.getInputHeight()
        landmark_width = landmark.input_width
        landmark_height = landmark.input_height

        detector_resize = self.pipeline.create(stereo.dai.node.ImageManip)
        detector_resize.setMaxOutputFrameSize(detector_width * detector_height * 3)
        detector_resize.initialConfig.setOutputSize(
            detector_width,
            detector_height,
            mode=stereo.dai.ImageManipConfig.ResizeMode.STRETCH,
        )
        detector_resize.initialConfig.setFrameType(stereo.dai.ImgFrame.Type.BGR888p)

        # One rate-limited camera branch with the 16:9 field of view, exactly as in
        # the example (there: 768x768, which crops the field of view instead).
        # No explicit fps here. The camera already carries the raw ISP output that
        # publishes /camera_topic, and asking one output for a different rate than
        # that stream leaves the ISP with nothing to deliver: measured with the
        # node's own pipeline, the colour queue stayed empty until the model chain
        # was taken out, while the same branch at the sensor's rate works. The rate
        # of the chain is set by the crop/landmark pairing, not by throttling the
        # camera.
        hand_mp_source = self._request_camera_branch(
            self._colour_branch_size("hand_tracking_mp")
        )
        self.hand_mp_source_size = (
            stereo.HAND_MP_SOURCE_WIDTH,
            stereo.HAND_MP_SOURCE_HEIGHT,
        )
        hand_mp_source.link(detector_resize.inputImage)

        detection_nn = self.pipeline.create(stereo.ParsingNeuralNetwork).build(
            detector_resize.out, detection_archive
        )
        detections_processor = self.pipeline.create(ProcessDetections).build(
            detections_input=detection_nn.out,
            padding=IMITATION_PALM_PADDING,
            target_size=(landmark_width, landmark_height),
        )

        cropper = (
            self.pipeline.create(stereo.FrameCropper)
            .fromManipConfigs(
                inputManipConfigs=detections_processor.config_output,
                maxOutputFrameSize=landmark_width * landmark_height * 3,
                waitForConfig=True,
            )
            .build(hand_mp_source)
        )
        # Frames held by the cropper come out of the camera's shared frame pool;
        # without this a long stretch without a detection stops the whole device
        # (measured for the imitation chain: 1208 frames, then everything stood).
        cropper_input = getattr(
            getattr(cropper, "_cropper_image_manip", None), "inputImage", None
        )
        if cropper_input is None:
            self.get_logger().warning(
                "stereo.FrameCropper internals changed: cannot stop it from holding frames"
            )
        else:
            cropper_input.setMaxSize(stereo.HAND_MP_CROPPER_QUEUE)
            cropper_input.setBlocking(False)

        # Plain network: the raw NNData is what carries the unclipped z.
        landmark_nn = self.pipeline.create(stereo.dai.node.NeuralNetwork)
        landmark_nn.setBlobPath(landmark.blob_path)
        landmark_nn.setNumShavesPerInferenceThread(landmark.shaves)
        cropper.out.link(landmark_nn.input)
        self.hand_landmark_input_size = landmark_width

        # Two host-side streams: the parsed palm detections (for the box and the
        # score) and the raw landmark results. They are paired by timestamp, which
        # is what removes the one-frame-at-a-stereo.time round trip.
        self.hand_mp_detection_queue = detection_nn.out.createOutputQueue(
            maxSize=stereo.HAND_MP_PAIR_WINDOW, blocking=False
        )
        self.hand_mp_landmark_queue = landmark_nn.out.createOutputQueue(
            maxSize=stereo.HAND_MP_PAIR_WINDOW, blocking=False
        )

    def _build_hand_fast_pipeline(self, composite):
        """Add the device-side palm, top-2 decoder and landmark chain.

        The Script computes both crop configurations and links them on the
        device. The only host queue is the detection result.
        """
        required = {
            "palm_detection_sh4",
            "pd_postprocessing_top2_sh1",
            "hand_landmark_full_sh4",
        }
        artifact_ids = set(composite.artifact_ids)
        if not required.issubset(artifact_ids):
            raise ValueError(
                "hand_tracking_fast composite is missing a palm, decoder, or "
                "landmark blob"
            )
        palm = self.model_registry.get("palm_detection_sh4")
        decoder = self.model_registry.get("pd_postprocessing_top2_sh1")
        landmark = self.model_registry.get("hand_landmark_full_sh4")
        branch_size = self._colour_branch_size(FAST_MODEL_ID)
        source = self._request_camera_branch(branch_size)
        self.hand_fast_source_size = self._branch_output_size(source, branch_size)
        source_width, source_height = self.hand_fast_source_size
        self.hand_fast_queue = build_fast_hand_graph(
            self.pipeline,
            source,
            palm,
            decoder,
            landmark,
            source_width,
            source_height,
        )

    def _hand_fast_chain_is_built(self):
        requested = any(
            active.model.model_id == FAST_MODEL_ID
            for active in getattr(self, "_pipeline_models", ())
        )
        if not requested:
            return False
        return getattr(self, "hand_fast_queue", None) is not None

    def _stop_pipeline(self):
        pipeline_lock = getattr(self, "_pipeline_lock", None)
        if pipeline_lock is None:
            pipeline_lock = stereo.threading.RLock()
            self._pipeline_lock = pipeline_lock
        with pipeline_lock, stereo.CameraNode._device_lifecycle_lock:
            pipeline = self.pipeline
            # True when there is nothing to stop. A pipeline that never reports
            # stopped used to return False before this cleanup, which left the
            # pipeline object and _device_owner in place. The next open then
            # failed with X_LINK_DEVICE_ALREADY_IN_USE for good.
            released = True
            if pipeline is not None:
                stop_failed = False
                released = False
                try:
                    pipeline.stop()
                except Exception as exc:
                    stop_failed = True
                    self.get_logger().warning(
                        f"Camera pipeline stop reported an error: {exc}"
                    )
                deadline = stereo.time.monotonic() + stereo.PIPELINE_STOP_TIMEOUT
                while stereo.time.monotonic() < deadline:
                    try:
                        if pipeline.isRunning() is not True:
                            released = True
                            break
                    except Exception:
                        released = not stop_failed
                        break
                    stereo.time.sleep(stereo.PIPELINE_STOP_POLL_INTERVAL)
                if not released:
                    self.get_logger().warning(
                        "Camera pipeline did not report stopped before timeout."
                    )
            owner_ref = stereo.CameraNode._device_owner
            if owner_ref is not None and owner_ref() is self:
                stereo.CameraNode._device_owner = None
            self.pipeline = None
            self.camRgb = None
            self.isp_out = None
            self.queue = None
            self.depth_queue = None
            self.imu_queue = None
            self.imu_available = False
            self.nn_queues = {}
            self._pending_nn_packets = {}
            self.hand_decoder_queue = None
            self.hand_palm_queue = None
            self.hand_roi_queue = None
            self.hand_landmark_queue = None
            self.hand_landmark_config_queue = None
            self.hand_landmark_input_size = 0
            self.hand_source_size = (0, 0)
            self._pending_hand_decoder_packet = None
            self.imitation_queue = None
            self.hand_mp_pairs = {}
            self._pending_imitation_packet = None
            self.imitation_source_size = (0, 0)
            self.face_crop_detection_queue = None
            self.face_crop_result_queue = None
            self.face_crop_pairs = {}
            self.face_crop_source_size = (0, 0)
            self._pending_face_crop_packet = None
            self.face_crop_model_id = None
            self.face_crop_translator = None
            self.hand_mp_detection_queue = None
            self.hand_mp_landmark_queue = None
            self.hand_mp_source_size = (0, 0)
            self.hand_fast_queue = None
            self.hand_fast_source_size = (0, 0)
            self._pending_hand_fast_packet = None
            self._colour_branch_capture_queues = {}
            self._pending_colour_branch_packet = None
            self._colour_branch_field = None
            if hasattr(self, "_pending_hands"):
                self._pending_hands.clear()
            return released

    def _start_pipeline(self, include_stereo):
        """Build and start a fresh pipeline with bounded retries."""
        pipeline_lock = getattr(self, "_pipeline_lock", None)
        if pipeline_lock is None:
            pipeline_lock = stereo.threading.RLock()
            self._pipeline_lock = pipeline_lock
        with pipeline_lock, stereo.CameraNode._device_lifecycle_lock:
            owner_ref = stereo.CameraNode._device_owner
            owner = owner_ref() if owner_ref is not None else None
            if owner is not None and owner is not self:
                self.get_logger().error(
                    "Camera pipeline start refused: this process already has "
                    "another OAK device holder."
                )
                return False
            if owner is self and self.pipeline is not None:
                self.get_logger().error(
                    "Camera pipeline start refused: the previous OAK holder "
                    "has not been released."
                )
                return False
            include_imu = True
            for attempt in range(stereo.PIPELINE_START_ATTEMPTS):
                try:
                    self._build_pipeline(include_stereo)
                    if include_imu:
                        include_imu = self._init_imu()
                        if not include_imu:
                            # Discard a graph in which IMU construction failed
                            # after creating a partial node.
                            self._build_pipeline(include_stereo)
                    self.pipeline.start()
                    stereo.CameraNode._device_owner = stereo.weakref.ref(self)
                    return True
                except Exception as exc:
                    self.get_logger().error(
                        "Camera pipeline build/start attempt "
                        f"{attempt + 1}/{stereo.PIPELINE_START_ATTEMPTS} failed: {exc}"
                    )
                    # False means the pipeline did not report stopped. The
                    # holder is already cleared, but the device may still be
                    # in use, so this attempt must not open another one.
                    # A later start can open it; retrying here is the
                    # X_LINK_DEVICE_ALREADY_IN_USE loop.
                    if not self._stop_pipeline():
                        return False
                    # Some OAK variants expose no IMU. If a graph containing
                    # stereo.dai.node.IMU is rejected at start, retry the same camera
                    # graph without it instead of taking down camera topics.
                    include_imu = False
                    if attempt + 1 < stereo.PIPELINE_START_ATTEMPTS:
                        stereo.time.sleep(stereo.PIPELINE_START_BACKOFF * (2**attempt))
            return False

    def _wait_for_color_frame(self, timeout):
        """Return the first measured colour packet, or None at the deadline."""
        deadline = stereo.time.monotonic() + timeout
        while True:
            packet = self.queue.tryGet()
            if packet is not None:
                return packet
            remaining = deadline - stereo.time.monotonic()
            if remaining <= 0:
                return None
            stereo.time.sleep(min(0.05, remaining))

    @staticmethod
    def _wait_for_queue_packet(queue, timeout):
        """Return the first packet from a non-colour queue before timeout."""
        deadline = stereo.time.monotonic() + timeout
        while True:
            packet = queue.tryGet()
            if packet is not None:
                return packet
            remaining = deadline - stereo.time.monotonic()
            if remaining <= 0:
                return None
            stereo.time.sleep(min(0.05, remaining))

    def _rebuild_models(self, active_models):
        self._pipeline_models = list(active_models)
        self._stop_pipeline()
        self.camera_available = self.init_pipeline()
        if self.camera_available:
            special_ids = {
                "hand_tracking",
                "imitation",
                "hand_tracking_mp",
                FAST_MODEL_ID,
                getattr(self, "face_crop_model_id", None),
            }
            missing = [
                active.model.model_id
                for active in self._pipeline_models
                if (
                    active.model.model_id == "hand_tracking"
                    and not self._hand_chain_is_built()
                )
                or (
                    active.model.model_id == "imitation"
                    and not self._imitation_chain_is_built()
                )
                or (
                    active.model.model_id == getattr(self, "face_crop_model_id", None)
                    and not self._face_crop_chain_is_built()
                )
                or (
                    active.model.model_id == "hand_tracking_mp"
                    and not self._hand_mp_chain_is_built()
                )
                or (
                    active.model.model_id == FAST_MODEL_ID
                    and not self._hand_fast_chain_is_built()
                )
                or (
                    active.model.model_id not in special_ids
                    and active.model.model_id not in self.nn_queues
                )
            ]
            if missing:
                self.get_logger().error(
                    "Camera pipeline started without requested model chain(s): "
                    + ", ".join(missing)
                )
                self._stop_pipeline()
                self.camera_available = False
        return self.camera_available

    def _verify_model_frames(self, timeout):
        requested_ids = {
            active.model.model_id for active in getattr(self, "_pipeline_models", ())
        }
        face_crop_id = getattr(self, "face_crop_model_id", None)
        special_ids = {
            "hand_tracking",
            "imitation",
            "hand_tracking_mp",
            FAST_MODEL_ID,
            face_crop_id,
        }
        if "hand_tracking" in requested_ids and not self._hand_chain_is_built():
            self.get_logger().error(
                "Cannot verify hand_tracking: the complete hand chain is absent"
            )
            return False
        if "imitation" in requested_ids and not self._imitation_chain_is_built():
            self.get_logger().error(
                "Cannot verify imitation: the gathered output queue is absent"
            )
            return False
        if face_crop_id in requested_ids and not self._face_crop_chain_is_built():
            self.get_logger().error(
                f"Cannot verify {face_crop_id}: the face-crop queues are absent"
            )
            return False
        if "hand_tracking_mp" in requested_ids and not self._hand_mp_chain_is_built():
            self.get_logger().error(
                "Cannot verify hand_tracking_mp: the reference chain is absent"
            )
            return False
        if FAST_MODEL_ID in requested_ids and not self._hand_fast_chain_is_built():
            self.get_logger().error(
                "Cannot verify hand_tracking_fast: the detection queue is absent"
            )
            return False
        if any(
            model_id not in special_ids and model_id not in self.nn_queues
            for model_id in requested_ids
        ):
            # A composite chain has no nn_queues entry: it is verified through its
            # own stage queues below. Without this, every composite other than the
            # two named ones failed verification before a single frame was read,
            # and the caller reported "no frames arrived".
            return False
        if stereo.QR_MODEL_ID in requested_ids:
            packet = self._wait_for_queue_packet(
                self.nn_queues[stereo.QR_MODEL_ID], timeout
            )
            if packet is None:
                self.get_logger().error(
                    f"{stereo.QR_MODEL_ID} chain started but its detector produced nothing"
                )
                return False
            self._pending_nn_packets[stereo.QR_MODEL_ID] = packet

        packet = self._wait_for_color_frame(timeout)
        if packet is None:
            return False
        self._pending_color_packet = packet
        if "hand_tracking" in requested_ids:
            palm_packet = self._wait_for_queue_packet(self.hand_palm_queue, timeout)
            if palm_packet is None:
                return False
            self._count_hand_stage("colour_isp")
            self._count_hand_stage("palm_detector_nn")
            packet = self._wait_for_queue_packet(self.hand_decoder_queue, timeout)
            if packet is None:
                return False
            self._pending_hand_decoder_packet = packet
            self._count_hand_stage("decoding_nn")
        if "imitation" in requested_ids:
            packet = self._wait_for_queue_packet(self.imitation_queue, timeout)
            if packet is None:
                return False
            self._pending_imitation_packet = packet
        if face_crop_id in requested_ids:
            # The classifier can only answer once a face sits inside a crop, so waiting
            # for its result would make the model unstartable in an empty room. The
            # detector stage is what proves the chain is up; the classifier stage is
            # proven by the first face that appears.
            packet = self._wait_for_queue_packet(
                self.face_crop_detection_queue, timeout
            )
            if packet is None:
                self.get_logger().error(
                    f"{face_crop_id} chain started but its detector produced nothing"
                )
                return False
            # Consumed, not stashed: the pairing code expects a classifier result in
            # _pending_face_crop_packet, and the detector stream runs at ~29 Hz.
            if self.face_crop_result_queue is not None and (
                self.face_crop_result_queue.tryGet() is None
            ):
                self.get_logger().warning(
                    "Face-crop chain is running (detector answering) but no face is "
                    "in view yet: the classifier stage is proven by the first "
                    "detected face."
                )
        if "hand_tracking_mp" in requested_ids:
            # The chain is only really running when the landmark stage answers;
            # waiting on the palm branch alone would accept a chain whose crops
            # never come back.
            packet = self._wait_for_queue_packet(self.hand_mp_landmark_queue, timeout)
            if packet is None:
                self.get_logger().error(
                    "hand_tracking_mp chain started but no landmark result arrived"
                )
                return False
            self._count_hand_stage("publish")
        if FAST_MODEL_ID in requested_ids:
            # The Script emits a result on every cycle, including a frame with
            # no hand, so this wait does not require a hand to be in view.
            packet = self._wait_for_queue_packet(self.hand_fast_queue, timeout)
            if packet is None:
                self.get_logger().error(
                    "hand_tracking_fast chain started but no detection result arrived"
                )
                return False
            self._pending_hand_fast_packet = packet
        return True

    def _revert_to_color_only(self):
        self._pipeline_models = []
        self._stop_pipeline()
        self.depth_available = False
        self.current_depth = None
        started = self._start_pipeline(include_stereo=False)
        self.camera_available = started
        if not started:
            return False
        return self._verify_model_frames(self.stereo_timeout)

    def _stereo_requested(self) -> bool:
        """Whether this pipeline should include the stereo depth path.

        ``on`` and ``off`` keep their meaning. ``auto`` now means "depth while
        idle": with a model running, the model branch and the depth path share the
        same camera/ISP budget, and none of the models needs the depth - the
        published z is hand-relative and comes from the landmarker. Depth is back
        as soon as the last model stops.
        """
        if self.stereo_mode == "off":
            return False
        if self.stereo_mode == "on":
            return True
        return not getattr(self, "_pipeline_models", [])

    def init_pipeline(self) -> bool:
        self.depth_available = False
        self.current_depth = None
        self._pending_color_packet = None

        if not self._stereo_requested():
            if not self._start_pipeline(include_stereo=False):
                self.get_logger().error(
                    "Camera not found: colour pipeline failed to start."
                )
                return False
            reason = (
                "mode=off"
                if self.stereo_mode == "off"
                else "a model is running (mode=auto)"
            )
            self.get_logger().warning(
                f"Stereo depth disabled - using colour-only pipeline ({reason})"
            )
            return True

        stereo_started = self._start_pipeline(include_stereo=True)
        if self.stereo_mode == "on":
            if not stereo_started:
                self.get_logger().error(
                    "Camera not found: stereo pipeline failed to start."
                )
                return False
            self.depth_available = True
            self.get_logger().info(
                "Stereo depth available - full colour + stereo pipeline active "
                "(mode=on)"
            )
            return True

        if stereo_started:
            try:
                first_packet = self._wait_for_color_frame(self.stereo_timeout)
            except Exception:
                first_packet = None
            if first_packet is not None:
                self._pending_color_packet = first_packet
                self.depth_available = True
                self.get_logger().info(
                    "Stereo depth available - full colour + stereo pipeline active "
                    "(mode=auto)"
                )
                return True

        self._stop_pipeline()
        if not self._start_pipeline(include_stereo=False):
            self.get_logger().error(
                "Camera not found: colour pipeline failed to start."
            )
            return False
        self.get_logger().warning(
            "Stereo depth unavailable - falling back to colour-only pipeline "
            "(depth disabled)"
        )
        return True
