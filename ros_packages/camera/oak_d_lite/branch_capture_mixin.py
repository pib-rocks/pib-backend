"""Colour branch field capture extracted from CameraNode."""

from . import stereo as stereo

from .parsed_detections import published_field


class BranchCaptureMixin:
    def _packet_transformation(self, packet):
        """Read a live ImgTransformation, the same source the hand chain uses."""
        getter = getattr(packet, "getTransformation", None)
        if not callable(getter):
            return None
        try:
            transformation = getter()
        except Exception:
            return None
        if transformation is None:
            return None
        valid = getattr(transformation, "isValid", None)
        if callable(valid):
            try:
                if valid() is False:
                    return None
            except Exception:
                return None
        return transformation

    @staticmethod
    def _size_pair(value):
        """Read a (width, height) pair from a device size object, if it is one."""
        if isinstance(value, (tuple, list)) and len(value) == 2:
            width, height = value
        else:
            width = getattr(value, "width", None)
            height = getattr(value, "height", None)
        if isinstance(width, bool) or isinstance(height, bool):
            return None
        if not isinstance(width, (int, float)) or not isinstance(height, (int, float)):
            return None
        if width <= 0 or height <= 0:
            return None
        return (int(width), int(height))

    def _branch_output_size(self, output, requested):
        """Read the dimensions a camera branch really delivers.

        The camera can hand back a stream whose size differs from the request,
        and every ImageManip crop on that branch has to match the frames that
        arrive rather than the ones that were asked for.
        """
        for getter_name in ("getSize", "getDimensions"):
            getter = getattr(output, getter_name, None)
            if getter is None:
                continue
            try:
                size = self._size_pair(getter())
            except Exception:
                size = None
            if size is not None:
                return size
        return (int(requested[0]), int(requested[1]))

    def _note_hand_branch_size(self, packet):
        """Track the hand branch size a device packet was actually produced from."""
        getter = getattr(packet, "getTransformation", None)
        if getter is None:
            return
        try:
            transformation = getter()
            source_getter = getattr(transformation, "getSourceSize", None)
            size = None if source_getter is None else self._size_pair(source_getter())
        except Exception:
            return
        if size is None or size == self.hand_source_size:
            return
        # Every crop is derived from this size, so a changed branch geometry has
        # to reach the landmark configs before the next frame is cropped.
        self._warn_hand_once(
            "Hand branch delivers "
            f"{size[0]}x{size[1]}, not {self.hand_source_size[0]}x"
            f"{self.hand_source_size[1]}; deriving hand crops from the "
            "delivered size."
        )
        self.hand_source_size = size

    def _colour_branch_capture_requested(self):
        """Return whether the operator asked to see the colour branch."""
        raw = stereo.os.environ.get("PIB_COLOUR_BRANCH_FIELD", "")
        if isinstance(raw, str) and raw.strip():
            self._colour_branch_capture_dir = raw.strip()
            return True
        if stereo.os.path.isfile(stereo.COLOUR_BRANCH_FIELD_REQUEST):
            self._colour_branch_capture_dir = stereo.COLOUR_BRANCH_FIELD_DIR
            return True
        return False

    def _plane_point(self, x, y):
        """Build a point the transformation remap can read."""
        point_type = getattr(stereo.dai, "Point2f", None)
        if not callable(point_type):
            return type("PlanePoint", (), {"x": float(x), "y": float(y)})()
        try:
            point = point_type(float(x), float(y))
        except TypeError:
            point = point_type()
            point.x = float(x)
            point.y = float(y)
            return point
        if getattr(point, "x", None) is None or getattr(point, "y", None) is None:
            try:
                point.x = float(x)
                point.y = float(y)
            except Exception:
                return type("PlanePoint", (), {"x": float(x), "y": float(y)})()
        return point

    def _point_xy(self, point):
        if isinstance(point, (tuple, list)) and len(point) >= 2:
            return float(point[0]), float(point[1])
        return float(point.x), float(point.y)

    def _remap_into_preview(self, source, preview, point):
        remap_from = getattr(preview, "remapPointFrom", None)
        if callable(remap_from):
            return remap_from(source, point)
        remap_to = getattr(source, "remapPointTo", None)
        if callable(remap_to):
            return remap_to(preview, point)
        return None

    def _field_from_packets(self, source_packet, frame_width, frame_height):
        """Field the source packet covers inside the published frame.

        The corners of the delivered image are remapped into the preview, then
        scaled into the published frame. The result is the window a fraction
        of that image occupies: origin plus size on each axis. Node outputs
        have no size getter here, so the size comes from the packet.
        """
        preview_packet = getattr(self, "_preview_packet", None)
        if source_packet is None or preview_packet is None:
            return None
        source = self._packet_transformation(source_packet)
        preview = self._packet_transformation(preview_packet)
        if source is None or preview is None:
            return None
        try:
            source_size = self._size_pair(source.getSize())
            preview_size = self._size_pair(preview.getSize())
            if source_size is None or preview_size is None:
                return None
            origin = self._remap_into_preview(source, preview, self._plane_point(0, 0))
            far = self._remap_into_preview(
                source,
                preview,
                self._plane_point(source_size[0], source_size[1]),
            )
            if origin is None or far is None:
                return None
            origin_xy = self._point_xy(origin)
            far_xy = self._point_xy(far)
            return stereo.published_field(
                origin_xy,
                far_xy,
                preview_size,
                (frame_width, frame_height),
            )
        except Exception:
            return None

    def _object_detection_field(self, packet, frame_width, frame_height):
        """Inverse-map object detections from the branch window.

        Horizontal boxes already match the published frame, including an
        object on the left or right border, so that axis stays the full
        width. The vertical axis uses the window the branch really covers.
        """
        derived = getattr(self, "_colour_branch_field", None)
        if derived is None:
            derived = self._field_from_packets(packet, frame_width, frame_height)
            if derived is not None:
                self._colour_branch_field = derived
        if derived is None:
            return (0.0, float(frame_width), 0.0, float(frame_height))
        return (0.0, float(frame_width), float(derived[2]), float(derived[3]))

    def _poll_colour_branch_capture(self):
        """Attach the capture to a node that is already running, then rebuild.

        Creating the request file must not start a second camera: this process
        already owns the device, and it rebuilds its own pipeline.
        """
        if getattr(self, "_colour_branch_capture", False):
            return False
        if getattr(self, "_colour_branch_capture_attempted", False):
            return False
        if not stereo.os.path.isfile(stereo.COLOUR_BRANCH_FIELD_REQUEST):
            return False
        self._colour_branch_capture_attempted = True
        with self._pipeline_lock:
            if not self._stop_pipeline():
                return False
            self._colour_branch_capture = True
            self._colour_branch_capture_dir = stereo.COLOUR_BRANCH_FIELD_DIR
            self._colour_branch_capture_written = False
            self.get_logger().info(
                "Colour branch field capture enabled; rebuilding the camera pipeline."
            )
            self.camera_available = self.init_pipeline()
        return True

    def _format_src_crops(self, transformation):
        getter = getattr(transformation, "getSrcCrops", None)
        if not callable(getter):
            return "unavailable"
        try:
            crops = list(getter())
        except Exception:
            return "unavailable"
        if not crops:
            return "none"
        parts = []
        for crop in crops:
            center = getattr(crop, "center", None)
            size = getattr(crop, "size", None)
            parts.append(
                "center=({0},{1}) size=({2},{3}) angle={4}".format(
                    getattr(center, "x", None),
                    getattr(center, "y", None),
                    getattr(size, "width", None),
                    getattr(size, "height", None),
                    getattr(crop, "angle", None),
                )
            )
        return "; ".join(parts)

    def _write_colour_branch_capture(self, model_id, packet):
        """Save the delivered branch beside the preview from this same tick."""
        preview_frame = self._decode_current_frame()
        if preview_frame is None or getattr(self, "_preview_packet", None) is None:
            self._pending_colour_branch_packet = (model_id, packet)
            return
        directory = getattr(self, "_colour_branch_capture_dir", None)
        if not directory:
            directory = stereo.COLOUR_BRANCH_FIELD_DIR
        stereo.os.makedirs(directory, exist_ok=True)
        frame = packet.getCvFrame()
        delivered = (int(frame.shape[1]), int(frame.shape[0]))
        transformation = self._packet_transformation(packet)
        transformation_size = None
        source_size = None
        crops = "unavailable"
        if transformation is not None:
            transformation_size = self._size_pair(
                getattr(transformation, "getSize", lambda: None)()
            )
            source_size = self._size_pair(
                getattr(transformation, "getSourceSize", lambda: None)()
            )
            crops = self._format_src_crops(transformation)
        frame_height, frame_width = preview_frame.shape[:2]
        field = self._field_from_packets(packet, frame_width, frame_height)
        if field is not None:
            self._colour_branch_field = field
        branch_path = stereo.os.path.join(directory, "colour-branch.png")
        preview_path = stereo.os.path.join(directory, "preview.png")
        report_path = stereo.os.path.join(directory, "field.txt")
        wrote_branch = stereo.cv2.imwrite(branch_path, frame)
        wrote_preview = stereo.cv2.imwrite(preview_path, preview_frame)
        report = (
            f"model={model_id}\n"
            f"delivered={delivered[0]}x{delivered[1]}\n"
            f"transformation_size={transformation_size}\n"
            f"source_size={source_size}\n"
            f"src_crops={crops}\n"
            f"published={frame_width}x{frame_height}\n"
            f"field={field}\n"
        )
        with open(report_path, "w", encoding="utf-8") as handle:
            handle.write(report)
        self._colour_branch_capture_written = True
        self._pending_colour_branch_packet = None
        self.get_logger().info(
            "Colour branch field capture wrote "
            f"{branch_path} and {preview_path} "
            f"(branch_image={wrote_branch}, preview_image={wrote_preview}). "
            f"{report.replace(chr(10), ' ')}"
        )

    def _drain_colour_branch_capture(self):
        if not getattr(self, "_colour_branch_capture", False):
            return
        if getattr(self, "_colour_branch_capture_written", False):
            return
        try:
            pending = getattr(self, "_pending_colour_branch_packet", None)
            if pending is not None:
                self._write_colour_branch_capture(pending[0], pending[1])
                return
            queues = getattr(self, "_colour_branch_capture_queues", {})
            for model_id, queue in queues.items():
                if queue is None:
                    continue
                packet = queue.tryGet()
                if packet is None:
                    continue
                self._write_colour_branch_capture(model_id, packet)
                return
        except Exception as exc:
            self.get_logger().warning(
                f"Colour branch field capture failed: {type(exc).__name__}: {exc}"
            )

    def _load_startup_camera_settings(self):
        """Load persisted camera settings from Flask on startup (PR-2140)."""
        base_url = stereo.os.environ.get("FLASK_API_BASE_URL", "").strip().rstrip("/")
        if not base_url:
            return
        try:
            with stereo.urlopen(f"{base_url}/camera-settings", timeout=2.0) as resp:
                data = stereo.json.loads(resp.read().decode("utf-8"))
        except Exception as exc:
            self.get_logger().warning(
                f"Startup camera settings unavailable from {base_url}: {exc}; using defaults."
            )
            return
        if not isinstance(data, dict):
            return
        res_x = data.get("resX")
        res_y = data.get("resY")
        if (
            isinstance(res_x, int)
            and isinstance(res_y, int)
            and res_x > 0
            and res_y > 0
        ):
            self.preview_width = res_x
            self.preview_height = res_y
        quality = data.get("qualityFactor")
        if isinstance(quality, int) and 1 <= quality <= 100:
            self.quality_factor = quality
        refresh = data.get("refreshRate")
        if (
            isinstance(refresh, (int, float))
            and not isinstance(refresh, bool)
            and stereo.math.isfinite(refresh)
            and refresh > 0
        ):
            self.timer_period = max(stereo.MIN_TIMER_PERIOD_SECONDS, float(refresh))

    def _start_mjpeg_server(self):
        """Start the embedded HTTP MJPEG server unless disabled (port <= 0)."""
        if not getattr(self, "camera_available", False):
            return
        raw_port = stereo.os.environ.get(
            "PIB_CAMERA_STREAM_PORT", str(stereo.DEFAULT_STREAM_PORT)
        )
        try:
            port = int(raw_port)
        except ValueError:
            port = stereo.DEFAULT_STREAM_PORT
        if port <= 0:
            return
        if getattr(self, "mjpeg_buffer", None) is None:
            self.mjpeg_buffer = stereo.MjpegFrameBuffer()
        server = stereo.MjpegStreamServer(frame_buffer=self.mjpeg_buffer, port=port)
        if server.start():
            self.mjpeg_server = server
            self.get_logger().info(
                f"Camera HTTP MJPEG stream listening on http://0.0.0.0:{server.server_port}/stream"
            )
        else:
            self.get_logger().warning(
                f"Camera HTTP MJPEG server could not bind port {port}"
            )

    def _has_active_frame(self) -> bool:
        """Whether a camera frame (hardware JPEG or decoded BGR) has arrived."""
        return bool(getattr(self, "current_jpeg_bytes", b"")) or (
            getattr(self, "current_frame", None) is not None
        )

    def _active_frame_dimensions(self):
        """Return ``(frame_width, frame_height)`` for detection scaling."""
        frame = getattr(self, "current_frame", None)
        if frame is not None and hasattr(frame, "shape"):
            return int(frame.shape[1]), int(frame.shape[0])
        return self._published_frame_size()

    def _decode_current_frame(self):
        """Return a BGR numpy frame, decoding ``current_jpeg_bytes`` on demand."""
        frame = getattr(self, "current_frame", None)
        if frame is not None:
            return frame
        jpeg_bytes = getattr(self, "current_jpeg_bytes", b"")
        if not jpeg_bytes:
            return None
        try:
            arr = stereo.np.frombuffer(jpeg_bytes, dtype=stereo.np.uint8)
            decoded = stereo.cv2.imdecode(arr, stereo.cv2.IMREAD_COLOR)
        except Exception:
            return None
        if decoded is not None:
            self.current_frame = decoded
        return decoded

    @staticmethod
    def _extract_packet_jpeg(packet):
        """Extract hardware-encoded JPEG bytes from a VideoEncoder packet."""
        get_data = getattr(packet, "getData", None)
        if callable(get_data):
            try:
                raw = get_data()
                if isinstance(raw, (bytes, bytearray, memoryview)):
                    data = bytes(raw)
                    if len(data) >= 2 and data[:2] == b"\xff\xd8":
                        return data
            except Exception:
                pass
        return None

    def _publish_compressed_image(self, jpeg_bytes: bytes):
        """Publish ``sensor_msgs/msg/CompressedImage`` when subscribed."""
        pub = getattr(self, "compressed_publisher_", None)
        if pub is None or pub.get_subscription_count() <= 0:
            return
        msg = stereo.CompressedImage()
        if hasattr(self, "get_clock"):
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = stereo.COMPRESSED_IMAGE_FRAME_ID
        msg.format = "jpeg"
        msg.data = list(jpeg_bytes)
        pub.publish(msg)
