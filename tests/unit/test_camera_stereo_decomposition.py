"""PR-2121: stereo.py decomposition checks."""

from __future__ import annotations

import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
STEREO_PATH = REPO_ROOT / "ros_packages/camera/oak_d_lite/stereo.py"
MIXIN_PATHS = (
    REPO_ROOT / "ros_packages/camera/oak_d_lite/hand_tracking_mixin.py",
    REPO_ROOT / "ros_packages/camera/oak_d_lite/pipeline_builders_mixin.py",
    REPO_ROOT / "ros_packages/camera/oak_d_lite/branch_capture_mixin.py",
)
MIXIN_CLASSES = {
    "HandTrackingMixin",
    "PipelineBuildersMixin",
    "BranchCaptureMixin",
    "CameraNode",
}

# Methods CameraNode exposed before PR-2121 (stable public surface for tests).
EXPECTED_CAMERA_NODE_METHODS = {
    "__init__",
    "_branch_output_size",
    "_build_face_crop_pipeline",
    "_build_gaze_pipeline",
    "_build_hand_fast_pipeline",
    "_build_hand_mp_pipeline",
    "_build_hand_pipeline",
    "_build_imitation_pipeline",
    "_build_pipeline",
    "_build_single_network_pipeline",
    "_colour_branch_capture_requested",
    "_colour_branch_size",
    "_configure_hand_manip",
    "_consume_imitation_packet",
    "_consume_landmark_result",
    "_count_hand_stage",
    "_count_imitation_stage",
    "_drain_colour_branch_capture",
    "_drop_landmark_result",
    "_empty_detections",
    "_encode_depth_frame",
    "_encode_frame",
    "_face_crop_chain_is_built",
    "_fast_detection_message",
    "_field_from_packets",
    "_format_src_crops",
    "_hand_chain_is_built",
    "_hand_chain_is_flowing",
    "_hand_detection_message",
    "_hand_fast_chain_is_built",
    "_hand_fingerprint_due",
    "_hand_landmark_handedness",
    "_hand_landmark_score",
    "_hand_landmark_tensor",
    "_hand_mp_chain_is_built",
    "_hand_mp_detection",
    "_hand_mp_palm",
    "_hand_mp_stamp",
    "_hand_tensor_text",
    "_imitation_chain_is_built",
    "_imitation_chain_is_flowing",
    "_imitation_detection",
    "_imu_message",
    "_imu_report_xyz",
    "_imu_status",
    "_init_imu",
    "_init_stereo_depth",
    "_landmark_crop_config",
    "_log_hand_assembly_fingerprint",
    "_log_hand_keypoint_fingerprint",
    "_log_hand_stage_counters",
    "_log_imitation_stage_counters",
    "_log_landmark_fingerprint",
    "_log_palm_fingerprint",
    "_map_hand_landmarks",
    "_nn_layer",
    "_note_hand_branch_size",
    "_object_detection_field",
    "_packet_layer_names",
    "_packet_transformation",
    "_plane_point",
    "_point_xy",
    "_poll_colour_branch_capture",
    "_process_face_crop",
    "_process_hand_mp",
    "_process_hand_tracking",
    "_process_hand_tracking_fast",
    "_process_imitation",
    "_process_imu",
    "_publish_colorized_depth",
    "_publish_face_crop_detection",
    "_publish_hand_detections",
    "_publish_hand_fast_detections",
    "_publish_hand_mp_detections",
    "_publish_hand_mp_pair",
    "_publish_imitation_detections",
    "_publish_parsed_detections",
    "_publish_qr_detections",
    "_publish_ready_face_crops",
    "_published_frame_size",
    "_queue_empty_landmark_frame",
    "_queue_landmark_crops",
    "_queue_state",
    "_read_hand_startup_grace",
    "_read_stereo_mode",
    "_read_stereo_timeout",
    "_rebuild_models",
    "_relax_branch_input",
    "_remap_into_preview",
    "_request_camera_branch",
    "_require_published_aspect",
    "_reset_hand_stage_counters",
    "_reset_imitation_stage_counters",
    "_revert_to_color_only",
    "_size_pair",
    "_start_pipeline",
    "_stereo_requested",
    "_stop_pipeline",
    "_stretch_to_network_input",
    "_verify_model_frames",
    "_wait_for_color_frame",
    "_wait_for_queue_packet",
    "_warn_hand_once",
    "_write_colour_branch_capture",
    "destroy_node",
    "get_camera_image_callback",
    "get_depth_frame_callback",
    "get_detections_callback",
    "get_distance_at_px_callback",
    "init_pipeline",
    "list_models_callback",
    "preview_size_callback",
    "publish_face_center",
    "publish_model_statuses",
    "quality_factor_callback",
    "start_model_callback",
    "stop_model_callback",
    "timer_callback",
    "timer_period_callback",
}


def test_stereo_module_is_under_line_budget():
    line_count = len(STEREO_PATH.read_text(encoding="utf-8").splitlines())
    assert line_count < 1200, f"stereo.py still {line_count} lines"


def test_mixin_modules_exist():
    for path in MIXIN_PATHS:
        assert path.is_file(), path


def _method_names_in_sources(*paths: Path) -> set[str]:
    names: set[str] = set()
    for path in paths:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in tree.body:
            if not isinstance(node, ast.ClassDef) or node.name not in MIXIN_CLASSES:
                continue
            for item in node.body:
                if isinstance(item, ast.FunctionDef):
                    names.add(item.name)
    return names


def test_camera_node_keeps_expected_methods():
    defined = _method_names_in_sources(STEREO_PATH, *MIXIN_PATHS)
    missing = EXPECTED_CAMERA_NODE_METHODS - defined
    assert not missing, f"CameraNode surface missing methods: {sorted(missing)}"


def test_oak_d_lite_modules_never_import_via_repo_root_ros_packages_prefix() -> None:
    """Installed colcon packages live at site-packages/oak_d_lite, where 'ros_packages' does not exist."""
    oak_dir = REPO_ROOT / "ros_packages" / "camera" / "oak_d_lite"
    for py_file in sorted(oak_dir.glob("*.py")):
        source = py_file.read_text(encoding="utf-8")
        assert (
            "ros_packages." not in source
        ), f"{py_file.name} imports 'ros_packages.*', which fails with ModuleNotFoundError inside the colcon container"
