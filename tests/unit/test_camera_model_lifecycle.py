"""Device-free tests for the camera model registry and pipeline manager."""

from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import MagicMock

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from ros_packages.camera.oak_d_lite.model_registry import ModelRegistry
from ros_packages.camera.oak_d_lite.pipeline_manager import PipelineManager


def _interface_fields(relative_path):
    """Return ROS interface fields, preserving request/response separation."""
    fields = []
    path = REPO_ROOT / "ros_packages/datatypes" / relative_path
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if line:
            fields.append(line)
    return fields


def _manifest(model_file="demo/demo.blob"):
    return {
        "models": [
            {
                "model_id": "demo",
                "task": "test",
                "licence": "Apache-2.0",
                "file": model_file,
                "sha256": "0" * 64,
                "size_bytes": 4,
                "shaves": 4,
                "input_width": 16,
                "input_height": 16,
                "format": "blob",
                "openvino_version": "2022.1",
                "notes": "",
            }
        ]
    }


def _composite_manifest():
    manifest = {"models": []}
    for model_id, size, shaves in (
        ("palm", 4, 4),
        ("decoder", 3, 1),
        ("landmark", 5, 4),
    ):
        entry = _manifest(f"{model_id}/{model_id}.blob")["models"][0]
        entry.update(
            {
                "model_id": model_id,
                "size_bytes": size,
                "shaves": shaves,
            }
        )
        manifest["models"].append(entry)
    manifest["models"].append(
        {
            "model_id": "hand_tracking",
            "task": "hand_tracking",
            "composite": True,
            "artifacts": ["palm", "decoder", "landmark"],
            "publish_topic": "detections/hand_tracking",
        }
    )
    return manifest


class TestModelRegistry(unittest.TestCase):
    def test_model_service_interfaces_preserve_the_public_contract(self):
        self.assertEqual(
            _interface_fields("srv/ListModels.srv"),
            ["---", "ModelInfo[] models"],
        )
        self.assertEqual(
            _interface_fields("srv/StartModel.srv"),
            [
                "string model_id",
                "int32 shaves",
                "string owner",
                "---",
                "bool success",
                "string message",
            ],
        )
        self.assertEqual(
            _interface_fields("srv/StopModel.srv"),
            [
                "string model_id",
                "string owner",
                "---",
                "bool success",
                "string message",
            ],
        )
        self.assertEqual(
            _interface_fields("srv/GetDetections.srv"),
            ["string model_id", "---", "DetectionArray detections"],
        )

    def test_model_list_and_status_messages_preserve_the_public_contract(self):
        self.assertEqual(
            _interface_fields("msg/ModelInfo.msg"),
            [
                "string model_id",
                "string task",
                "string licence",
                "int32 shaves",
                "uint32 size_bytes",
                "bool available",
                "bool active",
            ],
        )
        self.assertEqual(
            _interface_fields("msg/ModelStatus.msg"),
            [
                "string model_id",
                "bool active",
                "float32 fps",
                "int32 shaves",
                "string state",
                "string message",
            ],
        )
        self.assertEqual(
            _interface_fields("msg/ModelStatusArray.msg"),
            ["std_msgs/Header header", "ModelStatus[] models"],
        )

    def test_hand_tracking_manifest_has_empirical_shave_budgets(self):
        manifest = yaml.safe_load(
            (REPO_ROOT / "models/manifest.yaml").read_text(encoding="utf-8")
        )
        shaves = {
            entry["model_id"]: entry.get("shaves") for entry in manifest["models"]
        }

        self.assertEqual(
            {
                model_id: shaves[model_id]
                for model_id in (
                    "palm_detection_128x128",
                    "palm_detection_128x128_decoding",
                    "hand_landmark_224x224",
                )
            },
            {
                "palm_detection_128x128": 4,
                "palm_detection_128x128_decoding": 1,
                "hand_landmark_224x224": 4,
            },
        )

    def test_parses_manifest_and_marks_present_blob_available(self):
        with tempfile.TemporaryDirectory() as store:
            store_path = Path(store)
            (store_path / "demo").mkdir()
            (store_path / "demo/demo.blob").write_bytes(b"blob")
            (store_path / "manifest.yaml").write_text(
                yaml.safe_dump(_manifest()), encoding="utf-8"
            )

            registry = ModelRegistry(store_path)
            model = registry.get("demo")

            self.assertEqual(len(registry), 1)
            self.assertEqual(model.task, "test")
            self.assertEqual(model.licence, "Apache-2.0")
            self.assertEqual(model.shaves, 4)
            self.assertEqual(model.size_bytes, 4)
            self.assertEqual(model.blob_path, str(store_path / "demo/demo.blob"))
            self.assertTrue(model.available)

    def test_missing_manifest_warns_once_and_returns_empty_registry(self):
        logger = MagicMock()
        with tempfile.TemporaryDirectory() as store:
            registry = ModelRegistry(store, logger=logger)

        self.assertEqual(len(registry), 0)
        logger.warning.assert_called_once()

    def test_partial_manifest_warns_and_discards_all_entries(self):
        logger = MagicMock()
        with tempfile.TemporaryDirectory() as store:
            store_path = Path(store)
            manifest = _manifest()
            del manifest["models"][0]["task"]
            (store_path / "manifest.yaml").write_text(
                yaml.safe_dump(manifest), encoding="utf-8"
            )
            registry = ModelRegistry(store_path, logger=logger)

        self.assertEqual(len(registry), 0)
        logger.warning.assert_called_once()

    def test_missing_blob_makes_partial_store_empty_and_warns_once(self):
        logger = MagicMock()
        with tempfile.TemporaryDirectory() as store:
            store_path = Path(store)
            (store_path / "manifest.yaml").write_text(
                yaml.safe_dump(_manifest()), encoding="utf-8"
            )
            registry = ModelRegistry(store_path, logger=logger)

        self.assertEqual(len(registry), 0)
        logger.warning.assert_called_once()

    def test_composite_resolves_when_all_artifacts_are_present(self):
        with tempfile.TemporaryDirectory() as store:
            store_path = Path(store)
            for model_id, content in (
                ("palm", b"palm"),
                ("decoder", b"dec"),
                ("landmark", b"marks"),
            ):
                (store_path / model_id).mkdir()
                (store_path / model_id / f"{model_id}.blob").write_bytes(content)
            (store_path / "manifest.yaml").write_text(
                yaml.safe_dump(_composite_manifest()), encoding="utf-8"
            )

            model = ModelRegistry(store_path).get("hand_tracking")

        self.assertTrue(model.available)
        self.assertTrue(model.composite)
        self.assertEqual(model.artifact_ids, ("palm", "decoder", "landmark"))
        self.assertEqual(model.size_bytes, 12)
        self.assertEqual(model.shaves, 9)

    def test_composite_is_unavailable_when_one_artifact_is_missing(self):
        with tempfile.TemporaryDirectory() as store:
            store_path = Path(store)
            for model_id, content in (("palm", b"palm"), ("decoder", b"dec")):
                (store_path / model_id).mkdir()
                (store_path / model_id / f"{model_id}.blob").write_bytes(content)
            (store_path / "manifest.yaml").write_text(
                yaml.safe_dump(_composite_manifest()), encoding="utf-8"
            )

            model = ModelRegistry(store_path).get("hand_tracking")

        self.assertFalse(model.available)


class TestPipelineManager(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        store = Path(self.tempdir.name)
        (store / "demo").mkdir()
        (store / "demo/demo.blob").write_bytes(b"blob")
        (store / "manifest.yaml").write_text(
            yaml.safe_dump(_manifest()), encoding="utf-8"
        )
        self.registry = ModelRegistry(store)
        self.rebuild_calls = []
        self.revert = MagicMock(return_value=True)
        self.logger = MagicMock()

    def tearDown(self):
        self.tempdir.cleanup()

    def _manager(self, rebuild=None, verify=None, attempts=2, clock=None):
        def record_rebuild(models):
            self.rebuild_calls.append(
                [(active.model.model_id, active.shaves) for active in models]
            )
            return True

        return PipelineManager(
            self.registry,
            rebuild or record_rebuild,
            verify or (lambda timeout: True),
            self.revert,
            logger=self.logger,
            attempts=attempts,
            sleep=lambda delay: None,
            clock=clock or time.monotonic,
        )

    def test_reference_counts_owners_without_redundant_rebuilds(self):
        manager = self._manager()

        self.assertTrue(manager.start("demo", 0, "ui")[0])
        self.assertTrue(manager.start("demo", 0, "blockly")[0])
        self.assertTrue(manager.start("demo", 0, "ui")[0])
        self.assertEqual(self.rebuild_calls, [[("demo", 4)]])
        self.assertEqual(manager.status("demo")["owners"], {"ui", "blockly"})

        self.assertTrue(manager.stop("demo", "ui")[0])
        self.assertEqual(len(self.rebuild_calls), 1)
        self.assertTrue(manager.status("demo")["active"])

        self.assertTrue(manager.stop("demo", "blockly")[0])
        self.assertEqual(self.rebuild_calls[-1], [])
        self.assertEqual(manager.status("demo")["state"], "idle")

    def test_start_failure_retries_then_reverts_to_color(self):
        rebuild = MagicMock(return_value=True)
        verify = MagicMock(return_value=False)
        manager = self._manager(rebuild=rebuild, verify=verify, attempts=3)

        success, _ = manager.start("demo", 4, "ui")

        self.assertFalse(success)
        self.assertEqual(rebuild.call_count, 3)
        self.assertEqual(verify.call_count, 3)
        self.revert.assert_called_once()
        self.logger.warning.assert_called_once()
        status = manager.status("demo")
        self.assertEqual(status["state"], "failed")
        self.assertFalse(status["active"])
        self.assertEqual(status["owners"], set())

    def test_mark_failed_corrects_a_stale_running_status(self):
        manager = self._manager()
        self.assertTrue(manager.start("demo", 4, "ui")[0])

        self.assertTrue(
            manager.mark_failed("demo", "Physical model pipeline is not flowing")
        )

        status = manager.status("demo")
        self.assertEqual(status["state"], "failed")
        self.assertEqual(status["message"], "Physical model pipeline is not flowing")
        self.assertFalse(status["active"])
        self.assertEqual(status["fps"], 0.0)
        self.assertEqual(status["owners"], {"ui"})

    def test_startup_grace_delays_failure_but_does_not_hide_dead_chain(self):
        now = [10.0]

        def verify_after_slow_device_start(timeout):
            now[0] = 20.0
            return True

        manager = self._manager(
            verify=verify_after_slow_device_start, clock=lambda: now[0]
        )
        self.assertTrue(manager.start("demo", 4, "ui")[0])

        now[0] = 24.9
        self.assertFalse(
            manager.mark_failed("demo", "No downstream packets", startup_grace=5.0)
        )
        self.assertEqual(manager.status("demo")["state"], "running")

        now[0] = 25.0
        self.assertTrue(
            manager.mark_failed("demo", "No downstream packets", startup_grace=5.0)
        )
        self.assertEqual(manager.status("demo")["state"], "failed")
        self.assertFalse(manager.status("demo")["active"])

    def test_late_packet_flow_recovers_failed_model_and_real_fps(self):
        now = [20.0]
        manager = self._manager(clock=lambda: now[0])
        self.assertTrue(manager.start("demo", 4, "ui")[0])
        self.assertTrue(manager.mark_failed("demo", "No downstream packets"))

        now[0] = 21.0
        self.assertTrue(manager.mark_running("demo"))
        manager.record_packet("demo")
        manager.record_packet("demo")
        now[0] = 21.5
        manager.refresh_fps()

        status = manager.status("demo")
        self.assertEqual(status["state"], "running")
        self.assertTrue(status["active"])
        self.assertEqual(status["message"], "Model is running")
        self.assertEqual(status["fps"], 4.0)
        self.assertEqual(status["owners"], {"ui"})

    def test_rejects_shaves_that_do_not_match_the_compiled_blob(self):
        manager = self._manager()

        success, message = manager.start("demo", 2, "ui")

        self.assertFalse(success)
        self.assertIn("compiled for 4 shaves", message)
        self.assertEqual(self.rebuild_calls, [])

    def test_rejects_unknown_models_and_invalid_request_fields(self):
        manager = self._manager()

        self.assertEqual(
            manager.start("missing", 0, "ui"),
            (False, "Unknown model: missing"),
        )
        self.assertEqual(
            manager.start("demo", -1, "ui"),
            (False, "shaves must be zero or positive"),
        )
        self.assertEqual(
            manager.start("demo", 0, "   "),
            (False, "owner must not be empty"),
        )
        self.assertEqual(
            manager.stop("demo", ""),
            (False, "owner must not be empty"),
        )
        self.assertEqual(self.rebuild_calls, [])

    def test_stop_failure_reverts_and_marks_remaining_models_failed(self):
        manager = self._manager()
        self.assertTrue(manager.start("demo", 0, "ui")[0])
        manager._verify_frames = MagicMock(return_value=False)

        success, _ = manager.stop("demo", "ui")

        self.assertFalse(success)
        self.revert.assert_called_once()
        self.logger.warning.assert_called_once()
        self.assertEqual(manager.status("demo")["state"], "idle")

    def test_composite_model_reference_counts_owners_as_one_model(self):
        with tempfile.TemporaryDirectory() as store:
            store_path = Path(store)
            for model_id, content in (
                ("palm", b"palm"),
                ("decoder", b"dec"),
                ("landmark", b"marks"),
            ):
                (store_path / model_id).mkdir()
                (store_path / model_id / f"{model_id}.blob").write_bytes(content)
            (store_path / "manifest.yaml").write_text(
                yaml.safe_dump(_composite_manifest()), encoding="utf-8"
            )
            registry = ModelRegistry(store_path)
            rebuilds = []
            manager = PipelineManager(
                registry,
                lambda models: rebuilds.append(
                    [(active.model.model_id, active.shaves) for active in models]
                )
                or True,
                lambda timeout: True,
                lambda: True,
            )

            self.assertTrue(manager.start("hand_tracking", 0, "ui")[0])
            self.assertTrue(manager.start("hand_tracking", 0, "blockly")[0])
            self.assertEqual(rebuilds, [[("hand_tracking", 9)]])
            self.assertTrue(manager.stop("hand_tracking", "ui")[0])
            self.assertEqual(len(rebuilds), 1)
            self.assertTrue(manager.stop("hand_tracking", "blockly")[0])
            self.assertEqual(rebuilds[-1], [])


if __name__ == "__main__":
    unittest.main()
