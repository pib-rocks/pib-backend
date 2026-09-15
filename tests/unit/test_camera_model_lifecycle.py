"""Device-free tests for the camera model registry and pipeline manager."""

from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import MagicMock

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))

from ros_packages.camera.oak_d_lite.model_registry import ModelRegistry
from ros_packages.camera.oak_d_lite.pipeline_manager import PipelineManager


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


class TestModelRegistry(unittest.TestCase):
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

    def _manager(self, rebuild=None, verify=None, attempts=2):
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

    def test_rejects_shaves_that_do_not_match_the_compiled_blob(self):
        manager = self._manager()

        success, message = manager.start("demo", 2, "ui")

        self.assertFalse(success)
        self.assertIn("compiled for 4 shaves", message)
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


if __name__ == "__main__":
    unittest.main()
