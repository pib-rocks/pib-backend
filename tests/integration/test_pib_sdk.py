"""Integration tests for global pib-sdk installation, SDK initialization, and motor sweep verification."""

from __future__ import annotations

import sys
from unittest.mock import MagicMock, patch

import pytest


class TestPibSDKIntegration:
    """Test suite for pib-sdk availability and control features."""

    def test_pib_sdk_global_importability(self):
        """Verify pib-sdk is importable globally and contains required modules and symbols."""
        import pib_sdk

        assert hasattr(pib_sdk, "__version__")
        assert pib_sdk.__version__ is not None
        assert hasattr(pib_sdk, "Write")
        assert hasattr(pib_sdk, "head")
        assert hasattr(pib_sdk, "right_arm")
        assert hasattr(pib_sdk, "left_arm")
        assert hasattr(pib_sdk, "zero_position")
        assert hasattr(pib_sdk, "control")

    @patch("roslibpy.Ros")
    @patch("pib_sdk.control._wait_until_connected")
    def test_pib_sdk_initialization(self, mock_wait_conn, mock_ros_cls):
        """Verify Write client initializes correctly with host/port and handles context manager cleanup."""
        import pib_sdk
        from pib_sdk import Write

        mock_ros_inst = MagicMock()
        mock_ros_inst.is_connected = True
        mock_ros_cls.return_value = mock_ros_inst

        with Write(host="localhost", port=9090) as client:
            assert client.ros == mock_ros_inst
            mock_ros_cls.assert_called_once_with(host="localhost", port=9090)
            mock_ros_inst.run.assert_called_once()
            mock_wait_conn.assert_called_once_with(mock_ros_inst)

        mock_ros_inst.terminate.assert_called_once()

    @patch("roslibpy.Ros")
    @patch("pib_sdk.control._wait_until_connected")
    def test_pib_sdk_motor_sweep_single_joint(self, mock_wait_conn, mock_ros_cls):
        """Verify motor sweep test moving a joint between 2 angles (-15.0 deg to 15.0 deg)."""
        import pib_sdk
        from pib_sdk import Write

        mock_ros_inst = MagicMock()
        mock_ros_cls.return_value = mock_ros_inst

        client = Write(host="localhost", port=9090)
        mock_service = MagicMock()
        mock_service.call.return_value = {"successful": True}
        client._joint_trajectory_service = mock_service

        # Step 1: Move joint to angle 1 (-15.0 degrees = -1500 internal units)
        angle_1 = -15.0
        res_1 = client.move("head_pitch", angle_1)
        assert res_1 is True

        assert mock_service.call.call_count == 1
        req_1 = mock_service.call.call_args[0][0]
        traj_1 = req_1["joint_trajectory"]
        assert traj_1["joint_names"] == ["head_pitch"]
        assert traj_1["points"][0]["positions"] == [-1500.0]

        # Step 2: Move joint to angle 2 (+15.0 degrees = 1500 internal units)
        angle_2 = 15.0
        res_2 = client.move("head_pitch", angle_2)
        assert res_2 is True

        assert mock_service.call.call_count == 2
        req_2 = mock_service.call.call_args[0][0]
        traj_2 = req_2["joint_trajectory"]
        assert traj_2["joint_names"] == ["head_pitch"]
        assert traj_2["points"][0]["positions"] == [1500.0]

    @patch("roslibpy.Ros")
    @patch("pib_sdk.control._wait_until_connected")
    def test_pib_sdk_motor_sweep_group(self, mock_wait_conn, mock_ros_cls):
        """Verify motor sweep moving a group token between 2 angles."""
        import pib_sdk
        from pib_sdk import Write, head

        mock_ros_inst = MagicMock()
        mock_ros_cls.return_value = mock_ros_inst

        client = Write(host="localhost", port=9090)
        mock_service = MagicMock()
        mock_service.call.return_value = {"successful": True}
        client._joint_trajectory_service = mock_service

        # Move head group to angle 1 (-20.0 deg) then angle 2 (20.0 deg)
        res_1 = client.move(head, -20.0)
        res_2 = client.move(head, 20.0)

        assert res_1 is True
        assert res_2 is True
        assert mock_service.call.call_count == 2

    @patch("roslibpy.Ros")
    @patch("pib_sdk.control._wait_until_connected")
    def test_pib_sdk_motor_settings(self, mock_wait_conn, mock_ros_cls):
        """Verify set() method applies MotorSettings over service."""
        import pib_sdk
        from pib_sdk import Write

        mock_ros_inst = MagicMock()
        mock_ros_cls.return_value = mock_ros_inst

        client = Write(host="localhost", port=9090)
        mock_service = MagicMock()
        mock_service.call.return_value = {"settings_applied": True}
        client._motor_settings_service = mock_service

        res = client.set("head_pitch", velocity=6000, acceleration=10000)
        assert res is True
        mock_service.call.assert_called_once()
        req = mock_service.call.call_args[0][0]
        assert req["motor_settings"]["motor_name"] == "head_pitch"
        assert req["motor_settings"]["velocity"] == 6000
        assert req["motor_settings"]["acceleration"] == 10000
