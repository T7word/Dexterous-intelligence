"""Tests for DexHandLogger"""

import pytest
import json
import time
from pathlib import Path
from dataclasses import asdict
import numpy as np
from unittest.mock import patch, Mock

from pyzlg_dexhand.dexhand_logger import DexHandLogger
from pyzlg_dexhand.dexhand_interface import (
    HandFeedback, JointFeedback, StampedTouchFeedback,
    ControlMode, LogLevel
)

@pytest.fixture
def test_logger(tmp_path):
    """Create a logger instance with temporary directory"""
    return DexHandLogger(str(tmp_path), log_level=LogLevel.INFO)

@pytest.fixture
def mock_feedback():
    """Create mock HandFeedback for testing"""
    return HandFeedback(
        query_timestamp=time.time(),
        joints={
            'th_rot': JointFeedback(
                timestamp=time.time(),
                angle=45.0,
                encoder_position=1000
            ),
            'th_mcp': JointFeedback(
                timestamp=time.time(),
                angle=30.0,
                encoder_position=500
            )
        },
        touch={
            'th': StampedTouchFeedback(
                timestamp=time.time(),
                normal_force=1.5,
                normal_force_delta=100,
                tangential_force=0.5,
                tangential_force_delta=50,
                direction=180,
                proximity=500,
                temperature=25
            )
        }
    )

@pytest.fixture
def joint_commands():
    """Create mock joint commands dict for testing"""
    return {
        'th_rot': 45.0,
        'th_mcp': 30.0,
        'ff_mcp': 20.0
    }

class TestDexHandLogger:
    """Test DexHandLogger functionality"""

    def test_initialization(self, test_logger):
        """Test logger initialization"""
        assert test_logger.log_dir.exists()
        assert test_logger.session_dir.exists()
        assert (test_logger.session_dir / "left_commands.jsonl").exists()
        assert (test_logger.session_dir / "right_commands.jsonl").exists()
        assert (test_logger.session_dir / "left_feedback.jsonl").exists()
        assert (test_logger.session_dir / "right_feedback.jsonl").exists()

    def test_log_command(self, test_logger, joint_commands, mock_feedback):
        """Test command logging"""
        # Call log_command with correct parameters
        test_logger.log_command(
            command_type="move_joints",
            joint_commands=joint_commands,
            control_mode=ControlMode.IMPEDANCE_GRASP,
            hand="left",
            feedback=mock_feedback
        )

        # Wait for the writer queue to process all entries
        test_logger.writer.queue.join()

        # Check buffer
        assert len(test_logger.command_buffers["left"]) == 1
        cmd = test_logger.command_buffers["left"][0]
        assert cmd.command_type == "move_joints"
        assert "th_rot" in cmd.joint_commands
        assert cmd.joint_commands["th_rot"] == 45.0
        assert cmd.control_mode == ControlMode.IMPEDANCE_GRASP

        # Check file
        cmd_file = test_logger.session_dir / "left_commands.jsonl"
        with open(cmd_file) as f:
            lines = f.readlines()
            assert len(lines) >= 1  # May have multiple lines if feedback is also recorded
            data = json.loads(lines[0])
            assert data["command_type"] == "move_joints"
            assert data["joint_commands"]["th_rot"] == 45.0

        # Also check if feedback was recorded
        assert len(test_logger.feedback_buffers["left"]) == 1

    def test_log_feedback(self, test_logger, mock_feedback):
        """Test feedback logging"""
        test_logger.log_feedback(mock_feedback, "left")

        test_logger.writer.queue.join()

        # Check buffer
        assert len(test_logger.feedback_buffers["left"]) == 1
        fb = test_logger.feedback_buffers["left"][0]
        assert "th_rot" in fb.joints
        assert "th" in fb.touch

        # Check file
        fb_file = test_logger.session_dir / "left_feedback.jsonl"
        with open(fb_file) as f:
            lines = f.readlines()
            assert len(lines) == 1
            data = json.loads(lines[0])
            assert "joints" in data
            assert "th_rot" in data["joints"]
            assert "touch" in data
            assert "th" in data["touch"]

    def test_save_metadata(self, test_logger):
        """Test metadata saving"""
        metadata = {
            "test_param": 123,
            "test_string": "value"
        }
        test_logger.save_metadata(metadata)

        meta_file = test_logger.session_dir / "metadata.json"
        assert meta_file.exists()

        with open(meta_file) as f:
            loaded = json.load(f)
            assert loaded == metadata

    @pytest.mark.skipif(not pytest.importorskip("matplotlib", reason="matplotlib required"),
                     reason="matplotlib not available")
    def test_plot_session(self, test_logger, joint_commands, mock_feedback):
        """Test plotting functionality"""
        # Record some test data
        test_logger.log_command(
            command_type="move_joints",
            joint_commands=joint_commands,
            control_mode=ControlMode.IMPEDANCE_GRASP,
            hand="left"
        )
        test_logger.log_feedback(mock_feedback, "left")

        # Test plotting without showing but saving
        test_logger.plot_session(show=False, save=True)

        # Check if plot files were created
        assert (test_logger.session_dir / "left_joints.png").exists()
        # If touch data exists, should have touch plot
        if mock_feedback.touch:
            assert (test_logger.session_dir / "left_touch.png").exists()

    def test_close(self, test_logger, joint_commands, mock_feedback):
        """Test logger closing"""
        # Record some test data
        test_logger.log_command(
            command_type="move_joints",
            joint_commands=joint_commands,
            control_mode=ControlMode.IMPEDANCE_GRASP,
            hand="left"
        )
        test_logger.log_feedback(mock_feedback, "left")

        # Close logger
        test_logger.close()

        # Check metadata file
        meta_file = test_logger.session_dir / "metadata.json"
        assert meta_file.exists()

        with open(meta_file) as f:
            metadata = json.load(f)
            assert "statistics" in metadata
            assert metadata["statistics"]["num_commands"]["left"] == 1
            assert metadata["statistics"]["num_feedback"]["left"] == 1
            
    def test_log_level(self, test_logger, tmp_path):
        """Test logger with different log levels"""
        
        # Test INFO level - should log INFO messages
        with patch('pyzlg_dexhand.dexhand_logger.logger') as mock_logger:
            info_logger = DexHandLogger(str(tmp_path / "info"), log_level=LogLevel.INFO)
            info_logger.close(log_level=LogLevel.INFO)
            mock_logger.info.assert_called()
        

if __name__ == "__main__":
       pytest.main(["-v", __file__])