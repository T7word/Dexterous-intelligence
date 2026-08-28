import pytest
import numpy as np
from unittest.mock import Mock, patch, call
from dataclasses import asdict
import yaml
from pathlib import Path
from dataclasses import dataclass
from typing import Optional
import time     # 添加这个导入
import tempfile  # 添加这个导入
import shutil    # 添加这个导入
import datetime  # 添加这个导入

from pyzlg_dexhand.dexhand_interface import (
    DexHandBase, LeftDexHand, RightDexHand, HandConfig,
    JointFeedback, StampedTouchFeedback, HandFeedback, LogLevel, JointCommand
)
from pyzlg_dexhand.dexhand_protocol import BoardID, MessageType, FlashStorageTable
from pyzlg_dexhand.dexhand_protocol.commands import (
    ControlMode, CommandType, FeedbackMode
)
from pyzlg_dexhand.dexhand_protocol.messages import (
    BoardFeedback, MotorFeedback, TouchFeedback, ErrorInfo, ProcessedMessage, BoardError
)

from pyzlg_dexhand.dexhand_logger import DexHandLogger

# Test data generators
def create_mock_feedback(timestamp=1000.0):
    """Create mock feedback data for testing"""
    return BoardFeedback(
        motor1=MotorFeedback(
            current=100, 
            velocity=200, 
            position=1000, 
            angle=45.0
        ),
        motor2=MotorFeedback(
            current=-150, 
            velocity=-250, 
            position=-2000, 
            angle=-90.0
        ),
        position_sensor1=45.0,
        position_sensor2=-90.0,
        touch=TouchFeedback(
            normal_force=1.5,
            normal_force_delta=100,
            tangential_force=0.5,
            tangential_force_delta=50,
            direction=180,
            proximity=500,
            temperature=25
        )
    )

class TestHandConfiguration:
    """Test hand configuration loading and validation"""

    def test_config_loading(self, tmp_path):
        """Test loading valid configuration"""
        config_file = tmp_path / "test_config.yaml"
        config_data = {
            "channel": 0,
            "hall_scale": [2.1] * 12
        }
        config_file.write_text(yaml.dump(config_data))

        hand = DexHandBase(config_data, BoardID.LEFT_HAND_BASE)
        assert hand.config.channel == 0
        assert len(hand.config.hall_scale) == hand.NUM_MOTORS
        assert hand.config.hall_scale[0] == 2.1

    def test_invalid_config(self, tmp_path):
        """Test loading invalid configuration"""
        config_file = tmp_path / "invalid_config.yaml"
        config_data = {
            "channel": 0,
            # Missing hall_scale
        }
        config_file.write_text(yaml.dump(config_data))
        
        with pytest.raises(KeyError, match="hall_scale"):
            DexHandBase(config_data, BoardID.LEFT_HAND_BASE)

class TestHandInitialization:
    """Test hand initialization and device setup"""

    def test_successful_init(self):
        """Test successful hand initialization"""
        mock_zcan = Mock()
        mock_zcan.open.return_value = True
        mock_zcan.configure_channel.return_value = True

        hand = LeftDexHand(zcan=mock_zcan)
        assert hand.init(device_index=0)

        mock_zcan.configure_channel.assert_called_once_with(hand.config.channel)

    def test_init_failure(self):
        """Test handling of initialization failures"""
        mock_zcan = Mock()
        mock_zcan.open.return_value = True
        mock_zcan.configure_channel.return_value = False

        hand = LeftDexHand(zcan=mock_zcan)
        assert not hand.init(device_index=0)

    def test_shared_zcan(self):
        """Test initialization with shared ZCAN instance"""
        mock_zcan = Mock()
        mock_zcan.configure_channel.return_value = True

        left_hand = LeftDexHand(zcan=mock_zcan)
        right_hand = RightDexHand(zcan=mock_zcan)

        assert left_hand.init()
        assert right_hand.init()

        # Verify ZCAN is only configured for each channel
        assert mock_zcan.configure_channel.call_count == 2
        assert mock_zcan.configure_channel.call_args_list == [
            call(left_hand.config.channel),
            call(right_hand.config.channel)
        ]

class TestCommandExecution:
    """Test command execution and feedback handling"""

    @pytest.fixture
    def mock_hand(self):
        """Create a mock hand with configured mocks"""
        mock_zcan = Mock()
        mock_zcan.configure_channel.return_value = True
        hand = LeftDexHand(zcan=mock_zcan)
        hand.init()
        return hand

    def test_successful_command(self, mock_hand):
        """Test successful command execution"""
        mock_feedback = create_mock_feedback()

        mock_hand.zcan.send_fd_message.return_value = True
        mock_hand.zcan.receive_fd_messages.return_value = [(0x181, b'0' * 46, 1000)]

        with patch('pyzlg_dexhand.dexhand_protocol.messages.process_message') as mock_process_message:
            mock_process_message.return_value = ProcessedMessage(
                sender_id=0x181,
                msg_type=MessageType.MOTION_FEEDBACK,
                feedback=mock_feedback
            )

            result = mock_hand.move_joints(
                th_rot=30.0,
                th_mcp=45.0,
                control_mode=ControlMode.IMPEDANCE_GRASP
            )

            assert result is True


    def test_error_handling(self, mock_hand):
        """Test handling of hardware errors"""
        mock_error = ErrorInfo(
            error_type=BoardError.MOTOR1_ERROR,
            error_data=bytes([0x01]),
            description="Test error"
        )

        mock_hand.zcan.send_fd_message.return_value = True
        mock_hand.zcan.receive_fd_messages.return_value = [(0x601, b'error', 1000)]
        with patch('pyzlg_dexhand.dexhand_protocol.messages.process_message') as mock_process_message:
            mock_process_message.return_value = ProcessedMessage(
                sender_id=0x601,
                msg_type=MessageType.ERROR_MESSAGE,
                error_info=mock_error
            )

            result = mock_hand.move_joints(th_rot=30.0)

            assert result is True

    def test_communication_failure(self, mock_hand):
        """Test handling of communication failures"""
        mock_hand.zcan.send_fd_message.return_value = False

        result = mock_hand.move_joints(th_rot=30.0)
        assert result is False

class TestJointControl:
    """Test joint control functionality"""

    @pytest.fixture
    def mock_hand(self):
        mock_zcan = Mock()
        mock_zcan.configure_channel.return_value = True
        hand = LeftDexHand(zcan=mock_zcan)
        hand.init()
        return hand

    def test_angle_scaling(self, mock_hand):
        """Test angle scaling for different control modes"""
        # Test IMPEDANCE_GRASP mode
        scaled = mock_hand._scale_angle(0, 45.0, ControlMode.IMPEDANCE_GRASP)
        assert scaled == 4500  # 45 * 100

        # Test HALL_POSITION mode
        scaled = mock_hand._scale_angle(0, 45.0, ControlMode.HALL_POSITION)
        assert isinstance(scaled, int)
        assert scaled != 0  # Should be scaled by hall factor

    def test_reset_joints(self, mock_hand):
        """Test joint reset functionality"""
        mock_feedback = create_mock_feedback()
        mock_hand.zcan.send_fd_message.return_value = True
        mock_hand.zcan.receive_fd_messages.return_value = [(0x181, b'dummy', 1000)]

        with patch('pyzlg_dexhand.dexhand_protocol.messages.process_message') as mock_process_message:
            mock_process_message.return_value = ProcessedMessage(
                sender_id=0x181,
                msg_type=MessageType.MOTION_FEEDBACK,
                feedback=mock_feedback
            )
            result = mock_hand.reset_joints()
            assert result is True

    def test_get_feedback(self, mock_hand):
        """Test feedback collection without motion"""
        mock_feedback = create_mock_feedback()
        mock_hand.zcan.send_fd_message.return_value = True
        mock_hand.zcan.receive_fd_messages.return_value = [(0x181, b'dummy', 1000)]

        with patch('pyzlg_dexhand.dexhand_protocol.messages.process_message') as mock_process_message:
            mock_process_message.return_value = ProcessedMessage(
                sender_id=0x181,
                msg_type=MessageType.MOTION_FEEDBACK,
                feedback=mock_feedback
            )

            feedback = mock_hand.get_feedback()
            assert isinstance(feedback, HandFeedback)
            assert len(feedback.joints) > 0
            # Tactile feedback may not be present depending on the mock setup

class TestBoardAddressing:
    """Test correct board addressing and ID handling"""

    @pytest.fixture
    def mock_hand(self):
        """Create a mock hand with configured mocks"""
        mock_zcan = Mock()
        mock_zcan.configure_channel.return_value = True
        hand = LeftDexHand(zcan=mock_zcan)
        hand.init()
        return hand

    def test_left_right_separation(self):
        """Test left and right hand board ID separation"""
        left_hand = LeftDexHand()
        right_hand = RightDexHand()

        # Verify base IDs are different
        assert left_hand.base_id == BoardID.LEFT_HAND_BASE
        assert right_hand.base_id == BoardID.RIGHT_HAND_BASE

        # Verify no ID overlap
        left_ids = set(range(left_hand.base_id,
                           left_hand.base_id + left_hand.NUM_BOARDS))
        right_ids = set(range(right_hand.base_id,
                            right_hand.base_id + right_hand.NUM_BOARDS))
        assert not (left_ids & right_ids)  # No intersection

    def test_board_command_ids(self, mock_hand):
        """Test command ID generation for boards"""
        for i in range(mock_hand.NUM_BOARDS):
            cmd_id = mock_hand._get_command_id(MessageType.MOTION_COMMAND, i)
            assert MessageType.MOTION_COMMAND <= cmd_id < MessageType.MOTION_FEEDBACK
            assert (cmd_id - MessageType.MOTION_COMMAND) >= mock_hand.base_id

        with pytest.raises(ValueError):
            mock_hand._get_command_id(MessageType.MOTION_COMMAND, mock_hand.NUM_BOARDS)  # Invalid board index

class TestNewCommands:
    """Test new commands: set_safe_temperature, current_motor_control_torque, set_stall_time"""

    @pytest.fixture
    def mock_hand(self):
        """Create a mock hand with configured mocks"""
        mock_zcan = Mock()
        mock_zcan.configure_channel.return_value = True
        hand = LeftDexHand(zcan=mock_zcan)
        hand.init()
        return hand

    def test_set_safe_temperature(self, mock_hand):
        """Test setting safe temperature"""

        # Test valid temperature
        with patch('pyzlg_dexhand.dexhand_interface.DexHandBase._send_command') as mock_send_command:
            assert mock_hand.set_safe_temperature(55)
            expected_data = 55 .to_bytes(1, 'little')
            expected_command = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_SAFE_TEMPERATURE]) + expected_data
            mock_send_command.assert_called_once_with(expected_command, None)
        # Test invalid temperature
        with patch('pyzlg_dexhand.dexhand_interface.DexHandBase._send_command') as mock_send_command:
            assert not mock_hand.set_safe_temperature(256)
            mock_send_command.assert_not_called()

    # Test for current_motor_control_torque removed as the method has been deprecated

    def test_set_stall_time(self, mock_hand):
        """Test setting stall time"""
        # Test valid stall time
        with patch('pyzlg_dexhand.dexhand_interface.DexHandBase._send_command') as mock_send_command:
            assert mock_hand.set_stall_time("motor1", 1000)
            expected_data = 1000 .to_bytes(2, 'little')
            expected_command = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_STALL_TIME_MOTOR1]) + expected_data
            mock_send_command.assert_called_once_with(expected_command, None)

        # Test invalid stall time
        with patch('pyzlg_dexhand.dexhand_interface.DexHandBase._send_command') as mock_send_command:
            assert not mock_hand.set_stall_time("invalid_motor", 65536)
            mock_send_command.assert_not_called()


    def test_set_pressure_limit_value(self ,mock_hand):
        """Test setting pressure limit value"""
        # Test valid pressure limit value
        with patch('pyzlg_dexhand.dexhand_interface.DexHandBase._send_command') as mock_send_command:
            assert mock_hand.set_pressure_limit_value(20)
            expected_data = 2000 .to_bytes(2, 'little')
            expected_command = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_PRESSURE_LIMIT_VALUE]) + expected_data
            mock_send_command.assert_called_once_with(expected_command, None)

        # Test invalid pressure limit value
        with patch('pyzlg_dexhand.dexhand_interface.DexHandBase._send_command') as mock_send_command:
            assert not mock_hand.set_pressure_limit_value(30)
            mock_send_command.assert_not_called()

    def test_set_pressure_limit_enable(self, mock_hand):
        """Test setting pressure limit enable"""
        # Test valid pressure limit enable
        with patch('pyzlg_dexhand.dexhand_interface.DexHandBase._send_command') as mock_send_command:
            assert mock_hand.set_pressure_limit_enable(True)
            expected_data = 1 .to_bytes(1, 'little')
            expected_command = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_PRESSURE_LIMIT_ENABLE]) + expected_data
            mock_send_command.assert_called_once_with(expected_command, None)

        # Test invalid pressure limit enable
        with patch('pyzlg_dexhand.dexhand_interface.DexHandBase._send_command') as mock_send_command:
            assert not mock_hand.set_pressure_limit_enable(2)
            mock_send_command.assert_not_called()

    def test_log_level_behavior(self, mock_hand):
        """Test log level behavior with different method and global levels"""
        # Test scenario 1: Global log level INFO + Method log level INFO
        # Expected: All levels of logs will be recorded
        with patch('pyzlg_dexhand.dexhand_interface.logger') as mock_logger:
            mock_hand.log_level = LogLevel.INFO
            
            # Call a method with various log levels
            mock_hand.move_joints(log_level=LogLevel.INFO)
            
            # Verify that all levels of logs are recorded (simulating possible calls in the method)
            mock_logger.info.assert_called()  # INFO should be called
            
            # Reset the mock and call another method with a different log level
            mock_logger.reset_mock()
            mock_hand.set_safe_temperature(55, log_level=LogLevel.INFO)
            mock_logger.debug.assert_called()  # DEBUG should also be called
        
        # Test scenario 2: Global log level INFO + Method log level DEBUG
        # Expected: Only DEBUG and ERROR levels of logs will be recorded, INFO level will not be recorded
        with patch('pyzlg_dexhand.dexhand_interface.logger') as mock_logger:
            mock_hand.log_level = LogLevel.INFO
            
            # Call a method with DEBUG level specified
            mock_hand.set_safe_temperature(55, log_level=LogLevel.DEBUG)
            
            # Verify that only DEBUG and ERROR levels of logs are recorded
            mock_logger.info.assert_not_called()  # INFO should not be called
            mock_logger.debug.assert_called()     # DEBUG should be called
            
            # Reset and test another method
            mock_logger.reset_mock()
            mock_hand.set_safe_temperature(55, log_level=LogLevel.DEBUG)
            mock_logger.info.assert_not_called()
            mock_logger.debug.assert_called()
        
        # Test scenario 3: Global log level DEBUG + Method log level INFO 
        # Expected: Method level overrides global level, all logs will be recorded
        with patch('pyzlg_dexhand.dexhand_interface.logger') as mock_logger:
            mock_hand.log_level = LogLevel.DEBUG
            
            # Call a method with INFO level specified
            mock_hand.move_joints(55, log_level=LogLevel.INFO)
            
            # Verify that all logs are recorded
            mock_logger.info.assert_called()
            
            # Reset and test another method
            mock_logger.reset_mock()
            mock_hand.reset_joints(log_level=LogLevel.INFO)
            mock_logger.info.assert_called()
        
        # Test scenario 4: Global log level DEBUG + Method level not specified
        # Expected: Use the global level, only DEBUG and ERROR logs will be recorded
        with patch('pyzlg_dexhand.dexhand_interface.logger') as mock_logger:
            mock_hand.log_level = LogLevel.DEBUG
            
            # Call methods without specifying log_level
            mock_hand.reset_joints()
            mock_hand.set_safe_temperature(55)
            
            # Verify that only DEBUG and ERROR levels of logs are recorded
            mock_logger.info.assert_not_called()
            mock_logger.debug.assert_called()
        
        # Test scenario 5: Global log level ERROR + Any method level
        # Expected: Only ERROR logs will be recorded, INFO and DEBUG will not be recorded
        with patch('pyzlg_dexhand.dexhand_interface.logger') as mock_logger:
            mock_hand.log_level = LogLevel.ERROR
            
            # Without specifying method level
            mock_hand.set_safe_temperature(55)
            
            # Verify that only ERROR level logs are recorded
            mock_logger.info.assert_not_called()
            mock_logger.debug.assert_not_called()
            
            # Also test method level override
            mock_logger.reset_mock()
            mock_hand.set_safe_temperature(55, log_level=LogLevel.DEBUG)
            mock_logger.info.assert_not_called()
            mock_logger.debug.assert_called()

if __name__ == '__main__':
    pytest.main([__file__])