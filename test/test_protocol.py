"""
Unit tests for the Robotous RFT Series UART protocol handler.
Tests packet construction, validation, and data conversion without hardware.
"""

import sys
import os
import pytest

# Add source to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from ft_sensor_pipeline.robotous_protocol import (
    RobotousProtocol, FTReading,
    SOP, EOP, CMD_DATA_SIZE, RESP_DATA_SIZE,
    CMD_PACKET_SIZE, RESP_PACKET_SIZE,
    CMD_START_FT_OUTPUT, CMD_READ_FT_ONCE, CMD_STOP_FT_OUTPUT,
    CMD_SET_BIAS, MODEL_DIVIDERS,
)


class TestPacketConstruction:
    """Test command packet building."""

    def setup_method(self):
        self.proto = RobotousProtocol.__new__(RobotousProtocol)
        self.proto.DF = 50
        self.proto.DT = 2000

    def test_command_packet_size(self):
        """Command packets must be exactly 11 bytes."""
        pkt = self.proto._build_command(CMD_START_FT_OUTPUT)
        assert len(pkt) == CMD_PACKET_SIZE == 11

    def test_command_packet_framing(self):
        """Packets must start with SOP(0x55) and end with EOP(0xAA)."""
        pkt = self.proto._build_command(CMD_START_FT_OUTPUT)
        assert pkt[0] == SOP == 0x55
        assert pkt[-1] == EOP == 0xAA

    def test_command_id_placement(self):
        """Command ID should be the first byte of the data field."""
        pkt = self.proto._build_command(CMD_READ_FT_ONCE)
        assert pkt[1] == CMD_READ_FT_ONCE == 0x0A

    def test_command_params(self):
        """Parameters should follow the command ID in the data field."""
        pkt = self.proto._build_command(CMD_SET_BIAS, [0x01])
        assert pkt[1] == CMD_SET_BIAS
        assert pkt[2] == 0x01

    def test_checksum_calculation(self):
        """Checksum = sum of data field bytes & 0xFF."""
        pkt = self.proto._build_command(CMD_START_FT_OUTPUT)
        data_field = pkt[1:1 + CMD_DATA_SIZE]
        expected_checksum = sum(data_field) & 0xFF
        actual_checksum = pkt[1 + CMD_DATA_SIZE]
        assert actual_checksum == expected_checksum

    def test_start_streaming_packet(self):
        """Verify the exact bytes for Start F/T Output command."""
        pkt = self.proto._build_command(CMD_START_FT_OUTPUT)
        # SOP=0x55, data=[0x0B, 0,0,0,0,0,0,0], checksum=0x0B, EOP=0xAA
        assert pkt[0] == 0x55
        assert pkt[1] == 0x0B
        assert pkt[2:9] == bytes(7)  # padding zeros
        assert pkt[9] == 0x0B        # checksum
        assert pkt[10] == 0xAA

    def test_stop_streaming_packet(self):
        """Verify Stop F/T Output command."""
        pkt = self.proto._build_command(CMD_STOP_FT_OUTPUT)
        assert pkt[1] == 0x0C

    def test_bias_set_packet(self):
        """Verify Set Bias command with enable=True."""
        pkt = self.proto._build_command(CMD_SET_BIAS, [0x01])
        assert pkt[1] == 0x11
        assert pkt[2] == 0x01

    def test_bias_clear_packet(self):
        """Verify Set Bias command with enable=False."""
        pkt = self.proto._build_command(CMD_SET_BIAS, [0x00])
        assert pkt[1] == 0x11
        assert pkt[2] == 0x00


class TestResponseValidation:
    """Test response packet validation."""

    def setup_method(self):
        self.proto = RobotousProtocol.__new__(RobotousProtocol)
        self.proto.DF = 50
        self.proto.DT = 2000

    def _make_response(self, data_field: list) -> bytes:
        """Build a valid response packet from a 16-byte data field."""
        assert len(data_field) == RESP_DATA_SIZE
        checksum = sum(data_field) & 0xFF
        return bytes([SOP] + data_field + [checksum, EOP])

    def test_valid_response(self):
        """A well-formed response should be accepted."""
        data = [CMD_READ_FT_ONCE] + [0] * 15
        pkt = self._make_response(data)
        result = self.proto._validate_response(pkt)
        assert result is not None
        assert result[0] == CMD_READ_FT_ONCE

    def test_wrong_sop_rejected(self):
        """Response with wrong SOP should be rejected."""
        data = [CMD_READ_FT_ONCE] + [0] * 15
        pkt = self._make_response(data)
        pkt = bytes([0x00]) + pkt[1:]  # corrupt SOP
        result = self.proto._validate_response(pkt)
        assert result is None

    def test_wrong_eop_rejected(self):
        """Response with wrong EOP should be rejected."""
        data = [CMD_READ_FT_ONCE] + [0] * 15
        pkt = self._make_response(data)
        pkt = pkt[:-1] + bytes([0x00])  # corrupt EOP
        result = self.proto._validate_response(pkt)
        assert result is None

    def test_bad_checksum_rejected(self):
        """Response with corrupted checksum should be rejected."""
        data = [CMD_READ_FT_ONCE] + [0] * 15
        pkt = bytearray(self._make_response(data))
        pkt[-2] = (pkt[-2] + 1) & 0xFF  # corrupt checksum
        result = self.proto._validate_response(bytes(pkt))
        assert result is None

    def test_wrong_length_rejected(self):
        """Response with wrong length should be rejected."""
        result = self.proto._validate_response(bytes(10))
        assert result is None

    def test_response_packet_size(self):
        """Valid response packets are exactly 19 bytes."""
        data = [0x0A] + [0] * 15
        pkt = self._make_response(data)
        assert len(pkt) == RESP_PACKET_SIZE == 19


class TestDataConversion:
    """Test F/T data parsing and unit conversion."""

    def setup_method(self):
        self.proto = RobotousProtocol.__new__(RobotousProtocol)
        self.proto.DF = 50    # RFT40-SA01
        self.proto.DT = 2000  # RFT40-SA01

    def _make_ft_data(self, raw_values: list) -> bytearray:
        """Build a 16-byte F/T data field from 6 signed int16 values."""
        data = bytearray(16)
        data[0] = CMD_READ_FT_ONCE  # Response ID
        for i, val in enumerate(raw_values):
            # Convert signed int16 to unsigned for byte encoding
            if val < 0:
                val = val + 0x10000
            data[2 * i + 1] = (val >> 8) & 0xFF  # upper byte
            data[2 * i + 2] = val & 0xFF          # lower byte
        return data

    def test_zero_reading(self):
        """All zeros should produce zero forces and torques."""
        data = self._make_ft_data([0, 0, 0, 0, 0, 0])
        reading = self.proto._parse_ft_data(data)
        assert reading.fx == 0.0
        assert reading.fy == 0.0
        assert reading.fz == 0.0
        assert reading.tx == 0.0
        assert reading.ty == 0.0
        assert reading.tz == 0.0

    def test_positive_force(self):
        """raw=500 → 500/50 = 10.0N"""
        data = self._make_ft_data([500, 0, 0, 0, 0, 0])
        reading = self.proto._parse_ft_data(data)
        assert reading.fx == pytest.approx(10.0)

    def test_negative_force(self):
        """raw=-500 → -500/50 = -10.0N"""
        data = self._make_ft_data([-500, 0, 0, 0, 0, 0])
        reading = self.proto._parse_ft_data(data)
        assert reading.fx == pytest.approx(-10.0)

    def test_torque_conversion(self):
        """raw=1000 → 1000/2000 = 0.5Nm"""
        data = self._make_ft_data([0, 0, 0, 1000, 0, 0])
        reading = self.proto._parse_ft_data(data)
        assert reading.tx == pytest.approx(0.5)

    def test_negative_torque(self):
        """raw=-2000 → -2000/2000 = -1.0Nm"""
        data = self._make_ft_data([0, 0, 0, -2000, 0, 0])
        reading = self.proto._parse_ft_data(data)
        assert reading.tx == pytest.approx(-1.0)

    def test_full_scale_force(self):
        """Fx full scale: 100N → raw = 100*50 = 5000"""
        data = self._make_ft_data([5000, 0, 0, 0, 0, 0])
        reading = self.proto._parse_ft_data(data)
        assert reading.fx == pytest.approx(100.0)

    def test_full_scale_torque(self):
        """Tz full scale: 2.5Nm → raw = 2.5*2000 = 5000"""
        data = self._make_ft_data([0, 0, 0, 0, 0, 5000])
        reading = self.proto._parse_ft_data(data)
        assert reading.tz == pytest.approx(2.5)

    def test_all_axes(self):
        """Verify all 6 axes parse independently."""
        data = self._make_ft_data([100, 200, 300, 400, 500, 600])
        reading = self.proto._parse_ft_data(data)
        assert reading.fx == pytest.approx(2.0)    # 100/50
        assert reading.fy == pytest.approx(4.0)    # 200/50
        assert reading.fz == pytest.approx(6.0)    # 300/50
        assert reading.tx == pytest.approx(0.2)    # 400/2000
        assert reading.ty == pytest.approx(0.25)   # 500/2000
        assert reading.tz == pytest.approx(0.3)    # 600/2000

    def test_int16_boundary_positive(self):
        """Max signed int16 = 32767"""
        data = self._make_ft_data([32767, 0, 0, 0, 0, 0])
        reading = self.proto._parse_ft_data(data)
        assert reading.fx == pytest.approx(32767 / 50)

    def test_int16_boundary_negative(self):
        """Min signed int16 = -32768"""
        data = self._make_ft_data([-32768, 0, 0, 0, 0, 0])
        reading = self.proto._parse_ft_data(data)
        assert reading.fx == pytest.approx(-32768 / 50)

    def test_reading_has_timestamp(self):
        """Readings should have a non-zero timestamp."""
        data = self._make_ft_data([0, 0, 0, 0, 0, 0])
        reading = self.proto._parse_ft_data(data)
        assert reading.timestamp > 0


class TestOverloadParsing:
    """Test overload status byte parsing."""

    def test_no_overload(self):
        status = RobotousProtocol.parse_overload_status(0x00)
        assert not any(status.values())

    def test_fx_overload(self):
        status = RobotousProtocol.parse_overload_status(0x20)
        assert status['fx'] is True
        assert status['fy'] is False

    def test_all_overload(self):
        status = RobotousProtocol.parse_overload_status(0x3F)
        assert all(status.values())

    def test_torque_only_overload(self):
        status = RobotousProtocol.parse_overload_status(0x07)
        assert status['tx'] is True
        assert status['ty'] is True
        assert status['tz'] is True
        assert status['fx'] is False
        assert status['fy'] is False
        assert status['fz'] is False


class TestModelDividers:
    """Test that model dividers are correctly loaded."""

    def test_rft40_sa01_dividers(self):
        proto = RobotousProtocol.__new__(RobotousProtocol)
        dividers = MODEL_DIVIDERS['RFT40-SA01']
        assert dividers['DF'] == 50
        assert dividers['DT'] == 2000

    def test_rft80_6a02_dividers(self):
        dividers = MODEL_DIVIDERS['RFT80-6A02']
        assert dividers['DF'] == 50
        assert dividers['DT'] == 1000

    def test_unknown_model_defaults(self):
        """Unknown models should get safe defaults."""
        proto = RobotousProtocol.__new__(RobotousProtocol)
        dividers = MODEL_DIVIDERS.get('UNKNOWN', {'DF': 50, 'DT': 2000})
        assert dividers['DF'] == 50
        assert dividers['DT'] == 2000
