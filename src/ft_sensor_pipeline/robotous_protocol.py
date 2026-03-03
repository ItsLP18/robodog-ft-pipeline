"""
Robotous RFT Series - UART Serial Protocol Handler
Based on: RFT-Series_Manual_ver1_8.pdf

Protocol Summary (UART - RS232/RS422/USB):
  Command Packet:  SOP(0x55) + DataField(8 bytes) + Checksum(1 byte) + EOP(0xAA) = 11 bytes
  Response Packet: SOP(0x55) + DataField(16 bytes) + Checksum(1 byte) + EOP(0xAA) = 19 bytes

Data Format for F/T readings:
  Each axis = 2 bytes (signed 16-bit integer, big-endian: upper_byte * 256 + lower_byte)
  Force (N)  = raw_int16 / DF
  Torque (Nm) = raw_int16 / DT

Dividers for RFT40-SA01: DF=50, DT=2000

Default UART settings: 115200 bps, 8N1, No flow control
Default output rate: 200Hz
"""

import serial
import time
from dataclasses import dataclass
from typing import Optional


# ============================================================================
# Sensor model dividers (from manual Section 3.6.11)
# ============================================================================
MODEL_DIVIDERS = {
    'RFT40-SA01': {'DF': 50, 'DT': 2000},
    'RFT44-SB01': {'DF': 50, 'DT': 2000},
    'RFT60-HA01': {'DF': 50, 'DT': 2000},
    'RFT64-SB01': {'DF': 50, 'DT': 2000},
    'RFT64-6A01': {'DF': 50, 'DT': 1000},
    'RFT80-6A01': {'DF': 50, 'DT': 1000},
    'RFT80-6A02': {'DF': 50, 'DT': 1000},
}

# ============================================================================
# Protocol constants (from manual Section 3.5.2)
# ============================================================================
SOP = 0x55
EOP = 0xAA
CMD_DATA_SIZE = 8
RESP_DATA_SIZE = 16
CMD_PACKET_SIZE = 1 + CMD_DATA_SIZE + 1 + 1      # 11 bytes
RESP_PACKET_SIZE = 1 + RESP_DATA_SIZE + 1 + 1    # 19 bytes

# ============================================================================
# Command IDs (from manual Section 3.6.1)
# ============================================================================
CMD_READ_MODEL_NAME     = 0x01
CMD_READ_SERIAL_NUMBER  = 0x02
CMD_READ_FIRMWARE_VER   = 0x03
CMD_SET_COMM_ID         = 0x04
CMD_READ_COMM_ID        = 0x05
CMD_SET_BAUDRATE        = 0x06
CMD_READ_BAUDRATE       = 0x07
CMD_SET_FILTER          = 0x08
CMD_READ_FILTER         = 0x09
CMD_READ_FT_ONCE        = 0x0A
CMD_START_FT_OUTPUT     = 0x0B
CMD_STOP_FT_OUTPUT      = 0x0C
CMD_SET_OUTPUT_RATE     = 0x0F
CMD_READ_OUTPUT_RATE    = 0x10
CMD_SET_BIAS            = 0x11
CMD_READ_OVERLOAD_COUNT = 0x12

# ============================================================================
# Baud rate parameters (from manual Section 3.6.7)
# ============================================================================
BAUDRATE_PARAMS = {
    115200: 0x00, 921600: 0x01, 460800: 0x02,
    230400: 0x03, 57600: 0x05,
}

# Output rate parameters (from manual Section 3.6.16)
OUTPUT_RATE_PARAMS = {
    200: 0x00, 10: 0x01, 20: 0x02, 50: 0x03,
    100: 0x04, 333: 0x06, 500: 0x07, 1000: 0x08,
}

# Filter cutoff parameters (from manual Section 3.6.9)
FILTER_CUTOFFS = {
    0: 0x00, 500: 0x01, 300: 0x02, 200: 0x03, 150: 0x04,
    100: 0x05, 50: 0x06, 40: 0x07, 30: 0x08, 20: 0x09,
    10: 0x0A, 5: 0x0B, 3: 0x0C, 2: 0x0D, 1: 0x0E,
}


@dataclass
class FTReading:
    """A single 6-axis force-torque reading with converted values."""
    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0
    overload_status: int = 0
    timestamp: float = 0.0


class RobotousProtocol:
    """
    UART protocol handler for Robotous RFT Series F/T sensors.
    Implements the protocol from RFT-Series_Manual_ver1_8.pdf.
    """

    def __init__(self, port="/dev/ttyUSB0", baud_rate=115200,
                 timeout=0.01, sensor_model="RFT40-SA01"):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_conn: Optional[serial.Serial] = None
        self._buffer = bytearray()

        dividers = MODEL_DIVIDERS.get(sensor_model, {'DF': 50, 'DT': 2000})
        self.DF = dividers['DF']
        self.DT = dividers['DT']
        self.sensor_model = sensor_model

    # --- Connection ---
    def connect(self) -> bool:
        try:
            self.serial_conn = serial.Serial(
                port=self.port, baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE, timeout=self.timeout
            )
            time.sleep(0.5)
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            return True
        except serial.SerialException as e:
            print(f"[RobotousProtocol] Connection failed on {self.port}: {e}")
            return False

    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.stop_streaming()
                time.sleep(0.05)
            except Exception:
                pass
            self.serial_conn.close()

    # --- Packet construction (Section 3.5.2) ---
    def _build_command(self, cmd_id: int, params: list = None) -> bytes:
        """Build: SOP + 8-byte data field + checksum + EOP"""
        data = bytearray(CMD_DATA_SIZE)
        data[0] = cmd_id
        if params:
            for i, p in enumerate(params):
                if i + 1 < CMD_DATA_SIZE:
                    data[i + 1] = p & 0xFF
        checksum = sum(data) & 0xFF
        return bytes([SOP]) + bytes(data) + bytes([checksum, EOP])

    def _validate_response(self, packet: bytes) -> Optional[bytearray]:
        """Validate SOP, EOP, checksum and return 16-byte data field."""
        if len(packet) != RESP_PACKET_SIZE:
            return None
        if packet[0] != SOP or packet[-1] != EOP:
            return None
        data = packet[1:1 + RESP_DATA_SIZE]
        if packet[1 + RESP_DATA_SIZE] != (sum(data) & 0xFF):
            return None
        return bytearray(data)

    # --- Send / Receive ---
    def _send(self, cmd_id: int, params: list = None):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.write(self._build_command(cmd_id, params))
            self.serial_conn.flush()

    def _receive(self, timeout_sec=0.5) -> Optional[bytearray]:
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
        start = time.time()
        buf = bytearray()
        while (time.time() - start) < timeout_sec:
            avail = self.serial_conn.in_waiting
            if avail > 0:
                buf.extend(self.serial_conn.read(avail))
            while len(buf) >= RESP_PACKET_SIZE:
                sop_idx = -1
                for i in range(len(buf)):
                    if buf[i] == SOP:
                        sop_idx = i
                        break
                if sop_idx == -1:
                    buf.clear()
                    break
                if sop_idx > 0:
                    buf = buf[sop_idx:]
                if len(buf) < RESP_PACKET_SIZE:
                    break
                data = self._validate_response(bytes(buf[:RESP_PACKET_SIZE]))
                if data is not None:
                    buf = buf[RESP_PACKET_SIZE:]
                    return data
                else:
                    buf = buf[1:]
            time.sleep(0.001)
        return None

    # --- Streaming read ---
    def read_one(self) -> Optional[FTReading]:
        """Read one F/T packet from the streaming buffer."""
        if not self.serial_conn or not self.serial_conn.is_open:
            return None

        avail = self.serial_conn.in_waiting
        if avail > 0:
            self._buffer.extend(self.serial_conn.read(avail))

        while len(self._buffer) >= RESP_PACKET_SIZE:
            # Find SOP
            sop_idx = -1
            for i in range(len(self._buffer)):
                if self._buffer[i] == SOP:
                    sop_idx = i
                    break
            if sop_idx == -1:
                self._buffer = self._buffer[-1:]
                return None
            if sop_idx > 0:
                self._buffer = self._buffer[sop_idx:]
            if len(self._buffer) < RESP_PACKET_SIZE:
                return None

            candidate = bytes(self._buffer[:RESP_PACKET_SIZE])

            # Check EOP
            if candidate[-1] != EOP:
                self._buffer = self._buffer[1:]
                continue

            # Check checksum
            data = candidate[1:1 + RESP_DATA_SIZE]
            if candidate[1 + RESP_DATA_SIZE] != (sum(data) & 0xFF):
                self._buffer = self._buffer[1:]
                continue

            # Valid packet
            self._buffer = self._buffer[RESP_PACKET_SIZE:]
            resp_id = data[0]

            if resp_id not in (CMD_READ_FT_ONCE, CMD_START_FT_OUTPUT):
                continue

            return self._parse_ft_data(bytearray(data))

        return None

    def _parse_ft_data(self, data: bytearray) -> FTReading:
        """
        Parse F/T response data field (Section 3.6.11, 3.7.2).

        D1 = Response ID
        D2,D3  = Fx upper/lower byte -> signed 16-bit
        D4,D5  = Fy
        D6,D7  = Fz
        D8,D9  = Tx
        D10,D11 = Ty
        D12,D13 = Tz
        D14 = Overload status

        Conversion: cell = upper*256 + lower -> cast to signed short
          Force = signed_short / DF
          Torque = signed_short / DT
        """
        raw = []
        for i in range(6):
            upper = data[2 * i + 1]
            lower = data[2 * i + 2]
            unsigned = (upper << 8) | lower
            signed = unsigned - 0x10000 if unsigned >= 0x8000 else unsigned
            raw.append(signed)

        overload = data[13] if len(data) > 13 else 0

        return FTReading(
            fx=raw[0] / self.DF,  fy=raw[1] / self.DF,  fz=raw[2] / self.DF,
            tx=raw[3] / self.DT,  ty=raw[4] / self.DT,  tz=raw[5] / self.DT,
            overload_status=overload,
            timestamp=time.time()
        )

    # --- High-level commands ---
    def start_streaming(self):
        """Start continuous F/T output (Section 3.6.12)."""
        self._buffer.clear()
        self._send(CMD_START_FT_OUTPUT)

    def stop_streaming(self):
        """Stop F/T output (Section 3.6.13). No response packet."""
        self._send(CMD_STOP_FT_OUTPUT)
        time.sleep(0.05)
        if self.serial_conn:
            self.serial_conn.reset_input_buffer()
        self._buffer.clear()

    def read_ft_once(self) -> Optional[FTReading]:
        """Single F/T read (Section 3.6.11)."""
        self._send(CMD_READ_FT_ONCE)
        data = self._receive(timeout_sec=0.5)
        if data and data[0] == CMD_READ_FT_ONCE:
            return self._parse_ft_data(data)
        return None

    def set_bias(self, enable=True):
        """Set/clear bias (Section 3.6.17). Can be sent during streaming."""
        self._send(CMD_SET_BIAS, [0x01 if enable else 0x00])
        time.sleep(0.05)

    def clear_bias(self):
        self.set_bias(enable=False)

    def set_output_rate(self, rate_hz: int) -> bool:
        """Set output rate (Section 3.6.14). Must stop streaming first."""
        if rate_hz not in OUTPUT_RATE_PARAMS:
            print(f"Invalid rate. Valid: {list(OUTPUT_RATE_PARAMS.keys())}")
            return False
        self._send(CMD_SET_OUTPUT_RATE, [OUTPUT_RATE_PARAMS[rate_hz]])
        data = self._receive()
        return bool(data and data[0] == CMD_SET_OUTPUT_RATE and data[1] == 0x01)

    def set_filter(self, cutoff_hz: int) -> bool:
        """Set internal LPF (Section 3.6.9). Must stop streaming first."""
        if cutoff_hz not in FILTER_CUTOFFS:
            print(f"Invalid cutoff. Valid: {list(FILTER_CUTOFFS.keys())}")
            return False
        ftype = 0x00 if cutoff_hz == 0 else 0x01
        fparam = FILTER_CUTOFFS[cutoff_hz]
        self._send(CMD_SET_FILTER, [ftype, fparam])
        data = self._receive()
        return bool(data and data[0] == CMD_SET_FILTER and data[1] == 0x01)

    def set_baudrate(self, baudrate: int) -> bool:
        """Set baud rate (Section 3.6.7). Takes effect after reboot."""
        if baudrate not in BAUDRATE_PARAMS:
            return False
        self._send(CMD_SET_BAUDRATE, [BAUDRATE_PARAMS[baudrate]])
        data = self._receive()
        return bool(data and data[0] == CMD_SET_BAUDRATE and data[1] == 0x01)

    def read_model_name(self) -> Optional[str]:
        self._send(CMD_READ_MODEL_NAME)
        data = self._receive()
        if data and data[0] == CMD_READ_MODEL_NAME:
            return bytes(data[1:16]).decode('ascii', errors='ignore').strip('\x00')
        return None

    def read_serial_number(self) -> Optional[str]:
        self._send(CMD_READ_SERIAL_NUMBER)
        data = self._receive()
        if data and data[0] == CMD_READ_SERIAL_NUMBER:
            return bytes(data[1:16]).decode('ascii', errors='ignore').strip('\x00')
        return None

    def read_firmware_version(self) -> Optional[str]:
        self._send(CMD_READ_FIRMWARE_VER)
        data = self._receive()
        if data and data[0] == CMD_READ_FIRMWARE_VER:
            return bytes(data[1:16]).decode('ascii', errors='ignore').strip('\x00')
        return None

    def read_overload_count(self) -> Optional[dict]:
        self._send(CMD_READ_OVERLOAD_COUNT)
        data = self._receive()
        if data and data[0] == CMD_READ_OVERLOAD_COUNT:
            return {
                'fx': data[1], 'fy': data[2], 'fz': data[3],
                'tx': data[4], 'ty': data[5], 'tz': data[6],
            }
        return None

    @staticmethod
    def parse_overload_status(status_byte: int) -> dict:
        """Parse overload byte: bit5=Fx, bit4=Fy, bit3=Fz, bit2=Tx, bit1=Ty, bit0=Tz"""
        return {
            'fx': bool(status_byte & 0x20), 'fy': bool(status_byte & 0x10),
            'fz': bool(status_byte & 0x08), 'tx': bool(status_byte & 0x04),
            'ty': bool(status_byte & 0x02), 'tz': bool(status_byte & 0x01),
        }

    def check_overload(self, reading: FTReading) -> dict:
        overloads = self.parse_overload_status(reading.overload_status)
        return {k: v for k, v in overloads.items() if v}
