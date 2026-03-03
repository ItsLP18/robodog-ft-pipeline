# Robotous RFT Series — UART Protocol Reference

> Based on: *RFT-Series Installation and Operation Manual, Revision 1.8*

## Overview

The Robotous RFT Series sensors communicate over UART using a binary framed protocol. The USB interface variant (code D) uses an FTDI FT230X chip, presenting as a virtual COM port (`/dev/ttyUSB0`).

## Physical Layer

| Parameter | Value |
|-----------|-------|
| Baud Rate | 115,200 bps (default) |
| Data Bits | 8 |
| Parity | None |
| Stop Bits | 1 |
| Flow Control | None |

Supported baud rates: 57600, 115200, 230400, 460800, 921600

## Frame Structure

### Command Frame (Host → Sensor)

```
┌──────┬────────────────────────────────────────────┬──────────┬──────┐
│ SOP  │              Data Field (8 bytes)          │ Checksum │ EOP  │
│ 0x55 │ CMD_ID │ P1 │ P2 │ P3 │ P4 │ P5 │ P6 │ P7 │  Σdata   │ 0xAA │
└──────┴────────────────────────────────────────────┴──────────┴──────┘
Total: 11 bytes
```

### Response Frame (Sensor → Host)

```
┌──────┬───────────────────────────────────────────────────────────────────────┬──────────┬──────┐
│ SOP  │                        Data Field (16 bytes)                         │ Checksum │ EOP  │
│ 0x55 │ RESP_ID │ D1 │ D2 │ D3 │ D4 │ D5 │ D6 │ D7 │ D8 │...│ D14│ D15│   │  Σdata   │ 0xAA │
└──────┴───────────────────────────────────────────────────────────────────────┴──────────┴──────┘
Total: 19 bytes
```

### Checksum

```
checksum = (sum of all data field bytes) & 0xFF
```

## Command Set

| ID | Command | Params | Response | Notes |
|----|---------|--------|----------|-------|
| `0x01` | Read Model Name | 0 | ASCII string | |
| `0x02` | Read Serial Number | 0 | ASCII string | |
| `0x03` | Read Firmware Version | 0 | ASCII string | |
| `0x06` | Set Baud Rate | 1 | Success/Error | UART only, takes effect on reboot |
| `0x07` | Read Baud Rate | 0 | Current + pending | |
| `0x08` | Set Filter | 2 | Success/Error | Type + Parameter |
| `0x09` | Read Filter Setting | 0 | Type + Parameter | |
| `0x0A` | Read F/T Data (once) | 0 | F/T frame | Single shot |
| `0x0B` | Start F/T Output | 0 | F/T frames | Continuous streaming |
| `0x0C` | Stop F/T Output | 0 | *None* | Can send during streaming |
| `0x0F` | Set Output Rate | 1 | Success/Error | Stop streaming first |
| `0x10` | Read Output Rate | 0 | Rate parameter | Can send during streaming |
| `0x11` | Set Bias | 1 | *None* | Can send during streaming |
| `0x12` | Read Overload Count | 0 | 6 counters | |

## F/T Data Frame

The response data field for commands `0x0A` and `0x0B`:

```
Byte:  D1    D2   D3   D4   D5   D6   D7   D8   D9   D10  D11  D12  D13  D14-D16
       ID    Fx_H Fx_L Fy_H Fy_L Fz_H Fz_L Tx_H Tx_L Ty_H Ty_L Tz_H Tz_L OVL XX
```

### Data Conversion

```
unsigned_16bit = upper_byte × 256 + lower_byte
signed_16bit   = (int16_t) unsigned_16bit       // Two's complement

Force  (N)  = signed_16bit / DF
Torque (Nm) = signed_16bit / DT
```

### Model Dividers

| Model | DF | DT |
|-------|----|----|
| RFT40-SA01 | 50 | 2000 |
| RFT44-SB01 | 50 | 2000 |
| RFT60-HA01 | 50 | 2000 |
| RFT64-SB01 | 50 | 2000 |
| RFT64-6A01 | 50 | 1000 |
| RFT80-6A01 | 50 | 1000 |
| RFT80-6A02 | 50 | 1000 |

### Overload Status Byte (D14)

```
Bit:  7      6      5    4    3    2    1    0
      Rsvd   Rsvd   Fx   Fy   Fz   Tx   Ty   Tz
```

Bit = 1 when the axis exceeds **120%** of rated load capacity.

## Allowable Output Rates (UART)

| Baud Rate | 10 | 20 | 50 | 100 | 200 | 333 | 500 | 1000 Hz |
|-----------|----|----|-----|-----|-----|-----|-----|---------|
| 57,600 | ✓ | ✓ | ✓ | ✓ | ✓ | ✗ | ✗ | ✗ |
| 115,200 | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✗ | ✗ |
| 230,400 | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✗ |
| 460,800 | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✗ |
| 921,600 | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |

## Internal Filter Options

| Parameter | Cutoff (Hz) |
|-----------|-------------|
| `0x00` | Off |
| `0x01` | 500 |
| `0x02` | 300 |
| `0x03` | 200 |
| `0x04` | 150 |
| `0x05` | 100 |
| `0x06` | 50 |
| `0x07` | 40 |
| `0x08` | 30 |
| `0x09` | 20 |
| `0x0A` | 10 |
| `0x0B` | 5 |
| `0x0C` | 3 |
| `0x0D` | 2 |
| `0x0E` | 1 |

## Error Codes

| Code | Description |
|------|-------------|
| `0x01` | Unsupported command |
| `0x02` | Parameter out of range |
| `0x03` | Failed to set parameters |
