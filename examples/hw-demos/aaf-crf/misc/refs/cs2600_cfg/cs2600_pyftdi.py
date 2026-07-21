#!/usr/bin/env python3
"""
CS2600 bringup tool via pyftdi over CDB-CLOCKING (Hazelburn) USB board.

Step-by-step approach to establishing communication and configuring the PLL.
Every write is read back and verified. Designed for iterative debugging.

Usage:
  python3 cs2600_pyftdi.py --list                 # find FTDI devices
  python3 cs2600_pyftdi.py --probe                 # establish comms only
  python3 cs2600_pyftdi.py --dump                  # read all key registers
  python3 cs2600_pyftdi.py --configure             # full 300Hz mult config
  python3 cs2600_pyftdi.py --write 0x0058 0x005A   # manual register write
  python3 cs2600_pyftdi.py --read 0x0110           # manual register read

Transport:
  Default is SPI (Hazelburn CSV dump shows "SPI - 0x1").
  Use --i2c for I2C mode.

Prerequisites:
  pip3 install pyftdi
  On Linux: udev rule for FTDI, or run as root
"""

import sys
import time
import argparse


# --- Register map ---
REGS = {
    0x0002: "PLL_CFG1",
    0x0004: "PLL_CFG2",
    0x0006: "RATIO1_1",
    0x0008: "RATIO1_2",
    0x000A: "RATIO2_1",
    0x000C: "RATIO2_2",
    0x0016: "PLL_CFG3",
    0x001E: "PLL_CFG4",
    0x0058: "SW_RESET",
    0x0064: "DRIVE_STRENGTH1",
    0x0100: "OUTPUT_CFG1",
    0x0102: "OUTPUT_CFG2",
    0x0104: "ARC_CFG1",
    0x0108: "PHASE_ALIGN_CFG1",
    0x0110: "DEVICE_ID1",
    0x0112: "DEVICE_ID2",
    0x0114: "UNLOCK_IND",
    0x0116: "ERROR_STS",
    0x1104: "USER_KEY",
}


class CS2600_SPI:
    """CS2600 SPI transport via pyftdi.

    SPI protocol (datasheet Fig 4-25/4-26, Table 3-6):
      Write: [R/W=0 | A14..A0] [16-bit pad] [D15..D0]  total 48 clocks
      Read:  [R/W=1 | A14..A0] [16-bit pad] [D15..D0 on SDO]
      CPOL=0, CPHA=0 (Mode 0). Data on rising SCK edge.
      Max SCK = 17.5 MHz for config regs, 4.5 MHz for OTP.
    """

    def __init__(self, url, cs=0, freq=1_000_000):
        from pyftdi.spi import SpiController
        self._ctrl = SpiController(cs_count=1)
        self._ctrl.configure(url)
        # Try Mode 0 first (CPOL=0, CPHA=0) - datasheet says rising edge
        self._port = self._ctrl.get_port(cs=cs, freq=freq, mode=0)
        self._cs = cs
        self._freq = freq
        self._url = url
        print(f"[SPI] {url}  CS={cs}  freq={freq/1e6:.2f} MHz  mode=0")

    def write_reg(self, reg, val):
        b0 = (reg >> 8) & 0x7F
        b1 = reg & 0xFF
        cmd = bytes([b0, b1, 0x00, 0x00, (val >> 8) & 0xFF, val & 0xFF])
        self._port.exchange(cmd, duplex=True)

    def read_reg(self, reg):
        b0 = 0x80 | ((reg >> 8) & 0x7F)
        b1 = reg & 0xFF
        # CS2600 SPI read (Fig 4-26):
        #   SDI: [1|A14..A0] [16-bit don't care] [16-bit don't care]
        #   SDO: [hi-z 16]  [hi-z 16]           [D15..D0]
        # Total 48 clocks with CS held low
        cmd = bytes([b0, b1, 0x00, 0x00, 0x00, 0x00])
        resp = self._port.exchange(cmd, duplex=True)
        return (resp[4] << 8) | resp[5]

    def close(self):
        self._ctrl.close()


class CS2600_I2C:
    """CS2600 I2C transport via pyftdi.

    I2C protocol (datasheet Fig 4-19/4-20, Table 4-5):
      16-bit register address + 16-bit data word.
      Hazelburn board: I2C_ADDR pin has 100k to GND -> 0x58(W)/0x59(R) = 7-bit 0x2C
    """

    def __init__(self, url, addr7=0x2C):
        from pyftdi.i2c import I2cController
        self._ctrl = I2cController()
        self._ctrl.configure(url)
        self._port = self._ctrl.get_port(addr7)
        print(f"[I2C] {url}  7-bit addr=0x{addr7:02X}")

    def write_reg(self, reg, val):
        self._port.write(bytes([
            (reg >> 8) & 0xFF, reg & 0xFF,
            (val >> 8) & 0xFF, val & 0xFF
        ]))

    def read_reg(self, reg):
        data = self._port.exchange(
            bytes([(reg >> 8) & 0xFF, reg & 0xFF]), 2
        )
        return (data[0] << 8) | data[1]

    def close(self):
        self._ctrl.close()


# --- Utility functions ---

def reg_name(addr):
    return REGS.get(addr, f"0x{addr:04X}")


def write_verify(dev, reg, val, label=None):
    """Write a register and read it back. Returns True if verified."""
    dev.write_reg(reg, val)
    time.sleep(0.001)
    readback = dev.read_reg(reg)
    name = label or reg_name(reg)
    ok = readback == val
    status = "OK" if ok else "MISMATCH"
    print(f"  WR 0x{reg:04X} ({name:16s}) <- 0x{val:04X}  "
          f"RD -> 0x{readback:04X}  [{status}]")
    if not ok:
        print(f"     *** EXPECTED 0x{val:04X}, GOT 0x{readback:04X} ***")
    return ok


def read_show(dev, reg, label=None):
    """Read a register and display it."""
    val = dev.read_reg(reg)
    name = label or reg_name(reg)
    print(f"  RD 0x{reg:04X} ({name:16s}) = 0x{val:04X}")
    return val


def list_devices():
    from pyftdi.ftdi import Ftdi
    print("Scanning for FTDI devices...")
    devices = Ftdi.list_devices()
    if not devices:
        print("  No FTDI devices found.")
        print("  Check: USB cable? udev rules? (or run as root)")
        return
    for i, (desc, ifcount) in enumerate(devices):
        print(f"  [{i}] {desc}  interfaces={ifcount}")
        # desc is a UsbDeviceDescriptor namedtuple
        try:
            vid = desc.vid
            pid = desc.pid
            sn = desc.sn
            for iface in range(1, ifcount + 1):
                print(f"      ftdi://ftdi:{pid:04x}:{sn}/{iface}")
        except AttributeError:
            # Fallback: just print the tuple
            pass


def decode_error_sts(val):
    flags = []
    if val & 0x01: flags.append("CLK_IN_MISSING")
    if val & 0x02: flags.append("CLK_IN_UNSTABLE")
    if val & 0x04: flags.append("REF_CLK_MISSING")
    if val & 0x08: flags.append("INVALID_HW_CFG")
    if val & 0x10: flags.append("PLL_DISABLED")
    if val & 0x20: flags.append("INVALID_REG_CFG")
    if val & 0x40: flags.append("OTP_CORRUPT")
    if val & 0x80: flags.append("DEVICE_DEFECTIVE")
    return " | ".join(flags) if flags else "none"


def decode_unlock_ind(val):
    parts = []
    parts.append(f"F_LOCK={'YES' if not (val & 0x01) else 'NO'}")
    parts.append(f"F_STICKY={'1' if (val & 0x02) else '0'}")
    parts.append(f"P_LOCK={'YES' if not (val & 0x04) else 'NO'}")
    parts.append(f"P_STICKY={'1' if (val & 0x08) else '0'}")
    return "  ".join(parts)


# --- Commands ---

def cmd_probe(dev):
    """Just verify communication works."""
    print("\n--- Probe: read device ID ---")
    devid = dev.read_reg(0x0110)
    devid2 = dev.read_reg(0x0112)
    print(f"  DEVICE_ID1 = 0x{devid:04X}  {'[CS2600 OK]' if devid == 0x2600 else '[UNEXPECTED]'}")
    print(f"  DEVICE_ID2 = 0x{devid2:04X}  (revision)")
    if devid != 0x2600:
        print("\n  FAILED: Cannot communicate with CS2600.")
        print("  Troubleshooting:")
        print("    - S1 rotary at P8? (software control mode)")
        print("    - CONFIG1 pin grounded?")
        print("    - Board powered? (USB LED on?)")
        print("    - SPI vs I2C: the mode is latched at power-on by")
        print("      I2C_ADDR/SPI_CS pin state. Try --i2c if SPI fails.")
        print("    - Try lower freq: --freq 100000")
        print("    - Try --spi-scan to test all SPI modes")
        return False
    # Also show current status
    unlock = dev.read_reg(0x0114)
    err = dev.read_reg(0x0116)
    print(f"  UNLOCK_IND = 0x{unlock:04X}  ({decode_unlock_ind(unlock)})")
    print(f"  ERROR_STS  = 0x{err:04X}  ({decode_error_sts(err)})")
    return True


def cmd_i2c_scan(args):
    """Scan all I2C addresses to find the CS2600."""
    from pyftdi.i2c import I2cController

    url = args.url
    if not url:
        from pyftdi.ftdi import Ftdi
        devices = Ftdi.list_devices()
        if not devices:
            print("No FTDI devices")
            return
        desc, _ = devices[0]
        url = f"ftdi://ftdi:232h:{desc.sn}/1"

    print(f"\n--- I2C bus scan on {url} ---")
    print("  Probing all 7-bit addresses (0x08-0x77)...")
    print()

    ctrl = I2cController()
    ctrl.configure(url)

    found = []
    for addr in range(0x08, 0x78):
        port = ctrl.get_port(addr)
        try:
            port.read(0)  # zero-length read to check ACK
            found.append(addr)
        except Exception:
            pass

    ctrl.terminate()

    if found:
        print(f"  Devices found at: {', '.join(f'0x{a:02X}' for a in found)}")
        print()
        # CS2600 expected addresses from datasheet Table 4-5:
        cs2600_addrs = {0x2C: "100k to GND", 0x2A: "22k to GND",
                        0x2B: "4.7k to GND", 0x29: "0 to GND",
                        0x2E: "4.7k to VDD", 0x2F: "0 to VDD",
                        0x28: "100k to VDD", 0x2D: "22k to VDD"}
        for a in found:
            if a in cs2600_addrs:
                print(f"  0x{a:02X} -> CS2600 ({cs2600_addrs[a]})")
            else:
                print(f"  0x{a:02X} -> unknown device")
    else:
        print("  No devices found on I2C bus!")
        print("  -> Device may be in SPI mode but CS not wired correctly")
        print("  -> Or device not powered")
        print("  -> Try power-cycling the board (unplug/replug USB)")


def cmd_spi_scan(args):
    """Try all SPI modes and CS polarities to find the working combo."""
    from pyftdi.spi import SpiController

    url = args.url
    if not url:
        from pyftdi.ftdi import Ftdi
        devices = Ftdi.list_devices()
        if not devices:
            print("No FTDI devices")
            return
        desc, _ = devices[0]
        url = f"ftdi://ftdi:232h:{desc.sn}/1"

    print(f"\n--- SPI bus scan on {url} ---")
    print("  Testing all SPI modes (0-3) reading DEVICE_ID1 @ 0x0110")
    print()

    for mode in range(4):
        ctrl = SpiController(cs_count=1)
        ctrl.configure(url)
        port = ctrl.get_port(cs=0, freq=args.freq, mode=mode)

        # Read DEVICE_ID1
        reg = 0x0110
        b0 = 0x80 | ((reg >> 8) & 0x7F)
        b1 = reg & 0xFF
        cmd = bytes([b0, b1, 0x00, 0x00, 0x00, 0x00])
        resp = port.exchange(cmd, duplex=True)
        val = (resp[4] << 8) | resp[5]

        hit = " *** CS2600! ***" if val == 0x2600 else ""
        print(f"  Mode {mode} (CPOL={mode>>1}, CPHA={mode&1}): "
              f"resp={resp.hex()}  ID=0x{val:04X}{hit}")

        ctrl.terminate()

    print("\n  If all 0xFF: CS line not reaching device, or device not powered")
    print("  If all 0x00: SDO stuck low (wrong CS, or device in reset)")
    print("  If mixed garbage: mode mismatch (one mode should give 0x2600)")


def cmd_dump(dev):
    """Dump all important registers."""
    print("\n--- Register dump ---")
    addrs = sorted(REGS.keys())
    for addr in addrs:
        if addr == 0x0058:  # skip SW_RESET (write-only)
            continue
        val = dev.read_reg(addr)
        extra = ""
        if addr == 0x0110:
            extra = f"  {'[CS2600]' if val == 0x2600 else '[???]'}"
        elif addr == 0x0114:
            extra = f"  ({decode_unlock_ind(val)})"
        elif addr == 0x0116:
            extra = f"  ({decode_error_sts(val)})"
        print(f"  0x{addr:04X} {REGS[addr]:20s} = 0x{val:04X}{extra}")


def cmd_configure(dev):
    """Step-by-step 300 Hz multiplier configuration with full verification.

    Goal: CLK_IN=300Hz -> CLK_OUT=24.576MHz, BCLK=3.072MHz, FSYNC=48kHz

    Strategy: Match the SoundClear Studio sequence exactly, but verify
    every single write. If any write fails, stop and report.
    """
    print("\n" + "="*60)
    print("CS2600 300 Hz Multiplier Configuration")
    print("="*60)

    # Step 1: Software reset
    print("\n[1/7] Software reset")
    dev.write_reg(0x0058, 0x005A)
    time.sleep(0.010)  # 10ms to be safe
    devid = dev.read_reg(0x0110)
    print(f"  SW_RESET sent, waited 10ms")
    print(f"  DEVICE_ID1 readback = 0x{devid:04X}  "
          f"{'[OK]' if devid == 0x2600 else '[FAIL]'}")
    if devid != 0x2600:
        print("  ABORT: device not responding after reset")
        return False

    # Step 2: Set mode and ratio (PLL still disabled)
    print("\n[2/7] PLL mode selection (PLL disabled)")
    ok = True
    # S_RATIO_SEL=Ratio2 for holdover (bits 12:11 = 01)
    ok &= write_verify(dev, 0x0002, 0x0880, "PLL_CFG1: S_RATIO=R2")
    # PLL_MODE_SEL=1 (Multiplier), M_RATIO_SEL=00 (Ratio1), bit3 reserved
    ok &= write_verify(dev, 0x0004, 0x0009, "PLL_CFG2: Mult,R1")
    if not ok:
        print("  ABORT: mode registers failed to write")
        return False

    # Step 3: Set ratio
    print("\n[3/7] Ratio: 81920.0 in Q20.12 = 0x14000000")
    ok = True
    ok &= write_verify(dev, 0x0006, 0x1400, "RATIO1_1 [31:16]")
    ok &= write_verify(dev, 0x0008, 0x0000, "RATIO1_2 [15:0]")
    if not ok:
        print("  ABORT: ratio registers failed")
        return False

    # Step 4: Reference and FLL config
    print("\n[4/7] Reference clock and FLL bandwidth")
    ok = True
    # PLL_CFG3: RATIO_CFG=0 (20.12 format), REF_CLK_IN_DIV=÷1 (bits 4:3=10),
    #           SYSCLK_SRC=REF_CLK_IN (bits 2:1=01)
    ok &= write_verify(dev, 0x0016, 0x0012, "PLL_CFG3: ÷1,REF_CLK")
    # PLL_CFG4: FLL_BW=1Hz (bits 6:4=000), FLL_BW_MOD=x1 (bit7=0)
    ok &= write_verify(dev, 0x001E, 0x0000, "PLL_CFG4: 1Hz x1")
    if not ok:
        print("  ABORT: PLL config registers failed")
        return False

    # Step 5: Output configuration
    print("\n[5/7] Output configuration")
    ok = True
    # OUTPUT_CFG1: BCLK_DIV=0x5(÷8), FSYNC_DIV=0x5(÷512),
    #   BCLK_INV=1, FSYNC_INV=1, FSYNC_DUTY=50%
    ok &= write_verify(dev, 0x0100, 0x5582, "OUT_CFG1: ÷8,÷512,INV")
    # OUTPUT_CFG2: AUX1_OUT_SEL=001(CLK_IN), CLK_OUT_SEL=00(MCLK)
    ok &= write_verify(dev, 0x0102, 0x0400, "OUT_CFG2: AUX1=CLK_IN")
    # Phase alignment: EN=1, MODE=automatic(0), THR=8 MCLK(010), SPEED=00
    ok &= write_verify(dev, 0x0108, 0x8008, "PHASE_ALIGN: EN,auto")
    if not ok:
        print("  ABORT: output config registers failed")
        return False

    # Step 6: Enable PLL
    print("\n[6/7] Enable PLL")
    # PLL_EN1=1 (bit8), keep S_RATIO_SEL=Ratio2 (bits 12:11=01)
    dev.write_reg(0x0002, 0x0980)
    time.sleep(0.001)
    # PLL_EN2=1 (bit8), keep MODE=Multiplier, M_RATIO=Ratio1
    dev.write_reg(0x0004, 0x0109)
    time.sleep(0.001)
    print("  PLL_CFG1 <- 0x0980 (PLL_EN1=1)")
    print("  PLL_CFG2 <- 0x0109 (PLL_EN2=1)")

    # Verify enable bits took
    c1 = dev.read_reg(0x0002)
    c2 = dev.read_reg(0x0004)
    print(f"  Readback: PLL_CFG1=0x{c1:04X}  PLL_CFG2=0x{c2:04X}")
    if not (c1 & 0x0100):
        print("  WARNING: PLL_EN1 did not set!")
    if not (c2 & 0x0100):
        print("  WARNING: PLL_EN2 did not set!")

    # Clear any latched errors from before enable
    dev.write_reg(0x0114, 0x000A)
    dev.write_reg(0x0116, 0x00FF)

    # Step 7: Wait for lock
    print("\n[7/7] Waiting for lock...")
    print("  (300Hz * 160-700 UI = 0.5-2.3s for freq lock)")

    locked = False
    for i in range(60):  # 6 seconds
        time.sleep(0.1)
        unlock = dev.read_reg(0x0114)
        err = dev.read_reg(0x0116)
        f_lock = not (unlock & 0x01)
        p_lock = not (unlock & 0x04)

        # Print progress every second or on state change
        if (i % 10 == 0) or f_lock:
            print(f"  t={((i+1)*100):4d}ms: {decode_unlock_ind(unlock)}"
                  f"  err={decode_error_sts(err)}")

        if f_lock:
            print(f"\n  FREQUENCY LOCKED after {(i+1)*100}ms")
            locked = True
            break

    if not locked:
        print(f"\n  TIMEOUT: no frequency lock in 6s")
        print(f"  Final UNLOCK_IND = 0x{unlock:04X}")
        print(f"  Final ERROR_STS  = 0x{err:04X} ({decode_error_sts(err)})")
        print("\n  Possible causes:")
        print("    - No 300 Hz signal on CLK_IN pin (P8 connector)")
        print("    - REF_CLK (12 MHz crystal) not oscillating")
        print("    - Ratio or format mismatch")
        cmd_dump(dev)
        return False

    # Check phase lock
    if not p_lock:
        print("  Waiting for phase alignment...")
        for i in range(30):
            time.sleep(0.1)
            unlock = dev.read_reg(0x0114)
            if not (unlock & 0x04):
                print(f"  Phase LOCKED after additional {(i+1)*100}ms")
                break
        else:
            print("  Phase lock not achieved (PLL is freq-locked though)")

    # Final report
    print("\n" + "="*60)
    print("RESULT: PLL LOCKED")
    print("="*60)
    print("  CLK_OUT  = 24.576 MHz (should be)")
    print("  BCLK_OUT = 3.072 MHz")
    print("  FSYNC    = 48 kHz")
    print("  AUX1_OUT = CLK_IN (300 Hz buffered)")
    cmd_dump(dev)
    return True


def open_device(args):
    """Open FTDI device with auto-detection if no URL given."""
    if not args.url:
        from pyftdi.ftdi import Ftdi
        devices = Ftdi.list_devices()
        if not devices:
            print("ERROR: No FTDI devices found. Use --list to debug.")
            sys.exit(1)
        desc, ifcount = devices[0]
        try:
            args.url = f"ftdi://ftdi:232h:{desc.sn}/1"
        except AttributeError:
            args.url = f"ftdi://ftdi:232h/1"
        print(f"Auto: {args.url}")

    if args.i2c:
        return CS2600_I2C(args.url, args.i2c_addr)
    else:
        return CS2600_SPI(args.url, cs=args.cs, freq=args.freq)


def main():
    parser = argparse.ArgumentParser(
        description="CS2600 bringup via CDB-CLOCKING USB (pyftdi)")

    # Actions (mutually exclusive)
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--list", action="store_true",
                       help="List FTDI devices")
    group.add_argument("--probe", action="store_true",
                       help="Read device ID and status only")
    group.add_argument("--dump", action="store_true",
                       help="Dump all key registers")
    group.add_argument("--configure", action="store_true",
                       help="Full 300 Hz multiplier configuration")
    group.add_argument("--reset", action="store_true",
                       help="Software reset only")
    group.add_argument("--spi-scan", action="store_true",
                       help="Try all SPI modes to find working config")
    group.add_argument("--i2c-scan", action="store_true",
                       help="Scan I2C bus for devices")
    group.add_argument("--read", type=lambda x: int(x, 0), metavar="ADDR",
                       help="Read single register (hex addr)")
    group.add_argument("--write", nargs=2, metavar=("ADDR", "VAL"),
                       help="Write single register (hex addr, hex val)")

    # Transport options
    parser.add_argument("--i2c", action="store_true",
                        help="Use I2C (default: SPI)")
    parser.add_argument("--url", type=str, default=None,
                        help="FTDI URL")
    parser.add_argument("--cs", type=int, default=0,
                        help="SPI CS index (default: 0)")
    parser.add_argument("--i2c-addr", type=lambda x: int(x, 0), default=0x2C,
                        help="I2C 7-bit addr (default: 0x2C)")
    parser.add_argument("--freq", type=int, default=1_000_000,
                        help="Bus frequency Hz (default: 1000000)")

    args = parser.parse_args()

    if args.list:
        list_devices()
        return 0

    if args.spi_scan:
        cmd_spi_scan(args)
        return 0

    if args.i2c_scan:
        cmd_i2c_scan(args)
        return 0

    # All other commands need the device open
    dev = open_device(args)
    try:
        if args.probe:
            return 0 if cmd_probe(dev) else 1

        elif args.dump:
            if not cmd_probe(dev):
                return 1
            cmd_dump(dev)
            return 0

        elif args.reset:
            print("\n--- Software Reset ---")
            dev.write_reg(0x0058, 0x005A)
            time.sleep(0.010)
            return 0 if cmd_probe(dev) else 1

        elif args.read is not None:
            val = dev.read_reg(args.read)
            print(f"0x{args.read:04X} ({reg_name(args.read)}) = 0x{val:04X}")
            return 0

        elif args.write is not None:
            addr = int(args.write[0], 0)
            val = int(args.write[1], 0)
            write_verify(dev, addr, val)
            return 0

        elif args.configure:
            return 0 if cmd_configure(dev) else 1

        else:
            # Default: probe
            return 0 if cmd_probe(dev) else 1

    finally:
        dev.close()


if __name__ == "__main__":
    sys.exit(main())
