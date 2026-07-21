#!/usr/bin/env python3
"""
CS2600 I2C configuration: 300 Hz CLK_IN -> 24.576 MHz CLK_OUT (Multiplier Mode)
Requires: smbus2, I2C enabled. Auto-detects CS2600 at 0x2F/0x2E/0x2D/0x2C.

Usage: sudo /path/to/venv/bin/python3 cs2600_i2c_config.py
"""

from smbus2 import SMBus, i2c_msg
import time
import sys

BUS = 1
CS2600_ADDRS = [0x2F, 0x2E, 0x2D, 0x2C]
ADDR = None

def find_cs2600(bus):
    """Auto-detect CS2600 I2C address by reading DEVICE_ID1 at each candidate."""
    for addr in CS2600_ADDRS:
        try:
            write = i2c_msg.write(addr, [0x01, 0x10])
            read = i2c_msg.read(addr, 2)
            bus.i2c_rdwr(write, read)
            data = list(read)
            dev_id = (data[0] << 8) | data[1]
            if dev_id == 0x2600:
                return addr
        except OSError:
            continue
    return None

def read_reg(bus, reg_addr):
    write = i2c_msg.write(ADDR, [(reg_addr >> 8) & 0xFF, reg_addr & 0xFF])
    read = i2c_msg.read(ADDR, 2)
    bus.i2c_rdwr(write, read)
    data = list(read)
    return (data[0] << 8) | data[1]

def write_reg(bus, reg_addr, value):
    bus.i2c_rdwr(i2c_msg.write(ADDR, [
        (reg_addr >> 8) & 0xFF, reg_addr & 0xFF,
        (value >> 8) & 0xFF, value & 0xFF
    ]))
    time.sleep(0.001)

def write_verify(bus, reg_addr, value, name):
    write_reg(bus, reg_addr, value)
    rb = read_reg(bus, reg_addr)
    status = 'OK' if rb == value else 'MISMATCH'
    print(f'  {name:20s} (0x{reg_addr:04X}) = 0x{value:04X} -> read 0x{rb:04X} [{status}]')
    if rb != value:
        return False
    return True

def main():
    global ADDR
    bus = SMBus(BUS)

    print('=== CS2600 Multiplier Mode: 300 Hz -> 24.576 MHz ===')
    print()

    # Auto-detect CS2600 address
    ADDR = find_cs2600(bus)
    if ADDR is None:
        print(f'ERROR: CS2600 not found at any address {[hex(a) for a in CS2600_ADDRS]}')
        sys.exit(1)
    print(f'  Device found: CS2600 at 0x{ADDR:02X}')
    print()

    # Software reset
    print('1. Software reset...')
    write_reg(bus, 0x0058, 0x005A)
    time.sleep(0.1)
    dev_id = read_reg(bus, 0x0110)
    print(f'   DEVICE_ID1 after reset: 0x{dev_id:04X}')
    print()

    # Configure PLL (PLL disabled during config)
    print('2. Configure PLL...')
    write_verify(bus, 0x0004, 0x0009, 'PLL_CFG2')        # PLL_MODE_SEL = Multiplier
    write_verify(bus, 0x0006, 0x1400, 'RATIO1_1')        # Ratio 81920 (20.12 fmt) upper
    write_verify(bus, 0x0008, 0x0000, 'RATIO1_2')        # Ratio lower
    write_verify(bus, 0x0016, 0x0012, 'PLL_CFG3')        # REF_CLK_IN_DIV=/1, SYSCLK_SRC=REF_CLK_IN
    write_verify(bus, 0x001E, 0x0000, 'PLL_CFG4')        # FLL_BW=1Hz
    print()

    # Output config
    print('3. Output config...')
    write_verify(bus, 0x0100, 0x5582, 'OUTPUT_CFG1')     # BCLK=/8, FSYNC=/512, inverted
    write_verify(bus, 0x0102, 0x0400, 'OUTPUT_CFG2')     # AUX1=CLK_IN, CLK_OUT=MCLK
    print()

    # Phase alignment
    print('4. Phase alignment...')
    write_verify(bus, 0x0108, 0x8008, 'PHASE_ALIGN_CFG1') # EN=1, auto, threshold=8
    print()

    # Enable PLL
    print('5. Enable PLL...')
    write_verify(bus, 0x0002, 0x0980, 'PLL_CFG1')        # PLL_EN1=1, S_RATIO_SEL=Ratio2
    write_verify(bus, 0x0004, 0x0109, 'PLL_CFG2')        # PLL_EN2=1, Multiplier mode
    print()

    # Wait and check status
    print('6. Checking lock status (waiting 2s)...')
    time.sleep(2)
    unlock = read_reg(bus, 0x0114)
    error = read_reg(bus, 0x0116)
    print(f'   UNLOCK_IND: 0x{unlock:04X}')
    print(f'   ERROR_STS:  0x{error:04X}')
    print()

    if (unlock & 0x0005) == 0:
        print('PLL LOCKED - 24.576 MHz output active')
        print('  CLK_OUT  = 24.576 MHz')
        print('  BCLK_OUT = 3.072 MHz')
        print('  FSYNC    = 48 kHz')
        print('  AUX1_OUT = CLK_IN (300 Hz)')
        # Clear sticky bits
        write_reg(bus, 0x0114, 0x000A)
        time.sleep(0.01)
        print(f'  Sticky bits cleared, UNLOCK_IND: 0x{read_reg(bus, 0x0114):04X}')
    else:
        print('PLL NOT LOCKED - check CLK_IN (300 Hz on P8)')
        if unlock & 0x0001:
            print('  -> F_UNLOCK: frequency unlocked')
        if unlock & 0x0004:
            print('  -> P_UNLOCK: phase unlocked')

    bus.close()

if __name__ == '__main__':
    main()
