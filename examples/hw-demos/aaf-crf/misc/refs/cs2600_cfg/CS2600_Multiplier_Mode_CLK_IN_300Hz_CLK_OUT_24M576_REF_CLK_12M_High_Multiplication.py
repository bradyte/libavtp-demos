#!/usr/bin/python
#########################################################################
## @file   CS2600_Multiplier_Mode_CLK_IN_300Hz_CLK_OUT_24M576_REF_CLK_12M_High_Multiplication.py
## @brief  SoundClear Studio script to enable Multiplier Mode with High Multiplication format 20.12.
##         300 Hz CLK_IN, 12 MHz REF_CLK, 24.576 MHz CLK_OUT.
##         Holdover enabled, AUX1_OUT = CLK_IN, phase alignment automatic.
#########################################################################
"""**************************************************************************************
* CLK_IN must be 300Hz
* REF_CLK must be 12MHz
* CLK_OUT = 24.576MHz (ratio 81920 in 20.12 format)
* AUX1_OUT = CLK_IN (buffered for i226 SDP capture)
* BCLK_OUT = CLK_OUT / 8 = 3.072MHz
* FSYNC_OUT = CLK_OUT / 512 = 48kHz
* Phase alignment: automatic, threshold 8 MCLK, speed 1 MCLK/FSYNC
* Holdover: enabled
**************************************************************************************"""

from studiolink.StudioLink import StudioLink
device_CS2600 = StudioLink.get_device_by_type("CS2600")

###Software Reset
device_CS2600.writeRegisterByName("SW_RESET", 0x005A) # SW_RST = Software reset

###Select Multiplier Mode, enable holdover (S_RATIO_SEL != M_RATIO_SEL)
device_CS2600.writeRegisterByName("PLL_CFG1", 0x0880) # S_RATIO_SEL = Ratio 2 (holdover)
device_CS2600.writeRegisterByName("PLL_CFG2", 0x0009) # PLL_MODE_SEL = Multiplier Mode, M_RATIO_SEL = Ratio 1

###Ratio 1: 81920 in 20.12 format = 0x14000000 → RATIO1_1 = 0x1400, RATIO1_2 = 0x0000
device_CS2600.writeRegisterByName("RATIO1_1", 0x1400) # RATIO1 upper = 81920
device_CS2600.writeRegisterByName("RATIO1_2", 0x0000) # RATIO1 lower = 0

###REF_CLK_IN divider, REF_CLK Source selection
device_CS2600.writeRegisterByName("PLL_CFG3", 0x0012) # REF_CLK_IN_DIV = Divide by 1, SYSCLK_SRC = REF_CLK_IN

###FLL Bandwidth: 1 Hz (narrow for max jitter rejection)
device_CS2600.writeRegisterByName("PLL_CFG4", 0x0000) # FLL_BW = 1 Hz, FLL_BW_MOD = x1

###Output Configuration
device_CS2600.writeRegisterByName("OUTPUT_CFG1", 0x5582) # BCLK_DIV=/8, FSYNC_DIV=/512, BCLK_INV=1, FSYNC_INV=1, FSYNC_DUTY=50%
device_CS2600.writeRegisterByName("OUTPUT_CFG2", 0x0400) # AUX1_OUT_SEL = CLK_IN, CLK_OUT_SEL = MCLK

###Phase Alignment: automatic, threshold 8 MCLK, speed 1 MCLK/FSYNC
device_CS2600.writeRegisterByName("PHASE_ALIGNMENT_CFG1", 0x8008) # EN=1, MODE=auto, THR=8 MCLK (010), SPEED=00

###PLL Enable
device_CS2600.writeRegisterByName("PLL_CFG1", 0x0980) # PLL_EN1 = Enabled, S_RATIO_SEL = Ratio 2
device_CS2600.writeRegisterByName("PLL_CFG2", 0x0109) # PLL_EN2 = Enabled, PLL_MODE_SEL = Multiplier Mode
