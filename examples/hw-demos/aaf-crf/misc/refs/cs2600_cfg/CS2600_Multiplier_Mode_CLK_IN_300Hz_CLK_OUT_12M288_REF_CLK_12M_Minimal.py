#!/usr/bin/python
#########################################################################
## @file   CS2600_Multiplier_Mode_CLK_IN_300Hz_CLK_OUT_12M288_REF_CLK_12M_Minimal.py
## @brief  Minimal multiplier config: 300 Hz → 12.288 MHz. No phase alignment,
##         no holdover, no output config. Just PLL lock test.
#########################################################################
"""**************************************************************************************
* CLK_IN must be 300Hz
* REF_CLK must be 12MHz
* CLK_OUT = 12.288MHz (ratio 40960 in 20.12 format)
* No phase alignment, no holdover, no BCLK/FSYNC config
**************************************************************************************"""

from studiolink.StudioLink import StudioLink
device_CS2600 = StudioLink.get_device_by_type("CS2600")

###Software Reset
device_CS2600.writeRegisterByName("SW_RESET", 0x005A) # SW_RST = Software reset

###Select Multiplier Mode (simple — no holdover)
device_CS2600.writeRegisterByName("PLL_CFG2", 0x0009) # PLL_MODE_SEL = Multiplier Mode

###Ratio 1: 40960 in 20.12 format = 0x0A000000 → RATIO1_1 = 0x0A00
device_CS2600.writeRegisterByName("RATIO1_1", 0x0A00) # RATIO1 = 40960

###REF_CLK_IN divider, REF_CLK Source selection
device_CS2600.writeRegisterByName("PLL_CFG3", 0x0012) # REF_CLK_IN_DIV = Divide by 1, SYSCLK_SRC = REF_CLK_IN

###FLL Bandwidth: 8 Hz (faster acquisition, still within 300/23.4 = 12.8 Hz limit)
device_CS2600.writeRegisterByName("PLL_CFG4", 0x0070) # FLL_BW = 8 Hz, FLL_BW_MOD = x1

###PLL Enable
device_CS2600.writeRegisterByName("PLL_CFG1", 0x0100) # PLL_EN1 = Enabled
device_CS2600.writeRegisterByName("PLL_CFG2", 0x0109) # PLL_EN2 = Enabled, PLL_MODE_SEL = Multiplier Mode
