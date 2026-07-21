#!/usr/bin/python
#########################################################################
## @file   CS2600_Multiplier_Mode_CLK_IN_2kHz_CLK_OUT_24M576_REF_CLK_12M_High_Multiplication.py
## @brief  Multiplier Mode: 2 kHz → 24.576 MHz. Minimal config to validate
##         software mode matches hardware-strapped behavior.
#########################################################################
"""**************************************************************************************
* CLK_IN must be 2kHz
* REF_CLK must be 12MHz
* CLK_OUT = 24.576MHz (ratio 12288 in 20.12 format)
**************************************************************************************"""

from studiolink.StudioLink import StudioLink
device_CS2600 = StudioLink.get_device_by_type("CS2600")

###Software Reset
#device_CS2600.writeRegisterByName("SW_RESET", 0x005A) # SW_RST = Software reset

###Select Multiplier Mode
device_CS2600.writeRegisterByName("PLL_CFG2", 0x0009) # PLL_MODE_SEL = Multiplier Mode

###Ratio 1: 12288 in 20.12 format = 0x03000000 → RATIO1_1 = 0x0300
device_CS2600.writeRegisterByName("RATIO1_1", 0x0300) # RATIO1 = 12288

###REF_CLK_IN divider, REF_CLK Source selection
device_CS2600.writeRegisterByName("PLL_CFG3", 0x0012) # REF_CLK_IN_DIV = Divide by 1, SYSCLK_SRC = REF_CLK_IN

###PLL Enable
device_CS2600.writeRegisterByName("PLL_CFG1", 0x0100) # PLL_EN1 = Enabled
device_CS2600.writeRegisterByName("PLL_CFG2", 0x0109) # PLL_EN2 = Enabled, PLL_MODE_SEL = Multiplier Mode
