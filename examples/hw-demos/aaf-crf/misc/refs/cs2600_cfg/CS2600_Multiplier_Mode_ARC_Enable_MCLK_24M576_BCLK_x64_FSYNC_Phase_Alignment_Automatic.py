#!/usr/bin/python
#########################################################################
#  Copyright (c) 2024 Cirrus Logic, Inc and
#  Cirrus Logic International Semiconductor Ltd.  All rights reserved.
#
#  This software as well as any related documentation is furnished under
#  license and may only be used or copied in accordance with the terms of the
#  license.  The information in this file is furnished for informational use
#  only, is subject to change without notice, and should not be construed as
#  a commitment by Cirrus Logic.  Cirrus Logic assumes no responsibility or
#  liability for any errors or inaccuracies that may appear in this document
#  or any software that may be provided in association with this document.
#
#  Except as permitted by such license, no part of this document may be
#  reproduced, stored in a retrieval system, or transmitted in any form or by
#  any means without the express written consent of Cirrus Logic.
#
#  Warning
#    This software is specifically written for Cirrus Logic devices.
#    It may not be used with other devices.
#
#########################################################################
## @file   CS2600_Multiplier_Mode_ARC_Enable_MCLK_24M576_BCLK_x64_FSYNC_Phase_Alignment_Automatic.py
## @brief  SoundClear Studio script to enable ARC and Phase Aligment in Automatic Mode
#########################################################################
"""**************************************************************************************
* CLK_IN must be 32 kHz, 48 kHz,96 kHz, or 192 kHz / 44.1 kHz, 88.2 kHz,or 176.4 kHz
* REF_CLK_IN must 12MHz
* CLK_OUT = 24.576MHz / 22.5792MHz
* BCLK_OUT = FSYNC x64
* FSYNC = CLK_IN
**************************************************************************************"""

from studiolink.StudioLink import StudioLink
device_CS2600 = StudioLink.get_device_by_type("CS2600")

###Software Reset
device_CS2600.writeRegisterByName("SW_RESET", 0x005A) # SW_RST = Software reset

###Select Multiplier Mode
device_CS2600.writeRegisterByName("PLL_CFG2", 0x0009) # PLL_MODE_SEL = Multiplier Mode

###ARC Enable
device_CS2600.writeRegisterByName("AUTOMATIC_RATE_CONTROL_CFG1", 0x0041) # ARC_EN = Enabled, ARC_MCLK = 24.576/22.5792 MHz

###REF_CLK_IN divider and REF_CLK Source selection
device_CS2600.writeRegisterByName("PLL_CFG3", 0x0012) # REF_CLK_IN_DIV = Divide by 1, SYSCLK_SRC = REF_CLK_IN

###Phase Alignment Enable
device_CS2600.writeRegisterByName("PHASE_ALIGNMENT_CFG1", 0x8080) # PHASE_ALIGNMENT_EN = Enabled, PHASE_ALIGNMENT_STB_EN = Enabled    

###PLL Enable
device_CS2600.writeRegisterByName("PLL_CFG1", 0x0180) # PLL_EN1 = Enabled
device_CS2600.writeRegisterByName("PLL_CFG2", 0x0109) # PLL_EN2 = Enabled, PLL_MODE_SEL = Multiplier Mode

