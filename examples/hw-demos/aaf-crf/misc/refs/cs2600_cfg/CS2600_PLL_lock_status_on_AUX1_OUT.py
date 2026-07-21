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
## @file   CS2600_PLL_lock_status_on_AUX1_OUT.py
## @brief  SoundClear Studio script to enable PLL lock status on AUX1_OUT
#########################################################################
"""**************************************************************************************
* 
**************************************************************************************"""

from studiolink.StudioLink import StudioLink
device_CS2600 = StudioLink.get_device_by_type("CS2600")

###Software Reset
device_CS2600.writeRegisterByName("SW_RESET", 0x005A) # SW_RST = Software reset

###F_UNLOCK status on AUX1_OUT
device_CS2600.writeRegisterByName("OUTPUT_CFG2", 0x0C00) # AUX1_OUT_SEL = Frequency unlock (F_UNLOCK)

###PLL Enable
device_CS2600.writeRegisterByName("PLL_CFG1", 0x0180) # PLL_EN1 = Enabled
device_CS2600.writeRegisterByName("PLL_CFG2", 0x0108) # PLL_EN2 = Enabled