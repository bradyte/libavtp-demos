# Reference Material

External dependencies and documentation for the CRF hardware demo.

## Hardware

- **AutomotiveTimeHAT** — GPS PPS input for PTP grandmaster
  https://github.com/DerFetworty/AutomotiveTimeHAT

- **Intel i226 IGC driver (ppsfix)** — DKMS module with rising-edge PPS filtering
  for Ubuntu 6.8 RT kernel. See project wiki or contact maintainer.

## Audio

- **CS2600 Jitter Cleaner** — Cirrus Logic CS2600 PLL, downstream of PWM output.
  OTP programming guide: contact Cirrus Logic for CS2600_OTP_Programming_Guide.pdf

- **ALSA** — Standard Linux audio stack (alsa-lib, alsa-utils, alsa-plugins, alsa-tools)
  https://www.alsa-project.org/

## Software

- **linuxptp** — PTP stack (ts2phc, ptp4l, phc2sys)
  https://github.com/richardcochran/linuxptp
