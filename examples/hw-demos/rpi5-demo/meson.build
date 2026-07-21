# RPi5 Demo — CLOCK_REALTIME-based AAF/CRF listeners
# Stock Raspberry Pi 5, PTP sync via phc2sys

common_src = files('../../common.c')

# Shared modules from hw-demos/common/
crf_common_src = files('../common/crf-common.c')
crf_receiver_src = files('../common/crf-receiver.c')
phc_utils_src = files('../common/phc-utils.c')
pi_servo_src = files('../common/pi-hw.c')

inc = [
	include_directories('../common'),
	include_directories('../../../include'),
	include_directories('../../..'),
]

# CRF Listener — GPIO edge capture servo with RP1 clock steering
executable(
	'crf-listener-rpi',
	'crf-listener-rpi.c',
	'rp1-clock.c',
	common_src,
	crf_common_src,
	crf_receiver_src,
	phc_utils_src,
	pi_servo_src,
	include_directories: inc,
	link_with: avtp_lib,
	dependencies: [mdep],
	build_by_default: false,
)

# AAF Listener — presentation-time gating via CLOCK_REALTIME
alsa_dep = dependency('alsa', required: false)
if alsa_dep.found()
	executable(
		'aaf-listener-rpi',
		'aaf-listener-rpi.c',
		common_src,
		include_directories: inc,
		link_with: avtp_lib,
		dependencies: [alsa_dep],
		build_by_default: false,
	)
endif
