# RPi5 Demo — CLOCK_REALTIME-based AAF/CRF listeners
# Stock Raspberry Pi 5, PTP sync via phc2sys

common_src = files('../../common.c')

# Shared modules from hw-demos/common/
devmem_src = files('../common/devmem.c')
crf_profile_src = files('../common/crf-profile.c')
crf_receiver_src = files('../common/crf-receiver.c')
pi_servo_src = files('../common/pi-servo.c')

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
	devmem_src,
	common_src,
	crf_profile_src,
	crf_receiver_src,
	pi_servo_src,
	include_directories: inc,
	link_with: avtp_lib,
	dependencies: [mdep],
	build_by_default: false,
)

# The AAF listener is platform-neutral and lives in aaf-crf/. AF_XDP is a
# runtime option there (-q); omit it and it uses AF_PACKET, which is what a
# stock Raspberry Pi 5 needs — its kernel has no CONFIG_XDP_SOCKETS. One
# binary serves both boards, so there is no separate target here.
