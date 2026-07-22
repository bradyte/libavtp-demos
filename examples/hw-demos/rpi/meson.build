# Raspberry Pi hardware demos: CRF media clock recovery and AAF audio over TSN.
#
# One set of four programs serves every supported board. What differs between
# them - where feedback edges are captured and how the clock is steered - is
# behind common/media-clock.h, so the demos themselves carry no platform
# knowledge. Select a backend with -B, or let it probe the device tree.
#
# Backends:
#   cm4    Raspberry Pi CM4, Intel i226 capture, BCM2711 PWM into a CS2600
#   rpi5   stock Raspberry Pi 5, GPIO chardev capture, RP1 pll_audio  (WIP,
#          common/mc-rp1.c is written but not yet built or validated)

common_src = files('../../common.c')

# Protocol and control, platform-neutral
crf_profile_src  = files('common/crf-profile.c')
crf_receiver_src = files('common/crf-receiver.c')
aaf_profile_src  = files('common/aaf-profile.c')
pi_servo_src     = files('common/pi-servo.c')

# Host services
phc_utils_src = files('common/phc-utils.c')
ptp_extts_src = files('common/ptp-extts.c')
devmem_src    = files('common/devmem.c')
alsa_util_src = files('common/alsa-util.c')
xsk_src       = files('common/xsk-util.c')

# Media clock backends, plus the SoC drivers they sit on
media_clock_src = files(
	'common/media-clock.c',
	'common/mc-bcm2711.c',
	'common/bcm2711-pwm-clock.c',
	'common/cs2600.c',
)

inc = [
	include_directories('common'),
	include_directories('../../../include'),
	include_directories('../../..'),
]

# CRF talker: hardware-timestamped edge capture, packed into CRF at 50 Hz
executable(
	'crf-talker-rpi',
	'crf-talker-rpi.c',
	common_src,
	crf_profile_src,
	phc_utils_src,
	ptp_extts_src,
	include_directories: inc,
	link_with: avtp_lib,
	dependencies: [mdep],
	build_by_default: false,
)

# CRF listener: media clock recovery through the servo and a backend
executable(
	'crf-listener-rpi',
	'crf-listener-rpi.c',
	common_src,
	crf_profile_src,
	crf_receiver_src,
	pi_servo_src,
	media_clock_src,
	devmem_src,
	phc_utils_src,
	ptp_extts_src,
	include_directories: inc,
	link_with: avtp_lib,
	dependencies: [mdep],
	build_by_default: false,
)

# AAF audio over TSN, needs ALSA
alsa_dep = dependency('alsa', required: false)
if alsa_dep.found()
	executable(
		'aaf-talker-rpi',
		'aaf-talker-rpi.c',
		common_src,
		aaf_profile_src,
		alsa_util_src,
		include_directories: inc,
		link_with: avtp_lib,
		dependencies: [alsa_dep],
		build_by_default: false,
	)

	# AF_XDP is a runtime option (-q). Omit it on a stock Raspberry Pi 5,
	# whose kernel is built without CONFIG_XDP_SOCKETS.
	executable(
		'aaf-listener-rpi',
		'aaf-listener-rpi.c',
		common_src,
		aaf_profile_src,
		alsa_util_src,
		xsk_src,
		include_directories: inc,
		link_with: avtp_lib,
		dependencies: [alsa_dep],
		build_by_default: false,
	)
endif
