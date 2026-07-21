# AAF/CRF Hardware Demo
# Raspberry Pi CM4 + BCM2711 + i226 NIC

common_src = files('../../common.c')

# Shared utility modules (from hw-demos/common/)
phc_utils_src = files('../common/phc-utils.c')
bcm2711_pwm_clock_src = files('../common/bcm2711-pwm-clock.c')
cs2600_src = files('../common/cs2600.c')
pi_servo_src = files('../common/pi-hw.c')

# CRF protocol modules (from hw-demos/common/)
crf_common_src = files('../common/crf-common.c')
crf_receiver_src = files('../common/crf-receiver.c')

inc = [include_directories('../../../include'), include_directories('../../..'), include_directories('../common')]

# CRF talker with i226 SDP hardware timestamps
executable(
	'crf-talker-hw',
	'crf-talker-hw.c',
	common_src,
	phc_utils_src,
	include_directories: inc,
	link_with: avtp_lib,
	dependencies: [mdep],
	build_by_default: false,
)

# CRF listener with hardware clock recovery (servo + PWM feedback)
executable(
	'crf-listener-hw',
	'crf-listener-hw.c',
	common_src,
	crf_common_src,
	crf_receiver_src,
	bcm2711_pwm_clock_src,
	cs2600_src,
	phc_utils_src,
	pi_servo_src,
	include_directories: inc,
	link_with: avtp_lib,
	dependencies: [mdep],
	build_by_default: false,
)

# AF_XDP support (kernel headers only, no external deps)
xsk_src = files('../common/xsk-util.c')

# AAF Audio streaming over TSN (requires ALSA)
alsa_dep = dependency('alsa', required: false)
if alsa_dep.found()
	executable(
		'aaf-talker-hw',
		'aaf-talker-hw.c',
		common_src,
		include_directories: inc,
		link_with: avtp_lib,
		dependencies: [alsa_dep],
		build_by_default: false,
	)

	executable(
		'aaf-listener-hw',
		'aaf-listener-hw.c',
		common_src,
		xsk_src,
		include_directories: inc,
		link_with: avtp_lib,
		dependencies: [alsa_dep],
		build_by_default: false,
	)
endif
