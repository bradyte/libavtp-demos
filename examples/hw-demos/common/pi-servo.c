/*
 * PI Clock Servo for CRF media clock recovery.
 * Based on linuxptp pi.c (Copyright (C) 2011 Richard Cochran, GPL v2+)
 */

#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#include "pi-servo.h"

/* Defaults characterised against the CM4 PWM/CS2600 clock path at 300 Hz. */
#define CRF_KP			0.4
#define CRF_KI			5.0e-5
#define CRF_KI_FAST		3.0e-4
#define FAST_KI_SAMPLES		3000
#define SLEW_KP			2.0
#define SLEW_THRESH_NS		500
#define SERVO_MAX_PPB		2000000.0
#define SERVO_STEP_THRESH_NS	100000000.0

/* Upstream computes 0.016 / ki and clamps at 1000 s. With the ki above that
 * is 320 s, far too long for a media clock, so the window is set directly. */
#define FREQ_EST_INTERVAL_S	2.0

/* The acquisition window is extended by this fraction before comparing, so a
 * sample landing fractionally short does not cost another full window. */
#define FREQ_EST_MARGIN		0.001

struct pi_servo {
	struct pi_servo_config cfg;

	int64_t offset[2];
	uint64_t local[2];
	double drift;
	double last_freq;
	int count;
	unsigned int tracking_samples;
};

void pi_servo_config_init(struct pi_servo_config *cfg)
{
	if (!cfg)
		return;

	cfg->kp			= CRF_KP;
	cfg->ki			= CRF_KI;
	cfg->ki_fast		= CRF_KI_FAST;
	cfg->fast_ki_samples	= FAST_KI_SAMPLES;
	cfg->slew_kp		= SLEW_KP;
	cfg->slew_thresh_ns	= SLEW_THRESH_NS;
	cfg->max_ppb		= SERVO_MAX_PPB;
	cfg->step_threshold_ns	= SERVO_STEP_THRESH_NS;
	cfg->freq_est_interval_s = FREQ_EST_INTERVAL_S;
}

struct pi_servo *pi_servo_create(const struct pi_servo_config *cfg)
{
	struct pi_servo *s;

	if (!cfg || cfg->max_ppb <= 0.0 || cfg->freq_est_interval_s <= 0.0)
		return NULL;

	s = calloc(1, sizeof(*s));
	if (!s)
		return NULL;

	s->cfg = *cfg;

	return s;
}

void pi_servo_destroy(struct pi_servo *servo)
{
	free(servo);
}

double pi_servo_sample(struct pi_servo *servo,
		       int64_t offset,
		       uint64_t local_ts,
		       enum servo_state *state)
{
	double ki_term, ppb;
	double localdiff;

	if (!servo || !state)
		return 0.0;

	ppb = servo->last_freq;

	switch (servo->count) {
	case 0:
		/* First sample of the acquisition window. */
		servo->offset[0] = offset;
		servo->local[0] = local_ts;
		*state = SERVO_UNLOCKED;
		servo->count = 1;
		break;

	case 1:
		/* Acquiring. Apply nothing, just watch the phase error drift
		 * until the window has elapsed; its slope is the frequency
		 * error. */
		servo->offset[1] = offset;
		servo->local[1] = local_ts;

		if (servo->local[0] >= servo->local[1]) {
			*state = SERVO_UNLOCKED;
			servo->count = 0;
			break;
		}

		localdiff = (servo->local[1] - servo->local[0]) / 1e9;
		localdiff += localdiff * FREQ_EST_MARGIN;

		if (localdiff < servo->cfg.freq_est_interval_s) {
			*state = SERVO_UNLOCKED;
			break;
		}

		servo->drift += (1e9 - servo->drift) *
				(servo->offset[1] - servo->offset[0]) /
				(servo->local[1] - servo->local[0]);

		if (servo->drift < -servo->cfg.max_ppb)
			servo->drift = -servo->cfg.max_ppb;
		else if (servo->drift > servo->cfg.max_ppb)
			servo->drift = servo->cfg.max_ppb;

		*state = SERVO_LOCKED;
		ppb = servo->drift;
		servo->count = 2;
		break;

	case 2:
		/* Tracking. */
		if (servo->cfg.step_threshold_ns &&
		    servo->cfg.step_threshold_ns < llabs(offset)) {
			/* Too large to be drift. Reacquire from scratch. */
			*state = SERVO_UNLOCKED;
			servo->count = 0;
			break;
		}

		if (llabs(offset) > servo->cfg.slew_thresh_ns) {
			/* Large-error regime: proportional only. Note this
			 * leaves drift untouched, so it cannot correct a wrong
			 * frequency estimate. See the note in pi-servo.h. */
			ppb = servo->cfg.slew_kp * offset + servo->drift;
			servo->tracking_samples = 0;
		} else {
			double ki_active =
				servo->tracking_samples < servo->cfg.fast_ki_samples
				? servo->cfg.ki_fast : servo->cfg.ki;

			ki_term = ki_active * offset;
			ppb = servo->cfg.kp * offset + servo->drift + ki_term;
			servo->tracking_samples++;
		}

		if (ppb < -servo->cfg.max_ppb) {
			ppb = -servo->cfg.max_ppb;
		} else if (ppb > servo->cfg.max_ppb) {
			ppb = servo->cfg.max_ppb;
		} else if (llabs(offset) <= servo->cfg.slew_thresh_ns) {
			/* Only integrate when the correction was not clamped,
			 * so a saturated loop does not wind up. */
			servo->drift += ki_term;
		}

		*state = SERVO_LOCKED;
		break;
	}

	servo->last_freq = ppb;
	return ppb;
}

void pi_servo_reset(struct pi_servo *servo)
{
	if (servo)
		servo->count = 0;
}

void pi_servo_skip_freq_est(struct pi_servo *servo)
{
	if (!servo)
		return;

	servo->drift = 0.0;
	servo->count = 2;
}

double pi_servo_get_drift(const struct pi_servo *servo)
{
	return servo ? servo->drift : 0.0;
}
