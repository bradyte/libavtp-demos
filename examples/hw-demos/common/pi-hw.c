/*
 * PI Clock Servo for CRF media clock recovery.
 * Based on linuxptp pi.c (Copyright (C) 2011 Richard Cochran, GPL v2+)
 */

#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#include "pi-hw.h"

#define CRF_KP 0.4
#define CRF_KI 5.0e-5
#define CRF_KI_FAST 3.0e-4

#define ACQ_KP 2.0
#define ACQ_THRESH_NS 500
#define FAST_KI_SAMPLES 3000

#define FREQ_EST_MARGIN 0.001

struct pi_servo {
	int64_t offset[2];
	uint64_t local[2];
	double drift;
	double kp;
	double ki;
	double acq_kp;
	int64_t acq_thresh_ns;
	double last_freq;
	double max_frequency;
	double step_threshold;
	int count;
	unsigned int tracking_samples;
};

struct pi_servo *pi_servo_create(double max_ppb, double step_threshold_ns)
{
	struct pi_servo *s;

	if (max_ppb <= 0.0)
		return NULL;

	s = calloc(1, sizeof(*s));
	if (!s)
		return NULL;

	s->max_frequency = max_ppb;
	s->step_threshold = step_threshold_ns;
	s->kp = CRF_KP;
	s->ki = CRF_KI;
	s->acq_kp = ACQ_KP;
	s->acq_thresh_ns = ACQ_THRESH_NS;

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
	double freq_est_interval, localdiff;

	if (!servo || !state)
		return 0.0;

	ppb = servo->last_freq;

	switch (servo->count) {
	case 0:
		servo->offset[0] = offset;
		servo->local[0] = local_ts;
		*state = SERVO_UNLOCKED;
		servo->count = 1;
		break;

	case 1:
		servo->offset[1] = offset;
		servo->local[1] = local_ts;

		if (servo->local[0] >= servo->local[1]) {
			*state = SERVO_UNLOCKED;
			servo->count = 0;
			break;
		}

		localdiff = (servo->local[1] - servo->local[0]) / 1e9;
		localdiff += localdiff * FREQ_EST_MARGIN;

		freq_est_interval = 0.016 / servo->ki;
		if (freq_est_interval > 2.0)
			freq_est_interval = 2.0;
		if (localdiff < freq_est_interval) {
			*state = SERVO_UNLOCKED;
			break;
		}

		servo->drift += (1e9 - servo->drift) * (servo->offset[1] - servo->offset[0]) /
				(servo->local[1] - servo->local[0]);

		if (servo->drift < -servo->max_frequency)
			servo->drift = -servo->max_frequency;
		else if (servo->drift > servo->max_frequency)
			servo->drift = servo->max_frequency;

		*state = SERVO_LOCKED;
		ppb = servo->drift;
		servo->count = 2;
		break;

	case 2:
		if (servo->step_threshold &&
		    servo->step_threshold < llabs(offset)) {
			*state = SERVO_UNLOCKED;
			servo->count = 0;
			break;
		}

		if (llabs(offset) > servo->acq_thresh_ns) {
			ppb = servo->acq_kp * offset + servo->drift;
			servo->tracking_samples = 0;
		} else {
			double ki_active = servo->tracking_samples < FAST_KI_SAMPLES
					 ? CRF_KI_FAST : servo->ki;
			ki_term = ki_active * offset;
			ppb = servo->kp * offset + servo->drift + ki_term;
			servo->tracking_samples++;
		}

		if (ppb < -servo->max_frequency) {
			ppb = -servo->max_frequency;
		} else if (ppb > servo->max_frequency) {
			ppb = servo->max_frequency;
		} else if (llabs(offset) <= servo->acq_thresh_ns) {
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

void pi_servo_set_gains(struct pi_servo *servo, double kp, double ki)
{
	if (!servo)
		return;
	servo->kp = kp;
	servo->ki = ki;
}

void pi_servo_set_acq(struct pi_servo *servo, double acq_kp, int64_t acq_thresh_ns)
{
	if (!servo)
		return;
	servo->acq_kp = acq_kp;
	servo->acq_thresh_ns = acq_thresh_ns;
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
