/*
 * PI Clock Servo for CRF media clock recovery.
 * Based on linuxptp pi.c (Copyright (C) 2011 Richard Cochran, GPL v2+)
 */

#pragma once

#include <stdint.h>

enum servo_state {
	SERVO_UNLOCKED,
	SERVO_JUMP,
	SERVO_LOCKED,
};

struct pi_servo;

struct pi_servo *pi_servo_create(double max_ppb, double step_threshold_ns);

void pi_servo_destroy(struct pi_servo *servo);

double pi_servo_sample(struct pi_servo *servo,
			int64_t offset_ns,
			uint64_t local_ts_ns,
			enum servo_state *state);

void pi_servo_reset(struct pi_servo *servo);

void pi_servo_set_gains(struct pi_servo *servo, double kp, double ki);

void pi_servo_set_acq(struct pi_servo *servo, double acq_kp, int64_t acq_thresh_ns);

void pi_servo_skip_freq_est(struct pi_servo *servo);

double pi_servo_get_drift(const struct pi_servo *servo);
