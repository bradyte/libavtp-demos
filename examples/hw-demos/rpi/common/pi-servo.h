/*
 * PI Clock Servo for CRF media clock recovery.
 * Based on linuxptp pi.c (Copyright (C) 2011 Richard Cochran, GPL v2+)
 */

/* Proportional-integral servo for media clock recovery.
 *
 * Consumes a phase error per feedback edge and returns the frequency
 * correction to apply, in parts per billion. Pure arithmetic: it knows
 * nothing about edges, packets or hardware. Pairing an edge with the CRF
 * timestamp it belongs to happens above this, in the listener.
 *
 * Two stages, in order:
 *
 *   1. Frequency acquisition. Open loop. Returns zero correction and simply
 *      watches the phase error drift for freq_est_interval_s. The slope is
 *      the frequency error, which seeds the integrator. Skipping this leaves
 *      the loop with no idea how wrong the oscillator is.
 *
 *   2. Phase tracking. Closed loop. The frequency term is already correct, so
 *      this removes the residual phase offset and follows slow drift.
 *
 * Within tracking there is a large-error regime, entered when the phase error
 * exceeds slew_thresh_ns. Proportional gain does the slewing and the
 * integrator is held.
 *
 * Holding the integrator there is deliberate anti-windup, and it matters.
 * Measured on a closed-loop model of the CM4 path, recovering a 137 us
 * offset with a correct frequency seed:
 *
 *   integrator held      phase monotonic to zero in 2 s, no overshoot,
 *                        drift stays at its seeded value
 *   integrator running   drift winds up 87 percent past target, phase
 *                        overshoots to +30 us, 28 s to ring down
 *
 * The consequence is a hard requirement on the seed. In the large-error
 * regime the loop settles where the applied correction equals actual drift:
 *
 *     offset = (drift_actual - drift_estimate) / slew_kp
 *
 * and it only escapes to the PI branch once |offset| <= slew_thresh_ns, so
 *
 *     |drift_actual - drift_estimate| <= slew_kp * slew_thresh_ns
 *
 * With the defaults that is 1000 ppb. A seed further out than 1 ppm leaves
 * the loop parked at a fixed offset, reporting SERVO_LOCKED, with no path to
 * refine the estimate - the only branch that could is the one it cannot
 * reach. Stage 1 over 2 s resolves frequency to a few ppb, so acquisition
 * clears that bar by two orders of magnitude; the failure mode is only
 * reachable by starting tracking without a seed at all.
 *
 * A caller that must delay tracking, because it has just reprogrammed the
 * clock source and would otherwise measure the transient, should wait for
 * the source to settle and then let stage 1 run. Beginning tracking with no
 * frequency estimate does not work on any real oscillator.
 *
 * Divergence from linuxptp pi.c, for anyone comparing them:
 *   - freq_est_interval is clamped to 2 s here, 1000 s upstream. With the
 *     default ki the unclamped value is 320 s, so upstream would spend over
 *     five minutes acquiring. A 300 Hz media clock cannot wait that long.
 *   - kp and ki are supplied by the caller rather than derived from the sync
 *     interval and stability-normalised as upstream does.
 *   - The large-error regime and the fast-integrator warmup do not exist
 *     upstream.
 */

#pragma once

#include <stdint.h>

enum servo_state {
	SERVO_UNLOCKED,
	SERVO_JUMP,
	SERVO_LOCKED,
};

/* Servo tuning. Fill with pi_servo_config_init() and adjust as needed;
 * the defaults are the values this demo was characterised with. */
struct pi_servo_config {
	double kp;			/* proportional gain, ppb per ns */
	double ki;			/* integral gain, ppb per ns */

	/* Integral gain used for the first fast_ki_samples of tracking, to
	 * pull in the residual phase offset faster than ki alone would. */
	double ki_fast;
	unsigned int fast_ki_samples;

	/* Large-error regime: proportional-only gain, and the phase error
	 * above which it is used instead of the PI branch. */
	double slew_kp;
	int64_t slew_thresh_ns;

	/* Correction is clamped to +/- max_ppb. */
	double max_ppb;

	/* A phase error beyond this is treated as a step rather than drift:
	 * the servo restarts, including frequency acquisition. Zero disables
	 * the check. */
	double step_threshold_ns;

	/* Open-loop frequency acquisition window, in seconds. Longer gives a
	 * better estimate; the error scales as timestamp noise over this. */
	double freq_est_interval_s;
};

/* Populate with the defaults used by the CRF demos. */
void pi_servo_config_init(struct pi_servo_config *cfg);

struct pi_servo;

/* Create a servo. Returns NULL on bad configuration or allocation failure. */
struct pi_servo *pi_servo_create(const struct pi_servo_config *cfg);

void pi_servo_destroy(struct pi_servo *servo);

/* Feed one phase error and get the frequency correction to apply.
 *
 * @offset_ns:   measured phase error, edge minus reference
 * @local_ts_ns: local timestamp of this sample, used to time the frequency
 *               acquisition window
 * @state:       set to SERVO_UNLOCKED while acquiring, SERVO_LOCKED once
 *               tracking. Callers should not apply the correction while
 *               unlocked.
 *
 * Returns the correction in ppb. */
double pi_servo_sample(struct pi_servo *servo,
		       int64_t offset_ns,
		       uint64_t local_ts_ns,
		       enum servo_state *state);

/* Discard all state and reacquire from scratch, including the frequency
 * estimate. The correct response to a clock source that has been
 * reprogrammed, once it has settled. */
void pi_servo_reset(struct pi_servo *servo);

/* Current frequency estimate in ppb. */
double pi_servo_get_drift(const struct pi_servo *servo);
