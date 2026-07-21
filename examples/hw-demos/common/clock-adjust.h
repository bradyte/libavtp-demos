/*
 * Clock Adjustment Interface
 *
 * Abstraction for frequency adjustment of a media clock output.
 * Backends: BCM2711 PWM clock divider, kernel module ioctl, etc.
 */

#pragma once

typedef int (*clock_adjust_fn)(void *ctx, double ppb);
