/*
 * Copyright (c) 2024, Tom
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of Intel Corporation nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* ALSA PCM setup for the AAF talker and listener.
 *
 * Both open an interleaved PCM with the same hw_params sequence and differ
 * only in direction, buffer sizing and whether sw_params is needed. The
 * stream parameters come from struct aaf_profile so the PCM and the wire
 * format cannot disagree.
 */

#pragma once

#include <alsa/asoundlib.h>

#include "aaf-profile.h"

struct alsa_config {
	const char *device;
	snd_pcm_stream_t stream;	/* SND_PCM_STREAM_PLAYBACK or _CAPTURE */

	/* Requested sizes. Updated in place to what ALSA actually granted. */
	snd_pcm_uframes_t period;
	snd_pcm_uframes_t buffer;

	/* Playback only, ignored for capture. Zero leaves the ALSA default. */
	snd_pcm_uframes_t start_threshold;
	snd_pcm_uframes_t avail_min;
};

/* ALSA sample format for an AVTP_AAF_FORMAT_* code.
 *
 * AAF carries samples big-endian on the wire and callers byte-swap to host
 * order, so these are the little-endian forms. Returns
 * SND_PCM_FORMAT_UNKNOWN for a code with no ALSA equivalent. */
snd_pcm_format_t alsa_format_from_aaf(uint8_t aaf_format);

/* Open and configure a PCM from @cfg and the stream @profile.
 *
 * Rate, channel count and sample format come from the profile. On success
 * cfg->period and cfg->buffer hold the granted sizes and a one-line summary
 * is written to stderr.
 *
 * Returns the PCM handle, or NULL on error with a message. */
snd_pcm_t *alsa_open(struct alsa_config *cfg,
		     const struct aaf_profile *profile);

/* Write @frames of silence, in chunks, to prime a playback buffer.
 * Returns 0 on success, or the negative ALSA error that stopped it. */
int alsa_prefill_silence(snd_pcm_t *pcm, snd_pcm_uframes_t frames,
			 const struct aaf_profile *profile);
