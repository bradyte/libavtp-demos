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

/* ALSA PCM setup for the AAF talker and listener. */

#include <stdio.h>
#include <string.h>

#include "alsa-util.h"

/* Silence is written in chunks from a fixed buffer rather than allocating
 * one the size of the prefill. */
#define SILENCE_FRAMES		1024
#define SILENCE_MAX_CHANNELS	8

snd_pcm_format_t alsa_format_from_aaf(uint8_t aaf_format)
{
	switch (aaf_format) {
	case AVTP_AAF_FORMAT_INT_16BIT:
		return SND_PCM_FORMAT_S16_LE;
	case AVTP_AAF_FORMAT_INT_24BIT:
		return SND_PCM_FORMAT_S24_3LE;
	case AVTP_AAF_FORMAT_INT_32BIT:
		return SND_PCM_FORMAT_S32_LE;
	case AVTP_AAF_FORMAT_FLOAT_32BIT:
		return SND_PCM_FORMAT_FLOAT_LE;
	default:
		return SND_PCM_FORMAT_UNKNOWN;
	}
}

snd_pcm_t *alsa_open(struct alsa_config *cfg,
		     const struct aaf_profile *profile)
{
	const char *dir;
	snd_pcm_hw_params_t *params;
	snd_pcm_sw_params_t *swparams;
	snd_pcm_format_t format;
	snd_pcm_t *pcm;
	unsigned int rate;
	int res;

	if (!cfg || !cfg->device || !profile)
		return NULL;

	dir = cfg->stream == SND_PCM_STREAM_PLAYBACK ? "playback" : "capture";

	format = alsa_format_from_aaf(profile->format);
	if (format == SND_PCM_FORMAT_UNKNOWN) {
		fprintf(stderr, "ALSA: no format for AAF code %u\n",
			profile->format);
		return NULL;
	}

	rate = aaf_profile_sample_rate(profile);
	if (!rate) {
		fprintf(stderr, "ALSA: no sample rate for AAF nsr %u\n",
			profile->nsr);
		return NULL;
	}

	res = snd_pcm_open(&pcm, cfg->device, cfg->stream, 0);
	if (res < 0) {
		fprintf(stderr, "ALSA open failed: %s\n", snd_strerror(res));
		return NULL;
	}

	snd_pcm_hw_params_alloca(&params);
	snd_pcm_hw_params_any(pcm, params);

	snd_pcm_hw_params_set_access(pcm, params,
				     SND_PCM_ACCESS_RW_INTERLEAVED);
	snd_pcm_hw_params_set_format(pcm, params, format);
	snd_pcm_hw_params_set_channels(pcm, params, profile->channels);
	snd_pcm_hw_params_set_rate_near(pcm, params, &rate, 0);
	snd_pcm_hw_params_set_period_size_near(pcm, params, &cfg->period, 0);
	snd_pcm_hw_params_set_buffer_size_near(pcm, params, &cfg->buffer);

	res = snd_pcm_hw_params(pcm, params);
	if (res < 0) {
		fprintf(stderr, "ALSA hw_params failed: %s\n",
			snd_strerror(res));
		snd_pcm_close(pcm);
		return NULL;
	}

	if (cfg->stream == SND_PCM_STREAM_PLAYBACK &&
	    (cfg->start_threshold || cfg->avail_min)) {
		snd_pcm_sw_params_alloca(&swparams);
		snd_pcm_sw_params_current(pcm, swparams);

		if (cfg->start_threshold)
			snd_pcm_sw_params_set_start_threshold(pcm, swparams,
							      cfg->start_threshold);
		if (cfg->avail_min)
			snd_pcm_sw_params_set_avail_min(pcm, swparams,
							cfg->avail_min);

		res = snd_pcm_sw_params(pcm, swparams);
		if (res < 0) {
			fprintf(stderr, "ALSA sw_params failed: %s\n",
				snd_strerror(res));
			snd_pcm_close(pcm);
			return NULL;
		}
	}

	fprintf(stderr, "ALSA %s: %s, %s, %u Hz, %u ch, period=%lu, "
		"buffer=%lu", dir, cfg->device,
		snd_pcm_format_name(format), rate, profile->channels,
		cfg->period, cfg->buffer);
	if (cfg->start_threshold)
		fprintf(stderr, ", start_thr=%lu", cfg->start_threshold);
	fprintf(stderr, "\n");

	return pcm;
}

int alsa_prefill_silence(snd_pcm_t *pcm, snd_pcm_uframes_t frames,
			 const struct aaf_profile *profile)
{
	int32_t silence[SILENCE_FRAMES * SILENCE_MAX_CHANNELS];
	snd_pcm_uframes_t chunk_max;

	if (!pcm || !profile || profile->channels == 0)
		return -EINVAL;

	if (profile->channels > SILENCE_MAX_CHANNELS) {
		fprintf(stderr, "ALSA: prefill supports at most %d channels\n",
			SILENCE_MAX_CHANNELS);
		return -EINVAL;
	}

	memset(silence, 0, sizeof(silence));
	chunk_max = SILENCE_FRAMES;

	while (frames > 0) {
		snd_pcm_uframes_t chunk = frames < chunk_max ? frames : chunk_max;
		snd_pcm_sframes_t written = snd_pcm_writei(pcm, silence, chunk);

		if (written < 0)
			return written;

		frames -= written;
	}

	return 0;
}
