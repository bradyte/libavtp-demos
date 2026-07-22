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

/* Media clock backend registry. */

#include <stdio.h>
#include <string.h>

#include "media-clock.h"

extern const struct media_clock_ops mc_bcm2711_ops;

static const struct media_clock_ops *const backends[] = {
	&mc_bcm2711_ops,
};

#define BACKEND_COUNT (sizeof(backends) / sizeof(backends[0]))

const struct media_clock_ops *media_clock_select(const char *name)
{
	size_t i;

	if (name) {
		for (i = 0; i < BACKEND_COUNT; i++)
			if (!strcmp(backends[i]->name, name))
				return backends[i];

		fprintf(stderr, "Unknown media clock backend '%s' (have: %s)\n",
			name, media_clock_names());
		return NULL;
	}

	for (i = 0; i < BACKEND_COUNT; i++)
		if (backends[i]->probe && backends[i]->probe())
			return backends[i];

	fprintf(stderr, "No media clock backend matches this hardware "
		"(have: %s)\n", media_clock_names());
	return NULL;
}

const char *media_clock_names(void)
{
	static char buf[128];
	size_t i, off = 0;

	if (buf[0])
		return buf;

	for (i = 0; i < BACKEND_COUNT && off < sizeof(buf) - 1; i++)
		off += snprintf(buf + off, sizeof(buf) - off, "%s%s",
				i ? " " : "", backends[i]->name);

	return buf;
}
