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

/* Peripheral register access through /dev/mem.
 *
 * Maps one or more physical regions and hands back volatile pointers. Both
 * SoC clock drivers here need this - BCM2711 wants three separate blocks,
 * RP1 one - and both were unwinding partial failures by hand.
 *
 * Requires root, and a kernel that permits device MMIO through /dev/mem
 * (CONFIG_STRICT_DEVMEM allows it for device memory).
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

/* One mapped region. */
struct devmem_region {
	uint64_t phys;			/* physical base, filled by caller */
	size_t len;			/* length in bytes, filled by caller */
	const char *name;		/* for diagnostics, filled by caller */
	volatile uint32_t *base;	/* filled by devmem_map() */
};

/* Map @count regions from a single /dev/mem descriptor.
 *
 * Each region's phys, len and name must be set; base is filled in. On any
 * failure every region mapped so far is unmapped, the descriptor is closed,
 * and no region is left valid - so a failed call needs no cleanup.
 *
 * @fd receives the descriptor, to be passed to devmem_unmap().
 *
 * Returns 0 on success, -1 on error with a message naming the region that
 * failed. */
int devmem_map(struct devmem_region *regions, size_t count, int *fd);

/* Unmap every region with a non-NULL base and close @fd. Safe to call after
 * a partially initialised structure, and safe to call twice. */
void devmem_unmap(struct devmem_region *regions, size_t count, int fd);
