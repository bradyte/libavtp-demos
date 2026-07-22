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

/* Peripheral register access through /dev/mem. */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include "devmem.h"

void devmem_unmap(struct devmem_region *regions, size_t count, int fd)
{
	size_t i;

	if (regions) {
		for (i = 0; i < count; i++) {
			if (!regions[i].base)
				continue;
			munmap((void *)regions[i].base, regions[i].len);
			regions[i].base = NULL;
		}
	}

	if (fd >= 0)
		close(fd);
}

int devmem_map(struct devmem_region *regions, size_t count, int *fd)
{
	size_t i;
	int mem_fd;

	if (!regions || !fd || count == 0)
		return -1;

	for (i = 0; i < count; i++)
		regions[i].base = NULL;

	mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (mem_fd < 0) {
		fprintf(stderr, "devmem: cannot open /dev/mem: %s%s\n",
			strerror(errno),
			errno == EACCES ? " (needs root)" : "");
		return -1;
	}

	for (i = 0; i < count; i++) {
		void *p = mmap(NULL, regions[i].len, PROT_READ | PROT_WRITE,
			       MAP_SHARED, mem_fd, (off_t)regions[i].phys);

		if (p == MAP_FAILED) {
			fprintf(stderr,
				"devmem: mmap of %s at 0x%llx (%zu bytes) "
				"failed: %s\n",
				regions[i].name ? regions[i].name : "region",
				(unsigned long long)regions[i].phys,
				regions[i].len, strerror(errno));
			/* Leaves nothing mapped, so the caller has no
			 * partially initialised state to unwind. */
			devmem_unmap(regions, count, mem_fd);
			return -1;
		}

		regions[i].base = (volatile uint32_t *)p;
	}

	*fd = mem_fd;
	return 0;
}
