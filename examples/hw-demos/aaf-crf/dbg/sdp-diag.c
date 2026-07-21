/*
 * SDP Diagnostic Tool
 *
 * Checks and cleans up i210 SDP configuration
 */

#include <fcntl.h>
#include <linux/ptp_clock.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

int main(int argc, char *argv[])
{
	int fd;
	struct ptp_clock_caps caps;
	struct ptp_extts_request extts_req;
	struct ptp_perout_request perout_req;
	int i;

	if (argc < 2) {
		fprintf(stderr, "Usage: %s /dev/ptpX\n", argv[0]);
		return 1;
	}

	fd = open(argv[1], O_RDWR);
	if (fd < 0) {
		perror("open ptp device");
		return 1;
	}

	/* Get capabilities */
	if (ioctl(fd, PTP_CLOCK_GETCAPS, &caps) < 0) {
		perror("PTP_CLOCK_GETCAPS");
		close(fd);
		return 1;
	}

	printf("PTP Clock Capabilities:\n");
	printf("  Max adjustments: %d\n", caps.max_adj);
	printf("  External timestamps: %d\n", caps.n_ext_ts);
	printf("  Periodic outputs: %d\n", caps.n_per_out);
	printf("  PPS support: %d\n", caps.pps);
	printf("  Pins: %d\n", caps.n_pins);
	printf("\n");

	/* Disable all external timestamp requests */
	printf("Disabling all external timestamp channels...\n");
	for (i = 0; i < caps.n_ext_ts; i++) {
		memset(&extts_req, 0, sizeof(extts_req));
		extts_req.index = i;
		extts_req.flags = 0;

		if (ioctl(fd, PTP_EXTTS_REQUEST, &extts_req) < 0) {
			printf("  Channel %d: already disabled (ok)\n", i);
		} else {
			printf("  Channel %d: disabled\n", i);
		}
	}

	/* Disable all periodic outputs */
	printf("\nDisabling all periodic output channels...\n");
	for (i = 0; i < caps.n_per_out; i++) {
		memset(&perout_req, 0, sizeof(perout_req));
		perout_req.index = i;

		if (ioctl(fd, PTP_PEROUT_REQUEST, &perout_req) < 0) {
			printf("  Channel %d: already disabled (ok)\n", i);
		} else {
			printf("  Channel %d: disabled\n", i);
		}
	}

	printf("\nNow try enabling external timestamp on channel 0...\n");
	memset(&extts_req, 0, sizeof(extts_req));
	extts_req.index = 0;
	extts_req.flags = PTP_ENABLE_FEATURE | PTP_RISING_EDGE;

	if (ioctl(fd, PTP_EXTTS_REQUEST, &extts_req) < 0) {
		perror("PTP_EXTTS_REQUEST (enable)");
		printf("\nStill busy! Check:\n");
		printf("  1. Is ptp4l or other PTP daemon running?\n");
		printf("  2. Is another process using SDP?\n");
		printf("  3. Try: sudo killall ptp4l phc2sys ts2phc\n");
	} else {
		printf("SUCCESS! External timestamp enabled on channel 0\n");

		/* Disable it again */
		memset(&extts_req, 0, sizeof(extts_req));
		extts_req.index = 0;
		extts_req.flags = 0;
		ioctl(fd, PTP_EXTTS_REQUEST, &extts_req);
		printf("Cleaned up and disabled\n");
	}

	close(fd);
	return 0;
}
