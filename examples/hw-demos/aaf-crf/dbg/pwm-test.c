/*
 * PWM Frequency Test Tool
 * Outputs a square wave at a specified frequency on GPIO12.
 * Runs until Ctrl-C.
 *
 * Usage: sudo ./pwm-test <freq_hz>
 *   e.g. sudo ./pwm-test 2000
 *        sudo ./pwm-test 300
 *        sudo ./pwm-test 48000
 */

#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

#define BCM2711_PERI_BASE	0xFE000000
#define GPIO_BASE		(BCM2711_PERI_BASE + 0x200000)
#define PWM_BASE		(BCM2711_PERI_BASE + 0x20C000)
#define CLOCK_BASE		(BCM2711_PERI_BASE + 0x101000)
#define BLOCK_SIZE		4096

#define GPFSEL1			1
#define PWM_CTL			0
#define PWM_RNG1		4
#define PWM_DAT1		5
#define PWMCLK_CTL		40
#define PWMCLK_DIV		41

#define CLK_PASSWD		(0x5A << 24)
#define CLK_CTL_MASH1		(1 << 9)
#define CLK_CTL_SRC_PLLD	6
#define CLK_CTL_ENAB		(1 << 4)
#define CLK_CTL_BUSY		(1 << 7)

#define PLLD_FREQ_HZ		750000000UL

static volatile int running = 1;

static void sig_handler(int sig)
{
	running = 0;
}

int main(int argc, char *argv[])
{
	int mem_fd;
	volatile unsigned int *gpio, *pwm, *clk;
	void *gpio_map, *pwm_map, *clk_map;
	unsigned int freq_hz, range, divi, divf;
	unsigned long pwm_clk_target;
	double divider;

	if (argc != 2) {
		fprintf(stderr, "Usage: %s <freq_hz>\n", argv[0]);
		return 1;
	}

	freq_hz = atoi(argv[1]);
	if (freq_hz == 0 || freq_hz > 1000000) {
		fprintf(stderr, "Frequency must be 1–1000000 Hz\n");
		return 1;
	}

	/* Select PWM range for good resolution at target frequency */
	if (freq_hz >= 3000)
		range = 512;
	else if (freq_hz >= 750)
		range = 2048;
	else if (freq_hz >= 200)
		range = 8192;
	else
		range = 16384;

	pwm_clk_target = (unsigned long)freq_hz * range;
	divider = (double)PLLD_FREQ_HZ / pwm_clk_target;
	divi = (unsigned int)divider;
	divf = (unsigned int)((divider - divi) * 4096);

	if (divi > 4095) {
		fprintf(stderr, "Error: freq too low (divider %u > 4095)\n", divi);
		return 1;
	}

	mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (mem_fd < 0) {
		perror("Failed to open /dev/mem (need root)");
		return 1;
	}

	gpio_map = mmap(NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE,
			MAP_SHARED, mem_fd, GPIO_BASE);
	pwm_map = mmap(NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE,
		       MAP_SHARED, mem_fd, PWM_BASE);
	clk_map = mmap(NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE,
		       MAP_SHARED, mem_fd, CLOCK_BASE);

	if (gpio_map == MAP_FAILED || pwm_map == MAP_FAILED ||
	    clk_map == MAP_FAILED) {
		perror("mmap failed");
		close(mem_fd);
		return 1;
	}

	gpio = (volatile unsigned int *)gpio_map;
	pwm = (volatile unsigned int *)pwm_map;
	clk = (volatile unsigned int *)clk_map;

	signal(SIGINT, sig_handler);
	signal(SIGTERM, sig_handler);

	/* GPIO12 = ALT0 (PWM0) */
	gpio[GPFSEL1] = (gpio[GPFSEL1] & ~(7 << 6)) | (4 << 6);

	/* Stop PWM */
	pwm[PWM_CTL] = 0;
	usleep(10);

	/* Kill clock */
	clk[PWMCLK_CTL] = CLK_PASSWD | (1 << 5);
	usleep(10);
	clk[PWMCLK_CTL] = CLK_PASSWD | 0;
	usleep(100);

	for (int i = 0; i < 100; i++) {
		if (!(clk[PWMCLK_CTL] & CLK_CTL_BUSY))
			break;
		usleep(10);
	}

	/* Set divider */
	clk[PWMCLK_DIV] = CLK_PASSWD | (divi << 12) | divf;
	usleep(10);

	/* Start clock: PLLD, MASH-1 */
	clk[PWMCLK_CTL] = CLK_PASSWD | CLK_CTL_ENAB | CLK_CTL_MASH1 |
			   CLK_CTL_SRC_PLLD;
	usleep(100);

	/* Configure PWM channel 0: M/S mode, 50% duty */
	pwm[PWM_RNG1] = range;
	pwm[PWM_DAT1] = range / 2;
	usleep(10);

	/* Enable: MSEN1=1, PWEN1=1 */
	pwm[PWM_CTL] = (1 << 7) | (1 << 0);

	double actual = (double)PLLD_FREQ_HZ / ((double)divi + (double)divf / 4096.0) / range;
	fprintf(stderr, "PWM running on GPIO12: target=%u Hz, actual=%.3f Hz\n",
		freq_hz, actual);
	fprintf(stderr, "  DIVI=%u DIVF=%u RANGE=%u\n", divi, divf, range);
	fprintf(stderr, "Press Ctrl-C to stop\n");

	while (running)
		sleep(1);

	/* Cleanup: stop PWM, kill clock, reset GPIO to input */
	pwm[PWM_CTL] = 0;
	usleep(10);
	clk[PWMCLK_CTL] = CLK_PASSWD | (1 << 5);
	usleep(10);
	clk[PWMCLK_CTL] = CLK_PASSWD | 0;
	gpio[GPFSEL1] = gpio[GPFSEL1] & ~(7 << 6);

	fprintf(stderr, "Stopped.\n");

	munmap(gpio_map, BLOCK_SIZE);
	munmap(pwm_map, BLOCK_SIZE);
	munmap(clk_map, BLOCK_SIZE);
	close(mem_fd);
	return 0;
}
