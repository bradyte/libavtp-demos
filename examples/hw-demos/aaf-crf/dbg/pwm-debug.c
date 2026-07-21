/*
 * PWM Register Diagnostic Tool
 * Reads and displays BCM2711 PWM and clock registers
 */

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <unistd.h>

#define BCM2711_PERI_BASE	0xFE000000
#define GPIO_BASE		(BCM2711_PERI_BASE + 0x200000)
#define PWM_BASE		(BCM2711_PERI_BASE + 0x20C000)
#define CLOCK_BASE		(BCM2711_PERI_BASE + 0x101000)
#define BLOCK_SIZE		4096

#define GPFSEL0			0
#define GPFSEL1			1

#define PWM_CTL			0
#define PWM_STA			1
#define PWM_RNG1		4
#define PWM_DAT1		5
#define PWM_RNG2		8
#define PWM_DAT2		9

#define PWMCLK_CTL		40
#define PWMCLK_DIV		41

int main(int argc, char *argv[])
{
	int mem_fd;
	void *gpio_map, *pwm_map, *clk_map;
	volatile unsigned int *gpio, *pwm, *clk;
	unsigned int pin = 12;
	unsigned int fsel_reg, fsel_shift, fsel_val;

	if (argc > 1)
		pin = atoi(argv[1]);

	mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	if (mem_fd < 0) {
		perror("Failed to open /dev/mem");
		return 1;
	}

	gpio_map = mmap(NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE,
			MAP_SHARED, mem_fd, GPIO_BASE);
	pwm_map = mmap(NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE,
		       MAP_SHARED, mem_fd, PWM_BASE);
	clk_map = mmap(NULL, BLOCK_SIZE, PROT_READ | PROT_WRITE,
		       MAP_SHARED, mem_fd, CLOCK_BASE);

	if (gpio_map == MAP_FAILED || pwm_map == MAP_FAILED || clk_map == MAP_FAILED) {
		perror("mmap failed");
		close(mem_fd);
		return 1;
	}

	gpio = (volatile unsigned int *)gpio_map;
	pwm = (volatile unsigned int *)pwm_map;
	clk = (volatile unsigned int *)clk_map;

	printf("BCM2711 PWM Register Dump\n");
	printf("=========================\n\n");

	printf("GPIO Configuration:\n");
	fsel_reg = pin / 10;
	fsel_shift = (pin % 10) * 3;
	fsel_val = (gpio[fsel_reg] >> fsel_shift) & 0x7;
	printf("  GPIO%u FSEL%u[%u:%u] = %u (", pin, fsel_reg, fsel_shift + 2, fsel_shift, fsel_val);
	switch (fsel_val) {
	case 0: printf("INPUT"); break;
	case 1: printf("OUTPUT"); break;
	case 2: printf("ALT5"); break;
	case 3: printf("ALT4"); break;
	case 4: printf("ALT0"); break;
	case 5: printf("ALT1"); break;
	case 6: printf("ALT2"); break;
	case 7: printf("ALT3"); break;
	}
	printf(")\n\n");

	printf("PWM Registers:\n");
	printf("  CTL  = 0x%08x\n", pwm[PWM_CTL]);
	printf("    CH1 PWEN1=%u MSEN1=%u RPTL1=%u SBIT1=%u POLA1=%u USEF1=%u CLRF1=%u MODE1=%u\n",
	       (pwm[PWM_CTL] >> 0) & 1, (pwm[PWM_CTL] >> 7) & 1,
	       (pwm[PWM_CTL] >> 2) & 1, (pwm[PWM_CTL] >> 3) & 1,
	       (pwm[PWM_CTL] >> 4) & 1, (pwm[PWM_CTL] >> 5) & 1,
	       (pwm[PWM_CTL] >> 6) & 1, (pwm[PWM_CTL] >> 1) & 1);
	printf("    CH2 PWEN2=%u MSEN2=%u\n",
	       (pwm[PWM_CTL] >> 8) & 1, (pwm[PWM_CTL] >> 15) & 1);
	printf("  STA  = 0x%08x\n", pwm[PWM_STA]);
	printf("  RNG1 = %u\n", pwm[PWM_RNG1]);
	printf("  DAT1 = %u\n", pwm[PWM_DAT1]);
	printf("  RNG2 = %u\n", pwm[PWM_RNG2]);
	printf("  DAT2 = %u\n\n", pwm[PWM_DAT2]);

	printf("PWM Clock Registers:\n");
	printf("  CTL = 0x%08x\n", clk[PWMCLK_CTL]);
	printf("    ENAB=%u SRC=%u\n",
	       (clk[PWMCLK_CTL] >> 4) & 1, clk[PWMCLK_CTL] & 0xf);
	printf("  DIV = 0x%08x (DIVI=%u)\n",
	       clk[PWMCLK_DIV], clk[PWMCLK_DIV] >> 12);

	unsigned int divi = clk[PWMCLK_DIV] >> 12;
	unsigned int src = clk[PWMCLK_CTL] & 0xf;
	unsigned int range = pwm[PWM_RNG1];
	unsigned long base_freq = 0;

	printf("\nClock Source: ");
	switch (src) {
	case 0: printf("GND\n"); break;
	case 1: printf("Oscillator (19.2 MHz)\n"); base_freq = 19200000; break;
	case 4: printf("PLLA\n"); break;
	case 5: printf("PLLC\n"); break;
	case 6: printf("PLLD (750 MHz)\n"); base_freq = 750000000; break;
	case 7: printf("HDMI\n"); break;
	default: printf("Unknown (%u)\n", src); break;
	}

	if (base_freq && divi && range) {
		unsigned long pwm_clk = base_freq / divi;
		unsigned long output_freq = pwm_clk / range;
		printf("Calculated: %lu MHz / %u / %u = %lu Hz\n",
		       base_freq / 1000000, divi, range, output_freq);
	}

	munmap(gpio_map, BLOCK_SIZE);
	munmap(pwm_map, BLOCK_SIZE);
	munmap(clk_map, BLOCK_SIZE);
	close(mem_fd);

	return 0;
}
