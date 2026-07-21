/* rp1_pll_frac_actuator.c — glitchless frequency trim of RP1 pll_audio_core
 * via its 24-bit FBDIV_FRAC, driven in ppb by the CRF servo's PI output.
 *
 * VCO = f_xosc * (FBDIV_INT + FBDIV_FRAC / 2^24)
 * 1 LSB = f_xosc / 2^24 at the VCO  ->  ~1.94 ppb fractional at INT+frac=30.72
 *
 * Rules encoded here:
 *   - never touch FBDIV_INT at runtime (glitch); trim FRAC only
 *   - single aligned 32-bit store per update (atomic, glitchless by design)
 *   - authority clamp so a broken estimator can't drag the audio domain
 *
 * !!! VERIFY the three offsets below against the RP1 peripheral map for
 * !!! your silicon/doc revision before first run. Everything else derives
 * !!! from registers read at init.
 *
 * Build: gcc -O2 -c rp1_pll_frac_actuator.c    (run as root; STRICT_DEVMEM
 * permits device MMIO through /dev/mem)
 */
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <sys/mman.h>

/* ---- VERIFY against RP1 register map ---------------------------------- */
#define RP1_PERI_BASE     0x1f00000000ULL   /* RP1 peripherals via PCIe BAR */
#define CLOCKS_BLOCK_OFF  0x00018000UL      /* CLOCKS block within RP1      */
#define PLL_AUDIO_FBDIV_INT_OFF   0x0UL     /* <- fill from register map    */
#define PLL_AUDIO_FBDIV_FRAC_OFF  0x0UL     /* <- fill from register map    */
/* ----------------------------------------------------------------------- */

#define F_XOSC_HZ         50000000.0
#define FRAC_BITS         24
#define FRAC_MOD          (1u << FRAC_BITS)
#define AUTHORITY_PPM     100.0             /* clamp: +/-100 ppm from center */

static volatile uint32_t *clk;              /* mapped CLOCKS block           */
static uint32_t frac_center;                /* boot value = nominal          */
static double   ppb_per_lsb;
static long     max_codes;

static inline volatile uint32_t *reg(unsigned long off)
{
    return (volatile uint32_t *)((volatile uint8_t *)clk + off);
}

int actuator_init(void)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return -1; }

    clk = mmap(NULL, 0x4000, PROT_READ | PROT_WRITE, MAP_SHARED,
               fd, RP1_PERI_BASE + CLOCKS_BLOCK_OFF);
    close(fd);
    if (clk == MAP_FAILED) { perror("mmap"); return -1; }

    uint32_t fbint = *reg(PLL_AUDIO_FBDIV_INT_OFF);
    frac_center    = *reg(PLL_AUDIO_FBDIV_FRAC_OFF) & (FRAC_MOD - 1);

    double mult = (double)(fbint & 0xfff) + (double)frac_center / FRAC_MOD;
    double vco  = F_XOSC_HZ * mult;
    ppb_per_lsb = 1e9 / ((double)FRAC_MOD * mult);
    max_codes   = lround(AUTHORITY_PPM * 1000.0 / ppb_per_lsb);

    fprintf(stderr,
        "actuator: INT=%u FRAC=%u  VCO=%.1f Hz  %.3f ppb/LSB  clamp=+/-%ld\n",
        fbint & 0xfff, frac_center, vco, ppb_per_lsb, max_codes);

    /* sanity: expect VCO ~1.536 GHz for the 48k family */
    if (vco < 1.4e9 || vco > 1.7e9) {
        fprintf(stderr, "actuator: VCO implausible — wrong offsets? abort\n");
        return -1;
    }
    return 0;
}

/* Servo hook: trim relative to the boot-time nominal, in ppb.
 * Positive ppb speeds the media clock up. Returns applied ppb (post-
 * quantization/clamp) so the PI loop can track its own actuation. */
double actuator_set_ppb(double ppb)
{
    long codes = lround(ppb / ppb_per_lsb);
    if (codes >  max_codes) codes =  max_codes;
    if (codes < -max_codes) codes = -max_codes;

    long f = (long)frac_center + codes;
    if (f < 0 || f >= (long)FRAC_MOD) {     /* would need INT step: refuse  */
        fprintf(stderr, "actuator: frac range exhausted, holding\n");
        f = (f < 0) ? 0 : FRAC_MOD - 1;
    }
    *reg(PLL_AUDIO_FBDIV_FRAC_OFF) = (uint32_t)f;   /* single atomic store */
    return ((long)((uint32_t)f) - (long)frac_center) * ppb_per_lsb;
}
