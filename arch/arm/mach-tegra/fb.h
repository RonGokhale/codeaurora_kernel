
#include <linux/types.h>

void __init tegra_protected_aperture_init(unsigned long aperture);
void tegra_move_framebuffer(unsigned long to, unsigned long from,
		        unsigned long size);
void __init tegra_reserve(unsigned long carveout_size, unsigned long fb_size,
		        unsigned long fb2_size);

extern unsigned long tegra_bootloader_fb_start;
extern unsigned long tegra_bootloader_fb_size;
extern unsigned long tegra_fb_start;
extern unsigned long tegra_fb_size;
extern unsigned long tegra_fb2_start;
extern unsigned long tegra_fb2_size;
extern unsigned long tegra_carveout_start;
extern unsigned long tegra_carveout_size;
extern unsigned long tegra_lp0_vec_start;
extern unsigned long tegra_lp0_vec_size;
extern unsigned long tegra_grhost_aperture;


