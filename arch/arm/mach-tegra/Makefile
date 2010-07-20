obj-y                                   += common.o
obj-y                                   += io.o
obj-y                                   += irq.o legacy_irq.o
obj-y                                   += clock.o
obj-y                                   += timer.o
obj-y                                   += gpio.o
obj-y                                   += pinmux.o
obj-y                                   += devices.o
obj-y                                   += delay.o
obj-y                                   += powergate.o
obj-y					+= suspend.o
obj-y					+= fuse.o
obj-y                                   += usb_phy.o
obj-y					+= tegra_i2s_audio.o
obj-$(CONFIG_FIQ)                       += fiq.o fiq_glue.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += clock.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_clocks.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += tegra2_dvfs.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)         += suspend-t2.o
obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= tegra2_save.o
obj-$(CONFIG_CPU_V7)			+= cortex_a9_save.o

obj-$(CONFIG_ARCH_TEGRA_2x_SOC)		+= pinmux-t2-tables.o
obj-$(CONFIG_SMP)                       += platsmp.o localtimer.o headsmp.o
obj-$(CONFIG_TEGRA_SYSTEM_DMA)		+= dma.o
obj-$(CONFIG_CPU_FREQ)                  += cpu-tegra.o
obj-$(CONFIG_CPU_IDLE)			+= cpuidle.o
obj-$(CONFIG_TEGRA_IOVMM)               += iovmm.o
obj-$(CONFIG_TEGRA_IOVMM_GART)          += iovmm-gart.o

obj-${CONFIG_MACH_HARMONY}              += board-harmony.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony-pinmux.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony-panel.o
obj-${CONFIG_MACH_HARMONY}              += board-harmony-sdhci.o

obj-${CONFIG_MACH_VENTANA}              += board-ventana.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-pinmux.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-sdhci.o
obj-${CONFIG_MACH_VENTANA}              += board-ventana-power.o

obj-${CONFIG_MACH_SEABOARD}             += board-seaboard.o
obj-${CONFIG_MACH_SEABOARD}             += board-seaboard-pinmux.o
obj-${CONFIG_MACH_SEABOARD}             += board-seaboard-sdhci.o
obj-${CONFIG_MACH_SEABOARD}             += board-seaboard-power.o
