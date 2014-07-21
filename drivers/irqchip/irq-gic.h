#ifndef _IRQ_GIC_H_
#define _IRQ_GIC_H_

#include <linux/msi.h>

union gic_base {
	void __iomem *common_base;
	void __percpu * __iomem *percpu_base;
};

#ifdef CONFIG_ARM_GIC_V2M
struct v2m_data {
	spinlock_t msi_cnt_lock;
	struct resource res;      /* GICv2m resource */
	void __iomem *base;       /* GICv2m virt address */
	unsigned int spi_start;   /* The SPI number that MSIs start */
	unsigned int nr_spis;     /* The number of SPIs for MSIs */
	unsigned long *bm;        /* MSI vector bitmap */
};
#endif

struct gic_chip_data {
	union gic_base dist_base;
	union gic_base cpu_base;
#ifdef CONFIG_CPU_PM
	u32 saved_spi_enable[DIV_ROUND_UP(1020, 32)];
	u32 saved_spi_conf[DIV_ROUND_UP(1020, 16)];
	u32 saved_spi_target[DIV_ROUND_UP(1020, 4)];
	u32 __percpu *saved_ppi_enable;
	u32 __percpu *saved_ppi_conf;
#endif
	struct irq_domain *domain;
	unsigned int gic_irqs;
#ifdef CONFIG_GIC_NON_BANKED
	void __iomem *(*get_base)(union gic_base *);
#endif
	struct irq_chip *irq_chip;
	struct msi_chip msi_chip;
#ifdef CONFIG_ARM_GIC_V2M
	struct v2m_data v2m_data;
#endif
};

#ifdef CONFIG_OF
int _gic_of_init(struct device_node *node,
		 struct device_node *parent,
		 struct irq_chip *chip,
		 struct gic_chip_data **gic) __init;
#endif

void gic_mask_irq(struct irq_data *d);
void gic_unmask_irq(struct irq_data *d);
void gic_eoi_irq(struct irq_data *d);
int gic_set_type(struct irq_data *d, unsigned int type);
int gic_retrigger(struct irq_data *d);

#ifdef CONFIG_SMP
int gic_set_affinity(struct irq_data *d,
			    const struct cpumask *mask_val,
			    bool force);
#endif

#ifdef CONFIG_PM
int gic_set_wake(struct irq_data *d, unsigned int on);
#endif

#endif /* _IRQ_GIC_H_ */
