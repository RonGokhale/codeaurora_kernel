#ifndef _IRQ_GIC_V2M_H_
#define _IRQ_GIC_V2M_H_

struct v2m_data {
	spinlock_t msi_cnt_lock;
	struct resource res;      /* GICv2m resource */
	void __iomem *base;       /* GICv2m virt address */
	unsigned int spi_start;   /* The SPI number that MSIs start */
	unsigned int nr_spis;     /* The number of SPIs for MSIs */
	unsigned long *bm;        /* MSI vector bitmap */
};

#endif /*_IRQ_GIC_V2M_H_*/
