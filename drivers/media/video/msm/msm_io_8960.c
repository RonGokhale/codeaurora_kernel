/* Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/regulator/consumer.h>
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/vreg.h>
#include <mach/camera.h>
#include <mach/clk.h>
#include <mach/msm_bus.h>
#include <mach/msm_bus_board.h>
#include "msm_ispif.h"


/* MIPI	CSI	PHY registers */
#define MIPI_CSIPHY_LNn_CFG1_ADDR                0x0
#define MIPI_CSIPHY_LNn_CFG2_ADDR                0x4
#define MIPI_CSIPHY_LNn_CFG3_ADDR                0x8
#define MIPI_CSIPHY_LNn_CFG4_ADDR                0xC
#define MIPI_CSIPHY_LNn_CFG5_ADDR                0x10
#define MIPI_CSIPHY_LNCK_CFG1_ADDR               0x100
#define MIPI_CSIPHY_LNCK_CFG2_ADDR               0x104
#define MIPI_CSIPHY_LNCK_CFG3_ADDR               0x108
#define MIPI_CSIPHY_LNCK_CFG4_ADDR               0x10C
#define MIPI_CSIPHY_LNCK_CFG5_ADDR               0x110
#define MIPI_CSIPHY_LNCK_MISC1_ADDR              0x128
#define MIPI_CSIPHY_GLBL_T_INIT_CFG0_ADDR        0x1E0
#define MIPI_CSIPHY_T_WAKEUP_CFG0_ADDR           0x1E8
#define MIPI_CSIPHY_GLBL_PWR_CFG_ADDR           0x0144
#define MIPI_CSIPHY_INTERRUPT_STATUS0_ADDR      0x0180
#define MIPI_CSIPHY_INTERRUPT_STATUS1_ADDR      0x0184
#define MIPI_CSIPHY_INTERRUPT_STATUS2_ADDR      0x0188
#define MIPI_CSIPHY_INTERRUPT_STATUS3_ADDR      0x018C
#define MIPI_CSIPHY_INTERRUPT_STATUS4_ADDR      0x0190
#define MIPI_CSIPHY_INTERRUPT_MASK0_ADDR        0x01A0
#define MIPI_CSIPHY_INTERRUPT_MASK1_ADDR        0x01A4
#define MIPI_CSIPHY_INTERRUPT_MASK2_ADDR        0x01A8
#define MIPI_CSIPHY_INTERRUPT_MASK3_ADDR        0x01AC
#define MIPI_CSIPHY_INTERRUPT_MASK4_ADDR        0x01B0
#define MIPI_CSIPHY_INTERRUPT_CLEAR0_ADDR       0x01C0
#define MIPI_CSIPHY_INTERRUPT_CLEAR1_ADDR       0x01C4
#define MIPI_CSIPHY_INTERRUPT_CLEAR2_ADDR       0x01C8
#define MIPI_CSIPHY_INTERRUPT_CLEAR3_ADDR       0x01CC
#define MIPI_CSIPHY_INTERRUPT_CLEAR4_ADDR       0x01D0

/* MIPI	CSID registers */
#define CSID_CORE_CTRL_ADDR                     0x4
#define CSID_RST_CMD_ADDR                       0x8
#define CSID_CID_LUT_VC_0_ADDR                  0xc
#define CSID_CID_LUT_VC_1_ADDR                  0x10
#define CSID_CID_LUT_VC_2_ADDR                  0x14
#define CSID_CID_LUT_VC_3_ADDR                  0x18
#define CSID_CID_n_CFG_ADDR                     0x1C
#define CSID_IRQ_CLEAR_CMD_ADDR                 0x5c
#define CSID_IRQ_MASK_ADDR                      0x60
#define CSID_IRQ_STATUS_ADDR                    0x64
#define CSID_CAPTURED_UNMAPPED_LONG_PKT_HDR_ADDR    0x68
#define CSID_CAPTURED_MMAPPED_LONG_PKT_HDR_ADDR     0x6c
#define CSID_CAPTURED_SHORT_PKT_ADDR                0x70
#define CSID_CAPTURED_LONG_PKT_HDR_ADDR             0x74
#define CSID_CAPTURED_LONG_PKT_FTR_ADDR             0x78
#define CSID_PIF_MISR_DL0_ADDR                      0x7C
#define CSID_PIF_MISR_DL1_ADDR                      0x80
#define CSID_PIF_MISR_DL2_ADDR                      0x84
#define CSID_PIF_MISR_DL3_ADDR                      0x88
#define CSID_STATS_TOTAL_PKTS_RCVD_ADDR             0x8C
#define CSID_STATS_ECC_ADDR                         0x90
#define CSID_STATS_CRC_ADDR                         0x94
#define CSID_TG_CTRL_ADDR                           0x9C
#define CSID_TG_VC_CFG_ADDR                         0xA0
#define CSID_TG_DT_n_CFG_0_ADDR                     0xA8
#define CSID_TG_DT_n_CFG_1_ADDR                     0xAC
#define CSID_TG_DT_n_CFG_2_ADDR                     0xB0
#define CSID_TG_DT_n_CFG_3_ADDR                     0xD8

/* ISPIF registers */

static struct clk *camio_cam_clk;
static struct clk *camio_vfe_clk;
static struct clk *camio_csi_src_clk;
static struct clk *camio_csi0_vfe_clk;
/*static struct clk *camio_csi1_vfe_clk;*/
static struct clk *camio_csi0_clk;
static struct clk *camio_csi1_clk;
static struct clk *camio_csi0_pclk;
/*static struct clk *camio_csi1_pclk;*/
static struct clk *camio_csi1_src_clk;
static struct clk *camio_csi_pix_clk;
static struct clk *camio_csi_rdi_clk;
static struct clk *camio_csiphy0_timer_clk;
static struct clk *camio_csiphy1_timer_clk;

/*static struct clk *camio_vfe_pclk;*/
static struct clk *camio_jpeg_clk;
static struct clk *camio_jpeg_pclk;
static struct clk *camio_vpe_clk;
static struct clk *camio_vpe_pclk;
static struct regulator *fs_vfe;
static struct regulator *fs_ijpeg;
static struct regulator *fs_vpe;
static struct regulator *ldo11;
static struct regulator *lvs5;
static struct regulator *ldo12;

static struct msm_camera_io_ext camio_ext;
static struct msm_camera_io_clk camio_clk;
static struct platform_device *camio_dev;
static struct resource *csidio, *csiphyio;
void __iomem *csidbase, *csiphybase;

static struct msm_bus_vectors cam_init_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_preview_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 1521190000,
		.ib  = 1521190000,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 1521190000,
		.ib  = 1521190000,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_video_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 1521190000,
		.ib  = 1521190000,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 1521190000,
		.ib  = 1521190000,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 1521190000,
		.ib  = 1521190000,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
};

static struct msm_bus_vectors cam_snapshot_vectors[] = {
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 1521190000,
		.ib  = 1521190000,
	},
	{
		.src = MSM_BUS_MASTER_VFE,
		.dst = MSM_BUS_SLAVE_EBI_CH0,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_VPE,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 0,
		.ib  = 0,
	},
	{
		.src = MSM_BUS_MASTER_JPEG_ENC,
		.dst = MSM_BUS_SLAVE_SMI,
		.ab  = 1521190000,
		.ib  = 1521190000,
	},
};

static struct msm_bus_paths cam_bus_client_config[] = {
	{
		ARRAY_SIZE(cam_init_vectors),
		cam_init_vectors,
	},
	{
		ARRAY_SIZE(cam_preview_vectors),
		cam_preview_vectors,
	},
	{
		ARRAY_SIZE(cam_video_vectors),
		cam_video_vectors,
	},
	{
		ARRAY_SIZE(cam_snapshot_vectors),
		cam_snapshot_vectors,
	},
};

static struct msm_bus_scale_pdata cam_bus_client_pdata = {
		cam_bus_client_config,
		ARRAY_SIZE(cam_bus_client_config),
		.name = "msm_camera",
};


void msm_io_w(u32 data, void __iomem *addr)
{
	CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));
	writel_relaxed((data), (addr));
}

void msm_io_w_mb(u32 data, void __iomem *addr)
{
	CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));
	wmb();
	writel_relaxed((data), (addr));
	wmb();
}

u32 msm_io_r(void __iomem *addr)
{
	uint32_t data = readl_relaxed(addr);
	CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));
	return data;
}

u32 msm_io_r_mb(void __iomem *addr)
{
	uint32_t data;
	rmb();
	data = readl_relaxed(addr);
	rmb();
	CDBG("%s: %08x %08x\n", __func__, (int) (addr), (data));
	return data;
}

void msm_io_memcpy_toio(void __iomem *dest_addr,
	void __iomem *src_addr, u32 len)
{
	int i;
	u32 *d = (u32 *) dest_addr;
	u32 *s = (u32 *) src_addr;
	/* memcpy_toio does not work. Use writel_relaxed for now */
	for (i = 0; i < len; i++)
		writel_relaxed(*s++, d++);
}

void msm_io_dump(void __iomem *addr, int size)
{
	char line_str[128], *p_str;
	int i;
	u32 *p = (u32 *) addr;
	u32 data;
	CDBG("%s: %p %d\n", __func__, addr, size);
	line_str[0] = '\0';
	p_str = line_str;
	for (i = 0; i < size/4; i++) {
		if (i % 4 == 0) {
			sprintf(p_str, "%08x: ", (u32) p);
			p_str += 10;
		}
		data = readl_relaxed(p++);
		sprintf(p_str, "%08x ", data);
		p_str += 9;
		if ((i + 1) % 4 == 0) {
			CDBG("%s\n", line_str);
			line_str[0] = '\0';
			p_str = line_str;
		}
	}
	if (line_str[0] != '\0')
		CDBG("%s\n", line_str);
}

void msm_io_memcpy(void __iomem *dest_addr, void __iomem *src_addr, u32 len)
{
	CDBG("%s: %p %p %d\n", __func__, dest_addr, src_addr, len);
	msm_io_memcpy_toio(dest_addr, src_addr, len / 4);
	msm_io_dump(dest_addr, len);
}

static void msm_camera_vreg_enable(void)
{
	ldo11 = regulator_get(NULL, "8058_l11");
	if (IS_ERR(ldo11)) {
		CDBG("%s: VREG LDO11 get failed\n", __func__);
		ldo11 = NULL;
		return;
	}
	if (regulator_set_voltage(ldo11, 2850000, 2850000)) {
		CDBG("%s: VREG LDO11 set voltage failed\n",  __func__);
		goto ldo11_disable;
	}
	if (regulator_enable(ldo11)) {
		CDBG("%s: VREG LDO11 enable failed\n", __func__);
		goto ldo11_put;
	}

	lvs5 = regulator_get(NULL, "8058_lvs5");
	if (IS_ERR(lvs5)) {
		CDBG("%s: VREG LVS5 get failed\n", __func__);
		lvs5 = NULL;
		goto ldo11_disable;
	}
	if (regulator_enable(lvs5)) {
		CDBG("%s: VREG LVS5 enable failed\n", __func__);
		goto lvs5_put;
	}

	ldo12 = regulator_get(NULL, "8058_l12");
	if (IS_ERR(ldo12)) {
		CDBG("%s: VREG LDO12 get failed\n", __func__);
		ldo12 = NULL;
		goto lvs5_disable;
	}
	if (regulator_set_voltage(ldo12, 1200000, 1200000)) {
		CDBG("%s: VREG LDO25 set voltage failed\n",  __func__);
		goto ldo12_disable;
	}
	if (regulator_enable(ldo12)) {
		CDBG("%s: VREG LDO12 enable failed\n", __func__);
		goto ldo12_put;
	}

	fs_vfe = regulator_get(NULL, "fs_vfe");
	if (IS_ERR(fs_vfe)) {
		CDBG("%s: Regulator FS_VFE get failed %ld\n", __func__,
			PTR_ERR(fs_vfe));
		fs_vfe = NULL;
	} else if (regulator_enable(fs_vfe)) {
		CDBG("%s: Regulator FS_VFE enable failed\n", __func__);
		regulator_put(fs_vfe);
	}
	return;

ldo12_disable:
	regulator_disable(ldo12);
ldo12_put:
	regulator_put(ldo12);
lvs5_disable:
	regulator_disable(lvs5);
lvs5_put:
	regulator_put(lvs5);
ldo11_disable:
	regulator_disable(ldo11);
ldo11_put:
	regulator_put(ldo11);
}

static void msm_camera_vreg_disable(void)
{
	if (ldo11) {
		regulator_disable(ldo11);
		regulator_put(ldo11);
	}

	if (lvs5) {
		regulator_disable(lvs5);
		regulator_put(lvs5);
	}

	if (ldo12) {
		regulator_disable(ldo12);
		regulator_put(ldo12);
	}

	if (fs_vfe) {
		regulator_disable(fs_vfe);
		regulator_put(fs_vfe);
	}
}

int msm_camio_clk_enable(enum msm_camio_clk_type clktype)
{
	int rc = 0;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_CAM_MCLK_CLK:
		camio_cam_clk =
		clk = clk_get(NULL, "cam_clk");
		msm_camio_clk_rate_set_2(clk, camio_clk.mclk_clk_rate);
		break;

	case CAMIO_VFE_CLK:
		camio_vfe_clk =
		clk = clk_get(NULL, "vfe_clk");
		msm_camio_clk_rate_set_2(clk, camio_clk.vfe_clk_rate);
		break;

	case CAMIO_CSI0_VFE_CLK:
		camio_csi0_vfe_clk =
		clk = clk_get(NULL, "csi_vfe_clk");
		break;
/*
	case CAMIO_CSI1_VFE_CLK:
		camio_csi1_vfe_clk =
		clk = clk_get(&camio_dev->dev, "csi_vfe_clk");
		break;
*/
	case CAMIO_CSI_SRC_CLK:
		camio_csi_src_clk =
		clk = clk_get(NULL, "csi_src_clk");
		msm_camio_clk_rate_set_2(clk, 384000000);
		break;

	case CAMIO_CSI1_SRC_CLK:
		camio_csi1_src_clk =
			clk = clk_get(NULL, "csi_src_clk");
		msm_camio_clk_rate_set_2(clk, 384000000);
		break;

	case CAMIO_CSI0_CLK:
		camio_csi0_clk =
		clk = clk_get(NULL, "csi_clk");
		break;

	case CAMIO_CSI1_CLK:
		camio_csi1_clk =
		clk = clk_get(&camio_dev->dev, "csi_clk");
		break;

	case CAMIO_CSI_PIX_CLK:
		camio_csi_pix_clk =
		clk = clk_get(NULL, "csi_pix_clk");
		break;

	case CAMIO_CSI_RDI_CLK:
		camio_csi_rdi_clk =
		clk = clk_get(NULL, "csi_rdi_clk");
		break;

	case CAMIO_CSIPHY0_TIMER_CLK:
		camio_csiphy0_timer_clk =
		clk = clk_get(NULL, "csiphy0_timer_clk");
		break;

	case CAMIO_CSIPHY1_TIMER_CLK:
		camio_csiphy1_timer_clk =
		clk = clk_get(&camio_dev->dev, "csiphy1_timer_clk");
		break;

	case CAMIO_CSI0_PCLK:
		camio_csi0_pclk =
		clk = clk_get(NULL, "csi_pclk");
		break;

	case CAMIO_JPEG_CLK:
		camio_jpeg_clk =
		clk = clk_get(NULL, "ijpeg_clk");
		clk_set_min_rate(clk, 144000000);
		break;

	case CAMIO_JPEG_PCLK:
		camio_jpeg_pclk =
		clk = clk_get(NULL, "ijpeg_pclk");
		break;

	case CAMIO_VPE_CLK:
		camio_vpe_clk =
		clk = clk_get(NULL, "vpe_clk");
		msm_camio_clk_set_min_rate(clk, 150000000);
		break;

	case CAMIO_VPE_PCLK:
		camio_vpe_pclk =
		clk = clk_get(NULL, "vpe_pclk");
		break;

	default:
		break;
	}

	if (!IS_ERR(clk))
		clk_enable(clk);
	else
		rc = -1;
	return rc;
}

int msm_camio_clk_disable(enum msm_camio_clk_type clktype)
{
	int rc = 0;
	struct clk *clk = NULL;

	switch (clktype) {
	case CAMIO_CAM_MCLK_CLK:
		clk = camio_cam_clk;
		break;

	case CAMIO_VFE_CLK:
		clk = camio_vfe_clk;
		break;

/* TODO: CSIPHY, CSID and ISPIF reqd clocks */

	case CAMIO_JPEG_CLK:
		clk = camio_jpeg_clk;
		break;

	case CAMIO_JPEG_PCLK:
		clk = camio_jpeg_pclk;
		break;

	case CAMIO_VPE_CLK:
		clk = camio_vpe_clk;
		break;

	case CAMIO_VPE_PCLK:
		clk = camio_vpe_pclk;
		break;

	default:
		break;
	}

	if (!IS_ERR(clk)) {
		clk_disable(clk);
		clk_put(clk);
	} else
		rc = -1;
	return rc;
}

void msm_camio_vfe_clk_rate_set(int rate)
{
	struct clk *clk = camio_vfe_clk;
	if (rate > clk_get_rate(clk))
		clk_set_rate(clk, rate);
}

void msm_camio_clk_rate_set(int rate)
{
	struct clk *clk = camio_cam_clk;
	clk_set_rate(clk, rate);
}

void msm_camio_clk_rate_set_2(struct clk *clk, int rate)
{
	clk_set_rate(clk, rate);
}

void msm_camio_clk_set_min_rate(struct clk *clk, int rate)
{
	clk_set_min_rate(clk, rate);
}

static irqreturn_t msm_io_csi_irq(int irq_num, void *data)
{
	uint32_t irq;
	irq = msm_io_r(csidbase + CSID_IRQ_STATUS_ADDR);
	CDBG("%s CSID_IRQ_STATUS_ADDR = 0x%x\n", __func__, irq);
	msm_io_w(irq, csidbase + CSID_IRQ_CLEAR_CMD_ADDR);
	irq = msm_io_r(csidbase + 0x7C);
	CDBG("%s CSID_PIF_MISR_DL0 = 0x%x\n", __func__, irq);
	irq = msm_io_r(csidbase + 0x80);
	CDBG("%s CSID_PIF_MISR_DL1 = 0x%x\n", __func__, irq);
	irq = msm_io_r(csidbase + 0x84);
	CDBG("%s CSID_PIF_MISR_DL2 = 0x%x\n", __func__, irq);
	irq = msm_io_r(csidbase + 0x88);
	CDBG("%s CSID_PIF_MISR_DL3 = 0x%x\n", __func__, irq);
	irq = msm_io_r(csidbase + 0x8C);
	CDBG("%s PACKET Count = %d\n", __func__, irq);
	return IRQ_HANDLED;
}
/*
void msm_io_read_interrupt(void)
{
	uint32_t irq;
	irq = msm_io_r(csiphybase + MIPI_CSIPHY_INTERRUPT_STATUS0_ADDR);
	CDBG("%s MIPI_CSIPHY_INTERRUPT_STATUS0 = 0x%x\n", __func__, irq);
	irq = msm_io_r(csiphybase + MIPI_CSIPHY_INTERRUPT_STATUS0_ADDR);
	CDBG("%s MIPI_CSIPHY_INTERRUPT_STATUS0 = 0x%x\n", __func__, irq);
	irq = msm_io_r(csiphybase + MIPI_CSIPHY_INTERRUPT_STATUS1_ADDR);
	CDBG("%s MIPI_CSIPHY_INTERRUPT_STATUS1 = 0x%x\n", __func__, irq);
	irq = msm_io_r(csiphybase + MIPI_CSIPHY_INTERRUPT_STATUS2_ADDR);
	CDBG("%s MIPI_CSIPHY_INTERRUPT_STATUS2 = 0x%x\n", __func__, irq);
	irq = msm_io_r(csiphybase + MIPI_CSIPHY_INTERRUPT_STATUS3_ADDR);
	CDBG("%s MIPI_CSIPHY_INTERRUPT_STATUS3 = 0x%x\n", __func__, irq);
	irq = msm_io_r(csiphybase + MIPI_CSIPHY_INTERRUPT_STATUS4_ADDR);
	CDBG("%s MIPI_CSIPHY_INTERRUPT_STATUS4 = 0x%x\n", __func__, irq);
	msm_io_w(irq, csiphybase + MIPI_CSIPHY_INTERRUPT_CLEAR0_ADDR);
	msm_io_w(irq, csiphybase + MIPI_CSIPHY_INTERRUPT_CLEAR1_ADDR);
	msm_io_w(irq, csiphybase + MIPI_CSIPHY_INTERRUPT_CLEAR2_ADDR);
	msm_io_w(irq, csiphybase + MIPI_CSIPHY_INTERRUPT_CLEAR3_ADDR);
	msm_io_w(irq, csiphybase + MIPI_CSIPHY_INTERRUPT_CLEAR4_ADDR);
	msm_io_w(0x1, csiphybase + 0x164);
	msm_io_w(0x0, csiphybase + 0x164);
	return;
}
*/

static irqreturn_t msm_io_csiphy_irq(int irq_num, void *data)
{
	uint32_t irq;
	irq = msm_io_r(csiphybase + MIPI_CSIPHY_INTERRUPT_STATUS0_ADDR);
	CDBG("%s MIPI_CSIPHY_INTERRUPT_STATUS0 = 0x%x\n", __func__, irq);
	irq = msm_io_r(csiphybase + MIPI_CSIPHY_INTERRUPT_STATUS0_ADDR);
	CDBG("%s MIPI_CSIPHY_INTERRUPT_STATUS0 = 0x%x\n", __func__, irq);
	irq = msm_io_r(csiphybase + MIPI_CSIPHY_INTERRUPT_STATUS1_ADDR);
	CDBG("%s MIPI_CSIPHY_INTERRUPT_STATUS1 = 0x%x\n", __func__, irq);
	irq = msm_io_r(csiphybase + MIPI_CSIPHY_INTERRUPT_STATUS2_ADDR);
	CDBG("%s MIPI_CSIPHY_INTERRUPT_STATUS2 = 0x%x\n", __func__, irq);
	irq = msm_io_r(csiphybase + MIPI_CSIPHY_INTERRUPT_STATUS3_ADDR);
	CDBG("%s MIPI_CSIPHY_INTERRUPT_STATUS3 = 0x%x\n", __func__, irq);
	irq = msm_io_r(csiphybase + MIPI_CSIPHY_INTERRUPT_STATUS4_ADDR);
	CDBG("%s MIPI_CSIPHY_INTERRUPT_STATUS4 = 0x%x\n", __func__, irq);
	msm_io_w(irq, csiphybase + MIPI_CSIPHY_INTERRUPT_CLEAR0_ADDR);
	msm_io_w(irq, csiphybase + MIPI_CSIPHY_INTERRUPT_CLEAR1_ADDR);
	msm_io_w(irq, csiphybase + MIPI_CSIPHY_INTERRUPT_CLEAR2_ADDR);
	msm_io_w(irq, csiphybase + MIPI_CSIPHY_INTERRUPT_CLEAR3_ADDR);
	msm_io_w(irq, csiphybase + MIPI_CSIPHY_INTERRUPT_CLEAR4_ADDR);
	msm_io_w(0x1, csiphybase + 0x164);
	msm_io_w(0x0, csiphybase + 0x164);
	return IRQ_HANDLED;
}

int msm_camio_jpeg_clk_disable(void)
{
	int rc = 0;
	if (fs_ijpeg) {
		rc = regulator_disable(fs_ijpeg);
		if (rc < 0) {
			CDBG("%s: Regulator disable failed %d\n", __func__, rc);
			return rc;
		}
		regulator_put(fs_ijpeg);
	}
	rc = msm_camio_clk_disable(CAMIO_JPEG_PCLK);
	if (rc < 0)
		return rc;
	rc = msm_camio_clk_disable(CAMIO_JPEG_CLK);
	CDBG("%s: exit %d\n", __func__, rc);
	return rc;
}

int msm_camio_jpeg_clk_enable(void)
{
	int rc = 0;
	rc = msm_camio_clk_enable(CAMIO_JPEG_CLK);
	if (rc < 0)
		return rc;
	rc = msm_camio_clk_enable(CAMIO_JPEG_PCLK);
	if (rc < 0)
		return rc;
	fs_ijpeg = regulator_get(NULL, "fs_ijpeg");
	if (IS_ERR(fs_ijpeg)) {
		CDBG("%s: Regulator FS_IJPEG get failed %ld\n", __func__,
			PTR_ERR(fs_ijpeg));
		fs_ijpeg = NULL;
	} else if (regulator_enable(fs_ijpeg)) {
		CDBG("%s: Regulator FS_IJPEG enable failed\n", __func__);
		regulator_put(fs_ijpeg);
	}
	CDBG("%s: exit %d\n", __func__, rc);
	return rc;
}

int msm_camio_vpe_clk_disable(void)
{
	int rc = 0;
	if (fs_vpe) {
		regulator_disable(fs_vpe);
		regulator_put(fs_vpe);
	}

	rc = msm_camio_clk_disable(CAMIO_VPE_CLK);
	if (rc < 0)
		return rc;
	rc = msm_camio_clk_disable(CAMIO_VPE_PCLK);
	return rc;
}

int msm_camio_vpe_clk_enable(uint32_t clk_rate)
{
	int rc = 0;
	(void)clk_rate;
	fs_vpe = regulator_get(NULL, "fs_vpe");
	if (IS_ERR(fs_vpe)) {
		CDBG("%s: Regulator FS_VPE get failed %ld\n", __func__,
			PTR_ERR(fs_vpe));
		fs_vpe = NULL;
	} else if (regulator_enable(fs_vpe)) {
		CDBG("%s: Regulator FS_VPE enable failed\n", __func__);
		regulator_put(fs_vpe);
	}

	rc = msm_camio_clk_enable(CAMIO_VPE_CLK);
	if (rc < 0)
		return rc;
	rc = msm_camio_clk_enable(CAMIO_VPE_PCLK);
	return rc;
}

int msm_camio_enable(struct platform_device *pdev)
{
	int rc = 0;
	void __iomem *clkbase;
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;

	camio_dev = pdev;
	camio_ext = camdev->ioext;
	camio_clk = camdev->ioclk;

	clkbase = ioremap(0x4000000, 0x334);
	msm_io_w(0x0, clkbase + 0x204);
	msm_io_w(0x0, clkbase + 0x208);
	msm_io_w(0x0, clkbase + 0x20C);
	msm_io_w(0x0, clkbase + 0x210);

/* TODO */
	msm_camio_clk_enable(CAMIO_VFE_CLK);
	msm_camio_clk_enable(CAMIO_CSI0_VFE_CLK);
	msm_camio_clk_enable(CAMIO_CSI_SRC_CLK);
	msm_camio_clk_enable(CAMIO_CSI1_SRC_CLK);
	msm_camio_clk_enable(CAMIO_CSI0_CLK);
	msm_camio_clk_enable(CAMIO_CSI1_CLK);
	msm_camio_clk_enable(CAMIO_CSI_PIX_CLK);
	msm_camio_clk_enable(CAMIO_CSI_RDI_CLK);
	msm_camio_clk_enable(CAMIO_CSIPHY0_TIMER_CLK);
	msm_camio_clk_enable(CAMIO_CSIPHY1_TIMER_CLK);
	msm_camio_clk_enable(CAMIO_CSI0_PCLK);

	csidio = request_mem_region(camio_ext.csiphy,
		camio_ext.csisz, pdev->name);
	if (!csidio) {
		rc = -EBUSY;
		goto common_fail;
	}
	csidbase = ioremap(camio_ext.csiphy,
		camio_ext.csisz);
	if (!csidbase) {
		rc = -ENOMEM;
		goto csi_busy;
	}
	rc = request_irq(camio_ext.csiirq, msm_io_csi_irq,
		IRQF_TRIGGER_RISING, "csid", 0);
	if (rc < 0)
		goto csi_irq_fail;

	csiphyio = request_mem_region(camio_ext.csiphyphy,
		camio_ext.csiphysz, pdev->name);
	if (!csidio) {
		rc = -EBUSY;
		goto csi_irq_fail;
	}
	csiphybase = ioremap(camio_ext.csiphyphy,
		camio_ext.csiphysz);
	if (!csiphybase) {
		rc = -ENOMEM;
		goto csiphy_busy;
	}
	rc = request_irq(camio_ext.csiphyirq , msm_io_csiphy_irq,
		IRQF_TRIGGER_RISING, "csiphy", 0);
	if (rc < 0)
		goto csiphy_irq_fail;
	rc = msm_ispif_init(pdev);
	if (rc < 0)
		goto csiphy_irq_fail;
	CDBG("camio enable done\n");
	return 0;
csiphy_irq_fail:
	iounmap(csiphybase);
csiphy_busy:
	release_mem_region(camio_ext.csiphyphy, camio_ext.csiphysz);
csi_irq_fail:
	iounmap(csidbase);
csi_busy:
	release_mem_region(camio_ext.csiphy, camio_ext.csisz);
common_fail:
	msm_camio_clk_disable(CAMIO_CSI0_PCLK);
	msm_camio_clk_disable(CAMIO_CSIPHY1_TIMER_CLK);
	msm_camio_clk_disable(CAMIO_CSIPHY0_TIMER_CLK);
	msm_camio_clk_disable(CAMIO_CSI_RDI_CLK);
	msm_camio_clk_disable(CAMIO_CSI_PIX_CLK);
	msm_camio_clk_disable(CAMIO_CSI1_CLK);
	msm_camio_clk_disable(CAMIO_CSI0_CLK);
	msm_camio_clk_disable(CAMIO_CSI1_SRC_CLK);
	msm_camio_clk_disable(CAMIO_CSI_SRC_CLK);
	msm_camio_clk_disable(CAMIO_CSI0_VFE_CLK);
	msm_camio_clk_disable(CAMIO_VFE_CLK);
	msm_camera_vreg_disable();
	camdev->camera_gpio_off();
	return rc;
}

void msm_camio_disable(struct platform_device *pdev)
{

	free_irq(camio_ext.csiphy, 0);
	iounmap(csiphybase);
	release_mem_region(camio_ext.csiphyphy, camio_ext.csiphysz);

	free_irq(camio_ext.csiirq, 0);
	iounmap(csidbase);
	release_mem_region(camio_ext.csiphy, camio_ext.csisz);
	return;
	msm_camio_clk_disable(CAMIO_CSI0_PCLK);
	msm_camio_clk_disable(CAMIO_CSIPHY1_TIMER_CLK);
	msm_camio_clk_disable(CAMIO_CSIPHY0_TIMER_CLK);
	msm_camio_clk_disable(CAMIO_CSI_RDI_CLK);
	msm_camio_clk_disable(CAMIO_CSI_PIX_CLK);
	msm_camio_clk_disable(CAMIO_CSI1_CLK);
	msm_camio_clk_disable(CAMIO_CSI0_CLK);
	msm_camio_clk_disable(CAMIO_CSI1_SRC_CLK);
	msm_camio_clk_disable(CAMIO_CSI_SRC_CLK);
	msm_camio_clk_disable(CAMIO_CSI0_VFE_CLK);
	msm_camio_clk_disable(CAMIO_VFE_CLK);
	msm_ispif_release(pdev);
}

int msm_camio_sensor_clk_on(struct platform_device *pdev)
{
	int rc = 0;
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
	camio_dev = pdev;
	camio_ext = camdev->ioext;
	camio_clk = camdev->ioclk;

	msm_camera_vreg_enable();
	msleep(20);
	rc = camdev->camera_gpio_on();
	if (rc < 0)
		return rc;
	return msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
}

int msm_camio_sensor_clk_off(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
	msm_camera_vreg_disable();
	camdev->camera_gpio_off();
	return msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
}

void msm_camio_vfe_blk_reset(void)
{
	return;
}

int msm_camio_probe_on(struct platform_device *pdev)
{
	int rc = 0;
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
	camio_dev = pdev;
	camio_ext = camdev->ioext;
	camio_clk = camdev->ioclk;

	rc = camdev->camera_gpio_on();
	if (rc < 0)
		return rc;
	msm_camera_vreg_enable();
	return msm_camio_clk_enable(CAMIO_CAM_MCLK_CLK);
}

int msm_camio_probe_off(struct platform_device *pdev)
{
	struct msm_camera_sensor_info *sinfo = pdev->dev.platform_data;
	struct msm_camera_device_platform_data *camdev = sinfo->pdata;
	msm_camera_vreg_disable();
	camdev->camera_gpio_off();
	return msm_camio_clk_disable(CAMIO_CAM_MCLK_CLK);
}

int msm_camio_csid_cid_lut(struct msm_camera_csid_lut_params *csid_lut_params)
{
	int rc = 0;
	uint32_t val = 0;

	if (csid_lut_params->dt < 0x12 || csid_lut_params->dt > 0x37) {
		CDBG("%s: unsupported data type 0x%x\n",
			__func__, csid_lut_params->dt);
		return rc;
	}

	val = msm_io_r(csidbase + CSID_CID_LUT_VC_0_ADDR + csid_lut_params->vc)
		& ~(0xFF << csid_lut_params->cid * 8);
	val |= csid_lut_params->dt << csid_lut_params->cid * 8;
	msm_io_w(val, csidbase + CSID_CID_LUT_VC_0_ADDR + csid_lut_params->vc);

	val = csid_lut_params->decode_format << 4 |
		csid_lut_params->rdi_en << 1 |
		csid_lut_params->ispif_en;

	msm_io_w(val, csidbase + CSID_CID_n_CFG_ADDR +
		(csid_lut_params->cid * 4));
	return rc;
}

int msm_camio_csid_config(struct msm_camera_csid_params *csid_params)
{
	int rc = 0;
	struct msm_ispif_params ispif_params;
	uint32_t val = 0;
	val = csid_params->lane_cnt - 1;
	val |= csid_params->lane_assign << 2;
	val |= 0x1 << 10;
	val |= 0x1 << 11;
	val |= 0x1 << 12;
	val |= 0x1 << 28;
	msm_io_w(val, csidbase + CSID_CORE_CTRL_ADDR);

	rc = msm_camio_csid_cid_lut(&csid_params->lut_params);
	if (rc < 0)
		return rc;

	msm_io_w(0xFFFFFFFF, csidbase + CSID_IRQ_MASK_ADDR);
	msm_io_w(0xFFFFFFFF, csidbase + CSID_IRQ_CLEAR_CMD_ADDR);

	ispif_params.intftype = PIX0;
	ispif_params.cid_mask = 0x0001;
	ispif_params.csid = 0x01;

	msm_ispif_config(&ispif_params, 1);
	msm_ispif_start_intf_transfer(&ispif_params);

	msleep(20);
	return rc;
}

int msm_camio_csiphy_config(struct msm_camera_csiphy_params *csiphy_params)
{
	int rc = 0;
	int i = 0;
	uint32_t val = 0;
	if (csiphy_params->lane_cnt < 1 || csiphy_params->lane_cnt > 4) {
		CDBG("%s: unsupported lane cnt %d\n",
			__func__, csiphy_params->lane_cnt);
		return rc;
	}

	val = 0x3;
	msm_io_w(((2 * csiphy_params->lane_cnt - 1) << 2) | val,
			 csiphybase + MIPI_CSIPHY_GLBL_PWR_CFG_ADDR);
	msm_io_w(0x1, csiphybase + MIPI_CSIPHY_GLBL_T_INIT_CFG0_ADDR);
	msm_io_w(0x1, csiphybase + MIPI_CSIPHY_T_WAKEUP_CFG0_ADDR);

	for (i = 0; i < csiphy_params->lane_cnt; i++) {
		msm_io_w(0x10, csiphybase + MIPI_CSIPHY_LNn_CFG1_ADDR + 0x40*i);
		msm_io_w(0x5F, csiphybase + MIPI_CSIPHY_LNn_CFG2_ADDR + 0x40*i);
		msm_io_w(csiphy_params->settle_cnt,
			csiphybase + MIPI_CSIPHY_LNn_CFG3_ADDR + 0x40*i);
		msm_io_w(0x00000052,
			csiphybase + MIPI_CSIPHY_LNn_CFG5_ADDR + 0x40*i);
	}

	msm_io_w(0x00000000, csiphybase + MIPI_CSIPHY_LNCK_CFG1_ADDR);
	msm_io_w(0x5F, csiphybase + MIPI_CSIPHY_LNCK_CFG2_ADDR);
	msm_io_w(csiphy_params->settle_cnt,
			 csiphybase + MIPI_CSIPHY_LNCK_CFG3_ADDR);
	msm_io_w(0x5, csiphybase + MIPI_CSIPHY_LNCK_CFG4_ADDR);
	msm_io_w(0x2, csiphybase + MIPI_CSIPHY_LNCK_CFG5_ADDR);
	msm_io_w(0x0, csiphybase + 0x128);

	for (i = 0; i <= csiphy_params->lane_cnt; i++) {
		msm_io_w(0xFFFFFFFF,
			csiphybase + MIPI_CSIPHY_INTERRUPT_MASK0_ADDR + 0x4*i);
		msm_io_w(0xFFFFFFFF,
			csiphybase + MIPI_CSIPHY_INTERRUPT_CLEAR0_ADDR + 0x4*i);
	}
	return rc;
}

void msm_camio_set_perf_lvl(enum msm_bus_perf_setting perf_setting)
{
	static uint32_t bus_perf_client;
	int rc = 0;
	switch (perf_setting) {
	case S_INIT:
		bus_perf_client =
			msm_bus_scale_register_client(&cam_bus_client_pdata);
		if (!bus_perf_client) {
			CDBG("%s: Registration Failed!!!\n", __func__);
			bus_perf_client = 0;
			return;
		}
		CDBG("%s: S_INIT rc = %u\n", __func__, bus_perf_client);
		break;
	case S_EXIT:
		if (bus_perf_client) {
			CDBG("%s: S_EXIT\n", __func__);
			msm_bus_scale_unregister_client(bus_perf_client);
		} else
			CDBG("%s: Bus Client NOT Registered!!!\n", __func__);
		break;
	case S_PREVIEW:
		if (bus_perf_client) {
			rc = msm_bus_scale_client_update_request(
				bus_perf_client, 1);
			CDBG("%s: S_PREVIEW rc = %d\n", __func__, rc);
		} else
			CDBG("%s: Bus Client NOT Registered!!!\n", __func__);
		break;
	case S_VIDEO:
		if (bus_perf_client) {
			rc = msm_bus_scale_client_update_request(
				bus_perf_client, 2);
			CDBG("%s: S_VIDEO rc = %d\n", __func__, rc);
		} else
			CDBG("%s: Bus Client NOT Registered!!!\n", __func__);
		break;
	case S_CAPTURE:
		if (bus_perf_client) {
			rc = msm_bus_scale_client_update_request(
				bus_perf_client, 3);
			CDBG("%s: S_CAPTURE rc = %d\n", __func__, rc);
		} else
			CDBG("%s: Bus Client NOT Registered!!!\n", __func__);
		break;
	case S_DEFAULT:
		break;
	default:
		pr_warning("%s: INVALID CASE\n", __func__);
	}
}
