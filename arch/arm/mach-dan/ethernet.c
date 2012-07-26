/*
	All files except if stated otherwise in the begining of the file are under the ISC license:
	-----------------------------------------------------------------------------------

	Copyright (c) 2010-2012 Design Art Networks Ltd.

	Permission to use, copy, modify, and/or distribute this software for any
	purpose with or without fee is hereby granted, provided that the above
	copyright notice and this permission notice appear in all copies.

	THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
	WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
	MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
	ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
	WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
	ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
	OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/


/*
 *  linux/arch/arm/mach-dan/ethernet.c
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/stmmac.h>
#include <linux/phy.h>
#include <linux/mii.h>
#include <linux/netdevice.h>


#include <mach/irqs.h>
#include <asm/irq.h>
#include <asm/setup.h>

#include "board.h"


#define GMAC_MII_ADDR           0x00000010      /* MII Address */
#define GMAC_MII_DATA           0x00000014      /* MII Data */
#define MII_BUSY		0x00000001
#define MII_WRITE		0x00000002

struct phy_device *dan_pseudo_phy_device;

void dan_bus_setup(void __iomem *ioaddr)
{
	struct phy_device *phydev = dan_pseudo_phy_device;
	struct net_device *ndev = dev_get_by_name(&init_net, "eth0");

	if (!ndev) {
		printk("%s:%d not found\n", __func__, __LINE__);
	}
	else {
		if (ndev->phydev)
			phydev = ndev->phydev;
		else
			printk("%s:%d phydev is NULL\n", __func__, __LINE__);
		dev_put(ndev);
	}
	

	(void)ioaddr;

	phydev->speed 		= 100;
	phydev->duplex		= DUPLEX_FULL;
	phydev->state		= PHY_UP;
	phydev->link		= 1;
	phydev->interface	= PHY_INTERFACE_MODE;

	if (phydev->adjust_link) {
		phydev->adjust_link(phydev->attached_dev);
		phydev->speed 		= 1000;
		phydev->adjust_link(phydev->attached_dev);
	}
	if (phydev->adjust_state) {
		phydev->adjust_state(phydev->attached_dev);
	}
}


static const uint16_t mii_reg[] =
{
	[ MII_BMCR           ] = BMCR_FULLDPLX | BMCR_SPEED1000	| BMCR_ANENABLE/* Basic mode control register */,
	[ MII_BMSR           ] = BMSR_ESTATEN | BMSR_LSTATUS | BMSR_ANEGCOMPLETE /* Basic mode status register  */,
	[ MII_PHYSID1        ] = 0x01        /* PHYS ID 1                   */,
	[ MII_PHYSID2        ] = 0x01        /* PHYS ID 2                   */,
	[ MII_ADVERTISE      ] = 0x00        /* Advertisement control reg   */,
	[ MII_LPA            ] = 0x00        /* Link partner ability reg    */,
	[ MII_EXPANSION      ] = 0x06        /* Expansion register          */,
	[ MII_CTRL1000       ] = ADVERTISE_1000FULL /* 1000BASE-T control   */,
	[ MII_STAT1000       ] = LPA_1000FULL  /* 1000BASE-T status         */,
	[ MII_ESTATUS        ] = ESTATUS_1000_TFULL/* Extended Status       */,
	[ MII_DCOUNTER       ] = 0x12        /* Disconnect counter          */,
	[ MII_FCSCOUNTER     ] = 0x13        /* False carrier counter       */,
	[ MII_NWAYTEST       ] = 0x14        /* N-way auto-neg test reg     */,
	[ MII_RERRCOUNTER    ] = 0x15        /* Receive error counter       */,
	[ MII_SREVISION      ] = 0x16        /* Silicon revision            */,
	[ MII_RESV1          ] = 0x17        /* Reserved...                 */,
	[ MII_LBRERROR       ] = 0x18        /* Lpback, rx, bypass error    */,
	[ MII_PHYADDR        ] = 0x19        /* PHY address                 */,
	[ MII_RESV2          ] = 0x1a        /* Reserved...                 */,
	[ MII_TPISTATUS      ] = 0x1b        /* TPI status for 10mbps       */,
	[ MII_NCONFIG        ] = 0x1c        /* Network interface config    */
};


/**
 * dan_mdio_read
 * @bus: points to the mii_bus structure
 * @phyaddr: MII addr reg bits 15-11
 * @phyreg: MII addr reg bits 10-6
 * Description: it reads data from the MII register from within the phy device.
 * For the 7111 GMAC, we must set the bit 0 in the MII address register while
 * accessing the PHY registers.
 * Fortunately, it seems this has no drawback for the 7109 MAC.
 */
static int dan_mdio_read(struct mii_bus *bus, int phyaddr, int phyreg)
{
	(void)bus;
	(void)phyaddr;
	(void)phyreg;
	return mii_reg[phyreg];
}

/**
 * dan_mdio_write
 * @bus: points to the mii_bus structure
 * @phyaddr: MII addr reg bits 15-11
 * @phyreg: MII addr reg bits 10-6
 * @phydata: phy data
 * Description: it writes the data into the MII register from within the device.
 */
static int dan_mdio_write(struct mii_bus *bus, int phyaddr, int phyreg,
			     u16 phydata)
{
	(void)bus;
	(void)phyaddr;
	(void)phyreg;
	(void)phydata;
	return 0;
}

int dan_phy_fixup(struct phy_device *phydev)
{
	struct mii_bus	*bus	= phydev->bus;

	dan_pseudo_phy_device	= phydev;

	phydev->speed 		= 100;
	phydev->duplex		= DUPLEX_FULL;
	phydev->state		= PHY_UP;
	phydev->link		= 1;
	phydev->interface	= PHY_INTERFACE_MODE;

	bus->read		= dan_mdio_read;
	bus->write		= dan_mdio_write;

	if (phydev->adjust_link) {
		phydev->adjust_link(phydev->attached_dev);
		phydev->speed 		= 1000;
		phydev->adjust_link(phydev->attached_dev);
	}
	if (phydev->adjust_state)
		phydev->adjust_state(phydev->attached_dev);

	return 0;
}

