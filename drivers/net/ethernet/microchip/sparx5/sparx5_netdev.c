// SPDX-License-Identifier: GPL-2.0+
/* Microchip Sparx5 Switch driver
 *
 * Copyright (c) 2021 Microchip Technology Inc. and its subsidiaries.
 */

#include "sparx5_main_regs.h"
#include "sparx5_main.h"
#include "sparx5_port.h"
#include "sparx5_tc.h"

/* The IFH bit position of the first VSTAX bit. This is because the
 * VSTAX bit positions in Data sheet is starting from zero.
 */
#define VSTAX 73

#define ifh_encode_bitfield(ifh, value, pos, _width)			\
	({								\
		u32 width = (_width);					\
									\
		/* Max width is 5 bytes - 40 bits. In worst case this will
		 * spread over 6 bytes - 48 bits
		 */							\
		compiletime_assert(width <= 40,				\
				   "Unsupported width, must be <= 40");	\
		__ifh_encode_bitfield((ifh), (value), (pos), width);	\
	})

static void __ifh_encode_bitfield(void *ifh, u64 value, u32 pos, u32 width)
{
	u8 *ifh_hdr = ifh;
	/* Calculate the Start IFH byte position of this IFH bit position */
	u32 byte = (35 - (pos / 8));
	/* Calculate the Start bit position in the Start IFH byte */
	u32 bit  = (pos % 8);
	u64 encode = GENMASK_ULL(bit + width - 1, bit) & (value << bit);

	/* The b0-b7 goes into the start IFH byte */
	if (encode & 0xFF)
		ifh_hdr[byte] |= (u8)((encode & 0xFF));
	/* The b8-b15 goes into the next IFH byte */
	if (encode & 0xFF00)
		ifh_hdr[byte - 1] |= (u8)((encode & 0xFF00) >> 8);
	/* The b16-b23 goes into the next IFH byte */
	if (encode & 0xFF0000)
		ifh_hdr[byte - 2] |= (u8)((encode & 0xFF0000) >> 16);
	/* The b24-b31 goes into the next IFH byte */
	if (encode & 0xFF000000)
		ifh_hdr[byte - 3] |= (u8)((encode & 0xFF000000) >> 24);
	/* The b32-b39 goes into the next IFH byte */
	if (encode & 0xFF00000000)
		ifh_hdr[byte - 4] |= (u8)((encode & 0xFF00000000) >> 32);
	/* The b40-b47 goes into the next IFH byte */
	if (encode & 0xFF0000000000)
		ifh_hdr[byte - 5] |= (u8)((encode & 0xFF0000000000) >> 40);
}

void sparx5_set_port_ifh(struct sparx5 *sparx5, void *ifh_hdr, u16 portno)
{
	const struct sparx5_ops *ops = &sparx5->data->ops;
	u32 pipeline_pt =
		ops->get_pipeline_pt(SPX5_PACKET_PIPELINE_PT_ANA_DONE);
	int cpu_port0 = sparx5_get_internal_port(sparx5, PORT_CPU_0);

	/* VSTAX.RSV = 1. MSBit must be 1 */
	__ifh_encode_bitfield(ifh_hdr, 1,
			      ops->get_ifh_field_pos(IFH_VSTAX_RSV),
			      ops->get_ifh_field_width(IFH_VSTAX_RSV));
	/* VSTAX.INGR_DROP_MODE = Enable. Don't make head-of-line blocking */
	__ifh_encode_bitfield(ifh_hdr, 1,
			      ops->get_ifh_field_pos(IFH_VSTAX_INGR_DROP_MODE),
			      ops->get_ifh_field_width(IFH_VSTAX_INGR_DROP_MODE));
	/* MISC.CPU_MASK/DPORT = Destination port */
	__ifh_encode_bitfield(ifh_hdr, portno,
			      ops->get_ifh_field_pos(IFH_MISC_CPU_MASK_DPORT),
			      ops->get_ifh_field_width(IFH_MISC_CPU_MASK_DPORT));
	/* MISC.PIPELINE_PT */
	__ifh_encode_bitfield(ifh_hdr, pipeline_pt,
			      ops->get_ifh_field_pos(IFH_MISC_PIPELINE_PT),
			      ops->get_ifh_field_width(IFH_MISC_PIPELINE_PT));
	/* MISC.PIPELINE_ACT */
	__ifh_encode_bitfield(ifh_hdr, 1,
			      ops->get_ifh_field_pos(IFH_MISC_PIPELINE_ACT),
			      ops->get_ifh_field_width(IFH_MISC_PIPELINE_ACT));
	/* FWD.SRC_PORT = CPU */
	__ifh_encode_bitfield(ifh_hdr, cpu_port0,
			      ops->get_ifh_field_pos(IFH_FWD_SRC_PORT),
			      ops->get_ifh_field_width(IFH_FWD_SRC_PORT));
	/* FWD.SFLOW_ID (disable SFlow sampling) */
	__ifh_encode_bitfield(ifh_hdr, 124,
			      ops->get_ifh_field_pos(IFH_FWD_SFLOW_ID),
			      ops->get_ifh_field_width(IFH_FWD_SFLOW_ID));
	/* FWD.UPDATE_FCS = Enable. Enforce update of FCS. */
	__ifh_encode_bitfield(ifh_hdr, 1,
			      ops->get_ifh_field_pos(IFH_FWD_UPDATE_FCS),
			      ops->get_ifh_field_width(IFH_FWD_UPDATE_FCS));
}

void sparx5_set_port_ifh_rew_op(struct sparx5 *sparx5, void *ifh_hdr,
				u32 rew_op)
{
	const struct sparx5_ops *ops = &sparx5->data->ops;

	__ifh_encode_bitfield(ifh_hdr, rew_op,
			      ops->get_ifh_field_pos(IFH_VSTAX_REW_CMD),
			      ops->get_ifh_field_width(IFH_VSTAX_REW_CMD));
}

void sparx5_set_port_ifh_pdu_type(struct sparx5 *sparx5, void *ifh_hdr,
				  u32 pdu_type)
{
	const struct sparx5_ops *ops = &sparx5->data->ops;

	__ifh_encode_bitfield(ifh_hdr, pdu_type,
			      ops->get_ifh_field_pos(IFH_DST_PDU_TYPE),
			      ops->get_ifh_field_width(IFH_DST_PDU_TYPE));
}

void sparx5_set_port_ifh_pdu_w16_offset(struct sparx5 *sparx5,
					void *ifh_hdr, u32 pdu_w16_offset)
{
	const struct sparx5_ops *ops = &sparx5->data->ops;

	__ifh_encode_bitfield(ifh_hdr, pdu_w16_offset,
			      ops->get_ifh_field_pos(IFH_DST_PDU_W16_OFFSET),
			      ops->get_ifh_field_width(IFH_DST_PDU_W16_OFFSET));
}

void sparx5_set_port_ifh_timestamp(struct sparx5 *sparx5, void *ifh_hdr,
				   u64 timestamp)
{
	const struct sparx5_ops *ops = &sparx5->data->ops;

	__ifh_encode_bitfield(ifh_hdr, timestamp,
			      ops->get_ifh_field_pos(IFH_TS_TSTAMP),
			      ops->get_ifh_field_pos(IFH_TS_TSTAMP));
}

static int sparx5_port_open(struct net_device *ndev)
{
	struct sparx5_port *port = netdev_priv(ndev);
	int err = 0;

	sparx5_port_enable(port, true);
	err = phylink_of_phy_connect(port->phylink, port->of_node, 0);
	if (err) {
		netdev_err(ndev, "Could not attach to PHY\n");
		goto err_connect;
	}

	phylink_start(port->phylink);

	if (!ndev->phydev) {
		/* power up serdes */
		port->conf.power_down = false;
		if (port->conf.serdes_reset)
			err = sparx5_serdes_set(port->sparx5, port, &port->conf);
		else
			err = phy_power_on(port->serdes);
		if (err) {
			netdev_err(ndev, "%s failed\n", __func__);
			goto out_power;
		}
	}

	return 0;

out_power:
	phylink_stop(port->phylink);
	phylink_disconnect_phy(port->phylink);
err_connect:
	sparx5_port_enable(port, false);

	return err;
}

static int sparx5_port_stop(struct net_device *ndev)
{
	struct sparx5_port *port = netdev_priv(ndev);
	int err = 0;

	sparx5_port_enable(port, false);
	phylink_stop(port->phylink);
	phylink_disconnect_phy(port->phylink);

	if (!ndev->phydev) {
		/* power down serdes */
		port->conf.power_down = true;
		if (port->conf.serdes_reset)
			err = sparx5_serdes_set(port->sparx5, port, &port->conf);
		else
			err = phy_power_off(port->serdes);
		if (err)
			netdev_err(ndev, "%s failed\n", __func__);
	}
	return 0;
}

static void sparx5_set_rx_mode(struct net_device *dev)
{
	struct sparx5_port *port = netdev_priv(dev);
	struct sparx5 *sparx5 = port->sparx5;

	if (!test_bit(port->portno, sparx5->bridge_mask))
		__dev_mc_sync(dev, sparx5_mc_sync, sparx5_mc_unsync);
}

static int sparx5_port_get_phys_port_name(struct net_device *dev,
					  char *buf, size_t len)
{
	struct sparx5_port *port = netdev_priv(dev);
	int ret;

	ret = snprintf(buf, len, "p%d", port->portno);
	if (ret >= len)
		return -EINVAL;

	return 0;
}

static int sparx5_set_mac_address(struct net_device *dev, void *p)
{
	struct sparx5_port *port = netdev_priv(dev);
	struct sparx5 *sparx5 = port->sparx5;
	const struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	/* Remove current */
	sparx5_mact_forget(sparx5, dev->dev_addr,  port->pvid);

	/* Add new */
	sparx5_mact_learn(sparx5, sparx5_get_pgid_index(sparx5, PGID_CPU),
			  addr->sa_data, port->pvid);

	/* Record the address */
	eth_hw_addr_set(dev, addr->sa_data);

	return 0;
}

static int sparx5_get_port_parent_id(struct net_device *dev,
				     struct netdev_phys_item_id *ppid)
{
	struct sparx5_port *sparx5_port = netdev_priv(dev);
	struct sparx5 *sparx5 = sparx5_port->sparx5;

	ppid->id_len = sizeof(sparx5->base_mac);
	memcpy(&ppid->id, &sparx5->base_mac, ppid->id_len);

	return 0;
}

static int sparx5_port_ioctl(struct net_device *dev, struct ifreq *ifr,
			     int cmd)
{
	struct sparx5_port *sparx5_port = netdev_priv(dev);
	struct sparx5 *sparx5 = sparx5_port->sparx5;
	int ret = 0;

	if (phy_has_hwtstamp(dev->phydev)) {
		/* Don't return here immediately, because it might be required
		 * for the MAC to configure to copy all the PTP frames to CPU
		 */
		ret = phy_mii_ioctl(dev->phydev, ifr, cmd);
		if (ret)
			return ret;
	}

	if (sparx5->ptp) {
		switch (cmd) {
		case SIOCSHWTSTAMP:
			return sparx5_ptp_hwtstamp_set(sparx5_port, ifr);
		case SIOCGHWTSTAMP:
			return sparx5_ptp_hwtstamp_get(sparx5_port, ifr);
		}
	}

	return 0;
}

static const struct net_device_ops sparx5_port_netdev_ops = {
	.ndo_open               = sparx5_port_open,
	.ndo_stop               = sparx5_port_stop,
	.ndo_start_xmit         = sparx5_port_xmit_impl,
	.ndo_set_rx_mode        = sparx5_set_rx_mode,
	.ndo_get_phys_port_name = sparx5_port_get_phys_port_name,
	.ndo_set_mac_address    = sparx5_set_mac_address,
	.ndo_validate_addr      = eth_validate_addr,
	.ndo_get_stats64        = sparx5_get_stats64,
	.ndo_get_port_parent_id = sparx5_get_port_parent_id,
	.ndo_eth_ioctl          = sparx5_port_ioctl,
	.ndo_setup_tc           = sparx5_port_setup_tc,
};

bool sparx5_netdevice_check(const struct net_device *dev)
{
	return dev && (dev->netdev_ops == &sparx5_port_netdev_ops);
}

struct net_device *sparx5_create_netdev(struct sparx5 *sparx5, u32 portno)
{
	struct sparx5_port *spx5_port;
	struct net_device *ndev;

	ndev = devm_alloc_etherdev_mqs(sparx5->dev, sizeof(struct sparx5_port),
				       SPX5_PRIOS, 1);
	if (!ndev)
		return ERR_PTR(-ENOMEM);

	ndev->hw_features |= NETIF_F_HW_TC;
	ndev->features |= NETIF_F_HW_TC;

	SET_NETDEV_DEV(ndev, sparx5->dev);
	spx5_port = netdev_priv(ndev);
	spx5_port->ndev = ndev;
	spx5_port->sparx5 = sparx5;
	spx5_port->portno = portno;
	snprintf(ndev->name, IFNAMSIZ, "eth%d", portno);

	ndev->netdev_ops = &sparx5_port_netdev_ops;
	ndev->ethtool_ops = &sparx5_ethtool_ops;

	eth_hw_addr_gen(ndev, sparx5->base_mac, portno + 1);

	return ndev;
}

int sparx5_register_netdevs(struct sparx5 *sparx5)
{
	const struct sparx5_consts *consts = &sparx5->data->consts;
	int portno;
	int err;

	for (portno = 0; portno < consts->chip_ports; portno++)
		if (sparx5->ports[portno]) {
			err = register_netdev(sparx5->ports[portno]->ndev);
			if (err) {
				dev_err(sparx5->dev,
					"port: %02u: netdev registration failed\n",
					portno);
				return err;
			}
			sparx5_port_inj_timer_setup(sparx5->ports[portno]);
		}
	return 0;
}

void sparx5_destroy_netdevs(struct sparx5 *sparx5)
{
	const struct sparx5_consts *consts = &sparx5->data->consts;
	struct sparx5_port *port;
	int portno;

	for (portno = 0; portno < consts->chip_ports; portno++) {
		port = sparx5->ports[portno];
		if (port && port->phylink) {
			/* Disconnect the phy */
			rtnl_lock();
			sparx5_port_stop(port->ndev);
			phylink_disconnect_phy(port->phylink);
			rtnl_unlock();
			phylink_destroy(port->phylink);
			port->phylink = NULL;
		}
	}
}

void sparx5_unregister_netdevs(struct sparx5 *sparx5)
{
	const struct sparx5_consts *consts = &sparx5->data->consts;
	int portno;

	for (portno = 0; portno < consts->chip_ports; portno++)
		if (sparx5->ports[portno])
			unregister_netdev(sparx5->ports[portno]->ndev);
}
