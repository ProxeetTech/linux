/* SPDX-License-Identifier: GPL-2.0+ */
/* Microchip Sparx5 Switch driver
 *
 * Copyright (c) 2021 Microchip Technology Inc. and its subsidiaries.
 */

#ifndef __SPARX5_PORT_H__
#define __SPARX5_PORT_H__

#include "sparx5_main.h"

static inline bool sparx5_port_is_2g5(int portno)
{
	return portno >= 16 && portno <= 47;
}

static inline bool sparx5_port_is_5g(int portno)
{
	return portno <= 11 || portno == 64;
}

static inline bool sparx5_port_is_10g(int portno)
{
	return (portno >= 12 && portno <= 15) || (portno >= 48 && portno <= 55);
}

static inline bool sparx5_port_is_25g(int portno)
{
	return portno >= 56 && portno <= 63;
}

static inline bool sparx5_port_is_rgmii(int portno)
{
	return 0;
}

static inline u32 sparx5_to_high_dev(struct sparx5 *sparx5, int port)
{
	const struct sparx5_ops *ops = &sparx5->data->ops;

	if (ops->port_is_5g(port))
		return TARGET_DEV5G;
	if (ops->port_is_10g(port))
		return TARGET_DEV10G;
	return TARGET_DEV25G;
}

static inline u32 sparx5_to_pcs_dev(struct sparx5 *sparx5, int port)
{
	const struct sparx5_ops *ops = &sparx5->data->ops;

	if (ops->port_is_5g(port))
		return TARGET_PCS5G_BR;
	if (ops->port_is_10g(port))
		return TARGET_PCS10G_BR;
	return TARGET_PCS25G_BR;
}

static inline u32 sparx5_port_dev_index(struct sparx5 *sparx5, int port)
{
	const struct sparx5_ops *ops = &sparx5->data->ops;

	return ops->port_get_dev_index(sparx5, port);
}

int sparx5_port_init(struct sparx5 *sparx5,
		     struct sparx5_port *spx5_port,
		     struct sparx5_port_config *conf);

int sparx5_port_config(struct sparx5 *sparx5,
		       struct sparx5_port *spx5_port,
		       struct sparx5_port_config *conf);

int sparx5_port_pcs_set(struct sparx5 *sparx5,
			struct sparx5_port *port,
			struct sparx5_port_config *conf);

int sparx5_serdes_set(struct sparx5 *sparx5,
		      struct sparx5_port *spx5_port,
		      struct sparx5_port_config *conf);

struct sparx5_port_status {
	bool link;
	bool link_down;
	int  speed;
	bool an_complete;
	int  duplex;
	int  pause;
};

int sparx5_get_port_status(struct sparx5 *sparx5,
			   struct sparx5_port *port,
			   struct sparx5_port_status *status);

void sparx5_port_enable(struct sparx5_port *port, bool enable);
int sparx5_port_fwd_urg(struct sparx5 *sparx5, u32 speed);
u32 sparx5_port_dev_mapping(struct sparx5 *sparx5, int port);
int sparx5_get_internal_port(struct sparx5 *sparx5, int port);

/* Macros to read/write to both 2G5 and 5G/10G/25G device  */
#define SPX5_DEV_RD(value, port, name)									\
	{																	\
		u32 pix = sparx5_port_dev_index(port->sparx5, port->portno);			\
		u32 dev = sparx5_to_high_dev(port->sparx5, port->portno);			\
		void __iomem *devinst;											\
		if (sparx5_is_baser(port->conf.portmode)) {						\
			devinst = spx5_inst_get(port->sparx5, dev, pix);			\
			value = spx5_inst_rd(devinst, DEV10G_##name(0));			\
		} else {														\
			value = spx5_rd(port->sparx5, DEV2G5_##name(port->portno)); \
		}																\
	}

#define SPX5_DEV_WR(value, port, name)									\
	{																	\
		u32 pix = sparx5_port_dev_index(port->sparx5, port->portno);			\
		u32 dev = sparx5_to_high_dev(port->sparx5, port->portno);			\
		void __iomem *devinst;											\
		spx5_wr(value, port->sparx5, DEV2G5_##name(port->portno));		\
		if (sparx5_is_baser(port->conf.portmode)) {						\
			devinst = spx5_inst_get(port->sparx5, dev, pix);			\
			spx5_inst_wr(value, devinst, DEV10G_##name(0));				\
		}																\
	}

#define SPX5_DEV_RMW(value, mask, port, name)							\
	{																	\
		u32 pix = sparx5_port_dev_index(port->sparx5, port->portno);			\
		u32 dev = sparx5_to_high_dev(port->sparx5, port->portno);			\
		void __iomem *devinst;											\
		spx5_rmw(value, mask, port->sparx5, DEV2G5_##name(port->portno));\
		if (sparx5_is_baser(port->conf.portmode)) {						\
			devinst = spx5_inst_get(port->sparx5, dev, pix);			\
			spx5_inst_rmw(value, mask, devinst, DEV10G_##name(0));		\
		}																\
	}


#endif	/* __SPARX5_PORT_H__ */
