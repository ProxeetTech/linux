// SPDX-License-Identifier: GPL-2.0+
/* Microchip Sparx5 Switch driver
 *
 * Copyright (c) 2021 Microchip Technology Inc. and its subsidiaries.
 */

#include <linux/module.h>
#include <linux/device.h>

#include "sparx5_main_regs.h"
#include "sparx5_main.h"

/* QSYS calendar information */
#define SPX5_PORTS_PER_CALREG          10  /* Ports mapped in a calendar register */
#define SPX5_CALBITS_PER_PORT          3   /* Bit per port in calendar register */

/* DSM calendar information */
#define SPX5_DSM_CAL_LEN               64
#define SPX5_DSM_CAL_EMPTY             0xFFFF
#define SPX5_DSM_CAL_BW_LOSS           553

#define SPX5_TAXI_PORT_MAX             70

#define SPEED_12500                    12500

/* Maps from taxis to port numbers */
static u32 sparx5_taxi_ports[SPX5_DSM_CAL_TAXIS][SPX5_DSM_CAL_MAX_DEVS_PER_TAXI] = {
	{57, 12,  0,  1,  2, 16, 17, 18, 19, 20, 21, 22, 23},
	{58, 13,  3,  4,  5, 24, 25, 26, 27, 28, 29, 30, 31},
	{59, 14,  6,  7,  8, 32, 33, 34, 35, 36, 37, 38, 39},
	{60, 15,  9, 10, 11, 40, 41, 42, 43, 44, 45, 46, 47},
	{61, 48, 49, 50, 99, 99, 99, 99, 99, 99, 99, 99, 99},
	{62, 51, 52, 53, 99, 99, 99, 99, 99, 99, 99, 99, 99},
	{56, 63, 54, 55, 99, 99, 99, 99, 99, 99, 99, 99, 99},
	{64, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99},
};

u32 *sparx5_get_taxi(int idx)
{
	return sparx5_taxi_ports[idx];
}

struct sparx5_calendar_data {
	u32 schedule[SPX5_DSM_CAL_LEN];
	u32 avg_dist[SPX5_DSM_CAL_MAX_DEVS_PER_TAXI];
	u32 taxi_ports[SPX5_DSM_CAL_MAX_DEVS_PER_TAXI];
	u32 taxi_speeds[SPX5_DSM_CAL_MAX_DEVS_PER_TAXI];
	u32 dev_slots[SPX5_DSM_CAL_MAX_DEVS_PER_TAXI];
	u32 new_slots[SPX5_DSM_CAL_LEN];
	u32 temp_sched[SPX5_DSM_CAL_LEN];
	u32 indices[SPX5_DSM_CAL_LEN];
	u32 short_list[SPX5_DSM_CAL_LEN];
	u32 long_list[SPX5_DSM_CAL_LEN];
};

static u32 sparx5_target_bandwidth(struct sparx5 *sparx5)
{
	switch (sparx5->target_ct) {
	case SPX5_TARGET_CT_7546:
	case SPX5_TARGET_CT_7546TSN:
		return 65000;
	case SPX5_TARGET_CT_7549:
	case SPX5_TARGET_CT_7549TSN:
		return 91000;
	case SPX5_TARGET_CT_7552:
	case SPX5_TARGET_CT_7552TSN:
		return 129000;
	case SPX5_TARGET_CT_7556:
	case SPX5_TARGET_CT_7556TSN:
		return 161000;
	case SPX5_TARGET_CT_7558:
	case SPX5_TARGET_CT_7558TSN:
		return 201000;
	case SPX5_TARGET_CT_LAN9694RED:
	case SPX5_TARGET_CT_LAN9694TSN:
	case SPX5_TARGET_CT_LAN9691VAO:
	case SPX5_TARGET_CT_LAN9694:
		return 68000;
	case SPX5_TARGET_CT_LAN9696RED:
	case SPX5_TARGET_CT_LAN9696TSN:
	case SPX5_TARGET_CT_LAN9692VAO:
	case SPX5_TARGET_CT_LAN9696:
		return 88000;
	case SPX5_TARGET_CT_LAN9698RED:
	case SPX5_TARGET_CT_LAN9698TSN:
	case SPX5_TARGET_CT_LAN9693VAO:
	case SPX5_TARGET_CT_LAN9698:
		return 106000;
	default:
		return 0;
	}
}

/* This is used in calendar configuration */
enum sparx5_cal_bw {
	SPX5_CAL_SPEED_NONE = 0,
	SPX5_CAL_SPEED_1G   = 1,
	SPX5_CAL_SPEED_2G5  = 2,
	SPX5_CAL_SPEED_5G   = 3,
	SPX5_CAL_SPEED_10G  = 4,
	SPX5_CAL_SPEED_25G  = 5,
	SPX5_CAL_SPEED_0G5  = 6,
	SPX5_CAL_SPEED_12G5 = 7
};

static u32 sparx5_clk_to_bandwidth(enum sparx5_core_clockfreq cclock)
{
	switch (cclock) {
	case SPX5_CORE_CLOCK_250MHZ: return 83000; /* 250000 / 3 */
	case SPX5_CORE_CLOCK_328MHZ: return 109375; /* 328000 / 3 */
	case SPX5_CORE_CLOCK_500MHZ: return 166000; /* 500000 / 3 */
	case SPX5_CORE_CLOCK_625MHZ: return  208000; /* 625000 / 3 */
	default: return 0;
	}
	return 0;
}

static u32 sparx5_cal_speed_to_value(enum sparx5_cal_bw speed)
{
	switch (speed) {
	case SPX5_CAL_SPEED_1G:   return 1000;
	case SPX5_CAL_SPEED_2G5:  return 2500;
	case SPX5_CAL_SPEED_5G:   return 5000;
	case SPX5_CAL_SPEED_10G:  return 10000;
	case SPX5_CAL_SPEED_25G:  return 25000;
	case SPX5_CAL_SPEED_0G5:  return 500;
	case SPX5_CAL_SPEED_12G5: return 12500;
	default: return 0;
	}
}

static u32 sparx5_bandwidth_to_calendar(u32 bw)
{
	switch (bw) {
	case SPEED_10:      return SPX5_CAL_SPEED_0G5;
	case SPEED_100:     return SPX5_CAL_SPEED_0G5;
	case SPEED_1000:    return SPX5_CAL_SPEED_1G;
	case SPEED_2500:    return SPX5_CAL_SPEED_2G5;
	case SPEED_5000:    return SPX5_CAL_SPEED_5G;
	case SPEED_10000:   return SPX5_CAL_SPEED_10G;
	case SPEED_12500:   return SPX5_CAL_SPEED_12G5;
	case SPEED_25000:   return SPX5_CAL_SPEED_25G;
	case SPEED_UNKNOWN: return SPX5_CAL_SPEED_1G;
	default:            return SPX5_CAL_SPEED_NONE;
	}
}

static enum sparx5_cal_bw sparx5_get_port_cal_speed(struct sparx5 *sparx5,
						    u32 portno)
{
	const struct sparx5_consts *consts = &sparx5->data->consts;
	struct sparx5_port *port;

	if (portno >= consts->chip_ports) {
		/* Internal ports */
		if (portno == sparx5_get_internal_port(sparx5, PORT_CPU_0) ||
		    portno == sparx5_get_internal_port(sparx5, PORT_CPU_1)) {
			/* Equals 1.25G */
			return SPX5_CAL_SPEED_2G5;
		} else if (portno == sparx5_get_internal_port(sparx5,
							      PORT_VD0)) {
			/* IPMC only idle BW */
			return SPX5_CAL_SPEED_NONE;
		} else if (portno == sparx5_get_internal_port(sparx5,
							      PORT_VD1)) {
			/* OAM only idle BW */
			return SPX5_CAL_SPEED_NONE;
		} else if (portno == sparx5_get_internal_port(sparx5,
							      PORT_VD2)) {
			/* IPinIP gets only idle BW */
			return SPX5_CAL_SPEED_NONE;
		}
		/* not in port map */
		return SPX5_CAL_SPEED_NONE;
	}
	/* Front ports - may be used */
	port = sparx5->ports[portno];
	if (!port)
		return SPX5_CAL_SPEED_NONE;
	return sparx5_bandwidth_to_calendar(port->conf.bandwidth);
}

/* Auto configure the QSYS calendar based on port configuration */
int sparx5_config_auto_calendar(struct sparx5 *sparx5)
{
	const struct sparx5_consts *consts = &sparx5->data->consts;
	u32 cal[7], value, idx, portno;
	u32 max_core_bw;
	u32 total_bw = 0, used_port_bw = 0;
	int err = 0;
	enum sparx5_cal_bw spd;

	memset(cal, 0, sizeof(cal));

	max_core_bw = sparx5_clk_to_bandwidth(sparx5->coreclock);
	if (max_core_bw == 0) {
		dev_err(sparx5->dev, "Core clock not supported");
		return -EINVAL;
	}

	/* Setup the calendar with the bandwidth to each port */
	for (portno = 0; portno < consts->chip_ports_all; portno++) {
		u64 reg, offset, this_bw;

		spd = sparx5_get_port_cal_speed(sparx5, portno);
		if (spd == SPX5_CAL_SPEED_NONE)
			continue;

		this_bw = sparx5_cal_speed_to_value(spd);
		if (portno < consts->chip_ports)
			used_port_bw += this_bw;
		else
			/* Internal ports are granted half the value */
			this_bw = this_bw / 2;
		total_bw += this_bw;
		reg = portno;
		offset = do_div(reg, SPX5_PORTS_PER_CALREG);
		cal[reg] |= spd << (offset * SPX5_CALBITS_PER_PORT);
	}

	if (used_port_bw > sparx5_target_bandwidth(sparx5)) {
		dev_err(sparx5->dev,
			"Port BW %u above target BW %u\n",
			used_port_bw, sparx5_target_bandwidth(sparx5));
		return -EINVAL;
	}

	if (total_bw > max_core_bw) {
		dev_err(sparx5->dev,
			"Total BW %u above switch core BW %u\n",
			total_bw, max_core_bw);
		return -EINVAL;
	}

	/* Halt the calendar while changing it */
	spx5_rmw(QSYS_CAL_CTRL_CAL_MODE_SET(10),
		 QSYS_CAL_CTRL_CAL_MODE,
		 sparx5, QSYS_CAL_CTRL);

	/* Assign port bandwidth to auto calendar */
	for (idx = 0; idx < consts->auto_cal_cnt; idx++)
		spx5_wr(cal[idx], sparx5, QSYS_CAL_AUTO(idx));

	/* Increase grant rate of all ports to account for
	 * core clock ppm deviations
	 */
	spx5_rmw(QSYS_CAL_CTRL_CAL_AUTO_GRANT_RATE_SET(671), /* 672->671 */
		 QSYS_CAL_CTRL_CAL_AUTO_GRANT_RATE,
		 sparx5,
		 QSYS_CAL_CTRL);

	/* Grant idle usage to VD 0-2 */
	for (idx = 2; idx < 5; idx++)
		spx5_wr(HSCH_OUTB_SHARE_ENA_OUTB_SHARE_ENA_SET(12),
			sparx5,
			HSCH_OUTB_SHARE_ENA(idx));

	/* Enable Auto mode */
	spx5_rmw(QSYS_CAL_CTRL_CAL_MODE_SET(8),
		 QSYS_CAL_CTRL_CAL_MODE,
		 sparx5, QSYS_CAL_CTRL);

	/* Verify successful calendar config */
	value = spx5_rd(sparx5, QSYS_CAL_CTRL);
	if (QSYS_CAL_CTRL_CAL_AUTO_ERROR_GET(value)) {
		dev_err(sparx5->dev, "QSYS calendar error\n");
		err = -EINVAL;
	}
	return err;
}

static u32 sparx5_dsm_exb_gcd(u32 a, u32 b)
{
	if (b == 0)
		return a;
	return sparx5_dsm_exb_gcd(b, a % b);
}

static u32 sparx5_dsm_cal_len(u32 *cal)
{
	u32 idx = 0, len = 0;

	while (idx < SPX5_DSM_CAL_LEN) {
		if (cal[idx] != SPX5_DSM_CAL_EMPTY)
			len++;
		idx++;
	}
	return len;
}

static u32 sparx5_dsm_cp_cal(u32 *sched)
{
	u32 idx = 0, tmp;

	while (idx < SPX5_DSM_CAL_LEN) {
		if (sched[idx] != SPX5_DSM_CAL_EMPTY) {
			tmp = sched[idx];
			sched[idx] = SPX5_DSM_CAL_EMPTY;
			return tmp;
		}
		idx++;
	}
	return SPX5_DSM_CAL_EMPTY;
}

static int sparx5_dsm_calendar_calc(struct sparx5 *sparx5, u32 taxi,
				    struct sparx5_calendar_data *data)
{
	const struct sparx5_consts *consts = &sparx5->data->consts;
	int devs_per_taxi = consts->dsm_cal_max_devs_per_taxi;
	const struct sparx5_ops *ops = &sparx5->data->ops;
	bool slow_mode;
	u32 gcd, idx, sum, min, factor;
	u32 num_of_slots, slot_spd, empty_slots;
	u32 taxi_bw, clk_period_ps;

	clk_period_ps = sparx5_clk_period(sparx5->coreclock);
	taxi_bw = 128 * 1000000 / clk_period_ps;
	slow_mode = !!(clk_period_ps > 2000);
	memcpy(data->taxi_ports, ops->get_taxi(taxi),
	       devs_per_taxi * sizeof(u32));

	for (idx = 0; idx < SPX5_DSM_CAL_LEN; idx++) {
		data->new_slots[idx] = SPX5_DSM_CAL_EMPTY;
		data->schedule[idx] = SPX5_DSM_CAL_EMPTY;
		data->temp_sched[idx] = SPX5_DSM_CAL_EMPTY;
	}
	/* Default empty calendar */
	data->schedule[0] = devs_per_taxi;

	/* Map ports to taxi positions */
	for (idx = 0; idx < devs_per_taxi; idx++) {
		u32 portno = data->taxi_ports[idx];

		if (portno < consts->chip_ports_all) {
			data->taxi_speeds[idx] = sparx5_cal_speed_to_value
				(sparx5_get_port_cal_speed(sparx5, portno));
		} else {
			data->taxi_speeds[idx] = 0;
		}
	}

	sum = 0;
	min = 25000;
	for (idx = 0; idx < ARRAY_SIZE(data->taxi_speeds); idx++) {
		u32 jdx;

		sum += data->taxi_speeds[idx];
		if (data->taxi_speeds[idx] && data->taxi_speeds[idx] < min)
			min = data->taxi_speeds[idx];
		gcd = min;
		for (jdx = 0; jdx < ARRAY_SIZE(data->taxi_speeds); jdx++)
			gcd = sparx5_dsm_exb_gcd(gcd, data->taxi_speeds[jdx]);
	}
	if (sum == 0) /* Empty calendar */
		return 0;
	/* Make room for overhead traffic */
	factor = 100 * 100 * 1000 / (100 * 100 - SPX5_DSM_CAL_BW_LOSS);

	if (sum * factor > (taxi_bw * 1000)) {
		dev_err(sparx5->dev,
			"Taxi %u, Requested BW %u above available BW %u\n",
			taxi, sum, taxi_bw);
		return -EINVAL;
	}
	for (idx = 0; idx < 4; idx++) {
		u32 raw_spd;

		if (idx == 0)
			raw_spd = gcd / 5;
		else if (idx == 1)
			raw_spd = gcd / 2;
		else if (idx == 2)
			raw_spd = gcd;
		else
			raw_spd = min;
		slot_spd = raw_spd * factor / 1000;
		num_of_slots = taxi_bw / slot_spd;
		if (num_of_slots <= 64)
			break;
	}

	num_of_slots = num_of_slots > 64 ? 64 : num_of_slots;
	slot_spd = taxi_bw / num_of_slots;

	sum = 0;
	for (idx = 0; idx < ARRAY_SIZE(data->taxi_speeds); idx++) {
		u32 spd = data->taxi_speeds[idx];
		u32 adjusted_speed = data->taxi_speeds[idx] * factor / 1000;

		if (adjusted_speed > 0) {
			data->avg_dist[idx] = (128 * 1000000 * 10) /
				(adjusted_speed * clk_period_ps);
		} else {
			data->avg_dist[idx] = -1;
		}
		data->dev_slots[idx] = ((spd * factor / slot_spd) + 999) / 1000;
		if (is_sparx5(sparx5))
			/* Improved and allowed on lan969x, i.e. this check can be skipped  */
			if (spd != 25000 && (spd != 10000 || !slow_mode)) {
				if (num_of_slots < (5 * data->dev_slots[idx])) {
					dev_err(sparx5->dev,
						"Taxi %u, speed %u, Low slot sep.\n",
						taxi, spd);
					return -EINVAL;
				}
			}
		sum += data->dev_slots[idx];
		if (sum > num_of_slots) {
			dev_err(sparx5->dev,
				"Taxi %u with overhead factor %u\n",
				taxi, factor);
			return -EINVAL;
		}
	}

	empty_slots = num_of_slots - sum;

	for (idx = 0; idx < empty_slots; idx++)
		data->schedule[idx] = devs_per_taxi;

	for (idx = 1; idx < num_of_slots; idx++) {
		u32 indices_len = 0;
		u32 slot, jdx, kdx, ts;
		s32 cnt;
		u32 num_of_old_slots, num_of_new_slots, tgt_score;

		for (slot = 0; slot < ARRAY_SIZE(data->dev_slots); slot++) {
			if (data->dev_slots[slot] == idx) {
				data->indices[indices_len] = slot;
				indices_len++;
			}
		}
		if (indices_len == 0)
			continue;
		kdx = 0;
		for (slot = 0; slot < idx; slot++) {
			for (jdx = 0; jdx < indices_len; jdx++, kdx++)
				data->new_slots[kdx] = data->indices[jdx];
		}

		for (slot = 0; slot < SPX5_DSM_CAL_LEN; slot++) {
			if (data->schedule[slot] == SPX5_DSM_CAL_EMPTY)
				break;
		}

		num_of_old_slots =  slot;
		num_of_new_slots =  kdx;
		cnt = 0;
		ts = 0;

		if (num_of_new_slots > num_of_old_slots) {
			memcpy(data->short_list, data->schedule,
			       sizeof(data->short_list));
			memcpy(data->long_list, data->new_slots,
			       sizeof(data->long_list));
			tgt_score = 100000 * num_of_old_slots /
				num_of_new_slots;
		} else {
			memcpy(data->short_list, data->new_slots,
			       sizeof(data->short_list));
			memcpy(data->long_list, data->schedule,
			       sizeof(data->long_list));
			tgt_score = 100000 * num_of_new_slots /
				num_of_old_slots;
		}

		while (sparx5_dsm_cal_len(data->short_list) > 0 ||
		       sparx5_dsm_cal_len(data->long_list) > 0) {
			u32 act = 0;

			if (sparx5_dsm_cal_len(data->short_list) > 0) {
				data->temp_sched[ts] =
					sparx5_dsm_cp_cal(data->short_list);
				ts++;
				cnt += 100000;
				act = 1;
			}
			while (sparx5_dsm_cal_len(data->long_list) > 0 &&
			       cnt > 0) {
				data->temp_sched[ts] =
					sparx5_dsm_cp_cal(data->long_list);
				ts++;
				cnt -= tgt_score;
				act = 1;
			}
			if (act == 0) {
				dev_err(sparx5->dev,
					"Error in DSM calendar calculation\n");
				return -EINVAL;
			}
		}

		for (slot = 0; slot < SPX5_DSM_CAL_LEN; slot++) {
			if (data->temp_sched[slot] == SPX5_DSM_CAL_EMPTY)
				break;
		}
		for (slot = 0; slot < SPX5_DSM_CAL_LEN; slot++) {
			data->schedule[slot] = data->temp_sched[slot];
			data->temp_sched[slot] = SPX5_DSM_CAL_EMPTY;
			data->new_slots[slot] = SPX5_DSM_CAL_EMPTY;
		}
	}
	return 0;
}

static int sparx5_dsm_calendar_check(struct sparx5 *sparx5,
				     struct sparx5_calendar_data *data)
{
	const struct sparx5_consts *consts = &sparx5->data->consts;
	int devs_per_taxi = consts->dsm_cal_max_devs_per_taxi;
	u32 num_of_slots, idx, port;
	int cnt, max_dist;
	u32 slot_indices[SPX5_DSM_CAL_LEN], distances[SPX5_DSM_CAL_LEN];
	u32 cal_length = sparx5_dsm_cal_len(data->schedule);

	for (port = 0; port < devs_per_taxi; port++) {
		num_of_slots = 0;
		max_dist = data->avg_dist[port];
		for (idx = 0; idx < SPX5_DSM_CAL_LEN; idx++) {
			slot_indices[idx] = SPX5_DSM_CAL_EMPTY;
			distances[idx] = SPX5_DSM_CAL_EMPTY;
		}

		for (idx = 0; idx < cal_length; idx++) {
			if (data->schedule[idx] == port) {
				slot_indices[num_of_slots] = idx;
				num_of_slots++;
			}
		}

		slot_indices[num_of_slots] = slot_indices[0] + cal_length;

		for (idx = 0; idx < num_of_slots; idx++) {
			distances[idx] = (slot_indices[idx + 1] -
					  slot_indices[idx]) * 10;
		}

		for (idx = 0; idx < num_of_slots; idx++) {
			u32 jdx, kdx;

			cnt = distances[idx] - max_dist;
			if (cnt < 0)
				cnt = -cnt;
			kdx = 0;
			for (jdx = (idx + 1) % num_of_slots;
			     jdx != idx;
			     jdx = (jdx + 1) % num_of_slots, kdx++) {
				cnt =  cnt + distances[jdx] - max_dist;
				if (cnt < 0)
					cnt = -cnt;
				if (cnt > max_dist)
					goto check_err;
			}
		}
	}
	return 0;
check_err:
	dev_err(sparx5->dev,
		"Port %u: distance %u above limit %d\n",
		port, cnt, max_dist);
	return -EINVAL;
}

static int sparx5_dsm_calendar_update(struct sparx5 *sparx5, u32 taxi,
				      struct sparx5_calendar_data *data)
{
	u32 idx;
	u32 cal_len = sparx5_dsm_cal_len(data->schedule), len, val, act;

	if (!is_sparx5(sparx5)) {
		val = spx5_rd(sparx5, DSM_TAXI_CAL_CFG(taxi));
		act = DSM_TAXI_CAL_CFG_CAL_SEL_STAT_GET(val);
		spx5_rmw(DSM_TAXI_CAL_CFG_CAL_PGM_SEL_SET(!act),
			 DSM_TAXI_CAL_CFG_CAL_PGM_SEL,
			 sparx5, DSM_TAXI_CAL_CFG(taxi));
	}

	spx5_wr(DSM_TAXI_CAL_CFG_CAL_PGM_ENA_SET(1),
		sparx5,
		DSM_TAXI_CAL_CFG(taxi));
	for (idx = 0; idx < cal_len; idx++) {
		spx5_rmw(DSM_TAXI_CAL_CFG_CAL_IDX_SET(idx),
			 DSM_TAXI_CAL_CFG_CAL_IDX,
			 sparx5,
			 DSM_TAXI_CAL_CFG(taxi));
		spx5_rmw(DSM_TAXI_CAL_CFG_CAL_PGM_VAL_SET(data->schedule[idx]),
			 DSM_TAXI_CAL_CFG_CAL_PGM_VAL,
			 sparx5,
			 DSM_TAXI_CAL_CFG(taxi));
	}
	spx5_wr(DSM_TAXI_CAL_CFG_CAL_PGM_ENA_SET(0),
		sparx5,
		DSM_TAXI_CAL_CFG(taxi));
	len = DSM_TAXI_CAL_CFG_CAL_CUR_LEN_GET(spx5_rd(sparx5,
						       DSM_TAXI_CAL_CFG(taxi)));
	if (len != cal_len - 1)
		goto update_err;

	if (!is_sparx5(sparx5)) {
		spx5_rmw(DSM_TAXI_CAL_CFG_CAL_SWITCH_SET(1),
			 DSM_TAXI_CAL_CFG_CAL_SWITCH,
			 sparx5, DSM_TAXI_CAL_CFG(taxi));
	}

	return 0;
update_err:
	dev_err(sparx5->dev, "Incorrect calendar length: %u\n", len);
	return -EINVAL;
}

/* Configure the DSM calendar based on port configuration */
int sparx5_config_dsm_calendar(struct sparx5 *sparx5)
{
	const struct sparx5_consts *consts = &sparx5->data->consts;
	int taxi, cal_taxis = consts->dsm_cal_taxis;
	struct sparx5_calendar_data *data;
	int err = 0;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	for (taxi = 0; taxi < cal_taxis; ++taxi) {
		err = sparx5_dsm_calendar_calc(sparx5, taxi, data);
		if (err) {
			dev_err(sparx5->dev, "DSM calendar calculation failed\n");
			goto cal_out;
		}
		err = sparx5_dsm_calendar_check(sparx5, data);
		if (err) {
			dev_err(sparx5->dev, "DSM calendar check failed\n");
			goto cal_out;
		}
		err = sparx5_dsm_calendar_update(sparx5, taxi, data);
		if (err) {
			dev_err(sparx5->dev, "DSM calendar update failed\n");
			goto cal_out;
		}
	}
cal_out:
	kfree(data);
	return err;
}

void sparx5_calendar_fix(struct sparx5 *sparx5)
{
	spx5_wr(0x00200140, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00200141, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00200001, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00200001, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00208001, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00208215, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00210355, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00210555, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00218555, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00218755, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00220755, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00220955, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00228955, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00228b43, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00230a23, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00230c35, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00238d55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00238f55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00240f55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00241155, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00249155, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00249355, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00251355, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00251545, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00259445, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00259655, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00261755, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00261955, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00269955, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00269b55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00271b55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00271d55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00279d55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00279f47, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00281e67, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00282075, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x0028a155, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x0028a355, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00292355, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00292555, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x0029a555, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x0029a755, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002a2755, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002a2949, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002aa889, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002aaa95, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002b2b55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002b2d55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002bad55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002baf55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002c2f55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002c3155, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002cb155, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002cb34b, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002d32ab, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002d34b5, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002db555, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002db755, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002e3755, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002e3955, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002eb955, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002ebb55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002f3b55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002f3d4d, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002fbccd, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x002fbed5, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00303f55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00304155, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x0030c155, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x0030c355, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00314355, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00314555, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x0031c555, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x0031c74f, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x003246ef, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x003248f5, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x0032c955, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x0032cb55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00334b55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00334d55, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00334d54, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00734d54, sparx5, DSM_TAXI_CAL_CFG(0));
	spx5_wr(0x00200140, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00200141, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00200001, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00200001, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00208001, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00208215, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00210355, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00210555, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00218555, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00218755, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00220755, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00220955, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00228955, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00228b43, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00230a23, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00230c35, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00238d55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00238f55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00240f55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00241155, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00249155, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00249355, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00251355, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00251545, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00259445, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00259655, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00261755, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00261955, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00269955, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00269b55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00271b55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00271d55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00279d55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00279f47, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00281e67, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00282075, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x0028a155, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x0028a355, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00292355, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00292555, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x0029a555, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x0029a755, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002a2755, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002a2949, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002aa889, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002aaa95, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002b2b55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002b2d55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002bad55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002baf55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002c2f55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002c3155, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002cb155, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002cb34b, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002d32ab, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002d34b5, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002db555, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002db755, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002e3755, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002e3955, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002eb955, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002ebb55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002f3b55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002f3d4d, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002fbccd, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x002fbed5, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00303f55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00304155, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x0030c155, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x0030c355, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00314355, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00314555, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x0031c555, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x0031c74f, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x003246ef, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x003248f5, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x0032c955, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x0032cb55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00334b55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00334d55, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00334d54, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00734d54, sparx5, DSM_TAXI_CAL_CFG(1));
	spx5_wr(0x00200140, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00200141, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00200001, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00200001, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00208001, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00208215, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00210355, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00210555, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00218555, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00218755, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00220755, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00220955, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00228955, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00228b43, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00230a23, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00230c35, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00238d55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00238f55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00240f55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00241155, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00249155, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00249355, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00251355, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00251545, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00259445, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00259655, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00261755, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00261955, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00269955, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00269b55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00271b55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00271d55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00279d55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00279f47, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00281e67, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00282075, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x0028a155, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x0028a355, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00292355, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00292555, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x0029a555, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x0029a755, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002a2755, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002a2949, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002aa889, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002aaa95, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002b2b55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002b2d55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002bad55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002baf55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002c2f55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002c3155, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002cb155, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002cb34b, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002d32ab, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002d34b5, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002db555, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002db755, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002e3755, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002e3955, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002eb955, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002ebb55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002f3b55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002f3d4d, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002fbccd, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x002fbed5, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00303f55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00304155, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x0030c155, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x0030c355, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00314355, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00314555, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x0031c555, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x0031c74f, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x003246ef, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x003248f5, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x0032c955, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x0032cb55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00334b55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00334d55, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00334d54, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00734d54, sparx5, DSM_TAXI_CAL_CFG(2));
	spx5_wr(0x00200140, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00200141, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00200001, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00200015, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00208155, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00208341, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00210201, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00210403, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00218423, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00218635, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00220755, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00220941, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00228801, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00228a15, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00230b55, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00230d43, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00238c23, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00238e35, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00240f55, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00241141, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00249001, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00249215, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00251355, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00251543, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00259423, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00259635, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00261755, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00261941, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00269801, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00269a15, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00271b55, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00271d43, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00279c23, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00279e35, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00281f55, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00282141, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x0028a001, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x0028a215, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00292355, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00292543, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00292422, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00692422, sparx5, DSM_TAXI_CAL_CFG(3));
	spx5_wr(0x00200140, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00200141, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00200001, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00200015, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00208155, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00208341, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00210201, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00210403, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00218423, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00218635, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00220755, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00220941, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00228801, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00228a15, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00230b55, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00230d43, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00238c23, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00238e35, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00240f55, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00241141, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00249001, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00249215, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00251355, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00251543, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00259423, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00259635, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00261755, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00261941, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00269801, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00269a15, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00271b55, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00271d43, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00279c23, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00279e35, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00281f55, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00282141, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x0028a001, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x0028a215, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00292355, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00292543, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00292422, sparx5, DSM_TAXI_CAL_CFG(4));
	spx5_wr(0x00692422, sparx5, DSM_TAXI_CAL_CFG(4));

	spx5_rmw(PTP_PTP_DOM_CFG_PTP_ENA_SET(0), PTP_PTP_DOM_CFG_PTP_ENA,
		 sparx5, PTP_PTP_DOM_CFG);

	spx5_wr(0x18624dd2, sparx5, PTP_CLK_PER_CFG(0, 1));

	spx5_rmw(PTP_PTP_DOM_CFG_PTP_ENA_SET(1), PTP_PTP_DOM_CFG_PTP_ENA,
		 sparx5, PTP_PTP_DOM_CFG);
}
