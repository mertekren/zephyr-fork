/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(eth_ad3306, CONFIG_ETHERNET_LOG_LEVEL);

#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <ethernet/eth_stats.h>

#include "eth_ad3306_priv.h"

#define DT_DRV_COMPAT adi_ad3306

/**
 * Utility function to print current value of a register.
 */
void ad3306_log_reg(const struct device *dev, const uint32_t reg, const char *reg_name)
{
	int rc;
	struct ad3306_data *ctx = dev->data;
	uint32_t val;

	rc = oa_tc6_reg_read(ctx->tc6, reg, &val);
	if (rc < 0) {
		LOG_ERR("Failed to read register %s", reg_name);
		return;
	}
	LOG_DBG("[%s]: 0x%X", reg_name, val);
}

/**
 * Test SPI comms between host and MAC-PHY by reading
 * and verifying the value of the PHYID register.
 */
static int ad3306_spi_test(const struct device *dev)
{
	int rc;
	struct ad3306_data *ctx = dev->data;
	uint32_t val;

	rc = oa_tc6_reg_read(ctx->tc6, OA_PHYID, &val);
	if (rc < 0) {
		return rc;
	}

	if (val != AD3306_PHYID_RST_VAL) {
		rc = -ENODEV;
	}

	return rc;
}

int ad3306_data_spi_transfer(const struct device *dev, uint8_t *buf_rx, uint8_t *buf_tx,
			     uint32_t len)
{
	int rc;
	const struct ad3306_config *cfg = dev->config;

	struct spi_buf tx_buf[1];
	struct spi_buf rx_buf[1];
	struct spi_buf_set tx;
	struct spi_buf_set rx;

	tx_buf[0].buf = buf_tx;
	tx_buf[0].len = len;
	rx_buf[0].buf = buf_rx;
	rx_buf[0].len = len;

	rx.buffers = rx_buf;
	rx.count = 1;
	tx.buffers = tx_buf;
	tx.count = 1;

	rc = spi_transceive_dt(&cfg->spi, &tx, &rx);
	if (rc < 0) {
		LOG_ERR("SPI transfer failed! [%d]", rc);
	}

	return rc;
}

/**
 * Perform software reset the MAC-PHY.
 */
static int ad3306_sw_reset(const struct device *dev)
{
	int rc;
	struct ad3306_data *ctx = dev->data;
	uint32_t val;

	rc = oa_tc6_reg_write(ctx->tc6, OA_RESET, OA_RESET_SWRESET);
	if (rc < 0) {
		return rc;
	}

	/* Await completion of reset */
	k_sleep(K_USEC(AD3306_RESET_CYCLE_TIME_US));

	/* Read reset status register */
	rc = oa_tc6_reg_read(ctx->tc6, AD3306_MAC_RST_STATUS, &val);
	if (rc < 0) {
		return rc;
	}

	/* Verify that MAC Crystal and MAC Oscillator clocks are ready */
	if ((val & AD3306_MAC_RST_STATUS_MASK) != AD3306_RST_COMPLETE) {
		return -ETIMEDOUT;
	}

	/* Clear RESETC in STATUS0 */
	return oa_tc6_reg_write(ctx->tc6, OA_STATUS0, OA_STATUS0_RESETC);
}

/**
 *  Write a MAC address to the filter table
 *  to enable reception of frames with that MAC address.
 */
static int ad3306_mac_filter_write(const struct device *dev, const struct net_eth_addr *mac_address,
				   int slot)
{
	int rc;
	struct ad3306_data *ctx = dev->data;

	if (slot > AD3306_MAC_FILT_TABLE_MAX_SLOT) {
		LOG_ERR("Failed to add address to filter table - max capacity (16) reached");
		return -ENOSPC;
	}

	/* Write to upper register must precede write to lower register */
	rc = oa_tc6_reg_write(ctx->tc6, (AD3306_MAC_ADDR_FILT_UPR + slot),
			      sys_get_be16(&mac_address->addr[0]) |
				      AD3306_MAC_ADDR_FILT_APPLY2PORT1 |
				      AD3306_MAC_ADDR_FILT_TO_HOST);
	if (rc < 0) {
		return rc;
	}

	return oa_tc6_reg_write(ctx->tc6, (AD3306_MAC_ADDR_FILT_LWR + slot),
				sys_get_be32(&mac_address->addr[2]));
}

/**
 * Search MAC filter table for an existing MAC address entry.
 */
static int ad3306_mac_filter_find(const struct device *dev, const struct net_eth_addr *mac_address)
{
	int rc;
	struct ad3306_data *ctx = dev->data;
	uint32_t val;
	int slot = AD3306_MAC_FILT_TABLE_SLOT_SIZE;

	while (slot <= AD3306_MAC_FILT_TABLE_MAX_SLOT) {
		rc = oa_tc6_reg_read(ctx->tc6, (AD3306_MAC_ADDR_FILT_UPR + slot), &val);
		if (rc < 0) {
			return rc;
		}

		if ((val & GENMASK(15, 0)) == sys_get_be16(&mac_address->addr[0])) {
			rc = oa_tc6_reg_read(ctx->tc6, (AD3306_MAC_ADDR_FILT_LWR + slot), &val);
			if (rc < 0) {
				return rc;
			}

			if (val == sys_get_be32(&mac_address->addr[2])) {
				return slot;
			}
		}

		slot += AD3306_MAC_FILT_TABLE_SLOT_SIZE;
	}

	return -ENOENT;
}

/**
 * Find an empty slot and write MAC address to MAC filter table.
 */
static int ad3306_mac_filter_set(const struct device *dev, const struct net_eth_addr *mac_address)
{
	int rc;
	struct ad3306_data *ctx = dev->data;
	uint32_t val;
	int slot = AD3306_MAC_FILT_TABLE_BASE_SLOT;

	rc = ad3306_mac_filter_find(dev, mac_address);
	if (rc >= 0) {
		LOG_WRN("MAC address already in filter table!");
		return 0;
	}
	if (rc != -ENOENT) {
		return rc;
	}

	while (slot <= AD3306_MAC_FILT_TABLE_MAX_SLOT) {
		rc = oa_tc6_reg_read(ctx->tc6, (AD3306_MAC_ADDR_FILT_UPR + slot), &val);
		if (rc < 0) {
			return rc;
		}

		/* Empty slot */
		if (val == 0) {
			return ad3306_mac_filter_write(dev, mac_address, slot);
		}

		slot += AD3306_MAC_FILT_TABLE_SLOT_SIZE;
	}

	LOG_WRN("No free slots in MAC filter table!");
	return -ENOSPC;
}

/**
 * Clear entry from MAC filter table.
 */
static int ad3306_mac_filter_clear(const struct device *dev, const struct net_eth_addr *mac_address)
{
	int slot, rc;
	struct ad3306_data *ctx = dev->data;

	slot = ad3306_mac_filter_find(dev, mac_address);
	if (slot < 0) {
		LOG_ERR("Could not find entry in MAC filter table!");
		return slot;
	}

	/* Write to upper register must precede write to lower register */
	rc = oa_tc6_reg_write(ctx->tc6, (AD3306_MAC_ADDR_FILT_UPR + slot), 0x0);
	if (rc < 0) {
		return rc;
	}

	return oa_tc6_reg_write(ctx->tc6, (AD3306_MAC_ADDR_FILT_LWR + slot), 0x0);
}

/**
 * Enable reception of unicast frames.
 */
static int ad3306_filter_unicast(const struct device *dev)
{
	struct ad3306_data *ctx = dev->data;
	struct net_eth_addr unicast_addr;

	memcpy(unicast_addr.addr, ctx->mac_address, NET_ETH_ADDR_LEN);

	return ad3306_mac_filter_write(dev, &unicast_addr, AD3306_MAC_FILT_TABLE_UNICAST_SLOT);
}

/**
 * Enable reception of multicast frames.
 */
static int ad3306_filter_multicast(const struct device *dev)
{
	int rc;
	struct ad3306_data *ctx = dev->data;
	struct net_eth_addr multicast_addr = {.addr = {1U, 0, 0, 0, 0, 0}};
	uint8_t mask[NET_ETH_ADDR_LEN] = {1U, 0, 0, 0, 0, 0};

	rc = ad3306_mac_filter_write(dev, &multicast_addr, AD3306_MAC_FILT_TABLE_MULTICAST_SLOT);
	if (rc < 0) {
		return rc;
	}

	rc = oa_tc6_reg_write(ctx->tc6,
			      (AD3306_MAC_ADDR_MASK_UPR + AD3306_MAC_FILT_TABLE_MULTICAST_SLOT),
			      sys_get_be16(&mask[0]));
	if (rc < 0) {
		return rc;
	}

	return oa_tc6_reg_write(ctx->tc6,
				(AD3306_MAC_ADDR_MASK_LWR + AD3306_MAC_FILT_TABLE_MULTICAST_SLOT),
				sys_get_be32(&mask[2]));
}

/**
 * Set up default MAC address filter table entries.
 */
static int ad3306_default_filter_config(const struct device *dev)
{
	int rc;
	struct net_eth_addr broadcast_addr = {.addr = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}};

	/* Enable reception of broadcast frames */
	rc = ad3306_mac_filter_write(dev, &broadcast_addr, AD3306_MAC_FILT_TABLE_BROADCAST_SLOT);
	if (rc < 0) {
		return rc;
	}

	/* Enable reception of unicast frames */
	rc = ad3306_filter_unicast(dev);
	if (rc < 0) {
		return rc;
	}

	/* Enable reception of multicast frames */
	return ad3306_filter_multicast(dev);
}

/**
 * Congifure and enable PLCA.
 */
static int ad3306_plca_config(const struct device *dev)
{
	int rc;
	struct ad3306_data *ctx = dev->data;
	struct ad3306_plca_config *plca = ctx->plca;

	if (!plca->enable) {
		/* Disable PLCA */
		rc = oa_tc6_reg_write(ctx->tc6, OA_PHY_OA_PLCA_CTRL0, 0x0);
		if (rc == 0) {
			LOG_INF("PLCA: Disabled");
		}
		return rc;
	}

	/* Node ID and Node Count */
	if (plca->node_count < 1) {
		LOG_ERR("PLCA Node Count must be > 0. Restoring default value (8).");
		plca->node_count = 0x8;
	}
	rc = oa_tc6_reg_write(ctx->tc6, OA_PHY_OA_PLCA_CTRL1,
			      ((uint32_t)plca->node_count << OA_PHY_OA_PLCA_CTRL1_NODECOUNT) |
				      plca->node_id);
	if (rc < 0) {
		return rc;
	}

	/* TO Timer */
	rc = oa_tc6_reg_write(ctx->tc6, OA_PHY_OA_PLCA_TOTMR, (uint32_t)plca->to_timer);
	if (rc < 0) {
		return rc;
	}

	/* Burst Count and Timer */
	rc = oa_tc6_reg_write(
		ctx->tc6, OA_PHY_OA_PLCA_BURST,
		((uint32_t)plca->max_burst_count << OA_PHY_OA_PLCA_BURST_MAXBURSTCNT) |
			plca->burst_timer);
	if (rc < 0) {
		return rc;
	}

	/* Enable PLCA */
	rc = oa_tc6_reg_write(ctx->tc6, OA_PHY_OA_PLCA_CTRL0, OA_PHY_OA_PLCA_CTRL0_PLCAEN);
	if (rc < 0) {
		return rc;
	}

	LOG_INF("PLCA: Enabled - Node ID: %d, Node Count: %d, TO: %d", plca->node_id,
		plca->node_count, plca->to_timer);

	return rc;
}

/**
 * Enable promiscuous mode.
 * Frames with an unknown destination MAC address are forwarded to SPI host.
 */
static int ad3306_promiscuous_mode_en(const struct device *dev)
{
	int rc;
	struct ad3306_data *ctx = dev->data;
	uint32_t val;

	rc = oa_tc6_reg_read(ctx->tc6, OA_CONFIG2, &val);
	if (rc < 0) {
		return rc;
	}

	val |= OA_CONFIG2_FWD_UNK2HOST;

	return oa_tc6_reg_write(ctx->tc6, OA_CONFIG2, val);
}

/**
 * Indicate the MAC/PHY is configured by setting
 * the CONFIG0.SYNC bit.
 */
static int ad3306_set_sync(const struct device *dev)
{
	int rc;
	struct ad3306_data *ctx = dev->data;
	uint32_t val;

	rc = oa_tc6_reg_read(ctx->tc6, OA_CONFIG0, &val);
	if (rc < 0) {
		return rc;
	}

	val |= OA_CONFIG0_SYNC;

	return oa_tc6_reg_write(ctx->tc6, OA_CONFIG0, val);
}

static void ad3306_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct ad3306_data *ctx = dev->data;

	net_if_set_link_addr(iface, ctx->mac_address, sizeof(ctx->mac_address), NET_LINK_ETHERNET);

	if (!ctx->iface) {
		ctx->iface = iface;
	}

	ethernet_init(iface);

	/* T1S PHY link is considered always up */
	net_eth_carrier_on(iface);
}

static enum ethernet_hw_caps ad3306_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);
	return (ETHERNET_LINK_10BASE_T | ETHERNET_PROMISC_MODE | ETHERNET_HW_FILTERING |
		ETHERNET_PRIORITY_QUEUES);
}

static int ad3306_set_config(const struct device *dev, enum ethernet_config_type type,
			     const struct ethernet_config *config)
{
	int rc = -ENOTSUP;
	struct ad3306_data *ctx = dev->data;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		rc = ad3306_promiscuous_mode_en(dev);
		if (rc < 0) {
			LOG_ERR("Failed to enable promiscuous mode [%d]", rc);
		}
		break;

	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		/* Set MAC of Zephyr network interface */
		memcpy(ctx->mac_address, config->mac_address.addr, sizeof(ctx->mac_address));

		rc = net_if_set_link_addr(ctx->iface, ctx->mac_address, sizeof(ctx->mac_address),
					  NET_LINK_ETHERNET);

		/* Update unicast address in MAC filter table */
		rc = ad3306_filter_unicast(dev);
		if (rc < 0) {
			LOG_ERR("Failed to update unicast filter [%d]", rc);
		}

		LOG_INF("MAC address set to: [%02X:%02X:%02X:%02X:%02X:%02X]", ctx->mac_address[0],
			ctx->mac_address[1], ctx->mac_address[2], ctx->mac_address[3],
			ctx->mac_address[4], ctx->mac_address[5]);
		break;

	case ETHERNET_CONFIG_TYPE_T1S_PARAM:
		if (config->t1s_param.type != ETHERNET_T1S_PARAM_TYPE_PLCA_CONFIG) {
			break;
		}

		ctx->plca->enable = config->t1s_param.plca.enable;
		ctx->plca->node_id = config->t1s_param.plca.node_id;
		ctx->plca->node_count = config->t1s_param.plca.node_count;
		ctx->plca->max_burst_count = config->t1s_param.plca.burst_count;
		ctx->plca->burst_timer = config->t1s_param.plca.burst_timer;
		ctx->plca->to_timer = config->t1s_param.plca.to_timer;

		rc = ad3306_plca_config(dev);
		if (rc < 0) {
			LOG_ERR("Failed to configure PLCA [%d]", rc);
		}
		break;

	case ETHERNET_CONFIG_TYPE_FILTER:
		if (config->filter.type != ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS) {
			LOG_ERR("MAC address filtering supports destination addresses only!");
			break;
		}

		if (config->filter.set) {
			rc = ad3306_mac_filter_set(dev, &config->filter.mac_address);
			if (rc < 0) {
				LOG_ERR("Failed to add MAC addr to filter table! [%d]", rc);
			}
		} else {
			rc = ad3306_mac_filter_clear(dev, &config->filter.mac_address);
			if (rc < 0) {
				LOG_ERR("Failed to remove MAC addr from filter table! [%d]", rc);
			}
		}
		break;
	default:
		break;
	}

	return rc;
}

int ad3306_get_config(const struct device *dev, enum ethernet_config_type type,
		      struct ethernet_config *config)
{
	int rc = -ENOTSUP;
	struct ad3306_data *ctx = dev->data;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		uint32_t val;

		rc = oa_tc6_reg_read(ctx->tc6, OA_CONFIG2, &val);
		if (rc < 0) {
			break;
		}

		config->promisc_mode = false;
		if (val & OA_CONFIG2_FWD_UNK2HOST) {
			config->promisc_mode = true;
		}
		break;

	case ETHERNET_CONFIG_TYPE_T1S_PARAM:
		config->t1s_param.type = ETHERNET_T1S_PARAM_TYPE_PLCA_CONFIG;
		config->t1s_param.plca.enable = ctx->plca->enable;
		config->t1s_param.plca.node_id = ctx->plca->node_id;
		config->t1s_param.plca.node_count = ctx->plca->node_count;
		config->t1s_param.plca.burst_count = ctx->plca->max_burst_count;
		config->t1s_param.plca.burst_timer = ctx->plca->burst_timer;
		config->t1s_param.plca.to_timer = ctx->plca->to_timer;
		rc = 0;
		break;
	default:
		break;
	}

	return rc;
}

int ad3306_send_frame(const struct device *dev, struct net_pkt *pkt)
{
	int rc;
	struct ad3306_data *ctx = dev->data;
	struct oa_tc6 *tc6 = ctx->tc6;
	uint32_t hdr;
	uint16_t pkt_len = net_pkt_get_len(pkt);
	uint16_t frame_len, copy_len;
	uint8_t total_chunks, chunk;

	if (pkt_len == 0) {
		return -ENODATA;
	}

	total_chunks = pkt_len / tc6->cps;
	if (pkt_len % tc6->cps) {
		total_chunks++;
	}

	/* Check if MAC-PHY has any free internal buffer space */
	if (total_chunks > tc6->txc) {
		return -EIO;
	}

	/* Transform net_pkt into a buffer of TC6 chunks */
	for (chunk = 1, frame_len = 0; chunk <= total_chunks; chunk++) {
		hdr = FIELD_PREP(OA_DATA_HDR_DNC, 1) | FIELD_PREP(OA_DATA_HDR_DV, 1) |
		      FIELD_PREP(OA_DATA_HDR_NORX, 1) | FIELD_PREP(OA_DATA_HDR_SWO, 0);

		/* First chunk in frame - set SV */
		if (chunk == 1) {
			hdr |= FIELD_PREP(OA_DATA_HDR_SV, 1);
		}

		/* Last chunk in frame - set EV and EBO */
		if (chunk == total_chunks) {
			hdr |= FIELD_PREP(OA_DATA_HDR_EV, 1) |
			       FIELD_PREP(OA_DATA_HDR_EBO, (tc6->cps - 1));
		}

		/* Get parity bit and place chunk header in Tx buffer */
		hdr |= FIELD_PREP(OA_DATA_HDR_P, oa_tc6_get_parity(hdr));

		*(uint32_t *)&ctx->oa_tx_buf[frame_len] = sys_cpu_to_be32(hdr);
		frame_len += sizeof(uint32_t);

		/* Place chunk payload in Tx buffer */
		copy_len = pkt_len > tc6->cps ? tc6->cps : pkt_len;
		rc = net_pkt_read(pkt, &ctx->oa_tx_buf[frame_len], copy_len);
		if (rc < 0) {
			return rc;
		}

		frame_len += tc6->cps;
		pkt_len -= copy_len;
	}

	return ad3306_data_spi_transfer(dev, ctx->oa_rx_buf, ctx->oa_tx_buf, frame_len);
}

static int ad3306_send(const struct device *dev, struct net_pkt *pkt)
{
	int rc;
	struct ad3306_data *ctx = dev->data;
	uint32_t ftr;

	k_mutex_lock(&ctx->lock, K_FOREVER);

	rc = ad3306_send_frame(dev, pkt);

	k_mutex_unlock(&ctx->lock);

	oa_tc6_read_status(ctx->tc6, &ftr);

	/* Initiate Rx if data is available */
	if (ctx->tc6->rca > 0) {
		k_sem_give(&ctx->irq_sem);
	}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	if (rc < 0) {
		eth_stats_update_errors_tx(net_pkt_iface(pkt));
	} else {
		eth_stats_update_pkts_tx(ctx->iface);
	}
#endif /* CONFIG_NET_STATISTICS_ETHERNET */

	return rc;
}

static void ad3306_irq_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(pins);
	struct ad3306_data *ctx = CONTAINER_OF(cb, struct ad3306_data, gpio_irq_callback);

	k_sem_give(&ctx->irq_sem);
}

static int ad3306_rx_fifo_read(const struct device *dev)
{
	int rc;
	struct ad3306_data *ctx = dev->data;
	struct oa_tc6 *tc6 = ctx->tc6;
	struct net_pkt *pkt;
	uint32_t hdr, ftr;
	uint16_t len, rx_idx, pkt_len;
	static uint16_t pkt_idx;
	uint8_t chunk, swo, ebo;

	/* Prepare all OA Tx headers */
	for (chunk = 0, len = 0; chunk < tc6->rca; ++chunk) {
		hdr = FIELD_PREP(OA_DATA_HDR_DNC, 1);
		hdr |= FIELD_PREP(OA_DATA_HDR_P, oa_tc6_get_parity(hdr));

		*(uint32_t *)&ctx->oa_tx_buf[len] = sys_cpu_to_be32(hdr);
		len += sizeof(uint32_t) + tc6->cps;
	}

	rc = ad3306_data_spi_transfer(dev, ctx->oa_rx_buf, ctx->oa_tx_buf, len);
	if (rc < 0) {
		LOG_ERR("Failed to read Rx FIFO [%d]", rc);
		return rc;
	}

	/* Process Rx OA data buffer into net_pkt */
	for (chunk = 0, rx_idx = 0; chunk < tc6->rca; ++chunk) {
		ftr = sys_be32_to_cpu(*(uint32_t *)&ctx->oa_rx_buf[rx_idx + tc6->cps]);

		if (oa_tc6_get_parity(ftr)) {
			LOG_ERR("OA Rx: Footer parity error!");
			return -EIO;
		}

		if (!FIELD_GET(OA_DATA_FTR_SYNC, ftr)) {
			LOG_ERR("OA Rx: Configuration not SYNC'd!");
			return -EIO;
		}

		if (!FIELD_GET(OA_DATA_FTR_DV, ftr)) {
			LOG_DBG("OA Rx: Data chunk not valid, skip!");
			return -EIO;
		}

		if (FIELD_GET(OA_DATA_FTR_SV, ftr)) {
			swo = FIELD_GET(OA_DATA_FTR_SWO, ftr) * sizeof(uint32_t);
			if (swo != 0) {
				LOG_ERR("OA Rx: Misaligned start of frame!");
				return -EIO;
			}

			pkt_idx = 0;
		}

		ebo = FIELD_GET(OA_DATA_FTR_EBO, ftr) + 1;
		len = FIELD_GET(OA_DATA_FTR_EV, ftr) ? ebo : tc6->cps;

		memcpy(&ctx->pkt_buf[pkt_idx], &ctx->oa_rx_buf[rx_idx], len);
		pkt_idx += len;

		if (FIELD_GET(OA_DATA_FTR_EV, ftr)) {
			/* Minus CRC size */
			pkt_len = pkt_idx - sizeof(uint32_t);

			pkt = net_pkt_rx_alloc_with_buffer(ctx->iface, pkt_len, AF_UNSPEC, 0,
							   K_MSEC(CONFIG_ETH_AD3306_TIMEOUT));
			if (!pkt) {
				LOG_ERR("Failed to allocate buffer for Rx packet");
				return -ENOBUFS;
			}

			rc = net_pkt_write(pkt, ctx->pkt_buf, pkt_len);
			if (rc < 0) {
				net_pkt_unref(pkt);
				LOG_ERR("Failed to write net_pkt [%d]", rc);
				return rc;
			}

			rc = net_recv_data(ctx->iface, pkt);
			if (rc < 0) {
				LOG_ERR("Failed to process network Rx packet");
				net_pkt_unref(pkt);
			}
		}
		rx_idx += tc6->cps + sizeof(uint32_t);
	}

	return rc;
}

/**
 * Wake-up MAC-PHY by asserting WAKE pin.
 */
void ad3306_trigger_wake_up(const struct device *dev)
{
	const struct ad3306_config *cfg = dev->config;

	/* Pulse WAKE pin */
	gpio_pin_set_dt(&cfg->wake, 1U);
	k_msleep(AD3306_WAKE_PIN_ASSERT_TIME_MS);
	gpio_pin_set_dt(&cfg->wake, 0U);
}

/**
 * Enable interrupts from the MAC-PHY using IMASK0/IMASK1 registers.
 */
static int ad3306_interrupts_en(const struct device *dev)
{
	struct ad3306_data *ctx = dev->data;
	uint32_t val;

	val = AD3306_IMASK0_RST_VAL & ~GENMASK(12, 0);

	/* Unmask all interrupts in IMASK0 */
	return oa_tc6_reg_write(ctx->tc6, OA_IMASK0, val);
}

/**
 * Configure the features and settings of the MAC-PHY.
 */
static int ad3306_configure(const struct device *dev)
{
	int rc;
	struct ad3306_data *ctx = dev->data;
	uint32_t val, ftr;

	/* CONFIG0 */
	rc = oa_tc6_reg_read(ctx->tc6, OA_CONFIG0, &val);
	if (rc < 0) {
		return rc;
	}

	/* Zero-Align Receive Frame Enable */
	val |= OA_CONFIG0_RFA_ZARFE;

	/* Transmit Credit Threshold */
	val |= (AD3306_CONFIG0_TXCTHRESH_CREDIT_8 << OA_CONFIG0_TXCTHRESH);

	/* Transmit Frame Check Sequence Validation must be disabled
	 * to allow CRC appending by MAC (CONFIG2.CRC_APPEND)
	 */
	val &= ~OA_CONFIG0_TXFCSVE;

	if (ctx->tx_cut_through_en) {
		val |= OA_CONFIG0_TXCTE;
	}

	if (ctx->rx_cut_through_en) {
		val |= OA_CONFIG0_RXCTE;
	}

	/* Chunk Payload Size (CPS) */
	switch (ctx->tc6->cps) {
	case 64:
		val |= 0x6;
		break;
	case 32:
		val |= 0x5;
		break;
	case 16:
		val |= 0x4;
		break;
	case 8:
		val |= 0x3;
		break;
	default:
		LOG_WRN("Unsupported CPS! Using default (64)");
		break;
	}

	rc = oa_tc6_reg_write(ctx->tc6, OA_CONFIG0, val);
	if (rc < 0) {
		LOG_ERR("Failed to update CONFIG0 register [%d]", rc);
		return rc;
	}

	/* CONFIG2 */
	rc = oa_tc6_reg_read(ctx->tc6, OA_CONFIG2, &val);
	if (rc < 0) {
		return rc;
	}

	/* Enable CRC appending by MAC on low priority Tx queue */
	val |= OA_CONFIG2_LO_PRIO_FIFO_CRC_APPEND;

	rc = oa_tc6_reg_write(ctx->tc6, OA_CONFIG2, val);
	if (rc < 0) {
		LOG_ERR("Failed to update CONFIG2 register [%d]", rc);
		return rc;
	}

	/* Enable wake-up */
	rc = oa_tc6_reg_write(ctx->tc6, AD3306_A0_CFG_FIELDS_1,
			      (AD3306_A0_CFG_FIELDS_1_RST_VAL | AD3306_A0_CFG_FIELDS_1_CFG_VALID));

	/* Enable MAC-PHY interrupts */
	rc = ad3306_interrupts_en(dev);
	if (rc < 0) {
		LOG_ERR("Failed to enable interrupts [%d]", rc);
		return rc;
	}

	/* Configure default MAC address filters */
	rc = ad3306_default_filter_config(dev);
	if (rc < 0) {
		LOG_ERR("Failed to set up MAC filter table [%d]", rc);
		return rc;
	}

	/* Configure PLCA */
	rc = ad3306_plca_config(dev);
	if (rc < 0) {
		LOG_ERR("Failed to enable PLCA [%d]", rc);
		return rc;
	}

	/* MAC-PHY is configured */
	rc = ad3306_set_sync(dev);
	if (rc < 0) {
		LOG_ERR("Failed to sync MAC-PHY config [%d]", rc);
		return rc;
	}

	/* Get Tx credit count (buffers) so transmission can begin */
	rc = oa_tc6_read_status(ctx->tc6, &ftr);
	if (rc < 0) {
		LOG_ERR("Failed to read MAC-PHY status [%d]", rc);
	}

	return rc;
}

/**
 * Thread dedicated to handling IRQs and incoming network data
 * from the MAC-PHY. Runs each time the IRQn pin is asserted.
 */
static void ad3306_rx_thread(const struct device *dev)
{
	int rc;
	struct ad3306_data *ctx = dev->data;
	uint32_t ftr;

	while (1) {
		/* Await IRQ from MAC-PHY */
		k_sem_take(&ctx->irq_sem, K_FOREVER);

		k_mutex_lock(&ctx->lock, K_FOREVER);

		oa_tc6_read_status(ctx->tc6, &ftr);

		/* Read Rx frame(s) if available */
		if (ctx->tc6->rca > 0) {
			rc = ad3306_rx_fifo_read(dev);

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
			if (rc < 0) {
				eth_stats_update_errors_rx(ctx->iface);
			} else {
				eth_stats_update_pkts_rx(ctx->iface);
			}
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
		}

		/* Clear interrupts from the status registers.
		 * If there is a SYNC error, reset and reconfigure the MAC-PHY
		 */
		rc = oa_tc6_check_status(ctx->tc6);
		if (rc < 0) {
			ad3306_sw_reset(dev);
			ad3306_configure(dev);
		}

		k_mutex_unlock(&ctx->lock);
	}
}

/**
 * Device driver init API.
 */
static int ad3306_init(const struct device *dev)
{
	int rc;
	const struct ad3306_config *cfg = dev->config;
	struct ad3306_data *ctx = dev->data;

	/* Check SPI and GPIO interfaces are ready */
	__ASSERT(cfg->spi.config.frequency <= AD3306_SPI_MAX_FREQUENCY,
		 "SPI clock frequency exceeds supported maximum\n");

	if (!spi_is_ready_dt(&cfg->spi)) {
		LOG_ERR("SPI %s is not ready", cfg->spi.bus->name);
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->interrupt)) {
		LOG_ERR("Interrupt GPIO %s is not ready", cfg->interrupt.port->name);
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&cfg->wake)) {
		LOG_ERR("WAKE GPIO %s is not ready", cfg->wake.port->name);
		return -ENODEV;
	}

	/* Verify SPI comms between host and MAC-PHY */
	rc = ad3306_spi_test(dev);
	if (rc < 0) {
		LOG_ERR("Failed to communicate with MAC-PHY over SPI [%d]", rc);
		return rc;
	}

	/* Reset MAC-PHY */
	rc = ad3306_sw_reset(dev);
	if (rc < 0) {
		LOG_ERR("Failed to reset MAC-PHY [%d]", rc);
		return rc;
	}

	/* Config MAC-PHY IRQn interrupt service routine */
	rc = gpio_pin_configure_dt(&cfg->interrupt, GPIO_INPUT);
	if (rc < 0) {
		LOG_ERR("Failed to configure IRQn GPIO [%d]", rc);
		return rc;
	}

	rc = gpio_pin_interrupt_configure_dt(&cfg->interrupt, GPIO_INT_EDGE_TO_ACTIVE);
	if (rc != 0) {
		LOG_ERR("Failed to configure interrupt on IRQn pin [%d]", rc);
		return rc;
	}

	gpio_init_callback(&(ctx->gpio_irq_callback), ad3306_irq_callback, BIT(cfg->interrupt.pin));

	rc = gpio_add_callback(cfg->interrupt.port, &ctx->gpio_irq_callback);
	if (rc < 0) {
		LOG_ERR("Failed to add IRQn callback [%d]", rc);
		return rc;
	}

	/* Configure WAKE pin */
	rc = gpio_pin_configure_dt(&cfg->wake, GPIO_OUTPUT_INACTIVE);
	if (rc < 0) {
		LOG_ERR("Failed to configure WAKE GPIO pin [%d]", rc);
		return rc;
	}

	/* Configure the MAC-PHY features and settings */
	rc = ad3306_configure(dev);
	if (rc < 0) {
		LOG_ERR("Failed to configure MAC-PHY [%d]", rc);
		return rc;
	}

	/* Start up thread to offload IRQ/Rx data handling  */
	k_thread_create(&ctx->rx_thread, ctx->rx_thread_stack,
			CONFIG_ETH_AD3306_RX_THREAD_STACK_SIZE, (k_thread_entry_t)ad3306_rx_thread,
			(void *)dev, NULL, NULL, K_PRIO_COOP(CONFIG_ETH_AD3306_RX_THREAD_PRIO),
			K_ESSENTIAL, K_NO_WAIT);
	k_thread_name_set(&ctx->rx_thread, "eth_ad3306_rx");

	LOG_INF("10BASE-T1S Initialized [%02X:%02X:%02X:%02X:%02X:%02X]", ctx->mac_address[0],
		ctx->mac_address[1], ctx->mac_address[2], ctx->mac_address[3], ctx->mac_address[4],
		ctx->mac_address[5]);

	return rc;
}

static const struct ethernet_api ad3306_eth_api = {
	.iface_api.init = ad3306_iface_init,
	.get_capabilities = ad3306_get_capabilities,
	.set_config = ad3306_set_config,
	.get_config = ad3306_get_config,
	.send = ad3306_send,
};

#define AD3306_DEFINE(inst)                                                                        \
	static uint8_t __aligned(4) buffer_##inst[NET_ETH_MAX_FRAME_SIZE];                         \
                                                                                                   \
	static const struct ad3306_config ad3306_config_##inst = {                                 \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8), 0),                             \
		.interrupt = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                               \
		.wake = GPIO_DT_SPEC_INST_GET(inst, wake_gpios),                                   \
	};                                                                                         \
                                                                                                   \
	static struct oa_tc6 oa_tc6_##inst = {                                                     \
		.cps = 64,                                                                         \
		.protected = 1,                                                                    \
		.spi = &ad3306_config_##inst.spi,                                                  \
	};                                                                                         \
                                                                                                   \
	static struct ad3306_plca_config plca_config_##inst = {                                    \
		.enable = DT_INST_PROP(inst, plca_enable),                                         \
		.node_id = DT_INST_PROP(inst, plca_node_id),                                       \
		.node_count = DT_INST_PROP(inst, plca_node_count),                                 \
		.max_burst_count = DT_INST_PROP(inst, plca_max_burst_count),                       \
		.burst_timer = DT_INST_PROP(inst, plca_burst_timer),                               \
		.to_timer = DT_INST_PROP(inst, plca_to_timer),                                     \
	};                                                                                         \
                                                                                                   \
	static struct ad3306_data ad3306_data_##inst = {                                           \
		.lock = Z_MUTEX_INITIALIZER((ad3306_data_##inst).lock),                            \
		.irq_sem = Z_SEM_INITIALIZER((ad3306_data_##inst).irq_sem, 0, 1),                  \
		.mac_address = DT_INST_PROP(inst, local_mac_address),                              \
		.tc6 = &oa_tc6_##inst,                                                             \
		.plca = &plca_config_##inst,                                                       \
		.pkt_buf = buffer_##inst,                                                          \
		.tx_cut_through_en = DT_INST_PROP(inst, tx_cut_through_en),                        \
		.rx_cut_through_en = DT_INST_PROP(inst, rx_cut_through_en),                        \
	};                                                                                         \
                                                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(inst, ad3306_init, NULL, &ad3306_data_##inst,                \
				      &ad3306_config_##inst, CONFIG_ETH_INIT_PRIORITY,             \
				      &ad3306_eth_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(AD3306_DEFINE);
