/*
 * Copyright (c) 2024 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ETH_AD3306_PRIV_H__
#define ETH_AD3306_PRIV_H__

#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/util.h>

#include "oa_tc6.h"

/* Constants */
#define AD3306_SPI_MAX_FREQUENCY          MHZ(25)
#define AD3306_PHYID_RST_VAL              0x0283BE00
#define AD3306_RESET_CYCLE_TIME_US        2U
#define AD3306_WAKE_PIN_ASSERT_TIME_MS    100U
#define AD3306_RST_COMPLETE               0x3
#define AD3306_IMASK0_RST_VAL             0x00001FBF
#define AD3306_IMASK1_RST_VAL             0x43FA1F5A
#define AD3306_A0_CFG_FIELDS_1_RST_VAL    0x4
#define AD3306_CONFIG0_TXCTHRESH_CREDIT_8 0x2
#define AD3306_OA_SPI_BUF_SIZE            (255U * 68U) /* Max 255 68-byte chunks */

/* AD3306 Registers (non-OA) */
#define AD3306_MAC_RST_STATUS            MMS_REG(0x1, 0x3B)
#define AD3306_A0_CFG_FIELDS_1           MMS_REG(0xA, 0xB703)
#define AD3306_MAC_ADDR_FILT_UPR         MMS_REG(0x1, 0x50)
#define AD3306_MAC_ADDR_FILT_LWR         MMS_REG(0x1, 0x51)
#define AD3306_MAC_ADDR_MASK_UPR         MMS_REG(0x1, 0x70)
#define AD3306_MAC_ADDR_MASK_LWR         MMS_REG(0x1, 0x71)
#define AD3306_MAC_RST_STATUS_MASK       GENMASK(1, 0)
#define AD3306_STATUS1_RX_RDY            BIT(4)
#define AD3306_A0_CFG_FIELDS_1_CFG_VALID BIT(7)
#define AD3306_MAC_ADDR_FILT_APPLY2PORT1 BIT(30)
#define AD3306_MAC_ADDR_FILT_TO_HOST     BIT(16)

/* OA TC6 Registers */
#define OA_CONFIG2                         MMS_REG(0x0, 0x006)
#define OA_PHY_OA_PLCA_CTRL0               MMS_REG(0x4, 0xCA01)
#define OA_PHY_OA_PLCA_CTRL1               MMS_REG(0x4, 0xCA02)
#define OA_PHY_OA_PLCA_TOTMR               MMS_REG(0x4, 0xCA04)
#define OA_PHY_OA_PLCA_BURST               MMS_REG(0x4, 0xCA05)
#define OA_PHY_OA_PLCA_CTRL5               MMS_REG(0x4, 0x001B)
#define OA_IMASK0_PHYINTM                  BIT(7)
#define OA_CONFIG0_TXCTHRESH               10U
#define OA_CONFIG0_TXCTE                   BIT(9)
#define OA_CONFIG0_RXCTE                   BIT(8)
#define OA_CONFIG0_FTSE                    BIT(7)
#define OA_CONFIG0_FTSS                    BIT(6)
#define OA_CONFIG0_TXFCSVE                 BIT(14)
#define OA_CONFIG2_LO_PRIO_FIFO_CRC_APPEND BIT(17)
#define OA_CONFIG2_TX_RDY_ON_EMPTY         BIT(8)
#define OA_CONFIG2_HOST_CRC_APPEND         BIT(5)
#define OA_CONFIG2_FWD_UNK2HOST            BIT(2)
#define OA_STATUS0_LOFE                    BIT(4)
#define OA_STATUS0_RXBOE                   BIT(3)
#define OA_PHY_OA_PLCA_CTRL0_PLCAEN        BIT(15)
#define OA_PHY_OA_PLCA_CTRL1_NODECOUNT     8U
#define OA_PHY_OA_PLCA_BURST_MAXBURSTCNT   8U

/* MAC address filter table slots (16 total).
 * Slot addresses increment in 2.
 * Slot 0 is not available in U3/U4 silicon.
 */
#define AD3306_MAC_FILT_TABLE_MULTICAST_SLOT 2U
#define AD3306_MAC_FILT_TABLE_BROADCAST_SLOT 4U
#define AD3306_MAC_FILT_TABLE_UNICAST_SLOT   6U
#define AD3306_MAC_FILT_TABLE_BASE_SLOT      8U
#define AD3306_MAC_FILT_TABLE_SLOT_SIZE      2U
#define AD3306_MAC_FILT_TABLE_MAX_SLOT       30U

struct ad3306_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec interrupt;
	struct gpio_dt_spec wake;
};

struct ad3306_data {
	struct net_if *iface;
	struct gpio_callback gpio_irq_callback;
	struct k_mutex lock;
	struct k_sem irq_sem;
	struct oa_tc6 *tc6;
	struct ad3306_plca_config *plca;
	uint8_t mac_address[NET_ETH_ADDR_LEN];
	bool tx_cut_through_en: 1;
	bool rx_cut_through_en: 1;

	/* Frame/packet data buffers */
	uint8_t oa_tx_buf[AD3306_OA_SPI_BUF_SIZE];
	uint8_t oa_rx_buf[AD3306_OA_SPI_BUF_SIZE];
	uint8_t *pkt_buf;

	K_KERNEL_STACK_MEMBER(rx_thread_stack, CONFIG_ETH_AD3306_RX_THREAD_STACK_SIZE);
	struct k_thread rx_thread;
};

struct ad3306_plca_config {
	bool enable: 1;
	uint8_t node_id;
	uint8_t node_count;
	uint8_t max_burst_count;
	uint8_t burst_timer;
	uint8_t to_timer;
};

#endif /* ETH_AD3306_PRIV_H__ */
