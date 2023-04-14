// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2012-2018, 2020 The Linux Foundation. All rights reserved.
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 */
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/mfd/wcd9xxx/wcd9xxx_registers.h>
#include <asoc/wcd9xxx-slimslave.h>

struct wcd9xxx_slim_sch {
	u16 rx_port_ch_reg_base;
	u16 port_tx_cfg_reg_base;
	u16 port_rx_cfg_reg_base;
};

static struct wcd9xxx_slim_sch sh_ch;
static int wcd9xxx_configure_ports(struct wcd9xxx *wcd9xxx)
{
	if (wcd9xxx->codec_type->slim_slave_type ==
	    WCD9XXX_SLIM_SLAVE_ADDR_TYPE_0) {
		sh_ch.rx_port_ch_reg_base = 0x180;
		sh_ch.port_rx_cfg_reg_base = 0x040;
		sh_ch.port_tx_cfg_reg_base = 0x040;
	} else {
		sh_ch.rx_port_ch_reg_base =
			0x180 - (TAIKO_SB_PGD_OFFSET_OF_RX_SLAVE_DEV_PORTS * 4);
		sh_ch.port_rx_cfg_reg_base =
			0x040 - TAIKO_SB_PGD_OFFSET_OF_RX_SLAVE_DEV_PORTS;
		sh_ch.port_tx_cfg_reg_base = 0x050;
	}

	return 0;
}

/**
 * wcd9xxx_init_slimslave
 *
 * @wcd9xxx: pointer to wcd9xxx struct
 * @wcd9xxx_pgd_la: pgd_la value
 * @tx_num: tx number
 * @rx_num: rx number
 * @tx_slot: pointer to tx slot
 * @rx_slot: pointer to rx slot
 *
 * Returns 0 on success, appropriate error code otherwise
 */
int wcd9xxx_init_slimslave(struct wcd9xxx *wcd9xxx, u8 wcd9xxx_pgd_la,
			   unsigned int tx_num, unsigned int *tx_slot,
			   unsigned int rx_num, unsigned int *rx_slot)
{
	int ret = 0;
	int i;

	ret = wcd9xxx_configure_ports(wcd9xxx);
	if (ret) {
		pr_err("%s: Failed to configure register address offset\n",
		       __func__);
		goto err;
	}

	if (!rx_num || rx_num > wcd9xxx->num_rx_port) {
		pr_err("%s: invalid rx num %d\n", __func__, rx_num);
		return -EINVAL;
	}

	if (wcd9xxx->rx_chs) {
		wcd9xxx->num_rx_port = rx_num;
		for (i = 0; i < rx_num; i++) {
			wcd9xxx->rx_chs[i].ch_num = rx_slot[i];
			INIT_LIST_HEAD(&wcd9xxx->rx_chs[i].list);
		}
		wcd9xxx->sruntime_rx = slim_stream_allocate(wcd9xxx->slim, "WCD9xxx-SLIM-RX");
		if (wcd9xxx->sruntime_rx == NULL) {
			pr_err("%s: Failed to alloc %d rx slimbus channels\n",
				__func__, wcd9xxx->num_rx_port);
			kfree(wcd9xxx->rx_chs);
			wcd9xxx->rx_chs = NULL;
			wcd9xxx->num_rx_port = 0;
		}
	} else {
		pr_err("Not able to allocate memory for %d slimbus rx ports\n",
			wcd9xxx->num_rx_port);
	}

	if (!tx_num || tx_num > wcd9xxx->num_tx_port) {
		pr_err("%s: invalid tx num %d\n", __func__, tx_num);
		return -EINVAL;
	}
	if (wcd9xxx->tx_chs) {
		wcd9xxx->num_tx_port = tx_num;
		for (i = 0; i < tx_num; i++) {
			wcd9xxx->tx_chs[i].ch_num = tx_slot[i];
			INIT_LIST_HEAD(&wcd9xxx->tx_chs[i].list);
		}
		wcd9xxx->sruntime_tx = slim_stream_allocate(wcd9xxx->slim, "WCD9xxx-SLIM-TX");
		if (wcd9xxx->sruntime_tx == NULL) {
			pr_err("%s: Failed to alloc %d tx slimbus channels\n",
				__func__, wcd9xxx->num_tx_port);
			kfree(wcd9xxx->tx_chs);
			wcd9xxx->tx_chs = NULL;
			wcd9xxx->num_tx_port = 0;
		}
	} else {
		pr_err("Not able to allocate memory for %d slimbus tx ports\n",
			wcd9xxx->num_tx_port);
	}

	return 0;
err:
	return ret;
}
EXPORT_SYMBOL_GPL(wcd9xxx_init_slimslave);

/* Enable slimbus slave device for RX path */
int wcd9xxx_cfg_slim_sch_rx(struct wcd9xxx *wcd9xxx,
			    struct list_head *wcd9xxx_ch_list,
			    unsigned int rate, unsigned int bit_width,
			    unsigned int direction)
{
	u8 ch_cnt = 0;
	u8  payload = 0;
	u16 codec_port = 0;
	int ret = 0;
	struct wcd9xxx_ch *rx;
	int i = 0;
	struct slim_stream_config sconfig_rx;

	sconfig_rx.direction = direction;
	sconfig_rx.bps = bit_width;
	sconfig_rx.rate = rate;
	/* Configure slave interface device */
	list_for_each_entry(rx, wcd9xxx_ch_list, list) {
		payload |= 1 << rx->shift;
		ch_cnt++;
		sconfig_rx.port_mask |= BIT(rx->port);
	}

	pr_debug("ch_count=%d,mask=%d,rate=%d,WATER_MARK_VAL=%d\n",
				 ch_cnt, sconfig_rx.port_mask, sconfig_rx.rate, WATER_MARK_VAL);
	sconfig_rx.ch_count = ch_cnt;
	sconfig_rx.chs = kcalloc(sconfig_rx.ch_count, sizeof(unsigned int), GFP_KERNEL);

	if (!sconfig_rx.chs) {
		return -ENOMEM;
	}

	list_for_each_entry(rx, wcd9xxx_ch_list, list) {
		sconfig_rx.chs[i++] = rx->ch_num;
		codec_port = rx->port;
		pr_debug("%s: codec_port %d rx 0x%p, payload %d\n"
			 "sh_ch.rx_port_ch_reg_base0 0x%x\n"
			 "sh_ch.port_rx_cfg_reg_base 0x%x\n",
			 __func__, codec_port, rx, payload,
			 sh_ch.rx_port_ch_reg_base,
			sh_ch.port_rx_cfg_reg_base);

		/* look for the valid port range and chose the
		 * payload accordingly
		 */
		/* write to interface device */
		ret = wcd9xxx_interface_reg_write(wcd9xxx,
				SB_PGD_RX_PORT_MULTI_CHANNEL_0(
				sh_ch.rx_port_ch_reg_base, codec_port),
				payload);
		if (ret < 0) {
			pr_err("%s:Intf-dev fail reg[%d] payload[%d] ret[%d]\n",
				__func__,
				SB_PGD_RX_PORT_MULTI_CHANNEL_0(
				sh_ch.rx_port_ch_reg_base, codec_port),
				payload, ret);
			goto err_close_slim_sch;
		}
		/* configure the slave port for water mark and enable*/
		ret = wcd9xxx_interface_reg_write(wcd9xxx,
				SB_PGD_PORT_CFG_BYTE_ADDR(
				sh_ch.port_rx_cfg_reg_base, codec_port),
				WATER_MARK_VAL);
		if (ret < 0) {
			pr_err("%s:watermark set failure for port[%d] ret[%d]",
				__func__, codec_port, ret);
		}
	}

	/* slim_control_ch */
	ret = slim_stream_prepare(wcd9xxx->sruntime_rx, &sconfig_rx);
	if (ret < 0) {
		pr_err("%s: slim_stream_prepare failed ret[%d]\n",
			__func__, ret);
		goto err_close_slim_sch;
	}

	ret = slim_stream_enable(wcd9xxx->sruntime_rx);
	if (ret < 0) {
		pr_err("%s: slim_stream_enable failed ret[%d]\n",
			__func__, ret);
		goto err_close_slim_sch;
	}

	/*  release all acquired handles */
	sconfig_rx.port_mask = 0;
	kfree(sconfig_rx.chs);
	sconfig_rx.chs = NULL;
	return 0;

err_close_slim_sch:
	/*  release all acquired handles */
	sconfig_rx.port_mask = 0;
	kfree(sconfig_rx.chs);
	sconfig_rx.chs = NULL;
	return ret;
}
EXPORT_SYMBOL(wcd9xxx_cfg_slim_sch_rx);

/* Enable slimbus slave device for RX path */
int wcd9xxx_cfg_slim_sch_tx(struct wcd9xxx *wcd9xxx,
			    struct list_head *wcd9xxx_ch_list,
			    unsigned int rate, unsigned int bit_width,
			    unsigned int direction)
{
	u16 ch_cnt = 0;
	u16 payload = 0;
	u16 codec_port;
	int ret = 0;
	struct wcd9xxx_ch *tx;
	int i = 0;
	struct slim_stream_config sconfig_tx;

	sconfig_tx.direction = direction;
	sconfig_tx.bps = bit_width;
	sconfig_tx.rate = rate;

	list_for_each_entry(tx, wcd9xxx_ch_list, list) {
		payload |= 1 << tx->shift;
		ch_cnt++;
		sconfig_tx.port_mask |= BIT(tx->port);
	}

	sconfig_tx.ch_count = ch_cnt;
	sconfig_tx.chs = kcalloc(sconfig_tx.ch_count, sizeof(unsigned int), GFP_KERNEL);

	if (!sconfig_tx.chs) {
		return -ENOMEM;
	}

	list_for_each_entry(tx, wcd9xxx_ch_list, list) {
		codec_port = tx->port;
		sconfig_tx.chs[i++] = tx->ch_num;
		pr_debug("%s: codec_port %d tx 0x%p, payload 0x%x\n",
			 __func__, codec_port, tx, payload);
		/* write to interface device */
		ret = wcd9xxx_interface_reg_write(wcd9xxx,
				SB_PGD_TX_PORT_MULTI_CHANNEL_0(codec_port),
				payload & 0x00FF);
		if (ret < 0) {
			pr_err("%s:Intf-dev fail reg[%d] payload[%d] ret[%d]\n",
				__func__,
				SB_PGD_TX_PORT_MULTI_CHANNEL_0(codec_port),
				payload, ret);
			goto err;
		}
		/* ports 8,9 */
		ret = wcd9xxx_interface_reg_write(wcd9xxx,
				SB_PGD_TX_PORT_MULTI_CHANNEL_1(codec_port),
				(payload & 0xFF00)>>8);
		if (ret < 0) {
			pr_err("%s:Intf-dev fail reg[%d] payload[%d] ret[%d]\n",
				__func__,
				SB_PGD_TX_PORT_MULTI_CHANNEL_1(codec_port),
				payload, ret);
			goto err;
		}
		/* configure the slave port for water mark and enable*/
		ret = wcd9xxx_interface_reg_write(wcd9xxx,
				SB_PGD_PORT_CFG_BYTE_ADDR(
				sh_ch.port_tx_cfg_reg_base, codec_port),
				WATER_MARK_VAL);
		if (ret < 0) {
			pr_err("%s:watermark set failure for port[%d] ret[%d]",
				__func__, codec_port, ret);
		}
	}

	ret = slim_stream_prepare(wcd9xxx->sruntime_tx, &sconfig_tx);
	if (ret < 0) {
		pr_err("%s: slim_stream_prepare failed ret[%d]\n",
			__func__, ret);
		goto err;
	}

	ret = slim_stream_enable(wcd9xxx->sruntime_tx);
	if (ret < 0) {
		pr_err("%s: slim_stream_enable failed ret[%d]\n",
			__func__, ret);
		goto err;
	}

	/* release all acquired handles */
	sconfig_tx.port_mask = 0;
	kfree(sconfig_tx.chs);
	sconfig_tx.chs = NULL;
	return 0;
err:
	/* release all acquired handles */
	sconfig_tx.port_mask = 0;
	kfree(sconfig_tx.chs);
	sconfig_tx.chs = NULL;
	return ret;
}
EXPORT_SYMBOL_GPL(wcd9xxx_close_slim_sch);

int wcd9xxx_get_slave_port(unsigned int ch_num)
{
	int ret = 0;

	ret = (ch_num - BASE_CH_NUM);
	pr_debug("%s: ch_num[%d] slave port[%d]\n", __func__, ch_num, ret);
	if (ret < 0) {
		pr_err("%s: Error:- Invalid slave port found = %d\n",
			__func__, ret);
		return -EINVAL;
	}
	return ret;
}
EXPORT_SYMBOL(wcd9xxx_get_slave_port);

int wcd9xxx_disconnect_port_tx(struct wcd9xxx *wcd9xxx)
{
	int ret = 0;

	if (wcd9xxx->sruntime_tx == NULL) {
		pr_err("Channel not enabled yet. returning\n");
		return -EINVAL;
	}

	/* free the ports allocated to the stream */
	ret = slim_stream_unprepare_disconnect_port(wcd9xxx->sruntime_tx, true, true);

	if (ret != 0)
		pr_err("slim_stream_unprepare failed: returned val = %d\n", ret);

	ret = slim_stream_disable(wcd9xxx->sruntime_tx);

	if (ret != 0)
		pr_err("slim_stream_disable failed :returned val = %d\n", ret);

	return ret;
}
EXPORT_SYMBOL_GPL(wcd9xxx_disconnect_port_tx);

int wcd9xxx_disconnect_port_rx(struct wcd9xxx *wcd9xxx)
{
	int ret = 0;

	if (wcd9xxx->sruntime_rx == NULL) {
		pr_err("Channel not enabled yet. returning\n");
		return -EINVAL;
	}

	/* free the ports allocated to the stream */
	ret = slim_stream_unprepare_disconnect_port(wcd9xxx->sruntime_rx, true, true);
	if (ret != 0)
		pr_err("slim_stream_unprepare failed: returned val = %d\n", ret);

	ret = slim_stream_disable(wcd9xxx->sruntime_rx);
	if (ret != 0)
		pr_err("slim_stream_disable failed :returned val = %d\n", ret);
	return ret;
}
EXPORT_SYMBOL_GPL(wcd9xxx_disconnect_port_rx);

/* This function is called with mutex acquired */
int wcd9xxx_rx_vport_validation(u32 port_id,
				struct list_head *codec_dai_list)
{
	struct wcd9xxx_ch *ch;
	int ret = 0;

	pr_debug("%s: port_id %u\n", __func__, port_id);
	list_for_each_entry(ch,
		codec_dai_list, list) {
		pr_debug("%s: ch->port %u\n", __func__, ch->port);
		if (ch->port == port_id) {
			ret = -EINVAL;
			break;
		}
	}
	return ret;
}
EXPORT_SYMBOL(wcd9xxx_rx_vport_validation);


/* This function is called with mutex acquired */
int wcd9xxx_tx_vport_validation(u32 table, u32 port_id,
				struct wcd9xxx_codec_dai_data *codec_dai,
				u32 num_codec_dais)
{
	struct wcd9xxx_ch *ch;
	int ret = 0;
	u32 index;
	unsigned long vtable = table;
	u32 size = sizeof(table) * BITS_PER_BYTE;

	pr_debug("%s: vtable 0x%lx port_id %u size %d\n", __func__,
		 vtable, port_id, size);
	for_each_set_bit(index, &vtable, size) {
		if (index < num_codec_dais) {
			list_for_each_entry(ch,
					&codec_dai[index].wcd9xxx_ch_list,
					list) {
				pr_debug("%s: index %u ch->port %u vtable 0x%lx\n",
						__func__, index, ch->port,
						vtable);
				if (ch->port == port_id) {
					pr_err("%s: TX%u is used by AIF%u_CAP Mixer\n",
							__func__, port_id + 1,
							(index + 1)/2);
					ret = -EINVAL;
					break;
				}
			}
		} else {
			pr_err("%s: Invalid index %d of codec dai",
					__func__, index);
			ret = -EINVAL;
		}
		if (ret)
			break;
	}
	return ret;
}
EXPORT_SYMBOL(wcd9xxx_tx_vport_validation);

int wcd9xxx_deinit_slimslave(struct wcd9xxx *wcd9xxx)
{
	int ret = 0;

	if (wcd9xxx->sruntime_tx) {
		ret = slim_stream_free(wcd9xxx->sruntime_tx);
		wcd9xxx->sruntime_tx = NULL;
	}

	if (wcd9xxx->sruntime_rx) {
		ret = slim_stream_free(wcd9xxx->sruntime_rx);
		wcd9xxx->sruntime_rx = NULL;
	}
	return ret;
}
