/* Copyright 2008 - 2015 Freescale Semiconductor Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/kthread.h>
#include <linux/io.h>
#include <linux/if_arp.h>
#include <linux/if_vlan.h>
#include <linux/icmp.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/udp.h>
#include <linux/tcp.h>
#include <linux/net.h>
#include <linux/if_ether.h>
#include <linux/highmem.h>
#include <linux/percpu.h>
#include <linux/dma-mapping.h>
#include <soc/fsl/bman.h>

#include "fman.h"
#include "fman_port.h"

#include "mac.h"
#include "dpaa_eth.h"
#include "dpaa_eth_common.h"

/* Valid checksum indication */
#define DPA_CSUM_VALID		0xFFFF

#define DPAA_MSG_DEFAULT (NETIF_MSG_DRV | NETIF_MSG_PROBE | \
			  NETIF_MSG_LINK | NETIF_MSG_IFUP | \
			  NETIF_MSG_IFDOWN)

#define DPAA_INGRESS_CS_THRESHOLD 0x10000000
/* Ingress congestion threshold on FMan ports
 * The size in bytes of the ingress tail-drop threshold on FMan ports.
 * Traffic piling up above this value will be rejected by QMan and discarded
 * by FMan.
 */

static int debug = -1;
module_param(debug, int, S_IRUGO);
MODULE_PARM_DESC(debug, "Module/Driver verbosity level (0=none,...,16=all)");

static u16 tx_timeout = 1000;
module_param(tx_timeout, ushort, S_IRUGO);
MODULE_PARM_DESC(tx_timeout, "The Tx timeout in ms");

static u8 dpa_common_bpid;

static void dpa_rx_error(struct net_device *net_dev,
			 const struct dpa_priv *priv,
			 struct dpa_percpu_priv *percpu_priv,
			 const struct qm_fd *fd,
			 u32 fqid)
{
	if (net_ratelimit())
		netif_err(priv, hw, net_dev, "Err FD status = 0x%08x\n",
			  fd->status & FM_FD_STAT_RX_ERRORS);

	percpu_priv->stats.rx_errors++;

	if (fd->status & FM_FD_ERR_DMA)
		percpu_priv->rx_errors.dme++;
	if (fd->status & FM_FD_ERR_PHYSICAL)
		percpu_priv->rx_errors.fpe++;
	if (fd->status & FM_FD_ERR_SIZE)
		percpu_priv->rx_errors.fse++;
	if (fd->status & FM_FD_ERR_PRS_HDR_ERR)
		percpu_priv->rx_errors.phe++;

	dpa_fd_release(net_dev, fd);
}

static void dpa_tx_error(struct net_device *net_dev,
			 const struct dpa_priv *priv,
			 struct dpa_percpu_priv *percpu_priv,
			 const struct qm_fd *fd,
			 u32 fqid)
{
	struct sk_buff *skb;

	if (net_ratelimit())
		netif_warn(priv, hw, net_dev, "FD status = 0x%08x\n",
			   fd->status & FM_FD_STAT_TX_ERRORS);

	percpu_priv->stats.tx_errors++;

	/* If we intended the buffers from this frame to go into the bpools
	 * when the FMan transmit was done, we need to put it in manually.
	 */
	if (fd->bpid != FSL_DPAA_BPID_INV) {
		dpa_fd_release(net_dev, fd);
		return;
	}

	skb = dpa_cleanup_tx_fd(priv, fd);
	dev_kfree_skb(skb);
}

static int dpaa_eth_poll(struct napi_struct *napi, int budget)
{
	struct dpa_napi_portal *np =
			container_of(napi, struct dpa_napi_portal, napi);

	int cleaned = qman_p_poll_dqrr(np->p, budget);

	if (cleaned < budget) {
		int tmp;

		napi_complete(napi);
		tmp = qman_p_irqsource_add(np->p, QM_PIRQ_DQRI);
		WARN_ON(tmp);
	} else if (np->down) {
		qman_p_irqsource_add(np->p, QM_PIRQ_DQRI);
	}

	return cleaned;
}

static void dpa_tx_conf(struct net_device *net_dev,
			const struct dpa_priv *priv,
			struct dpa_percpu_priv *percpu_priv,
			const struct qm_fd *fd,
			u32 fqid)
{
	struct sk_buff	*skb;

	if (unlikely(fd->status & FM_FD_STAT_TX_ERRORS) != 0) {
		if (net_ratelimit())
			netif_warn(priv, hw, net_dev, "FD status = 0x%08x\n",
				   fd->status & FM_FD_STAT_TX_ERRORS);

		percpu_priv->stats.tx_errors++;
	}

	percpu_priv->tx_confirm++;

	skb = dpa_cleanup_tx_fd(priv, fd);

	dev_kfree_skb(skb);
}

static enum qman_cb_dqrr_result rx_error_dqrr(struct qman_portal *portal,
					      struct qman_fq *fq,
					      const struct qm_dqrr_entry *dq)
{
	struct net_device *net_dev;
	struct dpa_priv *priv;
	struct dpa_percpu_priv *percpu_priv;
	int *count_ptr;
	struct dpa_fq *dpa_fq = container_of(fq, struct dpa_fq, fq_base);

	net_dev = dpa_fq->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = this_cpu_ptr(priv->percpu_priv);
	count_ptr = this_cpu_ptr(priv->dpa_bp->percpu_count);

	if (dpaa_eth_napi_schedule(percpu_priv, portal))
		return qman_cb_dqrr_stop;

	if (dpaa_eth_refill_bpools(priv->dpa_bp, count_ptr))
		/* Unable to refill the buffer pool due to insufficient
		 * system memory. Just release the frame back into the pool,
		 * otherwise we'll soon end up with an empty buffer pool.
		 */
		dpa_fd_release(net_dev, &dq->fd);
	else
		dpa_rx_error(net_dev, priv, percpu_priv, &dq->fd, fq->fqid);

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result rx_default_dqrr(struct qman_portal *portal,
						struct qman_fq *fq,
						const struct qm_dqrr_entry *dq)
{
	struct net_device *net_dev;
	struct dpa_priv *priv;
	struct dpa_percpu_priv *percpu_priv;
	int *count_ptr;
	struct dpa_bp *dpa_bp;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);
	dpa_bp = priv->dpa_bp;

	percpu_priv = this_cpu_ptr(priv->percpu_priv);
	count_ptr = this_cpu_ptr(dpa_bp->percpu_count);

	if (unlikely(dpaa_eth_napi_schedule(percpu_priv, portal)))
		return qman_cb_dqrr_stop;

	/* Vale of plenty: make sure we didn't run out of buffers */

	if (unlikely(dpaa_eth_refill_bpools(dpa_bp, count_ptr)))
		/* Unable to refill the buffer pool due to insufficient
		 * system memory. Just release the frame back into the pool,
		 * otherwise we'll soon end up with an empty buffer pool.
		 */
		dpa_fd_release(net_dev, &dq->fd);
	else
		dpa_rx(net_dev, portal, priv, percpu_priv, &dq->fd, fq->fqid,
		       count_ptr);

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result conf_error_dqrr(struct qman_portal *portal,
						struct qman_fq *fq,
						const struct qm_dqrr_entry *dq)
{
	struct net_device *net_dev;
	struct dpa_priv *priv;
	struct dpa_percpu_priv *percpu_priv;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = this_cpu_ptr(priv->percpu_priv);

	if (dpaa_eth_napi_schedule(percpu_priv, portal))
		return qman_cb_dqrr_stop;

	dpa_tx_error(net_dev, priv, percpu_priv, &dq->fd, fq->fqid);

	return qman_cb_dqrr_consume;
}

static enum qman_cb_dqrr_result conf_dflt_dqrr(struct qman_portal *portal,
					       struct qman_fq *fq,
					       const struct qm_dqrr_entry *dq)
{
	struct net_device *net_dev;
	struct dpa_priv *priv;
	struct dpa_percpu_priv *percpu_priv;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);

	percpu_priv = this_cpu_ptr(priv->percpu_priv);

	if (dpaa_eth_napi_schedule(percpu_priv, portal))
		return qman_cb_dqrr_stop;

	dpa_tx_conf(net_dev, priv, percpu_priv, &dq->fd, fq->fqid);

	return qman_cb_dqrr_consume;
}

static void priv_ern(struct qman_portal *portal,
		     struct qman_fq *fq,
		     const struct qm_mr_entry *msg)
{
	struct net_device *net_dev;
	const struct dpa_priv *priv;
	struct sk_buff *skb;
	struct dpa_percpu_priv *percpu_priv;
	const struct qm_fd *fd = &msg->ern.fd;

	net_dev = ((struct dpa_fq *)fq)->net_dev;
	priv = netdev_priv(net_dev);
	percpu_priv = this_cpu_ptr(priv->percpu_priv);

	percpu_priv->stats.tx_dropped++;
	percpu_priv->stats.tx_fifo_errors++;
	count_ern(percpu_priv, msg);

	/* If we intended this buffer to go into the pool
	 * when the FM was done, we need to put it in
	 * manually.
	 */
	if (msg->ern.fd.bpid != FSL_DPAA_BPID_INV) {
		dpa_fd_release(net_dev, fd);
		return;
	}

	skb = dpa_cleanup_tx_fd(priv, fd);
	dev_kfree_skb_any(skb);
}

static const struct dpa_fq_cbs dpaa_fq_cbs = {
	.rx_defq = { .cb = { .dqrr = rx_default_dqrr } },
	.tx_defq = { .cb = { .dqrr = conf_dflt_dqrr } },
	.rx_errq = { .cb = { .dqrr = rx_error_dqrr } },
	.tx_errq = { .cb = { .dqrr = conf_error_dqrr } },
	.egress_ern = { .cb = { .ern = priv_ern } }
};

static void dpaa_eth_napi_enable(struct dpa_priv *priv)
{
	struct dpa_percpu_priv *percpu_priv;
	int i, j;

	for_each_possible_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);

		for (j = 0; j < qman_portal_max; j++) {
			percpu_priv->np[j].down = 0;
			napi_enable(&percpu_priv->np[j].napi);
		}
	}
}

static void dpaa_eth_napi_disable(struct dpa_priv *priv)
{
	struct dpa_percpu_priv *percpu_priv;
	int i, j;

	for_each_possible_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);

		for (j = 0; j < qman_portal_max; j++) {
			percpu_priv->np[j].down = 1;
			napi_disable(&percpu_priv->np[j].napi);
		}
	}
}

static int dpa_eth_priv_start(struct net_device *net_dev)
{
	struct dpa_priv *priv;
	int err;

	priv = netdev_priv(net_dev);
	dpaa_eth_napi_enable(priv);

	err = dpa_start(net_dev);
	if (err < 0)
		dpaa_eth_napi_disable(priv);

	return err;
}

static int dpa_eth_priv_stop(struct net_device *net_dev)
{
	struct dpa_priv *priv;
	int err;

	err = dpa_stop(net_dev);

	priv = netdev_priv(net_dev);
	dpaa_eth_napi_disable(priv);

	return err;
}

static struct net_device_ops dpaa_ops = {
	.ndo_open = dpa_eth_priv_start,
	.ndo_start_xmit = dpa_tx,
	.ndo_stop = dpa_eth_priv_stop,
	.ndo_tx_timeout = dpa_timeout,
	.ndo_get_stats64 = dpa_get_stats64,
	.ndo_set_mac_address = dpa_set_mac_address,
	.ndo_validate_addr = eth_validate_addr,
#ifdef CONFIG_FSL_DPAA_ETH_USE_NDO_SELECT_QUEUE
	.ndo_select_queue = dpa_select_queue,
#endif
	.ndo_change_mtu = dpa_change_mtu,
	.ndo_set_rx_mode = dpa_set_rx_mode,
	.ndo_init = dpa_ndo_init,
	.ndo_set_features = dpa_set_features,
	.ndo_fix_features = dpa_fix_features,
};

static int dpa_napi_add(struct net_device *net_dev)
{
	struct dpa_priv *priv = netdev_priv(net_dev);
	struct dpa_percpu_priv *percpu_priv;
	int i, cpu;

	for_each_possible_cpu(cpu) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, cpu);

		percpu_priv->np = devm_kzalloc(net_dev->dev.parent,
			qman_portal_max * sizeof(struct dpa_napi_portal),
			GFP_KERNEL);

		if (!percpu_priv->np)
			return -ENOMEM;

		for (i = 0; i < qman_portal_max; i++)
			netif_napi_add(net_dev, &percpu_priv->np[i].napi,
				       dpaa_eth_poll, NAPI_POLL_WEIGHT);
	}

	return 0;
}

void dpa_napi_del(struct net_device *net_dev)
{
	struct dpa_priv *priv = netdev_priv(net_dev);
	struct dpa_percpu_priv *percpu_priv;
	int i, cpu;

	for_each_possible_cpu(cpu) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, cpu);

		if (percpu_priv->np) {
			for (i = 0; i < qman_portal_max; i++)
				netif_napi_del(&percpu_priv->np[i].napi);

			devm_kfree(net_dev->dev.parent, percpu_priv->np);
		}
	}
}

static struct dpa_bp *dpa_priv_bp_probe(struct device *dev)
{
	struct dpa_bp *dpa_bp;

	dpa_bp = devm_kzalloc(dev, sizeof(*dpa_bp), GFP_KERNEL);
	if (!dpa_bp)
		return ERR_PTR(-ENOMEM);

	dpa_bp->percpu_count = devm_alloc_percpu(dev, *dpa_bp->percpu_count);
	dpa_bp->config_count = FSL_DPAA_ETH_MAX_BUF_COUNT;

	dpa_bp->seed_cb = dpa_bp_seed;
	dpa_bp->free_buf_cb = dpa_bp_free_pf;

	return dpa_bp;
}

/* Place all ingress FQs (Rx Default, Rx Error) in a dedicated CGR.
 * We won't be sending congestion notifications to FMan; for now, we just use
 * this CGR to generate enqueue rejections to FMan in order to drop the frames
 * before they reach our ingress queues and eat up memory.
 */
static int dpaa_eth_priv_ingress_cgr_init(struct dpa_priv *priv)
{
	struct qm_mcc_initcgr initcgr;
	u32 cs_th;
	int err;

	err = qman_alloc_cgrid(&priv->ingress_cgr.cgrid);
	if (err < 0) {
		if (netif_msg_drv(priv))
			pr_err("Error %d allocating CGR ID\n", err);
		goto out_error;
	}

	/* Enable CS TD, but disable Congestion State Change Notifications. */
	initcgr.we_mask = QM_CGR_WE_CS_THRES;
	initcgr.cgr.cscn_en = QM_CGR_EN;
	cs_th = DPAA_INGRESS_CS_THRESHOLD;
	qm_cgr_cs_thres_set64(&initcgr.cgr.cs_thres, cs_th, 1);

	initcgr.we_mask |= QM_CGR_WE_CSTD_EN;
	initcgr.cgr.cstd_en = QM_CGR_EN;

	/* This is actually a hack, because this CGR will be associated with
	 * our affine SWP. However, we'll place our ingress FQs in it.
	 */
	err = qman_create_cgr(&priv->ingress_cgr, QMAN_CGR_FLAG_USE_INIT,
			      &initcgr);
	if (err < 0) {
		if (netif_msg_drv(priv))
			pr_err("Error %d creating ingress CGR with ID %d\n",
			       err, priv->ingress_cgr.cgrid);
		qman_release_cgrid(priv->ingress_cgr.cgrid);
		goto out_error;
	}
	if (netif_msg_drv(priv))
		pr_debug("Created ingress CGR %d for netdev with hwaddr %pM\n",
			 priv->ingress_cgr.cgrid, priv->mac_dev->addr);

	priv->use_ingress_cgr = true;

out_error:
	return err;
}

static int dpa_priv_bp_create(struct net_device *net_dev, struct dpa_bp *dpa_bp,
			      size_t count)
{
	struct dpa_priv *priv = netdev_priv(net_dev);
	int i;

	priv->bp_count = count;

	for (i = 0; i < count; i++) {
		int err;

		err = dpa_bp_alloc(&dpa_bp[i]);
		if (err < 0) {
			dpa_bp_free(priv);
			priv->dpa_bp = NULL;
			return err;
		}

		priv->dpa_bp = &dpa_bp[i];
	}

	dpa_common_bpid = priv->dpa_bp->bpid;
	return 0;
}

static const struct of_device_id dpa_match[];

static int dpaa_eth_probe(struct platform_device *pdev)
{
	int err = 0, i, channel;
	struct device *dev;
	struct dpa_bp *dpa_bp;
	struct dpa_fq *dpa_fq, *tmp;
	size_t count = 1;
	struct net_device *net_dev = NULL;
	struct dpa_priv *priv = NULL;
	struct dpa_percpu_priv *percpu_priv;
	struct fm_port_fqs port_fqs;
	struct mac_device *mac_dev;
	struct task_struct *kth;

	dev = &pdev->dev;

	/* Get the buffer pool assigned to this interface;
	 * run only once the default pool probing code
	 */
	dpa_bp = (dpa_bpid2pool(dpa_common_bpid));
	if (!dpa_bp)
		dpa_bp = dpa_priv_bp_probe(dev);
	if (IS_ERR(dpa_bp))
		return PTR_ERR(dpa_bp);

	/* Allocate this early, so we can store relevant information in
	 * the private area
	 */
	net_dev = alloc_etherdev_mq(sizeof(*priv), DPAA_ETH_TX_QUEUES);
	if (!net_dev) {
		dev_err(dev, "alloc_etherdev_mq() failed\n");
		goto alloc_etherdev_mq_failed;
	}

#ifdef CONFIG_FSL_DPAA_ETH_FRIENDLY_IF_NAME
	snprintf(net_dev->name, IFNAMSIZ, "fm%d-mac%d",
		 dpa_mac_fman_index_get(pdev),
		 dpa_mac_hw_index_get(pdev));
#endif

	/* Do this here, so we can be verbose early */
	SET_NETDEV_DEV(net_dev, dev);
	dev_set_drvdata(dev, net_dev);

	priv = netdev_priv(net_dev);
	priv->net_dev = net_dev;

	priv->msg_enable = netif_msg_init(debug, DPAA_MSG_DEFAULT);

	mac_dev = dpa_mac_dev_get(pdev);
	if (IS_ERR(mac_dev)) {
		err = PTR_ERR(mac_dev);
		goto mac_probe_failed;
	}

	/* We have physical ports, so we need to establish
	 * the buffer layout.
	 */
	dpa_set_buffers_layout(mac_dev, &priv->buf_layout[0]);

	/* compute the size of the buffers used for reception */
	dpa_bp->size = dpa_bp_size();

	INIT_LIST_HEAD(&priv->dpa_fq_list);

	memset(&port_fqs, 0, sizeof(port_fqs));

	err = dpa_fq_probe_mac(dev, &priv->dpa_fq_list, &port_fqs, true, RX);
	if (!err)
		err = dpa_fq_probe_mac(dev, &priv->dpa_fq_list,
				       &port_fqs, true, TX);

	if (err < 0)
		goto fq_probe_failed;

	/* bp init */

	err = dpa_priv_bp_create(net_dev, dpa_bp, count);

	if (err < 0)
		goto bp_create_failed;

	priv->mac_dev = mac_dev;

	channel = dpa_get_channel();

	if (channel < 0) {
		err = channel;
		goto get_channel_failed;
	}

	priv->channel = (u16)channel;

	/* Start a thread that will walk the cpus with affine portals
	 * and add this pool channel to each's dequeue mask.
	 */
	kth = kthread_run(dpaa_eth_add_channel,
			  (void *)(unsigned long)priv->channel,
			  "dpaa_%p:%d", net_dev, priv->channel);
	if (!kth) {
		err = -ENOMEM;
		goto add_channel_failed;
	}

	dpa_fq_setup(priv, &dpaa_fq_cbs, priv->mac_dev->port[TX]);

	/* Create a congestion group for this netdev, with
	 * dynamically-allocated CGR ID.
	 * Must be executed after probing the MAC, but before
	 * assigning the egress FQs to the CGRs.
	 */
	err = dpaa_eth_cgr_init(priv);
	if (err < 0) {
		dev_err(dev, "Error initializing CGR\n");
		goto tx_cgr_init_failed;
	}
	err = dpaa_eth_priv_ingress_cgr_init(priv);
	if (err < 0) {
		dev_err(dev, "Error initializing ingress CGR\n");
		goto rx_cgr_init_failed;
	}

	/* Add the FQs to the interface, and make them active */
	list_for_each_entry_safe(dpa_fq, tmp, &priv->dpa_fq_list, list) {
		err = dpa_fq_init(dpa_fq, false);
		if (err < 0)
			goto fq_alloc_failed;
	}

	priv->tx_headroom = dpa_get_headroom(&priv->buf_layout[TX]);
	priv->rx_headroom = dpa_get_headroom(&priv->buf_layout[RX]);

	/* All real interfaces need their ports initialized */
	dpaa_eth_init_ports(mac_dev, dpa_bp, count, &port_fqs,
			    &priv->buf_layout[0], dev);

	priv->percpu_priv = devm_alloc_percpu(dev, *priv->percpu_priv);

	if (!priv->percpu_priv) {
		dev_err(dev, "devm_alloc_percpu() failed\n");
		err = -ENOMEM;
		goto alloc_percpu_failed;
	}
	for_each_possible_cpu(i) {
		percpu_priv = per_cpu_ptr(priv->percpu_priv, i);
		memset(percpu_priv, 0, sizeof(*percpu_priv));
	}

	/* Initialize NAPI */
	err = dpa_napi_add(net_dev);

	if (err < 0)
		goto napi_add_failed;

	err = dpa_netdev_init(net_dev, &dpaa_ops, tx_timeout);

	if (err < 0)
		goto netdev_init_failed;

	dpaa_eth_sysfs_init(&net_dev->dev);

	netif_info(priv, probe, net_dev, "Probed interface %s\n",
		   net_dev->name);

	return 0;

netdev_init_failed:
napi_add_failed:
	dpa_napi_del(net_dev);
alloc_percpu_failed:
	dpa_fq_free(dev, &priv->dpa_fq_list);
fq_alloc_failed:
	qman_delete_cgr_safe(&priv->ingress_cgr);
	qman_release_cgrid(priv->ingress_cgr.cgrid);
rx_cgr_init_failed:
	qman_delete_cgr_safe(&priv->cgr_data.cgr);
	qman_release_cgrid(priv->cgr_data.cgr.cgrid);
tx_cgr_init_failed:
add_channel_failed:
get_channel_failed:
	dpa_bp_free(priv);
bp_create_failed:
fq_probe_failed:
mac_probe_failed:
	dev_set_drvdata(dev, NULL);
	free_netdev(net_dev);
alloc_etherdev_mq_failed:
	if (atomic_read(&dpa_bp->refs) == 0)
		devm_kfree(dev, dpa_bp);

	return err;
}

static struct platform_device_id dpa_devtype[] = {
	{
		.name = "dpaa-ethernet",
		.driver_data = 0,
	}, {
	}
};
MODULE_DEVICE_TABLE(platform, dpa_devtype);

static struct platform_driver dpa_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.id_table = dpa_devtype,
	.probe = dpaa_eth_probe,
	.remove = dpa_remove
};

static int __init dpa_load(void)
{
	int err;

	pr_debug("FSL DPAA Ethernet driver\n");

	/* initialise dpaa_eth mirror values */
	dpa_rx_extra_headroom = fman_get_rx_extra_headroom();
	dpa_max_frm = fman_get_max_frm();

	err = platform_driver_register(&dpa_driver);
	if (err < 0)
		pr_err("Error, platform_driver_register() = %d\n", err);

	return err;
}
module_init(dpa_load);

static void __exit dpa_unload(void)
{
	platform_driver_unregister(&dpa_driver);

	/* Only one channel is used and needs to be relased after all
	 * interfaces are removed
	 */
	dpa_release_channel();
}
module_exit(dpa_unload);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("FSL DPAA Ethernet driver");
