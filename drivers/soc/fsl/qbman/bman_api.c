/* Copyright 2008 - 2015 Freescale Semiconductor, Inc.
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

#include "bman.h"

/* Compilation constants */
#define IRQNAME		"BMan portal %d"
#define MAX_IRQNAME	16	/* big enough for "BMan portal %d" */

struct bman_portal {
	struct bm_portal p;
	/* 2-element array. pools[0] is mask, pools[1] is snapshot. */
	struct bman_depletion *pools;
	int thresh_set;
	unsigned long irq_sources;
	struct bman_pool *rcri_owned; /* only 1 release WAIT_SYNC at a time */
	raw_spinlock_t sharing_lock; /* only used if is_shared */
	int is_shared;
	struct bman_portal *sharing_redirect;
	/* When the cpu-affine portal is activated, this is non-NULL */
	const struct bm_portal_config *config;
	/*
	 * 64-entry hash-table of pool objects that are tracking depletion
	 * entry/exit (ie. BMAN_POOL_FLAG_DEPLETION). This isn't fast-path, so
	 * we're not fussy about cache-misses and so forth - whereas the above
	 * members should all fit in one cacheline.
	 * BTW, with 64 entries in the hash table and 64 buffer pools to track,
	 * you'll never guess the hash-function ...
	 */
	struct bman_pool *cb[64];
	char irqname[MAX_IRQNAME];
	/* Track if the portal was alloced by the driver */
	bool alloced;
};

/*
 * For an explanation of the locking, redirection, or affine-portal logic,
 * please consult the QMan driver for details. This is the same, only simpler
 * (no fiddly QMan-specific bits.)
 */
static inline void portal_irq_lock(struct bman_portal *p,
				   unsigned long *irqflags)
{
	if (p->is_shared)
		raw_spin_lock_irqsave(&p->sharing_lock, *irqflags);
	else
		local_irq_save(*irqflags);
}

static inline void portal_irq_unlock(struct bman_portal *p,
				     unsigned long irqflags)
{
	if (p->is_shared)
		raw_spin_unlock_irqrestore(&p->sharing_lock,
					   irqflags);
	else
		local_irq_restore(irqflags);
}

static cpumask_t affine_mask;
static DEFINE_SPINLOCK(affine_mask_lock);
static DEFINE_PER_CPU(struct bman_portal, bman_affine_portal);
static inline struct bman_portal *get_raw_affine_portal(void)
{
	return &get_cpu_var(bman_affine_portal);
}
static inline struct bman_portal *get_affine_portal(void)
{
	struct bman_portal *p = get_raw_affine_portal();

	if (p->sharing_redirect)
		return p->sharing_redirect;
	return p;
}
static inline void put_affine_portal(void)
{
	put_cpu_var(bman_affine_portal);
}

/*
 * This object type refers to a pool, it isn't *the* pool. There may be
 * more than one such object per BMan buffer pool, eg. if different users of the
 * pool are operating via different portals.
 */
struct bman_pool {
	struct bman_pool_params params;
	/* Used for hash-table admin when using depletion notifications. */
	struct bman_portal *portal;
	struct bman_pool *next;
	/* stockpile state - NULL unless BMAN_POOL_FLAG_STOCKPILE is set */
	struct bm_buffer *sp;
	unsigned int sp_fill;
#ifdef CONFIG_FSL_DPA_CHECKING
	atomic_t in_use;
#endif
};

/* (De)Registration of depletion notification callbacks */
static void depletion_link(struct bman_portal *portal, struct bman_pool *pool)
{
	unsigned long irqflags = 0;

	pool->portal = portal;
	portal_irq_lock(portal, &irqflags);
	pool->next = portal->cb[pool->params.bpid];
	portal->cb[pool->params.bpid] = pool;
	if (!pool->next)
		/*
		 * First object for that bpid on this portal, enable the BSCN
		 * mask bit.
		 */
		bm_isr_bscn_mask(&portal->p, pool->params.bpid, 1);
	portal_irq_unlock(portal, irqflags);
}
static void depletion_unlink(struct bman_pool *pool)
{
	struct bman_pool *it, *last = NULL;
	struct bman_pool **base = &pool->portal->cb[pool->params.bpid];
	unsigned long irqflags = 0;

	portal_irq_lock(pool->portal, &irqflags);
	it = *base;	/* <-- gotcha, don't do this prior to the irq_save */
	while (it != pool) {
		last = it;
		it = it->next;
	}
	if (!last)
		*base = pool->next;
	else
		last->next = pool->next;
	if (!last && !pool->next) {
		/*
		 * Last object for that bpid on this portal, disable the BSCN
		 * mask bit.
		 */
		bm_isr_bscn_mask(&pool->portal->p, pool->params.bpid, 0);
		/* And "forget" that we last saw this pool as depleted */
		bman_depletion_unset(&pool->portal->pools[1],
					pool->params.bpid);
	}
	portal_irq_unlock(pool->portal, irqflags);
}

static u32 poll_portal_slow(struct bman_portal *p, u32 is);

/* Portal interrupt handler */
static irqreturn_t portal_isr(__always_unused int irq, void *ptr)
{
	struct bman_portal *p = ptr;
	struct bm_portal *portal = &p->p;
	u32 clear = p->irq_sources;
	u32 is = bm_in(portal, BM_REG_ISR) & p->irq_sources;

	clear |= poll_portal_slow(p, is);
	bm_out(portal, BM_REG_ISR, clear);
	return IRQ_HANDLED;
}

static int bm_rcr_init(struct bm_portal *portal, enum bm_rcr_pmode pmode,
		__maybe_unused enum bm_rcr_cmode cmode)
{
	struct bm_rcr *rcr = &portal->rcr;
	u32 cfg;
	u8 pi;

	rcr->ring = portal->addr.addr_ce + BM_CL_RCR;
	rcr->ci = bm_in(portal, BM_REG_RCR_CI_CINH) & (BM_RCR_SIZE - 1);
	pi = bm_in(portal, BM_REG_RCR_PI_CINH) & (BM_RCR_SIZE - 1);
	rcr->cursor = rcr->ring + pi;
	rcr->vbit = (bm_in(portal, BM_REG_RCR_PI_CINH) & BM_RCR_SIZE) ?
		BM_RCR_VERB_VBIT : 0;
	rcr->available = BM_RCR_SIZE - 1
		- bm_cyc_diff(BM_RCR_SIZE, rcr->ci, pi);
	rcr->ithresh = bm_in(portal, BM_REG_RCR_ITR);
#ifdef CONFIG_FSL_DPA_CHECKING
	rcr->busy = 0;
	rcr->pmode = pmode;
	rcr->cmode = cmode;
#endif
	cfg = (bm_in(portal, BM_REG_CFG) & 0xffffffe0)
		| (pmode & 0x3); /* BCSP_CFG::RPM */
	bm_out(portal, BM_REG_CFG, cfg);
	return 0;
}

static void bm_rcr_finish(struct bm_portal *portal)
{
	struct bm_rcr *rcr = &portal->rcr;
	u8 pi = bm_in(portal, BM_REG_RCR_PI_CINH) & (BM_RCR_SIZE - 1);
	u8 ci = bm_in(portal, BM_REG_RCR_CI_CINH) & (BM_RCR_SIZE - 1);

	DPA_ASSERT(!rcr->busy);
	if (pi != rcr_ptr2idx(rcr->cursor))
		pr_crit("losing uncommited RCR entries\n");
	if (ci != rcr->ci)
		pr_crit("missing existing RCR completions\n");
	if (rcr->ci != rcr_ptr2idx(rcr->cursor))
		pr_crit("RCR destroyed unquiesced\n");
}

/* --- Management command API --- */
static int bm_mc_init(struct bm_portal *portal)
{
	struct bm_mc *mc = &portal->mc;

	mc->cr = portal->addr.addr_ce + BM_CL_CR;
	mc->rr = portal->addr.addr_ce + BM_CL_RR0;
	mc->rridx = (__raw_readb(&mc->cr->__dont_write_directly__verb) &
			BM_MCC_VERB_VBIT) ?  0 : 1;
	mc->vbit = mc->rridx ? BM_MCC_VERB_VBIT : 0;
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = mc_idle;
#endif
	return 0;
}

static void bm_mc_finish(struct bm_portal *portal)
{
	__maybe_unused register struct bm_mc *mc = &portal->mc;

	DPA_ASSERT(mc->state == mc_idle);
#ifdef CONFIG_FSL_DPA_CHECKING
	if (mc->state != mc_idle)
		pr_crit("Losing incomplete MC command\n");
#endif
}

static inline struct bm_mc_command *bm_mc_start(struct bm_portal *portal)
{
	register struct bm_mc *mc = &portal->mc;

	DPA_ASSERT(mc->state == mc_idle);
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = mc_user;
#endif
	dpaa_zero(mc->cr);
	return mc->cr;
}

static inline void bm_mc_abort(struct bm_portal *portal)
{
	__maybe_unused register struct bm_mc *mc = &portal->mc;

	DPA_ASSERT(mc->state == mc_user);
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = mc_idle;
#endif
}

static inline void bm_mc_commit(struct bm_portal *portal, u8 myverb)
{
	register struct bm_mc *mc = &portal->mc;
	struct bm_mc_result *rr = mc->rr + mc->rridx;

	DPA_ASSERT(mc->state == mc_user);
	dma_wmb();
	mc->cr->__dont_write_directly__verb = myverb | mc->vbit;
	dpaa_flush(mc->cr);
	dpaa_invalidate_touch_ro(rr);
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = mc_hw;
#endif
}

static inline struct bm_mc_result *bm_mc_result(struct bm_portal *portal)
{
	register struct bm_mc *mc = &portal->mc;
	struct bm_mc_result *rr = mc->rr + mc->rridx;

	DPA_ASSERT(mc->state == mc_hw);
	/*
	 * The inactive response register's verb byte always returns zero until
	 * its command is submitted and completed. This includes the valid-bit,
	 * in case you were wondering...
	 */
	if (!__raw_readb(&rr->verb)) {
		dpaa_invalidate_touch_ro(rr);
		return NULL;
	}
	mc->rridx ^= 1;
	mc->vbit ^= BM_MCC_VERB_VBIT;
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = mc_idle;
#endif
	return rr;
}


struct bman_portal *bman_create_portal(
				       struct bman_portal *portal,
				       const struct bm_portal_config *config)
{
	struct bm_portal *p;
	const struct bman_depletion *pools = &config->public_cfg.mask;
	int ret;

	if (!portal) {
		portal = kmalloc(sizeof(*portal), GFP_KERNEL);
		if (!portal)
			return portal;
		portal->alloced = 1;
	} else
		portal->alloced = 0;

	p = &portal->p;

	/*
	 * prep the low-level portal struct with the mapped addresses from the
	 * config, everything that follows depends on it and "config" is more
	 * for (de)reference...
	 */
	p->addr.addr_ce = config->addr_virt[DPA_PORTAL_CE];
	p->addr.addr_ci = config->addr_virt[DPA_PORTAL_CI];
	if (bm_rcr_init(p, bm_rcr_pvb, bm_rcr_cce)) {
		pr_err("RCR initialisation failed\n");
		goto fail_rcr;
	}
	if (bm_mc_init(p)) {
		pr_err("MC initialisation failed\n");
		goto fail_mc;
	}
	portal->pools = kmalloc(2 * sizeof(*pools), GFP_KERNEL);
	if (!portal->pools)
		goto fail_pools;
	portal->pools[0] = *pools;
	bman_depletion_init(portal->pools + 1);
	/*
	 * Default to all BPIDs disabled, we enable as required at
	 * run-time.
	 */
	bm_isr_bscn_disable(p);

	portal->rcri_owned = NULL;
	raw_spin_lock_init(&portal->sharing_lock);
	portal->is_shared = config->public_cfg.is_shared;
	portal->sharing_redirect = NULL;
	memset(&portal->cb, 0, sizeof(portal->cb));
	/* Write-to-clear any stale interrupt status bits */
	bm_out(p, BM_REG_ISDR, 0xffffffff);
	portal->irq_sources = 0;
	bm_out(p, BM_REG_IER, portal->irq_sources);
	bm_out(p, BM_REG_ISR, 0xffffffff);
	snprintf(portal->irqname, MAX_IRQNAME, IRQNAME, config->public_cfg.cpu);
	if (request_irq(config->public_cfg.irq, portal_isr, 0, portal->irqname,
			portal)) {
		pr_err("request_irq() failed\n");
		goto fail_irq;
	}
	if ((config->public_cfg.cpu != -1) &&
			irq_can_set_affinity(config->public_cfg.irq) &&
			irq_set_affinity(config->public_cfg.irq,
				cpumask_of(config->public_cfg.cpu))) {
		pr_err("irq_set_affinity() failed\n");
		goto fail_affinity;
	}

	/* Need RCR to be empty before continuing */
	ret = bm_rcr_get_fill(p);
	if (ret) {
		pr_err("RCR unclean\n");
		goto fail_rcr_empty;
	}
	/* Success */
	portal->config = config;

	bm_out(p, BM_REG_ISDR, 0);
	bm_out(p, BM_REG_IIR, 0);
	return portal;
fail_rcr_empty:
fail_affinity:
	free_irq(config->public_cfg.irq, portal);
fail_irq:
	kfree(portal->pools);
fail_pools:
	bm_mc_finish(p);
fail_mc:
	bm_rcr_finish(p);
fail_rcr:
	if (portal->alloced)
		kfree(portal);
	return NULL;
}

struct bman_portal *bman_create_affine_portal(
			const struct bm_portal_config *config)
{
	struct bman_portal *portal;

	portal = &per_cpu(bman_affine_portal, config->public_cfg.cpu);
	portal = bman_create_portal(portal, config);
	if (portal) {
		spin_lock(&affine_mask_lock);
		cpumask_set_cpu(config->public_cfg.cpu, &affine_mask);
		spin_unlock(&affine_mask_lock);
	}
	return portal;
}


struct bman_portal *bman_create_affine_slave(struct bman_portal *redirect,
								int cpu)
{
	struct bman_portal *p = &per_cpu(bman_affine_portal, cpu);

	if (p->config || p->is_shared ||
	    !redirect->config->public_cfg.is_shared)
		return NULL;
	p->irq_sources = 0;
	p->sharing_redirect = redirect;
	put_affine_portal();
	return p;
}

void bman_destroy_portal(struct bman_portal *bm)
{
	const struct bm_portal_config *pcfg = bm->config;

	bm_rcr_cce_update(&bm->p);
	bm_rcr_cce_update(&bm->p);

	free_irq(pcfg->public_cfg.irq, bm);

	kfree(bm->pools);
	bm_mc_finish(&bm->p);
	bm_rcr_finish(&bm->p);
	bm->config = NULL;
	if (bm->alloced)
		kfree(bm);
}

const struct bm_portal_config *bman_destroy_affine_portal(void)
{
	struct bman_portal *bm = get_raw_affine_portal();
	const struct bm_portal_config *pcfg;

	if (bm->sharing_redirect) {
		bm->sharing_redirect = NULL;
		put_affine_portal();
		return NULL;
	}
	bm->is_shared = 0;
	pcfg = bm->config;
	bman_destroy_portal(bm);
	spin_lock(&affine_mask_lock);
	cpumask_clear_cpu(pcfg->public_cfg.cpu, &affine_mask);
	spin_unlock(&affine_mask_lock);
	put_affine_portal();
	return pcfg;
}

/*
 * When release logic waits on available RCR space, we need a global waitqueue
 * in the case of "affine" use (as the waits wake on different cpus which means
 * different portals - so we can't wait on any per-portal waitqueue).
 */
static DECLARE_WAIT_QUEUE_HEAD(affine_queue);

static u32 poll_portal_slow(struct bman_portal *p, u32 is)
{
	struct bman_depletion tmp;
	u32 ret = is;
	int timeout = 1000000;

	/*
	 * There is a gotcha to be aware of. If we do the query before clearing
	 * the status register, we may miss state changes that occur between the
	 * two. If we write to clear the status register before the query, the
	 * cache-enabled query command may overtake the status register write
	 * unless we use a heavyweight sync (which we don't want). Instead, we
	 * write-to-clear the status register then *read it back* before doing
	 * the query, hence the odd while loop with the 'is' accumulation.
	 */
	if (is & BM_PIRQ_BSCN) {
		struct bm_mc_result *mcr;
		unsigned long irqflags = 0;
		unsigned int i, j;
		u32 __is;

		bm_out(&p->p, BM_REG_ISR, BM_PIRQ_BSCN);
		while ((__is = bm_in(&p->p, BM_REG_ISR)) & BM_PIRQ_BSCN) {
			is |= __is;
			bm_out(&p->p, BM_REG_ISR, BM_PIRQ_BSCN);
		}
		is &= ~BM_PIRQ_BSCN;
		portal_irq_lock(p, &irqflags);
		bm_mc_start(&p->p);
		bm_mc_commit(&p->p, BM_MCC_VERB_CMD_QUERY);
		while (!(mcr = bm_mc_result(&p->p)) && timeout--)
			cpu_relax();
		if (timeout == 0) {
			pr_crit("BMan Query BSCN Timeout\n");
			return 0;
		}
		tmp = mcr->query.ds.state;
		portal_irq_unlock(p, irqflags);
		for (i = 0; i < 2; i++) {
			int idx = i * 32;
			/*
			 * tmp is a mask of currently-depleted pools.
			 * pools[0] is mask of those we care about.
			 * pools[1] is our previous view (we only want to
			 * be told about changes).
			 */
			tmp.state[i] &= p->pools[0].state[i];
			if (tmp.state[i] == p->pools[1].state[i])
				/* fast-path, nothing to see, move along */
				continue;
			for (j = 0; j <= 31; j++, idx++) {
				struct bman_pool *pool = p->cb[idx];
				int b4 = bman_depletion_get(&p->pools[1], idx);
				int af = bman_depletion_get(&tmp, idx);

				if (b4 == af)
					continue;
				while (pool) {
					pool->params.cb(p, pool,
						pool->params.cb_ctx, af);
					pool = pool->next;
				}
			}
		}
		p->pools[1] = tmp;
	}

	if (is & BM_PIRQ_RCRI) {
		unsigned long irqflags = 0;

		portal_irq_lock(p, &irqflags);
		bm_rcr_cce_update(&p->p);
		/*
		 * If waiting for sync, we only cancel the interrupt threshold
		 * when the ring utilisation hits zero.
		 */
		if (p->rcri_owned) {
			if (!bm_rcr_get_fill(&p->p)) {
				p->rcri_owned = NULL;
				bm_rcr_set_ithresh(&p->p, 0);
			}
		} else
			bm_rcr_set_ithresh(&p->p, 0);
		portal_irq_unlock(p, irqflags);
		wake_up(&affine_queue);
		bm_out(&p->p, BM_REG_ISR, BM_PIRQ_RCRI);
		is &= ~BM_PIRQ_RCRI;
	}

	/* There should be no status register bits left undefined */
	DPA_ASSERT(!is);
	return ret;
}

const struct bman_portal_config *bman_get_portal_config(void)
{
	struct bman_portal *p = get_affine_portal();
	const struct bman_portal_config *ret = &p->config->public_cfg;

	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(bman_get_portal_config);
int bman_p_irqsource_add(struct bman_portal *p, __maybe_unused u32 bits)
{
	unsigned long irqflags = 0;

	if (p->sharing_redirect)
		return -EINVAL;
	portal_irq_lock(p, &irqflags);
	set_bits(bits & BM_PIRQ_VISIBLE, &p->irq_sources);
	bm_out(&p->p, BM_REG_IER, p->irq_sources);
	portal_irq_unlock(p, irqflags);
	return 0;
}

const cpumask_t *bman_affine_cpus(void)
{
	return &affine_mask;
}
EXPORT_SYMBOL(bman_affine_cpus);

static const u32 zero_thresholds[4] = {0, 0, 0, 0};

struct bman_pool *bman_new_pool(const struct bman_pool_params *params)
{
	struct bman_pool *pool = NULL;
	u32 bpid;

	if (params->flags & BMAN_POOL_FLAG_DYNAMIC_BPID) {
		if (bman_alloc_bpid(&bpid))
			return NULL;
	} else
		bpid = params->bpid;
#ifdef CONFIG_FSL_BMAN
	if (params->flags & BMAN_POOL_FLAG_THRESH) {
		if (bm_pool_set(bpid, params->thresholds))
			goto err;
	}
#else
	if (params->flags & BMAN_POOL_FLAG_THRESH)
		goto err;
#endif
	pool = kmalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool)
		goto err;
	pool->sp = NULL;
	pool->sp_fill = 0;
	pool->params = *params;
#ifdef CONFIG_FSL_DPA_CHECKING
	atomic_set(&pool->in_use, 1);
#endif
	if (params->flags & BMAN_POOL_FLAG_DYNAMIC_BPID)
		pool->params.bpid = bpid;
	if (params->flags & BMAN_POOL_FLAG_STOCKPILE) {
		pool->sp = kmalloc(sizeof(struct bm_buffer) * BMAN_STOCKPILE_SZ,
					GFP_KERNEL);
		if (!pool->sp)
			goto err;
	}
	if (pool->params.flags & BMAN_POOL_FLAG_DEPLETION) {
		struct bman_portal *p = get_affine_portal();

		if (!p->pools || !bman_depletion_get(&p->pools[0], bpid)) {
			pr_err("Depletion events disabled for bpid %d\n", bpid);
			goto err;
		}
		depletion_link(p, pool);
		put_affine_portal();
	}
	return pool;
err:
#ifdef CONFIG_FSL_BMAN
	if (params->flags & BMAN_POOL_FLAG_THRESH)
		bm_pool_set(bpid, zero_thresholds);
#endif
	if (params->flags & BMAN_POOL_FLAG_DYNAMIC_BPID)
		bman_release_bpid(bpid);
	if (pool) {
		kfree(pool->sp);
		kfree(pool);
	}
	return NULL;
}
EXPORT_SYMBOL(bman_new_pool);

void bman_free_pool(struct bman_pool *pool)
{
#ifdef CONFIG_FSL_BMAN
	if (pool->params.flags & BMAN_POOL_FLAG_THRESH)
		bm_pool_set(pool->params.bpid, zero_thresholds);
#endif
	if (pool->params.flags & BMAN_POOL_FLAG_DEPLETION)
		depletion_unlink(pool);
	if (pool->params.flags & BMAN_POOL_FLAG_STOCKPILE) {
		if (pool->sp_fill)
			pr_err("Stockpile not flushed, has %u in bpid %u.\n",
				pool->sp_fill, pool->params.bpid);
		kfree(pool->sp);
		pool->sp = NULL;
		pool->params.flags ^= BMAN_POOL_FLAG_STOCKPILE;
	}
	if (pool->params.flags & BMAN_POOL_FLAG_DYNAMIC_BPID)
		bman_release_bpid(pool->params.bpid);
	kfree(pool);
}
EXPORT_SYMBOL(bman_free_pool);

const struct bman_pool_params *bman_get_params(const struct bman_pool *pool)
{
	return &pool->params;
}
EXPORT_SYMBOL(bman_get_params);

static noinline void update_rcr_ci(struct bman_portal *p, u8 avail)
{
	if (avail)
		bm_rcr_cce_prefetch(&p->p);
	else
		bm_rcr_cce_update(&p->p);
}

int bman_rcr_is_empty(void)
{
	unsigned long irqflags = 0;
	struct bman_portal *p = get_affine_portal();
	u8 avail;

	portal_irq_lock(p, &irqflags);
	update_rcr_ci(p, 0);
	avail = bm_rcr_get_fill(&p->p);
	portal_irq_unlock(p, irqflags);
	put_affine_portal();
	return avail == 0;
}
EXPORT_SYMBOL(bman_rcr_is_empty);

static inline struct bm_rcr_entry *try_rel_start(struct bman_portal **p,
					__maybe_unused struct bman_pool *pool,
					__maybe_unused unsigned long *irqflags,
					__maybe_unused u32 flags)
{
	struct bm_rcr_entry *r;
	u8 avail;

	*p = get_affine_portal();
	portal_irq_lock(*p, irqflags);
	if (unlikely((flags & BMAN_RELEASE_FLAG_WAIT) &&
			(flags & BMAN_RELEASE_FLAG_WAIT_SYNC))) {
		if ((*p)->rcri_owned) {
			portal_irq_unlock(*p, (*irqflags));
			put_affine_portal();
			return NULL;
		}
		(*p)->rcri_owned = pool;
	}
	avail = bm_rcr_get_avail(&(*p)->p);
	if (avail < 2)
		update_rcr_ci(*p, avail);
	r = bm_rcr_start(&(*p)->p);
	if (unlikely(!r)) {
		if (unlikely((flags & BMAN_RELEASE_FLAG_WAIT) &&
				(flags & BMAN_RELEASE_FLAG_WAIT_SYNC)))
			(*p)->rcri_owned = NULL;
		portal_irq_unlock(*p, (*irqflags));
		put_affine_portal();
	}
	return r;
}

static noinline struct bm_rcr_entry *__wait_rel_start(struct bman_portal **p,
					struct bman_pool *pool,
					__maybe_unused unsigned long *irqflags,
					u32 flags)
{
	struct bm_rcr_entry *rcr = try_rel_start(p, pool, irqflags, flags);

	if (!rcr)
		bm_rcr_set_ithresh(&(*p)->p, 1);
	return rcr;
}

static noinline struct bm_rcr_entry *wait_rel_start(struct bman_portal **p,
					struct bman_pool *pool,
					__maybe_unused unsigned long *irqflags,
					u32 flags)
{
	struct bm_rcr_entry *rcr;

	pool = NULL;
	if (flags & BMAN_RELEASE_FLAG_WAIT_INT)
		wait_event_interruptible(affine_queue,
			(rcr = __wait_rel_start(p, pool, irqflags, flags)));
	else
		wait_event(affine_queue,
			(rcr = __wait_rel_start(p, pool, irqflags, flags)));
	return rcr;
}

/*
 * to facilitate better copying of bufs into the ring without either (a) copying
 * noise into the first byte (prematurely triggering the command), nor (b) being
 * very inefficient by copying small fields using read-modify-write
 */
struct overlay_bm_buffer {
	u32 first;
	u32 second;
};

static inline int __bman_release(struct bman_pool *pool,
			const struct bm_buffer *bufs, u8 num, u32 flags)
{
	struct bman_portal *p;
	struct bm_rcr_entry *r;
	struct overlay_bm_buffer *o_dest;
	struct overlay_bm_buffer *o_src = (struct overlay_bm_buffer *)&bufs[0];
	__maybe_unused unsigned long irqflags;
	u32 i = num - 1;

	if (flags & BMAN_RELEASE_FLAG_WAIT)
		r = wait_rel_start(&p, pool, &irqflags, flags);
	else
		r = try_rel_start(&p, pool, &irqflags, flags);
	if (!r)
		return -EBUSY;
	/*
	 * We can copy all but the first entry, as this can trigger badness
	 * with the valid-bit. Use the overlay to mask the verb byte.
	 */
	o_dest = (struct overlay_bm_buffer *)&r->bufs[0];
	o_dest->first = (o_src->first & 0x0000ffff) |
		(((u32)pool->params.bpid << 16) & 0x00ff0000);
	o_dest->second = o_src->second;
	if (i)
		memcpy(&r->bufs[1], &bufs[1], i * sizeof(bufs[0]));
	bm_rcr_pvb_commit(&p->p, BM_RCR_VERB_CMD_BPID_SINGLE |
			(num & BM_RCR_VERB_BUFCOUNT_MASK));

	if (unlikely((flags & BMAN_RELEASE_FLAG_WAIT) &&
			(flags & BMAN_RELEASE_FLAG_WAIT_SYNC))) {
		/*
		 * if we wish to sync we need to set the threshold after
		 * h/w sees the new ring entry. As we're mixing cache-enabled
		 * and cache-inhibited accesses, this requires a heavy-weight
		 * sync.
		 */
		mb();
		bm_rcr_set_ithresh(&p->p, 1);
	}
	portal_irq_unlock(p, irqflags);
	put_affine_portal();
	if (unlikely((flags & BMAN_RELEASE_FLAG_WAIT) &&
			(flags & BMAN_RELEASE_FLAG_WAIT_SYNC))) {
		if (flags & BMAN_RELEASE_FLAG_WAIT_INT)
			wait_event_interruptible(affine_queue,
					(p->rcri_owned != pool));
		else
			wait_event(affine_queue, (p->rcri_owned != pool));
	}
	return 0;
}

int bman_release(struct bman_pool *pool, const struct bm_buffer *bufs, u8 num,
			u32 flags)
{
	int ret = 0;

#ifdef CONFIG_FSL_DPA_CHECKING
	if (!num || (num > 8))
		return -EINVAL;
	if (pool->params.flags & BMAN_POOL_FLAG_NO_RELEASE)
		return -EINVAL;
#endif
	/* Without stockpile, this API is a pass-through to the h/w operation */
	if (!(pool->params.flags & BMAN_POOL_FLAG_STOCKPILE))
		return __bman_release(pool, bufs, num, flags);
#ifdef CONFIG_FSL_DPA_CHECKING
	if (!atomic_dec_and_test(&pool->in_use))
		pr_crit("Parallel attempts to enter bman_released() detected.");
#endif
	/*
	 * This needs some explanation. Adding the given buffers may take the
	 * stockpile over the threshold, but in fact the stockpile may already
	 * *be* over the threshold if a previous release-to-hw attempt had
	 * failed. So we have 3 cases to cover;
	 *   1. we add to the stockpile and don't hit the threshold,
	 *   2. we add to the stockpile, hit the threshold and release-to-hw,
	 *   3. we have to release-to-hw before adding to the stockpile
	 *	(not enough room in the stockpile for case 2).
	 * Our constraints on thresholds guarantee that in case 3, there must be
	 * at least 8 bufs already in the stockpile, so all release-to-hw ops
	 * are for 8 bufs. Despite all this, the API must indicate whether the
	 * given buffers were taken off the caller's hands, irrespective of
	 * whether a release-to-hw was attempted.
	 */
	while (num) {
		/* Add buffers to stockpile if they fit */
		if ((pool->sp_fill + num) < BMAN_STOCKPILE_SZ) {
			memcpy(pool->sp + pool->sp_fill, bufs,
				sizeof(struct bm_buffer) * num);
			pool->sp_fill += num;
			num = 0; /* --> will return success no matter what */
		}
		/* Do hw op if hitting the high-water threshold */
		if ((pool->sp_fill + num) >= BMAN_STOCKPILE_HIGH) {
			ret = __bman_release(pool,
				pool->sp + (pool->sp_fill - 8), 8, flags);
			if (ret) {
				ret = (num ? ret : 0);
				goto release_done;
			}
			pool->sp_fill -= 8;
		}
	}
release_done:
#ifdef CONFIG_FSL_DPA_CHECKING
	atomic_inc(&pool->in_use);
#endif
	return ret;
}
EXPORT_SYMBOL(bman_release);

static inline int __bman_acquire(struct bman_pool *pool, struct bm_buffer *bufs,
					u8 num)
{
	struct bman_portal *p = get_affine_portal();
	struct bm_mc_command *mcc;
	struct bm_mc_result *mcr;
	unsigned long irqflags = 0;
	int ret, timeout = 1000000;

	portal_irq_lock(p, &irqflags);
	mcc = bm_mc_start(&p->p);
	mcc->acquire.bpid = pool->params.bpid;
	bm_mc_commit(&p->p, BM_MCC_VERB_CMD_ACQUIRE |
			(num & BM_MCC_VERB_ACQUIRE_BUFCOUNT));
	while (!(mcr = bm_mc_result(&p->p)) && timeout--)
		cpu_relax();
	if (timeout == 0) {
		pr_crit("BMan Acquire Timeout\n");
		portal_irq_unlock(p, irqflags);
		put_affine_portal();
		return -ETIMEDOUT;
	}
	ret = mcr->verb & BM_MCR_VERB_ACQUIRE_BUFCOUNT;
	if (bufs)
		memcpy(&bufs[0], &mcr->acquire.bufs[0],
				num * sizeof(bufs[0]));
	portal_irq_unlock(p, irqflags);
	put_affine_portal();
	if (ret != num)
		ret = -ENOMEM;
	return ret;
}

int bman_acquire(struct bman_pool *pool, struct bm_buffer *bufs, u8 num,
			u32 flags)
{
	int ret = 0;

#ifdef CONFIG_FSL_DPA_CHECKING
	if (!num || (num > 8))
		return -EINVAL;
	if (pool->params.flags & BMAN_POOL_FLAG_ONLY_RELEASE)
		return -EINVAL;
#endif
	/* Without stockpile, this API is a pass-through to the h/w operation */
	if (!(pool->params.flags & BMAN_POOL_FLAG_STOCKPILE))
		return __bman_acquire(pool, bufs, num);
#ifdef CONFIG_FSL_DPA_CHECKING
	if (!atomic_dec_and_test(&pool->in_use))
		pr_crit("Parallel attempts to enter bman_acquire() detected.");
#endif
	/* Only need a h/w op if we'll hit the low-water thresh */
	if (!(flags & BMAN_ACQUIRE_FLAG_STOCKPILE) &&
			(pool->sp_fill <= (BMAN_STOCKPILE_LOW + num))) {
		/*
		 * refill stockpile with max amount, but if max amount
		 * isn't available, try amount the user wants
		 */
		int bufcount = 8;

		ret = __bman_acquire(pool, pool->sp + pool->sp_fill, bufcount);
		if (ret < 0 && bufcount != num) {
			bufcount = num;
			/* Maybe buffer pool has less than 8 */
			ret = __bman_acquire(pool, pool->sp + pool->sp_fill,
						bufcount);
		}
		if (ret < 0)
			goto hw_starved;
		DPA_ASSERT(ret == bufcount);
		pool->sp_fill += bufcount;
	} else {
hw_starved:
		if (pool->sp_fill < num) {
			ret = -ENOMEM;
			goto acquire_done;
		}
	}
	memcpy(bufs, pool->sp + (pool->sp_fill - num),
		sizeof(struct bm_buffer) * num);
	pool->sp_fill -= num;
	ret = num;
acquire_done:
#ifdef CONFIG_FSL_DPA_CHECKING
	atomic_inc(&pool->in_use);
#endif
	return ret;
}
EXPORT_SYMBOL(bman_acquire);

int bman_flush_stockpile(struct bman_pool *pool, u32 flags)
{
	u8 num;
	int ret;

	while (pool->sp_fill) {
		num = ((pool->sp_fill > 8) ? 8 : pool->sp_fill);
		ret = __bman_release(pool, pool->sp + (pool->sp_fill - num),
				     num, flags);
		if (ret)
			return ret;
		pool->sp_fill -= num;
	}
	return 0;
}
EXPORT_SYMBOL(bman_flush_stockpile);

int bman_query_pools(struct bm_pool_state *state)
{
	struct bman_portal *p = get_affine_portal();
	struct bm_mc_result *mcr;
	unsigned long irqflags = 0;
	int timeout = 1000000;

	portal_irq_lock(p, &irqflags);
	bm_mc_start(&p->p);
	bm_mc_commit(&p->p, BM_MCC_VERB_CMD_QUERY);
	while (!(mcr = bm_mc_result(&p->p)) && timeout--)
		cpu_relax();
	if (timeout == 0) {
		pr_crit("BMan Query Pools timeout\n");
		portal_irq_unlock(p, irqflags);
		put_affine_portal();
		return -ETIMEDOUT;
	}
	DPA_ASSERT((mcr->verb & BM_MCR_VERB_CMD_MASK) == BM_MCR_VERB_CMD_QUERY);
	*state = mcr->query;
	portal_irq_unlock(p, irqflags);
	put_affine_portal();
	return 0;
}
EXPORT_SYMBOL(bman_query_pools);

#ifdef CONFIG_FSL_BMAN
u32 bman_query_free_buffers(struct bman_pool *pool)
{
	return bm_pool_free_buffers(pool->params.bpid);
}
EXPORT_SYMBOL(bman_query_free_buffers);

int bman_update_pool_thresholds(struct bman_pool *pool, const u32 *thresholds)
{
	u32 bpid;

	bpid = bman_get_params(pool)->bpid;

	return bm_pool_set(bpid, thresholds);
}
EXPORT_SYMBOL(bman_update_pool_thresholds);
#endif

int bman_shutdown_pool(u32 bpid)
{
	struct bman_portal *p = get_affine_portal();
	unsigned long irqflags = 0;
	int ret = 0;
	struct bm_mc_command *bm_cmd;
	struct bm_mc_result *bm_res;
	int aq_count = 0;
	bool stop = false;
	int timeout;

	portal_irq_lock(p, &irqflags);

	while (!stop) {
		/* Acquire buffers until empty */
		bm_cmd = bm_mc_start(&p->p);
		bm_cmd->acquire.bpid = bpid;
		bm_mc_commit(&p->p, BM_MCC_VERB_CMD_ACQUIRE |  1);
		timeout = 10000000; /* timeout if HW doesn't respond */
		while (!(bm_res = bm_mc_result(&p->p)) && timeout--)
			cpu_relax();
		if (timeout == 0)  {
			pr_crit("BMan Acquire Command timedout\n");
			stop = true;
			ret = -ETIMEDOUT;
		} else if (!(bm_res->verb & BM_MCR_VERB_ACQUIRE_BUFCOUNT)) {
			/* Pool is empty */
			stop = true;
		} else
			++aq_count;
	}

	portal_irq_unlock(p, irqflags);
	put_affine_portal();
	return ret;
}
EXPORT_SYMBOL(bman_shutdown_pool);

const struct bm_portal_config *
bman_get_bm_portal_config(const struct bman_portal *portal)
{
	return portal->sharing_redirect ? NULL : portal->config;
}

