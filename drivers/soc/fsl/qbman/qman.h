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

#include "qman_priv.h"

/* Portal register assists */

/* Cache-inhibited register offsets */
#define QM_REG_EQCR_PI_CINH	0x0000
#define QM_REG_EQCR_CI_CINH	0x0004
#define QM_REG_EQCR_ITR		0x0008
#define QM_REG_DQRR_PI_CINH	0x0040
#define QM_REG_DQRR_CI_CINH	0x0044
#define QM_REG_DQRR_ITR		0x0048
#define QM_REG_DQRR_DCAP	0x0050
#define QM_REG_DQRR_SDQCR	0x0054
#define QM_REG_DQRR_VDQCR	0x0058
#define QM_REG_DQRR_PDQCR	0x005c
#define QM_REG_MR_PI_CINH	0x0080
#define QM_REG_MR_CI_CINH	0x0084
#define QM_REG_MR_ITR		0x0088
#define QM_REG_CFG		0x0100
#define QM_REG_ISR		0x0e00
#define QM_REG_IER		0x0e04
#define QM_REG_ISDR		0x0e08
#define QM_REG_IIR		0x0e0c
#define QM_REG_ITPR		0x0e14

/* Cache-enabled register offsets */
#define QM_CL_EQCR		0x0000
#define QM_CL_DQRR		0x1000
#define QM_CL_MR		0x2000
#define QM_CL_EQCR_PI_CENA	0x3000
#define QM_CL_EQCR_CI_CENA	0x3100
#define QM_CL_DQRR_PI_CENA	0x3200
#define QM_CL_DQRR_CI_CENA	0x3300
#define QM_CL_MR_PI_CENA	0x3400
#define QM_CL_MR_CI_CENA	0x3500
#define QM_CL_CR		0x3800
#define QM_CL_RR0		0x3900
#define QM_CL_RR1		0x3940

/*
 * BTW, the drivers (and h/w programming model) already obtain the required
 * synchronisation for portal accesses and data-dependencies. Use of barrier()s
 * or other order-preserving primitives simply degrade performance. Hence the
 * use of the __raw_*() interfaces, which simply ensure that the compiler treats
 * the portal registers as volatile
 */

/* Cache-enabled ring access */
#define qm_cl(base, idx)	((void *)base + ((idx) << 6))

/*
 * Cyclic helper for rings. FIXME: once we are able to do fine-grain perf
 * analysis, look at using the "extra" bit in the ring index registers to avoid
 * cyclic issues.
 */
static inline u8 qm_cyc_diff(u8 ringsize, u8 first, u8 last)
{
	/* 'first' is included, 'last' is excluded */
	if (first <= last)
		return last - first;
	return ringsize + last - first;
}

/*
 * Portal modes.
 *   Enum types;
 *     pmode == production mode
 *     cmode == consumption mode,
 *     dmode == h/w dequeue mode.
 *   Enum values use 3 letter codes. First letter matches the portal mode,
 *   remaining two letters indicate;
 *     ci == cache-inhibited portal register
 *     ce == cache-enabled portal register
 *     vb == in-band valid-bit (cache-enabled)
 *     dc == DCA (Discrete Consumption Acknowledgment), DQRR-only
 *   As for "enum qm_dqrr_dmode", it should be self-explanatory.
 */
enum qm_eqcr_pmode {		/* matches QCSP_CFG::EPM */
	qm_eqcr_pci = 0,	/* PI index, cache-inhibited */
	qm_eqcr_pce = 1,	/* PI index, cache-enabled */
	qm_eqcr_pvb = 2		/* valid-bit */
};
enum qm_dqrr_dmode {		/* matches QCSP_CFG::DP */
	qm_dqrr_dpush = 0,	/* SDQCR  + VDQCR */
	qm_dqrr_dpull = 1	/* PDQCR */
};
enum qm_dqrr_pmode {		/* s/w-only */
	qm_dqrr_pci,		/* reads DQRR_PI_CINH */
	qm_dqrr_pce,		/* reads DQRR_PI_CENA */
	qm_dqrr_pvb		/* reads valid-bit */
};
enum qm_dqrr_cmode {		/* matches QCSP_CFG::DCM */
	qm_dqrr_cci = 0,	/* CI index, cache-inhibited */
	qm_dqrr_cce = 1,	/* CI index, cache-enabled */
	qm_dqrr_cdc = 2		/* Discrete Consumption Acknowledgment */
};
enum qm_mr_pmode {		/* s/w-only */
	qm_mr_pci,		/* reads MR_PI_CINH */
	qm_mr_pce,		/* reads MR_PI_CENA */
	qm_mr_pvb		/* reads valid-bit */
};
enum qm_mr_cmode {		/* matches QCSP_CFG::MM */
	qm_mr_cci = 0,		/* CI index, cache-inhibited */
	qm_mr_cce = 1		/* CI index, cache-enabled */
};

/* --- Portal structures --- */

#define QM_EQCR_SIZE		8
#define QM_DQRR_SIZE		16
#define QM_MR_SIZE		8

struct qm_eqcr {
	struct qm_eqcr_entry *ring, *cursor;
	u8 ci, available, ithresh, vbit;
#ifdef CONFIG_FSL_DPA_CHECKING
	u32 busy;
	enum qm_eqcr_pmode pmode;
#endif
};

struct qm_dqrr {
	const struct qm_dqrr_entry *ring, *cursor;
	u8 pi, ci, fill, ithresh, vbit;
#ifdef CONFIG_FSL_DPA_CHECKING
	enum qm_dqrr_dmode dmode;
	enum qm_dqrr_pmode pmode;
	enum qm_dqrr_cmode cmode;
#endif
};

struct qm_mr {
	const struct qm_mr_entry *ring, *cursor;
	u8 pi, ci, fill, ithresh, vbit;
#ifdef CONFIG_FSL_DPA_CHECKING
	enum qm_mr_pmode pmode;
	enum qm_mr_cmode cmode;
#endif
};

struct qm_mc {
	struct qm_mc_command *cr;
	struct qm_mc_result *rr;
	u8 rridx, vbit;
#ifdef CONFIG_FSL_DPA_CHECKING
	enum {
		/* Can be _mc_start()ed */
		qman_mc_idle,
		/* Can be _mc_commit()ed or _mc_abort()ed */
		qman_mc_user,
		/* Can only be _mc_retry()ed */
		qman_mc_hw
	} state;
#endif
};

#define QM_PORTAL_ALIGNMENT ____cacheline_aligned

struct qm_addr {
	void __iomem *addr_ce;	/* cache-enabled */
	void __iomem *addr_ci;	/* cache-inhibited */
};

struct qm_portal {
	/*
	 * In the non-CONFIG_FSL_DPA_CHECKING case, the following stuff up to
	 * and including 'mc' fits within a cacheline (yay!). The 'config' part
	 * is setup-only, so isn't a cause for a concern. In other words, don't
	 * rearrange this structure on a whim, there be dragons ...
	 */
	struct qm_addr addr;
	struct qm_eqcr eqcr;
	struct qm_dqrr dqrr;
	struct qm_mr mr;
	struct qm_mc mc;
} QM_PORTAL_ALIGNMENT;


/* Cache-inhibited register access. */
static inline u32 qm_in(struct qm_portal *p, u32 offset)
{
	return __raw_readl(p->addr.addr_ci + offset);
}

static inline void qm_out(struct qm_portal *p, u32 offset, u32 val)
{
	__raw_writel(val, p->addr.addr_ci + offset);
}

/* Cache Enabled Portal Access */
static inline void qm_cl_invalidate(struct qm_portal *p, u32 offset)
{
	dpaa_invalidate(p->addr.addr_ce + offset);
}

static inline void qm_cl_touch_ro(struct qm_portal *p, u32 offset)
{
	dpaa_touch_ro(p->addr.addr_ce + offset);
}
static inline void qm_cl_touch_rw(struct qm_portal *p, u32 offset)
{
	dpaa_touch_rw(p->addr.addr_ce + offset);
}

static inline u32 qm_cl_in(struct qm_portal *p, u32 offset)
{
	return __raw_readl(p->addr.addr_ce + offset);
}

static void qm_cl_out(struct qm_portal *p, u32 offset, u32 val)
{
	__raw_writel(val, p->addr.addr_ce + offset);
	dpaa_flush(p->addr.addr_ce + offset);
}

/* --- EQCR API --- */

/* Bit-wise logic to wrap a ring pointer by clearing the "carry bit" */
#define EQCR_CARRYCLEAR(p) \
	(void *)((unsigned long)(p) & (~(unsigned long)(QM_EQCR_SIZE << 6)))

/* Bit-wise logic to convert a ring pointer to a ring index */
static inline u8 EQCR_PTR2IDX(struct qm_eqcr_entry *e)
{
	return ((uintptr_t)e >> 6) & (QM_EQCR_SIZE - 1);
}

/* Increment the 'cursor' ring pointer, taking 'vbit' into account */
static inline void EQCR_INC(struct qm_eqcr *eqcr)
{
	/*
	 *  NB: this is odd-looking, but experiments show that it generates fast
	 * code with essentially no branching overheads. We increment to the
	 * next EQCR pointer and handle overflow and 'vbit'.
	 */
	struct qm_eqcr_entry *partial = eqcr->cursor + 1;

	eqcr->cursor = EQCR_CARRYCLEAR(partial);
	if (partial != eqcr->cursor)
		eqcr->vbit ^= QM_EQCR_VERB_VBIT;
}

static inline int qm_eqcr_init(struct qm_portal *portal,
				enum qm_eqcr_pmode pmode,
				unsigned int eq_stash_thresh,
				int eq_stash_prio)
{
	/*
	 *  This use of 'register', as well as all other occurrences, is because
	 * it has been observed to generate much faster code with gcc than is
	 * otherwise the case.
	 */
	register struct qm_eqcr *eqcr = &portal->eqcr;
	u32 cfg;
	u8 pi;

	eqcr->ring = portal->addr.addr_ce + QM_CL_EQCR;
	eqcr->ci = qm_in(portal, QM_REG_EQCR_CI_CINH) & (QM_EQCR_SIZE - 1);
	qm_cl_invalidate(portal, QM_CL_EQCR_CI_CENA);
	pi = qm_in(portal, QM_REG_EQCR_PI_CINH) & (QM_EQCR_SIZE - 1);
	eqcr->cursor = eqcr->ring + pi;
	eqcr->vbit = (qm_in(portal, QM_REG_EQCR_PI_CINH) & QM_EQCR_SIZE) ?
			QM_EQCR_VERB_VBIT : 0;
	eqcr->available = QM_EQCR_SIZE - 1 -
			qm_cyc_diff(QM_EQCR_SIZE, eqcr->ci, pi);
	eqcr->ithresh = qm_in(portal, QM_REG_EQCR_ITR);
#ifdef CONFIG_FSL_DPA_CHECKING
	eqcr->busy = 0;
	eqcr->pmode = pmode;
#endif
	cfg = (qm_in(portal, QM_REG_CFG) & 0x00ffffff) |
		(eq_stash_thresh << 28) | /* QCSP_CFG: EST */
		(eq_stash_prio << 26)	| /* QCSP_CFG: EP */
		((pmode & 0x3) << 24);	/* QCSP_CFG::EPM */
	qm_out(portal, QM_REG_CFG, cfg);
	return 0;
}

static inline unsigned int qm_eqcr_get_ci_stashing(struct qm_portal *portal)
{
	return (qm_in(portal, QM_REG_CFG) >> 28) & 0x7;
}

static inline void qm_eqcr_finish(struct qm_portal *portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	u8 pi = qm_in(portal, QM_REG_EQCR_PI_CINH) & (QM_EQCR_SIZE - 1);
	u8 ci = qm_in(portal, QM_REG_EQCR_CI_CINH) & (QM_EQCR_SIZE - 1);

	DPA_ASSERT(!eqcr->busy);
	if (pi != EQCR_PTR2IDX(eqcr->cursor))
		pr_crit("losing uncommited EQCR entries\n");
	if (ci != eqcr->ci)
		pr_crit("missing existing EQCR completions\n");
	if (eqcr->ci != EQCR_PTR2IDX(eqcr->cursor))
		pr_crit("EQCR destroyed unquiesced\n");
}

static inline struct qm_eqcr_entry *qm_eqcr_start_no_stash(struct qm_portal
								 *portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;

	DPA_ASSERT(!eqcr->busy);
	if (!eqcr->available)
		return NULL;


#ifdef CONFIG_FSL_DPA_CHECKING
	eqcr->busy = 1;
#endif
	dpaa_zero(eqcr->cursor);
	return eqcr->cursor;
}

static inline struct qm_eqcr_entry *qm_eqcr_start_stash(struct qm_portal
								*portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	u8 diff, old_ci;

	DPA_ASSERT(!eqcr->busy);
	if (!eqcr->available) {
		old_ci = eqcr->ci;
		eqcr->ci = qm_cl_in(portal, QM_CL_EQCR_CI_CENA)
			& (QM_EQCR_SIZE - 1);
		diff = qm_cyc_diff(QM_EQCR_SIZE, old_ci, eqcr->ci);
		eqcr->available += diff;
		if (!diff)
			return NULL;
	}
#ifdef CONFIG_FSL_DPA_CHECKING
	eqcr->busy = 1;
#endif
	dpaa_zero(eqcr->cursor);
	return eqcr->cursor;
}

static inline void qm_eqcr_abort(struct qm_portal *portal)
{
	__maybe_unused register struct qm_eqcr *eqcr = &portal->eqcr;

	DPA_ASSERT(eqcr->busy);
#ifdef CONFIG_FSL_DPA_CHECKING
	eqcr->busy = 0;
#endif
}

static inline struct qm_eqcr_entry *qm_eqcr_pend_and_next(
					struct qm_portal *portal, u8 myverb)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;

	DPA_ASSERT(eqcr->busy);
	DPA_ASSERT(eqcr->pmode != qm_eqcr_pvb);
	if (eqcr->available == 1)
		return NULL;
	eqcr->cursor->__dont_write_directly__verb = myverb | eqcr->vbit;
	dpaa_flush(eqcr->cursor);
	EQCR_INC(eqcr);
	eqcr->available--;
	dpaa_zero(eqcr->cursor);
	return eqcr->cursor;
}

#define EQCR_COMMIT_CHECKS(eqcr) \
do { \
	DPA_ASSERT(eqcr->busy); \
	DPA_ASSERT(eqcr->cursor->orp == (eqcr->cursor->orp & 0x00ffffff)); \
	DPA_ASSERT(eqcr->cursor->fqid == (eqcr->cursor->fqid & 0x00ffffff)); \
} while (0)

static inline void qm_eqcr_pci_commit(struct qm_portal *portal, u8 myverb)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;

	EQCR_COMMIT_CHECKS(eqcr);
	DPA_ASSERT(eqcr->pmode == qm_eqcr_pci);
	eqcr->cursor->__dont_write_directly__verb = myverb | eqcr->vbit;
	EQCR_INC(eqcr);
	eqcr->available--;
	dpaa_flush(eqcr->cursor);
	/* Need to sync to ensure setting PI doesn't bypass command on bus */
	mb();
	qm_out(portal, QM_REG_EQCR_PI_CINH, EQCR_PTR2IDX(eqcr->cursor));
#ifdef CONFIG_FSL_DPA_CHECKING
	eqcr->busy = 0;
#endif
}

static inline void qm_eqcr_pce_prefetch(struct qm_portal *portal)
{
	__maybe_unused register struct qm_eqcr *eqcr = &portal->eqcr;

	DPA_ASSERT(eqcr->pmode == qm_eqcr_pce);
	qm_cl_invalidate(portal, QM_CL_EQCR_PI_CENA);
	qm_cl_touch_rw(portal, QM_CL_EQCR_PI_CENA);
}

static inline void qm_eqcr_pce_commit(struct qm_portal *portal, u8 myverb)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;

	EQCR_COMMIT_CHECKS(eqcr);
	DPA_ASSERT(eqcr->pmode == qm_eqcr_pce);
	eqcr->cursor->__dont_write_directly__verb = myverb | eqcr->vbit;
	EQCR_INC(eqcr);
	eqcr->available--;
	dpaa_flush(eqcr->cursor);
	dma_wmb();
	qm_cl_out(portal, QM_CL_EQCR_PI_CENA, EQCR_PTR2IDX(eqcr->cursor));
#ifdef CONFIG_FSL_DPA_CHECKING
	eqcr->busy = 0;
#endif
}

static inline void qm_eqcr_pvb_commit(struct qm_portal *portal, u8 myverb)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	struct qm_eqcr_entry *eqcursor;

	EQCR_COMMIT_CHECKS(eqcr);
	DPA_ASSERT(eqcr->pmode == qm_eqcr_pvb);
	dma_wmb();
	eqcursor = eqcr->cursor;
	eqcursor->__dont_write_directly__verb = myverb | eqcr->vbit;
	dpaa_flush(eqcursor);
	EQCR_INC(eqcr);
	eqcr->available--;
#ifdef CONFIG_FSL_DPA_CHECKING
	eqcr->busy = 0;
#endif
}

static inline u8 qm_eqcr_cci_update(struct qm_portal *portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	u8 diff, old_ci = eqcr->ci;

	eqcr->ci = qm_in(portal, QM_REG_EQCR_CI_CINH) & (QM_EQCR_SIZE - 1);
	diff = qm_cyc_diff(QM_EQCR_SIZE, old_ci, eqcr->ci);
	eqcr->available += diff;
	return diff;
}

static inline void qm_eqcr_cce_prefetch(struct qm_portal *portal)
{
	qm_cl_touch_ro(portal, QM_CL_EQCR_CI_CENA);
}

static inline u8 qm_eqcr_cce_update(struct qm_portal *portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;
	u8 diff, old_ci = eqcr->ci;

	eqcr->ci = qm_cl_in(portal, QM_CL_EQCR_CI_CENA) & (QM_EQCR_SIZE - 1);
	qm_cl_invalidate(portal, QM_CL_EQCR_CI_CENA);
	diff = qm_cyc_diff(QM_EQCR_SIZE, old_ci, eqcr->ci);
	eqcr->available += diff;
	return diff;
}

static inline u8 qm_eqcr_get_ithresh(struct qm_portal *portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;

	return eqcr->ithresh;
}

static inline void qm_eqcr_set_ithresh(struct qm_portal *portal, u8 ithresh)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;

	eqcr->ithresh = ithresh;
	qm_out(portal, QM_REG_EQCR_ITR, ithresh);
}

static inline u8 qm_eqcr_get_avail(struct qm_portal *portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;

	return eqcr->available;
}

static inline u8 qm_eqcr_get_fill(struct qm_portal *portal)
{
	register struct qm_eqcr *eqcr = &portal->eqcr;

	return QM_EQCR_SIZE - 1 - eqcr->available;
}

/* --- DQRR API --- */

/*
 * FIXME: many possible improvements;
 * - look at changing the API to use pointer rather than index parameters now
 *   that 'cursor' is a pointer,
 * - consider moving other parameters to pointer if it could help (ci)
 */

#define DQRR_CARRYCLEAR(p) \
	(void *)((unsigned long)(p) & (~(unsigned long)(QM_DQRR_SIZE << 6)))

static inline u8 DQRR_PTR2IDX(const struct qm_dqrr_entry *e)
{
	return ((uintptr_t)e >> 6) & (QM_DQRR_SIZE - 1);
}

static inline const struct qm_dqrr_entry *DQRR_INC(
						const struct qm_dqrr_entry *e)
{
	return DQRR_CARRYCLEAR(e + 1);
}

static inline void qm_dqrr_set_maxfill(struct qm_portal *portal, u8 mf)
{
	qm_out(portal, QM_REG_CFG, (qm_in(portal, QM_REG_CFG) & 0xff0fffff) |
		((mf & (QM_DQRR_SIZE - 1)) << 20));
}

static inline int qm_dqrr_init(struct qm_portal *portal,
				const struct qm_portal_config *config,
				enum qm_dqrr_dmode dmode,
				__maybe_unused enum qm_dqrr_pmode pmode,
				enum qm_dqrr_cmode cmode, u8 max_fill)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	u32 cfg;

	/* Make sure the DQRR will be idle when we enable */
	qm_out(portal, QM_REG_DQRR_SDQCR, 0);
	qm_out(portal, QM_REG_DQRR_VDQCR, 0);
	qm_out(portal, QM_REG_DQRR_PDQCR, 0);
	dqrr->ring = portal->addr.addr_ce + QM_CL_DQRR;
	dqrr->pi = qm_in(portal, QM_REG_DQRR_PI_CINH) & (QM_DQRR_SIZE - 1);
	dqrr->ci = qm_in(portal, QM_REG_DQRR_CI_CINH) & (QM_DQRR_SIZE - 1);
	dqrr->cursor = dqrr->ring + dqrr->ci;
	dqrr->fill = qm_cyc_diff(QM_DQRR_SIZE, dqrr->ci, dqrr->pi);
	dqrr->vbit = (qm_in(portal, QM_REG_DQRR_PI_CINH) & QM_DQRR_SIZE) ?
			QM_DQRR_VERB_VBIT : 0;
	dqrr->ithresh = qm_in(portal, QM_REG_DQRR_ITR);
#ifdef CONFIG_FSL_DPA_CHECKING
	dqrr->dmode = dmode;
	dqrr->pmode = pmode;
	dqrr->cmode = cmode;
#endif
	/* Invalidate every ring entry before beginning */
	for (cfg = 0; cfg < QM_DQRR_SIZE; cfg++)
		dpaa_invalidate(qm_cl(dqrr->ring, cfg));
	cfg = (qm_in(portal, QM_REG_CFG) & 0xff000f00) |
		((max_fill & (QM_DQRR_SIZE - 1)) << 20) | /* DQRR_MF */
		((dmode & 1) << 18) |			/* DP */
		((cmode & 3) << 16) |			/* DCM */
		0xa0 |					/* RE+SE */
		(0 ? 0x40 : 0) |			/* Ignore RP */
		(0 ? 0x10 : 0);				/* Ignore SP */
	qm_out(portal, QM_REG_CFG, cfg);
	qm_dqrr_set_maxfill(portal, max_fill);
	return 0;
}

static inline void qm_dqrr_finish(struct qm_portal *portal)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;
#ifdef CONFIG_FSL_DPA_CHECKING
	if ((dqrr->cmode != qm_dqrr_cdc) &&
			(dqrr->ci != DQRR_PTR2IDX(dqrr->cursor)))
		pr_crit("Ignoring completed DQRR entries\n");
#endif
}

static inline const struct qm_dqrr_entry *qm_dqrr_current(
						struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;

	if (!dqrr->fill)
		return NULL;
	return dqrr->cursor;
}

static inline u8 qm_dqrr_cursor(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;

	return DQRR_PTR2IDX(dqrr->cursor);
}

static inline u8 qm_dqrr_next(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->fill);
	dqrr->cursor = DQRR_INC(dqrr->cursor);
	return --dqrr->fill;
}

static inline u8 qm_dqrr_pci_update(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	u8 diff, old_pi = dqrr->pi;

	DPA_ASSERT(dqrr->pmode == qm_dqrr_pci);
	dqrr->pi = qm_in(portal, QM_REG_DQRR_PI_CINH) & (QM_DQRR_SIZE - 1);
	diff = qm_cyc_diff(QM_DQRR_SIZE, old_pi, dqrr->pi);
	dqrr->fill += diff;
	return diff;
}

static inline void qm_dqrr_pce_prefetch(struct qm_portal *portal)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->pmode == qm_dqrr_pce);
	qm_cl_invalidate(portal, QM_CL_DQRR_PI_CENA);
	qm_cl_touch_ro(portal, QM_CL_DQRR_PI_CENA);
}

static inline u8 qm_dqrr_pce_update(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	u8 diff, old_pi = dqrr->pi;

	DPA_ASSERT(dqrr->pmode == qm_dqrr_pce);
	dqrr->pi = qm_cl_in(portal, QM_CL_DQRR_PI_CENA) & (QM_DQRR_SIZE - 1);
	diff = qm_cyc_diff(QM_DQRR_SIZE, old_pi, dqrr->pi);
	dqrr->fill += diff;
	return diff;
}

static inline void qm_dqrr_pvb_update(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;
	const struct qm_dqrr_entry *res = qm_cl(dqrr->ring, dqrr->pi);

	DPA_ASSERT(dqrr->pmode == qm_dqrr_pvb);
#ifndef CONFIG_FSL_PAMU
	/*
	 * If PAMU is not available we need to invalidate the cache.
	 * When PAMU is available the cache is updated by stash
	 */
	dpaa_invalidate_touch_ro(res);
#endif
	/*
	 *  when accessing 'verb', use __raw_readb() to ensure that compiler
	 * inlining doesn't try to optimise out "excess reads".
	 */
	if ((__raw_readb(&res->verb) & QM_DQRR_VERB_VBIT) == dqrr->vbit) {
		dqrr->pi = (dqrr->pi + 1) & (QM_DQRR_SIZE - 1);
		if (!dqrr->pi)
			dqrr->vbit ^= QM_DQRR_VERB_VBIT;
		dqrr->fill++;
	}
}

static inline void qm_dqrr_cci_consume(struct qm_portal *portal, u8 num)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->cmode == qm_dqrr_cci);
	dqrr->ci = (dqrr->ci + num) & (QM_DQRR_SIZE - 1);
	qm_out(portal, QM_REG_DQRR_CI_CINH, dqrr->ci);
}

static inline void qm_dqrr_cci_consume_to_current(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->cmode == qm_dqrr_cci);
	dqrr->ci = DQRR_PTR2IDX(dqrr->cursor);
	qm_out(portal, QM_REG_DQRR_CI_CINH, dqrr->ci);
}

static inline void qm_dqrr_cce_prefetch(struct qm_portal *portal)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->cmode == qm_dqrr_cce);
	qm_cl_invalidate(portal, QM_CL_DQRR_CI_CENA);
	qm_cl_touch_rw(portal, QM_CL_DQRR_CI_CENA);
}

static inline void qm_dqrr_cce_consume(struct qm_portal *portal, u8 num)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->cmode == qm_dqrr_cce);
	dqrr->ci = (dqrr->ci + num) & (QM_DQRR_SIZE - 1);
	qm_cl_out(portal, QM_CL_DQRR_CI_CENA, dqrr->ci);
}

static inline void qm_dqrr_cce_consume_to_current(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->cmode == qm_dqrr_cce);
	dqrr->ci = DQRR_PTR2IDX(dqrr->cursor);
	qm_cl_out(portal, QM_CL_DQRR_CI_CENA, dqrr->ci);
}

static inline void qm_dqrr_cdc_consume_1(struct qm_portal *portal, u8 idx,
					int park)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->cmode == qm_dqrr_cdc);
	DPA_ASSERT(idx < QM_DQRR_SIZE);
	qm_out(portal, QM_REG_DQRR_DCAP, (0 << 8) |	/* S */
		((park ? 1 : 0) << 6) |	/* PK */
		idx);			/* DCAP_CI */
}

static inline void qm_dqrr_cdc_consume_1ptr(struct qm_portal *portal,
					const struct qm_dqrr_entry *dq,
					int park)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;
	u8 idx = DQRR_PTR2IDX(dq);

	DPA_ASSERT(dqrr->cmode == qm_dqrr_cdc);
	DPA_ASSERT((dqrr->ring + idx) == dq);
	DPA_ASSERT(idx < QM_DQRR_SIZE);
	qm_out(portal, QM_REG_DQRR_DCAP, (0 << 8) | /* DQRR_DCAP::S */
	       ((park ? 1 : 0) << 6) |		    /* DQRR_DCAP::PK */
	       idx);				    /* DQRR_DCAP::DCAP_CI */
}

static inline void qm_dqrr_cdc_consume_n(struct qm_portal *portal, u16 bitmask)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->cmode == qm_dqrr_cdc);
	qm_out(portal, QM_REG_DQRR_DCAP, (1 << 8) | /* DQRR_DCAP::S */
	       ((u32)bitmask << 16));		    /* DQRR_DCAP::DCAP_CI */
}

static inline u8 qm_dqrr_cdc_cci(struct qm_portal *portal)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->cmode == qm_dqrr_cdc);
	return qm_in(portal, QM_REG_DQRR_CI_CINH) & (QM_DQRR_SIZE - 1);
}

static inline void qm_dqrr_cdc_cce_prefetch(struct qm_portal *portal)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->cmode == qm_dqrr_cdc);
	qm_cl_invalidate(portal, QM_CL_DQRR_CI_CENA);
	qm_cl_touch_ro(portal, QM_CL_DQRR_CI_CENA);
}

static inline u8 qm_dqrr_cdc_cce(struct qm_portal *portal)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->cmode == qm_dqrr_cdc);
	return qm_cl_in(portal, QM_CL_DQRR_CI_CENA) & (QM_DQRR_SIZE - 1);
}

static inline u8 qm_dqrr_get_ci(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->cmode != qm_dqrr_cdc);
	return dqrr->ci;
}

static inline void qm_dqrr_park(struct qm_portal *portal, u8 idx)
{
	__maybe_unused register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->cmode != qm_dqrr_cdc);
	qm_out(portal, QM_REG_DQRR_DCAP, (0 << 8) |  /* S */
	       (1 << 6) |			     /* PK */
	       (idx & (QM_DQRR_SIZE - 1)));	     /* DCAP_CI */
}

static inline void qm_dqrr_park_current(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;

	DPA_ASSERT(dqrr->cmode != qm_dqrr_cdc);
	qm_out(portal, QM_REG_DQRR_DCAP, (0 << 8) | /* S */
	       (1 << 6) |			    /* PK */
	       DQRR_PTR2IDX(dqrr->cursor));	    /* DCAP_CI */
}

static inline void qm_dqrr_sdqcr_set(struct qm_portal *portal, u32 sdqcr)
{
	qm_out(portal, QM_REG_DQRR_SDQCR, sdqcr);
}

static inline u32 qm_dqrr_sdqcr_get(struct qm_portal *portal)
{
	return qm_in(portal, QM_REG_DQRR_SDQCR);
}

static inline void qm_dqrr_vdqcr_set(struct qm_portal *portal, u32 vdqcr)
{
	qm_out(portal, QM_REG_DQRR_VDQCR, vdqcr);
}

static inline u32 qm_dqrr_vdqcr_get(struct qm_portal *portal)
{
	return qm_in(portal, QM_REG_DQRR_VDQCR);
}

static inline void qm_dqrr_pdqcr_set(struct qm_portal *portal, u32 pdqcr)
{
	qm_out(portal, QM_REG_DQRR_PDQCR, pdqcr);
}

static inline u32 qm_dqrr_pdqcr_get(struct qm_portal *portal)
{
	return qm_in(portal, QM_REG_DQRR_PDQCR);
}

static inline u8 qm_dqrr_get_ithresh(struct qm_portal *portal)
{
	register struct qm_dqrr *dqrr = &portal->dqrr;

	return dqrr->ithresh;
}

static inline void qm_dqrr_set_ithresh(struct qm_portal *portal, u8 ithresh)
{
	qm_out(portal, QM_REG_DQRR_ITR, ithresh);
}

static inline u8 qm_dqrr_get_maxfill(struct qm_portal *portal)
{
	return (qm_in(portal, QM_REG_CFG) & 0x00f00000) >> 20;
}

/* --- MR API --- */

#define MR_CARRYCLEAR(p) \
	(void *)((unsigned long)(p) & (~(unsigned long)(QM_MR_SIZE << 6)))

static inline u8 MR_PTR2IDX(const struct qm_mr_entry *e)
{
	return ((uintptr_t)e >> 6) & (QM_MR_SIZE - 1);
}

static inline const struct qm_mr_entry *MR_INC(const struct qm_mr_entry *e)
{
	return MR_CARRYCLEAR(e + 1);
}

static inline int qm_mr_init(struct qm_portal *portal, enum qm_mr_pmode pmode,
		enum qm_mr_cmode cmode)
{
	register struct qm_mr *mr = &portal->mr;
	u32 cfg;

	mr->ring = portal->addr.addr_ce + QM_CL_MR;
	mr->pi = qm_in(portal, QM_REG_MR_PI_CINH) & (QM_MR_SIZE - 1);
	mr->ci = qm_in(portal, QM_REG_MR_CI_CINH) & (QM_MR_SIZE - 1);
	mr->cursor = mr->ring + mr->ci;
	mr->fill = qm_cyc_diff(QM_MR_SIZE, mr->ci, mr->pi);
	mr->vbit = (qm_in(portal, QM_REG_MR_PI_CINH) & QM_MR_SIZE)
		? QM_MR_VERB_VBIT : 0;
	mr->ithresh = qm_in(portal, QM_REG_MR_ITR);
#ifdef CONFIG_FSL_DPA_CHECKING
	mr->pmode = pmode;
	mr->cmode = cmode;
#endif
	cfg = (qm_in(portal, QM_REG_CFG) & 0xfffff0ff) |
		((cmode & 1) << 8);		/* QCSP_CFG:MM */
	qm_out(portal, QM_REG_CFG, cfg);
	return 0;
}

static inline void qm_mr_finish(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;

	if (mr->ci != MR_PTR2IDX(mr->cursor))
		pr_crit("Ignoring completed MR entries\n");
}

static inline const struct qm_mr_entry *qm_mr_current(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;

	if (!mr->fill)
		return NULL;
	return mr->cursor;
}

static inline u8 qm_mr_cursor(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;

	return MR_PTR2IDX(mr->cursor);
}

static inline u8 qm_mr_next(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;

	DPA_ASSERT(mr->fill);
	mr->cursor = MR_INC(mr->cursor);
	return --mr->fill;
}

static inline u8 qm_mr_pci_update(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;
	u8 diff, old_pi = mr->pi;

	DPA_ASSERT(mr->pmode == qm_mr_pci);
	mr->pi = qm_in(portal, QM_REG_MR_PI_CINH);
	diff = qm_cyc_diff(QM_MR_SIZE, old_pi, mr->pi);
	mr->fill += diff;
	return diff;
}

static inline void qm_mr_pce_prefetch(struct qm_portal *portal)
{
	__maybe_unused register struct qm_mr *mr = &portal->mr;

	DPA_ASSERT(mr->pmode == qm_mr_pce);
	qm_cl_invalidate(portal, QM_CL_MR_PI_CENA);
	qm_cl_touch_ro(portal, QM_CL_MR_PI_CENA);
}

static inline u8 qm_mr_pce_update(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;
	u8 diff, old_pi = mr->pi;

	DPA_ASSERT(mr->pmode == qm_mr_pce);
	mr->pi = qm_cl_in(portal, QM_CL_MR_PI_CENA) & (QM_MR_SIZE - 1);
	diff = qm_cyc_diff(QM_MR_SIZE, old_pi, mr->pi);
	mr->fill += diff;
	return diff;
}

static inline void qm_mr_pvb_update(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;
	const struct qm_mr_entry *res = qm_cl(mr->ring, mr->pi);

	DPA_ASSERT(mr->pmode == qm_mr_pvb);
	/*
	 *  when accessing 'verb', use __raw_readb() to ensure that compiler
	 * inlining doesn't try to optimise out "excess reads".
	 */
	if ((__raw_readb(&res->verb) & QM_MR_VERB_VBIT) == mr->vbit) {
		mr->pi = (mr->pi + 1) & (QM_MR_SIZE - 1);
		if (!mr->pi)
			mr->vbit ^= QM_MR_VERB_VBIT;
		mr->fill++;
		res = MR_INC(res);
	}
	dpaa_invalidate_touch_ro(res);
}

static inline void qm_mr_cci_consume(struct qm_portal *portal, u8 num)
{
	register struct qm_mr *mr = &portal->mr;

	DPA_ASSERT(mr->cmode == qm_mr_cci);
	mr->ci = (mr->ci + num) & (QM_MR_SIZE - 1);
	qm_out(portal, QM_REG_MR_CI_CINH, mr->ci);
}

static inline void qm_mr_cci_consume_to_current(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;

	DPA_ASSERT(mr->cmode == qm_mr_cci);
	mr->ci = MR_PTR2IDX(mr->cursor);
	qm_out(portal, QM_REG_MR_CI_CINH, mr->ci);
}

static inline void qm_mr_cce_prefetch(struct qm_portal *portal)
{
	__maybe_unused register struct qm_mr *mr = &portal->mr;

	DPA_ASSERT(mr->cmode == qm_mr_cce);
	qm_cl_invalidate(portal, QM_CL_MR_CI_CENA);
	qm_cl_touch_rw(portal, QM_CL_MR_CI_CENA);
}

static inline void qm_mr_cce_consume(struct qm_portal *portal, u8 num)
{
	register struct qm_mr *mr = &portal->mr;

	DPA_ASSERT(mr->cmode == qm_mr_cce);
	mr->ci = (mr->ci + num) & (QM_MR_SIZE - 1);
	qm_cl_out(portal, QM_CL_MR_CI_CENA, mr->ci);
}

static inline void qm_mr_cce_consume_to_current(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;

	DPA_ASSERT(mr->cmode == qm_mr_cce);
	mr->ci = MR_PTR2IDX(mr->cursor);
	qm_cl_out(portal, QM_CL_MR_CI_CENA, mr->ci);
}

static inline u8 qm_mr_get_ci(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;

	return mr->ci;
}

static inline u8 qm_mr_get_ithresh(struct qm_portal *portal)
{
	register struct qm_mr *mr = &portal->mr;

	return mr->ithresh;
}

static inline void qm_mr_set_ithresh(struct qm_portal *portal, u8 ithresh)
{
	qm_out(portal, QM_REG_MR_ITR, ithresh);
}

/* --- Management command API --- */

static inline int qm_mc_init(struct qm_portal *portal)
{
	register struct qm_mc *mc = &portal->mc;

	mc->cr = portal->addr.addr_ce + QM_CL_CR;
	mc->rr = portal->addr.addr_ce + QM_CL_RR0;
	mc->rridx = (__raw_readb(&mc->cr->__dont_write_directly__verb) &
			QM_MCC_VERB_VBIT) ?  0 : 1;
	mc->vbit = mc->rridx ? QM_MCC_VERB_VBIT : 0;
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = qman_mc_idle;
#endif
	return 0;
}

static inline void qm_mc_finish(struct qm_portal *portal)
{
	__maybe_unused register struct qm_mc *mc = &portal->mc;

	DPA_ASSERT(mc->state == qman_mc_idle);
#ifdef CONFIG_FSL_DPA_CHECKING
	if (mc->state != qman_mc_idle)
		pr_crit("Losing incomplete MC command\n");
#endif
}

static inline struct qm_mc_command *qm_mc_start(struct qm_portal *portal)
{
	register struct qm_mc *mc = &portal->mc;

	DPA_ASSERT(mc->state == qman_mc_idle);
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = qman_mc_user;
#endif
	dpaa_zero(mc->cr);
	return mc->cr;
}

static inline void qm_mc_abort(struct qm_portal *portal)
{
	__maybe_unused register struct qm_mc *mc = &portal->mc;

	DPA_ASSERT(mc->state == qman_mc_user);
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = qman_mc_idle;
#endif
}

static inline void qm_mc_commit(struct qm_portal *portal, u8 myverb)
{
	register struct qm_mc *mc = &portal->mc;
	struct qm_mc_result *rr = mc->rr + mc->rridx;

	DPA_ASSERT(mc->state == qman_mc_user);
	dma_wmb();
	mc->cr->__dont_write_directly__verb = myverb | mc->vbit;
	dpaa_flush(mc->cr);
	dpaa_invalidate_touch_ro(rr);
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = qman_mc_hw;
#endif
}

static inline struct qm_mc_result *qm_mc_result(struct qm_portal *portal)
{
	register struct qm_mc *mc = &portal->mc;
	struct qm_mc_result *rr = mc->rr + mc->rridx;

	DPA_ASSERT(mc->state == qman_mc_hw);
	/*
	 *  The inactive response register's verb byte always returns zero until
	 * its command is submitted and completed. This includes the valid-bit,
	 * in case you were wondering...
	 */
	if (!__raw_readb(&rr->verb)) {
		dpaa_invalidate_touch_ro(rr);
		return NULL;
	}
	mc->rridx ^= 1;
	mc->vbit ^= QM_MCC_VERB_VBIT;
#ifdef CONFIG_FSL_DPA_CHECKING
	mc->state = qman_mc_idle;
#endif
	return rr;
}
