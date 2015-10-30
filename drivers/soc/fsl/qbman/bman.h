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

#include "bman_priv.h"

/***************************/
/* Portal register assists */
/***************************/

/* Cache-inhibited register offsets */
#define BM_REG_RCR_PI_CINH	0x0000
#define BM_REG_RCR_CI_CINH	0x0004
#define BM_REG_RCR_ITR		0x0008
#define BM_REG_CFG		0x0100
#define BM_REG_SCN(n)		(0x0200 + ((n) << 2))
#define BM_REG_ISR		0x0e00
#define BM_REG_IER		0x0e04
#define BM_REG_ISDR		0x0e08
#define BM_REG_IIR		0x0e0c

/* Cache-enabled register offsets */
#define BM_CL_CR		0x0000
#define BM_CL_RR0		0x0100
#define BM_CL_RR1		0x0140
#define BM_CL_RCR		0x1000
#define BM_CL_RCR_PI_CENA	0x3000
#define BM_CL_RCR_CI_CENA	0x3100

/*
 * Cyclic helper for rings. FIXME: once we are able to do fine-grain perf
 * analysis, look at using the "extra" bit in the ring index registers to avoid
 * cyclic issues.
 */
static inline u8 bm_cyc_diff(u8 ringsize, u8 first, u8 last)
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
 *   Enum values use 3 letter codes. First letter matches the portal mode,
 *   remaining two letters indicate;
 *     ci == cache-inhibited portal register
 *     ce == cache-enabled portal register
 *     vb == in-band valid-bit (cache-enabled)
 */
enum bm_rcr_pmode {		/* matches BCSP_CFG::RPM */
	bm_rcr_pci = 0,		/* PI index, cache-inhibited */
	bm_rcr_pce = 1,		/* PI index, cache-enabled */
	bm_rcr_pvb = 2		/* valid-bit */
};
enum bm_rcr_cmode {		/* s/w-only */
	bm_rcr_cci,		/* CI index, cache-inhibited */
	bm_rcr_cce		/* CI index, cache-enabled */
};


/* --- Portal structures --- */

#define BM_RCR_SIZE		8

struct bm_rcr {
	struct bm_rcr_entry *ring, *cursor;
	u8 ci, available, ithresh, vbit;
#ifdef CONFIG_FSL_DPA_CHECKING
	u32 busy;
	enum bm_rcr_pmode pmode;
	enum bm_rcr_cmode cmode;
#endif
};

struct bm_mc {
	struct bm_mc_command *cr;
	struct bm_mc_result *rr;
	u8 rridx, vbit;
#ifdef CONFIG_FSL_DPA_CHECKING
	enum {
		/* Can only be _mc_start()ed */
		mc_idle,
		/* Can only be _mc_commit()ed or _mc_abort()ed */
		mc_user,
		/* Can only be _mc_retry()ed */
		mc_hw
	} state;
#endif
};

struct bm_addr {
	void __iomem *addr_ce;	/* cache-enabled */
	void __iomem *addr_ci;	/* cache-inhibited */
};

struct bm_portal {
	struct bm_addr addr;
	struct bm_rcr rcr;
	struct bm_mc mc;
	struct bm_portal_config config;
} ____cacheline_aligned;

/* Cache-inhibited register access. */
static inline u32 bm_in(struct bm_portal *p, u32 offset)
{
	return __raw_readl(p->addr.addr_ci + offset);
}

static inline void bm_out(struct bm_portal *p, u32 offset, u32 val)
{
	__raw_writel(val, p->addr.addr_ci + offset);
}

/* Cache Enabled Portal Access */
static inline void bm_cl_invalidate(struct bm_portal *p, u32 offset)
{
	dpaa_invalidate(p->addr.addr_ce + offset);
}

static inline void bm_cl_touch_ro(struct bm_portal *p, u32 offset)
{
	dpaa_touch_ro(p->addr.addr_ce + offset);
}
static inline void bm_cl_touch_rw(struct bm_portal *p, u32 offset)
{
	dpaa_touch_rw(p->addr.addr_ce + offset);
}

static inline u32 bm_cl_in(struct bm_portal *p, u32 offset)
{
	return __raw_readl(p->addr.addr_ce + offset);
}

static void bm_cl_out(struct bm_portal *p, u32 offset, u32 val)
{
	__raw_writel(val, p->addr.addr_ce + offset);
	dpaa_flush(p->addr.addr_ce + offset);
}


/* --- RCR API --- */

#define RCR_SHIFT 6 /* (log2(sizeof(struct bm_rcr_entry))) */

/* Bit-wise logic to wrap a ring pointer by clearing the "carry bit" */
static struct bm_rcr_entry *rcr_carryclear(struct bm_rcr_entry *p)
{
	return (struct bm_rcr_entry *)
		((unsigned long)(p) & (~(unsigned long)
				       (BM_RCR_SIZE << RCR_SHIFT)));
}

/* Bit-wise logic to convert a ring pointer to a ring index */
static inline u8 rcr_ptr2idx(struct bm_rcr_entry *e)
{
	return ((uintptr_t)e >> RCR_SHIFT) & (BM_RCR_SIZE - 1);
}

/* Increment the 'cursor' ring pointer, taking 'vbit' into account */
static inline void rcr_inc(struct bm_rcr *rcr)
{
	/*
	 * NB: this is odd-looking, but experiments show that it generates
	 * fast code with essentially no branching overheads. We increment to
	 * the next RCR pointer and handle overflow and 'vbit'.
	 */
	struct bm_rcr_entry *partial = rcr->cursor + 1;

	rcr->cursor = rcr_carryclear(partial);
	if (partial != rcr->cursor)
		rcr->vbit ^= BM_RCR_VERB_VBIT;
}

static inline struct bm_rcr_entry *bm_rcr_start(struct bm_portal *portal)
{
	register struct bm_rcr *rcr = &portal->rcr;

	DPA_ASSERT(!rcr->busy);
	if (!rcr->available)
		return NULL;
#ifdef CONFIG_FSL_DPA_CHECKING
	rcr->busy = 1;
#endif
	dpaa_zero(rcr->cursor);
	return rcr->cursor;
}

static inline void bm_rcr_abort(struct bm_portal *portal)
{
	__maybe_unused register struct bm_rcr *rcr = &portal->rcr;

	DPA_ASSERT(rcr->busy);
#ifdef CONFIG_FSL_DPA_CHECKING
	rcr->busy = 0;
#endif
}

static inline struct bm_rcr_entry *bm_rcr_pend_and_next(
					struct bm_portal *portal, u8 myverb)
{
	register struct bm_rcr *rcr = &portal->rcr;

	DPA_ASSERT(rcr->busy);
	DPA_ASSERT(rcr->pmode != bm_rcr_pvb);
	if (rcr->available == 1)
		return NULL;
	rcr->cursor->__dont_write_directly__verb = myverb | rcr->vbit;
	dpaa_flush(rcr->cursor);
	rcr_inc(rcr);
	rcr->available--;
	dpaa_zero(rcr->cursor);
	return rcr->cursor;
}

static inline void bm_rcr_pci_commit(struct bm_portal *portal, u8 myverb)
{
	register struct bm_rcr *rcr = &portal->rcr;

	DPA_ASSERT(rcr->busy);
	DPA_ASSERT(rcr->pmode == bm_rcr_pci);
	rcr->cursor->__dont_write_directly__verb = myverb | rcr->vbit;
	rcr_inc(rcr);
	rcr->available--;
	/*
	 * Need to have a memory barrier to ensure the command
	 * has been completely written prior to incrementing PI
	 */
	mb();
	bm_out(portal, BM_REG_RCR_PI_CINH, rcr_ptr2idx(rcr->cursor));
#ifdef CONFIG_FSL_DPA_CHECKING
	rcr->busy = 0;
#endif
}

static inline void bm_rcr_pce_prefetch(struct bm_portal *portal)
{
	__maybe_unused register struct bm_rcr *rcr = &portal->rcr;

	DPA_ASSERT(rcr->pmode == bm_rcr_pce);
	bm_cl_invalidate(portal, BM_CL_RCR_PI_CENA);
	bm_cl_touch_rw(portal, BM_CL_RCR_PI_CENA);
}

static inline void bm_rcr_pce_commit(struct bm_portal *portal, u8 myverb)
{
	register struct bm_rcr *rcr = &portal->rcr;

	DPA_ASSERT(rcr->busy);
	DPA_ASSERT(rcr->pmode == bm_rcr_pce);
	rcr->cursor->__dont_write_directly__verb = myverb | rcr->vbit;
	rcr_inc(rcr);
	rcr->available--;
	dma_wmb();
	bm_cl_out(portal, BM_CL_RCR_PI_CENA, rcr_ptr2idx(rcr->cursor));
#ifdef CONFIG_FSL_DPA_CHECKING
	rcr->busy = 0;
#endif
}

static inline void bm_rcr_pvb_commit(struct bm_portal *portal, u8 myverb)
{
	register struct bm_rcr *rcr = &portal->rcr;
	struct bm_rcr_entry *rcursor;

	DPA_ASSERT(rcr->busy);
	DPA_ASSERT(rcr->pmode == bm_rcr_pvb);
	dma_wmb();
	rcursor = rcr->cursor;
	rcursor->__dont_write_directly__verb = myverb | rcr->vbit;
	dpaa_flush(rcursor);
	rcr_inc(rcr);
	rcr->available--;
#ifdef CONFIG_FSL_DPA_CHECKING
	rcr->busy = 0;
#endif
}

static inline u8 bm_rcr_cci_update(struct bm_portal *portal)
{
	register struct bm_rcr *rcr = &portal->rcr;
	u8 diff, old_ci = rcr->ci;

	DPA_ASSERT(rcr->cmode == bm_rcr_cci);
	rcr->ci = bm_in(portal, BM_REG_RCR_CI_CINH) & (BM_RCR_SIZE - 1);
	diff = bm_cyc_diff(BM_RCR_SIZE, old_ci, rcr->ci);
	rcr->available += diff;
	return diff;
}

static inline void bm_rcr_cce_prefetch(struct bm_portal *portal)
{
	__maybe_unused register struct bm_rcr *rcr = &portal->rcr;

	DPA_ASSERT(rcr->cmode == bm_rcr_cce);
	bm_cl_touch_ro(portal, BM_CL_RCR_CI_CENA);
}

static inline u8 bm_rcr_cce_update(struct bm_portal *portal)
{
	register struct bm_rcr *rcr = &portal->rcr;
	u8 diff, old_ci = rcr->ci;

	DPA_ASSERT(rcr->cmode == bm_rcr_cce);
	rcr->ci = bm_cl_in(portal, BM_CL_RCR_CI_CENA) & (BM_RCR_SIZE - 1);
	bm_cl_invalidate(portal, BM_CL_RCR_CI_CENA);
	diff = bm_cyc_diff(BM_RCR_SIZE, old_ci, rcr->ci);
	rcr->available += diff;
	return diff;
}

static inline u8 bm_rcr_get_ithresh(struct bm_portal *portal)
{
	register struct bm_rcr *rcr = &portal->rcr;

	return rcr->ithresh;
}

static inline void bm_rcr_set_ithresh(struct bm_portal *portal, u8 ithresh)
{
	register struct bm_rcr *rcr = &portal->rcr;

	rcr->ithresh = ithresh;
	bm_out(portal, BM_REG_RCR_ITR, ithresh);
}

static inline u8 bm_rcr_get_avail(struct bm_portal *portal)
{
	register struct bm_rcr *rcr = &portal->rcr;

	return rcr->available;
}

static inline u8 bm_rcr_get_fill(struct bm_portal *portal)
{
	register struct bm_rcr *rcr = &portal->rcr;

	return BM_RCR_SIZE - 1 - rcr->available;
}

#define SCN_REG(bpid) BM_REG_SCN((bpid) / 32)
#define SCN_BIT(bpid) (0x80000000 >> (bpid & 31))
static inline void bm_isr_bscn_mask(struct bm_portal *portal, u8 bpid,
					int enable)
{
	u32 val;

	/* REG_SCN for bpid=0..31, REG_SCN+4 for bpid=32..63 */
	val = bm_in(portal, SCN_REG(bpid));
	if (enable)
		val |= SCN_BIT(bpid);
	else
		val &= ~SCN_BIT(bpid);
	bm_out(portal, SCN_REG(bpid), val);
}

/* Disable all BSCN interrupts for the portal */
static inline void bm_isr_bscn_disable(struct bm_portal *portal)
{
	bm_out(portal, BM_REG_SCN(0), 0);
	bm_out(portal, BM_REG_SCN(1), 0);
}
