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

#ifndef __DPAA_SYS_H
#define __DPAA_SYS_H

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/ctype.h>
#include <asm/pgtable.h>
#ifdef CONFIG_HOTPLUG_CPU
#include <linux/cpu.h>
#endif

struct dpaa_resource {
	struct list_head free;
	spinlock_t lock;
	struct list_head used;
};

#define DECLARE_DPAA_RESOURCE(name)			\
struct dpaa_resource name = {				\
	.free = LIST_HEAD_INIT(name.free),		\
	.lock = __SPIN_LOCK_UNLOCKED(name.lock),	\
	.used = LIST_HEAD_INIT(name.used)               \
}

int dpaa_resource_new(struct dpaa_resource *alloc, u32 *result,
		      u32 count, u32 align, int partial);
u32 dpaa_resource_release(struct dpaa_resource *alloc,
			  u32 id, u32 count, int (*is_valid)(u32 id));
void dpaa_resource_seed(struct dpaa_resource *alloc, u32 base_id, u32 count);
int dpaa_resource_reserve(struct dpaa_resource *alloc, u32 base, u32 num);

/* For 2-element tables related to cache-inhibited and cache-enabled mappings */
#define DPA_PORTAL_CE 0
#define DPA_PORTAL_CI 1

/* Misc inline assists */
#if (L1_CACHE_BYTES == 32)
/* Inline cache manipulations for 32 byte cache lines */
static inline void dpaa_flush(void *p)
{
	__asm__ __volatile__ ("dcbf 0,%0" : : "r" (p+32) : "memory");
	__asm__ __volatile__ ("dcbf 0,%0" : : "r" (p) : "memory");
}
static inline void dpaa_zero(void *p)
{
	__asm__ __volatile__ ("dcbz 0,%0" : : "r" (p+32));
	__asm__ __volatile__ ("dcbz 0,%0" : : "r" (p));
}
static inline void dpaa_invalidate(const void *p)
{
	__asm__ __volatile__ ("dcbi 0,%0" : : "r" (p+32));
	__asm__ __volatile__ ("dcbi 0,%0" : : "r" (p));
}
static inline void dpaa_touch_ro(const void *p)
{
	__asm__ __volatile__ ("dcbt 0,%0" : : "r" (p+32));
	__asm__ __volatile__ ("dcbt 0,%0" : : "r" (p));
}
static inline void dpaa_touch_rw(void *p)
{
	__asm__ __volatile__ ("dcbtst 0,%0" : : "r" (p+32));
	__asm__ __volatile__ ("dcbtst 0,%0" : : "r" (p));
}
#elif (L1_CACHE_BYTES == 64)
/* Inline cache manipulations for 64 byte cache lines */
static inline void dpaa_flush(void *p)
{
	__asm__ __volatile__ ("dcbf 0,%0" : : "r" (p) : "memory");
}
static inline void dpaa_zero(void *p)
{
	__asm__ __volatile__ ("dcbzl 0,%0" : : "r" (p));
}
static inline void dpaa_invalidate(const void *p)
{
	__asm__ __volatile__ ("dcbi 0,%0" : : "r" (p) : "memory");
}
static inline void dpaa_touch_ro(const void *p)
{
	__asm__ __volatile__ ("dcbt 0,%0" : : "r" (p));
}
static inline void dpaa_touch_rw(void *p)
{
	__asm__ __volatile__ ("dcbtst 0,%0" : : "r" (p));
}
#else
#error "Unsupported Cacheline Size"
#endif

/* Commonly used combo */
static inline void dpaa_invalidate_touch_ro(const void *p)
{
	dpaa_invalidate(p);
	dpaa_touch_ro(p);
}


#ifdef CONFIG_FSL_DPA_CHECKING
#define DPA_ASSERT(x) WARN_ON(!(x))
#else
#define DPA_ASSERT(x)
#endif

/* Bootargs */

/*
 * QMan has "qportals=" and BMan has "bportals=", they use the same syntax
 * though; a comma-separated list of items, each item being a cpu index and/or a
 * range of cpu indices, and each item optionally be prefixed by "s" to indicate
 * that the portal associated with that cpu should be shared. See bman_driver.c
 * for more specifics.
 */
static int __parse_portals_cpu(const char **s, unsigned int *cpu)
{
	*cpu = 0;
	if (!isdigit(**s))
		return -EINVAL;
	while (isdigit(**s))
		*cpu = *cpu * 10 + (*((*s)++) - '0');
	return 0;
}
static inline int parse_portals_bootarg(char *str, struct cpumask *want_shared,
					struct cpumask *want_unshared,
					const char *argname)
{
	const char *s = str;
	unsigned int shared, cpu1, cpu2, loop;

keep_going:
	if (*s == 's') {
		shared = 1;
		s++;
	} else
		shared = 0;
	if (__parse_portals_cpu(&s, &cpu1))
		goto err;
	if (*s == '-') {
		s++;
		if (__parse_portals_cpu(&s, &cpu2))
			goto err;
		if (cpu2 < cpu1)
			goto err;
	} else
		cpu2 = cpu1;
	for (loop = cpu1; loop <= cpu2; loop++)
		cpumask_set_cpu(loop, shared ? want_shared : want_unshared);
	if (*s == ',') {
		s++;
		goto keep_going;
	} else if ((*s == '\0') || isspace(*s))
		return 0;
err:
	pr_crit("Malformed %s argument: %s, offset: %lu\n", argname, str,
		(unsigned long)s - (unsigned long)str);
	return -EINVAL;
}
#endif	/* __DPAA_SYS_H */
