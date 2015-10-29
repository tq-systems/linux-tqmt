/* Copyright (c) 2009 - 2015 Freescale Semiconductor, Inc.
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

/* Register offsets */
#define REG_POOL_SWDET(n)	(0x0000 + ((n) * 0x04))
#define REG_POOL_HWDET(n)	(0x0100 + ((n) * 0x04))
#define REG_POOL_SWDXT(n)	(0x0200 + ((n) * 0x04))
#define REG_POOL_HWDXT(n)	(0x0300 + ((n) * 0x04))
#define REG_POOL_CONTENT(n)	(0x0600 + ((n) * 0x04))
#define REG_FBPR_FPC		0x0800
#define REG_ECSR		0x0a00
#define REG_ECIR		0x0a04
#define REG_EADR		0x0a08
#define REG_EDATA(n)		(0x0a10 + ((n) * 0x04))
#define REG_SBEC(n)		(0x0a80 + ((n) * 0x04))
#define REG_IP_REV_1		0x0bf8
#define REG_IP_REV_2		0x0bfc
#define REG_FBPR_BARE		0x0c00
#define REG_FBPR_BAR		0x0c04
#define REG_FBPR_AR		0x0c10
#define REG_SRCIDR		0x0d04
#define REG_LIODNR		0x0d08
#define REG_ERR_ISR		0x0e00
#define REG_ERR_IER		0x0e04
#define REG_ERR_ISDR		0x0e08

/* Used by all error interrupt registers except 'inhibit' */
#define BM_EIRQ_IVCI	0x00000010	/* Invalid Command Verb */
#define BM_EIRQ_FLWI	0x00000008	/* FBPR Low Watermark */
#define BM_EIRQ_MBEI	0x00000004	/* Multi-bit ECC Error */
#define BM_EIRQ_SBEI	0x00000002	/* Single-bit ECC Error */
#define BM_EIRQ_BSCN	0x00000001	/* pool State Change Notification */

struct bman_hwerr_txt {
	u32 mask;
	const char *txt;
};

static const struct bman_hwerr_txt bman_hwerr_txts[] = {
	{ BM_EIRQ_IVCI, "Invalid Command Verb" },
	{ BM_EIRQ_FLWI, "FBPR Low Watermark" },
	{ BM_EIRQ_MBEI, "Multi-bit ECC Error" },
	{ BM_EIRQ_SBEI, "Single-bit ECC Error" },
	{ BM_EIRQ_BSCN, "Pool State Change Notification" },
};

/* Only trigger low water mark interrupt once only */
#define BMAN_ERRS_TO_DISABLE BM_EIRQ_FLWI

static u16 bman_pool_max;

/* Pointer to the start of the BMan's CCSR space */
static u32 __iomem *bm_ccsr_start;

static inline u32 bm_ccsr_in(u32 offset)
{
	return ioread32be(bm_ccsr_start + offset/4);
}
static inline void bm_ccsr_out(u32 offset, u32 val)
{
	iowrite32be(val, bm_ccsr_start + offset/4);
}

static void bm_get_version(u16 *id, u8 *major, u8 *minor)
{
	u32 v = bm_ccsr_in(REG_IP_REV_1);
	*id = (v >> 16);
	*major = (v >> 8) & 0xff;
	*minor = v & 0xff;
}

static u32 generate_thresh(u32 val, int roundup)
{
	u32 e = 0;	/* co-efficient, exponent */
	int oddbit = 0;

	while (val > 0xff) {
		oddbit = val & 1;
		val >>= 1;
		e++;
		if (roundup && oddbit)
			val++;
	}
	DPA_ASSERT(e < 0x10);
	return val | (e << 8);
}

static void bm_set_memory(u64 ba, int prio, u32 size)
{
	u32 exp = ilog2(size);
	/* choke if size isn't within range */
	DPA_ASSERT((size >= 4096) && (size <= 1073741824) &&
			is_power_of_2(size));
	/* choke if '[e]ba' has lower-alignment than 'size' */
	DPA_ASSERT(!(ba & (size - 1)));
	bm_ccsr_out(REG_FBPR_BARE, upper_32_bits(ba));
	bm_ccsr_out(REG_FBPR_BAR, lower_32_bits(ba));
	bm_ccsr_out(REG_FBPR_AR, (prio ? 0x40000000 : 0) | (exp - 1));
}

/* Location and size of BMan private memory */
static dma_addr_t fbpr_a;
static size_t fbpr_sz;

static int bman_fbpr(struct reserved_mem *rmem)
{
	fbpr_a = rmem->base;
	fbpr_sz = rmem->size;

	WARN_ON(!(fbpr_a && fbpr_sz));

	return 0;
}
RESERVEDMEM_OF_DECLARE(bman_fbpr, "fsl,bman-fbpr", bman_fbpr);

int bm_pool_set(u32 bpid, const u32 *thresholds)
{
	if (!bm_ccsr_start)
		return -ENODEV;

	DPA_ASSERT(bpid < bman_pool_max);

	bm_ccsr_out(REG_POOL_SWDET(bpid), generate_thresh(thresholds[0], 0));
	bm_ccsr_out(REG_POOL_SWDXT(bpid), generate_thresh(thresholds[1], 1));
	bm_ccsr_out(REG_POOL_HWDET(bpid), generate_thresh(thresholds[2], 0));
	bm_ccsr_out(REG_POOL_HWDXT(bpid), generate_thresh(thresholds[3], 1));

	return 0;
}
EXPORT_SYMBOL(bm_pool_set);

/* BMan interrupt handler */
static irqreturn_t bman_isr(int irq, void *ptr)
{
	u32 isr_val, ier_val, ecsr_val, isr_mask, i;
	struct device *dev = ptr;

	ier_val = bm_ccsr_in(REG_ERR_IER);
	isr_val = bm_ccsr_in(REG_ERR_ISR);
	ecsr_val = bm_ccsr_in(REG_ECSR);
	isr_mask = isr_val & ier_val;

	if (!isr_mask)
		return IRQ_NONE;

	for (i = 0; i < ARRAY_SIZE(bman_hwerr_txts); i++) {
		if (bman_hwerr_txts[i].mask & isr_mask) {
			dev_warn(dev, "ErrInt: %s\n", bman_hwerr_txts[i].txt);
			if (bman_hwerr_txts[i].mask & ecsr_val) {
				/* Re-arm error capture registers */
				bm_ccsr_out(REG_ECSR, ecsr_val);
			}
			if (bman_hwerr_txts[i].mask & BMAN_ERRS_TO_DISABLE) {
				dev_dbg(dev, "Un-enabling error 0x%x\n",
					bman_hwerr_txts[i].mask);
				ier_val &= ~bman_hwerr_txts[i].mask;
				bm_ccsr_out(REG_ERR_IER, ier_val);
			}
		}
	}
	bm_ccsr_out(REG_ERR_ISR, isr_val);

	return IRQ_HANDLED;
}

u32 bm_pool_free_buffers(u32 bpid)
{
	return bm_ccsr_in(REG_POOL_CONTENT(bpid));
}

/*
 * Debug support.  The following data can be queried:
 *
 * FPBR FPC   - FPBR Free pool count - indicates how much free space
 *              is available for BMan to track buffers help in HW
 * Pool Count - Queries the number of buffers currently available in
 *              the specific buffer pool
 * ERR ISR    - Queries the data of the BMan error interrupt status
 *              register
 * SBEC       - Single Bit ECC error count - Indicates number of ECC
 *              errors in the BMan internal stockpile memory
 */

static ssize_t show_fbpr_fpc(struct device *dev,
	struct device_attribute *dev_attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", bm_ccsr_in(REG_FBPR_FPC));
};

static ssize_t show_pool_count(struct device *dev,
	struct device_attribute *dev_attr, char *buf)
{
	u32 data;
	int i;

	if (kstrtoint(dev_attr->attr.name, 10, &i))
		return -EINVAL;
	data = bm_ccsr_in(REG_POOL_CONTENT(i));
	return snprintf(buf, PAGE_SIZE, "%d\n", data);
};

static ssize_t show_err_isr(struct device *dev,
	struct device_attribute *dev_attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%08x\n", bm_ccsr_in(REG_ERR_ISR));
};

static ssize_t show_sbec(struct device *dev,
	struct device_attribute *dev_attr, char *buf)
{
	int i;

	if (sscanf(dev_attr->attr.name, "sbec_%d", &i) != 1)
		return -EINVAL;
	return snprintf(buf, PAGE_SIZE, "%u\n", bm_ccsr_in(REG_SBEC(i)));
};

static DEVICE_ATTR(err_isr, S_IRUSR, show_err_isr, NULL);
static DEVICE_ATTR(fbpr_fpc, S_IRUSR, show_fbpr_fpc, NULL);

/*
 * Didn't use DEVICE_ATTR as 64 of this would be required.
 * Initialize them when needed.
 */
static char *name_attrs_pool_count;
static struct device_attribute *dev_attr_buffer_pool_count;

static DEVICE_ATTR(sbec_0, S_IRUSR, show_sbec, NULL);
static DEVICE_ATTR(sbec_1, S_IRUSR, show_sbec, NULL);

static struct attribute *bman_dev_attributes[] = {
	&dev_attr_fbpr_fpc.attr,
	&dev_attr_err_isr.attr,
	NULL
};

static struct attribute *bman_dev_ecr_attributes[] = {
	&dev_attr_sbec_0.attr,
	&dev_attr_sbec_1.attr,
	NULL
};

static struct attribute **bman_dev_pool_count_attributes;

/* root level */
static const struct attribute_group bman_dev_attr_grp = {
	.name = NULL,
	.attrs = bman_dev_attributes
};
static const struct attribute_group bman_dev_ecr_grp = {
	.name = "error_capture",
	.attrs = bman_dev_ecr_attributes
};
static struct attribute_group bman_dev_pool_countent_grp = {
	.name = "pool_count",
};

static int of_fsl_bman_probe(struct platform_device *ofdev)
{
	int ret, err_irq, i;
	struct device *dev = &ofdev->dev;
	struct device_node *node = dev->of_node;
	struct resource *res;
	u16 id;
	u8 major, minor;

	res = platform_get_resource(ofdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Can't get %s property 'IORESOURCE_MEM'\n",
			node->full_name);
		return -ENXIO;
	}
	bm_ccsr_start = devm_ioremap(dev, res->start,
				     res->end - res->start + 1);
	if (!bm_ccsr_start)
		return -ENXIO;

	bm_get_version(&id, &major, &minor);
	dev_info(dev, "Bman ver:%04x,%02x,%02x\n", id, major, minor);
	if (major == 1 && minor == 0)
		bman_pool_max = 64;
	else if (major == 2 && minor == 0)
		bman_pool_max = 8;
	else if (major == 2 && minor == 1)
		bman_pool_max = 64;
	else {
		dev_err(dev, "Unknown Bman version:%04x,%02x,%02x\n",
			id, major, minor);
		return -ENODEV;
	}

	bm_set_memory(fbpr_a, 0, fbpr_sz);

	err_irq = platform_get_irq(ofdev, 0);
	if (err_irq <= 0) {
		dev_info(dev, "Can't get %s IRQ\n", node->full_name);
		return -ENODEV;
	}
	ret = devm_request_irq(dev, err_irq, bman_isr, IRQF_SHARED, "bman-err",
			       dev);
	if (ret)  {
		dev_err(dev, "devm_request_irq() failed %d for '%s'\n",
			ret, node->full_name);
		return ret;
	}
	/* Disable Buffer Pool State Change */
	bm_ccsr_out(REG_ERR_ISDR, BM_EIRQ_BSCN);
	/*
	 * Write-to-clear any stale bits, (eg. starvation being asserted prior
	 * to resource allocation during driver init).
	 */
	bm_ccsr_out(REG_ERR_ISR, 0xffffffff);
	/* Enable Error Interrupts */
	bm_ccsr_out(REG_ERR_IER, 0xffffffff);

	ret = sysfs_create_group(&dev->kobj, &bman_dev_attr_grp);
	if (ret)
		goto done;
	ret = sysfs_create_group(&dev->kobj, &bman_dev_ecr_grp);
	if (ret)
		goto del_group_0;

	name_attrs_pool_count = devm_kmalloc(dev,
		sizeof(char) * bman_pool_max * 3, GFP_KERNEL);
	if (!name_attrs_pool_count)
		goto del_group_1;

	dev_attr_buffer_pool_count = devm_kmalloc(dev,
		sizeof(struct device_attribute) * bman_pool_max, GFP_KERNEL);
	if (!dev_attr_buffer_pool_count)
		goto del_group_1;

	bman_dev_pool_count_attributes = devm_kmalloc(dev,
		sizeof(struct attribute *) * (bman_pool_max + 1), GFP_KERNEL);
	if (!bman_dev_pool_count_attributes)
		goto del_group_1;

	for (i = 0; i < bman_pool_max; i++) {
		ret = scnprintf((name_attrs_pool_count + i * 3), 3, "%d", i);
		if (!ret)
			goto del_group_1;
		dev_attr_buffer_pool_count[i].attr.name =
			(name_attrs_pool_count + i * 3);
		dev_attr_buffer_pool_count[i].attr.mode = S_IRUSR;
		dev_attr_buffer_pool_count[i].show = show_pool_count;
		bman_dev_pool_count_attributes[i] =
			&dev_attr_buffer_pool_count[i].attr;
	}
	bman_dev_pool_count_attributes[bman_pool_max] = NULL;

	bman_dev_pool_countent_grp.attrs = bman_dev_pool_count_attributes;

	ret = sysfs_create_group(&dev->kobj, &bman_dev_pool_countent_grp);
	if (ret)
		goto del_group_1;

	bman_seed_bpid_range(0, bman_pool_max);
	goto done;

del_group_1:
	sysfs_remove_group(&dev->kobj, &bman_dev_ecr_grp);
del_group_0:
	sysfs_remove_group(&dev->kobj, &bman_dev_attr_grp);
done:
	if (ret)
		dev_err(dev, "Cannot create dev attributes ret=%d\n", ret);

	return ret;
};

static const struct of_device_id of_fsl_bman_ids[] = {
	{
		.compatible = "fsl,bman",
	},
	{}
};

static struct platform_driver of_fsl_bman_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = of_fsl_bman_ids,
		.suppress_bind_attrs = true,
	},
	.probe = of_fsl_bman_probe,
};

builtin_platform_driver(of_fsl_bman_driver);
