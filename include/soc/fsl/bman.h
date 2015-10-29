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

#ifndef __FSL_BMAN_H
#define __FSL_BMAN_H

/**
 * bman_alloc_bpid_range - Allocate a contiguous range of BPIDs
 * @result: is set by the API to the base BPID of the allocated range
 * @count: the number of BPIDs required
 * @align: required alignment of the allocated range
 * @partial: non-zero if the API can return fewer than @count BPIDs
 *
 * Returns the number of buffer pools allocated, or a negative error code. If
 * @partial is non zero, the allocation request may return a smaller range of
 * BPs than requested (though alignment will be as requested). If @partial is
 * zero, the return value will either be 'count' or negative.
 */
int bman_alloc_bpid_range(u32 *result, u32 count, u32 align, int partial);
static inline int bman_alloc_bpid(u32 *result)
{
	int ret = bman_alloc_bpid_range(result, 1, 0, 0);

	return (ret > 0) ? 0 : ret;
}

/**
 * bman_release_bpid_range - Release the specified range of buffer pool IDs
 * @bpid: the base BPID of the range to deallocate
 * @count: the number of BPIDs in the range
 *
 * This function can also be used to seed the allocator with ranges of BPIDs
 * that it can subsequently allocate from.
 */
void bman_release_bpid_range(u32 bpid, unsigned int count);
static inline void bman_release_bpid(u32 bpid)
{
	bman_release_bpid_range(bpid, 1);
}

int bman_reserve_bpid_range(u32 bpid, unsigned int count);
static inline int bman_reserve_bpid(u32 bpid)
{
	return bman_reserve_bpid_range(bpid, 1);
}

void bman_seed_bpid_range(u32 bpid, unsigned int count);

#endif	/* __FSL_BMAN_H */
