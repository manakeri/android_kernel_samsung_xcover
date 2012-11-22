/*
 *  linux/arch/arm/mach-mmp/pxa688_audiosram.c
 *
 *  based on Intel Memory Management
 *
 *  SRAM Allocation API
 *
 *  Todd Brandt
 *  Copyright (c) 2004, Intel Corporation (todd.e.brandt@intel.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.

 *(C) Copyright 2006 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/mm.h>
#include <linux/rmap.h>
#include <linux/vmalloc.h>
#include <linux/linkage.h>
#include <linux/compiler.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <linux/profile.h>
#include <linux/platform_device.h>
#include <linux/semaphore.h>

#include <asm/system.h>
#include <asm/pgtable.h>
#include <asm/page.h>
#include <asm/pgalloc.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>
#include <asm/mmu_context.h>
#include <asm/pgalloc.h>
#include <asm/tlbflush.h>
#include <asm/sizes.h>

#include <mach/mmp2_audiosram.h>

int audio_sram_sram_start;
int audio_sram_sram_size;

int audio_sram_sram_pages;
int audio_sram_malloc_map_size;

/* the audio_sram semaphore */
DECLARE_MUTEX(audio_sram_sem);

/* flag showing that this module was initialized */
static int module_initialized = 0;

u32 sram_num_free_sram_pages;

/* kernel level virtual memory list ordered only by allocation start address */
struct audio_sram_virt_t audio_malloc_list = {
	0, 0, 0, &audio_malloc_list, &audio_malloc_list
};

/* physical memory page list */
struct audio_sram_page_t *sram_page_list;

/* audio_sram_id list */
struct audio_sram_info_t sramid_list;
static u32 kernel_sramid = AUDIO_SRAM_FIRST_KERNEL_ID;
static int audio_sram_initialized = 0;

static int page_compatible(u32 start, u32 flags, u32 sramid);

/* error list */
const char *audio_sram_errors[] = {
	"Operation completed successfully",
	"System memory is too low, kernel malloc failed",
	"Insufficient space left",
	"The size argument is too small, size must be > 0 bytes",
	"The requested entry does not exist",
	"The address range requested is off limits to this process",
	"This functionality is currently unsupported",
	"The requested address range overlaps with an existing entry's \
		address range",
	"The requested audio sram module failed to initialize due to \
		a kernel error",
	"Unknown Error!"
};

#ifdef CONFIG_AUDIO_SRAM_DEBUG
#define audio_sram_debug(s, args...)   printk(KERN_DEBUG s, ## args)
#else
#define audio_sram_debug(s, args...)
#endif
#define audio_sram_failure(s)	printk(KERN_ERR "%s: file %s, line %d\n", s, __FILE__, __LINE__)

/*----------------------------------------------------------------------
 *
 *  High Level Call Tree:
 *
 *  [Application or Driver Call]
 *	audio_sram_malloc
 *		page_compatible
 *		audio_sram_alloc_pages
 *			audio_sram_map_page
 *	audio_sram_free
 *		release_entry
 *			audio_sram_free_pages
 *				audio_sram_unmap_page
 *	audio_sram_get_freespace
 *
 *  [Process Exit - from Policy Manager]
 *	audio_sram_free_sramid
 *		release_sramid
 *		audio_sram_free_pages_sramid
 *
 *  [Module Initialization - from Policy Manager]
 *	audio_sram_dau_sram_init
 *
 *--------------------------------------------------------------------*/

#ifdef CONFIG_AUDIO_SRAM_DEBUG
extern struct audio_sram_info_t sramid_list;
char *plist;

void set_plist(int s, int d, int c)
{
	static char plist_ch = 'A';

	switch (c) {
	case 0:		/* map */
		plist[s] = plist_ch;
		break;
	case 1:		/* unmap */
		plist[s] = '.';
		break;
	case 2:		/* remap */
		plist[d] = plist[s];
		plist[s] = '.';
		break;
	case 3:		/* swap */
		plist[d] = plist[s];
		plist[s] = '.';
		break;
	case 4:		/* update char */
		plist_ch++;
		if (plist_ch > 'Z')
			plist_ch = 'A';
		break;
	}
}

void print_page_list(char *s)
{
	int i, count, space = 0;
	struct audio_sram_page_t *p;

	if (!strcmp(s, "malloc"))
		set_plist(-1, -1, 4);

	printk(KERN_DEBUG "Page list changed as a result of %s\n", s);

	space = sram_num_free_sram_pages;
	printk(KERN_DEBUG "%d pages total, %d free\n",
	       &sram_page_list[audio_sram_sram_pages] -
	       &sram_page_list[0], space);
	for (count = 0, i = 0, p = &sram_page_list[0];
	     p < &sram_page_list[audio_sram_sram_pages]; p++, i++) {
		if (!PAGE_IS_MAPPED(p->flag))
			count++;
		printk(KERN_DEBUG "%c", plist[p->index]);
		if ((i + 1) % 64 == 0)
			printk(KERN_DEBUG "\n");
	}
	if (i % 64)
		printk(KERN_DEBUG "\n");
	if (count != space)
		printk(KERN_WARNING
		       "Free Count Mismatch, actual free pages is %d\n", count);
}

void print_page_map(int num, struct audio_sram_page_t **page_map, s16 num_pages)
{
	int i;

	printk(KERN_DEBUG "Scan %d reveals this page_map:\n", num);
	for (i = 0; i < num_pages; i++) {
		printk(KERN_DEBUG "%03d:%03d ", i,
		       (page_map[i]) ? page_map[i]->index : -1);
		if ((i + 1) % 9 == 0)
			printk(KERN_DEBUG "\n");
	}
	if ((i + 1) % 9)
		printk(KERN_DEBUG "\n");
}
#endif

static int remap_area_pte(pmd_t * pmd, unsigned long addr, unsigned long end,
			  unsigned long phys_addr, pgprot_t prot)
{
	pte_t *pte;

	pte = pte_alloc_kernel(pmd, addr);
	if (!pte)
		return -ENOMEM;

	do {
		if (!pte_none(*pte))
			goto bad;

		set_pte_ext(pte, pfn_pte(phys_addr >> PAGE_SHIFT, prot), 0);
		phys_addr += PAGE_SIZE;
	} while (pte++, addr += PAGE_SIZE, addr != end);
	return 0;

bad:
	printk(KERN_CRIT "remap_area_pte: page already exists\n");
	BUG();
}

static inline int remap_area_pmd(pgd_t * pgd, unsigned long addr,
				 unsigned long end, unsigned long phys_addr,
				 pgprot_t prot)
{
	unsigned long next;
	pmd_t *pmd;
	int ret = 0;

	pmd = pmd_alloc(&init_mm, pgd, addr);
	if (!pmd)
		return -ENOMEM;

	do {
		next = pmd_addr_end(addr, end);
		ret = remap_area_pte(pmd, addr, next, phys_addr, prot);
		if (ret)
			return ret;
		phys_addr += next - addr;
	} while (pmd++, addr = next, addr != end);
	return ret;
}

int sram_remap_area_pages(unsigned long start, unsigned long pfn,
			  unsigned long size, unsigned long flags)
{
	unsigned long addr = start;
	unsigned long next, end = start + size;
	unsigned long phys_addr = __pfn_to_phys(pfn);
	pgprot_t prot = __pgprot(L_PTE_PRESENT | L_PTE_YOUNG |
				 L_PTE_DIRTY | L_PTE_WRITE | flags);
	pgd_t *pgd;
	int err = 0;

	BUG_ON(addr >= end);
	pgd = pgd_offset_k(addr);
	do {
		next = pgd_addr_end(addr, end);
		err = remap_area_pmd(pgd, addr, next, phys_addr, prot);
		if (err)
			break;
		phys_addr += next - addr;
	} while (pgd++, addr = next, addr != end);

	return err;
}

/*************************************************************************
 *
 * Audio SRAM Single Page Functions
 *
 * audio_sram_map_page: create a new virtual-physical page mapping
 * audio_sram_unmap_page: remove a virtual-physical page mapping
 *
 *************************************************************************/
void audio_sram_map_page(struct audio_sram_page_t *page, u32 virtual, u32 flags,
			 struct audio_sram_info_t *audio_sram_info)
{
	pgd_t *dir;
	u32 f;

#ifdef CONFIG_AUDIO_SRAM_DEBUG
	set_plist(page->index, -1, 0);
#endif

	sram_num_free_sram_pages--;

	page->flag |= AUDIO_SRAM_INFO_USED | (flags & AUDIO_SRAM_INFO_DMA);
	page->flag &= ~AUDIO_SRAM_INFO_RESERVED;
	page->sramid = audio_sram_info->sramid;
	page->virt_addr = virtual;

	dir = pgd_offset_k(page->virt_addr);

	/* the initial cacheability depends on the HARDWARE flag */
	if (PAGE_IS_DMA(flags))
		f = L_PTE_EXEC;
	else
		f = L_PTE_BUFFERABLE | L_PTE_CACHEABLE;

	/* now map in the underlying physical memory */
	if (sram_remap_area_pages(page->virt_addr, (page->phys_addr)
				  >> PAGE_SHIFT, PAGE_SIZE, f))
		audio_sram_failure("remap_area_pages failed");
}

void audio_sram_unmap_page_lite(struct audio_sram_page_t *page,
				struct audio_sram_info_t *audio_sram_info)
{

#ifdef CONFIG_AUDIO_SRAM_DEBUG
	set_plist(page->index, -1, 1);
#endif

	sram_num_free_sram_pages++;

	page->flag &= AUDIO_SRAM_INFO_SRAM;
	page->sramid = 0;
	page->virt_addr = 0;
}

extern void unmap_kernel_range(unsigned long addr, unsigned long size);

void audio_sram_unmap_page(struct audio_sram_page_t *page,
			   struct audio_sram_info_t *audio_sram_info)
{
	/* if the page isn't mapped, forget it */
	if (PAGE_IS_MAPPED(page->flag))
		unmap_kernel_range(page->virt_addr, PAGE_SIZE);

	audio_sram_unmap_page_lite(page, audio_sram_info);
}

/* associate an error with an sramid */
void audio_sram_register_error(u32 sramid, u32 err)
{
	struct audio_sram_info_t *head, *p;

	head = &sramid_list;
	p = head->next;

	/* loop through the list til you hit the end or the sramid */
	while ((p != head) && (p->sramid != sramid))
		p = p->next;
	if (p->sramid == sramid)
		p->last_error = err;
}

/*************************************************************************
 *
 * AUDIO_SRAM Page Range Functions
 *
 * audio_sram_alloc_pages: given a valid virtual address range, map available
 *   physical pages.
 * audio_sram_free_pages: frees a specific range of physical pages
 * audio_sram_free_pages_sramid: frees all physical pages for a given sramid
 *
 *************************************************************************/

int audio_sram_alloc_pages(u32 virt, u32 size, u32 flags,
			   struct audio_sram_info_t *audio_sram_info)
{
	int i;
	u32 virt_start = virt & PAGE_MASK;
	u32 virt_end = PAGE_ALIGN(virt + size);
	u32 pstart, pend, sramid = audio_sram_info->sramid;
	u32 pages_needed, pages_found = 0;
	struct audio_sram_page_t **page_map, **page_map_ptr, *p;

	audio_sram_debug("audio_sram_alloc_pages: virt=%08X, "
			 " size=%d, flags=%03X, sramid=%d\n",
			 virt, size, flags, sramid);

	if ((virt % PAGE_SIZE) && page_compatible(virt, flags, sramid))
		pages_needed =
		    (virt_end - (virt_start + PAGE_SIZE)) / PAGE_SIZE;
	else
		pages_needed = (virt_end - virt_start) / PAGE_SIZE;

	/* if there aren't enough pages left leave now */
	if (pages_needed > sram_num_free_sram_pages)
		return AUDIO_SRAM_ERROR_NOSPACE;

	/* create a physical page mapping for the new allocation */
	page_map_ptr = kmalloc((pages_needed + 2)
			       * sizeof(struct audio_sram_page_t *),
			       GFP_ATOMIC);
	if (page_map_ptr == NULL)
		return AUDIO_SRAM_ERROR_KMALLOC;

	/* add two extra places in case we find border pages */
	for (i = 0; i < pages_needed + 2; i++)
		page_map_ptr[i] = NULL;

	page_map = &page_map_ptr[1];

	if (!PAGE_IS_DMA(flags)) {
		/* look for new and border pages */
		for (p = &sram_page_list[0];
		     (p < &sram_page_list[audio_sram_sram_pages])
		     && (pages_found < pages_needed); p++) {
			/* check all memory types for border pages */
			if (PAGE_IS_MAPPED(p->flag) &&
			    !PAGE_IS_DMA(p->flag) && (p->sramid == sramid)) {
				pstart = p->virt_addr;
				pend = p->virt_addr + PAGE_SIZE;
				if (virt_start == pstart) {
					pages_found++;
					page_map_ptr[0] = p;
				} else if (virt_end == pend) {
					pages_found++;
					page_map_ptr[pages_needed + 1] = p;
				}
			}
			if (!PAGE_IS_MAPPED(p->flag)) {
				page_map[pages_found++] = p;
				p->flag |= AUDIO_SRAM_INFO_RESERVED;
			}
		}

		/* we may have found up to two more than pages_needed */
		pages_found = min(pages_found, pages_needed);

		/* if we have found a starting border page, move the
		 * map forward */
		if (page_map_ptr[0])
			page_map[0] = page_map_ptr[0];

		/* if we have found an ending border page, overwrite
		 * the last page with it */
		if (page_map_ptr[pages_needed + 1])
			page_map[pages_needed - 1] =
			    page_map_ptr[pages_needed + 1];
	} else {
		/* next scan to pick up needed new pages */
		for (p = &sram_page_list[0], pages_found = 0;
		     (p < &sram_page_list[audio_sram_sram_pages])
		     && (pages_found < pages_needed); p++) {
			if (!PAGE_IS_MAPPED(p->flag))
				page_map[pages_found++] = p;
			else
				pages_found = 0;
		}
	}

	if (pages_found < pages_needed) {
		kfree(page_map_ptr);
		return AUDIO_SRAM_ERROR_NOSPACE;
	}
#ifdef CONFIG_AUDIO_SRAM_DEBUG
	for (i = 0; i < pages_needed; i++) {
		printk(KERN_DEBUG
		       "V(0x%08X) --> P(0x%08X), page index = %03d, %s\n",
		       (u32) (virt_start + (i * PAGE_SIZE)),
		       page_map[i]->phys_addr, page_map[i]->index,
		       (PAGE_IS_MAPPED(page_map[i]->flag)) ? "USED" : "NEW");
	}
#endif

	for (i = 0; i < pages_needed; i++) {
		p = page_map[i];
		if (!PAGE_IS_MAPPED(p->flag)) {
			audio_sram_map_page(p, virt_start + (i * PAGE_SIZE),
					    flags, audio_sram_info);
		}
	}

	/* flush the cache and TLBs afterward to ensure the mapping took */
	flush_cache_all();
	flush_tlb_all();

	kfree(page_map_ptr);
	return AUDIO_SRAM_ERROR_NONE;
}

void audio_sram_free_pages(u32 virt, u32 size,
			   struct audio_sram_info_t *audio_sram_info)
{
	struct audio_sram_page_t *p;
	u32 virt_start = virt & PAGE_MASK;
	u32 virt_end = PAGE_ALIGN(virt + size);
	s16 pages_freed, pages_found = 0;

	pages_freed = (virt_end - virt_start) / PAGE_SIZE;
	if (unlikely(!pages_freed)) {
		audio_sram_failure("audio_sram_free_pages called with 0 size");
		return;
	}

	/* first scan to pick up needed pages that are already mapped */
	for (p = &sram_page_list[0];
	     (p < &sram_page_list[audio_sram_sram_pages])
	     && (pages_found < pages_freed); p++) {
		if (PAGE_IS_MAPPED(p->flag)
		    && (p->sramid == audio_sram_info->sramid)
		    && ((p->virt_addr >= virt_start)
			&& (p->virt_addr < virt_end))) {
			audio_sram_unmap_page(p, audio_sram_info);
			pages_found++;
		}
	}

	flush_cache_all();
	flush_tlb_all();
}

int audio_sram_free_pages_sramid(struct audio_sram_info_t *audio_sram_info)
{
	struct audio_sram_page_t *p;
	u32 sramid = audio_sram_info->sramid;

	/* first scan to pick up needed pages that are already mapped */
	for (p = &sram_page_list[0];
	     p < &sram_page_list[audio_sram_sram_pages]; p++) {
		if (PAGE_IS_MAPPED(p->flag) && (p->sramid == sramid)) {
			audio_sram_unmap_page_lite(p, audio_sram_info);
		}
	}

	flush_cache_all();
	flush_tlb_all();

	return 0;
}

/*************************************************************************
 * virtual allocation list handlers
 *
 * release_entry: frees a specific malloc, or cacheability entry
 * release_sramid: frees all instances of mallocs, hotswaps, or cacheability
 *   changes for a given sramid.
 * page_compatible: (kernel pages only) determines if a given page is
 *   compatible with a new allocation's flags, this is necessary if a new
 *   allocation shares a page with an existing allocation
 *************************************************************************/

int sram_release_entry(void *address, struct audio_sram_info_t *audio_sram_info)
{
	struct audio_sram_virt_t *head = audio_sram_info->malloc_list,
	    *p = audio_sram_info->malloc_list->next;
	u32 start = 0, end = 0;
	u32 sramid = audio_sram_info->sramid;
	int found = 0;

	while (p != head) {
		if ((p->sramid == sramid)
		    && (p->start == (u32) address)) {
			/* calculate the page address range */
			start = p->start & PAGE_MASK;
			end = PAGE_ALIGN(p->end);

			/* we need to see if other allocations overlap
			   in the beginning or end pages of the range */
			if ((p->prev->sramid == sramid)
			    && (p->prev->end > start)) {
				start += PAGE_SIZE;
			}
			if ((p->next->sramid == sramid)
			    && (p->next->start < end)) {
				end -= PAGE_SIZE;
			}
			found = 1;
			break;
		}
		p = p->next;
	}

	if (!found)
		return AUDIO_SRAM_ERROR_ENTRY_NOTFOUND;

	audio_sram_info->used_space -= p->end - p->start;

	/* p now points to the memory block to be removed */
	p->prev->next = p->next;
	p->next->prev = p->prev;
	kfree(p);

	if (start < end)
		audio_sram_free_pages(start, end - start, audio_sram_info);

	return AUDIO_SRAM_ERROR_NONE;
}

int sram_release_sramid(struct audio_sram_info_t *audio_sram_info)
{
	struct audio_sram_virt_t *head = audio_sram_info->malloc_list,
	    *p = audio_sram_info->malloc_list->next;
	u32 sramid = audio_sram_info->sramid;

	while (p != head) {
		if (p->sramid == sramid) {
			/* p now points to the memory block to be removed */
			p->prev->next = p->next;
			p->next->prev = p->prev;
			kfree(p);
		}
		p = p->next;
	}
	return 0;
}

int page_compatible(u32 start, u32 flags, u32 sramid)
{
	u32 vstart = start & PAGE_MASK;
	struct audio_sram_page_t *p;

	/* if the dma flag is set, the page is not compatible */
	/* dma pages can't be moved, non-dma pages can, so */
	/* allocations can't share both */
	if (PAGE_IS_DMA(flags))
		return 0;

	/* find the page in question */
	for (p = &sram_page_list[0];
	     p < &sram_page_list[audio_sram_sram_pages]; p++) {
		if (SRAMID_OVERLAP(p->sramid, sramid)
		    && PAGE_IS_MAPPED(p->flag)
		    && ((p->virt_addr & PAGE_MASK) == vstart))
			break;
	}

	/* if the page isn't found, it's not compatible (shouldn't */
	/* occur, but just to be safe handle the error) */
	if (p >= &sram_page_list[audio_sram_sram_pages])
		return 0;

	/* if the dma flag is set, the page is not compatible */
	/* dma pages can't be moved, non-dma pages can, so */
	/* allocations can't share both */
	if (PAGE_IS_DMA(p->flag))
		return 0;

	return 1;
}

/*************************************************************************
 * Function: audio_sram_malloc
 * Description: Allocates a block and returns the kernel/user space
 * address
 *
 * Arguments:
 * size - the size in bytes of the requested allocation
 * flags - malloc flags
 * sramid - the id to assign this malloc to
 *
 * Return Value: the new allocation address (NULL for failure)
 *************************************************************************/

void *audio_sram_malloc(size_t size, u32 flags, u32 sramid)
{
	struct audio_sram_virt_t *head, *p, *new;
	struct audio_sram_info_t *audio_sram_info;
	int res;
	u32 address = 0, end;

	down(&audio_sram_sem);
	audio_sram_debug("audio_sram_malloc: size=%d, flags=%02X, sramid=%u\n",
			 size, flags, sramid);

	/* if this client isn't registered, fail */
	audio_sram_info = audio_sram_info_getentry(sramid);
	if (audio_sram_info == NULL) {
		up(&audio_sram_sem);
		return NULL;
	}

	/* if this module isn't initialized (very rare), fail */
	if (!module_initialized) {
		audio_sram_register_error(sramid, AUDIO_SRAM_ERROR_UNINIT);
		up(&audio_sram_sem);
		return NULL;
	}

	/* if the size is invalid, fail */
	if (size < 1) {
		audio_sram_register_error(sramid, AUDIO_SRAM_ERROR_SIZETOOLOW);
		up(&audio_sram_sem);
		return NULL;
	}

	if ((size + audio_sram_info->used_space) > audio_sram_sram_size) {

		audio_sram_register_error(sramid, AUDIO_SRAM_ERROR_NOSPACE);
		up(&audio_sram_sem);
		return NULL;
	}

	p = head = audio_sram_info->malloc_list;

	/* request is valid, let's do it */
	while (!address) {
		p = p->next;
		if (p->start - p->prev->end >= size) {
			address = p->prev->end;
			/* if this is in the middle of a page, make sure */
			/* the existing page is compatible */
			if ((address % PAGE_SIZE) &&
			    !page_compatible(address, flags, sramid)) {
				/* if not, align the allocation to next page */
				address = PAGE_ALIGN(address);
				/* we may now not have enough space */
				if (p->start - address < size)
					address = 0;
			}
			if (address) {
				/* if we're still in business, check the end
				 * address
				 */
				end = address + size;
				if ((end % PAGE_SIZE) && ((end & PAGE_MASK)
							  == (p->
							      start &
							      PAGE_MASK))
				    && !page_compatible(end, flags, sramid)) {
					/* in the unlikely event that an
					 * allocation is perfectly sandwiched
					 * between two others, and the end
					 * page is incompatible, move on
					 */
					address = 0;
				}
			}
		}
		if (p == head)
			break;
	}

	if (!address) {
		audio_sram_register_error(sramid, AUDIO_SRAM_ERROR_NOSPACE);
		up(&audio_sram_sem);
		return NULL;
	}

	res = audio_sram_alloc_pages(address, size, flags, audio_sram_info);
	if (res != 0) {
		audio_sram_register_error(sramid, res);
		up(&audio_sram_sem);
		return NULL;
	}

	/* p now points to the memory block just after the new block's dest */
	new = kmalloc(sizeof(struct audio_sram_virt_t), GFP_ATOMIC);
	if (new == NULL) {
		audio_sram_register_error(sramid, AUDIO_SRAM_ERROR_KMALLOC);
		up(&audio_sram_sem);
		return NULL;
	}

	new->start = address;
	new->end = address + size;
	new->sramid = sramid;

	new->next = p;
	new->prev = p->prev;
	new->next->prev = new;
	new->prev->next = new;

	audio_sram_info->used_space += size;
#ifdef CONFIG_AUDIO_SRAM_DEBUG
	print_page_list("malloc");
#endif
	audio_sram_register_error(sramid, AUDIO_SRAM_ERROR_NONE);
	up(&audio_sram_sem);
	return (void *)new->start;
}

EXPORT_SYMBOL(audio_sram_malloc);

/*************************************************************************
 *
 * Function: audio_sram_free
 * Description: Frees a block of audio_sram memory
 *
 * Arguments:
 * ptr - the address of the block to free
 * flags - release flags, can be 0 or AUDIO_SRAM_NO_REMAP
 *
 * Return Value:
 * the error condition, can be sent to audio_sram_error to retrieve the
 * error string
 *************************************************************************/

int audio_sram_free(void *address, u32 sramid)
{
	int res;
	struct audio_sram_info_t *audio_sram_info;

	down(&audio_sram_sem);
	audio_sram_debug("audio_sram_free: address=%08X, "
			 "sramid=%d\n", (u32) address, sramid);

	/* if this client isn't registered, forget it */
	audio_sram_info = audio_sram_info_getentry(sramid);
	if (audio_sram_info == NULL) {
		up(&audio_sram_sem);
		return AUDIO_SRAM_ERROR_UNKNOWN;
	}

	/* if this module isn't initialized (very rare), forget it */
	if (!module_initialized) {
		audio_sram_register_error(sramid, AUDIO_SRAM_ERROR_UNINIT);
		up(&audio_sram_sem);
		return AUDIO_SRAM_ERROR_UNINIT;
	}

	if (address == NULL) {
		audio_sram_register_error(sramid,
					  AUDIO_SRAM_ERROR_ENTRY_NOTFOUND);
		up(&audio_sram_sem);
		return AUDIO_SRAM_ERROR_ENTRY_NOTFOUND;
	}

	res = sram_release_entry(address, audio_sram_info);
#ifdef CONFIG_AUDIO_SRAM_DEBUG
	print_page_list("free");
#endif
	audio_sram_register_error(sramid, res);
	up(&audio_sram_sem);
	return res;
}

EXPORT_SYMBOL(audio_sram_free);

/*************************************************************************
 *
 * Function: audio_sram_free_sramid
 * Description: Frees all audio_sram memory assigned to a given process, for
 * applications the sramid is the process id, for drivers the sramid is the
 * value retrieved from audio_sram_sramid().
 *
 * Arguments:
 * flags - release flags, can be 0 or AUDIO_SRAM_NO_REMAP
 * sramid - the sramid for whom all sram allocations should be freed
 *
 * Return Value:
 * the error condition, can be sent to audio_sram_error to retrieve the
 * error string
 *************************************************************************/

int audio_sram_free_sramid(u32 sramid)
{
	int res;
	struct audio_sram_info_t *audio_sram_info;

	/* if this client isn't registered, forget it */
	if (!module_initialized)
		return 0;
	else {
		audio_sram_info = audio_sram_info_getentry(sramid);
		if (audio_sram_info == NULL)
			return 0;
	}

	res = sram_release_sramid(audio_sram_info);
	res = audio_sram_free_pages_sramid(audio_sram_info);
	audio_sram_info->used_space = 0;
#ifdef CONFIG_AUDIO_SRAM_DEBUG
	print_page_list("task exit");
#endif
	return res;
}

EXPORT_SYMBOL(audio_sram_free_sramid);

/*************************************************************************
 *
 * Function: audio_sram_get_freespace
 * Description: Figure out how much SRAM space is left
 *
 * Arguments:
 * flags - flags for the type of memory to check the free space for
 * sramid - the sramid for the client who's asking
 *
 * Return Value:
 * (unsigned int) the amount of SRAM remaining in bytes
 *************************************************************************/

unsigned int audio_sram_get_freespace(u32 flags, u32 sramid)
{
	struct audio_sram_page_t *p;
	unsigned int bytes = audio_sram_sram_size;
	struct audio_sram_info_t *audio_sram_info;

	down(&audio_sram_sem);

	/* if this client isn't registered, forget it */
	if (!module_initialized) {
		up(&audio_sram_sem);
		return 0;
	} else {
		audio_sram_info = audio_sram_info_getentry(sramid);
		if (audio_sram_info == NULL) {
			up(&audio_sram_sem);
			return 0;
		}
	}

	for (p = &sram_page_list[0];
	     p < &sram_page_list[audio_sram_sram_pages]; p++) {
		if (PAGE_IS_MAPPED(p->flag)
		    && (sramid != p->sramid))
			bytes -= PAGE_SIZE;
	}
	bytes -= audio_sram_info->used_space;
	if (unlikely(bytes < 0))
		audio_sram_failure("used_space failure");

	up(&audio_sram_sem);
	return min(bytes, audio_sram_sram_size - audio_sram_info->used_space);
}

EXPORT_SYMBOL(audio_sram_get_freespace);

void audio_sram_dau_sram_init(void)
{
	struct audio_sram_page_t *p;
	struct vm_struct *kernel_area;
	int i;

	audio_sram_sram_pages = audio_sram_sram_size / PAGE_SIZE;
	sram_num_free_sram_pages = audio_sram_sram_pages;
	audio_sram_malloc_map_size = audio_sram_sram_size << 1;

	sram_page_list = kzalloc(sizeof(struct audio_sram_page_t)
				 * audio_sram_sram_pages, GFP_KERNEL);
#ifdef CONFIG_AUDIO_SRAM_DEBUG
	plist = kzalloc(sizeof(char) * audio_sram_sram_pages, GFP_KERNEL);
#endif

	/* grab a vm_area struct (from mm/vmalloc.c) */
	kernel_area = get_vm_area(audio_sram_malloc_map_size, VM_IOREMAP);
	if ((!kernel_area) || ((u32) kernel_area->addr == 0)) {
		audio_sram_failure("get_vm_area failed\n");
		return;
	}

	audio_malloc_list.start = (u32) kernel_area->addr
	    + audio_sram_malloc_map_size;
	audio_malloc_list.end = (u32) kernel_area->addr;

	for (i = 0, p = &sram_page_list[0];
	     p < &sram_page_list[audio_sram_sram_pages]; p++, i++) {
#ifdef CONFIG_AUDIO_SRAM_DEBUG
		p->index = i;
		plist[i] = '.';
#endif
		p->phys_addr = audio_sram_sram_start + i * PAGE_SIZE;
		p->flag = AUDIO_SRAM_INFO_SRAM;
		p->sramid = 0;
		p->virt_addr = 0;
	}

	module_initialized = 1;
	pr_info("Audio SRAM Management - SRAM Allocation\n");
}

/*************************************************************************
 *
 * Function: audio_sram_get_physical
 * Description: Retrieve the physical address for a given audio_sram virtual
 * address
 *
 * Arguments:
 * virtual - address returned from audio_sram_malloc
 * sramid - the id of the caller
 *
 * Return Value:
 * the physical address, 0 on failure
 *************************************************************************/

u32 audio_sram_get_physical(void *v, u32 sramid)
{
	pmd_t *pmd;
	pte_t *pte;
	pgd_t *pgd;
	u32 val = 0, virtual = (u32) v;
	struct mm_struct *mm;

	mm = &init_mm;

	pgd = pgd_offset(mm, virtual);
	if (!pgd_none(*pgd) && !pgd_bad(*pgd)) {
		/* 1st level entry pointer */
		pmd = pmd_offset(pgd, virtual);
		if (!pmd_none(*pmd) && !pmd_bad(*pmd)) {
			/* 2nd level entry pointer */
			pte = pte_offset_kernel(pmd, virtual);
			if (pte) {
				val = (*(u32 *) ((u32) pte - 2048)) & PAGE_MASK;
				val += virtual % PAGE_SIZE;
			}
		} else if (!pmd_none(*pmd)) {
			/* work around */
			val = (unsigned long)virt_to_phys((void *)virtual);
			/*
			 * old method address conversion is not correct
			 val = (*(u32 *)pmd) & 0xFFF00000;
			 val += virtual%0x100000;
			 */
		}
	}

	return val;
}

EXPORT_SYMBOL(audio_sram_get_physical);

/*************************************************************************
 *
 * Function: audio_sram_get_virtual
 * Description: Retrieve the virtual address for a given physical
 * address
 *
 * Arguments:
 * physical - address of some memory in the sramid's address space
 * sramid - the id of the caller
 *
 * Return Value:
 * the virtual address, 0 on failure
 *************************************************************************/

u32 audio_sram_get_virtual(u32 p, struct task_struct * tsk)
{
	pmd_t *pmd;
	pte_t *pte;
	pgd_t *pgd;
	u32 val, v;
	struct mm_struct *mm;
	struct vm_area_struct *vm_search;

	down(&audio_sram_sem);
	if ((tsk == NULL) || (tsk->mm == NULL))
		return 0;
	mm = tsk->mm;

	for (vm_search = mm->mmap; vm_search; vm_search = vm_search->vm_next) {
		for (v = vm_search->vm_start; v < vm_search->vm_end;
		     v += PAGE_SIZE) {
			pgd = pgd_offset(mm, v);
			if (!pgd_none(*pgd) && !pgd_bad(*pgd)) {
				pmd = pmd_offset(pgd, v);
				if (!pmd_none(*pmd) && !pmd_bad(*pmd)) {
					pte = pte_offset_kernel(pmd, v);
					if (pte) {
						val = (*(u32 *) ((u32)
								 pte -
								 2048)) &
						    PAGE_MASK;
						val += p % PAGE_SIZE;
						if (p == val) {
							up(&audio_sram_sem);
							return v +
							    (p % PAGE_SIZE);
						}
					}
				}
			}
		}
	}
	up(&audio_sram_sem);
	return 0;
}

EXPORT_SYMBOL(audio_sram_get_virtual);

/*************************************************************************
 * audio_sram_info list handlers
 ************************************************************************/

/* retrieve an audio_sram_info object from the list */
struct audio_sram_info_t *audio_sram_info_getentry(u32 sramid)
{
	struct audio_sram_info_t *head, *p;

	if (sramid < 1)
		return NULL;
	head = &sramid_list;
	p = head->next;

	/* loop through the list til you hit the end or the sramid */
	while ((p != head) && (p->sramid != sramid))
		p = p->next;
	if (p->sramid == sramid)
		return p;
	else
		return NULL;
}

/* create a new audio_sram_info entry for a new sramid */
static struct audio_sram_info_t *audio_sram_info_newentry(u32 sramid,
							  char *name,
							  u32
							  user_audio_sram_space)
{
	struct audio_sram_info_t *head, *p, *new;

	head = &sramid_list;
	p = head->next;

	/* loop through the list til you hit the end or the sramid */
	while ((p != head) && (p->sramid != sramid))
		p = p->next;
	if (p->sramid == sramid)
		return p;
	else {
		audio_sram_debug("audio_sram_info_newentry called, "
				 "initializing a new audio_sram_info entry\n");
		new = kmalloc(sizeof(struct audio_sram_info_t), GFP_ATOMIC);
		if (new == NULL) {
			audio_sram_debug("audio_sram_info_newentry failed, "
					 "kmalloc returned NULL: %s, line %d\n",
					 __FILE__, __LINE__);
			return NULL;
		}

		new->sramid = sramid;
		new->last_error = 0;
		memcpy(new->name, name, min(strlen(name) + 1,
					    (size_t)
					    (AUDIO_SRAM_CLIENT_NAME_SIZE - 1)));
		new->name[AUDIO_SRAM_CLIENT_NAME_SIZE - 1] = '\0';
		new->mm = &init_mm;
		new->malloc_list = &audio_malloc_list;
		new->used_space = 0;

		new->next = p;
		new->prev = p->prev;
		new->next->prev = new;
		new->prev->next = new;

		return new;
	}
}

/* remove an sramid from the list */
static int audio_sram_info_delentry(u32 sramid)
{
	struct audio_sram_info_t *head, *p;

	head = &sramid_list;
	p = head->next;

	/* loop through the list til you hit the end or the sramid */
	while ((p != head) && (p->sramid != sramid))
		p = p->next;
	if (p->sramid == sramid) {
		p->prev->next = p->next;
		p->next->prev = p->prev;
		kfree(p);
		return 0;
	}
	return -1;
}

static int audio_sram_task_exit(struct notifier_block *self,
				unsigned long val, void *data)
{
	struct task_struct *tsk = (struct task_struct *)data;

	down(&audio_sram_sem);
	if (tsk && audio_sram_initialized) {
		/* cache must be cleared first since some of it may be */
		/* locked over SRAM */
		audio_sram_free_sramid(tsk->pid);
		audio_sram_info_delentry(tsk->pid);
	}
	up(&audio_sram_sem);

	return NOTIFY_OK;
}

EXPORT_SYMBOL(audio_sram_task_exit);

static struct notifier_block audio_sram_task_exit_nb = {
	.notifier_call = audio_sram_task_exit,
};

/*************************************************************************
 *
 * Function: audio_sram_register(kernel and user)
 * Description: Create a new sramid instance for either the kernel or user
 *   space.
 *
 * Return Value:
 * sramid - a new unique id which will never conflict with user sramids
 *
 *************************************************************************/

u32 audio_sram_register_kernel(char *name)
{
	struct audio_sram_info_t *audio_sram_info;

	down(&audio_sram_sem);
	if (name == NULL)
		name = AUDIO_SRAM_DEFAULT_DRIVER_NAME;
	audio_sram_debug("audio_sram_register_kernel called for %s, "
			 "new ID = %08X\n", name, kernel_sramid);
	audio_sram_info = audio_sram_info_newentry(kernel_sramid, name, 0);
	if (audio_sram_info == NULL) {
		up(&audio_sram_sem);
		return 0;
	}
	up(&audio_sram_sem);
	return kernel_sramid++;
}

EXPORT_SYMBOL(audio_sram_register_kernel);

/*************************************************************************
 *
 * Function: audio_sram_error
 * Description: Return the last error for the given sramid
 *
 * Return Value:
 * a const char pointer of the error string
 *
 *************************************************************************/

const char *audio_sram_error(u32 sramid)
{
	struct audio_sram_info_t *head, *p;

	down(&audio_sram_sem);
	head = &sramid_list;
	p = head->next;

	/* loop through the list til you hit the end or the sramid */
	while ((p != head) && (p->sramid != sramid))
		p = p->next;
	if (p->sramid == sramid) {
		if ((p->last_error >= 0)
		    && (p->last_error < AUDIO_SRAM_ERROR_UNKNOWN)) {
			up(&audio_sram_sem);
			return audio_sram_errors[p->last_error];
		}
	}
	up(&audio_sram_sem);
	return "The caller is not a client of AUDIO_SRAM";
}

EXPORT_SYMBOL(audio_sram_error);

static int __init __audio_sram_init(void)
{
	int exit_cb_err = 1;

	kernel_sramid = AUDIO_SRAM_FIRST_KERNEL_ID;

	/* the exit callback is needed for SRAM and Cache garbage collection */
	exit_cb_err = profile_event_register(PROFILE_TASK_EXIT,
					     &audio_sram_task_exit_nb);
	if (exit_cb_err)
		printk(KERN_ERR "Process exit callback failed to register, \
				all APIs have been disabled\n");

	sramid_list.sramid = 0;
	sramid_list.last_error = 0;
	sramid_list.mm = NULL;
	sramid_list.prev = &sramid_list;
	sramid_list.next = &sramid_list;
	sramid_list.malloc_list = NULL;
	sramid_list.used_space = 0;

	/* if the exit cb is available */
	if (!exit_cb_err)
		audio_sram_dau_sram_init();

	pr_info("Audio SRAM Management is now Enabled\n");

	audio_sram_initialized = 1;

	return 0;
}

static int __init audio_sram_probe(struct platform_device *pdev)
{
	struct resource *res;
	int ret = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "no memory resource defined\n");
		ret = -ENODEV;
		return ret;
	}
	audio_sram_sram_start = res->start;
	audio_sram_sram_size = res->end - res->start + 1;

	__audio_sram_init();

	return 0;
}

static int audio_sram_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver audio_sram_driver = {
	.probe = audio_sram_probe,
	.remove = audio_sram_remove,
	.driver = {
		   .name = "mmp2-audiosram",
		   },
};

static int __init audio_sram_init(void)
{
	return platform_driver_register(&audio_sram_driver);
}

static void __exit audio_sram_exit(void)
{
	platform_driver_unregister(&audio_sram_driver);
}

module_init(audio_sram_init);
module_exit(audio_sram_exit);
MODULE_LICENSE("GPL");
