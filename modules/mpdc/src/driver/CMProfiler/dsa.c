/*
** (C) Copyright 2009 Marvell International Ltd.
**  		All Rights Reserved

** This software file (the "File") is distributed by Marvell International Ltd. 
** under the terms of the GNU General Public License Version 2, June 1991 (the "License"). 
** You may use, redistribute and/or modify this File in accordance with the terms and 
** conditions of the License, a copy of which is available along with the File in the 
** license.txt file or by writing to the Free Software Foundation, Inc., 59 Temple Place, 
** Suite 330, Boston, MA 02111-1307 or on the worldwide web at http://www.gnu.org/licenses/gpl.txt.
** THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED WARRANTIES 
** OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY DISCLAIMED.  
** The License provides additional details about this warranty disclaimer.
*/

#include <linux/pagemap.h>
#include <linux/miscdevice.h>
#include <linux/version.h>

#include "cm_dsa.h"

extern struct px_cm_dsa *g_dsa;

#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28))

int dsa_vm_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	unsigned long offset;

	offset = vmf->pgoff << PAGE_SHIFT;
	
	if (offset > sizeof(struct px_cm_dsa))
	{
		return VM_FAULT_OOM;
	}

	vmf->page =vmalloc_to_page((void *)g_dsa + offset);

	get_page(vmf->page);
	
	return 0;
}

static struct vm_operations_struct pxcm_dsa_vm_ops = {
	.fault = dsa_vm_fault
};

#else
static struct page* pxcm_dsa_vm_nopage(struct vm_area_struct *vma, unsigned long address, int *type)
{
	unsigned long offset;
	struct page *p = NULL;

	offset = (address - vma->vm_start) + (vma->vm_pgoff << PAGE_SHIFT);

	if (offset > sizeof(struct px_cm_dsa))
	{
		return NULL;
	}

	p = vmalloc_to_page((void *)g_dsa + offset);
	*type = VM_FAULT_MINOR;

	get_page(p);

	return p;
}

static struct vm_operations_struct pxcm_dsa_vm_ops = {
	nopage: pxcm_dsa_vm_nopage,
};
#endif

static int pxcm_dsa_d_mmap(struct file *fp, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_ops       = &pxcm_dsa_vm_ops;
	
	return 0;
}

static int pxcm_dsa_d_open(struct inode *inode, struct file *fp)
{
	return 0;
}

static int pxcm_dsa_d_release(struct inode *inode, struct file *fp)
{
	return 0;
}

static struct file_operations pxcm_dsa_d_fops = {
	.owner   = THIS_MODULE,
	.open    = pxcm_dsa_d_open,
	.release = pxcm_dsa_d_release,
	.mmap    = pxcm_dsa_d_mmap,
};

struct miscdevice pxcm_dsa_d = {
	MISC_DYNAMIC_MINOR,
	"pxcm_dsa_d",
	&pxcm_dsa_d_fops,
};

