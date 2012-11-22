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

#include "css_dsa.h"

struct px_css_dsa *g_dsa;

#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28))

int dsa_vm_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	unsigned long offset;

	offset = vmf->pgoff << PAGE_SHIFT;
	
	if (offset > sizeof(struct px_css_dsa))
	{
		return VM_FAULT_OOM;
	}

	vmf->page =vmalloc_to_page((void *)g_dsa + offset);

	get_page(vmf->page);
	
	return 0;
}

static struct vm_operations_struct pxcss_dsa_vm_ops = {
	.fault = dsa_vm_fault
};

#else
struct page * pxcss_dsa_vm_nopage(struct vm_area_struct * vma,
                                 unsigned long address,
                                 int * type)
{
	unsigned int offset;
	struct page *p = 0;

	offset = (address - vma->vm_start ) + (vma->vm_pgoff << PAGE_SHIFT);
	
	if (offset > sizeof(struct px_css_dsa))
	{
		return NULL;
	}

	p = vmalloc_to_page((void *)g_dsa + offset);
	*type = VM_FAULT_MINOR;

	get_page(p);
	
	return p;
}

static struct vm_operations_struct pxcss_dsa_vm_ops = {
	nopage: pxcss_dsa_vm_nopage,
};
#endif

static int pxcss_dsa_mmap(struct file *fp, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_ops	      = &pxcss_dsa_vm_ops;

	return 0;
}

static int pxcss_dsa_open(struct inode *inode, struct file *fp)
{
	return 0;
}


static int pxcss_dsa_release(struct inode *inode, struct file *fp)
{
	return 0;
}

static struct file_operations pxcss_dsa_d_fops = {
	.owner   = THIS_MODULE,
	.open    = pxcss_dsa_open,
	.release = pxcss_dsa_release,
	.mmap    = pxcss_dsa_mmap, 
};

struct miscdevice pxcss_dsa_d = {
	MISC_DYNAMIC_MINOR,
	"pxcss_dsa_d",
	&pxcss_dsa_d_fops,
};

