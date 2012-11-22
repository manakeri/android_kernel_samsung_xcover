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

#include "ring_buffer.h"

extern struct RingBufferInfo g_module_buffer;

static unsigned int g_client_count = 0;

#if defined(LINUX_VERSION_CODE) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28))

int module_buf_vm_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	unsigned long offset;

	offset = vmf->pgoff << PAGE_SHIFT;
	
	if (offset > g_module_buffer.buffer.size)
	{
		return VM_FAULT_OOM;
	}

	vmf->page = vmalloc_to_page((void *)g_module_buffer.buffer.address + offset);

	get_page(vmf->page);
	
	return 0;
}

static struct vm_operations_struct pxhs_module_buf_vm_ops = {
	.fault = module_buf_vm_fault
};

#else

struct page * pxhs_module_buf_vm_nopage(struct vm_area_struct * vma,
									   unsigned long address,
					   int * type)
{
	unsigned int offset;
	struct page *p = 0;

	offset = (address - vma->vm_start ) + (vma->vm_pgoff << PAGE_SHIFT);

	p = vmalloc_to_page(g_module_buffer.buffer.address + offset);
	*type = VM_FAULT_MINOR;

	get_page(p);
	
	return p;
}

static struct vm_operations_struct pxhs_module_buf_vm_ops = {
	nopage: pxhs_module_buf_vm_nopage,
};
#endif

static int pxhs_module_buf_mmap(struct file *fp, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_ops       = &pxhs_module_buf_vm_ops;

	return 0;
};

static int pxhs_module_buf_open(struct inode *inode, struct file *fp)
{
	if (g_client_count > 0)
		return -EBUSY;
	
	g_client_count++;

	return 0;
}


static int pxhs_module_buf_release(struct inode *inode, struct file *fp)
{
	g_client_count--;

	return 0;
}

static int pxhs_module_buf_read(struct file *filp, char *buf, size_t count, loff_t *ppos)
{
	unsigned int count_read;

	count_read = read_ring_buffer(&g_module_buffer.buffer, buf);

	return count_read;
}


static struct file_operations pxhs_module_d_fops = {
	.owner   = THIS_MODULE,
	.open    = pxhs_module_buf_open,
	.release = pxhs_module_buf_release,
	.mmap    = pxhs_module_buf_mmap,
	.read    = pxhs_module_buf_read,
};

struct miscdevice pxhs_module_d = {
	MISC_DYNAMIC_MINOR,
	"pxhs_module_d",
	&pxhs_module_d_fops,
};

