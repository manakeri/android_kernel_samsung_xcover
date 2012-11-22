/*
 *
 * This software program is licensed subject to the GNU General Public License
 * (GPL).Version 2,June 1991, available at http://www.fsf.org/copyleft/gpl.html

 * (C) Copyright 2007 Marvell International Ltd.
 * All Rights Reserved
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/kdev_t.h>
#include <linux/interrupt.h>
#include <linux/uio.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/fb.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/miscdevice.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/page.h>
#include <asm/mman.h>
#include <asm/pgtable.h>
#include <asm/uaccess.h>
#include <asm/cacheflush.h>

#include <mach/pxa910_ire.h>
#include <mach/cputype.h>

#define	IRE_CTRL0		0x0000
#define IRE_Y0STARTADDR		0x0010
#define IRE_Y1STARTADDR		0x0014
#define IRE_Y2STARTADDR		0x0018
#define IRE_U0STARTADDR		0x001C
#define IRE_U1STARTADDR		0x0020
#define IRE_U2STARTADDR		0x0024
#define IRE_V0STARTADDR		0x0028
#define IRE_V1STARTADDR		0x002C
#define IRE_V2STARTADDR		0x0030
#define IRE_SIZE		0x0034
#define IRE_PREROTATE_PITCH	0x0038
#define IRE_Y0P0_START_ADDR	0x003C
#define IRE_Y1P0_START_ADDR	0x0040
#define IRE_Y2P0_START_ADDR	0x0044
#define IRE_U0P0_START_ADDR	0x0048
#define IRE_U1P0_START_ADDR	0x004C
#define IRE_U2P0_START_ADDR	0x0050
#define IRE_V0P0_START_ADDR	0x0054
#define IRE_V1P0_START_ADDR	0x0058
#define IRE_V2P0_START_ADDR	0x005C
#define IRE_POSTROTATE_PITCH	0x0060
#define IRE_SRAM_CTRL		0x0070
#define IRE_IRQ_RAW		0x0080
#define IRE_IRQ_MASK		0x0084
#define IRE_IRQ_STAT		0x0088

#define IRE_IRQ_AXI_READ_ERR		(1<<5)
#define IRE_IRQ_AXI_WRITE_ERR		(1<<6)
#define ire_write(dev, value, reg) writel(value, dev->ire_regs_base + reg)
#define ire_read(dev, reg) readl(dev->ire_regs_base + reg)

#define EOF_TIMEOUT	50
static int g_dma_done_err = 0;
/* #define DEBUG */
struct ire_buffer_pair {
	struct ire_buffer_phy src;
	struct ire_buffer_phy dst;
};

struct ire_device {
	struct device		*dev;
	struct clk 		*clk;

	unsigned char __iomem	*ire_regs_base;

	struct ire_context	*last_context;
	struct mutex		s_mutex;
	struct list_head	context_list;
	unsigned long		context_count;

	unsigned long		total_gmem_size;

	struct	ire_fmt		fmt;
	unsigned int		buf_count;
	unsigned int		buf_list;
	struct ire_buffer_pair	buf_pair[MAX_IRE_BUF_COUNT];
	struct completion 	eof[MAX_IRE_BUF_COUNT];
	unsigned int		irq;
	unsigned int		users;
};

struct ire_gmem {
	struct list_head	gmem_list;
	atomic_t		gmem_count;
	struct page		*gmem_pages;
	size_t			gmem_size;
	unsigned long		gmem_virt_addr;
	unsigned long		gmem_phys_addr;
	struct ire_device	*gmem_dev;
};

struct ire_mem_map {
	struct list_head	mmap_list;
	struct ire_gmem		*mmap_gmem;
	unsigned long		mmap_type;
	unsigned long		mmap_addr;
	unsigned long		mmap_pgoff;
	unsigned long		mmap_size;
};

struct ire_context {
	struct list_head	list;
	struct ire_device	*dev;
	struct task_struct	*task;

	struct list_head	mem_map_list;
	struct ire_mem_map	*mem_map;

	unsigned long		total_gmem_size;
	unsigned long	corner_y, corner_uv;
};

static void ire_free_gmem(struct ire_context *ctx, struct ire_gmem *gmem);
static struct page *__ire_follow_page(struct mm_struct *mm,
					unsigned long address);


static struct ire_device g_ire_dev;
#ifdef CONFIG_DYNAMIC_PRINTK_DEBUG
static void dump_ire_context(struct ire_context *ctx)
{
	struct ire_device *dev = ctx->dev;
	int i;

	dev_dbg(dev,"=================ire_context==================\n");
	dev_dbg(dev,"dev:0x%08x\n", ctx);
	dev_dbg(dev,"ire_regs_base: 0x%08x\n", dev->ire_regs_base);
	dev_dbg(dev,"buf_count:%d\n", dev->buf_count);
	dev_dbg(dev,"buf_list:%d\n", dev->buf_list);
	dev_dbg(dev,"fmt.width: %d\n", dev->fmt.width);
	dev_dbg(dev,"fmt.height: %d\n", dev->fmt.height);
	dev_dbg(dev,"fmt.fmt: 0x%08x\n", dev->fmt.fmt);
	dev_dbg(dev,"fmt.angle: 0x%08x\n", dev->fmt.rot_angle);
	dev_dbg(dev,"fmt.y_pitch: %d\n", dev->fmt.y_pitch);
	dev_dbg(dev,"fmt.uv_pitch: %d\n", dev->fmt.uv_pitch);
	dev_dbg(dev,"fmt.yp0_pitch: %d\n", dev->fmt.yp0_pitch);
	dev_dbg(dev,"fmt.uvp0_pitch: %d\n", dev->fmt.uvp0_pitch);
	for (i = 0; i < dev->buf_count && dev->buf_count < MAX_IRE_BUF_COUNT;
		i++) {
		dev_dbg(dev,"buf_pair.src[%d].y_paddr:0x%08x\n",
			i, dev->buf_pair[i].src.y_paddr);
		dev_dbg(dev,"buf_pair.src[%d].u_paddr:0x%08x\n",
			i, dev->buf_pair[i].src.u_paddr);
		dev_dbg(dev,"buf_pair.src[%d].v_paddr:0x%08x\n",
			i, dev->buf_pair[i].src.v_paddr);
		dev_dbg(dev,"buf_pair.dst[%d].y_paddr:0x%08x\n",
			i, dev->buf_pair[i].dst.y_paddr);
		dev_dbg(dev,"buf_pair.dst[%d].u_paddr:0x%08x\n",
			i, dev->buf_pair[i].dst.u_paddr);
		dev_dbg(dev,"buf_pair.dst[%d].v_paddr:0x%08x\n",
			i, dev->buf_pair[i].dst.v_paddr);
	}
	dev_dbg(dev,"\n");

}

static void dump_ire_regs(struct ire_context *ctx)
{
	struct ire_device *dev = ctx->dev;

	dev_dbg(dev,"=================ire registers==================\n");
	dev_dbg(dev,"IRE_CTRL0:0x%08x\n", ire_read(dev, IRE_CTRL0));
	dev_dbg(dev,"IRE_Y0STARTADDR:0x%08x\n",
		ire_read(dev, IRE_Y0STARTADDR));
	dev_dbg(dev,"IRE_Y1STARTADDR:0x%08x\n",
		ire_read(dev, IRE_Y1STARTADDR));
	dev_dbg(dev,"IRE_Y2STARTADDR:0x%08x\n",
		ire_read(dev, IRE_Y2STARTADDR));
	dev_dbg(dev,"IRE_U0STARTADDR:0x%08x\n",
		ire_read(dev, IRE_U0STARTADDR));
	dev_dbg(dev,"IRE_U1STARTADDR:0x%08x\n",
		ire_read(dev, IRE_U1STARTADDR));
	dev_dbg(dev,"IRE_U2STARTADDR:0x%08x\n",
		ire_read(dev, IRE_U2STARTADDR));
	dev_dbg(dev,"IRE_V0STARTADDR:0x%08x\n",
		ire_read(dev, IRE_V0STARTADDR));
	dev_dbg(dev,"IRE_V1STARTADDR:0x%08x\n",
		ire_read(dev, IRE_V1STARTADDR));
	dev_dbg(dev,"IRE_V2STARTADDR:0x%08x\n",
		ire_read(dev, IRE_V2STARTADDR));
	dev_dbg(dev,"IRE_SIZE:0x%08x\n",
		ire_read(dev, IRE_SIZE));
	dev_dbg(dev,"IRE_PREROTATE_PITCH:0x%08x\n",
		ire_read(dev, IRE_PREROTATE_PITCH));
	dev_dbg(dev,"IRE_Y0P0_START_ADDR:0x%08x\n",
		ire_read(dev, IRE_Y0P0_START_ADDR));
	dev_dbg(dev,"IRE_Y1P0_START_ADDR:0x%08x\n",
		ire_read(dev, IRE_Y1P0_START_ADDR));
	dev_dbg(dev,"IRE_Y2P0_START_ADDR:0x%08x\n",
		ire_read(dev, IRE_Y2P0_START_ADDR));
	dev_dbg(dev,"IRE_U0P0_START_ADDR:0x%08x\n",
		ire_read(dev, IRE_U0P0_START_ADDR));
	dev_dbg(dev,"IRE_U1P0_START_ADDR:0x%08x\n",
		ire_read(dev, IRE_U1P0_START_ADDR));
	dev_dbg(dev,"IRE_U2P0_START_ADDR:0x%08x\n",
		ire_read(dev, IRE_U2P0_START_ADDR));
	dev_dbg(dev,"IRE_V0P0_START_ADDR:0x%08x\n",
		ire_read(dev, IRE_V0P0_START_ADDR));
	dev_dbg(dev,"IRE_V1P0_START_ADDR:0x%08x\n",
		ire_read(dev, IRE_V1P0_START_ADDR));
	dev_dbg(dev,"IRE_V2P0_START_ADDR:0x%08x\n",
		ire_read(dev, IRE_V2P0_START_ADDR));
	dev_dbg(dev,"IRE_POSTROTATE_PITCH:0x%08x\n",
		ire_read(dev, IRE_POSTROTATE_PITCH));
	dev_dbg(dev,"IRE_SRAM_CTRL:0x%08x\n",
		ire_read(dev, IRE_SRAM_CTRL));
	dev_dbg(dev,"IRE_IRQ_RAW:0x%08x\n", ire_read(dev, IRE_IRQ_RAW));
	dev_dbg(dev,"IRE_IRQ_MASK:0x%08x\n", ire_read(dev, IRE_IRQ_MASK));
	dev_dbg(dev,"IRE_IRQ_STAT:0x%08x\n", ire_read(dev, IRE_IRQ_STAT));
	dev_dbg(dev,"\n");
}
#else
#define dump_ire_context(arg...)
#define dump_ire_regs(arg...)
#endif

static struct ire_context *ire_create_context(struct ire_device *dev)
{
	struct ire_context *ctx;
	int i;

	if (dev == NULL)
		return NULL;

	ctx = kzalloc(sizeof(struct ire_context), GFP_KERNEL);
	if (ctx == NULL)
		return NULL;

	ctx->dev  = dev;
	ctx->task = current;

	INIT_LIST_HEAD(&ctx->mem_map_list);
	INIT_LIST_HEAD(&dev->context_list);
	list_add_tail(&ctx->list, &dev->context_list);
	dev->context_count++;
	dev->buf_list = 0;
	for (i = 0; i < MAX_IRE_BUF_COUNT; i++)
		init_completion(&dev->eof[i]);

	return ctx;
}

static void ire_free_context(struct ire_context *ctx)
{
	struct ire_mem_map *mmap, *m;
	struct ire_device  *dev;

	if (ctx == NULL || ctx->dev == NULL)
		return;

	dev = ctx->dev;

	list_for_each_entry_safe(mmap, m, &ctx->mem_map_list, mmap_list) {
		if (mmap->mmap_gmem)
			ire_free_gmem(ctx, mmap->mmap_gmem);

		list_del(&mmap->mmap_list);
		kfree(mmap);
	}

	list_del(&ctx->list);
	dev->context_count--;
	if (dev->last_context == ctx)
		dev->last_context = NULL;

	kfree(ctx);
}

static struct ire_gmem *ire_alloc_gmem(struct ire_context *ctx, size_t rsize)
{
	struct ire_device *dev = ctx->dev;
	unsigned long size, order, kaddr;
	struct page *page, *end_page;
	struct ire_gmem *gmem;

	if (dev == NULL)
		return NULL;

	gmem = kmalloc(sizeof(struct ire_gmem), GFP_KERNEL);
	if (gmem == NULL)
		return NULL;

	size  = PAGE_ALIGN(rsize);
	order = get_order(size);

	page  = alloc_pages(GFP_KERNEL | GFP_DMA, order);
	if (page == NULL) {
		kfree(gmem);
		return NULL;
	}

	kaddr = (unsigned long)page_address(page);
	end_page = page + (1 << order);

	memset(page_address(page), 0, size);
	dmac_flush_range((void *)kaddr, (void *)(kaddr + PAGE_ALIGN(size)));

	gmem->gmem_size      = size;
	gmem->gmem_pages     = page;
	gmem->gmem_virt_addr = kaddr;
	gmem->gmem_phys_addr = __pa(kaddr);
	ctx->total_gmem_size += size;
	dev->total_gmem_size += size;

	do {
		atomic_set(&(page->_count), 1);
		SetPageReserved(page);
		page++;
	} while (size -= PAGE_SIZE);

	/* free the otherwise unused pages. */
	while (page < end_page) {
		atomic_set(&(page->_count), 1);
		__free_page(page);
		page++;
	}

	atomic_set(&gmem->gmem_count, 1);

	return gmem;
}

static void ire_free_gmem(struct ire_context *ctx, struct ire_gmem *gmem)
{
	struct ire_device *dev = ctx->dev;
	struct page *page = gmem->gmem_pages;
	size_t size = gmem->gmem_size;

	if (!atomic_dec_and_test(&gmem->gmem_count))
		return;

	ctx->total_gmem_size -= size;
	dev->total_gmem_size -= size;

	for (; size > 0; size -= PAGE_SIZE, page++) {
		ClearPageReserved(page);
		__free_page(page);
	}

	kfree(gmem);
}

static int ire_request_mem(
		struct file *file,
		struct ire_context *ctx,
		struct ire_mem_req *req)
{
	struct mm_struct *mm  = current->mm;
	struct ire_mem_map *mmap = NULL;
	struct ire_gmem *gmem;


	int ret;

	mmap = kzalloc(sizeof(struct ire_mem_map), GFP_KERNEL);
	if (mmap == NULL)
		return -ENOMEM;

	mmap->mmap_type = req->req_type;
	if (req->req_size == 0) {
		req->phys_addr = 0l;
		req->mmap_addr = 0l;
		req->mmap_size = 0l;
		return 0;
	}

	gmem = ire_alloc_gmem(ctx, req->req_size);

	if (gmem == NULL)
		return -ENOMEM;

	mmap->mmap_gmem  = gmem;
	mmap->mmap_pgoff = gmem->gmem_phys_addr >> PAGE_SHIFT;
	mmap->mmap_size  = gmem->gmem_size;
	list_add(&mmap->mmap_list, &ctx->mem_map_list);

	down_write(&mm->mmap_sem);
	ctx->mem_map = mmap;
	ret = do_mmap_pgoff(file, 0L,
			mmap->mmap_size,
			PROT_READ | PROT_WRITE,
			MAP_SHARED,
			mmap->mmap_pgoff);
	ctx->mem_map = NULL;
	up_write(&mm->mmap_sem);

	if (ret < 0 && ret > -1024) {
		if (mmap->mmap_gmem)
			ire_free_gmem(ctx, mmap->mmap_gmem);

		req->mmap_addr = 0l;
		req->mmap_size = 0l;
		kfree(mmap);
		return ret;
	}

	mmap->mmap_addr = (unsigned long)ret;

	req->mmap_addr  = (unsigned long)ret;
	req->mmap_size  = mmap->mmap_size;
	req->phys_addr  = mmap->mmap_pgoff << PAGE_SHIFT;

	return 0;
}

static void ire_release_mem(struct ire_context *ctx, unsigned long mmap_addr)
{
	struct mm_struct *mm = current->mm;
	struct ire_mem_map *mmap, *m;

	list_for_each_entry_safe(mmap, m, &ctx->mem_map_list, mmap_list) {
		if (mmap->mmap_addr == mmap_addr) {
			down_write(&mm->mmap_sem);
			do_munmap(mm, mmap->mmap_addr, mmap->mmap_size);
			up_write(&mm->mmap_sem);

			if (mmap->mmap_gmem)
				ire_free_gmem(ctx, mmap->mmap_gmem);

			list_del(&mmap->mmap_list);
			kfree(mmap);
		}
	}
}

static int ire_submit(struct ire_context *ctx, struct ire_submit_req submit_req)
{
	struct ire_device *dev;
	unsigned int buf_id;
	struct page *page;
	struct ire_buffer *src = &submit_req.src;
	struct ire_buffer *dst = &submit_req.dst;

	/* set src and dst buffer */
	unsigned long y_paddr, u_paddr, v_paddr;

	dev = ctx->dev;

	mutex_lock(&dev->s_mutex);
	buf_id = find_first_zero_bit(&dev->buf_list, MAX_IRE_BUF_COUNT);
	if (buf_id == MAX_IRE_BUF_COUNT) {
		buf_id = -EBUSY;
		goto out_unlock;
	}

	set_bit(buf_id,(unsigned long *)(&dev->buf_list));
	/*check IRE dma src buffer address 64 bytes alignment*/
	if (src->y_vaddr & 63) {
		printk(KERN_ERR "IRE: the src memory base 0x%08lx is not 64 \
		bytes align %s %d \n", src->y_vaddr ,  __func__, __LINE__);
		BUG_ON(1);
		buf_id = -EPERM;
		goto out_unlock;
	}
	/*check IRE dma dst buffer address 64 bytes alignment*/
	if (dst->y_vaddr & 63) {
		printk(KERN_ERR "IRE: the dst memory base 0x%08lx is not 64 \
		bytes align %s %d \n", dst->y_vaddr ,  __func__, __LINE__);
		BUG_ON(1);
		buf_id = -EPERM;
		goto out_unlock;
	}

	page = __ire_follow_page(current->mm, src->y_vaddr);
	if (!page) {
		printk(KERN_ERR "IRE: fail to get src page %s %d info\n",
			  __func__, __LINE__);
		buf_id = -EFAULT;
		goto out_unlock;
	}

	if (PageHighMem(page)) {
		printk(KERN_ERR "IRE: HIGHMEM is not supported %s %d info\n",
			  __func__, __LINE__);
		buf_id = -EFAULT;
		goto out_unlock;
	}

	y_paddr = dma_map_page(dev->dev, page, src->y_vaddr % PAGE_SIZE,
				submit_req.size, DMA_TO_DEVICE);
	if (!y_paddr) {
		printk(KERN_ERR"src mem is not contiguous %s %d\n",
			__func__, __LINE__);
		buf_id = -ENOMEM;
		goto out_unlock;
	}
	u_paddr = y_paddr + (src->u_vaddr - src->y_vaddr);
	v_paddr = y_paddr + (src->v_vaddr - src->y_vaddr);

	dev->buf_pair[buf_id].src.y_paddr = y_paddr;
	dev->buf_pair[buf_id].src.u_paddr = u_paddr;
	dev->buf_pair[buf_id].src.v_paddr = v_paddr;
	dev->buf_pair[buf_id].src.size    = submit_req.size;

	y_paddr += ctx->corner_y;
	u_paddr += ctx->corner_uv;
	v_paddr += ctx->corner_uv;
	ire_write(dev, y_paddr, IRE_Y0STARTADDR + buf_id*4);
	ire_write(dev, u_paddr, IRE_U0STARTADDR + buf_id*4);
	ire_write(dev, v_paddr, IRE_V0STARTADDR + buf_id*4);

	page = __ire_follow_page(current->mm, dst->y_vaddr);
	if (!page) {
		printk(KERN_ERR "IRE: fail to get dst page %s %d info\n",
			  __func__, __LINE__);
		buf_id = -EFAULT;
		goto out_unlock;
	}

	if (PageHighMem(page)) {
		printk(KERN_ERR "IRE: HIGHMEM is not supported %s %d info\n",
			 __func__, __LINE__);
		buf_id = -EFAULT;
		goto out_unlock;
	}

	y_paddr = dma_map_page(dev->dev, page, dst->y_vaddr % PAGE_SIZE,
				submit_req.size, DMA_FROM_DEVICE);
	if (!y_paddr) {
		printk(KERN_ERR"dst mem is not contiguous %s %d\n",
			 __func__, __LINE__);
		buf_id = -ENOMEM;
		goto out_unlock;
	}
	u_paddr = y_paddr + (dst->u_vaddr - dst->y_vaddr);
	v_paddr = y_paddr + (dst->v_vaddr - dst->y_vaddr);

	dev->buf_pair[buf_id].dst.y_paddr = y_paddr;
	dev->buf_pair[buf_id].dst.u_paddr = u_paddr;
	dev->buf_pair[buf_id].dst.v_paddr = v_paddr;
	dev->buf_pair[buf_id].dst.size    = submit_req.size;

	ire_write(dev, y_paddr, IRE_Y0P0_START_ADDR + buf_id*4);
	ire_write(dev, u_paddr, IRE_U0P0_START_ADDR + buf_id*4);
	ire_write(dev, v_paddr, IRE_V0P0_START_ADDR + buf_id*4);

out_unlock:
	mutex_unlock(&dev->s_mutex);
	return buf_id;
}

static int ire_desubmit(struct ire_context *ctx, unsigned int buf_id)
{
	struct ire_device *dev;

	dev = ctx->dev;

	mutex_lock(&dev->s_mutex);

	dma_unmap_page(dev->dev, dev->buf_pair[buf_id].src.y_paddr,
			 dev->buf_pair[buf_id].src.size, DMA_TO_DEVICE);
	dma_unmap_page(dev->dev, dev->buf_pair[buf_id].dst.y_paddr,
			dev->buf_pair[buf_id].src.size, DMA_FROM_DEVICE);

	clear_bit(buf_id, (unsigned long *)(&dev->buf_list));
	mutex_unlock(&dev->s_mutex);

	return buf_id;
}

static int ire_set_format(struct ire_context *ctx, struct ire_fmt *fmt)
{
	struct ire_device *dev = ctx->dev;
	unsigned long y_pitch, uv_pitch;
	unsigned long yp0_pitch, uvp0_pitch;
	int ret = 0;
	y_pitch = 0;
	uv_pitch = 0;
	yp0_pitch = 0;
	uv_pitch = 0;

	mutex_lock(&dev->s_mutex);

	switch(fmt->fmt){
		case IRE_PIXFMT_RGB565:
			y_pitch = fmt->width * 2;
			yp0_pitch = fmt->height* 2;
			if(fmt->rot_angle == IRE_ROT_180)
				yp0_pitch = y_pitch;
			break;
		case IRE_PIXFMT_RGB888:
			y_pitch = fmt->width * 4;
			yp0_pitch = fmt->height* 4;
			if(fmt->rot_angle == IRE_ROT_180)
				yp0_pitch = y_pitch;
			break;
		case IRE_PIXFMT_YUV422_PACKED_UYVY:/*YUV422 packed mode => YUV 444 packed*/
		case IRE_PIXFMT_YUV422_PACKED_VYUV:/*YUV422 packed mode => YUV 444 packed*/
		case IRE_PIXFMT_YUV422_PACKED_YUYV:/*YUV422 packed mode => YUV 444 packed*/
		case IRE_PIXFMT_YUV422_PACKED_YVYU:/*YUV422 packed mode => YUV 444 packed*/
			y_pitch = fmt->width * 2;
			if(fmt->rot_angle == IRE_ROT_180)
				yp0_pitch = y_pitch;
			else
				yp0_pitch = fmt->height * 4;
			break;
		case IRE_PIXFMT_YUV420_PLANAR:
			y_pitch = fmt->width;
			uv_pitch = fmt->width/2;
			yp0_pitch = fmt->height;
			uvp0_pitch = fmt->height/2;
			if(fmt->rot_angle == IRE_ROT_180){
				printk(KERN_ERR "IRE dosen't support YUV420 180 degree rotation!\n");
				ret = -EINVAL;
				goto out_unlock;
			}
			break;

		default:
			printk(KERN_ERR "IRE dosen't support this type of rotation!\n");
			ret = -EINVAL;
			goto out_unlock;
	}

	switch(fmt->rot_angle){
		case IRE_ROT_90:
			ctx->corner_y = (fmt->height-1)*y_pitch; /*bottom/left corner*/
		break;
		case IRE_ROT_270:
			ctx->corner_y = 0; /*top/left corner*/
		break;
		case IRE_ROT_180:
			ctx->corner_y = (fmt->height)*y_pitch-1; /*bottom/right corner*/
		break;
	}

	if((fmt->rot_angle == IRE_ROT_90) && (fmt->fmt == IRE_PIXFMT_YUV420_PLANAR)){
		ctx->corner_uv = uv_pitch*(fmt->height/2-1);
	}else
		ctx->corner_uv = 0;

	/* set format */
	ire_write(dev, fmt->height << 16 | fmt->width, IRE_SIZE);/*in pixels*/
	ire_write(dev, uv_pitch << 16 | y_pitch, IRE_PREROTATE_PITCH);/*in bytes*/
	ire_write(dev, uvp0_pitch << 16 | yp0_pitch, IRE_POSTROTATE_PITCH);/*in bytes*/

	if ((fmt->fmt == IRE_PIXFMT_YUV422_PACKED_UYVY)
		|| (fmt->fmt == IRE_PIXFMT_YUV422_PACKED_VYUV)
		|| (fmt->fmt == IRE_PIXFMT_YUV422_PACKED_YUYV)
		|| (fmt->fmt == IRE_PIXFMT_YUV422_PACKED_YVYU)){
		if(fmt->rot_angle == IRE_ROT_180)
			ire_write(dev, fmt->fmt | 2 << 21 | fmt->rot_angle << 11 | 0 << 30,
		IRE_CTRL0);
		else
			ire_write(dev, fmt->fmt | 1 << 21 | fmt->rot_angle << 11 | 0 << 30,
		IRE_CTRL0);
	}else{
		ire_write(dev, fmt->fmt | fmt->rot_angle << 11 | 0 << 30,
			IRE_CTRL0);
	}

out_unlock:
	mutex_unlock(&dev->s_mutex);

	return ret;
}

static int ire_dequeue(struct ire_context *ctx, unsigned int buf_id)
{
	struct ire_device *dev = ctx->dev;
	unsigned int value;
	int ret;

	mutex_lock(&dev->s_mutex);
	if (buf_id >= dev->buf_count) {
		ret = 0;
		goto out_unlock;
	}
	mutex_unlock(&dev->s_mutex);
	ret = wait_for_completion_timeout(&dev->eof[buf_id], HZ);
	if (!ret) {
		INIT_COMPLETION(dev->eof[buf_id]);
		printk(KERN_ERR "%s: wait for EOF timeout!\n", __func__);
		ret = -EBUSY;
	}else{
		ret = 0;
	}
	mutex_lock(&dev->s_mutex);
	/* disable eof interrupt*/
	value = ire_read(dev, IRE_IRQ_MASK);
	value &= ~(1 << buf_id);
	ire_write(dev, value, IRE_IRQ_MASK);

	dev->buf_count--;
	if(g_dma_done_err < 0){
		printk(KERN_ERR "%s: DAM transfer errror!\n", __func__);
		ret = -EBUSY;
	}

out_unlock:
	mutex_unlock(&dev->s_mutex);
	return ret;
}

static int
ire_enqueue(struct ire_context *ctx, unsigned int buf_id)
{
	struct ire_device *dev = ctx->dev;
	unsigned int value;
	int ret = 0;
	mutex_lock(&dev->s_mutex);
	if (buf_id >= MAX_IRE_BUF_COUNT) {
		ret = -EIO;
		goto out_unlock;
	}
	g_dma_done_err = 0;

	/* enable interrupt*/
	value = ire_read(dev, IRE_IRQ_MASK);
	value |= 1 << buf_id;
	ire_write(dev, value |IRE_IRQ_AXI_READ_ERR |IRE_IRQ_AXI_WRITE_ERR, IRE_IRQ_MASK);

	dev->buf_count++;
out_unlock:
	mutex_unlock(&dev->s_mutex);
	return ret;
}

static int
ire_enable(struct ire_context *ctx)
{
	struct ire_device *dev = ctx->dev;
	unsigned int value;
	int ret = 0;
	mutex_lock(&dev->s_mutex);

	if (dev->buf_count == 0) {
		ret = -EIO;
		goto out_unlock;
	}
	dump_ire_context(ctx);
	dump_ire_regs(ctx);
	/* enable IRE */
	value = ire_read(dev, IRE_CTRL0);
	value |= 0x1;
	ire_write(dev, value, IRE_CTRL0);

out_unlock:
	mutex_unlock(&dev->s_mutex);

	return 0;
}

static int
ire_disable(struct ire_context *ctx)
{
	struct ire_device *dev = ctx->dev;
	unsigned int value;
	mutex_lock(&dev->s_mutex);

	/* disable IRE */
	value = ire_read(dev, IRE_CTRL0);
	value &= ~0x1;
	ire_write(dev, value, IRE_CTRL0);

	mutex_unlock(&dev->s_mutex);

	return 0;
}

static irqreturn_t
ire_irq_handler(int irq, void *dev_id)
{
	struct ire_device *dev = dev_id;
	int status;

	status = ire_read(dev, IRE_IRQ_STAT);
	if((status & IRE_IRQ_AXI_WRITE_ERR) || (status & IRE_IRQ_AXI_READ_ERR))
		g_dma_done_err = -1;

	if (status & 0x1)
		complete(&dev->eof[0]);
	if (status & 0x2)
		complete(&dev->eof[1]);
	if (status & 0x4)
		complete(&dev->eof[2]);

	ire_write(dev, status, IRE_IRQ_STAT);

	return IRQ_HANDLED;
}

static int
ire_open(struct inode *inode, struct file *file)
{
	struct ire_context *ctx = file->private_data;
	struct ire_device *dev;
	u32 ret = 0;

	dev = &g_ire_dev;
	mutex_lock(&dev->s_mutex);
	clk_enable(dev->clk);
	if(! dev->users){
		dev->users++;
	}else{
		ret = -EBUSY;
		goto open_out;
	}

	ctx = ire_create_context(dev);
	if (!ctx){
		ret = -ENOMEM;
		goto open_out;
	}
	file->private_data = ctx;

open_out:
	mutex_unlock(&dev->s_mutex);
	return ret;
}

static int
ire_release(struct inode *inode, struct file *file)
{
	struct ire_context *ctx = file->private_data;
	struct ire_device *dev = &g_ire_dev;
	mutex_lock(&dev->s_mutex);
	ire_free_context(ctx);
	dev->users = 0;
	clk_disable(dev->clk);
	mutex_unlock(&dev->s_mutex);
	return 0;
}

static struct page *__ire_follow_page(struct mm_struct *mm,
				      unsigned long address)
{
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;
	pte_t *ptep, pte;
	unsigned long pfn;
	struct page *page;

	pgd = pgd_offset(mm, address);
	if (pgd_none(*pgd) || unlikely(pgd_bad(*pgd)))
		goto out;

	pud = pud_offset(pgd, address);
	if (pud_none(*pud) || unlikely(pud_bad(*pud)))
		goto out;

	pmd = pmd_offset(pud, address);
	if (pmd_none(*pmd) || unlikely(pmd_bad(*pmd)))
		goto out;

	ptep = pte_offset_map(pmd, address);
	if (!ptep)
		goto out;

	pte = *ptep;
	pte_unmap(ptep);
	if (pte_present(pte)) {
		pfn = pte_pfn(pte);
		if (pfn_valid(pfn)) {
			page = pfn_to_page(pfn);
			return page;
		}
	}

out:
	return NULL;
}

static int ire_fsync(struct file *file, struct dentry *dentry, int datasync)
{
	struct ire_context *ctx;
	struct ire_device *dev;
	int error = 0;
	int i;

	ctx = (struct ire_context *)(file->private_data);
	dev = ctx->dev;
	for (i = 0; i < dev->buf_count; i++)
		error = ire_dequeue(ctx, i);

	return error;
}
static int ire_ioctl(struct inode *inode, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct ire_context *ctx = file->private_data;
	struct ire_device  *dev = (struct ire_device *)ctx->dev;

	if (dev == NULL)
		return -ENODEV;

	switch (cmd) {
	/* request buffer */
	case IREIO_REQUEST_MEM:
	{
		struct ire_mem_req mem_req;
		void __user *argp = (void *)arg;
		int ret;

		if (copy_from_user(&mem_req, argp, sizeof(mem_req)))
			return -EFAULT;

		ret = ire_request_mem(file, ctx, &mem_req);
		if (ret < 0)
			return ret;

		if (copy_to_user(argp, &mem_req, sizeof(mem_req))) {
			ire_release_mem(ctx, mem_req.mmap_addr);
			return -EFAULT;
		}

		return 0;
	}

	/* release memory */
	case IREIO_RELEASE_MEM:
	{
		unsigned long mmap_addr = arg;
		ire_release_mem(ctx, mmap_addr);
		return 0;
	}

	case IREIO_FLUSH_MEM:
	{
		struct ire_mem_map *mmap;
		list_for_each_entry(mmap, &ctx->mem_map_list, mmap_list) {
			if (mmap->mmap_addr == arg)
				__cpuc_flush_user_range(
						mmap->mmap_addr,
						mmap->mmap_size,
						0);
		}
		return 0;
	}

	case IREIO_S_FMT:
	{
		void __user *argp = (void *)arg;
		if (copy_from_user(&dev->fmt, argp, sizeof(struct ire_fmt)))
			return -EFAULT;
		return ire_set_format(ctx, &dev->fmt);
	}

	case IREIO_G_FMT:
	{
		void __user *argp = (void *)arg;
		if (copy_to_user((void *)argp, &dev->fmt,
			sizeof(struct ire_fmt)))
			return -EFAULT;
		return 0;
	}

	case IREIO_SUBMIT:
	{
		struct ire_submit_req submit_req;
		void __user *argp = (void *)arg;
		int ret;

		if (copy_from_user(&submit_req, argp, sizeof(submit_req)))
			return -EFAULT;

		ret = ire_submit(ctx, submit_req);
		submit_req.buf_id = ret;
		if (copy_to_user(argp, &submit_req, sizeof(submit_req)))
			return -EFAULT;
		if (ret < 0)
			return -EBUSY;
		return 0;
	}

	case IREIO_DESUBMIT:
	{
		unsigned int buf_id = arg;

		return ire_desubmit(ctx, buf_id);
	}

	case IREIO_ENQUEUE:
	{
		unsigned int buf_id = arg;

		return ire_enqueue(ctx, buf_id);
	}


	case IREIO_DEQUEUE:
	{
		unsigned int buf_id = arg;

		return ire_dequeue(ctx, buf_id);
	}

	case IREIO_ENABLE:
		return ire_enable(ctx);

	case IREIO_DISABLE:
		return ire_disable(ctx);

	default:
		return -EINVAL;
		break;
	}

	return 0;
}

static int ire_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct ire_context *ctx = file->private_data;
	struct ire_mem_map *mmap = ctx->mem_map;
	pgprot_t pgprot;

	if (NULL == ctx)
		return -ENODEV;

	/* Within 4GB address space? */
	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;

	/* DO NOT allow arbitrary memory map request */
	if (NULL == mmap)
		return -EPERM;

	if (mmap->mmap_size != (vma->vm_end - vma->vm_start))
		return -EINVAL;

	switch (IRE_MEM_REQ_ATTR(mmap->mmap_type)) {
	case IRE_ATTR_COHERENT:
		pgprot = pgprot_noncached(vma->vm_page_prot);
		break;
	case IRE_ATTR_WRITECOMBINE:
		pgprot = pgprot_writecombine(vma->vm_page_prot);
		break;
	case IRE_ATTR_CACHEABLE:
	default:
		pgprot = vma->vm_page_prot;
		break;
	}

	vma->vm_page_prot = pgprot;
	vma->vm_flags |= (VM_IO | VM_RESERVED);

	if (remap_pfn_range(vma, vma->vm_start,
				mmap->mmap_pgoff,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot))
		return -EAGAIN;

	/* ire_flush_gmem(NULL); FIXME later*/

#ifdef DEBUG_CACHE_COHERENCY
	dump_page_table_entries(vma->vm_start, vma->vm_end, mmap->mmap_gmem);
#endif
	return 0;
}


static struct file_operations pxa910_ire_fops = {
	.owner		= THIS_MODULE,
	.open 		= ire_open,
	.release 	= ire_release,
	.ioctl		= ire_ioctl,
	.mmap		= ire_mmap,
	.fsync		= ire_fsync,
};

static struct miscdevice pxa910_ire_miscdev = {
	.minor		= MISC_DYNAMIC_MINOR,
	.name		= "ire",
	.fops		= &pxa910_ire_fops,
};

static int __devinit pxa910_ire_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct ire_device *dev = &g_ire_dev;
	int ret = 0, size, irq;
	mutex_init(&dev->s_mutex);
	dev->clk = clk_get(&pdev->dev, NULL);
	if(! dev->clk){
		printk(KERN_WARNING "%s: failed to get clk\n",
			__func__);
		goto exit;
	}
	ret = misc_register(&pxa910_ire_miscdev);
	if (ret) {
		printk(KERN_WARNING "%s: failed to register misc device\n",
			__func__);
		goto exit;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	size = res->end - res->start + 1;
	dev->ire_regs_base = ioremap_nocache(res->start, size);
	if (dev->ire_regs_base == NULL) {
		printk(KERN_WARNING "failed to request register memory\n");
		ret = -EBUSY;
		goto failed_deregmisc;
	}
	irq = platform_get_irq(pdev, 0);
	ret = request_irq(irq, ire_irq_handler,
					IRQF_DISABLED, "ire", dev);
	dev->irq = irq;

	if (ret)  {
		printk(KERN_WARNING "failed to request IRQ\n");
		goto failed_freemem;
	}

	platform_set_drvdata(pdev, dev);
	/* for power saving */
	clk_enable(dev->clk);
	clk_disable(dev->clk);

	printk(KERN_NOTICE "pxa910_ire: detected\n");
	return 0;

failed_freemem:
	iounmap(dev->ire_regs_base);
failed_deregmisc:
	misc_deregister(&pxa910_ire_miscdev);
exit:
	printk(KERN_WARNING "pxa910_ire: failed to initilize ire\n");
	return ret;
}

static int __devexit pxa910_ire_remove(struct platform_device *pdev)
{
	struct ire_device *dev = platform_get_drvdata(pdev);
	int irq;

	irq = platform_get_irq(pdev, 0);
	free_irq(irq, dev);
	iounmap(dev->ire_regs_base);
	misc_deregister(&pxa910_ire_miscdev);

	return 0;
}

static struct platform_driver pxa910_ire_driver = {
	.driver		= {
	.name		= "pxa910-ire",
	},
	.probe		= pxa910_ire_probe,
	.remove		= __devexit_p(pxa910_ire_remove)
};

static int __init pxa910_ire_init(void)
{
	return platform_driver_register(&pxa910_ire_driver);
}

static void __exit pxa910_ire_exit(void)
{
	platform_driver_unregister(&pxa910_ire_driver);
}

module_init(pxa910_ire_init);
module_exit(pxa910_ire_exit);

MODULE_DESCRIPTION("PXA910 Image Rotation");
MODULE_LICENSE("GPL");


