/*
 * drivers/uio/uio_vmeta.c
 *
 * Marvell multi-format video decoder engine UIO driver.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 * Based on an earlier version by Peter Liao.
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/slab.h>

#include <mach/hardware.h>
#include <mach/soc_vmeta.h>
#include <mach/dvfm.h>
#include <mach/debug_pm.h>

#define CONFIG_MEM_FOR_MULTIPROCESS
#define VDEC_HW_CONTEXT_SIZE	SZ_512K
#define VDEC_OBJ_SIZE			SZ_64K
#define KERNEL_SHARE_SIZE		SZ_4K

/* public */
#define UIO_VMETA_VERSION	"build-006"

#define VMETA_DEBUG 0

#if VMETA_DEBUG
#define vmeta_print printk
#else
#define vmeta_print(x,...)
#endif

/*In case we need to export some infomation for other components like GC*/
static struct vmeta_instance *vmeta_inst;

void vmeta_lock_init(struct vmeta_instance *vi)
{
	sema_init(vi->sema, 1);
	sema_init(vi->priv_sema, 1);
}

int vmeta_lock(unsigned long ms, struct vmeta_instance *vi)
{
	int ret;

	ret = down_timeout(vi->sema,msecs_to_jiffies(ms));

	return ret;
}

int vmeta_unlock(struct vmeta_instance *vi)
{
	if(vi->sema->count == 0) {
		up(vi->sema);
		return 0;
	}
	else if(vi->sema->count == 1) {
		return 0;
	}
	else {
		return -1;
	}
}

int vmeta_priv_lock(unsigned long ms, struct vmeta_instance *vi)
{
	int ret;

	ret = down_timeout(vi->priv_sema,msecs_to_jiffies(ms));

	return ret;
}

int vmeta_priv_unlock(struct vmeta_instance *vi)
{
	if(vi->priv_sema->count == 0) {
		up(vi->priv_sema);
		return 0;
	}
	else if(vi->priv_sema->count == 1) {
		return 0;
	}
	else {
		return -1;
	}
}

static struct dvfm_lock dvfm_lock = {
	.lock		= SPIN_LOCK_UNLOCKED,
	.dev_idx	= -1,
	.count		= 0,
};

static void set_dvfm_constraint(struct vmeta_instance* vi)
{
	int ret = 0;
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count++ == 0) {

		/* Disable dvfm for now for MG1,
		 * todo: later should try to restore to optimize for power */
		if(vi->plat_data->set_dvfm_constraint) {
			ret = vi->plat_data->set_dvfm_constraint(vi, dvfm_lock.dev_idx);
			if(ret) {
				printk(KERN_ERR "vmeta dvfm disable error with %d\n", ret);
			}
			vi->power_constraint = 1;
			if(timer_pending(&vi->power_timer)) {
				del_timer(&vi->power_timer);
			}
		}
	} else
		dvfm_lock.count--;
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

static void __unset_dvfm_constraint(struct vmeta_instance* vi)
{
	int ret;
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count == 0) {
		if(vi->plat_data->unset_dvfm_constraint) {
			ret = vi->plat_data->unset_dvfm_constraint(vi,dvfm_lock.dev_idx);
			if(ret) {
				printk(KERN_ERR "vmeta dvfm enable error with %d\n", ret);
			}

			vi->power_constraint = 0;
		}
	}
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

static void unset_dvfm_constraint(struct vmeta_instance* vi)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count == 0) {
		spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
		return;
	}
	if (--dvfm_lock.count == 0) {
		vi->power_timer.expires = jiffies + msecs_to_jiffies(vi->power_down_ms);
		if(!timer_pending(&vi->power_timer)) {
			add_timer(&vi->power_timer);
		}
	} else
		dvfm_lock.count++;
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

static void vmeta_power_timer_handler(unsigned long data)
{
	struct vmeta_instance* vi = (struct vmeta_instance*) data;

	__unset_dvfm_constraint(vi);
}

int vmeta_power_status(void)
{
	if (vmeta_inst)
		return vmeta_inst->power_status;

	return -1;
}
EXPORT_SYMBOL(vmeta_power_status);

int vmeta_power_on(struct vmeta_instance* vi)
{
	mutex_lock(&vi->mutex);
	if(vi->power_status ==1) {
		mutex_unlock(&vi->mutex);
		return 0;
	}

	dvfm_disable_lowpower(dvfm_lock.dev_idx);

	vmeta_pwr(VMETA_PWR_ENABLE);
	vi->power_status =1;

	mutex_unlock(&vi->mutex);
	return 0;
}

int vmeta_clk_on(struct vmeta_instance *vi)
{
	mutex_lock(&vi->mutex);
	if (vi->clk_status != 0) {
		mutex_unlock(&vi->mutex);
		return 0;
	}

	set_dvfm_constraint(vi);

	if (NULL != vi->axi_clk)
		clk_enable(vi->axi_clk);

	if (NULL != vi->clk)
		clk_enable(vi->clk);

	vi->clk_status = 2;
	mutex_unlock(&vi->mutex);

	gc_vmeta_stats_clk_event(VMETA_CLK_ON);
	return 0;
}

/*
vco bitmap
31~24           23~16     15~8        0~7
reserved        vop       step        flags
vop: vmeta op table defined in kernel space
step:  delta vop level;
flags:  0-set vop for vmeta; 1-increase/decrease vop by the value in step; 2-max vop; 3-min vop
e.g.
a) set vmeta as OP3 vco = 0x00030000
b) set vmeta higher vop by 1 step vco = 0x00000101

VOP Definition, see detail uio_vmeta.h
resolution <= VGA          -- VOP=[1~7]
VGA < resolution <=720p    -- VOP=[8~13]
resolution > 720p          -- VOP=[14]
VOP 0/15 are reserved to notify upper layer.
*/
int vmeta_clk_switch(struct vmeta_instance *vi, unsigned long clk_flag)
{
	int ret = -1;
	int flags;
	int vop = -1;

	mutex_lock(&vi->mutex);
	vmeta_print(KERN_INFO"[vmeta] clk switch 0x%lx \n",clk_flag);
	flags = clk_flag & 0xff;
	if(flags == 0) {
		vop = (clk_flag & (0xff<<16)) >> 16;
		vmeta_print(KERN_INFO "[vmeta] set vop as %d \n",vop);
	}
	else if(flags == 1) {
		signed char step;
		step = (signed char)((clk_flag & (0xff<<8)) >> 8 );
		vmeta_print(KERN_INFO "[vmeta] +/- step= %d \n",step);
		if(step > 0 && vi->plat_data->increase_core_freq)
			ret = vi->plat_data->increase_core_freq(vi, step);
		else if (step < 0 && vi->plat_data->decrease_core_freq)
			ret = vi->plat_data->decrease_core_freq(vi, 0-step);

		if(ret < 0) {
			mutex_unlock(&vi->mutex);
			return ret;
		}

		vop = ret;
		vmeta_print(KERN_INFO "[vmeta] set vop as %d \n",vop);
	}else if(flags == 2) {
		vop = VMETA_OP_VGA;
		vmeta_print(KERN_INFO "[vmeta] set vop as %d \n",vop);
	}else if(flags == 3) {
		vop = VMETA_OP_720P_MAX;
		vmeta_print(KERN_INFO "[vmeta] set vop as %d \n",vop);
	}

	printk("original vop =%d\n",vi->vop);
	if(vi->vop != vop && vop >= VMETA_OP_MIN && vop <= VMETA_OP_MAX) {
		vi->vop = vop;
		printk(KERN_INFO "set vop to %d \n",vi->vop);
	}

	ret = vi->vop;
	mutex_unlock(&vi->mutex);
	return ret;
}

int vmeta_turn_on(struct vmeta_instance *vi)
{
	int ret;

	ret = vmeta_power_on(vi);
	if (ret)
		return -1;

	ret = vmeta_clk_on(vi);
	if (ret)
		return -1;

	return 0;
}



int vmeta_power_off(struct vmeta_instance *vi)
{
	mutex_lock(&vi->mutex);
	if (vi->power_status == 0) {
		mutex_unlock(&vi->mutex);
		return 0;
	}

	vmeta_pwr(VMETA_PWR_DISABLE);
	vi->power_status = 0;
	
	dvfm_enable_lowpower(dvfm_lock.dev_idx);

	if (vi->plat_data->clean_dvfm_constraint) {
		vi->plat_data->clean_dvfm_constraint(vi, dvfm_lock.dev_idx);
		printk(KERN_INFO "vmeta op clean up\n");
	}

	mutex_unlock(&vi->mutex);
	return 0;
}

int vmeta_clk_off(struct vmeta_instance *vi)
{
	mutex_lock(&vi->mutex);
	if (vi->clk_status == 0) {
		mutex_unlock(&vi->mutex);
		return 0;
	}

	if (NULL != vi->clk)
		clk_disable(vi->clk);

	if (NULL != vi->axi_clk)
		clk_disable(vi->axi_clk);

	unset_dvfm_constraint(vi);
	vi->clk_status = 0;
	mutex_unlock(&vi->mutex);

	gc_vmeta_stats_clk_event(VMETA_CLK_OFF);
	return 0;
}

int vmeta_turn_off(struct vmeta_instance *vi)
{
	int ret;

	ret = vmeta_clk_off(vi);
	if (ret)
		return -1;

	ret = vmeta_power_off(vi);
	if (ret)
		return -1;

	return 0;
}

int vmeta_open(struct uio_info *info, struct inode *inode, void *file_priv)
{
	struct vmeta_instance *vi;
	struct uio_listener *priv = file_priv;
	int *p,i;

	priv->extend = kmalloc(sizeof(int)*MAX_VMETA_INSTANCE, GFP_KERNEL);
	if(!priv->extend) {
		printk(KERN_ERR "vmeta open error\n");
		return -1;
	}

	p = (int*) priv->extend;

	for(i=0;i<MAX_VMETA_INSTANCE;i++)
		p[i] = MAX_VMETA_INSTANCE;

	vi = (struct vmeta_instance *)info->priv;
	vmeta_turn_on(vi);

	return 0;
}

int vmeta_release(struct uio_info *info, struct inode *inode, void *file_priv)
{
	struct vmeta_instance *vi;
	kernel_share * ks;
	int current_id = 0;
	struct uio_listener *priv = file_priv;
	int *p,i;

	vi = (struct vmeta_instance *) info->priv;
	ks = (kernel_share*)vi->uio_info.mem[3].internal_addr;

	if(!priv)
		return -1;
	p = (int*) priv->extend;

	for(i=0;i<MAX_VMETA_INSTANCE;i++) {
		if(p[i]>=0 && p[i]<MAX_VMETA_INSTANCE) {
			/*if we go here, there is something abnormal*/
			current_id = p[i];
			printk(KERN_INFO "vmeta release current tgid(%d) pid(%d), id to be closed=%d, active id=%d\n",\
				   current->tgid, current->pid,current_id,ks->active_user_id);

			mutex_lock(&vi->mutex);
			if(ks->active_user_id == current_id) { //in case, current instance have been locked
				ks->active_user_id = MAX_VMETA_INSTANCE;
				if(ks->lock_flag==VMETA_LOCK_ON) {
					printk(KERN_ERR "vmeta error , instance id(%d) holds the lock and exit abnormally\n",current_id);
					ks->lock_flag = VMETA_LOCK_FORCE_INIT;
					vmeta_unlock(vi);
				}
			}

			if(ks->user_id_list[current_id].status != 0) {//in case, it's an abnormal exit, we should clear the instance
				printk(KERN_ERR "vmeta error, clear instance[%d],previous status=0x%x\n",\
					   current_id, ks->user_id_list[current_id].status);
				ks->ref_count--;
				memset(&(ks->user_id_list[current_id]),0x0,sizeof(id_instance));
				printk("ref_count=%d, lock flag=%d,active_user_id=%d\n",ks->ref_count,ks->lock_flag,current_id);
			}

			mutex_unlock(&vi->mutex);
			p[i] = MAX_VMETA_INSTANCE;
		}
	}

	mutex_lock(&vi->mutex);
	if(ks->ref_count == 0) {
		mutex_unlock(&vi->mutex);
		vmeta_turn_off(vi);
		mutex_lock(&vi->mutex);
		vi->vop = VMETA_OP_INVALID;
		vi->vop_real = VMETA_OP_INVALID;
	}
	mutex_unlock(&vi->mutex);
	kfree(priv->extend);

	return 0;
}

/* Either register and unregister use the same function */
int vmeta_reg_unreg(struct uio_listener *file_priv, int uid)
{
	int *p = (int*) file_priv->extend;
	int i;

	if(!p) {
		printk(KERN_ERR "vmeta register error, point NULL\n");
		return -1;
	}

	for(i=0;i<MAX_VMETA_INSTANCE;i++) {
		if( p[i] == uid) {
			p[i] = MAX_VMETA_INSTANCE;
			printk(KERN_DEBUG "vmeta kern unregister p[%d]= %d \n",i,uid);
			return 0;
		}
	}

	for(i=0;i<MAX_VMETA_INSTANCE;i++) {
		if( p[i] <0 || p[i] >= MAX_VMETA_INSTANCE)
			break;
	}

	if(i>=MAX_VMETA_INSTANCE) {
		printk(KERN_ERR "vmeta kern register full \n");
		return -1;
	}

	p[i] = uid;
	printk(KERN_DEBUG "vmeta kern register p[%d]= %d \n",i,p[i]);
	return 0;
}

static void __attribute__ ((unused))
vmeta_irq_poll_timer_handler(unsigned long data)
{
	struct vmeta_instance *vi = (struct vmeta_instance *)data;

	uio_event_notify(&vi->uio_info);
	mod_timer(&vi->irq_poll_timer, jiffies + HZ/100);/*10ms timer*/
}

static irqreturn_t
vmeta_func_irq_handler(int irq, struct uio_info *dev_info)
{
	struct vmeta_instance *priv = dev_info->priv;
	unsigned long flags;

	/* Just disable the interrupt in the interrupt controller, and
	 * remember the state so we can allow user space to enable it later.
	 */

	spin_lock_irqsave(&priv->lock, flags);
	if (!test_and_set_bit(0, &priv->flags))
		disable_irq_nosync(irq);

	spin_unlock_irqrestore(&priv->lock, flags);

	return IRQ_HANDLED;
}

static int vmeta_irqcontrol(struct uio_info *dev_info, s32 irq_on)
{
	struct vmeta_instance *priv = dev_info->priv;
	unsigned long flags;

	/* Allow user space to enable and disable the interrupt
	 * in the interrupt controller, but keep track of the
	 * state to prevent per-irq depth damage.
	 *
	 * Serialize this operation to support multiple tasks.
	 */

	spin_lock_irqsave(&priv->lock, flags);
	if (irq_on) {
		if (test_and_clear_bit(0, &priv->flags))
			enable_irq(dev_info->irq);
	} else {
		if (!test_and_set_bit(0, &priv->flags))
			disable_irq(dev_info->irq);
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int vmeta_ioctl(struct uio_info *info, unsigned int cmd, unsigned long arg, void *file_priv)
{
	int ret = 0;
	struct vmeta_instance *priv = info->priv;

	switch(cmd) {
		case VMETA_CMD_POWER_ON:
			ret = vmeta_power_on(priv);
			break;
		case VMETA_CMD_POWER_OFF:
			ret = vmeta_power_off(priv);
			break;
		case VMETA_CMD_CLK_ON:
			ret = vmeta_clk_on(priv);
			break;
		case VMETA_CMD_CLK_OFF:
			ret = vmeta_clk_off(priv);
			break;
		case VMETA_CMD_CLK_SWITCH:
			ret = vmeta_clk_switch(priv, arg);
			break;
		case VMETA_CMD_LOCK:
			ret = vmeta_lock(arg,priv);
			break;
		case VMETA_CMD_UNLOCK:
			ret = vmeta_unlock(priv);
			break;
		case VMETA_CMD_PRIV_LOCK:
			ret = vmeta_priv_lock(arg,priv);
			break;
		case VMETA_CMD_PRIV_UNLOCK:
			ret = vmeta_priv_unlock(priv);
			break;
		case VMETA_CMD_REG_UNREG:
			ret = vmeta_reg_unreg( (struct uio_listener*) file_priv,(int)arg);
		default:
			break;
	}

	return ret;
}

static int vmeta_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct vmeta_instance *vi;
	int ret;
	int irq_func, irq_bus;
	kernel_share* p_ks;
#ifdef CONFIG_MEM_FOR_MULTIPROCESS
	dma_addr_t mem_dma_addr;
	void *mem_vir_addr;
#endif

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		printk(KERN_ERR "vmeta_probe: no memory resources given\n");
		return -ENODEV;
	}

#ifndef CONFIG_VMETA_POLLING_MODE
	irq_func = platform_get_irq(pdev,0);
	if(irq_func < 0){
		printk(KERN_ERR "vmeta_probe: no function irq resources given in interrupt mode\n");
		return -ENODEV;
	}
#endif

	vi = kzalloc(sizeof(*vi), GFP_KERNEL);
	if (vi == NULL) {
		printk(KERN_ERR "vmeta_probe: out of memory\n");
		return -ENOMEM;
	}

	vi->plat_data = (struct vmeta_plat_data *) pdev->dev.platform_data;

	vi->sema = (struct semaphore *)kzalloc(sizeof(struct semaphore),GFP_KERNEL);
	vi->priv_sema = (struct semaphore *)kzalloc(sizeof(struct semaphore),GFP_KERNEL);

	if (vi->sema == NULL || vi->priv_sema == NULL) {
		printk(KERN_ERR "vmeta->sema: out of memory\n");
		return -ENOMEM;
	}

	vi->clk = clk_get(&pdev->dev, "VMETA_CLK");
	if (IS_ERR(vi->clk)) {
		printk(KERN_ERR "vmeta_probe: cannot get vmeta clock\n");
		ret = PTR_ERR(vi->clk);
		vi->clk = NULL;
		goto out_free;
	}

	if (vi->plat_data->axi_clk_available != 0) {
		vi->axi_clk = clk_get(&pdev->dev, "AXICLK");
		if (IS_ERR(vi->axi_clk)) {
			printk(KERN_ERR "vmeta_probe: cannot get AXI clock\n");
			ret = PTR_ERR(vi->axi_clk);
			vi->axi_clk = NULL;
			goto out_free;
		}
	}

	vi->reg_base = ioremap(res->start, res->end - res->start + 1);
	if (vi->reg_base == NULL) {
		printk(KERN_ERR "vmeta_probe: can't remap register area\n");
		ret = -ENOMEM;
		goto out_free;
	}

	platform_set_drvdata(pdev, vi);

	spin_lock_init(&vi->lock);
	vi->flags = 0; /* interrupt is enabled to begin with */

	vi->uio_info.name = UIO_VMETA_NAME;
	vi->uio_info.version = UIO_VMETA_VERSION;
	vi->uio_info.mem[0].internal_addr = vi->reg_base;
	vi->uio_info.mem[0].addr = res->start;
	vi->uio_info.mem[0].memtype = UIO_MEM_PHYS;
	vi->uio_info.mem[0].size = res->end - res->start + 1;
#ifdef CONFIG_MEM_FOR_MULTIPROCESS
	mem_vir_addr = dma_alloc_coherent(&pdev->dev, VDEC_HW_CONTEXT_SIZE,
			&mem_dma_addr, GFP_KERNEL);
	if (!mem_vir_addr) {
		ret = -ENOMEM;
		goto out_free;
	}
	vi->uio_info.mem[1].internal_addr = mem_vir_addr;
	vi->uio_info.mem[1].addr = mem_dma_addr;
	vi->uio_info.mem[1].memtype = UIO_MEM_PHYS;
	vi->uio_info.mem[1].size = VDEC_HW_CONTEXT_SIZE;
	vmeta_print("[1] internal addr[0x%08x],addr[0x%08x] size[%ld] \n",(unsigned int)vi->uio_info.mem[1].internal_addr,
				(unsigned int)vi->uio_info.mem[1].addr,vi->uio_info.mem[1].size);

	/*this memory is allocated for VDEC_OBJ*/
	mem_vir_addr = dma_alloc_coherent(&pdev->dev, VDEC_OBJ_SIZE,&mem_dma_addr, GFP_KERNEL);
	if (!mem_vir_addr) {
		ret = -ENOMEM;
		goto out_free;
	}

	vi->uio_info.mem[2].internal_addr = mem_vir_addr;
	vi->uio_info.mem[2].addr = (unsigned long)mem_dma_addr;
	vi->uio_info.mem[2].memtype = UIO_MEM_PHYS;
	vi->uio_info.mem[2].size = VDEC_OBJ_SIZE;
	vmeta_print("[2] internal addr[0x%08x],addr[0x%08x] size[%ld] \n",(unsigned int)vi->uio_info.mem[2].internal_addr,
				(unsigned int)vi->uio_info.mem[2].addr,vi->uio_info.mem[2].size);

	/*this memory is allocated for vmeta driver internally and shared between user space and kernel space*/
	mem_vir_addr = dma_alloc_coherent(&pdev->dev, KERNEL_SHARE_SIZE,&mem_dma_addr, GFP_KERNEL);
	if (!mem_vir_addr) {
		ret = -ENOMEM;
		goto out_free;
	}
	memset(mem_vir_addr,0,KERNEL_SHARE_SIZE);
	vi->uio_info.mem[3].internal_addr = mem_vir_addr;
	vi->uio_info.mem[3].addr = (unsigned long)mem_dma_addr;
	vi->uio_info.mem[3].memtype = UIO_MEM_PHYS;
	vi->uio_info.mem[3].size = KERNEL_SHARE_SIZE;
	vmeta_print("[3] internal addr[0x%08x],addr[0x%08x] size[%ld] \n",(unsigned int)vi->uio_info.mem[3].internal_addr,
				(unsigned int)vi->uio_info.mem[3].addr,vi->uio_info.mem[3].size);

	p_ks = (kernel_share*) mem_vir_addr;
	p_ks->active_user_id = MAX_VMETA_INSTANCE;

#endif

#ifdef CONFIG_VMETA_POLLING_MODE
	vi->uio_info.irq = UIO_IRQ_CUSTOM;
	init_timer(&vi->irq_poll_timer);
	vi->irq_poll_timer.data = (unsigned long)vi;
	vi->irq_poll_timer.function = vmeta_irq_poll_timer_handler;
#else
	vi->uio_info.irq_flags = IRQF_DISABLED;
	vi->uio_info.irq = irq_func;
	vi->uio_info.handler = vmeta_func_irq_handler;
	vi->uio_info.irqcontrol = vmeta_irqcontrol;
#endif
	vi->uio_info.priv = vi;

	vi->uio_info.open = vmeta_open;
	vi->uio_info.release = vmeta_release;
	vi->uio_info.ioctl = vmeta_ioctl;

	mutex_init(&(vi->mutex));
	vmeta_lock_init(vi);

	init_timer(&vi->power_timer);
	vi->power_timer.data = (unsigned long) vi;
	vi->power_timer.function = vmeta_power_timer_handler;
	vi->power_down_ms=10;

	vi->vop = VMETA_OP_INVALID;
	vi->vop_real = VMETA_OP_INVALID;
	ret = uio_register_device(&pdev->dev, &vi->uio_info);
	if (ret)
		goto out_free;

#ifdef CONFIG_VMETA_POLLING_MODE
	mod_timer(&vi->irq_poll_timer, jiffies + HZ/100);
#endif

	irq_bus = platform_get_irq(pdev,1);
	if(irq_bus < 0){
		printk(KERN_DEBUG "vmeta_probe: no bus irq resources given\n");
	}
	else {
		if(vi->plat_data->bus_irq_handler) {
			ret = request_irq(irq_bus, vi->plat_data->bus_irq_handler, 0, UIO_VMETA_BUS_IRQ_NAME, vi);
			if (ret) {
				printk(KERN_ERR "vmeta_probe: can't request bus irq\n");
				goto out_free;
			}
		}
	}
	if(vi->plat_data->init_dvfm_constraint) {
		vi->plat_data->init_dvfm_constraint(vi,dvfm_lock.dev_idx);
	}

	vmeta_inst = vi;
	return 0;

out_free:
	if (NULL != vi->clk) {
		clk_disable(vi->clk);
		clk_put(vi->clk);
	}

	if (NULL != vi->axi_clk) {
		clk_disable(vi->axi_clk);
		clk_put(vi->axi_clk);
	}

	kfree(vi->sema);
	kfree(vi->priv_sema);
	kfree(vi);

	return ret;
}

static int vmeta_remove(struct platform_device *pdev)
{
	struct vmeta_instance *vi = platform_get_drvdata(pdev);
#ifdef CONFIG_VMETA_POLLING_MODE
	del_timer_sync(&vi->irq_poll_timer);
#endif
#ifdef CONFIG_MEM_FOR_MULTIPROCESS
	dma_free_coherent(&pdev->dev, VDEC_HW_CONTEXT_SIZE,
		vi->uio_info.mem[1].internal_addr, vi->uio_info.mem[1].addr);
	dma_free_coherent(&pdev->dev, VDEC_OBJ_SIZE,
		vi->uio_info.mem[2].internal_addr, vi->uio_info.mem[2].addr);
	dma_free_coherent(&pdev->dev, KERNEL_SHARE_SIZE,
		vi->uio_info.mem[3].internal_addr, vi->uio_info.mem[3].addr);
#endif
	uio_unregister_device(&vi->uio_info);

	if (NULL != vi->clk) {
		clk_disable(vi->clk);
		clk_put(vi->clk);
	}

	if (NULL != vi->axi_clk) {
		clk_disable(vi->axi_clk);
		clk_put(vi->axi_clk);
	}

	iounmap(vi->reg_base);
	kfree(vi->sema);
	kfree(vi->priv_sema);
	kfree(vi);

	return 0;
}

static void vmeta_shutdown(struct platform_device *dev)
{
}

#ifdef CONFIG_PM
static int vmeta_suspend(struct platform_device *dev, pm_message_t state)
{
	return 0;
}

static int vmeta_resume(struct platform_device *dev)
{
	return 0;
}
#endif

static struct platform_driver vmeta_driver = {
	.probe		= vmeta_probe,
	.remove		= vmeta_remove,
	.shutdown	= vmeta_shutdown,
#ifdef CONFIG_PM
	.suspend	= vmeta_suspend,
	.resume		= vmeta_resume,
#endif
	.driver = {
		.name	= UIO_VMETA_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init vmeta_init(void)
{
	int ret;
	ret = dvfm_register("VMETA", &dvfm_lock.dev_idx);
	if(ret) {
		printk(KERN_ERR "vmeta dvfm register fail(%d)\n",ret);
	}
	return platform_driver_register(&vmeta_driver);
}

static void __exit vmeta_exit(void)
{
	dvfm_unregister("VMETA", &dvfm_lock.dev_idx);
	platform_driver_unregister(&vmeta_driver);
}

module_init(vmeta_init);
module_exit(vmeta_exit);

MODULE_DESCRIPTION("UIO driver for Marvell multi-format video codec engine");
MODULE_LICENSE("GPL");
