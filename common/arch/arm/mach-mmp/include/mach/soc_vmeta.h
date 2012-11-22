#ifndef _SOC_VMETA_H_
#define _SOC_VMETA_H_

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#include <linux/uio_driver.h>
#include <linux/uio_vmeta.h>

#if defined(CONFIG_PXA3xx_DVFM)
#define VMETA_DVFM_ENABLE 1
#else
#define VMETA_DVFM_ENABLE 0
#endif

#if VMETA_DVFM_ENABLE
#include <linux/notifier.h>
#include <linux/timer.h>
#include <mach/dvfm.h>
#endif

#define UIO_VMETA_NAME		"mmp-vmeta"
#define UIO_VMETA_BUS_IRQ_NAME  UIO_VMETA_NAME"-bus"
/* private */

struct vmeta_instance {
	void			*reg_base;
	struct uio_info		uio_info;
	struct timer_list	irq_poll_timer;
	spinlock_t		lock;
	unsigned long		flags;
	struct clk		*clk;
	struct clk		*axi_clk;
	struct mutex	mutex;
	int			power_constraint;
	struct			timer_list power_timer;
	int			power_down_ms;
	int			power_status; /* 0-off 1-on */
	int			vop;/* vmeta operating point 0-min, 15-max */
	int			vop_real;/* used in dvfm constraint only */
	int			clk_status;
	struct			semaphore *sema;
	struct			semaphore *priv_sema;
	struct			vmeta_plat_data *plat_data;
};

struct vmeta_plat_data {
	int (*set_dvfm_constraint)(struct vmeta_instance *vi, int idx);
	int (*unset_dvfm_constraint)(struct vmeta_instance *vi, int idx);
	int (*clean_dvfm_constraint)(struct vmeta_instance *vi, int idx);
	int (*init_dvfm_constraint)(struct vmeta_instance *vi, int idx);
	irqreturn_t (*bus_irq_handler)(int irq, void *dev_id);
	int axi_clk_available;
	int (*decrease_core_freq)(const struct vmeta_instance *vi, const int step);
	int (*increase_core_freq)(const struct vmeta_instance *vi, const int step);
};

#define VMETA_PWR_ENABLE 0x1
#define VMETA_PWR_DISABLE 0x0

void vmeta_pwr(unsigned int enableDisable);
irqreturn_t mmp_vmeta_bus_irq_handler(int irq, void *dev_id);
int mmp_vmeta_set_dvfm_constraint(int idx);
int mmp_vmeta_unset_dvfm_constraint(int idx);
void __init mmp_set_vmeta_info(void *info);
#endif
