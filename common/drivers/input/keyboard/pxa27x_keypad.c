/*
 * linux/drivers/input/keyboard/pxa27x_keypad.c
 *
 * Driver for the pxa27x matrix keyboard controller.
 *
 * Created:	Feb 22, 2007
 * Author:	Rodolfo Giometti <giometti@linux.it>
 *
 * Based on a previous implementations by Kevin O'Connor
 * <kevin_at_koconnor.net> and Alex Osborne <bobofdoom@gmail.com> and
 * on some suggestions by Nicolas Pitre <nico@fluxnic.net>.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/input/matrix_keypad.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/notifier.h>
#include <linux/timer.h>
#include <linux/delay.h>

#include <linux/workqueue.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>

#include <mach/hardware.h>
#include <plat/pxa27x_keypad.h>
#include <mach/dvfm.h>
#include <mach/sec_debug.h>	// + SAMSUNG_SMN.
#ifdef CONFIG_PXA95x
#include <mach/pxa95x_dvfm.h>
#include <mach/pxa95x_pm.h>
#include <linux/init.h>
#endif
#include <linux/workqueue.h>

/*
 * Keypad Controller registers
 */
#define KPC             0x0000 /* Keypad Control register */
#define KPDK            0x0008 /* Keypad Direct Key register */
#define KPREC           0x0010 /* Keypad Rotary Encoder register */
#define KPMK            0x0018 /* Keypad Matrix Key register */
#define KPAS            0x0020 /* Keypad Automatic Scan register */

/* Keypad Automatic Scan Multiple Key Presser register 0-3 */
#define KPASMKP0        0x0028
#define KPASMKP1        0x0030
#define KPASMKP2        0x0038
#define KPASMKP3        0x0040
#define KPKDI           0x0048

/* bit definitions */
#define KPC_MKRN(n)	((((n) - 1) & 0x7) << 26) /* matrix key row number */
#define KPC_MKCN(n)	((((n) - 1) & 0x7) << 23) /* matrix key column number */
#define KPC_DKN(n)	((((n) - 1) & 0x7) << 6)  /* direct key number */

#define KPC_AS          (0x1 << 30)  /* Automatic Scan bit */
#define KPC_ASACT       (0x1 << 29)  /* Automatic Scan on Activity */
#define KPC_MI          (0x1 << 22)  /* Matrix interrupt bit */
#define KPC_IMKP        (0x1 << 21)  /* Ignore Multiple Key Press */

#define KPC_MS(n)	(0x1 << (13 + (n)))	/* Matrix scan line 'n' */
#define KPC_MS_ALL      (0xff << 13)

#define KPC_ME          (0x1 << 12)  /* Matrix Keypad Enable */
#define KPC_MIE         (0x1 << 11)  /* Matrix Interrupt Enable */
#define KPC_DK_DEB_SEL	(0x1 <<  9)  /* Direct Keypad Debounce Select */
#define KPC_DI          (0x1 <<  5)  /* Direct key interrupt bit */
#define KPC_RE_ZERO_DEB (0x1 <<  4)  /* Rotary Encoder Zero Debounce */
#define KPC_REE1        (0x1 <<  3)  /* Rotary Encoder1 Enable */
#define KPC_REE0        (0x1 <<  2)  /* Rotary Encoder0 Enable */
#define KPC_DE          (0x1 <<  1)  /* Direct Keypad Enable */
#define KPC_DIE         (0x1 <<  0)  /* Direct Keypad interrupt Enable */

#define KPDK_DKP        (0x1 << 31)
#define KPDK_DK(n)	((n) & 0xff)

#define KPREC_OF1       (0x1 << 31)
#define kPREC_UF1       (0x1 << 30)
#define KPREC_OF0       (0x1 << 15)
#define KPREC_UF0       (0x1 << 14)

#define KPREC_RECOUNT0(n)	((n) & 0xff)
#define KPREC_RECOUNT1(n)	(((n) >> 16) & 0xff)

#define KPMK_MKP        (0x1 << 31)
#define KPAS_SO         (0x1 << 31)
#define KPASMKPx_SO     (0x1 << 31)

#define KPAS_MUKP(n)	(((n) >> 26) & 0x1f)
#define KPAS_RP(n)	(((n) >> 4) & 0xf)
#define KPAS_CP(n)	((n) & 0xf)

#define KPASMKP_MKC_MASK	(0xff)

#define keypad_readl(off)	__raw_readl(keypad->mmio_base + (off))
#define keypad_writel(off, v)	__raw_writel((v), keypad->mmio_base + (off))

#define MAX_MATRIX_KEY_NUM	(MAX_MATRIX_KEY_ROWS * MAX_MATRIX_KEY_COLS)
#define MAX_KEYPAD_KEYS		(MAX_MATRIX_KEY_NUM + MAX_DIRECT_KEY_NUM)

#ifdef CONFIG_PXA95x
#define D2_STABLE_JIFFIES               10
static int keypad_notifier_freq(struct notifier_block *nb,
		unsigned long val, void *data);

static struct notifier_block notifier_freq_block = {
	.notifier_call = keypad_notifier_freq,
};

static struct dvfm_lock dvfm_lock = {
	.lock		= __SPIN_LOCK_UNLOCKED(dvfm_lock.lock),
	.dev_idx	= -1,
	.count		= 0,
};

static struct timer_list kp_timer;
static void unset_dvfm_constraint(void);
#endif

// + SAMSUNG_LEEMY
struct work_struct work_dump;
struct workqueue_struct *dumpstate_dump_wq;
// - SAMSUNG_LEEMY

// + SAMSUNG_EUNYONG
static int debug_level_param=0;
// - SAMSUNG_EUNYONG

// + SAMSUNG_JBSON
struct work_struct work_capture;
// - SAMSUNG_JBSON

struct pxa27x_keypad {
	struct pxa27x_keypad_platform_data *pdata;

	struct clk *clk;
	struct input_dev *input_dev;
	void __iomem *mmio_base;

	int irq;

	unsigned short keycodes[MAX_KEYPAD_KEYS];
	int rotary_rel_code[2];

	/* state row bits of each column scan */
	uint32_t matrix_key_state[MAX_MATRIX_KEY_COLS];
	uint32_t direct_key_state;

	unsigned int direct_key_mask;
	struct notifier_block	notifier_freq_block;
	struct work_struct	keypad_lpm_work;
	struct workqueue_struct	*keypad_lpm_wq;

};
static struct pxa27x_keypad *g_pxa27x_keypad;



/* sys fs  */
struct class *key_class;
EXPORT_SYMBOL(key_class);
struct device *key_dev;
EXPORT_SYMBOL(key_dev);
 
static ssize_t key_show(struct device *dev, struct device_attribute *attr, char *buf);
static DEVICE_ATTR(key , 0444, key_show, NULL);

static ssize_t key_show(struct device *dev, struct device_attribute *attr, char *buf)
{
     int key_pressed = 0;
     int num_keys_pressed = 0;
     struct pxa27x_keypad *keypad = g_pxa27x_keypad;


     uint32_t kpas = keypad_readl(KPAS);

     num_keys_pressed = KPAS_MUKP(kpas);

     return sprintf(buf, "%d\n", num_keys_pressed );
    
}
/* sys fs */



// + SAMSUNG_SSENI : for forced ramdump
int waitForcedRamdump = 0;
int current_key = 0;
int state=0;
// - SAMSUNG_SSENI : for forced ramdump

static void pxa27x_keypad_build_keycode(struct pxa27x_keypad *keypad)
{
	struct pxa27x_keypad_platform_data *pdata = keypad->pdata;
	struct input_dev *input_dev = keypad->input_dev;
	unsigned short keycode;
	int i;

	for (i = 0; i < pdata->matrix_key_map_size; i++) {
		unsigned int key = pdata->matrix_key_map[i];
		unsigned int row = KEY_ROW(key);
		unsigned int col = KEY_COL(key);
		unsigned int scancode = MATRIX_SCAN_CODE(row, col,
							 MATRIX_ROW_SHIFT);

		keycode = KEY_VAL(key);
		keypad->keycodes[scancode] = keycode;
		__set_bit(keycode, input_dev->keybit);
	}

	for (i = 0; i < pdata->direct_key_num; i++) {
		keycode = pdata->direct_key_map[i];
		keypad->keycodes[MAX_MATRIX_KEY_NUM + i] = keycode;
		__set_bit(keycode, input_dev->keybit);
	}

	if (pdata->enable_rotary0) {
		if (pdata->rotary0_up_key && pdata->rotary0_down_key) {
			keycode = pdata->rotary0_up_key;
			keypad->keycodes[MAX_MATRIX_KEY_NUM + 0] = keycode;
			__set_bit(keycode, input_dev->keybit);

			keycode = pdata->rotary0_down_key;
			keypad->keycodes[MAX_MATRIX_KEY_NUM + 1] = keycode;
			__set_bit(keycode, input_dev->keybit);

			keypad->rotary_rel_code[0] = -1;
		} else {
			keypad->rotary_rel_code[0] = pdata->rotary0_rel_code;
			__set_bit(pdata->rotary0_rel_code, input_dev->relbit);
		}
	}

	if (pdata->enable_rotary1) {
		if (pdata->rotary1_up_key && pdata->rotary1_down_key) {
			keycode = pdata->rotary1_up_key;
			keypad->keycodes[MAX_MATRIX_KEY_NUM + 2] = keycode;
			__set_bit(keycode, input_dev->keybit);

			keycode = pdata->rotary1_down_key;
			keypad->keycodes[MAX_MATRIX_KEY_NUM + 3] = keycode;
			__set_bit(keycode, input_dev->keybit);

			keypad->rotary_rel_code[1] = -1;
		} else {
			keypad->rotary_rel_code[1] = pdata->rotary1_rel_code;
			__set_bit(pdata->rotary1_rel_code, input_dev->relbit);
		}
	}

	__clear_bit(KEY_RESERVED, input_dev->keybit);
}

// + SAMSUNG_LEEMY
static inline unsigned int lookup_matrix_keycode(
		struct pxa27x_keypad *keypad, int row, int col)
{
	return keypad->keycodes[(row << 3) + col];
}
// - SAMSUNG_LEEMY

static void dumpstate_dump()
{
	// struct subprocess_info *sub_info;
	char *argv_command[] = { "/system/bin/samsung_debug", "d", NULL };
	char *envp_command[] = {
				"HOME=/",
				"TERM=linux",
				"PATH=/sbin:/system/sbin:/system/bin:/system/xbin:/marvell/tel",
				"ANDROID_ROOT=/system",
				NULL };
	call_usermodehelper( argv_command[0], argv_command,
							envp_command, UMH_WAIT_PROC );
							//envp_command, UMH_NO_WAIT );
							//envp_command, UMH_WAIT_EXEC );
}

// + SAMSUNG_JBSON
static void screen_capture()
{
  printk("screen_calture() was called!!!\n");

	char *argv_command[] = { "/system/bin/samsung_debug", "s", NULL };
	char *envp_command[] = {
				"HOME=/",
				"TERM=linux",
				"PATH=/sbin:/system/sbin:/system/bin:/system/xbin:/marvell/tel:/sdcard/Pictures",
				"ANDROID_ROOT=/system",
                "BOOTCLASSPATH=/system/framework/core.jar:/system/framework/ext.jar:/system/framework/framework.jar:/system/framework/android.policy.jar:/system/framework/services.jar",
				NULL };
	call_usermodehelper( argv_command[0], argv_command,
							envp_command, UMH_WAIT_PROC );

}
// - SAMSUNG_JBSON

static void pxa27x_keypad_scan_matrix(struct pxa27x_keypad *keypad)
{
	struct pxa27x_keypad_platform_data *pdata = keypad->pdata;
	struct input_dev *input_dev = keypad->input_dev;
	int row, col, num_keys_pressed = 0;
	uint32_t new_state[MAX_MATRIX_KEY_COLS];
	uint32_t kpas = keypad_readl(KPAS);
		// + SAMSUNG_LEEMY
		int status;
		// - SAMSUNG_LEEMY

	num_keys_pressed = KPAS_MUKP(kpas);

	memset(new_state, 0, sizeof(new_state));

	if (num_keys_pressed == 0)
		goto scan;

	if (num_keys_pressed == 1) {
		col = KPAS_CP(kpas);
		row = KPAS_RP(kpas);

		/* if invalid row/col, treat as no key pressed */
		if (col >= pdata->matrix_key_cols ||
		    row >= pdata->matrix_key_rows)
			goto scan;

		new_state[col] = (1 << row);
		goto scan;
	}

	/* do not scan more than two keys to avoid fake key */
	if (num_keys_pressed > 3)
		return;

	if (num_keys_pressed > 1) {
		uint32_t kpasmkp0 = keypad_readl(KPASMKP0);
		uint32_t kpasmkp1 = keypad_readl(KPASMKP1);
		uint32_t kpasmkp2 = keypad_readl(KPASMKP2);
		uint32_t kpasmkp3 = keypad_readl(KPASMKP3);

		new_state[0] = kpasmkp0 & KPASMKP_MKC_MASK;
		new_state[1] = (kpasmkp0 >> 16) & KPASMKP_MKC_MASK;
		new_state[2] = kpasmkp1 & KPASMKP_MKC_MASK;
		new_state[3] = (kpasmkp1 >> 16) & KPASMKP_MKC_MASK;
		new_state[4] = kpasmkp2 & KPASMKP_MKC_MASK;
		new_state[5] = (kpasmkp2 >> 16) & KPASMKP_MKC_MASK;
		new_state[6] = kpasmkp3 & KPASMKP_MKC_MASK;
		new_state[7] = (kpasmkp3 >> 16) & KPASMKP_MKC_MASK;
	}
scan:
	for (col = 0; col < pdata->matrix_key_cols; col++) {
		uint32_t bits_changed;
		int code;

		bits_changed = keypad->matrix_key_state[col] ^ new_state[col];
		if (bits_changed == 0)
			continue;

		for (row = 0; row < pdata->matrix_key_rows; row++) {
			if ((bits_changed & (1 << row)) == 0)
				continue;
			// + SAMSUNG_SSENI
			current_key = lookup_matrix_keycode(keypad, row, col);
			state = new_state[col] & (1 << row);
			{
			    if(current_key == 0x66 && state != 0)
			    {
				printk("waitForcedRamdump = 1\n");
				waitForcedRamdump = 1;
			    }
			    else if(state == 0)
			    {
				printk("waitForcedRamdump = 0\n");
				waitForcedRamdump = 0;
			    }
			    if(waitForcedRamdump && current_key == 0x73 && debug_level_param > 0)
			    {
				/* + SAMSUNG_SMN-c: Writing the upload cause to DDR 
			     */
			    unsigned long *upload_cause = phys_to_virt(UPLOAD_CAUSE_ADDR);
				*upload_cause = UPLOAD_CAUSE_FORCED_CP_UPLOAD;
				/* - SAMSUNG_SMN */
				// + SAMSUNG_LEEMY
				printk("Force dumpstate and panic !!!\n");
				status = queue_work(dumpstate_dump_wq, &work_dump);
				//panic("FORCED RAMDUMP !!!\n");
				// - SAMSUNG_LEEMY
			    }
			}
			input_report_key(keypad->input_dev, current_key, state);


			if(state == 0)
			{
				//printk("key 0x%x released, pressed key_cnt = %d \n", current_key, num_keys_pressed);
				printk("key released\n");
			}
			else
			{
				//printk("key 0x%x pressed, pressed key_cnt = %d \n", current_key, num_keys_pressed);
				printk("key pressed\n");
			}

		
			// - SAMSUNG_SSENI

			code = MATRIX_SCAN_CODE(row, col, MATRIX_ROW_SHIFT);
			input_event(input_dev, EV_MSC, MSC_SCAN, code);
			input_report_key(input_dev, keypad->keycodes[code],
					 new_state[col] & (1 << row));
			dev_dbg(&input_dev->dev,"key 0x%x reported.\n",keypad->keycodes[code]);
		}
	}
	input_sync(input_dev);
	memcpy(keypad->matrix_key_state, new_state, sizeof(new_state));
}

#define DEFAULT_KPREC	(0x007f007f)

static inline int rotary_delta(uint32_t kprec)
{
	if (kprec & KPREC_OF0)
		return (kprec & 0xff) + 0x7f;
	else if (kprec & KPREC_UF0)
		return (kprec & 0xff) - 0x7f - 0xff;
	else
		return (kprec & 0xff) - 0x7f;
}

static void report_rotary_event(struct pxa27x_keypad *keypad, int r, int delta)
{
	struct input_dev *dev = keypad->input_dev;

	if (delta == 0)
		return;

	if (keypad->rotary_rel_code[r] == -1) {
		int code = MAX_MATRIX_KEY_NUM + 2 * r + (delta > 0 ? 0 : 1);
		unsigned char keycode = keypad->keycodes[code];

		/* simulate a press-n-release */
		input_event(dev, EV_MSC, MSC_SCAN, code);
		input_report_key(dev, keycode, 1);
		input_sync(dev);
		input_event(dev, EV_MSC, MSC_SCAN, code);
		input_report_key(dev, keycode, 0);
		input_sync(dev);
	} else {
		input_report_rel(dev, keypad->rotary_rel_code[r], delta);
		input_sync(dev);
	}
}

static void pxa27x_keypad_scan_rotary(struct pxa27x_keypad *keypad)
{
	struct pxa27x_keypad_platform_data *pdata = keypad->pdata;
	uint32_t kprec;

	/* read and reset to default count value */
	kprec = keypad_readl(KPREC);
	keypad_writel(KPREC, DEFAULT_KPREC);

	if (pdata->enable_rotary0)
		report_rotary_event(keypad, 0, rotary_delta(kprec));

	if (pdata->enable_rotary1)
		report_rotary_event(keypad, 1, rotary_delta(kprec >> 16));
}

static void pxa27x_keypad_scan_direct(struct pxa27x_keypad *keypad)
{
	struct pxa27x_keypad_platform_data *pdata = keypad->pdata;
	struct input_dev *input_dev = keypad->input_dev;
	unsigned int new_state;
	uint32_t kpdk, bits_changed;
	int i;

	kpdk = keypad_readl(KPDK);

	if (pdata->enable_rotary0 || pdata->enable_rotary1)
		pxa27x_keypad_scan_rotary(keypad);

	if (pdata->active_low)
		new_state = ~KPDK_DK(kpdk) & keypad->direct_key_mask;
	else
		new_state = KPDK_DK(kpdk) & keypad->direct_key_mask;

	bits_changed = keypad->direct_key_state ^ new_state;

	if (bits_changed == 0)
		return;

	for (i = 0; i < pdata->direct_key_num; i++) {
		if (bits_changed & (1 << i)) {
			int code = MAX_MATRIX_KEY_NUM + i;

			input_event(input_dev, EV_MSC, MSC_SCAN, code);
			input_report_key(input_dev, keypad->keycodes[code],
					 new_state & (1 << i));
		}
	}
	input_sync(input_dev);
	keypad->direct_key_state = new_state;
}

static void clear_wakeup_event(struct pxa27x_keypad *keypad)
{
	struct pxa27x_keypad_platform_data *pdata = keypad->pdata;

	if (pdata->clear_wakeup_event)
		(pdata->clear_wakeup_event)();
}

static irqreturn_t pxa27x_keypad_irq_handler(int irq, void *dev_id)
{
	struct pxa27x_keypad *keypad = dev_id;
	unsigned long kpc = keypad_readl(KPC);

	clear_wakeup_event(keypad);

	if (kpc & KPC_DI)
		pxa27x_keypad_scan_direct(keypad);

	if (kpc & KPC_MI)
		pxa27x_keypad_scan_matrix(keypad);

	return IRQ_HANDLED;
}

static void pxa27x_keypad_config(struct pxa27x_keypad *keypad)
{
	struct pxa27x_keypad_platform_data *pdata = keypad->pdata;
	unsigned int mask = 0, direct_key_num = 0;
	unsigned long kpc = 0;

	/* enable matrix keys with automatic scan */
	if (pdata->matrix_key_rows && pdata->matrix_key_cols) {
		kpc |= KPC_ASACT | KPC_MIE | KPC_ME | KPC_MS_ALL;
		kpc |= KPC_MKRN(pdata->matrix_key_rows) |
		       KPC_MKCN(pdata->matrix_key_cols);
	}

	/* enable rotary key, debounce interval same as direct keys */
	if (pdata->enable_rotary0) {
		mask |= 0x03;
		direct_key_num = 2;
		kpc |= KPC_REE0;
	}

	if (pdata->enable_rotary1) {
		mask |= 0x0c;
		direct_key_num = 4;
		kpc |= KPC_REE1;
	}

	if (pdata->direct_key_num > direct_key_num)
		direct_key_num = pdata->direct_key_num;

	keypad->direct_key_mask = ((2 << direct_key_num) - 1) & ~mask;

	/* enable direct key */
	if (direct_key_num)
		kpc |= KPC_DE | KPC_DIE | KPC_DKN(direct_key_num);

	keypad_writel(KPC, kpc | KPC_RE_ZERO_DEB);
	keypad_writel(KPREC, DEFAULT_KPREC);
	keypad_writel(KPKDI, pdata->debounce_interval);
}

static int pxa27x_keypad_open(struct input_dev *dev)
{
	struct pxa27x_keypad *keypad = input_get_drvdata(dev);

	/* Enable unit clock */
	clk_enable(keypad->clk);
	pxa27x_keypad_config(keypad);

	if (keypad->pdata->power_set)
		keypad->pdata->power_set(1);

	return 0;
}

static void pxa27x_keypad_close(struct input_dev *dev)
{
	struct pxa27x_keypad *keypad = input_get_drvdata(dev);

	/* Disable clock unit */
	clk_disable(keypad->clk);

	if (keypad->pdata->power_set)
		keypad->pdata->power_set(0);
}
#ifdef CONFIG_PXA95x

static void set_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count++ == 0) {
		/* Disable lowpower mode */
		dvfm_disable_op_name("D1", dvfm_lock.dev_idx);
		dvfm_disable_op_name("D2", dvfm_lock.dev_idx);
		dvfm_disable_op_name("CG", dvfm_lock.dev_idx);
	}
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

static void unset_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	if (dvfm_lock.count == 0) {
		printk(KERN_WARNING "Keypad constraint has been removed.\n");
	} else if (--dvfm_lock.count == 0) {
		/* Enable lowpower mode */
		dvfm_enable_op_name("D1", dvfm_lock.dev_idx);
		dvfm_enable_op_name("D2", dvfm_lock.dev_idx);
		dvfm_enable_op_name("CG", dvfm_lock.dev_idx);
	}
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
}

static void pxa27x_keypad_wq_func(struct work_struct *work)
{
	set_dvfm_constraint();
	mod_timer(&kp_timer, jiffies + D2_STABLE_JIFFIES);
}

/*
 * FIXME: Here a timer is used to disable entering D1/D2 for a while.
 * Because keypad event wakeup system from D1/D2 mode. But keypad device
 * can't detect the interrupt since it's in standby state.
 * Keypad device need time to detect it again. So we use a timer here.
 * D1/D2 idle is determined by idle time. It's better to comine these
 * timers together.
 */
static void keypad_timer_handler(unsigned long data)
{
	unset_dvfm_constraint();
}

extern void get_wakeup_source(pm_wakeup_src_t *);

static int keypad_notifier_freq(struct notifier_block *nb,
		unsigned long val, void *data)
{
	struct dvfm_freqs *freqs = (struct dvfm_freqs *)data;
	struct op_info *new = NULL;
	struct dvfm_md_opt *op;
	pm_wakeup_src_t src;
	struct pxa27x_keypad *keypad = g_pxa27x_keypad;

	if (freqs)
		new = &freqs->new_info;
	else
		return 0;

	op = (struct dvfm_md_opt *)new->op;
	if (val == DVFM_FREQ_POSTCHANGE) {
		if ((op->power_mode == POWER_MODE_D1) ||
				(op->power_mode == POWER_MODE_D2) ||
				(op->power_mode == POWER_MODE_CG)) {
			get_wakeup_source(&src);
			if (src.bits.mkey || src.bits.dkey) {
				/* If keypad event happens and wake system
				 * from D1/D2. Disable D1/D2 to make keypad
				 * work for a while.
				 */
				if (likely(keypad->keypad_lpm_wq)) {
					queue_work(keypad->keypad_lpm_wq, &keypad->keypad_lpm_work);
				}
			}
		}
	}
	return 0;
}
#endif

#ifdef CONFIG_ARCH_MMP
static void pxa27x_keypad_wq_func(struct work_struct *work)
{
	struct pxa27x_keypad *keypad = g_pxa27x_keypad;
	u32 kpc;

	keypad_writel(KPC, keypad_readl(KPC) | KPC_AS);
	do {
		mdelay(2);
		kpc = keypad_readl(KPC);
	} while (kpc & KPC_AS);

	if (kpc & KPC_ME)
		pxa27x_keypad_scan_matrix(keypad);

	if (kpc & KPC_DE)
		pxa27x_keypad_scan_direct(keypad);
}

void trigger_keypad_scan(void)
{
	struct pxa27x_keypad *keypad = g_pxa27x_keypad;

	queue_work(keypad->keypad_lpm_wq, &keypad->keypad_lpm_work);
	return;
}
EXPORT_SYMBOL(trigger_keypad_scan);

#endif

#ifdef CONFIG_PM
static int pxa27x_keypad_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pxa27x_keypad *keypad = platform_get_drvdata(pdev);

	if (device_may_wakeup(&pdev->dev))
		enable_irq_wake(keypad->irq);
	else {
		clk_disable(keypad->clk);
		if (keypad->pdata->power_set)
			keypad->pdata->power_set(0);
	}

	return 0;
}

static int pxa27x_keypad_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct pxa27x_keypad *keypad = platform_get_drvdata(pdev);
	struct input_dev *input_dev = keypad->input_dev;

	if (device_may_wakeup(&pdev->dev))
		disable_irq_wake(keypad->irq);
	else {
		mutex_lock(&input_dev->mutex);

		if (input_dev->users) {
			/* Enable unit clock */
			clk_enable(keypad->clk);
			pxa27x_keypad_config(keypad);
		}

		mutex_unlock(&input_dev->mutex);

		if (keypad->pdata->power_set)
			keypad->pdata->power_set(1);
	}

	return 0;
}

static const struct dev_pm_ops pxa27x_keypad_pm_ops = {
	.suspend	= pxa27x_keypad_suspend,
	.resume		= pxa27x_keypad_resume,
};
#endif

#ifdef CONFIG_PROC_FS
#define INPUT_LEN 100
static int pxa27x_keypad_write_proc(struct file *file,
		const char __user *buffer,  unsigned long count, void *data)
{
	struct input_dev *dev = data;
	char input[INPUT_LEN];
	int key;
	int i;

	if (count >= INPUT_LEN)
		return -EINVAL;
	if (copy_from_user(input, buffer, count))
		return -EFAULT;
	input[count] = 0;
	i = sscanf(input, "%d", &key);
	printk("\nreport key: %d\n", key);

	set_bit(key, dev->keybit);
	input_report_key(dev, key, 1);
	input_sync(dev);
	input_report_key(dev, key, 0);
	input_sync(dev);

	return count;
}

extern unsigned char get_Onkey_State(void);

static int pxa27x_keypad_read_proc(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int num_keys_pressed = 0;
	unsigned char onkey_pressed=0;
	struct pxa27x_keypad *keypad = g_pxa27x_keypad;
	uint32_t kpas = keypad_readl(KPAS);
	int ret = 0;


	onkey_pressed = get_Onkey_State();

	num_keys_pressed = KPAS_MUKP(kpas);

	if(onkey_pressed)
	{
		num_keys_pressed = num_keys_pressed + 1;
	}
	

	ret = sprintf(page, "num_keys_pressed = %d\n", num_keys_pressed);
	return ret;
}

static void pxa27x_keypad_create_proc_file(void *data)
{
	struct proc_dir_entry *proc_file =
		create_proc_entry("driver/pxa27x-keypad", 0644, NULL);

	if (proc_file) {
		proc_file->write_proc = pxa27x_keypad_write_proc;
		proc_file->read_proc  = pxa27x_keypad_read_proc;
		proc_file->data = data;
	}else {
		printk(KERN_INFO "proc file create failed!\n");
	}
}

extern struct proc_dir_entry proc_root;
static void pxa27x_keypad_remove_proc_file(void)
{
	remove_proc_entry("driver/pxa27x-keypad", &proc_root);
}
#endif


static int __devinit pxa27x_keypad_probe(struct platform_device *pdev)
{
	struct pxa27x_keypad_platform_data *pdata = pdev->dev.platform_data;
	struct pxa27x_keypad *keypad;
	struct input_dev *input_dev;
	struct resource *res;
	int irq, error;

	// + SAMSUNG_LEEMY
	INIT_WORK(&work_dump, dumpstate_dump);
	// - SAMSUNG_LEEMY
	
	// + SAMSUNG_JBSON
	INIT_WORK(&work_capture, screen_capture);
	// - SAMSUNG_JBSON

	if (pdata == NULL) {
		dev_err(&pdev->dev, "no platform data defined\n");
		return -EINVAL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get keypad irq\n");
		return -ENXIO;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to get I/O memory\n");
		return -ENXIO;
	}

	keypad = kzalloc(sizeof(struct pxa27x_keypad), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!keypad || !input_dev) {
		dev_err(&pdev->dev, "failed to allocate memory\n");
		error = -ENOMEM;
		goto failed_free;
	}

	keypad->pdata = pdata;
	keypad->input_dev = input_dev;
	keypad->irq = irq;

	device_set_wakeup_capable(pdev,1); /* to mention that keypad driver support wakeups */
	device_set_wakeup_enable(pdev,1);  /* by default wakeup is enabled for keypad, user space can change this per OS policy*/

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (res == NULL) {
		dev_err(&pdev->dev, "failed to request I/O memory\n");
		error = -EBUSY;
		goto failed_free;
	}

	keypad->mmio_base = ioremap(res->start, resource_size(res));
	if (keypad->mmio_base == NULL) {
		dev_err(&pdev->dev, "failed to remap I/O memory\n");
		error = -ENXIO;
		goto failed_free_mem;
	}

	keypad->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(keypad->clk)) {
		dev_err(&pdev->dev, "failed to get keypad clock\n");
		error = PTR_ERR(keypad->clk);
		goto failed_free_io;
	}

	input_dev->name ="sec_keypad";
	input_dev->id.bustype = BUS_HOST;
	input_dev->open = pxa27x_keypad_open;
	input_dev->close = pxa27x_keypad_close;
	input_dev->dev.parent = &pdev->dev;

	input_dev->keycode = keypad->keycodes;
	input_dev->keycodesize = sizeof(keypad->keycodes[0]);
	input_dev->keycodemax = ARRAY_SIZE(keypad->keycodes);

	input_set_drvdata(input_dev, keypad);

	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_set_capability(input_dev, EV_MSC, MSC_SCAN);

	pxa27x_keypad_build_keycode(keypad);

	if ((pdata->enable_rotary0 && keypad->rotary_rel_code[0] != -1) ||
	    (pdata->enable_rotary1 && keypad->rotary_rel_code[1] != -1)) {
		input_dev->evbit[0] |= BIT_MASK(EV_REL);
	}
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_REL, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	bitmap_fill(input_dev->keybit, KEY_MAX);
	bitmap_fill(input_dev->relbit, REL_MAX);
	bitmap_fill(input_dev->absbit, ABS_MAX);

	error = request_irq(irq, pxa27x_keypad_irq_handler, IRQF_DISABLED,
			    pdev->name, keypad);
	if (error) {
		dev_err(&pdev->dev, "failed to request IRQ\n");
		goto failed_put_clk;
	}

#ifdef CONFIG_PXA95x

	dvfm_register("Keypad", &dvfm_lock.dev_idx);
	dvfm_register_notifier(&notifier_freq_block,
			DVFM_FREQUENCY_NOTIFIER);

	init_timer(&kp_timer);
	kp_timer.function = keypad_timer_handler;
	kp_timer.data = 0;
	add_timer(&kp_timer);

#endif
	INIT_WORK(&keypad->keypad_lpm_work, pxa27x_keypad_wq_func);
	keypad->keypad_lpm_wq = create_workqueue("keypad_rx_lpm_wq");
	if (unlikely(keypad->keypad_lpm_wq == NULL))
		pr_info("[keypad] warning: create work queue failed\n");

	/* Register the input device */
	error = input_register_device(input_dev);
	if (error) {
		dev_err(&pdev->dev, "failed to register input device\n");
		goto failed_free_irq;
	}

	platform_set_drvdata(pdev, keypad);
	device_init_wakeup(&pdev->dev, 0);
	pxa27x_keypad_create_proc_file(input_dev);
	g_pxa27x_keypad = keypad;

        /* sys fs */
	key_class = class_create(THIS_MODULE, "key");
	if (IS_ERR(key_class))
		pr_err("Failed to create class(key)!\n");

	key_dev = device_create(key_class, NULL, 0, NULL, "key");
	if (IS_ERR(key_dev))
		pr_err("Failed to create device(key)!\n");

	if (device_create_file(key_dev, &dev_attr_key) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_key.attr.name); 
	/* sys fs */
	device_set_wakeup_capable(&pdev->dev,1); /* to mention that keypad driver support wakeups */
	device_set_wakeup_enable(&pdev->dev,1);  /* by default wakeup is enabled for keypad, user space can change this per OS policy*/

	return 0;

failed_free_irq:
	free_irq(irq, pdev);
failed_put_clk:
	clk_put(keypad->clk);
failed_free_io:
	iounmap(keypad->mmio_base);
failed_free_mem:
	release_mem_region(res->start, resource_size(res));
failed_free:
	input_free_device(input_dev);
	kfree(keypad);
	return error;
}

static int __devexit pxa27x_keypad_remove(struct platform_device *pdev)
{
	struct pxa27x_keypad *keypad = platform_get_drvdata(pdev);
	struct resource *res;

	// + SAMSUNG_LEEMY
	if (dumpstate_dump_wq)
		destroy_workqueue(dumpstate_dump_wq);
	// - SAMSUNG_LEEMY

	pxa27x_keypad_remove_proc_file();
	free_irq(keypad->irq, pdev);
	clk_put(keypad->clk);

	input_unregister_device(keypad->input_dev);
	input_free_device(keypad->input_dev);

	iounmap(keypad->mmio_base);

#ifdef CONFIG_PXA95x
	dvfm_unregister_notifier(&notifier_freq_block,
			DVFM_FREQUENCY_NOTIFIER);
	dvfm_unregister("Keypad", &dvfm_lock.dev_idx);
#endif
	if (likely(keypad->keypad_lpm_wq))
		destroy_workqueue(keypad->keypad_lpm_wq);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));

	platform_set_drvdata(pdev, NULL);
	kfree(keypad);

	return 0;
}

// + SAMSUNG_EUNYONG
static int sec_debug_set_debug_level(char *str)
{
        if (!str)
                return -EINVAL;
        if (strcmp(str, "HIGH") == 0)
        {   
                debug_level_param =2 ;
        }   
        else if (strcmp(str, "MID") == 0)
        {   
                debug_level_param =1 ;
        }   
        else 
        {   
                debug_level_param =0 ;
        }   
        return 0;
}
early_param("DEBUG_LEVEL", sec_debug_set_debug_level);
// - SAMSUNG_EUNYONG

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:pxa27x-keypad");

static struct platform_driver pxa27x_keypad_driver = {
	.probe		= pxa27x_keypad_probe,
	.remove		= __devexit_p(pxa27x_keypad_remove),
	.driver		= {
		.name	= "pxa27x-keypad",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pxa27x_keypad_pm_ops,
#endif
	},
};

static int __init pxa27x_keypad_init(void)
{
	printk("pxa27x_keypad_init\n");
	// + SAMSUNG_LEEMY
	dumpstate_dump_wq = create_workqueue("dumpstate_dump_wq");
	if (dumpstate_dump_wq == NULL ) return -ENOMEM;
	// - SAMSUNG_LEEMY
	return platform_driver_register(&pxa27x_keypad_driver);
}

static void __exit pxa27x_keypad_exit(void)
{
	// + SAMSUNG_LEEMY
	if (dumpstate_dump_wq)
		destroy_workqueue(dumpstate_dump_wq);
	// - SAMSUNG_LEEMY
	platform_driver_unregister(&pxa27x_keypad_driver);
}

module_init(pxa27x_keypad_init);
module_exit(pxa27x_keypad_exit);

MODULE_DESCRIPTION("PXA27x Keypad Controller Driver");
MODULE_LICENSE("GPL");
