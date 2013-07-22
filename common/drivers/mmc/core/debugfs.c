/*
 * Debugfs support for hosts and cards
 *
 * Copyright (C) 2008 Atmel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/debugfs.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/stat.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdhci.h>
#include <plat/sdhci.h>

#include "core.h"
#include "mmc_ops.h"

/* The debugfs functions are optimized away when CONFIG_DEBUG_FS isn't set. */
static int mmc_ios_show(struct seq_file *s, void *data)
{
	static const char *vdd_str[] = {
		[8]	= "2.0",
		[9]	= "2.1",
		[10]	= "2.2",
		[11]	= "2.3",
		[12]	= "2.4",
		[13]	= "2.5",
		[14]	= "2.6",
		[15]	= "2.7",
		[16]	= "2.8",
		[17]	= "2.9",
		[18]	= "3.0",
		[19]	= "3.1",
		[20]	= "3.2",
		[21]	= "3.3",
		[22]	= "3.4",
		[23]	= "3.5",
		[24]	= "3.6",
	};
	struct mmc_host	*host = s->private;
	struct mmc_ios	*ios = &host->ios;
	const char *str;

	seq_printf(s, "clock:\t\t%u Hz\n", ios->clock);
	seq_printf(s, "vdd:\t\t%u ", ios->vdd);
	if ((1 << ios->vdd) & MMC_VDD_165_195)
		seq_printf(s, "(1.65 - 1.95 V)\n");
	else if (ios->vdd < (ARRAY_SIZE(vdd_str) - 1)
			&& vdd_str[ios->vdd] && vdd_str[ios->vdd + 1])
		seq_printf(s, "(%s ~ %s V)\n", vdd_str[ios->vdd],
				vdd_str[ios->vdd + 1]);
	else
		seq_printf(s, "(invalid)\n");

	switch (ios->bus_mode) {
	case MMC_BUSMODE_OPENDRAIN:
		str = "open drain";
		break;
	case MMC_BUSMODE_PUSHPULL:
		str = "push-pull";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "bus mode:\t%u (%s)\n", ios->bus_mode, str);

	switch (ios->chip_select) {
	case MMC_CS_DONTCARE:
		str = "don't care";
		break;
	case MMC_CS_HIGH:
		str = "active high";
		break;
	case MMC_CS_LOW:
		str = "active low";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "chip select:\t%u (%s)\n", ios->chip_select, str);

	switch (ios->power_mode) {
	case MMC_POWER_OFF:
		str = "off";
		break;
	case MMC_POWER_UP:
		str = "up";
		break;
	case MMC_POWER_ON:
		str = "on";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "power mode:\t%u (%s)\n", ios->power_mode, str);
	seq_printf(s, "bus width:\t%u (%u bits)\n",
			ios->bus_width, 1 << ios->bus_width);

	switch (ios->timing) {
	case MMC_TIMING_LEGACY:
		str = "legacy";
		break;
	case MMC_TIMING_MMC_HS:
		str = "mmc high-speed";
		break;
	case MMC_TIMING_SD_HS:
		str = "sd high-speed";
		break;
	default:
		str = "invalid";
		break;
	}
	seq_printf(s, "timing spec:\t%u (%s)\n", ios->timing, str);

	return 0;
}

static int mmc_ios_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmc_ios_show, inode->i_private);
}

static const struct file_operations mmc_ios_fops = {
	.open		= mmc_ios_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#ifdef CONFIG_MMC_SDHCI
static int mmc_ops_usage(struct seq_file *s, void *data)
{
	printk(KERN_INFO "usage:\n"
			"\td: dump SDHCI registers\n");
	return 0;
}

static int mmc_ops_open(struct inode *inode, struct file *file)
{
	return single_open(file, mmc_ops_usage, inode->i_private);
}


static void mmc_recovery_dbg(struct mmc_host *host, int cd_force_status)
{
	struct sdhci_host *host1 = mmc_priv(host);
	struct sdhci_pxa *pxa = sdhci_priv(host1);
	if (pxa && pxa->pdata && pxa->pdata->cd_wakelock) {
		pxa->pdata->cd_force_status = cd_force_status;
		mmc_detect_change(host, 1);
		pxa->pdata->cd_wakelock(0);
		if (host->suspended) {
			do {
				msleep(1);
			} while(host->suspended);
			msleep(40);
		}
	}
}

static ssize_t mmc_ops(struct file *file,
			const char __user *buffer, size_t count, loff_t *ppos)
{
	struct seq_file *s = (struct seq_file *)file->private_data;
	struct mmc_host *host = s->private;

	if (!strncmp(buffer, "d", 1)) {
		printk(KERN_INFO "Dump SDHCI registers.\n");
		host->ops->dump_regs(host);
	} else if (!strncmp(buffer, "s1", 2)) {
	/* "s1" option should be active if SD-card used
	* whilst Android in suspned, for example - logging!
	* How to use:
  mkdir /data/temp
  mount -t debugfs debugfs /data/temp
              SAARB:mmc1 ALKON:mmc0
  echo s1 > /data/temp/mmc1/ops
  echo sd > /data/temp/mmc1/ops
	*/
		host->skip_suspend = 1;
		printk(KERN_INFO "MMC/SD: Force to skip suspend = yes. VLDO is always ON\n");
	} else if (!strncmp(buffer, "sdefault", 2)) {
		host->skip_suspend = 0;
		printk(KERN_INFO "MMC/SD: Force to skip suspend = NO/default\n");

	/* Statistic and Insert/Remove debug
  echo ss > /data/temp/mmc0/ops
  echo D > /data/temp/mmc0/ops
  echo R > /data/temp/mmc0/ops
  echo I > /data/temp/mmc0/ops
	*/
	} else if (!strncmp(buffer, "sstats", 2)) {
		extern u32 mmc_rq_in_suspend_stats;
		printk(KERN_INFO "MMC/SD statistic: requests under suspend = %u\n",
			mmc_rq_in_suspend_stats);
	} else if (!strncmp(buffer, "Delete", 1)) {
		mmc_recovery_dbg(host, -2);
	} else if (!strncmp(buffer, "Remove", 1)) {
		mmc_recovery_dbg(host, -1);
	} else if (!strncmp(buffer, "Insert", 1)) {
		mmc_recovery_dbg(host, 1);
	} else if (!strncmp(buffer, "CRCdn", 4)) {
/*
  echo CRCda > /data/temp/mmc1/ops
*/
		extern u32 sdhci_crc_error_data;
		sdhci_crc_error_data = buffer[4] - '0';
		printk(KERN_INFO "\n MMC/SD insert %d CRC Data errors\n", sdhci_crc_error_data);

	} else {
		printk(KERN_INFO "not supported\n");
		mmc_ops_usage(NULL, NULL);
	}

	return count;
}

static const struct file_operations mmc_ops_fops = {
	.open		= mmc_ops_open,
	.read		= seq_read,
	.write		= mmc_ops,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

void mmc_add_host_debugfs(struct mmc_host *host)
{
	struct dentry *root;

	root = debugfs_create_dir(mmc_hostname(host), NULL);
	if (IS_ERR(root))
		/* Don't complain -- debugfs just isn't enabled */
		return;
	if (!root)
		/* Complain -- debugfs is enabled, but it failed to
		 * create the directory. */
		goto err_root;

	host->debugfs_root = root;

	if (!debugfs_create_file("ios", S_IRUSR, root, host, &mmc_ios_fops))
		goto err_ios;

#ifdef CONFIG_MMC_SDHCI
	if (!debugfs_create_file("ops", S_IRUSR, root, host, &mmc_ops_fops))
		goto err_ios;
#endif

#ifdef CONFIG_MMC_CLKGATE
	if (!debugfs_create_u32("clk_delay", (S_IRUSR | S_IWUSR),
				root, &host->clk_delay))
		goto err_ios;
#endif
	return;

err_ios:
	debugfs_remove_recursive(root);
	host->debugfs_root = NULL;
err_root:
	dev_err(&host->class_dev, "failed to initialize debugfs\n");
}

void mmc_remove_host_debugfs(struct mmc_host *host)
{
	debugfs_remove_recursive(host->debugfs_root);
}

static int mmc_dbg_card_status_get(void *data, u64 *val)
{
	struct mmc_card	*card = data;
	u32		status;
	int		ret;

	mmc_claim_host(card->host);

	ret = mmc_send_status(data, &status);
	if (!ret)
		*val = status;

	mmc_release_host(card->host);

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(mmc_dbg_card_status_fops, mmc_dbg_card_status_get,
		NULL, "%08llx\n");

#define EXT_CSD_STR_LEN 1025

static int mmc_ext_csd_open(struct inode *inode, struct file *filp)
{
	struct mmc_card *card = inode->i_private;
	char *buf;
	ssize_t n = 0;
	u8 *ext_csd;
	int err, i;

	buf = kmalloc(EXT_CSD_STR_LEN + 1, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ext_csd = kmalloc(512, GFP_KERNEL);
	if (!ext_csd) {
		err = -ENOMEM;
		goto out_free;
	}

	mmc_claim_host(card->host);
	err = mmc_send_ext_csd(card, ext_csd);
	mmc_release_host(card->host);
	if (err)
		goto out_free;

	for (i = 511; i >= 0; i--)
		n += sprintf(buf + n, "%02x", ext_csd[i]);
	n += sprintf(buf + n, "\n");
	BUG_ON(n != EXT_CSD_STR_LEN);

	filp->private_data = buf;
	kfree(ext_csd);
	return 0;

out_free:
	kfree(buf);
	kfree(ext_csd);
	return err;
}

static ssize_t mmc_ext_csd_read(struct file *filp, char __user *ubuf,
				size_t cnt, loff_t *ppos)
{
	char *buf = filp->private_data;

	return simple_read_from_buffer(ubuf, cnt, ppos,
				       buf, EXT_CSD_STR_LEN);
}

static int mmc_ext_csd_release(struct inode *inode, struct file *file)
{
	kfree(file->private_data);
	return 0;
}

static const struct file_operations mmc_dbg_ext_csd_fops = {
	.open		= mmc_ext_csd_open,
	.read		= mmc_ext_csd_read,
	.release	= mmc_ext_csd_release,
};

void mmc_add_card_debugfs(struct mmc_card *card)
{
	struct mmc_host	*host = card->host;
	struct dentry	*root;

	if (!host->debugfs_root)
		return;

	root = debugfs_create_dir(mmc_card_id(card), host->debugfs_root);
	if (IS_ERR(root))
		/* Don't complain -- debugfs just isn't enabled */
		return;
	if (!root)
		/* Complain -- debugfs is enabled, but it failed to
		 * create the directory. */
		goto err;

	card->debugfs_root = root;

	if (!debugfs_create_x32("state", S_IRUSR, root, &card->state))
		goto err;

	if (mmc_card_mmc(card) || mmc_card_sd(card))
		if (!debugfs_create_file("status", S_IRUSR, root, card,
					&mmc_dbg_card_status_fops))
			goto err;

	if (mmc_card_mmc(card))
		if (!debugfs_create_file("ext_csd", S_IRUSR, root, card,
					&mmc_dbg_ext_csd_fops))
			goto err;

	return;

err:
	debugfs_remove_recursive(root);
	card->debugfs_root = NULL;
	dev_err(&card->dev, "failed to initialize debugfs\n");
}

void mmc_remove_card_debugfs(struct mmc_card *card)
{
	debugfs_remove_recursive(card->debugfs_root);
}
