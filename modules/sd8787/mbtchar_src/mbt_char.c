/** @file mbt_char.c
  *
  * @brief This file contains the char device function calls
  *
  * Copyright (C) 2010-2011, Marvell International Ltd. 
  * 
  * This software file (the "File") is distributed by Marvell International 
  * Ltd. under the terms of the GNU General Public License Version 2, June 1991 
  * (the "License").  You may use, redistribute and/or modify this File in 
  * accordance with the terms and conditions of the License, a copy of which 
  * is available by writing to the Free Software Foundation, Inc.,
  * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or on the
  * worldwide web at http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt.
  *
  * THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE 
  * IMPLIED WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE 
  * ARE EXPRESSLY DISCLAIMED.  The License provides additional details about 
  * this warranty disclaimer.
  *
  */
#include "mbt_char.h"

#define MODULE_NAME "mbtchar"
#include <linux/ioctl.h>
#define MBTCHAR_IOCTL_RELEASE       _IO('M', 1)
#define MBTCHAR_IOCTL_QUERY_TYPE    _IO('M', 2)

#define MAX_NUM_MBTCHAR         2

/** Char driver Major # */
static int mbtchar_major = MBTCHAR_MAJOR_NUM;
ulong drvdbg = 0x7;

static struct class *mbtchar_class;
LIST_HEAD(mbtchar_dev_list);
static DEFINE_SPINLOCK(mbtchar_dev_list_lock);

struct mbtchar_dev
{
    struct list_head list;
    int minor;
    struct cdev cdev;
    struct hci_dev *hdev;
};

/** 
 *  @brief write handler for char dev
 *  @param inode    pointer to structure inmode
 *  @param filp     pointer to structure file
 *  @return    number of bytes written
 */
ssize_t
mbtchar_write(struct file *filp, const char *buf, size_t count, loff_t * f_pos)
{
    int nwrite = 0;
    struct sk_buff *skb;
    struct hci_dev *hdev = NULL;
    struct mbtchar_dev *dev = (struct mbtchar_dev *) filp->private_data;
    ENTER();

    if (!dev || !dev->hdev) {
        LEAVE();
        return -ENXIO;
    }
    hdev = dev->hdev;
    nwrite = count;
    skb = bt_skb_alloc(count, GFP_ATOMIC);
    if (!skb) {
        PRINTM(MERROR, "mbtchar_write(): fail to alloc skb\n");
        LEAVE();
        return -ENOMEM;
    }

    if (copy_from_user((void *) skb_put(skb, count), buf, count)) {
        PRINTM(MERROR, "mbtchar_write(): cp_from_user failed\n");
        kfree_skb(skb);
        nwrite = -EFAULT;
        goto exit;
    }
    bt_cb(skb)->pkt_type = *((unsigned char *) skb->data);
    skb_pull(skb, 1);

    PRINTM(MDATA, "Write: pkt_type: 0x%x, len=%d @%lu\n", bt_cb(skb)->pkt_type,
           skb->len, jiffies);
    DBG_HEXDUMP(MDAT_D, "mbtchar_write", skb->data, skb->len);

    /* Send skb to the hci wrapper layer */
    if (hci_wrapper_send(hdev, skb)) {
        PRINTM(MERROR, "Write: Fail\n");
        nwrite = 0;
        /* Send failed */
        kfree_skb(skb);
    }
  exit:
    LEAVE();
    return nwrite;
}

/** 
 *  @brief read handler for char dev
 *  @param inode    pointer to structure inmode
 *  @param filp     pointer to structure file
 *  @return    number of bytes read
 */
ssize_t
mbtchar_read(struct file * filp, char *buf, size_t count, loff_t * f_pos)
{
    struct hci_dev *hdev = NULL;
    struct mbtchar_dev *dev = (struct mbtchar_dev *) filp->private_data;
    DECLARE_WAITQUEUE(wait, current);
    ssize_t ret = 0;
    struct sk_buff *skb = NULL;

    ENTER();
    if (!dev || !dev->hdev) {
        LEAVE();
        return -ENXIO;
    }
    hdev = dev->hdev;
    /* Wait for rx data */
    add_wait_queue(&hdev->req_wait_q, &wait);
    while (1) {
        set_current_state(TASK_INTERRUPTIBLE);
        skb = skb_dequeue(&hdev->rx_q);
        if (skb)
            break;
        if (!test_bit(HCI_UP, &hdev->flags)) {
            ret = -EBUSY;
            break;
        }
        if (filp->f_flags & O_NONBLOCK) {
            ret = -EAGAIN;
            break;
        }
        if (signal_pending(current)) {
            ret = -EINTR;
            break;
        }
        schedule();
    }
    set_current_state(TASK_RUNNING);
    remove_wait_queue(&hdev->req_wait_q, &wait);

    if (!skb)
        goto out;

    if (skb->len > count) {
        ret = -EOVERFLOW;
        goto outf;
    }
    /* Put type byte before the data */
    memcpy(skb_push(skb, 1), &bt_cb(skb)->pkt_type, 1);
    PRINTM(MDATA, "Read: pkt_type: 0x%x, len=%d @%lu\n", bt_cb(skb)->pkt_type,
           skb->len, jiffies);
    DBG_HEXDUMP(MDAT_D, "mbtchar_read", skb->data, skb->len);
    if (copy_to_user(buf, skb->data, skb->len)) {
        ret = -EFAULT;
        goto outf;
    }
    ret = skb->len;
  outf:
    kfree_skb(skb);
  out:
    LEAVE();
    return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
/** 
 *  @brief ioctl handler for char dev
 *  @param inode    pointer to structure inmode
 *  @param filp     pointer to structure file
 *  @return    0--success otherwise failure
 */
int
mbtchar_ioctl(struct inode *inode, struct file *filp,
              unsigned int cmd, unsigned long arg)
#else
/** 
 *  @brief ioctl handler for char dev
 *  @param filp     pointer to structure file
 *  @return    0--success otherwise failure
 */
long
mbtchar_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
#endif
{
    struct mbtchar_dev *dev = (struct mbtchar_dev *) filp->private_data;
    struct hci_dev *hdev = NULL;
    ENTER();
    if (!dev || !dev->hdev) {
        LEAVE();
        return -ENXIO;
    }
    hdev = dev->hdev;
    PRINTM(MINFO, "IOCTL: cmd=%d\n", cmd);
    switch (cmd) {
    case MBTCHAR_IOCTL_RELEASE:
        PRINTM(MINFO, "IOCTL_RELEASE: %d\n", hdev->id);
        hci_wrapper_close(hdev);
        break;
    case MBTCHAR_IOCTL_QUERY_TYPE:
        PRINTM(MINFO, "IOCTL_QUERY_TYPE: %d\n", hdev->type);
        if (copy_to_user((void *) arg, &hdev->type, sizeof(hdev->type)))
            PRINTM(MERROR, "IOCTL_QUERY_TYPE: Fail copy to user\n");
        break;
    default:
        break;
    }
    LEAVE();
    return 0;
}

/** 
 *  @brief open handler for char dev
 *  @param inode    pointer to structure inmode
 *  @param filp     pointer to structure file
 *  @return    0--success otherwise failure
 */
int
mbtchar_open(struct inode *inode, struct file *filp)
{
    int ret = 0;
    struct mbtchar_dev *dev = NULL;
    ENTER();

    dev = container_of(inode->i_cdev, struct mbtchar_dev, cdev);
    if (!dev->hdev) {
        LEAVE();
        return -ENXIO;
    }
    filp->private_data = dev;   /* for other methods */
    /* Call into hci wrapper */
    ret = hci_wrapper_open(dev->hdev);

    LEAVE();
    return ret;
}

/** 
 *  @brief release handler for char dev
 *  @param inode    pointer to structure inmode
 *  @param filp     pointer to structure file
 *  @return    0--success otherwise failure
 */
int
mbtchar_release(struct inode *inode, struct file *filp)
{
    int ret = 0;
    struct mbtchar_dev *dev = (struct mbtchar_dev *) filp->private_data;
    ENTER();
    if (!dev || !dev->hdev) {
        LEAVE();
        return -ENXIO;
    }
    ret = hci_wrapper_close(dev->hdev);
    filp->private_data = NULL;
    LEAVE();
    return ret;
}

/** 
 *  @brief poll handler for char dev
 *  @param filp     pointer to structure file
 *  @param wait     pointer to poll_table structure
 *  @return    mask
 */
static unsigned int
mbtchar_poll(struct file *filp, poll_table * wait)
{
    unsigned int mask;
    struct mbtchar_dev *dev = (struct mbtchar_dev *) filp->private_data;
    struct hci_dev *hdev = NULL;
    ENTER();
    if (!dev || !dev->hdev) {
        LEAVE();
        return -ENXIO;
    }
    hdev = dev->hdev;
    poll_wait(filp, &hdev->req_wait_q, wait);
    mask = POLLOUT | POLLWRNORM;
    if (skb_peek(&hdev->rx_q))
        mask |= POLLIN | POLLRDNORM;
    if (!test_bit(HCI_UP, &hdev->flags))
        mask |= POLLHUP;
    PRINTM(MINFO, "poll mask=0x%x\n", mask);
    LEAVE();
    return mask;
}

/* File ops for the Char driver */
struct file_operations mbtchar_fops = {
    .read = mbtchar_read,
    .write = mbtchar_write,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
    .ioctl = mbtchar_ioctl,
#else
    .unlocked_ioctl = mbtchar_ioctl,
#endif
    .open = mbtchar_open,
    .release = mbtchar_release,
    .poll = mbtchar_poll,
};

/** 
 *  @brief This function register hci_dev to mbtchar_dev
 *  @param dev 	pointer to structure hci_dev
 *  @return    0--success otherwise failure
 */
int
mbtchar_register_dev(struct hci_dev *hdev)
{
    unsigned long flags;
    struct list_head *p = NULL;
    struct mbtchar_dev *dev = NULL;

    ENTER();
    spin_lock_irqsave(&mbtchar_dev_list_lock, flags);
    list_for_each(p, &mbtchar_dev_list) {
        dev = list_entry(p, struct mbtchar_dev, list);
        if (dev->minor == hdev->id) {
            dev->hdev = hdev;
            break;
        }
    }
    spin_unlock_irqrestore(&mbtchar_dev_list_lock, flags);
    LEAVE();
    return 0;
}

/** 
 *  @brief This function unregister hci_dev to mbtchar_dev
 *  @param hdev  pointer to structure hci_dev
 *  @return    0--success otherwise failure
 */
int
mbtchar_unregister_dev(struct hci_dev *hdev)
{
    unsigned long flags;
    struct list_head *p = NULL;
    struct mbtchar_dev *dev = NULL;

    ENTER();
    spin_lock_irqsave(&mbtchar_dev_list_lock, flags);
    list_for_each(p, &mbtchar_dev_list) {
        dev = list_entry(p, struct mbtchar_dev, list);
        if (dev->minor == hdev->id) {
            dev->hdev = NULL;
            break;
        }
    }
    spin_unlock_irqrestore(&mbtchar_dev_list_lock, flags);
    LEAVE();
    return 0;
}

/** 
 *  @brief This function create the char dev
 *  @param dev 		a pointer to structure mbtchar_dev
 *  @return    0--success otherwise failure
 */
int
mbtchar_register_char_dev(struct mbtchar_dev *dev)
{
    int ret = 0, dev_num;
    unsigned long flags;
    ENTER();
    /* create the chrdev region */
    if (mbtchar_major) {
        dev_num = MKDEV(mbtchar_major, dev->minor);
        ret = register_chrdev_region(dev_num, 1, MODULE_NAME);
    } else {
        PRINTM(MINFO, "mbtchar: no major # yet\n");
        ret = alloc_chrdev_region(&dev_num, dev->minor, 1, MODULE_NAME);
    }

    if (ret) {
        PRINTM(MERROR, "mbtchar: create chrdev_region failed\n");
        LEAVE();
        return ret;
    }

    if (!mbtchar_major) {
        /* Store the allocated dev major # */
        mbtchar_major = MAJOR(dev_num);
    }
    cdev_init(&dev->cdev, &mbtchar_fops);
    dev->cdev.owner = THIS_MODULE;
    dev->cdev.ops = &mbtchar_fops;
    dev_num = MKDEV(mbtchar_major, dev->minor);

    if (cdev_add(&dev->cdev, dev_num, 1)) {
        PRINTM(MERROR, "mbtchar: cdev_add failed\n");
        ret = -EFAULT;
        goto free_cdev_region;
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
    device_create(mbtchar_class, NULL, MKDEV(mbtchar_major, dev->minor), NULL,
                  "mbtchar%d", dev->minor);
#else
    device_create(mbtchar_class, NULL, MKDEV(mbtchar_major, dev->minor),
                  "mbtchar%d", dev->minor);
#endif
    PRINTM(MINFO, "regier char dev=%d\n", dev->minor);

    spin_lock_irqsave(&mbtchar_dev_list_lock, flags);
    list_add_tail(&dev->list, &mbtchar_dev_list);
    spin_unlock_irqrestore(&mbtchar_dev_list_lock, flags);

    LEAVE();
    return ret;
  free_cdev_region:
    unregister_chrdev_region(MKDEV(mbtchar_major, dev->minor), 1);
    LEAVE();
    return ret;

}

/** 
 *  @brief This function delete the char dev
 *  @param dev	 a pointer to structure mbtchar_dev
 *  @return    0--success otherwise failure
 */
int
mbtchar_unregister_char_dev(struct mbtchar_dev *dev)
{
    ENTER();
    device_destroy(mbtchar_class, MKDEV(mbtchar_major, dev->minor));
    cdev_del(&dev->cdev);
    unregister_chrdev_region(MKDEV(mbtchar_major, dev->minor), 1);
    PRINTM(MINFO, "unregier char dev=%d\n", dev->minor);

    LEAVE();
    return 0;
}

/** 
 *  @brief This function initializes module.
 *  
 *  @return        MLAN_STATUS_SUCCESS or MLAN_STATUS_FAILURE
 */
static int
mbtchar_init(void)
{
    int i = 0;
    struct mbtchar_dev *dev = NULL;
    ENTER();

    mbtchar_class = class_create(THIS_MODULE, MODULE_NAME);
    if (IS_ERR(mbtchar_class)) {
        PRINTM(MERROR, "Unable to allocat class\n");
        LEAVE();
        return PTR_ERR(mbtchar_class);
    }
    for (i = 0; i < MAX_NUM_MBTCHAR; i++) {
        dev = kzalloc(sizeof(struct mbtchar_dev), GFP_KERNEL);
        if (!dev) {

            class_destroy(mbtchar_class);
            return -ENOMEM;
        }
        dev->minor = i;
        dev->hdev = NULL;
        mbtchar_register_char_dev(dev);
    }
    LEAVE();
    return 0;
}

/** 
 *  @brief This function cleans module
 *  
 *  @return        N/A
 */
static void
mbtchar_cleanup(void)
{
    unsigned long flags;
    struct list_head *p = NULL;
    struct mbtchar_dev *dev = NULL;
    ENTER();
    spin_lock_irqsave(&mbtchar_dev_list_lock, flags);
    do {
        dev = NULL;
        list_for_each(p, &mbtchar_dev_list) {
            dev = list_entry(p, struct mbtchar_dev, list);
            list_del(p);
            spin_unlock_irqrestore(&mbtchar_dev_list_lock, flags);
            mbtchar_unregister_char_dev(dev);
            kfree(dev);
            spin_lock_irqsave(&mbtchar_dev_list_lock, flags);
            break;
        }
    } while (dev);
    spin_unlock_irqrestore(&mbtchar_dev_list_lock, flags);
    class_destroy(mbtchar_class);
    LEAVE();
}

module_init(mbtchar_init);
module_exit(mbtchar_cleanup);
MODULE_LICENSE("GPL");
module_param(drvdbg, ulong, 0);
MODULE_PARM_DESC(drvdbg, "Driver debug");
MODULE_VERSION(DRIVER_VERSION);
MODULE_AUTHOR("Marvell International Ltd.");
MODULE_DESCRIPTION("BT CHAR Driver");
