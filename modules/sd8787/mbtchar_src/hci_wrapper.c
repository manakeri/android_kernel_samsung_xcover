/*
   BlueZ - Bluetooth protocol stack for Linux
   Copyright (C) 2000-2001 Qualcomm Incorporated
   Copyright (C) 2010-2011, Marvell International Ltd.

   Written 2000,2001 by Maxim Krasnyansky <maxk@qualcomm.com>

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License version 2 as
   published by the Free Software Foundation;

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
   OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT OF THIRD PARTY RIGHTS.
   IN NO EVENT SHALL THE COPYRIGHT HOLDER(S) AND AUTHOR(S) BE LIABLE FOR ANY
   CLAIM, OR ANY SPECIAL INDIRECT OR CONSEQUENTIAL DAMAGES, OR ANY DAMAGES
   WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
   ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
   OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

   ALL LIABILITY, INCLUDING LIABILITY FOR INFRINGEMENT OF ANY PATENTS,
   COPYRIGHTS, TRADEMARKS OR OTHER RIGHTS, RELATING TO USE OF THIS
   SOFTWARE IS DISCLAIMED.
*/

#include "mbt_char.h"

/* HCI device list */
LIST_HEAD(hci_dev_list);
static DEFINE_SPINLOCK(hci_dev_list_lock);

/** 
 *  @brief open hci device 
 *
 *  @return 0 - success, otherwise fail    
 */
int
hci_wrapper_open(struct hci_dev *hdev)
{
    int ret = 0;
    ENTER();
    PRINTM(MINFO, "open:%s %p", hdev->name, hdev);
    hci_req_lock(hdev);
    if (test_bit(HCI_UP, &hdev->flags)) {
        ret = -EALREADY;
        goto done;
    }
    if (hdev->open(hdev)) {
        ret = -EIO;
        goto done;
    }
    set_bit(HCI_UP, &hdev->flags);
  done:
    hci_req_unlock(hdev);
    return ret;
}

/** 
 *  @brief close hci device 
 *
 *  @return 0   
 */
int
hci_wrapper_close(struct hci_dev *hdev)
{
    ENTER();

    hci_req_lock(hdev);

    if (!test_and_clear_bit(HCI_UP, &hdev->flags)) {
        hci_req_unlock(hdev);
        LEAVE();
        return 0;
    }

    if (hdev->flush)
        hdev->flush(hdev);

    /* wait up pending read and unregister char dev */
    wake_up_interruptible(&hdev->req_wait_q);
    /* Drop queues */
    skb_queue_purge(&hdev->rx_q);

    /* After this point our queues are empty and no tasks are scheduled. */
    hdev->close(hdev);

    /* Clear flags */
    hdev->flags = 0;

    hci_req_unlock(hdev);

    LEAVE();
    return 0;
}

/** 
 *  @brief send data to hci device
 *
 *  @return 0 - success, otherwise fail    
 */
int
hci_wrapper_send(struct hci_dev *hdev, struct sk_buff *skb)
{
    int ret;
    ENTER();

    skb->dev = (void *) hdev;
    ret = hdev->send(skb);

    LEAVE();
    return ret;
}

/** 
 *  @brief Alloc HCI device 
 *
 *  @return    point to structure hci_dev or NULL
 */
struct hci_dev *
mbt_hci_alloc_dev(void)
{
    struct hci_dev *hdev;
    ENTER();

    hdev = kzalloc(sizeof(struct hci_dev), GFP_KERNEL);
    if (!hdev) {
        LEAVE();
        return NULL;
    }

    LEAVE();
    return hdev;
}

EXPORT_SYMBOL(mbt_hci_alloc_dev);

/** 
 *  @brief Free HCI device 
 *  @param hdev   pointer to structure hci_dev
 *  @return    N/A
 */
void
mbt_hci_free_dev(struct hci_dev *hdev)
{
    ENTER();
    kfree(hdev);
    LEAVE();
}

EXPORT_SYMBOL(mbt_hci_free_dev);

/** 
 *  @brief Register HCI device
 *  @param hdev   pointer to structure hci_dev
 *  @return    0
 */
int
mbt_hci_register_dev(struct hci_dev *hdev)
{
    unsigned char id = 0;
    int i;
    unsigned long flags;
    struct list_head *head = &hci_dev_list, *p;

    ENTER();

    PRINTM(MINFO, "register dev: %p name %s type %d owner %p\n", hdev,
           hdev->name, hdev->type, hdev->owner);

    if (!hdev->open || !hdev->close || !hdev->destruct) {
        PRINTM(MERROR, "hciwrap: hdev not init'ed ok\n");
        LEAVE();
        return -EINVAL;
    }

    spin_lock_irqsave(&hci_dev_list_lock, flags);

    /* Find first available device id */
    list_for_each(p, &hci_dev_list) {
        if (list_entry(p, struct hci_dev, list)->id != id)
              break;
        head = p;
        id++;
    }

    snprintf(hdev->name, sizeof(hdev->name), "hci%d", id);
    hdev->id = id;
    list_add(&hdev->list, head);

    atomic_set(&hdev->refcnt, 1);
    spin_lock_init(&hdev->lock);

    hdev->flags = 0;
    hdev->pkt_type = (HCI_DM1 | HCI_DH1 | HCI_HV1);
    hdev->esco_type = (ESCO_HV1);
    hdev->link_mode = (HCI_LM_ACCEPT);

    hdev->idle_timeout = 0;
    hdev->sniff_max_interval = 800;
    hdev->sniff_min_interval = 80;

    skb_queue_head_init(&hdev->rx_q);

    for (i = 0; i < 3; i++)
        hdev->reassembly[i] = NULL;

    init_waitqueue_head(&hdev->req_wait_q);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
    init_MUTEX(&hdev->req_lock);
#else
    sema_init(&hdev->req_lock, 1);
#endif

    memset(&hdev->stat, 0, sizeof(struct hci_dev_stats));

    atomic_set(&hdev->promisc, 0);

    spin_unlock_irqrestore(&hci_dev_list_lock, flags);
    if (mbtchar_register_dev(hdev)) {
        id = -EFAULT;
        goto err_btchar_register;
    }
    PRINTM(MINFO, "register dev: success\n");
    LEAVE();
    return id;

  err_btchar_register:
    list_del(&hdev->list);
    LEAVE();
    return id;
}

EXPORT_SYMBOL(mbt_hci_register_dev);

/** 
 *  @brief Unregister HCI device
 *  @param hdev   pointer to structure hci_dev
 *  @return    0
 */
int
mbt_hci_unregister_dev(struct hci_dev *hdev)
{
    unsigned long flags;
    int i;

    ENTER();

    PRINTM(MINFO, "unregiser_dev:%p name %s type %d", hdev, hdev->name,
           hdev->type);

    spin_lock_irqsave(&hci_dev_list_lock, flags);
    list_del(&hdev->list);
    spin_unlock_irqrestore(&hci_dev_list_lock, flags);

    hci_wrapper_close(hdev);

    for (i = 0; i < 3; i++)
        kfree_skb(hdev->reassembly[i]);
    mbtchar_unregister_dev(hdev);

    LEAVE();
    return 0;
}

EXPORT_SYMBOL(mbt_hci_unregister_dev);

/** 
 *  @brief suspend HCI device
 *  @param hdev   pointer to structure hci_dev
 *  @return    0
 */
int
mbt_hci_suspend_dev(struct hci_dev *hdev)
{
    return 0;
}

EXPORT_SYMBOL(mbt_hci_suspend_dev);

/** 
 *  @brief Resume HCI device
 *  @param hdev   pointer to structure hci_dev
 *  @return    0
 */
int
mbt_hci_resume_dev(struct hci_dev *hdev)
{
    return 0;
}

EXPORT_SYMBOL(mbt_hci_resume_dev);

/* Receive packet type fragment */
#define __reassembly(hdev, type)  ((hdev)->reassembly[(type) - 2])

/** 
 *  @brief Receive packet type fragment
 *  @param hdev   pointer to structure hci_dev
 *  @param data   pointer to data buffer
 *  @param count  number of byte
 *  @return    0
 */
int
mbt_hci_recv_fragment(struct hci_dev *hdev, int type, void *data, int count)
{
    if (type < HCI_ACLDATA_PKT || type > HCI_EVENT_PKT)
        return -EILSEQ;

    while (count) {
        struct sk_buff *skb = __reassembly(hdev, type);
        struct
        {
            int expect;
        } *scb;
        int len = 0;

        if (!skb) {
            /* Start of the frame */

            switch (type) {
            case HCI_EVENT_PKT:
                if (count >= HCI_EVENT_HDR_SIZE) {
                    struct hci_event_hdr *h = data;
                    len = HCI_EVENT_HDR_SIZE + h->plen;
                } else
                    return -EILSEQ;
                break;

            case HCI_ACLDATA_PKT:
                if (count >= HCI_ACL_HDR_SIZE) {
                    struct hci_acl_hdr *h = data;
                    len = HCI_ACL_HDR_SIZE + __le16_to_cpu(h->dlen);
                } else
                    return -EILSEQ;
                break;

            case HCI_SCODATA_PKT:
                if (count >= HCI_SCO_HDR_SIZE) {
                    struct hci_sco_hdr *h = data;
                    len = HCI_SCO_HDR_SIZE + h->dlen;
                } else
                    return -EILSEQ;
                break;
            }

            skb = bt_skb_alloc(len, GFP_ATOMIC);
            if (!skb) {
                BT_ERR("%s no memory for packet", hdev->name);
                return -ENOMEM;
            }

            skb->dev = (void *) hdev;
            bt_cb(skb)->pkt_type = type;

            __reassembly(hdev, type) = skb;

            scb = (void *) skb->cb;
            scb->expect = len;
        } else {
            /* Continuation */

            scb = (void *) skb->cb;
            len = scb->expect;
        }

        len = min(len, count);

        memcpy(skb_put(skb, len), data, len);

        scb->expect -= len;

        if (scb->expect == 0) {
            /* Complete frame */

            __reassembly(hdev, type) = NULL;

            bt_cb(skb)->pkt_type = type;
            hci_recv_frame(skb);
        }

        count -= len;
        data += len;
    }

    return 0;
}

EXPORT_SYMBOL(mbt_hci_recv_fragment);
