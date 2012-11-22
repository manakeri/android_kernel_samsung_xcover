/*
 * (C) Copyright 2006-2011 Marvell International Ltd.
 * All Rights Reserved
 */

#include <linux/version.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/device.h>
#include <asm/uaccess.h>

#include <linux/tty_ldisc.h>
//#include <common_datastub.h>


#define DRIVER_VERSION "v1.0"
#define DRIVER_AUTHOR "Johnson Wu"
#define DRIVER_DESC "CI Data TTY driver"

/* Module information */
MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION( DRIVER_DESC );
MODULE_LICENSE("GPL");

int cctdatadev_open(struct inode *inode, struct file *filp);
int cctdatadev_ioctl(struct inode *inode, struct file *filp,
		unsigned int cmd, unsigned long arg);
static int cidatatty_open(struct tty_struct *tty, struct file *file);
static void cidatatty_close(struct tty_struct *tty, struct file *file);
static int cidatatty_write(struct tty_struct * tty,	const unsigned char *buffer, int count);
static void cidatatty_set_termios(struct tty_struct *tty, struct ktermios *old_termios);
static int cidatatty_write_room(struct tty_struct *tty);
static int cidatatty_ioctl(struct tty_struct *tty, struct file *file,
		unsigned int cmd, unsigned long arg);
static int cidatatty_chars_in_buffer(struct tty_struct *tty);
static void cidatatty_throttle(struct tty_struct *tty);
static void cidatatty_unthrottle(struct tty_struct *tty);
static void cidatatty_flush_buffer(struct tty_struct *tty);
int cctdatadev_init_module(void);
void cctdatadev_cleanup_module(void);

//from common_datastub.h -- start

typedef enum 
{
	SVC_TYPE_CSD 				= 	0,
	SVC_TYPE_PDP_DIRECTIP		=	1,
	SVC_TYPE_PDP_PPP_MODEM		=	2,
	SVC_TYPE_PDP_BT_PPP_MODEM	=	3,
	SVC_TYPE_NULL				=	4 , // used for GCF Testing
	SVC_TYPES_MAX
}SVCTYPE;

#define CSD_CALL_ID_OFFSET		20

//from common_datastub.h -- end

typedef void (*DataRxCallbackFunc)(char* packet, int len,unsigned char cid);
extern int registerRxCallBack(SVCTYPE pdpTye, DataRxCallbackFunc callback);
extern int unregisterRxCallBack(SVCTYPE pdpTye);
extern void sendDataReqtoInternalList(unsigned char cid, char* buf, int len);
extern int search_handlelist_for_csd_cid(void);


DataRxCallbackFunc prevRxCb=NULL;;

/* Our fake UART values */
#define MCR_DTR		0x01
#define MCR_RTS		0x02
#define MCR_LOOP	0x04
#define MSR_CTS		0x08
#define MSR_CD		0x10
#define MSR_RI		0x20
#define MSR_DSR		0x40
#define RELEVANT_IFLAG(iflag) ((iflag) & (IGNBRK|BRKINT|IGNPAR|PARMRK|INPCK))

#define CIDATATTY_TTY_MAJOR		247	/* experimental range */
#define CIDATATTY_TTY_MINORS	1		
#define CCTDATADEV_MAJOR  248
#define CCTDATADEV_NR_DEVS 1
int cctdatadev_major =   CCTDATADEV_MAJOR;
int cctdatadev_minor =   0;
int cctdatadev_nr_devs = CCTDATADEV_NR_DEVS;

#define TIOENABLE	_IOW('T',206, int)  /* enable data */
#define TIODISABLE	_IOW('T',207, int)  /* disable data */
#define TIOPPPON _IOW('T',208, int)
#define TIOPPPOFF _IOW('T', 209, int)
/*
 *  The following Macro is used for debugging purpose
 */

//#define CIDATATTY_DEBUG
//#define DEBUG_BUF_CONTENT  /* define this to see the buffer content */

#undef PDEBUG             /* undef it, just in case */
#ifdef CIDATATTY_DEBUG
	#define PDEBUG(fmt, args...) printk( "CIDATATTY: "fmt,## args)
	#define F_ENTER() printk("CIDATATTY: ENTER %s\n",__FUNCTION__)
	#define F_LEAVE() printk("CIDATATTY: LEAVE %s\n",__FUNCTION__)
#else
	#define PDEBUG(fmt, args...) do{}while(0)
	#define	F_ENTER()	do{}while(0)
	#define	F_LEAVE()	do{}while(0)
#endif

struct cidatatty_serial {
	struct tty_struct	*tty;		/* pointer to the tty for this device */
	int			open_count;	/* number of times this port has been opened */
	struct semaphore	sem;		/* locks this structure */

	/* for tiocmget and tiocmset functions */
	int			msr;		/* MSR shadow */
	int			mcr;		/* MCR shadow */

	/* for ioctl fun */
	struct serial_struct	serial;
	wait_queue_head_t	wait;
	struct async_icount	icount;
	unsigned char cid;
	int writeokflag;
	int devinuse;
};
struct cctdatadev_dev {
	int nreaders, nwriters;      
	struct cdev cdev;                  /* Char device structure */
};
struct file_operations cctdatadev_fops = {
	.owner =    THIS_MODULE,
	.ioctl =    cctdatadev_ioctl,
	.open =     cctdatadev_open,
	.llseek =	no_llseek,	
};
static struct tty_operations serial_ops = {
	.open = cidatatty_open,
	.close = cidatatty_close,
	.write = cidatatty_write,
	.set_termios = cidatatty_set_termios,
	.write_room		= cidatatty_write_room,
//	.tiocmget = cidatatty_tiocmget,
//	.tiocmset = cidatatty_tiocmset,
	.ioctl = cidatatty_ioctl,
	.chars_in_buffer = cidatatty_chars_in_buffer,  
//	.flush_chars = cidatatty_flush_chars,          
//	.wait_until_sent = cidatatty_wait_until_sent,  
	.throttle = cidatatty_throttle,                
	.unthrottle = cidatatty_unthrottle,            
//	.stop = cidatatty_stop,
//	.start = cidatatty_start,
//	.hangup = cidatatty_hangup,
	.flush_buffer = cidatatty_flush_buffer,
//	.set_ldisc = cidatatty_set_ldisc,
};

static struct tty_driver *cidatatty_tty_driver;
static struct cidatatty_serial *cidatatty_table[CIDATATTY_TTY_MINORS];	/* initially all NULL */
struct cctdatadev_dev *cctdatadev_devices;
int ttycurindex=0xff;
unsigned char currcid=0xff;
/*
 * Prototypes for shared functions
 */
int data_rx_csd(char* packet, int len, unsigned char cid)
{
	struct tty_struct *tty = NULL;
	struct cidatatty_serial *cidatatty;
	int remain = len;
	int i;
	
	int	c;

	if (cid>=CSD_CALL_ID_OFFSET)
		cid-=CSD_CALL_ID_OFFSET;

	if (cid!=currcid && currcid!=0) {
		//currcid == 0 is ok call_id wasn't passed on PPP ON, but we support only one CSD at a time
		// so we don't care...
		printk("data_rx_csd currcid %d  != %d\n",currcid,cid);
		return 0;
	}

	if (ttycurindex==0xff || ttycurindex>=CIDATATTY_TTY_MINORS || cidatatty_table[ttycurindex] == NULL) {
		printk("data_rx_csd wrong index %d or NULL\n",ttycurindex);
		return 0;
	}

	if (packet==NULL) {
		printk("data_rx_csd NULL packet arrived index %d\n",ttycurindex);
		return 0;
	}

	if (len<=0) {
		printk("data_rx_csd wrong len arrived index %d len=%d\n",ttycurindex,len);
		return 0;
	}

	tty = cidatatty_table[ttycurindex]->tty;
/*
	for(i=0;i<CIDATATTY_TTY_MINORS;i++)
	{
		if((cidatatty_table[i]->cid == cid)&&
			(cidatatty_table[i]->devinuse == 1))
		{
			tty = cidatatty_table[i]->tty;
			PDEBUG("data_rx_csd: i=%d\n",i);
			break;
		}
	}
	if(i>=CIDATATTY_TTY_MINORS) PDEBUG( "data_rx_csd: not found tty device.\n " );
*/


	if (!tty) {
		printk("tty is NULL\n");
		return 0;
	} 

	

	cidatatty = tty->driver_data;
	/* drop packets if nobody uses cidatatty */
	if (cidatatty == NULL || cidatatty->open_count == 0) {
		printk("cidatatty is NULL or closed\n");
		return 0;
	}
	
	down(&cidatatty->sem);
	
	while(remain > 0){
		PDEBUG( "data_rx_csd: receive_room=0x%x\n ",tty->receive_room );
		c = tty->receive_room;
	

		/* give up if no room at all */
		if (c == 0)
			break;

		if (c > len)
			c = len;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 32)
		tty->ldisc->ops->receive_buf(tty, packet, NULL, c);
#else
		tty->ldisc.ops->receive_buf(tty, packet, NULL, c);
#endif
		wake_up_interruptible(&tty->read_wait);

		remain -= c;
		packet += c;
	}


	up(&cidatatty->sem);
	/* return the actual bytes got */
	return len - remain;
}

static int cidatatty_open(struct tty_struct *tty, struct file *file)
{
	struct cidatatty_serial *cidatatty;
	int index;

	F_ENTER();
	/* initialize the pointer in case something fails */
	tty->driver_data = NULL;

	/* get the serial object associated with this tty pointer */
	index = tty->index;
	ttycurindex = index;

	cidatatty = cidatatty_table[index];
	
	if (cidatatty == NULL) {
		/* first time accessing this device, let's create it */
		cidatatty = kmalloc(sizeof(struct cidatatty_serial), GFP_KERNEL);
		if (!cidatatty)
			return -ENOMEM;

		init_MUTEX(&cidatatty->sem);
		cidatatty->open_count = 0;
		

		cidatatty_table[index] = cidatatty;
	}
	
	down(&cidatatty->sem);

	/* save our structure within the tty structure */
	tty->driver_data = cidatatty;


	/* jzwu1 */
	tty->flags = TTY_NO_WRITE_SPLIT | tty->flags;

	cidatatty->tty = tty;

	++cidatatty->open_count;

	if (cidatatty->open_count == 1) {
		/* this is the first time this port is opened */
		/* do any hardware initialization needed here */
		prevRxCb=(DataRxCallbackFunc)registerRxCallBack(SVC_TYPE_CSD,data_rx_csd);
	}

	cidatatty_table[index]->writeokflag = 1;
	cidatatty_table[index]->devinuse = 1;
	cidatatty_table[index]->cid = currcid;


	up(&cidatatty->sem);

	F_LEAVE();
	return 0;
}

static void do_close(struct cidatatty_serial *cidatatty)
{
	F_ENTER();

	down(&cidatatty->sem);

	if (!cidatatty->open_count) {
		/* port was never opened */
		goto exit;
	}

	--cidatatty->open_count;

	if (cidatatty->open_count <= 0) {
		/* The port is being closed by the last user. */
		/* Do any hardware specific stuff here */
		registerRxCallBack(SVC_TYPE_CSD,prevRxCb);
	}

exit:
	up(&cidatatty->sem);

	PDEBUG( "Leaving do_close: " );
}

static void cidatatty_close(struct tty_struct *tty, struct file *file)
{
	struct cidatatty_serial *cidatatty = tty->driver_data;
	F_ENTER();


	printk("cidatatty_close %x\n",cidatatty);
	if (cidatatty){
		printk("close() cidatatty->open_count=%d\n",cidatatty->open_count);
		do_close(cidatatty);
	}

	F_LEAVE();
}	

static int cidatatty_write(struct tty_struct * tty,	const unsigned char *buffer, int count)
{
	struct cidatatty_serial *cidatatty = tty->driver_data;
	int retval = -EINVAL;
	/*int i;*/

	F_ENTER();
	/* for some reason, this function is called with count == 0 */
	if( count <= 0 )
	{
		PDEBUG("Error: count is %d.\n", count);
		return 0;
	}
	if (!cidatatty)
	{
		PDEBUG("Warning: cidatatty_write: cidatatty is NULL\n");	
		return -ENODEV;
	}

	down(&cidatatty->sem);

	if (!cidatatty->open_count)
	{
		printk("Error: cidatatty_write: port was not open\n");		    
		/* port was not opened */
		goto exit;
	}

#ifdef DEBUG_BUF_CONTENT
	//int i;
	printk("CIDATATTY Tx Buffer datalen is %d\n data:",count);
	for (i=0 ; i<count; i++)   //i=14?
		printk("%02x(%c)", buffer[i]&0xff, buffer[i]&0xff );
	//    printk(" %02x", buffer[i]&0xff );
	printk("\n");
#endif  

	if(cidatatty_table[tty->index]->writeokflag == 1)
	{
		if (currcid==0){
			int tmpcid;
			tmpcid=search_handlelist_for_csd_cid();
			if (tmpcid>CSD_CALL_ID_OFFSET){
				currcid=tmpcid-CSD_CALL_ID_OFFSET;
			} 
		} 

		if (currcid && currcid!=0xff) {

			int len;
			int offset=0;
			int total=count;
			// CP want CSD data to be in blocks of 160 bytes 
			// so if we got more split this to several writes to CP
			while (total>0) {
				len= (total>=160 ? 160 : total);
				sendDataReqtoInternalList(currcid+CSD_CALL_ID_OFFSET, buffer+offset, len);
				total-=len;
				offset+=len;
			}
		} else {
			if (printk_ratelimit())
				printk("No CSD cid in kernel curr=%d\n",currcid);
		}
		
	} else {
		PDEBUG("write failed write nok index %d",tty->index);
	}

exit:
	up(&cidatatty->sem);

	F_LEAVE();
	retval=count;

	return retval;
}


static void cidatatty_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
{
	unsigned int cflag;

	F_ENTER();
	cflag = tty->termios->c_cflag;

	/* check that they really want us to change something */
	if (old_termios) {
		if ((cflag == old_termios->c_cflag) &&
				(RELEVANT_IFLAG(tty->termios->c_iflag) == 
				 RELEVANT_IFLAG(old_termios->c_iflag))) {
			PDEBUG(" - nothing to change...\n");
			return;
		}
	}

	/* get the byte size */
	switch (cflag & CSIZE) {
		case CS5:
			PDEBUG( " - data bits = 5\n");
			break;
		case CS6:
			PDEBUG( " - data bits = 6\n");
			break;
		case CS7:
			PDEBUG( " - data bits = 7\n");
			break;
		default:
		case CS8:
			PDEBUG( " - data bits = 8\n");
			break;
	}

	/* determine the parity */
	if (cflag & PARENB)
		if (cflag & PARODD)
			PDEBUG( " - parity = odd\n");
		else
			PDEBUG( " - parity = even\n");
	else
		PDEBUG( " - parity = none\n");

	/* figure out the stop bits requested */
	if (cflag & CSTOPB)
		PDEBUG( " - stop bits = 2\n");
	else
		PDEBUG( " - stop bits = 1\n");

	/* figure out the hardware flow control settings */
	if (cflag & CRTSCTS)
		PDEBUG( " - RTS/CTS is enabled\n");
	else
		PDEBUG( " - RTS/CTS is disabled\n");

	/* determine software flow control */
	/* if we are implementing XON/XOFF, set the start and 
	 * stop character in the device */
	if (I_IXOFF(tty) || I_IXON(tty)) {
		// CHECKPOINT
	}

	/* get the baud rate wanted */
	PDEBUG( " - baud rate = %d", tty_get_baud_rate(tty));

	F_LEAVE();
}

static int cidatatty_write_room(struct tty_struct *tty)
{
	/*cidatatty no need to support this function, and add this stub to avoid tty system complain no write_room function*/
	return 2048;
}

static int cidatatty_tiocmget(struct tty_struct *tty, struct file *file)
{
	struct cidatatty_serial *cidatatty = tty->driver_data;
	unsigned int result = 0;
	unsigned int msr = cidatatty->msr;
	unsigned int mcr = cidatatty->mcr;
	F_ENTER();

	result = ((mcr & MCR_DTR)  ? TIOCM_DTR  : 0) |	/* DTR is set */
		((mcr & MCR_RTS)  ? TIOCM_RTS  : 0) |	/* RTS is set */
		((mcr & MCR_LOOP) ? TIOCM_LOOP : 0) |	/* LOOP is set */
		((msr & MSR_CTS)  ? TIOCM_CTS  : 0) |	/* CTS is set */
		((msr & MSR_CD)   ? TIOCM_CAR  : 0) |	/* Carrier detect is set*/
		((msr & MSR_RI)   ? TIOCM_RI   : 0) |	/* Ring Indicator is set */
		((msr & MSR_DSR)  ? TIOCM_DSR  : 0);	/* DSR is set */

	F_LEAVE();
	return result;
}

static int cidatatty_tiocmset(struct tty_struct *tty, struct file *file,
		unsigned int set, unsigned int clear)
{
	struct cidatatty_serial *cidatatty = tty->driver_data;
	unsigned int mcr = cidatatty->mcr;

	F_ENTER();
	if (set & TIOCM_RTS)
		mcr |= MCR_RTS;
	if (set & TIOCM_DTR)
		mcr |= MCR_RTS;
	if (set & TIOCM_LOOP)
		mcr |= MCR_RTS;
	if (set & TIOCM_CTS)
		mcr |= MCR_RTS;
	if (set & TIOCM_RI)
		mcr |= MCR_RTS;
	if (set & TIOCM_DSR)
		mcr |= MCR_RTS;
	if (set & TIOCM_CAR)
		mcr |= MCR_RTS;

	if (clear & TIOCM_RTS)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_DTR)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_LOOP)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_CTS)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_CAR)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_RI)
		mcr &= ~MCR_RTS;
	if (clear & TIOCM_DSR)
		mcr &= ~MCR_RTS;

	/* set the new MCR value in the device */
	cidatatty->mcr = mcr;

	F_LEAVE();
	return 0;
}

static int cidatatty_ioctl_tiocgserial(struct tty_struct *tty, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct cidatatty_serial *cidatatty = tty->driver_data;
	F_ENTER();

	if (cmd == TIOCGSERIAL) {
		struct serial_struct tmp;

		if (!arg)
			return -EFAULT;

		memset(&tmp, 0, sizeof(tmp));

		tmp.type		= cidatatty->serial.type;
		tmp.line		= cidatatty->serial.line;
		tmp.port		= cidatatty->serial.port;
		tmp.irq			= cidatatty->serial.irq;
		tmp.flags		= ASYNC_SKIP_TEST | ASYNC_AUTO_IRQ;
		tmp.xmit_fifo_size	= cidatatty->serial.xmit_fifo_size;
		tmp.baud_base		= cidatatty->serial.baud_base;
		tmp.close_delay		= 5*HZ;
		tmp.closing_wait	= 30*HZ;
		tmp.custom_divisor	= cidatatty->serial.custom_divisor;
		tmp.hub6		= cidatatty->serial.hub6;
		tmp.io_type		= cidatatty->serial.io_type;

		if (copy_to_user((void __user *)arg, &tmp, sizeof(struct serial_struct)))
			return -EFAULT;
		return 0;
	}

	F_LEAVE();
	return -ENOIOCTLCMD;
}

static int cidatatty_ioctl_tiocmiwait(struct tty_struct *tty, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct cidatatty_serial *cidatatty = tty->driver_data;
	F_ENTER();

	if (cmd == TIOCMIWAIT) {
		DECLARE_WAITQUEUE(wait, current);
		struct async_icount cnow;
		struct async_icount cprev;

		cprev = cidatatty->icount;
		while (1) {
			add_wait_queue(&cidatatty->wait, &wait);
			set_current_state(TASK_INTERRUPTIBLE);
			schedule();
			remove_wait_queue(&cidatatty->wait, &wait);

			/* see if a signal woke us up */
			if (signal_pending(current))
				return -ERESTARTSYS;

			cnow = cidatatty->icount;
			if (cnow.rng == cprev.rng && cnow.dsr == cprev.dsr &&
					cnow.dcd == cprev.dcd && cnow.cts == cprev.cts)
				return -EIO; /* no change => error */
			if (((arg & TIOCM_RNG) && (cnow.rng != cprev.rng)) ||
					((arg & TIOCM_DSR) && (cnow.dsr != cprev.dsr)) ||
					((arg & TIOCM_CD)  && (cnow.dcd != cprev.dcd)) ||
					((arg & TIOCM_CTS) && (cnow.cts != cprev.cts)) ) {
				return 0;
			}
			cprev = cnow;
		}

	}

	F_LEAVE();
	return -ENOIOCTLCMD;
}

static int cidatatty_ioctl_tiocgicount(struct tty_struct *tty, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	struct cidatatty_serial *cidatatty = tty->driver_data;

	F_ENTER();
	if (cmd == TIOCGICOUNT) {
		struct async_icount cnow = cidatatty->icount;
		struct serial_icounter_struct icount;

		icount.cts	= cnow.cts;
		icount.dsr	= cnow.dsr;
		icount.rng	= cnow.rng;
		icount.dcd	= cnow.dcd;
		icount.rx	= cnow.rx;
		icount.tx	= cnow.tx;
		icount.frame	= cnow.frame;
		icount.overrun	= cnow.overrun;
		icount.parity	= cnow.parity;
		icount.brk	= cnow.brk;
		icount.buf_overrun = cnow.buf_overrun;

		if (copy_to_user((void __user *)arg, &icount, sizeof(icount)))
			return -EFAULT;
		return 0;
	}

	F_LEAVE();
	return -ENOIOCTLCMD;
}

static int cidatatty_ioctl_tcsets(struct tty_struct *tty, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	F_ENTER();

	memcpy(tty->termios, (void *)arg, sizeof(struct termios));
	/*  
	    struct termios * new_termios = (struct termios *) arg;

	    tty->termios->c_iflag = new_termios->c_iflag;  
	    tty->termios->c_oflag= new_termios->c_oflag;
	    tty->termios->c_cflag= new_termios->c_cflag;
	    tty->termios->c_lflag= new_termios->c_lflag;
	    tty->termios->c_line= new_termios->c_line;

	    int i;

	    for(i = 0; i <NCCS; i++)
	    tty->termios->c_cc[i] = new_termios->c_cc[i];
	 */  
	F_LEAVE();

	return 0;
}                      

static int cidatatty_ioctl_tcgets(struct tty_struct *tty, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	F_ENTER();

	if (copy_to_user((void *)arg, tty->termios, sizeof(struct termios)))
	{
		PDEBUG("Failed to copy to user for tcgets.\n");
		return -EFAULT;
	}
	F_LEAVE();

	return 0;
}

/* the real cidatatty_ioctl function.  The above is done to get the small functions*/
static int cidatatty_ioctl(struct tty_struct *tty, struct file *file,
		unsigned int cmd, unsigned long arg)
{
	//unsigned char cid;
	F_ENTER();

	switch (cmd) {
		case TIOCGSERIAL:
			return cidatatty_ioctl_tiocgserial(tty, file, cmd, arg);
		case TIOCMIWAIT:
			return cidatatty_ioctl_tiocmiwait(tty, file, cmd, arg);
		case TIOCGICOUNT:
			return cidatatty_ioctl_tiocgicount(tty, file, cmd, arg);
		case TCSETS:
			return cidatatty_ioctl_tcsets(tty, file, cmd, arg);  
		case TCGETS:  //0x5401 ioctls.h  
			return cidatatty_ioctl_tcgets(tty, file, cmd, arg);
		case TCSETSF: //0x5404                                                
		case TCSETAF: //0x5408                                                
			return 0;   //has to return zero for qtopia to work
		
		default:
			PDEBUG("cidatatty_ioctl cmd: %d.\n", cmd);
			return -ENOIOCTLCMD;   // for PPPD to work?

			break;
	}

	F_LEAVE();

}

static int cidatatty_chars_in_buffer(struct tty_struct *tty)
{
	// this has to return zero? for PPPD to work
	return 0; 
}

static void cidatatty_flush_chars(struct tty_struct *tty)
{
	F_ENTER();
	return;
}

static void cidatatty_wait_until_sent(struct tty_struct *tty, int timeout)
{
	F_ENTER();
	return;
}

static void cidatatty_throttle(struct tty_struct *tty)
{
	F_ENTER();
	return;
}

static void cidatatty_unthrottle(struct tty_struct *tty)
{
	F_ENTER();
	return;
}

static void cidatatty_stop(struct tty_struct *tty)
{
	F_ENTER();
	return;
}

static void cidatatty_start(struct tty_struct *tty)
{
	F_ENTER();
	return;
}

static void cidatatty_hangup(struct tty_struct *tty)
{
	F_ENTER();
	return;
}

static void cidatatty_flush_buffer(struct tty_struct *tty)
{
	F_ENTER();
	return;
}

static void cidatatty_set_ldisc(struct tty_struct *tty)
{
	F_ENTER();
	return;
}



static int __init cidatatty_init(void)
{
	int retval;
	int i;
	F_ENTER();

	/* allocate the tty driver */
	cidatatty_tty_driver = alloc_tty_driver(CIDATATTY_TTY_MINORS);
	if (!cidatatty_tty_driver)
		return -ENOMEM;

	/* initialize the tty driver */
	cidatatty_tty_driver->owner = THIS_MODULE;
	cidatatty_tty_driver->driver_name = "cidatatty_tty";
	cidatatty_tty_driver->name = "cidatatty";
	cidatatty_tty_driver->major = CIDATATTY_TTY_MAJOR;
	cidatatty_tty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	cidatatty_tty_driver->subtype = SERIAL_TYPE_NORMAL;
	cidatatty_tty_driver->flags =  TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	cidatatty_tty_driver->init_termios = tty_std_termios;
	//B115200 | CS8 | CREAD | HUPCL | CLOCAL;
	cidatatty_tty_driver->init_termios.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
	cidatatty_tty_driver->init_termios.c_iflag = IGNBRK | IGNCR | IGNPAR;
	cidatatty_tty_driver->init_termios.c_oflag = 0;
	cidatatty_tty_driver->init_termios.c_lflag = 0;

	tty_set_operations(cidatatty_tty_driver, &serial_ops);

	/* register the tty driver */
	retval = tty_register_driver(cidatatty_tty_driver);
	if (retval) {
		printk(KERN_ERR "failed to register cidatatty tty driver");
		put_tty_driver(cidatatty_tty_driver);
		return retval;
	}

	for (i = 0; i < CIDATATTY_TTY_MINORS; ++i)
	{
		tty_register_device(cidatatty_tty_driver, i, NULL);
		/* Init Buffer */
	}

	printk(KERN_INFO DRIVER_DESC " " DRIVER_VERSION"\n");
	cctdatadev_init_module();
	F_LEAVE();
	return retval;
}

static void __exit cidatatty_exit(void)
{
	struct cidatatty_serial *cidatatty;
	int i;

	F_ENTER();
	for (i = 0; i < CIDATATTY_TTY_MINORS; ++i)
		tty_unregister_device(cidatatty_tty_driver, i);

	tty_unregister_driver(cidatatty_tty_driver);

	/*  free the memory */
	for (i = 0; i < CIDATATTY_TTY_MINORS; ++i) {
		cidatatty = cidatatty_table[i];
		if (cidatatty) {
			/* close the port */
			while (cidatatty->open_count)
				do_close(cidatatty);

			kfree(cidatatty);
			cidatatty_table[i] = NULL;
		}
	}
	cctdatadev_cleanup_module();
	F_LEAVE();
}

int cctdatadev_open(struct inode *inode, struct file *filp)
{
	/*struct cctdatadev_dev *dev;*/ /* device information */
	F_ENTER();
#if 0
	dev = container_of(inode->i_cdev, struct cctdatadev_dev, cdev);

	filp->private_data = dev; /* for other methods */


	/* used to keep track of how many readers */
	if (filp->f_mode & FMODE_READ)
		dev->nreaders++;
	if (filp->f_mode & FMODE_WRITE)
		dev->nwriters++;
#endif
	F_LEAVE();

	return nonseekable_open(inode, filp);          /* success */
}

int cctdatadev_ioctl(struct inode *inode, struct file *filp,
		unsigned int cmd, unsigned long arg)
{

	unsigned char cid;
	int i;
	
//	printk("%s %d %d",__FUNCTION__,cmd,(int)arg);

	switch (cmd) {
#if 0
		case TIOENABLE:
			cid = (unsigned char)arg;
			PDEBUG("cctdatadev_ioctl: cid=%d \n",cid);
			cidatatty_table[ttycurindex]->cid = cid;
			cidatatty_table[ttycurindex]->devinuse = 1;
			break;
		case TIODISABLE:
			cid = (unsigned char)arg;
			for(i=0;i<CIDATATTY_TTY_MINORS;i++)
			{
				if ((cidatatty_table[i]->cid == cid)&&
					(cidatatty_table[i]->devinuse == 1))
				{
					cidatatty_table[i]->devinuse= 0;
					cidatatty_table[i]->writeokflag= 0;
					break;
				}
			}
			if(i>=CIDATATTY_TTY_MINORS)
				PDEBUG("cctdatadev_ioctl: cidatatty dev not found.\n");
			break;
#endif
		case TIOPPPON:
			cid = (unsigned char)arg;
			PDEBUG("cctdatadev_ioctl: TIOPPPON, cid=%d \n",cid);
#if 0
			for(i=0;i<CIDATATTY_TTY_MINORS;i++)
			{
				if ((cidatatty_table[i]->cid == cid)&&
					(cidatatty_table[i]->devinuse == 1))
				{
					cidatatty_table[i]->writeokflag= 1;
					break;
				}
			}
#endif
			currcid =  (unsigned char)arg;
			break;
		case TIOPPPOFF:
			currcid = 0xff;
			break;
		default:
			PDEBUG("cctdatadev_ioctl cmd: %d.\n", cmd);
			return -ENOIOCTLCMD;  

			break;
	}
	return 0;

}

static struct class *cctdatadev_class;

void cctdatadev_cleanup_module(void)
{
	int i;
	dev_t devno = MKDEV(cctdatadev_major, cctdatadev_minor);

	/* Get rid of our char dev entries */
	if (cctdatadev_devices) {
		for (i = 0; i < cctdatadev_nr_devs; i++) {
			cdev_del(&cctdatadev_devices[i].cdev);
			device_destroy(cctdatadev_class,
				MKDEV(cctdatadev_major, cctdatadev_minor + i));
		}
		kfree(cctdatadev_devices);
	}
	
	class_destroy(cctdatadev_class);

	/* cleanup_module is never called if registering failed */
	unregister_chrdev_region(devno, cctdatadev_nr_devs);

}
static void cctdatadev_setup_cdev(struct cctdatadev_dev *dev, int index)
{
	int err, devno = MKDEV(cctdatadev_major, cctdatadev_minor + index);
	F_ENTER();

	cdev_init(&dev->cdev, &cctdatadev_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &cctdatadev_fops;
	err = cdev_add (&dev->cdev, devno, 1);
	/* Fail gracefully if need be */
	if (err)
		printk(KERN_NOTICE "Error %d adding cctdatadev%d", err, index);

	F_LEAVE();

}

int cctdatadev_init_module(void)
{
	int result, i;
	dev_t dev = 0;
	char name[256];
	F_ENTER();

	/*
	 * Get a range of minor numbers to work with, asking for a dynamic
	 * major unless directed otherwise at load time.
	 */
	if (cctdatadev_major) {
		dev = MKDEV(cctdatadev_major, cctdatadev_minor);
		result = register_chrdev_region(dev, cctdatadev_nr_devs, "cctdatadev");
	}
	else {
		result = alloc_chrdev_region(&dev, cctdatadev_minor, cctdatadev_nr_devs,
				"cctdatadev");
		cctdatadev_major = MAJOR(dev);
	}

	if (result < 0) {
		printk(KERN_WARNING "cctdatadev: can't get major %d\n", cctdatadev_major);
		return result;
	}

	/* 
	 * allocate the devices -- we can't have them static, as the number
	 * can be specified at load time
	 */
	cctdatadev_devices = kmalloc(cctdatadev_nr_devs * sizeof(struct cctdatadev_dev), GFP_KERNEL);
	if (!cctdatadev_devices) {
		result = -ENOMEM;
		goto fail;  
	}
	memset(cctdatadev_devices, 0, cctdatadev_nr_devs * sizeof(struct cctdatadev_dev));

	/* Initialize each device. */
	cctdatadev_class = class_create(THIS_MODULE, "cctdatadev");
	for (i = 0; i < cctdatadev_nr_devs; i++) {
		sprintf(name, "%s%d", "cctdatadev", cctdatadev_minor + i);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 28)
		device_create(cctdatadev_class, NULL,
			      MKDEV(cctdatadev_major, cctdatadev_minor + i), NULL,
			      name);
#else
		device_create(cctdatadev_class, NULL,
			      MKDEV(cctdatadev_major, cctdatadev_minor + i),
			      name);
#endif

		cctdatadev_setup_cdev(&cctdatadev_devices[i], i);

	}

	/* At this point call the init function for any friend device */
	//dev = MKDEV(cctdatadev_major, cctdatadev_minor + cctdatadev_nr_devs);



	F_LEAVE();

	return 0; /* succeed */

fail:
	cctdatadev_cleanup_module();

	return result;
}
module_init(cidatatty_init);
module_exit(cidatatty_exit);

