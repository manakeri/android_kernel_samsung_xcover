 /*****************************************************************************
 *
 * Title: Linux Device Driver for Proximity Sensor TAOSP002S00F
 * COPYRIGHT(C) : Samsung Electronics Co.Ltd, 2006-2015 ALL RIGHTS RESERVED
 *
 *****************************************************************************/
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <linux/wakelock.h>

#include <asm/gpio.h>  
#include <mach/mfp.h>
#include <mach/mfp-pxa3xx.h>
 
#include <linux/taos.h>

#if defined(CONFIG_PMIC_D1980)
#include <linux/regulator/consumer.h>
#include <linux/d1982/pmic.h>
#endif

#define TAOS_DEBUG 0

/*********** for debug **********************************************************/
#if TAOS_DEBUG 
#define gprintk(fmt, x... ) printk( "%s(%d): " fmt, __FUNCTION__ ,__LINE__, ## x)
#else
#define gprintk(x...) do { } while (0)
#endif
/*******************************************************************************/

/* Triton register offsets */
#define CNTRL				0x00
#define ALS_TIME			0X01
#define PRX_TIME			0x02
#define WAIT_TIME			0x03
#define ALS_MINTHRESHLO			0X04
#define ALS_MINTHRESHHI			0X05
#define ALS_MAXTHRESHLO			0X06
#define ALS_MAXTHRESHHI			0X07
#define PRX_MINTHRESHLO			0X08
#define PRX_MINTHRESHHI			0X09
#define PRX_MAXTHRESHLO			0X0A
#define PRX_MAXTHRESHHI			0X0B
#define INTERRUPT			0x0C
#define PRX_CFG				0x0D
#define PRX_COUNT			0x0E
#define GAIN				0x0F
#define REVID				0x11
#define CHIPID				0x12
#define STATUS				0x13
#define ALS_CHAN0LO			0x14
#define ALS_CHAN0HI			0x15
#define ALS_CHAN1LO			0x16
#define ALS_CHAN1HI			0x17
#define PRX_LO				0x18
#define PRX_HI				0x19
#define TEST_STATUS			0x1F

/* Triton cmd reg masks */
#define CMD_REG				0X80
#define CMD_BYTE_RW			0x00
#define CMD_WORD_BLK_RW			0x20
#define CMD_SPL_FN			0x60
#define CMD_PROX_INTCLR			0X05
#define CMD_ALS_INTCLR			0X06
#define CMD_PROXALS_INTCLR		0X07
#define CMD_TST_REG			0X08
#define CMD_USER_REG			0X09

/* Triton cntrl reg masks */
#define CNTL_REG_CLEAR			0x00
#define CNTL_PROX_INT_ENBL		0X20
#define CNTL_ALS_INT_ENBL		0X10
#define CNTL_WAIT_TMR_ENBL		0X08
#define CNTL_PROX_DET_ENBL		0X04
#define CNTL_ADC_ENBL			0x02
#define CNTL_PWRON			0x01
#define CNTL_ALSPON_ENBL		0x03
#define CNTL_INTALSPON_ENBL		0x13
#define CNTL_PROXPON_ENBL		0x0F
#define CNTL_INTPROXPON_ENBL		0x2F

/* Triton status reg masks */
#define STA_ADCVALID			0x01
#define STA_PRXVALID			0x02
#define STA_ADC_PRX_VALID		0x03
#define STA_ADCINTR			0x10
#define STA_PRXINTR			0x20

/* Lux constants */
#define	MAX_LUX				65535000
#define SCALE_MILLILUX			3
#define FILTER_DEPTH			3
#define	GAINFACTOR			9

/* Thresholds */
#define ALS_THRESHOLD_LO_LIMIT		0x0000
#define ALS_THRESHOLD_HI_LIMIT		0xFFFF
#define PROX_THRESHOLD_LO_LIMIT		0x0000
#define PROX_THRESHOLD_HI_LIMIT		0xFFFF

/* Device default configuration */
#define CALIB_TGT_PARAM			300000
#define ALS_TIME_PARAM			100
#define SCALE_FACTOR_PARAM		1
#define GAIN_TRIM_PARAM			512
#define GAIN_PARAM			1
#define ALS_THRSH_HI_PARAM		0xFFFF
#define ALS_THRSH_LO_PARAM		0

/* [HSS] This value depends on each model's H/W. It needs tunning */
#if (defined(CONFIG_MACH_COOPER) || defined(CONFIG_MACH_BENI))
	#define PRX_THRSH_HI_PARAM		550
	#define PRX_THRSH_LO_PARAM		400
#elif defined(CONFIG_MACH_TASS)
	#define PRX_THRSH_HI_PARAM		600
	#define PRX_THRSH_LO_PARAM		450
#else			
	#define PRX_THRSH_HI_PARAM		0x2BC // 700
	#define PRX_THRSH_LO_PARAM		0x226 // 600
#endif
#define PRX_INT_TIME_PARAM		0xFC
#define PRX_ADC_TIME_PARAM		0xFF // [HSS] Original value : 0XEE
#define PRX_WAIT_TIME_PARAM		0xF2
#define INTR_FILTER_PARAM		0x33
#define PRX_CONFIG_PARAM		0x00
#if defined(CONFIG_MACH_BENI)
#define PRX_PULSE_CNT_PARAM		0x0A //0x0F
#else
#define PRX_PULSE_CNT_PARAM		0x08
#endif
#define PRX_GAIN_PARAM			0x20



/* global var */
static struct i2c_client *opt_i2c_client = NULL;

struct class *proxsensor_class;

struct device *switch_cmd_dev;

static bool proximity_enable = OFF;

static short proximity_value = 0;

static struct wake_lock prx_wake_lock;

static ktime_t timeA,timeB;
#if USE_INTERRUPT
static ktime_t timeSub;
#endif
#if defined(CONFIG_PMIC_D1980)
static struct regulator *reg_prox = NULL;
#endif

/*************************************************************************/
/*		TAOS I2C_API	  				         */
/*************************************************************************/
/*  i2c read routine for taos  */
/*
#if USE_INTERRUPT

static int opt_i2c_read(u8 reg, u8 *val, unsigned int len )
{

	int err;
	u8 buf[3];
	//u8 rbuf[3];

	//memset(buf, 0, sizeof(buf));
	
	struct i2c_msg msg[2];


	buf[0] = reg; 

	msg[0].addr = opt_i2c_client->addr;
	msg[0].flags = 1;
	
	msg[0].len = 2;
	msg[0].buf = buf;
	err = i2c_transfer(opt_i2c_client->adapter, msg, 1);
	
	*val = buf[0] << 8 | buf[1];
	
    if (err >= 0) return 0;

    printk("%s %d i2c transfer error\n", __func__, __LINE__);
    return err;
}
#endif
*/
/*  i2c write routine for taos */
static int opt_i2c_write( u8 reg, u8 *val )
{
    int err;
    struct i2c_msg msg[1];
    unsigned char data[2];

    if( opt_i2c_client == NULL )
        return -ENODEV;

    data[0] = reg;
    data[1] = *val;

    msg->addr = opt_i2c_client->addr;
    msg->flags = I2C_M_WR;
    msg->len = 2;
    msg->buf = data;

    err = i2c_transfer(opt_i2c_client->adapter, msg, 1);

    if (err >= 0) return 0;

    printk("[HSS] %s %d i2c transfer error : reg = [%X]\n", __func__, __LINE__, reg);
    return err;
}


/*************************************************************************/
/*		TAOS sysfs	  				         */
/*************************************************************************/

short taos_get_proximity_value()
{
	return ((proximity_value==1)? 0:1);
}

EXPORT_SYMBOL(taos_get_proximity_value);


static ssize_t proxsensor_file_state_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	unsigned int detect = 0;

	gprintk("called %s \n",__func__);

	detect = gpio_get_value(GPIO_PROXI_INT);	

//	return sprintf(buf,"%u\n",detect);
	return detect;
}

static DEVICE_ATTR(proxsensor_file_state,0666, proxsensor_file_state_show, NULL);



/*****************************************************************************************
 *  
 *  function    : taos_work_func_prox 
 *  description : This function is for proximity sensor (using B-1 Mode ). 
 *                when INT signal is occured , it gets value from VO register.   
 *
 *                 
 */
#if USE_INTERRUPT
static void taos_work_func_prox(struct work_struct *work) 
{
	struct taos_data *taos = container_of(work, struct taos_data, work_prox);
	unsigned char vout=0;
	u16 adc_data;
	u16 threshold_high;
	u16 threshold_low;
	u8 prox_int_thresh[4];
	int i;
	/* Read VO & INT Clear */
	
	gprintk("[PROXIMITY] %s : \n",__func__);

	/* change Threshold */ 
	adc_data = i2c_smbus_read_word_data(opt_i2c_client, CMD_REG | PRX_LO);
	threshold_high= i2c_smbus_read_word_data(opt_i2c_client, (CMD_REG | PRX_MAXTHRESHLO) );
	threshold_low= i2c_smbus_read_word_data(opt_i2c_client, (CMD_REG | PRX_MINTHRESHLO) );
	if ( (threshold_high ==  (PRX_THRSH_HI_PARAM)) && (adc_data >=  (PRX_THRSH_HI_PARAM) ) )
	{
#if TAOS_DEBUG
		printk("[HSS] [%s] +++ adc_data=[%d], threshold_high=[%d],  threshold_min=[%d]\n", __func__, adc_data, threshold_high, threshold_low);
#endif
		proximity_value = 1;
		prox_int_thresh[0] = (PRX_THRSH_LO_PARAM) & 0xFF;
		prox_int_thresh[1] = (PRX_THRSH_LO_PARAM >> 8) & 0xFF;
		prox_int_thresh[2] = (0xFFFF) & 0xFF;
		prox_int_thresh[3] = (0xFFFF >> 8) & 0xFF; 
		for (i = 0; i < 4; i++)
		{
			opt_i2c_write((CMD_REG|(PRX_MINTHRESHLO + i)),&prox_int_thresh[i]);
		}
	}
	else if ( (threshold_high ==  (0xFFFF)) && (adc_data <=  (PRX_THRSH_LO_PARAM) ) )
	{
#if TAOS_DEBUG
		printk("[HSS] [%s] --- adc_data=[%d], threshold_high=[%d],  threshold_min=[%d]\n", __func__, adc_data, threshold_high, threshold_low);
#endif
		proximity_value = 0;
		prox_int_thresh[0] = (0x0000) & 0xFF;
		prox_int_thresh[1] = (0x0000 >> 8) & 0xFF;
		prox_int_thresh[2] = (PRX_THRSH_HI_PARAM) & 0xFF;
		prox_int_thresh[3] = (PRX_THRSH_HI_PARAM >> 8) & 0xFF; 
		for (i = 0; i < 4; i++)
		{
			opt_i2c_write((CMD_REG|(PRX_MINTHRESHLO + i)),&prox_int_thresh[i]);
		}
	}
    else
    {
#if TAOS_DEBUG
        printk("[HSS] [%s] Error! Not Common Case!adc_data=[%d], threshold_high=[%d],  threshold_min=[%d]\n", __func__, adc_data, threshold_high, threshold_low);
#endif
    }
      
	if(proximity_value ==0)
	{
		timeB = ktime_get();
		
		timeSub = ktime_sub(timeB,timeA);
#if TAOS_DEBUG
		printk(KERN_INFO "[PROXIMITY] timeSub sec = %d, timeSub nsec = %d \n",timeSub.tv.sec,timeSub.tv.nsec);
#endif
		
		if (timeSub.tv.sec>=3 )
		{
		    wake_lock_timeout(&prx_wake_lock,HZ/2);
#if TAOS_DEBUG
			printk(KERN_INFO "[PROXIMITY] wake_lock_timeout : HZ/2 \n");
#endif
		}
		else
		{
#if TAOS_DEBUG
			printk(KERN_INFO "[PROXIMITY] wake_lock is already set \n");
#endif
		}

	}

	if(USE_INPUT_DEVICE)
	{
		input_report_abs(taos->input_dev,ABS_DISTANCE,(int)vout);
		input_sync(taos->input_dev);
		mdelay(1);
	}

	/* reset Interrupt pin */
	/* to active Interrupt, TMD2771x Interuupt pin shoud be reset. */
	i2c_smbus_write_byte(opt_i2c_client,(CMD_REG|CMD_SPL_FN|CMD_PROXALS_INTCLR));

	/* enable INT */

	enable_irq(taos->irq);

	

}


static irqreturn_t taos_irq_handler(int irq, void *dev_id)
{
	struct taos_data *taos = dev_id;
#if TAOS_DEBUG
	printk("[PROXIMITY] taos->irq = %d\n",taos->irq);
#endif
	if(taos->irq !=-1)
	{
		disable_irq_nosync(taos->irq);
		gprintk("[PROXIMITY] disable_irq \n");

		queue_work(taos_wq, &taos->work_prox);

	}
#if TAOS_DEBUG	
	printk("[PROXIMITY] IRQ_HANDLED \n");
#endif
	return IRQ_HANDLED;
}

#endif



void taos_chip_init(int on)
{
	unsigned long proxi_en_mfp = 0;
#if TAOS_DEBUG	
	printk("%s called : on=%d\n",__func__, on); 
#endif
	if (gpio_request(GPIO_PROXI_EN, "PROXI_EN")) {
		printk(KERN_ERR "Proximity Request PROXI_EN GPIO failed!\n");
		return;
	}
	
	if (on) {
#if defined(CONFIG_PMIC_D1980)
        if(reg_prox == NULL)
            reg_prox = regulator_get(switch_cmd_dev, REGULATOR_PROXMITY);
        if(reg_prox)
        {
            regulator_set_voltage(reg_prox, OUTPUT_2_8V, OUTPUT_2_8V);
            regulator_enable(reg_prox);
        	printk(KERN_INFO "[%s] : Proximity enabled (%d) \n",__func__, on); 
        }
#endif /* CONFIG_PMIC_D1980 */
		proxi_en_mfp = pxa3xx_mfp_read(GPIO_PROXI_EN);
	  
		proxi_en_mfp |= 0x100; /* set high */
	  
		pxa3xx_mfp_write(GPIO_PROXI_EN, proxi_en_mfp & 0xffff);
	  
		gpio_direction_output(GPIO_PROXI_EN, 1);
	} else {
		gpio_direction_output(GPIO_PROXI_EN, 0);
		
		proxi_en_mfp = pxa3xx_mfp_read(GPIO_PROXI_EN);

		proxi_en_mfp &= ~0x100; /* set low*/

		pxa3xx_mfp_write(GPIO_PROXI_EN, proxi_en_mfp & 0xffff);

#if defined(CONFIG_PMIC_D1980)
        if(reg_prox == NULL)
            reg_prox = regulator_get(switch_cmd_dev, REGULATOR_PROXMITY);
        if(reg_prox)
        {
            regulator_disable(reg_prox);
        	printk(KERN_INFO "[%s] : Proximity enabled (%d) \n",__func__, on); 
        }
#endif /* CONFIG_PMIC_D1980 */
	}

	gpio_free(GPIO_PROXI_EN);

	/*After Power Supply is supplied, about 1ms delay is required before issuing read/write commands */
	mdelay(1);	
}



/*****************************************************************************************
 *  
 *  function    : taos_on 
 *  description : This function is power-on function for optical sensor.
 *
 *  int type    : Sensor type. Two values is available (PROXIMITY,LIGHT).
 *                it support power-on function separately.
 *                
 *                 
 */

void taos_on(struct taos_data *taos, int type)
{
	u8 value;
	u8 prox_int_thresh[4];
	int err = 0;
	int i;
	
	gprintk("taos_on(%d)\n",type);

	taos_chip_init(ON);

	//register irq to wakeup source
	//printk("[TAOS] register irq = %d\n",taos ->irq);
	//err = set_irq_wake(taos ->irq, 1);  // enable : 1, disable : 0
	//printk("[TAOS] register wakeup source = %d\n",err);
	//if (err) 
		//printk("[TAOS] register wakeup source failed\n");
	if(type == PROXIMITY)
	{
		gprintk("enable irq for proximity\n");
		enable_irq(taos ->irq);
	}

    mdelay(12); // [HSS] Fix Proximity I2C Fail : Add delay 12ms after VDD ON

	value = CNTL_REG_CLEAR;
	if ((err = (opt_i2c_write((CMD_REG|CNTRL),&value))) < 0){
			printk("[diony] i2c_smbus_write_byte_data to clr ctrl reg failed in ioctl prox_on\n");
		}
	value = PRX_INT_TIME_PARAM;
	if ((err = (opt_i2c_write((CMD_REG|ALS_TIME), &value))) < 0) {
			printk("[diony] i2c_smbus_write_byte_data to als time reg failed in ioctl prox_on\n");
		}
	value = PRX_ADC_TIME_PARAM;
	if ((err = (opt_i2c_write((CMD_REG|PRX_TIME), &value))) < 0) {
			printk("[diony] i2c_smbus_write_byte_data to prox time reg failed in ioctl prox_on\n");
		}
	value = PRX_WAIT_TIME_PARAM;
	if ((err = (opt_i2c_write((CMD_REG|WAIT_TIME), &value))) < 0){
			printk("[diony] i2c_smbus_write_byte_data to wait time reg failed in ioctl prox_on\n");
		}
	value = INTR_FILTER_PARAM;
	if ((err = (opt_i2c_write((CMD_REG|INTERRUPT), &value))) < 0) {
			printk("[diony] i2c_smbus_write_byte_data to interrupt reg failed in ioctl prox_on\n");
		}
	value = PRX_CONFIG_PARAM;
	if ((err = (opt_i2c_write((CMD_REG|PRX_CFG), &value))) < 0) {
			printk("[diony] i2c_smbus_write_byte_data to prox cfg reg failed in ioctl prox_on\n");
		}
	value = PRX_PULSE_CNT_PARAM;
	if ((err = (opt_i2c_write((CMD_REG|PRX_COUNT), &value))) < 0){
			printk("[diony] i2c_smbus_write_byte_data to prox cnt reg failed in ioctl prox_on\n");
		}
	value = PRX_GAIN_PARAM;
	if ((err = (opt_i2c_write((CMD_REG|GAIN), &value))) < 0) {
			printk("[diony] i2c_smbus_write_byte_data to prox gain reg failed in ioctl prox_on\n");
		}
		prox_int_thresh[0] = (0x0000) & 0xFF;
		prox_int_thresh[1] = (0x0000 >> 8) & 0xFF;
		prox_int_thresh[2] = (PRX_THRSH_HI_PARAM) & 0xFF;
		prox_int_thresh[3] = (PRX_THRSH_HI_PARAM >> 8) & 0xFF; 
	for (i = 0; i < 4; i++) {
		if ((err = (opt_i2c_write((CMD_REG|(PRX_MINTHRESHLO + i)),&prox_int_thresh[i]))) < 0) {
				printk("[diony]i2c_smbus_write_byte_data to prox int thrsh regs failed in ioctl prox_on\n");
			}
	}
	value = CNTL_INTPROXPON_ENBL;
	if ((err = (opt_i2c_write((CMD_REG|CNTRL), &value))) < 0) {
			printk("[diony]i2c_smbus_write_byte_data to ctrl reg "
						"failed in ioctl prox_on\n");
		}
}

/*****************************************************************************************
 *  
 *  function    : taos_off 
 *  description : This function is power-off function for optical sensor.
 *
 *  int type    : Sensor type. Three values is available (PROXIMITY,LIGHT,ALL).
 *                it support power-on function separately.
 *                
 *                 
 */

void taos_off(struct taos_data *taos, int type)
{
	u8 value;
	int err = 0;
	
	gprintk("taos_off(%d)\n",type);
	
	value = CNTL_REG_CLEAR;
	opt_i2c_write((u8)(CMD_REG|CNTRL) , &value);

	taos_chip_init(OFF);

	//delete irq from wakeup source
#if TAOS_DEBUG
	printk("[TAOS] unregister irq = %d\n",taos ->irq);
#endif
	err = set_irq_wake(taos ->irq, 0); // enable : 1, disable : 0
#if TAOS_DEBUG
	printk("[TAOS] register wakeup source = %d\n",err);
#endif
	if (err) 
		printk("[TAOS] register wakeup source failed\n");
	
	gprintk("taos power off \n");
	if(type == PROXIMITY || type==ALL)
	{
		gprintk("disable irq for proximity \n");
		disable_irq_nosync(taos ->irq);
	}
}


/*************************************************************************/
/*		TAOS file operations  				         */
/*************************************************************************/
static int proximity_open(struct inode *ip, struct file *fp)
{
	return 0;

}

static int proximity_release(struct inode *ip, struct file *fp)
{
	return 0;

}

static long proximity_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

	struct taos_data *taos = dev_get_drvdata(switch_cmd_dev);
	int ret=0;
	switch(cmd) {

		case TAOS_PROX_OPEN:
			{
#if TAOS_DEBUG
				printk(KERN_INFO "[PROXIMITY] %s : case OPEN\n", __FUNCTION__);
#endif
				taos_on(taos,PROXIMITY);
				proximity_enable =1;
				
			}
			break;

		case TAOS_PROX_CLOSE:
			{
#if TAOS_DEBUG
				printk(KERN_INFO "[PROXIMITY] %s : case CLOSE\n", __FUNCTION__);
#endif
				taos_off(taos,PROXIMITY);
				proximity_enable=0;
			}
			break;

		default:
			printk(KERN_INFO "[PROXIMITY] unknown ioctl %d\n", cmd);
			ret = -1;
			break;
	}
	return ret;
}



static struct file_operations proximity_fops = {
	.owner  = THIS_MODULE,
	.open   = proximity_open,
	.release = proximity_release,
	.unlocked_ioctl = proximity_ioctl,
};
                 
static struct miscdevice proximity_device = {
    .minor  = MISC_DYNAMIC_MINOR,
    .name   = "proximity",
    .fops   = &proximity_fops,
};


//------------------------------------------------------------------------------------

static int taos_opt_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int err = 0, out_pin;
	unsigned char value;
#if USE_INTERRUPT
	int irq;
#endif

	struct taos_data *taos;
#if TAOS_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif
	taos_chip_init(ON);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk(KERN_INFO "[TAOS] i2c_check_functionality error\n");
		err = -ENODEV;
		goto exit;
	}
	if ( !i2c_check_functionality(client->adapter,I2C_FUNC_SMBUS_BYTE_DATA) ) {
		printk(KERN_INFO "[TAOS] byte op is not permited.\n");
		goto exit;
	}

	/* OK. For now, we presume we have a valid client. We now create the
	client structure, even though we cannot fill it completely yet. */
	if (!(taos = kzalloc(sizeof(struct taos_data), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	memset(taos, 0, sizeof(struct taos_data));
	taos->client = client;
	i2c_set_clientdata(client, taos);
	opt_i2c_client = client;
#if TAOS_DEBUG
	printk("[%s] slave addr = %x\n", __func__, client->addr);		
#endif
	mdelay(12); // [HSS] Fix Proximity I2C Fail : Add delay 12ms after VDD ON
   
	/* Basic Software Operation for TMD2771X Module */
	value = CNTL_REG_CLEAR;
	opt_i2c_write((u8)(CMD_REG|CNTRL) , &value);
	value = ATIME;
	opt_i2c_write((u8)(CMD_REG|ALS_TIME) , &value);
	value = PTIME;
	opt_i2c_write((u8)(CMD_REG|PRX_TIME) , &value);
	value = WTIME;
	opt_i2c_write((u8)(CMD_REG|WAIT_TIME) , &value);
	value = PPCOUNT;
	opt_i2c_write((u8)(CMD_REG|PRX_COUNT) , &value);
	value = PDRIVE | PDIODE | PGAIN | AGAIN;
	opt_i2c_write((u8)(CMD_REG|GAIN) , &value);
	value = (WEN | PEN | PON); 
	opt_i2c_write((u8)(CMD_REG|CNTRL) , &value);
	mdelay(12);
	if (i2c_smbus_read_byte(client) < 0)
	{
		printk(KERN_ERR "[TAOS] i2c_smbus_read_byte error!!\n");
		goto exit_kfree;
	}
	else
	{
#if TAOS_DEBUG
		printk("TAOS Device detected!\n");
#endif
	}
	
	/* Input device Settings */
	if(USE_INPUT_DEVICE)
	{
		taos->input_dev = input_allocate_device();
		if (taos->input_dev == NULL) 
		{
			pr_err("Failed to allocate input device\n");
			return -ENOMEM;
		}
		taos->input_dev->name = "proximity";
	
		set_bit(EV_SYN,taos->input_dev->evbit);
		set_bit(EV_ABS,taos->input_dev->evbit);
		
 	    input_set_abs_params(taos->input_dev, ABS_DISTANCE, 0, 1, 0, 0);
		
	
		err = input_register_device(taos->input_dev);
		if (err) 
		{
			pr_err("Unable to register %s input device\n", taos->input_dev->name);
			input_free_device(taos->input_dev);
			kfree(taos);
			return -1;
		}

	}

#if USE_INTERRUPT
	/* WORK QUEUE Settings */
	taos_wq = create_singlethread_workqueue("taos_wq");
	if (!taos_wq)
		return -ENOMEM;
	INIT_WORK(&taos->work_prox, taos_work_func_prox);
	gprintk("Workqueue Settings complete\n");
#endif

	/* misc device Settings */
	err = misc_register(&proximity_device);
	if(err) {
		pr_err(KERN_ERR "misc_register failed - prox \n");
	}

	/* wake lock init */
	wake_lock_init(&prx_wake_lock, WAKE_LOCK_SUSPEND, "prx_wake_lock");

	/* set sysfs for light sensor */
	proxsensor_class = class_create(THIS_MODULE, "proxsensor");
	if (IS_ERR(proxsensor_class))
		pr_err("Failed to create class(proxsensor)!\n");

	switch_cmd_dev = device_create(proxsensor_class, NULL, 0, NULL, "switch_cmd");	
	if (device_create_file(switch_cmd_dev, &dev_attr_proxsensor_file_state) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_proxsensor_file_state.attr.name);
	
	dev_set_drvdata(switch_cmd_dev,taos);
	
	/* ktime init */
	timeA = ktime_set(0,0);
	timeB = ktime_set(0,0);
	
	/* gpio config */
	out_pin = mfp_to_gpio(GPIO_PROXI_INT);
	if (gpio_request(out_pin, "Proximity Out")) {
		printk(KERN_ERR "Proximity Request GPIO_%d failed!\n", out_pin);
	}
	
	gpio_direction_input(out_pin);
	gpio_set_value(out_pin, 1);
	gpio_free(out_pin);

#if defined(CONFIG_PMIC_D1980)
    if(reg_prox == NULL)
    {
        reg_prox = regulator_get(switch_cmd_dev, REGULATOR_PROXMITY);
        if(reg_prox == NULL)
        {
            printk(KERN_ERR "[TAOS] Failure get proximity sensor regulator...!\n");
            goto exit_kfree;
        }
    }
#endif /* CONFIG_PMIC_D1980 */

	mdelay(2);
#if USE_INTERRUPT
	/* INT Settings */	
	irq = gpio_to_irq(mfp_to_gpio(GPIO_PROXI_INT));
	taos->irq = -1;
	set_irq_type(irq, IRQ_TYPE_EDGE_FALLING);

	err = request_irq(irq, taos_irq_handler, IRQF_DISABLED, "taos_int", taos);
	if (err)
	{
		printk("[TAOS] request_irq failed for taos\n");
		goto exit_kfree;
	}
#if TAOS_DEBUG
	printk("[TAOS] register irq = %d\n",irq);
#endif
	//err = set_irq_wake(irq, 1);
	//printk("[TAOS] register wakeup source = %d\n",err);
	//if (err) 
		//printk("[TAOS] register wakeup source failed\n");
	
	taos->irq = irq;
	gprintk("INT Settings complete\n");
#endif
	
	// maintain power-down mode before using sensor
	taos_off(taos,ALL);
#if TAOS_DEBUG	
	printk("taos_opt_probe is OK!!\n");
#endif
	return 0;
	
exit_kfree:
	kfree(taos);
exit:
	return err;
}


static int taos_opt_remove(struct i2c_client *client)
{
	struct taos_data *taos = i2c_get_clientdata(client);
#if TAOS_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif

#if USE_INTERRUPT
	free_irq(gpio_to_irq(GPIO_PROXI_INT), NULL);
#endif

	if (taos_wq)
		destroy_workqueue(taos_wq);

	if(USE_INPUT_DEVICE)
		input_unregister_device(taos->input_dev);
	kfree(taos);

	misc_deregister(&proximity_device);

	return 0;
}

#ifdef CONFIG_PM
static int taos_opt_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct taos_data *taos = i2c_get_clientdata(client);
#if TAOS_DEBUG
	printk(KERN_INFO "[%s] TAOS !!suspend mode!!\n",__FUNCTION__);
#endif
	if(proximity_enable)
	{
		disable_irq_nosync(taos ->irq);
#if TAOS_DEBUG
		printk(KERN_INFO "[%s] TAOS !!suspend mode proximity_enable \n",__FUNCTION__);
#endif
	}
	return 0;
}

static int taos_opt_resume(struct i2c_client *client)
{
	struct taos_data *taos = i2c_get_clientdata(client);
#if TAOS_DEBUG
	printk(KERN_INFO "[%s] TAOS !!resume mode!!\n",__FUNCTION__);
#endif
	/* wake_up source handler */
	if(proximity_enable)
	{
		enable_irq(taos->irq);
		wake_lock_timeout(&prx_wake_lock,3 * HZ);
		timeA = ktime_get();
#if TAOS_DEBUG		
		printk("[%s] : wake_lock_timeout 3 Sec \n",__func__);
#endif
	}
	return 0;
}
#else
#define taos_opt_suspend NULL
#define taos_opt_resume NULL
#endif

static unsigned short normal_i2c[] = { I2C_CLIENT_END};
I2C_CLIENT_INSMOD_1(taos);

static const struct i2c_device_id taos_id[] = {
	{ "taos", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, taos_id);

static struct i2c_driver taos_opt_driver = {	
	.driver = {
		.owner	= THIS_MODULE,	
		.name	= "taos",
	},
//	.class		= I2C_CLASS_HWMON,
	.id_table	= taos_id,
//	.address_data	= &addr_data,
	.probe		= taos_opt_probe,
	.remove		= taos_opt_remove,
	.suspend	= taos_opt_suspend,
	.resume		= taos_opt_resume,
};

static int __init taos_opt_init(void)
{
#if TAOS_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif
	return i2c_add_driver(&taos_opt_driver);
}

static void __exit taos_opt_exit(void)
{
	i2c_del_driver(&taos_opt_driver);
#if TAOS_DEBUG
	printk(KERN_INFO "%s\n",__FUNCTION__);
#endif
}

module_init( taos_opt_init );
module_exit( taos_opt_exit );

MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("Optical Sensor driver for taosp002s00f");
MODULE_LICENSE("GPL");
