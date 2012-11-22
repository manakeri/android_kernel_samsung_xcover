/*
 *  at42qt.c - Linux kernel modules for atmel tsp
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/i2c/atmel_qt602240.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/sizes.h>

#define AT42QT_DRV_NAME	"at42qt"
#define DRIVER_VERSION		"0.0.2"

/*
 * Defines
 */
#define AT42QT_CHIPID       0x0F
#define AT42QT_CI    0
#define AT42QT_CV    1
#define AT42QT_CA    2
#define AT42QT_RS    3
#define AT42QT_BR    4
#define AT42QT_PT    5

#define AT42QT_LM    656

#define AT42QT_REG_NUM		768 /*0..0x300*/
/*
 * Structs
 */

struct at42qt_data {
	struct i2c_client *client;
	struct mutex update_lock;

	unsigned int power_state : 1;
};

struct at42qt_read_cache {
	u8  data;
	u8  hit;
	u8  cachable;
	u8  spare;
};

typedef struct qtpacket_t{
	u16 pkt_address;
	u8 pkt_data[4];
	};
	
static struct at42qt_read_cache at42qt_regs[AT42QT_REG_NUM];
u16 qt5480_pkt_address;
u8 qt_read_burst[4];
u8 quantum_msg[6];
EXPORT_SYMBOL(qt5480_pkt_address);
EXPORT_SYMBOL(qt_read_burst);
EXPORT_SYMBOL(quantum_msg);

/*
 * Global data
 */

/* Unique ID allocation */
struct i2c_client *g_client;


#define ACK			0
#define NACK			1
#define I2C_OK		1
#define I2C_FAIL		0
#define READ_FLAG	0x01
#define BUS_DELAY	1
#define QT26XB_I2C_ADDRESS		0x60
#define I2C_MAX_SEND_LENGTH     300
	
/************** WRITE  *****************************************/

struct i2c_client* at42qt_get_client(void)
{
   return g_client;
}
EXPORT_SYMBOL(at42qt_get_client);

int at42qt_write(u16 reg, u8 val)
{
	int ret = 0;
	int status;
	u8 data[3];

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	at42qt_regs[reg].hit = 0;

	data[0] = reg & 0xFF;
	data[1] = reg >> 8;
	data[2] = val;

	ret = i2c_master_send(g_client, data, 3);
	if (ret == 3) {
		at42qt_regs[reg].data = val;
		at42qt_regs[reg].hit = at42qt_regs[reg].cachable;
		status = 0;
	} else {
		status = -EIO;
	}

	return status;
}
EXPORT_SYMBOL(at42qt_write);

int at42qt_write_by_size(u16 reg, u8 size, u8 *val)
{
      int i=0;
	int ret = 0;
	int status;
	unsigned char data[I2C_MAX_SEND_LENGTH];

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	if(size+2 >300)
	{
		printk("[TSP][ERROR] %s() data length error\n", __FUNCTION__);
		return -ENODEV;
	}

	at42qt_regs[reg].hit = 0;

	data[0] = reg & 0xFF;
	data[1] = reg >> 8;

	for (i = 0; i < size; i++)
	{
		data[i+2] = *(val+i);
	}
    
	ret = i2c_master_send(g_client, data, size+2);
	if (ret == size+2) {
		at42qt_regs[reg].data = val;
		at42qt_regs[reg].hit = at42qt_regs[reg].cachable;
		status = 0;
	} else {
		status = -EIO;
	}
	return status;
}
EXPORT_SYMBOL(at42qt_write_by_size);

/************** READ *****************************************/
int at42qt_read_with_address(u16 reg, u16 address)
{
	int ret =0;
	int status =0;
	u8 values[6], retry = 0;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	/* Cache read of the at42qtxxxx register */
	if (at42qt_regs[reg].hit) {
		return at42qt_regs[reg].data;
	}
	ret = at42qt_write(address, reg >> 2);

    do {
	    ret = i2c_master_recv(g_client, values, 6); // read five byte data packet

    	quantum_msg[0]=values[0];
    	quantum_msg[1]=values[1];
      	quantum_msg[2]=values[2];
      	quantum_msg[3]=values[3];
      quantum_msg[4]=values[4];
      	quantum_msg[5]=values[5];
    } while(retry--);


    //printk("m0 = %d, m1= %d, m2 = %d, m3=%d, m4=%d, m5=%d", values[0], values[1], values[2], values[3], values[4], values[5]);
 
	//return status; SKC
	return ret;
}
EXPORT_SYMBOL(at42qt_read_with_address);

int at42qt_read_org(u16 reg)
{
	int ret;
	int status;
	u8 value[5], retry = 0;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	/* Cache read of the at42qtxxxx register */
	if (at42qt_regs[reg].hit) {
		return at42qt_regs[reg].data;
	}
	ret = at42qt_write(AT42QT_PT, reg >> 2);

    do {
	    ret = i2c_master_recv(g_client, value, 5); // read five byte data packet

     	if (((reg >> 2 )== value[0]) && (ret == 5))
    	{
			at42qt_regs[reg].data = value[(reg & 0x03)+1];
			at42qt_regs[reg].hit = at42qt_regs[reg].cachable;
			status = at42qt_regs[reg].data;
 			break;
    	}
		else
		{
			mdelay(1);
			status = -EIO;
		}
    } while(retry--);
 
	return status;
}
EXPORT_SYMBOL(at42qt_read_org);


int at42qt_read(u16 reg)
{
	int ret =0;
	int status =0;
	u8 values[6], retry = 0;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	/* Cache read of the at42qtxxxx register */
	if (at42qt_regs[reg].hit) {
		return at42qt_regs[reg].data;
	}
	ret = at42qt_write(AT42QT_PT, reg >> 2);

    do {
	    ret = i2c_master_recv(g_client, values, 6); // read five byte data packet

    	quantum_msg[0]=values[0];
    	quantum_msg[1]=values[1];
      	quantum_msg[2]=values[2];
      	quantum_msg[3]=values[3];
      quantum_msg[4]=values[4];
      	quantum_msg[5]=values[5];
    } while(retry--);

 
	//return status; SKC
	return ret;
}
EXPORT_SYMBOL(at42qt_read);



 /* + SKC on100820 for i2c read by size */
int at42qt_read_by_size(u16 reg, u8 size, u8* mem)
{
	int ret =0;
	int status =0;
	u8 values[6], retry = 0;
      int i =0; 
      u8 data[2]; 

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	//ret = at42qt_write(AT42QT_PT, reg >> 2);



       data[0] = reg & 0xFF;
	data[1] = reg >> 8;


	ret = i2c_master_send(g_client, data, 2);

      ret = i2c_master_recv(g_client, mem, size); // read  data packet by size
    
	return ret;   /* return how many i2c read */
}
EXPORT_SYMBOL(at42qt_read_by_size);
 /* - SKC on100820 for i2c read by size */





int at42qt_Marvell_read(u16 reg)
{
	int ret;
	int status;
	u8 values[5], retry = 1;

	if (g_client == NULL)	/* No global client pointer? */
		return -1;

	/* Cache read of the at42qtxxxx register */
	if (at42qt_regs[reg].hit) {
		return at42qt_regs[reg].data;
	}
//	ret = at42qt_write(AT42QT_PT, reg >> 2);


    do {
	    ret = i2c_master_recv(g_client, values, 5); // read five byte data packet
	    
 //   	printk("%x,	%x,	%x,	%x,	%x\n", values[0],values[1],values[2],values[3],values[4]);
    	qt5480_pkt_address = values[0]<<2 ;
    	qt_read_burst[0]=values[1];
    	qt_read_burst[1]=values[2];
    	qt_read_burst[2]=values[3];
    	qt_read_burst[3]=values[4];


    } while(--retry);
 
	return status;
}

EXPORT_SYMBOL(at42qt_Marvell_read);


/*
 * Management functions
 */

static int at42qt_set_power_state(struct i2c_client *client, int state)
{
	struct at42qt_data *data = i2c_get_clientdata(client);
	int ret = 0;

	if (state == 0)
	{
		ret = at42qt_write(AT42QT_LM, 0);
	}
	else {
		ret = at42qt_write(AT42QT_LM, 31);
	}

	data->power_state = state;

 	return ret;
}

/*
 * SysFS support
 */

static ssize_t at42qt_show_power_state(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct at42qt_data *data = i2c_get_clientdata(to_i2c_client(dev));

	return sprintf(buf, "%u\n", data->power_state);
}

static ssize_t at42qt_store_power_state(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct at42qt_data *data = i2c_get_clientdata(client);
	unsigned long val = simple_strtoul(buf, NULL, 10);
	int ret;

	if (val < 0 || val > 1)
		return -EINVAL;

	mutex_lock(&data->update_lock);
	ret = at42qt_set_power_state(client, val);
	mutex_unlock(&data->update_lock);

	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(power_state, S_IWUSR | S_IRUGO,
		   at42qt_show_power_state, at42qt_store_power_state);

static struct attribute *at42qt_attributes[] = {
	&dev_attr_power_state.attr,
	NULL
};

static const struct attribute_group at42qt_attr_group = {
	.attrs = at42qt_attributes,
};

/*
 * Initialization function
 */
static int at42qt_init_client(struct i2c_client *client)
{
	struct at42qt_data *data = i2c_get_clientdata(client);
	int ret;

	at42qt_set_power_state(client, 1);
	mdelay(1);

	/*
	 * Probe the chip. To do so we try to
	 * read chipid and get back the 0x40 code
	 */
	if (at42qt_read_org(AT42QT_CI) != AT42QT_CHIPID)
		return -ENODEV;
	data->power_state = 1;

	mdelay(1);
	ret = at42qt_read_org(AT42QT_CV);
	dev_info(&client->dev, "%s chip found, version: %d.%d\n", client->name, (ret >> 4), (ret & 0xf));

 	return 0;
}

/*
 * I2C init/probing/exit functions
 */

static struct i2c_driver at42qt_driver;
static int __devinit at42qt_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct at42qt_data *data;
	int err = 0;
	int i =0;
    
//#ifdef __SKC_TRACE_TOUCH__
//printk("---------------------------------------------------------------------\n");
//printk("[SKC_MULTI] at42qt_probe START\n ");
//printk("---------------------------------------------------------------------\n\n");
//#endif
    
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}

	data = kzalloc(sizeof(struct at42qt_data), GFP_KERNEL);
	if (!data) {
		err = -ENOMEM;
		goto exit;
	}
	data->client = client;
	i2c_set_clientdata(client, data);

	mutex_init(&data->update_lock);

	g_client = client;

	/* Initialize the AT42QTXXXX chip */
    #if 0
	err = at42qt_init_client(client);
	if (err)
	{printk("at42qt_init_client fail : %d\n", err);
		goto exit_kfree;
	}
    #endif

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &at42qt_attr_group);
	if (err)
        {printk("sysfs_create_group fail : %d\n", err);
		goto exit_kfree;
        }

	dev_info(&client->dev, "support ver. %s enabled\n", DRIVER_VERSION);

	return 0;

exit_kfree:
	kfree(data);
exit:
	return err;
}

static int __devexit at42qt_remove(struct i2c_client *client)
{
	sysfs_remove_group(&client->dev.kobj, &at42qt_attr_group);

	/* Power down the device */
	at42qt_set_power_state(client, 0);

	kfree(i2c_get_clientdata(client));

	return 0;
}

#ifdef CONFIG_PM

static int at42qt_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;// _SSENI //at42qt_set_power_state(client, 0);
}

static int at42qt_resume(struct i2c_client *client)
{
	return 0;// _SSENI //at42qt_set_power_state(client, 1);
}

#else

#define at42qt_suspend		NULL
#define at42qt_resume		NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id at42qt_id[] = {
	{ "at42qt", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, at42qt_id);

static struct i2c_driver at42qt_driver = {
	.driver = {
		.name	= AT42QT_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = at42qt_suspend,
	.resume	= at42qt_resume,
	.probe	= at42qt_probe,
	.remove	= __devexit_p(at42qt_remove),
	.id_table = at42qt_id,
};

static int __init at42qt_init(void)
{

//#ifdef __SKC_TRACE_TOUCH__
//printk("---------------------------------------------------------------------\n");
//printk("[SKC_MULTI] at42qt_init START\n ");
//printk("---------------------------------------------------------------------\n\n");
//#endif
    
	return i2c_add_driver(&at42qt_driver);
}

static void __exit at42qt_exit(void)
{
	i2c_del_driver(&at42qt_driver);
}

MODULE_AUTHOR("TsungLung, Yang <tlyang@marvell.com>");
MODULE_DESCRIPTION("Atmel AT42QTXXXX tsp driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(at42qt_init);
module_exit(at42qt_exit);
