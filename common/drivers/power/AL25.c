#ifndef __AL25_C__
#define __AL25_C__

/*
 * Tavor AL25 PMIC Management Routines
 *
 *  linux/i2c/drivers/chips/AL25.c
 *
 *  driver for pxa935 - AL25.c
 *
 *  Copyright (C) 2010, Samsung Electronics
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/power_supply.h>
#include <linux/proc_fs.h>
#include <mach/AL25.h>
#include <mach/spa.h>
#include <plat/mfp.h>
#include <asm/uaccess.h>

#ifdef CONFIG_PXA95x_DVFM
#include <mach/dvfm.h>
#endif
#include <plat/vbus.h> // +SAMSUNG_YOUNGJUN : for ACAT connection
#include <mach/gpio.h>

#ifdef CONFIG_AL25_FUELGAIC_TEST
#include<linux/wakelock.h>
static char al25_fuelgaic_test_charger_exist=0;
static struct wake_lock fuelgaic_suspend_wakeup;
static struct wake_lock fuelgaic_idle_wakeup;
#endif


#ifdef CONFIG_MACH_ALKON
#define MD_AL25_REG_CHGCTRL5_EOCS 		MD_AL25_REG_CHGCTRL5_EOCS_130MA
#define MD_AL25_REG_CHGCTRL4_MBCICHWRCH	MD_AL25_REG_CHGCTRL4_MBCICHWRCH_600MA
#else
#define MD_AL25_REG_CHGCTRL5_EOCS 		MD_AL25_REG_CHGCTRL5_EOCS_100MA
#define MD_AL25_REG_CHGCTRL4_MBCICHWRCH	MD_AL25_REG_CHGCTRL4_MBCICHWRCH_550MA
#endif


#ifdef FEAT_EN_USE_MAX14577
static struct AL25_read_cache   				AL25_regs[AL25_REG_NUM];
static unsigned int 							MAX14577_print 	= 0;  //I2C Read, Write printk.

#define MAX14577_PRINT_WRITE(x,y)		if(MAX14577_print) printk("======> AL25_PRINT_WRITE	: 0x%02X <- 0x%02X \n",y,x)
#define MAX14577_PRINT_READ(x,y)		if(MAX14577_print) printk("======> AL25_PRINT_READ	: 0x%02X -> 0x%02X \n",x,y)
#endif


#if defined(CONFIG_PXA95x_DVFM)
static struct dvfm_lock dvfm_lock = {
	.lock		= SPIN_LOCK_UNLOCKED,
	.dev_idx	= -1,
	.count		= 0,
};

static void set_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	/* Disable D0CS */
	dvfm_disable_op_name("D0CS", dvfm_lock.dev_idx);
	/* Disable D1/D2 mode */
	dvfm_disable_op_name("D1", dvfm_lock.dev_idx);
	dvfm_disable_op_name("D2", dvfm_lock.dev_idx);
		dvfm_disable_op_name("CG", dvfm_lock.dev_idx);
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
	printk(KERN_INFO "set_dvfm_constraint!\n");
}

static void unset_dvfm_constraint(void)
{
	spin_lock_irqsave(&dvfm_lock.lock, dvfm_lock.flags);
	/* Enable D0CS */
	dvfm_enable_op_name("D0CS", dvfm_lock.dev_idx);
	/* Enable D1/D2 mode */
	dvfm_enable_op_name("D1", dvfm_lock.dev_idx);
	dvfm_enable_op_name("D2", dvfm_lock.dev_idx);
		dvfm_enable_op_name("CG", dvfm_lock.dev_idx);
	spin_unlock_irqrestore(&dvfm_lock.lock, dvfm_lock.flags);
	printk(KERN_INFO "un-set_dvfm_constraint\n");
}

static int spa_jig_lpm;
#else
static void set_dvfm_constraint(void) {}
static void unset_dvfm_constraint(void) {}
#endif


/*+++++++++++++++++++++++++++++++++ P R O C FS ++++++++++++++++++++++++++++++++++++*/
#ifdef	CONFIG_PROC_FS
#define AL25_PROC_FILE	"driver/al25"
static struct proc_dir_entry *al25_proc_file;
static int index;

/*
 * (1) cat /proc/driver/AL25
 * will print the value of register located in address of index.
 */

static ssize_t al25_proc_read(struct file *filp,
        char __user *buffer, size_t count, loff_t *offset)
{
	u8			reg_val;
	ssize_t 		len;
	const char 	fmt[] 	= "register 0x%02x: 0x%02x\n";
	const char 	fmtErr[] = "register 0x%02x: failed\n";

	char buf[sizeof(fmt)];

	if ((index < 0) || (index > AL25_REG_NUM))
		return 0;

	AL25_regs[index].hit = 0; /* Force to read but not take from cache */

	if (spa_I2C_Read(MD_AL25_IIC_ADDRESS, index,1,&reg_val))
		len = sprintf(buf, fmt, index, reg_val);
	else
		len = sprintf(buf, fmtErr, index);

	return simple_read_from_buffer(buffer, count, offset, buf, len);
}


/*
 * (1) echo -0x0b > /proc/driver/AL25
 * will set index to 0x0b such that next proc_read will be done from this new index value.
 *
 * (2) echo 0x0b > /proc/driver/AL25
 * will set value of 0x0b for the register in address of index
 *
* */

static ssize_t al25_proc_write(struct file *filp,
		const char *buff, size_t len, loff_t *off)
{
	u8 	reg_val;
	char messages[256], vol[256];

	if (len > 256)
		len = 256;

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	/* 	Calibration Start
	  *	- Disable IRQ
	  *	- Cancel Voltage & Temperature delayed workQueue
	  * 	- Disable charge bit 
	  * 	- Stop charge timer to avoid timer interrupt
	  *	- Stop reChargeTest timer to avoid timer interrupt
	  */
	if ('-' == messages[0]) {
		/* set the register index */
		memcpy(vol, messages+1, len-1);
		index = (int) simple_strtoul(vol, NULL, 16);
		printk(KERN_INFO "\nal25_proc_write:register # was set = 0x%x\n", index);
	}else if	('?' == messages[0]) {
		printk(KERN_INFO "\n register to r/w: echo -0x56 > /proc/driver/al25 \n");
		printk(KERN_INFO " read: cat /proc/driver/al25 \n");
		printk(KERN_INFO " write: echo 0x56 > /proc/driver/al25 \n");
	} else {
		/* set the register value */
		reg_val = (int)simple_strtoul(messages, NULL, 16);
		spa_I2C_Write(MD_AL25_IIC_ADDRESS, index,1,&reg_val);
	}
	return len;
}


static struct file_operations al25_proc_ops = {
	.read = al25_proc_read,
	.write = al25_proc_write,
};


static void create_AL25_proc_file(void)
{
	al25_proc_file = create_proc_entry(AL25_PROC_FILE, 0644, NULL);

	if (al25_proc_file) {
		al25_proc_file->proc_fops = &al25_proc_ops;
	} else
		printk(KERN_INFO "create_AL25_proc_file called!\n");
}

static void remove_AL25_proc_file(void)
{
	remove_proc_entry(AL25_PROC_FILE, NULL);
	printk(KERN_INFO "remove_AL25_proc_file called!\n");
}
#endif
/*------------------------------------------ P R O C FS -----------------------------------------------*/

/**-----------------------------------------------------------------------------
 *
 * SAMSUNG debug
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_Dbg_AddMetric( MAX14577_CurrentState_t   curState )
{
  //
  // Check for rollover of dbg buffer
  // 
  if (v_Max14577_dbgBufferIndex == MAX14577_DBG_BUFFER_MAX_SIZE)
  {
    v_Max14577_dbgBufferIndex = 0;
  }

  v_Max14577_dbgBuffer[v_Max14577_dbgBufferIndex].curState = curState;
  memcpy( &v_Max14577_dbgBuffer[v_Max14577_dbgBufferIndex].regs, 
          &gMD_AL25_I2CReg,
          MD_AL25_REG_MAX );

  #if FEAT_EN_SAMSUNG_API
//    v_Max14577_dbgBuffer[v_Max14577_dbgBufferIndex++].v_rtk_tick = v_Rtk_TickCounter;
//  #else
    {
      static int counter = 0;
      v_Max14577_dbgBuffer[v_Max14577_dbgBufferIndex++].v_rtk_tick = counter++;
    }
  #endif
}


/**-----------------------------------------------------------------------------
 *
 * Debug routine to dump I2C registers
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_Dbg_DumpRegs( void )
{
  #if FEAT_EN_SAMSUNG_API
  printk(KERN_INFO "Reg Dump ->" );
  printk(KERN_INFO "DEVICE:  %02X\n",
                          gMD_AL25_I2CReg[ 0x0 ] );
  printk(KERN_INFO "INT:     %02X %02X %02X\n",
                          gMD_AL25_I2CReg[ 0x1 ],
                          gMD_AL25_I2CReg[ 0x2 ],
                          gMD_AL25_I2CReg[ 0x3 ] );
  printk(KERN_INFO "INTSTAT: %02X %02X %02X\n",
                          gMD_AL25_I2CReg[ 0x4 ],
                          gMD_AL25_I2CReg[ 0x5 ],
                          gMD_AL25_I2CReg[ 0x6 ] );
  printk(KERN_INFO "INTMASK: %02X %02X %02X\n",
                          gMD_AL25_I2CReg[ 0x7 ],
                          gMD_AL25_I2CReg[ 0x8 ],
                          gMD_AL25_I2CReg[ 0x9 ] );
  printk(KERN_INFO "CHGDET:  %02X %02X\n",
                          gMD_AL25_I2CReg[ 0xA ],
                          gMD_AL25_I2CReg[ 0xB ] );
  printk(KERN_INFO "CTRL1-3: %02X %02X %02X\n",
                          gMD_AL25_I2CReg[ 0xC ],
                          gMD_AL25_I2CReg[ 0xD ],
                          gMD_AL25_I2CReg[ 0xE ] );
  printk(KERN_INFO "CHG1-4:  %02X %02X %02X %02X\n",
                          gMD_AL25_I2CReg[ 0x0F ],
                          gMD_AL25_I2CReg[ 0x10 ],
                          gMD_AL25_I2CReg[ 0x11 ],
                          gMD_AL25_I2CReg[ 0x12 ] );
  printk(KERN_INFO "CHG5-7:  %02X %02X %02X\n",
                          gMD_AL25_I2CReg[ 0x13 ],
                          gMD_AL25_I2CReg[ 0x14 ],
                          gMD_AL25_I2CReg[ 0x15 ] );
  #endif
}


/**-----------------------------------------------------------------------------
 *
 * DEBUG ONLY Function
 *   Used to compare resistors in device with local copy in global memory
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_Dbg_CompareRegs( void )
{
  //
  // todo
  // 
}


/**-----------------------------------------------------------------------------
 *
 * Handler for ADC and/or ChargeType interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_Adc_ChgTyp( void )
{
  #if FEAT_EN_TRC_ACCESSORY
  printk(KERN_INFO "MD_AL25_IntHandler_Adc_ChgTyp: ENTER\n" );
  #endif

  //
  // Check AdcLow is HI
  // 
  if ( MD_AL25_obj.intStat.statAdcLow )
  {
    //
    // Notify App
    // 
    MG_AL25_App_NotifyAcc( MD_AL25_obj.intStat.statAdc, 
                           MD_AL25_obj.intStat.statChargerType );
  }
  else if ( MD_AL25_obj.intStat.statAdc == MD_AL25_ADC_GND_ADCLOW )
  {
    MD_AL25_obj.intStat.statAdc = MD_AL25_ADC_GND;

    //
    // Notify App
    // 
    MG_AL25_App_NotifyAcc( MD_AL25_obj.intStat.statAdc, 
                           MD_AL25_obj.intStat.statChargerType );
  }
  else
  {
    //
    // todo: Error
    //
    #if FEAT_EN_SAMSUNG_API
    printk(KERN_INFO "ERROR: MD_AL25_IntHandler_Adc_ChgTyp: AdcLow Lo but ADC not 0\n" );
    #endif
  }
}


/**-----------------------------------------------------------------------------
 *
 * Handler for Dead Battery Charge (DBCHG) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_DbChg( MCS_BOOL_T state )
{
  //
  // Notify App
  // 
  MG_AL25_App_NotifyINT_DbChg( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for Data Contact Detect Timeout (DCD_T) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_DcdT( MCS_BOOL_T state )
{
  //
  // If DCD_T interrupt set, set DCD_EN to disable to clear DCD_T interrupt.
  //   Immediately re-enable DCD_EN
  // 
  if ( state == MCS_TRUE )
  {
    //
    // Clear DCD_EN bit
    // 
    gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] &= ~MD_AL25_M_CDETCTRL_DCDEN;

    MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
                         MD_AL25_REG_CDETCTRL,  
                         1,                     
                         &gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] ); 

    //
    // Set DCD_EN bit
    // 
    gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] |= MD_AL25_REG_CDETCTRL_DCDEN_ENABLE;

    MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
                         MD_AL25_REG_CDETCTRL,  
                         1,                     
                         &gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] ); 
  }

  //
  // Notify App
  // 
  MG_AL25_App_NotifyINT_DcdT( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for ADC Error (ADCERR) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_AdcErr( MCS_BOOL_T state )
{
  //
  // Notify App
  // 
  MG_AL25_App_NotifyINT_AdcError( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for Vb Voltage (VBVOLT) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_VbVolt( MCS_BOOL_T state )
{
  //
  // Notify App
  // 
  MG_AL25_App_NotifyINT_VbVolt( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for Over Voltage Protection (OVP) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_Ovp( MCS_BOOL_T state )
{
  //
  // Notify App
  // 
  MG_AL25_App_NotifyINT_Ovp( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for Charge Detection Running (CHGDETRUN) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_ChgDetRun( MCS_BOOL_T state )
{
  //
  // Notify App
  // 
  MG_AL25_App_NotifyINT_ChgDetRun( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for MbcChgErr (Battery Fast Charging Timer Expire) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_MbcChgErr( MCS_BOOL_T state )
{
  //
  // Notify App
  // 
  MG_AL25_App_NotifyINT_MbcChgErr( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for CgMbc (Charger Power OK) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_CgMbc( MCS_BOOL_T state )
{
  //
  // Notify App
  // 
  MG_AL25_App_NotifyINT_CgMbc( state );
}


/**-----------------------------------------------------------------------------
 *
 * Handler for End of Charge (EOC) interrupt
 *
 * @param state   in : new state of interrupt
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_IntHandler_EOC( MCS_BOOL_T state )
{
	#if FEAT_EN_SAMSUNG_API
	printk(KERN_INFO "MD_AL25_IntHandler_EOC: state = %d\n", state);
	#endif
	
	MG_AL25_App_NotifyINT_EOC( state );
}


/**-----------------------------------------------------------------------------
 *
 * Sets the CHG_TYP_M bit
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_CHG_TYP_M_Set( void )
{
    #if FEAT_EN_SAMSUNG_API
  printk(KERN_INFO "MD_AL25_ChgTypM_Set: ENTER\n" );
  #endif

  //
  // Read register so we can modify CHG_TYP_M bit
  //
  MG_AL25_HW_I2CRead( MD_AL25_IIC_ADDRESS,
                       MD_AL25_REG_CDETCTRL,
                       sizeof( gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] ),
                       &gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] );

  //
  // Enable CHG_TYP_M
  // 
  gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] |= MD_AL25_REG_CDETCTRL_CHGTYPM_ENABLE;

  //
  // Write modified register value
  //
  MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
                        MD_AL25_REG_CDETCTRL,  
                        1,                     
                        &gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] ); 

  MD_AL25_obj.forceChgTypM = MCS_TRUE;

  MD_AL25_AddMetric( MAX14577_ChgTypM_Set );
}


/**-----------------------------------------------------------------------------
 *
 * Read AL25 Interrupt Status registers and returns parameter values
 *
 * NOTE: this will clear AL25 Interrupt
 *
 * @param    pInterrupt   out : structure of interrupt register params
 *
 * @return   TRUE  : Successful
 *           FALSE : Failure
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
MCS_BOOL_T MD_AL25_ReadStatus( MD_AL25_INT_T       *pInt,
                               MD_AL25_INTSTAT_T   *pIntStat )
{

  MG_AL25_HW_I2CRead( MD_AL25_IIC_ADDRESS,
                      MD_AL25_REG_DEVICEID,
                      7,
                      &gMD_AL25_I2CReg[ MD_AL25_REG_DEVICEID ] );

  #if FEAT_EN_TRC_I2C_TRAFFIC
  MI_TRC_3NumericVarsMsg( 0, 
                          "MD_AL25_ReadStatus: INT  0x%02x 0x%02x 0x%02x",
                          gMD_AL25_I2CReg[ MD_AL25_REG_INT1 ],
                          gMD_AL25_I2CReg[ MD_AL25_REG_INT2 ],
                          gMD_AL25_I2CReg[ MD_AL25_REG_INT3 ] );
  MI_TRC_3NumericVarsMsg( 0, 
                          "MD_AL25_ReadStatus: STAT 0x%02x 0x%02x 0x%02x",
                          gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT1 ],
                          gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT2 ],
                          gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT3 ] );
  #endif

  //
  // Interrupt Reg 1
  // 
  pInt->intAdc = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INT1 ] & MD_AL25_M_INT1_ADC    ) >> MD_AL25_B_INT1_ADC    );
  pInt->intAdcLow = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INT1 ] & MD_AL25_M_INT1_ADCLOW ) >> MD_AL25_B_INT1_ADCLOW );
  pInt->intAdcError = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INT1 ] & MD_AL25_M_INT1_ADCERR ) >> MD_AL25_B_INT1_ADCERR );

  //
  // Interrupt Reg 2
  // 
  pInt->intChargerType = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INT2 ] & MD_AL25_M_INT2_CHGTYP    ) >> MD_AL25_B_INT2_CHGTYP    );
  pInt->intChargerDetectRun = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INT2 ] & MD_AL25_M_INT2_CHGDETRUN ) >> MD_AL25_B_INT2_CHGDETRUN );
  pInt->intDataContactDetectTimeout = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INT2 ] & MD_AL25_M_INT2_DCDTMR    ) >> MD_AL25_B_INT2_DCDTMR    );
  pInt->intDeadBatteryChargeMode = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INT2 ] & MD_AL25_M_INT2_DBCHG     ) >> MD_AL25_B_INT2_DBCHG     );
  pInt->intVbVoltage = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INT2 ] & MD_AL25_M_INT2_VBVOLT    ) >> MD_AL25_B_INT2_VBVOLT    );

  //
  // Interrupt Reg 3
  // 
  pInt->intMbcChgErr = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INT3 ] & MD_AL25_M_INT3_MBCCHGERR ) >> MD_AL25_B_INT3_MBCCHGERR );
  pInt->intVbOverVoltageProtection = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INT3 ] & MD_AL25_M_INT3_OVP       ) >> MD_AL25_B_INT3_OVP       );
  pInt->intCgMbc = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INT3 ] & MD_AL25_M_INT3_CGMBC     ) >> MD_AL25_B_INT3_CGMBC     );
  pInt->intEndOfCharge = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INT3 ] & MD_AL25_M_INT3_EOC       ) >> MD_AL25_B_INT3_EOC       );

  //
  // Interrupt Status Reg 1
  // 
  pIntStat->statAdc = ( MD_AL25_ADC_T )
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT1 ] & MD_AL25_M_INTSTAT1_ADC    ) >> MD_AL25_B_INTSTAT1_ADC    );
  pIntStat->statAdcLow = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT1 ] & MD_AL25_M_INTSTAT1_ADCLOW ) >> MD_AL25_B_INTSTAT1_ADCLOW );
  pIntStat->statAdcError = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT1 ] & MD_AL25_M_INTSTAT1_ADCERR ) >> MD_AL25_B_INTSTAT1_ADCERR );

  //
  // Interrupt Status Reg 2
  // 
  pIntStat->statChargerType = ( MD_AL25_CHGTYP_T )
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT2 ] & MD_AL25_M_INTSTAT2_CHGTYP    ) >> MD_AL25_B_INTSTAT2_CHGTYP    );
  pIntStat->statChargerDetectRun = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT2 ] & MD_AL25_M_INTSTAT2_CHGDETRUN ) >> MD_AL25_B_INTSTAT2_CHGDETRUN );
  pIntStat->statDataContactDetectTimeout = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT2 ] & MD_AL25_M_INTSTAT2_DCDTMR    ) >> MD_AL25_B_INTSTAT2_DCDTMR    );
  pIntStat->statDeadBatteryChargeMode = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT2 ] & MD_AL25_M_INTSTAT2_DBCHG     ) >> MD_AL25_B_INTSTAT2_DBCHG     );
  pIntStat->statVbVoltage = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT2 ] & MD_AL25_M_INTSTAT2_VBVOLT    ) >> MD_AL25_B_INTSTAT2_VBVOLT    );

  //
  // Interrupt Status Reg 3
  // 
  pIntStat->statMbcChgErr = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT3 ] & MD_AL25_M_INTSTAT3_MBCCHGERR ) >> MD_AL25_B_INTSTAT3_MBCCHGERR );
  pIntStat->statVbOverVoltageProtection = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT3 ] & MD_AL25_M_INTSTAT3_OVP       ) >> MD_AL25_B_INTSTAT3_OVP       );
  pIntStat->statCgMbc = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT3 ] & MD_AL25_M_INTSTAT3_CGMBC     ) >> MD_AL25_B_INTSTAT3_CGMBC     );
  pIntStat->statEndOfCharge = 
    (( gMD_AL25_I2CReg[ MD_AL25_REG_INTSTAT3 ] & MD_AL25_M_INTSTAT3_EOC       ) >> MD_AL25_B_INTSTAT3_EOC       );


  MD_AL25_AddMetric( MAX14577_ReadStatus );

  return( MCS_TRUE );
}


/*==============================================================================
 *
 *                         E X T E R N A L   M E T H O D S
 *
 *==============================================================================
 */


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_ModuleInit( MD_AL25_USERCFG_T *userCfgData )
{
  MCS_U32_T   index = 0;

#if FEAT_EN_SAMSUNG_API
  printk(KERN_INFO "AL25_ModuleInit\n" );
#endif

  MD_AL25_AddMetric( MAX14577_INIT_STATE );

  //
  // Setup initial parameters of the AL25 object
  // 
  MD_AL25_obj.forceChgTypM   =   MCS_FALSE;
  MD_AL25_obj.intStat.statAdc   =   MD_AL25_ADC_INIT;

  //
  // Setup user config data to default of user choices
  // 
  
 if( userCfgData == MCS_NULL )
  {
    MD_AL25_obj.userCfg.rcps          = MD_AL25_RCPS_DISABLE;
    MD_AL25_obj.userCfg.usbCplnt      = MD_AL25_USBCPLNT_DISABLE;
    MD_AL25_obj.userCfg.sfOutOrd      = MD_AL25_SFOUTORD_NORMAL;
    MD_AL25_obj.userCfg.sfOutAsrt     = MD_AL25_SFOUTASRT_NORMAL;
    MD_AL25_obj.userCfg.lowPwr        = MD_AL25_LOWPWR_DISABLE;

    MD_AL25_obj.userCfg.dchk          = MD_AL25_DCHK_50MS;
    MD_AL25_obj.userCfg.usbOtgCapable = MCS_TRUE;
  }
  else
  {
    MD_AL25_obj.userCfg = *userCfgData;
  }

  //
  // Set global I2C regs to zero
  // 
  for ( index = MD_AL25_REG_MIN ; index < MD_AL25_REG_MAX ; index++ )
  {
    gMD_AL25_I2CReg[ index ] = 0x00;
  }
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_ModuleOpen( void )
{
	
#if FEAT_EN_SAMSUNG_API
	  printk(KERN_INFO "AL25_ModuleOpen\n" );
#endif

 //
  // Read Data Registers BUT leave interrupt registers alone!
  //   They will be read during ServiceStateMachine
  // 

  MG_AL25_HW_I2CRead( MD_AL25_IIC_ADDRESS,
                      MD_AL25_REG_INTMASK1,
                      ( MD_AL25_REG_MAX - MD_AL25_REG_INTMASK1 ),
                      &( gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK1 ] ));

//  MD_AL25_Dbg_DumpRegs();

  // 
  // Setup Default register values
  //
  gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK1 ] =
    (
      MD_AL25_REG_INTMASK1_ADCERR_ENABLE   |
      MD_AL25_REG_INTMASK1_ADCLOW_ENABLE   |
      MD_AL25_REG_INTMASK1_ADC_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK2 ] =
    (
      MD_AL25_REG_INTMASK2_VBVOLT_DISABLE      |
      MD_AL25_REG_INTMASK2_DBCHG_ENABLE        |
      MD_AL25_REG_INTMASK2_DCDTMR_ENABLE       |
      MD_AL25_REG_INTMASK2_CHGDETRUN_DISABLE   |
      MD_AL25_REG_INTMASK2_CHGTYP_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK3 ] =
    (
      MD_AL25_REG_INTMASK3_MBCCHGERR_ENABLE   |
      MD_AL25_REG_INTMASK3_OVP_ENABLE         |
      MD_AL25_REG_INTMASK3_CGMBC_DISABLE       |
      MD_AL25_REG_INTMASK3_EOC_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ] =
    (
      MD_AL25_REG_CDETCTRL_CDPDET_VDP_SRC    |
      MD_AL25_REG_CDETCTRL_DBEXIT_DISABLE    |
      MD_AL25_obj.userCfg.dchk               |
      MD_AL25_REG_CDETCTRL_DCD2SCT_EXIT      |
      MD_AL25_REG_CDETCTRL_DCDEN_ENABLE      |
      MD_AL25_REG_CDETCTRL_CHGTYPM_DISABLE   |
      MD_AL25_REG_CDETCTRL_CHGDETEN_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
    (
      MD_AL25_REG_CTRL1_IDBEN_OPEN    |
      MD_AL25_REG_CTRL1_MICEN_OPEN    |
      MD_AL25_REG_CTRL1_COMP2SW_HIZ   |
      MD_AL25_REG_CTRL1_COMN1SW_HIZ
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
    (
      MD_AL25_obj.userCfg.rcps             |
      MD_AL25_obj.userCfg.usbCplnt         |
      MD_AL25_REG_CTRL2_ACCDET_ENABLE     |
      MD_AL25_REG_CTRL2_SFOUTORD_NORMAL    |
      MD_AL25_REG_CTRL2_SFOUTASRT_NORMAL   |
      MD_AL25_REG_CTRL2_CPEN_DISABLE       |
      MD_AL25_REG_CTRL2_ADCEN_ENABLE       |
      MD_AL25_REG_CTRL2_LOWPWR_DISABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
    (
      MD_AL25_REG_CTRL3_WBTH_3P7V        |
      MD_AL25_REG_CTRL3_ADCDBSET_38P62MS   |
      MD_AL25_REG_CTRL3_BOOTSET_AUTO     |
      MD_AL25_REG_CTRL3_JIGSET_AUTO
    );

  // 
  // todo
  //   Defaults for reg ChgCtrl 1-7
  // 
  MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
                       MD_AL25_REG_INTMASK1,
                       ( MD_AL25_REG_MAX - MD_AL25_REG_INTMASK1 ),
                       &(gMD_AL25_I2CReg[ MD_AL25_REG_INTMASK1 ]) );    

//  MD_AL25_Dbg_DumpRegs();

#ifdef CONFIG_PXA95x_DVFM
	if (dvfm_register("spa_JIG", &spa_jig_lpm)) {
		printk(KERN_ERR "%s %s  dvfm_register failed\n", __FILE__,__func__);
		WARN_ON(1);
		return -1;
	}
#endif

  //
  // Setup all initial conidtions
  // 
#if 0
  {
    MD_AL25_INT_T       newInt;

    MD_AL25_ReadStatus( &newInt, &MD_AL25_obj.intStat );

    MD_AL25_IntHandler_Adc_ChgTyp();
    MD_AL25_IntHandler_DbChg(     MD_AL25_obj.intStat.statDeadBatteryChargeMode    );
    MD_AL25_IntHandler_DcdT(      MD_AL25_obj.intStat.statDataContactDetectTimeout );
    MD_AL25_IntHandler_AdcErr(    MD_AL25_obj.intStat.statAdcError                 );
    MD_AL25_IntHandler_VbVolt(    MD_AL25_obj.intStat.statVbVoltage                );
    MD_AL25_IntHandler_Ovp(       MD_AL25_obj.intStat.statVbOverVoltageProtection  );
    MD_AL25_IntHandler_ChgDetRun( MD_AL25_obj.intStat.statChargerDetectRun         );
    MD_AL25_IntHandler_MbcChgErr( MD_AL25_obj.intStat.statMbcChgErr                );
    MD_AL25_IntHandler_CgMbc(     MD_AL25_obj.intStat.statCgMbc                    );
    //MD_AL25_IntHandler_EOC(       MD_AL25_obj.intStat.statEndOfCharge              );
  }
#endif

#ifdef CONFIG_PROC_FS
	create_AL25_proc_file();
#endif	//CONFIG_PROC_FS

  MD_AL25_AddMetric( MAX14577_ModuleOpen );

  //
  // Set ChgTypM to force a ChgTyp INT to start ServiceStateMachine
  // 
  MD_AL25_CHG_TYP_M_Set();

  MD_AL25_ServiceStateMachine();
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_ModuleClose( void )
{
  //
  // Todo
  //   Set AccDet to 1
  //   Set DBExit to 1
  // 

#ifdef CONFIG_PROC_FS
	remove_AL25_proc_file();
#endif	//CONFIG_PROC_FS

#ifdef CONFIG_PXA95x_DVFM
	if (dvfm_unregister("spa_JIG", &spa_jig_lpm)) {
		printk(KERN_ERR "%s %s  dvfm_unregister failed\n", __FILE__,__func__);
		WARN_ON(1);
	}
#endif


  gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ]   |=   MD_AL25_M_CDETCTRL_DBEXIT;
  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ]      |=   MD_AL25_M_CTRL2_ACCDET;

  MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
                        MD_AL25_REG_CDETCTRL,
                        1,
                        &(gMD_AL25_I2CReg[ MD_AL25_REG_CDETCTRL ]) );    

  MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
                        MD_AL25_REG_CTRL2,
                        1,
                        &(gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ]) );    
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
u8 gv_DBD_AL25_NBD = 0; 

void MD_AL25_ServiceStateMachine( void )
{
  MD_AL25_INT_T       newInt;
  MD_AL25_INTSTAT_T   newIntStat;

  MCS_BOOL_T   done = MCS_FALSE;

  #if FEAT_EN_SAMSUNG_API
  printk(KERN_INFO "*****AL25_SM: Enter\n" );
  #endif

  MD_AL25_AddMetric( MAX14577_ServiceStateMachine );

  gv_DBD_AL25_NBD = 0; 

  do
  {
   printk(KERN_INFO "AL25 gv_DBD_AL25_NBD = %d\n",gv_DBD_AL25_NBD);
  MD_AL25_ReadStatus( &newInt, &newIntStat );

 //  MD_AL25_Dbg_DumpRegs();
	
  //
    // Check to ensure INT line cleared on previous interrupt register 
    //   read during ReadStatus()
    // 
    if ( MG_AL25_HW_ISR_IsAsserted() == MCS_FALSE )
    {
      done = MCS_TRUE;
    }
    else
    {
	    done = MCS_FALSE;
    }
    
    //
  // See if this the very first time thru the algorithm
  // 
  if ( MD_AL25_obj.intStat.statAdc == MD_AL25_ADC_INIT )
  {
    #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "AL25_SM: 1st Time\n" );
    #endif

    //
    // Wait for ChgTyp interrupt before initializing all statuses.
    // 
    if (( newInt.intChargerType ) || ( newIntStat.statAdc == 0x1C) || ( newIntStat.statAdc == 0x18) )
    {
      //
      // If 1st time servicing the ISR and ChgTyp INT is triggered,
      //   set all interrupts to ensure initial conditions are notified.
      // 
      newInt.intAdc                      = MCS_TRUE;
      newInt.intAdcError                 = MCS_TRUE;
      newInt.intAdcLow                   = MCS_TRUE;
      newInt.intCgMbc                    = MCS_FALSE;
      newInt.intChargerDetectRun         = MCS_FALSE;
      newInt.intChargerType              = MCS_TRUE;
      newInt.intDataContactDetectTimeout = MCS_TRUE;
      newInt.intDeadBatteryChargeMode    = MCS_TRUE;
      newInt.intEndOfCharge              = MCS_TRUE;
      newInt.intMbcChgErr                = MCS_TRUE;
      newInt.intVbOverVoltageProtection  = MCS_TRUE;
      newInt.intVbVoltage                = MCS_FALSE;
    }
    else
    {
      #if FEAT_EN_TRC_MAIN_STATE_MACHINE
         printk(KERN_INFO "AL25_SM: 1st Time, No ChgTyp INT\n" );
      #endif

      //
      // Initially, until ChgTyp INT occurs, ignore all other INTs
      // 
      return;
    }
  }

  //
  // Causes for running new accessory detection FW algorithm are
  //   - change in ADC
  //   - change in ChgTyp
  //   - force HW state machine with ChgTypM
  //   - adc Debounce Timer expiration
  // 
  if (( newInt.intAdc                               )   ||
      ( newInt.intChargerType                       )   ||
      ( MD_AL25_obj.forceChgTypM        == MCS_TRUE ))
  {
    //
    // Save new interrupt status
    // 
    MD_AL25_obj.intStat.statAdc         = newIntStat.statAdc;
    MD_AL25_obj.intStat.statAdcLow      = newIntStat.statAdcLow;
    MD_AL25_obj.intStat.statChargerType = newIntStat.statChargerType;
    MD_AL25_obj.forceChgTypM            = MCS_FALSE;

    #if FEAT_EN_TRC_MAIN_STATE_MACHINE
    printk(KERN_INFO  "AL25_SM: run AccId (ADC %d %d, ChgTyp %d), fChgTyp %d\n", 
                            newIntStat.statAdc,
                            newIntStat.statAdcLow,
                            newIntStat.statChargerType,
                            MD_AL25_obj.forceChgTypM );
    #endif

    if ( newIntStat.statChargerDetectRun == MCS_FALSE )
    {
    MD_AL25_IntHandler_Adc_ChgTyp();
  }
    else
    {
      #if FEAT_EN_TRC_MAIN_STATE_MACHINE
	printk(KERN_INFO "AL25_SM: ADC/ChgTyp Wait for ChgDetRun");
//	MC_DIN_ERROR(NO_BLOCKING, 0xff);
      #endif
    }
  }

  //
  // DBCHG Interrupt
  // 
  if ( newInt.intDeadBatteryChargeMode )
  {
    #if FEAT_EN_TRC_MAIN_STATE_MACHINE
    printk(KERN_INFO "AL25_SM: newDBCHG stat %d\n", newIntStat.statDeadBatteryChargeMode );
    #endif

    MD_AL25_obj.intStat.statDeadBatteryChargeMode = newIntStat.statDeadBatteryChargeMode;

    MD_AL25_IntHandler_DbChg( MD_AL25_obj.intStat.statDeadBatteryChargeMode );
  }

  //
  // DCD_T Interrupt
  // 
  if ( newInt.intDataContactDetectTimeout )
  {
    #if FEAT_EN_TRC_MAIN_STATE_MACHINE
    printk(KERN_INFO "AL25_SM: newDCD_T stat %d\n", newIntStat.statDataContactDetectTimeout );
    #endif

    MD_AL25_obj.intStat.statDataContactDetectTimeout = newIntStat.statDataContactDetectTimeout;

    MD_AL25_IntHandler_DcdT( MD_AL25_obj.intStat.statDataContactDetectTimeout );
  }

  //
  // ADC Error Interrupt
  // 
  if ( newInt.intAdcError )
  {
    #if FEAT_EN_TRC_MAIN_STATE_MACHINE
    printk(KERN_INFO "AL25_SM: newAdcErr stat %d\n", newIntStat.statAdcError );
    #endif

    MD_AL25_obj.intStat.statAdcError = newIntStat.statAdcError;

    MD_AL25_IntHandler_AdcErr( MD_AL25_obj.intStat.statAdcError );
  }

  //
  // Vb Voltage Interrupt
  // 
  if ( newInt.intVbVoltage )
  {
    #if FEAT_EN_TRC_MAIN_STATE_MACHINE
    printk(KERN_INFO "AL25_SM: newVbVolt stat %d\n", newIntStat.statVbVoltage );
    #endif

    MD_AL25_obj.intStat.statVbVoltage = newIntStat.statVbVoltage;

    MD_AL25_IntHandler_VbVolt( MD_AL25_obj.intStat.statVbVoltage );
  }

  //
  // Vb Over Voltage Protection Interrupt
  // 
  if ( newInt.intVbOverVoltageProtection )
  {
    #if FEAT_EN_TRC_MAIN_STATE_MACHINE
    printk(KERN_INFO "AL25_SM: newOvp stat %d\n", newIntStat.statVbOverVoltageProtection );
    #endif

    MD_AL25_obj.intStat.statVbOverVoltageProtection = newIntStat.statVbOverVoltageProtection;

    MD_AL25_IntHandler_Ovp( MD_AL25_obj.intStat.statVbOverVoltageProtection );
  }

  //
  // Charger Detection Running Interrupt
  // 
  if ( newInt.intChargerDetectRun )
  {
    #if FEAT_EN_TRC_MAIN_STATE_MACHINE
    printk(KERN_INFO "AL25_SM: newChgDetRun stat %d\n", newIntStat.statChargerDetectRun );
    #endif

    MD_AL25_obj.intStat.statChargerDetectRun = newIntStat.statChargerDetectRun;

    MD_AL25_IntHandler_ChgDetRun( MD_AL25_obj.intStat.statChargerDetectRun );
  }


  //
  // MbcChgErr Interrupt (Battery Fast Charge Timer Expire)
  // 
  if ( newInt.intMbcChgErr )
  {
    #if FEAT_EN_TRC_MAIN_STATE_MACHINE
    printk(KERN_INFO "AL25_SM: newMbcChgErr stat %d\n", newIntStat.statMbcChgErr );
    #endif

    MD_AL25_obj.intStat.statMbcChgErr = newIntStat.statMbcChgErr;

    MD_AL25_IntHandler_MbcChgErr( MD_AL25_obj.intStat.statMbcChgErr );
  }


  //
  // CgMbc Interrupt (Charger Voltage OK)
  // 
  if ( newInt.intCgMbc )
  {
    #if FEAT_EN_TRC_MAIN_STATE_MACHINE
    printk(KERN_INFO "AL25_SM: newCgMbc stat %d\n", newIntStat.statCgMbc );
    #endif

    MD_AL25_obj.intStat.statCgMbc = newIntStat.statCgMbc;

    MD_AL25_IntHandler_CgMbc( MD_AL25_obj.intStat.statCgMbc );
  }


  //
  // End Of Charge (EOC) Interrupt
  // 
  if ( newInt.intEndOfCharge )
  {
    #if FEAT_EN_TRC_MAIN_STATE_MACHINE
    printk(KERN_INFO "AL25_SM: newEOC stat %d\n", newIntStat.statEndOfCharge );
    #endif

    MD_AL25_obj.intStat.statEndOfCharge = newIntStat.statEndOfCharge;

    MD_AL25_IntHandler_EOC( MD_AL25_obj.intStat.statEndOfCharge );
  }

    gv_DBD_AL25_NBD++; 

    if (gv_DBD_AL25_NBD > 200)
    {
//	  MC_DIN_ERROR(BLOCKING, 0xff);    
#if FEAT_EN_SAMSUNG_API
 	 printk(KERN_INFO "\n!!! ERROR : MAX14577 is abnormal !!!\n\n");
#endif
    }	
	  
  } while ( done == MCS_FALSE );

  #if FEAT_EN_SAMSUNG_API
  printk(KERN_INFO "*****AL25_SM: Exit\n" );
  #endif
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_None( void )
{
  #if FEAT_EN_TRC_ACCESSORY
  printk(KERN_INFO "MD_AL25_AccCfg_None: ENTER\n" );
  #endif

  //
  // CTRL1
  //   IdbEn OPEN
  //   MicEn OPEN
  //   Switches OPEN
  // 
  // CTRL2
  //   RCPS userCfg
  //   UsbCplnt userCfg
  //   AccDet 0
  //   SfOutOrd userCfg
  //   SfOutAsrt userCfg
  //   CpEn disable
  //   AdcEn enable
  //   LowPwr usercfg
  // 
  // CTRL3
  //   default
  // 
  // CHGCTRL1
  //   TCHW disable
  // 
  // CHGCTRL2
  //   VCHGR_RC enable //when the battery is low(no power to be booted before charging, under 3.4v), 
  //   MBHOSTEN enable //if MBHOSTEN bit is disabled, there's no way to be charged. Thus, when the accessory is removed, enable this bit.
  // 
  // CHGCTRL3
  //   MBCCVWRC 4.2V
  // 
  // CHGCTRL4
  //   MBCICHWRCL 1
  //   MBCICHWRCH 400mA
  // 
  // CHGCTRL5
  //   EOCS 100mA
  // 
  // CHGCTRL6
  //   AutoStop enable
  // 
  // CHGCTRL7
  //   OTPCGHCVS 7.5V
  // 

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
    (
      MD_AL25_REG_CTRL1_IDBEN_OPEN    |
      MD_AL25_REG_CTRL1_MICEN_OPEN    |
      MD_AL25_REG_CTRL1_COMP2SW_HIZ   |
      MD_AL25_REG_CTRL1_COMN1SW_HIZ
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
    (
      MD_AL25_obj.userCfg.rcps           |
      MD_AL25_obj.userCfg.usbCplnt       |
      MD_AL25_REG_CTRL2_ACCDET_ENABLE   |
      MD_AL25_obj.userCfg.sfOutOrd       |
      MD_AL25_obj.userCfg.sfOutAsrt      |
      MD_AL25_REG_CTRL2_CPEN_DISABLE     |
      MD_AL25_REG_CTRL2_ADCEN_ENABLE     |
      MD_AL25_obj.userCfg.lowPwr
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
    (
      MD_AL25_REG_CTRL3_WBTH_3P7V        |
      MD_AL25_REG_CTRL3_ADCDBSET_38P62MS   |
      MD_AL25_REG_CTRL3_BOOTSET_AUTO     |
      MD_AL25_REG_CTRL3_JIGSET_AUTO
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] =
    (
      MD_AL25_REG_CHGCTRL1_TCHW_DISABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
    (
      MD_AL25_REG_CHGCTRL2_VCHGR_RC_ENABLE   |
      MD_AL25_REG_CHGCTRL2_MBHOSTEN_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =
    (
      MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
    (
      MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
      MD_AL25_REG_CHGCTRL4_MBCICHWRCH
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =
    (
      MD_AL25_REG_CHGCTRL5_EOCS
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =
    (
      MD_AL25_REG_CHGCTRL6_AUTOSTOP_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =
    (
      MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P5V
    );

  MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
                       MD_AL25_REG_CTRL1,
                       10,                     
                       &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));

  MD_AL25_AddMetric( MAX14577_AccCfg_None );
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_DedChgr( void )
{
#if FEAT_EN_TRC_ACCESSORY
  printk(KERN_INFO "MD_AL25_AccCfg_DedChgr: ENTER\n");
#endif

  //
  // CTRL1
  //   IdbEn OPEN
  //   MicEn OPEN
  //   Switches OPEN
  // 
  // CTRL2
  //   RCPS userCfg
  //   UsbCplnt userCfg
  //   AccDet 0
  //   SfOutOrd userCfg
  //   SfOutAsrt userCfg
  //   CpEn disable
  //   AdcEn enable
  //   LowPwr usercfg
  // 
  // CTRL3
  //   default
  // 
  // CHGCTRL1
  //   TCHW disable      (usrCfg? todo)
  // 
  // CHGCTRL2
  //   VCHGR_RC enable
  //   MBHOSTEN enable
  // 
  // CHGCTRL3
  //   MBCCVWRC 4.2V     (usrCfg? todo)
  // 
  // CHGCTRL4
  //   MBCICHWRCL 1      (usrCfg? todo)
  //   MBCICHWRCH 400mA  (usrCfg? todo)
  // 
  // CHGCTRL5
  //   EOCS 100mA         (usrCfg? todo)
  // 
  // CHGCTRL6
  //   AutoStop enable   (usrCfg? todo)
  // 
  // CHGCTRL7
  //   OTPCGHCVS 7.5V    (usrCfg? todo)
  // 

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
    (
      MD_AL25_REG_CTRL1_IDBEN_OPEN    |
      MD_AL25_REG_CTRL1_MICEN_OPEN    |
      MD_AL25_REG_CTRL1_COMP2SW_HIZ   |
      MD_AL25_REG_CTRL1_COMN1SW_HIZ
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
    (
      MD_AL25_obj.userCfg.rcps           |
      MD_AL25_obj.userCfg.usbCplnt       |
      MD_AL25_REG_CTRL2_ACCDET_ENABLE   |
      MD_AL25_obj.userCfg.sfOutOrd       |
      MD_AL25_obj.userCfg.sfOutAsrt      |
      MD_AL25_REG_CTRL2_CPEN_DISABLE     |
      MD_AL25_REG_CTRL2_ADCEN_ENABLE     |
      MD_AL25_obj.userCfg.lowPwr
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
    (
      MD_AL25_REG_CTRL3_WBTH_3P7V        |
      MD_AL25_REG_CTRL3_ADCDBSET_38P62MS   |
      MD_AL25_REG_CTRL3_BOOTSET_AUTO     |
      MD_AL25_REG_CTRL3_JIGSET_AUTO
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] =
    (
      MD_AL25_REG_CHGCTRL1_TCHW_DISABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
    (
      MD_AL25_REG_CHGCTRL2_VCHGR_RC_ENABLE   |
      MD_AL25_REG_CHGCTRL2_MBHOSTEN_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =
    (
      MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
    (
      MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
      MD_AL25_REG_CHGCTRL4_MBCICHWRCH
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =
    (
      MD_AL25_REG_CHGCTRL5_EOCS
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =
    (
      MD_AL25_REG_CHGCTRL6_AUTOSTOP_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =
    (
      MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P5V
    );

  MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
                       MD_AL25_REG_CTRL1,
                       10,                     
                       &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));

  MD_AL25_AddMetric( MAX14577_AccCfg_DedChgr );
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_Usb( void )
{
  #if FEAT_EN_TRC_ACCESSORY
  printk(KERN_INFO "MD_AL25_AccCfg_Usb: ENTER\n" );
  #endif

  //
  // CTRL1
  //   IdbEn OPEN
  //   MicEn OPEN
  //   Switches Usb
  // 
  // CTRL2
  //   RCPS userCfg
  //   UsbCplnt userCfg
  //   AccDet 0
  //   SfOutOrd userCfg
  //   SfOutAsrt userCfg
  //   CpEn enable
  //   AdcEn enable
  //   LowPwr userCfg
  // 
  // CTRL3
  //   default
  // 
  // CHGCTRL1
  //   TCHW 5hr          (usrCfg? todo)
  // 
  // CHGCTRL2
  //   if not USB Compliant
  //     VCHGR_RC enable
  //     MBHOSTEN enable
  //   else
  //     VCHGR_RC disable
  //     MBHOSTEN disable
  // 
  // CHGCTRL3
  //   MBCCVWRC 4.2V     (usrCfg? todo)
  // 
  // CHGCTRL4
  //   MBCICHWRCL 1      (usrCfg? todo)
  //   MBCICHWRCH 400mA  (usrCfg? todo)
  // 
  // CHGCTRL5
  //   EOCS 100mA         (usrCfg? todo)
  // 
  // CHGCTRL6
  //   AutoStop enable   (usrCfg? todo)
  // 
  // CHGCTRL7
  //   OTPCGHCVS 7.5V    (usrCfg? todo)
  // 

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
    (
      MD_AL25_REG_CTRL1_IDBEN_OPEN    |
      MD_AL25_REG_CTRL1_MICEN_OPEN    |
      MD_AL25_REG_CTRL1_COMP2SW_DP2   |
      MD_AL25_REG_CTRL1_COMN1SW_DN1
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
    (
      MD_AL25_obj.userCfg.rcps           |
      MD_AL25_obj.userCfg.usbCplnt       |
      MD_AL25_REG_CTRL2_ACCDET_ENABLE   |
      MD_AL25_obj.userCfg.sfOutOrd       |
      MD_AL25_obj.userCfg.sfOutAsrt      |
      MD_AL25_REG_CTRL2_CPEN_ENABLE      |
      MD_AL25_REG_CTRL2_ADCEN_ENABLE     |
      MD_AL25_obj.userCfg.lowPwr
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
    (
      MD_AL25_REG_CTRL3_WBTH_3P7V        |
      MD_AL25_REG_CTRL3_ADCDBSET_38P62MS   |
      MD_AL25_REG_CTRL3_BOOTSET_AUTO     |
      MD_AL25_REG_CTRL3_JIGSET_AUTO
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] =
    (
      MD_AL25_REG_CHGCTRL1_TCHW_DISABLE
    );

  //
  // If USB Compliant is disabled, 
  //   user wants immediate charging from USB cables
  // 
  // Otherwise, charging is disabled until user enumerates
  //   and asks permission from host to charge
  // 
  if ( MD_AL25_obj.userCfg.usbCplnt == MD_AL25_USBCPLNT_DISABLE )
  {
    gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
      (
        MD_AL25_REG_CHGCTRL2_VCHGR_RC_ENABLE   |
        MD_AL25_REG_CHGCTRL2_MBHOSTEN_ENABLE
      );
  }
  else
  {
    gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
      (
        MD_AL25_REG_CHGCTRL2_VCHGR_RC_DISABLE   |
        MD_AL25_REG_CHGCTRL2_MBHOSTEN_DISABLE
      );
  }

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =
    (
      MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
    (
      MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
      MD_AL25_REG_CHGCTRL4_MBCICHWRCH_450MA
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =
    (
      MD_AL25_REG_CHGCTRL5_EOCS
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =
    (
      MD_AL25_REG_CHGCTRL6_AUTOSTOP_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =
    (
      MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P5V
    );

  MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
                       MD_AL25_REG_CTRL1,
                       10,
                       &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));

  MD_AL25_AddMetric( MAX14577_AccCfg_Usb );
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_DwnStrmChgPort( void )
{
  // 
  // todo
  //   Same as USB?
  // 

  #if FEAT_EN_TRC_ACCESSORY
  printk(KERN_INFO "MD_AL25_AccCfg_DwnStrmChgPort: ENTER\n" );
  #endif
  
  MD_AL25_AccCfg_Usb();
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_Uart( void )
{
  #if FEAT_EN_TRC_ACCESSORY
  printk(KERN_INFO "MD_AL25_AccCfg_Uart: ENTER\n" );
  #endif
  
  //
  // CTRL1
  //   IdbEn OPEN
  //   MicEn OPEN
  //   Switches Uart
  // 
  // CTRL2
  //   RCPS userCfg
  //   UsbCplnt userCfg
  //   AccDet 0
  //   SfOutOrd userCfg
  //   SfOutAsrt userCfg
  //   CpEn enable
  //   AdcEn enable
  //   LowPwr userCfg
  // 
  // CTRL3
  //   default
  // 
  // CHGCTRL1
  //   TCHW 5hr
  // 
  // CHGCTRL2
  //   VCHGR_RC disable
  //   MBHOSTEN disable
  // 
  // CHGCTRL3
  //   MBCCVWRC 4.2V
  // 
  // CHGCTRL4
  //   MBCICHWRCL 1
  //   MBCICHWRCH 400mA
  // 
  // CHGCTRL5
  //   EOCS 100mA
  // 
  // CHGCTRL6
  //   AutoStop enable
  // 
  // CHGCTRL7
  //   OTPCGHCVS 7.5V
  // 

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
    (
      MD_AL25_REG_CTRL1_IDBEN_OPEN    |
      MD_AL25_REG_CTRL1_MICEN_OPEN    |
      MD_AL25_REG_CTRL1_COMP2SW_UT2   |
      MD_AL25_REG_CTRL1_COMN1SW_UT1
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
    (
      MD_AL25_obj.userCfg.rcps           |
      MD_AL25_obj.userCfg.usbCplnt       |
      MD_AL25_REG_CTRL2_ACCDET_ENABLE   |
      MD_AL25_obj.userCfg.sfOutOrd       |
      MD_AL25_obj.userCfg.sfOutAsrt      |
      MD_AL25_REG_CTRL2_CPEN_ENABLE      |
      MD_AL25_REG_CTRL2_ADCEN_ENABLE     |
      MD_AL25_obj.userCfg.lowPwr
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
    (
      MD_AL25_REG_CTRL3_WBTH_3P7V        |
      MD_AL25_REG_CTRL3_ADCDBSET_38P62MS   |
      MD_AL25_REG_CTRL3_BOOTSET_AUTO     |
      MD_AL25_REG_CTRL3_JIGSET_AUTO
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] =
    (
      MD_AL25_REG_CHGCTRL1_TCHW_DISABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
    (
      MD_AL25_REG_CHGCTRL2_VCHGR_RC_ENABLE   |
      MD_AL25_REG_CHGCTRL2_MBHOSTEN_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =
    (
      MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
    (
      MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
      MD_AL25_REG_CHGCTRL4_MBCICHWRCH
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =
    (
      MD_AL25_REG_CHGCTRL5_EOCS
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =
    (
      MD_AL25_REG_CHGCTRL6_AUTOSTOP_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =
    (
      MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P5V
    );

  MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
                       MD_AL25_REG_CTRL1,
                       10,
                       &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));

  MD_AL25_AddMetric( MAX14577_AccCfg_Uart );
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_Audio( void )
{
  #if FEAT_EN_TRC_ACCESSORY
  printk(KERN_INFO "MD_AL25_AccCfg_Audio: ENTER\n" );
  #endif

  //
  // CTRL1
  //   IdbEn OPEN
  //   MicEn Connected
  //   Switches Audio
  // 
  // CTRL2
  //   RCPS disable
  //   UsbCplnt userCfg
  //   AccDet 0
  //   SfOutOrd userCfg
  //   SfOutAsrt userCfg
  //   CpEn enable
  //   AdcEn enable
  //   LowPwr userCfg
  //
  // CTRL3
  //   default
  // 
  // CHGCTRL1
  //   TCHW 5hr
  // 
  // CHGCTRL2
  //   VCHGR_RC disable
  //   MBHOSTEN disable
  // 
  // CHGCTRL3
  //   MBCCVWRC 4.2V
  // 
  // CHGCTRL4
  //   MBCICHWRCL 1
  //   MBCICHWRCH 400mA
  // 
  // CHGCTRL5
  //   EOCS 100mA
  // 
  // CHGCTRL6
  //   AutoStop enable
  // 
  // CHGCTRL7
  //   OTPCGHCVS 7.5V
  // 

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
    (
	    MD_AL25_REG_CTRL1_IDBEN_OPEN    |
      MD_AL25_REG_CTRL1_MICEN_CONN    |
      MD_AL25_REG_CTRL1_COMP2SW_SR2   |
      MD_AL25_REG_CTRL1_COMN1SW_SL1
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
    (
      MD_AL25_REG_CTRL2_RCPS_DISABLE     |
      MD_AL25_obj.userCfg.usbCplnt       |
      MD_AL25_REG_CTRL2_ACCDET_ENABLE   |
      MD_AL25_obj.userCfg.sfOutOrd       |
      MD_AL25_obj.userCfg.sfOutAsrt      |
      MD_AL25_REG_CTRL2_CPEN_ENABLE      |
      MD_AL25_REG_CTRL2_ADCEN_ENABLE     |
      MD_AL25_obj.userCfg.lowPwr
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
    (
      MD_AL25_REG_CTRL3_WBTH_3P7V        |
      MD_AL25_REG_CTRL3_ADCDBSET_38P62MS   |
      MD_AL25_REG_CTRL3_BOOTSET_AUTO     |
      MD_AL25_REG_CTRL3_JIGSET_AUTO
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] =
    (
      MD_AL25_REG_CHGCTRL1_TCHW_DISABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
    (
      MD_AL25_REG_CHGCTRL2_VCHGR_RC_ENABLE   |
      MD_AL25_REG_CHGCTRL2_MBHOSTEN_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =
    (
      MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
    (
      MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
      MD_AL25_REG_CHGCTRL4_MBCICHWRCH
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =
    (
      MD_AL25_REG_CHGCTRL5_EOCS
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =
    (
      MD_AL25_REG_CHGCTRL6_AUTOSTOP_DISABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =
    (
      MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P5V
    );

  MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
                       MD_AL25_REG_CTRL1,
                       10,
                       &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_TTY( void )
{
  #if FEAT_EN_TRC_ACCESSORY
  printk(KERN_INFO "MD_AL25_AccCfg_TTY: ENTER\n" );
  #endif

  //
  // Same as Audio
  // 
  MD_AL25_AccCfg_Audio();
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_FactoryUsb( MCS_BOOL_T boot )
{
  MCS_U8_T bootSet = 0;


  #if FEAT_EN_TRC_ACCESSORY
  printk(KERN_INFO "MD_AL25_AccCfg_FactoryUsb: ENTER\n" );
  #endif

  //
  // CTRL1
  //   IdbEn OPEN
  //   MicEn OPEN
  //   Switches Usb
  // 
  // CTRL2
  //   RCPS userCfg
  //   UsbCplnt userCfg
  //   AccDet 0
  //   SfOutOrd userCfg
  //   SfOutAsrt userCfg
  //   CpEn enable
  //   AdcEn enable
  //   LowPwr userCfg
  //
  // CTRL3
  //   WBth default
  //   ADCDbSet default
  //   BootSet : 01 if boot FALSE, 10 if boot TRUE
  //   JigSet 01
  // 
  // CHGCTRL1
  //   TCHW 5hr
  // 
  // CHGCTRL2
  //   VCHGR_RC disable
  //   MBHOSTEN disable
  // 
  // CHGCTRL3
  //   MBCCVWRC 4.2V
  // 
  // CHGCTRL4
  //   MBCICHWRCL 1
  //   MBCICHWRCH 400mA
  // 
  // CHGCTRL5
  //   EOCS 100mA
  // 
  // CHGCTRL6
  //   AutoStop enable
  // 
  // CHGCTRL7
  //   OTPCGHCVS 7.5V
  // 

  if ( boot == MCS_FALSE )
  {
    bootSet = MD_AL25_REG_CTRL3_BOOTSET_LO;
  }
  else
  {
    bootSet = MD_AL25_REG_CTRL3_BOOTSET_HI;
  }

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
    (
	    MD_AL25_REG_CTRL1_IDBEN_OPEN    |
      MD_AL25_REG_CTRL1_MICEN_OPEN    |
      MD_AL25_REG_CTRL1_COMP2SW_DP2   |
      MD_AL25_REG_CTRL1_COMN1SW_DN1
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
    (
      MD_AL25_obj.userCfg.rcps           |
      MD_AL25_obj.userCfg.usbCplnt       |
      MD_AL25_REG_CTRL2_ACCDET_ENABLE   |
      MD_AL25_obj.userCfg.sfOutOrd       |
      MD_AL25_obj.userCfg.sfOutAsrt      |
      MD_AL25_REG_CTRL2_CPEN_ENABLE      |
      MD_AL25_REG_CTRL2_ADCEN_ENABLE     |
      MD_AL25_obj.userCfg.lowPwr
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
    (
      MD_AL25_REG_CTRL3_WBTH_3P7V        |
      MD_AL25_REG_CTRL3_ADCDBSET_38P62MS   |
      bootSet                            |
      MD_AL25_REG_CTRL3_JIGSET_HI
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] =
    (
      MD_AL25_REG_CHGCTRL1_TCHW_DISABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
    (
	MD_AL25_REG_CHGCTRL2_VCHGR_RC_ENABLE   |
	MD_AL25_REG_CHGCTRL2_MBHOSTEN_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =
    (
      MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
    (
      MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
      MD_AL25_REG_CHGCTRL4_MBCICHWRCH
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =
    (
      MD_AL25_REG_CHGCTRL5_EOCS
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =
    (
      MD_AL25_REG_CHGCTRL6_AUTOSTOP_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =
    (
      MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P5V
    );

  MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
                       MD_AL25_REG_CTRL1,
                       10,
                       &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));

  MD_AL25_AddMetric( MAX14577_AccCfg_FactoryUsb );
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_FactoryUart( MCS_BOOL_T boot )
{
  MCS_U8_T bootSet = 0;


  #if FEAT_EN_TRC_ACCESSORY
  printk(KERN_INFO "MD_AL25_AccCfg_FactoryUart: ENTER\n" );
  #endif
  
  //
  // CTRL1
  //   IdbEn OPEN
  //   MicEn OPEN
  //   Switches Uart
  // 
  // CTRL2
  //   RCPS userCfg
  //   UsbCplnt userCfg
  //   AccDet 0
  //   SfOutOrd userCfg
  //   SfOutAsrt userCfg
  //   CpEn enable
  //   AdcEn enable
  //   LowPwr userCfg
  // 
  // CTRL3
  //   WBth default
  //   ADCDbSet default
  //   BootSet : 01 if boot FALSE, 10 if boot TRUE
  //   JigSet 01
  // 
  // CHGCTRL1
  //   TCHW 5hr
  // 
  // CHGCTRL2
  //   VCHGR_RC disable
  //   MBHOSTEN disable
  // 
  // CHGCTRL3
  //   MBCCVWRC 4.2V
  // 
  // CHGCTRL4
  //   MBCICHWRCL 1
  //   MBCICHWRCH 400mA
  // 
  // CHGCTRL5
  //   EOCS 100mA
  // 
  // CHGCTRL6
  //   AutoStop enable
  // 
  // CHGCTRL7
  //   OTPCGHCVS 7.5V
  // 

  if ( boot == MCS_FALSE )
  {
    bootSet = MD_AL25_REG_CTRL3_BOOTSET_LO;
  }
  else
  {
    bootSet = MD_AL25_REG_CTRL3_BOOTSET_HI;
  }

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] =
    (
      MD_AL25_REG_CTRL1_IDBEN_OPEN    |
      MD_AL25_REG_CTRL1_MICEN_OPEN    |
      MD_AL25_REG_CTRL1_COMP2SW_UT2   |
      MD_AL25_REG_CTRL1_COMN1SW_UT1
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL2 ] =
    (
      MD_AL25_obj.userCfg.rcps           |
      MD_AL25_obj.userCfg.usbCplnt       |
      MD_AL25_REG_CTRL2_ACCDET_ENABLE   |
      MD_AL25_obj.userCfg.sfOutOrd       |
      MD_AL25_obj.userCfg.sfOutAsrt      |
      MD_AL25_REG_CTRL2_CPEN_ENABLE      |
      MD_AL25_REG_CTRL2_ADCEN_ENABLE     |
      MD_AL25_obj.userCfg.lowPwr
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CTRL3 ] =
    (
      MD_AL25_REG_CTRL3_WBTH_3P7V        |
      MD_AL25_REG_CTRL3_ADCDBSET_38P62MS   |
      bootSet                            |
      MD_AL25_REG_CTRL3_JIGSET_HI
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL1 ] =
    (
      MD_AL25_REG_CHGCTRL1_TCHW_DISABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
    (
      MD_AL25_REG_CHGCTRL2_VCHGR_RC_ENABLE   |
      MD_AL25_REG_CHGCTRL2_MBHOSTEN_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL3 ] =
    (
      MD_AL25_REG_CHGCTRL3_MBCCVWRC_4P20V
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL4 ] =
    (
      MD_AL25_REG_CHGCTRL4_MBCICHWRCL_HI      |
      MD_AL25_REG_CHGCTRL4_MBCICHWRCH
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL5 ] =
    (
      MD_AL25_REG_CHGCTRL5_EOCS
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL6 ] =
    (
      MD_AL25_REG_CHGCTRL6_AUTOSTOP_ENABLE
    );

  gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL7 ] =
    (
      MD_AL25_REG_CHGCTRL7_OTPCGHCVS_7P5V
    );

  MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
                       MD_AL25_REG_CTRL1,
                       10,
                       &( gMD_AL25_I2CReg[ MD_AL25_REG_CTRL1 ] ));

  MD_AL25_AddMetric( MAX14577_AccCfg_FactoryUart );
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_AV( MCS_BOOL_T conn )
{
  // 
  // todo
  // 
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg_UsbOtg( void )
{
  // 
  // todo
  // 
}


/**-----------------------------------------------------------------------------
 *
 * see MD_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MD_AL25_AccCfg( void )
{
  //
  // todo: Raw config of an accessory
  // 
}








/* ------------------------------ MD_AL25.c -------------------------------*/













/* -------------------------- MG_AL25_Samsung.c +++++++++++++++++++++++++++*/




/*==============================================================================
 *
 *                          L O C A L   C O N S T A N T S
 *
 *==============================================================================
 */


/**-----------------------------------------------------------------------------
 *
 * Glue code debug message enabling/disabling
 *
 *------------------------------------------------------------------------------
 */
#if defined( MCFG_DEBUG )
  #define   FEAT_EN_TRC_GLUE                          0
  #define   FEAT_EN_TRC_GLUE_NOTIFY_ACC               0
  #define   FEAT_EN_TRC_GLUE_NOTIFY_INT               0
  #define   FEAT_EN_TRC_GLUE_TIMER                    1
  #define   FEAT_EN_TRC_GLUE_I2C                      1
  #define   FEAT_EN_TRC_GLUE_KEYPRESS_STATE_MACHINE   1  
  
  #define   FEAT_EN_TRC_GLUE_SAMSUNG                  1
#endif


/*==============================================================================
 *
 *                            L O C A L   M A C R O S
 *
 *==============================================================================
 */


/**-----------------------------------------------------------------------------
 *
 * Determines if param is Audio Device Type 1 accessory
 *
 * @param __acc   in : accessory in question
 *
 * @return MCS_TRUE  : accessory is Audio Device Type 1
 *         MCS_FALSE : accessory is NOT Audio Device Type 1
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
#define MG_AL25_IsAccessoryAudioType1( __acc )                   \
  (                                                              \
    (( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1     )   ||   \
     ( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S0  )   ||   \
     ( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S1  )   ||   \
     ( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S2  )   ||   \
     ( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S3  )   ||   \
     ( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S4  )   ||   \
     ( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S5  )   ||   \
     ( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S6  )   ||   \
     ( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S7  )   ||   \
     ( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S8  )   ||   \
     ( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S9  )   ||   \
     ( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S10 )   ||   \
     ( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S11 )   ||   \
     ( __acc   ==   MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S12 ))       \
   )


/*==============================================================================
 *
 *                             L O C A L   T Y P E S
 *
 *==============================================================================
 */


/**-----------------------------------------------------------------------------
 *
 * Internal Key Handling State Machine States
 *
 *------------------------------------------------------------------------------
 */
typedef enum {
  MG_AL25_KEYSTATE_NO_KEY_PRESS,       /** No Key is Pressed                         */
  MG_AL25_KEYSTATE_WAIT_TKP,           /** Key Pressed - waiting for tkp             */
  MG_AL25_KEYSTATE_WAIT_TLKP,          /** Key Pressed - waiting for tlkp            */
  MG_AL25_KEYSTATE_WAIT_TLKP_RELEASE   /** Key Pressed - after tklp, waiting release */

} MG_AL25_KEYSTATE_T;


/**-----------------------------------------------------------------------------
 *
 * Glue object parameters
 *
 *------------------------------------------------------------------------------
 */
typedef struct {
  MG_AL25_KEYSTATE_T    curKeyState;
  MG_AL25_KEYEVENT_T    curKeyPressed;
  MG_AL25_USERCFG_T     userCfg;

//  MD_TMR_ID_T           keyTmrId;
  MG_AL25_ACCESSORY_T   currentAcc;
  MG_AL25_ACCESSORY_T   previousAcc;

} MG_AL25_INSTANCE_T;


/*==============================================================================
 *
 *                          L O C A L   P R O T O T Y P E S
 *
 *==============================================================================
 */


/**-----------------------------------------------------------------------------
 *
 * Forward reference required by
 *    - MG_AL25_KeyHandler_TmrExp
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_KeyHandler_SM( MG_AL25_KEYEVENT_T newKeyEvent );


/*==============================================================================
 *
 *                           L O C A L   V A R I A B L E S
 *
 *==============================================================================
 */


/**-----------------------------------------------------------------------------
 *
 * Used to manage the module object.
 *
 *------------------------------------------------------------------------------
 */
MG_AL25_INSTANCE_T   MG_AL25_obj;


/**-----------------------------------------------------------------------------
 *
 * Lookup table takes ADC and CHG_TYP as input to determine accessories.
 *
 * NOTE - NOT Handled by table
 *   - CHG_TYP = 100, 101, 110, 111
 *   - ADC = GND
 *
 * All resistor values from 2.0k - 1000k and Open are handled by this table.
 *
 *------------------------------------------------------------------------------
 */
MG_AL25_ACCESSORY_T
  MG_AL25_AccessoryLookupTable[ MD_AL25_ADC_TABLE_MAX][ 4 ] =
{
                     // CHG_TYP
                     // 000                                      001                                         010                                        011
  /* Gnd+ADCLow */  { MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 2.0k       */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S0,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 2.6k       */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S1,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 3.2k       */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S2,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 4.0k       */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S3,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 4.8k       */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S4,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 6.0k       */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S5,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 8.0k       */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S6,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 10k        */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S7,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 12k        */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S8,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 14.5k      */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S9,         MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 17.3k      */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S10,        MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 20.5k      */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S11,        MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 24k        */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S12,        MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 28.7k      */   { MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 34k        */   { MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 40k        */   { MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 50k        */   { MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 65k        */   { MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 80k        */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_2,                  MG_AL25_ACCESSORY_AUDDEV_TYPE_2,                  MG_AL25_ACCESSORY_AUDDEV_TYPE_2,                 MG_AL25_ACCESSORY_AUDDEV_TYPE_2 },
  /* 102k       */   { MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 121k       */   { MG_AL25_ACCESSORY_TTY_CONVERTER,            MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 150k       */   { MG_AL25_ACCESSORY_UART_NO_CHGR,             MG_AL25_ACCESSORY_UART_MANUAL_CHGR,         MG_AL25_ACCESSORY_UART_AUTO_CHGR,          MG_AL25_ACCESSORY_UART_AUTO_CHGR },
  /* 200k       */   { MG_AL25_ACCESSORY_NONE,                     MG_AL25_ACCESSORY_DEDCHGR_1P8A,             MG_AL25_ACCESSORY_DEDCHGR_1P8A,            MG_AL25_ACCESSORY_DEDCHGR_1P8A },
  /* 255k       */   { MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF,     MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF,     MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF,    MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF }, 
  /* 301k       */   { MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON,      MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON,      MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON,     MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON },  
  /* 365k       */   { MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 442k       */   { MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* 523k       */   { MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF,    MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF,    MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF,   MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF }, 
  /* 619k       */   { MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON,     MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON,     MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON,    MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON },  
  /* 1000k      */   { MG_AL25_ACCESSORY_AUDDEV_TYPE_1,            MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
  /* open       */   { MG_AL25_ACCESSORY_NONE,                     MG_AL25_ACCESSORY_USB,                      MG_AL25_ACCESSORY_USBCHGR,                 MG_AL25_ACCESSORY_DEDCHGR_1P8A },
  /* Gnd-ADCLow */ {   MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                  MG_AL25_ACCESSORY_ILLEGAL,                 MG_AL25_ACCESSORY_ILLEGAL },
};


/*==============================================================================
 *
 *                           L O C A L   M E T H O D S
 *
 *==============================================================================
 */


/**-----------------------------------------------------------------------------
 *
 * DEBUG Routine
 *   - Logs info on newAdc and newChgTyp I2C values
 *
 * @param   newAdc      in : new ADC value
 * @param   newChgTyp   in : new CHGTYP value
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_Dbg_LogNewAcc( MD_AL25_ADC_T    newAdc, 
                            MD_AL25_CHGTYP_T newChgTyp )
{
  switch( newAdc )
  {
  case MD_AL25_ADC_GND:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: USB OTG\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_GND_ADCLOW:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: Audio/Video with Load\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_2K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 2K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_2P6K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 2.6K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_3P2K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 3.2K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_4K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 4K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_4P8K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 4.8K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_6K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 6K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_8K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 8K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_10K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 10K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_12K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 12K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_14K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 14K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_17K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 17K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_20K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 20K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_24K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 24K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_29K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 29K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_34K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 34K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_40K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 40K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_50K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 50K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_65K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 65K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_80K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 80K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_102K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 102K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_121K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 121K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_150K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 150K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_200K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 200K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_255K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 255K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_301K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 301K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_365K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 365K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_442K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 442K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_523K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 523K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_619K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 619K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_1000K:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: 1000K\n" );
      #endif
    }
    break;

  case MD_AL25_ADC_OPEN:
    {

    switch ( newChgTyp )
    {
    case MD_AL25_CHGTYP_NO_VOLTAGE:
      {
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO "+++ NotifyAcc: NONE\n" );
        #endif
      }
      break;

    case MD_AL25_CHGTYP_USB:
      {
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO "+++ NotifyAcc: USB\n" );
        #endif
      }
      break;

    case MD_AL25_CHGTYP_DOWNSTREAM_PORT:
      {
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO "+++ NotifyAcc: Downstream Charging Port\n" );
        #endif
      }
      break;

    case MD_AL25_CHGTYP_DEDICATED_CHGR:
      {
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO "+++ NotifyAcc: Dedicated Charger\n" );
        #endif
      }
      break;

    case MD_AL25_CHGTYP_500MA:
      {
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO "+++ NotifyAcc: 500mA Charger\n" );
        #endif
      }
      break;

    case MD_AL25_CHGTYP_1A:
      {
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO "+++ NotifyAcc: 1A Charger\n" );
        #endif
      }
      break;

    case MD_AL25_CHGTYP_RFU:
      {
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO "+++ NotifyAcc: RFU\n" );
        #endif
      }
      break;

    case MD_AL25_CHGTYP_DB_100MA:
      {
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO "+++ NotifyAcc: Dead Battery 100mA Charger\n" );
        #endif
      }
      break;

    default:
      {
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO "+++ NotifyAcc: ERROR ChgTyp\n" );
        #endif
      }
      break;
    }
    }
    break;

  default:
    {
      #if FEAT_EN_SAMSUNG_API
      printk(KERN_INFO "+++ NotifyAcc: ERROR Adc\n" );
      #endif
    }
    break;
  }
}


/**-----------------------------------------------------------------------------
 *
 * Callback function for expiration of key press handling timer
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_KeyHandler_TmrExp( void )
{
  #if FEAT_EN_SAMSUNG_API
  printk(KERN_INFO "MG_AL25_KeyHandler_TmrExp: call Key State Machine\n" );
  #endif

  MG_AL25_KeyHandler_SM( MG_AL25_KEYEVENT_TIMER_EXPIRE );
}


/**-----------------------------------------------------------------------------
 *
 * Key Handler State Machine
 *
 * State Machine for managing short key press vs long key press
 *
 * @param newKeyEvent   in : events used by state machine
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_KeyHandler_SM( MG_AL25_KEYEVENT_T newKeyEvent )
{
  #if FEAT_EN_SAMSUNG_API
  printk(KERN_INFO 
                          "MG_AL25_KeyHandler_SM: ENTER (curState %d, newKeyEvent %d)\n", 
                          MG_AL25_obj.curKeyState,
                          newKeyEvent );
  #endif

  switch( MG_AL25_obj.curKeyState )
  {
  case MG_AL25_KEYSTATE_NO_KEY_PRESS:
    {
      if ( newKeyEvent == MG_AL25_KEYEVENT_RELEASE )
      {
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO 
                                "MG_AL25_KeyHandler_SM: why here? (state %d, keyEvt %d)\n",
                                MG_AL25_obj.curKeyState,
                                newKeyEvent );
        #endif
      }
      else if ( newKeyEvent != MG_AL25_KEYEVENT_TIMER_EXPIRE )
      {
        MG_AL25_obj.curKeyState = MG_AL25_KEYSTATE_WAIT_TKP;

        MG_AL25_App_NotiftyKey( newKeyEvent, MG_AL25_KEYPRESS_INIT );

        //
        // Start Timer for time = tkp
        // 
        MG_AL25_HW_TimerKeyStart( MG_AL25_obj.userCfg.keyTimerValue_tkp, 
                                  &MG_AL25_KeyHandler_TmrExp );

        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO 
                              "MG_AL25_KeyHandler_SM: Start Tkp Timer (key %d)\n",
                              newKeyEvent );
        #endif
      }
      else
      {
        #if FEAT_EN_SAMSUNG_API
       printk(KERN_INFO 
                                "MG_AL25_KeyHandler_SM: why here? (state %d, keyEvt %d)\n",
                                MG_AL25_obj.curKeyState,
                                newKeyEvent );
        #endif
      }
    }
    break;

  case MG_AL25_KEYSTATE_WAIT_TKP:
    {
      if ( newKeyEvent == MG_AL25_KEYEVENT_TIMER_EXPIRE )
      {
        MG_AL25_obj.curKeyState = MG_AL25_KEYSTATE_WAIT_TLKP;

        //
        // Start Timer for time = tlkp
        //
        MG_AL25_HW_TimerKeyStart( MG_AL25_obj.userCfg.keyTimerValue_tlkp,
                                  MG_AL25_KeyHandler_TmrExp );

        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO
                              "MG_AL25_KeyHandler_SM: Start Tlongkp Timer (key %d)\n",
                              newKeyEvent );
        #endif
      }
      else if ( newKeyEvent == MG_AL25_KEYEVENT_RELEASE )
      {
        MG_AL25_obj.curKeyState = MG_AL25_KEYSTATE_NO_KEY_PRESS;

        //
        // Stop Timer
        // 
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO "MG_AL25_KeyHandler_SM: Stop Timer\n" );
        #endif

        MG_AL25_HW_TimerKeyStop();
      }
      else
      {
        //
        // todo: need to check for same key?
        // 

        //
        // Stop Timer
        // Start Timer for time = tkp
        //
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO "MG_AL25_KeyHandler_SM: Stop Timer\n" );
        #endif

        MG_AL25_HW_TimerKeyStop();

        MG_AL25_HW_TimerKeyStart( MG_AL25_obj.userCfg.keyTimerValue_tkp, 
                                  &MG_AL25_KeyHandler_TmrExp );

        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO
                              "MG_AL25_KeyHandler_SM: Start Tkp Timer (key %d)\n",
                              newKeyEvent );
        #endif

        MG_AL25_App_NotiftyKey( newKeyEvent, MG_AL25_KEYPRESS_INIT );
      }
    }
    break;

  case MG_AL25_KEYSTATE_WAIT_TLKP:
    {
      if ( newKeyEvent == MG_AL25_KEYEVENT_TIMER_EXPIRE )
      {
        MG_AL25_App_NotiftyKey( MG_AL25_obj.curKeyPressed, MG_AL25_KEYPRESS_LONG );

        MG_AL25_obj.curKeyState = MG_AL25_KEYSTATE_WAIT_TLKP_RELEASE;
      }
      else if ( newKeyEvent == MG_AL25_KEYEVENT_RELEASE )
      {
        //
        // Stop Timer
        // 
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO "MG_AL25_KeyHandler_SM: Stop Timer\n" );
        #endif

        MG_AL25_HW_TimerKeyStop();

        MG_AL25_App_NotiftyKey( MG_AL25_obj.curKeyPressed, MG_AL25_KEYPRESS_SHORT );

        MG_AL25_obj.curKeyState = MG_AL25_KEYSTATE_NO_KEY_PRESS;
      }
      else
      {
        //
        // Stop Timer
        // 
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO "MG_AL25_KeyHandler_SM: Stop Timer\n" );
        #endif

        MG_AL25_HW_TimerKeyStop();

        MG_AL25_App_NotiftyKey( MG_AL25_obj.curKeyPressed, MG_AL25_KEYPRESS_SHORT );

        MG_AL25_obj.curKeyState = MG_AL25_KEYSTATE_WAIT_TKP;

        //
        // Start Timer for time = tkp
        // 
        MG_AL25_HW_TimerKeyStart( MG_AL25_obj.userCfg.keyTimerValue_tkp,
                                  &MG_AL25_KeyHandler_TmrExp );

        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO
                              "MG_AL25_KeyHandler_SM: Start Tkp Timer (key %d)\n",
                              newKeyEvent );
        #endif

        MG_AL25_App_NotiftyKey( newKeyEvent, MG_AL25_KEYPRESS_INIT );
      }

    }
    break;

  case MG_AL25_KEYSTATE_WAIT_TLKP_RELEASE:
    {
      if ( newKeyEvent == MG_AL25_KEYEVENT_RELEASE )
      {
        MG_AL25_App_NotiftyKey( MG_AL25_obj.curKeyPressed, MG_AL25_KEYPRESS_RELEASE );

        MG_AL25_obj.curKeyState = MG_AL25_KEYSTATE_NO_KEY_PRESS;
      }
      else if ( newKeyEvent == MG_AL25_KEYEVENT_TIMER_EXPIRE )
      {
        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO 
                                "MG_AL25_KeyHandler_SM: why here? (state %d, keyEvt %d)\n",
                                MG_AL25_obj.curKeyState,
                                newKeyEvent );
        #endif
      }
      else
      {
        //
        // todo: need to check same key press?  or event = keyPress?
        // 
        MG_AL25_App_NotiftyKey( MG_AL25_obj.curKeyPressed, MG_AL25_KEYPRESS_RELEASE );

        MG_AL25_obj.curKeyState = MG_AL25_KEYSTATE_WAIT_TKP;

        //
        // Start Timer for time = tkp
        // 
        MG_AL25_HW_TimerKeyStart( MG_AL25_obj.userCfg.keyTimerValue_tkp,
                                  &MG_AL25_KeyHandler_TmrExp );

        #if FEAT_EN_SAMSUNG_API
        printk(KERN_INFO
                              "MG_AL25_KeyHandler_SM: Start Tkp Timer (key %d)\n",
                              newKeyEvent );
        #endif

        MG_AL25_App_NotiftyKey( newKeyEvent, MG_AL25_KEYPRESS_INIT );
      }
    }
    break;

  default:
    {
      #if FEAT_EN_SAMSUNG_API
//      MI_TRC_Die();
      #endif
    }
    break;
  }

  if ( newKeyEvent != MG_AL25_KEYEVENT_TIMER_EXPIRE )
  {
    MG_AL25_obj.curKeyPressed = newKeyEvent;
  }

  #if FEAT_EN_SAMSUNG_API
  printk(KERN_INFO
                          "MG_AL25_KeyHandler_SM: EXIT (curState %d, curKeyPressed %d)\n", 
                          MG_AL25_obj.curKeyState,
                          MG_AL25_obj.curKeyPressed );
  #endif
}


/**-----------------------------------------------------------------------------
 *
 * Takes action based on incoming accessory
 *   - handles key inputs for Samsung specific key handler
 *   - configures I2C regs based on new accessory
 *
 * @param newAcc   in : new Accessory to config
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_SetAccessory( MG_AL25_ACCESSORY_T newAcc )
{
  #if FEAT_EN_TRC_GLUE_NOTIFY_ACC
  printk(KERN_INFO "*** MG_AL25_SetAccessory: newAcc %d\n", newAcc );
  #endif

  MD_AL25_AddMetric( MAX14577_SetAccessory );

  switch( newAcc )
  {
  case MG_AL25_ACCESSORY_NONE:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_2:
  case MG_AL25_ACCESSORY_ILLEGAL:
    {
      MD_AL25_AccCfg_None();
    }
    break;


  case MG_AL25_ACCESSORY_USB:
    {
      MD_AL25_AccCfg_Usb();
    }
    break;

  case MG_AL25_ACCESSORY_USBCHGR:
    {
      MD_AL25_AccCfg_DwnStrmChgPort();
    }
    break;


  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S0:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S1:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S2:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S3:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S4:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S5:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S6:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S7:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S8:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S9:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S10:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S11:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S12:
    {
      //
      // If previous accessory was any type of Audio Device Type 1, 
      //   do NOT re-write I2C Control data
      //
      if ( MG_AL25_IsAccessoryAudioType1( MG_AL25_obj.currentAcc ) == MCS_FALSE )
      {
        //
        // Previous accessory NOT Audio Device Type 1 device, 
        //   need to configure I2C registers
        // 
        MD_AL25_AccCfg_Audio();

        //
        // Samsung requested that Key Press NOT be notified if 
        //   key pressed during insertion!
        // 
//        if ( newAcc != MD_MAX8929_ACCESSORY_AUDDEV_TYPE_1 )
//        {
//          MD_MAX8929_NotifyKey_Internal( (MD_MAX8929_KEYEVENT_T)newAcc );
//        }
      }
      //
      // Previous Accessory was already some form of Audio Device Type 1 
      //   so skip the IIC writes and just notify button press/release
      // 
      else
      {
        MG_AL25_KeyHandler_SM( (MG_AL25_KEYEVENT_T)newAcc );
      }

    }
    break;

  case MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF:
    {
      MD_AL25_AccCfg_FactoryUsb( MCS_FALSE );
    }
    break;

  case MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON:
    {
      MD_AL25_AccCfg_FactoryUsb( MCS_TRUE );
    }
    break;

  case MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF:
    {
      MD_AL25_AccCfg_FactoryUart( MCS_FALSE );
    }
    break;

  case MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON:
    {
      MD_AL25_AccCfg_FactoryUart( MCS_TRUE );
    }
    break;

  case MG_AL25_ACCESSORY_DEDCHGR_1P8A:
  case MG_AL25_ACCESSORY_DEDCHGR_500MA:
  case MG_AL25_ACCESSORY_DEDCHGR_1A:
  case MG_AL25_ACCESSORY_DEDCHGR_100MA:
    {
      #if FEAT_EN_TRC_GLUE_NOTIFY_ACC
      printk(KERN_INFO "MG_AL25_SetAccessory: setup Chgr x\n" );
      #endif

      MD_AL25_AccCfg_DedChgr();

      // todo: other chargers same as 1.8A ?
    }
    break;

  case MG_AL25_ACCESSORY_UART_NO_CHGR:
  case MG_AL25_ACCESSORY_UART_MANUAL_CHGR:
  case MG_AL25_ACCESSORY_UART_AUTO_CHGR:
    {
      #if FEAT_EN_TRC_GLUE_NOTIFY_ACC
      printk(KERN_INFO "MG_AL25_SetAccessory: setup UART Accessory\n" );
      #endif

      MD_AL25_AccCfg_Uart();
    }
    break;

  case MG_AL25_ACCESSORY_TTY_CONVERTER:
    {
      #if FEAT_EN_TRC_GLUE_NOTIFY_ACC
      printk(KERN_INFO "MG_AL25_SetAccessory: setup TTY Accessory\n" );
      #endif

      MD_AL25_AccCfg_TTY();
    }
    break;

  default:
    {
      #if FEAT_EN_TRC_GLUE_NOTIFY_ACC
      printk(KERN_INFO "MG_AL25_SetAccessory: invalid acc %d\n", newAcc );
//      MI_TRC_Die();
      #endif
    }
    break;
  }
}


/**-----------------------------------------------------------------------------
 *
 * Returns current accessory from Glue object
 *
 * @return   MG_AL25_ACCESSORY_T : current configured accessory
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
MG_AL25_ACCESSORY_T   MG_AL25_GetAccessory( void )
{
  return( MG_AL25_obj.currentAcc );
}


/**-----------------------------------------------------------------------------
 *
 * Returns previous accessory from Glue object
 *
 * @return   MG_AL25_ACCESSORY_T : current configured accessory
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
MG_AL25_ACCESSORY_T   MG_AL25_GetPrevAccessory( void )
{
  return( MG_AL25_obj.previousAcc );
}


/**-----------------------------------------------------------------------------
 *
 * SAMSUNG specific function from AJ86
 *   - STUB added for compile, link and testing 
 *   - Should be removed for customer integration
 *
 * @param   event   in : new Samsung MUS event
 *
 * @reference 
 * - AJ86 driver
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
#if FEAT_EN_USE_SAMSUNG_STUB
extern void mv_usb_vbus_change(int status);
void SAMSUNG_STUB_ChargingMgr_Notifier( Charging_Notifier_t event )
{
    #if FEAT_EN_TRC_GLUE
      printk(KERN_INFO"*** ChargingMgr_Notifier():event[%d]",event); 
    #endif
  
	#if FEAT_EN_SAMSUNG_API
	switch( event )
	{
		case  CHARGINGMGR_NOTIFIER_INVALID_EVENT:
		{
		    #if FEAT_EN_SAMSUNG_API
			printk(KERN_INFO "SAMSUNG: Invalid Event\n" );
			#endif
		}
		break;

		case  CHARGINGMGR_NOTIFIER_TA_ATTACHED:
		{
		    #if FEAT_EN_SAMSUNG_API
			printk(KERN_INFO "SAMSUNG: CHARGINGMGR_NOTIFIER_TA_ATTACHED\n" );
			#endif
			spa_TA_Attached();
    	}
    	break;

		case  CHARGINGMGR_NOTIFIER_TA_DETACHED:
		{
		    #if FEAT_EN_SAMSUNG_API
			printk(KERN_INFO "SAMSUNG: CHARGINGMGR_NOTIFIER_TA_DETACHED\n" );
			#endif
			spa_TA_Detached();
		}
		break;

		case  CHARGINGMGR_NOTIFIER_USB_ATTACHED:
		{
		    #if FEAT_EN_SAMSUNG_API
			printk(KERN_INFO "SAMSUNG: CHARGINGMGR_NOTIFIER_USB_ATTACHED\n" );
			#endif
			spa_USB_Attached();
			set_dvfm_constraint();
//dh0318.lee 110601 enabling_usb_with_dialogPMOC	
#ifdef CONFIG_USB_VBUS_88PM860X
			pxa_vbus_handler(1); // +SAMSUNG_YOUNGJUN : for ACAT connection
#else
 mv_usb_vbus_change(1);
#endif 
		}
		break;

		case  CHARGINGMGR_NOTIFIER_USB_DETACHED:
		{
		    #if FEAT_EN_SAMSUNG_API
			printk(KERN_INFO "SAMSUNG: CHARGINGMGR_NOTIFIER_USB_DETACHED\n" );
			#endif
			spa_USB_Detached();
			unset_dvfm_constraint();
//dh0318.lee 110601 enabling_usb_with_dialogPMOC	
#ifdef CONFIG_USB_VBUS_88PM860X
			pxa_vbus_handler(0); // +SAMSUNG_YOUNGJUN : for ACAT connection
#else
 mv_usb_vbus_change(0);
#endif 
		}
		break;

		case  CHARGINGMGR_NOTIFIER_FACTORY_USB_ATTACHED:
		{
		    #if FEAT_EN_SAMSUNG_API
			printk(KERN_INFO "SAMSUNG: FACTORY USB Attached\n" );
			#endif
		}
		break;

		case  CHARGINGMGR_NOTIFIER_FACTORY_USB_DETACHED:
		{
			#if FEAT_EN_SAMSUNG_API
			printk(KERN_INFO "SAMSUNG: FACTORY USB Detached\n" );
			#endif
		}
		break;

		case  CHARGINGMGR_NOTIFIER_FACTORY_UART_ATTACHED:
		{
		    #if FEAT_EN_SAMSUNG_API
			printk(KERN_INFO "SAMSUNG: FACTORY UARTAttached\n" );
			#endif
#ifdef CONFIG_AL25_FUELGAIC_TEST 
			wake_lock(&fuelgaic_suspend_wakeup);
			wake_lock(&fuelgaic_idle_wakeup);
			if(MD_AL25_obj.intStat.statChargerType)
			{
				if(al25_fuelgaic_test_charger_exist==0)
				{
					spa_TA_Attached();
					al25_fuelgaic_test_charger_exist=1;
				}
			}
			else
			{
				if(al25_fuelgaic_test_charger_exist==1)
				{
					spa_TA_Detached();
					al25_fuelgaic_test_charger_exist=0;
				}
			}
#else
			spa_JIG_Attached();
#endif

#ifdef CONFIG_PXA95x_DVFM
			dvfm_disable_op_name("CG", spa_jig_lpm);
			dvfm_disable_op_name("D1", spa_jig_lpm);
			dvfm_disable_op_name("D2", spa_jig_lpm);
#endif
		}
		break;

		case  CHARGINGMGR_NOTIFIER_FACTORY_UART_DETACHED:
		{
			#if FEAT_EN_SAMSUNG_API
			printk(KERN_INFO "SAMSUNG: FACTORY UARTDetached\n" );
			#endif
#ifdef CONFIG_AL25_FUELGAIC_TEST 
			if(al25_fuelgaic_test_charger_exist==1)
			{
				spa_TA_Detached();
				al25_fuelgaic_test_charger_exist=0;
			}
			wake_unlock(&fuelgaic_suspend_wakeup);
			wake_unlock(&fuelgaic_idle_wakeup);
#else
			spa_JIG_Detached();
#endif

#ifdef CONFIG_PXA95x_DVFM
			dvfm_enable_op_name("CG", spa_jig_lpm);
			dvfm_enable_op_name("D1", spa_jig_lpm);
			dvfm_enable_op_name("D2", spa_jig_lpm);
#endif

		}
		break;

		case  CHARGINGMGR_NOTIFIER_CARKIT_ATTACHED:
		{
		    #if FEAT_EN_SAMSUNG_API	
			printk(KERN_INFO "SAMSUNG: CarKit Attached\n" );
			#endif
		}
		break;

		case  CHARGINGMGR_NOTIFIER_CARKIT_DETACHED:
		{
		    #if FEAT_EN_SAMSUNG_API
			printk(KERN_INFO "SAMSUNG: CarKit Detached\n" );
			#endif
		}
		break;

		case  CHARGINGMGR_NOTIFIER_COMPLETE_CHARGING:
		{
		    #if FEAT_EN_SAMSUNG_API
			printk(KERN_INFO "SAMSUNG: Charge Complete\n" );
			#endif
			spa_EOC_Interrupt();
		}
		break;

		case  CHARGINGMGR_NOTIFIER_STOP_BY_TEMP:
		{
		    #if FEAT_EN_SAMSUNG_API
			printk(KERN_INFO "SAMSUNG: Stop By Temp\n" );
			#endif
		}
		break;

		case  CHARGINGMGR_NOTIFIER_GO_BY_TEMP:
		{
		    #if FEAT_EN_SAMSUNG_API
			printk(KERN_INFO "SAMSUNG: Go By Temp\n" );
			#endif
		}
		break;

		case CHARGINGMGR_NOTIFIER_EARJACK_ATTACHED:
		case CHARGINGMGR_NOTIFIER_EARJACK_DETACHED:
		case CHARGINGMGR_NOTIFIER_SHORTKEY_PRESSED:
		case CHARGINGMGR_NOTIFIER_LONGKEY_PRESSED:
		case CHARGINGMGR_NOTIFIER_LONGKEY_RELEASED:
		   #if FEAT_EN_SAMSUNG_API
			printk(KERN_INFO "SAMSUNG: NOT USED\n" );
		#endif
		break;
			
			
	}
	
	#endif   // FEAT_EN_TRC_GLUE_SAMSUNG
}
#endif   // FEAT_EN_USE_SAMSUNG_STUB



/**-----------------------------------------------------------------------------
 *
 * SAMSUNG specific function from AJ86
 *   - Converts Maxim Accessory ID to Samsung Accessory ID
 *
 * @param   newAccessory   in : Maxim new accessory type
 *
 * @reference 
 * - AJ86 driver
 *
 * @constraints 
 * Standard
 *
 *------------------------------------------------------------------------------
 */
void SAMSUNG_STUB_NotifyNewAccessory( MG_AL25_ACCESSORY_T newAccessory )
{
  MG_AL25_ACCESSORY_T prevAcc = MG_AL25_GetPrevAccessory();

  MD_AL25_AddMetric( MAX14577_NotifyNewAccessory );

  #if FEAT_EN_TRC_GLUE
  printk(KERN_INFO "NotifyNewAccessory: newAcc [%d], preAcc [%d]\n", newAccessory, prevAcc );
  #endif

  
  switch( newAccessory )
  {
  case MG_AL25_ACCESSORY_NONE:
    {
      if (( prevAcc == MG_AL25_ACCESSORY_DEDCHGR_1P8A )  ||  // todo - other ded chgrs?
          (prevAcc == MG_AL25_ACCESSORY_DEDCHGR_500MA) ||
	  (prevAcc == MG_AL25_ACCESSORY_DEDCHGR_1A) ||
	  (prevAcc == MG_AL25_ACCESSORY_DEDCHGR_100MA) )
      {
        SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_TA_DETACHED );
      }
      else if (( prevAcc == MG_AL25_ACCESSORY_USB     )   ||
               ( prevAcc == MG_AL25_ACCESSORY_USBCHGR ))
      {
        SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_USB_DETACHED );
      }
      else if (( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1     )   ||
               ( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S0  )   ||
               ( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S1  )   ||
               ( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S2  )   ||
               ( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S3  )   ||
               ( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S4  )   ||
               ( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S5  )   ||
               ( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S6  )   ||
               ( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S7  )   ||
               ( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S8  )   ||
               ( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S9  )   ||
               ( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S10 )   ||
               ( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S11 )   ||
               ( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S12 ))
      {
        SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_EARJACK_DETACHED );
      }
     else if (( prevAcc == MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF	   )||
		  ( prevAcc == MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON 	))
      {
	  SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_FACTORY_USB_DETACHED );	  
      }
      else if (( prevAcc == MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF 	)||
		  ( prevAcc == MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON	 ))
      {
	  SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_FACTORY_UART_DETACHED );	  
      }
      else if ( prevAcc == MG_AL25_ACCESSORY_AUDDEV_TYPE_2 )
      {
        SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_CARKIT_DETACHED );
      }
      else if ( prevAcc == MG_AL25_ACCESSORY_UNKNOWN ) /* JYKim_20100524: when the TA or USB is plugged out during the early stage of booting up, the previous acc is unknown. */
      {
        SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_USB_DETACHED );
      	}
 
      else
      {
        //
        // Actually no need to notify anything
        // 
        //ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_INVALID_EVENT );
      }
    }
    break;

  case MG_AL25_ACCESSORY_USB:
    {
      SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_USB_ATTACHED );
    }
    break;

  case MG_AL25_ACCESSORY_USBCHGR:
    {
      SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_USB_ATTACHED );
    }
    break;
  
  case MG_AL25_ACCESSORY_DEDCHGR_1P8A:   // todo - other ded chgrs ?
  case MG_AL25_ACCESSORY_DEDCHGR_500MA:
  case MG_AL25_ACCESSORY_DEDCHGR_1A:	
  case MG_AL25_ACCESSORY_DEDCHGR_100MA:	
    {
		/*JYKim_20100520: if the TA is inserted somewhat slowly, it's recognized as USB in the meantime. 
		thus, when the previous acc is not NONE but USB, treat the USB detach first. */
	if (( prevAcc == MG_AL25_ACCESSORY_USB      )   ||
              ( prevAcc == MG_AL25_ACCESSORY_USBCHGR ))
      {
        SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_USB_DETACHED );
      }
	
      SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_TA_ATTACHED );
    }
    break;

  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S0:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S1:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S2:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S3:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S4:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S5:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S6:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S7:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S8:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S9:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S10:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S11:
  case MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S12:
    {
      if (( prevAcc != MG_AL25_ACCESSORY_AUDDEV_TYPE_1     )   &&
          ( prevAcc != MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S0  )   &&
          ( prevAcc != MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S1  )   &&
          ( prevAcc != MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S2  )   &&
          ( prevAcc != MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S3  )   &&
          ( prevAcc != MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S4  )   &&
          ( prevAcc != MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S5  )   &&
          ( prevAcc != MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S6  )   &&
          ( prevAcc != MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S7  )   &&
          ( prevAcc != MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S8  )   &&
          ( prevAcc != MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S9  )   &&
          ( prevAcc != MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S10 )   &&
          ( prevAcc != MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S11 )   &&
          ( prevAcc != MG_AL25_ACCESSORY_AUDDEV_TYPE_1_S12 ))
      {
        SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_EARJACK_ATTACHED );
      }
    }
    break;

  case MG_AL25_ACCESSORY_AUDDEV_TYPE_2:
	 {
	   SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_CARKIT_ATTACHED );
	 }
	 break;
  
   case MG_AL25_ACCESSORY_FACTORY_USB_BOOT_OFF:
   case  MG_AL25_ACCESSORY_FACTORY_USB_BOOT_ON:	 /* STE solution doesn't use boot signal. thus, don't need to treat them seperately */
      {
	   SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_FACTORY_USB_ATTACHED );
      }
      break;
  
   case MG_AL25_ACCESSORY_FACTORY_UART_BOOT_OFF:
   case MG_AL25_ACCESSORY_FACTORY_UART_BOOT_ON:	 /* STE solution doesn't use boot signal. thus, don't need to treat them seperately */
      {
	   SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_FACTORY_UART_ATTACHED );
      }
      break;

  default:
    { 
      //
      // Actually no need to notify anything
      // 
      //SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_INVALID_EVENT );
    }
    break;
  }
}




/*==============================================================================
 *
 *                         E X T E R N A L   M E T H O D S
 *
 *==============================================================================
 */


////////////////////////////////////////////////////////////////////////////////
// 
// GlueAPIs
// 
// These routines pass data and information INTO the Maxim 14561 Driver.  They 
//   are meant to provide Glue code to ->
//  
//   1) Translate type definitions from one architecture to another
//   2) Manage some automatic settings for options not used
//   3) etc
// 
// NOTE: You can also call any public AL25 Driver Function directly!
// 
////////////////////////////////////////////////////////////////////////////////


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
 
void MG_AL25_ModuleInit( void )
{
#ifdef CONFIG_AL25_FUELGAIC_TEST
 wake_lock_init(&fuelgaic_suspend_wakeup, WAKE_LOCK_SUSPEND, "fuelgaic_suspend_wakeups");
 wake_lock_init(&fuelgaic_idle_wakeup, WAKE_LOCK_IDLE, "fuelgaic_idle_wakeups");
#endif

  MG_AL25_obj.currentAcc    =   MG_AL25_ACCESSORY_UNKNOWN;
  MG_AL25_obj.previousAcc   =   MG_AL25_ACCESSORY_UNKNOWN;
  MG_AL25_obj.curKeyState   =   MG_AL25_KEYSTATE_NO_KEY_PRESS;

  //
  // Initialize Timer IDs
  // 
#if FEAT_EN_SAMSUNG_API
  {
  }
#else
  {
  MG_AL25_obj.keyTmrId         =   0;
  MG_AL25_obj.chgEndTmrId      =   0;
  MG_AL25_obj.newAccTmrId      =   0;
  MG_AL25_obj.initTmrId        =   0;
  }
  #endif

  MG_AL25_obj.userCfg.keyTimerValue_tkp  = 300;
  MG_AL25_obj.userCfg.keyTimerValue_tlkp = 700;

  //
  // Allocate Timers
  // 
//  MG_AL25_obj.keyTmrId = 
//    MD_TMR_Allocate( MCS_FUNC_PTR( MD_TMR_CALLBACK_F, &MG_AL25_KeyHandler_TmrExp ));

  //
  // Initialize Driver
  // 
  {
    MD_AL25_USERCFG_T  driverCfg;

    driverCfg.rcps        = MD_AL25_RCPS_DISABLE;
    driverCfg.usbCplnt    = MD_AL25_USBCPLNT_DISABLE;
    driverCfg.sfOutOrd    = MD_AL25_SFOUTORD_NORMAL;
    driverCfg.sfOutAsrt   = MD_AL25_SFOUTASRT_NORMAL;
    driverCfg.lowPwr      = MD_AL25_LOWPWR_DISABLE;
    driverCfg.dchk        = MD_AL25_DCHK_50MS;

    MD_AL25_ModuleInit( &driverCfg );

    MD_AL25_ModuleOpen();

  }

  //
  // Enable Driver ISR
  // 
  MG_AL25_HW_EnableISR();
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
int MG_AL25_HW_I2CRead( MCS_U8_T    devAddr,
                         MCS_U8_T    devReg,
                         MCS_U8_T    numBytesToRead,
                         MCS_U8_T   *pData )
{
	int ret;
	u8  i;

#if FEAT_EN_SAMSUNG_API
	ret =  spa_I2C_Read(devAddr, devReg, numBytesToRead, pData);
#else
#endif

	for ( i=0; i<numBytesToRead; i++)
		MAX14577_PRINT_READ(devReg+i,pData[i]);

	return ret;
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
int MG_AL25_HW_I2CWrite( MCS_U8_T   devAddr,
                          MCS_U8_T   devReg,        
                          MCS_U16_T  numBytesToWrite,
                          MCS_U8_T * pData )        
{

	int ret;
	u8  i;

	for ( i=0; i<numBytesToWrite; i++)
		AL25_regs[devReg+i].hit = 0;

#if FEAT_EN_SAMSUNG_API
	ret =  spa_I2C_Write(devAddr, devReg, numBytesToWrite, pData);
#else
#endif

	if (ret == 0) {
		for ( i=0; i<numBytesToWrite; i++) {
			MAX14577_PRINT_WRITE(pData[i],devReg+i);
			AL25_regs[devReg+i].data = pData[i];
			AL25_regs[devReg+i].hit  = AL25_regs[devReg+i].cachable;
		}
	} 

	return ret;
}
  

/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_HW_EnableISR( void )
{
#if FEAT_EN_SAMSUNG_API
#else
  EXTI_InitTypeDef EXTI_InitStructure;


  /* Configure EXTI Line2 to generate an interrupt on falling edge */  
  EXTI_InitStructure.EXTI_Line    = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;  //Rising; //EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init( &EXTI_InitStructure );
#endif
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_HW_DisableISR( void )
{
#if FEAT_EN_SAMSUNG_API
#else
  EXTI_InitTypeDef EXTI_InitStructure;


  /* Configure EXTI Line2 to generate an interrupt on falling edge */  
  EXTI_InitStructure.EXTI_Line    = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init( &EXTI_InitStructure );
#endif
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_HW_ClearISR( void )
{
#if FEAT_EN_SAMSUNG_API
#else
  EXTI_ClearITPendingBit( EXTI_Line2 );
#endif
}

typedef enum
{
	GPIO_LEVEL_LOW,
	GPIO_LEVEL_HIGH,
	GPIO_LEVEL_IGNOR
} GPIO_LEVEL;

MCS_BOOL_T MG_AL25_HW_ISR_IsAsserted( void )
{
   #if FEAT_EN_SAMSUNG_API
   {
 	GPIO_LEVEL		level;

	level=gpio_get_value(mfp_to_gpio(MFP_PIN_GPIO92));
       if ( level == GPIO_LEVEL_LOW )
		return (MCS_TRUE);
      else  
	  	return (MCS_FALSE);
   }
   #else
   {
	 u8 bitStatus = GPIO_ReadInputDataBit( GPIOC, GPIO_Pin_2 );
   
	 if ( bitStatus == Bit_RESET )
	 {
	   MI_TRC_Msg( 0, "ERROR: ISR_IsAsserted TRUE!" );
	   return( MCS_TRUE );
	 }
	 else
	 {
	   return( MCS_FALSE );
	 }
   }
   #endif
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_HW_TimerKeyStart( MCS_U32_T             timeoutValue, 
                               MG_AL25_TMR_EXP_F    *pCallbackFunc )
{
  #if FEAT_EN_SAMSUNG_API
  printk(KERN_INFO "MG_AL25_HW_TimerKeyStart: timeout %d\n", timeoutValue );
  #endif

  //
  // Start Timer
  //
//  MD_TMR_Schedule( MG_AL25_obj.keyTmrId, timeoutValue, MCS_FALSE );
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_HW_TimerKeyStop( void )
{
  #if FEAT_EN_SAMSUNG_API
  printk(KERN_INFO "MG_AL25_HW_TimerKeyStop: ENTER\n" );
  #endif

  //
  // Stop Timer
  // 
 // MD_TMR_Unschedule( MG_AL25_obj.keyTmrId );
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */         
void MG_AL25_App_NotifyAcc( MD_AL25_ADC_T newAdc, MD_AL25_CHGTYP_T newChgTyp )
{
  MG_AL25_ACCESSORY_T newAcc = 0;

  #if FEAT_EN_SAMSUNG_API
//	printk(KERN_INFO "GLUE(App_NotifyNewAccessory): newAdc 0x%02X, newChgTyp 0x%X\n", newAdc, newChgTyp );
	MG_AL25_Dbg_LogNewAcc( newAdc, newChgTyp );
  #endif

#ifdef CONFIG_AL25_FUELGAIC_TEST
if((newAdc==MD_AL25_ADC_523K) && newChgTyp!=0)
{
	printk("newChgType = %d\n",newChgTyp);
	newChgTyp=1;
}
#endif

  MD_AL25_AddMetric( MAX14577_App_NotifyAcc );


  if (( newChgTyp >  MD_AL25_CHGTYP_DEDICATED_CHGR ) && ( newAdc != MD_AL25_ADC_OPEN )) 
  {
    newAcc = MG_AL25_ACCESSORY_ILLEGAL;
  }
  else if ( newAdc == MD_AL25_ADC_OPEN || newAdc ==MD_AL25_ADC_GND )
  {
    switch ( newChgTyp )
    {
    case  MD_AL25_CHGTYP_NO_VOLTAGE:
      {
        newAcc = MG_AL25_ACCESSORY_NONE;
      }
      break;
    case  MD_AL25_CHGTYP_USB:
      {
        newAcc = MG_AL25_ACCESSORY_USB;
      }
      break;
    case  MD_AL25_CHGTYP_DOWNSTREAM_PORT:
      {
        newAcc = MG_AL25_ACCESSORY_USBCHGR;
      }
      break;
    case  MD_AL25_CHGTYP_DEDICATED_CHGR:
      {
        newAcc = MG_AL25_ACCESSORY_DEDCHGR_1P8A;
      }
      break;
    case  MD_AL25_CHGTYP_500MA:
      {
        newAcc = MG_AL25_ACCESSORY_DEDCHGR_500MA;
      }
      break;
    case  MD_AL25_CHGTYP_1A:
      {
        newAcc = MG_AL25_ACCESSORY_DEDCHGR_1A;
      }
      break;
    case  MD_AL25_CHGTYP_RFU:
      {
        newAcc = MG_AL25_ACCESSORY_ILLEGAL;
      }
      break;
    case  MD_AL25_CHGTYP_DB_100MA:
      {
        newAcc = MG_AL25_ACCESSORY_DEDCHGR_100MA;
      }
      break;
    }
  }
  else
  {
    newAcc = MG_AL25_AccessoryLookupTable[ newAdc ][ newChgTyp ];
  }

  #if FEAT_EN_TRC_GLUE
  printk(KERN_INFO "MD_AL25_App_NotifyAcc():newAcc[%d]", newAcc );
  #endif

  MG_AL25_SetAccessory( newAcc );

  MG_AL25_obj.previousAcc = MG_AL25_obj.currentAcc;
  MG_AL25_obj.currentAcc  = newAcc;

  //
  // Code below was copied from MAX8929 project with Samsung
  //   Used to notify Samsung API of attach/detach events
  // 
  SAMSUNG_STUB_NotifyNewAccessory( newAcc );
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_DcdT( MCS_BOOL_T state )
{
  #if FEAT_EN_TRC_GLUE_NOTIFY_INT
  printk(KERN_INFO "--- App_NotifyINT_DcdT: state %d\n", state );
  #endif

  //
  // Data Contact Detect HW algorithm timed out. HW cannot determine accessory.
  // 
  // User may want to notify of Invalid or Unknown Accessory
  // 
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_AdcError( MCS_BOOL_T state )
{
  #if FEAT_EN_TRC_GLUE_NOTIFY_INT
  printk(KERN_INFO "--- App_NotifyINT_AdcError: state %d\n", state );
  #endif

  //
  // Occurs when the ADC cannot settle on an exact value, the ADC value is 
  //   constantly changing.
  // 
  // User may want to notify of Invalid or Unknown Accessory
  // 
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_DbChg( MCS_BOOL_T state )
{
  #if FEAT_EN_TRC_GLUE_NOTIFY_INT
  printk(KERN_INFO "--- App_NotifyINT_DbChg: state %d\n", state );
  #endif
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_VbVolt( MCS_BOOL_T state )
{
  #if FEAT_EN_TRC_GLUE_NOTIFY_INT
  printk(KERN_INFO "--- App_NotifyINT_VbVolt: state %d\n", state );
  #endif

  //
  // Adequate voltage on Vbus
  // 
  // User typically would ignore or mask this interrupt
  // 
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_Ovp( MCS_BOOL_T state )
{
  #if FEAT_EN_TRC_GLUE_NOTIFY_INT
  printk(KERN_INFO "--- App_NotifyINT_Ovp: state %d\n", state );
  #endif

    spa_OVP_Interrupt(state);

  //
  // Vbus is above Max threshold
  // 
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_ChgDetRun( MCS_BOOL_T state )
{
  #if FEAT_EN_TRC_GLUE_NOTIFY_INT
  printk(KERN_INFO "--- App_NotifyINT_ChgDetRun: state %d\n", state );
  #endif

  //
  // HW Charger Detection State Machine is running.
  // 
  // User typically would ignore or mask this interrupt
  // 
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_MbcChgErr( MCS_BOOL_T state )
{
  #if FEAT_EN_TRC_GLUE_NOTIFY_INT
  printk(KERN_INFO "--- App_NotifyINT_MbcChgErr state %d\n", state );
  #endif

  //
  // Battery has reached normal (fast) charge level 
  // AND 
  // Max fast charge timer expired, HW algorithm never saw current decrease
  // 
  // todo - EOC occur at same time?
  // 
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_CgMbc( MCS_BOOL_T state )
{
  #if FEAT_EN_TRC_GLUE_NOTIFY_INT
  printk(KERN_INFO "--- App_NotifyINT_CgMbc: state %d\n", state );
  #endif

  //
  // Vbus voltage is above threshold to charge
  // 
  // User typically would ignore or mask this interrupt
  // 
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotifyINT_EOC( MCS_BOOL_T state )
{
#if FEAT_EN_SAMSUNG_API
	printk(KERN_INFO "MG_AL25_App_NotifyINT_EOC: state %d\n", state );
#endif
  
	if(state)
	{
	
	    SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_COMPLETE_CHARGING );

  			
		gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] =
		(
			MD_AL25_REG_CHGCTRL2_VCHGR_RC_ENABLE   |
			//MD_AL25_REG_CHGCTRL2_MBHOSTEN_DISABLE
			MD_AL25_REG_CHGCTRL2_MBHOSTEN_ENABLE
		);
		
		
		MG_AL25_HW_I2CWrite( MD_AL25_IIC_ADDRESS,
			MD_AL25_REG_CHGCTRL2,
			1,                     
			&( gMD_AL25_I2CReg[ MD_AL25_REG_CHGCTRL2 ] ));
	}
}


/**-----------------------------------------------------------------------------
 *
 * see MG_AL25.h
 *
 *------------------------------------------------------------------------------
 */
void MG_AL25_App_NotiftyKey( MG_AL25_KEYEVENT_T curKey, 
                             MG_AL25_KEYPRESS_T keyPress )
{
  #if FEAT_EN_TRC_GLUE_NOTIFY_INT
  printk(KERN_INFO "GLUE(MG_AL25_App_NotiftyKey): curKey %d, keyPress %d\n",
                          curKey,
                          keyPress );
  #endif

  if ( curKey == MG_AL25_KEYEVENT_S0_PRESS )
  {
    switch ( keyPress )
    {
    case  MG_AL25_KEYPRESS_RELEASE:
      {
//        SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_LONGKEY_RELEASED );
      }
      break;

    case  MG_AL25_KEYPRESS_SHORT:
      {
 //       SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_SHORTKEY_PRESSED );
      }
      break;

    case  MG_AL25_KEYPRESS_LONG:
      {
//        SAMSUNG_STUB_ChargingMgr_Notifier( CHARGINGMGR_NOTIFIER_LONGKEY_PRESSED );
      }
      break;
    }
  }

  #if FEAT_EN_TRC_GLUE
  {
    char keyName[ 20 ];
    char keyPressName[ 20 ];

    switch ( curKey )
    {
    case  MG_AL25_KEYEVENT_RELEASE:
      {
        strcpy( keyName, "RELEASE" );
      }
      break;
    case  MG_AL25_KEYEVENT_S0_PRESS:
      {
        strcpy( keyName, "S0" );
      }
      break;
    case  MG_AL25_KEYEVENT_S1_PRESS:
      {
        strcpy( keyName, "S1" );
      }
      break;
    case  MG_AL25_KEYEVENT_S2_PRESS:
      {
        strcpy( keyName, "S2" );
      }
      break;
    case  MG_AL25_KEYEVENT_S3_PRESS:
      {
        strcpy( keyName, "S3" );
      }
      break;
    case  MG_AL25_KEYEVENT_S4_PRESS:
      {
        strcpy( keyName, "S4" );
      }
      break;
    case  MG_AL25_KEYEVENT_S5_PRESS:
      {
        strcpy( keyName, "S5" );
      }
      break;
    case  MG_AL25_KEYEVENT_S6_PRESS:
      {
        strcpy( keyName, "S6" );
      }
      break;
    case  MG_AL25_KEYEVENT_S7_PRESS:
      {
        strcpy( keyName, "S7" );
      }
      break;
    case  MG_AL25_KEYEVENT_S8_PRESS:
      {
        strcpy( keyName, "S8" );
      }
      break;
    case  MG_AL25_KEYEVENT_S9_PRESS:
      {
        strcpy( keyName, "S9" );
      }
      break;
    case  MG_AL25_KEYEVENT_S10_PRESS:
      {
        strcpy( keyName, "S10" );
      }
      break;
    case  MG_AL25_KEYEVENT_S11_PRESS:
      {
        strcpy( keyName, "S11" );
      }
      break;
    case  MG_AL25_KEYEVENT_S12_PRESS:
      {
        strcpy( keyName, "S12" );
      }
      break;
    default:
      {
        strcpy( keyName, "ERROR?" );
      }
      break;
    }

    switch ( keyPress )
    {
    case  MG_AL25_KEYPRESS_RELEASE:
      {
        strcpy( keyPressName, "RELEASE" );
      }
      break;
    case  MG_AL25_KEYPRESS_INIT:
      {
        strcpy( keyPressName, "INIT" );
      }
      break;
    case  MG_AL25_KEYPRESS_SHORT:
      {
        strcpy( keyPressName, "SHORT" );
      }
      break;
    case  MG_AL25_KEYPRESS_LONG:
      {
        strcpy( keyPressName, "LONG" );
      }
      break;
    default:
      {
        strcpy( keyPressName, "ERROR?" );
      }
      break;
    }

    printk(KERN_INFO "\nSAMSUNG: new Key %s(%d), keyPress %s(%d)\n",
                            keyName,
                            curKey,
                            keyPressName,
                            keyPress );
  }
  #endif   // FEAT_EN_TRC_GLUE
}





/* -------------------------- MG_AL25_Samsung.c ------------------------------*/


#endif //__AL25_C__
