/*
 *  stc3105_battery.c 
 *  fuel-gauge systems for lithium-ion (Li+) batteries
 *
 *  Copyright (C) 2011 STMicroelectronics.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <mach/stc3105_battery.h>


/* ******************************************************************************** */
/*        CURRENT AND VOLTAGE SENSING PARAMETERS                                    */
/*   TO BE DEFINED ACCORDING TO HARDWARE IMPLEMENTATION                             */
/* -------------------------------------------------------------------------------- */
/*                                                                                  */
#define SENSERESISTOR   20   /* current sense resistor in milliOhms (10min, 100max) */
#define VINRESISTOR   1000   /* VIN serial resistor in Ohms (0min, 5000max)         */
/*                                                                                  */
/* note:  use 0 for VINRESISTOR if no R C filter is used on VIN                     */
/* ******************************************************************************** */


/* ******************************************************************************** */
/*        BATTERY PARAMETERS                                                        */
/*   TO BE DEFINED ACCORDING TO BATTERY CHARACTERISTICS                             */
/* -------------------------------------------------------------------------------- */
/*                                                                                  */
#define BATT_DEF_CAPACITY  1535   /* battery nominal capacity in mAh                */
#define BATT_CHG_VOLTAGE   4100   /* min voltage at the end of the charge (mV)      */
#define BATT_MAX_SOC        100   /* correponding SOC at battery full (%)           */
#define BATT_MIN_VOLTAGE      0   /* voltage under load at battery nearly empty (mV) */
#define BATT_MIN_SOC         10   /* correponding SOC at battery nearly empty (%)   */
#define BATT_INT_RESIST     220   /* nominal internal resistance (milliOhms)        */
/*                                                                                  */
#define BATT_LOW_VOLT_ALM  3400   /* low battery voltage alarm (mV) using IO0 output (set to 0 if not used) */
#define BATT_LOW_SOC_ALM     16   /* low battery SOC alarm (in %) using IO0 output (set to 0 if not used) */
/*                                                                                  */
#define CHG_MIN_CURRENT      50   /* min charge current in mA                       */
#define CHG_END_CURRENT       5   /* end charge current in mA                       */
#define APP_MIN_CURRENT     (-5)  /* minimum application current consumption in mA ( <0 !) */
#define BATT_LOW_CURRENT   (-75)  /* battery max discharge current for OCV measurement in mA (<0) */
#define BATT_RELAX_TIME     600   /* battery relaxation time before OCV measurement in sec */
/*                                                                                  */
/* tables of open-circuit voltage (OCV) in mV and state-of-charge (SOC) levels in % */
/* OCV and SOC tables are in decreasing order, NTABLE define the number of points:  */
#define NTABLE            9   /* number of points in OCV and SOC tables             */
#define OCVTABLE          { 4170, 4060, 3980, 3900, 3810, 3760, 3690, 3630, 3490 }  
#define SOCTABLE          {  100,   90,   80,   65,   50,   25,   10,    3,    0 }  
/* ******************************************************************************** */










/* Private define ------------------------------------------------------------*/

/* enable one of the 2 statements below, depending on actual STC310x device: */
/* #define STC3100 */
#define STC3105


#define GG_VERSION "1.2"

#define STC310x_SLAVE_ADDRESS            0xE0   /* STC310x 8-bit address byte */

/*Address of the STC310x register --------------------------------------------*/
#define STC310x_REG_MODE                 0x00    /* Mode Register             */
#define STC310x_REG_CTRL                 0x01    /* Control and Status Register */
#define STC310x_REG_CHARGE               0x02    /* Gas Gauge Charge Data (2 bytes) */
#define STC310x REG_COUNTER              0x04    /* Number of Conversion (2 bytes) */
#define STC310x_REG_CURRENT              0x06    /* Battery Current (2 bytes) */
#define STC310x_REG_VOLTAGE              0x08    /* Battery Voltage (2 bytes) */
#ifdef STC3100
#define STC310x_REG_TEMPERATURE          0x0A    /* Temperature (2 bytes)     */
#endif
#ifdef STC3105
#define STC310x_REG_SOC_BASE             0x0A    /* SOC base value (2 bytes)     */
#define STC310x_REG_ALARM_SOC            0x0C    /* SOC alarm level (2 bytes)    */
#define STC310x_REG_ALARM_VOLTAGE        0x0E    /* Low voltage alarm level      */
#define STC310x_REG_CURRENT_THRES        0x0F    /* Current threshold for relaxation */
#define STC310x_REG_RELAX_COUNT          0x10    /* Voltage relaxation counter   */
#endif

#define STC310x_REG_ID                   0x18    /* Chip ID (1 byte)       */
#ifdef STC3100
#define STC310x_ID                       0x10    /* STC3100 ID */
#endif
#ifdef STC3105
#define STC310x_ID                       0x12    /* STC3105 ID */
#endif

#define STC310x_REG_RAM                  0x20    /* General Purpose RAM Registers */
#ifdef STC3100
#define RAM_SIZE                         32      /* Total RAM size of STC3100 in bytes */
#endif
#ifdef STC3105
#define RAM_SIZE                         16      /* Total RAM size of STC3105 in bytes */
#endif


#define M_EOC  0x0400       /* GG_EOC mask in STC310x_BattDataTypeDef status word */
#define M_STAT 0x1010       /* GG_RUN & PORDET mask in STC310x_BattDataTypeDef status word */
#define M_RUN  0x0010       /* GG_RUN mask in STC310x_BattDataTypeDef status word */

#define OK 0


/* Battery charge state definition for BattState */
#define  BATT_CHARGING  3
#define  BATT_ENDCHARG  2
#define  BATT_FULCHARG  1
#define  BATT_IDLE      0
#define  BATT_DISCHARG (-1)
#define  BATT_LOWBATT  (-2)

/* STC310x RAM test word */
#define RAM_TSTWORD 0x53A9

/* Gas gauge states */
#define GG_INIT     'I'
#define GG_RUNNING  'R'
#define GG_POWERDN  'D'





/* gas gauge structure definition ------------------------------------*/

#ifndef TEST_DRIVER
typedef struct  {
  int Voltage;        /* battery voltage in mV */
  int Current;        /* battery current in mA */
  int Temperature;    /* battery temperature in 0.1°C */
  int ChargeValue;    /* battery absolute state-of-charge value (mAh) */
  int ChargePercent;  /* battery relative state-of-charge value (%) */
  int RemTime;        /* battery remaining operating time during discharge (min) */
  int ChargeNominal;  /* battery full charge capacity (mAh) */
  signed char State;  /* charge (>0)/discharge(<0) state */
} GasGauge_DataTypeDef;
#endif

/* Private constants ---------------------------------------------------------*/

static const char SocPoint[NTABLE] = SOCTABLE;
static const int  OcvPoint[NTABLE] = OCVTABLE;


/* Private variables ---------------------------------------------------------*/

/* structure of the STC310x battery monitoring data */
typedef struct  {
  short Voltage;     /* voltage in mV            */
  short Current;     /* current in mA            */
  short Temperature; /* temperature in 0.1°C     */
  short Charge;      /* Coulomb counter charge in mA.h   */
  short ConvCounter; /* convertion counter       */
  short RelaxTimer;
  short Status;      /* status word              */
} STC310x_BattDataTypeDef;

static STC310x_BattDataTypeDef BattData;   /* STC310x data */

/* Gas Gauge local variables */
static int BattState;               /* battery GG state              */
static int BattChargeNominal;       /* battery nominal charge (mAh)  */
static int BattChargeValue;         /* battery charge value (mAh)    */
static char BattFullCharge;         /* full charge done              */
static char BattFullDischarge;      /* full discharge done           */
static char BattCalStat;            /* capacity calibration done     */
static int  BattRelaxTime;          /* battery relaxation time in 0.5s units */
static int  CountDiff;              /* temp variable */
static int  RelaxC;                 /* relaxation filter counter */
static int  BattAvgOCV;             /* battery averaged OCV */

static int is_attached;


/* structure of the STC310x RAM registers for the Gas Gauge algorithm data */
static union {
  unsigned char db[RAM_SIZE];  /* last byte holds the CRC */
  struct {
    short int TstWord;     /* 0-1 */
    short int BattCap;     /* 2-3 */
    short int BaseCharge;  /* 4-5 */
    short int LastCharge;  /* 6-7 */
    short int LastCount;   /* 8-9 */
    char Status;           /* 10  */
    /* bytes ..RAM_SIZE-2 are free, last byte RAM_SIZE-1 is the CRC */
  } reg;
} GG_Ram;







#define STC3100_DELAY		1000
#define STC3100_BATTERY_FULL	95



struct stc31xx_chip {
	struct i2c_client		*client;
};


/* -------------------------------------------------------------------------- */
/* I2C interface */

static struct stc31xx_chip *sav_client = NULL;

static int STC310x_Write(int length, int reg , unsigned char *values)
{
	struct i2c_client *client = sav_client->client;
	int ret;

	ret = i2c_smbus_write_i2c_block_data(client, reg, length, values);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	return ret;
}


static int STC310x_Read(int length, int reg , unsigned char *values)
{
	struct i2c_client *client = sav_client->client;
	int ret;

	ret = i2c_smbus_read_i2c_block_data(client, reg, length, values);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	return ret;
}


/* -------------------------------------------------------------------------- */
/* STC310x gas gauge algorithm code */




/*******************************************************************************
* Function Name  : STC310x_ReadByte
* Description    : utility function to read the value stored in one register
* Input          : RegAddress: STC310x register,
* Return         : 8-bit value, or 0 if error
*******************************************************************************/
static int STC310x_ReadByte(int RegAddress)
{
  int value;
  unsigned char data[2];
  int res;

  res=STC310x_Read(1, RegAddress, data);

  if (res >= 0)
  {
    /* no error */
    value = data[0];
  }
  else
    value=0;

  return(value);
}



/*******************************************************************************
* Function Name  : STC310x_WriteByte
* Description    : utility function to write a 8-bit value into a register
* Input          : RegAddress: STC310x register, Value: 8-bit value to write
* Return         : error status (OK, !OK)
*******************************************************************************/
static int STC310x_WriteByte(int RegAddress, unsigned char Value)
{
  int res;
  unsigned char data[2];

  data[0]= Value; 
  res = STC310x_Write(1, RegAddress, data);

  return(res);

}







/*******************************************************************************
* Function Name  : STC310x_Startup
* Description    :  initialize and start the STC310x at application startup
* Input          : None
* Return         : 0 is ok, 1 if STC310x reset, -1 if error
*******************************************************************************/
static int STC310x_Startup(void)
{
  int res;

  /* first, check the presence of the STC310x by reading first byte of dev. ID */
  res = STC310x_ReadByte(STC310x_REG_ID);
  if (res!= STC310x_ID) return (-1);

  /* read REG_CTRL to detect possible reset event */
  res = STC310x_ReadByte(STC310x_REG_CTRL);
   
  /* write 0x03 into the REG_CTRL to reset the accumulator and counter and clear the PORDET bit (IO0 pin open), */
  STC310x_WriteByte(STC310x_REG_CTRL, 0x13);
  STC310x_WriteByte(STC310x_REG_CTRL, 0x03);
   
  /* then 0x10 into the REG_MODE register to start the STC3100 in 14-bit resolution mode. */
  STC310x_WriteByte(STC310x_REG_MODE, 0x10);

  if (res&0x10) return (1); /* reset detected */
  return (0);  /* ok */
}


#ifdef STC3105
/*******************************************************************************
* Function Name  : STC3105_Setup
* Description    :  initialize and start the STC3105 at application startup
* Input          : None
* Return         : 0 is ok, 1 if STC310x reset, -1 if error
*******************************************************************************/
static int STC3105_Setup(void)
{
  int v;

  /* is the alarm function to be used? */
  /* low voltage alarm threshold */
  /* threshold register LSB is: 2.44mV*8 = 19.52mV */
  v = (long) BATT_LOW_VOLT_ALM  * 100 / 1952;
  STC310x_WriteByte(STC310x_REG_ALARM_VOLTAGE, v);

  /* low SOC alarm threshold */
  /* threshold register is a positive value and LSB is: 6.7uVh/R */
  v = (long) GG_Ram.reg.BattCap * BATT_LOW_SOC_ALM * SENSERESISTOR  / 670;
  STC310x_WriteByte(STC310x_REG_ALARM_SOC, v&255);
  STC310x_WriteByte(STC310x_REG_ALARM_SOC+1, v>>8);

  /* SOC base register */
  /*  LSB is: 6.7uVh/R */
  v = (long) GG_Ram.reg.BaseCharge * SENSERESISTOR * 100 / 670;
  STC310x_WriteByte(STC310x_REG_SOC_BASE, v&255);
  STC310x_WriteByte(STC310x_REG_SOC_BASE+1, v>>8);

  /* relaxation timer: */
  /* threshold register is a positive value (1-255) and LSB is: 11.77uV/R*8 = 94.16uV/R */
  v = (long) (-BATT_LOW_CURRENT) * SENSERESISTOR * 100 / 9416;
  if (v>255) v = 255 ;
  STC310x_WriteByte(STC310x_REG_CURRENT_THRES, v);

  /* then 0x10 into the REG_MODE register to start the STC3105 in active mode, alarm disabled */
  STC310x_WriteByte(STC310x_REG_MODE, 0x10);
  
  return (0);  /* ok */
}
#endif



/*******************************************************************************
* Function Name  : STC310x_Powerdown
* Description    :  stop the STC310x at application power down
* Input          : None
* Return         : error status (OK, !OK)
*******************************************************************************/
static int STC310x_Powerdown(void)
{
  int res;
  
  /* write 0x01 into the REG_CTRL to release IO0 pin open, */
  STC310x_WriteByte(STC310x_REG_CTRL, 0x01);

  /* write 0 into the REG_MODE register to put the STC310x in standby mode */
   res = STC310x_WriteByte(STC310x_REG_MODE, 0);
   if (res!= OK) return (res);

   return (OK);
}




/* -------------------------------------------------------------------------- */


#define CurrentFactor  (48210/SENSERESISTOR)        /* LSB=11.77uV/R= ~48210/R/4096 - convert to mA  */
#define ChargeCountFactor  (27443/SENSERESISTOR)    /* LSB=6.7uVh/R ~27443/R/4096 - converter to mAh */
#define VoltageFactor  9994                         /* LSB=2.44mV ~9994/4096 - convert to mV         */
#define TemperatureFactor  5120                     /* LSB=0.125°C ~5120/4096 - convert to 0.1°C     */

#define VinResFactor  (VINRESISTOR/100)             /* VINRESISTOR/400000 = ~(VINRESISTOR/100)/4096  */
#define SenseResFactor (SENSERESISTOR*4)            /* SENSERESISTOR/1000 = ~(SENSERESISTOR*4)/4096  */


/*******************************************************************************
* Function Name  : conv
* Description    : conversion utility 
*  convert a raw 16-bit value from STC310x registers into user units (mA, mAh, mV, °C)
*  (optimized routine for efficient operation on 8-bit processors such as STM8)
* Input          : value, factor
* Return         : result = value * factor / 4096
*******************************************************************************/
static int conv(short value, unsigned short factor)
{
  return( ( (long) value * factor ) >> 12 );
}



/*******************************************************************************
* Function Name  : STC310x_ReadBatteryData
* Description    :  utility function to read the battery data from STC310x
*                  to be called every 5s or so
* Input          : ref to BattData structure
* Return         : error status (OK, !OK)
*******************************************************************************/
static int STC310x_ReadBatteryData(STC310x_BattDataTypeDef *BattData)
{
  unsigned char data[12];
  int res;
  int value;
		
  /* read STC310x registers 0 to 11 */
  res=STC310x_Read(12, 0, data);
  if (res<0) return(res);  /* read failed */
	
  /* fill the battery status data */
  /* STC310x status */
  value=data[1]; value = (value<<8) + data[0];
  BattData->Status = value;

  /* charge count */
  value=data[3]; value = (value<<8) + data[2];
  BattData->Charge = conv(value,ChargeCountFactor);  /* result in mAh */

  /* conversion counter */
  value=data[5]; value = (value<<8) + data[4];
  BattData->ConvCounter = value;

  /* current */
  value=data[7]; value = (value<<8) + data[6];
  value &= 0x3fff;   /* mask unused bits */
  if (value>=0x2000) value -= 0x4000;  /* convert to signed value */
  BattData->Current = conv(value,CurrentFactor);  /* result in mA */

  /* voltage */
  value=data[9]; value = (value<<8) + data[8];
  value &= 0x0fff; /* mask unused bits */
  if (value>=0x0800) value -= 0x1000;  /* convert to signed value */
  value = conv(value,VoltageFactor);  /* result in mV */
#ifdef STC3100
  /* correction for the resistor at VIN pin (RC filter): */
  value += conv(value, VinResFactor);                /* i.e. value += (long) value * VINRESISTOR / 400000; */
#endif
  /* correction for the voltage drop in Rsense: */
  value -= conv(BattData->Current, SenseResFactor);  /* i.e. value -= (long) (BattData->Current) * SENSERESISTOR / 1000; */
  BattData->Voltage = value;  /* result in mV */

#ifdef STC3100  
  /* temperature */
  value=data[11]; value = (value<<8) + data[10];
  value &= 0x0fff;   /* mask unused bits */
  if (value>=0x0800) value -= 0x1000;  /* convert to signed value */
  BattData->Temperature = conv(value,TemperatureFactor);  /* result in 0.1°C */
#else
  BattData->Temperature = 220;   /* assume normal ambient temperature (22°C)*/
#endif

#ifdef STC3105
  res=STC310x_Read(1, STC310x_REG_RELAX_COUNT, data);
  if (res<0) return(res);  /* read failed */
  value = data[0]; 
  BattData->RelaxTimer = value*8;  /* result in sec */
#else
  BattData->RelaxTimer = 0;
#endif

  return(OK);
}




/*******************************************************************************
* Function Name  : STC310x_ReadRamData
* Description    : utility function to read the RAM data from STC310x
* Input          : ref to RAM data array
* Return         : error status (OK, !OK)
*******************************************************************************/
static int STC310x_ReadRamData(unsigned char *RamData)
{
  return(STC310x_Read(RAM_SIZE, STC310x_REG_RAM, RamData));
}


/*******************************************************************************
* Function Name  : STC310x_WriteRamData
* Description    : utility function to write the RAM data into STC310x
* Input          : ref to RAM data array
* Return         : error status (OK, !OK)
*******************************************************************************/
int STC310x_WriteRamData(unsigned char *RamData)
{
  return(STC310x_Write(RAM_SIZE, STC310x_REG_RAM, RamData));
}



/******************************************************************************* 
* Function Name  : OCV2Cap
* Description    : Finds the %age of battery depending on the open circuit 
                   voltage of the battery
* Input          : Open Circuit Voltage
* Return         : relative capacity (0-100%)
*******************************************************************************/
static int OCV2Cap(int voltage)
{  
  int index;
  int capacity;
  
  if (voltage >= OcvPoint[0])
    capacity = 100;
  else if (voltage <= OcvPoint[NTABLE-1])
    capacity = 0;
  else
  {
    /*  find voltage interval */
    for (index= 1;index<NTABLE;index++)
      if (voltage > OcvPoint[index]) break;
    /*  interpolate capacity */
    capacity = (SocPoint[index-1] - SocPoint[index]) * (voltage - OcvPoint[index]) / (OcvPoint[index-1] - OcvPoint[index]);
    capacity += SocPoint[index];
  }  

  return capacity;
}


/*******************************************************************************
* Function Name  : calcCRC8
* Description    : calculate the CRC8
* Input          : data: pointer to byte array, n: number of vytes
* Return         : CRC calue
*******************************************************************************/
static int calcCRC8(unsigned char *data, int n)
{
  int crc=0;   /* initial value */
  int i, j;

  for (i=0;i<n;i++)
  {
    crc ^= data[i];
    for (j=0;j<8;j++) 
    {
      crc <<= 1;
      if (crc & 0x100)  crc ^= 7;
    }
  }
  return(crc & 255);

}


/*******************************************************************************
* Function Name  : UpdateRamCrc
* Description    : calculate the RAM CRC
* Input          : none
* Return         : CRC value
*******************************************************************************/
static int UpdateRamCrc(void)
{
  int res;
  
  res=calcCRC8(GG_Ram.db,RAM_SIZE-1);
  GG_Ram.db[RAM_SIZE-1] = res;   /* last byte holds the CRC */
  return(res);
}

/*******************************************************************************
* Function Name  : Init_RAM
* Description    : Init the STC310x RAM registers with valid test word and CRC
* Input          : none
* Return         : none
*******************************************************************************/
static void Init_RAM(void)
{
  int index;

  for (index=0;index<RAM_SIZE;index++) 
    GG_Ram.db[index]=0;
  GG_Ram.reg.TstWord=RAM_TSTWORD;  /* id. to check RAM integrity */
  GG_Ram.reg.BattCap= BATT_DEF_CAPACITY;
  GG_Ram.reg.Status= GG_INIT;
  /* update the crc */
  UpdateRamCrc();
}



/*******************************************************************************
* Function Name  : UpdateCapacity
* Description    : update the nominal capacity (fully charge battery)
* Use            : BattData structure
* Affect         : BattChargeNominal, BattChargeValue
*******************************************************************************/
static void UpdateCapacity(int soc)
{
  int tentativeValue;

  if (soc==BATT_MAX_SOC)
  {
        /* a full charge has just been done, update capacity */
        tentativeValue = (long) BattChargeValue *  100 / BATT_MAX_SOC ;
        BattChargeNominal = (BattChargeNominal * 3 + tentativeValue) / 4;  /* simple filter to avoid sharp step */
        BattChargeValue = (long) BattChargeNominal * BATT_MAX_SOC /100;
  }
  else if (soc==BATT_MIN_SOC)
  {
       /* min voltage has been reached  */
        tentativeValue = (long) (BattChargeNominal-BattChargeValue) *  100 / (100- BATT_MIN_SOC) ;
        BattChargeNominal = (BattChargeNominal * 3 + tentativeValue) / 4;  /* simple filter to avoid sharp step */
        BattChargeValue = (long) BattChargeNominal * BATT_MIN_SOC /100;
  }
  BattCalStat+=2;
}
  
  

/*******************************************************************************
* Function Name  : CheckUpdateSOC
* Description    : update the SOC using measured SOC %
* Use            : BattData structure
* Affect         : BattChargeValue
*******************************************************************************/
static void CheckUpdateSOC(void)
{
  int SOC;
  int v, diff;
  int tentativeValue;
  int ok;
/* 
#if 0
  BattRelaxTime = BattData.RelaxTimer*2;
  if (BattRelaxTime == 0) 
    BattAvgOCV = BattData.Voltage;
  else
     BattAvgOCV = (BattAvgOCV * 3 + BattData.Voltage) / 4;
#else  */
  if (BattData.Current > BATT_LOW_CURRENT) 
  {
     BattRelaxTime += CountDiff;
     RelaxC = 0;
     BattAvgOCV = (BattAvgOCV * 3 + BattData.Voltage) / 4;
     
  }
  else
  {
    RelaxC++;
    if (RelaxC>2) BattRelaxTime=0;
    BattAvgOCV = BattData.Voltage;
  }
//#endif
  ok=1;
  if ((BattRelaxTime/2) < BATT_RELAX_TIME)  ok=0;
  if (BattData.Temperature < 100) ok=0;  /* do not allow capacity update at low temperature <10°C  */
  if (BattData.Temperature > 450) ok=0;  /* do not allow capacity update at high temperature >45°C */

  if (ok)
    {
    /* we know here that current is low, relaxation time is ok */
    v = BattAvgOCV;
    v -= BATT_INT_RESIST * BattData.Current / 1000;  /* correction for non-true OCV */
    SOC = OCV2Cap(v);
    diff = SOC - (long) BattChargeValue *100 / BattChargeNominal;
    if (diff<0) diff = -diff;

    /* SOC is updated if diff > 10, without other condition, and no filtering
      otherwise SOC updated with filtering unless:
        voltage >4.1V or <3.6V
        diff<=6 and voltage inside 3.7-3.8V
        diff<=3
    */
    
    if (diff>10) {
      BattChargeValue = (long) BattChargeNominal * SOC / 100;
      BattCalStat++;
    }	      
    else {
      if ( (v>4100)|| (v<3600) ) ok=0;  /* nok at >4.1V or < 3.6V */
      if ( (v>3700)&&(v<3800)&&(diff<=6) ) ok=0;  /* nok */
      if (diff<=3) ok=0;
      if (ok)
      {
        tentativeValue = (long) BattChargeNominal * SOC / 100;
        if (tentativeValue<BattChargeValue) 
          BattChargeValue = (BattChargeValue * 3 + tentativeValue) / 4;  /* simple filter to avoid sharp SOC step */
        else
          BattChargeValue = (BattChargeValue * 7 + tentativeValue) / 8;  /* simple filter to avoid sharp SOC step */
        BattCalStat++;
      }
    }
    BattRelaxTime = BATT_RELAX_TIME*2 / 2;   /* restart relax time counter with half period */
  }
  if (BattCalStat>=100) BattCalStat=0;
}
  



/*******************************************************************************
* Function Name  : GG_FSM
* Description    : process the Gas Gauge state machine
* Input          : BattData
* Return         : 
* Affect         : Global Gas Gauge data
*******************************************************************************/
static void GG_FSM(void)
{
  int tentativeValue;
  
  switch (BattState)
  {
    case BATT_CHARGING:
      if (BattData.Current < APP_MIN_CURRENT)
      {
        BattFullDischarge = 0;
        BattRelaxTime=0;
        BattState = BATT_DISCHARG;  /*  discharging */
      }
      else if ( (BattData.Current < CHG_MIN_CURRENT)) 
      {
        BattState = BATT_ENDCHARG;        /* end of charge */
      }
      break;
    case BATT_ENDCHARG:  /* end of charge state. check if fully charged or charge interrupted */
      if ( BattData.Current < CHG_END_CURRENT ) 
        BattState = BATT_IDLE;     /* charge interrupted */
      else
        BattState = BATT_FULCHARG;  /* end of charge */
      break;
    case BATT_FULCHARG:  /* full charge state. wait for actual end of charge current */
      if ( (BattData.Current > CHG_MIN_CURRENT)) 
        BattState = BATT_CHARGING;  /* charge again */
      else if ( BattData.Current < CHG_END_CURRENT ) 
      {
	if(BattData.Voltage > BATT_CHG_VOLTAGE)
	{
		if (BattFullDischarge)
		  UpdateCapacity(BATT_MAX_SOC);
		BattChargeValue = BattChargeNominal;
		BattFullDischarge = 0;
		BattFullCharge = 1;        /* full charge done */
	}
        BattState = BATT_IDLE;     /* end of charge cycle */
      }
      break;
    case BATT_IDLE:  /* no charging, no discharging */
      if (BattData.Current > CHG_MIN_CURRENT) 
      {
        BattFullCharge = 0;
        BattRelaxTime=0;
        BattState = BATT_CHARGING; /* charging again */
      }
      else if (BattData.Current < APP_MIN_CURRENT) 
      {
        BattFullDischarge = 0;
        BattRelaxTime=0;
        BattState = BATT_DISCHARG; /* discharging again */
      }
      break;
    case BATT_DISCHARG:
      if (BattData.Current > APP_MIN_CURRENT) 
      {
        BattState = BATT_IDLE;
      }
      else if (BattData.Voltage < BATT_MIN_VOLTAGE) 
      {
        tentativeValue = (long) BattChargeNominal * BATT_MIN_SOC / 100;
        if (BattChargeValue>tentativeValue) BattChargeValue = tentativeValue;
        BattFullCharge = 0;
        BattFullDischarge = 1;  /* full discharge done */
        BattState = BATT_LOWBATT;
      }
      else
        CheckUpdateSOC();   /* check if time to update SOC from OCV */
      break;
    case BATT_LOWBATT:  /* battery nearly empty... */
      if (BattData.Current > APP_MIN_CURRENT) 
      {
        BattState = BATT_IDLE;   /* idle */
      }
      break;
    default:
        BattState = BATT_IDLE;   /* idle */
   
  } /* end switch */
  
  
}


/*******************************************************************************
* Function Name  : Reset_FSM_GG
* Description    : reset the gas gauge state machine and flags
* Input          : None
* Return         : None
*******************************************************************************/
static void Reset_FSM_GG(void)
{
  BattState = BATT_IDLE;
  BattFullCharge=0;
  BattFullDischarge=0;
  BattCalStat=0;
}


/*******************************************************************************
* Function Name  : UpdateBaseCharge
* Description    : update the base charge with the last charge count from STC310x
* Input          : None
* Return         : None
*******************************************************************************/
static void UpdateBaseCharge(void)
{
  int v;

  GG_Ram.reg.BaseCharge += GG_Ram.reg.LastCharge;
  GG_Ram.reg.LastCharge= 0;

#ifdef STC3105
  /* update the base register for alarm */
   /* SOC base register */
  /*  LSB is: 6.7uVh/R */
  v = (long) GG_Ram.reg.BaseCharge * SENSERESISTOR * 100 / 670;
  //STC310x_WriteByte(STC310x_REG_SOC_BASE, v&255);
  //STC310x_WriteByte(STC310x_REG_SOC_BASE+1, v>>8);
#endif 
 
}



/* -------------------- firmware interface functions ------------------------------------------- */

/*******************************************************************************
* Function Name  : GasGauge_Stop
* Description    : Stop the Gas Gauge system
* Input          : None
* Return         : 0 is ok, -1 if I2C error
*******************************************************************************/
static int GasGauge_Stop(void)
{
  int res;

  STC310x_ReadRamData(GG_Ram.db);
  GG_Ram.reg.Status= GG_POWERDN;
  /* at application power down, the last charge count must be saved in the charge base in STC310x RAM */
  UpdateBaseCharge();
  /* update the crc */
  UpdateRamCrc();
  STC310x_WriteRamData(GG_Ram.db);
      
  res=STC310x_Powerdown();
  if (res!=0) return (-1);  /* error */

  return(0);  
}

/*******************************************************************************
* Function Name  : GasGauge_Start
* Description    : Start the Gas Gauge system
* Input          : None
* Return         : 0 is ok, -1 if STC310x not found or I2C error
* Affect         : global STC310x data and gas gauge variables
*******************************************************************************/
static int GasGauge_Start(void)
{
   int res;
  
  /* check STC310x reset bit and start STC310x */
  res=STC310x_Startup();
  if (res<0) return (-1);  /* error */
  
  /* check RAM valid */
  STC310x_ReadRamData(GG_Ram.db);
  
  if ( (res==1) || (GG_Ram.reg.TstWord != RAM_TSTWORD) || (calcCRC8(GG_Ram.db,RAM_SIZE)!=0) )
  {
    /* RAM invalid */
    Init_RAM();
  }
  else
  {
    /* RAM valid, check status */
    switch (GG_Ram.reg.Status) 
    {
      case GG_RUNNING:
        /* the application restarted without stopping the gas Gauge properly, do it now */
	GasGauge_Stop();
	GasGauge_Start();
	return(0);
        break;
      case GG_POWERDN:
        /* goto RUNNING state ( INIT state is also an option) */
        GG_Ram.reg.Status=GG_RUNNING;
        break;
      default:
        GG_Ram.reg.Status=GG_INIT;
    }
  }
  /* update the crc */
  UpdateRamCrc();
  STC310x_WriteRamData(GG_Ram.db);

  Reset_FSM_GG();

  return(0);
}

/*******************************************************************************
* Function Name  : GasGauge_Task
* Description    : Periodic Gas Gauge task, to be called e.g. every 5 sec.
* Input          : pointer to gas gauge data structure
* Return         : 1 if data available, 0 si no data, -1 if error
* Affect         : global STC310x data and gas gauge variables
*******************************************************************************/

static int GasGauge_Task(GasGauge_DataTypeDef *GG)
{
  int res;
  int value;

  res=STC310x_ReadBatteryData(&BattData);  /* read battery data into global variables */
  if (res!=0) return(-1); /* abort in case of I2C failure */
    
  /* check if RAM data is ok (battery has not been changed) */
  STC310x_ReadRamData(GG_Ram.db);
  if ( (GG_Ram.reg.TstWord!= RAM_TSTWORD) || (calcCRC8(GG_Ram.db,RAM_SIZE)!=0) )
  {
    /* if RAM non ok, reset it and set default nominal capacity */
    Init_RAM(); 
    STC310x_Startup();  /* restart STC310x */     
  }    

  if (GG_Ram.reg.Status == GG_INIT)
  {
    /* INIT state, wait for a voltage value available: */
    if (BattData.ConvCounter>1) 
    {
      value = BattData.Voltage;
      value -= (long) BattData.Current * BATT_INT_RESIST / 1000; /* correction for non-true OCV */

      if ((value>2500)&&(value<4500))
      {
          GG_Ram.reg.BaseCharge = (long) BATT_DEF_CAPACITY * OCV2Cap(value) / 100;
          GG_Ram.reg.Status=GG_RUNNING;
          Reset_FSM_GG();
      }
    }
  }  

  if ((BattData.Status & M_STAT) != M_RUN)
  {
    /* unexpected STC310x status */
    if (GG_Ram.reg.Status== GG_RUNNING) 
        UpdateBaseCharge();
    STC310x_Startup();  /* and restart STC310x */
  }
  
  
  /* Battery state of charge (mAh): */
  BattChargeValue= BattData.Charge + GG_Ram.reg.BaseCharge; 
  /* Battery nominal capacity (mAh) */
  BattChargeNominal=GG_Ram.reg.BattCap;

  CountDiff = BattData.ConvCounter - GG_Ram.reg.LastCount;
  if (CountDiff<0) CountDiff += 65536;
  
  /* ---------- process the Gas Gauge algorithm -------- */
  if (GG_Ram.reg.Status == GG_RUNNING) {
    /* if Gas Gauge is running ... */
    if ( (BattData.Status & M_EOC) != 0)  {
      /* ... and if new data available: */
      GG_FSM();
    }
  }
  /* --------------------------------------------------- */  
  

  /* update the Gas Gauge status variables back into STC310x registers  */
  GG_Ram.reg.LastCharge=BattData.Charge;
  GG_Ram.reg.LastCount=BattData.ConvCounter;
  GG_Ram.reg.BaseCharge=BattChargeValue - BattData.Charge;
  GG_Ram.reg.BattCap=BattChargeNominal;
  UpdateRamCrc();
  STC310x_WriteRamData(GG_Ram.db);
  
#ifdef STC3105
  /* update base register for alarm */
#endif  
  
  /* -------- APPLICATION RESULTS ------------ */

  /* fill gas gauge data with battery data */
  GG->State = BattState;
  GG->Voltage=BattData.Voltage;
  GG->Current=BattData.Current;
  GG->Temperature=BattData.Temperature;
  
  GG->ChargeNominal = GG_Ram.reg.BattCap;

  /* State-of-Charge (mAh) */
  GG->ChargeValue = GG_Ram.reg.BaseCharge + BattData.Charge;
  
  /* State-Of-Charge (%) */
  value= (long) GG->ChargeValue * 100 / GG->ChargeNominal;

  if (value < 0) value = 0;
  if (value > 100) value = 100;
    
  GG->ChargePercent = value;
  
  /* remaining operating time (during discharge only i.e. current is <0): */
  if (BattData.Current < APP_MIN_CURRENT) {  
    value = (long) GG->ChargeValue * 60 / (-BattData.Current);  /* in minutes */
    if (value<0) value=0;
  }
  else
    value = -1;  /* to indicate no value */
  GG->RemTime = value;


#ifdef TEST_DRIVER  
  GG->CalStat = BattCalStat;
#endif
  
  /* STC3100: software management of the alarm output */
  /* (on STC3105, the ALM pin is managed by hardware) */
  if (GG_Ram.reg.Status==GG_RUNNING) {
    /* drive the IO0 output to signal low battery voltage/charge condition */
    if ((BattData.Voltage<BATT_LOW_VOLT_ALM)||(GG->ChargePercent<BATT_LOW_SOC_ALM))
      STC310x_WriteByte(STC310x_REG_CTRL, 0x00);
    else
      STC310x_WriteByte(STC310x_REG_CTRL, 0x01);  
  }
 
  if (GG_Ram.reg.Status==GG_RUNNING)
    return(1);
  else
    return(0);
}





/* -------------------------------------------------------------------------- */



static void stc31xx_get_version(struct i2c_client *client)
{
    printk("STC3105 Fuel-Gauge Ver %s\n", GG_VERSION);
}


/* -------------------------------------------------------------- */


static irqreturn_t stc3105_Irq_Handler(int irq, void *dev_id)
{
	struct i2c_client *client = dev_id;
	struct stc31xx_platform_data *pdata = client->dev.platform_data;
	struct power_supply *psy;

	printk(KERN_ALERT "stc3105_Irq_Handler\n");

	/* clear the irq */
	pdata->ack_irq();

	psy = power_supply_get_by_name("battery");
	if (!psy)
		printk("%s : get battery info failed\n", __func__);
	else
		power_supply_changed(psy);

	return IRQ_HANDLED;
}

unsigned int fg_read_vcell(void)
{
	GasGauge_DataTypeDef GasGaugeData;
	int res;
	
	res=GasGauge_Task(&GasGaugeData);  /* process gas gauge algorithm, returns results */
	if (res>0) 
	{
		return GasGaugeData.Voltage;
	}
	else
		return 4000;
}

unsigned int fg_read_soc(void)
{
	GasGauge_DataTypeDef GasGaugeData;
	int res;
	
	res=GasGauge_Task(&GasGaugeData);  /* process gas gauge algorithm, returns results */
	if (res>0) 
	{
		return GasGaugeData.ChargePercent;
	}
	else
		return 90;
}

unsigned int fg_read_current(void)
{
	GasGauge_DataTypeDef GasGaugeData;
	int res;
	
	res=GasGauge_Task(&GasGaugeData);  /* process gas gauge algorithm, returns results */
	if (res>0) 
	{
		return GasGaugeData.Current;
	}
	else
		return 600;
}

static int __devinit stc31xx_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;

	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct stc31xx_platform_data *pdata = client->dev.platform_data;

	/*First check the functionality supported by the host*/
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_READ_I2C_BLOCK)) {
		printk("jgb 333\n");
		return -EIO;
	}
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)) {
		printk("jgb 444\n");
		return -EIO;
	}

	/*OK. For now, we presume we have a valid client. We now create the
	client structure*/
	if (sav_client == NULL ) {
		sav_client = kzalloc(sizeof(struct stc31xx_chip), GFP_KERNEL);
		if (sav_client == NULL ) {
			return -ENOMEM;
		}
	}

	printk("jgb STC31xx probe started\n");

	sav_client->client = client;

	i2c_set_clientdata(client, sav_client);
  
	stc31xx_get_version(client);

	/* start gas gauge system */
	GasGauge_Start();

	/* init irq */
	/* irq = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO110)) */
	pdata->init_irq();

	ret = request_irq(client->irq, stc3105_Irq_Handler, IRQF_TRIGGER_FALLING, "STC3105", client);
	if (ret) {
		printk(KERN_ALERT "Request IRQ for AL25 failed, return:%d\n", ret);
		return ret;
	}

	STC3105_Setup();

	is_attached = 1;
	
	return 0;
  
}



static int __devexit stc31xx_remove(struct i2c_client *client)
{
	struct stc31xx_chip *chip = i2c_get_clientdata(client);

	pr_info("[BATT] %s\n", __func__);

	/* stop gas gauge system */
	GasGauge_Stop();

	i2c_set_clientdata(client, NULL);
	sav_client->client = NULL;
	kfree(chip);
	
	return 0;
}



#if 0//def CONFIG_PM

static int stc31xx_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct stc31xx_chip *chip = i2c_get_clientdata(client);

	cancel_delayed_work(&chip->work);
	return 0;
}

static int stc31xx_resume(struct i2c_client *client)
{
	struct stc31xx_chip *chip = i2c_get_clientdata(client);

	schedule_delayed_work(&chip->work, STC3100_DELAY);
	return 0;
}

#else

#define stc31xx_suspend NULL
#define stc31xx_resume NULL

#endif /* CONFIG_PM */





/* Every chip have a unique id */
static const struct i2c_device_id stc31xx_id[] = {
	{ "fuelgauge_stc3105", 0 },
	{ }
};

/* Every chip have a unique id and we need to register this ID using MODULE_DEVICE_TABLE*/
MODULE_DEVICE_TABLE(i2c, stc31xx_id);

static struct i2c_driver stc31xx_i2c_driver = {
	.driver	= {
		.name	= "fuelgauge_stc3105",
	},
	.probe		= stc31xx_probe,
	.remove		= __devexit_p(stc31xx_remove),
	.suspend	= stc31xx_suspend,    
	.resume		= stc31xx_resume,  
	.id_table	= stc31xx_id,
};

#if 1
/*To register this I2C chip driver, the function i2c_add_driver should be called 
with a pointer to the struct i2c_driver*/
static int __init stc31xx_init(void)
{
	return i2c_add_driver(&stc31xx_i2c_driver);
}
module_init(stc31xx_init);

/*To unregister the I2C chip driver, the i2c_del_driver function should be called
with the same pointer to the struct i2c_driver*/
static void __exit stc31xx_exit(void)
{
	i2c_del_driver(&stc31xx_i2c_driver);
}
module_exit(stc31xx_exit);

MODULE_AUTHOR("ST IMS SYSTEMS LAB");
MODULE_DESCRIPTION("STC3105 Fuel Gauge");
MODULE_LICENSE("GPL");
#endif
