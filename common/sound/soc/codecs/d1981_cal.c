/* d1981.c  --  (Wildcat Lynx) Codec kernel driver

Copyright (C) 2011 Dialog Semiconductor Ltd.

Platform: Samsung Froyo

Customised version intended for Samsung/ Marvell platform

Driver Responsibilities:

Registering with Kernel
Initialising codec registers
Downloading DSP (Digital Signal Processor) Firmware
Configuring DSP via mailbox commands
Defining and registering ALSA controls
Implementing I2C, ALSA interfaces

Additional responsibilities for Samsung/Marvell platform:

ALSA controls to add ACM path handling controls.
Translation of path commands to corresponding Use Cases.


 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
//#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/regs-ost.h>
#include <linux/delay.h>         /* for msleep / usleep definition */

#include "d1981.h"
#include "d1981_reg.h"

/* debug */
#define D1981_DEBUG 1
#if D1981_DEBUG
#define dbg(format, arg...) printk(KERN_INFO "d1981: " format "\n", ## arg)
#else
#define dbg(format, arg...)
#endif

#define ARRAY_AND_SIZE(x)       x, ARRAY_SIZE(x)


#define D1981_VERSION "1.0" 

enum {

CAL_AnaMIC1_TO_I2S_1	=	7,//from mic1 to ap at recording
CAL_AnaMIC1_LOUD_TO_I2S_1, //from loud mic1 to ap at recording
CAL_AnaMIC2_TO_I2S_1	 , //from headset mic1 to ap at recording
CAL_AUX1_TO_I2S_1      , 

//MIC to I2S/PCM 2(CP) output
CAL_AnaMIC1_TO_PCM_2 ,//     = 11,
CAL_AnaMIC1_LOUD_TO_PCM_2 ,//= 12,
CAL_AnaMIC2_TO_PCM_2,//     = 13,
CAL_AUX1_TO_PCM_2 ,//        = 14,

// From I2S/PCM 1 VIA DAC
CAL_I2S_1_TO_HS_VIA_DAC12 ,// 15, //bsw from ap to headset at playback
CAL_I2S_1_TO_SPKR_VIA_DAC3,// 16 //bsw frome ap to loud speaker at playback
CAL_I2S_1_TO_RCV_VIA_DAC4 ,
CAL_I2S_1_TO_LINEOUT_VIA_DAC4,

// From I2S/PCM 2 VIA DAC
CAL_PCM_2_TO_HS_VIA_DAC12 ,//     = 19, //bsw from cp to reciever at voice call
CAL_PCM_2_TO_SPKR_VIA_DAC3,//     = 20, //bsw frome cp to loud speaker at voice call
CAL_PCM_2_TO_RCV_VIA_DAC4 ,
CAL_PCM_2_TO_LINEOUT_VIA_DAC4,

// From AUX VIA DAC
CAL_AUX1_TO_HS_VIA_DAC12, // 23
CAL_AUX1_SPKR_VIA_DAC3,
CAL_AUX1_TO_RCV_VIA_DAC4 ,
CAL_AUX1_TO_LINEOUT_VIA_DAC4,


CAL2_AnaMIC1_TO_I2S_1	,//27//from mic1 to ap at recording
CAL2_AnaMIC1_LOUD_TO_I2S_1, //from loud mic1 to ap at recording
CAL2_AnaMIC2_TO_I2S_1	 , //from headset mic1 to ap at recording
CAL2_AUX1_TO_I2S_1      , 

//MIC to I2S/PCM 2(CP) output
CAL2_AnaMIC1_TO_PCM_2 ,//   31
CAL2_AnaMIC1_LOUD_TO_PCM_2 ,//
CAL2_AnaMIC2_TO_PCM_2,// 
CAL2_AUX1_TO_PCM_2 ,//     

// From I2S/PCM 1 VIA DAC
CAL2_I2S_1_TO_HS_VIA_DAC12 ,// 35, //bsw from ap to headset at playback
CAL2_I2S_1_TO_SPKR_VIA_DAC3,//36 //bsw frome ap to loud speaker at playback
CAL2_I2S_1_TO_RCV_VIA_DAC4 ,
CAL2_I2S_1_TO_LINEOUT_VIA_DAC4,

// From I2S/PCM 2 VIA DAC
CAL2_PCM_2_TO_HS_VIA_DAC12 ,//39 bsw from cp to reciever at voice call
CAL2_PCM_2_TO_SPKR_VIA_DAC3,//, //bsw frome cp to loud speaker at voice call
CAL2_PCM_2_TO_RCV_VIA_DAC4 ,
CAL2_PCM_2_TO_LINEOUT_VIA_DAC4,

// From AUX VIA DAC
CAL2_AUX1_TO_HS_VIA_DAC12, // 43
CAL2_AUX1_SPKR_VIA_DAC3,
CAL2_AUX1_TO_RCV_VIA_DAC4 ,
CAL2_AUX1_TO_LINEOUT_VIA_DAC4,
CAL2_47,
CAL2_48,
CAL2_49,
CAL2_50,
CAL2_51,
CAL2_52,
CAL2_53,
CAL2_54,
CAL2_55,
CAL2_56,
CAL2_57,
CAL2_58,
CAL2_59,
CAL2_60,
CAL2_61,
CAL2_62,
CAL2_63,
CAL2_64,
CAL2_65,
CAL2_66,
CAL2_67,
CAL2_68,
CAL2_69,
CAL2_70,


CAL2_71,
CAL2_72,
CAL2_73,
CAL2_74,
CAL2_75,
CAL2_76,
CAL2_77,
CAL2_78,
CAL2_79,
CAL2_80,
CAL2_81,
CAL2_82,
CAL2_83,
CAL2_84,
CAL2_85,
CAL2_86,
CAL2_87,
CAL2_88,
CAL2_89,

CAL2_90,
CAL2_91,
CAL2_92,
CAL2_93,
CAL2_94,
CAL2_95,
CAL2_96,
CAL2_97,
CAL2_98,
CAL2_99,
CAL2_100,
CAL2_101,
CAL2_102,
CAL2_103,
CAL2_104

} ;
extern unsigned char d1981_codec_force_speaker_gain[2][D1981_MAX_GAINS_PER_PATH] ;
extern unsigned char d1981_path_enabled[D1981_TOTAL_VALID_PATHS];

extern u8		gActiveUseCase; 
extern u8 d1981_reg_read(u8 const reg);
extern int d1981_reg_write(u8 const reg, u8 const val);
extern int d1981PathVolumeSet(u8 path, ACM_AudioVolume volume,unsigned char dynamic);
extern int d1981PathVolume2Set(u8 path, ACM_AudioVolume volume,unsigned char dynamic);
//extern int d1981Aux1PgaVolumeSet(void);
//extern unsigned char d1981_aux1_pga_gain;
#if 0
int d1981_CAL2_AUX1_PGA_Set(unsigned char volume)
{       
       printk(KERN_ALERT "d1981_CAL2_AUX1_PGA_Set  volume=0x%x \n ",volume);

//	d1981_aux1_pga_gain=volume;
//	d1981Aux1PgaVolumeSet();

	return(1);
}

int d1981_CAL2_AUX1_PGA_Get(void)
{       
       printk(KERN_ALERT "d1981_CAL2_AUX1_PGA_Get \n ");
	return 0;//d1981_aux1_pga_gain;	
}
#endif
int d1981_CAL_VolumeSet(u8 path,unsigned char volume)
{
    printk(KERN_ALERT "d1981_CAL_VolumeSet Path=%d volume=0x%x \n",path,volume);

	d1981_codec_gain[path][0]=volume;

	if(d1981_path_enabled[path]==1)
	    d1981PathVolumeSet(path,volume,0xff);


	return(1);
}

int d1981_CAL2_VolumeSet(u8 path,unsigned char volume)
{
        printk(KERN_ALERT "d1981_CAL2_VolumeSet Path=%d volume=0x%x \n",path,volume);

	d1981_codec_gain[path][1]=volume;
	if(d1981_path_enabled[path]==1)
	    d1981PathVolume2Set(path,volume,0xff);


	return(1);
}

int d1981_CAL_VolumeGet(u8 path)
{
       printk(KERN_ALERT "d1981_CAL_VolumeGet Path=%d \n ",path);

	return d1981_codec_gain[path][0];	
}

int d1981_CAL2_VolumeGet(u8 path)
{
       
       printk(KERN_ALERT "d1981_CAL2_VolumeGet Path=%d \n",path);

	return d1981_codec_gain[path][1];	

}

int d1981_CAL_Ring_DACVolumeSet(u8 speaker,unsigned char volume)
{
    printk(KERN_ALERT "d1981_CAL_Ring_DACVolumeSet speaker=%d volume=0x%x \n",speaker,volume);

	d1981_codec_force_speaker_gain[speaker][1]=volume;

	if(d1981_path_enabled[D1981_I2S_1_TO_SPKR_HS]==1)
	    d1981PathVolume2Set(D1981_I2S_1_TO_SPKR_HS,volume,0xff);


	return(1);
}

int d1981_CAL_Ring_ALAVolumeSet(u8 speaker,unsigned char volume)
{
        printk(KERN_ALERT "d1981_CAL_Ring_ALAVolumeSet speaker=%d volume=0x%x \n",speaker,volume);

	d1981_codec_force_speaker_gain[speaker][0]=volume;
	if(d1981_path_enabled[D1981_I2S_1_TO_SPKR_HS]==1)
	    d1981PathVolumeSet(D1981_I2S_1_TO_SPKR_HS,volume,0xff);


	return(1);
}

int d1981_CAL_Ring_DACVolumeGet(u8 speaker)
{
       printk(KERN_ALERT "d1981_CAL_Ring_DACVolumeGet speaker=%d \n ",speaker);

	return d1981_codec_force_speaker_gain[speaker][1];	
}

int d1981_CAL_Ring_ALAVolumeGet(u8 speaker)
{
       
       printk(KERN_ALERT "d1981_CAL_Ring_ALAVolumeGet speaker=%d \n",speaker);

	return d1981_codec_force_speaker_gain[speaker][0];	

}

static int d1981_CAL_AnaMIC1_TO_I2S_1_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL_VolumeSet(D1981_AnaMIC1_TO_PCM_2_APPS,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_AnaMIC1_TO_I2S_1_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AnaMIC1_TO_PCM_2_APPS);
	return(1);
}

static int d1981_CAL_AnaMIC1_LOUD_TO_I2S_1_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL_VolumeSet(D1981_AnaMIC1_LOUD_TO_PCM_2_APPS,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_AnaMIC1_LOUD_TO_I2S_1_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AnaMIC1_LOUD_TO_PCM_2_APPS);
	return(1);
}


static int d1981_CAL_AnaMIC2_TO_I2S_1_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL_VolumeSet(D1981_AnaMIC2_TO_PCM_2_APPS,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_AnaMIC2_TO_I2S_1_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AnaMIC2_TO_PCM_2_APPS);
	return(1);
}

static int d1981_CAL_AUX1_TO_I2S_1_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL_VolumeSet(D1981_AUX1_TO_I2S_1,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_AUX1_TO_I2S_1_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AUX1_TO_I2S_1);
	return(1);
}


static int d1981_CAL_AnaMIC1_TO_PCM_2_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL_VolumeSet(D1981_AnaMIC1_TO_PCM_2,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_AnaMIC1_TO_PCM_2_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AnaMIC1_TO_PCM_2);
	return(1);
}


static int d1981_CAL_AnaMIC1_LOUD_TO_PCM_2_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL_VolumeSet(D1981_AnaMIC1_LOUD_TO_PCM_2,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_AnaMIC1_LOUD_TO_PCM_2_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AnaMIC1_LOUD_TO_PCM_2);
	return(1);
}

static int d1981_CAL_AnaMIC2_TO_PCM_2_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
       d1981_CAL_VolumeSet(D1981_AnaMIC2_TO_PCM_2,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_AUX1_TO_PCM_2_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL_VolumeSet(D1981_AUX1_TO_PCM_2,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_I2S_1_TO_HS_VIA_DAC12_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    if(gActiveUseCase == USE_CASE_3B)
        d1981_CAL_VolumeSet(D1981_AUX1_TO_HS_VIA_DAC12,ucontrol->value.integer.value[0]);
    else
	d1981_CAL_VolumeSet(D1981_I2S_1_TO_HS_VIA_DAC12,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_I2S_1_TO_SPKR_VIA_DAC3_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    if(gActiveUseCase == USE_CASE_3B)
      d1981_CAL_VolumeSet(D1981_AUX1_SPKR_VIA_DAC3,ucontrol->value.integer.value[0]);
    else
       d1981_CAL_VolumeSet(D1981_I2S_1_TO_SPKR_VIA_DAC3,ucontrol->value.integer.value[0]);
	
	return(1);
}

static int d1981_CAL_I2S_1_TO_RCV_VIA_DAC4_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL_VolumeSet(D1981_I2S_1_TO_RCV_VIA_DAC4,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_PCM_2_TO_HS_VIA_DAC12_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
       d1981_CAL_VolumeSet(D1981_PCM_2_TO_HS_VIA_DAC12,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_PCM_2_TO_SPKR_VIA_DAC3_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL_VolumeSet(D1981_PCM_2_TO_SPKR_VIA_DAC3,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_PCM_2_TO_RCV_VIA_DAC4_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL_VolumeSet(D1981_PCM_2_TO_RCV_VIA_DAC4,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_AUX1_TO_HS_VIA_DAC12_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	//d1981_CAL_VolumeSet(D1981_AUX1_TO_HS_VIA_DAC12,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_AUX1_SPKR_VIA_DAC3_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	//d1981_CAL_VolumeSet(D1981_AUX1_SPKR_VIA_DAC3,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_AUX1_TO_RCV_VIA_DAC4_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL_VolumeSet(D1981_AUX1_TO_RCV_VIA_DAC4,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_AUX1_TO_LINEOUT_VIA_DAC4_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL_VolumeSet(D1981_AUX1_TO_LINEOUT_VIA_DAC4,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_I2S_1_TO_LINEOUT_VIA_DAC4_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL_VolumeSet(D1981_I2S_1_TO_LINEOUT_VIA_DAC4,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_PCM_2_TO_LINEOUT_VIA_DAC4_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL_VolumeSet(D1981_PCM_2_TO_LINEOUT_VIA_DAC4,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL_AnaMIC2_TO_PCM_2_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AnaMIC2_TO_PCM_2);
	return(1);
}

static int d1981_CAL_AUX1_TO_PCM_2_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
       ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AUX1_TO_PCM_2);
	return(1);
}

static int d1981_CAL_I2S_1_TO_HS_VIA_DAC12_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    if(gActiveUseCase == USE_CASE_3B)
        ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AUX1_TO_HS_VIA_DAC12);
    else
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_I2S_1_TO_HS_VIA_DAC12);
	return(1);
}

static int d1981_CAL_I2S_1_TO_SPKR_VIA_DAC3_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    if(gActiveUseCase == USE_CASE_3B)
        ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AUX1_SPKR_VIA_DAC3);
    else
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_I2S_1_TO_SPKR_VIA_DAC3);
	return(1);
}

static int d1981_CAL_I2S_1_TO_RCV_VIA_DAC4_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_I2S_1_TO_RCV_VIA_DAC4);
	return(1);
}

static int d1981_CAL_I2S_1_TO_LINEOUT_VIA_DAC4_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_I2S_1_TO_LINEOUT_VIA_DAC4);
	return(1);
}

static int d1981_CAL_PCM_2_TO_HS_VIA_DAC12_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_PCM_2_TO_HS_VIA_DAC12);
	return(1);
}

static int d1981_CAL_PCM_2_TO_SPKR_VIA_DAC3_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_PCM_2_TO_SPKR_VIA_DAC3);
	return(1);
}

static int d1981_CAL_PCM_2_TO_RCV_VIA_DAC4_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_PCM_2_TO_RCV_VIA_DAC4);
	return(1);
}

static int d1981_CAL_PCM_2_TO_LINEOUT_VIA_DAC4_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_PCM_2_TO_LINEOUT_VIA_DAC4);
	return(1);
}


static int d1981_CAL_AUX1_TO_HS_VIA_DAC12_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=0;//d1981_CAL_VolumeGet(D1981_AUX1_TO_HS_VIA_DAC12);
	return(1);
}

static int d1981_CAL_AUX1_SPKR_VIA_DAC3_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=0;//d1981_CAL_VolumeGet(D1981_AUX1_SPKR_VIA_DAC3);
	return(1);
}

static int d1981_CAL_AUX1_TO_RCV_VIA_DAC4_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
 	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AUX1_TO_RCV_VIA_DAC4);
	return(1);
}

static int d1981_CAL_AUX1_TO_LINEOUT_VIA_DAC4_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
 	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AUX1_TO_LINEOUT_VIA_DAC4);
	return(1);
}

static int d1981_CAL2_AnaMIC1_TO_I2S_1_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_AnaMIC1_TO_PCM_2_APPS,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_AnaMIC1_LOUD_TO_I2S_1_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_AnaMIC1_LOUD_TO_PCM_2_APPS,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_AnaMIC2_TO_I2S_1_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_AnaMIC2_TO_PCM_2_APPS,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_AUX1_TO_I2S_1_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_AUX1_TO_I2S_1,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_AnaMIC1_TO_PCM_2_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_AnaMIC1_TO_PCM_2,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_AnaMIC1_LOUD_TO_PCM_2_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_AnaMIC1_LOUD_TO_PCM_2,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_AnaMIC2_TO_PCM_2_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
       d1981_CAL2_VolumeSet(D1981_AnaMIC2_TO_PCM_2,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_AUX1_TO_PCM_2_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_AUX1_TO_PCM_2,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_I2S_1_TO_HS_VIA_DAC12_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    if(gActiveUseCase == USE_CASE_3B)
        d1981_CAL2_VolumeSet(D1981_AUX1_TO_HS_VIA_DAC12,ucontrol->value.integer.value[0]);
    else
	d1981_CAL2_VolumeSet(D1981_I2S_1_TO_HS_VIA_DAC12,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_I2S_1_TO_SPKR_VIA_DAC3_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
   if(gActiveUseCase == USE_CASE_3B)
       d1981_CAL2_VolumeSet(D1981_AUX1_SPKR_VIA_DAC3,ucontrol->value.integer.value[0]);
   else
       d1981_CAL2_VolumeSet(D1981_I2S_1_TO_SPKR_VIA_DAC3,ucontrol->value.integer.value[0]);
	
	return(1);
}

static int d1981_CAL2_I2S_1_TO_RCV_VIA_DAC4_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_I2S_1_TO_RCV_VIA_DAC4,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_PCM_2_TO_HS_VIA_DAC12_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
       d1981_CAL2_VolumeSet(D1981_PCM_2_TO_HS_VIA_DAC12,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_PCM_2_TO_SPKR_VIA_DAC3_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_PCM_2_TO_SPKR_VIA_DAC3,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_PCM_2_TO_RCV_VIA_DAC4_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_PCM_2_TO_RCV_VIA_DAC4,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_AUX1_TO_HS_VIA_DAC12_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_AUX1_TO_HS_VIA_DAC12,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_AUX1_SPKR_VIA_DAC3_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_AUX1_SPKR_VIA_DAC3,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_AUX1_TO_RCV_VIA_DAC4_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_AUX1_TO_RCV_VIA_DAC4,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_AUX1_TO_LINEOUT_VIA_DAC4_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_AUX1_TO_LINEOUT_VIA_DAC4,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_I2S_1_TO_LINEOUT_VIA_DAC4_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_I2S_1_TO_LINEOUT_VIA_DAC4,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_PCM_2_TO_LINEOUT_VIA_DAC4_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	d1981_CAL2_VolumeSet(D1981_PCM_2_TO_LINEOUT_VIA_DAC4,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_AnaMIC1_TO_I2S_1_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AnaMIC1_TO_PCM_2_APPS);
	return(1);
}

static int d1981_CAL2_AnaMIC1_LOUD_TO_I2S_1_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AnaMIC1_LOUD_TO_PCM_2_APPS);
	return(1);
}

static int d1981_CAL2_AnaMIC2_TO_I2S_1_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AnaMIC1_TO_PCM_2_APPS);
	return(1);
}

static int d1981_CAL2_AUX1_TO_I2S_1_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AUX1_TO_I2S_1);
	return(1);
}

static int d1981_CAL2_AnaMIC1_TO_PCM_2_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AnaMIC1_TO_PCM_2);
	return(1);
}

static int d1981_CAL2_AnaMIC1_LOUD_TO_PCM_2_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AnaMIC1_LOUD_TO_PCM_2);
	return(1);
}

static int d1981_CAL2_AnaMIC2_TO_PCM_2_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AnaMIC2_TO_PCM_2);
	return(1);
}

static int d1981_CAL2_AUX1_TO_PCM_2_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AUX1_TO_PCM_2);
	return(1);
}

static int d1981_CAL2_I2S_1_TO_HS_VIA_DAC12_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    if(gActiveUseCase == USE_CASE_3B)
        ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AUX1_TO_HS_VIA_DAC12);        
    else
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_I2S_1_TO_HS_VIA_DAC12);
	return(1);
}

static int d1981_CAL2_I2S_1_TO_SPKR_VIA_DAC3_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    if(gActiveUseCase == USE_CASE_3B)
        ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AUX1_SPKR_VIA_DAC3);        
    else
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_I2S_1_TO_SPKR_VIA_DAC3);
	return(1);
}

static int d1981_CAL2_I2S_1_TO_RCV_VIA_DAC4_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_I2S_1_TO_RCV_VIA_DAC4);
	return(1);
}

static int d1981_CAL2_I2S_1_TO_LINEOUT_VIA_DAC4_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_I2S_1_TO_LINEOUT_VIA_DAC4);
	return(1);
}



static int d1981_CAL2_PCM_2_TO_HS_VIA_DAC12_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_PCM_2_TO_HS_VIA_DAC12);
	return(1);
}

static int d1981_CAL2_PCM_2_TO_SPKR_VIA_DAC3_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_PCM_2_TO_SPKR_VIA_DAC3);
	return(1);
}

static int d1981_CAL2_PCM_2_TO_RCV_VIA_DAC4_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_PCM_2_TO_RCV_VIA_DAC4);
	return(1);
}

static int d1981_CAL2_PCM_2_TO_LINEOUT_VIA_DAC4_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_PCM_2_TO_LINEOUT_VIA_DAC4);
	return(1);
}

static int d1981_CAL2_AUX1_TO_HS_VIA_DAC12_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AUX1_TO_HS_VIA_DAC12);
	return(1);
}

static int d1981_CAL2_AUX1_SPKR_VIA_DAC3_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AUX1_SPKR_VIA_DAC3);
	return(1);
}

static int d1981_CAL2_AUX1_TO_RCV_VIA_DAC4_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
 	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AUX1_TO_RCV_VIA_DAC4);
	return(1);
}

static int d1981_CAL2_AUX1_TO_LINEOUT_VIA_DAC4_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
 	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AUX1_TO_LINEOUT_VIA_DAC4);
	return(1);
}

static int d1981_CAL2_Temp_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	//d1981_CAL2_VolumeSet(D1981_PCM_2_TO_LINEOUT_VIA_DAC4,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_Temp_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AnaMIC1_TO_I2S_1);
	return(1);
}

static int d1981_CAL2_AUX1_PGA_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	//d1981_aux1_pga_gain=ucontrol->value.integer.value[0];
	d1981_CAL_VolumeSet(D1981_AUX1_TO_HS_VIA_DAC12,ucontrol->value.integer.value[0]);
	return(1);
}

static int d1981_CAL2_AUX1_PGA_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	//ucontrol->value.integer.value[0]=d1981_aux1_pga_gain;
	ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AUX1_TO_HS_VIA_DAC12);
	return(1);
}

static int dps_command_mode=0;
static int dsp_count=0;
static int dsp_total_count=0;
static u8 dsp_buf[2048];
extern int load_use_case(const u8 *use_case, u16 len);

static int d1981_CAL2_DSP_75_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    //printk(KERN_ALERT "d1981_CAL2_DSP_75_set 0x%x !!!!!!!!!!\n",ucontrol->value.integer.value[0]);
    #if 1
    dps_command_mode=1;
	d1981_reg_write(0x75, ucontrol->value.integer.value[0]);
	#endif
	//msleep(1);
	return(1);
}

static int d1981_CAL2_DSP_75_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_reg_read(0x75);
	return(1);
}

static int d1981_CAL2_DSP_76_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    //printk(KERN_ALERT "d1981_CAL2_DSP_76_set 0x%x !!!!!!!!!!\n",ucontrol->value.integer.value[0]);
    dps_command_mode=1;
    if(dps_command_mode==0)
    {
        if(dsp_total_count < 0)
        {
            dsp_total_count=-1;
            printk(KERN_ALERT "d1981_CAL2_DSP_76_set ERROR \n");
            return 0;
        }
        if(dsp_total_count==0)
        {
            if(ucontrol->value.integer.value[0]==0)
                dsp_total_count=256*3*2;
            else
                dsp_total_count=(ucontrol->value.integer.value[0]*3*2);

           printk(KERN_ALERT "d1981_CAL2_DSP_76_set Size=%d !!!!!!!!!!\n",dsp_total_count);
        }
        
        dsp_buf[dsp_count++]=0x76;
        dsp_buf[dsp_count++]=ucontrol->value.integer.value[0];
        if(dsp_count==dsp_total_count)
        {
          msleep(1);
          load_use_case(dsp_buf,dsp_total_count/2); 
          dsp_count=0;
          dsp_total_count=0;
        }
    }
    else
    {
	    d1981_reg_write(0x76, ucontrol->value.integer.value[0]);
	}
	//msleep(1);
	return(1);
}

static int d1981_CAL2_DSP_76_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_reg_read(0x76);
	return(1);
}

static int d1981_CAL2_DSP_77_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    //printk(KERN_ALERT "d1981_CAL2_DSP_77_set 0x%x !!!!!!!!!!\n",ucontrol->value.integer.value[0]);
    dps_command_mode=1;
    if(dps_command_mode==0)
    {
        if(dsp_total_count <= 0)
        {
            dsp_total_count=-1;
            printk(KERN_ALERT "d1981_CAL2_DSP_77_set ERROR \n");
            return 0;
        }
        dsp_buf[dsp_count++]=0x77;
        dsp_buf[dsp_count++]=ucontrol->value.integer.value[0];
        if(dsp_count==dsp_total_count)
        {
          load_use_case(dsp_buf,dsp_total_count/2); 
          dsp_count=0;
          dsp_total_count=0;
        }
    }
    else
    {
	    d1981_reg_write(0x77, ucontrol->value.integer.value[0]);
	}
	//msleep(1);
	return(1);
}

static int d1981_CAL2_DSP_77_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_reg_read(0x77);
	return(1);
}
static int d1981_CAL2_DSP_78_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
   // printk(KERN_ALERT "d1981_CAL2_DSP_78_set 0x%x !!!!!!!!!!\n",ucontrol->value.integer.value[0]);
   dps_command_mode=1;
    if(dps_command_mode==0)
    {
        if(dsp_total_count <= 0)
        {
            dsp_total_count=-1;
            printk(KERN_ALERT "d1981_CAL2_DSP_78_set ERROR \n");
            return 0;
        }
        dsp_buf[dsp_count++]=0x78;
        dsp_buf[dsp_count++]=ucontrol->value.integer.value[0];
        if(dsp_count==dsp_total_count)
        {
           printk(KERN_ALERT "d1981_CAL2_DSP_78_set load user case !!!!\n");
          load_use_case(dsp_buf,dsp_total_count/2); 
          dsp_count=0;
          dsp_total_count=0;
        }
    }
    else  
    {
    	d1981_reg_write(0x78, ucontrol->value.integer.value[0]);
    	dps_command_mode=0;
    	dsp_count=0;
    	dsp_total_count=0;
    	msleep(5);
	}
	return(1);
}

static int d1981_CAL2_DSP_78_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_reg_read(0x78);
	return(1);
}

static char d1981_rigeter_Address=0;
static char d1981_rigeter_Value=0;

static int d1981_CAL2_RegisterRead_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_rigeter_Address= ucontrol->value.integer.value[0];

	return(1);
}

static int d1981_CAL2_RegisterRead_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_reg_read(d1981_rigeter_Address);
	return(1);
}

static int d1981_CAL2_RegisterWrite_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_rigeter_Value= ucontrol->value.integer.value[0];

	return(1);
}

static int d1981_CAL2_RegisterWrite_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0]=d1981_reg_write(d1981_rigeter_Address, d1981_rigeter_Value);
	return(1);
}


static int CAL2_MIC1_Preamp_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AnaMIC1_TO_PCM_2_VT);
    return 1;

}

static int CAL2_MIC1_Preamp_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL_VolumeSet(D1981_AnaMIC1_TO_PCM_2_VT,ucontrol->value.integer.value[0]);
    return 1;
}

static int CAL2_MIC1_PGA_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AnaMIC1_TO_PCM_2_VT);
    return 1;

}

static int CAL2_MIC1_PGA_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL2_VolumeSet(D1981_AnaMIC1_TO_PCM_2_VT,ucontrol->value.integer.value[0]);
    return 1;
}

static int CAL2_MIC1_ADC_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    return 1;

}

static int CAL2_MIC1_ADC_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{    
    return 1;
}

static int CAL2_MIC2_Preamp_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

    ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AnaMIC2_TO_PCM_2_VT);
    return 1;
}

static int CAL2_MIC2_Preamp_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL_VolumeSet(D1981_AnaMIC2_TO_PCM_2_VT,ucontrol->value.integer.value[0]);
    return 1;

}

static int CAL2_MIC2_PGA_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AnaMIC2_TO_PCM_2_VT);
    return 1;

}

static int CAL2_MIC2_PGA_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL2_VolumeSet(D1981_AnaMIC2_TO_PCM_2_VT,ucontrol->value.integer.value[0]);
    return 1;
}

static int CAL2_MIC2_ADC_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

    return 1;
}

static int CAL2_MIC2_ADC_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    return 1;

}

static int CAL2_MIC3_Preamp_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AnaMIC1_LOUD_TO_PCM_2_VT);
    return 1;

}

static int CAL2_MIC3_Preamp_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL_VolumeSet(D1981_AnaMIC1_LOUD_TO_PCM_2_VT,ucontrol->value.integer.value[0]);
    return 1;
}

static int CAL2_MIC3_PGA_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AnaMIC1_LOUD_TO_PCM_2_VT);
    return 1;

    
}

static int CAL2_MIC3_PGA_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL2_VolumeSet(D1981_AnaMIC1_LOUD_TO_PCM_2_VT,ucontrol->value.integer.value[0]);
    return 1;

}

static int CAL2_MIC3_ADC_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    return 1;

}

static int CAL2_MIC3_ADC_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    return 1;

}
static int CAL2_DAC_PCM_HP_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_PCM_2_TO_HS_VIA_DAC12_VT);
	return(1);

}
static int CAL2_DAC_PCM_HP_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL_VolumeSet(D1981_PCM_2_TO_HS_VIA_DAC12_VT,ucontrol->value.integer.value[0]);
	return(1);

}

static int CAL2_PCM_HP_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
   ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_PCM_2_TO_HS_VIA_DAC12_VT);
   return(1);


}
static int CAL2_PCM_HP_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

    d1981_CAL2_VolumeSet(D1981_PCM_2_TO_HS_VIA_DAC12_VT,ucontrol->value.integer.value[0]);
	return(1);
}
static int CAL2_DAC_PCM_SPK_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
   ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_PCM_2_TO_SPKR_VIA_DAC3_VT);
   return(1);

}
static int CAL2_DAC_PCM_SPK_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{


    d1981_CAL_VolumeSet(D1981_PCM_2_TO_SPKR_VIA_DAC3_VT,ucontrol->value.integer.value[0]);
	return(1);
}

static int CAL2_SPK_OUT3_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_PCM_2_TO_SPKR_VIA_DAC3_VT);
   return(1);

}
static int CAL2_SPK_OUT3_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL2_VolumeSet(D1981_PCM_2_TO_SPKR_VIA_DAC3_VT,ucontrol->value.integer.value[0]);
	return(1);

}
static int CAL2_DAC_PCM_RCV_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
   ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_PCM_2_TO_RCV_VIA_DAC4_VT);
   return(1);

}
static int CAL2_DAC_PCM_RCV_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL_VolumeSet(D1981_PCM_2_TO_RCV_VIA_DAC4_VT,ucontrol->value.integer.value[0]);
	return(1);

}

static int CAL2_RCV_OUT4_VT_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
   ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_PCM_2_TO_RCV_VIA_DAC4_VT);
   return(1);

}
static int CAL2_RCV_OUT4_VT_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL2_VolumeSet(D1981_PCM_2_TO_RCV_VIA_DAC4_VT,ucontrol->value.integer.value[0]);
	return(1);

}

static int CAL2_RING_DAC_1_2_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
   ucontrol->value.integer.value[0]=d1981_CAL_Ring_DACVolumeGet(0);
   return(1);

}
static int CAL2_RING_DAC_1_2_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL_Ring_DACVolumeSet(0,ucontrol->value.integer.value[0]);
	return(1);

}

static int CAL2_RING_DAC_3_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
   ucontrol->value.integer.value[0]=d1981_CAL_Ring_DACVolumeGet(1);
   return(1);

}
static int CAL2_RING_DAC_3_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL_Ring_DACVolumeSet(1,ucontrol->value.integer.value[0]);
	return(1);

}

static int CAL2_RING_HPL_R_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
   ucontrol->value.integer.value[0]=d1981_CAL_Ring_ALAVolumeGet(0);
   return(1);

}
static int CAL2_RING_HPL_R_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL_Ring_ALAVolumeSet(0,ucontrol->value.integer.value[0]);
	return(1);

}

static int CAL2_RING_LIN3_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
   ucontrol->value.integer.value[0]=d1981_CAL_Ring_ALAVolumeGet(1);
   return(1);

}
static int CAL2_RING_LIN3_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL_Ring_ALAVolumeSet(1,ucontrol->value.integer.value[0]);
	return(1);

}


static int CAL2_MIC1_Preamp_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AnaMIC1_TO_PCM_2_VOIP);
    return 1;

}

static int CAL2_MIC1_Preamp_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL_VolumeSet(D1981_AnaMIC1_TO_PCM_2_VOIP,ucontrol->value.integer.value[0]);
    return 1;
}

static int CAL2_MIC1_PGA_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AnaMIC1_TO_PCM_2_VOIP);
    return 1;

}

static int CAL2_MIC1_PGA_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL2_VolumeSet(D1981_AnaMIC1_TO_PCM_2_VOIP,ucontrol->value.integer.value[0]);
    return 1;
}

static int CAL2_MIC1_ADC_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    return 1;

}

static int CAL2_MIC1_ADC_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{    
    return 1;
}

static int CAL2_MIC2_Preamp_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

    ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AnaMIC2_TO_PCM_2_VOIP);
    return 1;
}

static int CAL2_MIC2_Preamp_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL_VolumeSet(D1981_AnaMIC2_TO_PCM_2_VOIP,ucontrol->value.integer.value[0]);
    return 1;

}

static int CAL2_MIC2_PGA_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AnaMIC2_TO_PCM_2_VOIP);
    return 1;

}

static int CAL2_MIC2_PGA_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL2_VolumeSet(D1981_AnaMIC2_TO_PCM_2_VOIP,ucontrol->value.integer.value[0]);
    return 1;
}

static int CAL2_MIC2_ADC_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

    return 1;
}

static int CAL2_MIC2_ADC_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    return 1;

}

static int CAL2_MIC3_Preamp_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_AnaMIC1_LOUD_TO_PCM_2_VOIP);
    return 1;

}

static int CAL2_MIC3_Preamp_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL_VolumeSet(D1981_AnaMIC1_LOUD_TO_PCM_2_VOIP,ucontrol->value.integer.value[0]);
    return 1;
}

static int CAL2_MIC3_PGA_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_AnaMIC1_LOUD_TO_PCM_2_VOIP);
    return 1;

    
}

static int CAL2_MIC3_PGA_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL2_VolumeSet(D1981_AnaMIC1_LOUD_TO_PCM_2_VOIP,ucontrol->value.integer.value[0]);
    return 1;

}

static int CAL2_MIC3_ADC_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    return 1;

}

static int CAL2_MIC3_ADC_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    return 1;

}
static int CAL2_DAC_PCM_HP_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_PCM_2_TO_HS_VIA_DAC12_VOIP);
	return(1);

}
static int CAL2_DAC_PCM_HP_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL_VolumeSet(D1981_PCM_2_TO_HS_VIA_DAC12_VOIP,ucontrol->value.integer.value[0]);
	return(1);

}

static int CAL2_PCM_HP_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
   ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_PCM_2_TO_HS_VIA_DAC12_VOIP);
   return(1);


}
static int CAL2_PCM_HP_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{

    d1981_CAL2_VolumeSet(D1981_PCM_2_TO_HS_VIA_DAC12_VOIP,ucontrol->value.integer.value[0]);
	return(1);
}
static int CAL2_DAC_PCM_SPK_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
   ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_PCM_2_TO_SPKR_VIA_DAC3_VOIP);
   return(1);

}
static int CAL2_DAC_PCM_SPK_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{


    d1981_CAL_VolumeSet(D1981_PCM_2_TO_SPKR_VIA_DAC3_VOIP,ucontrol->value.integer.value[0]);
	return(1);
}

static int CAL2_SPK_OUT3_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_PCM_2_TO_SPKR_VIA_DAC3_VOIP);
   return(1);

}
static int CAL2_SPK_OUT3_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL2_VolumeSet(D1981_PCM_2_TO_SPKR_VIA_DAC3_VOIP,ucontrol->value.integer.value[0]);
	return(1);

}
static int CAL2_DAC_PCM_RCV_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
   ucontrol->value.integer.value[0]=d1981_CAL_VolumeGet(D1981_PCM_2_TO_RCV_VIA_DAC4_VOIP);
   return(1);

}
static int CAL2_DAC_PCM_RCV_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL_VolumeSet(D1981_PCM_2_TO_RCV_VIA_DAC4_VOIP,ucontrol->value.integer.value[0]);
	return(1);

}

static int CAL2_RCV_OUT4_VOIP_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
   ucontrol->value.integer.value[0]=d1981_CAL2_VolumeGet(D1981_PCM_2_TO_RCV_VIA_DAC4_VOIP);
   return(1);

}
static int CAL2_RCV_OUT4_VOIP_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    d1981_CAL2_VolumeSet(D1981_PCM_2_TO_RCV_VIA_DAC4_VOIP,ucontrol->value.integer.value[0]);
	return(1);

}

const struct snd_kcontrol_new d1981_gain_controls[MAX_D1981_PATH] = {
	SOC_SINGLE_EXT("CAL_AnaMIC1_TO_I2S_1", CAL_AnaMIC1_TO_I2S_1, 0, 0x07, 0,         d1981_CAL_AnaMIC1_TO_I2S_1_get,  		d1981_CAL_AnaMIC1_TO_I2S_1_set),
	SOC_SINGLE_EXT("CAL_AnaMIC1_LOUD_TO_I2S_1", CAL_AnaMIC1_LOUD_TO_I2S_1, 0, 0x07, 0,         d1981_CAL_AnaMIC1_LOUD_TO_I2S_1_get,  	d1981_CAL2_Temp_set),
	SOC_SINGLE_EXT("CAL_AnaMIC2_TO_I2S_1", CAL_AnaMIC2_TO_I2S_1, 0, 0x07, 0,       d1981_CAL_AnaMIC2_TO_I2S_1_get,    d1981_CAL_AnaMIC2_TO_I2S_1_set),
	SOC_SINGLE_EXT("CAL_AUX1_TO_I2S_1", CAL_AUX1_TO_I2S_1, 0, 0xff, 0,             d1981_CAL_AUX1_TO_I2S_1_get,       d1981_CAL_AUX1_TO_I2S_1_set),
	SOC_SINGLE_EXT("CAL_AnaMIC1_TO_PCM_2", CAL_AnaMIC1_TO_PCM_2, 0, 0x07, 0, d1981_CAL_AnaMIC1_TO_PCM_2_get,	d1981_CAL_AnaMIC1_TO_PCM_2_set),
	SOC_SINGLE_EXT("CAL_AnaMIC1_LOUD_TO_PCM_2", CAL_AnaMIC1_LOUD_TO_PCM_2, 0, 0x07, 0, d1981_CAL_AnaMIC1_LOUD_TO_PCM_2_get,	d1981_CAL_AnaMIC1_LOUD_TO_PCM_2_set),

	SOC_SINGLE_EXT("CAL_AnaMIC2_TO_PCM_2", CAL_AnaMIC2_TO_PCM_2, 0, 0x07, 0,           d1981_CAL_AnaMIC2_TO_PCM_2_get,		d1981_CAL_AnaMIC2_TO_PCM_2_set),
	SOC_SINGLE_EXT("CAL_AUX1_TO_PCM_2", CAL_AUX1_TO_PCM_2, 0, 0xff, 0,         d1981_CAL_AUX1_TO_PCM_2_get,  		d1981_CAL_AUX1_TO_PCM_2_set),
	SOC_SINGLE_EXT("CAL_I2S_1_TO_HS_VIA_DAC12", CAL_I2S_1_TO_HS_VIA_DAC12, 0, 0x0f, 0,         d1981_CAL_I2S_1_TO_HS_VIA_DAC12_get,  	d1981_CAL_I2S_1_TO_HS_VIA_DAC12_set),
	SOC_SINGLE_EXT("CAL_I2S_1_TO_SPKR_VIA_DAC3", CAL_I2S_1_TO_SPKR_VIA_DAC3, 0, 0x0f, 0,       d1981_CAL_I2S_1_TO_SPKR_VIA_DAC3_get,    d1981_CAL_I2S_1_TO_SPKR_VIA_DAC3_set),
	SOC_SINGLE_EXT("CAL_I2S_1_TO_RCV_VIA_DAC4", CAL_I2S_1_TO_RCV_VIA_DAC4, 0, 0x0f, 0,             d1981_CAL_I2S_1_TO_RCV_VIA_DAC4_get,       d1981_CAL_I2S_1_TO_RCV_VIA_DAC4_set),

	SOC_SINGLE_EXT("CAL_I2S_1_TO_LINEOUT_VIA_DAC4", CAL_I2S_1_TO_LINEOUT_VIA_DAC4, 0, 0xff, 0, d1981_CAL_I2S_1_TO_LINEOUT_VIA_DAC4_get,	d1981_CAL_I2S_1_TO_LINEOUT_VIA_DAC4_set),
	SOC_SINGLE_EXT("CAL_PCM_2_TO_HS_VIA_DAC12", CAL_PCM_2_TO_HS_VIA_DAC12, 0, 0x0f, 0,           d1981_CAL_PCM_2_TO_HS_VIA_DAC12_get,		d1981_CAL_PCM_2_TO_HS_VIA_DAC12_set),
	SOC_SINGLE_EXT("CAL_PCM_2_TO_SPKR_VIA_DAC3", CAL_PCM_2_TO_SPKR_VIA_DAC3, 0, 0x0f, 0,           		d1981_CAL_PCM_2_TO_SPKR_VIA_DAC3_get,		d1981_CAL_PCM_2_TO_SPKR_VIA_DAC3_set),	// development testing
	SOC_SINGLE_EXT("CAL_PCM_2_TO_RCV_VIA_DAC4", CAL_PCM_2_TO_RCV_VIA_DAC4, 0, 0x0f, 0, d1981_CAL_PCM_2_TO_RCV_VIA_DAC4_get,	d1981_CAL_PCM_2_TO_RCV_VIA_DAC4_set),
	SOC_SINGLE_EXT("CAL_PCM_2_TO_LINEOUT_VIA_DAC4", CAL_PCM_2_TO_LINEOUT_VIA_DAC4, 0, 0xff, 0,           d1981_CAL_PCM_2_TO_LINEOUT_VIA_DAC4_get,		d1981_CAL_PCM_2_TO_LINEOUT_VIA_DAC4_set),

	SOC_SINGLE_EXT("CAL_AUX1_TO_HS_VIA_DAC12", CAL_AUX1_TO_HS_VIA_DAC12, 0, 0x0f, 0,           		d1981_CAL_AUX1_TO_HS_VIA_DAC12_get,		d1981_CAL_AUX1_TO_HS_VIA_DAC12_set),	// development testing
	SOC_SINGLE_EXT("CAL_AUX1_SPKR_VIA_DAC3", CAL_AUX1_SPKR_VIA_DAC3, 0, 0x0f, 0,           		d1981_CAL_AUX1_SPKR_VIA_DAC3_get,		d1981_CAL_AUX1_SPKR_VIA_DAC3_set),	// development testing
	SOC_SINGLE_EXT("CAL_AUX1_TO_RCV_VIA_DAC4", CAL_AUX1_TO_RCV_VIA_DAC4, 0, 0x0f, 0,           		d1981_CAL_AUX1_TO_RCV_VIA_DAC4_get,		d1981_CAL_AUX1_TO_RCV_VIA_DAC4_set),	// development testing
	SOC_SINGLE_EXT("CAL_AUX1_TO_LINEOUT_VIA_DAC4", CAL_AUX1_TO_LINEOUT_VIA_DAC4, 0, 0xff, 0,           		d1981_CAL_AUX1_TO_LINEOUT_VIA_DAC4_get,		d1981_CAL_AUX1_TO_LINEOUT_VIA_DAC4_set),	// development testing
};     

const struct snd_kcontrol_new d1981_gain2_controls[MAX_D1981_PATH] = {
	SOC_SINGLE_EXT("CAL2_AnaMIC1_TO_I2S_1", CAL2_AnaMIC1_TO_I2S_1, 0, 0x1f, 0,         d1981_CAL2_AnaMIC1_TO_I2S_1_get,  		d1981_CAL2_AnaMIC1_TO_I2S_1_set),
	SOC_SINGLE_EXT("CAL2_AnaMIC1_LOUD_TO_I2S_1", CAL2_AnaMIC1_LOUD_TO_I2S_1, 0, 0x1f, 0,         d1981_CAL2_AnaMIC1_LOUD_TO_I2S_1_get,  	d1981_CAL2_AnaMIC1_LOUD_TO_I2S_1_set),
	SOC_SINGLE_EXT("CAL2_AnaMIC2_TO_I2S_1", CAL2_AnaMIC2_TO_I2S_1, 0, 0x1f, 0,       d1981_CAL2_AnaMIC2_TO_I2S_1_get,    d1981_CAL2_AnaMIC2_TO_I2S_1_set),
	SOC_SINGLE_EXT("CAL2_AUX1_TO_I2S_1", CAL2_AUX1_TO_I2S_1, 0, 0xff, 0,             d1981_CAL2_AUX1_TO_I2S_1_get,       d1981_CAL2_AUX1_TO_I2S_1_set),
	SOC_SINGLE_EXT("CAL2_AnaMIC1_TO_PCM_2", CAL2_AnaMIC1_TO_PCM_2, 0, 0x1f, 0, d1981_CAL2_AnaMIC1_TO_PCM_2_get,	d1981_CAL2_AnaMIC1_TO_PCM_2_set),
	SOC_SINGLE_EXT("CAL2_AnaMIC1_LOUD_TO_PCM_2", CAL2_AnaMIC1_LOUD_TO_PCM_2, 0, 0x1f, 0, d1981_CAL2_AnaMIC1_LOUD_TO_PCM_2_get,	d1981_CAL2_AnaMIC1_LOUD_TO_PCM_2_set),

	SOC_SINGLE_EXT("CAL2_AnaMIC2_TO_PCM_2", CAL2_AnaMIC2_TO_PCM_2, 0, 0x1f, 0,           d1981_CAL2_AnaMIC2_TO_PCM_2_get,		d1981_CAL2_AnaMIC2_TO_PCM_2_set),
	SOC_SINGLE_EXT("CAL2_AUX1_TO_PCM_2", CAL2_AUX1_TO_PCM_2, 0, 0xff, 0,         d1981_CAL2_AUX1_TO_PCM_2_get,  		d1981_CAL2_AUX1_TO_PCM_2_set),
	SOC_SINGLE_EXT("CAL2_I2S_1_TO_HS_VIA_DAC12", CAL2_I2S_1_TO_HS_VIA_DAC12, 0, 0x77, 0,         d1981_CAL2_I2S_1_TO_HS_VIA_DAC12_get,  	d1981_CAL2_I2S_1_TO_HS_VIA_DAC12_set),
	SOC_SINGLE_EXT("CAL2_I2S_1_TO_SPKR_VIA_DAC3", CAL2_I2S_1_TO_SPKR_VIA_DAC3, 0, 0x77, 0,       d1981_CAL2_I2S_1_TO_SPKR_VIA_DAC3_get,    d1981_CAL2_I2S_1_TO_SPKR_VIA_DAC3_set),
	SOC_SINGLE_EXT("CAL2_I2S_1_TO_RCV_VIA_DAC4", CAL2_I2S_1_TO_RCV_VIA_DAC4, 0, 0x77, 0,             d1981_CAL2_I2S_1_TO_RCV_VIA_DAC4_get,       d1981_CAL2_I2S_1_TO_RCV_VIA_DAC4_set),

	SOC_SINGLE_EXT("CAL2_I2S_1_TO_LINEOUT_VIA_DAC4", CAL2_I2S_1_TO_LINEOUT_VIA_DAC4, 0, 0xff, 0, d1981_CAL2_I2S_1_TO_LINEOUT_VIA_DAC4_get,	d1981_CAL2_I2S_1_TO_LINEOUT_VIA_DAC4_set),
	SOC_SINGLE_EXT("CAL2_PCM_2_TO_HS_VIA_DAC12", CAL2_PCM_2_TO_HS_VIA_DAC12, 0, 0x77, 0,           d1981_CAL2_PCM_2_TO_HS_VIA_DAC12_get,		d1981_CAL2_PCM_2_TO_HS_VIA_DAC12_set),
	SOC_SINGLE_EXT("CAL2_PCM_2_TO_SPKR_VIA_DAC3", CAL2_PCM_2_TO_SPKR_VIA_DAC3, 0, 0x77, 0,           		d1981_CAL2_PCM_2_TO_SPKR_VIA_DAC3_get,		d1981_CAL2_PCM_2_TO_SPKR_VIA_DAC3_set),	// development testing
	SOC_SINGLE_EXT("CAL2_PCM_2_TO_RCV_VIA_DAC4", CAL2_PCM_2_TO_RCV_VIA_DAC4, 0, 0x77, 0, d1981_CAL2_PCM_2_TO_RCV_VIA_DAC4_get,	d1981_CAL2_PCM_2_TO_RCV_VIA_DAC4_set),
	SOC_SINGLE_EXT("CAL2_PCM_2_TO_LINEOUT_VIA_DAC4", CAL2_PCM_2_TO_LINEOUT_VIA_DAC4, 0, 0xff, 0,           d1981_CAL2_PCM_2_TO_LINEOUT_VIA_DAC4_get,		d1981_CAL2_PCM_2_TO_LINEOUT_VIA_DAC4_set),

	SOC_SINGLE_EXT("CAL2_AUX1_TO_HS_VIA_DAC12", CAL2_AUX1_TO_HS_VIA_DAC12, 0, 0x77, 0,           		d1981_CAL2_AUX1_TO_HS_VIA_DAC12_get,		d1981_CAL2_AUX1_TO_HS_VIA_DAC12_set),	// development testing
	SOC_SINGLE_EXT("CAL2_AUX1_SPKR_VIA_DAC3", CAL2_AUX1_SPKR_VIA_DAC3, 0, 0x77, 0,           		d1981_CAL2_AUX1_SPKR_VIA_DAC3_get,		d1981_CAL2_AUX1_SPKR_VIA_DAC3_set),	// development testing
	SOC_SINGLE_EXT("CAL2_AUX1_TO_RCV_VIA_DAC4", CAL2_AUX1_TO_RCV_VIA_DAC4, 0, 0x77, 0,           		d1981_CAL2_AUX1_TO_RCV_VIA_DAC4_get,		d1981_CAL2_AUX1_TO_RCV_VIA_DAC4_set),	// development testing
	SOC_SINGLE_EXT("CAL2_AUX1_TO_LINEOUT_VIA_DAC4", CAL2_AUX1_TO_LINEOUT_VIA_DAC4, 0, 0x77, 0,           		d1981_CAL2_AUX1_TO_LINEOUT_VIA_DAC4_get,		d1981_CAL2_AUX1_TO_LINEOUT_VIA_DAC4_set),	// development testing
};     

const struct snd_kcontrol_new d1981_gain3_controls[D1981_MAX_OP_CONTROL] = {
	SOC_SINGLE_EXT("CAL2_47", CAL2_47, 0, 0xff, 0,         d1981_CAL2_Temp_get,  		d1981_CAL2_Temp_set),
	SOC_SINGLE_EXT("CAL2_48", CAL2_48, 0, 0xff, 0,         d1981_CAL2_Temp_get,  	d1981_CAL2_Temp_set),
	SOC_SINGLE_EXT("CAL2_49", CAL2_49, 0, 0xff, 0,       d1981_CAL2_Temp_get,    d1981_CAL2_Temp_set),
	SOC_SINGLE_EXT("CAL2_50", CAL2_50, 0, 0xff, 0,             d1981_CAL2_Temp_get,       d1981_CAL2_Temp_set),
	SOC_SINGLE_EXT("CAL2_51", CAL2_51, 0, 0xff, 0, d1981_CAL2_Temp_get,	d1981_CAL2_Temp_set),

	SOC_SINGLE_EXT("CAL2_52_MIC3_LOUD", CAL2_52, 0, 0xff, 0, d1981_CAL_AnaMIC1_LOUD_TO_PCM_2_get,	d1981_CAL_AnaMIC1_LOUD_TO_PCM_2_set),
	SOC_SINGLE_EXT("CAL2_53_MIC3_LOUD", CAL2_53, 0, 0xff, 0,           d1981_CAL2_AnaMIC1_LOUD_TO_PCM_2_get,		d1981_CAL2_AnaMIC1_LOUD_TO_PCM_2_set),
	SOC_SINGLE_EXT("CAL2_54_MIC3_LOUD", CAL2_54, 0, 0xff, 0,         d1981_CAL2_Temp_get,  		d1981_CAL2_Temp_set),
	SOC_SINGLE_EXT("CAL2_55", CAL2_55, 0, 0xff, 0,         d1981_CAL2_Temp_get,  	d1981_CAL2_Temp_set),
	SOC_SINGLE_EXT("CAL2_56", CAL2_56, 0, 0xff, 0,       d1981_CAL2_Temp_get,    d1981_CAL2_Temp_set),

	SOC_SINGLE_EXT("CAL2_57_VR_LOUD", CAL2_57, 0, 0xff, 0,             d1981_CAL_AnaMIC1_LOUD_TO_I2S_1_get,       d1981_CAL_AnaMIC1_LOUD_TO_I2S_1_set),
	SOC_SINGLE_EXT("CAL2_58_VR_LOUD", CAL2_58, 0, 0xff, 0, d1981_CAL2_AnaMIC1_LOUD_TO_I2S_1_get,	d1981_CAL2_AnaMIC1_LOUD_TO_I2S_1_set),
	SOC_SINGLE_EXT("CAL2_59", CAL2_59, 0, 0xff, 0,           d1981_CAL2_Temp_get,		d1981_CAL2_Temp_set),
	SOC_SINGLE_EXT("CAL2_60", CAL2_60, 0, 0xff, 0,         d1981_CAL2_AUX1_PGA_get,		d1981_CAL2_AUX1_PGA_set),	// development testing
	SOC_SINGLE_EXT("CAL2_61", CAL2_61, 0, 0xff, 0,           d1981_CAL2_Temp_get,		d1981_CAL2_Temp_set),

	SOC_SINGLE_EXT("CAL2_62", CAL2_62, 0, 0xff, 0,         d1981_CAL2_Temp_get,  		d1981_CAL2_Temp_set),
	SOC_SINGLE_EXT("CAL2_63", CAL2_63, 0, 0xff, 0,         d1981_CAL2_Temp_get,  	d1981_CAL2_Temp_set),
	SOC_SINGLE_EXT("CAL2_64", CAL2_64, 0, 0xff, 0,       d1981_CAL2_Temp_get,    d1981_CAL2_Temp_set),
	SOC_SINGLE_EXT("CAL2_65", CAL2_65, 0, 0xff, 0,             d1981_CAL2_DSP_75_get,       d1981_CAL2_DSP_75_set),
	SOC_SINGLE_EXT("CAL2_66", CAL2_66, 0, 0xff, 0, d1981_CAL2_DSP_76_get,	d1981_CAL2_DSP_76_set),

	SOC_SINGLE_EXT("CAL2_67", CAL2_67, 0, 0xff, 0,           d1981_CAL2_DSP_77_get,		d1981_CAL2_DSP_77_set),
	SOC_SINGLE_EXT("CAL2_68", CAL2_68, 0, 0xff, 0,           d1981_CAL2_DSP_78_get,		d1981_CAL2_DSP_78_set),
	SOC_SINGLE_EXT("CAL2_69", CAL2_69, 0, 0xff, 0,           d1981_CAL2_RegisterRead_get,		d1981_CAL2_RegisterRead_set),
	SOC_SINGLE_EXT("CAL2_70", CAL2_70, 0, 0xff, 0,           d1981_CAL2_RegisterWrite_get,		d1981_CAL2_RegisterWrite_set),
/////////////VT 
	SOC_SINGLE_EXT("CAL2_MIC1_Preamp_VT", CAL2_71, 0, 0xff, 0, CAL2_MIC1_Preamp_VT_get,	CAL2_MIC1_Preamp_VT_set),
	SOC_SINGLE_EXT("CAL2_MIC1_PGA_VT", CAL2_72, 0, 0xff, 0,           CAL2_MIC1_PGA_VT_get,		CAL2_MIC1_PGA_VT_set),
	SOC_SINGLE_EXT("CAL2_MIC1_ADC_VT", CAL2_73, 0, 0xff, 0,           		CAL2_MIC1_ADC_VT_get,		CAL2_MIC1_ADC_VT_set),	

	SOC_SINGLE_EXT("CAL2_MIC2_Preamp_VT", CAL2_74, 0, 0xff, 0,           		CAL2_MIC2_Preamp_VT_get,		CAL2_MIC2_Preamp_VT_set),	
	SOC_SINGLE_EXT("CAL2_MIC2_PGA_VT", CAL2_75, 0, 0xff, 0,           		CAL2_MIC2_PGA_VT_get,		CAL2_MIC2_PGA_VT_set),	
	SOC_SINGLE_EXT("CAL2_MIC2_ADC_VT", CAL2_76, 0, 0xff, 0,           		CAL2_MIC2_ADC_VT_get,		CAL2_MIC2_ADC_VT_set),	

	SOC_SINGLE_EXT("CAL2_MIC3_Preamp_VT", CAL2_77, 0, 0xff, 0, CAL2_MIC3_Preamp_VT_get,	CAL2_MIC3_Preamp_VT_set),
	SOC_SINGLE_EXT("CAL2_MIC3_PGA_VT", CAL2_78, 0, 0xff, 0,           CAL2_MIC3_PGA_VT_get,		CAL2_MIC3_PGA_VT_set),
	SOC_SINGLE_EXT("CAL2_MIC3_ADC_VT", CAL2_79, 0, 0xff, 0,           		CAL2_MIC3_ADC_VT_get,		CAL2_MIC3_ADC_VT_set),	

	SOC_SINGLE_EXT("CAL2_DAC_PCM_HP_VT", CAL2_80, 0, 0xff, 0,           		CAL2_DAC_PCM_HP_VT_get,		CAL2_DAC_PCM_HP_VT_set),	
	SOC_SINGLE_EXT("CAL2_HP_VT", CAL2_81, 0, 0xff, 0,           		CAL2_PCM_HP_VT_get,		CAL2_PCM_HP_VT_set),	
	SOC_SINGLE_EXT("CAL2_DAC_PCM_SPK_VT", CAL2_82, 0, 0xff, 0,           		CAL2_DAC_PCM_SPK_VT_get,		CAL2_DAC_PCM_SPK_VT_set),	
	SOC_SINGLE_EXT("CAL2_SPK_OUT3_VT", CAL2_83, 0, 0xff, 0,           		CAL2_SPK_OUT3_VT_get,		CAL2_SPK_OUT3_VT_set),	
	SOC_SINGLE_EXT("CAL2_DAC_PCM_RCV_VT", CAL2_84, 0, 0xff, 0,           		CAL2_DAC_PCM_RCV_VT_get,		CAL2_DAC_PCM_RCV_VT_set),	
	SOC_SINGLE_EXT("CAL2_RCV_OUT4_VT", CAL2_85, 0, 0xff, 0,           		CAL2_RCV_OUT4_VT_get,		CAL2_RCV_OUT4_VT_set),	
//////////////RINGTONE
	SOC_SINGLE_EXT("CAL2_RING_DAC_1_2", CAL2_86, 0, 0xff, 0,           		CAL2_RING_DAC_1_2_get,		CAL2_RING_DAC_1_2_set),	
	SOC_SINGLE_EXT("CAL2_RING_DAC_3", CAL2_87, 0, 0xff, 0,           		CAL2_RING_DAC_3_get,		CAL2_RING_DAC_3_set),	
	SOC_SINGLE_EXT("CAL2_RING_HPL_R", CAL2_88, 0, 0xff, 0,           		CAL2_RING_HPL_R_get,		CAL2_RING_HPL_R_set),	
	SOC_SINGLE_EXT("CAL2_RING_LIN3", CAL2_89, 0, 0xff, 0,           		CAL2_RING_LIN3_get,		CAL2_RING_LIN3_set),	
///////////////VOIP
	SOC_SINGLE_EXT("CAL2_MIC1_Preamp_VOIP", CAL2_90, 0, 0xff, 0, CAL2_MIC1_Preamp_VOIP_get,	CAL2_MIC1_Preamp_VOIP_set),
	SOC_SINGLE_EXT("CAL2_MIC1_PGA_VOIP", CAL2_91, 0, 0xff, 0,           CAL2_MIC1_PGA_VOIP_get,		CAL2_MIC1_PGA_VOIP_set),
	SOC_SINGLE_EXT("CAL2_MIC1_ADC_VOIP", CAL2_92, 0, 0xff, 0,           		CAL2_MIC1_ADC_VOIP_get,		CAL2_MIC1_ADC_VOIP_set),	

	SOC_SINGLE_EXT("CAL2_MIC2_Preamp_VOIP", CAL2_93, 0, 0xff, 0,           		CAL2_MIC2_Preamp_VOIP_get,		CAL2_MIC2_Preamp_VOIP_set),	
	SOC_SINGLE_EXT("CAL2_MIC2_PGA_VOIP", CAL2_94, 0, 0xff, 0,           		CAL2_MIC2_PGA_VOIP_get,		CAL2_MIC2_PGA_VOIP_set),	
	SOC_SINGLE_EXT("CAL2_MIC2_ADC_VOIP", CAL2_95, 0, 0xff, 0,           		CAL2_MIC2_ADC_VOIP_get,		CAL2_MIC2_ADC_VOIP_set),	

	SOC_SINGLE_EXT("CAL2_MIC3_Preamp_VOIP", CAL2_96, 0, 0xff, 0, CAL2_MIC3_Preamp_VOIP_get,	CAL2_MIC3_Preamp_VOIP_set),
	SOC_SINGLE_EXT("CAL2_MIC3_PGA_VOIP", CAL2_97, 0, 0xff, 0,           CAL2_MIC3_PGA_VOIP_get,		CAL2_MIC3_PGA_VOIP_set),
	SOC_SINGLE_EXT("CAL2_MIC3_ADC_VOIP", CAL2_98, 0, 0xff, 0,           		CAL2_MIC3_ADC_VOIP_get,		CAL2_MIC3_ADC_VOIP_set),	

	SOC_SINGLE_EXT("CAL2_DAC_I2S_HP_VOIP", CAL2_99, 0, 0xff, 0,           		CAL2_DAC_PCM_HP_VOIP_get,		CAL2_DAC_PCM_HP_VOIP_set),	
	SOC_SINGLE_EXT("CAL2_HP_VOIP", CAL2_100, 0, 0xff, 0,           		CAL2_PCM_HP_VOIP_get,		CAL2_PCM_HP_VOIP_set),	
	SOC_SINGLE_EXT("CAL2_DAC_I2S_SPK_VOIP", CAL2_101, 0, 0xff, 0,           		CAL2_DAC_PCM_SPK_VOIP_get,		CAL2_DAC_PCM_SPK_VOIP_set),	
	SOC_SINGLE_EXT("CAL2_SPK_OUT3_VOIP", CAL2_102, 0, 0xff, 0,           		CAL2_SPK_OUT3_VOIP_get,		CAL2_SPK_OUT3_VOIP_set),	
	SOC_SINGLE_EXT("CAL2_DAC_I2S_RCV_VOIP", CAL2_103, 0, 0xff, 0,           		CAL2_DAC_PCM_RCV_VOIP_get,		CAL2_DAC_PCM_RCV_VOIP_set),	
	SOC_SINGLE_EXT("CAL2_RCV_OUT4_VOIP", CAL2_104, 0, 0xff, 0,           		CAL2_RCV_OUT4_VOIP_get,		CAL2_RCV_OUT4_VOIP_set),	

//	SOC_SINGLE_EXT("CAL2_DAC_PCM_SPK_VT", CAL2_82, 0, 0x77, 0,           		d1981_CAL2_AUX1_TO_LINEOUT_VIA_DAC4_get,		d1981_CAL2_AUX1_TO_LINEOUT_VIA_DAC4_set),	// development testing
};     

