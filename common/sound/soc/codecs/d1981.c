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
#include <linux/d1982/pmic.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <mach/d1980_hs.h>
#define NORMAL_STARTUP 1
#define HW_TUNNING
#define D1981_FAST_LOADING
//#define D1982_DELAYED_WORK
#define D1981_PCM_RECORDING
#include "d1981.h"
#include "d1981_reg.h"
#include "d1981_dsp_v0_14.h"
#include "d1981_MB_API_CMDS.h"
#include "d1981_TransitionScript.h"
#include "d1981_LynxSetup.h"

#include "d1981_UseCase1_Reg_v0_13_pcsyncRemove.h"
#include "Jetta_UC1Commander_v1507.h"

#include "d1981_UC2_UC4_Reg_v1491.h"
#include "Jetta_UC2_UC4_8kSRCommander_v1507.h"
#include "Jetta_UC2_UC4_16kSRCommander_v1507.h"
#include "Jetta_UC2_UC4_48kSRCommander_v1507.h"

#include "d1981_UseCase3a_Reg.h"
#include "Jetta_UC3aCommander_v1507.h"

#include "d1981_Jetta_UseCase3b_Reg_v1507.h"
#include "Jetta_UC3bCommander_v1507.h"

#include "d1981_UseCase3c_Reg.h"
#include "d1981_UseCase3c_DSP.h"

#include "d1981_UseCase45_DSP.h"
#include "d1981_I2S1_ToAllDACS_withPLL_hwth.h"

//#define FMRADIO_VOLUME_TUNNING

#define D1981_DEBUG

#ifdef D1981_DEBUG
#define dbg(format, arg...) printk(KERN_INFO "d1981: " format "\n", ## arg)
#else
#define dbg(format, arg...)
#endif


#define ARRAY_AND_SIZE(x)       x, ARRAY_SIZE(x)

//enable multiple I2C transfers
#define I2C_REPEATED_WRITE_MODE	1
#define D1981_VERSION "1.0"

//#define I2C_PAGE_SIZE	32
#define I2C_PAGE_SIZE	4

//static struct snd_soc_device *d1981_socdev;
static struct snd_soc_codec *d1981_codec;

unsigned char gCurrentPath;
unsigned char gCurrentUseCase = 0;
#ifndef FMRADIO_VOLUME_TUNNING
unsigned char gFmRadioVolume =0xff;
unsigned char gFmRadioSpeakerVolume =0xff;
#endif
#define  PATH_SELECT		1
#define  PATH_ENABLE		2
#define  PATH_DISABLE		3
#define  PATH_MUTE			4
#define  PATH_VOLUME_SET	5
#define  CODEC_INIT			6
#define TEST_FUNCTION		7

#define DISABLE 0
#define ENABLE 1

#define WORKING_SPEAKER 0
#define WORKING_HEADSET 1
#define WORKING_RCV 2
#define WORKING_HEADSET_SPEAKER 3
#define WORKING_NONE 8


/*************************************************************************************************************************
ACM Interface. Path to UseCase conversion
*************************************************************************************************************************/

/* mapping from ACM path to Codec Use Case.
# = not supported */
static const unsigned char use_case_table[D1981_TOTAL_VALID_PATHS] =
{
	/* Use Case		   ID 	Path */
	USE_CASE_2A,			/* 0 */
	USE_CASE_2A,			/* 1 */
	USE_CASE_2B,			/* 2 */
	USE_CASE_3B,		/* 3 */
	USE_CASE_1A,			/* 4 */
	USE_CASE_1C,			/* 5 */
	USE_CASE_1B,			/* 6 */
	USE_CASE_UNDEFINED,	/* 7 */
	USE_CASE_3A,		/* 8 */
	USE_CASE_3A,		/* 9 */
	USE_CASE_3C,		/* 10 */
	USE_CASE_3A,		/* 11 */
	USE_CASE_1B,			/* 12 */
	USE_CASE_1C,			/* 13 */
	USE_CASE_1A,			/* 14 */
	USE_CASE_UNDEFINED,	/* 15 */
	USE_CASE_3B,		/* 16 */
	USE_CASE_3B,		/* 17 */
	USE_CASE_3B,		/* 18 */
	USE_CASE_UNDEFINED,	/* 19 */

	USE_CASE_UNDEFINED,	/* 20 */
	USE_CASE_UNDEFINED,	/* 21 */
	USE_CASE_UNDEFINED,	/* 22 */
	USE_CASE_4A,
	USE_CASE_4A,
	USE_CASE_4A,
	USE_CASE_5A,
	USE_CASE_5A,
	USE_CASE_5A,
	USE_CASE_3AB,/*29*/
	
	USE_CASE_6A,			/* 30 */
	USE_CASE_6C,			/* 31 */
	USE_CASE_6B,			/* 32 */
	USE_CASE_6B,			/* 33 */
	USE_CASE_6C,			/* 34 */
	USE_CASE_6A,			/* 35 */
	
	USE_CASE_2A,	/* 36 */
	USE_CASE_2C,	/* 37 */
	USE_CASE_2B,	/* 38 */

	USE_CASE_1A,	/* 39 */
	USE_CASE_1C,	/* 40 */
	USE_CASE_1B,	/* 41 */
	
	USE_CASE_7B,	/* 42 */
	USE_CASE_7C,	/* 43 */
	USE_CASE_7A,	/* 44 */

	USE_CASE_1A,	/* 45 */
	USE_CASE_1C,	/* 46 */
	USE_CASE_1B,	/* 47 */
	USE_CASE_1B,	/* 48 */
	USE_CASE_1C,	/* 49 */
	USE_CASE_1A,	/* 50 */
};

/*** Path settings table - initialised by ACM layer from values in .nvm file  ***/
unsigned char d1981_codec_gain[D1981_TOTAL_VALID_PATHS][D1981_MAX_GAINS_PER_PATH] =
{
// default values {codec_gain, codec_gain2}
	{0x05, 0x1b},		//D1981_AnaMIC1_TO_I2S_1
	{0x07, 0x1f},		//D1981_AnaMIC1_LOUD_TO_I2S_1
	{0x07, 0x1f},		//D1981_AnaMIC2_TO_I2S_1
	{0xff, 0xff},		//not use D1981_AUX1_TO_I2S_1

	{0x04, 0x19},		//D1981_AnaMIC1_TO_PCM_2
	{0x04, 0x19},		//D1981_AnaMIC1_LOUD_TO_PCM_2
	{0x04, 0x19},		//D1981_AnaMIC2_TO_PCM_2
	{0xff, 0xff},		//not use D1981_AUX1_TO_PCM_2

	{0x0f, 0x10},		//D1981_I2S_1_TO_HS_VIA_DAC12
	{0x0f, 0xe},		//D1981_I2S_1_TO_SPKR_VIA_DAC3   //LIN3 DAC2
	{0x0f, 0x10},		//D1981_I2S_1_TO_RCV_VIA_DAC4
	{0xff, 0xff},		//not use D1981_PCM_2_TO_LINEOUT_VIA_DAC4

	{0x02, 0x10},		//D1981_PCM_2_TO_HS_VIA_DAC12
	{0x0f, 0x09},		//D1981_PCM_2_TO_SPKR_VIA_DAC3
	{0x0f, 0x18},		//D1981_PCM_2_TO_RCV_VIA_DAC4
	{0xff, 0xff},		//not use D1981_PCM_2_TO_LINEOUT_VIA_DAC4

	{0x3f, 0x20},		//D1981_AUX1_TO_HS_VIA_DAC12 aux, DAC max
	{0x3f, 0x0c},		//D1981_AUX1_SPKR_VIA_DAC3  aux, DAC max
	{0x3f, 0x10},		//D1981_AUX1_TO_RCV_VIA_DAC4
	{0xff, 0xff},		//not use D1981_AUX1_TO_LINEOUT_VIA_DAC4

	{0, 0},
	{0, 0},
	{0, 0},
	{0x07, 0x10},		//D1981_LOOPBACK_MIC1_TO_HS_VIA_DAC12
	{0x07, 0x10},		//D1981_LOOPBACK_MIC1_TO_SPKR_VIA_DAC3
	{0x07, 0x10},		//D1981_LOOPBACK_MIC1_TO_RCV_VIA_DAC4
	{0x07, 0x10},		//D1981_LOOPBACK_MIC2_TO_HS_VIA_DAC12
	{0x07, 0x10},		//D1981_LOOPBACK_MIC2_TO_SPKR_VIA_DAC3

	{0x07, 0x10},	// 28	//D1981_LOOPBACK_MIC2_TO_RCV_VIA_DAC4
	{0x0, 0x0},		//D1981_I2S_1_TO_SPKR_HS -> move to seperate table

    {0x04, 0x19},		//D1981_AnaMIC1_TO_PCM_2_VT //preamp, pga
	{0x07, 0x10},		//D1981_AnaMIC1_LOUD_TO_PCM_2_VT //preamp, pga
	{0x04, 0x19},	// 32	//D1981_AnaMIC2_TO_PCM_2_VT	 //preamp, pga
	{0x08, 0x10},		//D1981_PCM_2_TO_HS_VIA_DAC12_VT // HP, DAC
	{0x0f, 0x10},		//D1981_PCM_2_TO_SPKR_VIA_DAC3_VT //LIN3,DAC
	{0x0f, 0x1a},		//D1981_PCM_2_TO_RCV_VIA_DAC4_VT //LIN4,DAC

    {0x07, 0x10},	//36	//D1981_AnaMIC1_TO_PCM_2_APPS //preamp, pga
	{0x07, 0x10},		//D1981_AnaMIC1_LOUD_TO_PCM_2_APPS //preamp, pga
	{0x07, 0x10},		//38 D1981_AnaMIC2_TO_PCM_2_APPS	 //preamp, pga
	
	{0x04, 0x19},	//39	//D1981_AnaMIC1_TO_PCM_2_VOIP//preamp, pga
	{0x07, 0x10},		//D1981_AnaMIC1_LOUD_TO_PCM_2_VOIP //preamp, pga
	{0x04, 0x19},	// 41	//D1981_AnaMIC2_TO_PCM_2_VOIP	 //preamp, pga
	{0x08, 0x10},		//D1981_PCM_2_TO_HS_VIA_DAC12_VOIP //HP, DAC
	{0x0f, 0x10},		//D1981_PCM_2_TO_SPKR_VIA_DAC3_VOIP //LIN3, DAC
	{0x0f, 0x1a},		//D1981_PCM_2_TO_RCV_VIA_DAC4_VOIP //LIN4, DAC

	
	{0x0f, 0x1f}, //45		//D1981_AnaMIC1_TO_PCM_2_LB //preamp, pga
	{0x0f, 0x1f},		//D1981_AnaMIC1_LOUD_TO_PCM_2_LB //preamp, pga
	{0x0f, 0x1f},	// 47	//D1981_AnaMIC2_TO_PCM_2_LB	 //preamp, pga

	{0x0f, 0x1f},		//D1981_PCM_2_TO_HS_VIA_DAC12_LB //HP, DAC
	{0x0f, 0x1f},		//D1981_PCM_2_TO_SPKR_VIA_DAC3_LB //LIN3,DAC
	{0x0f, 0x1f},		//D1981_PCM_2_TO_RCV_VIA_DAC4_LB // LIN4, DAC
};

unsigned char d1981_codec_force_speaker_gain[2][D1981_MAX_GAINS_PER_PATH] =
{
    {0x0f, 0x10},		//headset, HPL/HPR, DAC
	{0x0f, 0xe},       //speaker, LIN3, DAC

};

unsigned char d1981_path_direction[D1981_TOTAL_VALID_PATHS]= //0: interface1 I2S 1: interface1 PCM  2: both
{	//MIC to I2S/PCM 1(AP) output
  2,//	D1981_AnaMIC1_TO_I2S_1	  = 0, //from mic1 to ap at recording
  2,//	D1981_AnaMIC1_LOUD_TO_I2S_1 = 1, //from loud mic1 to ap at recording
  2,//  D1981_AnaMIC2_TO_I2S_1	  = 2, //from headset mic1 to ap at recording
  2, //D1981_AUX1_TO_I2S_1       = 3, 

	//MIC to I2S/PCM 2(CP) output
  1,//D1981_AnaMIC1_TO_PCM_2      = 4,
  1, //D1981_AnaMIC1_LOUD_TO_PCM_2 = 5,
  1,//D1981_AnaMIC2_TO_PCM_2     = 6,
  1,//D1981_AUX1_TO_PCM_2         = 7,

	// From I2S/PCM 1 VIA DAC
  0,//	D1981_I2S_1_TO_HS_VIA_DAC12      = 8, //bsw from ap to headset at playback
  0,//	D1981_I2S_1_TO_SPKR_VIA_DAC3     = 9, //bsw frome ap to loud speaker at playback
  0,//	D1981_I2S_1_TO_RCV_VIA_DAC4      = 10,
  0,//	D1981_I2S_1_TO_LINEOUT_VIA_DAC4      = 11,
	
	// From I2S/PCM 2 VIA DAC
  1,//	D1981_PCM_2_TO_HS_VIA_DAC12      = 12, //bsw from cp to reciever at voice call
  1,//	D1981_PCM_2_TO_SPKR_VIA_DAC3     = 13, //bsw frome cp to loud speaker at voice call
  1,//	D1981_PCM_2_TO_RCV_VIA_DAC4      = 14,
  1,//	D1981_PCM_2_TO_LINEOUT_VIA_DAC4      = 15,

	// From AUX VIA DAC
  2,//       D1981_AUX1_TO_HS_VIA_DAC12       = 16,
  2,//	D1981_AUX1_SPKR_VIA_DAC3= 17,
  2,//	D1981_AUX1_TO_RCV_VIA_DAC4     = 18,
  2,//	D1981_AUX1_TO_LINEOUT_VIA_DAC4          = 19,
	
	// Side Tone
  2,//	D1981_IN_L_TO_OUT           = 20,
  2,//	D1981_IN_R_TO_OUT           = 21,
  2,//	D1981_IN_BOTH_TO_OUT        = 22,

  0,//    D1981_LOOPBACK_MIC1_TO_HS_VIA_DAC12         =23,
  0,//    D1981_LOOPBACK_MIC1_TO_SPKR_VIA_DAC3         =24,
  0,//   D1981_LOOPBACK_MIC1_TO_RCV_VIA_DAC4         =25,
  0,//   D1981_LOOPBACK_MIC2_TO_HS_VIA_DAC12         =26,
  0,//  D1981_LOOPBACK_MIC2_TO_SPKR_VIA_DAC3         =27,
  0,//  D1981_LOOPBACK_MIC2_TO_RCV_VIA_DAC4         =28,
  0,//  D1981_I2S_1_TO_SPKR_HS         =29,


	//VT 
  2,//	D1981_AnaMIC1_TO_PCM_2_VT      = 30, //from mic1 to cp at voice call
  2,//	D1981_AnaMIC1_LOUD_TO_PCM_2_VT = 31, //loud speaker to cp at voice call
  2,//	D1981_AnaMIC2_TO_PCM_2_VT     = 32, //from headset mic to cp at voice call

	// From I2S/PCM 2 VIA DAC
  2,//	D1981_PCM_2_TO_HS_VIA_DAC12_VT      = 33, // from cp to headset at voice call
  2,//	D1981_PCM_2_TO_SPKR_VIA_DAC3_VT     = 34, // frome cp to loud speaker at voice call
  2,//	D1981_PCM_2_TO_RCV_VIA_DAC4_VT      = 35, // from cp to reciever at voice call

    //Voice Recording PCM
  2,//   D1981_AnaMIC1_TO_PCM_2_APPS      = 36, //from mic1 to cp at voice recording
  2,//	D1981_AnaMIC1_LOUD_TO_PCM_2_APPS = 37, //loud speaker to cp at voice recording
  2,//	D1981_AnaMIC2_TO_PCM_2_APPS     = 38, //from headset mic to cp at voice recording

 //VOIP
  2,//	D1981_AnaMIC1_TO_PCM_2_VOIP   = 39,//from mic1 to cp at VOIP
  2,//	D1981_AnaMIC1_LOUD_TO_PCM_2_VOIP = 40,//loud speaker to cp at VOIP
  2,//	D1981_AnaMIC2_TO_PCM_2_VOIP     = 41, //  from headset mic to cp at OIP

	// From I2S/PCM 2 VIA DAC
  2,//	D1981_PCM_2_TO_HS_VIA_DAC12_VOIP      = 42, // from cp to headset at VOIP
  2,//	D1981_PCM_2_TO_SPKR_VIA_DAC3_VOIP     = 43, // frome cp to loud speaker at VOIP
  2,//	D1981_PCM_2_TO_RCV_VIA_DAC4_VOIP      = 44, // from cp to reciever at VOIP

 //LOOPBACK
  2,//	D1981_AnaMIC1_TO_PCM_2_LB   = 45,//from mic1 to cp at VOIP
  2,//	D1981_AnaMIC1_LOUD_TO_PCM_2_LB = 46,//loud speaker to cp at VOIP
  2,//	D1981_AnaMIC2_TO_PCM_2_LB     = 47, //  from headset mic to cp at OIP

	// From I2S/PCM 2 VIA DAC
  2,//	D1981_PCM_2_TO_HS_VIA_DAC12_LB      = 48, // from cp to headset at VOIP
  2,//	D1981_PCM_2_TO_SPKR_VIA_DAC3_LB     = 49, // frome cp to loud speaker at VOIP
  2,//	D1981_PCM_2_TO_RCV_VIA_DAC4_LB      = 50, // from cp to reciever at VOIP
	
  2,//	D1981_TOTAL_VALID_PATHS,  				/* Must the be here - counting path amount */
  2,//	D1981_DUMMY_PATH,  						/* for disabling purposes */

  2,//	D1981_PATHS_ENUM_32_BIT = 0xFF /* 32bit enum compiling enforcement */
};

unsigned char d1981_path_enabled[D1981_TOTAL_VALID_PATHS];

unsigned char d1981_enabled_path[3];

//unsigned char d1981_path_flags[D1981_TOTAL_VALID_PATHS];

//unsigned char d1981_aux1_pga_gain=0x35;
unsigned char g_d1981_power_state=0;



/* amixer alsa interface functions */
int d1981PathEnable(u8 path, u8 enable);
int d1981PathMute(u8 path, ACM_AudioMute mute);
int d1981PathVolumeSet(u8 path, ACM_AudioVolume volume,unsigned char dynamic);
int d1981PathVolume2Set(u8 path, ACM_AudioVolume volume,unsigned char dynamic);
int d1981PowerControl(u8 reinit);

extern void enable_oscc_pout(void); //bsw
extern void disable_oscc_pout(void); //bsw

extern int d1981_mic_bias(unsigned char on);
extern void d1980_hookswitch_polling_stop(void);
extern void d1980_hookswitch_polling_start(void);
static int d1981_init(struct snd_soc_device *socdev);
static void d1981_hp_dc_offset_cancelation(void);
static void run_script(const u8 *script, u16 len);
void d1980_ldo15A_set(int V16);

extern u8 d1981_reg_read(u8 reg);
extern int d1981_reg_write(u8 reg, u8 val);
extern int d1981_block_write(u16 num_bytes, const u8 *src);

static u32 wait_for_mailbox_status(void);
static u32 get_mailbox_status(void);
static u32 get_mailbox_counter(void);
/*static*/ int load_use_case(const u8 *use_case, u16 len);
int d1981LoadUseCase(unsigned char use_case, ACM_AudioVolume volume,unsigned char force);
static int _d1981LoadUseCase(unsigned char use_case, ACM_AudioVolume volume,unsigned char force);
static void dsp_download(void);
static void d1981_init_dsp(void);
static void _d1981_init_dsp(void);
static void d1981_init_codec(struct snd_soc_codec *codec /* needed? _kg */);


extern unsigned char g_dlg_4pole_detect;
/* Codec private data */
struct d1981_priv {
#ifdef D1981_PCM_RECORDING
    unsigned int		playback_samplerate;
	unsigned int		capture_samplerate;
#endif	
	int power;
	int classD;
	int hookMode;
	unsigned char working_path;
}; /* currently not used */

/*
 * Register cache
 * We can't read the d1981 register space when we
 * are using 2 wire for device control, so we cache them instead.
 */
static const u8 d1981_reg[256];

// Activated use case
 u8		gActiveUseCase = USE_CASE_UNDEFINED;

// Settings for DC offset cancellation
static u8		gHP_offset_sign[2];

/*************************************************************************************************************************
I2C read/write functions
*************************************************************************************************************************/

/*
 * Read d1981 register cache
 */
static inline unsigned int d1981_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u8 *cache = codec->reg_cache;
	BUG_ON(reg > ARRAY_SIZE(d1981_reg));
	return cache[reg];
}

/*
 * Write d1981 register cache
 */
static inline void d1981_reg_write_cache(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	u8 *cache = codec->reg_cache;

	cache[reg] = value;
}

/*
 * Write to the d1981 register space
 */
static int d1981_soc_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{

	u8 data[2];

	/* data[0] d1981 register offset */
	/* data[1] register data */
	data[0] = reg & 0xff;
	data[1] = value & 0xff;


	/* small delay */
	msleep(1);

	/* write the cache only if hardware write is successful */
	if (d1981_reg_write(reg, value) == 2) {
		if (data[0] < ARRAY_SIZE(d1981_reg)) {
			d1981_reg_write_cache(codec, data[0], data[1]);
		}
		return 0;
	}
	return 0;
}

/*
 * Read from the d1981 register space.
 */
static inline unsigned int d1981_soc_read(struct snd_soc_codec *codec,
					   unsigned int reg)
{
	return d1981_read_reg_cache(codec, reg);
}

/*

Read-modify-write I2C register bits
Read register value, 
AND value with mask
Set or clear bits
Write value

*/

static inline void d1981_reg_write_bit(unsigned int reg, unsigned char mask, unsigned char bits, unsigned char set)
{
	unsigned char value;

	value = d1981_reg_read(reg);
	value &= mask;
	if (set)
		value = SET_BIT(value,bits);
	else // clear bit
		value = CLR_BIT(value,bits);
	
	d1981_reg_write( reg, value);
	//printk(KERN_ALERT "d1981: 12C write reg[%x]=0x%x\n", reg, value);

}

#define D1981_SCRIPT_SEND(script)		run_script(script, sizeof(script)/2)
#define D1981_LOAD_USE_CASE(script)		load_use_case(script, sizeof(script)/2)
#ifdef I2C_REPEATED_WRITE_MODE
	#define D1981_MBOX_SEND(script)		d1981_block_write(sizeof(script), script)
#else
	#define D1981_MBOX_SEND(script)		_d1981_mailbox_send(sizeof(script), script)
	int _d1981_mailbox_send(unsigned int length, const u8 *script)
	{
		int script_pos, ret = 0;
		for (script_pos=0; script_pos < length; script_pos++) {
			ret = d1981_reg_write(script[2*i], script[2*i+1]);
			if (ret < 0) break;
		}
		return ret;
	}
#endif


/*************************************************************************************************************************
 Error Handling declarations
 *************************************************************************************************************************/
#ifdef D1981_EH_SUPPORT
//#define D1981_EH_LOCK		// For concurrent tasks support (i.e. interrupt error handling routine)
//#define D1981_EH_SIM_TEST	// Error Simulation for testing EH routines

// DSP request enumeration for error handling routines
typedef enum {
	ehNone = 0,	// No error detected
	ehRetry,	// Last use case need resending
	ehAbort,	// Aborting and reloading last use case
	ehReinit,	// Reinit framework and reload use case 
	ehReset,	// DSP requires reset and complete initialisation
} ehDspRequest;

// Error handler state is contained here
struct ehDsp_s {
	ehDspRequest 	request;		// Error recovery request level
	dspErrorCode	code;			// Last error code from DSP
#ifdef D1981_EH_LOCK
	struct mutex 	dsp_lock;		// general dsp access lock
#endif
} g_ehDsp;

#ifdef D1981_EH_LOCK
	static inline void ehLockDsp(void)		{mutex_lock(&g_ehDsp.dsp_lock);}
	static inline void ehUnlockDsp(void)		{mutex_unlock(&g_ehDsp.dsp_lock);}
#else
	static inline void ehLockDsp(void)		{}
	static inline void ehUnlockDsp(void)		{}
#endif

// Reset DSP only
static void d1981DspReset(void)
{
	d1981_reg_write(D1981_REG_DSP_CTRL, 0x00);
	msleep(1);
	d1981_reg_write(D1981_REG_DSP_CTRL, 0x01);
}

// ehGetRequest() returns recovery request for DSP.
// If DSP request is different than ehNone, first call ehRecoverDsp()
// before any further actions.
static inline ehDspRequest ehGetRequest(void)
{
	return g_ehDsp.request;
}

// ehSetRequest() signalizes that DSP recovery is required. It sets 
// g_ehDsp.request accordingly. 
// Requested recovery level is passed by new_state parameter but it can
// only increase current request to higher level.
static inline ehDspRequest ehSetRequest(ehDspRequest new_request)
{
	if (new_request > g_ehDsp.request) g_ehDsp.request = new_request;
	return g_ehDsp.request;
}

// ehRecoverDsp() executes DSP recovery depending on g_ehDsp.request.
// When finished, it resets request to ehNone (0).
void ehRecoverDsp(void)
{
	ehDspRequest request = g_ehDsp.request;

	g_ehDsp.request = ehNone;	// Has to be cleared before any following DSP access

	if (request >= ehAbort) dbg("\nDSP RECOVERY: request=%d\n\n", request);
	
	switch (request) {
	case ehNone:				// No need to recover - everything is fine
	case ehRetry:				// No need to recover - just repeat the script
		break;

	case ehAbort:				// Send Abort before retrying again
		D1981_MBOX_SEND(DSP_ABORT);
		break;
		
	case ehReinit:				// Restart framework before retrying again
		D1981_MBOX_SEND(DSP_ABORT_RESET_FW);
		break;
		
	case ehReset:				// Complete DSP reset and initialisation
		do {
			g_ehDsp.request = ehNone;
			d1981DspReset();
			_d1981_init_dsp();
		} while (ehGetRequest() != ehNone);
		break;
		
	default:					// Unsupported request - shouldn't happen
		dbg("\nD1981 CODEC: In function %s: line %d - unsupported request (%d).\n\n", __FUNCTION__, __LINE__, request);
		break;
	}
}

// ehDspCheck() tests g_ehDsp.request and pends DSP recovery if needed.
// Function always finishes succesfully and returns g_ehDsp.request value from before call.
// Recovery can be performed on higher level than saved in g_ehDsp.request, when minLevel 
// is set even higher. This is used to force stronger recovery, if previous did not help
// inside loops.
ehDspRequest ehDspCheck(ehDspRequest minLevel)
{
	ehDspRequest request = ehGetRequest();
	if (request != ehNone) ehSetRequest(minLevel);
	ehRecoverDsp();
	return request;
}


#ifdef D1981_EH_SIM_TEST
// Error simulation for Error Handling tests
enum {
	ehNoResp = 0,	// Mailbox returns 0 - busy/unavailable state
	ehBadStatus,	// Mailbox returns error code
	ehBadCounter,	// Manipulation of DSP command counter to get bad command count
	ehTotal
};
int ehIfError(unsigned int error)
{
	static struct ehErrorDef {
		unsigned int 	counter;
		unsigned int 	period;
		u8 				when[20];	// when error should appear
	} g_ehErrors[ehTotal] = {
		{0, 20, {1,1,1,1,1,0,0,}},
		{0, 19, {0,0,0,0,0,0,1,0,0,0,0,1,1}},
		{0, 7, {0,0,0,0,0,0,1}}
	};

	int ret = 0;

	if (g_ehErrors[error].counter >= g_ehErrors[error].period) g_ehErrors[error].counter = 0;
	ret = g_ehErrors[error].when[g_ehErrors[error].counter];
	++g_ehErrors[error].counter;
	return ret;
}

#endif /* D1981_EH_SIM_TEST */
#endif /* D1981_EH_SUPPORT */

/*************************************************************************************************************************
ALSA Controls
*************************************************************************************************************************/



/*
 * Registers direct access control
 */
static const struct snd_kcontrol_new d1981_direct_access[] = {
		SOC_SINGLE("D1981_REG_STATUS", 			D1981_REG_STATUS, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_REF1", 			D1981_REG_REF1, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_BIAS_EN", 		D1981_REG_BIAS_EN, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_BIAS1", 			D1981_REG_BIAS1, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_BIAS2", 			D1981_REG_BIAS2, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_BIAS3", 			D1981_REG_BIAS3, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_BIAS4", 			D1981_REG_BIAS4, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_MICBIAS2", 		D1981_REG_MICBIAS2, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_MICBIAS1", 		D1981_REG_MICBIAS1, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_MICDET", 			D1981_REG_MICDET, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_MIC1_PRE", 		D1981_REG_MIC1_PRE, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_MIC1", 			D1981_REG_MIC1, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_MIC2_PRE", 		D1981_REG_MIC2_PRE, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_MIC2", 			D1981_REG_MIC2, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_AUX1L", 			D1981_REG_AUX1L, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_AUX1R", 			D1981_REG_AUX1R, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_MIC3_PRE", 		D1981_REG_MIC3_PRE, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_MIC3", 			D1981_REG_MIC3, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_INP_PINBIAS", 	D1981_REG_INP_PINBIAS, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_INP_ZC_EN", 		D1981_REG_INP_ZC_EN, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_INP_MUX", 		D1981_REG_INP_MUX, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_HP_DET", 			D1981_REG_HP_DET, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_HPL_DAC_OFFSET", 		D1981_REG_HPL_DAC_OFFSET, 	0, 0xff, 0),
		SOC_SINGLE("D1981_REG_HPL_DAC_OFF_CNTL", 	D1981_REG_HPL_DAC_OFF_CNTL, 	0, 0xff, 0),
		SOC_SINGLE("D1981_REG_HPL_OUT_OFFSET", 		D1981_REG_HPL_OUT_OFFSET, 	0, 0xff, 0),
		SOC_SINGLE("D1981_REG_HPL", 			D1981_REG_HPL, 			0, 0xff, 0),
		SOC_SINGLE("D1981_REG_HPL_VOL", 		D1981_REG_HPL_VOL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_HPR_DAC_OFFSET", 		D1981_REG_HPR_DAC_OFFSET, 	0, 0xff, 0),
		SOC_SINGLE("D1981_REG_HPR_DAC_OFF_CNTL", 	D1981_REG_HPR_DAC_OFF_CNTL, 	0, 0xff, 0),
		SOC_SINGLE("D1981_REG_HPR_OUT_OFFSET", 		D1981_REG_HPR_OUT_OFFSET, 	0, 0xff, 0),
		SOC_SINGLE("D1981_REG_HPR", 			D1981_REG_HPR, 			0, 0xff, 0),
		SOC_SINGLE("D1981_REG_HPR_VOL", 		D1981_REG_HPR_VOL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_LIN2", 	D1981_REG_LIN2, 	0, 0xff, 0),
		SOC_SINGLE("D1981_REG_LIN3", 	D1981_REG_LIN3, 	0, 0xff, 0),
		SOC_SINGLE("D1981_REG_LIN4", 	D1981_REG_LIN4, 	0, 0xff, 0),
		//SOC_SINGLE("D1981_REG_LIN1L", 			D1981_REG_LIN1L, 		0, 0xff, 0),
		//SOC_SINGLE("D1981_REG_LIN1L_VOL", 		D1981_REG_LIN1L_VOL, 		0, 0xff, 0),
		//SOC_SINGLE("D1981_REG_LIN1R_DAC_OFFSET", 	D1981_REG_LIN1R_DAC_OFFSET, 	0, 0xff, 0),
		//SOC_SINGLE("D1981_REG_LIN1R_DAC_OFF_CNTL", 	D1981_REG_LIN1R_DAC_OFF_CNTL, 	0, 0xff, 0),
		//SOC_SINGLE("D1981_REG_LIN1R_OUT_OFFSET", 	D1981_REG_LIN1R_OUT_OFFSET, 	0, 0xff, 0),
		//SOC_SINGLE("D1981_REG_LIN1R", 			D1981_REG_LIN1R, 		0, 0xff, 0),
		//SOC_SINGLE("D1981_REG_LIN1R_VOL", 		D1981_REG_LIN1R_VOL, 		0, 0xff, 0),
		//SOC_SINGLE("D1981_REG_lIN2", 			D1981_REG_lIN2, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_OUT_ZC_EN", 		D1981_REG_OUT_ZC_EN, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_HP_LIN1_GNDSEL", 	D1981_REG_HP_LIN1_GNDSEL, 	0, 0xff, 0),
		//SOC_SINGLE("D1981_REG_PDM", 			D1981_REG_PDM, 			0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_HP1", 			D1981_REG_CP_HP1, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_HP2", 			D1981_REG_CP_HP2, 		0, 0xff, 0),
		//SOC_SINGLE("D1981_REG_CP_LIN1_1", 		D1981_REG_CP_LIN1_1, 		0, 0xff, 0),
		//SOC_SINGLE("D1981_REG_CP_LIN1_2", 		D1981_REG_CP_LIN1_2, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_CTRL1", 			D1981_REG_CP_CTRL1, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_CTRL2", 			D1981_REG_CP_CTRL2, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_CTRL3", 			D1981_REG_CP_CTRL3, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_LEVEL_MASK", 	D1981_REG_CP_LEVEL_MASK, 	0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_DET", 			D1981_REG_CP_DET, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_STATUS", 		D1981_REG_CP_STATUS, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_THRESH1", 		D1981_REG_CP_THRESH1, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_THRESH2", 		D1981_REG_CP_THRESH2, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_THRESH3", 		D1981_REG_CP_THRESH3, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_THRESH4", 		D1981_REG_CP_THRESH4, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_THRESH5", 		D1981_REG_CP_THRESH5, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_THRESH6", 		D1981_REG_CP_THRESH6, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_THRESH7", 		D1981_REG_CP_THRESH7, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CP_THRESH8", 		D1981_REG_CP_THRESH8, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_PLL_DIV_LO", 		D1981_REG_PLL_DIV_LO, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_PLL_DIV_MID",	 	D1981_REG_PLL_DIV_MID,	 	0, 0xff, 0),
		SOC_SINGLE("D1981_REG_PLL_DIV_HI", 		D1981_REG_PLL_DIV_HI, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_PLL_CTRL", 		D1981_REG_PLL_CTRL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CLK_CTRL", 		D1981_REG_CLK_CTRL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CLK_DSP", 		D1981_REG_CLK_DSP, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CLK_EN1", 		D1981_REG_CLK_EN1, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CLK_EN2", 		D1981_REG_CLK_EN2, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CLK_EN3", 		D1981_REG_CLK_EN3, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CLK_EN4", 		D1981_REG_CLK_EN4, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CLK_EN5", 		D1981_REG_CLK_EN5, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_AIF_MCLK",		D1981_REG_AIF_MCLK,		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_AIFA1", 			D1981_REG_AIFA1, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_AIFA2", 			D1981_REG_AIFA2, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_AIFA3", 			D1981_REG_AIFA3, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_AIFB1", 			D1981_REG_AIFB1, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_AIFB2", 			D1981_REG_AIFB2, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_AIFB3", 			D1981_REG_AIFB3, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_PC_CTRL", 		D1981_REG_PC_CTRL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DATA_ROUTE", 		D1981_REG_DATA_ROUTE, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DSP_CTRL", 		D1981_REG_DSP_CTRL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_CIF_CTRL2", 		D1981_REG_CIF_CTRL2, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_HANDSHAKE", 		D1981_REG_HANDSHAKE, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_MBOX0", 			D1981_REG_MBOX0, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_MBOX1", 			D1981_REG_MBOX1, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_MBOX2", 			D1981_REG_MBOX2, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_MBOX_STATUS", 	D1981_REG_MBOX_STATUS, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_SPARE1_OUT", 		D1981_REG_SPARE1_OUT, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_SPARE2_OUT", 		D1981_REG_SPARE2_OUT, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_SPARE1_IN", 		D1981_REG_SPARE1_IN, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_ID", 				D1981_REG_ID, 			0, 0xff, 0),
		SOC_SINGLE("D1981_REG_ADC1_PD", 		D1981_REG_ADC1_PD, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_ADC1_HPF", 		D1981_REG_ADC1_HPF, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_ADC1_SEL", 		D1981_REG_ADC1_SEL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_ADC1_EQ12", 		D1981_REG_ADC1_EQ12, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_ADC1_EQ34", 		D1981_REG_ADC1_EQ34, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_ADC1_EQ5", 		D1981_REG_ADC1_EQ5, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_ADC2_PD", 		D1981_REG_ADC2_PD, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_ADC2_HPF", 		D1981_REG_ADC2_HPF, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_ADC2_SEL", 		D1981_REG_ADC2_SEL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_ADC2_EQ12", 		D1981_REG_ADC2_EQ12, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_ADC2_EQ34", 		D1981_REG_ADC2_EQ34, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_ADC2_EQ5", 		D1981_REG_ADC2_EQ5, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC1_HPF", 		D1981_REG_DAC1_2_HPF, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC1_2_L_VOL", 		D1981_REG_DAC1_2_L_VOL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC1_2_R_VOL", 		D1981_REG_DAC1_2_R_VOL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC1_2_SEL", 			D1981_REG_DAC1_2_SEL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC1_2_SOFTMUTE", 	D1981_REG_DAC1_2_SOFTMUTE, 	0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC3_4_EQ12", 		D1981_REG_DAC3_4_EQ12, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC3_4_EQ34", 		D1981_REG_DAC3_4_EQ34, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC3_4_EQ5", 			D1981_REG_DAC3_4_EQ5, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC3_4_HPF", 			D1981_REG_DAC3_4_HPF, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC3_4_L_VOL", 		D1981_REG_DAC3_4_L_VOL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC3_4_R_VOL", 		D1981_REG_DAC3_4_R_VOL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC3_4_SEL", 			D1981_REG_DAC3_4_SEL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC3_4_SOFTMUTE", 	D1981_REG_DAC3_4_SOFTMUTE, 	0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC3_4_EQ12", 		D1981_REG_DAC3_4_EQ12, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC3_4_EQ34", 		D1981_REG_DAC3_4_EQ34, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC3_4_EQ5", 		D1981_REG_DAC3_4_EQ5, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC5_HPF", 		D1981_REG_DAC5_HPF, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC5_L_VOL", 		D1981_REG_DAC5_L_VOL, 		0, 0xff, 0),
		//SOC_SINGLE("D1981_REG_DAC3_R_VOL", 		D1981_REG_DAC5_R_VOL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC5_SEL", 		D1981_REG_DAC5_SEL, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC5_SOFTMUTE", 	D1981_REG_DAC5_SOFTMUTE, 	0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC5_EQ12", 		D1981_REG_DAC5_EQ12, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC5_EQ34", 		D1981_REG_DAC5_EQ34, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DAC5_EQ5", 		D1981_REG_DAC5_EQ5, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_BIQ_BYP", 		D1981_REG_BIQ_BYP, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DMA_CMD", 		D1981_REG_DMA_CMD, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DMA_ADDR0", 		D1981_REG_DMA_ADDR0, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DMA_ADDR1", 		D1981_REG_DMA_ADDR1, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DMA_DATA0", 		D1981_REG_DMA_DATA0, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DMA_DATA1", 		D1981_REG_DMA_DATA1, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DMA_DATA2", 		D1981_REG_DMA_DATA2, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DMA_DATA3", 		D1981_REG_DMA_DATA3, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_DMA_STATUS", 		D1981_REG_DMA_STATUS, 		0, 0xff, 0),
		SOC_SINGLE("D1981_REG_BROWNOUT", 		D1981_REG_BROWNOUT, 		0, 0xff, 0),
		//SOC_SINGLE("D1981_REG_TEST_REFGEN", 		D1981_REG_TEST_REFGEN, 		0, 0xff, 0)
};

/* utility controls - development testing  */

static const char *d1981_mic_bias_voltage[] = {"1.4", "2.0", "1.4", "2.0"};

static const struct soc_enum d1981_enum[] = {
	SOC_ENUM_SINGLE(D1981_REG_MICBIAS1, 0, 4, d1981_mic_bias_voltage),
};

static const struct snd_kcontrol_new d1981_snd_controls[] = {
	SOC_DOUBLE_R("HeadPhone Playback Volume", D1981_REG_HPL_VOL, D1981_REG_HPR_VOL, 0, 0x0f, 0),
	SOC_ENUM("Mic Bias Voltage", d1981_enum[0]),
};

static int acm_path_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) {return(0);}
static int acm_path_enable_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) {return(0);}
static int acm_path_disable_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) {return(0);}
static int acm_path_volume_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) {return(0);}
static int acm_path_mute_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) {return(0);}
static int acm_init_get(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol) {return(0);}

/* acm_path_select

Alsa callback function:
Select acm path

*/

int d1981DynamicFMVolume(u8 path,ACM_AudioVolume volume)
{
    int max_gain;
    
    max_gain=d1981_codec_gain[path][1]; //DAC gain
    
     volume=(max_gain+100)-volume; //100 : mute 0x20(32): max
    if(volume==(100+max_gain))
        volume=0x7f; //mute

    return volume;  
}

#define VOLUME_STEP 5
int d1981DynamicRxVolume(u8 path,ACM_AudioVolume volume)
{
    
    int numVolume;

    dbg("\n\n########################## d1981DynamicRxVolume volume=%d ########################## \n\n", (int)volume);
    
    if(volume==0) //in case of loopback
        volume=d1981_codec_gain[path][1];
    #if 1 //100 83 66 49 32
        else if(volume <= 100 && volume > 88)
            volume=d1981_codec_gain[path][1];
        else if(volume <=88 && volume >72)
            volume=d1981_codec_gain[path][1] + VOLUME_STEP*1;
        else if(volume <=72 && volume > 55)
            volume=d1981_codec_gain[path][1] + VOLUME_STEP*2;
        else if(volume <=55 && volume > 40)
            volume=d1981_codec_gain[path][1] + VOLUME_STEP*3;
        else
            volume=d1981_codec_gain[path][1] + VOLUME_STEP*4;
    #else
    numVolume=volume/20; //100,83,66,49,32 -> 5,4,3,2,1

    if(numVolume==0)
        volume=0x7f; //mute
    else
    {                
        volume=d1981_codec_gain[path][1] + VOLUME_STEP*(5-numVolume); // 3dB 4step DAC gain

    }
    #endif
    return volume;
}

void d1981DynamicForceSpeakerRxVolume(ACM_AudioVolume volume)
{
    
    int numVolume;

    dbg("\n\n########################## d1981DynamicForceSpeakerRxVolume volume=%d ########################## \n\n", (int)volume);

    d1981_reg_write_bit(D1981_REG_DAC3_4_L_VOL, 0x00, d1981_codec_force_speaker_gain[1][1], 1);
    d1981_reg_write_bit(D1981_REG_DAC1_2_L_VOL, 0x00, d1981_codec_force_speaker_gain[0][1], 1);	// DAC1-L 0 max volume
    d1981_reg_write_bit(D1981_REG_DAC1_2_R_VOL, 0x00, d1981_codec_force_speaker_gain[0][1], 1);	// DAC1-R volume

#if 0                
    if(volume==0) //in case of loopback
        volume=d1981_codec_gain[path][1];
    else if(volume <= 100 && volume > 88)
        volume=d1981_codec_gain[path][1];
    else if(volume <=88 && volume >72)
        volume=d1981_codec_gain[path][1] + VOLUME_STEP*1;
    else if(volume <=72 && volume > 55)
        volume=d1981_codec_gain[path][1] + VOLUME_STEP*2;
    else if(volume <=55 && volume > 40)
        volume=d1981_codec_gain[path][1] + VOLUME_STEP*3;
    else
        volume=d1981_codec_gain[path][1] + VOLUME_STEP*4;
    return volume;
#endif    
}

static int acm_path_select(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];
	int ret=1;

	gCurrentPath = (unsigned char)value;

	return(ret);
}

/* acm_path_enable

Alsa callback function:
Enable acm path

*/
int d1981PathPutEnable(u8 path, ACM_AudioVolume volume)
{
	int ret=1;
	u8 use_case = use_case_table[path];

	dbg("\n\n########################## ACM Path %d Enabled ########################## \n\n", (int)gCurrentPath);

    if(use_case== USE_CASE_3AB)
    {
        d1981PathEnable(path, ENABLE);
    }
    else
    {
        if((gActiveUseCase==21 || gActiveUseCase==22 || gActiveUseCase==23)
        && (use_case ==31 || use_case ==32 || use_case ==33))
        {
            dbg("\n\n#### PASS USE CASE !!!!!!!111 ##");
        }
        else
            d1981LoadUseCase(use_case, volume,0);        
        d1981PathEnable(path, ENABLE);
    }
	return(0);

}

static int acm_path_enable(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];
	unsigned char path;
	ACM_AudioVolume volume;
	int ret=1;
	u8 use_case;

    path = gCurrentPath;
    use_case = use_case_table[path];
      
	volume = (ACM_AudioVolume) value;

    if(use_case==USE_CASE_3B)
    {
        gFmRadioVolume=d1981DynamicFMVolume(path,volume);
    }    
	d1981PathPutEnable(path,volume);

    d1981_enabled_path[d1981_path_direction[path]]++;
    d1981_path_enabled[path]=1;

////////////////////
    ret=d1981PathMute(path, 0);	

    d1981PathVolume2Set(path, volume,volume);
    d1981PathVolumeSet(path, volume,volume);
//////////////////////    
	return(ret);
}

/* acm_path_disable

Alsa callback function:
Disable acm path

*/
static int acm_path_disable(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
    struct snd_soc_codec *codec =  d1981_codec;
    struct d1981_priv *d1981=(struct d1981_priv *)codec->drvdata;
	int value = ucontrol->value.integer.value[0];
	unsigned char path;
	int ret=1;
	u8 use_case;
    
	path = (unsigned char)value;

	use_case = use_case_table[path];

    dbg("\n\n   ###   acm_path_disable PATH = %d  ### \n\n",path);

    d1981_path_enabled[path]=0;

    d1981_enabled_path[d1981_path_direction[path]]--;
#if 0
    if(d1981_enabled_path[2]==0)
    {
        if(d1981_enabled_path[0]==0)
        {
             d1981_reg_write_bit(D1981_REG_AIFA3, ~D1981_AIF_EN, D1981_AIF_EN, 0);	  
             dbg("\n\n   ###   [CodecD] Interface 1 disable  ### \n\n");
        }
        if(d1981_enabled_path[1]==0)
        {
            d1981_reg_write_bit(D1981_REG_AIFB3, ~D1981_AIF_EN, D1981_AIF_EN, 0);
            dbg("\n\n   ###   [CodecD] Interface 2 disable  ### \n\n");
        }
    }
#endif    
	switch(path)
	{
        case D1981_I2S_1_TO_HS_VIA_DAC12:
        case D1981_I2S_1_TO_SPKR_VIA_DAC3:
            if(d1981->working_path == WORKING_HEADSET_SPEAKER)
                return ret;
            break;

        case D1981_I2S_1_TO_SPKR_HS:
            if(d1981->working_path != WORKING_SPEAKER)
            {
                ret=d1981PathEnable(path, DISABLE);
                return ret;
            }
            break;
            
         case D1981_AnaMIC1_TO_I2S_1:
         case D1981_AnaMIC1_LOUD_TO_I2S_1:
         case D1981_AnaMIC2_TO_I2S_1:
         case D1981_AUX1_TO_I2S_1:
         case D1981_AnaMIC1_TO_PCM_2:
         case D1981_AnaMIC1_LOUD_TO_PCM_2:
         case D1981_AnaMIC2_TO_PCM_2:
         case D1981_AUX1_TO_PCM_2:

         case D1981_AnaMIC1_TO_PCM_2_VOIP:
         case D1981_AnaMIC1_LOUD_TO_PCM_2_VOIP:
         case D1981_AnaMIC2_TO_PCM_2_VOIP:

         case D1981_AnaMIC1_TO_PCM_2_VT:
         case D1981_AnaMIC1_LOUD_TO_PCM_2_VT:
         case D1981_AnaMIC2_TO_PCM_2_VT:

         case D1981_AnaMIC1_TO_PCM_2_LB:
         case D1981_AnaMIC1_LOUD_TO_PCM_2_LB:
         case D1981_AnaMIC2_TO_PCM_2_LB:
            ret=d1981PathEnable(path, DISABLE);
            return ret;

        case D1981_AnaMIC1_LOUD_TO_PCM_2_APPS:
            //if(d1981_path_enabled[D1981_AnaMIC1_TO_PCM_2_APPS]==1)
            //{
            //    return ret;
            //}
        case D1981_AnaMIC1_TO_PCM_2_APPS:    	
    	case D1981_AnaMIC2_TO_PCM_2_APPS:
	
         default:
            break;           

	}

    if(gActiveUseCase == use_case)
	{
		ret=d1981PathEnable(path, DISABLE);
	    gActiveUseCase = USE_CASE_UNDEFINED;
	}

	return(ret);
}


/* acm_path_mute

Alsa callback function:
Mute acm path

*/
static int acm_path_mute(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];
	unsigned char path;
	ACM_AudioMute mute;
	int ret=1;
    mute = (ACM_AudioVolume) value;
    u8 use_case;

        
    path=gCurrentPath;
    use_case = use_case_table[path];

	if (gActiveUseCase != use_case && use_case != USE_CASE_3AB)
	{
		dbg("\n\n   ###    ( ACM Path %d Different!!!!, ActiveUseCase=%hhu)    ### \n\n", (int)gCurrentPath, gActiveUseCase);

        if(mute==0)
        {
		    ret=d1981PathPutEnable(path,0xff);
		    if(ret== -1)
		        return 1;
		}
	} 
		
	if(mute)
	{
		dbg( "\n\n########################## ACM Path %d Mute ########################## \n\n", (int)gCurrentPath);
        ret=d1981PathMute(path, mute);
	}
	else
	{
		dbg( "\n\n########################## ACM Path %d Unmute ########################## \n\n", (int)gCurrentPath);
          

        ret=d1981PathMute(path, mute);	

        //d1981PathVolume2Set(path, 0xff,0xff);
        //d1981PathVolumeSet(path, 0xff,0xff);
	}
		

	return(1);
}

void d1981PathExternalMute(unsigned char mute)
{
    dbg( "\nD1981 Codec Driver d1981PathExternalMute path=%d mute =%d \n",gCurrentPath,mute);
    d1981PathMute(gCurrentPath,mute);
}
/* acm_path_volume_set

Alsa callback function:
Set acm path volume

*/
static int acm_path_volume_set(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int value = ucontrol->value.integer.value[0];
	unsigned char path;
	ACM_AudioVolume volume;
	int ret=1;
	u8 use_case;

    path = gCurrentPath;
	
		volume = (ACM_AudioVolume) value;
	dbg( "\n ######### D1981 Codec Driver - ACM Path %d Volume %d #########\n", (int)gCurrentPath, (int)volume);

	ret=d1981PathVolumeSet(path, volume,volume);
	ret=d1981PathVolume2Set(path, volume,volume);		

            
	return(ret);
}

/* acm_init

Alsa callback function:
Initialise paths

*/
static int acm_init(struct snd_kcontrol *kcontrol, struct snd_ctl_elem_value *ucontrol)
{
	int ret=1;
	int value = ucontrol->value.integer.value[0];

	printk(KERN_ALERT "D1981 Codec Driver - ACM Init\n");
	
	return(ret);
}


/* Samsung Marvell ACM interface controls with callback handler references */
static const struct snd_kcontrol_new d1981_acm_controls[] = {
	SOC_SINGLE_EXT("PATH_SELECT", PATH_SELECT, 0, 0xff, 0,         acm_path_get,  		acm_path_select),
	SOC_SINGLE_EXT("PATH_ENABLE", PATH_ENABLE, 0, 0xff, 0,         acm_path_enable_get,  	acm_path_enable),
	SOC_SINGLE_EXT("PATH_DISABLE", PATH_DISABLE, 0, 0xff, 0,       acm_path_disable_get,    acm_path_disable),
	SOC_SINGLE_EXT("PATH_MUTE", PATH_MUTE, 0, 0xff, 0,             acm_path_mute_get,       acm_path_mute),
	SOC_SINGLE_EXT("PATH_VOLUME_SET", PATH_VOLUME_SET, 0, 0xff, 0, acm_path_volume_get,	acm_path_volume_set),	
	SOC_SINGLE_EXT("CODEC_INIT", CODEC_INIT, 0, 0xff, 0,           acm_init_get,			acm_init),
};

extern const struct snd_kcontrol_new d1981_gain_controls[MAX_D1981_PATH] ;
extern const struct snd_kcontrol_new d1981_gain2_controls[MAX_D1981_PATH] ;
extern const struct snd_kcontrol_new d1981_gain3_controls[D1981_MAX_OP_CONTROL] ;

/*
 *	Register all controls with alsa sub-system
 */

static int d1981_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	/* acm interface controls */
	for (i = 0; i < ARRAY_SIZE(d1981_acm_controls); i++) {
		err = snd_ctl_add(codec->card, snd_soc_cnew(&d1981_acm_controls[i],\
								  codec, NULL));
		if (err < 0)
			return err;
	}

	/* register direct controls 
	for (i = 0; i < ARRAY_SIZE(d1981_direct_access) - 7; i++) {
		err = snd_ctl_add(codec->card, snd_soc_cnew(&d1981_direct_access[i],\
								  codec, NULL));
		if (err < 0)
			return err;
	}*/
	/* register direct controls */
#ifdef HW_TUNNING	
	for (i = 0; i < ARRAY_SIZE(d1981_gain_controls); i++) {
		err = snd_ctl_add(codec->card, snd_soc_cnew(&d1981_gain_controls[i],\
								  codec, NULL));
		if (err < 0)
			return err;
	}

	for (i = 0; i < ARRAY_SIZE(d1981_gain2_controls); i++) {
		err = snd_ctl_add(codec->card, snd_soc_cnew(&d1981_gain2_controls[i],\
								  codec, NULL));
		if (err < 0)
			return err;
	}
	
	for (i = 0; i < ARRAY_SIZE(d1981_gain3_controls); i++) {
		err = snd_ctl_add(codec->card, snd_soc_cnew(&d1981_gain3_controls[i],\
								  codec, NULL));
		if (err < 0)
			return err;
	}
#else
	/* utility controls */
	for (i = 0; i < ARRAY_SIZE(d1981_snd_controls); i++) {
		err = snd_ctl_add(codec->card, snd_soc_cnew(&d1981_snd_controls[i],\
								  codec, NULL));
		if (err < 0)
			return err;
	}
#endif	
	return 0;
}

void d1981PowerOn(void)
{

 struct regulator *regAUD_A = NULL;
 struct regulator *regAUD_B = NULL;
 struct snd_soc_codec *codec =  d1981_codec;
 struct d1981_priv *d1981=(struct d1981_priv *)codec->drvdata;
 
 regAUD_A = regulator_get(NULL, REGULATOR_AUDIO_A);
 regAUD_B = regulator_get(NULL, REGULATOR_AUDIO_B);
 
 dbg("D1981 Audio Codec Power ON !!!!!!!!!!\n");

 enable_oscc_pout();
 g_d1981_power_state=1;

     regulator_enable(regAUD_A);

     regulator_enable(regAUD_B);
 
     msleep(10);
     d1981_init_codec(codec);
     d1981->power=1;

}

extern struct d1981_hs_data *g_hs_data;
extern u8 d1980_set_adcIn_enable(u8 on);

void d1981LowPowerHookDetection(u8 on)
{
    struct snd_soc_codec *codec =  d1981_codec;
    struct d1981_priv *d1981=(struct d1981_priv *)codec->drvdata;    

    if(g_dlg_4pole_detect ==1)
    {
        if(on==1) //hootdetection on and 4 pole
        {
            dbg("[CodecD] D1981 d1981LowPowerHookDetection =  %d \n",on);
            run_script(Lynx_LowPowerHookDetectionOn, sizeof(Lynx_LowPowerHookDetectionOn)/2);
            run_script(Lynx_CodecLowPowerModeDSPHookDet, sizeof(Lynx_CodecLowPowerModeDSPHookDet)/2);
            d1981->hookMode=1;
        }
        else
        {
            dbg("[CodecD] D1981 d1981LowPowerHookDetection =  %d \n",on);
            run_script(Lynx_LowPowerHookDetectionOff, sizeof(Lynx_LowPowerHookDetectionOff)/2);

            d1981_mic_bias(1);
		#if 1
			d1980_set_adcIn_enable(1); //adc on
			//dbg(" d1981LowPowerHookDetection-adc[%d]\n", d1980_adc_in_read_auto());
		#endif
            d1980_hookswitch_polling_start();
        }
    }
    else
    {
        d1981_mic_bias(0);

    }
}

void d1981GetHookDetectionState(void)
{
    u8 value1,value2,value3;

    run_script(Lynx_HookDetectionState, sizeof(Lynx_HookDetectionState)/2);

    value1=d1981_reg_read( 0x76);
	value2=d1981_reg_read( 0x77);
	value3=d1981_reg_read( 0x78);
	printk(KERN_ALERT "d1981GetHookDetectionState = 0x%02x%02x%02x\n", value3, value2, value1);

}


void d1981ClassDEnable(int on)
{
    struct regulator *reg = NULL;
    struct snd_soc_codec *codec =  d1981_codec;
    struct d1981_priv *d1981=(struct d1981_priv *)codec->drvdata;
    
    reg = regulator_get(NULL, REGULATOR_CLASSD);
  
    if(on==1)
    {       
        if(d1981->classD==0)
        {
            dbg("d1981ClassDEnable ON !!!!!!!!!!\n");
            regulator_enable(reg);
            d1981->classD=1;
        }
     }
     else
     {        
         if(d1981->classD==1)
        {
            dbg( "d1981ClassDEnable OFF !!!!!!!!!!\n");
            regulator_disable(reg);
            d1981->classD=0;
        }
     }
}

int d1981PowerControl(u8 value)
{   
 struct snd_soc_codec *codec =  d1981_codec;
 struct d1981_priv *d1981=(struct d1981_priv *)codec->drvdata;

 if(gActiveUseCase==USE_CASE_1A 
 || gActiveUseCase==USE_CASE_1B 
 || gActiveUseCase==USE_CASE_1C
 || gActiveUseCase==USE_CASE_6A 
 || gActiveUseCase==USE_CASE_6B 
 || gActiveUseCase==USE_CASE_6C
 || gActiveUseCase==USE_CASE_3B
 || gActiveUseCase==USE_CASE_2A 
 || gActiveUseCase==USE_CASE_2B 
 || gActiveUseCase==USE_CASE_2C) //voice call
    return 1;
    
 if(value==1)
 {    
    
     dbg( "D1981 Audio Codec Power ON!!!!!!!\n");
    g_d1981_power_state=1;
    d1980_ldo15A_set(1);
    //LDO15A will come up at AC version
    enable_oscc_pout();
    //msleep(10);
    d1981LowPowerHookDetection(0);
    //run_script(Lynx_LowPowerHookDetectionOff, sizeof(Lynx_LowPowerHookDetectionOff)/2);
    run_script(Lynx_CodecPowerup, sizeof(Lynx_CodecPowerup)/2);
    //d1981LoadUseCase(gCurrentPath, 0,1);
    d1981->power=1;
 }
 else
 {
    dbg("D1981 Audio Codec LOW Power MODE!!!!!!!\n");
		if(g_dlg_4pole_detect == 1)
		{
			d1980_hookswitch_polling_stop();
			msleep(100);
		}

		g_d1981_power_state=0;
    d1981PathEnable(gCurrentPath, DISABLE);
    run_script(DSP_ABORT_RESET_FW,sizeof(DSP_ABORT_RESET_FW)/2);
    
    if(g_dlg_4pole_detect == 1)
    {        
        d1981LowPowerHookDetection(1);
        disable_oscc_pout();
        d1980_ldo15A_set(0);
        //LDO15A will be come down at AC version
    }
    else
    {
        run_script(Lynx_CodecLowPowerMode, sizeof(Lynx_CodecLowPowerMode)/2); 
        disable_oscc_pout();
        d1980_ldo15A_set(0);
        //LDO15A will be come down at AC version
        run_script(Lynx_DisableInternalRefOsc, sizeof(Lynx_DisableInternalRefOsc)/2); 
    }
         
    d1981->power=0;
    d1981ClassDEnable(0); 
    //gCurrentUseCase=USE_CASE_UNDEFINED;
 }

    return 1;
}

/* d1981PathToUseCase

Identify use case from path and load if necessary

*/
static int _d1981LoadUseCase(unsigned char use_case, ACM_AudioVolume volume,unsigned char force)
{	
#if 1 //ndef D1981_FAST_LOADING
    dbg("\n\n ############## Old Use Case %d.  New Use Case %d ############# \n\n", gActiveUseCase, use_case);
#endif

	if(use_case == USE_CASE_UNDEFINED) 
	{
		printk(KERN_ALERT "D1981 Codec Driver Use Case Undefined\n");
		return(-1);
	}
		
	if (use_case != gActiveUseCase || force==1) 
	{
	/* load new use case */
#ifndef D1981_FAST_LOADING
		dbg( "D1981 Codec Driver Loading New Use Case %d\n", use_case);
#endif		
		gCurrentUseCase = use_case;


		/* 1/ Run UC_Transition (this disables all ADCs and DACs - and DSP PC count) */
		run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
        
		switch(use_case) 
		{

			case USE_CASE_1A:
			    if(gActiveUseCase != USE_CASE_1B && gActiveUseCase != USE_CASE_1C)
			    {
			        //run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
                    load_use_case(Jetta_UC1Commander_v1507, sizeof(Jetta_UC1Commander_v1507)/2);
    			}
				run_script(Lynx_UC1_MICL_MIC1_trans_13MHz_SR16k_SlaveMode, sizeof(Lynx_UC1_MICL_MIC1_trans_13MHz_SR16k_SlaveMode)/2);//Lynx_UC1_MICL_MIC1_trans_13MHz_SR16k_SlaveMode_SIZE);
				break;

			case USE_CASE_1B:
			    if(gActiveUseCase != USE_CASE_1A && gActiveUseCase != USE_CASE_1C)
			    {
			        //run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
                    load_use_case(Jetta_UC1Commander_v1507, sizeof(Jetta_UC1Commander_v1507)/2);
                }
                run_script(Lynx_UC1_MICR_MIC2_trans_13MHz_SR16k_SlaveMode, sizeof(Lynx_UC1_MICR_MIC2_trans_13MHz_SR16k_SlaveMode)/2);//Lynx_UC1_MICR_MIC2_trans_13MHz_SR16k_SlaveMode_SIZE);
				break;

			case USE_CASE_1C:
			    if(gActiveUseCase != USE_CASE_1A && gActiveUseCase != USE_CASE_1B)
			    {
			        //run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
                    load_use_case(Jetta_UC1Commander_v1507, sizeof(Jetta_UC1Commander_v1507)/2);
			    
                }	
                run_script(Lynx_UC1_MICR_MIC3_trans_13MHz_SR16k_SlaveMode, sizeof(Lynx_UC1_MICR_MIC3_trans_13MHz_SR16k_SlaveMode)/2);//Lynx_UC1_MICL_MIC1_trans_13MHz_SR16k_SlaveMode_SIZE);
				break;

				case USE_CASE_6A:
			    if(gActiveUseCase != USE_CASE_6B && gActiveUseCase != USE_CASE_6C)
			    {
			        //run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
    				load_use_case(Jetta_UC1Commander_v1507, sizeof(Jetta_UC1Commander_v1507)/2);
    			
				}
				run_script(Lynx_UC1_MICL_MIC1_trans_13MHz_SR16k_SlaveMode, sizeof(Lynx_UC1_MICL_MIC1_trans_13MHz_SR16k_SlaveMode)/2);//Lynx_UC1_MICL_MIC1_trans_13MHz_SR16k_SlaveMode_SIZE);
				break;

			case USE_CASE_6B:
			    if(gActiveUseCase != USE_CASE_6A && gActiveUseCase != USE_CASE_6C)
			    {
                    //run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
    				load_use_case(Jetta_UC1Commander_v1507, sizeof(Jetta_UC1Commander_v1507)/2);
    			}
                run_script(Lynx_UC1_MICR_MIC2_trans_13MHz_SR16k_SlaveMode, sizeof(Lynx_UC1_MICR_MIC2_trans_13MHz_SR16k_SlaveMode)/2);//Lynx_UC1_MICR_MIC2_trans_13MHz_SR16k_SlaveMode_SIZE);
				break;

			case USE_CASE_6C:
			    if(gActiveUseCase != USE_CASE_6A && gActiveUseCase != USE_CASE_6B)
			    {
			        //run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
    				load_use_case(Jetta_UC1Commander_v1507, sizeof(Jetta_UC1Commander_v1507)/2);
    		    }	
                run_script(Lynx_UC1_MICR_MIC3_trans_13MHz_SR16k_SlaveMode, sizeof(Lynx_UC1_MICR_MIC3_trans_13MHz_SR16k_SlaveMode)/2);//Lynx_UC1_MICL_MIC1_trans_13MHz_SR16k_SlaveMode_SIZE);
				break;

			case USE_CASE_2A:
			case USE_CASE_7A:	
			    //run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
			    load_use_case(Jetta_UC2_UC4_16kSRCommander_v1507, sizeof(Jetta_UC2_UC4_16kSRCommander_v1507)/2);
                run_script(Lynx_UC4a_trans_26MHz_SR16k_SlaveI2S2_MasterI2S1, sizeof(Lynx_UC4a_trans_26MHz_SR16k_SlaveI2S2_MasterI2S1)/2);//Lynx_UC1_MICL_MIC1_trans_13MHz_SR16k_SlaveMode_SIZE);
				break;

            case USE_CASE_2C:
            case USE_CASE_7C:
                //run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
                load_use_case(Jetta_UC2_UC4_16kSRCommander_v1507, sizeof(Jetta_UC2_UC4_16kSRCommander_v1507)/2);//LynxUseCase2aCommander_SIZE);
				run_script(Lynx_UC4a_trans_26MHz_SR16k_SlaveI2S2_MasterI2S1, sizeof(Lynx_UC4a_trans_26MHz_SR16k_SlaveI2S2_MasterI2S1)/2);//Lynx_UC2_trans_13MHz_SR44_1k_MasterMode_SIZE);
			    break;
				
			case USE_CASE_2B:
			case USE_CASE_7B:
			    //run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
			    load_use_case(Jetta_UC2_UC4_16kSRCommander_v1507, sizeof(Jetta_UC2_UC4_16kSRCommander_v1507)/2);
			    run_script(Lynx_UC4b_trans_26MHz_SR16k_SlaveI2S2_MasterI2S1, sizeof(Lynx_UC4b_trans_26MHz_SR16k_SlaveI2S2_MasterI2S1)/2);
			    break;

			case USE_CASE_3A:  
			case USE_CASE_3AB: 
			    if(gActiveUseCase != USE_CASE_3A && gActiveUseCase != USE_CASE_3AB)
			    {
			        //run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
    				load_use_case(Jetta_UC3aCommander_v1507, sizeof(Jetta_UC3aCommander_v1507)/2);
    				run_script(Lynx_UC3A_Trans_13MHz_SR44_1k_MasterMode, sizeof(Lynx_UC3A_Trans_13MHz_SR44_1k_MasterMode)/2);//Lynx_UC3A_Trans_13MHz_SR44_1k_MasterMode_SIZE);
                }
				break;

			case USE_CASE_3B:
			    //run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
				//d1981_reg_write(D1981_REG_HPL, 0x90);	// FM_radio requires this on path change
				//d1981_reg_write(D1981_REG_HPR, 0x90);	// 	-''-
				//d1981_reg_write(D1981_REG_LIN3, 0x00);	// 	-''-
				//d1981_reg_write(D1981_REG_LIN2, 0x00);	// 	-''-
				load_use_case(Jetta_UC3bCommander_v1507,sizeof(Jetta_UC3bCommander_v1507)/2);
			    run_script(Lynx_UC3B_trans_13MHz_SR44_1k_MasterMode, sizeof(Lynx_UC3B_trans_13MHz_SR44_1k_MasterMode)/2);
				break;

            case USE_CASE_3C: // hifi to receiver for production test
                //run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
				load_use_case(Test_HiFiReceiverCommander,sizeof(Test_HiFiReceiverCommander)/2);
				run_script(Lynx_LBTest_MICLR_MIC12_trans_26MHz_SR44_1k_MasterMode,sizeof(Lynx_LBTest_MICLR_MIC12_trans_26MHz_SR44_1k_MasterMode)/2);
				break;

            case USE_CASE_4A: // hifi to receiver for production test
                //run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
				load_use_case(LBTest_MIC1ToAllDacsCommander,sizeof(LBTest_MIC1ToAllDacsCommander)/2);
				run_script(Lynx_LBTest_MICLR_MIC12_trans_26MHz_SR44_1k_MasterMode, sizeof(Lynx_LBTest_MICLR_MIC12_trans_26MHz_SR44_1k_MasterMode)/2);
				break;	

            case USE_CASE_5A: // hifi to receiver for production test
                //run_script(UC_Transition, sizeof(UC_Transition)/2);//UC_Transition_SIZE);
				load_use_case(LBTest_MIC2ToAllDacsCommander,sizeof(LBTest_MIC2ToAllDacsCommander)/2);
				run_script(Lynx_LBTest_MICLR_MIC12_trans_26MHz_SR44_1k_MasterMode, sizeof(Lynx_LBTest_MICLR_MIC12_trans_26MHz_SR44_1k_MasterMode)/2);
				
			default:  break;
		}
		// Head phones DC offset setup
		{
			const u64 setup_DC_offset_for_use_cases = (
					1ull<<USE_CASE_1A | 1ull<<USE_CASE_1B | 1ull<<USE_CASE_1C 
					| 1ull<<USE_CASE_3A | 1ull<<USE_CASE_3B | 1ull<<USE_CASE_3C | 1ull<<USE_CASE_3AB
				);

			if (use_case < 64 && (setup_DC_offset_for_use_cases & 1ull<<use_case)) {
				d1981_reg_write_bit(D1981_REG_HPL, ~D1981_HPL_OUT_COMPO, D1981_HPL_OUT_COMPO, gHP_offset_sign[0]);
				d1981_reg_write_bit(D1981_REG_HPR, ~D1981_HPR_OUT_COMPO, D1981_HPR_OUT_COMPO, gHP_offset_sign[1]);
				//msleep(1);	// Delay for HP DC offset to stabilize
			}
		}
	}
	
    else 
    {
    		dbg("D1981 Codec Driver Use Case unchanged\n");

    }
	return(1);

}

#ifdef D1981_EH_SUPPORT
int d1981LoadUseCase(unsigned char use_case, ACM_AudioVolume volume,unsigned char force)
{
	int 			ret;
	ehDspRequest	minLevel = ehRetry;
	
	ehLockDsp();
	while (1) {
		ret = _d1981LoadUseCase(use_case, volume, force);
		if (ehDspCheck(minLevel) == ehNone) break;
		if (++minLevel > ehReset) minLevel = ehReset;
	}
    gActiveUseCase = use_case;
	ehUnlockDsp();
	return ret;
}
#else /* D1981_EH_SUPPORT */
int d1981LoadUseCase(unsigned char use_case, ACM_AudioVolume volume,unsigned char force)
{
    struct timeval now,now2;
 	int ret = 0;

    do_gettimeofday(&now);  
 	ret=_d1981LoadUseCase(use_case, volume, force);
    gActiveUseCase = use_case;
    do_gettimeofday(&now2);
    dbg( "d1981LoadUseCase Usecase download time =%ld usec.\n",1000000 * (now2.tv_sec - now.tv_sec)+now2.tv_usec-now.tv_usec);
	return ret;
}
#endif /* D1981_EH_SUPPORT */

void d1981SpeakerMute(void)
{
    d1981ClassDEnable(0);
    
    d1981_reg_write(D1981_REG_DAC3_4_SOFTMUTE, 0x80); // softmute enable
    d1981_reg_write(D1981_REG_DAC3_4_L_VOL, 0x7f); // volume=mute
    d1981_reg_write_bit(D1981_REG_DAC3_4_SEL, 0xff, (0x04), 1);
    
}

void d1981HeadsetMute(void)
{
    d1981_reg_write(D1981_REG_DAC1_2_SOFTMUTE, 0xc0); // softmute enable
    d1981_reg_write(D1981_REG_DAC1_2_L_VOL, 0x7f); // mute
    d1981_reg_write(D1981_REG_DAC1_2_R_VOL, 0x7f); // mute
    d1981_reg_write_bit(D1981_REG_HPL, 0xbf, 0x40, 1); //amp mute    
    d1981_reg_write_bit(D1981_REG_HPR, 0xbf, 0x40, 1);

}

void d1981RcvMute(void)
{
    d1981_reg_write(D1981_REG_DAC5_L_VOL, 0x7f); // volume=mute
    d1981_reg_write(D1981_REG_DAC5_SOFTMUTE, 0x80); // softmute enable	
}

int d1981PathEnable(u8 path, u8 enable)
{
    struct snd_soc_codec *codec =  d1981_codec;
    struct d1981_priv *d1981=(struct d1981_priv *)codec->drvdata;
    
    dbg("\n\n   ###   d1981PathEnable enable=%d PATH = %d  ### \n\n",enable,path);
	switch(path) 
	{
		// UseCase 1
		case D1981_AnaMIC1_TO_PCM_2:
		case D1981_AnaMIC1_TO_PCM_2_VT:
		case D1981_AnaMIC1_TO_PCM_2_VOIP:
		case D1981_AnaMIC1_TO_PCM_2_LB:
		case D1981_AnaMIC1_TO_PCM_2_APPS:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_APPS:
		       if(enable)
		       {			        
			        d1981_reg_write_bit(D1981_REG_MIC3, 0xff, D1981_PGA_ENABLE_BIT, 0);
			        d1981_reg_write_bit(D1981_REG_MIC2, 0xff, D1981_PGA_ENABLE_BIT, 0);
			        d1981_reg_write_bit(D1981_REG_MIC1, 0xff, D1981_PGA_ENABLE_BIT, 1);
			        
			   }
			   else
			   {
                    d1981_reg_write_bit(D1981_REG_MIC1, 0xff, D1981_PGA_ENABLE_BIT, 0);
			   }
			break;
		
		case D1981_AnaMIC1_LOUD_TO_PCM_2:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_VT:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_VOIP:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_LB:		
		        if(enable)
		        {
			        d1981_reg_write_bit(D1981_REG_MIC1, 0xff, D1981_PGA_ENABLE_BIT, 0);			        
			        d1981_reg_write_bit(D1981_REG_MIC2, 0xff, D1981_PGA_ENABLE_BIT, 0);
			        d1981_reg_write_bit(D1981_REG_MIC3, 0xff, D1981_PGA_ENABLE_BIT, 1);
			        
			    }
			    else
			    {
                    d1981_reg_write_bit(D1981_REG_MIC3, 0xff, D1981_PGA_ENABLE_BIT, 0);
			    }
			break;
		
		case D1981_AnaMIC2_TO_PCM_2:
		case D1981_AnaMIC2_TO_PCM_2_VT:
		case D1981_AnaMIC2_TO_PCM_2_VOIP:
		case D1981_AnaMIC2_TO_PCM_2_LB:
		case D1981_AnaMIC2_TO_PCM_2_APPS:
  			    if(enable)
		        {
			        d1981_reg_write_bit(D1981_REG_MIC1, 0xff, D1981_PGA_ENABLE_BIT, 0);
			        d1981_reg_write_bit(D1981_REG_MIC3, 0xff, D1981_PGA_ENABLE_BIT, 0);
			        d1981_reg_write_bit(D1981_REG_MIC2, 0xff, D1981_PGA_ENABLE_BIT, 1);
			        
			    }
			    else
			    {
                    d1981_reg_write_bit(D1981_REG_MIC2, 0xff, D1981_PGA_ENABLE_BIT, 0);
			    }
			break;
		
		case D1981_PCM_2_TO_HS_VIA_DAC12:
		case D1981_PCM_2_TO_HS_VIA_DAC12_VT:
		case D1981_PCM_2_TO_HS_VIA_DAC12_VOIP:
		case D1981_PCM_2_TO_HS_VIA_DAC12_LB:
		    if(enable)
		    { //rcv disable
		        d1981_reg_write_bit(D1981_REG_LIN2, 0xff, (D1981_DAC_ENABLE_BIT | D1981_PGA_ENABLE_BIT), 0);
		    }
			d1981_reg_write_bit(D1981_REG_HPL, 0xff, (D1981_DAC_ENABLE_BIT | D1981_HPL_OUT_HIZ_EN), enable);
			d1981_reg_write_bit(D1981_REG_HPR, 0xff, (D1981_DAC_ENABLE_BIT | D1981_HPR_OUT_HIZ_EN), enable);
			break;
#if 0		
		case D1981_PCM_2_TO_SPKR_VIA_DAC3:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_VT:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_VOIP:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_LB:
		    d1981ClassDEnable(enable);
			d1981_reg_write_bit(D1981_REG_LIN3, 0xff, (D1981_DAC_ENABLE_BIT | D1981_PGA_ENABLE_BIT), enable);
			//d1981_reg_write_bit(0xb3, 0xff, (0x09), enable);
			break;
#endif		
		case D1981_PCM_2_TO_RCV_VIA_DAC4:
		case D1981_PCM_2_TO_RCV_VIA_DAC4_VT:
		case D1981_PCM_2_TO_RCV_VIA_DAC4_VOIP:
		case D1981_PCM_2_TO_RCV_VIA_DAC4_LB:
			d1981_reg_write_bit(D1981_REG_LIN2, 0xff, (D1981_DAC_ENABLE_BIT | D1981_PGA_ENABLE_BIT), enable);
			break;

		// UseCase 2
		case D1981_AnaMIC1_TO_I2S_1:
			d1981_reg_write_bit(D1981_REG_MIC1, 0xff, D1981_PGA_ENABLE_BIT, enable);
			break;
		
		case D1981_AnaMIC1_LOUD_TO_I2S_1:
		#if 1
		    d1981_reg_write_bit(D1981_REG_MIC1, 0xff, D1981_PGA_ENABLE_BIT, enable);
		#else
			d1981_reg_write_bit(D1981_REG_MIC3, 0xff, D1981_PGA_ENABLE_BIT, enable);
	    #endif
			break;
		
		case D1981_AnaMIC2_TO_I2S_1:
			d1981_reg_write_bit(D1981_REG_MIC2, 0xff, D1981_PGA_ENABLE_BIT, enable);
			break;

		// UseCase 3a				
		case D1981_I2S_1_TO_HS_VIA_DAC12:
		case D1981_AUX1_TO_HS_VIA_DAC12:
		    if(enable)
		    { //rcv disable
		        d1981_reg_write_bit(D1981_REG_LIN2, 0xff, (D1981_DAC_ENABLE_BIT | D1981_PGA_ENABLE_BIT), 0);
		    }
			d1981_reg_write_bit(D1981_REG_HPL, 0xff, (D1981_DAC_ENABLE_BIT | D1981_HPL_OUT_HIZ_EN), enable);
			d1981_reg_write_bit(D1981_REG_HPR, 0xff, (D1981_DAC_ENABLE_BIT | D1981_HPR_OUT_HIZ_EN), enable);
			break;
		
		case D1981_I2S_1_TO_SPKR_VIA_DAC3:
		case D1981_AUX1_SPKR_VIA_DAC3:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_VT:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_VOIP:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_LB:
		    if(enable)
		    { //rcv disable
		        d1981_reg_write_bit(D1981_REG_LIN2, 0xff, (D1981_DAC_ENABLE_BIT | D1981_PGA_ENABLE_BIT), 0);
		    }
		    d1981ClassDEnable(enable);
			d1981_reg_write_bit(D1981_REG_LIN3, 0xff, (D1981_DAC_ENABLE_BIT | D1981_PGA_ENABLE_BIT), enable);
			break;
		//UserCase 3c
		case D1981_I2S_1_TO_RCV_VIA_DAC4:
		case D1981_AUX1_TO_RCV_VIA_DAC4:
			d1981_reg_write_bit(D1981_REG_LIN2, 0xff, (D1981_DAC_ENABLE_BIT | D1981_PGA_ENABLE_BIT), enable);
			break;

		case D1981_I2S_1_TO_SPKR_HS:
		    if(enable && d1981->working_path != WORKING_HEADSET)
		    {
		         d1981_reg_write_bit(D1981_REG_HPL, 0xff, (D1981_DAC_ENABLE_BIT | D1981_HPL_OUT_HIZ_EN), enable);
			     d1981_reg_write_bit(D1981_REG_HPR, 0xff, (D1981_DAC_ENABLE_BIT | D1981_HPR_OUT_HIZ_EN), enable);
            }
            d1981ClassDEnable(enable);
			d1981_reg_write_bit(D1981_REG_LIN3, 0xff, (D1981_DAC_ENABLE_BIT | D1981_PGA_ENABLE_BIT), enable);
			break;
			
		case D1981_I2S_1_TO_LINEOUT_VIA_DAC4:
			// not connected
			break;

		// UseCase 3b				
		case D1981_AUX1_TO_I2S_1: 
			// Enable / Disable FM recording
			// 0x63, 0x88 (enabled)
			// 0x63, 0x80 (disabled)
			break;
#if 0		
	    case D1981_AUX1_TO_HS_VIA_DAC12:
			d1981_reg_write_bit(D1981_REG_HPL, 0xff, (D1981_DAC_ENABLE_BIT | D1981_HPL_OUT_HIZ_EN), enable);
			d1981_reg_write_bit(D1981_REG_HPR, 0xff, (D1981_DAC_ENABLE_BIT | D1981_HPR_OUT_HIZ_EN), enable);
			break;
		
		case D1981_AUX1_SPKR_VIA_DAC3:
		    d1981ClassDEnable(enable);
			d1981_reg_write_bit(D1981_REG_LIN3, 0xff, (D1981_DAC_ENABLE_BIT | D1981_PGA_ENABLE_BIT), enable);
			break;
	
		case D1981_AUX1_TO_RCV_VIA_DAC4:
			d1981_reg_write_bit(D1981_REG_LIN2, 0xff, (D1981_DAC_ENABLE_BIT | D1981_PGA_ENABLE_BIT), enable);
			break;
#endif	
		// UseCase undefined		
		case D1981_AUX1_TO_PCM_2:
			break;
		
		case D1981_PCM_2_TO_LINEOUT_VIA_DAC4:
			break;
		
		case D1981_AUX1_TO_LINEOUT_VIA_DAC4:
			break;

        case D1981_LOOPBACK_MIC1_TO_HS_VIA_DAC12:
            d1981_reg_write_bit(D1981_REG_HPL, 0xff, (D1981_DAC_ENABLE_BIT | D1981_HPL_OUT_HIZ_EN), enable);
			d1981_reg_write_bit(D1981_REG_HPR, 0xff, (D1981_DAC_ENABLE_BIT | D1981_HPR_OUT_HIZ_EN), enable);
            break;
            
		case D1981_LOOPBACK_MIC1_TO_SPKR_VIA_DAC3:
		    d1981ClassDEnable(enable);
			d1981_reg_write_bit(D1981_REG_LIN3, 0xff, (D1981_DAC_ENABLE_BIT | D1981_PGA_ENABLE_BIT), enable);
            break;
        case D1981_LOOPBACK_MIC1_TO_RCV_VIA_DAC4:
            d1981_reg_write_bit(D1981_REG_LIN2, 0xff, (D1981_DAC_ENABLE_BIT | D1981_PGA_ENABLE_BIT), enable);
            break;
        case D1981_LOOPBACK_MIC2_TO_HS_VIA_DAC12:
            d1981_reg_write_bit(D1981_REG_HPL, 0xff, (D1981_DAC_ENABLE_BIT | D1981_HPL_OUT_HIZ_EN), enable);
			d1981_reg_write_bit(D1981_REG_HPR, 0xff, (D1981_DAC_ENABLE_BIT | D1981_HPR_OUT_HIZ_EN), enable);
            break;
        case D1981_LOOPBACK_MIC2_TO_SPKR_VIA_DAC3:
            d1981ClassDEnable(enable);
			d1981_reg_write_bit(D1981_REG_LIN3, 0xff, (D1981_DAC_ENABLE_BIT | D1981_PGA_ENABLE_BIT), enable);
            break;
        case D1981_LOOPBACK_MIC2_TO_RCV_VIA_DAC4:
            d1981_reg_write_bit(D1981_REG_LIN2, 0xff, (D1981_DAC_ENABLE_BIT | D1981_PGA_ENABLE_BIT), enable);
            break;
		default:
			break;
			
	}
	return(1);
}

/* d1981PathMute

Mute acm path

*/


int d1981PathMute(u8 path, ACM_AudioMute mute)
{
    struct snd_soc_codec *codec =  d1981_codec;
    struct d1981_priv *d1981=(struct d1981_priv *)codec->drvdata;
 
	switch(path) 
	{
		// UseCase 1
		case D1981_AnaMIC1_TO_PCM_2:
		case D1981_AnaMIC1_TO_PCM_2_VT:
		case D1981_AnaMIC1_TO_PCM_2_VOIP:
		case D1981_AnaMIC1_TO_PCM_2_LB:
		case D1981_AnaMIC1_TO_PCM_2_APPS:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_APPS:
			d1981_reg_write_bit(D1981_REG_MIC1, 0xff, D1981_MUTE_BIT, mute);
			break;
		
		case D1981_AnaMIC1_LOUD_TO_PCM_2:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_VT:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_VOIP:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_LB:		
			d1981_reg_write_bit(D1981_REG_MIC3, 0xff, D1981_MUTE_BIT, mute);
			break;
		
		case D1981_AnaMIC2_TO_PCM_2:
		case D1981_AnaMIC2_TO_PCM_2_VT:
		case D1981_AnaMIC2_TO_PCM_2_VOIP:
		case D1981_AnaMIC2_TO_PCM_2_LB:
		case D1981_AnaMIC2_TO_PCM_2_APPS:
			d1981_reg_write_bit(D1981_REG_MIC2, 0xff, D1981_MUTE_BIT, mute);
			break;
		
		case D1981_PCM_2_TO_HS_VIA_DAC12:
		case D1981_PCM_2_TO_HS_VIA_DAC12_VT:
		case D1981_PCM_2_TO_HS_VIA_DAC12_VOIP:
		case D1981_PCM_2_TO_HS_VIA_DAC12_LB:
		case D1981_AUX1_TO_HS_VIA_DAC12:
		case D1981_I2S_1_TO_HS_VIA_DAC12:
			if(mute)
			{
				d1981_reg_write(D1981_REG_DAC1_2_SOFTMUTE, 0xc0); // softmute enable

				d1981_reg_write_bit(D1981_REG_HPL, 0xbf, 0x40, 1); //amp mute
			    d1981_reg_write_bit(D1981_REG_HPR, 0xbf, 0x40, 1); //amp mute	
				
			} 
			else
			{				
       	        d1981_reg_write(D1981_REG_DAC1_2_L_VOL, 0x0F); // left volume=default
				d1981_reg_write(D1981_REG_DAC1_2_R_VOL, 0x0F); // right volume=default
				
				d1981_reg_write(D1981_REG_DAC1_2_SOFTMUTE, 0x40); // softmute disable

				d1981_reg_write_bit(D1981_REG_HPL_VOL, 0xf0, 0x01, 1);	
                d1981_reg_write_bit(D1981_REG_HPR_VOL, 0xf0, 0x01, 1);
            
			    d1981_reg_write_bit(D1981_REG_HPL, 0xbf, 0x40, 0); //amp unmute
			    d1981_reg_write_bit(D1981_REG_HPR, 0xbf, 0x40, 0); //amp unmute
			    d1981->working_path=WORKING_HEADSET;
			}
			break;
		
		case D1981_PCM_2_TO_SPKR_VIA_DAC3:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_VT:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_VOIP:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_LB:
		case D1981_AUX1_SPKR_VIA_DAC3:
		case D1981_I2S_1_TO_SPKR_VIA_DAC3:
			if(mute) 
			{
   
			   d1981_reg_write(D1981_REG_DAC3_4_SOFTMUTE, 0xc0); // softmute enable
			   d1981_reg_write(D1981_REG_DAC3_4_L_VOL, 0x7f); // volume=mute
			   
      	       d1981_reg_write_bit(D1981_REG_DAC3_4_SEL, 0xff, (0x04), 1);
      	       d1981ClassDEnable(0);
			} 
			else 
			{			    
				//-> ringtone -> headset unplug
				d1981_reg_write(D1981_REG_DAC1_2_SOFTMUTE, 0xc0); // softmute enable

				d1981_reg_write_bit(D1981_REG_HPL, 0xbf, 0x40, 1); //amp mute
			    d1981_reg_write_bit(D1981_REG_HPR, 0xbf, 0x40, 1); //amp mute
				///<---
				d1981_reg_write_bit(D1981_REG_DAC3_4_SEL, 0xff, (0x04), 0);
				d1981_reg_write(D1981_REG_DAC3_4_SOFTMUTE, 0x00); // softmute disable
				//d1981_reg_write(D1981_REG_DAC2_L_VOL, 0x10); // volume=default
			    
			    d1981ClassDEnable(1);
			    d1981->working_path=WORKING_SPEAKER;
			}
			break;
		
		case D1981_PCM_2_TO_RCV_VIA_DAC4:
		case D1981_PCM_2_TO_RCV_VIA_DAC4_VT:
		case D1981_PCM_2_TO_RCV_VIA_DAC4_VOIP:
		case D1981_PCM_2_TO_RCV_VIA_DAC4_LB:
		case D1981_AUX1_TO_RCV_VIA_DAC4:
		case D1981_I2S_1_TO_RCV_VIA_DAC4:
			if(mute) 
			{
				d1981_reg_write(D1981_REG_DAC5_SOFTMUTE, 0x80); // softmute enable
				d1981_reg_write(D1981_REG_DAC5_L_VOL, 0x7f); // volume=mute
			} 
			else 
			{ 	
				d1981_reg_write(D1981_REG_DAC5_SOFTMUTE, 0x40); // softmute disable
				d1981->working_path=WORKING_RCV;
			}
			break;

		// UseCase 2
		case D1981_AnaMIC1_TO_I2S_1:
			d1981_reg_write_bit(D1981_REG_MIC1, 0xff, D1981_MUTE_BIT, mute);
			break;
		
		case D1981_AnaMIC1_LOUD_TO_I2S_1:		    
		    d1981_reg_write_bit(D1981_REG_MIC1, 0xff, D1981_MUTE_BIT, mute);		    
			break;
		
		case D1981_AnaMIC2_TO_I2S_1:
			d1981_reg_write_bit(D1981_REG_MIC2, 0xff, D1981_MUTE_BIT, mute);
			break;

	
		case D1981_I2S_1_TO_SPKR_HS:
		    if(mute) 
			{			    
			    d1981ClassDEnable(0);		    
				d1981_reg_write(D1981_REG_DAC3_4_SOFTMUTE, 0xc0); // softmute enable
				d1981_reg_write(D1981_REG_DAC3_4_L_VOL, 0x7f); // volume=mute	
			} 
			else 
			{
			    if( d1981->working_path != WORKING_HEADSET)
			    {
    			    d1981_reg_write(D1981_REG_DAC1_2_SOFTMUTE, 0x40); // softmute disable
    				
    			    d1981_reg_write_bit(D1981_REG_HPL, 0xbf, 0x40, 0); //amp unmute
    			    d1981_reg_write_bit(D1981_REG_HPR, 0xbf, 0x40, 0); //amp unmute
			    }
			    d1981_reg_write_bit(D1981_REG_DAC3_4_SEL, 0xff, (0x04), 0);
				d1981_reg_write(D1981_REG_DAC3_4_SOFTMUTE, 0x00); // softmute disable
				
			    d1981ClassDEnable(1);
			    d1981->working_path=WORKING_HEADSET_SPEAKER;
			}
		    break;
		    
		case D1981_I2S_1_TO_LINEOUT_VIA_DAC4:
			// lineout not used;
			break;

		// UseCase 3b				
		case D1981_AUX1_TO_I2S_1: 
			break;
		// UseCase undefined		
		case D1981_AUX1_TO_PCM_2:
			break;
		
		case D1981_PCM_2_TO_LINEOUT_VIA_DAC4:
			break;
		
		case D1981_AUX1_TO_LINEOUT_VIA_DAC4:
			break;

        case D1981_LOOPBACK_MIC1_TO_HS_VIA_DAC12:
            if(mute) 
			{
				d1981_reg_write(D1981_REG_DAC1_2_SOFTMUTE, 0xc0); // softmute enable
				d1981_reg_write(D1981_REG_DAC1_2_L_VOL, 0x7f); // left volume=mute
				d1981_reg_write(D1981_REG_DAC1_2_R_VOL, 0x7f); // right volume=mute
			} 
			else 
			{
			    d1981ClassDEnable(0);
				d1981_reg_write(D1981_REG_DAC1_2_SOFTMUTE, 0x40); // softmute disable
				//d1981_reg_write(D1981_REG_DAC1_L_VOL, 0x09); // left volume=default
				//d1981_reg_write(D1981_REG_DAC1_R_VOL, 0x09); // right volume=default
			}
            break;
        case D1981_LOOPBACK_MIC1_TO_SPKR_VIA_DAC3:
            if(mute) 
			{
			    d1981ClassDEnable(0);
				d1981_reg_write(D1981_REG_DAC3_4_SOFTMUTE, 0xc0); // softmute enable
				d1981_reg_write(D1981_REG_DAC3_4_L_VOL, 0x7f); // volume=mute				
			} 
			else 
			{
			    d1981ClassDEnable(1);
				d1981_reg_write(D1981_REG_DAC3_4_SOFTMUTE, 0x40); // softmute disable						       
			}
            break;
        case D1981_LOOPBACK_MIC1_TO_RCV_VIA_DAC4:
            if(mute) 
			{
				d1981_reg_write(D1981_REG_DAC5_SOFTMUTE, 0xc0); // softmute enable
				d1981_reg_write(D1981_REG_DAC5_L_VOL, 0x7f); // volume=mute
			} 
			else 
			{
			    d1981ClassDEnable(0);
				d1981_reg_write(D1981_REG_DAC5_SOFTMUTE, 0x40); // softmute disable				
			}
            break;
        case D1981_LOOPBACK_MIC2_TO_HS_VIA_DAC12:
            if(mute) 
			{
				d1981_reg_write(D1981_REG_DAC1_2_SOFTMUTE, 0xc0); // softmute enable
				d1981_reg_write(D1981_REG_DAC1_2_L_VOL, 0x7f); // left volume=mute
				d1981_reg_write(D1981_REG_DAC1_2_R_VOL, 0x7f); // right volume=mute
			} 
			else 
			{
			    d1981ClassDEnable(0);
				d1981_reg_write(D1981_REG_DAC1_2_SOFTMUTE, 0x40); // softmute disable				
			}
            break;
        case D1981_LOOPBACK_MIC2_TO_SPKR_VIA_DAC3:
            if(mute) 
			{
			    d1981ClassDEnable(0);
				d1981_reg_write(D1981_REG_DAC3_4_SOFTMUTE, 0xc0); // softmute enable
				d1981_reg_write(D1981_REG_DAC3_4_L_VOL, 0x7f); // volume=mute				
			} 
			else 
			{
			    d1981ClassDEnable(1);
				d1981_reg_write(D1981_REG_DAC3_4_SOFTMUTE, 0x40); // softmute disable	
		    }
            break;
        case D1981_LOOPBACK_MIC2_TO_RCV_VIA_DAC4:
            if(mute) 
			{
				d1981_reg_write(D1981_REG_DAC5_SOFTMUTE, 0xc0); // softmute enable
				d1981_reg_write(D1981_REG_DAC5_L_VOL, 0x7f); // volume=mute
			} 
			else 
			{
			    d1981ClassDEnable(0);
				d1981_reg_write(D1981_REG_DAC5_SOFTMUTE, 0x40); // softmute disable				
			}
            break;
		default:
			break;
			
	}


	return(1);
}


int d1981PathVolume2Set(u8 path, ACM_AudioVolume volume,unsigned char dynamic)
{
	volume=d1981_codec_gain[path][1]; //by request of samsung hardware
	
	switch(path) {
		// UseCase 1
		case D1981_AnaMIC1_TO_PCM_2:
		case D1981_AnaMIC1_TO_PCM_2_VT:
		case D1981_AnaMIC1_TO_PCM_2_VOIP:
		case D1981_AnaMIC1_TO_PCM_2_LB:
		case D1981_AnaMIC1_TO_PCM_2_APPS:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_APPS:
			d1981_reg_write_bit(D1981_REG_MIC1, 0xe0, volume, 1); //mic1 pga gain
			break;
		
		case D1981_AnaMIC1_LOUD_TO_PCM_2:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_VT:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_VOIP:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_LB:		
			d1981_reg_write_bit(D1981_REG_MIC3, 0xe0, volume, 1); //mic1 pga gain
			break;
		
		case D1981_AnaMIC2_TO_PCM_2:
		case D1981_AnaMIC2_TO_PCM_2_VT:
		case D1981_AnaMIC2_TO_PCM_2_VOIP:
		case D1981_AnaMIC2_TO_PCM_2_LB:
		case D1981_AnaMIC2_TO_PCM_2_APPS:
			d1981_reg_write_bit(D1981_REG_MIC2, 0xe0, volume, 1); //mic2 pga gain
			break;
		
		case D1981_PCM_2_TO_HS_VIA_DAC12:
		case D1981_PCM_2_TO_HS_VIA_DAC12_VT:
		case D1981_PCM_2_TO_HS_VIA_DAC12_VOIP:
		case D1981_PCM_2_TO_HS_VIA_DAC12_LB:
		    if(dynamic != 0xFF)
		        volume=d1981DynamicRxVolume(path,dynamic);
			d1981_reg_write_bit(D1981_REG_DAC1_2_L_VOL, 0x00, volume, 1);	// DAC1-L  0 max volume
			d1981_reg_write_bit(D1981_REG_DAC1_2_R_VOL, 0x00, volume, 1);	// DAC1-R volume
			break;
		
		case D1981_PCM_2_TO_SPKR_VIA_DAC3:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_VT:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_VOIP:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_LB:
		    if(dynamic != 0xFF)
		        volume=d1981DynamicRxVolume(path,dynamic);
			d1981_reg_write_bit(D1981_REG_DAC3_4_L_VOL, 0x00, volume, 1);	// DAC2-L volume
			break;
		
		case D1981_PCM_2_TO_RCV_VIA_DAC4:
		case D1981_PCM_2_TO_RCV_VIA_DAC4_VT:
		case D1981_PCM_2_TO_RCV_VIA_DAC4_VOIP:
		case D1981_PCM_2_TO_RCV_VIA_DAC4_LB:
		    if(dynamic != 0xFF)
		        volume=d1981DynamicRxVolume(path,dynamic);
			d1981_reg_write_bit(D1981_REG_DAC5_L_VOL, 0x00, volume, 1);	// DAC3-L volume
			break;

		// UseCase 2
		case D1981_AnaMIC1_TO_I2S_1:
			d1981_reg_write_bit(D1981_REG_MIC1, 0xe0, volume, 1); //mic1 pga gain
			break;
		
		case D1981_AnaMIC1_LOUD_TO_I2S_1:
		    #if 1
		    d1981_reg_write_bit(D1981_REG_MIC1, 0xe0, volume, 1); //mic1 pga gain
		    #else
			d1981_reg_write_bit(D1981_REG_MIC3, 0xe0, volume, 1); //mic1 pga gain
			#endif
			break;
		
		case D1981_AnaMIC2_TO_I2S_1:
			d1981_reg_write_bit(D1981_REG_MIC2, 0xe0, volume, 1); //mic2 pga gain
			break;

		// UseCase 3a				
		case D1981_I2S_1_TO_HS_VIA_DAC12:
			d1981_reg_write_bit(D1981_REG_DAC1_2_L_VOL, 0x00, volume, 1);	// DAC1-L  0 max volume
			d1981_reg_write_bit(D1981_REG_DAC1_2_R_VOL, 0x00, volume, 1);	// DAC1-R  volume
			break;
		
		case D1981_I2S_1_TO_SPKR_VIA_DAC3:
			d1981_reg_write_bit(D1981_REG_DAC3_4_L_VOL, 0x00, volume, 1);	// DAC2-L volume
			break;
		
		case D1981_I2S_1_TO_RCV_VIA_DAC4:
			d1981_reg_write_bit(D1981_REG_DAC5_L_VOL, 0x00, volume, 1);	// DAC3-L volume
			break;

		case D1981_I2S_1_TO_SPKR_HS: //ringtone	
		#if 0
		    if(dynamic != 0xFF)
		           d1981DynamicForceSpeakerRxVolume(dynamic);
            else
        #endif            
            {
                d1981_reg_write_bit(D1981_REG_DAC3_4_L_VOL, 0x00, d1981_codec_force_speaker_gain[1][1], 1);
                d1981_reg_write_bit(D1981_REG_DAC1_2_L_VOL, 0x00, d1981_codec_force_speaker_gain[0][1], 1);	// DAC1-L 0 max volume
                d1981_reg_write_bit(D1981_REG_DAC1_2_R_VOL, 0x00, d1981_codec_force_speaker_gain[0][1], 1);	// DAC1-R volume
            }
			break;
			
		case D1981_I2S_1_TO_LINEOUT_VIA_DAC4:
			break;

		// UseCase 3b				
		case D1981_AUX1_TO_I2S_1: 
			break;
		
	    case D1981_AUX1_TO_HS_VIA_DAC12:
#ifndef FMRADIO_VOLUME_TUNNING	    
            if(dynamic != 0xFF)
            {
                gFmRadioVolume=d1981DynamicFMVolume(path,dynamic);
            }
			d1981_reg_write_bit(D1981_REG_DAC1_2_L_VOL, 0x00, gFmRadioVolume, 1);	// DAC1-L 0 max volume
			d1981_reg_write_bit(D1981_REG_DAC1_2_R_VOL, 0x00, gFmRadioVolume, 1);	// DAC1-R volume
#else
            d1981_reg_write_bit(D1981_REG_DAC1_2_L_VOL, 0x00, gFmRadioVolume, 1);	// DAC1-L 0 max volume
			d1981_reg_write_bit(D1981_REG_DAC1_2_R_VOL, 0x00, gFmRadioVolume, 1);	// DAC1-R volume
#endif
			 d1981_reg_write_bit(D1981_REG_HPL_VOL, 0xf0, 0xf, 1);	// HP-L amp volume
			 d1981_reg_write_bit(D1981_REG_HPR_VOL, 0xf0, 0xf, 1);	// HP-R amp volume
			break;
		
		case D1981_AUX1_SPKR_VIA_DAC3:
#ifndef FMRADIO_VOLUME_TUNNING			
            if(dynamic != 0xFF)
            {
                dbg( "BSBWBSBSB dynamic=%d \n", dynamic);
                gFmRadioVolume=d1981DynamicFMVolume(path,dynamic);
            }
			d1981_reg_write_bit(D1981_REG_DAC3_4_L_VOL, 0x00, gFmRadioVolume, 1);	//  DAC2-L
#else
            d1981_reg_write_bit(D1981_REG_DAC3_4_L_VOL, 0x00, gFmRadioVolume, 1);
#endif
            d1981_reg_write_bit(D1981_REG_LIN3, 0xf0, 0xf, 1);
			break;
		
		case D1981_AUX1_TO_RCV_VIA_DAC4:
			d1981_reg_write_bit(D1981_REG_DAC5_L_VOL, 0x00, volume, 1);	//  DAC3-L
			break;

		// UseCase undefined		
		case D1981_AUX1_TO_PCM_2:
			break;
		
		case D1981_PCM_2_TO_LINEOUT_VIA_DAC4:
			break;
		
		case D1981_AUX1_TO_LINEOUT_VIA_DAC4:
			break;
		
		default:
			break;
			
	}


	return(1);
}
int d1981PathVolumeSet(u8 path, ACM_AudioVolume volume,unsigned char dynamic)
{
	volume=d1981_codec_gain[path][0]; //by request of samsung hardware
	
	switch(path) {
		// UseCase 1
		case D1981_AnaMIC1_TO_PCM_2:
		case D1981_AnaMIC1_TO_PCM_2_VT:
		case D1981_AnaMIC1_TO_PCM_2_VOIP:
		case D1981_AnaMIC1_TO_PCM_2_LB:
		case D1981_AnaMIC1_TO_PCM_2_APPS:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_APPS:
			d1981_reg_write_bit(D1981_REG_MIC1_PRE, 0x00, volume, 1);
			break;
		
		case D1981_AnaMIC1_LOUD_TO_PCM_2:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_VT:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_VOIP:
		case D1981_AnaMIC1_LOUD_TO_PCM_2_LB:		
			d1981_reg_write_bit(D1981_REG_MIC3_PRE, 0x00, volume, 1);
			break;
		
		case D1981_AnaMIC2_TO_PCM_2:
		case D1981_AnaMIC2_TO_PCM_2_VT:
		case D1981_AnaMIC2_TO_PCM_2_VOIP:
		case D1981_AnaMIC2_TO_PCM_2_LB:
		case D1981_AnaMIC2_TO_PCM_2_APPS:
			d1981_reg_write_bit(D1981_REG_MIC2_PRE, 0x00, volume, 1);
			break;
		
		
		case D1981_PCM_2_TO_HS_VIA_DAC12:
		case D1981_PCM_2_TO_HS_VIA_DAC12_VT:
		case D1981_PCM_2_TO_HS_VIA_DAC12_VOIP:
		case D1981_PCM_2_TO_HS_VIA_DAC12_LB:
			d1981_reg_write_bit(D1981_REG_HPL_VOL, 0xf0, volume, 1);	// HP-L amp volume
			d1981_reg_write_bit(D1981_REG_HPR_VOL, 0xf0, volume, 1);	// HP-R amp volume
			break;
		
		case D1981_PCM_2_TO_SPKR_VIA_DAC3:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_VT:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_VOIP:
		case D1981_PCM_2_TO_SPKR_VIA_DAC3_LB:
			d1981_reg_write_bit(D1981_REG_LIN3, 0xf0, volume, 1);	// HP-R amp volume
			break;
		
		case D1981_PCM_2_TO_RCV_VIA_DAC4:
		case D1981_PCM_2_TO_RCV_VIA_DAC4_VT:
		case D1981_PCM_2_TO_RCV_VIA_DAC4_VOIP:
		case D1981_PCM_2_TO_RCV_VIA_DAC4_LB:
			d1981_reg_write_bit(D1981_REG_LIN2, 0xf0, volume, 1);	// HP-R amp volume
			break;

		// UseCase 2
		case D1981_AnaMIC1_TO_I2S_1:
			d1981_reg_write_bit(D1981_REG_MIC1_PRE, 0x00, volume, 1);
			break;
		
		case D1981_AnaMIC1_LOUD_TO_I2S_1:
		    #if 1
		    d1981_reg_write_bit(D1981_REG_MIC1_PRE, 0x00, volume, 1);
		    #else
			d1981_reg_write_bit(D1981_REG_MIC3_PRE, 0x00, volume, 1);
			#endif
			break;
		
		case D1981_AnaMIC2_TO_I2S_1:
			d1981_reg_write_bit(D1981_REG_MIC2_PRE, 0x00, volume, 1);
			break;
		

		// UseCase 3a				
		case D1981_I2S_1_TO_HS_VIA_DAC12:
			d1981_reg_write_bit(D1981_REG_HPL_VOL, 0xf0, volume, 1);	// HP-L amp volume
			d1981_reg_write_bit(D1981_REG_HPR_VOL, 0xf0, volume, 1);	// HP-R amp volume
			break;
		
		case D1981_I2S_1_TO_SPKR_VIA_DAC3:
			d1981_reg_write_bit(D1981_REG_LIN3, 0xf0, volume, 1);	// HP-R amp volume
			break;
		
		case D1981_I2S_1_TO_RCV_VIA_DAC4:
			d1981_reg_write_bit(D1981_REG_LIN2, 0xf0, volume, 1);	// HP-R amp volume
			break;

		case D1981_I2S_1_TO_SPKR_HS: //ringtone
		    d1981_reg_write_bit(D1981_REG_LIN3, 0xf0, d1981_codec_force_speaker_gain[1][0], 1);
            d1981_reg_write_bit(D1981_REG_HPL_VOL, 0xf0, d1981_codec_force_speaker_gain[0][0], 1);	
            d1981_reg_write_bit(D1981_REG_HPR_VOL, 0xf0, d1981_codec_force_speaker_gain[0][0], 1);		    
		    break;
		    
		case D1981_I2S_1_TO_LINEOUT_VIA_DAC4:
			break;

		// UseCase 3b				
		case D1981_AUX1_TO_I2S_1: 
			break;
		
	    case D1981_AUX1_TO_HS_VIA_DAC12:
	         d1981_reg_write_bit(D1981_REG_AUX1L, 0xc0, volume, 1);
	         d1981_reg_write_bit(D1981_REG_AUX1R, 0xc0, volume, 1);
			break;
		
		case D1981_AUX1_SPKR_VIA_DAC3:
	 	    d1981_reg_write_bit(D1981_REG_AUX1L, 0xc0, volume, 1);
            d1981_reg_write_bit(D1981_REG_AUX1R, 0xc0, volume, 1);
			break;
		
		case D1981_AUX1_TO_RCV_VIA_DAC4:
			d1981_reg_write_bit(D1981_REG_LIN2, 0xf0, volume, 1);	// HP-R amp volume
			break;

		// UseCase undefined		
		case D1981_AUX1_TO_PCM_2:
			break;
		
		case D1981_PCM_2_TO_LINEOUT_VIA_DAC4:
			break;
		
		case D1981_AUX1_TO_LINEOUT_VIA_DAC4:
			break;
		
		default:
			break;
			
	}


	return(1);
}


/*************************************************************************************************************************
D1981 control functions
*************************************************************************************************************************/
static u32 wait_for_mailbox_status(void)
{
	unsigned int i;
	u32	value;

	//d1981_reg_read(1);						// Short delay, just to give some time to DSP
	for (i = 0; i <= 3; i++) {
		if (i) msleep(i);						// Waiting up to 6 ms in total
		value = get_mailbox_status();
		if (value) return value;
	}
	
	dbg("DSP Not responding.\n");
	return (value);
}

static u32 get_mailbox_status(void)
{
	u32	value;

	D1981_MBOX_SEND(DSP_MB_STATUS_REQ);
	value = d1981_reg_read(0x76);
#if 0		// Only first byte of status is used by the codec driver
	value |= d1981_reg_read(0x77)<<8;
	value |= d1981_reg_read(0x78)<<16;
#endif
#ifdef D1981_EH_SIM_TEST
	// Error simulation
	if (ehIfError(ehNoResp)) value = 0;
	if (ehIfError(ehBadStatus)) value = 0xFEFE00;
#endif
#ifndef D1981_FAST_LOADING
	dbg("MBOX status 0x%06X\n", value);
#endif	
	return (value);
}

static u32 get_mailbox_counter(void)
{
	u32	value;
	
	D1981_MBOX_SEND(DSP_MB_CMD_COUNT_REQ);
	value = d1981_reg_read( 0x76);
	value |= d1981_reg_read( 0x77)<<8;
	value |= d1981_reg_read( 0x78)<<16;
#ifdef D1981_EH_SIM_TEST
	// Error simulation
	if (ehIfError(ehBadCounter)) value -= 2;
#endif
#ifndef D1981_FAST_LOADING
	dbg("MBOX cmd count 0x%06X\n", value);
#endif	
	return value;
}


#if 0	// not used
/* dsp_dump

DSP memory dump utility function

*/
static void dsp_dump(struct snd_soc_codec *codec, int select)
{
	unsigned int i, value1, value2, value3, value4;

	if(select==PMEM) {
		d1981_soc_write(codec, DMA_CMD, PMEM | 0x10);
		d1981_soc_write(codec, DMA_ADDR0, BYTE0(_PMEM_start));
		d1981_soc_write(codec, DMA_ADDR1, BYTE1(_PMEM_start));
		for (i=0; i<_PMEM_size; i++) {
			value1=d1981_reg_read( DMA_DATA0);
			value2=d1981_reg_read( DMA_DATA1);
			value3=d1981_reg_read( DMA_DATA2);
			value4=d1981_reg_read( DMA_DATA3);
			printk(KERN_ALERT "0x%02x 0x%02x 0x%02x 0x%02x\n",
				value1, value2, value3, value4);
		}
	} else if(select==XMEM){
		d1981_soc_write(codec, DMA_CMD, XMEM | 0x10);
		d1981_soc_write(codec, DMA_ADDR0, BYTE0(_PMEM_start));
		d1981_soc_write(codec, DMA_ADDR1, BYTE1(_PMEM_start));
		for (i=0; i<_XMEM_size; i++) {
			value2=d1981_reg_read( DMA_DATA1);
			value3=d1981_reg_read( DMA_DATA2);
			value4=d1981_reg_read( DMA_DATA3);
			printk(KERN_ALERT "0x%02x, 0x%02x 0x%02x\n",
				value2, value3, value4);
		}
	} else if(select==YMEM){
		d1981_soc_write(codec, DMA_CMD, YMEM | 0x10);
		d1981_soc_write(codec, DMA_ADDR0, BYTE0(_PMEM_start));
		d1981_soc_write(codec, DMA_ADDR1, BYTE1(_PMEM_start));
		for (i=0; i<_YMEM_size; i++) {
			value2=d1981_reg_read( DMA_DATA1);
			value3=d1981_reg_read( DMA_DATA2);
			value4=d1981_reg_read( DMA_DATA3);
			printk(KERN_ALERT "0x%02x 0x%02x 0x%02x\n",
				value2, value3, value4);
		}
	}

}
#endif

/* load_use_case

Load DSP use case from header file array via mailbox registers

*/
/*static*/ int load_use_case(const u8 *use_case, u16 len)
{
	unsigned int i,j;
	u32	status, count;

	status = wait_for_mailbox_status();
	if (status != CMD_INTERP_STATUS_OK) {
#ifdef D1981_EH_SUPPORT
		// If error code is returned before sending any command list - reinitialize framework.
		// If DSP did not responded at all (busy state) - perform reset as we cannot send any command to it.
		if (status) ehSetRequest(ehReinit);		
		else ehSetRequest(ehReset);
		return (-1);
#else
#ifndef D1981_FAST_LOADING
		dbg("load_use_case() DSP not ready. mbox_status=0x%06X\n", status);
#endif		
#endif
	}

	count = get_mailbox_counter();				// Used to verify if all commands were executed

#ifdef I2C_REPEATED_WRITE_MODE
#if 0
	for (i = len/(MAILBOX_I2C_PAGE_SIZE/2); i; i--) {
		d1981_block_write(MAILBOX_I2C_PAGE_SIZE, use_case);
		use_case += MAILBOX_I2C_PAGE_SIZE;
	}
	d1981_block_write((2*len)%MAILBOX_I2C_PAGE_SIZE, use_case);
#endif

	for (i=0, j=0; i<(2*len)/MAILBOX_I2C_PAGE_SIZE; i++, j+=MAILBOX_I2C_PAGE_SIZE) {
		d1981_block_write(MAILBOX_I2C_PAGE_SIZE, &use_case[j] );
	}
	for (i=0; i<((2*len)%MAILBOX_I2C_PAGE_SIZE)/2; i++, j+=2) {
		d1981_block_write(2, &use_case[j]);
	}

#else
	for (i = 0; i < len*2; i += 2)
		d1981_reg_write(use_case[i], use_case[i+1] );
#endif

	// It does the same as wait_for_mailbox_status(), except it counts mbox commands,
	// so we can test, whether command counter is correct.
	for (i = 0; i <= 5; i++) {
		if (i) msleep(i);						// Waiting up to 6 ms in total
		status = get_mailbox_status();
		count++;
		if (status) break;
	}
	if (status != CMD_INTERP_STATUS_OK) {
#ifdef D1981_EH_SUPPORT
		// If error code is returned, first trying to abort and reinitialise framework.
		// If DSP is not responding at all - perform reset as we cannot send any command to it.
		if (status) ehSetRequest(ehReinit);
		else ehSetRequest(ehReset);
#else
		if (status) dbg("load_use_case() DSP failed to execute command.\n");
		else dbg("load_use_case() DSP not responding.\n");
#endif
		return(-1);
	}

#ifdef D1981_EH_SUPPORT
	// Finally, command counter is tested against script length.
	count = ((get_mailbox_counter() & 0x00FFFFFF) - count - 1)*3;		// Limiting value to 24 bits word size
	if (count != len) {
		dbg("load_use_case() Command counter indicates incomplete execution (%u != %hu)\n", count, len);
		ehSetRequest(ehRetry);
		return (-1);
	}
#endif

	return(0);
}


/* run_script

Run script (sequence of I2C register writes) from header file array

*/
static void run_script(const u8 *script, u16 len)
{
	unsigned int i;

	for (len *= 2, i = 0; i < len; i += 2)
		d1981_reg_write(script[i], script[i+1]);

	// Unable to optimise this to block writes, because of problems with stability.
	// Probably some writes require small delay before continuing.
}


/*************************************************************************************************
 Codec hardware initialisation
 *************************************************************************************************/
static void _d1981_init_dsp(void)
{
	d1981_reg_write(D1981_REG_DSP_CTRL, 0x01);		/* make sure DSP is in reset */

	dsp_download();

	d1981_reg_write(D1981_REG_DSP_CTRL, 0x03);		/* DSP ctrl release from reset */
	d1981_reg_write(D1981_REG_DATA_ROUTE, 0x7f);	/* routeing */

	if (wait_for_mailbox_status() != FRAMEWORK_INIT_OK) {
		dbg("DSP loading failed - No INIT_DONE status\n");
#ifdef D1981_EH_SUPPORT
		ehSetRequest(ehReset);
		return;
#endif
	}

	d1981_reg_write(D1981_REG_CLK_DSP, 0x07);		/* DSP CK 96MHz */

	D1981_MBOX_SEND(DSP_ABORT_RESET_FW);

	if (wait_for_mailbox_status() != CMD_INTERP_STATUS_OK) {
		dbg("DSP framework not started - No READY status\n");
#ifdef D1981_EH_SUPPORT
		ehSetRequest(ehReset);
#endif
	}
}

#ifdef D1981_EH_SUPPORT
static void d1981_init_dsp(void)
{
	// Calling _d1981_init_dsp() using error handling routine.
	// It already handles DSP errors.
	ehLockDsp();
	ehSetRequest(ehReset);
	ehRecoverDsp();
	ehUnlockDsp();
}
#else
static inline void d1981_init_dsp(void)
{
	_d1981_init_dsp();
}
#endif

/* d1981_init_codec

Main codec initialisation function

*/
static void d1981_init_codec(struct snd_soc_codec *codec /* needed? _kg */)
{
#ifdef I2C_REPEATED_WRITE_MODE
	d1981_reg_write(D1981_REG_CIF_CTRL2, 0x03);		/* Enable I2C repeated write mode */
	d1981_reg_read(1);								/* dummy read for mode change*/
	d1981_reg_read(1);								/* dummy read */
#endif

	D1981_SCRIPT_SEND(CodecInit);

	d1981_init_dsp();

 	d1981_hp_dc_offset_cancelation();

}

/* dsp_download

Download DSP firmware

Done once only at driver initialisation time

*/
static void dsp_download(void)
{
	int i;
	//int timestamp;
	const u8 *p;

    struct timeval now,now2;
    
#ifdef I2C_REPEATED_WRITE_MODE

    do_gettimeofday(&now);   
    
	/* DSP i2c repeated write mode */
	d1981_reg_write(0x74, 0x03);

	//timestamp = jiffies;
	d1981_reg_write(D1981_REG_DMA_CMD, PMEM);
	d1981_reg_write(D1981_REG_DMA_ADDR0, BYTE0(_PMEM_start));
	d1981_reg_write(D1981_REG_DMA_ADDR1, BYTE1(_PMEM_start));

#if 0
    for (i=0, p=_PMEM_val; i<_PMEM_size/32; i++, p+=8*32) {
		d1981_block_write( 8*32, p);
	}
	//for (i=0; i<_PMEM_size % I2C_PAGE_SIZE; i++, p+=8) 
	{
		d1981_block_write( _PMEM_size % 32, p);
	}
#else
	for (i=0, p=_PMEM_val; i<_PMEM_size/I2C_PAGE_SIZE; i++, p+=8*I2C_PAGE_SIZE) {
		d1981_block_write( 8*I2C_PAGE_SIZE, p);
	}
	for (i=0; i<_PMEM_size % I2C_PAGE_SIZE; i++, p+=8) {
		d1981_block_write( 8, p);
	}
#endif
    do_gettimeofday(&now2);
    dbg( "d1981 CODEC: PMEM Dsp download time =%ld usec.\n",1000000 * (now2.tv_sec - now.tv_sec)+now2.tv_usec-now.tv_usec);
       
    
	//dbg(KERN_ALERT "Lynx: DSP - PMEM downloaded %d bytes %d sec\n", _PMEM_size, (jiffies-timestamp)/HZ);

	//timestamp = jiffies;
	d1981_reg_write(D1981_REG_DMA_CMD, XMEM);
	d1981_reg_write(D1981_REG_DMA_ADDR0, BYTE0(_XMEM_start));
	d1981_reg_write(D1981_REG_DMA_ADDR1, BYTE1(_XMEM_start));
#if 0	
    for (i=0, p=_XMEM_val; i<_XMEM_size/32; i++, p+=6*32) {
		d1981_block_write(6*32, p);
	}
	//for (i=0; i<_XMEM_size % I2C_PAGE_SIZE; i++, p+=6) 
	{
		d1981_block_write( _XMEM_size%32, p);
	}
#else
	for (i=0, p=_XMEM_val; i<_XMEM_size/I2C_PAGE_SIZE; i++, p+=6*I2C_PAGE_SIZE) {
		d1981_block_write(6*I2C_PAGE_SIZE, p);
	}
	for (i=0; i<_XMEM_size % I2C_PAGE_SIZE; i++, p+=6) {
		d1981_block_write( 6, p);
	}
#endif	
	//printk(KERN_ALERT "Lynx: DSP - XMEM downloaded %d bytes %d sec\n", _XMEM_size, (jiffies-timestamp)/HZ);
    do_gettimeofday(&now);
    dbg("d1981 CODEC: XMEM Dsp download time =%ld usec.\n",1000000 * (now.tv_sec - now2.tv_sec)+now.tv_usec-now2.tv_usec);
       
	//timestamp = jiffies;
	d1981_reg_write(D1981_REG_DMA_CMD, YMEM);
	d1981_reg_write(D1981_REG_DMA_ADDR0, BYTE0(_YMEM_start));
	d1981_reg_write(D1981_REG_DMA_ADDR1, BYTE1(_YMEM_start));
	#if 0
	    d1981_block_write( _YMEM_size*6, _YMEM_val);
	#else
	for (i=0, p=_YMEM_val; i<_YMEM_size/I2C_PAGE_SIZE; i++, p+=6*I2C_PAGE_SIZE) {
		d1981_block_write( 6*I2C_PAGE_SIZE, p);
	}
	for (i=0; i<_YMEM_size % I2C_PAGE_SIZE; i++, p+=6) {
		d1981_block_write( 6, p);
	}
	#endif
	//printk(KERN_ALERT "Lynx: DSP - YMEM downloaded %d bytes %d sec\n", _YMEM_size, (jiffies-timestamp)/HZ);
    do_gettimeofday(&now2);
    dbg( "d1981 CODEC: YMEM Dsp download time =%ld usec.\n",1000000 * (now2.tv_sec - now.tv_sec)+now2.tv_usec-now.tv_usec);
       
#else
	timestamp = jiffies;
	d1981_reg_write(D1981_REG_DMA_CMD, PMEM);
	d1981_reg_write(D1981_REG_DMA_ADDR0, BYTE0(_PMEM_start));
	d1981_reg_write(D1981_REG_DMA_ADDR1, BYTE1(_PMEM_start));
	for (i=0, p=_PMEM_val; i<_PMEM_size*4; i++) {
		d1981_reg_write( *p++, *p++);
	}
	dbg( "Lynx: DSP - PMEM downloaded %d bytes %d sec\n", _PMEM_size, (jiffies-timestamp)/HZ);

	timestamp = jiffies;
	d1981_reg_write(D1981_REG_DMA_CMD, XMEM);
	d1981_reg_write(D1981_REG_DMA_ADDR0, BYTE0(_XMEM_start));
	d1981_reg_write(D1981_REG_DMA_ADDR1, BYTE1(_XMEM_start));
	for (i=0, p=_XMEM_val; i<_XMEM_size*3; i++) {
		d1981_reg_write( *p++, *p++);
	}
	dbg( "Lynx: DSP - XMEM downloaded %d bytes %d sec\n", _XMEM_size, (jiffies-timestamp)/HZ);

	timestamp = jiffies;
	d1981_reg_write(D1981_REG_DMA_CMD, YMEM);
	d1981_reg_write(D1981_REG_DMA_ADDR0, BYTE0(_YMEM_start));
	d1981_reg_write(D1981_REG_DMA_ADDR1, BYTE1(_YMEM_start));
	for (i=0, p=_YMEM_val; i<_YMEM_size*3; i++) {
		d1981_reg_write( *p++, *p++);
	}
	dbg("Lynx: DSP - YMEM downloaded %d bytes %d sec\n", _YMEM_size, (jiffies-timestamp)/HZ);

#endif
}


static int d1981_audio_startup(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct d1981_priv *d1981_audio = codec->drvdata;
#ifdef D1981_PCM_RECORDING
	dai->playback.rates = d1981_audio->playback_samplerate;
	dai->capture.rates = d1981_audio->capture_samplerate;
#endif
	return 0;
}

static int d1981_audio_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai __maybe_unused)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
#if 0
	int rate, value;
	rate =  params_rate(params);
	value = d1981_audio_read(codec, 0x0e);
	value &= 0xf0;
	switch(rate){
		case 48000:
			value |= 0x08;
			break;
		case 44100:
			value |= 0x07;
			break;
		case 32000:
			value |= 0x06;
			break;
		case 24000:
			value |= 0x05;
			break;
		case 22050:
			value |= 0x04;
			break;
		case 16000:
			value |= 0x03;
			break;
		case 12000:
			value |= 0x02;
			break;
		case 11025:
			value |= 0x01;
			break;
		case 8000:
			value |= 0x00;
			break;
		default:
			pr_debug("unsupported rate\n");
			return -EINVAL;
			break;
	}
	/*d1981_audio_write(codec, 0x0e, value); d1981 has sample rate switching issue*/
#endif
	return 0;
}

//*
#ifdef D1981_PCM_RECORDING
#define D1981_AUDIO_HIFI_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		                SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)
#define D1981_AUDIO_VOICE_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
				SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_48000)
#define D1981_AUDIO_HIFI_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)
#define D1981_AUDIO_VOICE_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)

/*
Template: no dai ops currently defined
*/

static struct snd_soc_dai_ops d1981_dai_ops = { 
    .startup = d1981_audio_startup,
//	.shutdown = d1981_hifi_shutdown,
	.hw_params = d1981_audio_hifi_hw_params,
//	.digital_mute = d1981_audio_mute,
//	.set_fmt = d1981_aduio_hifi_set_dai_fmt,
	.set_clkdiv = NULL,
	.set_pll = NULL,
	.set_sysclk = NULL,
//	.trigger = d1981_audio_trigger,

};

/*
 * HIFI DAI
 */
struct snd_soc_dai d1981_audio_dai[]={
/* DAI HIFI mode*/
	{
		.name = "d1981 audio HiFi",
		.id = 0,
		.playback = {
			.stream_name = "HiFi Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = D1981_AUDIO_HIFI_RATES,
			.formats = D1981_AUDIO_HIFI_FORMATS,
		},
		.capture = {
			.stream_name = "HiFi Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = D1981_AUDIO_HIFI_RATES,
			.formats = D1981_AUDIO_HIFI_FORMATS,
		},
		.ops = &d1981_dai_ops,
	},

	{
		.name = "d1981 audio pcm",
		.id = 1,
		.playback = {
			.stream_name = "Pcm Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = D1981_AUDIO_VOICE_RATES,
			.formats = D1981_AUDIO_VOICE_FORMATS,
		},
		.capture = {
			.stream_name = "Pcm Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = D1981_AUDIO_VOICE_RATES,
			.formats = D1981_AUDIO_VOICE_FORMATS,
		},
		.ops = &d1981_dai_ops,
	},

	{
		.name = "d1981 audio HiFi playback only",
		.id = 2,
		.playback = {
			.stream_name = "HiFi Playback only",
			.channels_min = 1,
			.channels_max = 2,
			.rates = D1981_AUDIO_HIFI_RATES,
			.formats = D1981_AUDIO_HIFI_FORMATS,
		},
		.ops = &d1981_dai_ops,
	},

	{
		.name = "d1981 audio pcm capture only",
		.id = 3,
		.capture = {
			.stream_name = "HiFi Capture only",
			.channels_min = 1,
			.channels_max = 2,
			.rates = D1981_AUDIO_VOICE_RATES,
			.formats = D1981_AUDIO_VOICE_FORMATS,
		},
		.ops = &d1981_dai_ops,
	},
};
#else

#define D1981_HIFI_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000)

#define D1981_VOICE_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
		SNDRV_PCM_RATE_32000)

#define D1981_PCM_FORMATS (SNDRV_PCM_FMTBIT_S16_LE)

/*
Template: no dai ops currently defined
*/

static struct snd_soc_dai_ops d1981_dai_ops = {
//	.prepare = d1981_hifi_prepare,
//	.shutdown = d1981_hifi_shutdown,
//	.hw_params = d1981_audio_hifi_hw_params,
//	.digital_mute = d1981_audio_mute,
//	.set_fmt = d1981_aduio_hifi_set_dai_fmt,
	.set_clkdiv = NULL,
	.set_pll = NULL,
	.set_sysclk = NULL,
//	.trigger = d1981_audio_trigger,

};

struct snd_soc_dai d1981_audio_dai[] = {
{
	.name = "I2S HiFi",
	.playback = {
		.stream_name = "HiFi Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = D1981_HIFI_RATES,
		.formats = D1981_PCM_FORMATS,},
        .capture = {
		.stream_name = "HiFi Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = D1981_HIFI_RATES,
		.formats = D1981_PCM_FORMATS,},
	.ops = &d1981_dai_ops,
	},
	{
	.name = "PCM Voice",
	.playback = {
		.stream_name = "Voice Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = D1981_VOICE_RATES,
		.formats = D1981_PCM_FORMATS,},
	.capture = {
		.stream_name = "Voice Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = D1981_VOICE_RATES,
		.formats = D1981_PCM_FORMATS,},
	.ops = &d1981_dai_ops,
	},
};
#endif
#ifdef D1982_DELAYED_WORK
static void d1981_audio_work(struct work_struct *work)
{
	struct snd_soc_codec *codec =
			container_of(work, struct snd_soc_codec, delayed_work.work);
	//d1981_audio_set_bias_level(codec, codec->bias_level);
}
#endif

#if (0 && D1981_EH_LOCK)	// Not linked to interrupt event && requires DSP lock enabled

// d1981_event_handler() routine tries to recover DSP from any error state by resetting it.
// This should be linked with Audio Codec Interrupt but called in work queue context,
// because the routine may wait long and should not be reentered until finished.
void d1981_event_handler(void)
{
	u32	status;
	int path;

	ehLockDsp();

	// Event should be handled only when CODEC is initialised and enabled
	if ( d1981_codec && g_d1981_power_state ) {
		// If exception arose, when use case change was being executed, use case error handling routine 
		// should have already taken care about the exception, so check it.
		status = get_mailbox_status();
		if (BYTE0(status) == FRAMEWORK_FATAL_ERR) {
			dbg("Critical DSP Failure - handler called (mbox_status=%u)\n", status);
			ehSetRequest(ehReset);
			ehRecoverDsp();
			if (isAnyPathEnabled()) {
				for (path = 0; path < D1981_TOTAL_VALID_PATHS; path++)
					if (isPathEnabled(path)) d1981PathPutEnable(path, 230 /* ??? volume ??? */);
			}
		}
	}

	ehUnlockDsp();
}
EXPORT_SYMBOL_GPL(d1981_event_handler);

#endif

static int d1981_soc_suspend(struct platform_device *pdev,
	pm_message_t state)
{
   d1981PowerControl(0);

	return 0;
}

static int d1981_soc_resume(struct platform_device *pdev)
{
    d1981PowerControl(1);
 
	return 0;
}

static int d1981_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	/* For codec Power Management */
	return 0;
}

#if (D1981_HPL_DAC_OFF_CNTL_COMPO != (1<<3) || D1981_HPR_DAC_OFF_CNTL_COMPO != (1<<3))
	#error "D1981_HPL_DAC_OFF_CNTL_COMPO or D1981_HPR_DAC_OFF_CNTL_COMPO bit has been changed! Update source code!"
#endif
#if (D1981_HPL_OUT_COMPO != (1<<3) || D1981_HPR_OUT_COMPO != (1<<3))
	#error "D1981_HPL_OUT_COMPO or D1981_HPR_OUT_COMPO bit has been changed! Update source code!"
#endif
static void d1981_dac_offset_adjust(void)
{
	u8 offset[2], step = 0x20, sign[2];

	// Initialize DAC offset calibration circuits and registers 
	d1981_reg_write(D1981_REG_HPL_DAC_OFFSET, ~0 & 0x7F);
	d1981_reg_write(D1981_REG_HPR_DAC_OFFSET, ~0 & 0x7F);
	d1981_reg_write(D1981_REG_HPL_DAC_OFF_CNTL, 0x3);
	d1981_reg_write(D1981_REG_HPR_DAC_OFF_CNTL, 0x3);

	// Wait for voltage stabilization
	msleep(1);

	// Check DAC offset sign
	sign[0] = (d1981_reg_read(D1981_REG_HPL_DAC_OFF_CNTL) & D1981_HPL_DAC_OFF_CNTL_COMPO);
	sign[1] = (d1981_reg_read(D1981_REG_HPR_DAC_OFF_CNTL) & D1981_HPR_DAC_OFF_CNTL_COMPO);

	// Binary search DAC offset values (both channels at once)
	offset[0] = sign[0]<<3;
	offset[1] = sign[1]<<3;
	do {
		offset[0] |= step;
		offset[1] |= step;
		d1981_reg_write(D1981_REG_HPL_DAC_OFFSET, ~offset[0] & 0x7F);
		d1981_reg_write(D1981_REG_HPR_DAC_OFFSET, ~offset[1] & 0x7F);
		msleep(1);
		if ((d1981_reg_read(D1981_REG_HPL_DAC_OFF_CNTL) & D1981_HPL_DAC_OFF_CNTL_COMPO) ^ sign[0]) offset[0] &= ~step;
		if ((d1981_reg_read(D1981_REG_HPR_DAC_OFF_CNTL) & D1981_HPR_DAC_OFF_CNTL_COMPO) ^ sign[1]) offset[1] &= ~step;
		step >>= 1;
	} while (step);

	// Write final DAC offsets to registers
	d1981_reg_write(D1981_REG_HPL_DAC_OFFSET, ~offset[0] & 0x7F);
	d1981_reg_write(D1981_REG_HPR_DAC_OFFSET, ~offset[1] & 0x7F);

	// End DAC calibration mode
	d1981_reg_write(D1981_REG_HPL_DAC_OFF_CNTL, 0x2);
	d1981_reg_write(D1981_REG_HPR_DAC_OFF_CNTL, 0x2);
	
	printk(KERN_ALERT "DC offset cancellation for Headphones DACs  L:%c0x%02X  R:%c0x%02X\n",
	         (offset[0] & D1981_HPL_DAC_OFFSET_DAC_SIGN ? '+' : '-'),  offset[0] & 0x3F, 
	         (offset[1] & D1981_HPR_DAC_OFFSET_DAC_SIGN ? '+' : '-'), offset[1] & 0x3F);
}

static void d1981_output_offset_adjust(void)
{
	u8 offset[2] = {0, 0}, step = 0x80, sign[2];

	// Initialize output offset calibration circuits and registers 
	d1981_reg_write(D1981_REG_HPL_OUT_OFFSET, 0);
	d1981_reg_write(D1981_REG_HPR_OUT_OFFSET, 0);
	d1981_reg_write(D1981_REG_HPL, D1981_HPL_OUT_COMP | D1981_HPL_OUT_EN);
	d1981_reg_write(D1981_REG_HPR, D1981_HPL_OUT_COMP | D1981_HPR_OUT_EN);
	
	// Wait for voltage stabilization
	msleep(1);

	// Check output offset sign
	sign[0] = d1981_reg_read(D1981_REG_HPL) & D1981_HPL_OUT_COMPO;
	sign[1] = d1981_reg_read(D1981_REG_HPR) & D1981_HPR_OUT_COMPO;
	gHP_offset_sign[0] = sign[0]>>3;
	gHP_offset_sign[1] = sign[1]>>3;
	d1981_reg_write(D1981_REG_HPL, D1981_HPL_OUT_COMP | D1981_HPL_OUT_EN | gHP_offset_sign[0]);
	d1981_reg_write(D1981_REG_HPR, D1981_HPR_OUT_COMP | D1981_HPR_OUT_EN | gHP_offset_sign[1]);
	
	// Binary search output offset values (both channels at once)
	do {
		offset[0] |= step;
		offset[1] |= step;
		d1981_reg_write(D1981_REG_HPL_OUT_OFFSET, offset[0]);
		d1981_reg_write(D1981_REG_HPR_OUT_OFFSET, offset[1]);
		msleep(1);
		if ((d1981_reg_read(D1981_REG_HPL) & D1981_HPL_OUT_COMPO) ^ sign[0]) offset[0] &= ~step;
		if ((d1981_reg_read(D1981_REG_HPR) & D1981_HPR_OUT_COMPO) ^ sign[1]) offset[1] &= ~step;
		step >>= 1;
	} while (step);

	// Write final DAC offsets to registers
	d1981_reg_write(D1981_REG_HPL_OUT_OFFSET, offset[0]);
	d1981_reg_write(D1981_REG_HPR_OUT_OFFSET, offset[1]);
	printk(KERN_ALERT "DC offset cancellation for Headphones output AMPs  L:%c0x%02X  R:%c0x%02X\n", 
	         (sign[0] ? '+' : '-'), offset[0], 
	         (sign[1] ? '+' : '-'), offset[1]);
	         
    printk(KERN_ALERT "DC offset cancellation for Headphones output AMPs  gHP_offset_sign[0]=0x%x, gHP_offset_sign[1]=0x%x \n", 
	         gHP_offset_sign[0],gHP_offset_sign[1]);
}

static void d1981_hp_dc_offset_cancelation(void)
{
	// Enable output path for HP
	run_script(Lynx_UC_HP_offset_cancellation, Lynx_UC_HP_offset_cancellation_SIZE);

	// DC offsets cancellation
	d1981_dac_offset_adjust();
	d1981_output_offset_adjust();

	// Disable output path for HP
	d1981_reg_write(D1981_REG_HPL, 0x00);
	d1981_reg_write(D1981_REG_HPR, 0x00);
}

/*
Codec Driver initialisation

*/
static int d1981_init(struct snd_soc_device *socdev)
{
	struct snd_soc_codec *codec = d1981_codec;
	int ret = 0;

	codec->name = "d1981 audio";
	codec->owner = THIS_MODULE;
	codec->read = d1981_soc_read;
	codec->write = d1981_soc_write;
	codec->set_bias_level = d1981_set_bias_level;
	codec->dai = d1981_audio_dai;
	codec->num_dai = ARRAY_SIZE(d1981_audio_dai);
	codec->reg_cache_size = sizeof(d1981_reg);
	codec->reg_cache = kmemdup(d1981_reg, sizeof(d1981_reg), GFP_KERNEL);

	if(codec->reg_cache == NULL)
		return -ENOMEM;


	/* register pcms */
	pr_debug("[audio]-->d1981_init: create cards and their pcms.\n");
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "d1981 audio: failed to create pcms\n");
		goto pcm_err;
	}

    gCurrentPath = 72;	
    gCurrentUseCase=USE_CASE_UNDEFINED;

	d1981PowerOn();
	
	codec->bias_level = SND_SOC_BIAS_STANDBY;
#ifdef D1982_DELAYED_WORK
	schedule_delayed_work(&codec->delayed_work,msecs_to_jiffies(2));
#endif

	d1981_add_controls(codec);

	return ret;

#if 0	// Not used
card_err:
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);
#endif

pcm_err:
	kfree(codec->reg_cache);
	return ret;
}


/*
Codec Driver probe

*/
static int d1981_soc_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	socdev->card->codec = d1981_codec;
	d1981_init(socdev);

	return 0;
}

#ifdef D1982_DELAYED_WORK
/*
 * This function forces any delayed work to be queued and run.
 */
static int run_delayed_work(struct delayed_work *dwork)
{
	int ret;

	/* cancel any work waiting to be queued. */
	ret = cancel_delayed_work(dwork);

	/* if there was any work waiting then we run it now and
	 * wait for it's completion */
	if (ret) {
		schedule_delayed_work(dwork, 0);
		flush_scheduled_work();
	}
	return ret;
}
#endif

/*
Driver remove function

*/
static int d1981_soc_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec->control_data)
		d1981_set_bias_level(codec, SND_SOC_BIAS_OFF);

	//run_delayed_work(&codec->delayed_work);
	snd_soc_free_pcms(socdev);
	snd_soc_dapm_free(socdev);

	kfree(codec->drvdata);

	kfree(codec);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_d1981_audio = {
	.probe = 	d1981_soc_probe,
	.remove = 	d1981_soc_remove,
	.suspend =	d1981_soc_suspend,
	.resume = 	d1981_soc_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_d1981_audio);

static int __init d1981_audio_modinit(void)
{
#if 1
	//struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	//struct _setup_data *setup;
	struct snd_soc_codec *codec;
	struct d1981_priv *d1981_audio;
	int ret;

	pr_debug("[audio]-->d1981_soc_probe: start.\n");

	//setup = socdev->codec_data;
	codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

    codec->name = "d1981 audio";
	codec->owner = THIS_MODULE;
	codec->read = d1981_soc_read;
	codec->write = d1981_soc_write;
	codec->dai = d1981_audio_dai;
	codec->num_dai = ARRAY_SIZE(d1981_audio_dai);
	
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	d1981_codec = codec;
	//d1981_socdev = socdev;

	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err;
	}
#ifdef D1982_DELAYED_WORK
	INIT_DELAYED_WORK(&codec->delayed_work, d1981_audio_work);
#endif

	d1981_audio = kzalloc(sizeof(struct d1981_priv), GFP_KERNEL);
	if (d1981_audio == NULL) {
		
		goto err_codec;
	}
    d1981_audio->power=0;
    d1981_audio->classD=0;
    d1981_audio->hookMode=0;
    d1981_audio->working_path=WORKING_NONE;
#ifdef D1981_PCM_RECORDING    
    d1981_audio->playback_samplerate = SNDRV_PCM_RATE_44100;
	d1981_audio->capture_samplerate = SNDRV_PCM_RATE_16000;
#endif	
	codec->drvdata = d1981_audio;
#endif

	pr_debug("[audio]-->d1981_audio_modinit: register d1981 dai.\n");
	ret= snd_soc_register_dais(ARRAY_AND_SIZE(d1981_audio_dai));

	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAIs: %d\n", ret);
		goto err_codec;
	}

	return 0;
	
err_codec:
	snd_soc_unregister_codec(codec);
	
err:
	kfree(codec);
	return ret;
}
module_init(d1981_audio_modinit);

static void __exit d1981_audio_exit(void)
{
	snd_soc_unregister_dais(ARRAY_AND_SIZE(d1981_audio_dai));
	snd_soc_unregister_codec(d1981_codec);
}
module_exit(d1981_audio_exit);

MODULE_DESCRIPTION("ASoC d1981 driver");
MODULE_AUTHOR("Dialog Semiconductor Ltd");
MODULE_LICENSE("GPL");


