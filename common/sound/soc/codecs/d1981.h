/*
 * d1981.h  --  audio driver for d1981
 *
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef _D1981_H
#define _D1981_H

struct d1981_setup_data {
   int i2c_bus;
   unsigned short i2c_address;
};

#define D1981_EH_SUPPORT

#define D1981_DAI_IIS 0

#define	USE_CASE_UNDEFINED	0	/*   */
#define	USE_CASE_1A		11	/*  In Call Voice Mode */
#define	USE_CASE_1B		12	/*  In Call Voice Mode - HS */
#define	USE_CASE_1C		13	/*  In Call Voice Mode - Loud */
#define	USE_CASE_2A		21	/*  Out of Call Voice Record */
#define	USE_CASE_2B		22	/*  Out of Call Voice Record - HS */
#define	USE_CASE_2C		23	/*  Out of Call Voice Record - HS */
#define	USE_CASE_3A		31	/*  Music Playback I2S input (Including voice playback) */
#define	USE_CASE_3B		32	/*  Music Playback ADC input (FM playback) */
#define	USE_CASE_3C		33   /*  Music Playback I2S input to Reciever */
#define	USE_CASE_4A		41
#define	USE_CASE_5A		51
#define	USE_CASE_3AB	34

#define	USE_CASE_6A		61	/*  In VT Mode */
#define	USE_CASE_6B		62	/*  In VT - HS */
#define	USE_CASE_6C		63	/*  In VT - Loud */

#define	USE_CASE_7A		71	/*  In VOIP Mode */
#define	USE_CASE_7B		72	/*  In VOIP - HS */
#define	USE_CASE_7C		73	/*  In VOIP - Loud */
//extern struct snd_soc_dai d1981_audio_dai[];
//extern struct snd_soc_codec_device soc_codec_dev_d1981_audio;

#define D1981_MAX_GAINS_PER_PATH	2

#define MAX_D1981_PATH 20
/* supported paths */
enum {
	//MIC to I2S/PCM 1(AP) output
	D1981_AnaMIC1_TO_I2S_1	  = 0, //from mic1 to ap at recording
	D1981_AnaMIC1_LOUD_TO_I2S_1 = 1, //from loud mic1 to ap at recording
	D1981_AnaMIC2_TO_I2S_1	  = 2, //from headset mic1 to ap at recording
	D1981_AUX1_TO_I2S_1       = 3, 

	//MIC to I2S/PCM 2(CP) output
	D1981_AnaMIC1_TO_PCM_2      = 4, //from mic1 to cp at voice call
	D1981_AnaMIC1_LOUD_TO_PCM_2 = 5, //loud speaker to cp at voice call
	D1981_AnaMIC2_TO_PCM_2     = 6, //from headset mic to cp at voice call
	D1981_AUX1_TO_PCM_2         = 7,

	// From I2S/PCM 1 VIA DAC
	D1981_I2S_1_TO_HS_VIA_DAC12      = 8, // from ap to headset at playback
	D1981_I2S_1_TO_SPKR_VIA_DAC3     = 9, // frome ap to loud speaker at playback
	D1981_I2S_1_TO_RCV_VIA_DAC4      = 10, //frome ap to reciever at playback
	D1981_I2S_1_TO_LINEOUT_VIA_DAC4      = 11,
	
	// From I2S/PCM 2 VIA DAC
	D1981_PCM_2_TO_HS_VIA_DAC12      = 12, // from cp to headset at voice call
	D1981_PCM_2_TO_SPKR_VIA_DAC3     = 13, // frome cp to loud speaker at voice call
	D1981_PCM_2_TO_RCV_VIA_DAC4      = 14, // from cp to reciever at voice call
	D1981_PCM_2_TO_LINEOUT_VIA_DAC4      = 15,

	// From AUX VIA DAC
       D1981_AUX1_TO_HS_VIA_DAC12       = 16, //from fm to headset for FM radio
	D1981_AUX1_SPKR_VIA_DAC3= 17, //from fm to loud speaker for FM radio
	D1981_AUX1_TO_RCV_VIA_DAC4     = 18, //from fm to reciever fro FM radio
	D1981_AUX1_TO_LINEOUT_VIA_DAC4          = 19,
	
	// Side Tone
	D1981_IN_L_TO_OUT           = 20,
	D1981_IN_R_TO_OUT           = 21,
	D1981_IN_BOTH_TO_OUT        = 22,

	D1981_LOOPBACK_MIC1_TO_HS_VIA_DAC12         =23,
    D1981_LOOPBACK_MIC1_TO_SPKR_VIA_DAC3         =24,
    D1981_LOOPBACK_MIC1_TO_RCV_VIA_DAC4         =25,
    D1981_LOOPBACK_MIC2_TO_HS_VIA_DAC12         =26,
    D1981_LOOPBACK_MIC2_TO_SPKR_VIA_DAC3         =27,
    D1981_LOOPBACK_MIC2_TO_RCV_VIA_DAC4         =28,
    D1981_I2S_1_TO_SPKR_HS         =29,


	//VT 
	D1981_AnaMIC1_TO_PCM_2_VT      = 30, //from mic1 to cp at voice call
	D1981_AnaMIC1_LOUD_TO_PCM_2_VT = 31, //loud speaker to cp at voice call
	D1981_AnaMIC2_TO_PCM_2_VT     = 32, //from headset mic to cp at voice call

	// From I2S/PCM 2 VIA DAC
	D1981_PCM_2_TO_HS_VIA_DAC12_VT      = 33, // from cp to headset at voice call
	D1981_PCM_2_TO_SPKR_VIA_DAC3_VT     = 34, // frome cp to loud speaker at voice call
	D1981_PCM_2_TO_RCV_VIA_DAC4_VT      = 35, // from cp to reciever at voice call

    //Voice Recording PCM
    D1981_AnaMIC1_TO_PCM_2_APPS      = 36, //from mic1 to cp at voice recording
	D1981_AnaMIC1_LOUD_TO_PCM_2_APPS = 37, //loud speaker to cp at voice recording
	D1981_AnaMIC2_TO_PCM_2_APPS     = 38, //from headset mic to cp at voice recording

 //VOIP
	D1981_AnaMIC1_TO_PCM_2_VOIP   = 39,//from mic1 to cp at VOIP
	D1981_AnaMIC1_LOUD_TO_PCM_2_VOIP = 40,//loud speaker to cp at VOIP
	D1981_AnaMIC2_TO_PCM_2_VOIP     = 41, //  from headset mic to cp at OIP

	// From I2S/PCM 2 VIA DAC
	D1981_PCM_2_TO_HS_VIA_DAC12_VOIP      = 42, // from cp to headset at VOIP
	D1981_PCM_2_TO_SPKR_VIA_DAC3_VOIP     = 43, // frome cp to loud speaker at VOIP
	D1981_PCM_2_TO_RCV_VIA_DAC4_VOIP      = 44, // from cp to reciever at VOIP

 //LOOPBACK
	D1981_AnaMIC1_TO_PCM_2_LB   = 45,//from mic1 to cp at VOIP
	D1981_AnaMIC1_LOUD_TO_PCM_2_LB = 46,//loud speaker to cp at VOIP
	D1981_AnaMIC2_TO_PCM_2_LB     = 47, //  from headset mic to cp at OIP

	// From I2S/PCM 2 VIA DAC
	D1981_PCM_2_TO_HS_VIA_DAC12_LB      = 48, // from cp to headset at VOIP
	D1981_PCM_2_TO_SPKR_VIA_DAC3_LB     = 49, // frome cp to loud speaker at VOIP
	D1981_PCM_2_TO_RCV_VIA_DAC4_LB      = 50, // from cp to reciever at VOIP
	
	D1981_TOTAL_VALID_PATHS,  				/* Must the be here - counting path amount */
	D1981_DUMMY_PATH,  						/* for disabling purposes */

	D1981_PATHS_ENUM_32_BIT = 0xFF /* 32bit enum compiling enforcement */
};
#if 1
/* paths requiring Audio Interface 1 (I2S_1) */
#define D1981_AIF1_PATHS	{ \
								D1981_AnaMIC1_TO_I2S_1,\
								D1981_AnaMIC1_LOUD_TO_I2S_1,\
								D1981_AnaMIC2_TO_I2S_1,\
								D1981_AUX1_TO_I2S_1,\
								\
								D1981_I2S_1_TO_HS_VIA_DAC12,\
								D1981_I2S_1_TO_SPKR_VIA_DAC3,\
								D1981_I2S_1_TO_RCV_VIA_DAC4,\
								D1981_I2S_1_TO_LINEOUT_VIA_DAC4,\
								\
								D1981_I2S_1_TO_SPKR_HS,\
                                \
								D1981_AnaMIC1_TO_PCM_2,\
								D1981_AnaMIC1_LOUD_TO_PCM_2,\
								D1981_AnaMIC2_TO_PCM_2,\
								D1981_AUX1_TO_PCM_2,\
								\
								D1981_PCM_2_TO_HS_VIA_DAC12,\
								D1981_PCM_2_TO_SPKR_VIA_DAC3,\
								D1981_PCM_2_TO_RCV_VIA_DAC4,\
								D1981_PCM_2_TO_LINEOUT_VIA_DAC4,\
								\
								D1981_AnaMIC1_TO_PCM_2_VT,\
								D1981_AnaMIC1_LOUD_TO_PCM_2_VT,\
								D1981_AnaMIC2_TO_PCM_2_VT,\
								\
								D1981_PCM_2_TO_HS_VIA_DAC12_VT,\
								D1981_PCM_2_TO_SPKR_VIA_DAC3_VT,\
								D1981_PCM_2_TO_RCV_VIA_DAC4_VT,\
								\
								D1981_AUX1_TO_HS_VIA_DAC12 , \
	                            D1981_AUX1_SPKR_VIA_DAC3, \
	                            D1981_AUX1_TO_RCV_VIA_DAC4, \
	                            \
								D1981_AnaMIC1_TO_PCM_2_APPS,\
								D1981_AnaMIC1_LOUD_TO_PCM_2_APPS,\
								D1981_AnaMIC2_TO_PCM_2_APPS,\
								\
								D1981_AnaMIC1_TO_PCM_2_VOIP,\
								D1981_AnaMIC1_LOUD_TO_PCM_2_VOIP,\
								D1981_AnaMIC2_TO_PCM_2_VOIP,\
								\
								D1981_PCM_2_TO_HS_VIA_DAC12_VOIP,\
								D1981_PCM_2_TO_SPKR_VIA_DAC3_VOIP,\
								D1981_PCM_2_TO_RCV_VIA_DAC4_VOIP,\
								\
								D1981_AnaMIC1_TO_PCM_2_LB,\
								D1981_AnaMIC1_LOUD_TO_PCM_2_LB,\
								D1981_AnaMIC2_TO_PCM_2_LB,\
								\
								D1981_PCM_2_TO_HS_VIA_DAC12_LB,\
								D1981_PCM_2_TO_SPKR_VIA_DAC3_LB,\
								D1981_PCM_2_TO_RCV_VIA_DAC4_LB \
							}

/* paths requiring Audio Interface 2 (PCM_2) */
#define D1981_AIF2_PATHS	{ \
								D1981_AnaMIC1_TO_PCM_2,\
								D1981_AnaMIC1_LOUD_TO_PCM_2,\
								D1981_AnaMIC2_TO_PCM_2,\
								D1981_AUX1_TO_PCM_2,\
								\
								D1981_PCM_2_TO_HS_VIA_DAC12,\
								D1981_PCM_2_TO_SPKR_VIA_DAC3,\
								D1981_PCM_2_TO_RCV_VIA_DAC4,\
								D1981_PCM_2_TO_LINEOUT_VIA_DAC4,\
								\
								D1981_AnaMIC1_TO_PCM_2_VT,\
								D1981_AnaMIC1_LOUD_TO_PCM_2_VT,\
								D1981_AnaMIC2_TO_PCM_2_VT,\
								\
								D1981_PCM_2_TO_HS_VIA_DAC12_VT,\
								D1981_PCM_2_TO_SPKR_VIA_DAC3_VT,\
								D1981_PCM_2_TO_RCV_VIA_DAC4_VT,\
								\
								D1981_AnaMIC1_TO_PCM_2_APPS,\
								D1981_AnaMIC1_LOUD_TO_PCM_2_APPS,\
								D1981_AnaMIC2_TO_PCM_2_APPS,\
								\
								D1981_AnaMIC1_TO_PCM_2_VOIP,\
								D1981_AnaMIC1_LOUD_TO_PCM_2_VOIP,\
								D1981_AnaMIC2_TO_PCM_2_VOIP,\
								\
								D1981_PCM_2_TO_HS_VIA_DAC12_VOIP,\
								D1981_PCM_2_TO_SPKR_VIA_DAC3_VOIP,\
								D1981_PCM_2_TO_RCV_VIA_DAC4_VOIP,\
								\
								D1981_AnaMIC1_TO_PCM_2_LB,\
								D1981_AnaMIC1_LOUD_TO_PCM_2_LB,\
								D1981_AnaMIC2_TO_PCM_2_LB,\
								\
								D1981_PCM_2_TO_HS_VIA_DAC12_LB,\
								D1981_PCM_2_TO_SPKR_VIA_DAC3_LB,\
								D1981_PCM_2_TO_RCV_VIA_DAC4_LB \
							}

#endif
#define D1981_MAX_OP_CONTROL 43+15

extern unsigned char d1981_codec_gain[D1981_TOTAL_VALID_PATHS][D1981_MAX_GAINS_PER_PATH];

typedef unsigned char ACM_AudioVolume;
typedef unsigned char ACM_AudioMute;
typedef char ACM_DigitalGain;
typedef char ACM_AnalogGain;

#endif

