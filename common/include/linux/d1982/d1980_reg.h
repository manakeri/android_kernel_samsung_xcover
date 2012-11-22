/*
 * d1980 register declarations.
  *
 * Copyright(c) 2010 Dialog Semiconductor Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __LINUX_D1980_REG_H
#define __LINUX_D1980_REG_H

#define D1980_PAGECON0_REG		        (0)
#define D1980_STATUSA_REG		        (1)
#define D1980_STATUSB_REG		        (2)
#define D1980_STATUSC_REG		        (3)
#define D1980_STATUSD_REG		        (4)
#define D1980_EVENTA_REG		        (5)
#define D1980_EVENTB_REG		        (6)
#define D1980_EVENTC_REG		        (7)
#define D1980_EVENTD_REG		        (8)
#define D1980_FAULTLOG_REG		        (9)
#define D1980_IRQMASKA_REG		        (10)
#define D1980_IRQMASKB_REG		        (11)
#define D1980_IRQMASKC_REG		        (12)
#define D1980_IRQMASKD_REG		        (13)
#define D1980_CONTROLA_REG		        (14)
#define D1980_CONTROLB_REG		        (15)
#define D1980_CONTROLC_REG		        (16)
#define D1980_CONTROLD_REG		        (17)
#define D1980_PDDIS_REG		            (18)
#define D1980_INTERFACE_REG		        (19)
#define D1980_RESET_REG		            (20)

/* GPIO 0-1-2-3-4-5 */
#define D1980_GPIO0001_REG		        (21)
#define D1980_GPIO0203_REG		        (25)
#define D1980_GPIO0405_REG		        (27)

#define D1980_ID01_REG			        (29)
#define D1980_ID23_REG			        (30)
#define D1980_ID45_REG			        (31)
#define D1980_ID67_REG			        (32)
#define D1980_ID89_REG			        (33)
#define D1980_ID1011_REG		        (34)
#define D1980_ID1213_REG		        (35)
#define D1980_ID1415_REG		        (36)
#define D1980_ID1617_REG		        (37)
#define D1980_ID1819_REG		        (38)
#define D1980_ID2021_REG		        (39)
#define D1980_SEQSTATUS_REG		        (40)
#define D1980_SEQA_REG			        (41)
#define D1980_SEQB_REG			        (42)
#define D1980_SEQTIMER_REG		        (43)

/* Regulator Register set 1 */
#define D1980_BUCKA_REG			        (44)
#define D1980_BUCKB_REG			        (45)
#define D1980_BUCK1_REG			        (46)
#define D1980_BUCK2_REG			        (47)
#define D1980_BUCK3_REG			        (48)
#define D1980_BUCK4_REG			        (49)
#define D1980_LDO1_REG			        (50)
#define D1980_LDO2_REG			        (51)
#define D1980_LDO3_REG			        (52)
#define D1980_LDO4_REG			        (53)
#define D1980_LDO5_REG			        (54)
#define D1980_LDO6_REG			        (55)
#define D1980_LDO7_REG			        (56)
#define D1980_LDO8_REG			        (57)
#define D1980_LDO9_REG			        (58)
#define D1980_LDO10_REG			        (59)
#define D1980_LDO12_REG			        (60)
#define D1980_LDO13_REG			        (61)
/* Regulator Register set 1 end */

#define D1980_PULLDOWN_REG_A		    (62)
#define D1980_PULLDOWN_REG_B		    (63)
#define D1980_PULLDOWN_REG_C		    (64)

/* Regulator Register set 2 */
#define D1980_LDO14_REG		            (65)
#define D1980_LDO16_REG		            (66)
#define D1980_LDO17_REG		            (67)
#define D1980_LDO18_REG		            (68)
#define D1980_LDO19_REG		            (69)
/* Regulator Register set 2 end */


#define D1980_SUPPLY_REG		        (70)
#define D1980_WAITCONT_REG		        (71)
#define D1980_ONKEYCONT_REG		        (72)
#define D1980_POWERCONT_REG		        (73)
#define D1980_AUDIO_CONF1_REG		    (74)
#define D1980_AUDIO_CONF2_REG_A		    (75)
#define D1980_AUDIO_CONF2_REG_B         (76)
#define D1980_BBATCONT_REG		        (77)
#define D1980_ADCMAN_REG		        (81)
#define D1980_ADCCONT_REG		        (82)
#define D1980_ADCRESL_REG		        (83)
#define D1980_ADCRESH_REG		        (84)
#define D1980_VBATRES_REG_MSB		    (85)
#define D1980_VBAT_THR_REG		        (86)
#define D1980_TEMPRES_REG_MSB		    (90)
#define D1980_TEMPHIGHTH_REG		    (91)
#define D1980_TEMPLOWTH_REG		        (93)
#define D1980_TOFFSET_REG		        (94)
#define D1980_VREF_REG_MSB		        (95)
#define D1980_ADCINRES_REG_MSB	        (98)
#define D1980_TJUNCRES_REG		        (104)
#define D1980_AUTORES_REG_1		        (105) /* VBAT Result LSB, TEMP Result LSB */
#define D1980_AUTORES_REG_2		        (106) /* VF Result LSB, ADC IN Result LSB */
#define D1980_AUTORES_REG_3		        (107) /* Tjunc Result LSB */


#define D1980_COUNTS_REG		        (111)
#define D1980_COUNTMI_REG		        (112)
#define D1980_COUNTH_REG		        (113)
#define D1980_COUNTD_REG		        (114)
#define D1980_COUNTMO_REG		        (115)
#define D1980_COUNTY_REG		        (116)
#define D1980_ALARMS_REG		        (117)
#define D1980_ALARMMI_REG		        (118)
#define D1980_ALARMH_REG		        (119)
#define D1980_ALARMD_REG		        (120)
#define D1980_ALARMMO_REG		        (121)
#define D1980_ALARMY_REG		        (122)

#define D1980_CHIPID_REG		        (129)
#define D1980_CONFIGID_REG		        (130)
#define D1980_OTPCONT_REG		        (131)
#define D1980_OSCTRIM_REG		        (132)
#define D1980_GPID0_REG		            (133)
#define D1980_GPID1_REG		            (134)
#define D1980_GPID2_REG		            (135)
#define D1980_GPID3_REG		            (136)
#define D1980_GPID4_REG		            (137)
#define D1980_GPID5_REG		            (138)
#define D1980_GPID6_REG		            (139)
#define D1980_GPID7_REG		            (140)
#define D1980_GPID8_REG		            (141)
#define D1980_GPID9_REG		            (142)

#define D1980_PAGE0_REG_START	        (D1980_STATUSA_REG)
#define D1980_PAGE0_REG_END	            (D1980_ALARMY_REG)

#define D1980_PAGE1_REG_START	        (D1980_CHIPID_REG)
#define D1980_PAGE1_REG_END	            (D1980_GPID9_REG)

/************************PAGE CONFIGURATION ***************************/

/* PAGE CONFIGURATION 128 REGISTER */
/* RESERVED */
/*
	#define D1980_PAGECON0_REGPAGE		(1<<7) 
*/

/* PAGE CONFIGURATION 128 REGISTER */
#define D1980_PAGECON128_REGPAGE	    (1<<7)


/************************SYSTEM REGISTER ***************************/

/* STATUS REGISTER A */
#define D1980_STATUSA_NONKEY		    (1<<0)


/* STATUS REGISTER B */
#define D1980_STATUSB_SEQUENCING	    (1<<6)

/* STATUS REGISTER C */
#define D1980_STATUSC_JIG_ON            (1<<4)
#define D1980_STATUSC_TA_LEVEL          (1<<3)
#define D1980_STATUSC_INT_IN		    (1<<2)
#define D1980_STATUSC_GPI1		        (1<<1)
#define D1980_STATUSC_GPI0		        (1<<0)

/* STATUS REGISTER D */
#define D1980_STATUSD_GPI5		        (1<<5)
#define D1980_STATUSD_GPI4		        (1<<4)
#define D1980_STATUSD_GPI3		        (1<<3)
#define D1980_STATUSD_GPI2		        (1<<0)


/* EVENT REGISTER A */
#define D1980_EVENTA_ETICK		        (1<<7)
#define D1980_EVENTA_ESEQRDY		    (1<<6)
#define D1980_EVENTA_EALRAM		        (1<<5)

/* EVENT REGISTER B */
#define D1980_EVENTB_EADCEOM		    (1<<5)
#define D1980_EVENTB_ENONKEY		    (1<<0)

/* EVENT REGISTER C */
#define D1980_EVENTC_ETA                (1<<3)
#define D1980_EVENTC_EAUDIO		        (1<<2)
#define D1980_EVENTC_EGPI1		        (1<<1)
#define D1980_EVENTC_EGPI0		        (1<<0)

/* EVENT REGISTER D */
#define D1980_EVENTC_EGPI5		        (1<<5)
#define D1980_EVENTC_EGPI4		        (1<<4)
#define D1980_EVENTC_EGPI3		        (1<<3)
#define D1980_EVENTC_EGPI2		        (1<<0)


/* FAULT LOG REGISTER */
#define D1980_FAULTLOG_WAITSET		    (1<<7)
#define D1980_FAULTLOG_KEYSHUT		    (1<<5)
#define D1980_FAULTLOG_TEMPOVER		    (1<<3)
#define D1980_FAULTLOG_VDDSTART		    (1<<2)
#define D1980_FAULTLOG_VDDFAULT		    (1<<1)
#define D1980_FAULTLOG_TWDERROR		    (1<<0)

/* IRQ_MASK REGISTER A */
#define D1980_IRQMASKA_MTICK		    (1<<7)
#define D1980_IRQMASKA_MSEQRDY		    (1<<6)
#define D1980_IRQMASKA_MALRAM		    (1<<5)
#define D1980_IRQMASKA_BIT4		        (1<<4)

/* IRQ_MASK REGISTER B */
#define D1980_IRQMASKB_MADCEOM		    (1<<5)
#define D1980_IRQMASKB_BIT4		        (1<<4)
#define D1980_IRQMASKB_MNONKEY		    (1<<0)

/* IRQ_MASK REGISTER C */
#define D1980_IRQMASKC_TA               (1<<3)
#define D1980_IRQMASKC_MAUDIO		    (1<<2)
#define D1980_IRQMASKC_MGPI1		    (1<<1)
#define D1980_IRQMASKC_MGPI0		    (1<<0)

/* IRQ_MASK REGISTER D */
#define D1980_IRQMASKD_MGPI5		    (1<<5)
#define D1980_IRQMASKD_MGPI4		    (1<<4)
#define D1980_IRQMASKD_MGPI3		    (1<<3)
#define D1980_IRQMASKD_MGPI2		    (1<<0)

/* CONTROL REGISTER A */
#define D1980_CONTROLA_GPIV		        (1<<7)
#define D1980_CONTROLA_PMOTYPE		    (1<<5)
#define D1980_CONTROLA_PMOPU		    (1<<4)
#define D1980_CONTROLA_PMIV		        (1<<3)
#define D1980_CONTROLA_PMIFV		    (1<<3)
#define D1980_CONTROLA_PWR1EN		    (1<<2)
#define D1980_CONTROLA_PWREN		    (1<<1)
#define D1980_CONTROLA_SYSEN		    (1<<0)

/* CONTROL REGISTER B */
#define D1980_CONTROLB_SHUTDOWN		    (1<<7)
#define D1980_CONTROLB_DEEPSLEEP	    (1<<6)
#define D1980_CONTROLB_WRITEMODE	    (1<<5)
#define D1980_CONTROLB_I2C1_SPEED	    (1<<4)
#define D1980_CONTROLB_OTPREADEN	    (1<<3)
#define D1980_CONTROLB_AUTOBOOT		    (1<<2)


/* CONTROL REGISTER C */
#define D1980_CONTROLC_DEBOUNCING	    (7<<2)
#define D1980_CONTROLC_PMFBPIN		    (1<<0)

/* CONTROL REGISTER D */
#define D1980_CONTROLD_WATCHDOG		    (1<<7)
#define D1980_CONTROLD_KEEPACTEN	    (1<<3)
#define D1980_CONTROLD_TWDSCALE		    (7<<0)

/* POWER DOWN DISABLE REGISTER */
#define D1980_PDDIS_PMCONTPD		    (1<<7)
#define D1980_PDDIS_OUT32KPD		    (1<<6)
#define D1980_PDDIS_CHGBBATPD		    (1<<5)
#define D1980_PDDIS_HSIFPD		        (1<<3)
#define D1980_PDDIS_PMIFPD		        (1<<2)
#define D1980_PDDIS_GPADCPD		        (1<<1)
#define D1980_PDDIS_GPIOPD		        (1<<0)

/* INTERFACE REGISTER */
#define D1980_INTERFACE_IFBASEADDR	    (7<<5)

/* RESET REGISTER */
#define D1980_RESET_RESETEVENT		    (3<<6)
#define D1980_RESET_RESETTIMER		    (63<<0)


/************************GPIO REGISTERS***************************/

/* GPIO control register for PIN 0 and 1 */
#define D1980_GPIO01_GPIO1MODE		    (1<<7)
#define D1980_GPIO01_GPIO1TYPE		    (1<<6)
#define D1980_GPIO01_GPIO1PIN		    (3<<4)
#define D1980_GPIO01_GPIO0MODE		    (1<<3)
#define D1980_GPIO01_GPIO0TYPE		    (1<<2)
#define D1980_GPIO01_GPIO0PIN		    (3<<0)

/* GPIO control register for PIN 2 and 3 */
#define D1980_GPIO23_GPIO3MODE		    (1<<7)
#define D1980_GPIO23_GPIO3TYPE		    (1<<6)
#define D1980_GPIO23_GPIO3PIN		    (3<<4)
#define D1980_GPIO23_GPIO2MODE		    (1<<3)
#define D1980_GPIO23_GPIO2TYPE		    (1<<2)
#define D1980_GPIO23_GPIO2PIN		    (3<<0)

/* GPIO control register for PIN 4 and 5 */
#define D1980_GPIO45_GPIO5MODE		    (1<<7)
#define D1980_GPIO45_GPIO5TYPE		    (1<<6)
#define D1980_GPIO45_GPIO5PIN		    (3<<4)
#define D1980_GPIO45_GPIO4MODE		    (1<<3)
#define D1980_GPIO45_GPIO4TYPE		    (1<<2)
#define D1980_GPIO45_GPIO4PIN		    (3<<0)




/*****************POWER SEQUENCER REGISTER*********************/

/* SEQ control register for ID 0 and 1 */
#define D1980_ID01_LDO1STEP		        (15<<4)
#define D1980_ID01_WAITIDALWAYS		    (1<<2)
#define D1980_ID01_SYSPRE		        (1<<2)
#define D1980_ID01_DEFSUPPLY		    (1<<1)
#define D1980_ID01_nRESMODE             (1<<0)

/* SEQ control register for ID 2 and 3 */
#define D1980_ID23_LDO3STEP             (15<<4)
#define D1980_ID23_LDO2STEP             (15<<0)

/* SEQ control register for ID 4 and 5 */
#define D1980_ID45_LDO5STEP	            (15<<4)
#define D1980_ID45_LDO4STEP             (15<<0)

/* SEQ control register for ID 6 and 7 */
#define D1980_ID67_LDO7STEP             (15<<4)
#define D1980_ID67_LDO6STEP             (15<<0)

/* SEQ control register for ID 8 and 9 */
#define D1980_ID89_LDO9STEP             (15<<4)
#define D1980_ID89_LDO8STEP             (15<<0)

/* SEQ control register for ID 10 and 11 */
#define D1980_ID1011_PDDISSTEP          (15<<4)
#define D1980_ID1011_LDO10STEP          (15<<0)

/* SEQ control register for ID 12 and 13 */
#define D1980_ID1213_LDO13STEP          (15<<4)
#define D1980_ID1213_LDO12STEP          (15<<0)

/* SEQ control register for ID 14 and 15 */
#define D1980_ID1415_BUCK2              (15<<4)
#define D1980_ID1415_BUCK1              (15<<0)

/* SEQ control register for ID 16 and 17 */
#define D1980_ID1617_BUCK4              (15<<4)
#define D1980_ID1617_BUCK3              (15<<0)

/* Power SEQ Status register */
#define D1980_SEQSTATUS_SEQPOINTER      (15<<4)
#define D1980_SEQSTATUS_WAITSTEP        (15<<0)

/* Power SEQ A register */
#define D1980_SEQA_POWEREND             (15<<4)
#define D1980_SEQA_SYSTEMEND            (15<<0)

/* Power SEQ B register */
#define D1980_SEQB_PARTDOWN             (15<<4)
#define D1980_SEQB_MAXCOUNT             (15<<0)

/* Power SEQ TIMER register */
#define D1980_SEQTIMER_SEQDUMMY         (15<<4)
#define D1980_SEQTIMER_SEQTIME          (15<<0)


/***************** REGULATOR REGISTER*********************/

/* BUCK REGISTER A */
#define D1980_BUCKA_BUCK2ILIM           (3<<6)
#define D1980_BUCKA_BUCK2MODE           (3<<4)
#define D1980_BUCKA_BUCK1ILIM           (3<<2)
#define D1980_BUCKA_BUCK1MODE           (3<<0)

/* BUCK REGISTER B */
#define D1980_BUCKB_BUCK4ILIM           (3<<6)
#define D1980_BUCKB_BUCK4IMODE          (3<<4)
#define D1980_BUCKB_BUCK3ILIM           (3<<2)
#define D1980_BUCKB_BUCK3MODE           (3<<0)

/* SUPPLY REGISTER */
#define D1980_SUPPLY_VLOCK              (1<<7)
#define D1980_SUPPLY_LDO15BEN           (1<<6)
#define D1980_SUPPLY_LDO15AEN           (1<<5)
#define D1980_SUPPLY_BBCHGEN            (1<<4)
#define D1980_SUPPLY_VBUCK4GO           (1<<3)
#define D1980_SUPPLY_VBUCK3GO           (1<<2)
#define D1980_SUPPLY_VBUCK2GO           (1<<1)
#define D1980_SUPPLY_VBUCK1GO           (1<<0)

/* PULLDOWN REGISTER A */
#define D1980_PULLDOWN_A_LDO4PDDIS	    (1<<7)
#define D1980_PULLDOWN_A_LDO3PDDIS	    (1<<6)
#define D1980_PULLDOWN_A_LDO2PDDIS	    (1<<5)
#define D1980_PULLDOWN_A_LDO1PDDIS	    (1<<4)
#define D1980_PULLDOWN_A_BUCK4PDDIS	    (1<<3)
#define D1980_PULLDOWN_A_BUCK3PDDIS	    (1<<2)
#define D1980_PULLDOWN_A_BUCK2PDDIS	    (1<<1)
#define D1980_PULLDOWN_A_BUCK1PDDIS	    (1<<0)


/* PULLDOWN REGISTER B */
#define D1980_PULLDOWN_B_LDO13PDDIS	    (1<<7)
#define D1980_PULLDOWN_B_LDO12PDDIS	    (1<<6)
#define D1980_PULLDOWN_B_LDO10PDDIS	    (1<<5)
#define D1980_PULLDOWN_B_LDO9PDDIS	    (1<<4)
#define D1980_PULLDOWN_B_LDO8PDDIS	    (1<<3)
#define D1980_PULLDOWN_B_LDO7PDDIS	    (1<<2)
#define D1980_PULLDOWN_B_LDO6PDDIS	    (1<<1)
#define D1980_PULLDOWN_B_LDO5PDDIS	    (1<<0)



/* PULLDOWN REGISTER C */
#define D1980_PULLDOWN_C_LDO19PDDIS	    (1<<6)
#define D1980_PULLDOWN_C_LDO18PDDIS	    (1<<5)
#define D1980_PULLDOWN_C_LDO17PDDIS	    (1<<4)
#define D1980_PULLDOWN_C_LDO16PDDIS	    (1<<3)
#define D1980_PULLDOWN_C_LDO15BPDDIS    (1<<2)
#define D1980_PULLDOWN_C_LDO15APDDIS    (1<<1)
#define D1980_PULLDOWN_C_LDO14PDDIS	    (1<<0)


/* WAIT CONTROL REGISTER */
#define D1980_WAITCONT_WAITDIR		    (1<<7)
#define D1980_WAITCONT_RTCCLOCK		    (1<<6)
#define D1980_WAITCONT_WAITMODE		    (1<<5)
#define D1980_WAITCONT_EN32KOUT		    (1<<4)
#define D1980_WAITCONT_DELAYTIME	    (15<<0)

/* ONKEY CONTROL REGISTER */
#define D1980_ONKEYCONT_PRESSTIME	    (15<<0)

/* POWER CONTROL REGISTER */
#define D1980_POWERCONT_nSLEEP		    (1<<0)


/***************** BAT CHARGER REGISTER *********************/

/* BACKUP BATTERY CONTROL REGISTER */
#define D1980_BBATCONT_BCHARGERISET	    (15<<4)
#define D1980_BBATCONT_BCHARGERVSET	    (15<<0)

/***************** AUDIO CONF2 REGISTERS********************/
#define D1980_AUDIOCONF_CLASSDEN	    (1<<0)

/***************** ADC REGISTERS********************/

/* ADC MAN registers */
#define D1980_ADCMAN_MANCONV            (1<<4)
#define D1980_ADCMAN_MUXSEL_MASK	    (0xFF<<0)
#define D1980_ADCMAN_MUXSEL_VBAT	    (0x0<<0)
#define D1980_ADCMAN_MUXSEL_TEMP	    (0x2<<0)
#define D1980_ADCMAN_MUXSEL_VF		    (0x4<<0)
#define D1980_ADCMAN_MUXSEL_ADCIN	    (0x5<<0)
#define D1980_ADCMAN_MUXSEL_TJUNC	    (0x8<<0)
#define D1980_ADCMAN_MUXSEL_VBBAT	    (0x9<<0)

/* ADC CONTROL regsisters */
#define D1980_ADCCONT_AUTOADCEN		    (1<<7)
#define D1980_ADCCONT_ADCMODE		    (1<<6)
#define D1980_ADCCONT_TEMPISRCEN	    (1<<5)
#define D1980_ADCCONT_VFISRCEN		    (1<<4)
#define D1980_ADCCONT_AUTOAINEN		    (1<<2)
#define D1980_ADCCONT_AUTOVFEN		    (1<<1)
#define D1980_ADCCONT_AUTOVBATEN	    (1<<0)

/* ADC 12 BIT MANUAL CONVERSION RESULT LSB register */
#define D1980_ADCRESL_ADCRESLSB			(15<<0)
#define D1980_ADCRESL_ADCRESLSB_MASK    D1980_ADCRESL_ADCRESLSB

/* ADC 12 BIT MANUAL CONVERSION RESULT MSB register */
#define D1980_ADCRESH_ADCRESMSB			(255<<0)
#define D1980_ADCRESH_ADCRESMSB_MASK	D1980_ADCRESH_ADCRESMSB

/* VBAT 10 BIT RES MSB regsister*/
#define D1980_VBATRES_VBATRESMSB		(255<<0)
#define D1980_VBATRES_VBATRESMSB_MASK	D1980_VBATRES_VBATRESMSB

/* VBAT 10 BIT RES LSB regsister*/
#define D1980_VBATRES_VBATRESLSB		(3<<2)
#define D1980_VBATRES_VBATRESLSB_MASK	D1980_VBATRES_VBATRESLSB

/* TEMP 10 BIT RES MSB regsister*/
#define D1980_TEMPRES_TEMPRESMSB		(255<<0)
#define D1980_TEMPRES_TEMPRESMSB_MASK	D1980_TEMPRES_TEMPRESMSB

/* TEMP 10 BIT RES LSB regsister*/
#define D1980_TEMPRES_TEMPRESLSB		(3<<6)
#define D1980_TEMPRES_TEMPRESLSB_MASK	D1980_TEMPRES_TEMPRESLSB
                                        
/* T_OFFSET regsister*/
#define D1980_TOFFSET_TOFFSET			(255<<0)

/* VF 10 BIT RES MSB regsister*/
#define D1980_VFRES_VFRESMSB			(255<<0)
#define D1980_VFRES_VFRESMSB_MASK		D1980_VFRES_VFRESMSB

/* VF 10 BIT RES LSB regsister*/
#define D1980_VFRES_VFRESLSB			(3<<2)
#define D1980_VFRES_VFRESLSB_MASK		D1980_VFRES_VFRESLSB

/* ADCIN 10 BIT RES MSB regsister*/
#define D1980_ADCINRES_ADCINRESMSB		(255<<0)
#define D1980_ADCINRES_ADCINRESMSB_MASK	D1980_ADCINRES_ADCINRESMSB

/* ADCIN 10 BIT RES LSB regsister*/
#define D1980_ADCINRES_ADCINRESLSB		(3<<6)
#define D1980_ADCINRES_ADCINRESLSB_MASK	D1980_ADCINRES_ADCINRESLSB

/* TJUNC 10 BIT RES MSB regsister*/
#define D1980_TJUNCRES_TJUNCRESMSB		(255<<0)
#define D1980_TJUNCRES_TJUNCRESMSB_MASK	D1980_TJUNCRES_TJUNCRESMSB

/* ADCIN 10 BIT RES LSB regsister*/
#define D1980_TJUNCRES_TJUNCRESLSB		(3<<6)
#define D1980_TJUNCRES_TJUNCRESLSB_MASK	D1980_TJUNCRES_TJUNCRESLSB


/*****************RTC REGISTER*********************/

/* RTC TIMER SECONDS REGISTER */
#define D1980_COUNTS_COUNTSEC		    (63<<0)

/* RTC TIMER MINUTES REGISTER */
#define D1980_COUNTMI_COUNTMIN		    (63<<0)

/* RTC TIMER HOUR REGISTER */
#define D1980_COUNTH_COUNTHOUR		    (31<<0)

/* RTC TIMER DAYS REGISTER */
#define D1980_COUNTD_COUNTDAY		    (31<<0)

/* RTC TIMER MONTHS REGISTER */
#define D1980_COUNTMO_COUNTMONTH	    (15<<0)

/* RTC TIMER YEARS REGISTER */
#define D1980_COUNTY_MONITOR		    (1<<6)
#define D1980_COUNTY_COUNTYEAR		    (63<<0)

/* RTC ALARM SECONDS REGISTER */
#define D1980_ALARMMI_COUNTSEC		    (63<<0)

/* RTC ALARM MINUTES REGISTER */
#define D1980_ALARMMI_TICKTYPE		    (1<<7)
#define D1980_ALARMMI_ALARMMIN		    (63<<0)

/* RTC ALARM HOURS REGISTER */
#define D1980_ALARMH_ALARMHOUR		    (31<<0)

/* RTC ALARM DAYS REGISTER */
#define D1980_ALARMD_ALARMDAY		    (31<<0)

/* RTC ALARM MONTHS REGISTER */
#define D1980_ALARMMO_ALARMMONTH	    (15<<0)

/* RTC ALARM YEARS REGISTER */
#define D1980_ALARMY_TICKON		        (1<<7)
#define D1980_ALARMY_ALARMON		    (1<<6)
#define D1980_ALARMY_ALARMYEAR		    (63<<0)


/*****************OTP REGISTER*********************/
/* CHIP IDENTIFICATION REGISTER */
#define D1980_CHIPID_MRC		        (15<<4)
#define D1980_CHIPID_TRC		        (15<<0)

/* CONFIGURATION IDENTIFICATION REGISTER */
#define D1980_CONFIGID_CONFID		    (7<<0)

/* OTP CONTROL REGISTER */
#define D1980_OTPCONT_GPWRITEDIS	    (1<<7)
#define D1980_OTPCONT_OTPCONFLOCK	    (1<<6)
#define D1980_OTPCONT_OTPGPLOCK		    (1<<5)
#define D1980_OTPCONT_OTPCONFG		    (1<<3)
#define D1980_OTPCONT_OTPGP		        (1<<2)
#define D1980_OTPCONT_OTPRP		        (1<<1)
#define D1980_OTPCONT_OTPTRANSFER	    (1<<0)

/* RTC OSCILLATOR TRIM REGISTER */
#define D1980_OSCTRIM_TRIM32K		    (255<<0)

/* GP ID REGISTER 0 */
#define D1980_GPID0_GP0		            (255<<0)

/* GP ID REGISTER 1 */
#define D1980_GPID1_GP1		            (255<<0)

/* GP ID REGISTER 2 */
#define D1980_GPID2_GP2		            (255<<0)

/* GP ID REGISTER 3 */
#define D1980_GPID3_GP3		            (255<<0)

/* GP ID REGISTER 4 */
#define D1980_GPID4_GP4		            (255<<0)

/* GP ID REGISTER 5 */
#define D1980_GPID5_GP5		            (255<<0)

/* GP ID REGISTER 6 */
#define D1980_GPID6_GP6		            (255<<0)

/* GP ID REGISTER 7 */
#define D1980_GPID7_GP7		            (255<<0)

/* GP ID REGISTER 8 */
#define D1980_GPID8_GP8		            (255<<0)

/* GP ID REGISTER 9 */
#define D1980_GPID9_GP9		            (255<<0)

#endif /* __LINUX_D1980_REG_H */
