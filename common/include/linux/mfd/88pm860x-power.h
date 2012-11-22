#ifndef __PM860X_POWER_H
#define __PM860X_POWER_H

#if 0
#define PM8607_A0_ID  	0x40
#define PM8607_A1_ID  	0x41
#define PM8607_B0_ID  	0x48
#define PM8607_C0_ID  	0x50
#define PM8607_C1D_ID	0x51
#define PM8607_END_ID 	PM8607_B0_ID

#define PM8606_A0_ID 	0x60
#define PM8606_A1_ID 	0x48
#define PM8606_A2_ID 	0x41
#define PM8606_END_ID 	PM8606_A2_ID

/* Item: PM8607_STATUS_2 */
#define PM8607_ONKEY_STATUS            (1 << 0)
#define PM8607_EXTON_STATUS            (1 << 1)
#define PM8607_CHG_STATUS              (1 << 2)
#define PM8607_BAT_STATUS              (1 << 3)
#define PM8607_PEN_STATUS              (1 << 4)
#define PM8607_HEADSET_STATUS          (1 << 5)
#define PM8607_HOOK_STATUS             (1 << 6)
#define PM8607_MICIN_STATUS            (1 << 7)


#define PM8607_CC_READ			0x47
#define CC_AVG_SEL_4                    (0x4 << 3)
#define CC_AVG_SEL_MASK                 (0x7 << 3)

/* Item: PM8607_CHG_CTRL1 */
#define CHARGER_MODE_MASK		(0x3 << 0)
#define CHARGER_MODE_CHARGE_OFF		(0x0 << 0)
#define CHARGER_MODE_PRE_CHARGE		(0x1 << 0)
#define CHARGER_MODE_FAST_CHARGE	(0x2 << 0)
#define ITERM_SET_MASK			(0x3 << 2)
#define ITERM_SET_60MA			(0x2 << 2)
#define ITERM_SET_20MA			(0x0 << 2)
#define VFCHG_SET_MASK			(0xF << 4)
#define VFCHG_SET_4P2V			(0x9 << 4)

/* Item: PM8607_CHG_CTRL2 */
#define ICHG_SET_MASK			(0x1F << 0)
#define ICHG_SET_500MA			(0x9 << 0)
#define ICHG_SET_1000MA			(0x13 << 0)
#define BB_LRSW_EN			(0x1 << 5)
#define BB_PREG_OFF			(0x1 << 6)

/* Item: PM8607_CHG_CTRL3 */
#define CHG_TIMER_SET_DISABLE		(0xF << 4)
#define CHG_TIMER_SET_MASK		(0xF << 4)
#define CHG_TIMER_SET_240MIN		(0x8 << 4)

/* Item: PM8607_CHG_CTRL4 */
#define IPRE_SET_MASK			(0xF << 0)
#define IPRE_SET_40MA			(0x7 << 0)
#define VPCHG_SET_MASK			(0x3 << 4)
#define VPCHG_SET_3P2V			(0x3 << 4)
#define IFCHG_MON_EN			(0x1 << 6)
#define BTEMP_MON_EN			(0x1 << 7)

/* Item: PM8607_CHG_CTRL5 */
#define VCHG_ON_CNT_SEL_MASK		0x3
#define VCHG_ON_CNT_SEL_16SEC		(0x2 << 0)

/* Item: PM8607_CHG_CTRL6 */
#define BD_MSK_MASK			(0x3 << 0)
#define BD_MSK_GPDAC1			(0x01 << 0)
#define BC_OV_VBAT_EN			(1 << 2)
#define BC_UV_VBAT_EN			(1 << 3)

/* Item: PM8607_CHG_CTRL7 */
#define BAT_REM_EN			(1 << 3)
#define ILIM_LONGTMREN			(1 << 6)
#define IFSM_EN				(1 << 7)

#define PM8607_MEAS_ENABLE1		0x50
#define PM8607_MEAS_EN1_VBAT		(1 << 0)
#define PM8607_MEAS_EN1_VCHG		(1 << 1)
#define PM8607_MEAS_EN1_VSYS		(1 << 2)
#define PM8607_MEAS_EN1_TINT		(1 << 3)
#define PM8607_MEAS_EN1_RFTMP		(1 << 4)
#define PM8607_MEAS_EN1_TBAT		(1 << 5)
#define PM8607_MEAS_EN1_GPADC2		(1 << 6)
#define PM8607_MEAS_EN1_GPADC3		(1 << 7)

#define PM8607_MEAS_ENABLE2		0x51

#define PM8607_MEAS_ENABLE3		0x52
#define PM8607_MEAS_EN3_IBAT		(1 << 0)
#define PM8607_MEAS_EN3_IBAT_COMP	(1 << 1)
#define PM8607_MEAS_EN3_BAT_DET_EN_B0	(1 << 1)
#define PM8607_MEAS_EN3_COULOMB_COUNTER (1 << 2)
#define PM8607_MEAS_EN3_PENDET		(1 << 3)
#define PM8607_MEAS_EN3_TSIX		(1 << 4)
#define PM8607_MEAS_EN3_TSIY		(1 << 5)
#define PM8607_MEAS_EN3_TSIZ1		(1 << 6)
#define PM8607_MEAS_EN3_TSIZ2		(1 << 7)

/* Item: PM8607_MEAS_OFF_TIME1 */
#define PM8607_MEASOFFTIME1_DOUBLE_TSI		(1 << 0)
#define PM8607_MEASOFFTIME1_MEAS_EN_SLP		(1 << 1)
#define PM8607_MEASOFFTIME1_MEAS_OFF_TIME1_MASK	(0x3F << 1)

/* Item: PM8607_MEAS_OFF_TIME2 */
#define PM8607_TSI_PREBIAS_TIME		0x55
#define PM8607_PD_PREBIAS_TIME		0x56

#define PM8607_GPADC_MISC1_GPFSM_EN	(1 << 0)
#define PM8607_GPPADC_GP_PREBIAS_TIME	(01 << 1)
#define PM8607_GPADC_MISC1_MASK		(SANREMO_GPADC_MISC1_GPFSM_EN | SANREMO_GPPADC_GP_PREBIAS_TIME)

#define PM8607_SW_CAL			0x58
#define PM8607_GPADC_MISC2		0x59
#define PM8607_GPADC0_GP_BIAS_A0	(1 << 0)
#define PM8607_GPADC1_GP_BIAS_A1	(1 << 1)
#define PM8607_GPADC2_GP_BIAS_A2	(1 << 2)
#define PM8607_GPADC3_GP_BIAS_A3	(1 << 3)

#define PM8607_GP_BIAS2			0x5a
#define GP1_BIAS_SET_MASK		(0xF << 4)
#define GP1_BIAS_SET_15UA		(0x2 << 4)


/*Test registers for FSM debug, Address 0xC8 to 0xCF Page 1*/
#define        PM8607_TEST_PAGE1_C8	0xC8
#define        PM8607_TEST_PAGE1_C9	0xC9
#define        PM8607_TEST_PAGE1_CA	0xCA
#define        PM8607_TEST_PAGE1_CB	0xCB
#define        PM8607_TEST_PAGE1_CC	0xCC
#define        PM8607_TEST_PAGE1_CD	0xCD
#define        PM8607_TEST_PAGE1_CE	0xCE
#define        PM8607_TEST_PAGE1_CF	0xCF
#define        PM8607_TEST_PAGE1_D6	0xD6
#define        PM8607_TEST_PAGE1_D7	0xD7

/*VSYS treshold*/
#define VSYS_LOW_TH			2800
#define VSYS_UPP_TH			5400

/*VBAT treshold*/
/* battery is empty in discharge state*/
#define VBAT_TS_DISCHARGE_DPOINT	3100
#define VBAT_TS_DISCHARGE_LOW		0000
#define VBAT_TS_DISCHARGE_UPP		3150

/* battery is low - limited functionalites*/
#define VBAT_TS_LOWBAT_DPOINT		3450
#define VBAT_TS_LOWBAT_LOW		3000
#define VBAT_TS_LOWBAT_UPP		3500

/* battery is in normal state - user can work */
#define VBAT_TS_NORMAL_DPOINT		4100
#define VBAT_TS_NORMAL_LOW		3400
#define VBAT_TS_NORMAL_UPP		4300

/* battery is in full state - user can work - high preformance */
#define VBAT_TS_FULL_LOW		4000
#define VBAT_TS_FULL_UPP		5400

/*TBAT threshold*/
/* table in 				mV	C*/
#define TBAT_TEC_MIN			1472	/*(-25) */
#define TBAT_TEC_LOW_TH			1125	/*(-20) */
#define TBAT_TEC_DPONIT_TH		962	/*(-17) */
#define TBAT_TEC_UPP_TH			868	/*(-15) */
#define TBAT_C_LOW_TH			423	/*0 */
#define TBAT_C_DPONIT_TH   		386	/*2 */
#define TBAT_C_UPP_TH			338	/*5 */
#define TBAT_H_LOW_TH			87	/*40 */
#define TBAT_H_DPONIT_TH   		81	/*42 */
#define TBAT_H_UPP_TH			73	/*45 */
#define TBAT_EH_LOW_TH			53	/*55 */
#define TBAT_EH_DPONIT_TH  		49	/*57 */
#define TBAT_EH_UPP_TH			46	/*60 */
#define TBAT_EH_MAX			15	/*100 */

/* Item: PM8607_INTERRUPT_STATUS1 */
#define PM8607_ONKEY_INT		(1 << 0)
#define PM8607_EXTON_INT		(1 << 1)
#define PM8607_CHG_INT			(1 << 2)
#define PM8607_BAT_INT			(1 << 3)
#define PM8607_RTC_INT			(1 << 4)

/* Item: PM8607_INTERRUPT_STATUS2 */
#define PM8607_VBAT_INT			(1 << 0)
#define PM8607_VCHG_INT			(1 << 1)
#define PM8607_VSYS_INT			(1 << 2)
#define PM8607_TINT_INT			(1 << 3)
#define PM8607_GPADC0_INT		(1 << 4)
#define PM8607_GPADC1_INT		(1 << 5)
#define PM8607_GPADC2_INT		(1 << 6)
#define PM8607_GPADC3_INT		(1 << 7)

/* Item: PM8607_INTERRUPT_STATUS3 */
#define PM8607_PEN_INT			(1 << 1)
#define PM8607_HEADSET_INT		(1 << 2)
#define PM8607_HOOK_INT			(1 << 3)
#define PM8607_MICIN_INT		(1 << 4)
#define PM8607_CHG_FAIL_INT		(1 << 5)
#define PM8607_CHG_DONE_INT		(1 << 6)
#define PM8607_CHG_IOVER_INT		(1 << 7)

/* Item: PM8607_INTERRUPT_ENABLE1 */
#define PM8607_ONKEY_MASK		(1 << 0)
#define PM8607_EXTON_MASK		(1 << 1)
#define PM8607_CHG_MASK			(1 << 2)
#define PM8607_BAT_MASK			(1 << 3)
#define PM8607_RTC_MASK			(1 << 4)

/* Item: PM8607_INTERRUPT_ENABLE2 */
#define PM8607_VBAT_MASK		(1 << 0)
#define PM8607_VCHG_MASK		(1 << 1)
#define PM8607_VSYS_MASK		(1 << 2)
#define PM8607_TINT_MASK		(1 << 3)
#define PM8607_GPADC0_MASK		(1 << 4)
#define PM8607_GPADC1_MASK		(1 << 5)
#define PM8607_GPADC2_MASK		(1 << 6)
#define PM8607_GPADC3_MASK		(1 << 7)

/* Item: PM8607_INTERRUPT_ENABLE3 */
#define PM8607_PEN_MASK			(1 << 1)
#define PM8607_HEADSET_MASK		(1 << 2)
#define PM8607_HOOK_MASK		(1 << 3)
#define PM8607_MICIN_MASK		(1 << 4)
#define PM8607_CHG_FAIL_MASK		(1 << 5)
#define PM8607_CHG_DONE_MASK		(1 << 6)
#define PM8607_CHG_IOVER_MASK		(1 << 7)


/* Item: PM8606_PREREGULATORA */
#define PM8606_PREREG_VSYS_SET		(0x0 << 4)
#define PM8606_PREREG_CURLIM_SET	(0xe << 0)

/* Define the event */
#define	PMIC_EVENT_INIT			(0)
#define	PMIC_EVENT_EXTON		(1 << 0)
#define	PMIC_EVENT_VBUS			(1 << 1)
#define	PMIC_EVENT_USB			(PMIC_EVENT_EXTON | PMIC_EVENT_VBUS)
#define	PMIC_EVENT_TOUCH		(1 << 2)
#define PMIC_EVENT_OTGCP_IOVER		(1 << 3)
#define	PMIC_EVENT_TBAT			(1 << 4)
#define	PMIC_EVENT_REV_IOVER		(1 << 5)
#define	PMIC_EVENT_IOVER		(1 << 6)
#define	PMIC_EVENT_CHDET		(1 << 7)
#define	PMIC_EVENT_VBATMON		(1 << 8)
#define PMIC_EVENT_HSDETECT		(1 << 9)
#define PMIC_EVENT_HOOKSWITCH		(1 << 10)
#define PMIC_EVENT_ON_KEY		(1 << 11)
#define PMIC_EVENT_MICDETECT 		(1 << 12)
#define PMIC_EVENT_BAT_DET		(1 << 13)
#define PMIC_EVENT_VBAT_TS		(1 << 14)
#define PMIC_EVENT_VCHG_TS		(1 << 15)
#define PMIC_EVENT_VSYS			(1 << 16)
#define PMIC_EVENT_TINT			(1 << 17)
#define PMIC_EVENT_CAHRG_FAIL		(1 << 18)
#define PMIC_EVENT_CHARG_COMP		(1 << 19)
#define PMIC_EVENT_MICIN		(1 << 20)
#define PMIC_EVENT_RF_TEMP		(1 << 21)
#define PMIC_EVENT_ONKEY		(1 << 22)
#define PMIC_EVENT_CC			(1 << 23)
#define	PMIC_EVENT_DEFAULT		(0xFF)
#endif

/*Chg current value*/
#define USB_CHARGER_CURRENT 		ICHG_SET_500MA
#define AC_OTHER_CHARGER_CURRENT	ICHG_SET_700MA
#define AC_STANDARD_CHARGER_CURRENT	ICHG_SET_1000MA

enum enum_result {
	ENUMERATION_START    = 0,
	ENUMERATION_500MA    = 1,
};
enum enum_charger_type {
	USB_CHARGER			= 0,
	AC_STANDARD_CHARGER		= 1,
	AC_OTHER_CHARGER		= 2,
};
void pm860x_set_charger_type(enum enum_charger_type type);

#endif

