#ifndef	__PXA3XX_USB_OTG_CONTROLLER__
#define	__PXA3XX_USB_OTG_CONTROLLER__

#include <linux/types.h>

enum otg_function {
	OTG_B_DEVICE = 0,
	OTG_A_DEVICE
};

/* OTG working mode */
#define	USB_OTG_OFF		0
#define	USB_NON_OTG_CLIENT_SEP	1
#define	USB_NON_OTG_HOST_SEP	2
#define	USB_EXT_OTG_CLIENT_SEP	3
#define	USB_EXT_OTG_HOST_SEP	4
#define	USB_NON_OTG_HOST_SEP_CLIENT_DP	5
#define	USB_NON_OTG_CLIENT_SEP_HOST_DP	6
#define	USB_INT_OTG_HOST_DP	7
#define	USB_INT_OTG_CLIENT_DP	8
#define	USB_NON_OTG_CLIENT_DP	9
#define	USB_NON_OTG_HOST_DP	10
#define USB_OTG_LP		11
#define USB_OTG_CARKIT		12
#define USB_OTG_PRE_SYNCH	13

/* UDC Handle Structure */
struct pxa_otgc {
	/* internal variables*/
	u8 a_set_b_hnp_en;	   /* A-Device set b_hnp_en */

	/* OTG inputs*/
	u8 a_bus_drop;
	u8 a_bus_req;
	u8 a_bus_resume;
	u8 a_bus_suspend;
	u8 a_conn;
	u8 a_sess_vld;
	u8 a_srp_det;
	u8 a_vbus_vld;
	u8 a_clr_err;
	u8 b_bus_req;		   /* B-Device Require Bus */
	u8 b_bus_resume;
	u8 b_bus_suspend;
	u8 b_conn;
	u8 b_se0_srp;
	u8 b_sess_end;
	u8 b_sess_vld;
	u8 a_suspend_req;

	/*Timer event*/
	u8 a_aidl_bdis_timeout;
	u8 b_ase0_brst_timeout;
	u8 a_bidl_adis_timeout;
	u8 a_wait_bcon_timeout;
	u8 a_wait_vfall_timeout;

	/* Timer identifiers */
	int a_wait_vrise_tmr;	    /* Identifier of timer */
	int a_wait_vfall_tmr;
	int a_wait_bcon_tmr;
	int a_aidl_bdis_tmr;
	int b_ase0_brst_tmr;
	int b_srp_fail_tmr;
	int b_se0_srp_tmr;
	int a_srp_rspns_tmr;
	int a_bidl_adis_tmr;
	int a_drv_rsm_tmr;
};

struct otg_pmic_ops {
	int (*otg_vbus_init)(void);
	int (*otg_set_vbus)(int vbus_type);
	int (*otg_set_vbus_ic)(int function);
	int (*otg_get_vbus_state)(unsigned base);
};
struct pxa_otg;

struct otg_ctrl_ops {
	int (*otgc_init)(void*);
	void (*otgc_interrupt_init)(void);
	int (*otgc_interrupt_handle)(struct pxa_otg *pOtgHandle);
	void (*otgc_init_gadget)(void);
	void (*otgc_deinit)(void);
};

struct otg_xceiv_ops {
	int (*otgx_get_mode)(void);
	int (*otgx_set_mode)(int mode);
	void (*otgx_init)(void);
	enum otg_function (*otgx_detect_default_func)(void);
	int (*otgx_dp_session)(void);
	int (*otgx_vbus_session)(struct pxa_otg *pOtgHandle);
	int (*otgx_check_b_hnp)(void);
	int (*otgx_check_vbus)(void);
	int (*otgx_start_autoresume)(void);
	int (*otgx_drive_resume)(void);
	void (*reset_xcvr_init)(void);
	void (*ulpi_dat3_work)(void);

};

struct pxa_otg_plat_info {
	struct otg_pmic_ops *pmic_ops;
	struct otg_ctrl_ops *ctrl_ops;
	struct otg_xceiv_ops *xceiv_ops;
};

struct pxa_otg {
	struct otg_transceiver otg;
	struct usb_device       *udev;

	unsigned working : 1;

	struct work_struct work;
	unsigned long todo;
#define WORK_UPDATE_OTG  0                      /* update OTG state machine */
#define WORK_HOST_RESUME 1                      /* resume host */
#define WORK_TIMER       2                      /* timer fired */
#define WORK_STOP        3                      /* don't resubmit */
	struct pxa_otgc *otg_ctrl;
	struct otg_pmic_ops *pmic_ops;
	struct otg_ctrl_ops *ctrl_ops;
	struct otg_xceiv_ops *xceiv_ops;
	struct pxa_otg_plat_info *info;
};

/* OTG software error code */
/* #define	OTG_SUCCESS	0 */
#define	OTG_INVALID_PARAMETER	-1
#define	OTG_UDC_DISABLED	-2
#define	OTG_WRONG_STATE		-3
#define	OTG_WRONG_RESISTER	-4
#define	OTG_I2C_ERROR		-5
#define	OTG_MFP_FAILED		-6
#define	OTG_TIMER_FAILED	-7

/* Timer's interval, unit 10ms */
#define	T_A_WAIT_VRISE		100
#define T_A_WAIT_BCON		2000
#define T_A_AIDL_BDIS		100
#define T_A_BIDL_ADIS		20
#define T_SSEND_LKG		100
/* In order to debug, we use a longer timer */
/* #define T_B_ASE0_BRST		400 */
#define T_B_ASE0_BRST		400
#define T_B_SE0_SRP		300
#define T_B_SRP_FAIL		2000
#define T_B_DATA_PLS		10
#define T_B_SRP_INIT		100
#define T_A_SRP_RSPNS		10
#define T_A_DRV_RSM		5

/* USBOTG EVENTS for require bus */
#define USBOTG_VBUS_VALID      (1 << 0)
#define USBOTG_SRP_DETECT      (1 << 1)

#define USBOTG_STATUS_REQUIRED (USBOTG_VBUS_VALID | USBOTG_SRP_DETECT)

#define PXA3xx_OTG_VBUS_PMIC

extern void pxa3xx_otg_require_bus(int require);

#endif

