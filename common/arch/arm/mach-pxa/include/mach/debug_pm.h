
#ifndef DEBUG_PM_H
#define DEBUG_PM_H

/* for pm logger debugfs */
#define PM_LOGGER_COMM_DISPLAY 0
#define PM_LOGGER_APPS_DISPLAY 1
#define PM_LOGGER_BUF_CLEAR 2
#define PM_LOGGER_START_LOG 3
#define PM_LOGGER_STOP_LOG 4
#define PM_LOGGER_CHANGE_BUF_SIZE 5
#define PM_LOGGER_CHANGE_ONESHOT_MODE 6
#define PM_LOGGER_CHANGE_REG_MODE 7
#define PM_LOGGER_SET_MAX_D2_ACTIVE_TIME 8

void print_pm_logger_usage(void);

enum pxa9xx_force_lpm {
	PXA9xx_Force_None,
	PXA9xx_Force_D2,
	PXA9xx_Force_D1,
	PXA9xx_Force_CGM,
	PXA9xx_Force_count
};

enum stats_clk_event{
	GC_CLK_ON,
	GC_CLK_OFF,
	VMETA_CLK_ON,
	VMETA_CLK_OFF
};


/* for debugfs*/
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>

#ifdef CONFIG_DEBUG_FS
#define LPM_NAMES_LEN 20
extern const char pxa9xx_force_lpm_names__[][LPM_NAMES_LEN];
#endif
extern enum pxa9xx_force_lpm ForceLPM;
extern enum pxa9xx_force_lpm LastForceLPM;
extern unsigned int ForceLPMWakeup;
extern int RepeatMode;
extern int ForceVCTCXO_EN;

extern uint32_t profilerRecommendationPP;
extern uint32_t profilerRecommendationEnable;
void pxa_9xx_power_init_debugfs(void);
void pxa_9xx_power_cleanup_debugfs(void);
void gc_vmeta_stats_clk_event(enum stats_clk_event event);

#endif
