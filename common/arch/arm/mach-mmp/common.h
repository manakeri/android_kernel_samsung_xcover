#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

struct sys_timer;

extern void timer_init(int, int);
extern void mmp2_clear_pmic_int(void);

extern struct sys_timer pxa168_timer;
extern struct sys_timer pxa910_timer;
extern struct sys_timer mmp2_timer;
extern struct sys_timer mmp3_timer;
extern void __init pxa168_init_irq(void);
extern void __init pxa910_init_irq(void);
extern void __init mmp2_init_icu(void);
extern void __init mmp3_init_icu(void);
extern void __init mmp3_init_gic(void);
extern void __init mmp2_init_irq(void);
extern void __init mmp3_init_irq(void);
extern int mmp2_set_wake(unsigned int, unsigned int);
extern int pxa910_set_wake(unsigned int, unsigned int);

extern void __init icu_init_irq(void);
extern void __init pxa_map_io(void);

extern void ripc_release(void);
extern void ripc_get(void);
