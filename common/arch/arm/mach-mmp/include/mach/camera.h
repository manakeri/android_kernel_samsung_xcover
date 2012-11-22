#ifndef __ASM_ARCH_CAMERA_H__
#define __ASM_ARCH_CAMERA_H__

#define SENSOR_LOW  0
#define SENSOR_HIGH 1

struct cam_platform_data {
	unsigned int vsync_gpio;
	int (*init)(void);
	void (*deinit)(void);
	void (*suspend)(void);
	void (*resume)(void);
	void (*sync_to_gpio)(void);
	void (*sync_from_gpio)(void);
};

struct clk;
struct sensor_platform_data {
	int id;
	int (*power_on)(int,int);
	int (*platform_set)(int, struct clk*);
};

#endif

