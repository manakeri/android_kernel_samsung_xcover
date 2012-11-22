extern struct platform_device pxa9xx_device_u2o;
extern struct platform_device pxa9xx_device_u2oehci;
extern struct platform_device pxa9xx_device_u2ootg;
extern struct platform_device pxa_device_mci;
extern struct platform_device pxa3xx_device_mci2;
extern struct platform_device pxa3xx_device_mci3;
extern struct platform_device pxa25x_device_udc;
extern struct platform_device pxa27x_device_udc;
extern struct platform_device pxa_device_fb;
extern struct platform_device pxa_device_ffuart;
extern struct platform_device pxa_device_btuart;
extern struct platform_device pxa_device_stuart;
extern struct platform_device pxa_device_hwuart;
extern struct platform_device pxa_device_i2c;
extern struct platform_device pxa_device_i2s;
extern struct platform_device pxa_device_ficp;
extern struct platform_device sa1100_device_rtc;
extern struct platform_device pxa_device_rtc;
extern struct platform_device pxa_device_ac97;

extern struct platform_device pxa27x_device_i2c_power;
extern struct platform_device pxa27x_device_ohci;
extern struct platform_device pxa27x_device_keypad;

extern struct platform_device pxa25x_device_ssp;
extern struct platform_device pxa25x_device_nssp;
extern struct platform_device pxa25x_device_assp;
extern struct platform_device pxa27x_device_ssp1;
extern struct platform_device pxa27x_device_ssp2;
extern struct platform_device pxa27x_device_ssp3;
extern struct platform_device pxa3xx_device_ssp4;

extern struct platform_device pxa25x_device_pwm0;
extern struct platform_device pxa25x_device_pwm1;
extern struct platform_device pxa27x_device_pwm0;
extern struct platform_device pxa27x_device_pwm1;
extern struct platform_device pxa95x_device_pwm4;
extern struct platform_device pxa95x_device_pwm5;
extern struct platform_device pxa95x_device_pwm6;
extern struct platform_device pxa95x_device_pwm7;

extern struct platform_device pxa930_acipc_device;

extern struct platform_device pxa3xx_device_nand;
extern struct platform_device pxa3xx_device_i2c_power;

extern struct platform_device pxa3xx_device_gcu;

extern struct platform_device pxa95x_device_i2c1;
extern struct platform_device pxa95x_device_i2c2;
extern struct platform_device pxa95x_device_i2c3;
extern struct platform_device pxa95x_device_csi;
extern struct platform_device pxa95x_device_cam0;
extern struct platform_device pxa95x_device_cam1;

extern int pxa9xx_usb_phy_init(unsigned int base);
void __init pxa_register_device(struct platform_device *dev, void *data);
