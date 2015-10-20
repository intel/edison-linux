#ifndef __INTEL_MID_PWM_H__
#define __INTEL_MID_PWM_H__

#define MAX_DUTYCYCLE_PERCENTAGE 100

enum {
	PWM_LED = 0,
	PWM_VIBRATOR,
	PWM_LCD_BACKLIGHT,
	PWM_NUM,
};

struct intel_mid_pwm_device_data {
	u16 reg_clkdiv0;
	u16 reg_clkdiv1;
	u16 reg_dutycyc;
	u8 val_clkdiv0;
	u8 val_clkdiv1;
};

struct intel_mid_pwm_platform_data {
	int pwm_num;
	struct intel_mid_pwm_device_data *ddata;
	u16 reg_clksel;
	u8 val_clksel;
};

int intel_mid_pwm(int id, int value);
#endif

