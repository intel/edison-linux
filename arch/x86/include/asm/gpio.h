#ifndef _ARCH_X86_GPIO_H
#define _ARCH_X86_GPIO_H

#if CONFIG_ARCH_HAVE_CUSTOM_GPIO_H

#if CONFIG_ARCH_NR_GPIO > 0
#define ARCH_NR_GPIOS CONFIG_ARCH_NR_GPIO
#endif

#include <asm-generic/gpio.h>

/* The trivial gpiolib dispatchers */
#define gpio_get_value	__gpio_get_value
#define gpio_set_value	__gpio_set_value
#define gpio_cansleep	__gpio_cansleep
#define gpio_to_irq	__gpio_to_irq

#else /* ! CONFIG_ARCH_HAVE_CUSTOM_GPIO_H */

#ifndef __LINUX_GPIO_H
#warning Include linux/gpio.h instead of asm/gpio.h
#include <linux/gpio.h>
#endif

#endif /* ! CONFIG_ARCH_HAVE_CUSTOM_GPIO_H */

#endif /* _ARCH_X86_GPIO_H */
