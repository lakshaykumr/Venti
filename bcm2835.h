#ifndef bcm2835
#define bcm2835

extern void bcm2835_gpio_set(int pin);
extern void bcm2835_gpio_clr(int pin);
extern void bcm2835_gpio_fsel(int pin, char *mode);

#endif

