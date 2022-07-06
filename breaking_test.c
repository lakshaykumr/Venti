#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include "sir_valve_turbine.h"
#include "bcm2835.h"

int main()
{
system("config-pin P9_16 pwm >/dev/null 2>&1"); //lk
pwm_init();
bcm2835_gpio_fsel(48,"out");
bcm2835_gpio_fsel(115,"out"); //Q8
bcm2835_gpio_fsel(112,"out"); //Q7

bcm2835_gpio_set(115);
bcm2835_gpio_clr(112);
bcm2835_gpio_clr(48);

while(1)
	{
	bcm2835_gpio_set(115);
	usleep(1);
	bcm2835_gpio_clr(112);
	usleep(1);
	bcm2835_gpio_clr(48);
        usleep(1);
	turbine_duty_cycle(40);
	sleep(1);

	turbine_duty_cycle(0);
	usleep(1);
	bcm2835_gpio_set(48);
	usleep(1);
	bcm2835_gpio_set(112);
        usleep(1);
	bcm2835_gpio_clr(115);
	sleep(1);

	}
}
