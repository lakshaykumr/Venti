#include "breaking.h"
#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
//#include "sir_valve_turbine.h"
#include "bcm2835.h"

int break_init(void)
	{
	bcm2835_gpio_fsel(48,"out");
	bcm2835_gpio_fsel(46,"out"); //Q8
	//bcm2835_gpio_fsel(27,"out"); //Q7

	bcm2835_gpio_set(46);
	//bcm2835_gpio_clr(27);
	bcm2835_gpio_clr(48);
	}




int break_off(void)
        {
        bcm2835_gpio_set(46);
        usleep(1);
       // bcm2835_gpio_clr(27);
       // usleep(1);
        bcm2835_gpio_clr(48);
        usleep(1);
	}




int break_on(void)
	{
        bcm2835_gpio_set(48);
        usleep(1);
       // bcm2835_gpio_set(27);
       // usleep(1);
        bcm2835_gpio_clr(46);
	usleep(1);
        }

float break_time( float peep, float Pinsp)
	{
	int breaking_time;
       	float ratio = peep/Pinsp;
        printf("Ratio = %f \n",ratio);
			if (ratio > 0.8)
        		{
                                breaking_time = 8;
                                printf("Breaking time3 = 8ms \n");
				return breaking_time;
                        }
                        if (ratio >= 0.15 && ratio <= 0.4)
                        {
                                breaking_time = 250;
                                printf("Breaking time2 = 200ms \n");
				return breaking_time;
                        }

                        if (ratio > 0.4 && ratio <= 0.8)
                        {
                                breaking_time = 40;
                                printf("Breaking time4 = 40ms \n");
				return breaking_time;
                        }

                        if (ratio < 0.15 && ratio >= 0.0)
                        {
                                breaking_time = 200;
                                printf("Breaking time1 = 200ms \n");
				return breaking_time;
                        }

	}




/*
int main()
{
break_init();
while(1)
	{
	break_off();
	//usleep(1);
	//turbine_duty_cycle(40);
        sleep(1);
	//turbine_duty_cycle(40);
        //usleep(1);
	break_on();
	sleep(1);
	}
}

*/
