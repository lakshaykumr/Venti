#include<stdio.h>
#include "bcm2835.h"
#include <unistd.h>
#include <stdbool.h>
#include "sir_valve_turbine.h"
#include "turbine.h"
#include"breaking_function.h"
#define TIME_DELAY_BREAKING 50000
//#define BREAKING_PIN 21

bool breaking_flag = false;
float insp_pressure_breaking = 0;
int break_init(void)
        {
        bcm2835_gpio_fsel(48,"out");
        //bcm2835_gpio_fsel(115,"out"); //Q8
        bcm2835_gpio_fsel(112,"out"); //Q7

        //bcm2835_gpio_set(115);
        bcm2835_gpio_clr(112);
        bcm2835_gpio_clr(48);
        }

int breaking_enable(float pplat , float peep , int time)
{
	insp_pressure_breaking = pplat;
	breaking_flag = true;
//	printf(" Breaking +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ start\n");0
//	bcm2835_gpio_clr(BREAKING_PIN);
	bcm2835_gpio_clr(112);
        usleep(1);
        bcm2835_gpio_clr(48);
        usleep(1);

}


int breaking(float pplat , float peep,int time)
{
	float ratio = peep/insp_pressure_breaking;
	float breaking_time = 0;
//	printf(" ratio  = %f ,  pplat %f pee %f \n", ratio,peep,pplat);
	if (ratio > 0.8)
        {
                                breaking_time = 8;
                                //printf("Breaking time = 8ms \n");
                        }

        if (ratio >= 0.15 && ratio <= 0.4)
        {
                                breaking_time = 200;
                                //printf("Breaking time = 200ms \n");

                        }

        if (ratio > 0.4 && ratio <= 0.8)
        {
                                breaking_time =40;
                                //printf("Breaking time = 40ms \n");

                        }

        if (ratio < 0.15 && ratio >= 0.0)
        {
                                breaking_time = 200;
                                //printf("Breaking time = 200ms \n");

         }
//	printf(" %d timer \n", time);
//	printf(" %f breakking time \n", breaking_time);
	if(breaking_flag &&(time > breaking_time))
	{
		printf(" Breaking +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ END\n");
		printf(" Breaking    %d    +++++++++++++++++ \n",time);
		breaking_flag = false;
//	usleep(breaking_time *1000);
//		bcm2835_gpio_set(BREAKING_PIN);
		bcm2835_gpio_set(48);
        	usleep(1);
        	bcm2835_gpio_set(112);
        	usleep(1);
		float duty_cycle = transfer_function(peep);
		turbine_duty_cycle(duty_cycle);


	}

}


