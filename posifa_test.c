#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
//#include "sir_valve_turbine.h"



float flow();
int file ;
char *bus = "/dev/i2c-2";

void start_bus() 
{
//int file ;
//char *bus = "/dev/i2c-2";


        if ((file = open(bus, O_RDWR)) < 0) 
        {
                printf("Failed to open the bus. \n");
                exit(1);
        }

ioctl(file, I2C_SLAVE, 0x48);
char config[3] = {0};
        config[0] = 0x01;
        config[1] = 0x81;            //BA
        config[2] = 0x93;           //83
        write(file, config, 3);
        sleep(1);

        // Read 2 bytes of data from register(0x00)
        // raw_adc msb, raw_adc lsb
        char reg[1] = {0x00};
        write(file, reg, 1);
//	fclose(file);
}

float flow()
{
float flow_rate;
char data[2]={0};

//start_bus();
if(read(file, data, 2) != 2)
        {
                printf("Error : Input/Output Error \n");
        }
        else
        {
                // Convert the data
                int raw_adc = (data[0] * 256 + data[1]);
                //float mV  = (raw_adc * 0.1875)/1000  ;
              	//float  flow_rate = (((mV-2.57)/4) * 400) ;
		float Voltage = (6.144/32768.0) * raw_adc;

		//printf("gain : %f \n" , mV);
		printf("raww_adc : %d \n" , raw_adc);
		printf("voltage : %f \n" , Voltage);

		//fclose(file);
		return flow_rate;

	}
//return flow_rate;
}


void main()
{

//float factor , flow_avg , flow_final , flow_rate_final;
int n =20;
int sum = 0;

//pwm_init();
//turbine_duty_cycle(500);

start_bus();
for(int i=0; i<n; i++)
{
float factor = flow();
sum = sum + factor;
}
float flow_avg = sum/n;
sum=0;

while(1)
{
//turbine_duty_cycle(20);
float flow_final=flow();
float flow_rate_final= flow_final-flow_avg;
float flow_lpm = flow_rate_final * 1.09;
//printf("voltage : %f \n" , mV);
//printf("flow_rate : %f \n" , flow_rate_final);

//printf("flow_rate : %f \n" , flow_lpm);

}
}
//for(int i=0; i<n; i++)
 //{

