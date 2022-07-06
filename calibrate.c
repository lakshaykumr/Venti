//#include"define.h"
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <syslog.h>
#include <linux/i2c-dev.h>


#include"error.h"
#define IO_ERROR_AMS -998
#define FILE_OPEN_ERROR_AMS -999


char *bus = "/dev/i2c-2";
char flow_calib_path[] = "/home/debian/emergency/flow_data.txt";

char error_start_bus[]                  = "Failed to open the bus \n";
char error_flow_read[]                  =" Input/Output error at reading flow \n";
char error_pressure_read[]              = "Input/Output error at reading pressure \n";
char error_open_calib[]                 = "can't open flow_data.txt file \n";

float flow(int address);
float pressure();
int start_bus();
int file;
char data[4] = {0},data2[4] = {0};
double Flow_lpm ;
//double prescmh20;


int start_bus()                                                           //to start the bus
{
        if((file = open(bus, O_RDWR)) < 0)
        {
                printf("Failed to open the bus. \n");
        //      exit(1);
                //syslog_function(error_start_bus);
                return FILE_OPEN_ERROR_AMS;
        }
        return 0;
}

int file_write_flow(float val)
{
	FILE *fp;
	fp = fopen("flow_data.txt","a");
	if(fp == NULL)       { printf("Error in openning file"); return 0; }

	fprintf(fp,"%f,",val);
	fclose(fp);
   return 1;
}


int file_clear_flow()
{
        if(fclose(fopen("flow_data.txt", "w")))       { printf("file clear error\n"); return 0; }
        return 1;
}


int file_write_pressure(float val)
{
	FILE *fp;
	fp = fopen("pressure_data.txt","a");
	if(fp == NULL)       { printf("Error in openning file"); return 0; }

	fprintf(fp,"%f,",val);
	fclose(fp);
   return 1;
}

int file_clear_pressure()
{
        if(fclose(fopen("pressure_data.txt", "w")))       { printf("file clear error\n"); return 0; }
        return 1;
}



//----------------------------------------flow function-----------------------------------------------------
float flow(int address)                                                   //function for flow sensor
{
                address = 0x28;
                float density=1.225;
                double area_1=0.000950625;
                double area_2=0.000085785;
                int pressure_1 , pressure_reading;
                double Pressure_pascal,Pressure_CMH2O,massFlow,volume_divisor,Flow,Flow_min;

                //volume_divisor=((1.0/((area_2*area_2))-(1.0/(area_1*area_1))));


                ioctl(file, I2C_SLAVE, address);
                if(read(file, data, 4) != 4)
        {
                printf("Error : Input/Output error from Flow \n");
                //syslog_function(error_flow_read);
                return IO_ERROR_AMS;
        }


                        pressure_1 = (0x3f & data[0]) << 8;
                        pressure_reading = pressure_1 | data[1];
                        Pressure_pascal = (((500.0 - (-500.0))/( 14765.0 -1638.0))* (pressure_reading - 1638.0) + (-500.0));
                        Pressure_CMH2O = Pressure_pascal *0.0101972 ;
                        volume_divisor=((1.0/((area_2*area_2))-(1.0/(area_1*area_1))));
                        massFlow=1000*(sqrt((abs(Pressure_pascal)*2)*density/volume_divisor));
                        Flow =(massFlow/density)* 1000 * 0.72 ;                                       //ml/sec
                        Flow_min=(Flow*60) ;                                                           //ml/min
                        Flow_lpm=Flow_min * 0.001    ;                                                 //lpm
                        return (float)Flow_lpm;
}

//--------------------------------pressure function----------------------------------------------------------------------
float pressure(int address)
{
	address = 0x78;

        ioctl(file, I2C_SLAVE, address);
        if(read(file, data, 4) != 4)
        {
                printf("Error : Input/Output error \n");
                return IO_ERROR_AMS;
        }
                int pressure_1 = (data[0] * 256 + data[1]);
                //pressure2 = ((((pressure_1 - 3277.0) / ((26214.0) / 3.0))-1.5)*70.30);
        //      printf("circuit pressure = %d\n",pressure2);
               float  pressuref = ((pressure_1 - 3277.0) / ((26214.0) / 1.5)) + 0;
                float pressure2 = pressuref * 70.307;      ////nidhi
		return (pressure2);
}
int callibration_pressure(void)
{
double offset_press=0;
double average_press;
int n=20;

start_bus();

for(int i=0; i<n; i++)
        {
        float prescmh20 = pressure(0x78);
        offset_press = prescmh20 + offset_press;
	//printf("pressure=%f \n",prescmh20);
        }
//printf("off = %f \n", offset_press);
average_press = offset_press/n;
printf("pressure = %f \n",average_press);

if(file_write_pressure(average_press))   printf("file data written\n");//pressure value
        				else    printf("error writting data\n");
return 1;
}


int callibration_flow(void)
{
double calib_value_flow;
double average_flow;
int n=20;

start_bus();

for(int i=0; i<n; i++)
        {
        Flow_lpm = flow(0x28);
        calib_value_flow= Flow_lpm + calib_value_flow;
        //printf("calibrating \n");
        }

average_flow = calib_value_flow/n;

printf("Flow = %f \n",average_flow);

if(file_write_flow(average_flow))   printf("file data written\n");
else    printf("error writting data\n");
return 1;
}


int main(int argc,char* argv[])
{
	int rec_arg = 0;
	printf("no. of arguments passed %d \n",argc);
	if(argc == 1)	{	printf("no valid argument passed \n");	exit(0);	}
	printf("argument value %d\n",atoi(argv[1]));
	if(rec_arg = atoi(argv[1]))	printf("conversion completed\n");
	else	printf("error main argument conversion \n");
	switch (rec_arg)
   		{
			case 1:  	if(callibration_flow()) printf("callibration process completed for flow sensor \n");
					else printf("callibration fault for flow sensor  \n");
					break;

			case 2:		if(callibration_pressure()) printf("callibration process completed for pressure sensor \n");
					else printf("callibration fault for pressure sensor  \n");
					break;

			case 3: 	printf("In progress (OXYGEN) \n");
					/*if(callibration_oxygen()) printf("callibration process completed for oxygen sensor \n");
					else printf("callibration fault for oxygen sensor  \n");*/
					break;

			case 4:		printf("turbine callibration in progress\n");
					break;

			default:	printf("Choice other than 1 2 3 4 is entered in callib code\n");
					break;
		}
}
