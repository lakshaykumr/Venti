//#include "bcm2835.h"
#include <time.h>
#include <unistd.h>
#include "mine.h"
#include <stdio.h>
#include "flow_thread_1.h"
#include <string.h>
#include <stdlib.h>
#define resolution  (5.048/32767.00)
#define BI_FLOW 2.57
#define SINGLE_FLOW 1.00
char flow_calib_path[] = "/home/debian/emergency_lak/flow_data.txt";
char flow_data_calib[40];
char *rt;
float calib_value_flow[5];//= { 0,0,0,0,0};
FILE *file_risu;
/*

float insp_calib()
{
	int i =0;
	float sensor_raw_value =0;
	float output = 0;
	float value = 0;
	int msb = 0;
	int8_t analog_data[3] = {0,0,0};
        int8_t analog_data2[3] = {0,0,0};
	uint8_t read_command[3] =  {0b00010010,0,0} ;
	ADC114S08_RegWrite( INPMUX_ADDR_MASK, 0b10111010,3 );
	usleep(410);
	for (i = 0; i< 50; i++)
        {
       	 	bcm2835_spi_transfernb(read_command,analog_data,3);

        	msb = (int)analog_data[1] << 8 ;
       	 	output = msb + analog_data[2];
        	value = (resolution) * (float)output;
		usleep(10000);
		sensor_raw_value = value + sensor_raw_value;
	}
	sensor_raw_value = sensor_raw_value/50.00;
	sensor_raw_value = 2.500 - sensor_raw_value;
	printf( " calibrated value == %f \n" , sensor_raw_value);
	return sensor_raw_value;

     //   flow1_voltage = value /1000.00 - 0.022;
}

float exp_calib()
{
	int i =0;
	float sensor_raw_value =0;
	float output = 0;
	float value = 0;
	int msb = 0;
	int8_t analog_data[3] = {0,0,0};
        int8_t analog_data2[3] = {0,0,0};
	uint8_t read_command[3] =  {0b00010010,0,0} ;
//	ADC114S08_RegWrite( INPMUX_ADDR_MASK, 0b10111010,3 );
	ADC114S08_RegWrite( INPMUX_ADDR_MASK, 0b10011000,3 );
	usleep(410);
	for (i = 0; i< 50; i++)
        {
       	 	bcm2835_spi_transfernb(read_command,analog_data,3);

        	msb = (int)analog_data[1] << 8 ;
       	 	output = msb + analog_data[2];
        	value = (resolution) * (float)output;
		usleep(10000);
		sensor_raw_value = value + sensor_raw_value;
	}
	sensor_raw_value = sensor_raw_value/50.00;
	sensor_raw_value = 1.00 - sensor_raw_value;
	printf( " calibrated value == %f \n" , sensor_raw_value);
	return sensor_raw_value;

     //   flow1_voltage = value /1000.00 - 0.022;
}
*/

int both_calib(float *insp_error , float *exp_error)
{
	int i = 0;
 	file_risu = fopen(flow_calib_path,"r");
        if(file_risu == NULL)
        {
                printf(" We cant find any flow_callib file path \n");
                return -1;                                                      // Return -1 if file is $
        }

        fscanf(file_risu,"%s",flow_data_calib);
        fclose(file_risu);

	printf(" Hello \n");
        rt = strtok (flow_data_calib,",");
		printf("%s \n ", flow_data_calib);
	printf(" Hello 2 \n");
        while (rt != NULL) {
				printf(" %d \n", i);
        			calib_value_flow[i] = atof(rt);
                                 printf("%f\n",atof(rt));
                                rt = strtok (NULL, ",");
			/*
                                if(compare_string[i] != rxd_para[i])
                                {
                                        compare_string[i] = rxd_para[i];
                                        find_para(i,rxd_para[i]);
                                }
                        //      printf("compare output %f\n" ,compare_string[i]);
			*/
			        i++;
                      //          if(i > 10)
                        //        {
                          //              break;
                            //    }
                        }
		printf(" c1= %f, c2= %f, c3= %f , c4= %f \n",calib_value_flow[0],calib_value_flow[1],calib_value_flow[2],calib_value_flow[3]);
		*insp_error = BI_FLOW - calib_value_flow[1];
		*exp_error = SINGLE_FLOW  - calib_value_flow[0] ;
		 printf(" Hello 3\n");

		return 0;
}
