#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include<stdbool.h>
#include "alarm.h"
#define PRESSURE_ALARM_INDEX 5
#define VTI_ALARM_INDEX 6
#define MVE_ALARM_INDEX 7
#define RR_ALARM_INDEX 8
#define PEEP_ALARM_INDEX 9
#define PATIENT_ALARM_INDEX 10
#define BACKUP_ALARM_INDEX 11
#define SPO2_ALARM_INDEX 12

#define alarm_pressure_index 10
#define alarm_Vt_index 12
#define alarm_Mv_index 14
#define alarm_RR_index 16
#define alarm_Ti_index 18
#define patient_disc_index 20
#define backup_ventilation 52
#define alarm_spo2_index 22

typedef struct alarm_limits{
        float Pressure_low;
        float Pressure_high;
        float Vte_low;
        float Vte_high;
        float Mve_low;
        float Mve_high;
        float RR_low;
        float RR_high;
        float PEEP_low;
        float PEEP_high;
	float spo2_high;
	float spo2_low;
}alarm_limits;

alarm_limits alarm_struct;

char alarm_path[] = "/home/debian/emergency_lak/alarm_limit.txt";
char buzzer_path[] ="/home/debian/emergency_lak/buzzer.txt";
char alarm_setting[100];
char prev_alarm_setting[100];
float alarm_lim[10];
float compare_limits[10];
bool local_pressure_alarm = false;
bool local_Vte_alarm = false;
bool local_Mve_alarm = false;
bool local_RR_alarm = false;
bool local_PEEP_alarm = false;
bool local_patient_alarm = false;
bool local_backup_alarm = false;
bool local_spo2_alarm = false;
char *clear_array = "0,0,0,0,0,0,0,0,0,0,";

FILE *alarm_ptr;
FILE *buzzer_ptr;

int clear_buzzer()
{

	FILE *file_dis ;
	printf(" %s and clear status \n" ,__func__);
	file_dis = fopen(buzzer_path,"w");
//	printf(" %d \n ", file_dis);
        if(file_dis == NULL)
        {
                printf("Error in readinfg file \n");
                return 1;
        }
	for(int j = 0; j< 5 ;j++)
		fprintf(file_dis , clear_array,sizeof(clear_array) );
	fclose(file_dis);
	return 0;
}
void alarm_setting_funct()
{

        alarm_ptr = fopen(alarm_path,"r");
        if(alarm_ptr == NULL)
        {
                printf("Error in readinfg file alarm limits \n");
        //        return 1;                                                      // Return -1 if file is not opened
        }
	else{
        	fscanf(alarm_ptr , "%s" , alarm_setting);
        	fclose(alarm_ptr);

	        int i = 0;

  //      printf("%s \n" ,alarm_settings);
        	if(strcmp(alarm_setting, prev_alarm_setting) != 0)
                	{
               	 	//      printf(" Strings are not same\n ");
                 	       strcpy(prev_alarm_setting,alarm_setting);
                	//      printf("%s \n" , prev_alarm_lim);
                        	char *pt;
                        	pt = strtok (alarm_setting,",");
                        	while (pt != NULL) {
                        	        alarm_lim[i] = atof(pt);

	                                printf("%d  %f\n",i,alarm_lim[i]);
        	                        pt = strtok (NULL, ",");

                	           //     if(compare_limits[i] != alarm_lim[i])
                        	       // {
                            	 //          compare_limits[i] = alarm_lim[i];
                            	          //  setting_changed = true;
                            	            set_limit(i,alarm_lim[i]);
                            	   // }
                            	    //printf("compare output %f\n" ,compare_limits[i]);
                            	    i++;
                            	    if(i > 10)
                            	    {
                            	            break;
					//    return 1;
                            	    }
                        	}

			}
	//else
		//	printf(" String lengths are not same\n");

	//	return 0;
	}
	///return 1;
}


void set_limit(int index ,float value)
{
	switch(index){
		case 0: alarm_struct.Pressure_low = value;printf(" Value = %f \n", alarm_struct.Pressure_low);break;
		case 1: alarm_struct.Pressure_high = value;printf(" Value = %f \n", alarm_struct.Pressure_high);break;
                case 2: alarm_struct.Vte_low = value;printf(" Value = %f \n", alarm_struct.Vte_low);break;
                case 3: alarm_struct.Vte_high = value;;printf(" Value = %f \n", alarm_struct.Vte_high);break;
                case 8: alarm_struct.Mve_low = value;;printf(" Value = %f \n", alarm_struct.Mve_low);break;
                case 9: alarm_struct.Mve_high = value;;printf(" Value = %f \n", alarm_struct.Mve_high);break;
                case 6: alarm_struct.RR_low = value;;printf(" Value = %f \n", alarm_struct.RR_low);break;
                case 7: alarm_struct.RR_high = value;;printf(" Value = %f \n", alarm_struct.RR_high);break;
                case 4: alarm_struct.PEEP_low = value;;printf(" Value = %f \n", alarm_struct.PEEP_low);break;
                case 5: alarm_struct.PEEP_high = value;;printf(" Value = %f \n", alarm_struct.PEEP_high);break;
		case 10: alarm_struct.spo2_high = value;;printf(" Value = %f \n", alarm_struct.PEEP_high);break;
		case 11:alarm_struct.spo2_low = value;;printf(" Value = %f \n", alarm_struct.PEEP_high);break;
		default: printf("index = %d \n", index);printf(" Error in Alarm limit file \n");break;

	}
}

char buzzer_array[22];

int buzzer(int index , bool alarm_flag)
{


	printf(" Inside the buzzer program \n");
	switch(index){
		case 5: if(alarm_flag)
			{	buzzer_write_function(  alarm_pressure_index , '1'); //buzzer_array[10] = '1';
				printf(" Pressure alarm set to high\n");}
			else
			{	 buzzer_write_function(  alarm_pressure_index , '0');//buzzer_array[10] = '0';
				printf(" Pressure alarm set to Lowwwwwwww\n");
			}
			printf(" Pressure alarm genrated ++++++++++++++++++++++++++++\n");
			break;
		case 6: if(alarm_flag)
                                buzzer_write_function(  alarm_Vt_index , '1'); //buzzer_array[12] = '1';
                        else
                                buzzer_write_function(  alarm_Vt_index , '0'); //buzzer_array[12] = '0';
                        printf(" Vte alarm generated +++++++++++++++++++++++++\n");
                        break;

		case 7: if(alarm_flag)
                                buzzer_write_function(  alarm_Mv_index , '1'); //buzzer_array[14] = '1';
                        else
                                buzzer_write_function(  alarm_Mv_index , '0');  //buzzer_array[14] = '0';
                        printf(" MVe genrated +++++++++++++++++++++++++++++ \n");
                        break;

		case 8: if(alarm_flag)
                                buzzer_write_function(  alarm_RR_index , '1'); //buzzer_array[16] = '1';
                        else
                                buzzer_write_function(  alarm_RR_index , '0'); //buzzer_array[16] = '0';
                        printf(" RR genrated ++++++++++++++++++++++++++++++\n");
                        break;
		case 9: if(alarm_flag)
                                buzzer_write_function(alarm_Ti_index , '1'); //buzzer_array[18] = '1';
                        else
                                buzzer_write_function(  alarm_Ti_index , '0'); //buzzer_array[18] = '0';
                        printf(" Ti apnea genrated ++++++++++++++++++++++++\n");
                        break;
		//case 11: if(
		case 10: if(alarm_flag)
                                buzzer_write_function(patient_disc_index , '1'); //buzzer_array[18] = '1';
                        else
                                buzzer_write_function( patient_disc_index , '0'); //buzzer_array[18] = '0';
                        printf(" patient deisconnection ++++++++++++++++++++++++\n");
                        break;
		case 11: if(alarm_flag)
                                buzzer_write_function(backup_ventilation , '1'); //buzzer_array[18] = '1';
                        else
                                buzzer_write_function(backup_ventilation , '0'); //buzzer_array[18] = '0';
                        printf(" patient deisconnection ++++++++++++++++++++++++\n");
                        break;

		case 12: if(alarm_flag)
                                buzzer_write_function(alarm_spo2_index , '1'); //buzzer_array[18] = '1';
                        else
                                buzzer_write_function(alarm_spo2_index , '0'); //buzzer_array[18] = '0';
                        printf(" patient deisconnection ++++++++++++++++++++++++\n");
                        break;

	}


}
int buzzer_write_function( int offset , char value)
{
	//int count =1;
	//int buzzer_ptr;
	printf(" Offset  = %d \n",offset);
	buzzer_ptr = fopen(buzzer_path,"r+");
        if(buzzer_ptr == NULL)
        {
                printf("Error in readinfg file \n");
                return 1;
        }
	fscanf(buzzer_ptr , "%s" , buzzer_array);
	printf("befor %ld\n", ftell(buzzer_ptr));
	fseek(buzzer_ptr, 0 ,SEEK_SET);
	printf("after %ld\n", ftell(buzzer_ptr));
	printf(" Array before %s \n" , buzzer_array);
	buzzer_array[offset] = value;
	printf(" Array after %s \n" , buzzer_array);
	fprintf(buzzer_ptr , "%s", buzzer_array );
	//printf(" befor : - %d \n " ,buzzer_ptr);
	//printf("%ld \n", ftell(buzzer_ptr));

	//fseek(buzzer_ptr,offset-1,SEEK_END);

	// printf(" after 1 : - %d \n " ,buzzer_ptr);
	 //printf("%ld \n ", ftell(buzzer_ptr));
	//fwrite(&value , 1 , 1 , buzzer_ptr);
	// printf(" after 2  : - %d \n " ,buzzer_ptr);

	//pwrite(buzzer_ptr, value, count, offset);
        fclose(buzzer_ptr);
	printf("value %c and offset = %d \n",value,offset);
	return 0;
}

int pressure_alarm(float pip)
{

		printf(" Calling function set limittttttttttttttttttttttttt\n");
			printf(" input_pressure = %f \n",pip);
			printf(" low _ value = %f high value = %f \n",alarm_struct.Pressure_low , alarm_struct.Pressure_high);
	                if(((pip > alarm_struct.Pressure_high )|| (pip < alarm_struct.Pressure_low ))&& !local_pressure_alarm)
                        {               local_pressure_alarm = true;
                                        buzzer(PRESSURE_ALARM_INDEX,local_pressure_alarm);
					printf(" Pressure alarm genrated\n");
                        }
                        else if (((pip < alarm_struct.Pressure_high ) && (pip > alarm_struct.Pressure_low ))&& local_pressure_alarm)
                        {		printf(" Pressure alarm band ho gya hai \n");
                                        local_pressure_alarm = false;
                                        buzzer(PRESSURE_ALARM_INDEX,local_pressure_alarm);
                        }
			printf(" Calling function End \n");
			return 0;
}
int Vte_alarm(float pip)
{

                printf(" Calling function set limittttttttttttttttttttttttt  %d\n" , local_Vte_alarm);
                        printf(" input_pressure = %f \n",pip);
                       if(((pip > alarm_struct.Vte_high )|| (pip < alarm_struct.Vte_low ))&& !local_Vte_alarm)
                        {               local_Vte_alarm = true;
                                        buzzer(VTI_ALARM_INDEX,local_Vte_alarm);
                                        printf(" Pressure alarm genrated\n");
                        }
                        else if (((pip < alarm_struct.Vte_high ) && (pip > alarm_struct.Vte_low ))&& local_Vte_alarm)
                        {               printf(" Pressure alarm band ho gya hai \n");
                                        local_Vte_alarm = false;
                                        buzzer(VTI_ALARM_INDEX,local_Vte_alarm);
                        }
                        return 0;
}

int Mve_alarm(float pip)
{

		pip = (pip /1000.00) * -1;   
             printf(" Calling function set limittttttttttttttttttttttttt  %d\n", local_Mve_alarm);
                        printf(" input_pressure = %f \n",pip);
                       if(((pip > alarm_struct.Mve_high )|| (pip < alarm_struct.Mve_low ))&& !local_Mve_alarm)
                        {               local_Mve_alarm = true;
                                        buzzer(MVE_ALARM_INDEX,local_Mve_alarm);
                                        printf(" Pressure alarm genrated\n");
                        }
                        else if (((pip < alarm_struct.Mve_high ) && (pip > alarm_struct.Mve_low ))&& local_Mve_alarm)
                        {               printf(" Pressure alarm band ho gya hai \n");
                                        local_Mve_alarm = false;
                                        buzzer(MVE_ALARM_INDEX,local_Mve_alarm);
                        }
                        return 0;
}

int RR_alarm(float pip)
{

                printf(" Calling function set limittttttttttttttttttttttttt  %d\n" , local_RR_alarm);
                        printf(" input_pressure = %f \n",pip);
                       if(((pip > alarm_struct.RR_high )|| (pip < alarm_struct.RR_low ))&& !local_RR_alarm)
                        {               local_RR_alarm = true;
                                        buzzer(RR_ALARM_INDEX,local_RR_alarm);
                                        printf(" Pressure alarm genrated\n");
                        }
                        else if (((pip < alarm_struct.RR_high ) && (pip > alarm_struct.RR_low ))&& local_RR_alarm)
                        {               printf(" Pressure alarm band ho gya hai \n");
                                        local_RR_alarm = false;
                                        buzzer(RR_ALARM_INDEX,local_RR_alarm);
                        }
                        return 0;
}

int PEEP_alarm(float pip)
{

                printf(" Calling function set limittttttttttttttttttttttttt     %d\n" , local_PEEP_alarm);
                        printf(" input_pressure = %f \n",pip);
                       if(((pip > alarm_struct.PEEP_high )|| (pip < alarm_struct.PEEP_low ))&& !local_PEEP_alarm)
                        {               local_PEEP_alarm = true;
                                        buzzer(PEEP_ALARM_INDEX,local_PEEP_alarm);
                                        printf(" Pressure alarm genrated\n");
                        }
                        else if (((pip < alarm_struct.PEEP_high ) && (pip > alarm_struct.PEEP_low ))&& local_PEEP_alarm)
                        {               printf(" Pressure alarm band ho gya hai \n");
                                        local_PEEP_alarm = false;
                                        buzzer(PEEP_ALARM_INDEX,local_PEEP_alarm);
                        }
                        return 0;
}

int patient_disconn_alarm(bool pip)
{

                	//printf(" Calling function set limittttttttttttttttttttttttt\n");
                        //printf(" input_pressure = %f \n",pip);
                       if(pip )//if(((pip > alarm_struct.PEEP_high )|| (pip < alarm_struct.PEEP_low ))&& !local_PEEP_alarm)
                        {               local_patient_alarm = true;
                                        buzzer(PATIENT_ALARM_INDEX,local_patient_alarm);
                                        printf(" Pressure alarm genrated\n");
                        }
                        else if (!pip)//(((pip < alarm_struct.PEEP_high ) && (pip > alarm_struct.PEEP_low ))&& local_PEEP_alarm)
                        {               printf(" Pressure alarm band ho gya hai \n");
                                        local_patient_alarm = false;
                                        buzzer(PATIENT_ALARM_INDEX,local_patient_alarm);
                        }
                        return 0;
}

int backup_ventilation_alarm(bool pip)
{

                        //printf(" Calling function set limittttttttttttttttttttttttt\n");
                        //printf(" input_pressure = %f \n",pip);
                       if(pip )//if(((pip > alarm_struct.PEEP_high )|| (pip < alarm_struct.PEEP_low ))&& !local_PEEP_alarm)
                        {               local_backup_alarm = true;
                                        buzzer(BACKUP_ALARM_INDEX,local_backup_alarm);
                                        printf(" backup alarm genrated\n");
                        }
                        else if (!pip)//(((pip < alarm_struct.PEEP_high ) && (pip > alarm_struct.PEEP_low ))&& local_PEEP_alarm)
                        {               printf(" backup alarm band ho gya hai \n");
                                        local_backup_alarm = false;
                                        buzzer(BACKUP_ALARM_INDEX,local_backup_alarm);
                        }
                        return 0;
}


int spo2_alarm(bool pip)
{

                        //printf(" Calling function set limittttttttttttttttttttttttt\n");
                        //printf(" input_pressure = %f \n",pip);
                       if(pip )//if(((pip > alarm_struct.PEEP_high )|| (pip < alarm_struct.PEEP_low ))&& !local_PEEP_alarm)
                        {               local_spo2_alarm = true;
                                        buzzer(SPO2_ALARM_INDEX,local_spo2_alarm);
                                        printf(" backup alarm genrated\n");
                        }
                        else if (!pip)//(((pip < alarm_struct.PEEP_high ) && (pip > alarm_struct.PEEP_low ))&& local_PEEP_alarm)
                        {               printf(" backup alarm band ho gya hai \n");
                                        local_spo2_alarm = false;
                                        buzzer(SPO2_ALARM_INDEX,local_spo2_alarm);
                        }
                        return 0;
}

