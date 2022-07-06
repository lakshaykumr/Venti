#include<stdio.h>
#include<string.h>
#include "settings.h"
#include <stdlib.h>
#include "time.h"
#include <stdbool.h>
#define APNEA_VENTI_IE_RATIO	0.5
#define MAX_PRESS_LIMIT   65

////////////////////////// Setting Path///////////////////////////

char setting_path[] = "/home/debian/emergency_lak/setting.txt";
char mannual_breath_path[] = "/home/debian/emergency_lak/mannual_breath.txt";
char hold_path[] = "/home/debian/emergency_lak/hold.txt";
char mode_path[] = "/home/debian/emergency_lak/mode.txt";

//extern struct timespec backup_start;
//extern struct timespec backup_start;
extern bool backup_start_time;

char old_mode[4] = "11";
char return_string[4];
char settings_string[100];
char prev_string[100];
float rxd_para[20] ;
extern float inhale_time;
extern float exhale_time;
extern int cal_RR;
extern bool man_breath_flag;
extern bool Ehold , Ihold;
extern float Ihold_time, Ehold_time;
char hold_array[5]={0,0,0,0,0} ;
extern bool first_time;
extern bool setting_changed;
extern bool APNEA_on_off;
char man_breath =0;

///////////////////////// Setting Structure///////////////////////
FILE *fptr;
//////////////////////// Read settings////////////
float compare_string[20]; /* {final_setting.Plimit , final_setting.VTi,final_setting.peep,final_setting.RR_rxd,
				final_setting.Trigger,final_setting.Pinsp,final_setting.Ti,final_setting.PEAK_flow,
				final_setting.Fio2,final_setting.Psupp,final_setting.slope,final_setting.slope_flag}
				*/

extern int mode_para;

int read_settings(){

//	int current_mode = 0;
	fptr = fopen(setting_path,"r");
	if(fptr == NULL)
	{
		printf("Error in readinfg file \n");
	//	return -1;							// Return -1 if file is not opened
	}
	fscanf(fptr , "%s" , settings_string);
	fclose(fptr);

	int i = 0;

//	printf("%s \n" ,settings_string);
	if(strcmp(settings_string , prev_string) != 0)
		{
			//clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);

		//	printf(" Strings are not same\n ");
			strcpy(prev_string,settings_string);
		//	printf("%s \n" , prev_string);
			char *pt;
    			pt = strtok (settings_string,",");
    			while (pt != NULL) {
      			 	rxd_para[i] = atof(pt);

        		//	printf("%d  %f\n",i,rxd_para[i]);
        			pt = strtok (NULL, ",");

				if(compare_string[i] != rxd_para[i])
				{
					compare_string[i] = rxd_para[i];
					setting_changed = true;
					find_para(i,rxd_para[i]);
				}
				//printf("compare output %f\n" ,compare_string[i]);
				i++;
				if(i > 17)
				{
					break;
				}
			}

		}
	else
		{
	//	printf( " String are the same \n");
		}

	fptr = fopen(mannual_breath_path,"r");
//	printf(" Mannual breath bro **********************************************************7 \n");
        if(fptr == NULL)
        {
                printf("Error in Reading mannual Breath path \n");
        //        return -1;                                                      // Return -1 if file is $
        }

        fscanf(fptr , "%c" , &man_breath);
	//printf(" mannusl  == %c \n" , man_breath);
        fclose(fptr);
	if(man_breath == '1')
	{
		man_breath_flag = true;
//		printf(" Mannual breath bro **********************************************************8 \n");
		fptr = fopen(mannual_breath_path,"w");
		fprintf(fptr , "%c", '0');
		fclose(fptr);
	}
	fptr = fopen(hold_path,"r+");
        if(fptr == NULL)
        {
            //    printf("Error in Reading hold path path \n");
			;
		        //        return -1;                                                      // Return -1 if file is $
        }
	else
        {	fscanf(fptr , "%s" , hold_array);
	        fclose(fptr);
	}
//	printf(" %s \n ", hold_array);
	if((hold_array[0] == 'E') &&  (hold_array[1] == 'H') && (strlen(hold_array) == 5))
	{
		printf(" Exhale hold \n");
		char third[3];// = hold_array[2];
                memcpy(third,&hold_array[2],3);
                Ehold_time = atof(third);

//		if (remove(hold_path) == 0){
  //                      printf(" file sucessfully deleted \n");
    //            }
      //          else
        //                printf(" Problem in deleting file\n");

		Ehold =true;
		memset(hold_array, 0,strlen(hold_array));
	}
	else if((hold_array[0] == 'I') &&  (hold_array[1] == 'H') && (strlen(hold_array) == 5))
        {
		Ihold = true;
		char third[3];// = hold_array[2];
		memcpy(third,&hold_array[2],3);
		Ihold_time = atof(third);
	//	printf(" Inhale timde == %f\n ", Ihold_time);
                if (remove(hold_path) == 0){
			printf(" file sucessfully deleted \n");
		}
		else
			printf(" Problem in deleting file\n");

	//	printf(" Inhale hold \n");
		memset(hold_array, 0,strlen(hold_array));

        }

/*	current_mode = mode_change();
	if( current_mode != mode_para)
	{
		mode_para = current_mode;
		break;
	}
*/
//	printf(" mannual breath  ==========================%c \n", man_breath);
	return 0;								// Succesfully open
}

int mode_change()
{
	char mode[4] = {0,0,0,0};
	fptr = fopen(mode_path,"r");
        if(fptr == NULL)
        {
                printf("Error in readinfg file \n");
              return 0;                                                      // Return -1 if file is not opened
        }
        fscanf(fptr , "%s" , mode);
        fclose(fptr);
//	if(!(strcmp(mode ,"13") || strcmp(mode ,"12")))// || strcmp(mode ,"32"))) 
//		clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
	//printf(" mode string = %s \n", mode);
//	printf(" mode string length %d \n",strlen(mode));
	if(!strcmp(mode ,"32"))
	{
//		//mode[] = "41";
	//mode[0] = '4';
//	mode[1] = '1';

	sprintf(mode,"%s","41");
	//fscanf(fptr , "41" , mode);
	}
//	int result = strcmp(mode ,"18");
//	printf(" result == %d \n ", result);
	if(!strcmp(mode, "18"))
	{
//	mode[0] = '5';
  //      mode[1] = '1';
		//mode[] = "";
	sprintf(mode,"%s","51");
//	printf(" mode changed \n");
	//fscanf(fptr , "51  " , &mode);
	}
//	printf(" Mode == %s \n" , mode);
//	printf(" Mode == %s \n " , mode);
	if(atoi(mode) == 0.00)
	{
		//strcpy(	return_string , old_mode );
		return atoi(old_mode);
	}
	else
        {
		strcpy(old_mode, mode);
                return atoi(mode);
        }


}
void find_para(int index , float para)
{
	switch (index )

	{
	case 0:
			final_setting.Plimit = para;
			if(final_setting.Pinsp > final_setting.Plimit)
			{
				final_setting.Pinsp = final_setting.Plimit;
			}
	//		printf(" Preesure limit\n ");
			break;
	case 1:
			final_setting.VTi = para;
			first_time = true;
	//		printf(" vti \n"); 
			break;
	case 2:
			final_setting.peep = para;
	//		printf(" 3t \n"); 
			break;
	case 3:
                        final_setting.RR_rxd = para;
	//		printf(" 33\n ");
			exhale_time = ((60.00/final_setting.RR_rxd) - final_setting.Ti );
                        if(exhale_time < 0.3)
                        {
                                exhale_time = 0.3;
                        }

                        break;
        case 4:
                        final_setting.Trigger = para;
                        break;
        case 5:
                        final_setting.Pinsp = para;
			if(final_setting.Pinsp > final_setting.Plimit)
                        {
                                final_setting.Pinsp = final_setting.Plimit;

                        }
			if (final_setting.Pinsp > MAX_PRESS_LIMIT)
					final_setting.Pinsp = MAX_PRESS_LIMIT;
                        break;
	case 6:
                        final_setting.Ti = para;
			inhale_time = final_setting.Ti;
			exhale_time = ((60.00/final_setting.RR_rxd) - final_setting.Ti );
			if(exhale_time < 0.2)
			{
				exhale_time = 0.2;
			}
                        break;
        case 7:
                        final_setting.PEAK_flow = para;
                        break;
        case 8:
                        final_setting.Fio2 = para;
                        break;
	case 9:
                        final_setting.Psupp = para;
                        break;
        case 10:
                        final_setting.slope = para;
                        break;
//	case 11:
  //                      final_setting.slope_flag = para;
    //                    break;
	case 11:
			final_setting.Tlow = para;
                        break;
	case 12:
			final_setting.insp_term = para;
	//		printf("insp term == %f\n",final_setting.slope_flag);
                        break;
	case 13:
			if(para == 1)
			{
	//			printf(" Apnea ventilation OFF \n");
				APNEA_on_off = true;
				backup_start_time = true;
				final_setting.apnea_on_off = 1;
			}
			else{
	                        final_setting.apnea_on_off = 0;
				backup_start_time = false;
				APNEA_on_off = false;
			}
			break;
	case 14:
                        final_setting.apnea_RR = para;
			float total_breath_time = (60/final_setting.apnea_RR);
			final_setting.apnea_Ti = total_breath_time/3.00;
			final_setting.apnea_Te = total_breath_time - (final_setting.apnea_Ti);
			final_setting.apnea_Ti = total_breath_time - (final_setting.apnea_Ti * 2);
	//		printf(" Apnea Ti = %f Apnea RR = %f \n", final_setting.apnea_Ti ,final_setting.apnea_Te);
          //              printf("insp term == %f\n",final_setting.slope_flag);
                        break;
	case 15:
                        final_setting.apnea_time = para;
            //            printf("insp term == %f\n",final_setting.slope_flag);
                        break;

	case 16:
                        final_setting.apnea_Vt = para;
              //          printf("insp term == %f\n",final_setting.slope_flag);
                        break;
	case 17:
                        final_setting.apnea_trigger_flow = para;
                //        printf("insp term == %f\n",final_setting.slope_flag);
                        break;


	default:
			printf(" Setting indexing Error \n");
			break;
	}
}

