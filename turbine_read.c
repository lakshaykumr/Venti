#include<stdio.h>
#include<string.h>
#include "settings.h"
#include <stdlib.h>
#include "time.h"
#include <stdbool.h>

char error_OPEN_CSV[] = "Can't open txt file\n";
void setting()
{

char* array[100];
int i = 0, n=4;

FILE* fp = fopen("tf_constant.txt", "r");

	if (!fp){
		printf("Can't open txt file\n");
		//syslog_function(error_OPEN_CSV);
		}
	else {
		 //Here we have taken size of
		//array 1024 you can modify it
		char buffer[1024];
		while (fgets(buffer,1024, fp))
			{
			char* value = strtok(buffer, ", ");
				while (value || i<n) {
				array[i]=malloc(sizeof(char)*strlen(value));
				strcpy(array[i],value);
				//printf("%s \n",value);
				value = strtok(NULL, ", ");
				i++;
			}

					}

		// Close the file
		fclose(fp);

		}
		
for(int i=0; i<n; i++)
{
printf("data%d = %s \n",i+1,array[i]);
}
long double lax = atof(array[0]);
long double y = atof(array[1]);
long double in_time = atof(array[2]);
long double ex_time = atof(array[3]);

printf("x = %.15lf \n", lax);
printf("y = %.15lf \n", y);
printf("IN_TIME = %.15lf \n", in_time);
printf("EX_TIME = %.15lf \n", ex_time);


//duty_in = (-0.0001018 * pow(x,3)) + (0.0009962* pow(x,2)) + (1.813 *x) + 5.543; //CALCULATE PPLAT DUTYCYCLE
//duty_ex = (-0.0001018 * pow(y,3)) + (0.0009962* pow(y,2)) + (1.813 *y) + 5.543; //CALCULATE PEEP DUTYCYCLE

/*
printf(" Duty_in = %d\n", duty_in);
printf(" Duty_ex = %d\n", duty_ex);
*/

FILE *fpt;

fpt = fopen("demo_2.txt", "w+");
if (!fpt){
	printf("Can't open txt file\n");
//	syslog_function(error_OPEN_CALIB);
	}
else
	{
	for(i=0; i<4; i++)
	{
	//x = atof(array())
	fprintf(fpt,"%0.15lf,",atof(array[i]));
	}
	fclose(fpt);
	}
for(i=0; i<n; i++)
{
free(array[i]);
}


}

int main()
{
setting();
}
