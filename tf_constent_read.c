#include<stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include"turbine.h"
// Return 1 if file is not present
int order_count = 0, const_count = 0;//y=10;
char* value ;
char *stopstring;
extern double order_3;// =    constant[3];//(8.91528337)*0.1 ;//pow( 1$
extern double order_2;// =    constant[2];//(-7.94133717)*1 ;//pow( 10$
extern double order_1;// =    constant[1];//( 5.05125439)*10 ;//pow( 1$
extern double order_0;// =    constant[0];//(5.32425150)*10; //pow( 10$

long double  constant [11];

int  turbine_constant()
{
/*
    FILE  *fr;

	fr = fopen("/home/debian/emergency_lak/tf_constant.txt", "r");

    if (!fr){
        printf("Can't open file\n");
	return (1);
	}
    else {

	char buffer[1024];


        	while (fgets(buffer,1024, fr))
		 {
            		char* value = strtok(buffer, ",");

            		printf("long double order%d  ",order_count);

			long double data = strtold(value,&stopstring);// 'data' stores before 'e' values
               		value = strtok(NULL, ",");

			float pow_data = atof(value); // 'pow_data' stores  power of 'e' data
			//constant[const_count] = data * pow(10,pow_data);
			constant[const_count] = pow_data;
			printf("  %.30lf", constant[const_count]); // constant[] store all constants
           		printf("\n");
			const_count++; order_count++;
	        }
	fclose(fr);

	const_count = 0;
*/

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
/*
long double lax = atof(array[0]);
long double y = atof(array[1]);
long double in_time = atof(array[2]);
long double ex_time = atof(array[3]);

*/
	order_3 =  atof(array[3]);//(8.91528337)*0.1 ;//pow( 1$
	order_2 =  atof( array[2]);//(-7.94133717)*1 ;//pow( 10$
	order_1 =  atof(array[1]);//( 5.05125439)*10 ;//pow( 1$
	order_0 =  atof(array[0]);//(5.32425150)*10; //pow( 10$
    
printf("order_0 = %.15lf \n", order_0);
printf("order_1 = %.15lf \n", order_1);
printf("order_2 = %.15lf \n", order_2);
printf("order_3 = %.15lf \n", order_3);

	

    return 0;
}
/*
int main()
{

turbine_constant();
}
*/
