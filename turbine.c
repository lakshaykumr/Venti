#include <math.h>
#include <stdio.h>
#include "turbine.h"

extern long double constant[11];
/////////////////////////////////////////////

extern float order_10;//=    constant[10];// (-1.35380345)*0.0000000000001;//pow(10,order_10_po$
extern float order_9;// =    constant[9];//( 5.64933421)*0.00000000001;//pow( 10 , order_9_po$
extern float order_8;// =    constant[8];//( -1.00993791)*0.00000001; //pow( 10 , order_8_pow)
extern float order_7;// =    constant[7];//(1.01042270 )*0.000001;//pow( 10 , order_7_pow)
extern float order_6;// =    constant[6];//(-6.20248001)*0.00001;//pow( 10 , order_6_pow)
extern float order_5;// =    constant[5];//( 2.41040457)*0.001;//pow( 10 , order_5_pow)
extern float order_4;// =    constant[4];//(-5.91526874)*0.01;//pow( 10 , order_4_pow)
extern double order_3;// =    constant[3];//(8.91528337)*0.1 ;//pow( 10 , order_3_pow)
extern double order_2;// =    constant[2];//(-7.94133717)*1 ;//pow( 10 , order_2_pow)
extern double order_1;// =    constant[1];//( 5.05125439)*10 ;//pow( 10 , order_1_pow)
extern double order_0;// =    constant[0];//(5.32425150)*10; //pow( 10 , order_0_pow)


////////////////////////// Power of constant /////////////////////


int transfer_function(int pressure)
{
/* int order_10_pow = -12;
int order_9_pow  = -10;
int order_8_pow  = -8;
int order_7_pow  = -6;
int order_6_pow  = -4;
int order_5_pow  = -3;
int order_4_pow  = -1;
int order_3_pow  =  0;
int order_2_pow  =  1;
int order_1_pow  =  1;
int order_0_pow  =  1;
///////////////////////Constant calculation ///////////////////
*/
//	printf
	float output_duty_cycle=(-0.000049 * pow(pressure,3)) + (0.004940 * pow(pressure,2)) + (1.033051 * pressure) + 6.789890;

	//float output_duty_cycle = (order_3 * pow(pressure,3)) + (order_2 * pow(pressure,2)) + (order_1 * pressure) + order_0;

	//float output_duty_cycle=(-0.000017 * pow(pressure,3)) + (-0.000190 * pow(pressure,2)) + (1.167719 * pressure) + 5.801494;
//19v	//float output_duty_cycle=(0.000015 * pow(pressure,3)) + (-0.000930 * pow(pressure,2)) + (0.749517 * pressure) + 4.093695;
	//float output_duty_cycle=(-0.0001662 * pow(pressure,3)) + (0.004159 * pow(pressure,2)) + (1.92 * pressure) + 5.845;
//	float output_duty_cycle = (-0.0001018 * pow(pressure,3)) + (0.0009962 * pow(pressure,2)) + (1.813 * pressure) + 5.543;
//	printf(" Output = %f" ,output_duty_cycle);
	return (int)(output_duty_cycle );

}

/*
int main()

{
int y;
printf( "Enter a value :");
scanf(" %d", &y);

int x  = transfer_function(y);
printf("PWM = %d", x);

return 0;

}
*/
