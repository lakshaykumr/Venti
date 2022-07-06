#include <stdio.h>
#include <math.h>

#include <stdlib.h>

float LEAK_FLow(float inp_MVi,float inp_MVe)
{
	printf(" Insput == %f %f \n" ,inp_MVi,inp_MVe);
	float leak_per = ((inp_MVi + (inp_MVe))/inp_MVi)*100;
	printf(" Leak === Function flow === %f \n",leak_per);
	return leak_per;
}

float compliance_function(float VTi_volume , float peak_insp , float peep_pressure)
{

	float compliance = VTi_volume /(peak_insp - peep_pressure);
	return compliance;
}


