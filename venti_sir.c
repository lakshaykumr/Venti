#include "settings.h"
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include "sir_valve_turbine.h"
#include "turbine.h"
#include <pthread.h>
//#include <stdio.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "timer_function.h"
//n #include "pwm.h"           //nidhi
#include "error.h"         //nidhi
#include "serial.h"
#include <math.h>
////n #include "flow_thread_1.h"
////n #include "mine.h"
#include "bcm2835.h"
#include "extra.h"
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include "callibration_function.h"
#include "venti_parameter.h"
#include "exhale_valve.h"              //nidhi
////n #include "tf_constent_read.h"
/////////////////////////////////////////////////////
#define BREAKING 100 //ms
#define HEALTHY_COMPLIENCE 30.00
#define MAX_TI 3.6
#define READING_TIME 30
#define PMEAN_ARRAY_SIZE ((MAX_TI*1000)/READING_TIME)
/////////////////////////////////////////////////////
#define IO_ERROR_AMS -998
#define FILE_OPEN_ERROR_AMS -999
#define VREF 5.065
#define RES VREF/32767.00
#define DELTA 0.98
#define INHALE_VALVE 49    //nidhi
#define HOLD_TUBINE_FACTOR 0.50
#define FLOW_TIME 13
void string_a(float, float , int , float );
void string_b(float Pinsp_pressure , int VTI_insp , float PFRi , float Pmean_insp,float Mvi_insp,int x,float Ti_insp , float rxd_pressure,float Rise_time_inhale  );
void string_c(float, float , int , float  );
void string_d(float peep_pressure , int bpm , int fio2 , float PFRe , float MVE_exhale , float leak_percentage  , int unknown_var , int volume ,float Te_exhale);
//float flow_init(void);
float flow_insp(int address);   //nidhi
float flow_insp_error=0 , flow_exp_error=0;
///////////////////////////////////////////////
float order_10=    (-1.35380345)*0.0000000000001;//pow(10,order_10_pow)
float order_9 =    ( 5.64933421)*0.00000000001;//pow( 10 , order_9_pow)
float order_8 =   ( -1.00993791)*0.00000001; //pow( 10 , order_8_pow)
float order_7 =    (1.01042270 )*0.000001;//pow( 10 , order_7_pow)
float order_6 =    (-6.20248001)*0.00001;//pow( 10 , order_6_pow)
float order_5 =    ( 2.41040457)*0.001;//pow( 10 , order_5_pow)
float order_4 =    (-5.91526874)*0.01;//pow( 10 , order_4_pow)
float order_3 =     (8.91528337)*0.1 ;//pow( 10 , order_3_pow)
float order_2 =    (-7.94133717)*1 ;//pow( 10 , order_2_pow)
float order_1 =    ( 5.05125439)*10 ;//pow( 10 , order_1_pow)
float order_0 =     (5.32425150)*10; //pow( 10 , order_0_pow)

//////////////////////////////////////////////////////////

//extern float calc_flow_insp = 0 ;
float volume_final = 0;
int mode_para = 0;
/////////////////////////////////////// Only for pressure sensor insp (0x16)////////////////////////////////////////////
float Pmax = 1.5;
float Pmin = 0;
float Digout_max = 29491.0;
float Digout_min = 3277.0;
int read_pressure(int);
int start_bus();
int file;
char *bus = "/dev/i2c-2";
int pressure_1 = 0;
char data[4] = {0};
float pressure2=0 ;
float pressuref;     ////n
/////////////////////////////////////// Thread global variable ////////////////////////////////////
float calc_pressure = 0;
pthread_mutex_t mutex;
//////////////////////////////////////Ventilator parameter/////////////////////////////////////
float inhale_time;
float exhale_time;
int cal_RR;
clock_t Timer;
float inhale_timer , exhale_timer;
///////////////////////////////////////Timer Structure ///////////////////////////////////////////
struct timespec start_time = {0,0};
struct timespec end_time = {0,0};
struct timespec deltat = {0,0};
////////////////////////////////////// Volume parameter ////////////////////////////////////////
struct timespec volume_start_time = {0,0};
struct timespec volume_end_time = {0,0};
struct timespec volume_deltat = {0,0};
struct timespec start_time_exhale = {0,0};
struct timespec end_time_exhale = {0,0};
struct timespec simv_start_time_exhale = {0,0};
struct timespec simv_end_time_exhale = {0,0};

float voloume_final = 0;
float insp_previous_value = 0,exp_previous_value = 0;
/////////////////////////////////////////////////////////////////////////////////////////////

int n = 4;
int *RR_array;// = {0,0,0,0};
int *MVe_array ;
int *MVi_array ;
int *Pmean_array;
int trigger_exp_time = 0;
int cmv_exp_time = 0;
int simv_exp_time = 0;

///////////////flow variables(added extra by nidhi)////////////////
//float flow_insp(int address);
double  final_flow =0;
//float volume_final =0;

/////////////////////////////////////////////////////////////////////////////////////////
bool Exhale_flag = false;
bool man_breath_flag = false;
bool Ehold , Ihold;
float Ihold_time =0, Ehold_time = 0;
bool first_time = false;
bool setting_changed = false;
//////////////////////////////////////////////////////////////////////////////////////////
void debug();

////////////////////////////////////// Pressure threads ////////////////////////////////////////

int start_bus()
{
        if((file = open(bus, O_RDWR)) < 0)
        {
                printf("Failed to open the bus. \n");
        //      exit(1);
                return FILE_OPEN_ERROR_AMS;
        }
        return 0;
}

float pressure(int address)
{       address = 0x78;
        ioctl(file, I2C_SLAVE, address);
        if(read(file, data, 4) != 4)
        {
                printf("Error : Input/Output error \n");
                return IO_ERROR_AMS;
        }
                pressure_1 = (data[0] * 256 + data[1]);
        ////n        pressure2 = ((((pressure_1 - 3277.0) / ((26214.0) / 3.0))-1.5)*70.30);
		pressuref = ((pressure_1 - 3277.0) / ((26214.0) / 1.5)) + 0;
                pressure2 = pressuref * 70.307;      ////nidhi

        //      printf("circuit pressure = %d\n",pressure2);
                return (pressure2);
}

bool thread_control_pressure = false;
float exp_pressure_reading = 0;

void* press_thread()
{	printf(" Pressure Thread_ created \n");
	bool local_pressure_thread = false;
        while(1){
	//	printf(" inside the loop\n");
		pthread_mutex_trylock(&mutex);
		local_pressure_thread = thread_control_pressure;
		pthread_mutex_unlock(&mutex);
		while(local_pressure_thread)
        	{//	printf(" running in loop \n");
                	pthread_mutex_trylock(&mutex);
                	calc_pressure = pressure(0x78);
			exp_pressure_reading = pressure(0x78);
			local_pressure_thread = thread_control_pressure;
			printf("preesure using thread %f \n",calc_pressure);
                	pthread_mutex_unlock(&mutex);
                	usleep(20000);
        	}
		usleep(1000000);

	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
setting_parameter final_setting;



/*///////////////////////////??FLow Thread ////////////////////
float flow_thread()
{
        uint8_t adc_ID = 0x4;//id = 100 or 0x04 first 3 bits
        int8_t analog_data2[3] = {0,0,0};
        int8_t analog_data[3] = {0,0,0};
        uint8_t read_command[3] =  {0b00010010,0,0} ;
        int output = 0;
        int output2 = 0;
        float value = -1;
        float value2 = -1;
        int msb = 0;
        int msb2 = 0;
        float input_data = 0;
        float output_data = 0;
        float input_data2 = 0;
        float output_data2 = 0;
        usleep(22000);
        spi_init();
        bcm2835_spi_transfer(RESET_OPCODE_MASK);
        if(adc_ID == ((0x4) & (ADC114S08_RegRead(ID_ADDR_MASK ,0 ,3))))
        {
                printf("Device found");
        }
       if(0 == ADC114S08_RegRead(STATUS_ADDR_MASK ,0 ,3))//0x01
        {
		 printf("adc ready");
	 }
        adc_start();
        while(1)
        {

	        ADC114S08_RegWrite( INPMUX_ADDR_MASK, 0b10111010,3 );
//		printf("ADC114S08_RegWrite-1\n");
//		usleep(5000);
	        bcm2835_spi_transfernb(read_command , analog_data,3);
		printf("bcm2835_spi_transfernb-1\n");
	        msb = (int)analog_data[1] << 8 ;
	        output = msb + analog_data[2];
	        value = (RES) * (float)output * 1000;
        	input_data = value/1000.00;
        	output_data=voltage_flow(input_data);
///		printf("Hello\n");
/////////////////////////////////////////////////////////////////////////////////////////
//		usleep(5000);
        	ADC114S08_RegWrite( INPMUX_ADDR_MASK, 0b10011000,3 );
		printf("ADC114S08_RegWrite-2\n");
		usleep(5000);
        	bcm2835_spi_transfernb(read_command , analog_data2,3);
		printf("bcm2835_spi_transfernb-2\n");
        	msb2 = (int)analog_data2[1] << 8 ;
        	output2 = msb2 + analog_data2[2];
        	value2 = (RES) * (float)output2 * 1000;
		input_data2 = value2/1000.00;
        	output_data2 = voltage_flow(input_data2);

        	pthread_mutex_trylock(&mutex);
        	calc_flow_insp = output_data - output_data2;
   	   	pthread_mutex_unlock(&mutex);
        }
}

//nidhi//float flow_init()
{

	uint8_t adc_ID = 0x4;//id = 100 or 0x04 first 3 bits
        int8_t analog_data[3] = {0,0,0};
        int8_t analog_data2[3] = {0,0,0};
        uint8_t read_command[3] =  {0b00010010,0,0} ;
        int output = 0;
        float value = -1;
        int msb = 0;
        int output2 = 0;
        float value2 = -1;
        int msb2 = 0;
	float volume_timer = 0;

//        uint8_t adc_ID = 0x4;//id = 100 or 0x04 first 3 bits
  //      int8_t analog_data[3] = {0,0,0};
    //    int8_t analog_data2[3] = {0,0,0};
      //  uint8_t read_command[3] =  {0b00010010,0,0} ;
    //    int output = 0;
     //   float value = -1;
      //  int msb = 0;
       // int output2 = 0;
       // float value2 = -1;
       // int msb2 = 0;
        float output_insp=0 , output_exp =0;
        float final_flow = 0;
        float flow1_voltage = 0.0;

        usleep(22000);
        spi_init();
        bcm2835_spi_transfer(RESET_OPCODE_MASK);
        if(adc_ID == ((0x4) & (ADC114S08_RegRead(ID_ADDR_MASK ,0 ,3))))
                {
                        printf("Device found");
                }
        if(0 == ADC114S08_RegRead(STATUS_ADDR_MASK ,0 ,3))//0x01
                {
                        printf("adc ready");
                }
        adc_start();
        bcm2835_spi_transfer(0x19);//self offset command
        usleep(5000);
	bcm2835_spi_transfer(START_OPCODE_MASK);///STOP_OPCODE_MASK
//              adc_start(0b10111010);
//              bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
        //      bcm2835_spi_transfer(START_OPCODE_MASK);
                ADC114S08_RegWrite( INPMUX_ADDR_MASK, 0b10111010,3 );
//              adc_start(0b10111010);
              //  bcm2835_spi_transfer(START_OPCODE_MASK);
                usleep(813);
//      for(int i=0;i<20;i++)
//              {
//                      ADC114S08_RegWrite( INPMUX_ADDR_MASK, 0b10111010,3 );

                        bcm2835_spi_transfernb(read_command , analog_data,3);
                        msb = (int)analog_data[1] << 8 ;
                        output = msb + analog_data[2];
                        value = (RES) * (float)output;
                //      printf(" Analog voltage 1  == %f \n " , value);
		flow1_voltage = value;
		output_insp =  bi_directional(flow1_voltage + flow_insp_error);//-494.5602 + 465.636*flow1_voltage - 160.6871*pow(flow1_voltage,2) + 21.42495*pow(flow1_voltage,3);  //voltage_flow(value /1000.00);  //            bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, HIGH);
//////////////////////////////////////////////////////////////////////////////////////
                ADC114S08_RegWrite( INPMUX_ADDR_MASK, 0b10011000,3 );
                usleep(813);
                        bcm2835_spi_transfernb(read_command , analog_data2,3);
			clock_gettime(CLOCK_MONOTONIC_RAW, &volume_end_time);
			volume_timer = delta_t(&volume_start_time, &volume_end_time,&volume_deltat);
			clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
                        msb2 = (int)analog_data2[1] << 8 ;
                        output2 = msb2 + analog_data2[2];
                        value2 = (RES) * (float)output2 ;
		output_exp = voltage_flow(value2 + flow_exp_error);
	//	final_flow =(output_insp * 0.9 +insp_previous_value) - (output_exp * 0.9 +exp_previous_value);
	//	printf(" isnp = %f Exp = %f volt = %f  Exp = %f \n",output_insp,output_exp,(value + flow_insp_error) * 1000 , (value2 + flow_exp_error) *1000);
//		if((Exhale_flag) && (final_flow < 6))
//			volume_final = ((final_flow) * volume_timer * 1.0)/60.00;
//		else
		//	volume_final = ((final_flow) * volume_timer * 1.0)/60.00  + volume_final;
	//	exp_previous_value = output_exp * 0.10;
	//	insp_previous_value = output_insp * 0.10;
  //      bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, HIGH);
//      usleep(17000);

//        while(1)
}
float flow_insp()
        {

	float volume_timer = 0;

	uint8_t adc_ID = 0x4;//id = 100 or 0x04 first 3 bits
        int8_t analog_data[3] = {0,0,0};
        int8_t analog_data2[3] = {0,0,0};
        uint8_t read_command[3] =  {0b00010010,0,0} ;
        int output = 0;
        float value = -1;
        int msb = 0;
        int output2 = 0;
        float value2 = -1;
        int msb2 = 0;
	float output_insp=0 , output_exp =0;
	float final_flow = 0;
	float flow1_voltage = 0.0;
//              adc_start(0b10111010);
//              bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
        //      bcm2835_spi_transfer(START_OPCODE_MASK);
                ADC114S08_RegWrite( INPMUX_ADDR_MASK, 0b10111010,3 );
//              adc_start(0b10111010);
              //  bcm2835_spi_transfer(START_OPCODE_MASK);
                usleep(410);
//      for(int i=0;i<20;i++)
//              {
//                      ADC114S08_RegWrite( INPMUX_ADDR_MASK, 0b10111010,3 );

                        bcm2835_spi_transfernb(read_command , analog_data,3);
                        msb = (int)analog_data[1] << 8 ;
                        output = msb + analog_data[2];
                        value = (RES) * (float)output;
                //      printf(" Analog voltage 1  == %f \n " , value);
			flow1_voltage = value;
			output_insp =  bi_directional(flow1_voltage + flow_insp_error);//-494.5602 + 465.636*flow1_voltage - 160.6871*pow(flow1_voltage,2) + 21.42495*pow(flow1_voltage,3);  //voltage_flow(value /1000.00);  //            bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, HIGH);
		//	if(!Exhale_flag)
			//	output_insp = output_insp - (output_insp * 0.1848);
//////////////////////////////////////////////////////////////////////////////////////
                	ADC114S08_RegWrite( INPMUX_ADDR_MASK, 0b10011000,3 );
                	usleep(410);
                        bcm2835_spi_transfernb(read_command , analog_data2,3);
			clock_gettime(CLOCK_MONOTONIC_RAW, &volume_end_time);
			volume_timer = delta_t(&volume_start_time, &volume_end_time,&volume_deltat);
			clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
                        msb2 = (int)analog_data2[1] << 8 ;
                        output2 = msb2 + analog_data2[2];
                        value2 = (RES) * (float)output2 ;
			output_exp = voltage_flow(value2 + flow_exp_error);
			output_exp = (output_exp* 0.9+ exp_previous_value) *1.0915248;//- (output_exp * 0.07);
			final_flow =(output_insp * 0.90 +insp_previous_value) - (output_exp * 0.90 +exp_previous_value);
//		if((Exhale_flag) && (final_flow < 6))
//			volume_final = ((final_flow) * volume_timer * 1.0)/60.00;
//		else
	//	if((abs((int)(output_exp)) - abs((int)(output_insp))) > 2){
			if((abs((int)final_flow) > 6) )
			{
			volume_final = ((final_flow) * volume_timer * 1.0)/60.00  + volume_final;
			//volume_final = DELTA * volume_final;
		//	printf(" Volume == %f \n", volume_final);
			}
//			if((abs((int)final_flow) < 10) )
//					final_flow = 0;		// for varsha
		//	else if(!Exhale_flag)
		//		volume_final = ((final_flow) * volume_timer * 1.0)/60.00  + volume_final;
			printf(" insp flow  = %f  Exp FLow  = %f \n",(flow1_voltage + flow_insp_error),(value2 + flow_exp_error));
		exp_previous_value = output_exp * 0.10;
		insp_previous_value = output_insp * 0.10;

		return (final_flow);
}////////////////////changes by nidhi
*/

////changes added by nidhi
float flow_insp(int address)
{
                address = 0x28;
                float density=1.225;
                double area_1=0.000950625;
                double area_2=0.000085785;
                int msb , pressure_reading;
                double Pressure_pascal,Pressure_CMH2O,massFlow,volume_divisor,Flow,Flow_min;
                float volume_timer = 0;
                //float final_flow = 0;
                char data[4] = {0};


                //volume_divisor=((1.0/((area_2*area_2))-(1.0/(area_1*area_1))));


                ioctl(file, I2C_SLAVE, address);
                if(read(file, data, 4) != 4)
        {
                printf("Error : Input/Output error from Flow \n");
                //syslog_function(error_flow_read);
                return IO_ERROR_AMS;
        }


                        clock_gettime(CLOCK_MONOTONIC_RAW, &volume_end_time);
                        volume_timer = delta_t(&volume_start_time, &volume_end_time,&volume_deltat);
                        clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
                        msb = (0x3f & data[0]) << 8;
                        pressure_reading = msb | data[1];
                        Pressure_pascal = (((500.0 - (-500.0))/( 14765.0 -1638.0))* (pressure_reading - 1638.0) + (-500.0));
                        Pressure_CMH2O = Pressure_pascal *0.0101972 ;
                        volume_divisor=((1.0/((area_2*area_2))-(1.0/(area_1*area_1))));
                        massFlow=1000*(sqrt((abs(Pressure_pascal)*2)*density/volume_divisor));
                        Flow =(massFlow/density)* 1000 * 0.72 ;                                       //ml/sec
                        Flow_min=(Flow*60) ;                                                           //ml/min
                        final_flow=Flow_min * 0.001    ;  //change name to final_flow                    //lpm
                        //volume_final = (Flow * (diff / 1000.0 ));
                        volume_final = (Flow * (volume_timer/1000.0 )) + volume_final;
                        return (float)final_flow;

}


int error(int x)
{
		printf(" Error = %d \n" ,x );
               if(x == EDEADLK)
                   printf("A deadlock was detected");
               else if(x == EINVAL)
                   printf("thread is not a joinable thread");
               else if(x == ESRCH)
                   printf("No thread with the ID thread could be found");
               else if(x == EPERM)
                   printf("No permission");
               else if(x == EAGAIN)
                   printf("Insufficient resources to create another thread");
               else if(x == EBUSY)
                   printf("Mount device busy ");
               else if(x == EACCES)
                   printf("Permission denied ");
               else if(x == EADDRNOTAVAIL)
                   printf("Address not available ");
               else if(x == ENOMEM)
                   printf("Insufficient memory ");

}

bool slope_thread_global = false;
bool trigger_thread = false;
bool thread_exit = false;
bool volume_flag = false;
/* void* slope_thread_parallel(void* arg)
{
        float pplat_pwm , peep_pwm;
        float inc_f=0.0;
        float slope=0.0;
        int pplat_pres , peep_pres ,flag_rec ,Psupport = 0;
        struct setting_parameter *struct_ptr = (struct setting_parameter*) arg;
	float new_factor = 0;
	bool slope_thread_control = false;
	bool trigger_pressure = false;
	bool local_thread_exit = true;
	bool internal_volume_flag = false;
	while(local_thread_exit){
		usleep(1000);
		pthread_mutex_trylock(&mutex);
		internal_volume_flag = volume_flag;
		local_thread_exit = thread_exit;
		slope_thread_control = slope_thread_global;
		trigger_pressure = trigger_thread;
		pthread_mutex_unlock(&mutex);
//	if(slope_thread_control)
//		printf("THread is on \n");
	if(slope_thread_control)
	{
		(pthread_mutex_trylock(&mutex));
		 slope = struct_ptr->slope;
		if(slope <= 0)
		{
				slope = 1;
		}
		if(!trigger_pressure && internal_volume_flag){
                        pplat_pres = struct_ptr->Plimit;
			slope = 1;
			printf(" pplat_pres = %d \n ", pplat_pres);
			}
		else if(!trigger_pressure && !internal_volume_flag){
	        	pplat_pres = struct_ptr->Pinsp;
			slope = slope*inhale_time*10;
			}
		else if(trigger_pressure){
			pplat_pres = struct_ptr->Psupp;
			slope = slope*inhale_time*10;
			}

	        peep_pres = struct_ptr->peep;
	        flag_rec = struct_ptr->slope_flag;

//		if(internal_volume_flag)

//		else
		//	slope = slope*inhale_time*10;
       		(pthread_mutex_unlock(&mutex));

        // we have to write transfer function
      			pplat_pwm =  transfer_function((int)(pplat_pres)); // -0.0000003386 * pow(pplat_pres,6) + 0.00006778 * pow(pplat_pres,5) - 0.0053 * pow(pp$

	      		peep_pwm =  transfer_function((int)(peep_pres)); //-0.0000003386 * pow(peep_pres,6) + 0.00006778 * pow(peep_pres,5) - 0.0053 * pow(peep_$

        ///	if(flag_rec)
        //	{
			printf(" PWM == %f \n", (pplat_pwm - peep_pwm));
                	inc_f = (pplat_pwm - peep_pwm)/slope;

  	              	new_factor = peep_pwm;
        	        for(int i = 0; i <= slope; i++)
                        {
			if(!slope_thread_control)
			{
				printf(" End of loop \n");
				break;
			}
                        new_factor = new_factor+ inc_f;
                        turbine_duty_cycle(new_factor);
		        usleep(800);
			pthread_mutex_trylock(&mutex);
	        	slope_thread_control = slope_thread_global;
        		pthread_mutex_unlock(&mutex);
//                      printf("pwm inc %f\n",new_factor);

                        }
			pthread_mutex_trylock(&mutex);
                        slope_thread_global = false;
			trigger_thread = false;
                        pthread_mutex_unlock(&mutex);
		//	slope_thread_control = false;
   //             	printf("set ciruit pressure (p_plat) %d\n",pplat_pres);
     //           	printf("slope time %f\n",slope);
       //         	printf("set peep %d\n",peep_pres);
         //       	printf("flag == 1\n");
        //	}
        //	else{

          //      	turbine_duty_cycle(pplat_pwm);
            //    	printf("flag = 0 p_plat reached \n");
	 //	}
//	return NULL;


			printf("End of thread \n");
	}

	}
	printf(" Killed pressure thread\n");
	(pthread_exit(NULL));
}*/

////////////////////////////////////////////////////////Pressure Modes /////////////////////////////////////////////////////////////////
void pressure_modes()
 {
	//pwm_init();
	
//	flow_init();
//	start_bus();
//	volume
	pthread_mutex_trylock(&mutex);
        thread_control_pressure = true;
        pthread_mutex_unlock(&mutex);
	struct timespec Break_start_time = {0,0};
	struct timespec Break_end_time = {0,0};
	struct timespec Break_deltat = {0,0};
	volume_flag = false;
	struct timespec  reading_start = {0,0};
	struct timespec reading_end = {0,0};
	struct timespec deltat_reading = {0,0};
	int timer = 0;
	int break_timer = 0;
	int reading_timer = 0;
	int duty_cycle = 0;
	int insp_time = 0;
	int exp_time =1000;
	int insp_trigger_time = 0;
	float pressure_insp = 0;
	const int reading_set_time = READING_TIME;
	float flow_combine = 0;
	int fio2_cal = 21;
	float PFRe_cal = 0;
	float MVe =0;
	float MVi =0 ;
	float leak = 0;
	float Ve = 0;
	float Vi = 0;
	int counter = 0;
	float check_rr;
	int RR_cal = 0;
	float final_rr = 0;
	float pip = 0;
	float rise_time = 0;
	bool rise_flag = false;
	bool rr_flag = false;
	float pmean = 0;
	float final_flow =0;
	float leak_flow = 0.0;
//	float Pmean_array[(int)PMEAN_ARRAY_SIZE];
	int Pmean_count = 0;
	double Pmean_add = 0;
	float Pmean = 0;
	float Pif_insp = 0;
	float Pef_exp = 0;
	float duty_cycle_valve_cal = 0;
	float TiTOT = 0.0;
	float trigger = 0;
	bool exp_flag = true;
	char A_string[] = {0};
	char b_string[] = {0};
	char c_string[] = {0};
	char d_string[] = {0};
	int current_mode = 0;
	Pmean_array =  (int*)malloc(300*sizeof(float));
	MVe_array = (int*)malloc(4*sizeof(float));
	MVi_array = (int*)malloc(4*sizeof(float));
	RR_array =  (int*)malloc(4*sizeof(int));			// Allocate array with the help of malloc
//	turbine_duty_cycle(0);
	clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
	clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
	bool trigger_flag = false;
	bool trigger_enable = false;
	bool trigger_mandatory = false;
	bool cmv =false;
	bool psv_flag = false;
	float mean_airway_pressure = 0;
	float TiTOT_slope =0;// insp_time * final_setting.slope /100.00;
        float TiTOT_remain = 0;// insp_time * (100 - final_setting.slope) /100.00;
        float mean_airway_pressure_thread = 0;
	float flow_cycle_per = 0.25;
	bool pc_ac_flag = false;
	bool patient_disconnection = false;
	//float leak_flow = 0;
	float exp_pressure = 0;
//////////////////////////////////////////////////////////////////////////////////
	struct timespec  hold_start_time = {0,0};
        struct timespec hold_end_time = {0,0};
        struct timespec delta_hold = {0,0};
	float hold_time =0;
	float cal_peep =0;
	int flow_count = 0;
	bcm2835_gpio_fsel(48,"out");
	while(1)
	{
		read_settings();
		 final_flow=flow_insp(0x28);     //nidhi
          	 printf("Flow =%f\n" ,  final_flow);    //nidhi
		current_mode = mode_change();
		
 /*      		if( current_mode/10 != mode_para/10)
        	{
                //	mode_para = current_mode;
                	break;
        	}*/
		trigger_mandatory = false;
		trigger_enable = false;
//		printf(" Pressure == %f Time == %d    %f\n" , pressure_insp ,exp_time,duty_cycle_\valve_cal);
		switch(current_mode%10)
		{
			case 1 : trigger_flag =false ;cmv = true;psv_flag =false; pc_ac_flag = false;break;		// Trigger flag indicate for pc_simv
			case 2 : trigger_flag = true;cmv = false; psv_flag =false ;pc_ac_flag = false;break;		// psv_flag indicate that every trigger breath is on Pinsp
			case 3 : psv_flag =true; cmv = false;trigger_flag = true;pc_ac_flag = false;break;		//pc_ac_flag indicate that this is not flow cycled
			case 7 : psv_flag =true; cmv = false;trigger_flag = true;pc_ac_flag = true;break;
			default : printf(" We are working on it \n");break;
		}
		if(exp_time > (FLOW_TIME * flow_count))
				{	flow_combine = flow_insp(0x28);
					flow_count++;
				}

	//	flow_combine = flow_insp();
		clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                if(reading_timer > reading_set_time)
                {			pthread_mutex_trylock(&mutex);
                                	pressure_insp = calc_pressure;
					exp_pressure = exp_pressure_reading;
                                	pthread_mutex_unlock(&mutex);
                                //	printf("insp pressure  = %f  Exp pressure = %f \n", pressure_insp ,exp_pressure);

				//	pthread_mutex_unlock(&mutex);
					sprintf(c_string, "C@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final * -1,TiTOT);
					serial_data_write(c_string);
                                        clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                }
		clock_gettime(CLOCK_MONOTONIC_RAW, &end_time_exhale);
		exp_time =  delta_t(&start_time_exhale, &end_time_exhale,&deltat);
		cmv_exp_time = exp_time;
		if(( trigger_flag && (flow_combine  > final_setting.Trigger)) && !patient_disconnection)
		{
			printf(" Trigger Enable \n ");
//			printf("inp 1 =  %d \n" , timer+insp_time);
//			printf("inp 1 =  %d \n" , inhale_time + exhale_time);
			if((exp_time ) > ((exhale_time) * 1000 * 0.80))
			{
				printf("  Mandatory Trigger \n");
				trigger_mandatory = true;

			}
			else
			{
				printf(" Normal trigger \n");
				trigger_enable = true;
				simv_exp_time = exp_time - simv_exp_time;
//
			}
		}
		if(pressure_insp <= 5) //LAKSHAY
				{
				//printf("+++++++++++++++++++++++breaking----------------");
				//printf("valve close @ Pressure => %f \n", pressure_insp);
				 valve_duty_cycle(100);
				}

	//	printf( " time == %d \n" , cmv_exp_time);
		if((((cmv_exp_time +insp_time) >= (inhale_time+exhale_time)*1000)) ||  man_breath_flag || trigger_enable || trigger_mandatory)
		{	//exp_time = timer;

			if(Ehold && !trigger_enable){
                        bcm2835_gpio_set(INHALE_VALVE);
                        int new_pres = final_setting.peep + final_setting.peep*HOLD_TUBINE_FACTOR;
                //      (new_pres > 70)?new_pres=70:new_press=new_press;// 
                        if(new_pres > 70)
                        {
                                new_pres = 70;
                        }
                        duty_cycle = transfer_function(new_pres);
                        turbine_duty_cycle(duty_cycle);

                        clock_gettime(CLOCK_MONOTONIC_RAW, &hold_start_time);
                        clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);

		        hold_time = delta_t(&start_time, &end_time,&delta_hold);
                        while ( (hold_time <= (Ehold_time * 1000)))
                        {       flow_combine = flow_insp(0x28);    //nidhi
                                pthread_mutex_trylock(&mutex);
                                pressure_insp = calc_pressure;
                                exp_pressure = exp_pressure_reading;
                                pthread_mutex_unlock(&mutex);
                                clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                                 if(reading_timer > reading_set_time)
                                {
                                      //nidhi  sprintf(A_string, "A@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final,trigger );
					sprintf(A_string, "A@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,0,0 );

                                        serial_data_write(A_string);    // Error Code
                                        clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                                }
                                clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);
                                hold_time = delta_t(&hold_start_time, &hold_end_time,&delta_hold);
                        }
                        valve_duty_cycle(98);
                        bcm2835_gpio_clr(INHALE_VALVE);
                        Ehold = false;
                        Ehold_time = 0;
               		}
///////////////////////////////// inhale start///////////////////////

			clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
			turbine_duty_cycle(100);  // nidh
                        //usleep(100000);            //   nidhi

			MVe_array[counter] = volume_final * final_setting.RR_rxd;
			counter++;
			if(counter >= 4 )
                        {

                                counter = 0;
                        }
						///
			if( rr_flag)
			{
				MVe = ((MVe_array[0] +MVe_array[1] +MVe_array[2] +MVe_array[3] )/4.0);
				if(MVi <= 0 )
				{
					MVi = 1;
				}
				leak_flow  = LEAK_FLow(MVi,MVe);//((MVi - (-1) *MVe)/MVi)*100;
                        	if(leak_flow < 0)
					leak_flow = 0;
			}
			printf(" leak ========== %f\n",leak_flow);
			if(leak_flow > 90){
				patient_disconnection = true;
				serial_data_write("ACK00");
				}
			else{
				patient_disconnection = false;
				serial_data_write("ACK01");
			    }
			Ve = volume_final;
			printf(" before compensation %f \n",Ve);

			Ve = Ve+1.2*(pip-cal_peep);
			printf(" after compensation %f \n",Ve);
			cal_peep = pressure_insp;
			Exhale_flag = false;										//
			sprintf(d_string, "D@%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f#", round(pressure_insp) , final_rr , 21 ,(-1)*Pef_exp ,(-1)*MVe/1000.00 ,  leak ,mean_airway_pressure , Ve*-1 ,cmv_exp_time/1000.00);
			printf("%s  \n", d_string);
			printf("data ============== %d \n ", strlen(d_string));
			serial_data_write(d_string);    // Error Code
			Pef_exp = 0;
			//turbine_duty_cycle(100);   nidhi
 //908                  
			usleep(90000);             //  nidhi

		//	printf(" Flag =====   %d \n" ,trigger_enable);
			if(trigger_enable)
			{	printf(" Triggered  pressure \n ");
				printf("final_setting.psuppppppppp %f :  \n"  , final_setting.Psupp);
				duty_cycle = transfer_function((int)(final_setting.Psupp));
				printf("duty cycle 1: %d \n " , duty_cycle);
				//turbine_duty_cycle(duty_cycle);
				turbine_duty_cycle(duty_cycle);
			if(!psv_flag){
			//	turbine_duty_cycle(41);
			/*	pthread_mutex_trylock(&mutex);
                                slope_thread_global = true;
				trigger_thread = true;
                                pthread_mutex_unlock(&mutex);*/    
				printf("PC-SIMV triggered breath\n");
	             //                   simv_exp_time = 0;
				}
			else
				{
				pthread_mutex_trylock(&mutex);
                                slope_thread_global = true;
                              //  trigger_thread = true;
                                pthread_mutex_unlock(&mutex);
				  printf("PSV triggered breath\n");
				}
			}
			else// if(trigger_enable)
			{	printf(" mandartory  pressure \n ");
				 printf("final_setting.pinssspppppp %f :  \n"  , final_setting.Pinsp);

				duty_cycle = transfer_function((int)(final_setting.Pinsp));
				 printf("duty cycle 2 : %d \n " , duty_cycle);

                        //       turbine_duty_cycle(duty_cycle);
				/*pthread_mutex_trylock(&mutex);
                        	slope_thread_global = true;
                       		pthread_mutex_unlock(&mutex);*/
				turbine_duty_cycle(duty_cycle);
				simv_exp_time = 0;
			}
			clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
			valve_duty_cycle(100);
			
			man_breath_flag = false;
			timer = 0;
			rise_flag = true;
			pip = 0;
			Pmean_count = 0;
			volume_final = 0;
			pthread_mutex_trylock(&mutex);
                        pressure_insp = calc_pressure;
                        pthread_mutex_unlock(&mutex);
			Pif_insp = 0;
			clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
			flow_combine = flow_insp(0x28);  //nidhi
			if (flow_combine < 0){
				flow_combine=	flow_combine * -1;
			}
///			printf( " flow == %f \n " , flow_combine);
                        //        pip = max(pip , pressure_insp);
          //              Pif_insp = max(Pif_insp,flow_combine);
			clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
                        timer = delta_t(&start_time, &end_time,&deltat);

//			printf("  pif === %f \n ", Pif_insp);
		//	printf(" %f \n" , final_setting.Psupp);
	//		printf(" flow_combine  == %f Pif_insp == %f \n", flow_combine  , Pif_insp);
	//		printf(" Timer == %f \n", timer);
	//		printf(" Step 1 \n");
			flow_count = 1;
			while((timer < (inhale_time)* 1000))  // && (flow_combine > (Pif_insp *flow_cycle_per))))// || (!trigger_enable && psv_flag && (timer < (inhale_time)* 1000) && (flow_combine > (Pif_insp * flow_cycle_per)))  )
			{	//	printf(" After Error 1 \n");
				//flow_combine = flow_insp();
				
				pip = max(pip , pressure_insp);
				//turbine_duty_cycle(100);   nidhi
				//usleep(100000);               nidhi

			//	printf("%f ,%d\n" , flow_combine ,timer);
			//	printf(" flow_combine  == %f Pif_insp == %f \n", flow_combine  , Pif_insp * 0.25);
			//	if((flow_combine < 1)  && (flow_combine > -4))
			//		flow_combine = 1;
				Pif_insp = max(Pif_insp,flow_combine);

				if((flow_combine <= (int) (Pif_insp  * final_setting.insp_term/100.0)) && ( trigger_enable || psv_flag) && !pc_ac_flag && !slope_thread_global)
				{	printf(" Breakinnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnng \n");
						break;
				}
				pthread_mutex_trylock(&mutex);
				pressure_insp = calc_pressure;
				exp_pressure = exp_pressure_reading;
                		pthread_mutex_unlock(&mutex);
			//	printf(" insp pressure  = %f Exp pressure = %f \n", pressure_insp ,exp_pressure);
			//	A_string = itoa(pressure);
				clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
				reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
		//		 printf(" After Error 2 \n");
				if(trigger_enable || trigger_mandatory)
					trigger = 1.0;
				else
					trigger = 0.0;
			//	printf( " trigger = %f \n" , trigger);
				if(timer > (FLOW_TIME * flow_count))
				{//	printf(" Timer = %d \n",timer);
					flow_combine = flow_insp(0x28);
					flow_count++;
				}

				if(reading_timer > reading_set_time)
				{
				//	Pmean_array[Pmean_count] = pressure_insp;
				//	Pmean_count++;
					sprintf(A_string, "A@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final,trigger );
		//			printf("%s \n " , A_string);
					serial_data_write(A_string);	// Error Code
					clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
				}
		//		printf(" After Error 3 \n");
				if((pressure_insp >= final_setting.Pinsp -1) && rise_flag)
				{
					clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
	                                rise_time = delta_t(&start_time, &end_time,&deltat);
					rise_flag = false;
				}
				else if(rise_flag)
					{
						rise_time = 0;
					}


				clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
				timer = delta_t(&start_time, &end_time,&deltat);
		//		printf("Timer: %d\n", timer);
			//	printf(" Pressure == %f Time == %d \n" , pressure_insp ,timer);


			}





//////////////nidhi breaking loop///////////////////////////////////////////////////



	/*	if (peak_insp_pressure < =0)
		{peak_insp_pressure = 1;
		ratio = 1;
		if (ratio > 0.8){
		     fd = open("/sys/class/gpio/gpio50/direction", O_WRONLY);
	             if (fd < 0)
	             {
		       printf("Unable to open direction\n");
		       return -1;
	             }
	             if (write(fd, "out", 3) != 3)
	             {
		        printf("Error writing to direction\n");
		        return -1;
	             }
	             close(fd);

		     usleep(50000);

		                }
		}


*/

////////////////////////nidhiii breaking loop endsss///////////////////////////////////////////////////////nidhi
  			turbine_duty_cycle(12);
		/*pthread_mutex_trylock(&mutex);
                        slope_thread_global = false;
                        pthread_mutex_unlock(&mutex);*/

			if(!trigger_enable)
				insp_time = timer;
			else
				insp_trigger_time =timer;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
			if(Ihold && !trigger_enable){
	                bcm2835_gpio_set(INHALE_VALVE);
			int new_pres = final_setting.Pinsp + final_setting.Pinsp*HOLD_TUBINE_FACTOR;
		//	(new_pres > 70)?new_pres=70:new_press=new_press;// 
			if(new_pres > 70)
			{
				new_pres = 70;
			}
			duty_cycle = transfer_function(new_pres);
                        turbine_duty_cycle(duty_cycle);

			clock_gettime(CLOCK_MONOTONIC_RAW, &hold_start_time);
			clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);
                        hold_time = delta_t(&start_time, &end_time,&delta_hold);
			while ( (hold_time <= (Ihold_time * 1000)))
			{	flow_combine = flow_insp(0x28);
				pthread_mutex_trylock(&mutex);
                                pressure_insp = calc_pressure;
                                exp_pressure = exp_pressure_reading;
                                pthread_mutex_unlock(&mutex);
				clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
				 if(reading_timer > reading_set_time)
                                {
                                        sprintf(A_string, "A@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final,trigger );
                                        serial_data_write(A_string);    // Error Code
                                        clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                                }
				clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);
                        	hold_time = delta_t(&hold_start_time, &hold_end_time,&delta_hold);
			}
		

	//		int new_pres = final_setting.Pinsp + final_setting.Pinsp*HOLD_TUBINE_FACTOR;
                //      (new_pres > 70)?new_pres=70:new_press=new_press;// 
          //              if(new_pres > 70)
            //            {
              //                  new_pres = 70;
                //        }
			//valve_duty_cycle(0);
			duty_cycle = transfer_function(final_setting.peep);
                        turbine_duty_cycle(duty_cycle);
			usleep(500000);
			bcm2835_gpio_clr(INHALE_VALVE);
			Ihold = false;
			Ihold_time = 0;
			}
			else
			valve_duty_cycle(98);
			
			//BREAKING
			int break_timer = 0, breaking_time;
			float ratio = final_setting.peep/final_setting.Pinsp;
			printf("Ratio = %f \n",ratio);
			if (ratio > 0.8)
			{
				breaking_time = 200;
				printf("Breaking time = 200ms \n");
			}
			if (ratio >= 0.15 && ratio <= 0.4)
                        {
                                breaking_time = 400;
				printf("Breaking time = 400ms \n");

                        }

			if (ratio > 0.4 && ratio <= 0.8)
                        {
                                breaking_time = 300;
				printf("Breaking time = 300ms \n");

                       	}

			if (ratio < 0.15 && ratio >= 0.0)
                        {
                                breaking_time = 400;
				printf("Breaking time = 400ms \n");

                        }


			clock_gettime(CLOCK_MONOTONIC_RAW, &Break_start_time);
			while(break_timer < breaking_time)
			{
			bcm2835_gpio_set(48); //LAKSHAY   //nidhi
			printf("=================================breaking===================================================================== \n");
			turbine_duty_cycle(0);
			clock_gettime(CLOCK_MONOTONIC_RAW, &Break_end_time);
			break_timer = delta_t(&Break_start_time, &Break_end_time, &Break_deltat);
			}
			//usleep(BREAKING);//b delay
			bcm2835_gpio_clr(48);   //nidhi
			duty_cycle = transfer_function(final_setting.peep);
			printf("PEEP => %f duty cycle => %d ",final_setting.peep,duty_cycle);
                        turbine_duty_cycle(duty_cycle);
			///////////////////////////


				if(!trigger_enable)
			{//	printf("Wait for joining \n");
			//	error(pthread_join(slope_thread,NULL));
			//	printf("Wait complete \n");
				printf(" Mandatory exhale timer  \n");
				clock_gettime(CLOCK_MONOTONIC_RAW, &start_time_exhale);
				clock_gettime(CLOCK_MONOTONIC_RAW, &end_time_exhale);
				exp_time = delta_t(&start_time_exhale, &end_time_exhale,&deltat_reading);
				printf(" Exp_time == %d  \n",exp_time);

			}
			else
			{	 printf(" After Error 5 \n");
				clock_gettime(CLOCK_MONOTONIC_RAW, &simv_start_time_exhale);
				clock_gettime(CLOCK_MONOTONIC_RAW, &simv_end_time_exhale);
                             	timer = delta_t(&simv_start_time_exhale, &simv_end_time_exhale,&deltat_reading);
			}
			
			//valve_
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//	pthread_mutex_trylock(&mutex);
                  //      slope_thread_global = false;
                    //    pthread_mutex_unlock(&mutex);

		//	printf(" After Error 4 \n");
			//rise_time = 0;

			//valve_duty_cycle(95);
                	duty_cycle = transfer_function((int)(final_setting.peep));
                	turbine_duty_cycle(duty_cycle);
			 //printf(" After Error 6 \n");
			if(trigger_enable){
				RR_cal =  (int) ((60*1000)/(insp_trigger_time + simv_exp_time));
                                check_rr = ((10*1000*60)/(insp_time * 1.00 + cmv_exp_time * 1.00)) - (10 * (int)RR_cal);
			}
			else
			{	RR_cal =  (int) ((60*1000)/(insp_time +cmv_exp_time));

			//	printf(" insp_time == %d  exp time === %d \n", insp_time , cmv_exp_time);
			//	printf(" RR == %d \n ", RR_cal);
				check_rr = ((10*1000*60)/(insp_time * 1.00 + cmv_exp_time * 1.00)) - (10 * (int)RR_cal);
			}
			// printf(" After Error 7 \n");
		        if(counter >= 3 )
                        {
                                rr_flag = true;
                        //        counter = 0;
                        }
		//	printf("Check RR = %f \n" , check_rr);
			if(check_rr > 0.5)
			{
				RR_cal = RR_cal + 1;
			}
		//	printf(" RR_cal after adding == %d \n" ,RR_cal);
			RR_array[counter] = RR_cal;
			for ( int i = 0; i < Pmean_count ;i++)
			{
				Pmean_add = Pmean_array[i] + Pmean_add ;
			}
			Pmean = (Pmean_add /(Pmean_count *1.0));
			Pmean_count = 0;
			Vi = volume_final;
			printf(" before compensation %f \n",Vi);
			Vi = Vi-1.2*(pip-cal_peep);
			printf(" after compensation %f \n",Vi);
			MVi_array[counter] = Vi *  final_setting.RR_rxd;
		//	printf("%d \n ", counter);
			if(rr_flag)
			{	final_rr = (RR_array[0]  +RR_array[1] +RR_array[2] +RR_array[3]) /4.0 ;
				MVi =( MVi_array[0] +MVi_array[1] +MVi_array[2] +MVi_array[3] )/4.00;
			}
		//	printf(" MVi == %f\n",MVi);
			volume_final = 0;

		//	Pmean = Pmean_add/(Pmean_count*1.0);
		//	 printf(" Pmean  == %f\n",Pmean_count);
			sprintf(b_string, "B@%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f#",round(pip) , Vi , Pif_insp ,  Pmean, MVi/1000.0, 0.0,insp_time/1000.00 , pressure_insp,0,rise_time/1000.00);
			printf (" %s \n" , b_string);								// Pmean	TRIGGER_FLOW
                        serial_data_write(b_string);    // Error Code
			Pmean_add = 0;
			Pmean_count = 0;
			Pmean = 0;
			timer = 0;
			clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
			reading_timer =0;
			Pif_insp = 0;
			duty_cycle_valve_cal = 0;
			exp_flag = true;
			TiTOT = (insp_time/(inhale_time*1000.00 + exhale_time*1000.00)) *100;
		//	printf(" Ti/tot = %f \n" ,TiTOT);
			Exhale_flag = true;
			flow_count = 1;
			while((!trigger_enable &&(exp_time <  300)) ||  (trigger_enable && (timer < (300))))
			{
				flow_combine = flow_insp(0x28);
				Pef_exp = min(Pef_exp,flow_combine);
		       		pthread_mutex_trylock(&mutex);
				pressure_insp = calc_pressure;
				exp_pressure = exp_pressure_reading;
				pthread_mutex_unlock(&mutex);
				if(exp_time > (FLOW_TIME * flow_count))
				{	flow_combine = flow_insp(0x28);
					flow_count++;
				}

			//	printf(" insp pressure  = %f  Exp pressure = %f \n", pressure_insp ,exp_pressure);
			//	float Kp= 0.1;
			//	float Kc = 1.4;
				if((pressure_insp <= (final_setting.peep ))) //&& (((exp_time >= 300.0)))) //&& !trigger_enable ) || ((timer >= 300.0)&& trigger_enable )))
				{//	exp_flag = false;
			/*	float i = 0;
				for ( i = 90.0; i<= 100.00 ; i = i +0.1)
				{	duty_cycle_valve_cal = i ; //duty_cycle_valve_cal + 1;
				//	modified_pressure = (set_peep - pressure)*0.2 * 2 + modified_pressure
				//	duty_cycle_valve_cal = Kp * Kc * duty_cycle_valve_cal + duty_cycle_valve_cal;
				//	if(duty_cycle_valve_cal >= 100)
				//	{
				//	}
						printf(" Pressure == %f  %f \n" , pressure_insp , i );
					 pthread_mutex_trylock(&mutex);
	                                pressure_insp = calc_pressure;
        	                        pthread_mutex_unlock(&mutex);
					flow_combine = flow_insp();
				*/
				//	printf(" In Exhale_loop  %d \n ", i);
					duty_cycle_valve_cal = 100;
					valve_duty_cycle(98);
					printf("==============Exhale exit=====================");
					break;
					if(duty_cycle_valve_cal >= 100)
					{
						break;
						printf("=======breakinggggggggggggggggggggggggggggggggg at Timer = \n");
					}
			   //      clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                            //    reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                             //   if(reading_timer > reading_set_time)
                             //   {
                               //           sprintf(c_string, "C@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final * -1,TiTOT);
                                 //         serial_data_write(c_string);    // Error Code
                                   //       printf(" Flow == %f \n" , flow_combine);
                                     //     clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);

                        //      }

			//		usleep(5000);
				//	pressure_Exhale = false;

				//	usleep(200000);
				}
/*				else if(exp_flag && ((int)pressure_insp >= (final_setting.peep)+2))
				{
					duty_cycle_valve_cal = duty_cycle_valve_cal - 1;
					if(duty_cycle_valve_cal <= 1)
                                        {
                                                duty_cycle_valve_cal = 1;
                                        }
					valve_duty_cycle(duty_cycle_valve_cal);

				}
*/
				clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                                if(reading_timer > reading_set_time)
                                {
					  sprintf(c_string, "C@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final * -1,TiTOT);
                                          serial_data_write(c_string);    // Error Code
				//	  printf(" Flow == %f \n" , flow_combine);
				          clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);

                                }
				if(!trigger_enable){
					clock_gettime(CLOCK_MONOTONIC_RAW, &end_time_exhale);
	                		exp_time =  delta_t(&start_time_exhale, &end_time_exhale,&deltat);
				}
				else
				{
					   clock_gettime(CLOCK_MONOTONIC_RAW, &simv_end_time_exhale);
  	                                   timer = delta_t(&simv_start_time_exhale, &simv_end_time_exhale,&deltat_reading);
				}
			//	usleep(1000);

			}
			valve_duty_cycle(98);
	//		printf(" Pip = %f \n" , pip);
	//		printf(" Pressure_insp  = %f \n", pressure_insp);
	//		printf(" TiTOT = %f \n",TiTOT);
//			mean_airway_pressure = ((pip - pressure_insp) * TiTOT/100.00) + pressure_insp ;
			TiTOT_slope = (insp_time * final_setting.slope /100.00)/(inhale_time*1000.00 + exhale_time*1000.00);
                        TiTOT_remain = (insp_time * (100 - final_setting.slope) /100.00)/(inhale_time*1000.00 + exhale_time*1000.00);
                        mean_airway_pressure_thread = ((pip - pressure_insp) * TiTOT_slope)  * 0.5;
                        mean_airway_pressure =  ((pip - pressure_insp) * TiTOT_remain) ;
                        mean_airway_pressure = mean_airway_pressure +mean_airway_pressure_thread + pressure_insp;

		//	printf(" the pressure = %f \n" , mean_airway_pressure);
		//	printf(" MVe == %f\n",MVe);
		//	printf(" Leak = %f \n" , leak);
	//		printf(" CMV_ exp _time  = %d\n" ,cmv_exp_time);

		}

	}

}





////////////////////////////////////////////////////////Volume  Modes //////////////////////////////////////////////////////////////////
//.....................................................................................................................................//
//.....................................................................................................................................//



void volume_modes() {
//	start_bus();
//	volume
	pthread_mutex_trylock(&mutex);
        thread_control_pressure = true;
        pthread_mutex_unlock(&mutex);
	volume_flag = false;
	struct timespec  reading_start = {0,0};
	struct timespec reading_end = {0,0};
	struct timespec deltat_reading = {0,0};
	int timer = 0;
	int reading_timer = 0;
	int duty_cycle = 0;
	int insp_time = 0;
	int exp_time =1000;
	int insp_trigger_time = 0;
	float pressure_insp = 0;
	const int reading_set_time = READING_TIME;
	float flow_combine = 0;
	int fio2_cal = 21;
	float PFRe_cal = 0;
	float MVe =0;
	float MVi =0 ;
	float leak = 0;
	float Ve = 0;
	float Vi = 0;
	int counter = 0;
	float check_rr;
	int RR_cal = 0;
	float final_rr = 0;
	float pip = 0;
	float rise_time = 0;
	bool rise_flag = false;
	bool rr_flag = false;
	float pmean = 0;
	float final_flow =0;
	float leak_flow = 0.0;
//	float Pmean_array[(int)PMEAN_ARRAY_SIZE];
	int Pmean_count = 0;
	double Pmean_add = 0;
	float Pmean = 0;
	float Pif_insp = 0;
	float Pef_exp = 0;
	float duty_cycle_valve_cal = 0;
	float TiTOT = 0.0;
	float trigger = 0;
	bool exp_flag = true;
	char A_string[] = {0};
	char b_string[] = {0};
	char c_string[] = {0};
	char d_string[] = {0};
	int current_mode = 0;
	Pmean_array =  (int*)malloc(300*sizeof(float));
	MVe_array = (int*)malloc(4*sizeof(float));
	MVi_array = (int*)malloc(4*sizeof(float));
	RR_array =  (int*)malloc(4*sizeof(int));			// Allocate array with the help of malloc
//	turbine_duty_cycle(0);
	clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
	clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
	bool trigger_flag = false;
	bool trigger_enable = false;
	bool trigger_mandatory = false;
	bool cmv =false;
	bool psv_flag = false;
	float mean_airway_pressure = 0;
	float TiTOT_slope =0;// insp_time * final_setting.slope /100.00;
        float TiTOT_remain = 0;// insp_time * (100 - final_setting.slope) /100.00;
        float mean_airway_pressure_thread = 0;
	float flow_cycle_per = 0.25;
	bool vc_ac_flag = false;
	bool patient_disconnection = false;
	float complience = 0;
	float new_pressure = 0;
////////////////////////////////////////
	struct timespec  hold_start_time = {0,0};
        struct timespec hold_end_time = {0,0};
        struct timespec delta_hold = {0,0};
	float hold_time =0;
	float cal_peep =0;
	float COMPLIENCE_pressure = final_setting.VTi/30.00 + final_setting.peep;
	////n duty_cycle = transfer_function((int)(final_setting.peep));
        ////n turbine_duty_cycle(duty_cycle);


	while(1)
	{
		read_settings();
		current_mode = mode_change();
       		if( current_mode/10 != mode_para/10)
        	{
                //	mode_para = current_mode;
                	break;
        	}
		trigger_mandatory = false;
		trigger_enable = false;
//		printf(" Pressure == %f Time == %d    %f\n" , pressure_insp ,exp_time,duty_cycle_valve_cal);
		switch(current_mode%10)
		{
			case 1 : trigger_flag =false ;cmv = true;psv_flag =false; vc_ac_flag = false;break;		// Trigger flag indicate for pc_simv
			case 2 : trigger_flag = true;cmv = false; psv_flag =false ;vc_ac_flag = false;break;		// psv_flag indicate that every trigger breath is on Pinsp
		//	case 3 : psv_flag =true; cmv = false;trigger_flag = true;vc_ac_flag = false;break;		//vc_ac_flag indicate that this is not flow cycled
			case 5 : psv_flag =true; cmv = false;trigger_flag = true;vc_ac_flag = true;break;
			default : printf(" We are working on it \n");break;
		}
		flow_combine = flow_insp(0x28);    //n
		clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                if(reading_timer > reading_set_time)
                {			pthread_mutex_trylock(&mutex);
                                	pressure_insp = calc_pressure;
					pthread_mutex_unlock(&mutex);
					sprintf(c_string, "C@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final * -1,TiTOT);
					serial_data_write(c_string);
                                        clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                }
		clock_gettime(CLOCK_MONOTONIC_RAW, &end_time_exhale);
		exp_time =  delta_t(&start_time_exhale, &end_time_exhale,&deltat);
		cmv_exp_time = exp_time;
		if(( trigger_flag && (flow_combine  > final_setting.Trigger)) && !patient_disconnection)
		{
			printf(" Trigger Enable \n ");
//			printf("inp 1 =  %d \n" , timer+insp_time);
//			printf("inp 1 =  %d \n" , inhale_time + exhale_time);
			if((exp_time ) > ((exhale_time) * 1000 * 0.80))
			{
				printf("  Mandatory Trigger \n");
				trigger_mandatory = true;

			}
			else
			{
				printf(" Normal trigger \n");
				trigger_enable = true;
				simv_exp_time = exp_time - simv_exp_time;

			}
		}
//		printf( " time == %d \n" , cmv_exp_time);
		if(first_time)
		 COMPLIENCE_pressure = final_setting.VTi/HEALTHY_COMPLIENCE + pressure_insp;										//
		if((((cmv_exp_time +insp_time) >= (inhale_time+exhale_time)*1000)) ||  man_breath_flag || trigger_enable || trigger_mandatory)
		{	//exp_time = timer;
			if(Ehold && !trigger_enable){
                        bcm2835_gpio_set(INHALE_VALVE);
                        int new_pres = final_setting.peep + final_setting.peep*HOLD_TUBINE_FACTOR;
                //      (new_pres > 70)?new_pres=70:new_press=new_press;// 
                        if(new_pres > 70)
                        {
                                new_pres = 70;
                        }
                        duty_cycle = transfer_function(new_pres);
                        //turbine_duty_cycle(duty_cycle);
			turbine_duty_cycle(duty_cycle);    //nidhi

                        clock_gettime(CLOCK_MONOTONIC_RAW, &hold_start_time);
                        clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);

		        hold_time = delta_t(&start_time, &end_time,&delta_hold);
                        while ( (hold_time <= (Ehold_time * 1000)))
                        {       flow_combine = flow_insp(0x28);   //n
                                pthread_mutex_trylock(&mutex);
                                pressure_insp = calc_pressure;
                          //      exp_pressure = exp_pressure_reading;
                                pthread_mutex_unlock(&mutex);
                                clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                                 if(reading_timer > reading_set_time)
                                {
                                        sprintf(A_string, "A@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final,trigger );
                                        serial_data_write(A_string);    // Error Code
                                        clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                                }
                                clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);
                                hold_time = delta_t(&hold_start_time, &hold_end_time,&delta_hold);
                        }
                        // nidhi valve_duty_cycle(98);
			turbine_duty_cycle(20);
                        bcm2835_gpio_clr(INHALE_VALVE);
                        Ehold = false;
                        Ehold_time = 0;
               		 }
			clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
			MVe_array[counter] = volume_final * final_setting.RR_rxd;
                        counter++;
                        if(counter >= 4 )
                        {

                                counter = 0;
                        }
                                                ///
                        if( rr_flag)
                        {
                                MVe = ((MVe_array[0] +MVe_array[1] +MVe_array[2] +MVe_array[3] )/4.0);
                                if(MVi <= 0 )
                                {
                                        MVi = 1;
                                }
                                leak_flow  = LEAK_FLow(MVi,MVe);//((MVi - (-1) *MVe)/MVi)*100;
                                if(leak_flow < 0)
                                        leak_flow = 0;
                        }
                        printf(" leak ========== %f\n",leak_flow);
                        if(leak_flow > 90){
                                patient_disconnection = true;
                                serial_data_write("ACK05");
					first_time =true;
                                }
                        else{
                                patient_disconnection = false;
                                serial_data_write("ACK06");

                            }

			Ve = volume_final;
			sprintf(d_string, "D@%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f#", pressure_insp , final_rr , 21 ,(-1)*Pef_exp ,(-1)*MVe/1000.00 ,  leak , 0.0 , Ve*-1 ,cmv_exp_time/1000.00);
			printf("%s  \n", d_string);
			serial_data_write(d_string);    // Error Code\
			Pef_exp = 0;
		//	printf(" Flag =====   %d \n" ,trigger_enable);
			if(trigger_enable)
			{	printf(" Triggered  pressure \n ");

//				duty_cycle = transfer_function((int)(final_setting.Psupp));
//				turbine_duty_cycle(duty_cycle);
			if(!psv_flag){
				pthread_mutex_trylock(&mutex);
                                slope_thread_global = true;
				trigger_thread = true;
                                pthread_mutex_unlock(&mutex);
				printf("PC-SIMV triggered breath\n");
	             //                   simv_exp_time = 0;
				}
			else
				{
				pthread_mutex_trylock(&mutex);
                                slope_thread_global = true;
                              //  trigger_thread = true;
                                pthread_mutex_unlock(&mutex);
				  printf("PSV triggered breath\n");
				}
			}
			else// if(trigger_enable)
			{	printf(" mandartory  pressure \n ");
				if(first_time){
				duty_cycle = transfer_function((int)(COMPLIENCE_pressure));
                                turbine_duty_cycle(duty_cycle);
				}
				else
				duty_cycle = transfer_function((int)(new_pressure));
                                turbine_duty_cycle(duty_cycle);

			//	pthread_mutex_trylock(&mutex);
                        //	slope_thread_global = true;
                       	//	pthread_mutex_unlock(&mutex);
				simv_exp_time = 0;
			}
			clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
			valve_duty_cycle(100);
			man_breath_flag = false;
			timer = 0;
			rise_flag = true;
			pip = 0;
			volume_final = 0;
			pthread_mutex_trylock(&mutex);
                        pressure_insp = calc_pressure;
                        pthread_mutex_unlock(&mutex);
			Pif_insp = 0;
			clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
			flow_combine = flow_insp(0x28);   //n
			if (flow_combine < 0){
				flow_combine=	flow_combine * -1;
			}
///			printf( " flow == %f \n " , flow_combine);
                        //        pip = max(pip , pressure_insp);
          //              Pif_insp = max(Pif_insp,flow_combine);
			clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
                        timer = delta_t(&start_time, &end_time,&deltat);

//			printf("  pif === %f \n ", Pif_insp);
		//	printf(" %f \n" , final_setting.Psupp);
	//		printf(" flow_combine  == %f Pif_insp == %f \n", flow_combine  , Pif_insp);
	//		printf(" Timer == %f \n", timer);
	//		printf(" Step 1 \n");
			while((volume_final <= final_setting.VTi ) && (timer < (inhale_time)* 1000))  // && (flow_combine > (Pif_insp *flow_cycle_per))))// || (!trigger_enable && psv_flag && (timer < (inhale_time)* 1000) && (flow_combine > (Pif_insp * flow_cycle_per)))  )
			{	//	printf(" After Error 1 \n");
			//	printf(" VOlume = %f \n" , volume_final);
				flow_combine = flow_insp(0x28);   //n
				pip = max(pip , pressure_insp);
				Pif_insp = max(Pif_insp,flow_combine);
				//printf("%f ,%d\n" , pressure_insp ,timer);
			//	printf(" flow_combine  == %f Pif_insp == %f \n", flow_combine  , Pif_insp * 0.25);
				if((flow_combine < 0)  && (flow_combine > -4))
					flow_combine = 1;
				if((flow_combine <= (int) (Pif_insp  * final_setting.insp_term)) && ( trigger_enable || psv_flag) && !vc_ac_flag && !slope_thread_global)
					{//	printf(" Breakinnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnng \n");
						break;
					}
				pthread_mutex_trylock(&mutex);
				pressure_insp = calc_pressure;
                		pthread_mutex_unlock(&mutex);
			//	A_string = itoa(pressure);
				clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
				reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
		//		 printf(" After Error 2 \n");
				if(trigger_enable || trigger_mandatory)
					trigger = 1.0;
				else
					trigger = 0.0;
			//	printf( " trigger = %f \n" , trigger);
				if(reading_timer > reading_set_time)
				{
				//	Pmean_array[Pmean_count] = pressure_insp;
				//	Pmean_count++;
					sprintf(A_string, "A@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final,trigger );
		//			printf("%s \n " , A_string);
					serial_data_write(A_string);	// Error Code
					clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
				}
		//		 printf(" After Error 3 \n");
				if((pressure_insp >= final_setting.Pinsp -1) && rise_flag)
				{
					clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
	                                rise_time = delta_t(&start_time, &end_time,&deltat);
					rise_flag = false;
				}
				else if(rise_flag)
					{
						rise_time = 0;
					}


				clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
				timer = delta_t(&start_time, &end_time,&deltat);
		//		printf("Timer: %d\n", timer);
			//	printf(" Pressure == %f Time == %d \n" , pressure_insp ,timer);
			}
		//	printf("Total time taken : %d\n", timer);
			pthread_mutex_trylock(&mutex);
                        slope_thread_global = false;
                        pthread_mutex_unlock(&mutex);
			if(!trigger_enable)
				insp_time = timer;
			else
				insp_trigger_time =timer;
			if(Ihold && !trigger_enable){
			bcm2835_gpio_set(INHALE_VALVE);
			int new_pres = final_setting.Pinsp + final_setting.Pinsp*HOLD_TUBINE_FACTOR;
		//	(new_pres > 70)?new_pres=70:new_press=new_press;// 
			if(new_pres > 70)
			{
				new_pres = 70;
			}
			duty_cycle = transfer_function(new_pres);
                        turbine_duty_cycle(duty_cycle);

			clock_gettime(CLOCK_MONOTONIC_RAW, &hold_start_time);
			clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);
                        hold_time = delta_t(&start_time, &end_time,&delta_hold);
			while ( (hold_time <= (Ihold_time * 1000)))
			{	flow_combine = flow_insp(0x28);   //n
				pthread_mutex_trylock(&mutex);
                                pressure_insp = calc_pressure;
//                                exp_pressure = exp_pressure_reading;
                                pthread_mutex_unlock(&mutex);
				clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
				 if(reading_timer > reading_set_time)
                                {
                                        sprintf(A_string, "A@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final,trigger );
                                        serial_data_write(A_string);    // Error Code
                                        clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                                }
				clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);
                        	hold_time = delta_t(&hold_start_time, &hold_end_time,&delta_hold);
			}
			duty_cycle = transfer_function(final_setting.peep);
                        turbine_duty_cycle(duty_cycle);

			valve_duty_cycle(0);
			bcm2835_gpio_clr(INHALE_VALVE);
			Ihold = false;
			Ihold_time = 0;
			}
			else
			valve_duty_cycle(98);
			printf(" After Error 4 \n");
		//	valve_duty_cycle(98);
                //	duty_cycle = transfer_function((int)(final_setting.peep));
                //	turbine_duty_cycle(duty_cycle);
			//rise_time = 0;
			if(!trigger_enable)
			{//	printf("Wait for joining \n");
			//	error(pthread_join(slope_thread,NULL));
			//	printf("Wait complete \n");
				printf(" Mandatory exhale timer  \n");
				clock_gettime(CLOCK_MONOTONIC_RAW, &start_time_exhale);
				clock_gettime(CLOCK_MONOTONIC_RAW, &end_time_exhale);
				exp_time = delta_t(&start_time_exhale, &end_time_exhale,&deltat_reading);
				printf(" Exp_time == %d  \n",exp_time);

			}
			else
			{	 printf(" After Error 5 \n");
				clock_gettime(CLOCK_MONOTONIC_RAW, &simv_start_time_exhale);
				clock_gettime(CLOCK_MONOTONIC_RAW, &simv_end_time_exhale);
                             	timer = delta_t(&simv_start_time_exhale, &simv_end_time_exhale,&deltat_reading);
			}
			//valve_duty_cycle(98);
                	duty_cycle = transfer_function((int)(final_setting.peep));
                	turbine_duty_cycle(duty_cycle);
			printf(" After Error 6 \n");
			if(trigger_enable){
				RR_cal =  (int) ((60*1000)/(insp_trigger_time + simv_exp_time));
                                check_rr = ((10*1000*60)/(insp_time * 1.00 + cmv_exp_time * 1.00)) - (10 * (int)RR_cal);
			}
			else
			{	RR_cal =  (int) ((60*1000)/(insp_time +cmv_exp_time));

				printf(" insp_time == %d  exp time === %d \n", insp_time , cmv_exp_time);
				printf(" RR == %d \n ", RR_cal);
				check_rr = ((10*1000*60)/(insp_time * 1.00 + cmv_exp_time * 1.00)) - (10 * (int)RR_cal);
			}
			 printf(" After Error 7 \n");
		        if(counter >= 3 )
                        {
                                rr_flag = true;
                        //        counter = 0;
                        }
			printf("Check RR = %f \n" , check_rr);
			if(check_rr > 0.5)
			{
				RR_cal = RR_cal + 1;
			}
			printf(" RR_cal after adding == %d \n" ,RR_cal);
			RR_array[counter] = RR_cal;
			for ( int i = 0; i < Pmean_count ;i++)
			{
				Pmean_add = Pmean_array[i] + Pmean_add ;
			}
			Pmean = (Pmean_add /(Pmean_count *1.0));
			Pmean_count = 0;
			Vi = volume_final;
			MVi_array[counter] = Vi *  final_setting.RR_rxd;
		//	printf("%d \n ", counter);
			if(rr_flag)
			{	final_rr = (RR_array[0]  +RR_array[1] +RR_array[2] +RR_array[3]) /4.0 ;
				MVi =( MVi_array[0] +MVi_array[1] +MVi_array[2] +MVi_array[3] )/4.00;
			}
		//	printf(" MVi == %f\n",MVi);
			volume_final = 0;

		//	Pmean = Pmean_add/(Pmean_count*1.0);
		//	 printf(" Pmean  == %f\n",Pmean_count);
			sprintf(b_string, "B@%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f#",pip , Vi , Pif_insp ,  Pmean, MVi/1000.0, 0.0,insp_time/1000.00 , pressure_insp,0,rise_time/1000.00);
			printf (" %s \n" , b_string);								// Pmean	TRIGGER_FLOW
                        serial_data_write(b_string);    // Error Code
			Pmean_add = 0;
			Pmean_count = 0;
			Pmean = 0;
			timer = 0;
			clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
			reading_timer =0;
			Pif_insp = 0;
			duty_cycle_valve_cal = 0;
			exp_flag = true;
			TiTOT = (insp_time/(inhale_time*1000.00 + exhale_time*1000.00)) *100;
			printf(" Ti/tot = %f \n" ,TiTOT);
////////////////////////////////////////////////Complieance////////////////////////////////////////
			if(first_time){
				complience = compliance_function(Vi , pip , final_setting.peep);
			 	new_pressure = final_setting.VTi / complience + final_setting.peep;
				printf(" New_ pressure == %f \n ", new_pressure);
				printf(" New === %f \n" , complience);
				first_time = false;
				printf( "1\n");
			}
			else if(setting_changed){
				new_pressure = final_setting.VTi / complience + final_setting.peep;
				printf( "2\n");
				setting_changed = false;

			}
		//	else if(first_time && (Vi <= (final_setting.VTi))){
			else if( (Vi <= (final_setting.VTi))){
				new_pressure = new_pressure + 1;
				printf(" pressure is incremented \n");
				printf( "3\n");

			}
			else if( (Vi >= (final_setting.VTi))){
		//	else if(first_time && (Vi >= (final_setting.VTi + 20))){
				new_pressure = new_pressure - 1;
				printf(" pressure is decremented \n");
				printf( "4\n");

			}
			printf(" Input  vti = %f  pip = %f peep = %f \n", Vi , pip , final_setting.peep);
			 printf(" New  pressure === %f \n" , new_pressure);

			if(new_pressure >= 80)
				new_pressure = 80;
			bool pressure_Exhale = true;
			while((!trigger_enable &&(exp_time < 300)) ||  (trigger_enable && (timer < (  300))))
			{
				flow_combine = flow_insp(0x28);   //n
				Pef_exp = min(Pef_exp,flow_combine);
		       		pthread_mutex_trylock(&mutex);
				pressure_insp = calc_pressure;
				pthread_mutex_unlock(&mutex);
			//	float Kp= 0.1;
			//	float Kc = 1.4;
				if((pressure_insp <= (final_setting.peep )) && pressure_Exhale) //&& (((exp_time >= 300.0)))) //&& !trigger_enable ) || ((timer >= 300.0)&& trigger_enable )))
				{//	exp_flag = false;
		/*		float i = 0;
				for ( i = 90.0; i<= 100.00 ; i = i +0.1)
				{	duty_cycle_valve_cal = i ; //duty_cycle_valve_cal + 1;
				//	modified_pressure = (set_peep - pressure)*0.2 * 2 + modified_pressure
				//	duty_cycle_valve_cal = Kp * Kc * duty_cycle_valve_cal + duty_cycle_valve_cal;
				//	if(duty_cycle_valve_cal >= 100)
				//	{
				//	}
						printf(" Pressure == %f  %f \n" , pressure_insp , i );
					 pthread_mutex_trylock(&mutex);
	                                pressure_insp = calc_pressure;
        	                        pthread_mutex_unlock(&mutex);
					flow_combine = flow_insp();
				*/
				//	printf(" In Exhale_loop  %d \n ", i);
					duty_cycle_valve_cal = 100;
					valve_duty_cycle(98);
					if(duty_cycle_valve_cal >= 100)
					{
						break;
						printf("breakinggggggggggggggggggggggggggggggggg at Timer = \n");
					}
			   //      clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                            //    reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                             //   if(reading_timer > reading_set_time)
                             //   {
                               //           sprintf(c_string, "C@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final * -1,TiTOT);
                                 //         serial_data_write(c_string);    // Error Code
                                   //       printf(" Flow == %f \n" , flow_combine);
                                     //     clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);

                        //        }

			//		usleep(5000);
					pressure_Exhale = false;

				//	usleep(200000);
				}
/*				else if(exp_flag && ((int)pressure_insp >= (final_setting.peep)+2))
				{
					duty_cycle_valve_cal = duty_cycle_valve_cal - 1;
					if(duty_cycle_valve_cal <= 1)
                                        {
                                                duty_cycle_valve_cal = 1;
                                        }
					valve_duty_cycle(duty_cycle_valve_cal);

				}
*/
				clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                                if(reading_timer > reading_set_time)
                                {
					  sprintf(c_string, "C@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final * -1,TiTOT);
                                          serial_data_write(c_string);    // Error Code
					  printf(" Flow == %f \n" , flow_combine);
				          clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);

                                }
				if(!trigger_enable){
					clock_gettime(CLOCK_MONOTONIC_RAW, &end_time_exhale);
	                		exp_time =  delta_t(&start_time_exhale, &end_time_exhale,&deltat);
				}
				else
				{
					   clock_gettime(CLOCK_MONOTONIC_RAW, &simv_end_time_exhale);
  	                                   timer = delta_t(&simv_start_time_exhale, &simv_end_time_exhale,&deltat_reading);
				}
			//	usleep(1000);

			}
			//valve_duty_cycle(100);
		/*	Ve = volume_final;
			MVe_array[counter] = volume_final * final_setting.RR_rxd;
			counter++;
			if(counter >= 4 )
                        {

                                counter = 0;
                        }
						///
			if( rr_flag)
			{
				MVe = ((MVe_array[0] +MVe_array[1] +MVe_array[2] +MVe_array[3] )/4.0);
				if(MVi <= 0 )
				{
					MVi = 1;
				}
				leak = ((MVi - (-1) *MVe)/MVi)*100;
                        	if(leak <= -1)
                                leak = 0;
			}
			*/
	//		printf(" Pip = %f \n" , pip);
	//		printf(" Pressure_insp  = %f \n", pressure_insp);
	//		printf(" TiTOT = %f \n",TiTOT);
//			mean_airway_pressure = ((pip - pressure_insp) * TiTOT/100.00) + pressure_insp ;
			TiTOT_slope = (insp_time * final_setting.slope /100.00)/(inhale_time*1000.00 + exhale_time*1000.00);
                        TiTOT_remain = (insp_time * (100 - final_setting.slope) /100.00)/(inhale_time*1000.00 + exhale_time*1000.00);
                        mean_airway_pressure_thread = ((pip - pressure_insp) * TiTOT_slope)  * 0.5;
                        mean_airway_pressure =  ((pip - pressure_insp) * TiTOT_remain) ;
                        mean_airway_pressure = mean_airway_pressure +mean_airway_pressure_thread + pressure_insp;
			//first_time = true;
			printf(" the pressure = %f \n" , mean_airway_pressure);
		//	printf(" MVe == %f\n",MVe);
		//	printf(" Leak = %f \n" , leak);
	//		printf(" CMV_ exp _time  = %d\n" ,cmv_exp_time);

		}

	}

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;///
void stand_by(){
	//    pwm_init();
	    int current_mode = 0;
	    turbine_duty_cycle(0);
	     pthread_mutex_trylock(&mutex);
             thread_control_pressure = false;
             pthread_mutex_unlock(&mutex);
	turbine_duty_cycle(0);

	    printf(" Standby mode \n");
	    while(1){
	//	read_setting()
                current_mode = mode_change();
                if( current_mode/10 != 9)
                {
                //      mode_para = current_mode;
                        	break;
                }

                //      mode_para = current_mode;
                        //	break;
		usleep(2000000);
          }
		////n bcm2835_close();
		 pwm_init();
		 ////n flow_init();  
//nidhi		turbine_constant();
		}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;//
void cpap(){
	pthread_mutex_trylock(&mutex);
        thread_control_pressure = true;
        pthread_mutex_unlock(&mutex);
        volume_flag = false;
	float insp_duty_cycle = 0;
	float valve_cycle = 100;
	//int y = 0;
	struct timespec  reading_start = {0,0};
        struct timespec reading_end = {0,0};
        struct timespec deltat_reading = {0,0};
	const int reading_set_time = READING_TIME;
	float previous_duty_cycle = 0;
	clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
	char A_string[] = {0};
	int reading_timer = 0;
        //clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
	float modified_pressure = 0;
	int current_mode = 0;
	float pressure_insp = 0;
	float flow_combine = 0;
	float peep_duty_cycle = 0;
	float modified_pressure_duty_cycle = 0;
	valve_duty_cycle(valve_cycle);
	insp_duty_cycle = transfer_function((int)(final_setting.peep));
        turbine_duty_cycle(insp_duty_cycle);
	usleep(1000000);
	int loop = 0;

	while(1){
		read_settings();
                current_mode = mode_change();
                if( current_mode/10 != mode_para/10)
                {
                //      mode_para = current_mode;
                        break;
                }
		flow_combine = flow_insp(0x28);  //n
                pthread_mutex_trylock(&mutex);
                pressure_insp = calc_pressure;
                pthread_mutex_unlock(&mutex);
		clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
		if(reading_timer > reading_set_time)
                {
                                       // Pmean_array[Pmean_count] = pressure_insp;
                                       // Pmean_count++;
                                        sprintf(A_string, "A@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,0,0 );
                //                      printf("%s \n " , A_string);
                                        serial_data_write(A_string);    // Error Code
                                        clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                }
		if( (int)(pressure_insp - final_setting.peep) >= 1)
		{
			//modified_pressure = final_setting.peep;
                        modified_pressure_duty_cycle = transfer_function(modified_pressure);
			peep_duty_cycle = transfer_function(final_setting.peep);
			if(modified_pressure_duty_cycle > peep_duty_cycle)
			{
                        //turbine_duty_cycle(insp_duty_cycle);
			valve_cycle = (final_setting.peep )*0.3 *(final_setting.peep - pressure_insp) + valve_cycle;
			if(valve_cycle > 100)
				valve_cycle = 100;
			else if(valve_cycle < 0)
				valve_cycle = 0;
			if((valve_cycle > 0) && (valve_cycle != previous_duty_cycle))
			{
				valve_duty_cycle(valve_cycle);
			}
			loop = 1;
			modified_pressure = final_setting.peep;
			insp_duty_cycle = transfer_function((int)(modified_pressure));
                        turbine_duty_cycle(insp_duty_cycle);
			}
			printf(" Loop 1 with single _condition\n");
		}
///////////////////////////////////// These conditions are new ////////////////
/*		else if(((pressure_insp) <= final_setting.peep * 0.6) && (flow_combine > 30)&& modified_pressure > (final_setting.peep)) {
                                modified_pressure =  final_setting.peep;//(final_setting.peep - pressure_insp)*0.2 * 2 + modified_pressure;
                                insp_duty_cycle = transfer_function((int)(modified_pressure));
                                turbine_duty_cycle(insp_duty_cycle);
                                //valve_cycle = (2 )*0.4*(set_peep - pressure) + x
                                valve_cycle = 100;
                                valve_duty_cycle(valve_cycle);
                                printf("Loop new modified with double _condition\n");

                }

		else if(((pressure_insp) <= final_setting.peep * 0.9) && (flow_combine > 30)){
                                modified_pressure = final_setting.peep ;//(final_setting.peep - pressure_insp)*0.2 * 2 + modified_pressure;
                                insp_duty_cycle = transfer_function((int)(modified_pressure));
                                turbine_duty_cycle(insp_duty_cycle);
                                //valve_cycle = (2 )*0.4*(set_peep - pressure) + x
                                valve_cycle = 100;
                                valve_duty_cycle(valve_cycle);
                                printf("Loop new without modified with double _condition\n");

                }
////////////////////////////////////////////////////////////////////////////
*/		else if((( final_setting.peep - pressure_insp) >= 1) && flow_combine > 10){
				modified_pressure = (final_setting.peep - pressure_insp)*0.2 * 2 + modified_pressure;
				insp_duty_cycle = transfer_function((int)(modified_pressure));
	                        turbine_duty_cycle(insp_duty_cycle);
				//valve_cycle = (2 )*0.4*(set_peep - pressure) + x
				valve_cycle = 0;
				valve_duty_cycle(valve_cycle);
				printf("Loop 2 with double _condition\n");

		}

/*		//////////////////////// Added
		else if(((modified_pressure - final_setting.peep) >= 1) && (valve_cycle < 100)){
				valve_cycle = 6 * 0.4*(pressure_insp - final_setting.peep) + valve_cycle;
				modified_pressure = (final_setting.peep - pressure_insp)*0.1*2 + modified_pressure; 
                                if(modified_pressure < final_setting.peep){
                                                modified_pressure = final_setting.peep;
                                                insp_duty_cycle = transfer_function(modified_pressure);
                                                turbine_duty_cycle(insp_duty_cycle);
                                                if(valve_cycle > 100)
                                                        valve_cycle =100;
                                                if(valve_cycle < 0 )
                                                        valve_cycle =0;
                                                if((valve_cycle >=0 )&& (valve_cycle != previous_duty_cycle))
                                                                valve_duty_cycle(valve_cycle);
                                }
			}
*/		////////////////////////////////
		else if ((( final_setting.peep - pressure_insp) >= 1) && flow_combine > -2){
			if ( flow_combine > -2){
				valve_cycle = 6 * 0.4*(final_setting.peep - pressure_insp) + valve_cycle;
				modified_pressure = (pressure_insp - final_setting.peep)*0.1*2 + modified_pressure; 
				if(modified_pressure < final_setting.peep){
                                                modified_pressure = final_setting.peep;
                                                insp_duty_cycle = transfer_function(modified_pressure);
                                                turbine_duty_cycle(insp_duty_cycle);
						if(valve_cycle > 100)
							valve_cycle =100;
						if(valve_cycle < 0 )
							valve_cycle =0;
						if((valve_cycle >=0 )&& (valve_cycle != previous_duty_cycle))
								valve_duty_cycle(valve_cycle);
				}
				loop = 2;
			}
			printf("Loop 3 with double _condition\n");
		}
		else if((final_setting.peep - pressure_insp) >=1){
				modified_pressure = ( final_setting.peep - pressure_insp)*0.2 * 2 + modified_pressure;
                                insp_duty_cycle = transfer_function(modified_pressure);
                                turbine_duty_cycle(insp_duty_cycle);
                                loop = 3;
			printf("Loop 4 with single _condition\n");
		}
		usleep(10000);
		previous_duty_cycle = valve_cycle;
	//	printf(" Modified  == %f Valve cuycle = %f loop = %d  pressure = %f flow = %f \n" , modified_pressure , valve_cycle,loop,pressure_insp,flow_combine);
		//printf(" %f \n", flow_combine);
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Bpap_address
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void bpap(){
	pthread_mutex_trylock(&mutex);
        thread_control_pressure = true;
        pthread_mutex_unlock(&mutex);

	volume_flag = false;
	struct timespec  reading_start = {0,0};
	struct timespec reading_end = {0,0};
	struct timespec deltat_reading = {0,0};
	int timer = 0;
	int reading_timer = 0;
	int duty_cycle = 0;
	int insp_time = 0;
	int exp_time =1000;
	int insp_trigger_time = 0;
	float pressure_insp = 0;
	const int reading_set_time = READING_TIME;
	float flow_combine = 0;
	int fio2_cal = 21;
	float PFRe_cal = 0;
	float MVe =0;
	float MVi =0 ;
	float leak = 0;
	float Ve = 0;
	float Vi = 0;
	int counter = 0;
	float check_rr;
	int RR_cal = 0;
	float final_rr = 0;
	float pip = 0;
	float rise_time = 0;
	bool rise_flag = false;
	bool rr_flag = false;
	float pmean = 0;
	float final_flow =0;
	float leak_flow = 0.0;
//	float Pmean_array[(int)PMEAN_ARRAY_SIZE];
	int Pmean_count = 0;
	double Pmean_add = 0;
	float Pmean = 0;
	float Pif_insp = 0;
	float Pef_exp = 0;
	float duty_cycle_valve_cal = 0;
	float TiTOT = 0.0;
	float trigger = 0;
	bool exp_flag = true;
	char A_string[] = {0};
	char b_string[] = {0};
	char c_string[] = {0};
	char d_string[] = {0};
	int current_mode = 0;
	Pmean_array =  (int*)malloc(300*sizeof(float));
	MVe_array = (int*)malloc(4*sizeof(float));
	MVi_array = (int*)malloc(4*sizeof(float));
	RR_array =  (int*)malloc(4*sizeof(int));			// Allocate array with the help of malloc
//	turbine_duty_cycle(0);
	clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
	clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
	bool trigger_flag = false;
	bool trigger_enable = false;
	bool trigger_mandatory = false;
	bool cmv =false;
	bool psv_flag = false;
	float mean_airway_pressure = 0;
	float TiTOT_slope =0;// insp_time * final_setting.slope /100.00;
        float TiTOT_remain = 0;// insp_time * (100 - final_setting.slope) /100.00;
        float mean_airway_pressure_thread = 0;
	float flow_cycle_per = 0.25;
	bool pc_ac_flag = false;
	bool patient_disconnection = false;
	//float leak_flow = 0;
	float exp_pressure = 0;
	while(1)
	{
		read_settings();
		current_mode = mode_change();
       		if( current_mode/10 != mode_para/10)
        	{
                //	mode_para = current_mode;
                	break;
        	}
		trigger_mandatory = false;
		trigger_enable = false;
//		printf(" Pressure == %f Time == %d    %f\n" , pressure_insp ,exp_time,duty_cycle_\valve_cal);
		switch(current_mode%10)
		{
		//	case 1 : trigger_flag =false ;cmv = true;psv_flag =false; pc_ac_flag = false;break;		// Trigger flag indicate for pc_simv
			case 2 : trigger_flag = true;cmv = false; psv_flag =false ;pc_ac_flag = false;break;		// psv_flag indicate that every trigger breath is on Pinsp
		//	case 3 : psv_flag =true; cmv = false;trigger_flag = true;pc_ac_flag = false;break;		//pc_ac_flag indicate that this is not flow cycled
		//	case 7 : psv_flag =true; cmv = false;trigger_flag = true;pc_ac_flag = true;break;
			default : printf(" We are working on it \n");break;
		}
		flow_combine = flow_insp(0x28);   //n
		clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                if(reading_timer > reading_set_time)
                {			pthread_mutex_trylock(&mutex);
                                	pressure_insp = calc_pressure;
					exp_pressure = exp_pressure_reading;
                                	pthread_mutex_unlock(&mutex);
                                //	printf("insp pressure  = %f  Exp pressure = %f \n", pressure_insp ,exp_pressure);

				//	pthread_mutex_unlock(&mutex);
					sprintf(c_string, "C@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final * -1,TiTOT);
					serial_data_write(c_string);
                                        clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                }
		clock_gettime(CLOCK_MONOTONIC_RAW, &end_time_exhale);
		exp_time =  delta_t(&start_time_exhale, &end_time_exhale,&deltat);
		cmv_exp_time = exp_time;
		if(( trigger_flag && (flow_combine  > final_setting.Trigger)) && !patient_disconnection)
		{
			printf(" Trigger Enable \n ");
//			printf("inp 1 =  %d \n" , timer+insp_time);
//			printf("inp 1 =  %d \n" , inhale_time + exhale_time);
			if((exp_time ) > ((exhale_time) * 1000 * 0.80))
			{
				printf("  Mandatory Trigger \n");
				trigger_mandatory = true;

			}
			else
			{
				printf(" Normal trigger \n");
				trigger_enable = true;
				simv_exp_time = exp_time - simv_exp_time;

			}
		}
	//	printf( " time == %d \n" , cmv_exp_time);
		if((((cmv_exp_time +insp_time) >= (inhale_time+exhale_time)*1000)) ||  man_breath_flag || trigger_enable || trigger_mandatory)
		{	//exp_time = timer;
			clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);

			MVe_array[counter] = volume_final * final_setting.RR_rxd;
			counter++;
			if(counter >= 4 )
                        {

                                counter = 0;
                        }
						///
			if( rr_flag)
			{
				MVe = ((MVe_array[0] +MVe_array[1] +MVe_array[2] +MVe_array[3] )/4.0);
				if(MVi <= 0 )
				{
					MVi = 1;
				}
				leak_flow  = LEAK_FLow(MVi,MVe);//((MVi - (-1) *MVe)/MVi)*100;
                        	if(leak_flow < 0)
					leak_flow = 0;
			}
			printf(" leak ========== %f\n",leak_flow);
			if(leak_flow > 90){
				patient_disconnection = true;
				serial_data_write("ACK00");
				}
			else{
				patient_disconnection = false;
				serial_data_write("ACK01");
			    }
			Ve = volume_final;
			Exhale_flag = false;										//
			sprintf(d_string, "D@%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f#", round(pressure_insp) , final_rr , 21 ,(-1)*Pef_exp ,(-1)*MVe/1000.00 ,  leak ,mean_airway_pressure , Ve*-1 ,cmv_exp_time/1000.00);
			printf("%s  \n", d_string);
			serial_data_write(d_string);    // Error Code
					Pef_exp = 0;
		//	printf(" Flag =====   %d \n" ,trigger_enable);
			if(trigger_enable)
			{	printf(" Triggered  pressure \n ");
				duty_cycle = transfer_function((int)(final_setting.Psupp));
				turbine_duty_cycle(duty_cycle);
		//	if(!psv_flag){
		//		pthread_mutex_trylock(&mutex);
                  //              slope_thread_global = true;
		//		trigger_thread = true;
                  //              pthread_mutex_unlock(&mutex);
				printf("PC-SIMV triggered breath\n");
	             //                   simv_exp_time = 0;
				}
		//	else
		//		{
		//		pthread_mutex_trylock(&mutex);
                  //              slope_thread_global = true;
                              //  trigger_thread = true;
                    //            pthread_mutex_unlock(&mutex);
		//		printf("PSV triggered breath\n");
		//		}
		//	}
			else// if(trigger_enable)
			{	printf(" mandartory  pressure \n ");
				duty_cycle = transfer_function((int)(final_setting.Pinsp));
                               turbine_duty_cycle(duty_cycle);
			//	pthread_mutex_trylock(&mutex);
                        //	slope_thread_global = true;
                       	//	pthread_mutex_unlock(&mutex);
				simv_exp_time = 0;
			}
			clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
			valve_duty_cycle(100);
			man_breath_flag = false;
			timer = 0;
			rise_flag = true;
			pip = 0;
			volume_final = 0;
			pthread_mutex_trylock(&mutex);
                        pressure_insp = calc_pressure;
                        pthread_mutex_unlock(&mutex);
			Pif_insp = 0;
			clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
			flow_combine = flow_insp(0x28);   //n
			if (flow_combine < 0){
				flow_combine=	flow_combine * -1;
			}
///			printf( " flow == %f \n " , flow_combine);
                        //        pip = max(pip , pressure_insp);
          //              Pif_insp = max(Pif_insp,flow_combine);
			clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
                        timer = delta_t(&start_time, &end_time,&deltat);

//			printf("  pif === %f \n ", Pif_insp);
		//	printf(" %f \n" , final_setting.Psupp);
	//		printf(" flow_combine  == %f Pif_insp == %f \n", flow_combine  , Pif_insp);
	//		printf(" Timer == %f \n", timer);
	//		printf(" Step 1 \n");
			while((timer < (inhale_time)* 1000))  // && (flow_combine > (Pif_insp *flow_cycle_per))))// || (!trigger_enable && psv_flag && (timer < (inhale_time)* 1000) && (flow_combine > (Pif_insp * flow_cycle_per)))  )
			{	//	printf(" After Error 1 \n");
				flow_combine = flow_insp(0x28);   //n
				pip = max(pip , pressure_insp);
				Pif_insp = max(Pif_insp,flow_combine);
				//printf("%f ,%d\n" , pressure_insp ,timer);
			//	printf(" flow_combine  == %f Pif_insp == %f \n", flow_combine  , Pif_insp * 0.25);
		//		if((flow_combine < 0)  && (flow_combine > -4))
		//			flow_combine = 1;
				if((flow_combine <= (int) (Pif_insp  * 0.25)) && ( trigger_enable))
					{//	printf(" Breakinnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnng \n");
						break;
				}
				pthread_mutex_trylock(&mutex);
				pressure_insp = calc_pressure;
				exp_pressure = exp_pressure_reading;
                		pthread_mutex_unlock(&mutex);
			//	printf(" insp pressure  = %f Exp pressure = %f \n", pressure_insp ,exp_pressure);
			//	A_string = itoa(pressure);
				clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
				reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
		//		 printf(" After Error 2 \n");
				if(trigger_enable || trigger_mandatory)
					trigger = 1.0;
				else
					trigger = 0.0;
			//	printf( " trigger = %f \n" , trigger);
				if(reading_timer > reading_set_time)
				{
					Pmean_array[Pmean_count] = pressure_insp;
					Pmean_count++;
					sprintf(A_string, "A@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final,trigger );
		//			printf("%s \n " , A_string);
					serial_data_write(A_string);	// Error Code
					clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
				}
		//		 printf(" After Error 3 \n");
				if((pressure_insp >= final_setting.Pinsp -1) && rise_flag)
				{
					clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
	                                rise_time = delta_t(&start_time, &end_time,&deltat);
					rise_flag = false;
				}
				else if(rise_flag)
					{
						rise_time = 0;
					}


				clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
				timer = delta_t(&start_time, &end_time,&deltat);
		//		printf("Timer: %d\n", timer);
			//	printf(" Pressure == %f Time == %d \n" , pressure_insp ,timer);
			}
			pthread_mutex_trylock(&mutex);
                        slope_thread_global = false;
                        pthread_mutex_unlock(&mutex);

		//	printf(" After Error 4 \n");
			//rise_time = 0;
			if(!trigger_enable)
			{//	printf("Wait for joining \n");
			//	error(pthread_join(slope_thread,NULL));
			//	printf("Wait complete \n");
				printf(" Mandatory exhale timer  \n");
				clock_gettime(CLOCK_MONOTONIC_RAW, &start_time_exhale);
				clock_gettime(CLOCK_MONOTONIC_RAW, &end_time_exhale);
				exp_time = delta_t(&start_time_exhale, &end_time_exhale,&deltat_reading);
				printf(" Exp_time == %d  \n",exp_time);

			}
			else
			{	 printf(" After Error 5 \n");
				clock_gettime(CLOCK_MONOTONIC_RAW, &simv_start_time_exhale);
				clock_gettime(CLOCK_MONOTONIC_RAW, &simv_end_time_exhale);
                             	timer = delta_t(&simv_start_time_exhale, &simv_end_time_exhale,&deltat_reading);
			}
			if(!trigger_enable)
				insp_time = timer;
			else
				insp_trigger_time =timer;
			valve_duty_cycle(98);
                	duty_cycle = transfer_function((int)(final_setting.peep));
                	turbine_duty_cycle(duty_cycle);
			 //printf(" After Error 6 \n");
			if(trigger_enable){
				RR_cal =  (int) ((60*1000)/(insp_trigger_time + simv_exp_time));
                                check_rr = ((10*1000*60)/(insp_time * 1.00 + cmv_exp_time * 1.00)) - (10 * (int)RR_cal);
			}
			else
			{	RR_cal =  (int) ((60*1000)/(insp_time +cmv_exp_time));

			//	printf(" insp_time == %d  exp time === %d \n", insp_time , cmv_exp_time);
			//	printf(" RR == %d \n ", RR_cal);
				check_rr = ((10*1000*60)/(insp_time * 1.00 + cmv_exp_time * 1.00)) - (10 * (int)RR_cal);
			}
			// printf(" After Error 7 \n");
		        if(counter >= 3 )
                        {
                                rr_flag = true;
                        //        counter = 0;
                        }
		//	printf("Check RR = %f \n" , check_rr);
			if(check_rr > 0.5)
			{
				RR_cal = RR_cal + 1;
			}
		//	printf(" RR_cal after adding == %d \n" ,RR_cal);
			RR_array[counter] = RR_cal;
			for ( int i = 0; i < Pmean_count ;i++)
			{
				Pmean_add = Pmean_array[i] + Pmean_add ;
			}
			Pmean = (Pmean_add /(Pmean_count *1.0));
			Pmean_count = 0;
			Vi = volume_final;
			MVi_array[counter] = Vi *  final_setting.RR_rxd;
		//	printf("%d \n ", counter);
			if(rr_flag)
			{	final_rr = (RR_array[0]  +RR_array[1] +RR_array[2] +RR_array[3]) /4.0 ;
				MVi =( MVi_array[0] +MVi_array[1] +MVi_array[2] +MVi_array[3] )/4.00;
			}
		//	printf(" MVi == %f\n",MVi);
			volume_final = 0;

		//	Pmean = Pmean_add/(Pmean_count*1.0);
		//	 printf(" Pmean  == %f\n",Pmean_count);
			sprintf(b_string, "B@%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f#",round(pip) , Vi , Pif_insp ,  Pmean, MVi/1000.0, 0.0,insp_time/1000.00 , pressure_insp,0,rise_time/1000.00);
			printf (" %s \n" , b_string);								// Pmean	TRIGGER_FLOW
                        serial_data_write(b_string);    // Error Code
			Pmean_add = 0;
			Pmean_count = 0;
			Pmean = 0;
			timer = 0;
			clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
			reading_timer =0;
			Pif_insp = 0;
			duty_cycle_valve_cal = 0;
			exp_flag = true;
			TiTOT = (insp_time/(inhale_time*1000.00 + exhale_time*1000.00)) *100;
		//	printf(" Ti/tot = %f \n" ,TiTOT);
			Exhale_flag = true;
			while((!trigger_enable &&(exp_time <  300)) ||  (trigger_enable && (timer < (300))))
			{
				flow_combine = flow_insp(0x28);   //n
				Pef_exp = min(Pef_exp,flow_combine);
		       		pthread_mutex_trylock(&mutex);
				pressure_insp = calc_pressure;
				exp_pressure = exp_pressure_reading;
				pthread_mutex_unlock(&mutex);
			//	printf(" insp pressure  = %f  Exp pressure = %f \n", pressure_insp ,exp_pressure);
			//	float Kp= 0.1;
			//	float Kc = 1.4;
				if((pressure_insp <= (final_setting.peep ))) //&& (((exp_time >= 300.0)))) //&& !trigger_enable ) || ((timer >= 300.0)&& trigger_enable )))
				{//	exp_flag = false;
		/*		float i = 0;
				for ( i = 90.0; i<= 100.00 ; i = i +0.1)
				{	duty_cycle_valve_cal = i ; //duty_cycle_valve_cal + 1;
				//	modified_pressure = (set_peep - pressure)*0.2 * 2 + modified_pressure
				//	duty_cycle_valve_cal = Kp * Kc * duty_cycle_valve_cal + duty_cycle_valve_cal;
				//	if(duty_cycle_valve_cal >= 100)
				//	{
				//	}
						printf(" Pressure == %f  %f \n" , pressure_insp , i );
					 pthread_mutex_trylock(&mutex);
	                                pressure_insp = calc_pressure;
        	                        pthread_mutex_unlock(&mutex);
					flow_combine = flow_insp();
				*/
				//	printf(" In Exhale_loop  %d \n ", i);
					duty_cycle_valve_cal = 100;
					valve_duty_cycle(98);
					if(duty_cycle_valve_cal >= 100)
					{
						break;
						printf("breakinggggggggggggggggggggggggggggggggg at Timer = \n");
					}
			   //      clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                            //    reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                             //   if(reading_timer > reading_set_time)
                             //   {
                               //           sprintf(c_string, "C@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final * -1,TiTOT);
                                 //         serial_data_write(c_string);    // Error Code
                                   //       printf(" Flow == %f \n" , flow_combine);
                                     //     clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);

                        //        }

			//		usleep(5000);
				//	pressure_Exhale = false;

				//	usleep(200000);
				}
/*				else if(exp_flag && ((int)pressure_insp >= (final_setting.peep)+2))
				{
					duty_cycle_valve_cal = duty_cycle_valve_cal - 1;
					if(duty_cycle_valve_cal <= 1)
                                        {
                                                duty_cycle_valve_cal = 1;
                                        }
					valve_duty_cycle(duty_cycle_valve_cal);

				}
*/
				clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                                if(reading_timer > reading_set_time)
                                {
					  sprintf(c_string, "C@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final * -1,TiTOT);
                                          serial_data_write(c_string);    // Error Code
				//	  printf(" Flow == %f \n" , flow_combine);
				          clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);

                                }
				if(!trigger_enable){
					clock_gettime(CLOCK_MONOTONIC_RAW, &end_time_exhale);
	                		exp_time =  delta_t(&start_time_exhale, &end_time_exhale,&deltat);
				}
				else
				{
					   clock_gettime(CLOCK_MONOTONIC_RAW, &simv_end_time_exhale);
  	                                   timer = delta_t(&simv_start_time_exhale, &simv_end_time_exhale,&deltat_reading);
				}
			//	usleep(1000);

			}
			//valve_duty_cycle(100);
	//		printf(" Pip = %f \n" , pip);
	//		printf(" Pressure_insp  = %f \n", pressure_insp);
	//		printf(" TiTOT = %f \n",TiTOT);
//			mean_airway_pressure = ((pip - pressure_insp) * TiTOT/100.00) + pressure_insp ;
			TiTOT_slope = (insp_time * final_setting.slope /100.00)/(inhale_time*1000.00 + exhale_time*1000.00);
                        TiTOT_remain = (insp_time * (100 - final_setting.slope) /100.00)/(inhale_time*1000.00 + exhale_time*1000.00);
                        mean_airway_pressure_thread = ((pip - pressure_insp) * TiTOT_slope)  * 0.5;
                        mean_airway_pressure =  ((pip - pressure_insp) * TiTOT_remain) ;
                        mean_airway_pressure = mean_airway_pressure +mean_airway_pressure_thread + pressure_insp;

		//	printf(" the pressure = %f \n" , mean_airway_pressure);
		//	printf(" MVe == %f\n",MVe);
		//	printf(" Leak = %f \n" , leak);
	//		printf(" CMV_ exp _time  = %d\n" ,cmv_exp_time);

		}

	}
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//.....................................................................................................................................................///
//......................................................................................................................................................//
//....................................................................................................................................................../
void APRV_mode(){
	pthread_mutex_trylock(&mutex);
        thread_control_pressure = true;
        pthread_mutex_unlock(&mutex);

	volume_flag = false;
	struct timespec  reading_start = {0,0};
	struct timespec reading_end = {0,0};
	struct timespec deltat_reading = {0,0};
	int timer = 0;
	int reading_timer = 0;
	int duty_cycle = 0;
	int insp_time = 0;
	int exp_time =1000;
	int insp_trigger_time = 0;
	float pressure_insp = 0;
	const int reading_set_time = READING_TIME;
	float flow_combine = 0;
	int fio2_cal = 21;
	float PFRe_cal = 0;
	float MVe =0;
	float MVi =0 ;
	float leak = 0;
	float Ve = 0;
	float Vi = 0;
	int counter = 0;
	float check_rr;
	int RR_cal = 0;
	float final_rr = 0;
	float pip = 0;
	float rise_time = 0;
	bool rise_flag = false;
	bool rr_flag = false;
	float pmean = 0;
	float final_flow =0;
	float leak_flow = 0.0;
//	float Pmean_array[(int)PMEAN_ARRAY_SIZE];
	int Pmean_count = 0;
	double Pmean_add = 0;
	float Pmean = 0;
	float Pif_insp = 0;
	float Pef_exp = 0;
	float duty_cycle_valve_cal = 0;
	float TiTOT = 0.0;
	float trigger = 0;
	bool exp_flag = true;
	char A_string[] = {0};
	char b_string[] = {0};
	char c_string[] = {0};
	char d_string[] = {0};
	int current_mode = 0;
	Pmean_array =  (int*)malloc(300*sizeof(float));
	MVe_array = (int*)malloc(4*sizeof(float));
	MVi_array = (int*)malloc(4*sizeof(float));
	RR_array =  (int*)malloc(4*sizeof(int));			// Allocate array with the help of malloc
//	turbine_duty_cycle(0);
	clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
	clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
	bool trigger_flag = false;
	bool trigger_enable = false;
	bool trigger_mandatory = false;
	bool cmv =false;
	bool psv_flag = false;
	float mean_airway_pressure = 0;
	float TiTOT_slope =0;// insp_time * final_setting.slope /100.00;
        float TiTOT_remain = 0;// insp_time * (100 - final_setting.slope) /100.00;
        float mean_airway_pressure_thread = 0;
	float flow_cycle_per = 0.25;
	bool pc_ac_flag = false;
	bool patient_disconnection = false;
	//float leak_flow = 0;
	float exp_pressure = 0;
//////////////////////////////////////////////////////////////////////////////////
	struct timespec  hold_start_time = {0,0};
        struct timespec hold_end_time = {0,0};
        struct timespec delta_hold = {0,0};
	float hold_time =0;
	float cal_peep =0;
	float previous_duty_cycle = 0;
	float modified_pressure = 0;
	int loop =0;
	float insp_duty_cycle = 0;
	float valve_cycle = 0;
	bool algo_flag = false;
	int  delay_in_ms_algo = 100;//in ms
	float  multiplier = 0;
	while(1)
	{
		read_settings();
		current_mode = mode_change();
       		if( current_mode/10 != mode_para/10)
        	{
                //	mode_para = current_mode;
                	break;
        	}
		trigger_mandatory = false;
		trigger_enable = false;
		flow_combine = flow_insp(0x28);   //n
		clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                if(reading_timer > reading_set_time)
                {			pthread_mutex_trylock(&mutex);
                                	pressure_insp = calc_pressure;
					exp_pressure = exp_pressure_reading;
                                	pthread_mutex_unlock(&mutex);
                                //	printf("insp pressure  = %f  Exp pressure = %f \n", pressure_insp ,exp_pressure);

				//	pthread_mutex_unlock(&mutex);
					sprintf(c_string, "C@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final * -1,TiTOT);
					serial_data_write(c_string);
                                        clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                }
		clock_gettime(CLOCK_MONOTONIC_RAW, &end_time_exhale);
		exp_time =  delta_t(&start_time_exhale, &end_time_exhale,&deltat);
		cmv_exp_time = exp_time;


	//	printf( " time == %d \n" , cmv_exp_time);
		if((((cmv_exp_time +insp_time) >= (inhale_time+final_setting.Tlow)*1000)) ||  man_breath_flag )//|| trigger_enable || trigger_mandatory)
		{	//exp_time = timer;
			algo_flag = false;

			if(Ehold && !trigger_enable){
                        bcm2835_gpio_set(INHALE_VALVE);
                        int new_pres = final_setting.peep + final_setting.peep*HOLD_TUBINE_FACTOR;
                //      (new_pres > 70)?new_pres=70:new_press=new_press;// 
                        if(new_pres > 70)
                        {
                                new_pres = 70;
                        }
                        duty_cycle = transfer_function(new_pres);
                        turbine_duty_cycle(duty_cycle);

                        clock_gettime(CLOCK_MONOTONIC_RAW, &hold_start_time);
                        clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);

		        hold_time = delta_t(&start_time, &end_time,&delta_hold);
                        while ( (hold_time <= (Ehold_time * 1000)))
                        {       flow_combine = flow_insp(0x28);   //n
                                pthread_mutex_trylock(&mutex);
                                pressure_insp = calc_pressure;
                                exp_pressure = exp_pressure_reading;
                                pthread_mutex_unlock(&mutex);
                                clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                                 if(reading_timer > reading_set_time)
                                {
                                        sprintf(A_string, "A@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final,trigger );
                                        serial_data_write(A_string);    // Error Code
                                        clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                                }
                                clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);
                                hold_time = delta_t(&hold_start_time, &hold_end_time,&delta_hold);
                        }
                        valve_duty_cycle(100);
                        bcm2835_gpio_clr(INHALE_VALVE);
                        Ehold = false;
                        Ehold_time = 0;
               		}
			clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
			valve_duty_cycle(100);
			MVe_array[counter] = volume_final * final_setting.RR_rxd;
			counter++;
			if(counter >= 4 )
                        {

                                counter = 0;
                        }
						///
			if( rr_flag)
			{
				MVe = ((MVe_array[0] +MVe_array[1] +MVe_array[2] +MVe_array[3] )/4.0);
				if(MVi <= 0 )
				{
					MVi = 1;
				}
				leak_flow  = LEAK_FLow(MVi,MVe);//((MVi - (-1) *MVe)/MVi)*100;
                        	if(leak_flow < 0)
					leak_flow = 0;
			}
			printf(" leak ========== %f\n",leak_flow);
			if(leak_flow > 90){
				patient_disconnection = true;
				serial_data_write("ACK00");
				}
			else{
				patient_disconnection = false;
				serial_data_write("ACK01");
			    }
			Ve = volume_final;
			printf(" before compensation %f \n",Ve);

			Ve = Ve+1.2*(pip-cal_peep);
			printf(" after compensation %f \n",Ve);
			cal_peep = pressure_insp;
			Exhale_flag = false;										//
			sprintf(d_string, "D@%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f#", round(pressure_insp) , final_rr , 21 ,(-1)*Pef_exp ,(-1)*MVe/1000.00 ,  leak ,mean_airway_pressure , Ve*-1 ,cmv_exp_time/1000.00);
			printf("%s  \n", d_string);
			printf("data ============== %d \n ", strlen(d_string));
			serial_data_write(d_string);    // Error Code
			Pef_exp = 0;
			multiplier = 0;
		//	pthread_mutex_trylock(&mutex);
                  //      slope_thread_global = true;
                   //    	pthread_mutex_unlock(&mutex);
		//	simv_exp_time = 0;
			insp_duty_cycle = transfer_function((int)(final_setting.Pinsp));
                        turbine_duty_cycle(insp_duty_cycle);
			clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
			valve_duty_cycle(100);
			man_breath_flag = false;
			timer = 0;
			rise_flag = true;
			pip = 0;
			Pmean_count = 0;
			volume_final = 0;
			pthread_mutex_trylock(&mutex);
                        pressure_insp = calc_pressure;
                        pthread_mutex_unlock(&mutex);
			Pif_insp = 0;
			clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
			flow_combine = flow_insp(0x28);   //n
			clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
                        timer = delta_t(&start_time, &end_time,&deltat);
			while((timer < (inhale_time)* 1000))  // && (flow_combine > (Pif_insp *flow_cycle_per))))// || (!trigger_enable && psv_flag && (timer < (inhale_time)* 1000) && (flow_combine > (Pif_insp * flow_cycle_per)))  )
			{	//	printf(" After Error 1 \n");
				flow_combine = flow_insp(0x28);   //n
				pip = max(pip , pressure_insp);
				pthread_mutex_trylock(&mutex);
				pressure_insp = calc_pressure;
				exp_pressure = exp_pressure_reading;
                		pthread_mutex_unlock(&mutex);
//				printf("%f ,%d\n" , flow_combine ,timer);
			//	printf(" flow_combine  == %f Pressure == %f \n", flow_combine  ,pressure_insp);

				clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
				reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
		//		 printf(" After Error 2 \n");
				if(reading_timer > reading_set_time)
				{
				//	Pmean_array[Pmean_count] = pressure_insp;
				//	Pmean_count++;
					sprintf(A_string, "A@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final,trigger );
		//			printf("%s \n " , A_string);
					serial_data_write(A_string);	// Error Code
					clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
				}
		//		printf(" After Error 3 \n");
				if((pressure_insp >= final_setting.Pinsp -1) && rise_flag)
				{
					clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
	                                rise_time = delta_t(&start_time, &end_time,&deltat);
					rise_flag = false;
				}
				else if(rise_flag)
					{
						rise_time = 0;
					}


				clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
				timer = delta_t(&start_time, &end_time,&deltat);

	//////////////////////////////////////////////////////Algo start
	//printf(" abs %f \ ",abs((final_setting.Pinsp - pressure_insp)));
	if((((pressure_insp - final_setting.Pinsp)) >= -1) && !algo_flag)
	{
		printf(" Alo started \n");
		algo_flag = true; 
	}
//	algo_flag = false;
	if(algo_flag && (timer > (delay_in_ms_algo * multiplier))){
		 if( (int)(pressure_insp - final_setting.Pinsp) >= 2)
                {	multiplier++;
                        valve_cycle = (final_setting.Pinsp )*0.3 *(final_setting.Pinsp - pressure_insp) + valve_cycle;
                        if(valve_cycle > 100)
                                valve_cycle = 100;
                        else if(valve_cycle < 0)
                                valve_cycle = 0;
                        if((valve_cycle > 0) && (valve_cycle != previous_duty_cycle))
                        {
                                valve_duty_cycle(valve_cycle);
                        }
                        loop = 1;
                        modified_pressure = final_setting.Pinsp;
                        insp_duty_cycle = transfer_function((int)(modified_pressure));
                        turbine_duty_cycle(insp_duty_cycle);
                }
                else if((( final_setting.Pinsp - pressure_insp) >= 2) && flow_combine > 10){
                                modified_pressure = (final_setting.Pinsp - pressure_insp)*0.2 * 2 + modified_pressure;
                                insp_duty_cycle = transfer_function((int)(modified_pressure));
                                turbine_duty_cycle(insp_duty_cycle);
                                //valve_cycle = (2 )*0.4*(set_peep - pressure) + x
                                valve_cycle = 0;
                                valve_duty_cycle(valve_cycle);
                }

		 else if ((( final_setting.Pinsp - pressure_insp) >= 2) && flow_combine > -2){
                        if ( flow_combine > -2){
                                valve_cycle = 6 * 0.4*(final_setting.Pinsp - pressure_insp) + valve_cycle;
                                modified_pressure = (pressure_insp - final_setting.Pinsp)*0.1*2 + modified_pressure; 
                                if(modified_pressure < final_setting.Pinsp){
                                                modified_pressure = final_setting.Pinsp;
                                                insp_duty_cycle = transfer_function(modified_pressure);
                                                turbine_duty_cycle(insp_duty_cycle);
                                                if(valve_cycle > 100)
                                                        valve_cycle =100;
                                                if(valve_cycle < 0 )
                                                        valve_cycle =0;
                                                if((valve_cycle >=0 )&& (valve_cycle != previous_duty_cycle))
                                                                valve_duty_cycle(valve_cycle);
                                }
                                loop = 2;
                        }
                }
                else if((final_setting.Pinsp - pressure_insp) >=2){
                                modified_pressure = ( final_setting.Pinsp - pressure_insp)*0.2 * 2 + modified_pressure;
                                insp_duty_cycle = transfer_function(modified_pressure);
                                turbine_duty_cycle(insp_duty_cycle);
                                loop = 3;
                }
                //usleep(10000);
                previous_duty_cycle = valve_cycle;
        //      printf(" Modified  == %f Valve cuycle = %f loop = %d  pressure = %f flow = %f \n" , modified_pressure , valve_cycle,loop,pressure_insp,flow_combine);
        //        printf(" %f \n", flow_combine);
	}
		/////////////////////////////////////////////////// Algo End
	}
			pthread_mutex_trylock(&mutex);
                        slope_thread_global = false;
                        pthread_mutex_unlock(&mutex);

			if(!trigger_enable)
				insp_time = timer;
			else
				insp_trigger_time =timer;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
			if(Ihold && !trigger_enable){
			bcm2835_gpio_set(INHALE_VALVE);
			int new_pres = final_setting.Pinsp + final_setting.Pinsp*HOLD_TUBINE_FACTOR;
		//	(new_pres > 70)?new_pres=70:new_press=new_press;// 
			if(new_pres > 70)
			{
				new_pres = 70;
			}
			duty_cycle = transfer_function(new_pres);
                        turbine_duty_cycle(duty_cycle);

			clock_gettime(CLOCK_MONOTONIC_RAW, &hold_start_time);
			clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);
                        hold_time = delta_t(&start_time, &end_time,&delta_hold);
			while ( (hold_time <= (Ihold_time * 1000)))
			{	flow_combine = flow_insp(0x28);   //n
				pthread_mutex_trylock(&mutex);
                                pressure_insp = calc_pressure;
                                exp_pressure = exp_pressure_reading;
                                pthread_mutex_unlock(&mutex);
				clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
				 if(reading_timer > reading_set_time)
                                {
                                        sprintf(A_string, "A@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final,trigger );
                                        serial_data_write(A_string);    // Error Code
                                        clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                                }
				clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);
                        	hold_time = delta_t(&hold_start_time, &hold_end_time,&delta_hold);
			}
			duty_cycle = transfer_function(final_setting.peep);
                        turbine_duty_cycle(duty_cycle);

			valve_duty_cycle(0);
			bcm2835_gpio_clr(INHALE_VALVE);
			Ihold = false;
			Ihold_time = 0;
			}
			else
			valve_duty_cycle(98);
			//valve_
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//	pthread_mutex_trylock(&mutex);
                  //      slope_thread_global = false;
                    //    pthread_mutex_unlock(&mutex);

		//	printf(" After Error 4 \n");
			//rise_time = 0;
		//	if(!trigger_enable)
		//	{//	printf("Wait for joining \n");
			//	error(pthread_join(slope_thread,NULL));
			//	printf("Wait complete \n");
				printf(" Mandatory exhale timer  \n");
				clock_gettime(CLOCK_MONOTONIC_RAW, &start_time_exhale);
				clock_gettime(CLOCK_MONOTONIC_RAW, &end_time_exhale);
				exp_time = delta_t(&start_time_exhale, &end_time_exhale,&deltat_reading);
				printf(" Exp_time == %d  \n",exp_time);

		/*	}
			else
			{	 printf(" After Error 5 \n");
				clock_gettime(CLOCK_MONOTONIC_RAW, &simv_start_time_exhale);
				clock_gettime(CLOCK_MONOTONIC_RAW, &simv_end_time_exhale);
                             	timer = delta_t(&simv_start_time_exhale, &simv_end_time_exhale,&deltat_reading);
			}
		*/	//valve_duty_cycle(95);
                	duty_cycle = transfer_function((int)(final_setting.peep));
                	turbine_duty_cycle(duty_cycle);
			 //printf(" After Error 6 \n");
		//	if(trigger_enable){
		//		RR_cal =  (int) ((60*1000)/(insp_trigger_time + simv_exp_time));
                  //              check_rr = ((10*1000*60)/(insp_time * 1.00 + cmv_exp_time * 1.00)) - (10 * (int)RR_cal);
		//	}
		//	else
		//	{
				RR_cal =  (int) ((60*1000)/(insp_time +cmv_exp_time));

			//	printf(" insp_time == %d  exp time === %d \n", insp_time , cmv_exp_time);
			//	printf(" RR == %d \n ", RR_cal);
				check_rr = ((10*1000*60)/(insp_time * 1.00 + cmv_exp_time * 1.00)) - (10 * (int)RR_cal);
		//	}
			// printf(" After Error 7 \n");
		        if(counter >= 3 )
                        {
                                rr_flag = true;
                        //        counter = 0;
                        }
		//	printf("Check RR = %f \n" , check_rr);
			if(check_rr > 0.5)
			{
				RR_cal = RR_cal + 1;
			}
		//	printf(" RR_cal after adding == %d \n" ,RR_cal);
			RR_array[counter] = RR_cal;
			for ( int i = 0; i < Pmean_count ;i++)
			{
				Pmean_add = Pmean_array[i] + Pmean_add ;
			}
			Pmean = (Pmean_add /(Pmean_count *1.0));
			Pmean_count = 0;
			Vi = volume_final;
			printf(" before compensation %f \n",Vi);
			Vi = Vi-1.2*(pip-cal_peep);
			printf(" after compensation %f \n",Vi);
			MVi_array[counter] = Vi *  final_setting.RR_rxd;
		//	printf("%d \n ", counter);
			if(rr_flag)
			{	final_rr = (RR_array[0]  +RR_array[1] +RR_array[2] +RR_array[3]) /4.0 ;
				MVi =( MVi_array[0] +MVi_array[1] +MVi_array[2] +MVi_array[3] )/4.00;
			}
		//	printf(" MVi == %f\n",MVi);
			volume_final = 0;

		//	Pmean = Pmean_add/(Pmean_count*1.0);
		//	 printf(" Pmean  == %f\n",Pmean_count);
			sprintf(b_string, "B@%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f#",round(pip) , Vi , Pif_insp ,  Pmean, MVi/1000.0, 0.0,insp_time/1000.00 , pressure_insp,0,rise_time/1000.00);
			printf (" %s \n" , b_string);								// Pmean	TRIGGER_FLOW
                        serial_data_write(b_string);    // Error Code
			Pmean_add = 0;
			Pmean_count = 0;
			Pmean = 0;
			timer = 0;
			clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
			reading_timer =0;
			Pif_insp = 0;
			duty_cycle_valve_cal = 0;
			exp_flag = true;
			TiTOT = (insp_time/(inhale_time*1000.00 + final_setting.Tlow*1000.00)) *100;
		//	printf(" Ti/tot = %f \n" ,TiTOT);
			Exhale_flag = true;
			while(exp_time < (final_setting.Tlow * 1000))// ||  (trigger_enable && (timer < (exhale_time * 300))))
			{
				flow_combine = flow_insp(0x28);   //n
				Pef_exp = min(Pef_exp,flow_combine);
		       		pthread_mutex_trylock(&mutex);
				pressure_insp = calc_pressure;
				exp_pressure = exp_pressure_reading;
				pthread_mutex_unlock(&mutex);
			//	printf(" insp pressure  = %f  Exp pressure = %f \n", pressure_insp ,exp_pressure);
			//	float Kp= 0.1;
			//	float Kc = 1.4;
				if((pressure_insp <= (final_setting.peep ))) //&& (((exp_time >= 300.0)))) //&& !trigger_enable ) || ((timer >= 300.0)&& trigger_enable )))
				{//	exp_flag = false;
					duty_cycle_valve_cal = 100;
					valve_duty_cycle(98);
					if(duty_cycle_valve_cal >= 100)
					{
						break;
						printf("breakinggggggggggggggggggggggggggggggggg at Timer = \n");
					}
				}
				clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                                if(reading_timer > reading_set_time)
                                {
					  sprintf(c_string, "C@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final * -1,TiTOT);
                                          serial_data_write(c_string);    // Error Code
				//	  printf(" Flow == %f \n" , flow_combine);
				          clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);

                                }
			//	if(!trigger_enable){
					clock_gettime(CLOCK_MONOTONIC_RAW, &end_time_exhale);
	                		exp_time =  delta_t(&start_time_exhale, &end_time_exhale,&deltat);
			//	}
			//	else
			//	{
			//		   clock_gettime(CLOCK_MONOTONIC_RAW, &simv_end_time_exhale);
  	                //                   timer = delta_t(&simv_start_time_exhale, &simv_end_time_exhale,&deltat_reading);
			//	}
			//	usleep(1000);

			}
			valve_duty_cycle(98);
	//		printf(" Pip = %f \n" , pip);
	//		printf(" Pressure_insp  = %f \n", pressure_insp);
	//		printf(" TiTOT = %f \n",TiTOT);
//			mean_airway_pressure = ((pip - pressure_insp) * TiTOT/100.00) + pressure_insp ;
			TiTOT_slope = (insp_time * final_setting.slope /100.00)/(inhale_time*1000.00 + exhale_time*1000.00);
                        TiTOT_remain = (insp_time * (100 - final_setting.slope) /100.00)/(inhale_time*1000.00 + exhale_time*1000.00);
                        mean_airway_pressure_thread = ((pip - pressure_insp) * TiTOT_slope)  * 0.5;
                        mean_airway_pressure =  ((pip - pressure_insp) * TiTOT_remain) ;
                        mean_airway_pressure = mean_airway_pressure +mean_airway_pressure_thread + pressure_insp;

		//	printf(" the pressure = %f \n" , mean_airway_pressure);
		//	printf(" MVe == %f\n",MVe);
		//	printf(" Leak = %f \n" , leak);
	//		printf(" CMV_ exp _time  = %d\n" ,cmv_exp_time);

		}

	}
}
 
void debug()
{
//while(1){
   printf("code stopped here \n");
//}
}



void main()
{
//	int mode_number  = 0;
	valve_direction();              //nidhi
	pwm_init();
//	transfer_function(70);
	//turbine_duty_cycle(50);
//nidhi	turbine_duty_cycle(30);
//nidhi	debug();
	//n flow_init();
//nidhi	final_flow=flow_insp(0x28);  //nidhi
        printf("hyyeee\n");
/////////////////////////////////////////////////////////////
//nidhi	bcm2835_gpio_fsel(INHALE_VALVE,BCM2835_GPIO_FSEL_OUTP);
	bcm2835_gpio_fsel(INHALE_VALVE,"out");
	bcm2835_gpio_clr(INHALE_VALVE);
	read_settings();
//nidhi	both_calib(&flow_insp_error , &flow_exp_error);
//nidhi	both_calib(&flow_insp_error);
	both_calib(&flow_insp_error);
	printf( " Insp Error = = %f \n" ,flow_insp_error);
//nidhi	printf( " Exp Error = = %f \n" ,flow_exp_error);

//	flow_insp_error = insp_calib();
//	flow_exp_error = exp_calib();
	thread_exit = true;
	thread_control_pressure = true;
	start_bus();
	pthread_mutex_init(&mutex,NULL);
	pthread_t pressure_thread;
	pthread_t slope_thread;

//	pthread_mutex_trylock(&mutex);
  //                      slope_thread_control = slope_thread_global; 
    //                    pthread_mutex_unlock(&mutex); 
	//final_setting.slope_flag = 1.0;
//nidhi	error(pthread_create(&slope_thread,NULL,slope_thread_parallel,&final_setting));

//	pthread_t floww_thread;
        error(pthread_create(&pressure_thread,NULL,press_thread,NULL));
  //      pthread_create(&floww_thread,NULL,flow_thread,NULL);


	//pressure_modes();
	 //while (1)
        //{
	  printf("bbyyyeee \n"); 
 while(1)
{
printf("Flow =%f\n" ,  final_flow);



	while(1)
	{
		mode_para = mode_change();
		printf(" mode  ==== %f \n",mode_para);
		switch(mode_para/10)
		{
			case 1 : pressure_modes();break;
			case 2 : printf(" Volume mode is under development\n");volume_modes();break;
			case 3 : printf(" We are working on CPAP and bipap \n"); cpap();break;
			case 4 : bpap();break;
			case 5 : printf(" Aprv selectd \n");APRV_mode();break;
			case 9 : stand_by() ;printf("  Stand buy mode is under development \n");break;
			default : printf(" Invalid  string \n");break; 
//	printf("%d \n ", x);
//	pressure_modes();

		}   
}
}
}
