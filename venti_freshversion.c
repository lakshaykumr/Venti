#include "settings.h"
//#include "ntc_thread.h" //nidhi
#include <signal.h> //lk
#include "breaking.h" //lk
#include "exhale_valve.h" //lk
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include "sir_valve_turbine.h"
#include "turbine.h"
#include <pthread.h>
#include <string.h>

//#include "clear.h" //nidhi
//#include <stdio.h> //nidhi
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "timer_function.h"
#include "serial.h"
#include <math.h>
#include "flow_thread_1.h" //nidhi
#include "mine.h" //nidhi
#include "bcm2835.h"
#include "extra.h"
#include <stdbool.h>
#include <errno.h>
#include "callibration_function.h"
#include "venti_parameter.h"
//#include "tf_constent_read.h" //nidhi
#include "alarm.h"
#include <sys/time.h>
//#include"breaking_function.h" //lk


/////////////////////////////////////////////////////
#define RISE_TIME 100000 //lk
#define INHALE_VALVE 49//nidhi
#define EXHALE_VALVE_LOWER_CYCLE 800
#define EXHALE_VALVE_SET_TIME	150
#define STEP_SIZE_EXHALE_VALVE 2
#define FLOW_AVERAGE_LEAK_SIZE 10
#define HEALTHY_COMPLIENCE 20.00
#define MAX_TI 3.6
#define READING_TIME 50
#define PMEAN_ARRAY_SIZE ((MAX_TI*1000)/READING_TIME)
#define CIRCUIT_RESITANCE 0.06326;
/////////////////////////////////////////////////////
#define IO_ERROR_AMS -998
#define FILE_OPEN_ERROR_AMS -999
#define VREF 5.065
#define RES VREF/32767.00
#define DELTA 0.98
#define INHALE_VALVE 49//nidhi
#define HOLD_TUBINE_FACTOR 0
#define AVERAGE_SIZE  10
#define A_STRING_SIZE 50
#define B_STRING_SIZE 100
#define C_STRING_SIZE 50
#define D_STRING_SIZE 100
#define VOLUME_FIXED_COMPLIANCE 28.00
#define CPAP_DEFAULT_BACKUP_TIME 20
#define CPAP_MODE_CODE 31
#define FLOW_TIME_SET 5000
//#define TIME_DELAY_BREAKING 50000
#define BREAKING_PIN 21
#define MANDATORY_EXHALE 300
#define PATIENT_DISCONNECTION_VALUE 5
#define AVERAGE_FACTOR 20 //lk

//float break_time( float peep, float Pinsp);
bool flow_flag = false; //lk
float flow_array[50]; //lk
float average_flow=0; //lk
int average_count=0; //lk

////////////////////////////////////////

#define INSP_PRESS_ADD		0x17
#define EXP_PRESS_ADD		0x17

#define EXP_DUTY_CYCLE		1000
#define INSP_DUTY_CYCLE         1000


//int A_STRING_SIZE = 50;
char back_ventilation_activated[] =     "ACK38";
char backup_ventilation_end[]=          "ACK48";

bool CPAP_flag = false;
bool change_compliance = false;
bool Triggered_breath = false;
//	bool CPAP_flag = true;
bool CPAP_start 		= false;
bool pc_simv_flag_backup 	= false;
bool pc_psv_flag_backup 	= false;
bool pc_ac_flag_backup 		= false;
bool vc_simv_flag_backup 	= false;
bool vc_ac_flag_backup 		= false;
bool APNEA_on_off 		= false;

//alarm_type alarm_struct;

//alarm_limits alarm_struct;
char OXYGEN_REAIND_path[]= "/home/pi/oxygen_read.txt";
char oxygen[4] ;

float oxygen2_perc_cur = 21;


void string_a(float, float , int , float );
void string_b(float Pinsp_pressure , int VTI_insp , float PFRi , float Pmean_insp,float Mvi_insp,int x,float Ti_insp , float rxd_pressure,float Rise_time_inhale  );
void string_c(float, float , int , float  );
void string_d(float peep_pressure , int bpm , int fio2 , float PFRe , float MVE_exhale , float leak_percentage  , int unknown_var , int volume ,float Te_exhale);
float flow_init(void);
float flow_insp(void);
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
float moving_average_insp[(int)AVERAGE_SIZE];
float moving_average_exp[(int)AVERAGE_SIZE];
int average_index = 0;
bool flow_average_flag = false;
////////////////////////////////////////////////////////
//int flow_moving_average[FLOW_AVERAGE_LEAK_SIZE];
float sum_of_lower_half,sum_of_upper_half, flow_moving_average[FLOW_AVERAGE_LEAK_SIZE];
bool leak_flow_compensation_flag = false;
int flow_moving_average_index = 0;
float leak_ayush_sir = 0.0;
////////////////////////////////////////////////////////

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
char *bus = "/dev/i2c-2";  //lk
int pressure_1 = 0;
char data[4] = {0};
float pressure2=0;
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
struct timespec start_100ms = {0,0};
struct timespec end_100ms = {0,0};
struct timespec deltat_100ms = {0,0};
////////////////////////////////////// Volume parameter ////////////////////////////////////////
struct timespec volume_start_time = {0,0};
struct timespec volume_end_time = {0,0};
struct timespec volume_deltat = {0,0};
struct timespec start_time_exhale = {0,0};
struct timespec end_time_exhale = {0,0};
struct timespec simv_start_time_exhale = {0,0};
struct timespec simv_end_time_exhale = {0,0};
struct timespec backup_start = {0,0};
struct timespec backup_end = {0,0};
struct timespec backup_deltat = {0,0};
/////////////////////////////////////////bREAKING////////////////////////////////////////////////
  struct timespec Break_start_time = {0,0};
        struct timespec Break_end_time = {0,0};
        struct timespec Break_deltat = {0,0};

////////////////////////////////////// Variable for Compliance///////////////////////////////////////
//float *volume_array;
//float *pressure_array;
long double sum_of_volume = 0;
long double sum_of_press_vol=0;
long double sum_of_volume_exp = 0;
long double sum_of_press_vol_exp=0;
float final_compliance = 0;
//int index = 0;
////////////////////////////////////////////////////////////////////////////////////////////////////

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

/////////////////////////////////////////////////////////////////////////////////////////
bool Exhale_flag = false;
bool man_breath_flag = false;
bool Ehold , Ihold;
float Ihold_time =0, Ehold_time = 0;
bool first_time = false;
bool setting_changed = false;
int apnea_ventilation(int);

bool backup_start_time = false;
//////////////////////////////////////////////////////////////////////////////////////////
//int* ptr;
////////////////////////////////////// Pressure threads ////////////////////////////////////////
//////////////////////ntc timer//////////////////////////////
struct timeval start, end;
long secs_used,micros_used;
///////////////////ntc timer end////////////////////////////

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
{
//	start_bus();
        ioctl(file, I2C_SLAVE, address);
        if(read(file, data, 4) != 4)
        {
                printf("Error : Input/Output error \n");
                return 0;
        }
                pressure_1 = (data[0] * 256 + data[1]);
                pressure2 = ((((pressure_1 - 3277.0) / ((26214.0) / 3.0))-1.5)*70.30);
        //      printf("circuit pressure = %d\n",pressure2);
                return (pressure2);
//	close(file);
}

bool thread_control_pressure = false;
float exp_pressure_reading = 0;

void* press_thread()
{	printf(" Pressure Thread_ created \n");
	bool local_pressure_thread = false;
//	bool local_pressure_alarm = false;
//	float local_pressure=0;
        while(1){
	//	printf(" inside the loop\n");
		pthread_mutex_trylock(&mutex);
		local_pressure_thread = thread_control_pressure;
		pthread_mutex_unlock(&mutex);
		while(local_pressure_thread)
        	{//	printf(" running in loop \n");
                	pthread_mutex_trylock(&mutex);
                	calc_pressure = pressure(INSP_PRESS_ADD) + 0.3;
			//local_pressure = calc_pressure;
			exp_pressure_reading = pressure(EXP_PRESS_ADD);
			//printf(" Exhale _ vavlue  ===  %f\n",exp_pressure_reading); 
			local_pressure_thread = thread_control_pressure;
			//printf("preesure %f \n",calc_pressure);
                	pthread_mutex_unlock(&mutex);
                	usleep(20000);
        	}
		usleep(1000000);

	}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool global_exhale_thread = false;

/*void *exhale_valve_control()
{

	bool local_exhale_thread = false;
	int  local_timer = 0;
	int  local_cal_duty  = 0;
	struct timespec valve_control_start_time = {0,0};
	struct timespec valve_control_end_time = {0,0};
	struct timespec valve_control_deltat = {0,0};
	int total_time = EXHALE_VALVE_SET_TIME;
	int duty_cycle = EXHALE_VALVE_LOWER_CYCLE;
	int usleep_delay = ((float)total_time /(float) (1000.00 - duty_cycle))*1000;
	usleep_delay = usleep_delay * 0.80;
	usleep_delay = usleep_delay * STEP_SIZE_EXHALE_VALVE;
//	printf(" Exhale valve thread create \n");
	while(1)
	{
		pthread_mutex_trylock(&mutex);
		local_exhale_thread = global_exhale_thread;
		pthread_mutex_unlock(&mutex);
		clock_gettime(CLOCK_MONOTONIC_RAW, &valve_control_start_time);

		if(local_exhale_thread)
		{
			//clock_gettime(CLOCK_MONOTONIC_RAW, &valve_control_start_time_time);
                        //volume_timer = delta_t(&volume_start_time, &volume_end_time,&volume_deltat);
			while(1)
			{
				//clock_gettime(CLOCK_MONOTONIC_RAW, &valve_control_start_time_time);
				//local_timer = delta_t(&volume_start_time, &volume_end_time,&volume_deltat);
				local_cal_duty++;

				valve_duty_cycle(EXP_DUTY_CYCLE - (STEP_SIZE_EXHALE_VALVE* local_cal_duty));
				usleep(usleep_delay);
				if((EXP_DUTY_CYCLE - (STEP_SIZE_EXHALE_VALVE * local_cal_duty)) <=EXHALE_VALVE_LOWER_CYCLE)
					break;
				//clock_gettime(CLOCK_MONOTONIC_RAW, &valve_control_start_time_time);
                                //local_timer = delta_t(&volume_start_time, &volume_end_time,&volume_deltat);
				//printf(" Exhale duty cycle = %d \n" , EXP_DUTY_CYCLE - (1 * local_cal_duty));

			}
			clock_gettime(CLOCK_MONOTONIC_RAW, &valve_control_end_time);
                        local_timer = delta_t(&valve_control_start_time, &valve_control_end_time,&valve_control_deltat);
			printf(" calculated delay ============================================================================================================= %d \n ", usleep_delay);
			printf(" TImer ============================================================================================================= %d \n ", local_timer);
			local_cal_duty = 0;
			local_exhale_thread = false;
			pthread_mutex_trylock(&mutex);
	                global_exhale_thread = false;
        	        pthread_mutex_unlock(&mutex);


		}
		usleep(10000);

	}

}




*/
setting_parameter final_setting;


/*
////////////////////////////
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
		 printf("adc ready");average_exp_volt
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
*/
float flow_init()
{
	int adc_ID = 0x4;//id = 100 or 0x04 first 3 bits
	uint8_t analog_data[3] = {0,0,0};
	uint8_t analog_data2[3] = {0,0,0};
	int output = 0;
	int output2 = 0;
	float value = 0;
	float value2 = 0;
	int msb = 0;
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
  //      float final_flow = 0;
         float flow1_voltage;

//        usleep(22000);
        CS_Init();
        CS_High();
        spiInit();
        usleep(22000);  //nidhi
        
      //RESET
	uint8_t buf1[1] = {RESET_OPCODE_MASK};
	spiTx(buf1,1);
	//usleep(5000);
	
       if(adc_ID == ((0x4) & (ADC114S08_RegRead(ID_ADDR_MASK))))
	{
		printf("ADS114S08  found\n");
	}

	if(0x80 == ADC114S08_RegRead(STATUS_ADDR_MASK))
	{
		printf("adc ready\n");
	}
	ADC114S08_RegWrite( STATUS_ADDR_MASK,   0x00); //clear FL_POR flag
	
        adc_start();
        
	//SELF OFFSET calibration
	uint8_t buf2[1] = {0x19};
	spiTx(buf2,1);

        usleep(5000);
	uint8_t buf3[1] = {START_OPCODE_MASK};
	spiTx(buf3,1);
	usleep(5000);  //nidhi

                ADC114S08_RegWrite( INPMUX_ADDR_MASK,   0xBA);
                usleep(813);

                    	uint8_t buf4[1] = {RDATA_OPCODE_MASK};
			spiTx(buf4,1);
			spiRx(analog_data,4);
                     	msb = (int)analog_data[1] << 8 ;
                     	output = msb + analog_data[2];
                     	value = (RES) * (float)output;
                //      printf(" Analog voltage 1  == %f \n " , value);
			flow1_voltage = value;
		//nidhi output_insp =  bi_directional(flow1_voltage + flow_insp_error);//-494.5602 + 465.636*flow1_voltage - 160.6871*pow(flow1_voltage,2) + 21.42495*pow(flow1_voltage,3);  //voltage_flow(value /1000.00);  //            bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, HIGH);
//////////////////////////////////////////////////////////////////////////////////////
                ADC114S08_RegWrite( INPMUX_ADDR_MASK,   0x98);
                usleep(813);
                uint8_t buf5[1] = {RDATA_OPCODE_MASK};
		spiTx(buf5,1);
		spiRx(analog_data2,4);
			clock_gettime(CLOCK_MONOTONIC_RAW, &volume_end_time);
			volume_timer = delta_t(&volume_start_time, &volume_end_time,&volume_deltat);
			clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
                        msb2 = (int)analog_data2[1] << 8;
                        output2 = msb2 + analog_data2[2];
                        value2 = (RES) * (float)output2;
	//nidhi	output_exp = voltage_flow(value2 + flow_exp_error);
	//	final_flow =(output_insp * 0.9 +insp_previous_value) - (output_exp * 0.9 +exp_previous_value);
	//	printf(" isnp = %f Exp = %f volt = %f  Exp = %f \n",output_insp,output_exp,(value + flow_insp_error) * 1000 , (value2 + flow_exp_error) *1000);
//		if((Exhale_flag) && (final_flow < 6))
//			volume_final = ((final_flow) * volume_timer * 1.0)/60.00;
//		else
		//	volume_final = ((final_flow) * volume_timer * 1.0)/60.00  + average_exp_voltvolume_final;
	//	exp_previous_value = output_exp * 0.10;
	//	insp_previous_value = output_insp * 0.10;
  //      bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, HIGH);
//      usleep(17000);

//        while(1)
	return 0;
}
bool flow_insp_flag = false;
float previous_cycle_leak = 0;
int sum_of_time = 0 ;
int index_leak = 0;
float array_100ms[100];
int previous_max_index = 0;
bool timer_flip_reading = false;
float Cx1=0, Cx2=0,Cy1=0,Cy2 =0 , m = 0;

float flow_insp()
{

	float volume_timer = 0;
//	uint8_t adc_ID = 0x4;//id = 100 or 0x04 first 3 bits
        uint8_t analog_data[3] = {0,0,0};
	uint8_t analog_data2[3] = {0,0,0};
        //char read_command[3] =  {0b00010010,0,0} ;
       int output = 0;
	int output2 = 0;
	float value = 0;
	float value2 = 0;
	int msb = 0;
	int msb2 = 0;
	float output_insp=0 , output_exp =0;
	float final_flow = 0;
	float flow1_voltage = 0.0;
	float average_exp_volt = 0;
	float average_insp_volt = 0;
//	spi_init();

//              adc_start(0b10111010);
//              bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
        //      bcm2835_spi_transfer(START_OPCODE_MASK);
           ADC114S08_RegWrite( INPMUX_ADDR_MASK,   0x98);
//              adc_start(0b10111010);
              //  bcm2835_spi_transfer(START_OPCODE_MASK);
                usleep(410);
//      for(int i=0;i<20;i++)
//              {
//                      ADC114S08_RegWrite( INPMUX_ADDR_MASK, 0b10111010,3 );
                       uint8_t buf41[1] = {RDATA_OPCODE_MASK};
			spiTx(buf41,1);
			spiRx(analog_data,4);
			
                       msb = analog_data[1] << 8 ;
                       output = msb + analog_data[2];
                       value = (RES) * (float)output;
//                      printf(" Analog voltage 1  == %f \n " , value);
			flow1_voltage = value;
		//	if(!Exhale_flag)
			moving_average_insp[average_index] = flow1_voltage;
//////////////////////////////////////////////////////////////////////////////////////
                	ADC114S08_RegWrite( INPMUX_ADDR_MASK,   0xBA);
                	usleep(410);
                	
                     uint8_t buf51[1] = {RDATA_OPCODE_MASK};
			spiTx(buf51,1);
			spiRx(analog_data2,4);
			clock_gettime(CLOCK_MONOTONIC_RAW, &volume_end_time);
			volume_timer = delta_t(&volume_start_time, &volume_end_time,&volume_deltat);
			clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
                        msb2 = (int)analog_data2[1] << 8 ;
                        output2 = msb2 + analog_data2[2];
                        value2 = (RES) * (float)output2 ;
//			 printf(" Analog voltage 2  == %f \n " , value2);

			moving_average_exp[average_index] = value2;
			//output_exp  = 0;
			//output_insp = 0;
/*///////////////ntc added//////////////////////////
                        if(micros_used >= 500000)
                        {
				ADC114S08_RegWrite( INPMUX_ADDR_MASK, 0x4C,3 );
                                usleep(410);
                                bcm2835_spi_transfernb(read_command , analog_data2,3);
                                msb2 = (int)analog_data2[1] << 8 ;
                                output2 = msb2 + analog_data2[2];
                                value2 = (RES) * (float)output2 * 1000;
			//	printf("voltage of oxygen = %f\n",value2/1000.00);
//				oxygen2_perc_cur = (45.97268 * (value2/1000.00)) - 3.438523;
	//			oxygen2_perc_cur = ((95.0/2.5)*(value2/1000.00))*1.05;
				oxygen2_perc_cur = (45.96027 * (value2/1000.00)) - 3.410601;
				if(oxygen2_perc_cur >= 100.0)	oxygen2_perc_cur = 100.0;
				if(oxygen2_perc_cur <= 21.0)     oxygen2_perc_cur = 21.0;
                                //average2 = average2 + value2;
				sprintf(oxygen , "%f" , oxygen2_perc_cur);
			//	printf(" Oxygen ========================= %f \n", oxygen2_perc_cur);

                          //      printf("inside timer \n");
                                select_ntc_temp();
    	                        gettimeofday(&start, NULL);// timer start
                                bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
                                bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
				int oxyptr = open(OXYGEN_REAIND_path, O_WRONLY | O_CREAT);
				if(oxyptr != 0)
				{
					write(	oxyptr ,oxygen , 4);
					close(oxyptr);
				}

                        }
                        gettimeofday(&end, NULL);
                        secs_used=(end.tv_sec - start.tv_sec); //avoid overflow by subtracting first
                        micros_used= ((secs_used*1000000) + end.tv_usec) - (start.tv_usec);

//////////////ntc end///////////////////////////////*/
			if(flow_average_flag)
			{
				for (int j = 0; j<AVERAGE_SIZE;j++){
					if(j < 5){
						sum_of_lower_half  = sum_of_lower_half +  flow_moving_average[j];
						sum_of_upper_half =  sum_of_upper_half +  flow_moving_average[FLOW_AVERAGE_LEAK_SIZE -j -1];
					}
					//if(j < 10)
					//	printf(" %f " , flow_moving_average[j]);
 					average_insp_volt =moving_average_insp[j] + average_insp_volt ; 	//(moving_average_exp[0] +  moving_average_exp[1] +  moving_average_exp[2] +  moving_average_exp[3] +  moving_average_exp[4] )/5.0;
					average_exp_volt  = moving_average_exp[j] + average_exp_volt ;	//(moving_average_insp[0] +  moving_average_insp[1] +  moving_average_insp[2] +  moving_average_insp[3] +  moving_average_insp[4] )/5.0;
					}
					//printf("\n");
				sum_of_lower_half = sum_of_lower_half/5.0;
				sum_of_upper_half = sum_of_upper_half/5.0;
				if(flow_moving_average_index >= 10)
				{
						for( int i = 0; i< (FLOW_AVERAGE_LEAK_SIZE -1);i++){
							 flow_moving_average[i] = flow_moving_average[i+1];
						}
					flow_moving_average_index = 9;
					}
				average_insp_volt =  average_insp_volt/AVERAGE_SIZE;
				average_exp_volt =  average_exp_volt/ AVERAGE_SIZE;
			//	printf("average_insp_volt = %f \n ",average_insp_volt);
			//	printf("average_exp_volt = %f \n ",average_exp_volt);

			//nidhi	output_insp = ((output_insp * 0.90) + insp_previous_value ) * 1.0;//0915248;//(output_insp * 0.2)
				output_insp = bi_directional(average_insp_volt + flow_insp_error);//-494.5602 + 465.636*flow1_voltage - 160.6871*pow(flow1_voltage,2) + 21.42495*pow(flow1_voltage,3);  //voltage_flow(value /1000.00);  //            bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, HIGH);
			//	 printf("output insp_1 = %f \n ",output_insp);

				 output_insp = ((output_insp * 0.90) + insp_previous_value ) * 1.0915248; //0915248;//(output_insp * 0.2)
			//	 printf("output insp = %f \n ",output_insp);

				output_exp =voltage_flow(average_exp_volt + flow_exp_error);
			//	   printf("output EXPp_1 = %f \n ",output_exp);

				output_exp = ((output_exp*0.90) + exp_previous_value);        //1.0915248;//(output_exp * 0.14);
			//	printf("output EXPp = %f \n ",output_exp);

			//	printf("Average_index = %d \n ",average_index);
				final_flow =(output_insp * 0.90 +insp_previous_value) - (output_exp * 0.90 +exp_previous_value);
			//	 printf("final flow = %f \n ",final_flow);

				//if(Exhale_flag){
				flow_moving_average[flow_moving_average_index] = final_flow;
				flow_moving_average_index++;

//		if((Exhale_flag) && (final_flow < 6))
//			volume_final = ((final_flow) * volume_timer * 1.0)/60.00;
//		else
	//	if((abs((int)(output_exp)) - abs((int)(output_insp))) > 2){
		//	if((abs((int)final_flow) > 6) )
		//	{
//				if((((sum_of_lower_half * 0.95 )< sum_of_upper_half < (sum_of_lower_half * 1.05)) && !Exhale_flag &&(final_flow > 0) && (!Triggered_breath)))//  || (final_flow <= -0.5)))
//							float inhale_leak = final_flow;

				if(((((sum_of_lower_half * 0.95 )< sum_of_upper_half) && (sum_of_upper_half  < (sum_of_lower_half * 1.05))) && Exhale_flag &&(final_flow > 0) && (!Triggered_breath)))//  || (final_flow <= -0.5)))
				{
					//flow_insp_flag = false;
					clock_gettime(CLOCK_MONOTONIC_RAW, &end_100ms);
					float diff_time =  delta_t(&start_100ms, &end_100ms,&deltat_100ms);
					sum_of_time = sum_of_time + diff_time;
					clock_gettime(CLOCK_MONOTONIC_RAW, &start_100ms);
					leak_ayush_sir = final_flow;
					array_100ms[index_leak] = leak_ayush_sir;
					if(sum_of_time > 100)
					{	previous_max_index = index_leak;
						sum_of_time =  0;
						index_leak = 0;
					}
					index_leak++;
					//printf(" Leak updated ======  %f      time  =  %f   index = %d previous max index = %d \n " , leak_ayush_sir,diff_time , index_leak,previous_max_index);
				}

				if(Triggered_breath)
				{
					if(index_leak > previous_max_index)
					{
						leak_ayush_sir = array_100ms[0];
					//	printf(" 100ms of old value flow %f\n",array_100ms[0]);
					}
					else
					{	leak_ayush_sir = array_100ms[index_leak];

					//	printf(" 100ms of old value flow %f\n",array_100ms[index_leak]);
					}
				}
				Cy1 =  abs(flow_moving_average[9]);
				Cy2 =  abs(flow_moving_average[0]);
				m = ((float)(Cy1 - Cy2))/170.00;//(Cx2 - Cx1);

			//	else if(trigger_enable)
/*				if(timer_flip_reading)
				{	Cx1 = volume_timer;
					//Cy1 = final_flow;
					timer_flip_reading = !(timer_flip_reading);
					m = ((float)(Cy1 - Cy2))/(Cx2 - Cx1);
				//	printf(" 1st_value  %d \n" , timer_flip_reading);
				}
				else
				{       Cx2 = volume_timer;
                                    //    Cy2 = final_flow;
					timer_flip_reading= !(timer_flip_reading);
					m = ((float)(Cy1 - Cy2))/(Cx1 - Cx2);
				//	printf(" 2nd_value %d \n", timer_flip_reading);


                                }
*/
			//	printf(" Cy1 = %f Cy2 = %f Cx1 =%f Cx2 = %f m == %f \n " ,Cy1,Cy2,Cx1,Cx2, m);
				//if(Exhale_flag)
			//	if((final_flow >= 0.5) || (final_flow <= -1))
			//	{
				//printf("%d  %f  %f \t ",Exhale_flag,previous_cycle_leak,final_flow);
				//printf(" leak updated %f \n",final_flow -previous_cycle_leak );
			//	if(Exhale_flag)
				final_flow = final_flow - previous_cycle_leak;//leak_ayush_sir;
			//	else
			//		final_flow = final_flow + previous_cycle_leak;
			//	}
				//printf(" final_flow  = %f previous_cycle_leak = %f leak_ayush_sir = %f   %d\n",volume,previous_cycle_leak,leak_ayush_sir, Triggered_breath);

				if((final_flow > 4) && (!Exhale_flag))
					volume_final = ((final_flow) * volume_timer * 1.0)/60.00  + volume_final;
					//printf(" Volume_finalll > 4 = %f \n",  volume_final);

				if((final_flow < -4 ) && (Exhale_flag))
					volume_final = ((final_flow) * volume_timer * 1.0)/60.00  + volume_final;
//				printf(" Volume_timer = %f \n",  volume_timer);
				//printf(" Volume_finalll < -4 = %f \n",  volume_final);

				//printf(" lower half == %f upper half == %f flow == %f \n",sum_of_lower_half,sum_of_upper_half,final_flow);
/*				{
					if(flow_moving_avarage_index >= 10)
					{
						for(int i = 0; i<FLOW_AVERAGE_LEAK_SIZE -1 ;i++)
						{
							flow_moving_average[i] = flow_moving_average[i+1];
							printf("FLow avaerage  = %f \n" , flow_moving_average[i]);
						}
						flow_moving_avarage_index = 10;
						leak_flow_compensation_flag = true;

					}
					flow_moving_average[flow_moving_avarage_index] = final_flow;
					flow_moving_avarage_index++;
					if(leak_flow_compensation_flag )
					{
						printf("flow_moving_avaerage_index = %d \n" ,flow_moving_avarage_index);
						for (int j = 0 ; j <= FLOW_AVERAGE_LEAK_SIZE/2;j++)
						{
							sum_of_lower_half  = sum_of_lower_half +  flow_moving_average[j];
							sum_of_upper_half =  sum_of_upper_half +  flow_moving_average[FLOW_AVERAGE_LEAK_SIZE - j];
						}
						sum_of_lower_half = sum_of_lower_half/5.0;
						sum_of_upper_half = sum_of_upper_half/5.0;
						if((sum_of_lower_half * 0.95) <= sum_of_upper_half <= (sum_of_lower_half * 1.05))
					//	if((sum_of_lower_half - 0.05 *sum_of_lower_half) <=(sum_of_lower_half - sum_of_upper_half) <= (sum_of_lower_half + 0.05 *sum_of_lower_half) )
							printf(" upper half == %f  leak == %f  flow ==  %f\n ", sum_of_lower_half , sum_of_upper_half ,sum_of_lower_half-sum_of_upper_half , final_flow);

					}
					sum_of_lower_half = 0;
					sum_of_upper_half= 0;
				}
				else
					{
						leak_flow_compensation_flag = false;
					}
*/

		//printf("insp_flow  == %f Exp_flow == %f  Flow = %f and volumer = %f \n",output_insp , output_exp,previous_cycle_leak, volume_final);
			}

			//volume_final = DELTA * volume_final;
		//	printf(" Volume == %f \n", volume_final);
		//	}
//			if((abs((int)final_flow) < 10) )value2 + flow_exp_error
//					final_flow = 0;		// for varsha,
		//	else if(!Exhale_flag)
		//		volume_final = ((final_flow) * volume_timer * 1.0)/60.00  + volume_final;
	//	printf(" Insp = %f  exp   = %f diff = %f \n",average_insp_volt+flow_insp_error,average_exp_volt+flow_exp_error, (output_insp-output_exp));
	//	printf(" Exp_ volt = %f Exp_flow = %f Inp_vol = %f Insp_flow = %f\n", (value2+ flow_exp_error) ,output_exp,(flow1_voltage+ flow_insp_error),output_insp);
		exp_previous_value = output_exp * 0.10;
		insp_previous_value = output_insp * 0.10;
		sum_of_lower_half = 0;
		sum_of_upper_half = 0;
		//bcm2835_spi_end();
		average_index++;
		if(average_index >= AVERAGE_SIZE)
		{
				flow_average_flag = true;
				average_index = 0;
		}
//		printf("=============Volume = %f ===================\n",volume_final);
		//printf("=============flow = %f ===================\n",final_flow);
		return (final_flow);
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

	return 0;
}

bool slope_thread_global = false;
bool trigger_thread = false;
bool thread_exit = false;
bool volume_flag = false;
/*void* slope_thread_parallel(void* arg)
{
        float pplat_pwm , peep_pwm;
        float inc_f=0.0;
        float slope=0.0;
        int pplat_pres , peep_pres ,flag_rec ;//,Psupport = 0;
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
			printf(" inc_f  ======================================= %f \n " , inc_f);
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

////////////////////////////////////////////////////////Pressure Modes //////////////////////////////////////////////////////////////////
//.....................................................................................................................................//
//.....................................................................................................................................//
void pressure_modes() {
//	pwm_init();
//	flow_init();
//	start_bus();
//	volume
	pthread_mutex_trylock(&mutex);
        thread_control_pressure = true;
        pthread_mutex_unlock(&mutex);

	volume_flag = false;
//	bool local_pressure_alarm = false;
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
	char A_string[A_STRING_SIZE] = {0};
	char b_string[B_STRING_SIZE] = {0};
	char c_string[C_STRING_SIZE] = {0};
	char d_string[D_STRING_SIZE] = {0};
	int current_mode = 0;
	//clear_buzzer(); lk
	//Pmean_array =  (int*)malloc(300*sizeof(float));
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
	float Final_peep = 0;
	float resis_press = 0;
	int backup_timer = 0;
	long double sum_of_flow =  0;
	long double sum_of_press_flow =  0;
	int PRESSURE_MODES_code = 0;
	previous_cycle_leak = 0;
	leak_ayush_sir = 0;
	final_setting.apnea_time = CPAP_DEFAULT_BACKUP_TIME;
	//bcm2835_gpio_fsel(BREAKING_PIN,BCM2835_GPIO_FSEL_OUTP);
        //bcm2835_gpio_set(BREAKING_PIN);
	clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
	 //break_init();
	while(1)

	{
		alarm_setting_funct();
		read_settings();
		final_flow=flow_insp();     //nidhi
                printf("Flow =%f\n" ,  final_flow);    //nidhi


		current_mode = mode_change();
		if(backup_start_time){
			backup_start_time = false;
			clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
		}
//////////////////////////////// Backup //////////////////////////////// venti
		if(trigger_flag && (PRESSURE_MODES_code > 0) && (final_setting.apnea_on_off > 0 )){
 	        	clock_gettime(CLOCK_MONOTONIC_RAW, &backup_end);
	        	backup_timer = delta_t(&backup_start, &backup_end,&backup_deltat);
	        	printf(" backup timer %d \n",backup_timer);
	        	if(backup_timer > (final_setting.apnea_time * 1000)){
	                	printf(" We shifting to backup ventilation \n");
				serial_data_write(back_ventilation_activated);
                        	CPAP_flag =true;
				backup_ventilation_alarm(1);
                        	apnea_ventilation(PRESSURE_MODES_code);
				serial_data_write(backup_ventilation_end);
				backup_ventilation_alarm(0);
                        	clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
              		}
		}
///////////////////////////////backup ///////////////////////////////// venti.c

       		if( current_mode != mode_para)
        	{
                	mode_para = current_mode;
                	break;
			printf(" exitinggggggggggggggggggggggggggggggggg from mode\n");

        	}
		trigger_mandatory = false;
		trigger_enable = false;
//		printf(" Pressure == %f Time == %d    %f\n" , pressure_insp ,exp_time,duty_cycle_\valve_cal);
		switch(current_mode%10)
		{
			case 1 : trigger_flag =false ;cmv = true;psv_flag =false; pc_ac_flag = false;break;		// Trigger flag indicate for pc_simv
			case 2 : trigger_flag = true;cmv = false; psv_flag =false ;pc_ac_flag = false;PRESSURE_MODES_code = 12;break;		// psv_flag indicate that every trigger breath is on Pinsp
			case 3 : psv_flag =true; cmv = false;trigger_flag = true;pc_ac_flag = false;PRESSURE_MODES_code = 13;break;		//pc_ac_flag indicate that this is not flow cycled
			case 7 : psv_flag =true; cmv = false;trigger_flag = true;pc_ac_flag = true;PRESSURE_MODES_code = 0;break;
			default : printf(" We are working on it \n");break;
		}
		flow_combine = flow_insp();
		clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                if(reading_timer > reading_set_time)
                {			pthread_mutex_trylock(&mutex);
                                	pressure_insp = calc_pressure;
					exp_pressure = exp_pressure_reading;
                                	pthread_mutex_unlock(&mutex);
                                	//printf("insp pressure  = %f  Exp pressure = %f \n", pressure_insp ,exp_pressure);

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
			clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
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
			Triggered_breath = true;
		}
		else
			Triggered_breath = false;

		   sum_of_press_vol_exp= sum_of_press_vol+ pressure_insp * (volume_final*-1);
                   sum_of_volume_exp = sum_of_volume + volume_final * volume_final;
		  //Final_peep =
	//	printf( " time == %d \n" , cmv_exp_time);
	//	if(volume_final > 0 )
          //                      {
                //             //           printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                  //
               // else
                 //               {
                               //         printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                   //             }
//////////////////////////////////////////////////Inhale start////////////////////////////////////////// 
		if((((cmv_exp_time +insp_time) >= (inhale_time+exhale_time)*1000)) ||  man_breath_flag || trigger_enable || trigger_mandatory)
		{

			//printf(" Exp Time  ===  %f \n" , exp_time*1000);
                        //printf(" Insp Time  === %f\n ",insp_time*1000);

			Final_peep = pressure_insp;
			//Exhale_flag = false;
			if(!patient_disconnection)
				final_compliance = sum_of_volume/sum_of_press_vol;
			printf(" Final Compliance ====================================  %f \n " , final_compliance);
			//final_compliance = (sum_of_volume+sum_of_volume_exp)/(sum_of_press_vol + sum_of_press_vol_exp);
			//printf(" finAL Compliance both ================================ %f \n " ,final_compliance);
			sum_of_press_vol_exp = 0;
                        sum_of_volume_exp = 0;
			sum_of_press_vol = 0;
			sum_of_volume = 0;
			if(Ehold && !trigger_enable){
                        	bcm2835_gpio_set(INHALE_VALVE);
                        	int new_pres = final_setting.peep + final_setting.peep*HOLD_TUBINE_FACTOR;
                //      (new_pres > 70)?new_pres=70:new_press=new_press;// 
                        	if(new_pres > 65)
                        	{
                        	        new_pres = 65;
                        	}
                        	duty_cycle = transfer_function(new_pres);
                        	turbine_duty_cycle(duty_cycle);
                        	clock_gettime(CLOCK_MONOTONIC_RAW, &hold_start_time);
                        	clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);
		        	hold_time = delta_t(&start_time, &end_time,&delta_hold);
                        	while ( (hold_time <= (Ehold_time * 1000)))
                        	{       flow_combine = flow_insp();
                                	pthread_mutex_trylock(&mutex);
                                	pressure_insp = calc_pressure;
                                	exp_pressure = exp_pressure_reading;
                                	pthread_mutex_unlock(&mutex);
                                	clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                                	reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                                	 if(reading_timer > reading_set_time)
                                	{
                                        	sprintf(A_string, "C@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final,trigger );
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
			clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
			 //int duty_cycle2 = transfer_function((int)(final_setting.Pinsp));
                        //turbine_duty_cycle(duty_cycle2);
 
                        turbine_duty_cycle(90);  // nidh
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
				pressure_alarm((float)pip);				//Alarm function
				Vte_alarm((float)Vi);					//
				RR_alarm((float)final_rr);				//
				PEEP_alarm((float)pressure_insp);			//
				Mve_alarm((float)MVe);					//

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
				printf(" Patient disconnection alarm$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$\n");
				serial_data_write("ACK05");
				}
			else{
				patient_disconnection = false;
				printf(" Patient connected  green alarm $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ \n");
				serial_data_write("ACK06");
			    }
			patient_disconn_alarm(patient_disconnection);
			Ve = volume_final;
			printf(" before compensation Ve %f \n",Ve);    //nidhi

			Ve = Ve+1.2*(pip-cal_peep);
			printf(" after compensation Ve %f \n",Ve);    //nidhi
			cal_peep = pressure_insp;
			Exhale_flag = false;										//
			sprintf(d_string, "D@%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f#", round(pressure_insp) , final_rr ,oxygen2_perc_cur ,(-1)*Pef_exp ,(-1)*MVe/1000.00 , final_compliance  ,mean_airway_pressure , Ve*-1 ,cmv_exp_time/1000.00);
			printf("%s  \n", d_string);
			printf("data ============== %d \n ", strlen(d_string));
			serial_data_write(d_string);    // Error Code
			Pef_exp = 0;

            //turbine_duty_cycle(100);   nidhi
 //908
                        usleep(RISE_TIME);   //boost          //  nidhi

                //      printf(" Flag =====   %d \n" ,trigger_enable);


		//	printf(" Flag =====   %d \n" ,trigger_enable);
			if(trigger_enable)
			{	printf(" Triggered  pressure \n ");
				duty_cycle = transfer_function((int)(final_setting.Psupp)); //lk
				turbine_duty_cycle(duty_cycle); //lk
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
				duty_cycle = transfer_function((int)(final_setting.Pinsp)); //lk
                              	turbine_duty_cycle(duty_cycle); //lk
				pthread_mutex_trylock(&mutex);
                        	slope_thread_global = true;
                       		pthread_mutex_unlock(&mutex);
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
			flow_combine = flow_insp();
			sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                        sum_of_volume = sum_of_volume + volume_final * volume_final;
			if (flow_combine < 0){
				flow_combine  =	flow_combine * -1;
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
				flow_combine = flow_insp();
				pip = max(pip , pressure_insp);
				resis_press = flow_combine * CIRCUIT_RESITANCE;//0.05326;
				//printf(" Pressure = %f  flow_combine = %f  calcu_pressure = %f " , pressure_insp , flow_combine , resis_press);
				if(pressure_insp < resis_press){
				//	printf(" Patiant Disconnected \n");
					patient_disconnection=true;
				}
				else{
					patient_disconnection=false;
				//	printf(" Patiant ----><---- \n");
				}

//				sum_of_press_vol= sum_of_press_vol+ pressure_insp * volume_final;
//				sum_of_volume = sum_of_volume + volume_final * volume_final;
				if(flow_combine > 0){
					sum_of_press_vol= sum_of_press_vol+ ((pressure_insp-Final_peep) * (volume_final));
                         		sum_of_volume = sum_of_volume + (volume_final * volume_final);
					sum_of_flow = sum_of_flow + (flow_combine *flow_combine);
					sum_of_press_flow = sum_of_press_flow + (pressure_insp  * flow_combine);
					//printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
				}

		/*		if(volume_final > 0 )
                                {
                                        sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                                        sum_of_volume = sum_of_volume + volume_final * volume_final;
                                 //       printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                                }
                                else
                                {
                                        sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final) * -1;
                                        sum_of_volume = sum_of_volume + volume_final * volume_final;
                                   //     printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                                }
*/

			//	printf("%f ,%d\n" , flow_combine ,timer);
			//	printf(" flow_combine  == %f Pif_insp == %f \n", flow_combine  , Pif_insp * 0.25);
				//if((flow_combine < 1)  && (flow_combine > -4))
				//	flow_combine = 1;
				Pif_insp = max(Pif_insp,flow_combine);
				//printf("Peak Inspiratory flow ================ %f  \n", Pif_insp);
				if(((flow_combine <= (int) (Pif_insp  * final_setting.insp_term/100.0)) && ( trigger_enable || psv_flag) && !pc_ac_flag && !slope_thread_global) && flow_combine >=1)
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
				if(reading_timer > reading_set_time)
				{
				//	Pmean_array[Pmean_count] = pressure_insp;
				//	Pmean_count++;
					sprintf(A_string, "A@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final,trigger );
					//printf("%s \n " , A_string);
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


///////////////////////////////////////////////Exhale start///////////////////////////////////////////////////
			index_leak = 0;
			clock_gettime(CLOCK_MONOTONIC_RAW, &start_100ms);
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
/*			if(((pip > alarm_struct.Pressure_high )|| (pip < alarm_struct.Pressure_low ))&& !local_pressure_alarm)
			{		local_pressure_alarm = true;
					buzzer(5,local_pressure_alarm);
			}
			else if (((pip < alarm_struct.Pressure_high )|| (pip > alarm_struct.Pressure_low ))&& local_pressure_alarm)
			{
					local_pressure_alarm = false;
                                        buzzer(5,local_pressure_alarm);
			}

*/
				int new_pres = final_setting.Pinsp + final_setting.Pinsp*HOLD_TUBINE_FACTOR;
		//	(new_pres > 70)?new_pres=70:new_press=new_press;// 
				if(new_pres > 65)
				{
					new_pres = 65;
				}
				duty_cycle = transfer_function(new_pres);
                        	turbine_duty_cycle(duty_cycle);

				clock_gettime(CLOCK_MONOTONIC_RAW, &hold_start_time);
				clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);
                        	hold_time = delta_t(&start_time, &end_time,&delta_hold);
				while ( (hold_time <= (Ihold_time * 1000)))
				{	flow_combine = flow_insp();
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
			//usleep(500000);
			bcm2835_gpio_clr(INHALE_VALVE);
			Ihold = false;
			Ihold_time = 0;
			}
			else
			valve_duty_cycle(98);
		//	breaking(pressure_insp, final_setting.peep,0);
			//breaking_enable(pressure_insp,0,0); //lk

    //BREAKING

/*
                        int break_timer = 0, breaking_time;
                        float ratio = final_setting.peep/final_setting.Pinsp;
                        printf("Ratio = %f \n",ratio);
                        if (ratio > 0.8)
                        {
                                breaking_time = 8;
                                printf("Breaking time3 = 8ms \n");
                        }
                        if (ratio >= 0.15 && ratio <= 0.4)
                        {
                                breaking_time = 250;
                                printf("Breaking time2 = 200ms \n");

                        }

                        if (ratio > 0.4 && ratio <= 0.8)
                        {
                                breaking_time = 40;
                                printf("Breaking time4 = 40ms \n");

                        }

                        if (ratio < 0.15 && ratio >= 0.0)
                        {
                                breaking_time = 200;
                                printf("Breaking time1 = 200ms \n");

                        }
*/
			int break_timer = 0;

			int breaking_time = break_time(final_setting.peep,final_setting.Pinsp);
			turbine_duty_cycle(0);
                        break_on(); //LAKSHAY   //nidhi
			clock_gettime(CLOCK_MONOTONIC_RAW, &Break_start_time);
                        while(break_timer < breaking_time)
                        {

                       // printf("=================================breaking===================================================================== \n");
                       flow_combine = flow_insp();
		     //printf("Flowwwwwwwwwwwwwwwwwwwww = %f \n",flow_combine);
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


                        clock_gettime(CLOCK_MONOTONIC_RAW, &Break_end_time);
                        break_timer = delta_t(&Break_start_time, &Break_end_time, &Break_deltat);
                        }
                        //usleep(BREAKING);//b delay
                        break_off();   //nidhi
                        duty_cycle = transfer_function(final_setting.peep);
                        printf("PEEP => %f duty cycle => %d ",final_setting.peep,duty_cycle);
                        turbine_duty_cycle(duty_cycle);
                        ///////////////////////////

	//		bcm2835_gpio_clr(BREAKING_PIN);
	//		usleep(TIME_DELAY_BREAKING);
	//		bcm2835_gpio_set(BREAKING_PIN);

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

			//valve_duty_cycle(95);&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                	//duty_cycle = transfer_function((int)(final_setting.peep));       //// uncomment this line 
                	//turbine_duty_cycle(duty_cycle);
			 //printf(" After Error 6 \n");
			if(trigger_enable){
				RR_cal =  (int) ((60*1000)/(insp_trigger_time + simv_exp_time));
                                check_rr = ((10*1000*60)/(insp_time * 1.00 + cmv_exp_time * 1.00)) - (10 * (int)RR_cal);
			}
			else
			{	RR_cal =  (int) ((60*1000)/(insp_time +cmv_exp_time));

				check_rr = ((10*1000*60)/(insp_time * 1.00 + cmv_exp_time * 1.00)) - (10 * (int)RR_cal);
			}
			// printf(" After Error 7 \n");
			printf(" insp_time == %f  exp time === %f \n", insp_time , cmv_exp_time);
			printf(" RR == %d \n ", RR_cal);

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
			//pressure_alarm(pip);
		//	pressure_alarm(pip);
			printf(" RR_cal after adding == %d \n" ,RR_cal);
			RR_array[counter] = RR_cal;
			for ( int i = 0; i < Pmean_count ;i++)
			{
				Pmean_add = Pmean_array[i] + Pmean_add ;
			}
			Pmean = (Pmean_add /(Pmean_count *1.0));
			Pmean_count = 0;
			Vi = volume_final;
			printf(" before compensation vi %f \n",Vi);
			Vi = Vi-1.2*(pip-cal_peep);
			printf(" after compensation Vi %f \n",Vi);
			MVi_array[counter] = Vi *  final_setting.RR_rxd;
		//	printf("%d \n ", counter);
			if(rr_flag)
			{	final_rr = (RR_array[0]  +RR_array[1] +RR_array[2] +RR_array[3]) /4.0 ;
				printf(" Final RR = %f \n",final_rr);
				MVi =( MVi_array[0] +MVi_array[1] +MVi_array[2] +MVi_array[3] )/4.00;
			}
			previous_cycle_leak = leak_ayush_sir;
			printf(" MViiiiiiiiiiiiiiiiiiiiiiiiiiiiiii ==================== %f\n",MVi);
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
			flow_insp_flag = true;/*
`
			if(volume_final > 0 )
			{
				sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                        	sum_of_volume = sum_of_volume + volume_final * volume_final;
                        	//printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
			}
			else
			{
				sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final) * -1;
                                sum_of_volume = sum_of_volume + volume_final * volume_final;
                                //printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
			}*/
			sum_of_press_vol_exp= sum_of_press_vol+ pressure_insp * (volume_final)*-1;
                        sum_of_volume_exp = sum_of_volume + volume_final * volume_final;
			while((!trigger_enable &&(exp_time <  300)) ||  (trigger_enable && (timer < (300))))
			{
				flow_combine = flow_insp();
				Pef_exp = min(Pef_exp,flow_combine);
			//	printf("Peak Expiratory flow ================ %f  \n", Pef_exp);

				if(!trigger_enable)
						breaking(pressure_insp, final_setting.peep,(int)exp_time);
				else
						breaking(pressure_insp, final_setting.peep,(int)timer);
			//	sum_of_press_vol_exp= sum_of_press_vol+ pressure_insp * (volume_final)*-1;
                        //	sum_of_volume_exp = sum_of_volume + volume_final * volume_final;

			/*	if(volume_final > 0 )
                        	{
                                	sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                                	sum_of_volume = sum_of_volume + volume_final * volume_final;
                                	//printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                        	}
                        	else
                        	{
                                	sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final) * -1;
                                	sum_of_volume = sum_of_volume + volume_final * volume_final;
                                //	printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                        	}

	*/		//	sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                          //      sum_of_volume = sum_of_volume + volume_final * volume_final;
                            //    printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
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
					valve_duty_cycle(100);
					//if(duty_cycle_valve_cal >= 100)
					//{
					//	break;
						//printf("breakinggggggggggggggggggggggggggggggggg at Timer = \n");
					//}
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
			valve_duty_cycle(98);

	//		printf(" Pip = %f \n" , pip);
	//		printf(" Pressure_insp  = %f \n", pressure_insp);
	//		printf(" TiTOT = %f \n",TiTOT);
//			mean_airway_pressure = ((pip - pressure_insp) * TiTOT/100.00) + pressure_insp ;

		//	printf(" MVe == %f\n",MVe);
		//	printf(" Leak = %f \n" , leak);
	//		printf(" CMV_ exp _time  = %d\n" ,cmv_exp_time);
			printf(" Exp Time  ===  %f \n" , exp_time*1000);
                        printf(" Insp Time  === %f\n ",insp_time*1000);
			TiTOT_slope = (insp_time * final_setting.slope /100.00)/(insp_time*1000.00 + cmv_exp_time*1000.00);
                        TiTOT_remain = (insp_time * (100 - final_setting.slope) /100.00)/(insp_time*1000.00 + cmv_exp_time*1000.00);
			printf(" TiTOT_slope  ===  %f \n" , TiTOT_slope);
			printf(" TiTOT Remain === %f\n ", TiTOT_remain);
                        mean_airway_pressure_thread = ((pip - pressure_insp) * TiTOT_slope) * 0.5;
                        mean_airway_pressure =  ((pip - pressure_insp) * TiTOT_remain);
                        mean_airway_pressure = mean_airway_pressure +mean_airway_pressure_thread + pressure_insp;
			printf(" Mean Airway pressure === %f \n", mean_airway_pressure);
		}

	}

}
////////////////////////////////////////////////////////Volume  Modes //////////////////////////////////////////////////////////////////
//.....................................................................................................................................//
//.....................................................................................................................................//
void volume_modes() {
//	pwm_init();
//	flow_init();
//	start_bus();
//	volume
	pthread_mutex_trylock(&mutex);
        thread_control_pressure = true;
        pthread_mutex_unlock(&mutex);

	volume_flag = false;
//	bool local_pressure_alarm = false;
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
//	int fio2_cal = 21;
//	float PFRe_cal = 0;
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
//	float pmean = 0;
//	float final_flow =0;
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
	char A_string[A_STRING_SIZE] = {0};
	char b_string[B_STRING_SIZE] = {0};
	char c_string[C_STRING_SIZE] = {0};
	char d_string[D_STRING_SIZE] = {0};
	int current_mode = 0;
	//Pmean_array =  (int*)malloc(300*sizeof(float));
	MVe_array = (int*)malloc(4*sizeof(float));
	MVi_array = (int*)malloc(4*sizeof(float));
	RR_array =  (int*)malloc(4*sizeof(int));			// Allocate array with the help of malloc
//	turbine_duty_cycle(0);
	clear_buzzer();
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
//	float flow_cycle_per = 0.25;
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
	float Final_peep = 0;
	float compliance_pressure = 28;
	int counter_volume_modes = 0;
	bool only_single_time = true;
	float sum_of_flow =0 ,sum_of_press_flow = 0,final_resistance = 0;
	bool triggring_compliance = false;
	//float pressure_80 = 0;
//	float complience_array[5];
	float current_complience = 0;
	float first_complience = 0;
	float current_volume = 0;
	bool complianc_not_cal  = false;
	float resis_press = 0;
	int PRESSURE_MODES_code = 0;
	int backup_timer= 0;
	previous_cycle_leak = 0;
	//	previous_cycle_leak = 0;
	leak_ayush_sir = 0;
	final_setting.apnea_time = CPAP_DEFAULT_BACKUP_TIME;
	//	bcm2835_gpio_fsel(BREAKING_PIN,BCM2835_GPIO_FSEL_OUTP); //lk
  //      bcm2835_gpio_set(BREAKING_PIN);//lk
	//int cpap_trigger_counter = 0;
	while(1)
	{
		alarm_setting_funct();
		read_settings();
		current_mode = mode_change();
       		if( current_mode/10 != mode_para/10)
        	{
                //	mode_para = current_mode;
                	break;
        	}
		triggring_compliance = trigger_enable;
		trigger_mandatory = false;
		trigger_enable = false;
//		printf(" Pressure == %f Time == %d    %f\n" , pressure_insp ,exp_time,duty_cycle_\valve_cal);
                resis_press = flow_combine * CIRCUIT_RESITANCE;//0.05326;
                //printf(" Pressure = %f  flow_combine = %f  calcu_pressure = %f " , pressure_insp , flow_combine , resis_press$
                if(pressure_insp < resis_press){
         //                               printf(" Patiant Disconnected \n");
                                        complianc_not_cal = true;
                                        patient_disconnection=true;
                                }
                else{
                                        patient_disconnection=false;
           //                             printf(" Patiant ----><---- \n");
                }

		switch(current_mode%10)
		{
			case 1 : trigger_flag =false ;cmv = true;psv_flag =false; pc_ac_flag = false;break;		// Trigger flag indicate for pc_simv
			case 2 : trigger_flag = true;cmv = false; psv_flag =false ;pc_ac_flag = false;PRESSURE_MODES_code = 22;break;		// psv_flag indicate that every trigger breath is on Pinsp
		//	case 3 : psv_flag =true; cmv = false;trigger_flag = true;pc_ac_flag = false;break;		//pc_ac_flag indicate that this is not flow cycled
			case 5 : psv_flag =true; cmv = false;trigger_flag = true;pc_ac_flag = true;PRESSURE_MODES_code =25;break;
			default : printf(" We are working on it \n");break;
		}
		flow_combine = flow_insp();
		clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                if(reading_timer > reading_set_time)
                {			pthread_mutex_trylock(&mutex);
                                	pressure_insp = calc_pressure;
					exp_pressure = exp_pressure_reading;
                                	pthread_mutex_unlock(&mutex);
                                	//printf("insp pressure  = %f  Exp pressure = %f \n", pressure_insp ,exp_pressure);

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
			clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
			Triggered_breath = true;
			printf(" Trigger Enable \n ");
//			printf("inp 1 =  %d \n" , timer+insp_time);
//			printf("inp 1 =  %d \n" , inhale_time + exhale_time);
		//	cpap_trigger_counter++;
		//	if((cpap_trigger_counter >4) && CPAP_flag)
		//	{
		//		break;
		//	}
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
		else
			Triggered_breath = false;//cpap_trigger_counter = 0;


		  // sum_of_press_vol_exp= sum_of_press_vol+ pressure_insp * (volume_final*-1);
                   //sum_of_volume_exp = sum_of_volume + volume_final * volume_final;
		  //Final_peep =
	//	printf( " time == %d \n" , cmv_exp_time);
	//	if(volume_final > 0 )
          //                      {
                //             //           printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                  //              }
               // else
                 //               {
                               //         printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                   //             }
                   
///////////////////////////////////////Inhale start////////////                   
		if((((cmv_exp_time +insp_time) >= (inhale_time+exhale_time)*1000)) ||  man_breath_flag || trigger_enable || trigger_mandatory)
		{//	if(Vi < final_setting.VTi)
	//		{
				//change_compliance = true;
	//		}
	//		else
				//change_compliance = false;
			//if(((final_setting.VTi - Vi) > 50) && !only_single_time)
			//{
			//	only_single_time = true;
			//}
			if(!patient_disconnection){
		/*	else*/ if( (Vi < (final_setting.VTi * 0.95))){
				//new_pressure = new_pressure + 1;
				compliance_pressure = compliance_pressure + 1;
				printf(" pressure is incremented \n");
				printf( "3\n");

			}
			else if( (Vi > (final_setting.VTi * 1.05))){
		//	else if(first_time && (Vi >= (final_setting.VTi + 20))){
			//	new_pressure = new_pressure - 1;
				compliance_pressure = compliance_pressure - 1;
				printf(" pressure is decremented \n");
				printf( "4\n");

			}
			else if(insp_time < (inhale_time * 1000)){
				compliance_pressure = compliance_pressure - 1;
                                printf(" pressure is decremented \n");
                                printf( "4\n");
				}
			}

//////////////////////////////// Backup //////////////////////////////// venti
		if(trigger_flag && (PRESSURE_MODES_code > 0) && (final_setting.apnea_on_off > 0 )){
 	        	clock_gettime(CLOCK_MONOTONIC_RAW, &backup_end);
	        	backup_timer = delta_t(&backup_start, &backup_end,&backup_deltat);
	        	printf(" backup timer %d \n",backup_timer);
	        	if(backup_timer > (final_setting.apnea_time * 1000)){
	                	printf(" We shifting to backup ventilation \n");
                        	CPAP_flag =true;
				serial_data_write(back_ventilation_activated);
                        	apnea_ventilation(PRESSURE_MODES_code);
                        	clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
				serial_data_write(backup_ventilation_end);
              		}
		}

///////////////////////////////backup ///////////////////////////////// venti.c

			Vi = 0;
			Final_peep = pressure_insp;
			Exhale_flag = false;
			if(!patient_disconnection &&!complianc_not_cal ){
				final_compliance = sum_of_volume /sum_of_press_vol;
				final_resistance = sum_of_press_flow/sum_of_flow;
				if(counter_volume_modes <4)
					counter_volume_modes++;
				if(only_single_time)
				{	first_complience = final_compliance;
					//compliance_

				}
				printf(" Value of first complience  = %f \n", first_complience);

			}

			//printf(" Final Resistance  ====================================  %f \n " , final_resistance);
			printf(" %d %d %d %d\n",counter_volume_modes, setting_changed ,only_single_time, !triggring_compliance);			//compliance_array
			current_complience = compliance_pressure;
			if((counter_volume_modes > 2) &&  (setting_changed || only_single_time) && !triggring_compliance && !patient_disconnection)
			{	printf(" Entering in Complience block \n");
				if(final_compliance > 0){
//					printf(" P = %f  v = %f comp = %f peep = %f sum_of_volume = %ld sum_of_press_vol = %d\n" ,compliance_pressure,final_setting.VTi,final_compliance,final_setting.peep,sum_of_volume,sum_of_press_vol);
					if((current_volume <= (final_setting.VTi * 0.80) )|| (current_volume >= (final_setting.VTi * 1.20) )) {
						current_volume = final_setting.VTi;
						compliance_pressure = final_setting.VTi / first_complience +  final_setting.peep;
//						pressure_80 =  compliance_pressure ;
						compliance_pressure = compliance_pressure * 0.80;
						counter_volume_modes = 0;
						only_single_time = false;
						setting_changed = false;
					}

				}
			}
			else if((counter_volume_modes <= 1) && only_single_time )
			{
				compliance_pressure = final_setting.VTi / VOLUME_FIXED_COMPLIANCE + final_setting.peep;
				printf(" Complience pressure &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& inside %f \n",compliance_pressure);
			}
			printf(" Complience === %f  ***********************************************\n" ,final_compliance); 
			printf(" Complience pressure &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& %f \n",compliance_pressure);
			if(compliance_pressure  >65)
				compliance_pressure = 65;
			//printf(" Final Compliance ====================================  %f \n " , final_compliance);
			//printf(" Final Pressure ========== %f \n", compliance_pressure);
			//final_compliance = (sum_of_volume+sum_of_volume_exp)/(sum_of_press_vol + sum_of_press_vol_exp);
		//	printf(" finAL Compliance both ================================ %f \n " ,final_compliance);
			sum_of_press_vol_exp = 0;
                        sum_of_volume_exp = 0;
			sum_of_press_vol = 0;
			sum_of_volume = 0;
			sum_of_flow = 0;
			sum_of_press_flow =0;
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
				printf("dutyyyyy cycle sirrrrrrrrrrrrrrrrr :%f\n" , duty_cycle);
//                        	valve_duty_cycle(98);  //sir

                        	clock_gettime(CLOCK_MONOTONIC_RAW, &hold_start_time);
                        	clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);

		        	hold_time = delta_t(&start_time, &end_time,&delta_hold);
                        	while ( (hold_time <= (Ehold_time * 1000)))
                        	{       flow_combine = flow_insp();
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
                        	valve_duty_cycle(98);
                        	bcm2835_gpio_clr(INHALE_VALVE);
                        	Ehold = false;
                        	Ehold_time = 0;
               			}//valve_duty_cycle(100); 
			clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
			turbine_duty_cycle(80);  // nidhi
			MVe_array[counter] = volume_final * final_setting.RR_rxd;
			counter++;
			if(counter >= 4 )
                        {

                                counter = 0;
                        }
						///
			if( rr_flag)
			{	pressure_alarm((float)pip);				//Alarm function
				Vte_alarm((float)Vi);					//
				RR_alarm((float)final_rr);				//
				PEEP_alarm((float)pressure_insp);			//
				Mve_alarm((float)MVe);					//

				MVe = ((MVe_array[0] +MVe_array[1] +MVe_array[2] +MVe_array[3] )/4.0);
				if(MVi <= 0 )
				{
					MVi = 1;
				}
				leak_flow  = LEAK_FLow(MVi,MVe);//((MVi - (-1) *MVe)/MVi)*100;
                        	if(leak_flow < 0)
					leak_flow = 0;
			}
			//printf(" leak ========== %f\n",leak_flow);
			if(leak_flow > 90){
				patient_disconnection = true;
				printf(" Patient disconnection alarm \n");
				serial_data_write("ACK05");
				}
			else{
				patient_disconnection = false;
				printf(" Patient connected alarm \n");
				serial_data_write("ACK06");
			    }
			Ve = volume_final;
			printf(" before compensation %f \n",Ve);

			Ve = Ve+1.2*(pip-cal_peep);
			printf(" after compensation %f \n",Ve);
			cal_peep = pressure_insp;
												//
			sprintf(d_string, "D@%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f#", round(pressure_insp) , final_rr  ,oxygen2_perc_cur ,(-1)*Pef_exp ,(-1)*MVe/1000.00 ,  leak ,mean_airway_pressure , Ve*-1 ,cmv_exp_time/1000.00);
			printf("%s  \n", d_string);
			printf("data ============== %d \n ", strlen(d_string));
			serial_data_write(d_string);    // Error Code
			Pef_exp = 0;
			 usleep(RISE_TIME);   //boost          //  nidhi
		//	printf(" Flag =====   %d \n" ,trigger_enable);
			if(trigger_enable)
			{	printf(" Triggered  pressure \n ");
//				duty_cycle = transfer_function((int)(final_setting.Psupp));
//				turbine_duty_cycle(duty_cycle);
			if(!psv_flag){
			//	pthread_mutex_trylock(&mutex);
                            //    slope_thread_global = true;
			//	trigger_thread = true;
                          //      pthread_mutex_unlock(&mutex);
                              	duty_cycle = transfer_function((int)(final_setting.Psupp));
                              	turbine_duty_cycle(duty_cycle);

				printf("PC-SIMV triggered breath\n");
	             //                   simv_exp_time = 0;
				}
			else
				{
				//pthread_mutex_trylock(&mutex);
                                //slope_thread_global = true;
                              //  trigger_thread = true;
				 duty_cycle = transfer_function((int)(compliance_pressure));
                                 turbine_duty_cycle(duty_cycle);
                                //pthread_mutex_unlock(&mutex);

				  printf("PSV triggered breath\n");
				}
			}
			else// if(trigger_enable)
			{	printf(" mandartory  pressure \n ");
				duty_cycle = transfer_function((int)(compliance_pressure));
                                turbine_duty_cycle(duty_cycle);
         //nidhi				printf("Duty_cycle4 = %d \n",duty_cycle);

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
			Pmean_count = 0;
			volume_final = 0;
			pthread_mutex_trylock(&mutex);
                        pressure_insp = calc_pressure;
                        pthread_mutex_unlock(&mutex);
			Pif_insp = 0;
			clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
			flow_combine = flow_insp();
			sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                        sum_of_volume = sum_of_volume + volume_final * volume_final;
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
			complianc_not_cal = false;
			while((timer < (inhale_time)* 1000))// && (volume_final <= final_setting.VTi ))  // && (flow_combine > (Pif_insp *flow_cycle_per))))// || (!trigger_enable && psv_flag && (timer < (inhale_time)* 1000) && (flow_combine > (Pif_insp * flow_cycle_per)))  )
			{	//	printf(" After Error 1 \n");
				//printf(" volume = %f \n" , volume_final);
				if(volume_final >= final_setting.VTi )
				{
					duty_cycle = transfer_function((int)(compliance_pressure * 0.80));
	                                turbine_duty_cycle(duty_cycle);

				}
				flow_combine = flow_insp();
				pip = max(pip , pressure_insp);
				resis_press = flow_combine * CIRCUIT_RESITANCE;//0.05326;
	//			printf(" Pressure = %f  flow_combine = %f  calcu_pressure = %f " , pressure_insp , flow_combine , resis_press);
				if(pressure_insp < resis_press){
					//printf(" Patiant Disconnected \n");
					complianc_not_cal = true;
					patient_disconnection=true;
				}
				else{
					patient_disconnection=false;
					//printf(" Patiant ----><---- \n");
				}

//				sum_of_press_vol= sum_of_press_vol+ pressure_insp * volume_final;
//				sum_of_volume = sum_of_volume + volume_final * volume_final;
				if(flow_combine > 0){
					sum_of_press_vol= sum_of_press_vol+ ((pressure_insp-Final_peep) * (volume_final));
                         		sum_of_volume = sum_of_volume + (volume_final * volume_final);
					sum_of_flow = sum_of_flow + (flow_combine *flow_combine);
					sum_of_press_flow = sum_of_press_flow + (pressure_insp  * flow_combine);
				//	printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
				}

//				sum_of_press_vol= sum_of_press_vol+ pressure_insp * volume_final;
//				sum_of_volume = sum_of_volume + volume_final * volume_final;
//				printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
		/*		if(volume_final > 0 )
                                {
                                        sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                                        sum_of_volume = sum_of_volume + volume_final * volume_final;
                                 //       printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                                }
                                else
                                {
                                        um_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final) * -1;
                                        sum_of_volume = sum_of_volume + volume_final * volume_final;
                                   //     printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                                }
7777*/
			//	sum_of_press_vol= sum_of_press_vol+ ((pressure_insp-Final_peep) * (volume_final));
                         //	sum_of_volume = sum_of_volume + (volume_final * volume_final);
				//printf("Sum of volume = %f sum of press and vol  =  %f \n",sum_of_volume,sum_of_press_vol);
			//	printf("%f ,%d\n" , flow_combine ,timer);
			//	printf(" flow_combine  == %f Pif_insp == %f \n", flow_combine  , Pif_insp * 0.25);
				//if((flow_combine < 1)  && (flow_combine > -4))
				//	flow_combine = 1;
				Pif_insp = max(Pif_insp,flow_combine);
				if(((flow_combine <= (int) (Pif_insp  * final_setting.insp_term/100.0)) && ( trigger_enable || psv_flag) && !pc_ac_flag && !slope_thread_global) && flow_combine >=1)
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
			pthread_mutex_trylock(&mutex);
                        slope_thread_global = false;
                        pthread_mutex_unlock(&mutex);
//			if(timer < (inhale_time * 1000))
///			{	 duty_cycle = transfer_function((int)(compliance_pressure * 0.80));
 //                                turbine_duty_cycle(duty_cycle);

//				while(timer< (inhale_time * 1000))
//				{

//				}
//			}

			//pressure_alarm(pip);
			if(!trigger_enable)
				insp_time = timer;
			else
				insp_trigger_time =timer;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
			if(Ihold && !trigger_enable){
			bcm2835_gpio_set(INHALE_VALVE);
/*			if(((pip > alarm_struct.Pressure_high )|| (pip < alarm_struct.Pressure_low ))&& !local_pressure_alarm)
			{		local_pressure_alarm = true;
					buzzer(5,local_pressure_alarm);
			}
			else if (((pip < alarm_struct.Pressure_high )|| (pip > alarm_struct.Pressure_low ))&& local_pressure_alarm)
			{
					local_pressure_alarm = false;
                                        buzzer(5,local_pressure_alarm);
			}

*/
				int new_pres = final_setting.Pinsp + final_setting.Pinsp*HOLD_TUBINE_FACTOR;
		//	(new_pres > 70)?new_pres=70:new_press=new_press;// 
				if(new_pres > 70)
				{
					new_pres = 70;
				}
				duty_cycle = transfer_function(new_pres);
                        	turbine_duty_cycle(duty_cycle);
				  printf("Duty_cycle6 = %d \n",duty_cycle);



				clock_gettime(CLOCK_MONOTONIC_RAW, &hold_start_time);
				clock_gettime(CLOCK_MONOTONIC_RAW, &hold_end_time);
                        	hold_time = delta_t(&start_time, &end_time,&delta_hold);
				while ( (hold_time <= (Ihold_time * 1000)))
				{	flow_combine = flow_insp();
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
				valve_duty_cycle(98);     //100   //nidhiiii_30
                                                //breaking_enable(pressure_insp , 0,0); //lk
			
    //BREAKING
/*  

                      int break_timer = 0, breaking_time;
                        float ratio = final_setting.peep/final_setting.Pinsp;
                        printf("Ratio = %f \n",ratio);
                        if (ratio > 0.8)
                        {
                                breaking_time = 8;
                                printf("Breaking time3 = 8ms \n");
                        }
                        if (ratio >= 0.15 && ratio <= 0.4)
                        {
                                breaking_time = 200;
                                printf("Breaking time2 = 200ms \n");

                        }

                        if (ratio > 0.4 && ratio <= 0.8)
                        {
                                breaking_time = 40;
                                printf("Breaking time4 = 40ms \n");

                        }

                        if (ratio < 0.15 && ratio >= 0.0)
                        {
                                breaking_time = 200;
                                printf("Breaking time1 = 200ms \n");

                        }
*/
			 int break_timer = 0;

                        int breaking_time = break_time(final_setting.peep,final_setting.Pinsp);

			turbine_duty_cycle(0);
                        break_on(); //LAKSHAY   //nidhi
			clock_gettime(CLOCK_MONOTONIC_RAW, &Break_start_time);
                        while(break_timer < breaking_time)
                        {
			
                       // printf("=================================breaking===================================================================== \n");
                       flow_combine = flow_insp();
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


                        clock_gettime(CLOCK_MONOTONIC_RAW, &Break_end_time);
                        break_timer = delta_t(&Break_start_time, &Break_end_time, &Break_deltat);
                        }
                        //usleep(BREAKING);//b delay
                          //nidhi
			break_off();
                        duty_cycle = transfer_function(final_setting.peep);
                        printf("PEEP => %f duty cycle => %d ",final_setting.peep,duty_cycle);
                        turbine_duty_cycle(duty_cycle);
                     
			 ///////////////////////////

	//		bcm2835_gpio_clr(BREAKING_PIN);
	//		usleep(TIME_DELAY_BREAKING);
	//		bcm2835_gpio_set(BREAKING_PIN);
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
                //	duty_cycle = transfer_function((int)(final_setting.peep));
                //	turbine_duty_cycle(duty_cycle);
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
			if(!patient_disconnection)
				previous_cycle_leak = leak_ayush_sir;
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
			flow_insp_flag = true;
	//		if(first_time){
	//			complience = compliance_function(Vi , pip , final_setting.peep);
	////		 	new_pressure = final_setting.VTi / complience + final_setting.peep;
	//			printf(" New_ pressure == %f \n ", new_pressure);
	//			printf(" New === %f \n" , complience);
	//			first_time = false;
	//			printf( "1\n");
	//		}
/*
			if(volume_final > 0 )
			{
				sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                        	sum_of_volume = sum_of_volume + volume_final * volume_final;
                        	//printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
			}
			else
			{
				sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final) * -1;
                                sum_of_volume = sum_of_volume + volume_final * volume_final;
                                //printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
			}*/
			sum_of_press_vol_exp= sum_of_press_vol+ pressure_insp * (volume_final)*-1;
                        sum_of_volume_exp = sum_of_volume + volume_final * volume_final;
			while((!trigger_enable &&(exp_time <  300)) ||  (trigger_enable && (timer < (300))))
			{

				if(!trigger_enable)
						breaking(pressure_insp, final_setting.peep,(int)exp_time);
				else
						breaking(pressure_insp, final_setting.peep,(int)timer);

				flow_combine = flow_insp();
				Pef_exp = min(Pef_exp,flow_combine);
				sum_of_press_vol_exp= sum_of_press_vol+ pressure_insp * (volume_final)*-1;
                        	sum_of_volume_exp = sum_of_volume + volume_final * volume_final;
			/*	if(volume_final > 0 )
                        	{
                                	sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                                	sum_of_volume = sum_of_volume + volume_final * volume_final;
                                	//printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                        	}
                        	else
                        	{
                                	sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final) * -1;
                                	sum_of_volume = sum_of_volume + volume_final * volume_final;
                                //	printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                        	}

	*/		//	sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                          //      sum_of_volume = sum_of_volume + volume_final * volume_final;
                            //    printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
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
					valve_duty_cycle(100);   ///100 ///////nidhi_30
				//	if(duty_cycle_valve_cal >= 100)
				//	{
					//	break;
					//	printf("breakinggggggggggggggggggggggggggggggggg at Timer = \n");
				//	}
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
			valve_duty_cycle(98);   ////nidhi_30
			TiTOT_slope = (insp_time * final_setting.slope /100.00)/(insp_time*1000.00 + exp_time*1000.00);
                        TiTOT_remain = (insp_time * (100 - final_setting.slope) /100.00)/(insp_time*1000.00 + exp_time*1000.00);
                        mean_airway_pressure_thread = ((pip - pressure_insp) * TiTOT_slope)  * 0.5;
                        mean_airway_pressure =  ((pip - pressure_insp) * TiTOT_remain) ;
                        mean_airway_pressure = mean_airway_pressure +mean_airway_pressure_thread + pressure_insp;
			printf(" Mean Airway pressure === %f \n", mean_airway_pressure);
	//		printf(" Pip = %f \n" , pip);
	//		printf(" Pressure_insp  = %f \n", pressure_insp);
	//		printf(" TiTOT = %f \n",TiTOT);
//			mean_airway_pressure = ((pip - pressure_insp) * TiTOT/100.00) + pressure_insp ;

		//	printf(" MVe == %f\n",MVe);
		//	printf(" Leak = %f \n" , leak);
	//		printf(" CMV_ exp _time  = %d\n" ,cmv_exp_time);

		}

	}

}
/**

void volume_modes() {
//	start_bus();
//	volume
	pthread_mutex_trylock(&mutex);
        thread_control_pressure = true;
        pthread_mutex_unlock(&mutex);
	volume_flag = false;
	float Final_peep = 0;
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
	char A_string[A_STRING_SIZE] = {0};
	char b_string[B_STRING_SIZE] = {0};
	char c_string[C_STRING_SIZE] = {0};
	char d_string[D_STRING_SIZE] = {0};
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
	float COMPLIENCE_pressure = 28;//final_setting.VTi/30.00 + final_setting.peep;
	duty_cycle = transfer_function((int)(final_setting.peep));
        turbine_duty_cycle(duty_cycle);
	Exhale_flag = false;
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
		flow_combine = flow_insp();
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
		//if(first_time)
			// COMPLIENCE_pressure = final_setting.VTi/HEALTHY_COMPLIENCE + pressure_insp;										//
		if((((cmv_exp_time +insp_time) >= (inhale_time+exhale_time)*1000)) ||  man_breath_flag || trigger_enable || trigger_mandatory)
		{	//exp_time = timer;
			Exhale_flag = false;
			Final_peep = pressure_insp;
			final_compliance = sum_of_volume/sum_of_press_vol;
                        printf(" Final Compliance ====================================  %f \n " , final_compliance);
			if(final_compliance < 0 )
				final_compliance = 28;
			COMPLIENCE_pressure = final_setting.VTi/final_compliance;
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
                        {       flow_combine = flow_insp();
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
                        valve_duty_cycle(98);
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
			flow_combine = flow_insp();
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
				sum_of_press_vol= sum_of_press_vol+ ((pressure_insp-Final_peep) * (volume_final));
                                sum_of_volume = sum_of_volume + (volume_final * volume_final);
				flow_combine = flow_insp();
				pip = max(pip , pressure_insp);
				Pif_insp = max(Pif_insp,flow_combine);
				//printf("%f ,%d\n" , pressure_insp ,timer);
			//	printf(" flow_combine  == %f Pif_insp == %f \n", flow_combine  , Pif_insp * 0.25);
			//	if((flow_combine < 0)  && (flow_combine > -4))
			//		flow_combine = 1;
				if((flow_combine <= (int) (Pif_insp  * final_setting.insp_term)) && ( trigger_enable || psv_flag) && !vc_ac_flag && !slope_thread_global &&(flow_combine > 1))
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
			{	flow_combine = flow_insp();
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
			Exhale_flag =true;
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
				flow_combine = flow_insp();
				Pef_exp = min(Pef_exp,flow_combine);
		       		pthread_mutex_trylock(&mutex);
				pressure_insp = calc_pressure;
				pthread_mutex_unlock(&mutex);
			//	float Kp= 0.1;
			//	float Kc = 1.4;
				if((pressure_insp <= (final_setting.peep )) && pressure_Exhale) //&& (((exp_time >= 300.0)))) //&& !trigger_enable ) || ((timer >= 300.0)&& trigger_enable )))
				{//	exp_flag = false;
				float i = 0;
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
				else if(exp_flag && ((int)pressure_insp >= (final_setting.peep)+2))
				{
					duty_cycle_valve_cal = duty_cycle_valve_cal - 1;
					if(duty_cycle_valve_cal <= 1)
                                        {
                                                duty_cycle_valve_cal = 1;
                                        }
					valve_duty_cycle(duty_cycle_valve_cal);

				}

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
			Ve = volume_final;
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

	//		printf(" Pip = %f \n" , pip);
	//		printf(" Pressure_insp  = %f \n", pressure_insp);
	//		printf(" TiTOT = %f \n",TiTOT);
//			mean_airway_pressure = ((pip - pressure_insp) * TiTOT/100.00) + pressure_insp ;
			TiTOT_slope = (insp_time * final_setting.slope /100.00)/(inhale_time*1000.00 + exhale_time*1000.00);
                        TiTOT_remain = (insp_time * (100 - final_setting.slope) /100.00)/(inhale_time*1000.00 + exhale_time*1000.00);
                        mean_airway_pressure_thread = ((pip - pressure_insp) * TiTOT_slope)  * 0.5;
			printf(" the pressure = %f \n" , mean_airway_pressure);
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
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;///
void stand_by(){
	//    pwm_init();
	    	int current_mode = 0;
	//	bcm2835_gpio_fsel(BREAKING_PIN,BCM2835_GPIO_FSEL_OUTP);
        	//bcm2835_gpio_clr(BREAKING_PIN); //lk
	//	usleep(100000);
		turbine_duty_cycle(0);
		usleep(50000);
		bcm2835_gpio_set(BREAKING_PIN);
	    	valve_duty_cycle(0);
	     	pthread_mutex_trylock(&mutex);
             	thread_control_pressure = false;
             	pthread_mutex_unlock(&mutex);
		//turbine_duty_cycle(0);
		clear_buzzer();
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
		//bcm2835_close(); //lk
		 pwm_init();
		 flow_init();
//		clear_file(); //lk

		both_calib(&flow_insp_error , &flow_exp_error);
		printf( " Insp Error = = %f \n" ,flow_insp_error);
        	printf( " Exp Error = = %f \n" ,flow_exp_error);
		bcm2835_gpio_clr(INHALE_VALVE);
		//turbine_constant();//lk
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
	struct timespec  backup_start = {0,0};
        struct timespec backup_end = {0,0};
//        struct timespec backup_reading = {0,0};
	struct timespec flow_leak_start = {0,0};
        struct timespec  flow_leak_end= {0,0};
        struct timespec deltat_flow_leak = {0,0};
	const int reading_set_time = READING_TIME;
	float previous_duty_cycle = 0;
	clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
	clear_buzzer();
	char A_string[] = {0};
	int reading_timer = 0;
        //clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
	float modified_pressure = 0;
	int current_mode = 0;
	float pressure_insp = 0;
	float flow_combine = 0;
	valve_duty_cycle(valve_cycle);
	insp_duty_cycle = transfer_function((int)(final_setting.peep));
        turbine_duty_cycle(insp_duty_cycle);
	usleep(1000000);
	int loop = 0;
	bool trigger_flag = false;
	Exhale_flag = true;
	float pip , Pif_insp = 0;
	int trigger = 0;
	int backup_timer = 0;
	CPAP_start = true;
	float leak_timer = 0;
	clock_gettime(CLOCK_MONOTONIC_RAW, &flow_leak_start);
	clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
	float Support_comp = final_setting.peep;
	float pressure_trigger = 3;
	float resis_press = 0;
	bool  patient_disconnection=false;
//	float backup_time = 20 * 1000; /// In milli second
	previous_cycle_leak = 0;
	leak_ayush_sir = 0;
	while(1){
		if(final_setting.Psupp  > Support_comp)
		{	pressure_trigger = final_setting.Trigger;
			if(pressure_trigger < final_setting.peep)
			{
					pressure_trigger = final_setting.peep-1;
			}
		}
		else
			pressure_trigger = 2;
		clock_gettime(CLOCK_MONOTONIC_RAW, &backup_end);
		backup_timer = delta_t(&backup_start, &backup_end,&backup_deltat);
		printf(" backup timer %d \n",backup_timer);
		if(backup_timer > (CPAP_DEFAULT_BACKUP_TIME * 1000)){
			printf(" We shifting to backup ventilation \n");
			CPAP_flag =true;
			//apnea_ventilation(CPAP_MODE_CODE); //lk
			clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
		}


/*		if(final_setting.Psupp  > final_setting.peep)
                {       pressure_trigger = 3;
                }
*/

		read_settings();
		Support_comp = final_setting.peep;
                current_mode = mode_change();
                if( current_mode/10 != mode_para/10)
                {
                //      mode_para = current_mode;
                        break;
                }
		flow_combine = flow_insp();
 //lk
 
	   resis_press = flow_combine * CIRCUIT_RESITANCE;//0.05326; //lk
                //printf(" Pressure = %f  flow_combine = %f  calcu_pressure = %f " , pressure_insp , flow_combine , resis_press$
                if(pressure_insp < resis_press){
					patient_disconnection=false;
                                        
                                       printf(" Patiant Disconnected \n");//lk
                                     //   complianc_not_cal = true;
                                       // patient_disconnection=true;
                                } 
                else{
                                        printf(" Patiant ----><---- \n");
                }


                pthread_mutex_trylock(&mutex);
                pressure_insp = calc_pressure;
                pthread_mutex_unlock(&mutex);
		printf(" Pressure_ = %f \n" , pressure_insp);
		printf(" algo diffrence  = %f \n " , (final_setting.peep - pressure_insp));
		clock_gettime(CLOCK_MONOTONIC_RAW, &flow_leak_end);
		clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
		leak_timer = delta_t(&flow_leak_start, &flow_leak_end,&deltat_flow_leak);
		if((leak_timer >= FLOW_TIME_SET) && !((final_setting.peep - pressure_insp) >= pressure_trigger))
		{
			clock_gettime(CLOCK_MONOTONIC_RAW, &flow_leak_start);
			//previous_cycle_leak = leak_ayush_sir;//_leak;
		}

                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
		if(reading_timer > reading_set_time)
                {
                                       // Pmean_array[Pmean_count] = pressure_insp;
                                       // Pmean_count++;
                                        sprintf(A_string, "A@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final,0.0 );
                //                      printf("%s \n " , A_string);
                                        serial_data_write(A_string);    // Error Code
                                        clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                }
//		if(m > 0.17)
//		{
//			trigger_flag = true;
//			printf(" Breath is triggered\n");
//		}

//		if(trigger_flag && (final_setting.Psupp > 0) && (final_setting.Psupp > final_setting.peep))
//		{
//			insp_duty_cycle = transfer_function(final_setting.Psupp);
 //	               	turbine_duty_cycle(insp_duty_cycle);
//			printf(" P supp pressure  setted  %f  M = %f\n",final_setting.Psupp , m);
//
//		}

//////////////////////////////////////////////////// For Psupport only /////////////////////////////////////////////////
///////////////////////########-------------------------------------------------------########/////////////////////////
	if(final_setting.Psupp > final_setting.peep){
		if( (int)(pressure_insp - final_setting.peep) >= pressure_trigger)
		{
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
		//	insp_duty_cycle = transfer_function((int)(modified_pressure));
                  //      turbine_duty_cycle(insp_duty_cycle);
		}
		else if((( final_setting.peep - pressure_insp) >= pressure_trigger) || ((pressure_insp < final_setting.peep) && (flow_combine > 5)  && (final_setting.Psupp > Support_comp))){// && flow_combine > 10){
				printf("Just got inside\n");
			if(final_setting.Psupp > Support_comp){
			     insp_duty_cycle = transfer_function(final_setting.Psupp+10);
                             turbine_duty_cycle(insp_duty_cycle);
			     usleep(20000);
                 	     insp_duty_cycle = transfer_function(final_setting.Psupp);
                 	     turbine_duty_cycle(insp_duty_cycle);
			}

			//printf("FLow = %f \n",flow_combine);
			while( (final_setting.Psupp > Support_comp))
			{

				read_settings();
				Support_comp = final_setting.peep;
                		current_mode = mode_change();
                		if( current_mode/10 != mode_para/10)
                		{
                		//      mode_para = current_mode;
                        		break;
                		}

				//printf(" insode while loop \n");
				valve_cycle = 100;
				valve_duty_cycle(valve_cycle);
				Triggered_breath = true;
				trigger_flag = true;
				//printf(" Psupport supply \n");
				flow_combine = flow_insp();
                        	pip = max(pip , pressure_insp);
				Pif_insp = max(Pif_insp,flow_combine);
				printf(" %f   %f \n", flow_combine,Pif_insp);
		                resis_press = flow_combine * CIRCUIT_RESITANCE;//0.05326;
        		        // printf(" Pressure = %f  flow_combine = %f  calcu_pressure = %f " , pressure_insp , flow_combine , resis_press$
                		 if(pressure_insp < resis_press){
                        	               printf(" Patiant Disconnected \n");
                                	 //      complianc_not_cal = true;
                                       // patient_disconnection=true;
                                }
				  else{
                			  patient_disconnection=false;
                                      
                                        printf(" Patiant ----><---- \n");
                		}
				if(flow_combine <= (int) (Pif_insp  * final_setting.insp_term/100.0))//) && ( trigger_enable || psv_flag) && !$
                        	        {      	printf(" Breakinnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnng \n");
						insp_duty_cycle = transfer_function(final_setting.peep);
               		       			turbine_duty_cycle(insp_duty_cycle);
						valve_duty_cycle(100);

                               			break;
                            	}
	                	pthread_mutex_trylock(&mutex);
        		        pressure_insp = calc_pressure;
                        //exp_pressure = exp_pressure_reading;
                 		pthread_mutex_unlock(&mutex);
				if(pressure_insp <=  final_setting.Psupp){
					modified_pressure = (final_setting.Psupp - pressure_insp)*0.2 * 2 + modified_pressure;
 
                                //valve_cycle = (2 )*0.4*(set_peep - pressure) + x
                                //	valve_cycle = 0;
                               	//	valve_duty_cycle(valve_cycle);
				}
				else
				{

					 //modified_pressure = (pressure_insp - final_setting.Psupp)*0.1*2 + modified_pressure;
					modified_pressure = (final_setting.Psupp - pressure_insp)*0.2 * 2 + modified_pressure; 
                                	 if(modified_pressure < final_setting.Psupp){
                                                modified_pressure = final_setting.Psupp;
						}
						turbine_duty_cycle(insp_duty_cycle);
				}
				if(modified_pressure > 60)
						modified_pressure  = 60;
    	        		insp_duty_cycle = transfer_function((int)(modified_pressure));
                          	turbine_duty_cycle(insp_duty_cycle);
				printf(" Modified pressure = %f \n ",modified_pressure);
                        //      printf(" insp pressure  = %f Exp pressure = %f \n", pressure_insp ,exp_pressure);
                        //      A_string = itoa(pressure);
                        	clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
				reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
				if(reading_timer > reading_set_time)
                       		{
                                //      Pmean_array[Pmean_count] = pressure_insp;
                                //      Pmean_count++;
                                       	sprintf(A_string, "A@%.2f,%.2f,%.2f,%d#",pressure_insp,flow_combine,volume_final,trigger );
                           		//printf("%s \n " , A_string);
					//printf(" Data is send \n");
                                        serial_data_write(A_string);    // Error Code
                                       	clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                     		}
				 //printf(" While looop End  \n");

				usleep(20000);
			clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);

			}
			Pif_insp = 0;
		if(!(final_setting.Psupp > Support_comp))
			{	modified_pressure = (final_setting.peep - pressure_insp)*0.2 * 2 + modified_pressure;
			//	insp_duty_cycle = transfer_function((int)(modified_pressure));
	                  //      turbine_duty_cycle(insp_duty_cycle);
				//valve_cycle = (2 )*0.4*(set_peep - pressure) + x
				valve_cycle = 0;
				valve_duty_cycle(valve_cycle);
			}
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
		else if ((( final_setting.peep - pressure_insp) >= pressure_trigger) && flow_combine > -2){
			if ( flow_combine > -2){
				valve_cycle = 6 * 0.4*(final_setting.peep - pressure_insp) + valve_cycle;
				modified_pressure = (pressure_insp - final_setting.peep)*0.1*2 + modified_pressure; 
				if(modified_pressure < final_setting.peep){
                                                modified_pressure = final_setting.peep;
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
		if(!(final_setting.Psupp > Support_comp)){
		 	if((final_setting.peep - pressure_insp) >=pressure_trigger){
					modified_pressure = ( final_setting.peep - pressure_insp)*0.2 * 2 + modified_pressure;
                        	//        insp_duty_cycle = transfer_function(modified_pressure);
                        	  //      turbine_duty_cycle(insp_duty_cycle);
                                	loop = 3;
				}
		}
		usleep(10000);
		if(modified_pressure > 60)
			modified_pressure = 60; 
		insp_duty_cycle = transfer_function(modified_pressure);
                turbine_duty_cycle(insp_duty_cycle);
		previous_duty_cycle = valve_cycle;
		//printf(" Modified  == %f Valve cuycle = %f loop = %d  pressure = %f flow = %f \n" , modified_pressure , valve_cycle,loop,pressure_insp,flow_combine);
		//printf(" %f \n", flow_combine);
		}

////////////////////////////////////////////////////Psupport Code End /////////////////////////////////////////////////
///////////////////////########-------------------------------------------------------########/////////////////////////

	else
	//	printf(" Support pressure is 0 \n");
	{
				if( (int)(pressure_insp - final_setting.peep) >= 2)
		{
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
		//	insp_duty_cycle = transfer_function((int)(modified_pressure));
                  //      turbine_duty_cycle(insp_duty_cycle);
		}
		else if((( final_setting.peep - pressure_insp) >= 2) && flow_combine > 10){
				modified_pressure = (final_setting.peep - pressure_insp)*0.2 * 2 + modified_pressure;
			//	insp_duty_cycle = transfer_function((int)(modified_pressure));
	                  //      turbine_duty_cycle(insp_duty_cycle);
				//valve_cycle = (2 )*0.4*(set_peep - pressure) + x
				 clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);

				valve_cycle = 0;
				valve_duty_cycle(valve_cycle);
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
		else if ((( final_setting.peep - pressure_insp) >= 2) && flow_combine > -2){
			if ( flow_combine > -2){
				 clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);

				valve_cycle = 6 * 0.4*(final_setting.peep - pressure_insp) + valve_cycle;
				modified_pressure = (pressure_insp - final_setting.peep)*0.1*2 + modified_pressure; 
				if(modified_pressure < final_setting.peep){
                                                modified_pressure = final_setting.peep;
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
		else if((final_setting.peep - pressure_insp) >=2){
				modified_pressure = ( final_setting.peep - pressure_insp)*0.2 * 2 + modified_pressure;
                        //        insp_duty_cycle = transfer_function(modified_pressure);
                          //      turbine_duty_cycle(insp_duty_cycle);
                                loop = 3;
		}
		usleep(20000);
		if(modified_pressure > 60)
			modified_pressure = 60;
		insp_duty_cycle = transfer_function(modified_pressure);
                turbine_duty_cycle(insp_duty_cycle);
		previous_duty_cycle = valve_cycle;
		printf(" Modified  == %f Valve cuycle = %f loop = %d  pressure = %f flow = %f \n" , modified_pressure , valve_cycle,loop,pressure_insp,flow_combine);
		//printf(" %f \n", flow_combine);
	}

	}

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Bpap_address
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void bipap(){
	pthread_mutex_trylock(&mutex);
        thread_control_pressure = true;
        pthread_mutex_unlock(&mutex);
	previous_cycle_leak = 0;
        volume_flag = false;
	float insp_duty_cycle = 0;
	float valve_cycle = 100;
	//int y = 0;
	struct timespec  reading_start = {0,0};
        struct timespec reading_end = {0,0};
        struct timespec deltat_reading = {0,0};
//	struct timespec  backup_start = {0,0};
  //      struct timespec backup_end = {0,0};
    //    struct timespec backup_reading = {0,0};
//	struct timespec flow_leak_start = {0,0};
//        struct timespec  flow_leak_end= {0,0};
 //      struct timespec deltat_flow_leak = {0,0};
     //   struct timespec TIMER_start = {0,0};
      //  struct timespec  TIMER_end= {0,0};
        //struct timespec TIMER_delta = {0,0};
	float Timer = 0;
	const int reading_set_time = READING_TIME;
	float previous_duty_cycle = 0;
	clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
//	char A_string[] = {0};
	int reading_timer = 0;
        //clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
	float modified_pressure = 0;
	int current_mode = 0;
	float pressure_insp = 0;
	float flow_combine = 0;
	valve_duty_cycle(valve_cycle);
	insp_duty_cycle = transfer_function((int)(final_setting.peep));
        turbine_duty_cycle(insp_duty_cycle);
	usleep(1000000);
	int loop = 0;
	bool trigger_flag = false;
	Exhale_flag = true;
	float pip , Pif_insp = 0;
	int trigger = 0;
	int backup_timer = 0;
	CPAP_start = true;
//	float leak_timer = 0;
//	clock_gettime(CLOCK_MONOTONIC_RAW, &flow_leak_start);
//	clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
	float Support_comp = final_setting.peep;
	float pressure_trigger = 3;
	float resis_press = 0;
//	bool  patient_disconnection=false;
//	float backup_time = 20 * 1000; /// In milli second
	float insp_time = 0;
	float exp_time = 0;
	float Vi = 0;
	float MVi = 0;
	float Ve = 0;
//	float Pef_exp = 0;
	char A_string[A_STRING_SIZE] = {0};
	char b_string[B_STRING_SIZE] = {0};
	char c_string[C_STRING_SIZE] = {0};
	char d_string[D_STRING_SIZE] = {0};
	int counter = 0;
	float RR_cal = 0;
	float check_rr = 0;
	float RR_array[4];
	float final_rr = 0;
	clear_buzzer();
	int Patient_disconnection_counter = 0;
	bool patient_disconnection = false;
	int patient_connection_counter = 0;
	int PRESSURE_MODES_code = 41;
	clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
        clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
	while(1){

	//	if(final_setting.Psupp  > Support_comp)
	//		pressure_trigger = 3;
		Pif_insp = 0;
		if(final_setting.Psupp  > Support_comp)
		{	pressure_trigger = final_setting.Trigger;
			if(pressure_trigger < final_setting.peep)
			{
					pressure_trigger = final_setting.peep-1;
			}
		}
		else
			pressure_trigger = 2;




//		clock_gettime(CLOCK_MONOTONIC_RAW, &backup_end);
//		backup_timer = delta_t(&backup_start, &backup_end,&backup_deltat);
//		printf(" backup timer %d \n",backup_timer);
//		if(backup_timer > (CPAP_DEFAULT_BACKUP_TIME * 1000)){
//			printf(" We shifting to backup ventilation \n");
//			CPAP_flag =true;
//			apnea_ventilation(CPAP_MODE_CODE);
//			clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
//		}


/*		if(final_setting.Psupp  > final_setting.peep)
                {       pressure_trigger = 3;
                }
*/

		read_settings();
		Support_comp = final_setting.peep;
                current_mode = mode_change();
                if( current_mode/10 != mode_para/10)
                {
                //      mode_para = current_mode;
                        break;
                }
		flow_combine = flow_insp();
                resis_press = flow_combine * CIRCUIT_RESITANCE;//0.05326;
                //printf(" Pressure = %f  flow_combine = %f  calcu_pressure = %f " , pressure_insp , flow_combine , resis_press$
                if(pressure_insp < resis_press){
                                        printf(" Patiant Disconnected \n");
					Patient_disconnection_counter++;
					patient_connection_counter = 0;
                                     //   complianc_not_cal = true;
                                       // patient_disconnection=true;
                	}
                else{
                                        Patient_disconnection_counter = 0;
					patient_connection_counter ++ ;
                                        printf(" Patiant ----><---- \n");
                	}
		if((Patient_disconnection_counter > PATIENT_DISCONNECTION_VALUE) && !patient_disconnection)
		{
			//	patient_disconnection = true;
				Patient_disconnection_counter = 0;
		}
		else if(patient_connection_counter > PATIENT_DISCONNECTION_VALUE)
			//patient_disconnection =false;

                pthread_mutex_trylock(&mutex);
                pressure_insp = calc_pressure;
                pthread_mutex_unlock(&mutex);
	//	printf(" Pressure_ = %f \n" , pressure_insp);
	//	printf(" algo diffrence  = %f \n " , (final_setting.peep - pressure_insp));
	//	clock_gettime(CLOCK_MONOTONIC_RAW, &flow_leak_end);
		clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
		//leak_timer = delta_t(&flow_leak_start, &flow_leak_end,&deltat_flow_leak);
//		if((leak_timer >= FLOW_TIME_SET) && !((final_setting.peep - pressure_insp) >= pressure_trigger))
//		{
//			clock_gettime(CLOCK_MONOTONIC_RAW, &flow_leak_start);
//			//previous_cycle_leak = leak_ayush_sir;//_leak;
//		}

                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
		if(reading_timer > reading_set_time)
                {
                                       // Pmean_array[Pmean_count] = pressure_insp;
                                       // Pmean_count++;
                                        sprintf(c_string, "C@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,0.0,0.0 );
                //                      printf("%s \n " , A_string);
                                        serial_data_write(c_string);    // Error Code
                                        clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                }

			clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
                	Timer = delta_t(&start_time, &end_time,&deltat);
			exp_time = Timer;
	//		printf("timer = %f  exp_time  == %f \n" , Timer,exp_time);
		if(trigger_flag && (PRESSURE_MODES_code > 0) && (final_setting.apnea_on_off > 0 )){
 	        	clock_gettime(CLOCK_MONOTONIC_RAW, &backup_end);
	        	backup_timer = delta_t(&backup_start, &backup_end,&backup_deltat);
	        	printf(" backup timer %d \n",backup_timer);
	        	if(backup_timer > (final_setting.apnea_time * 1000)){
	                	printf(" We shifting to backup ventilation \n");
				serial_data_write(back_ventilation_activated);
                        	CPAP_flag =true;
				backup_ventilation_alarm(1);
                        	apnea_ventilation(PRESSURE_MODES_code);
				serial_data_write(backup_ventilation_end);
				backup_ventilation_alarm(0);
                        	clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
              		}
		}

		if(  (insp_time + exp_time)  >=((inhale_time + exhale_time) * 1000))
		{	Ve = volume_final;

			RR_cal =  (int) ((60*1000)/(insp_time +exp_time));
			check_rr = ((10*1000*60)/(insp_time * 1.00 + exp_time * 1.00)) - (10 * (int)RR_cal);
			if(check_rr > 0.5)
			{
				RR_cal = RR_cal + 1;
			}

			RR_array[counter] = RR_cal;
			counter++;
			if(counter>=4)
			{
				final_rr = (RR_array[0] + RR_array[1] + RR_array[2]+ RR_array[3])/4.0;
				counter = 0;

			}

			sprintf(d_string, "D@%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f#", round(pressure_insp) , 0.0,oxygen2_perc_cur ,0.0 ,0.0 , 0.0  ,0.0 , 100.0 ,(float)exp_time/1000.00);
			printf("%s  \n", d_string);
			printf("data ============== %d \n ", strlen(d_string));
			serial_data_write(d_string);    // Error Code

			volume_final = 0;
			clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
			clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);

			Timer = delta_t(&start_time, &end_time,&deltat);
			insp_duty_cycle = transfer_function(final_setting.Pinsp+10);//lk
                        turbine_duty_cycle(insp_duty_cycle);
			printf("Duty = %f ****** pressureinsp = %f \n", insp_duty_cycle,final_setting.Pinsp); //lk
			usleep(20000);
                 	insp_duty_cycle = transfer_function(final_setting.Pinsp);
                 	turbine_duty_cycle(insp_duty_cycle);
			//printf("FLow = %f \n",flow_combine);
			//valve_cycle = 100;
			valve_duty_cycle(100.00);
			while((Timer < (inhale_time * 1000)))
			{
				clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
				Timer = delta_t(&start_time, &end_time,&deltat);
				//pip = max(pip , pressure_insp);

			//	printf("timer = %f   inhale_time  = %f \n" , Timer , inhale_time *1000);
				printf("timer = %f   inhale_time  = %f \n" , Timer , inhale_time *1000);
		//		printf("timer = %f   inhale_time  = %f \n" , Timer , inhale_time *1000);
		//		printf("timer = %f   inhale_time  = %f \n" , Timer , inhale_time *1000);
				//read_settings();
				//Support_comp = final_setting.peep;
                		//current_mode = mode_change();
                	//	if( current_mode/10 != mode_para/10)
                	//	{
                		//      mode_para = current_mode;
                        //		break;
                	//	}
				//printf(" insode while loop \n");;
			//	Triggered_breath = true;
			//	trigger_flag = true;
				//printf(" Psupport supply \n");
				printf("  Inside the loop for inhale  in pFLow The rogram \n");
				flow_combine = flow_insp();
                        	pip = max(pip , pressure_insp);
				Pif_insp = max(Pif_insp,flow_combine);
				//printf(" %f   %f \n", flow_combine,Pif_insp);
		                resis_press = flow_combine * CIRCUIT_RESITANCE;//0.05326;
        		        //printf(" Pressure = %f  flow_combine = %f  calcu_pressure = %f " , pressure_insp , flow_combine , resis_press$
/*                		if(pressure_insp < resis_press){
                        	                printf(" Patiant Disconnected \n");
                                	 //       complianc_not_cal = true;
                                       // patient_disconnection=true;
                                	}
                		else{
                                        patient_disconnection=false;
                                        printf(" Patiant ----><---- \n");
                		}

*/	/*			if(flow_combine <= (int) (Pif_insp  * final_setting.insp_term/100.0))//) && ( trigger_enable || psv_flag) && !$
                        	        {      	printf(" Breakinnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnng \n");
						insp_duty_cycle = transfer_function(final_setting.peep);
               		       			turbine_duty_cycle(insp_duty_cycle);
						valve_duty_cycle(100);

                               			break;
                            	}
*/
	                	pthread_mutex_trylock(&mutex);
        		        pressure_insp = calc_pressure;
                        //exp_pressure = exp_pressure_reading;
                 		pthread_mutex_unlock(&mutex);
				if(pressure_insp <=  final_setting.Pinsp){
					modified_pressure = (final_setting.Pinsp - pressure_insp)*0.2 * 2 + modified_pressure;

                                //valve_cycle = (2 )*0.4*(set_peep - pressure) + x
                                //	valve_cycle = 0;
                               	//	valve_duty_cycle(valve_cycle);
				}
				else
				{

					 //modified_pressure = (pressure_insp - final_setting.Psupp)*0.1*2 + modified_pressure;
					modified_pressure = (final_setting.Pinsp - pressure_insp)*0.2 * 2 + modified_pressure; 
                                	if(modified_pressure < final_setting.Pinsp){
                                                modified_pressure = final_setting.Pinsp;
						}
						turbine_duty_cycle(insp_duty_cycle);
				}
				if(modified_pressure > 60)
						modified_pressure  = 60;
    	        		insp_duty_cycle = transfer_function((int)(modified_pressure));
                          	turbine_duty_cycle(insp_duty_cycle);
				//printf(" Modified pressure = %f \n ",modified_pressure);
                        //      printf(" insp pressure  = %f Exp pressure = %f \n", pressure_insp ,exp_pressure);
                        //      A_string = itoa(pressure);
                        	clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
				reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
				if(reading_timer > reading_set_time)
                       		{
                                //      Pmean_array[Pmean_count] = pressure_insp;
                                //      Pmean_count++;
                                       	sprintf(A_string, "A@%.2f,%.2f,%.2f,%d#",pressure_insp,flow_combine,volume_final,trigger );
                           		//printf("%s \n " , A_string);
					//printf(" Data is send \n");
                                        serial_data_write(A_string);    // Error Code
                                       	clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                     		}
				 //printf(" While looop End  \n");

				usleep(20000);


				//clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
			}
			float cal_peep = 3;
			float rise_time = 0;
			insp_time = Timer;
			Vi = volume_final;
			Vi = Vi-1.2*(pip-cal_peep);
			MVi = 0;
			turbine_duty_cycle(final_setting.peep);
			sprintf(b_string, "B@%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f#",round(pip) , Vi , Pif_insp ,  0.0, MVi/1000.0, 0.0,insp_time/1000.00 , pressure_insp,0,rise_time/1000.00);
			serial_data_write(b_string);
			printf(" %s \n",b_string);
			clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
			pip = 0;
			Pif_insp = 0;
			//previous_cycle_leak = leak_ayush_sir;
			valve_duty_cycle(97);

			while(Timer < MANDATORY_EXHALE)
			{
				clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
                        	Timer = delta_t(&start_time, &end_time,&deltat);
				clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
    	                	reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                                	if(reading_timer > reading_set_time)
                                	{
                                	//      Pmean_array[Pmean_count] = pressure_insp;
                                	//      Pmean_count++;
                                        	sprintf(c_string, "C@%.2f,%.2f,%.2f,%d#",pressure_insp,flow_combine,volume_final,trigger );

                                        //printf("%s \n " , A_string);
                                        //printf(" Data is send \n");
                                  	      serial_data_write(c_string);    // Error Code
                                  	      clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                                	}
		usleep(1000);

		}

		}
			printf(" outside the ;loop\n");
                        clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
                        Timer = delta_t(&start_time, &end_time,&deltat);


//		if(m > 0.17)
//		{
//			trigger_flag = true;
//			printf(" Breath is triggered\n");
//		}

//		if(trigger_flag && (final_setting.Psupp > 0) && (final_setting.Psupp > final_setting.peep))
//		{
//			insp_duty_cycle = transfer_function(final_setting.Psupp);
 //	               	turbine_duty_cycle(insp_duty_cycle);
//			printf(" P supp pressure  setted  %f  M = %f\n",final_setting.Psupp , m);
//
//		}

//////////////////////////////////////////////////// For Psupport only /////////////////////////////////////////////////
///////////////////////########-------------------------------------------------------########/////////////////////////
//	if(final_setting.Psupp > final_setting.peep){
		if( (int)(pressure_insp - final_setting.peep) >= pressure_trigger)
		{
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
		//	insp_duty_cycle = transfer_function((int)(modified_pressure));
                  //      turbine_duty_cycle(insp_duty_cycle);
		}
		else if((( final_setting.peep - pressure_insp) >= pressure_trigger) || ((pressure_insp < final_setting.peep) && (flow_combine > 5)  && (final_setting.Psupp > Support_comp)) && (!patient_disconnection)){// && flow_combine > 10){

				volume_final = 0;
				printf("Just got inside\n");
			//sprintf(d_string, "D@%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f#", round(pressure_insp) , 00,21 ,0 ,0 , 0  ,0 , 0 ,exp_time/1000.00,0);
			//printf("%s  \n", d_string);
			serial_data_write(d_string);    // Error Code
			if(final_setting.Psupp > Support_comp){
			     insp_duty_cycle = transfer_function(final_setting.Psupp+10);
                             turbine_duty_cycle(insp_duty_cycle);
			     usleep(20000);
                 	     insp_duty_cycle = transfer_function(final_setting.Psupp);
                 	     turbine_duty_cycle(insp_duty_cycle);
			}

			printf("FLow = %f \n",flow_combine);
			while( (final_setting.Psupp > Support_comp))
			{

				read_settings();
				Support_comp = final_setting.peep;
                		current_mode = mode_change();
                		if( current_mode/10 != mode_para/10)
                		{
                		//      mode_para = current_mode;
                        		break;
                		}
				//printf(" insode while loop \n");
				valve_cycle = 100;
				valve_duty_cycle(valve_cycle);
				Triggered_breath = true;
				trigger_flag = true;
				//printf(" Psupport supply \n");
				flow_combine = flow_insp();
                        	pip = max(pip , pressure_insp);
				Pif_insp = max(Pif_insp,(flow_combine));
				printf(" %f   %f \n", flow_combine,Pif_insp);
		                resis_press = flow_combine * CIRCUIT_RESITANCE;//0.05326;
        		        //printf(" Pressure = %f  flow_combine = %f  calcu_pressure = %f " , pressure_insp , flow_combine , resis_press$
                		if(pressure_insp < resis_press){
                        	                printf(" Patiant Disconnected \n");
						Patient_disconnection_counter++;
                                	 //       complianc_not_cal = true;
                                       // patient_disconnection=true;
                                	}
                		else{
                                        patient_disconnection=false;
					Patient_disconnection_counter = 0;
                                        printf(" Patiant ----><---- \n");
                		}
	//			if(Patient_disconnection_counter > PATIENT_DISCONNECTION_VALUE)
	//			{
	//				patient_disconnection = true;
	//				break;
	//			}
				printf(" Support pressure breath\n");
				if(flow_combine <= (int) (Pif_insp  * final_setting.insp_term/100.0))//) && ( trigger_enable || psv_flag) && !$
                        	        {      	printf(" Breakinnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnng \n");
						insp_duty_cycle = transfer_function(final_setting.peep);
               		       			turbine_duty_cycle(insp_duty_cycle);
						valve_duty_cycle(100);

                               			break;
                            	}
	                	pthread_mutex_trylock(&mutex);
        		        pressure_insp = calc_pressure;
                        //exp_pressure = exp_pressure_reading;
                 		pthread_mutex_unlock(&mutex);
				if(pressure_insp <=  final_setting.Psupp){
					modified_pressure = (final_setting.Psupp - pressure_insp)*0.2 * 2 + modified_pressure;

                                //valve_cycle = (2 )*0.4*(set_peep - pressure) + x
                                //	valve_cycle = 0;
                               	//	valve_duty_cycle(valve_cycle);
				}
				else
				{

					 //modified_pressure = (pressure_insp - final_setting.Psupp)*0.1*2 + modified_pressure;
					modified_pressure = (final_setting.Psupp - pressure_insp)*0.2 * 2 + modified_pressure; 
                                	 if(modified_pressure < final_setting.Psupp){
                                                modified_pressure = final_setting.Psupp;
						}
						turbine_duty_cycle(insp_duty_cycle);
				}
				if(modified_pressure > 60)
						modified_pressure  = 60;
    	        		insp_duty_cycle = transfer_function((int)(modified_pressure));
                          	turbine_duty_cycle(insp_duty_cycle);
				printf(" Modified pressure = %f \n ",modified_pressure);
                        //      printf(" insp pressure  = %f Exp pressure = %f \n", pressure_insp ,exp_pressure);
                        //      A_string = itoa(pressure);
                        	clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
				reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
				if(reading_timer > reading_set_time)
                       		{
                                //      Pmean_array[Pmean_count] = pressure_insp;
                                //      Pmean_count++;
                                       	sprintf(A_string, "A@%.2f,%.2f,%.2f,%d#",pressure_insp,flow_combine,volume_final,trigger );
                           		//printf("%s \n " , A_string);
					//printf(" Data is send \n");
                                        serial_data_write(A_string);    // Error Code
                                       	clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                     		}
				 //printf(" While looop End  \n");

				usleep(20000);
			//clock_gettime(CLOCK_MONOTONIC_RAW, &backup_start);
				//Pif_insp = 0;
			}


		if(!(final_setting.Psupp > Support_comp))
			{	modified_pressure = (final_setting.peep - pressure_insp)*0.2 * 2 + modified_pressure;
			//	insp_duty_cycle = transfer_function((int)(modified_pressure));
	                  //      turbine_duty_cycle(insp_duty_cycle);
				//valve_cycle = (2 )*0.4*(set_peep - pressure) + x
				valve_cycle = 0;
				valve_duty_cycle(valve_cycle);
			}

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
		else if ((( final_setting.peep - pressure_insp) >= pressure_trigger) && flow_combine > -2){
			if ( flow_combine > -2){
				valve_cycle = 6 * 0.4*(final_setting.peep - pressure_insp) + valve_cycle;
				modified_pressure = (pressure_insp - final_setting.peep)*0.1*2 + modified_pressure; 
				if(modified_pressure < final_setting.peep){
                                                modified_pressure = final_setting.peep;
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
		if(!(final_setting.Psupp > Support_comp)){
		 	if((final_setting.peep - pressure_insp) >=pressure_trigger){
					modified_pressure = ( final_setting.peep - pressure_insp)*0.2 * 2 + modified_pressure;
                        	//        insp_duty_cycle = transfer_function(modified_pressure);
                        	  //      turbine_duty_cycle(insp_duty_cycle);
                                	loop = 3;
				}
		}
		usleep(10000);
		if(modified_pressure > 60)
			modified_pressure = 60; 
		insp_duty_cycle = transfer_function(modified_pressure);
                turbine_duty_cycle(insp_duty_cycle);
		previous_duty_cycle = valve_cycle;
		//printf(" Modified  == %f Valve cuycle = %f loop = %d  pressure = %f flow = %f \n" , modified_pressure , valve_cycle,loop,pressure_insp,flow_combine);
		//printf(" %f \n", flow_combine);
//		}

////////////////////////////////////////////////////Psupport Code End /////////////////////////////////////////////////
///////////////////////########-------------------------------------------------------########/////////////////////////


}


}
/*
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
//	char A_string[] = {0};
//	char b_string[] = {0};
//	char c_string[] = {0};
//	char d_string[] = {0};
	char A_string[A_STRING_SIZE] = {0};
	char b_string[B_STRING_SIZE] = {0};
	char c_string[C_STRING_SIZE] = {0};
	char d_string[D_STRING_SIZE] = {0};
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
	printf(" Backup ventilation start \n");
	 trigger_flag = true;
	cmv = false;
	psv_flag =false ;
	pc_ac_flag = false;
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
			case 1 :		// psv_flag indicate that every trigger breath is on Pinsp
		//	case 3 : psv_flag =true; cmv = false;trigger_flag = true;pc_ac_flag = false;break;		//pc_ac_flag indicate that this is not flow cycled
		//	case 7 : psv_flag =true; cmv = false;trigger_flag = true;pc_ac_flag = true;break;
			default : printf(" We are working on it \n");break;
		}
		flow_combine = flow_insp();
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
		//	Exhale_flag = false;
		//	previous_cycle_leak =leak_ayush_sir;
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
			flow_combine = flow_insp();
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
				flow_combine = flow_insp();
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
				flow_combine = flow_insp();
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
				float i = 0;
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
				else if(exp_flag && ((int)pressure_insp >= (final_setting.peep)+2))
				{
					duty_cycle_valve_cal = duty_cycle_valve_cal - 1;
					if(duty_cycle_valve_cal <= 1)
                                        {
                                                duty_cycle_valve_cal = 1;
                                        }
					valve_duty_cycle(duty_cycle_valve_cal);

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
			Exhale_flag = true;
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
*/
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
//	int fio2_cal = 21;
//	float PFRe_cal = 0;
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
//	float pmean = 0;
//	float final_flow =0;
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
//	char A_string[] = {0};
//	char b_string[] = {0};
//	char c_string[] = {0};
//	char d_string[] = {0};
	char A_string[A_STRING_SIZE] = {0};
	char b_string[B_STRING_SIZE] = {0};
	char c_string[C_STRING_SIZE] = {0};
	char d_string[D_STRING_SIZE] = {0};
	int current_mode = 0;
	Pmean_array =  (int*)malloc(300*sizeof(float));
	MVe_array = (int*)malloc(4*sizeof(float));
	MVi_array = (int*)malloc(4*sizeof(float));
	RR_array =  (int*)malloc(4*sizeof(int));			// Allocate array with the help of malloc
//	turbine_duty_cycle(0);
	clock_gettime(CLOCK_MONOTONIC_RAW, &start_time);
	clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
//	bool trigger_flag = false;
	bool trigger_enable = false;
	bool trigger_mandatory = false;
//	bool cmv =false;
//	bool psv_flag = false;
	float mean_airway_pressure = 0;
	float TiTOT_slope =0;// insp_time * final_setting.slope /100.00;
        float TiTOT_remain = 0;// insp_time * (100 - final_setting.slope) /100.00;
        float mean_airway_pressure_thread = 0;
//	float flow_cycle_per = 0.25;
//	bool pc_ac_flag = false;
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
		flow_combine = flow_insp();
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
                        {       flow_combine = flow_insp();
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
			flow_combine = flow_insp();
			clock_gettime(CLOCK_MONOTONIC_RAW, &end_time);
                        timer = delta_t(&start_time, &end_time,&deltat);
			while((timer < (inhale_time)* 1000))  // && (flow_combine > (Pif_insp *flow_cycle_per))))// || (!trigger_enable && psv_flag && (timer < (inhale_time)* 1000) && (flow_combine > (Pif_insp * flow_cycle_per)))  )
			{	//	printf(" After Error 1 \n");
				flow_combine = flow_insp();
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
			{	flow_combine = flow_insp();
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
			//breaking_enable(pressure_insp , 0,0); //lk
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
				
			//	if(!trigger_enable)
						breaking(pressure_insp, final_setting.peep,(int)exp_time);
			//	else
			//			breaking(pressure_insp, final_setting.peep,(int)timer);
				flow_combine = flow_insp();
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


void INThandler(int sig)
{
        signal(sig, SIG_IGN);
        turbine_duty_cycle(0);
        //printf("Stop\n");
        exit(0);
}
int main()
{

system("config-pin P9_18 spi");
system("config-pin P9_21 spi");
system("config-pin P9_22 spi_sclk"); //nidhi
system("config-pin P9_42 pwm >/dev/null 2>&1"); //lk
	//sleep(10);
//	FILE *pointer;
  //      pointer = fopen(mode_path,"r");
    //    if(pointer == NULL)
      //  {
        //        printf("Error in readinfg file \n");
              //return 0;                                                      // Return -1 if file is not opened
   //     }
     //   fprintf(pointer , "%s" , "90");
       // fclose(pointer);

	gettimeofday(&start, NULL);// timer start
//	int mode_number  = 0;
	break_init();
	pwm_init();
 valve_direction();              //nidhi
        signal(SIGINT, INThandler); //lk
//	bcm2835_gpio_fsel(BREAKING_PIN,BCM2835_GPIO_FSEL_OUTP);
  //      bcm2835_gpio_clr(BREAKING_PIN);
    //    usleep(100000);
	turbine_duty_cycle(0);
//	sleep(10);
//	usleep(100000);
  //      bcm2835_gpio_set(BREAKING_PIN);


        valve_duty_cycle(0);
	flow_init();
	//clear_file();
	//turbine_constant();
	//flow_constant();

	//printf(" Array before %s \n" , buzzer_array);
/////////////////////////////////////////////////////////////
bcm2835_gpio_fsel(INHALE_VALVE,"out"); //lk
	//bcm2835_gpio_fsel(INHALE_VALVE,BCM2835_GPIO_FSEL_OUTP); //lk
	bcm2835_gpio_clr(INHALE_VALVE);
	read_settings();
	clear_buzzer();
//	both_calib(&flow_insp_error , &flow_exp_error);
//	printf( " Insp Error = = %f \n" ,flow_insp_error);
//	printf( " Exp Error = = %f \n" ,flow_exp_error);

//	flow_insp_error = insp_calib();
//	flow_exp_error = exp_calib();
	thread_exit = true;
	thread_control_pressure = true;
	start_bus();
	pthread_mutex_init(&mutex,NULL);
	pthread_t pressure_thread;
	pthread_t slope_thread;
	pthread_t exhale_valve_thread;

//	pthread_mutex_trylock(&mutex);
  //                      slope_thread_control = slope_thread_global; 
    //                    pthread_mutex_unlock(&mutex); 
	//final_setting.slope_flag = 1.0;
	//nidhi error(pthread_create(&slope_thread,NULL,slope_thread_parallel,&final_setting));
        error(pthread_create(&pressure_thread,NULL,press_thread,NULL));

//        error(pthread_create(&slope_thread,NULL,slope_thread_parallel,&final_setting));
    //    error(pthread_create(&exhale_valve_thread,NULL,exhale_valve_control,NULL));




  //      pthread_create(&floww_thread,NULL,flow_thread,NULL);

	while(1)
	{
		mode_para = mode_change();
	//	printf(" mode  ==== %f \n",mode_para);
		switch(mode_para/10)
		{
			case 1 : pressure_modes();break;
			case 2 : printf(" Volume mode is under development\n");volume_modes();break;
			case 3 : printf(" We are working on CPAP and bipap \n"); cpap();break;
			case 4 : bipap();break;
			case 5 : printf(" Aprv selectd \n");APRV_mode();break;
			case 9 : stand_by() ;printf("  Stand buy mode is under development \n");break;
			default : printf(" Invalid  string \n");break;
//	printf("%d \n ", x);
//	pressure_modes();

		}

}
		return 0;
}

int apnea_ventilation(int mode_code)
{
	pthread_mutex_trylock(&mutex);
        thread_control_pressure = true;
        pthread_mutex_unlock(&mutex);

	volume_flag = false;
//	bool local_pressure_alarm = false;
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
//	int fio2_cal = 21;
//	float PFRe_cal = 0;
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
//	float pmean = 0;
//	float final_flow =0;
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
	char A_string[A_STRING_SIZE] = {0};
	char b_string[B_STRING_SIZE] = {0};
	char c_string[C_STRING_SIZE] = {0};
	char d_string[D_STRING_SIZE] = {0};
	int current_mode = 0;
	//Pmean_array =  (int*)malloc(300*sizeof(float));
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
//	float flow_cycle_per = 0.25;
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
	float Final_peep = 0;
	float compliance_pressure = 28;
	int counter_volume_modes = 0;
	bool only_single_time = true;
	float sum_of_flow =0 ,sum_of_press_flow = 0,final_resistance = 0;
	bool triggring_compliance = false;
	float pressure_80 = 0;
//	float complience_array[5];
	float current_complience = 0;
	float first_complience = 0;
	float current_volume = 0;
	bool complianc_not_cal  = false;
	float resis_press = 0;
	int cpap_trigger_counter = 0;
	trigger_flag = true;
	cmv = false;
	psv_flag =false;
	pc_ac_flag = false;//break;
	//int cpap_trigger_counter = 0;
	while(1)
	{
		alarm_setting_funct();
		read_settings();
		current_mode = mode_change();
		printf("Current mode = %d mode =%d \n",current_mode,mode_code); 
       		if( current_mode != mode_code)
        	{
                //	mode_para = current_mode;
			free(MVe_array);
			free(MVi_array);
			free(RR_array);
                	break;
        	}
		triggring_compliance = trigger_enable;
		trigger_mandatory = false;
		trigger_enable = false;
//		printf(" Pressure == %f Time == %d    %f\n" , pressure_insp ,exp_time,duty_cycle_\valve_cal);
                resis_press = flow_combine * CIRCUIT_RESITANCE;//0.05326;
                //printf(" Pressure = %f  flow_combine = %f  calcu_pressure = %f " , pressure_insp , flow_combine , resis_press$
                if(pressure_insp < resis_press){
                                        printf(" Patiant Disconnected \n");
                                        complianc_not_cal = true;
                                        patient_disconnection=true;
                            }
                else{
                                        patient_disconnection=false;
                                        printf(" Patiant ----><---- \n");
                }

//		switch(current_mode%10)
//		{
//			case 1 : trigger_flag =false ;cmv = true;psv_flag =false; pc_ac_flag = false;break;		// Trigger flag indicate for pc_simv
//			case 2 : trigger_flag = true;cmv = false; psv_flag =false ;pc_ac_flag = false;break;		// psv_flag indicate that every trigger breath is on Pinsp
		//	case 3 : psv_flag =true; cmv = false;trigger_flag = true;pc_ac_flag = false;break;		//pc_ac_flag indicate that this is not flow cycled
//			case 5 : psv_flag =true; cmv = false;trigger_flag = true;pc_ac_flag = true;break;
//			default : printf(" We are working on it \n");break;
//		}
		printf(" Apnea is started  %d \n",mode_code);
		flow_combine = flow_insp();
		clock_gettime(CLOCK_MONOTONIC_RAW, &reading_end);
                reading_timer = delta_t(&reading_start, &reading_end,&deltat_reading);
                if(reading_timer > reading_set_time)
                {			pthread_mutex_trylock(&mutex);
                                	pressure_insp = calc_pressure;
					exp_pressure = exp_pressure_reading;
                                	pthread_mutex_unlock(&mutex);
                                	//printf("insp pressure  = %f  Exp pressure = %f \n", pressure_insp ,exp_pressure);

				//	pthread_mutex_unlock(&mutex);
					sprintf(c_string, "C@%.2f,%.2f,%.2f,%.2f#",pressure_insp,flow_combine,volume_final * -1,TiTOT);
					serial_data_write(c_string);
                                        clock_gettime(CLOCK_MONOTONIC_RAW, &reading_start);
                }
		clock_gettime(CLOCK_MONOTONIC_RAW, &end_time_exhale);
		exp_time =  delta_t(&start_time_exhale, &end_time_exhale,&deltat);
		cmv_exp_time = exp_time;
		if(( trigger_flag && (flow_combine  > final_setting.apnea_trigger_flow)) && !patient_disconnection)
		{
			Triggered_breath = true;
			printf(" Trigger Enable \n ");
//			printf("inp 1 =  %d \n" , timer+insp_time);
//			printf("inp 1 =  %d \n" , inhale_time + exhale_time);
		//	cpap_trigger_counter++;
		//	if((cpap_trigger_counter >4) && CPAP_flag)
		//	{
		//		break;
		//	}
			if((exp_time ) > ((final_setting.apnea_Te) * 1000 * 0.80))
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
		else
			Triggered_breath = false;//cpap_trigger_counter = 0;


		  // sum_of_press_vol_exp= sum_of_press_vol+ pressure_insp * (volume_final*-1);
                   //sum_of_volume_exp = sum_of_volume + volume_final * volume_final;
		  //Final_peep =
	//	printf( " time == %d \n" , cmv_exp_time);
	//	if(volume_final > 0 )
          //                      {
                //             //           printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                  //              }
               // else
                 //               {
                               //         printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                   //             }
		if((((cmv_exp_time +insp_time) >= (final_setting.apnea_Ti+final_setting.apnea_Te)*1000)) ||  man_breath_flag || trigger_enable || trigger_mandatory)
		{//	if(Vi < final_setting.VTi)
	//		{
				//change_compliance = true;
	//		}
	//		else
				//change_compliance = false;
			//if(((final_setting.apnea_Vt - Vi) > 50) && !only_single_time)
			//{
			//	only_single_time = true;
			//}
			if(!patient_disconnection){
		/*	else*/ if( (Vi < (final_setting.apnea_Vt * 0.95))){
				//new_pressure = new_pressure + 1;
				compliance_pressure = compliance_pressure + 1;
				printf(" pressure is incremented \n");
				printf( "3\n");

			}
			else if( (Vi > (final_setting.apnea_Vt * 1.05))){
		//	else if(first_time && (Vi >= (final_setting.apnea_Vt + 20))){
			//	new_pressure = new_pressure - 1;
				compliance_pressure = compliance_pressure - 1;
				printf(" pressure is decremented \n");
				printf( "4\n");

			}
			else if(insp_time < (final_setting.apnea_Ti * 1000)){
				compliance_pressure = compliance_pressure - 1;
                                printf(" pressure is decremented \n");
                                printf( "4\n");
				}
			}

			Vi = 0;
			Final_peep = pressure_insp;
			Exhale_flag = false;
			if(!patient_disconnection &&!complianc_not_cal ){
				final_compliance = sum_of_volume /sum_of_press_vol;
				final_resistance = sum_of_press_flow/sum_of_flow;
				if(counter_volume_modes <4)
					counter_volume_modes++;
			}
			if(only_single_time)
			{	first_complience = final_compliance;
				//compliance_

			}
			//change_compliance = false;
			if(trigger_enable ||trigger_mandatory)
			{
				//printf("inp 1 =  %d \n" , inhale_time + exhale_time);
				cpap_trigger_counter++;
				printf(" cpap trigger counter ================================================ %d\n",cpap_trigger_counter);
				if((cpap_trigger_counter > 3) && CPAP_flag)
				{
					//exit();
					printf(" Shifting back to CPAP \n");
					return 0 ;
				}
			}
			else{
				cpap_trigger_counter = 0;
				printf(" Cnvert Cpap trigger 0\n");
			}

			//printf(" Final Resistance  ====================================  %f \n " , final_resistance);
			printf(" %d %d %d %d\n",counter_volume_modes, setting_changed ,only_single_time, !triggring_compliance);
			//compliance_array
			current_complience = compliance_pressure;
			if((counter_volume_modes > 2) &&  (setting_changed || only_single_time) && !triggring_compliance && !patient_disconnection)
			{	printf(" Entering in Complience block \n");
				if(final_compliance > 0){
	//				printf(" P = %f  v = %f comp = %f peep = %f sum_of_volume = %ld sum_of_press_vol = %d\n" ,compliance_pressure,final_setting.apnea_Vt,final_compliance,final_setting.peep,sum_of_volume,sum_of_press_vol);
					if(current_volume != final_setting.apnea_Vt){
						current_volume = final_setting.apnea_Vt;
						compliance_pressure = final_setting.apnea_Vt / first_complience + final_setting.peep;
						//pressure_80 =  compliance_pressure ;
						compliance_pressure = compliance_pressure * 0.80;
						counter_volume_modes = 0;
						only_single_time = false;
						setting_changed = false;
					}

				}
			}
			else if(counter_volume_modes < 1)
				compliance_pressure = final_setting.apnea_Vt / VOLUME_FIXED_COMPLIANCE + final_setting.peep;
			printf(" Complience pressure &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&& %f \n",compliance_pressure);
			if(compliance_pressure  >65)
				compliance_pressure = 65;
			//printf(" Final Compliance ====================================  %f \n " , final_compliance);
			//printf(" Final Pressure ========== %f \n", compliance_pressure);
			//final_compliance = (sum_of_volume+sum_of_volume_exp)/(sum_of_press_vol + sum_of_press_vol_exp);
		//	printf(" finAL Compliance both ================================ %f \n " ,final_compliance);
			sum_of_press_vol_exp = 0;
                        sum_of_volume_exp = 0;
			sum_of_press_vol = 0;
			sum_of_volume = 0;
			sum_of_flow = 0;
			sum_of_press_flow =0;
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
                        	{       flow_combine = flow_insp();
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
                        	valve_duty_cycle(98);
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
				pressure_alarm((float)pip);				//Alarm function
				Vte_alarm((float)Vi);					//
				RR_alarm((float)final_rr);				//
				PEEP_alarm((float)pressure_insp);			//
				Mve_alarm((float)MVe);					//

				MVe = ((MVe_array[0] +MVe_array[1] +MVe_array[2] +MVe_array[3] )/4.0);
				if(MVi <= 0 )
				{
					MVi = 1;
				}
				leak_flow  = LEAK_FLow(MVi,MVe);//((MVi - (-1) *MVe)/MVi)*100;
                        	if(leak_flow < 0)
					leak_flow = 0;
			}
			//printf(" leak ========== %f\n",leak_flow);
			if(leak_flow > 90){
				patient_disconnection = true;
				printf(" Patient disconnection alarm \n");
				serial_data_write("ACK05");
				}
			else{
				patient_disconnection = false;
				printf(" Patient connected alarm \n");
				serial_data_write("ACK06");
			    }
			Ve = volume_final;
			printf(" before compensation %f \n",Ve);

			Ve = Ve+1.2*(pip-cal_peep);
			printf(" after compensation %f \n",Ve);
			cal_peep = pressure_insp;
												//
			sprintf(d_string, "D@%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f#", round(pressure_insp) , final_rr , 21 ,(-1)*Pef_exp ,(-1)*MVe/1000.00 ,  leak ,mean_airway_pressure , Ve*-1 ,cmv_exp_time/1000.00);
			printf("%s  \n", d_string);
			printf("data ============== %d \n ", strlen(d_string));
			serial_data_write(d_string);    // Error Code
			Pef_exp = 0;
		//	printf(" Flag =====   %d \n" ,trigger_enable);
			if(trigger_enable)
			{	printf(" Triggered  pressure \n ");
//				duty_cycle = transfer_function((int)(final_setting.Psupp));
//				turbine_duty_cycle(duty_cycle);
			if(!psv_flag){
			//	pthread_mutex_trylock(&mutex);
                            //    slope_thread_global = true;
			//	trigger_thread = true;
                          //      pthread_mutex_unlock(&mutex);
                              	duty_cycle = transfer_function((int)(final_setting.Psupp));
                              	turbine_duty_cycle(duty_cycle);

				printf("PC-SIMV triggered breath\n");
	             //                   simv_exp_time = 0;
				}
			else
				{
				//pthread_mutex_trylock(&mutex);
                                //slope_thread_global = true;
                              //  trigger_thread = true;
                                //pthread_mutex_unlock(&mutex);
				 duty_cycle = transfer_function((int)(compliance_pressure));
                                 turbine_duty_cycle(duty_cycle);

				  printf("PSV triggered breath\n");
				}
			}
			else// if(trigger_enable)
			{	printf(" mandartory  pressure \n ");
				duty_cycle = transfer_function((int)(compliance_pressure));
                                turbine_duty_cycle(duty_cycle);
			//	pthread_mutex_trylock(&mutex);
                        //	slope_thread_global = true;
                       	//	pthread_mutex_unlock(&mutex);
				simv_exp_time = 0;
			}
			clock_gettime(CLOCK_MONOTONIC_RAW, &volume_start_time);
			valve_duty_cycle(100);
//			pressure_alarm((float)pip);				//Alarm function
//			Vte_alarm((float)Vi);					//
//			RR_alarm((float)final_rr);				//
//			PEEP_alarm((float)pressure_insp);			//
//			Mve_alarm((float)MVe);					//
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
			flow_combine = flow_insp();
			sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                        sum_of_volume = sum_of_volume + volume_final * volume_final;
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
			complianc_not_cal = false;
			while((timer < (final_setting.apnea_Ti)* 1000))// && (volume_final <= final_setting.apnea_Vt ))  // && (flow_combine > (Pif_insp *flow_cycle_per))))// || (!trigger_enable && psv_flag && (timer < (inhale_time)* 1000) && (flow_combine > (Pif_insp * flow_cycle_per)))  )
			{	//	printf(" After Error 1 \n");
				//printf(" volume = %f \n" , volume_final);
				if(volume_final >= final_setting.apnea_Vt )
                                {
                                        duty_cycle = transfer_function((int)(compliance_pressure * 0.80));
                                        turbine_duty_cycle(duty_cycle);
                                }


				flow_combine = flow_insp();
				pip = max(pip , pressure_insp);
				resis_press = flow_combine * CIRCUIT_RESITANCE;//0.05326;
				printf(" Pressure = %f  flow_combine = %f  calcu_pressure = %f " , pressure_insp , flow_combine , resis_press);
				if(pressure_insp < resis_press){
					printf(" Patiant Disconnected \n");
					complianc_not_cal = true;
					patient_disconnection=true;
				}
				else{
					patient_disconnection=false;
					printf(" Patiant ----><---- \n");
				}

//				sum_of_press_vol= sum_of_press_vol+ pressure_insp * volume_final;
//				sum_of_volume = sum_of_volume + volume_final * volume_final;
				if(flow_combine > 0){
					sum_of_press_vol= sum_of_press_vol+ ((pressure_insp-Final_peep) * (volume_final));
                         		sum_of_volume = sum_of_volume + (volume_final * volume_final);
					sum_of_flow = sum_of_flow + (flow_combine *flow_combine);
					sum_of_press_flow = sum_of_press_flow + (pressure_insp  * flow_combine);
					printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
				}

//				sum_of_press_vol= sum_of_press_vol+ pressure_insp * volume_final;
//				sum_of_volume = sum_of_volume + volume_final * volume_final;
//				printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
		/*		if(volume_final > 0 )
                                {
                                        sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                                        sum_of_volume = sum_of_volume + volume_final * volume_final;
                                 //       printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                                }
                                else
                                {
                                        um_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final) * -1;
                                        sum_of_volume = sum_of_volume + volume_final * volume_final;
                                   //     printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                                }
*/
			//	sum_of_press_vol= sum_of_press_vol+ ((pressure_insp-Final_peep) * (volume_final));
                         //	sum_of_volume = sum_of_volume + (volume_final * volume_final);
				//printf("Sum of volume = %f sum of press and vol  =  %f \n",sum_of_volume,sum_of_press_vol);
			//	printf("%f ,%d\n" , flow_combine ,timer);
			//	printf(" flow_combine  == %f Pif_insp == %f \n", flow_combine  , Pif_insp * 0.25);
				//if((flow_combine < 1)  && (flow_combine > -4))
				//	flow_combine = 1;
				Pif_insp = max(Pif_insp,flow_combine);
				if(((flow_combine <= (int) (Pif_insp  * final_setting.insp_term/100.0)) && ( trigger_enable || psv_flag) && !pc_ac_flag && !slope_thread_global) && flow_combine >=1)
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
			pthread_mutex_trylock(&mutex);
                        slope_thread_global = false;
                        pthread_mutex_unlock(&mutex);
			//pressure_alarm(pip);
			if(!trigger_enable)
				insp_time = timer;
			else
				insp_trigger_time =timer;
	//////////////////////////////////////////////////////////////////////////////////////////////////////////
			if(Ihold && !trigger_enable){
			bcm2835_gpio_set(INHALE_VALVE);
/*			if(((pip > alarm_struct.Pressure_high )|| (pip < alarm_struct.Pressure_low ))&& !local_pressure_alarm)
			{		local_pressure_alarm = true;
					buzzer(5,local_pressure_alarm);
			}
			else if (((pip < alarm_struct.Pressure_high )|| (pip > alarm_struct.Pressure_low ))&& local_pressure_alarm)
			{
					local_pressure_alarm = false;
                                        buzzer(5,local_pressure_alarm);
			}

*/
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
				{	flow_combine = flow_insp();
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
                        //breaking_enable(pressure_insp , 0,0); //lk

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
			//Vi = Vi-1.2*(pip-cal_peep);
			printf(" after compensation %f \n",Vi);
			MVi_array[counter] = Vi *  final_setting.apnea_RR;
		//	printf("%d \n ", counter);
			if(rr_flag)
			{	final_rr = (RR_array[0]  +RR_array[1] +RR_array[2] +RR_array[3]) /4.0 ;
				MVi =( MVi_array[0] +MVi_array[1] +MVi_array[2] +MVi_array[3] )/4.00;
			}
			if(!patient_disconnection)
				previous_cycle_leak = leak_ayush_sir;
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
			TiTOT = (insp_time/(inhale_time*1000.00 + final_setting.apnea_Te*1000.00)) *100;
		//	printf(" Ti/tot = %f \n" ,TiTOT);
			Exhale_flag = true;
			flow_insp_flag = true;
	//		if(first_time){
	//			complience = compliance_function(Vi , pip , final_setting.peep);
	////		 	new_pressure = final_setting.apnea_Vt / complience + final_setting.peep;
	//			printf(" New_ pressure == %f \n ", new_pressure);
	//			printf(" New === %f \n" , complience);
	//			first_time = false;
	//			printf( "1\n");
	//		}
/*
			if(volume_final > 0 )
			{
				sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                        	sum_of_volume = sum_of_volume + volume_final * volume_final;
                        	//printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
			}
			else
			{
				sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final) * -1;
                                sum_of_volume = sum_of_volume + volume_final * volume_final;
                                //printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
			}*/
			sum_of_press_vol_exp= sum_of_press_vol+ pressure_insp * (volume_final)*-1;
                        sum_of_volume_exp = sum_of_volume + volume_final * volume_final;
			while((!trigger_enable &&(exp_time <  300)) ||  (trigger_enable && (timer < (300))))
			{

				if(!trigger_enable)
						breaking(pressure_insp, final_setting.peep,(int)exp_time);
				else
						breaking(pressure_insp, final_setting.peep,(int)timer);

				flow_combine = flow_insp();
				Pef_exp = min(Pef_exp,flow_combine);
				sum_of_press_vol_exp= sum_of_press_vol+ pressure_insp * (volume_final)*-1;
                        	sum_of_volume_exp = sum_of_volume + volume_final * volume_final;
			/*	if(volume_final > 0 )
                        	{
                                	sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                                	sum_of_volume = sum_of_volume + volume_final * volume_final;
                                	//printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                        	}
                        	else
                        	{
                                	sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final) * -1;
                                	sum_of_volume = sum_of_volume + volume_final * volume_final;
                                //	printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
                        	}

	*/		//	sum_of_press_vol= sum_of_press_vol+ pressure_insp * (volume_final);
                          //      sum_of_volume = sum_of_volume + volume_final * volume_final;
                            //    printf("sum of press and vol  == %Lf  sum of volume square = %Lf \n",sum_of_press_vol,sum_of_volume);
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
					//	break;
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
			valve_duty_cycle(98);
			TiTOT_slope = (insp_time * final_setting.slope /100.00)/(insp_time*1000.00 + exp_time*1000.00);
                        TiTOT_remain = (insp_time * (100 - final_setting.slope) /100.00)/(insp_time*1000.00 + exp_time*1000.00);
                        mean_airway_pressure_thread = ((pip - pressure_insp) * TiTOT_slope)  * 0.5;
                        mean_airway_pressure =  ((pip - pressure_insp) * TiTOT_remain) ;
                        mean_airway_pressure = mean_airway_pressure +mean_airway_pressure_thread + pressure_insp;
			printf(" Mean Airway pressure === %f \n", mean_airway_pressure);
	//		printf(" Pip = %f \n" , pip);
	//		printf(" Pressure_insp  = %f \n", pressure_insp);
	//		printf(" TiTOT = %f \n",TiTOT);
//			mean_airway_pressure = ((pip - pressure_insp) * TiTOT/100.00) + pressure_insp ;

		//	printf(" MVe == %f\n",MVe);
		//	printf(" Leak = %f \n" , leak);
	//		printf(" CMV_ exp _time  = %d\n" ,cmv_exp_time);

		}

	}
	return 0;
}
