//#include "define.h"
#include "bcm2835.h"
#include "sir_valve_turbine.h"
#include "error.h"
#include "exhale_valve.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>






float nanosec;
int nanoint;
char string[20];
//void pwm_init(void);
//int pwm_write_duty(int duty);

/////////////    Error defination for  diffrent types of errors /////////////////////
char error_period_open[]                ="Error in opening /dev/pwm/ecap0/period\n";
char error_period_write[]               ="unable to write /dev/pwm/ecap0/period\n";
char error_duty_cycle_open[]            ="Error in opening /dev/pwm/ecap0/duty_cycle\n";
char error_duty_cycle_write[]           ="unable to write /dev/pwm/ecap0/duty_cycle\n";
char error_enable_open[]                ="Error in opening /dev/pwm/ecap0/enable\n";
char error_enable_write[]               ="unable to write /dev/pwm/ecap0/enable\n";
//char pwmchip0_path[]                    ="/dev/pwm/ecap0/period"; // PWM chip 0 location after export

void pwm_init(void)

 {
  /*  if((access(pwmchip0_path , F_OK ) == 0))
    {				// location check for pwm0. if pwm0 is avialable then they skip export part 
	printf( " Chip is already populated dont need to export \n");
    }

    else
    {
    	int fd11 = open("/sys/class/pwm/pwmchip0/export", O_WRONLY);
    if (fd11 == -1)
    {
        perror("Unable to open /sys/class/pwm/pwmchip0/export");
        //syslog_function(error_period_open);
        exit(1);
    }

    if (write(fd11, "1", 1) != 1)
    {
        perror("Error writing to /sys/class/pwm/pwmchip0/export");
        //syslog_function(error_period_write);
        exit(1);
    }
    printf("turbine is still running ");

    close(fd11);
} */

int fd1 = open("/dev/pwm/ecap0/period", O_WRONLY);
    if (fd1 == -1)
    {
        perror("Unable to open /dev/pwm/ecap0/period");
	syslog_function(error_period_open);
        exit(1);
    }

    if (write(fd1, "20000000", 8) != 8)
    {
        perror("Error writing to /dev/pwm/ecap0/period");
	syslog_function(error_period_write);
        exit(1);
    }

    close(fd1);



int fd3 = open("/dev/pwm/ecap0/enable", O_WRONLY);
    if (fd3 == -1)
    {
        perror("Unable to open /dev/pwm/ecap0/enable");
	syslog_function(error_enable_open);
        exit(1);
    }

    if (write(fd3, "1", 1) != 1)
    {
        perror("Error writing to /dev/pwm/ecap0/enable");
	syslog_function(error_enable_write);
        exit(1);
    }

    close(fd3);

/*int fd4 = open("/dev/pwm/ecap0/polarity", O_WRONLY);
    if (fd4 == -1)
    {
        perror("Unable to open /dev/pwm/ecap0/polarity");
	//syslog_function(error_period_open);
        exit(1);
    }

    if (write(fd4, "inversed", 8) != 8)
    {
        perror("Error writing to /dev/pwm/ecap0/polarity");
	//syslog_function(error_period_write);
        exit(1);
    }

    close(fd4);*/
}

int turbine_duty_cycle(float duty)//int pwm_write_duty(int duty)
{          //duty = duty/10                       //nidhi
	  
           nanosec=(duty/100.0)*20000000;
           nanoint=(int)(nanosec);
           sprintf(string , "%d",nanoint);

/*int turbine_duty_cycle(float duty)//int pwm_write_duty(int duty)
{          //duty = duty/10 ;                      //nidhi
	if (duty > 1000)
	{ 
	 duty=1000;
	 nanosec=(duty/1000.0)*20000000;
         nanoint=(int)(nanosec);
         sprintf(string , "%d",nanoint);
//	 printf("dutyyyyy changeeddddddddddddd");

	}
else
        {   nanosec=(duty/1000.0)*20000000;
           nanoint=(int)(nanosec);
           sprintf(string , "%d",nanoint);
	}

*/


int fd2 = open("/dev/pwm/ecap0/duty_cycle", O_WRONLY);
    if (fd2 == -1)
    {
        perror("Unable to open /dev/pwm/ecap0/duty_cycle");
	syslog_function(error_duty_cycle_open);

        exit(1);
    }

    if (write(fd2, string, 8) != 8)
    {
        perror("Error writing to /dev/pwm/ecap0/duty_cycle");
	syslog_function(error_duty_cycle_write);
        exit(1);
    }

    close(fd2);
}

/*int fd12 = open("/sys/class/pwm/pwmchip0/unexport", O_WRONLY);
    if (fd1 == -1)
    {
        perror("Unable to open /sys/class/pwm/pwmchip0/unexport");
        //syslog_function(error_period_open);
        exit(1);
    }

    if (write(fd12, "1", 1) != 1)
    {
        perror("Error writing to /sys/class/pwm/pwmchip0/unexport");
        //syslog_function(error_period_write);
        exit(1);
    }

    close(fd12);*/

//}

/*
{
	int pwm_write_duty(int tdc);
	return 0;
}
*/
//int valve_duty_cycle(float vdc)
int valve_duty_cycle(int vdc)
{
	if (vdc < 100)
	 { valve_open();
	 }
        else
	 valve_close();
	 
}
 

//int pwm_write_duty(int vdc);
//return 0;
//}






