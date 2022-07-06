//#include "define.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include "sir_valve_turbine.h"
#include"exhale_valve.h"
#include "error.h"


/////////////    Error defination for  diffrent types of errors /////////////////////
char error_gpio60_direction_open[]      = "Error in opening /sys/class/gpio/gpio60/value\n";
char error_gpio60_direction_write[]     = "Unable to write /sys/class/gpio/gpio60/value\n";
//char error_inhale_valve_open[]          ="Error in opening /sys/class/gpio/gpio60/value\n";
//char error_inhale_valve_write[]         ="Unable to write /sys/class/gpio/gpio60/value\n"; 
char error_exhale_valve_open[]          ="Error in opening /sys/class/gpio/gpio60/value\n"; 
char error_exhale_valve_write[]         = "Unable to write /sys/class/gpio/gpio60/value\n"; 
//char pin_export_path_1[]                  = "/sys/class/gpio/gpio60/value"  ;



/*void pin_export(void)
{
  if((access(pin_export_path_1 , F_OK ) == 0))
    {				// location check for pwm0. if pwm0 is avialable then they skip export part 
	printf( " gpio pin is already populated dont need to export \n");
    }

 else
  {
    int fd1 = open("/sys/class/gpio/export", O_WRONLY);
    if (fd1 == -1) {
        perror("Unable to open /sys/class/gpio/export");
        exit(1);
    }

    if (write(fd1, "32", 2) != 2) {
        perror("Error writing to /sys/class/gpio/export");
        exit(1);
    }


    close(fd1);
}
}*/
void valve_direction(void)
{
    int fd2 = open("/sys/class/gpio/gpio60/direction",O_WRONLY); //EXHALE VALVE OUTPUT MODE
    if (fd2 == -1)
    {
        perror("Unable to open /sys/class/gpio/gpio60/direction");
        //syslog_function(error_gpio60_direction_open);
	exit(1);
    }

    if (write(fd2, "out", 3) != 3)
    {
        perror("Error writing to /sys/class/gpio/gpio60/direction");
        //syslog_function(error_gpio60_direction_write);
	exit(1);
    }

      close(fd2);

}

void valve_open(void)
{
                 int fd3 = open("/sys/class/gpio/gpio60/value",O_WRONLY);
                 if (fd3 == -1)
    {
                 perror("Unable to open /sys/class/gpio/gpio60/value");
		// syslog_function(error_exhale_valve_open);
                 exit(1);
    }

                 if (write(fd3, "0", 1) != 1)
    {
                 perror("Error writing to /sys/class/gpio/gpio60/value");
		 //syslog_function(error_exhale_valve_write);
                 exit(1);
    }

                  close(fd3);
}


void valve_close(void)

{      
	  int fd4 = open("/sys/class/gpio/gpio60/value",O_WRONLY);
        if (fd4 == -1)
    {
        perror("Unable to open /sys/class/gpio/gpio60/value");
        //syslog_function(error_inhale_valve_open);
        exit(1);
    }

         if (write(fd4, "1", 1) != 1)
    {
        perror("Error writing to /sys/class/gpio/gpio60/value");
        //syslog_function(error_inhale_valve_write);
        exit(1);
    }
    close(fd4);
}
/*void main()
{
//pin_export();
valve_direction();


while (1)
{
valve_open();
sleep(2);
valve_close();
sleep(2);
}
} */


/*void pin_unexport(void)
{
int fd8 = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd8 == -1) {
        perror("Unable to open /sys/class/gpio/unexport");
        exit(1);
    }

    if (write(fd8, "34", 2) != 2) {
        perror("Error writing to /sys/class/gpio/unexport");
        exit(1);
    }

    close(fd8);
}*/




