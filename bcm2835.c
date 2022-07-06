#include "bcm2835.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
/////////////    Error defination for  diffrent types of errors /////////////////////
char error_gpio46_direction_open[]      = "Error in opening /sys/class/gpio/gpio46/direction\n";
char error_gpio46_direction_write[]     = "Unable to write /sys/class/gpio/gpio46/direction\n";
char error_inhale_valve_open[]          ="Error in opening /sys/class/gpio/gpio46/value\n";
char error_inhale_valve_write[]         ="Unable to write /sys/class/gpio/gpio46/value\n";
//char error_exhale_valve_open[]          ="Error in opening /sys/class/gpio/gpio46/value\n";
//char error_exhale_valve_write[]         = "Unable to write /sys/class/gpio/gpio46/value\n";
//char pin_export_path[]                  = "/sys/class/gpio/gpio46/value"  ;
////////////////////
//char pin_export_path_breaking[]         = "/sys/class/gpio/gpio48/value"  ;
char error_gpio48_direction_open[]      = "Error in opening /sys/class/gpio/gpio48/direction\n";
char error_gpio48_direction_write[]     = "Unable to write /sys/class/gpio/gpio48/direction\n";
char error_inhale_valve_open_breaking[]          ="Error in opening /sys/class/gpio/gpio48/value\n";
char error_inhale_valve_write_breaking[]         ="Unable to write /sys/class/gpio/gpio48/value\n";

char error_gpio115_direction_open[]      = "Error in opening /sys/class/gpio/gpio115/direction\n";
char error_gpio115_direction_write[]     = "Unable to write /sys/class/gpio/gpio115/direction\n";
char error_inhale_valve_open_breaking2[]          ="Error in opening /sys/class/gpio/gpio115/value\n";
char error_inhale_valve_write_breaking2[]         ="Unable to write /sys/class/gpio/gpio115/value\n";

char error_gpio27_direction_open[]      = "Error in opening /sys/class/gpio/gpio27/direction\n";
char error_gpio27_direction_write[]     = "Unable to write /sys/class/gpio/gpio27/direction\n";
char error_inhale_valve_open_breaking3[]          ="Error in opening /sys/class/gpio/gpio27/value\n";
char error_inhale_valve_write_breaking3[]         ="Unable to write /sys/class/gpio/gpio27/value\n";


//char error_exhale_valve_open[]          ="Error in opening /sys/class/gpio/gpio46/value\n";
//char error_exhale_valve_write[]         = "Unable to write /sys/class/gpio/gpio46/value\n";
//char pin_export_path[]                  = "/sys/class/gpio/gpio46/value"  ;
//char *mode;
void bcm2835_gpio_fsel(int pin, char *mode)
{

	if (pin == 46)

	{

    		int fd2 = open("/sys/class/gpio/gpio46/direction",O_WRONLY); //EXHALE VALVE OUTPUT MODE
    		if (fd2 == -1)
    		{
        		perror("Unable to open /sys/class/gpio/gpio46/direction");
        		//syslog_function(error_PB2_direction_open);
			exit(1);
    		}

    		if (write(fd2, mode, strlen(mode)) !=  strlen(mode))
    		{
        		perror("Error writing to /sys/class/gpio/gpio46/direction");
        		//syslog_function(error_PB2_direction_write);
			exit(1);
    		}

    		close(fd2);
	}

/////////////////////////////
else if (pin == 48)

        {


                int fd2 = open("/sys/class/gpio/gpio48/direction",O_WRONLY); //EXHALE VALVE OUTPUT MODE
                if (fd2 == -1)
                {
                        perror("Unable to open /sys/class/gpio/gpio48/direction");
                        //syslog_function(error_PB2_direction_open);
                        exit(1);
                }

                if (write(fd2, mode, strlen(mode)) !=  strlen(mode))
                {
                        perror("Error writing to /sys/class/gpio/gpio48/direction");
                        //syslog_function(error_PB2_direction_write);
                        exit(1);
                }

                close(fd2);
	}




else if (pin == 115)

        {

                int fd2 = open("/sys/class/gpio/gpio115/direction",O_WRONLY); //EXHALE VALVE OUTPUT MODE
                if (fd2 == -1)
                {
                        perror("Unable to open /sys/class/gpio/gpio115/direction");
                        //syslog_function(error_PB2_direction_open);
                        exit(1);
                }

                if (write(fd2, mode, strlen(mode)) !=  strlen(mode))
                {
                        perror("Error writing to /sys/class/gpio/gpio115/direction");
                        //syslog_function(error_PB2_direction_write);
                        exit(1);
                }

                close(fd2);

        }


else if (pin == 27)

        {

                int fd2 = open("/sys/class/gpio/gpio27/direction",O_WRONLY); //EXHALE VALVE OUTPUT MODE
                if (fd2 == -1)
                {
                        perror("Unable to open /sys/class/gpio/gpio27/direction");
                        //syslog_function(error_PB2_direction_open);
                        exit(1);
                }

                if (write(fd2, mode, strlen(mode)) !=  strlen(mode))
                {
                        perror("Error writing to /sys/class/gpio/gpio27/direction");
                        //syslog_function(error_PB2_direction_write);
                        exit(1);
                }

                close(fd2);

        }




	else
	{
		printf("Wrong pin");
	}


}

/////////////////////////////
/* if (pin == 121)

        {

                if((access(pin_export_path_breaking , F_OK ) == 0))
                        {                               // location check for pwm0. if pwm0 is avialable then they skip export part 
                        printf( " gpio pin is already populated dont need to export \n");
                        }

                else
                {
                        int fd1 = open("/sys/class/gpio/export", O_WRONLY);
                        if (fd1 == -1) {
                        perror("Unable to open /sys/class/gpio/export");
                        exit(1);
                        }

                if (write(fd1, pin_s, strlen(pin_s)) != strlen(pin_s)) 
                        {
                        perror("Error writing to /sys/class/gpio/export");
                        exit(1);
                        }


                close(fd1);

                }
///////////////////////////////////////////////////////
                int fd3 = open("/sys/class/gpio/gpio48/direction",O_WRONLY); //EXHALE VALVE OUTPUT MODE
                if (fd3 == -1)
                {
                        perror("Unable to open /sys/class/gpio/gpio48/direction");
                        //syslog_function(error_gpio46_direction_open);
                        exit(1);
                }

                if (write(fd3, mode, strlen(mode)) !=  strlen(mode))
                {
                        perror("Error writing to /sys/class/gpio/gpio48/direction");
                        //syslog_function(error_gpio46_direction_write);
                        exit(1);
                }

                close(fd3);///////////////////////////////nidhiiiiiiiiiiiiiii
	}

	else
	{
		printf("Wrong pin");
	}

}*/

void bcm2835_gpio_set(int pin)
{
//	int x = atoi(pin);

      //  if (pin == 34)
     //   {

if (pin == 46)
{

                int fd4 = open("/sys/class/gpio/gpio46/value",O_WRONLY);
                if (fd4 == -1)
                {
                        perror("Unable to open /sys/class/gpio/gpio46/value");
                        //syslog_function(error_inhale_valve_open);
                        exit(1);
                }

                if (write(fd4, "1", 1) != 1)
                {
                        perror("Error writing to /sys/class/gpio/gpio46/value");
                        //syslog_function(error_inhale_valve_write);
                        exit(1);
                }
                close(fd4);
}
	//}

else if (pin == 48)
{

	int fd4 = open("/sys/class/gpio/gpio48/value",O_WRONLY);
                if (fd4 == -1)
                {
                        perror("Unable to open /sys/class/gpio/gpio48/value");
                        //syslog_function(error_inhale_valve_open);
                        exit(1);
                }

                if (write(fd4, "1", 1) != 1)
                {
                        perror("Error writing to /sys/class/gpio/gpio48/value");
                        //syslog_function(error_inhale_valve_write);
                        exit(1);
                }
                close(fd4);

		//printf("breaking on \n");
}


else if (pin == 115)
{

        int fd4 = open("/sys/class/gpio/gpio115/value",O_WRONLY);
                if (fd4 == -1)
                {
                        perror("Unable to open /sys/class/gpio/gpio115/value");
                        //syslog_function(error_inhale_valve_open);
                        exit(1);
                }

                if (write(fd4, "1", 1) != 1)
                {
                        perror("Error writing to /sys/class/gpio/gpio115/value");
                        //syslog_function(error_inhale_valve_write);
                        exit(1);
                }
                close(fd4);

                //printf("breaking on \n");
}




else if (pin == 27)
        {

        int fd6 = open("/sys/class/gpio/gpio27/value",O_WRONLY);
                if (fd6 == -1)
                {
                        perror("Unable to open /sys/class/gpio/gpio27/value");
                        //syslog_function(error_inhale_valve_open);
                        exit(1);
                }

                if (write(fd6, "1", 1) != 1)
                {
                        perror("Error writing to /sys/class/gpio/gpio27/value");
                        //syslog_function(error_inhale_valve_write);
                        exit(1);
                }
                close(fd6);

                //printf("Breaking off \n");

        }

        else
        {
        printf("Wrong pin on \n");
        }


}


void bcm2835_gpio_clr(int pin)

{
//int x = atoi(int pin);

        if (pin == 46)
	{
		int fd6 = open("/sys/class/gpio/gpio46/value",O_WRONLY);
        	if (fd6 == -1)
    		{
        		perror("Unable to open /sys/class/gpio/gpio46/value");
        		//syslog_function(error_inhale_valve_open);
        		exit(1);
    		}

        	if (write(fd6, "0", 1) != 1)
    		{
        		perror("Error writing to /sys/class/gpio/gpio46/value");
        		//syslog_function(error_inhale_valve_write);
        		exit(1);
    		}
    		close(fd6);
	}


	else if (pin == 48)
	{

	int fd6 = open("/sys/class/gpio/gpio48/value",O_WRONLY);
                if (fd6 == -1)
                {
                        perror("Unable to open /sys/class/gpio/gpio48/value");
                        //syslog_function(error_inhale_valve_open);
                        exit(1);
                }

                if (write(fd6, "0", 1) != 1)
                {
                        perror("Error writing to /sys/class/gpio/gpio48/value");
                        //syslog_function(error_inhale_valve_write);
                        exit(1);
                }
                close(fd6);

		//printf("Breaking off \n");

	}


	else if (pin == 115)
        {

        int fd6 = open("/sys/class/gpio/gpio115/value",O_WRONLY);
                if (fd6 == -1)
                {
                        perror("Unable to open /sys/class/gpio/gpio115/value");
                        //syslog_function(error_inhale_valve_open);
                        exit(1);
                }

                if (write(fd6, "0", 1) != 1)
                {
                        perror("Error writing to /sys/class/gpio/gpio115/value");
                        //syslog_function(error_inhale_valve_write);
                        exit(1);
                }
                close(fd6);

                //printf("Breaking off \n");

        }



	else if (pin == 27)
        {

        int fd6 = open("/sys/class/gpio/gpio27/value",O_WRONLY);
                if (fd6 == -1)
                {
                        perror("Unable to open /sys/class/gpio/gpio27/value");
                        //syslog_function(error_inhale_valve_open);
                        exit(1);
                }

                if (write(fd6, "0", 1) != 1)
                {
                        perror("Error writing to /sys/class/gpio/gpio27/value");
                        //syslog_function(error_inhale_valve_write);
                        exit(1);
                }
                close(fd6);

                //printf("Breaking off \n");

        }





else
	{
	printf("Wrong pin off \n");
	}

}

/*
int main()
{

bcm2835_gpio_fsel(115,"out");
bcm2835_gpio_fsel(27,"out");

while(1)
	{
	bcm2835_gpio_set(115);
	bcm2835_gpio_set(27);
        sleep(1);
        bcm2835_gpio_clr(115);
	bcm2835_gpio_clr(27);
	sleep(1);
	}


}
*/


