#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
//#include <wiringPi.h>
//#include <wiringSerial.h>

FILE *fd;
bool standby_flag = true;
int compare(char *recieved_string, char *compare_string)
{
	if(strcmp(&recieved_string, &compare_string))
	{
		printf("correct\n");
		return 0;
	}
	else
	{
		printf("Wrong\n");
		return 1;
	}

}

int main()
{


  	FILE *serial_port ;
  	char data[256];
	char *str = "90";
	char *values;
	/*if(fd = fopen("mode.txt","w")
	{
        	(fd == NULL)
        	{
              		printf("error found in mode file\n");
              		exit(1);
        	}
		fclose(fd);
	}*/

	if ((serial_port = serialOpen ("/dev/ttyS0", 9600)) < 0)
  	{
    		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno));
    		return 1 ;
  	}
	if (wiringPiSetup () == -1)
  	{
  		fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    		return 1 ;
  	}

	int i = 0;
	char buff = "";
  	while(1)
	{
		i = 0;
		data[0];

		while(serialDataAvail(serial_port))
		{
			buff = (serialGetchar(serial_port));
			data[i] =  (char *)buff;
                        i++;

			if(i>250)
			{
				break;
			}

			int j = i;
			if(j>0)
			{
				j--;
			}
		}
		printf("%s", data);
		printf("\n");
		if(!strcmp(&data, "HS"))
		{
			write(serial_port,"ACK51",6);
			usleep(50000);
			write(serial_port,"CALIB03030",11);
			write(serial_port,"ACK02",6);
		}

		if((!strcmp(&data, "11")) || (!strcmp(&data, "12")) || (!strcmp(&data, "13")) || (!strcmp(&data, "17")) || (!strcmp(&data, "18"))
                || (!strcmp(&data, "21")) || (!strcmp(&data, "22")) || (!strcmp(&data, "23")) || (!strcmp(&data, "24")) || (!strcmp(&data, "25"))
                || (!strcmp(&data, "26")) || (!strcmp(&data, "31")) || (!strcmp(&data, "32")) || (!strcmp(&data, "33")) || (!strcmp(&data, "34"))
                || (!strcmp(&data, "35")))
		{
			//printf("inside\n");
			write(serial_port, "STND00",7);
			fd = fopen("mode_new.txt", "w");
			if(fd == NULL)
			{
				printf("error_File!");
				exit(1);
			}
			fprintf(fd,"%s\n", data);
			fclose(fd);
			write(serial_port,"ACK02",6);
		}

		int x = strlen(data);
		if(x > 4)
		{
			//printf("hjjkhjkj\n");
			values = strtok (data,",");
			//printf("hhehh\n");
			//while(values != NULL)
			//{
			//	printf("%s", values);
				//values = strtok (NULL, ",");
				//printf("err\n");
				//printf("data: %s",pt[i]);
				//i++;
			//}
			//printf("hhehe\n");
			if((values[0] == 'S') || (values[14], '#'))
			{
				//printf("second\n");
				fd = fopen("setting.txt", "w") ;
				if(fd == NULL)
				{
					printf("error_file1!\n");
					exit(1);
				}
				if(values != NULL)
				{
					fprintf(fd,"%s",values);
				}
				fclose(fd);
				write(serial_port,"ACK04",6);
			}
		}

		if(!strcmp(&data, "CM+STANDBY"))
		{
			//char *str = "90";
			standby_flag = true;
			printf("dhfhj\n");
			fd = fopen("mode_new.txt", "w");
			if(fd == NULL)
                        {
                        	printf("error_file1!\n");
                                exit(1);
                        }
			fprintf(fd,"%s",str);
			fclose(fd);
			write(serial_port,"STND01",7);
		}

		if(!strcmp(&data, "CM+SC4"))
		{
			standby_flag = true;
			printf("standby_flag\n");
			system("sudo python turbine.py");
		}

		if(!strcmp(&data, "CM+SC2"))
                {
			standby_flag = true;
                        //printf("standby_flag\n");
                        system("sudo ./rishub_callib.o 1");
                }

		if(!strcmp(&data, "CM+SC1"))
                {
			standby_flag = true;
                        //printf("standby_flag\n");
                        system("sudo ./rishub_callib.o 2");
                }

		if(!strcmp(&data, "CM+SC3"))
                {
			standby_flag = true;
                        //printf("standby_flag\n");
                        system("sudo python turbine.py");
                }

		if(!strcmp(&data, "CM+SA"))
                {
			system("sudo ./rishub_self_test.o");
                       	fd = fopen("connection_data.txt","r");
			if(fd == NULL)
			{
				printf("error found in connection\n");
				exit(1);
			}
			printf("readline\n");
			fgets(data,50,fd);
			fclose(fd);
			printf("%s\n", data);
			//write(serial_port, "SA" + data[2, 9] + ",0,0,0" + "!");

                }

		if(!strcmp(&data, "CM+BREATH"))
                {
                        char *fb = "1";
                        fd = fopen("mannual_breath.txt","w");
                        if(fd == NULL)
                        {
                                printf("error found in connection\n");
                                exit(1);
                        }
                        fprintf(fd ,"%s", fb);
                        fclose(fd);
                }

                if(!strcmp(&data, "CMR+STP"))
                {
                        write(serial_port,"STP007",7);
                        system("sudo python self_test.py");
                }

		if(!strcmp(&data, "IH") || (&data, "EH"))
                {
                        if(!strcmp(&data, "EH"))
                        {
                                write(serial_port,"ACK67",6);
                        }
                        else if(!strcmp(&data, "IH"))
                        {
                                write(serial_port,"ACK66",6);
                        }
                        else
                        {
                                ///sdjkcsdkh
                        }
                        fd = fopen("hold.txt","w+");
                        if(fd == NULL)
                        {
                                printf("error found hold file\n");
                                exit(1);
                        }
                        fclose(fd);
                }

                if(!strcmp(&data, "CMR+BOOT"))
                {
                        system("sudo python fastboot.py");
                }


	memset(&data, '\0', sizeof(data));
	usleep(200000);
	}
}

