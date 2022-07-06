#include <termios.h>
#include <stdio.h>
#include<stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <time.h>
#include <string.h>
#define BAUDRATE B9600
#define PORT "/dev/ttyO1"
//unsigned char count =0;
int i,d=0, wait_flag = 1;
char command[20];
bool standby_flag = true;
FILE *fd;
void signal_handler_IO()
{
	wait_flag=0;
}

void delay(unsigned int numOfMiliSec)
{
        unsigned long numOfMicroSec = 1000 * numOfMiliSec;
        clock_t startTime = clock();
        clock_t Ttime = startTime + numOfMicroSec;
        while(clock() < Ttime);
}

int main ()
{
	system("config-pin P9_24 uart >/dev/null 2>&1");
	system("config-pin P9_26 uart >/dev/null 2>&1");
	int serial_port,  res=0, check=0;
	struct termios oldtio, newtio;
	struct sigaction saio;

	

	int bytes;
        char buff[80];
	char command[] = "HS";
	char *msg = "ACK51";
	char *calib = "CALIB03030";
	char *values;
	char *modechnge = "ACK02";
	char *mode = "11";
	char *str = "90";
	char data_string[]={0};
	float flow = 50.5, pressure = 52.5;
	int len = 0;
	memset( buff, 0, sizeof(buff) );

	while(1)
	{

	serial_port = open(PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (serial_port<0)
	{
		perror(PORT);
		exit(-1);
	}

	fcntl(serial_port, F_SETFL, FASYNC);
	saio.sa_handler=signal_handler_IO;
	saio.sa_flags=0;
	saio.sa_restorer = NULL;
	sigaction(SIGIO, &saio,NULL);

	check = tcgetattr(serial_port, &oldtio);
	if (check<0)
	{
		perror("tcgetattr");
	}

	check = newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD ;
	if (check<0)
	{
		perror("c_cflag");
	}

	check = newtio.c_iflag = IGNPAR | IXON;
	if (check<0)
	{
		perror("c_iflag");
	}

	check = newtio.c_oflag = 0;
	if (check<0)
	{
		perror("c_oflag");
	}

	check = newtio.c_lflag = 0;
	if (check<0)
	{
		perror("c_lflag  ");
	}

	check = newtio.c_cc[VMIN]=1;
	if (check<0)
	{
		perror("c_cc[VMIN]");
	}

	check = newtio.c_cc[VTIME]=0;
	if (check<0)
	{
		perror("c_cc[VTIME]");
	}

	check = tcflush(serial_port, TCIFLUSH);
	if (check<0)
	{
		perror("tcsetattr");
	}

	check = tcsetattr(serial_port, TCSANOW, &newtio);
	if (check<0)
	{
		perror("tcsetattr");
	}	


		// read data
		res = read(serial_port, &buff, sizeof(buff));
		buff[res] = '\0';
	/*	if (res == -1)
    {
        printf("Error reading from serial port\n");
        break;
    }
    else if (res == 0)
    {
        printf("No more data\n");
        break;
    }
    else
    {
        buff[res] = '\0';
        printf("%s", buff);
    } */
		//usleep(50000);

		printf("%s\n",buff);
		printf(" Len == %d \n ", strlen(buff));
		if(strcmp(buff,"HS") == 0)

			{
			printf(" Hs Rxd \n");
			write(serial_port,"ACK51",6);
			usleep(50000);
			write(serial_port,"CALIB03030",11);
			write(serial_port,"ACK02",6);
			}

		if((!strcmp(buff, "11")) || (!strcmp(buff, "12")) || (!strcmp(buff, "13")) || (!strcmp(buff, "17")) || (!strcmp(buff, "18"))
                || (!strcmp(buff, "21")) || (!strcmp(buff, "22")) || (!strcmp(buff, "23")) || (!strcmp(buff, "24")) || (!strcmp(buff, "25"))
                || (!strcmp(buff, "26")) || (!strcmp(buff, "31")) || (!strcmp(buff, "32")) || (!strcmp(buff, "33")) || (!strcmp(buff, "34"))
                || (!strcmp(buff, "35")))
		{
			//printf("inside\n");
			write(serial_port, "STND00",7);
			fd = fopen("mode.txt", "w");
			if(fd == NULL)
			{
				printf("error_File!");
				exit(1);
			}
			fprintf(fd,"%s\n", buff);
			fclose(fd);
			write(serial_port,"ACK02",6);
		}
		if(strcmp(buff,"PING") == 0)
		{
		printf("Ping found \n");
		write(serial_port, "RQDN0", 5);
		}

		int x = strlen(buff);
		if(x > 4)
		{
			//printf("hjjkhjkj\n");
			//values = strtok (buff,",");
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
			if((buff[0] == 'S') && (buff[x-1], '#'))
			{
					int sz =0;
					sz= strlen(buff);
			int a = sz;
			char values[sz-3];
			//printf("buff = %s \n size = %d \n",buff,sz);
			int k=2;

			while(buff[k]!= '#')
        		{
        		values[k-2] = buff[k];
        		k++;
        		}
			values[k-2]='\0';
			int j = strlen(values);
			printf("value string = %s \n", values);
			fd = fopen("setting.txt", "w+") ;
			if(fd == NULL)
        			{
        			printf("error_file1!\n");
        			exit(1);
        			}

			if(values != NULL)
        			{
        			fprintf(fd,"%s",values);
        			}
				 memset(values, 0, strlen(values));
        			fclose(fd);
				write(serial_port,"ACK04",6);
			}
		}

		if(!strcmp(buff, "CM+STANDBY"))
		{
			//char *str = "90";
			standby_flag = true;
			printf("dhfhj\n");
			fd = fopen("mode.txt", "w");
			if(fd == NULL)
                        {
                        	printf("error_file1!\n");
                                exit(1);
                        }
			fprintf(fd,"%s",str);
			fclose(fd);
			write(serial_port,"STND01",7);
		}

		if(!strcmp(buff, "CM+SC4"))
		{
			standby_flag = true;
			printf("standby_flag\n");
			printf("Turbine in progress \n");
			//system("Turbine in progress \n");
		}

		if(!strcmp(buff, "CM+SC2"))
                {
			standby_flag = true;
                        //printf("standby_flag\n");
                        system("sudo ./calibrate 2");
                }

		if(!strcmp(buff, "CM+SC1"))
                {
			standby_flag = true;
                        //printf("standby_flag\n");
                        system("sudo ./calibrate 1");
                }

 		if(!strcmp(buff, "CM+SC3"))
                {
			standby_flag = true;
                        //printf("standby_flag\n");
                        system("sudo ./calibrate 3");
                }

		if(!strcmp(buff, "CM+SA"))
                {
			system("sudo ./i2c_test");
                       	fd = fopen("connection_data.txt","r");
			if(fd == NULL)
			{
				printf("error found in connection\n");
				exit(1);
			}
			printf("readline\n");
			fgets(buff,50,fd);
			fclose(fd);
			printf("%s\n", buff);
			//write(serial_port, "SA" + data[2, 9] + ",0,0,0" + "!");

                }

		if(!strcmp(buff, "CM+BREATH"))
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

                if(!strcmp(buff, "CMR+STP"))
                {
                        write(serial_port,"STP007",7);
                        system("sudo python self_test.py");
                }

		if(!strcmp(buff, "IH") || (buff, "EH"))
                {
                        if(!strcmp(buff, "EH"))
                        {
                                write(serial_port,"ACK67",6);
                        }
                        else if(!strcmp(buff, "IH"))
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

                if(!strcmp(buff, "CMR+BOOT"))
                {
                        system("sudo python fastboot.py");
                }



	memset(buff, 0, strlen(buff));
	usleep(30000);
	close (serial_port);
	}
	
}

