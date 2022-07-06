#include <termios.h>
#include <stdio.h>
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

	int fd,  res=0, check=0;
	struct termios oldtio, newtio;
	struct sigaction saio;

	fd = open(PORT, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd<0)
	{
		perror(PORT);
		exit(-1);
	}

	fcntl(fd, F_SETFL, FASYNC);
	saio.sa_handler=signal_handler_IO;
	saio.sa_flags=0;
	saio.sa_restorer = NULL;
	sigaction(SIGIO, &saio,NULL);

	check = tcgetattr(fd, &oldtio);
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

	check = tcflush(fd, TCIFLUSH);
	if (check<0)
	{
		perror("tcsetattr");
	}

	check = tcsetattr(fd, TCSANOW, &newtio);
	if (check<0)
	{
		perror("tcsetattr");
	}

	int bytes;
        unsigned char buff[50];
	char command[] = "HS";
	char *msg = "ACK51";
	char *calib = "CALIB03030";
	char *modechnge = "ACK02";
	char *mode = "11";
	char data_string[]={0};
	float flow = 50.5, pressure = 52.5;
	int len = 0;
	memset( buff, 0, sizeof(buff) );

	while(1)
	{

		// read data
		res = read(fd, buff,50);
		printf("%s\n",buff);
		printf(" Len == %d \n ", strlen(buff));
		if(strcmp(buff,"HS") == 0)

			{
			printf(" Hs Rxd \n");
			
			write(fd, msg,6);
	//		close(fd);
			delay(2000);
			write(fd, calib, 11);
			printf("calib done\n");
			//write(fd,modechnge,16);
			//printf("mode changed\n");
			}

		if( strcmp(buff,"11") == 0 || strcmp(buff,"11") == 0 || strcmp(buff,"12") == 0 || strcmp(buff,"120") == 0 || strcmp(buff,"130") == 0
		 || strcmp(buff,"13") == 0 || strcmp(buff,"14") == 0 || strcmp(buff,"14") == 0 || strcmp(buff,"210") == 0 || strcmp(buff,"21") == 0
		 || strcmp(buff,"22") == 0 || strcmp(buff,"221") == 0 || strcmp(buff,"23") == 0 || strcmp(buff,"231") == 0 || strcmp(buff,"24") == 0
		 || strcmp(buff,"241") == 0 || strcmp(buff,"25") == 0 || strcmp(buff,"251") == 0 || strcmp(buff,"310") == 0 || strcmp(buff,"311") == 0
		 || strcmp(buff,"31") == 0 || strcmp(buff,"32") == 0 || strcmp(buff,"33") == 0 || strcmp(buff,"331") == 0 || strcmp(buff,"340") == 0
		 || strcmp(buff,"341") == 0 || strcmp(buff,"350") == 0 || strcmp(buff,"351") == 0 || strcmp(buff,"260") == 0 || strcmp(buff,"261") == 0)
			{
			printf("got 11");
			write(fd,modechnge,5);
			usleep(300000);
			write(fd,"ACK04" ,6);

			}

		if(strcmp(buff,"PING") == 0)
		{
		printf("Ping found \n");
		write(fd, "RQDN0", 5);
		}

		//sprintf(data_string, "A@%.2f,%.2f,%.2f,%.2f#", flow, pressure, 0, 0);
		printf("%s\n",data_string);
		char *hello = data_string;
		printf("%d \n", strlen(hello));
		write(fd, hello, strlen(hello));
		memset(data_string ,0 , strlen(data_string));
		printf("writing data %s  \n", data_string);
		memset( buff, 0, strlen(buff) );

		printf("\n");

		usleep(100000);
	}
	close (fd);
}

