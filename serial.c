#include "serial.h"
#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <time.h>
#include<string.h>
#define BAUDRATE B9600
#define PORT "/dev/ttyO1"

//unsigned char count =0;
int i, wait_flag = 1;
void signal_handler_IO()
{
	wait_flag=0;
}

int serial_data_read()
{
	unsigned char buff[50];

	//system("config-pin P9_24 uart >/dev/null 2>&1");
	//system("config-pin P9_26 uart >/dev/null 2>&1");

	int fd,  res=0, check=0;
	struct termios oldtio, newtio;
	struct sigaction saio;

	fd = open(PORT, O_RDONLY | O_NOCTTY | O_NONBLOCK);
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



memset( buff, 0, strlen(buff) );
	res = read(fd, buff,50);
	printf("%s\n",buff);
memset( buff, 0, strlen(buff) );
close(fd);
return res;
}


int serial_data_write(char* data)
{
int fd,  res=0, check=0;
	struct termios oldtio, newtio;
	struct sigaction saio;

	fd = open(PORT, O_WRONLY | O_NOCTTY | O_NONBLOCK);
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
/*
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
*/

	write(fd, data, strlen(data));

close(fd);

return 0;
}




