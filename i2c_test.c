#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <syslog.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/time.h>
#define IO_ERROR_AMS -998
#define FILE_OPEN_ERROR_AMS -999
int read_pressure(int);
int start_bus();
int file;
char *bus = "/dev/i2c-2";


int start_bus()
{
	if((file = open(bus, O_RDWR)) < 0)
	{
		printf("Failed to open the bus. \n");
		return FILE_OPEN_ERROR_AMS;
	}
	return 0;
}


int file_clear()
{
        if(fclose(fopen("connection_data.txt", "w")))       { printf("file clear error\n"); return 0; }
        return 1;
}



int file_write(int val)
{
   FILE *fp;
   fp = fopen("connection_data.txt","a");
   if(fp == NULL)       { printf("Error in openning file"); return 0; }

   fprintf(fp,"%d,",val);
   fclose(fp);
  // printf("fclose value : %d\n",test); 

   return 1;
}
/*
int read_add(int index)
{
char buff[55],i2c_add[2];
int  i=0;
int add = 0;
				for(int j=0;j<1;j++)
				{
                        		while(i<2)
                                		{
                                        		i2c_add[i] = buff[index +(i-1)];
                                        		i++;
                                		}
                        		i2c_add[i] = '\0';
                        		if(add = atoi(i2c_add))
						{
							printf("pressure sensor connected %d\n",add);
							if(file_write(1))   printf("file data written\n");
                                			else    printf("error writting data\n");
						}
                        		else
						{
							printf("pressure sensor not connected %d\n",add);
							if(file_write(0))   printf("file data written\n");
                                			else    printf("error writting data\n");
						}
                        		i=0;
				}
}

*/
int pressure_connection_check()
{
        system("sudo >i2c.txt i2cdetect -y -r -a 2 >> i2c.txt");
        FILE *fp;
        char buff[55],i2c_add[2];
	int index = 29;
	//int index[6] = {5,17,20,23,26,29};
        int i=0;
        int add = 0;
        fp = fopen("i2c.txt", "r");
        if(fp == NULL)  {       printf("file cannot be opened\n");      return 0;}
        else    {
                        fgets(buff, 55,fp);
                        fgets(buff, 55,fp);
                        fgets(buff, 55,fp);
			fgets(buff, 55,fp);
			for(int j=0;j<1;j++)
                                {
                                        while(i<2)
                                                {
                                                        i2c_add[i] = buff[index +(i-1)];
                                                        i++;
                                                }
                                        i2c_add[i] = '\0';
                                        if(add = atoi(i2c_add))
                                                {
                                                        printf("Flow sensor connected %d\n",add);
                                                        if(file_write(1))   printf("file data written\n");
                                                        else    printf("error writting data\n");
                                                }
                                        else
                                                {
                                                        printf("Flow sensor not connected %d\n",add);
                                                        if(file_write(0))   printf("file data written\n");
                                                        else    printf("error writting data\n");
                                                }
                                        i=0;
                                }

			//read_add(index);
			
			fgets(buff, 55,fp);
                        fgets(buff, 55,fp);
			for(int j=0;j<1;j++)
                                {
                                        while(i<2)
                                                {
                                                        i2c_add[i] = buff[index +(i-1)];
                                                        i++;
                                                }
                                        i2c_add[i] = '\0';
                                        if(add = atoi(i2c_add))
                                                {
                                                        printf("ADC connected %d\n",add);
                                                        if(file_write(1))   printf("file data written\n");
                                                        else    printf("error writting data\n");
                                                }
                                        else
                                                {
                                                        printf("ADC not connected %d\n",add);
                                                        if(file_write(0))   printf("file data written\n");
                                                        else    printf("error writting data\n");
                                                }
                                        i=0;
                                }
			
			//read_add(index);
			
			fgets(buff, 55,fp);
                        fgets(buff, 55,fp);
                        fgets(buff, 55,fp);
			for(int j=0;j<1;j++)
                                {
                                        while(i<2)
                                                {
                                                        i2c_add[i] = buff[index +(i-1)];
                                                        i++;
                                                }
                                        i2c_add[i] = '\0';
                                        if(add = atoi(i2c_add))
                                                {
                                                        printf("pressure sensor connected %d\n",add);
                                                        if(file_write(1))   printf("file data written\n");
                                                        else    printf("error writting data\n");
                                                }
                                        else
                                                {
                                                        printf("pressure sensor not connected %d\n",add);
                                                        if(file_write(0))   printf("file data written\n");
                                                        else    printf("error writting data\n");
                                                }
                                        i=0;
                                }
			
			//read_add(index);
                        
			fclose(fp);
                        return 1;
                }
}

int main()
{
file_clear();
pressure_connection_check();

}
