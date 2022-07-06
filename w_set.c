#include<unistd.h>
#include<stdio.h>
#include<string.h>
#include<stdlib.h>
FILE *fd;

void main()
{
char *buff = "S,60,500,250,16,5,17,1.7,150,21,6,10,1.9,25,#";
int sz =0;
sz= strlen(buff);
int a = sz;
char values[sz-3];
printf("buff = %s \n size = %d \n",buff,sz);
int k=2;

while(buff[k]!= '#')
	{
        values[k-2] = buff[k];
	k++;
        }
values[k-2]='\0';
int j = strlen(values);
printf("values buff length = %d \n",j);
printf("value string = %s \n", values);
printf("updating setting \n");
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
}
