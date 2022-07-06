#include "sir_valve_turbine.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#define I2C_IO_ERROR -100
#define IO_ERROR_AMS -998
#define FILE_OPEN_ERROR_AMS -999
#define NSEC_PER_SEC 1000000000
float Pmax = 1.5;
float pressure_raw,pressuref,prescmh20;
float Pmin = 0;
float Digout_max = 29491.0;
float Digout_min = 3277.0;
int read_pressure(int);
int start_bus();
int file;
char *bus = "/dev/i2c-2";
int pressure_1 = 0;
float Pressure, Flow;
char  data2[4] = {0};



void gaussEliminationLS(int m, int n, double a[m][n], double x[n-1]){
    int i,j,k;
    for(i=0;i<m-1;i++){
        //Partial Pivoting
        for(k=i+1;k<m;k++){
            //If diagonal element(absolute vallue) is smaller than any of the terms below it
            if(fabs(a[i][i])<fabs(a[k][i])){
                //Swap the rows
                for(j=0;j<n;j++){
                    double temp;
                    temp=a[i][j];
                    a[i][j]=a[k][j];
                    a[k][j]=temp;
                }
            }
        }
        //Begin Gauss Elimination
        for(k=i+1;k<m;k++){
            double  term=a[k][i]/ a[i][i];
            for(j=0;j<n;j++){
                a[k][j]=a[k][j]-term*a[i][j];
            }
        }

    }
    //Begin Back-substitution
    for(i=m-1;i>=0;i--){
        x[i]=a[i][n-1];
        for(j=i+1;j<n-1;j++){
            x[i]=x[i]-a[i][j]*x[j];
        }
        x[i]=x[i]/a[i][i];
    }

}
/*******
Function that prints the elements of a matrix row-wise
Parameters: rows(m),columns(n),matrix[m][n]
*******/
void printMatrix(int m, int n, double matrix[m][n]){
    int i,j;
    for(i=0;i<m;i++){
        for(j=0;j<n;j++){
            printf("%lf\t",matrix[i][j]);
        }
        printf("\n");
    }
}


float delta_t(struct timespec *start, struct timespec *stop, struct timespec *delta)
{
int dt_sec = stop->tv_sec - start->tv_sec;
int dt_nsec = stop->tv_nsec - start->tv_nsec;
if(dt_sec == 0){
	//printf("dt less than sec\n");
	if(dt_nsec > 0 && dt_nsec < NSEC_PER_SEC){
		delta->tv_sec = dt_sec;
		delta->tv_nsec = dt_nsec;
	}
	else if(dt_nsec >= NSEC_PER_SEC){
		delta->tv_sec = 1;
                delta->tv_nsec = dt_nsec - NSEC_PER_SEC;
	}
	else{
		delta->tv_sec = dt_sec;
                delta->tv_nsec = dt_nsec;
	}
	}
else if (dt_sec >= 1){
	//printf("dt grater than 1 sec\n");
        if(dt_nsec  > 0 && dt_nsec < NSEC_PER_SEC){
//		printf("got in first loop\n");
                delta->tv_sec = dt_sec;
                delta->tv_nsec = dt_nsec;
        }
        else if(dt_nsec >= NSEC_PER_SEC){
//		printf("Got in second looop\n");
                delta->tv_sec = dt_sec + 1;
                delta->tv_nsec = dt_nsec - NSEC_PER_SEC;
        }
        else{
//		printf("got in third loop\n");
                delta->tv_sec = dt_sec - 1;
                delta->tv_nsec = dt_nsec + NSEC_PER_SEC;
        }
}
return ((delta->tv_sec*1000) +(delta->tv_nsec/1000000));
}



int start_bus()
{
	if((file = open(bus, O_RDWR)) < 0)
	{
		printf("Failed to open the bus. \n");
	//	exit(1);
		return FILE_OPEN_ERROR_AMS;
	}
	return 0;
}

float pressure(int address2)
{
ioctl(file, I2C_SLAVE, address2);
	if(read(file, data2, 4) != 4)
        {
                printf("Error : Input/Output error \n");
		//syslog_function(error_PRESSURE_I2C_IO);
                return I2C_IO_ERROR;

        }

 		int pressure_1 = (data2[0] * 256 + data2[1]);
                float pressure2 = ((((pressure_1 - 3277.0)/((26214.0) / 3.0))-1.5)*70.30);
        //      printf("circuit pressure = %d\n",pressure2);
                return (pressure2);
//	pressure_raw = (data2[0] * 256 + data2[1]);
//	pressuref = ((pressure_raw - 3277.0) / ((26214.0) / 1.5)) + 0;
//	prescmh20 = pressuref * 70.307;
//	return prescmh20 ;
}




int main()

{

struct timespec start_time = {0,0};
struct timespec end_time = {0,0};
struct timespec deltat = {0,0};

pwm_init();
start_bus();
    //no. of data-points
int N = 10;
//int N = 100;
    //degree of polynomial
int n = 3; 
float sum=0, counter = 0;
   /* printf("Enter the no. of data-points:\n");
    scanf("%d",&N); */
    //arrays to store the c and y-axis data-points
double x[N], y[N],timer;
double  avrg =0.0;
    //printf("Enter the x-axis values:\n");
int i,j;

/*
    for(i=0;i<N;i++){
        scanf("%lf",&x[i]);
    }
    //printf("Enter the y-axis values:\n");
    for(i=0;i<N;i++){
        scanf("%lf",&y[i]);
    }

    printf("Enter the degree of polynomial to be used:\n");
    scanf("%d",&n); */

//for(i=0; i<N; i++)
for(i=0; i<N; i++)
	{
	//y[i] = i+1;
	y[i] = (i+1)*10;
	printf("PWM ===========> %d", (i+1) * 10);
	//turbine_duty_cycle(i + 1);
	turbine_duty_cycle((i + 1)*10);
	sleep(2);
	timer = 0;
	clock_gettime(CLOCK_REALTIME, &start_time);
	while(timer < 500)
		{
			float pressure_t = pressure(0x17);
			printf("Pressure = %f \n", pressure_t);
			sum = sum + pressure_t;
			counter++;
			//printf("timer = %lf",timer);
			clock_gettime(CLOCK_REALTIME, &end_time);
        		timer = delta_t(&start_time, &end_time,&deltat);

		}
	//printf("sum = %f counter = %f \n",sum,counter);
	//printf("PWM ===========> %d \n", i + 1);
	//printf("PWM ===========> %d \n", (i + 1)*10);

	avrg = sum/counter;
	printf("Pressure ===========> %f \n", avrg);

	x[i] = avrg;
	sum = 0;
	counter = 0;
	avrg = 0;
	 timer = 0;
        clock_gettime(CLOCK_REALTIME, &start_time);
        while(timer < 6000)
                {
		 turbine_duty_cycle(0);
		//printf("Time left for second reading = %ld \n", (60 - (timer/1000)));
		if (timer > 50000)
			{
			printf("PUT BACK THE STOPPER \n");
			}
		clock_gettime(CLOCK_REALTIME, &end_time);
                timer = delta_t(&start_time, &end_time,&deltat);

		}
	turbine_duty_cycle(0);
	//sleep(40);
	printf("Put back the stopper \n");
	//sleep(20);


	}
turbine_duty_cycle(0);


for(i =0;i<N;i++)
{
printf("data = (%lf,%lf)" , x[i],y[i]);
}

// an array of size 2*n+1 for storing N, Sig xi, Sig xi^2, ...., etc. which are the independent components of the normal matrix
    double X[2*n+1];  
    for(i=0;i<=2*n;i++){
        X[i]=0;
        for(j=0;j<N;j++){
            X[i]=X[i] + pow(x[j],i);
        }
    }
    //the normal augmented matrix
    double B[n+1][n+2];  
    // rhs
    double Y[n+1];      
    for(i=0;i<=n;i++){
        Y[i]=0;
        for(j=0;j<N;j++){
            Y[i]=Y[i] + pow(x[j],i)*y[j];
        }
    }
    for(i=0;i<=n;i++){
        for(j=0;j<=n;j++){
            B[i][j]=X[i+j]; 
        }
    }
    for(i=0;i<=n;i++){
        B[i][n+1]=Y[i];
    }
    double A[n+1];
    printf("The polynomial fit is given by the equation:\n");
    printMatrix(n+1,n+2,B);
    gaussEliminationLS(n+1,n+2,B,A);
    for(i=0;i<=n;i++){
        printf("%lfx^%d+",A[i],i);
	} 
FILE *fpt;

fpt = fopen("tf_constant.txt", "w+");
if (!fpt){
        printf("Can't open txt file\n");
//      syslog_function(error_OPEN_CALIB);
        }
else
        {
        for(i=0; i<n; i++)
        {
        //x = atof(array())
        fprintf(fpt,"%0.15lf,",A[i]);
        }
        fclose(fpt);
	}

}
