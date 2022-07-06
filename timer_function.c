#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#define NSEC_PER_SEC 1000000000

#define ITERATIONS 129499

extern struct timespec start_time;
extern struct timespec end_time;
extern struct timespec deltat;


float delta_t(struct timespec *start, struct timespec *stop, struct timespec *delta){

int dt_sec = stop->tv_sec - start->tv_sec;
int dt_nsec = stop->tv_nsec - start->tv_nsec;
//printf("got inside with %d\n", dt_nsec);
if(dt_sec == 0){
//	printf("dt less than sec\n");
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
//	printf("dt grater than 1 sec\n");
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
//	printf("Time in sec : %ld\n", delta->tv_sec);
//	printf("Time in nsec : %ld\n", delta->tv_nsec);

	return (delta->tv_sec*1000.00 + delta->tv_nsec/1000000.00);
}
/*
int main(void){
struct timespec start_time = {0,0};
struct timespec end_time = {0,0};
struct timespec deltat = {0,0};
clock_gettime(CLOCK_REALTIME, &start_time);
//for(int i = 0; i < ITERATIONS; i++){
//;
}
clock_gettime(CLOCK_REALTIME, &end_time);
delta_t(&start_time, &end_time,&deltat);
return 0;
}
*/
