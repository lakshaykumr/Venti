#ifndef timer_function
#define timer_function

float delta_t(struct timespec *start, struct timespec *stop, struct timespec *delta);

/*struct timespec start_time = {0,0};
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

#endif

