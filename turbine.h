#ifndef turbine
#define turbine
/*
int order_10_pow = -12;
int order_9_pow  = -10;
int order_8_pow  = -8;
int order_7_pow  = -6;
int order_6_pow  = -4;
int order_5_pow  = -3;
int order_4_pow  = -1;
int order_3_pow  =  0;
int order_2_pow  =  1;
int order_1_pow  =  1;
int order_0_pow  =  1;*/
///////////////////////Constant calculation ///////////////////
float order_10=   -1.71*0.000000000001;//*pow(10,order_10_pow)
float order_9 =     5.46207*0.0000000001;//pow( 10 , order_9_pow)
float order_8 =   -7.43836*0.00000001; //pow( 10 , order_8_pow)
float order_7 =     5.67195*0.000001;//pow( 10 , order_7_pow)
float order_6 =    -2.65547*0.0001;//pow( 10 , order_6_pow)
float order_5 =     7.87639*0.001;//pow( 10 , order_5_pow)
float order_4 =    -1.47579*0.1;//pow( 10 , order_4_pow)
double order_3 =    1.69683*1 ;//pow( 10 , order_3_pow)
double order_2 =    -1.14982*10 ;//pow( 10 , order_2_pow)
double order_1 =     5.57701*10 ;//pow( 10 , order_1_pow)
double order_0 =     5.33930*10; //pow( 10 , order_0_pow)

int transfer_function(int);

#endif
