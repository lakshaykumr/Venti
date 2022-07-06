#include <pthread.h>
#include "flow_thread_1.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include "mine.h"
#include <unistd.h>
#include <math.h>
#include <stdint.h>
#define VREF 5.048
#define RES VREF/32767.00
//global shared variable
float calc_flow_insp = 0.0;
float calc_flow_exp = 0.0;

extern pthread_mutex_t mutex;

//float voltage_flow(float);
//void adc_start2();
//int spi_init(void);
//int ADC114S08_RegWrite(uint8_t regnum, uint8_t data , uint8_t length);
//int ADC114S08_RegRead(uint8_t regnum, uint8_t data, uint8_t length );
//void adc_start();
//void set_adc_pin(uint8_t pin_no , uint8_t gain);



static uint8_t spiMode = SPI_MODE_1;
static uint8_t bits = 8;
static uint32_t speed = 61000;
static uint8_t lsbFirst = 0;	//Writing a 0 indicates MSb first

struct spi_ioc_transfer xfer[2];
int fd_gpio;
int fd;

int CS_Init()
{
	//Set the pin output
	fd_gpio = open("/sys/class/gpio/gpio5/direction", O_WRONLY);
	if (fd_gpio == -1)
	{
		perror("Unable to open direction\n");
		return -1;
	}
	if (write(fd_gpio, "out", 3) != 3)
	{
		perror("Error writing to direction\n");
		return -1;
	}
	close(fd_gpio);
}

int CS_High()
{
	fd_gpio = open("/sys/class/gpio/gpio5/value", O_WRONLY);
	if (fd_gpio == -1)
	{
		perror("Unable to open value\n");
		return -1;
	}
	if (write(fd_gpio, "1", 1) != 1)
	{
		perror("Error writing to value 1\n");
		return -1;
	}
	close(fd_gpio);
}

int CS_Low()
{
	fd_gpio = open("/sys/class/gpio/gpio5/value", O_WRONLY);
	if (fd_gpio == -1)
	{
		perror("Unable to open value\n");
		return -1;
	}
	if (write(fd_gpio, "0", 1) != 1)
	{
		perror("Error writing to value 1\n");
		return -1;
	}
	close(fd_gpio);
}


void spiRx(uint8_t *bufR, uint8_t length)
{
	memset(bufR, 0, sizeof bufR);

	xfer[1].rx_buf = (unsigned long) bufR;
	xfer[1].len = length;

	CS_Low();
	ioctl(fd, SPI_IOC_MESSAGE(2), xfer);
	CS_High();
}

void spiTx(uint8_t bufT[], uint8_t length)
{
	memset(xfer, 0, sizeof xfer);

	xfer[0].tx_buf = (unsigned long) bufT;
	xfer[0].len = length;

	CS_Low();
	ioctl(fd, SPI_IOC_MESSAGE(2), xfer);
	CS_High();
}


int ADC114S08_RegRead(uint8_t regnum)
{
	uint8_t tx_buff[2];
	uint8_t rx_buff[3];

	memset(tx_buff, 0, sizeof(tx_buff));
	memset(rx_buff, 0, sizeof(rx_buff));

	tx_buff[0] = 0x20 + (regnum & 0x1f);
	tx_buff[1] = 0x00;

	spiTx(tx_buff,2);

	spiRx(rx_buff,1);
//	printf("rx_buff:%x \n", rx_buff[0]);
	return rx_buff[0];
}
int ADC114S08_RegWrite(uint8_t regnum, uint8_t data)
{
	uint8_t tx_buff[3];
	uint8_t rx_buff[3];

	memset(tx_buff, 0, sizeof(tx_buff));
	memset(rx_buff, 0, sizeof(rx_buff));

	tx_buff[0] = 0x40 + (regnum & 0x1f);
	tx_buff[1] = 0x00;
	tx_buff[2] = data;

	spiTx(tx_buff,3);

	uint8_t rxd_Data = 0;
	rxd_Data = ADC114S08_RegRead(regnum);
	if(rxd_Data == data)
	{
		//printf("%x match\n",data);
		return 0;
	}
	else
	{
		//printf("%x not match\n",data);
		return -1;
	}
}

float voltage_flow(float inp)
{//		printf(" Function input voltage awm = %f\n" , inp);
		float input_data3;
        	float output_data3 = 0;
                input_data3 = inp;
//		printf(" Input data after divide %f \n" ,input_data3);
  //      value2 = value2/1000.00;
                if (input_data3>=0.99 && input_data3<=2.89)
                        {//      printf(" Analog output  == %f" , value);
                                output_data3 =  15.78947*input_data3 - 15.63158;
                              //  printf(" loop 1 %f," ,output_data3);
                        }
                else if (input_data3>2.89 && input_data3<=3.6)
                        {//      printf(" Analog output  == %f " , value);
                                output_data3 =28.16901*input_data3 - 51.40845 ;
                            //    printf(" loop 2 %f," ,output_data);
                        }
                else if (input_data3>3.6 && input_data3<=4.4)
                        {//      printf(" Analog output  == %f " , value);
                                output_data3 =62.5*input_data3 - 175 ;
                        ///	printf(" loop 3 %f," ,output_data);
                        }
                else if (input_data3>4.4 && input_data3<=4.7)
                        {//      printf(" Analog output  == %f " , value);
                                output_data3 = 166.6667*input_data3 - 633.3333 ;
                           //     printf(" loop 4 %f," ,output_data);
                        }
                else if (input_data3>4.7 && input_data3<=4.9)
                        {//      printf(" Analog output  == %f " , value);
                                output_data3 = 250*input_data3 - 1025 ;
                             //   printf(" loop 5 %f," ,output_data);
                        }

		 else if( input_data3>4.9 )
                        {//      printf(" Analog output  == %f " , value);
                                output_data3 = 357.1429 * input_data3 - 1585.714;
                               // printf(" loop 6 %f," ,output_data);
                        }
		else if (input_data3 <= 0.5 || input_data3 >= 5.5){
			output_data3 = 0;
			}
//	printf(" flow from awm sensor %f\n" ,output_data3);

	return output_data3;

}
float bi_directional(float input_voltage_bi)
{
	float y = 0;
//	printf(" Function voltage_posifaaa = %f\n" , input_voltage_bi);
	
	
	  y = ((( input_voltage_bi-2.52)/4) * 400) ;
//	  printf("flow from posifa :%f\n" , y);

/*	float y = 0;
	float x = input_voltage_bi;
	if(input_voltage_bi  >= 0.86 &&  input_voltage_bi  <= 1.26)
	{
			y =  125*x - 307.5;
	}
	else if(input_voltage_bi  > 1.26 &&  input_voltage_bi  <= 1.65) 
        {
                        y = 128.2051*x - 311.5385;
        }
	else if(input_voltage_bi  > 1.65 &&  input_voltage_bi  <= 2.07) 
        {
                        y = 119.0476*x - 296.4286;
        }
	else if(input_voltage_bi  > 2.07 &&  input_voltage_bi  <= 2.23) 
        {
			y = 125*x - 308.75;

                     //   y = 90.90909*x - 227.2727;
		//	printf(" in loop 2  %f \n",y);
        }
	else if(input_voltage_bi  > 2.23 &&  input_voltage_bi  <= 2.5) 
        {
                        y = 111.1111*x - 277.7778;

                     //   y = 90.90909*x - 227.2727;
                  //      printf(" in loop 2  %f \n",y);
        }

	else if(input_voltage_bi  > 2.5 &&  input_voltage_bi  <= 2.8) 
        {			y = 100*x - 250;
                      //  y = 181.8182*x - 554.5455;
	//		 printf(" in loop 3  %f \n",y);
        }
	else if(input_voltage_bi  > 2.8 &&  input_voltage_bi  <= 2.96) 
        {
                        y = 125*x - 320;
                      //  y = 181.8182*x - 554.5455;
          //               printf(" in loop 3  %f \n",y);
        }

	else if(input_voltage_bi  > 2.96 &&  input_voltage_bi  <= 3.38)
        {
                        y = 119.0476*x - 302.381;
	//		 printf(" in loop 4  %f \n",y);   
        }

	else if ((input_voltage_bi > 3.38) || (input_voltage_bi < 3.8))
	{		
			y= 119.0476*x - 302.381;
	}
	
	 else if ((input_voltage_bi > 3.8) || (input_voltage_bi < 4.25))
        { 
			y= 111.1111*x - 272.2222 ;
        }
	else if ((input_voltage_bi > 4.25) || (input_voltage_bi < 0.3))
	{		y= 0 ;
		printf(" Not in range \n");

	}
	printf("flow from posifa :%f\n" , y);*/



	  return y;


}

int spiInit()
{
	uint8_t readSpiMode;
	uint8_t readBits;
	uint32_t readSpeed;
	uint8_t readLsbFirst;

	fd = open("/dev/spidev0.1", O_RDWR);
	if (fd < 0)
	{
		perror("Can't open spi device");
		return -1;
	}

	//spi mode
	int ret = ioctl(fd, SPI_IOC_WR_MODE, &spiMode);
	if (ret == -1)
	{
		perror("can't set spi mode");
		return -1;
	}

	ret = ioctl(fd, SPI_IOC_RD_MODE, &readSpiMode);
	if (ret == -1)
	{
		perror("can't get spi mode");
		return -1;
	}

	//bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
	{
		perror("can't set bits per word");
		return -1;
	}

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &readBits);
	if (ret == -1)
	{
		perror("can't get bits per word");
		return -1;
	}

	//max speed hz
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
	{
		perror("can't set max speed hz");
		return -1;
	}

	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &readSpeed);
	if (ret == -1)
	{
		perror("can't get max speed hz");
		return -1;
	}

	//lsb first
	ret = ioctl(fd, SPI_IOC_WR_LSB_FIRST, &lsbFirst);
	if (ret == -1)
	{
		perror("can't set lsb first");
		return -1;
	}

	ret = ioctl(fd, SPI_IOC_RD_LSB_FIRST, &readLsbFirst);
	if (ret == -1)
	{
		perror("can't get lsb first hz");
		return -1;
	}

	printf("spi mode: %d\n", readSpiMode);
	printf("bits per word: %d\n", readBits);
	printf("max speed: %d Hz\n", readSpeed);
	printf("lsb first: %d\n", readLsbFirst);
	printf("===================================\n");
	return 1;
}

void adc_start()
{
	ADC114S08_RegWrite( STATUS_ADDR_MASK,   0x80);// flpor write 1 to clear the flag
	ADC114S08_RegWrite( INPMUX_ADDR_MASK,   0xBA);// +v ain0 and -ve ain1  differential mode
	ADC114S08_RegWrite( PGA_ADDR_MASK,      0xE0);//14.tmod pga power down gain = 2
	ADC114S08_RegWrite( DATARATE_ADDR_MASK, 0x1D);// g_chp dis , internal clk=4.09Mhz , continous , l$
	ADC114S08_RegWrite( REF_ADDR_MASK,      0x00);// ref mon disable , +ve and -ve buffer disable,ref p0 a$
	ADC114S08_RegWrite( IDACMAG_ADDR_MASK,  0x00);// dasable pga , low side open write 00 , idac off
	ADC114S08_RegWrite( IDACMUX_ADDR_MASK,  0xff);// rdac both disconnected
	ADC114S08_RegWrite( VBIAS_ADDR_MASK,    0x00);// vbias off
	ADC114S08_RegWrite( SYS_ADDR_MASK,      0x1B);//disable sys monitor 8 sampls , spi & crc& status = dis$
	ADC114S08_RegWrite( OFCAL0_ADDR_MASK,   0x00);
	ADC114S08_RegWrite( OFCAL1_ADDR_MASK,   0x00);
	ADC114S08_RegWrite( OFCAL2_ADDR_MASK,   0x00);
	ADC114S08_RegWrite( FSCAL0_ADDR_MASK,   0x00);
	ADC114S08_RegWrite( FSCAL1_ADDR_MASK,   0x00);
	ADC114S08_RegWrite( FSCAL2_ADDR_MASK,   0x40);// default
	ADC114S08_RegWrite( GPIODAT_ADDR_MASK,  0x00);
	ADC114S08_RegWrite( GPIOCON_ADDR_MASK,  0x00);
}
//      bcm2835_spi_transfer(0x16);//self offset command
        // uint8_t buf21[1] = {0x19};
	 //spiTx(buf21,1);

        //usleep(5000);
//        bcm2835_spi_transfer(START_OPCODE_MASK);
//}




