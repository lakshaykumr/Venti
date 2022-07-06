//#include"define.h"
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <syslog.h>
#include <linux/i2c-dev.h>
#include "mine.h"
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#define VREF 5.048
#define RES VREF/32767.00

#include"error.h"
#define IO_ERROR_AMS -998
#define FILE_OPEN_ERROR_AMS -999

//system("config-pin P9_18 spi");
//system("config-pin P9_21 spi");
//system("config-pin P9_22 spi_sclk");

static uint8_t spiMode = SPI_MODE_1;
static uint8_t bits = 8;
static uint32_t speed = 61000;
static uint8_t lsbFirst = 0;	//Writing a 0 indicates MSb first

struct spi_ioc_transfer xfer[2];
int fd_gpio;
int fd;

char *bus = "/dev/i2c-2";
char flow_calib_path[] = "/home/debian/emergency/flow_data.txt";

char error_start_bus[]                  = "Failed to open the bus \n";
char error_flow_read[]                  =" Input/Output error at reading flow \n";
char error_pressure_read[]              = "Input/Output error at reading pressure \n";
char error_open_calib[]                 = "can't open flow_data.txt file \n";

float flow(int address);
float pressure();
int start_bus();
int file;
char data[4] = {0},data2[4] = {0};
double Flow_lpm ;
//double prescmh20;


int start_bus()                                                           //to start the bus
{
        if((file = open(bus, O_RDWR)) < 0)
        {
                printf("Failed to open the bus. \n");
        //      exit(1);
                //syslog_function(error_start_bus);
                return FILE_OPEN_ERROR_AMS;
        }
        return 0;
}

int file_write_flow(float val1, float val2)
{
	FILE *fp;
	fp = fopen("flow_data.txt","a");
	if(fp == NULL)       { printf("Error in openning file"); return 0; }

	fprintf(fp,"%f,%f,",val1,val2);
	fclose(fp);
   return 1;
}


int file_clear_flow()
{
        if(fclose(fopen("flow_data.txt", "w")))       { printf("file clear error\n"); return 0; }
        return 1;
}


int file_write_pressure(float val)
{
	FILE *fp;
	fp = fopen("pressure_data.txt","a");
	if(fp == NULL)       { printf("Error in openning file"); return 0; }

	fprintf(fp,"%f,",val);
	fclose(fp);
   return 1;
}

int file_clear_pressure()
{
        if(fclose(fopen("pressure_data.txt", "w")))       { printf("file clear error\n"); return 0; }
        return 1;
}



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

	//printf("spi mode: %d\n", readSpiMode);
	//printf("bits per word: %d\n", readBits);
	//printf("max speed: %d Hz\n", readSpeed);
	//printf("lsb first: %d\n", readLsbFirst);
	//printf("===================================\n");
	return 1;
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
	printf("rx_buff:%x \n", rx_buff[0]);
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

void adc_start()
{
	ADC114S08_RegWrite( STATUS_ADDR_MASK,   0x80);// flpor write 1 to clear the flag
	ADC114S08_RegWrite( INPMUX_ADDR_MASK,   0xBA);// +v ain0 and -ve ain1  differential mode
	ADC114S08_RegWrite( PGA_ADDR_MASK,      0xE0);//14.tmod pga power down gain = 2
	ADC114S08_RegWrite( DATARATE_ADDR_MASK, 0x1A);// g_chp dis , internal clk=4.09Mhz , continous , l$
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



float callibration_flow(void)
{
	if(file_clear_flow())        printf("flow file cleared\n");
        else    printf("flow file not cleared\n");	
	CS_Init();
	CS_High();
	spiInit();
	usleep(5000);

	//pwm_init();
        //turbine_duty_cycle(40);


	int adc_ID = 0x4;//id = 100 or 0x04 first 3 bits
	uint8_t analog_data1[3] = {0,0,0};
	uint8_t analog_data2[3] = {0,0,0};
	int output1 = 0;
	int output2 = 0;
	float value1 = 0.0;
	float value2 = 0.0;
	int msb1 = 0;
	int msb2 = 0;
	int sum = 0;
	float total1 = 0;
	float total2 = 0;
	int n = 20;
	float average1[n];
	float average2[n];
	float flow1_voltage = 0.0;
	float flow2_voltage = 0.0;

	//RESET
	uint8_t buf1[1] = {RESET_OPCODE_MASK};
	spiTx(buf1,1);
	usleep(5000);

	if(adc_ID == ((0x4) & (ADC114S08_RegRead(ID_ADDR_MASK))))
	{
		printf("ADS114S08  found\n");
	}

	if(0x80 == ADC114S08_RegRead(STATUS_ADDR_MASK))
	{
		printf("adc ready\n");
	}
	ADC114S08_RegWrite( STATUS_ADDR_MASK,   0x00); //clear FL_POR flag

	adc_start();

	//SELF OFFSET calibration
	uint8_t buf2[1] = {0x19};
	spiTx(buf2,1);

	//START
	uint8_t buf3[1] = {START_OPCODE_MASK};
	spiTx(buf3,1);
	usleep(5000);

	for(int i=0;i<n;i++)
        {
		ADC114S08_RegWrite( INPMUX_ADDR_MASK,   0xBA);
		usleep(410);
		uint8_t buf4[1] = {RDATA_OPCODE_MASK};
		spiTx(buf4,1);
		spiRx(analog_data1,4);
		analog_data1[1] = analog_data1[1] & 0b01111111;
		//for(int i=0; i<4; i++)
		//{
			//printf("%d ", analog_data[i]);
		//}
		//printf("\n");

		msb1 = analog_data1[1] << 8;
		output1 = msb1 + analog_data1[2];
		value1 = (RES) * (float)output1 * 1000.0;
		//printf("Output1: %d\n", output1);
		printf("Analog voltage 1: %f\n", value1/1000.0);
		average1[i] = value1/1000.00;
//		usleep(200000);
		
//		printf("===============\n");
//##########################################################################################
        	ADC114S08_RegWrite( INPMUX_ADDR_MASK,   0x98);
		usleep(410);
		uint8_t buf5[1] = {RDATA_OPCODE_MASK};
		spiTx(buf5,1);
		spiRx(analog_data2,4);
		analog_data2[1] = analog_data2[1] & 0b01111111;
		//for(int i=0; i<4; i++)
		//{
			//printf("%d ", analog_data[i]);
		//}
		//printf("\n");
		msb2 = analog_data2[1] << 8;
		output2 = msb2 + analog_data2[2];
		value2 = (RES) * (float)output2 * 1000.0;
		//printf("Output: %d\n", output2);
		printf("Analog voltage 2: %f\n", value2/1000.0);
		average2[i] = value2/1000.00;
//		printf(" Analog voltage 2  == %f \n " , value2);
        }
	
	for(int j=0;j<n;j++)
		{
			total1 += average1[j];
			total2 += average2[j];
		}
	flow1_voltage = total1/(float)n;
	flow2_voltage = total2/(float)n;
	printf("flow voltage average value 1 = %f \n",total1/(float)n);
	printf("flow voltage average value 2 = %f \n",total2/(float)n);
	
	if(file_write_flow(flow1_voltage,flow2_voltage))   printf("file data written\n");
else    printf("error writting data\n");
return 1;
}
//--------------------------------pressure function----------------------------------------------------------------------
float pressure(int address)
{
	address = 0x78;

        ioctl(file, I2C_SLAVE, address);
        if(read(file, data, 4) != 4)
        {
                printf("Error : Input/Output error \n");
                return IO_ERROR_AMS;
        }
                int pressure_1 = (data[0] * 256 + data[1]);
                //pressure2 = ((((pressure_1 - 3277.0) / ((26214.0) / 3.0))-1.5)*70.30);
        //      printf("circuit pressure = %d\n",pressure2);
               float  pressuref = ((pressure_1 - 3277.0) / ((26214.0) / 1.5)) + 0;
                float pressure2 = pressuref * 70.307;      ////nidhi
		return (pressure2);
}

int callibration_pressure(void)
{
double offset_press=0;
double average_press;
int n=20;

start_bus();

for(int i=0; i<n; i++)
        {
        float prescmh20 = pressure(0x78);
        offset_press = prescmh20 + offset_press;
	//printf("pressure=%f \n",prescmh20);
        }
//printf("off = %f \n", offset_press);
average_press = offset_press/n;
printf("pressure = %f \n",average_press);

if(file_write_pressure(average_press))   printf("file data written\n");//pressure value
        				else    printf("error writting data\n");
return 1;
}


int main(int argc,char* argv[])
{
	int rec_arg = 0;
	printf("no. of arguments passed %d \n",argc);
	if(argc == 1)	{	printf("no valid argument passed \n");	exit(0);	}
	printf("argument value %d\n",atoi(argv[1]));
	if(rec_arg = atoi(argv[1]))	printf("conversion completed\n");
	else	printf("error main argument conversion \n");
	switch (rec_arg)
   		{
			case 1:  	if(callibration_flow()) printf("callibration process completed for flow sensor \n");
					else printf("callibration fault for flow sensor  \n");
					break;

			case 2:		if(callibration_pressure()) printf("callibration process completed for pressure sensor \n");
					else printf("callibration fault for pressure sensor  \n");
					break;

			case 3: 	printf("In progress (OXYGEN) \n");
					/*if(callibration_oxygen()) printf("callibration process completed for oxygen sensor \n");
					else printf("callibration fault for oxygen sensor  \n");*/
					break;

			case 4:		printf("turbine callibration in progress\n");
					break;

			default:	printf("Choice other than 1 2 3 4 is entered in callib code\n");
					break;
		}
}
