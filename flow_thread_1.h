#ifndef flow_thread_1
#define flow_thread_1
#include <stdint.h>

float voltage_flow(float);
float bi_directional(float);
int CS_Init();
int CS_High();
int CS_Low();
int spiInit();
void spiRx(uint8_t *bufR, uint8_t length);
void spiTx(uint8_t bufT[], uint8_t length);
int ADC114S08_RegRead(uint8_t regnum);
int ADC114S08_RegWrite(uint8_t regnum, uint8_t data);
void adc_start();

#endif


