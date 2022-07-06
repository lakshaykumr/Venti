#ifndef alarm
#define alarm
#include <stdbool.h>
/*typedef struct alarm_limits{
        float Pressure_low;
        float Pressure_high;
        float Vte_low;
        float Vte_high;
        float Mve_low;
        float Mve_high;
        float RR_low;
        float RR_high;
        float Ti_low;
        float Ti_high;
}alarm_limits;

extern  alarm_limits alarm_struct;
*/

void alarm_setting_funct();
void set_limit(int index ,float value);
int clear_buzzer();
int buzzer(int index , bool alarm_flag);
int buzzer_write_function(int, char);
int pressure_alarm(float pip);
int Vte_alarm(float pip);
int RR_alarm(float pip);
int PEEP_alarm(float pip);
int Mve_alarm(float pip);
int spo2_alarm(bool);
int patient_disconn_alarm(bool);
int backup_ventilation_alarm(bool);

#endif


