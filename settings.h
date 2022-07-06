#ifndef settings
#define settings

int read_settings(void);
int mode_change(void);
void find_para(int , float);
typedef struct setting_parameter{
        float Pinsp;
        float peep;
        float Ti;
        float Fio2 ;
        float slope;
        float RR_rxd;
        float Plimit;
        float VTi;
        float Trigger;
        float PEAK_flow;
        float Psupp;
        float slope_flag;
	float Tlow;
	float insp_term;
	float apnea_on_off;
	float apnea_RR;
	float apnea_Vt;
	float apnea_trigger_flow;
	float apnea_time;
	float apnea_Ti;
	float apnea_Te;
}setting_parameter;
extern setting_parameter final_setting;



#endif
