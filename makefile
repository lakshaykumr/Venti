all:
	gcc -c -g venti_freshversion.c
	gcc -o venti_freshversion  flow_thread_1.o venti_freshversion.o bcm2835.o callibration_functionlk.o error.o exhale_valve.o extra.o serial.o settings.o sir_valve_turbine.o timer_function.o turbine.o venti_parameter.o breaking.o alarm.o  -lm -lpthread

serial:
	gcc -c -g serial_read.c
	gcc -o sr serial_read.o serial.o

breaking:
	gcc -c -g breaking_test.c
	gcc -o breaking_test breaking_test.o sir_valve_turbine.o error.o exhale_valve.o bcm2835.o

clean:
	rm venti.o

