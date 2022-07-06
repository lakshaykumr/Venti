from ctypes import*
import os
import time
import numpy as np

so_file = "/home/debian/emergency_lak/pressure.so"

pressureAms = CDLL(so_file)
pressureAms.start_bus()
#print(pressureAms.pressure(0x17))
sum=0
counter=0
x=Store_Ams=[]
y=Store_DC=[]
i=0
os.system("config-pin P9_42 pwm > /dev/null 2>&1")
os.system( "echo 20000000 >/dev/pwm/ecap0/period")
os.system("echo normal >/dev/pwm/ecap0/polarity")
os.system("echo 1 > /dev/pwm/ecap0/enable")
os.system("echo out > /sys/class/gpio/gpio33/direction")
os.system("echo 1 > /sys/class/gpio/gpio33/value")
try:
        while True:
                start=time.time()
		dtc=(i/1000.0)*(20000000)
		dc=int(dtc)
                os.system("echo {} >/dev/pwm/ecap0/duty_cycle".format(dc))
               # print("duty cycle  : {}" .format(i))
                time.sleep(2)

                while (time.time() - start <3):

                        Pressure = pressureAms.pressure(0x17)
                        pressuref = ((((Pressure - 3277.0) / ((26214.0) / 3.0))-1.5)*70.30)
                        #prescmh20 = pressuref * 70.3069
                        sum=pressuref + sum
                        counter=counter+1

                average= sum/counter
                #print("Average : {}".format(average))
                Store_Ams.append(average)
		#print("Average : {}".format(x))
		Store_DC.append(i)

		print("Duty Cycle : {}".format(i))
		print("Pressure Reading : {}".format(average))
       		print("==============================")

                sum=0
                counter=0
                i=i+100
                if (i>1000):
			curve = np.polyfit(x,y,3)
			poly  = np.poly1d(curve)
			print("Equation   :  {} \n" .format(poly))
		        print("------------------------------------------")
			os.system("echo 0 >/dev/pwm/ecap0/enable")
			break

except KeyboardInterrupt:

        os.system("echo 0 >/dev/pwm/ecap0/enable")
        print("OFF")




