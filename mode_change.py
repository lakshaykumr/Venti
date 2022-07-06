#!/user/bin/env python
import time
import serial
import subprocess
import os
import time
#import RPi.GPIO as GPIO #lk

f = open("/home/debian/emergency_lak/mode.txt","w")
f.write("90")
f.close()

print('i am in here')
setting_flag = 0
counter_flag = 0
calib_flag = 0
ping_delay = 0
ping_flag = 0
counter = 6
flag = 0
ping_time = 0
ping_delay = 0
stand_by_flag = 1
#time.sleep(5) #lk
#GPIO.setmode(GPIO.BCM) #lk
#GPIO.setup(12, GPIO.OUT)
#GPIO.setup(37, GPIO.OUT)
#GPIO.setup(22, GPIO.OUT)
#GPIO.setup(18,GPIO.OUT)
#motor_1=GPIO.PWM(18,50)
############################################rishub edit####################333
"""
wd_ser = serial.Serial(
        port='/dev/ttySOFT0',
        baudrate = 2400,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.05
)

wd_ser.close()
"""
############################################################################
ser = serial.Serial(
	port='/dev/ttyO1',
	baudrate = 9600,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS,
	timeout=0.05
)
#GPIO.output(12, GPIO.LOW)
#try:
 #       cipher_scan = CDLL("/home/debian/emergency_lak/cipher_scan.so")
#except:
 #       print("Error in cipher file opening")

#while 1:
#	ser.write("VER:CVP3_1AMPM1.5_2SMPM1000_H00M2")

ping_set = 1
f = open('/home/debian/emergency_lak/MUTE.txt',"w")
f.write("0")
f.close()
def buzzer(position, bit):
    try:
	f = open('/home/debian/emergency_lak/buzzer.txt', 'r')
	data = f.readline()
	f.close()
	print(data)
	print(position)
	print(bit)
	if(len(data) > 10):
	    data = data.split(',')
	    data = map(int,data)
	    data[position] = bit
	    data = map(str,data)
	    data = ','.join(data)
	else:
	    data = '0,0,0,0,0,0,0,0,0,0'
	print(data)
        try:
            f = open('/home/debian/emergency_lak/buzzer.txt','w')
	    f.write(str(data))
            f.close()
        except:
            print('unable to write to file')
    except:
	print('unable to read file')


while 1:
#	print('hello')
        time_now = time.time()
	try:
	    x=ser.readline()
	    print x
	except:
	    print('returned nothing')
	if(x == "CE"):
	    calib_flag = 1
	if(x[0:2] == "IH" or x[0:2] == "EH"):
	    if(x[0:2] == "EH"):
		ser.write("ACK67");
	    elif(x[0:2] == "IH"):
                ser.write("ACK66");

	  #  try:
	    f = open("/home/debian/emergency_lak/hold.txt","w")
	    f.write(x)
	    f.close()
	   # except IOError:
	   #	print("unable to write to file")
#----------------------------------Mannual Breath -------------
	if(x == "CM+BREATH"):
		try:
            		f = open("/home/debian/emergency_lak/mannual_breath.txt","w")
            		f.write('1')
            		f.close()
		except:
			print " Error in mannual Breath"
#---------------------------------------------------------------
	if(x == "CM+VER"):
	    try:
		ser.write("VER:CVP3_1AMPM1.5_2SMPM1000_H00M202_S_2_5.2AR_Z$")
	    except:
	   	print("unable to send")
	if(x == "BHOLD1"):
	    try:
	        f = open("/home/debian/emergency_lak/BHOLD.txt","w")
	        f.write("1")
	        f.close()
	    except:
		print("unable to write")
	if(x == 'CM+RTC'):
     	    os.system('sudo modprobe rtc-ds1307')
	    os.system("sudo echo 'ds1307 0x68' | sudo tee /sys/class/i2c-adapter/i2c-1/new_device")
	    os.system('sudo hwclock -r')
	    os.system('sudo hwclock -w')
	    os.system('sudo hwclock -r')
	if(x == 'CM+KILLALL'):
	    try:
		f = open("/home/debian/emergency_lak/flow_sensor.txt","r")
		data = f.read()
		f.close()
		time.sleep(0.01)
		if(data == "5"):
		    data = "10"
		else:
		    data = "5"
		f = open("/home/debian/emergency_lak/flow_sensor.txt","w")
		f.write(data)
		f.close()
	    except:
		print("unable open flow sensor file")
	    os.system('sudo shutdown -h now')
	if(x == 'CMR+LIFE'):
            #GPIO.output(37, GPIO.LOW)
	    subprocess.Popen("sudo python /home/debian/emergency_lak/life.py", shell = True)
            os.system('sudo pkill -f buzzer.py')
 #           os.system('sudo pkill -f recover.py')
            os.system('sudo pkill -f AgVa_relay_low.py')
            os.system('sudo pkill -f AgVa_relay.py')
            os.system('sudo pkill -f power.py')
            os.system('sudo pkill -f ABP_1.py')
            os.system('sudo pkill -f SDP.py')
            os.system('sudo pkill -f debug.py')
            motor_1.start(0)
	    #GPIO.output(22, GPIO.LOW)
#            subprocess.Popen("sudo python /home/debian/emergency_lak/life.py", shell = True)
            while(1):
                time.sleep(0.5)
                #GPIO.output(37, GPIO.HIGH)
                time.sleep(0.8)
                #GPIO.output(37, GPIO.LOW)
	if(x == 'CM+LCOMP1'):
	    try:
		f = open("/home/debian/emergency_lak/flowComp.txt","w")
		f.write("1")
		f.close()
	    except:
		print("unable to write CM+LCOMP1")
	if(x == 'CM+LCOMP0'):
	    try:
		f = open("/home/debian/emergency_lak/flowComp.txt","w")
		f.write("0")
		f.close()
	    except:
		print("unable to write CM+LCOMP0")
	if(x == 'CM+CALIBRATE'):
	    #GPIO.output(37, GPIO.LOW)
            os.system('sudo pkill -f buzzer.py')
 #           os.system('sudo pkill -f recover.py')
            os.system('sudo pkill -f AgVa_relay_low.py')
            os.system('sudo pkill -f AgVa_relay.py')
            os.system('sudo pkill -f power.py')
            os.system('sudo pkill -f ABP_1.py')
            os.system('sudo pkill -f SDP.py')
            os.system('sudo pkill -f debug.py')
	    subprocess.Popen('sudo python /home/debian/emergency_lak/O2_cal.py', shell = True)
	    #GPIO.output(22, GPIO.LOW)
            while(1):
                time.sleep(0.5)
                #GPIO.output(37, GPIO.HIGH)
                time.sleep(0.8)
                #GPIO.output(37, GPIO.LOW)
#	if(x == 'CM+STANDBY'):
#	    GPIO.output(37, GPIO.LOW)
#	    os.system('sudo pkill -f AgVa_relay_low.py')
#	    os.system('sudo pkill -f buzzer.py')
#	    GPIO.output(22, GPIO.LOW)
#	    ser.write("STND01")
#	    GPIO.output(18, GPIO.LOW)
#	    while(1):
#		x=ser.readline()
#		if(x == 'CM+WAKEUP'):
#		    subprocess.Popen('sudo python /home/debian/emergency_lak/AgVa_relay_low.py', shell = True)
#		    subprocess.Popen('sudo python /home/debian/emergency_lak/buzzer.py', shell = True)
#		    ser.write("STND00")
#		    break;
 #               time.sleep(0.5)
#                GPIO.output(37, GPIO.HIGH)
 #               time.sleep(0.8)
  #              GPIO.output(37, GPIO.LOW)
	#print stand_by_flag
        if(x == 'CM+STANDBY'):
                stand_by_flag = 1

                f = open("/home/debian/emergency_lak/mode.txt","w")
                f.write("90")
                f.close()
		ser.write("STND01")
#               x = ser.readline()
#	if(x == 'CM+WAKEUP'):
#		if( cipher_scan.read == 1):
#			stand_by_flag = 0
#		else if( cipher_scan.read == 0):
#			subprocess.Popen('sudo /home/debian/emergency_lak/venti.o', shell=True)

        if(x == 'CM+SC4' and (stand_by_flag == 1)):
                print("stand_by_flag")
                subprocess.Popen('sudo python /home/debian/emergency_lak/turbine.py', shell=True)

        if(x == 'CM+SC2' and(stand_by_flag == 1)):
                subprocess.Popen('sudo ./rishub_callib.o 2', shell=True)

        if(x == 'CM+SC1' and (stand_by_flag == 1)):
                subprocess.Popen('sudo ./rishub_callib.o 1', shell=True)


        if(x == 'CM+SC3' and (stand_by_flag == 1)):
                subprocess.Popen('sudo ./rishub_callib.o 3', shell=True)
	if(x == 'CM+SA' and (stand_by_flag == 1)):
		subprocess.Popen('sudo /home/pi/rishub_self_test/self_test/rishub_self_test.o', shell=True)
		time.sleep(4)
		f = open("connection_data.txt","r")
		data = f.readline()
		f.close()
		print data
		ser.write("SA" + data[2:15] + "!") #2 to 9 contains F1, F2, O2l, O2h and then co2, spo2, temp

	if(x == 'CMR+CLEAN'):
	    os.system('sudo pkill -f AgVa_relay_low.py')
	    os.system('sudo pkill -f buzzer.py')
	    #GPIO.output(22, GPIO.LOW)
	    #GPIO.output(18, GPIO.HIGH)
	    while(1):
                time.sleep(0.5)
             #   GPIO.output(37, GPIO.HIGH)
                time.sleep(0.8)
              #  GPIO.output(37, GPIO.LOW)       
	if(x == 'CM+MUTE0'):
	    try:
	        f = open('/home/debian/emergency_lak/MUTE.txt','w')
	        f.write('0')
		f.close()
	    except:
		print("not able to open file")
        elif(x == 'CM+MUTE1'):
            try:
                f = open('/home/debian/emergency_lak/MUTE.txt','w')
                f.write('1')
                f.close()
            except:
                print("not able to open file")
        elif(x == 'CM+MUTE2'):
            try:
                f = open('/home/debian/emergency_lak/MUTE.txt','w')
                f.write('2')
                f.close()
            except:
                print("not able to open file")
	if(x == 'CMR+STP'):
	    #GPIO.output(37, GPIO.LOW)
	    os.system('sudo pkill -f AgVa_relay_low.py')
	    os.system('sudo pkill -f buzzer.py')
	    #GPIO.output(22, GPIO.LOW)
	    ser.write("STP007")
	    #GPIO.output(18, GPIO.LOW)
	    subprocess.Popen('sudo python /home/debian/emergency_lak/self_test.py', shell = True)
	    while(1):
                time.sleep(0.5)
                #GPIO.output(37, GPIO.HIGH)
                time.sleep(0.8)
                #GPIO.output(37, GPIO.LOW)
	if(x == 'CMR+AgVa'):
	    subprocess.Popen("sudo python /home/debian/emergency_lak/AgVa_relay.py", shell = True)
	    try:
		ser.write('Running AgVa')
	    except:
		print('Error 101')
	if(x == 'CMR+power'):
	    subprocess.Popen("sudo python /home/debian/emergency_lak/power.py", shell = True)
	    try:
		ser.write('Running power')
	    except:
		print('Error 102')
	if(x == 'CM+KILLAgVa'):
	    os.system('sudo pkill -f AgVa.py')
	if(x == 'CM+KILLpower'):
	    os.system('sudo pkill -f power.py')
	if(x == 'CMR+ABP'):
	    subprocess.Popen("sudo python /home/debian/emergency_lak/ABP_1.py", shell = True)
	if(x == 'CMR+SDP'):
	    subprocess.Popen("sudo python /home/debian/emergency_lak/SDP.py", shell = True)
	if(x == 'CM+KILLABP'):
	    os.system('sudo pkill -f ABP_1.py')
	if(x == 'CM+KILLSDP'):
	    os.system('sudo pkill -f SDP.py')
	if(x == 'CMR+DATA.py'):
	    subprocess.Popen("sudo python /home/debian/emergency_lak/debug.py", shell = True)
	if(x == 'CM+SHUTDOWN'):
	    try:
		ser.write('ACK50')
		#GPIO.output(37, GPIO.HIGH)
	    except:
		print('unable to send')
	    GPIO.output(12, GPIO.HIGH)
	    print('gpio high')
	#    os.system('sudo shutdown -h now')
	if(x == 'CM+KILLDATA'):
	    os.system('sudo pkill -f debug.py')
	    ser.write('DATA killed')
	if(x == 'CMR+BOOT'):
	    subprocess.Popen("sudo python /home/debian/emergency_lak/fastboot.py", shell = True)
	if(x == "HS"):
#	    GPIO.output(32, GPIO.LOW)
	    time.sleep(0.1)
	    counter = 6
            try:
                ser.write('ACK51')
                time.sleep(2)
       #         try:
       #             f = open("/home/debian/emergency_lak/init_calib.py","r")
       #             data = f.readline()
       #             f.close()
       #             data = "CALIB"+data
       #             print(len(data))
       #             print(data)
       #             if(len(data) == 10):
       #                 ser.write(data)
       #         except:
       #             print("ERROR")
                ser.write("CALIB03030")
            except:
                print('unable to send')
	    ping_time = time.time()
	    counter_flag = 1
	    #GPIO.output(37, GPIO.LOW)
	 #   print(os.path.exists("/home/debian/emergency_lak/MODEL"))
	  #  if((os.path.exists("/home/debian/emergency_lak/MODEL")) == False):
	#	time.sleep(5)
	#	print("GOT INSIDE")
	#	ser.write("RQDN0")
	exists= os.path.isfile('/home/debian/emergency_lak/setting.txt')
#	print(x[0:13])
	if(x[0:12] =="device_name="):
	    print(x[12:].isalpha())
	    if(x[12:].isdigit() == False):
		ser.write("RQDN2")
	    else:
	        try:
	            f = open("/home/MODEL","w")
	            f.write("0D "+x[12:])
	            f.close()
		    ser.write("RQDN1")
	        except:
		    ser.write("RQDN2")
	if(exists and x == "222"):
	    try:
	        ser.write('ACK03')
	    except:
		print('not able to send')
	if(x == 'PING'):
            if((os.path.exists("/home/MODEL")) == False):
        #       time.sleep(5)
                print("GOT INSIDE")
                ser.write("RQDN0")
	    if(calib_flag == 1):
		ser.write("ACK68")
	    ping_delay = time.time() - ping_time
	    ping_time = time.time()
	    ping_flag = 1
	if(ping_flag == 1):
	    if(ping_set == 1):
		buzzer(3,0)
		ping_set = 0
	    ping_delay = time.time() - ping_time
	if(ping_delay > 30):
	    if(ping_set == 0):
	        buzzer(3,1)
		ping_set = 1
	if(len(x) >4):
	    values = x.split(',')
	    print( "Length = " + str(len(values)))
	    try:
  	        if(values[0] == 'S' or values[0] == 'PINGS') and (values[19] =='#' or values[19] == '#S' or values[0] == '#PING'):
		   try:
			    print "first"
			    values.pop(0)
			#values.pop(36)
		#	if(len(values) == 12):
#			    print "Second"
		            f = open("/home/debian/emergency_lak/setting.txt","w")
			    now_str = (','.join(values[0:18]))
		            f.write(now_str + ',')
		            f.close()
###############################################################rishub edit################3

#			    print "start watchdog"
#			    wd_string = now_str
#			    wd_string = wd_string.split(',')
#			    wd_string = ('S'+wd_string[0]+','+wd_string[3]+','+wd_string[6]+'#')
#			    print(wd_string)
#			    wd_ser.open()
#			    wd_ser.write(wd_string)
#			    wd_ser.close()
#			    print "end watchdog"

#############################################################################################
			    setting_flag = 1
			    ser.write("ACK04")
			    print "Second"
                            f = open("/home/debian/emergency_lak/fio2.txt","w")
                            #now_str = (','.join(values[0:18]))
                            f.write(values[8])
                            f.close()

		   except:
			    print('unable to open file')
	    	elif(values[0] == 'L' or values[0] == 'PINGL') and (values[15] =='#' or values[15] == '#L' or values[15] == '#PING'):
		#	try:
                            print "first"
                            values.pop(0)
			    #values.pop(15)
                        #values.pop(36)
                #       if(len(values) == 12):
#                           print "Second"
                            f = open("/home/debian/emergency_lak/alarm_limit.txt","w")
                            now_str = (','.join(values[0:14]))
			    print now_str
                            f.write(now_str + ',')
                            f.close()
                            setting_flag = 1
                     #       ser.write("ACK04")
                   	#except:
                            #print('unable to open file')

#		elif(values[0] == 'S2' and setting_flag == 1):
#		    print('1st step')
#		    try:
#			print('2nd step')
#			values.pop(0)
#			try:
#			    f = open("/home/debian/emergency_lak/O2_set.txt","w")
#			    f.write(values[3])
#			    f.close()
#			except:
#			    print("unable to save fio2 set")
#			try:
#			    f = open("/home/debian/emergency_lak/flag.txt","w")
#			    f.write("1")
#			    f.close()
#			except:
#			    print("hello")
#			values.pop(3)
#			print('3rd step')
#			print('trying to append')
#		        f = open("/home/debian/emergency_lak/setting.txt","a")
#		        f.write(','.join(values))
#		        f.close()
#			try:
#			    ser.write("S@" + ','.join(values) + "#")
#			except:
#			    print("unable to send setting updated")
#			setting_flag = 0
#		        try:
#			    ser.write('ACK04')
#		        except:
#			    print('not able to send')
#		    except:
#			print('unable to open send')
	    except:
	        print('error opening file')


#	print(x)
	mode_exists = os.path.isfile('/home/debian/emergency_lak/mode.txt')
#	print('counter is')
#	print(counter)
#	print('counter flag is')
#	print(counter_flag)
	if(counter < 1 and counter_flag == 1):
	    print(counter)
	    counter = counter + 1
	    flag = 1
	    try:
		ser.write('ACK02')
	    except:
		print('not able to send')
	if(counter >=5 and flag == 1):
	    flag = 0
	    print('setting flag to zero')
	    counter_flag = 0
	if((len(x) ==2) and (x == "11" or  x =="12" or x =="13" or x=="17" or x == "18" or x =="21"  or x =="22"  or x =="23" or x =="24"or x =="25" or x =="31" or x =="32" or x =="33" or x =="34"  or  x == "35" or  x == "26")):
	    stand_by_flag = 0
	    ser.write("STND00")
	    try:
		f = open("/home/debian/emergency_lak/mode.txt","w")
		f.write(str(x))
		f.close()
		print('sending the data')
		counter = 0
		try:
			ser.write('ACK02')
			counter = counter +1

		except:
			print('unable to update mode')
	    except:
		print('unable to open mode file')
##	ser.flush()
#        print(time.time() - time_now)
