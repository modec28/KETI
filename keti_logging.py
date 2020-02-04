import os, sys, serial, time
import struct
from threading import Thread

#ser = serial.Serial('/dev/ttyUSB1', 9600, 8 ,timeout = 0.1)
ser = serial.Serial('/dev/ttyUSB0', 9600, 8, timeout = 0.1)
timer = 0
time_flag=False
mode = 'testx'
#print(ser.is_open)
def th_timer():
	while True:
		global timer
		timer = timer+1
		time.sleep(1)
		global time_flag
		if(time_flag==True):
			break

def run():
	global timer
	global time_flag
	print("timer : ",timer)
	print("Require Data from Standard meter")
	
	while True:
                #Large
                ser.write(bytes(bytearray([0x01,0x03,0x00,0x00,0x00,0x14,0x45,0xC5])))
                large = ser.read(50)
                #print(large)
                #Small_integrated
                #ser2.write(bytes(bytearray([0x02,0x03,0x00,0x00,0x00,0x13,0x04,0x34])))
                #Small_instantaneous
                
                try:
                        hex_list = ["{:x}".format(ord(c)).zfill(2) for c in large]
                        result = ''.join(hex_list)
                        temperature = result[10:14]+result[6:10]
                        float_temper = struct.unpack('!f',temperature.decode('hex'))[0]
                        print("temperature : ", float_temper)
                except:
                        pass

                try:
                        current_flowrate_large = result[34:38] + result[30:34]
                        float_current_flow = struct.unpack('!f',current_flowrate_large.decode('hex'))[0]
                        print("Large Standard instantaneous flow rate : ",round(float_current_flow,5))
                except:
                        pass

                try:
                        intergrated_flowrate_large_H = struct.unpack('!f',(result[42:46] + result[38:42]).decode('hex'))[0]
                        intergrated_flowrate_large_L = struct.unpack('!f',(result[50:54] + result[46:50]).decode('hex'))[0]
                        integ_flow = intergrated_flowrate_large_H*100+intergrated_flowrate_large_L
                        global temp_integ_flow
                        global count
                        if(count==0):
                                temp_integ_flow = integ_flow
                        if(timer>60):
                                timer=0
                                print("")
                                print("")
                                print(integ_flow-temp_integ_flow)
                                print("")
                                print("")
                                ##test
                                if(mode=='test'):
                                        time_flag=True
                                break
                except:
                        pass
                ser.write(bytes(bytearray([0x02,0x04,0x00,0x00,0x00,0x03,0xB0,0x38])))
                small = ser.read(50)
                #hex_list2 = ["{:x}".format(ord(d)).zfill(2) for d in small]
                #print(hex_list2)
                #result2 = ''.join(hex_list2)
                #print(result2)
                result2 = ''.join(format(x,'02x')for x in small)
                current_flowrate_small = result2[14:18]
                int_flow = int(current_flowrate_small,16)
                print("Small Standard instantaneous flow rate : ",round(int_flow*0.01,2))

def main():
	t1 = Thread(target=th_timer)
	t1.start()
	global count
	count = 0
	global time_flag
	while True:
		run()
		count = count+1
		if(time_flag==True):
			print("end")
			break
main()

#while True:
#	run()
#	time.sleep(1)
