import RPi.GPIO as GPIO
#import smbus
from smbus2 import SMBus 
import time
import struct,binascii
GPIO.setmode(GPIO.BCM)
GPIO.setup(17,GPIO.IN,pull_up_down=GPIO.PUD_UP)

i2c = SMBus(1)
Slave_address = 0x0A
value1 = [0xAA, 0x09, 0x04, 0x97, 0x01, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x00]
value2 = [0xAA, 0x09, 0x04, 0x99, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9D, 0x00]
#value = [0xAA, 0x09, 0x04, 0x98, 0x01, 0x01, 0x00, 0x00, 0x00, 0x9E, 0x00]
#value = [0xAA, 0x09, 0x04, 0x97, 0x01, 0x01, 0x00, 0x00, 0x00, 0x9D, 0x00]
value= [0xAA, 0x09, 0x04, 0x96, 0x01, 0x01, 0x00, 0x00, 0x00, 0x9C, 0x00]
wave_stop = [0xAA, 0x09, 0x04, 0x97, 0x01, 0x00, 0x00, 0x00, 0x00, 0x9C, 0x00]
i2c.write_i2c_block_data(Slave_address,0x55,value)
#i2c.write_i2c_block_data(Slave_address,0x55,value2)

#i2c.read_i2c_block_data(Slave_address,0x0D)

#time.sleep(2)
#i2c.read_i2c_block_data(Slave_address,0x0D)

count = 0
#def count_edge(channel):
#	global count
#	count = count+1
#	if(count==1):
#		i2c.read_i2c_block_data(Slave_address)
#	else:
#		i2c.read_i2c_block_data(Slave_address)
data = []
#[0xAA, 0x09, 0x04, 0x93, 0x01, 0x0B, 0x00, 0x00, 0x00, 0xA3, 0x00], sample for crc
param = [
    [0xAA, 0x09, 0x04, 0x80, 0x01, 0x45, 0x01, 0x00, 0x00, 0xCB, 0x00],
    [0xAA, 0x09, 0x04, 0x81, 0x01, 0x08, 0x00, 0x00, 0x00, 0x8E, 0x00],
    [0xAA, 0x09, 0x04, 0x82, 0x01, 0x40, 0x1F, 0x00, 0x00, 0xE6, 0x00],
    [0xAA, 0x09, 0x04, 0x83, 0x01, 0xFA, 0x00, 0x00, 0x00, 0x82, 0x01],
    [0xAA, 0x09, 0x04, 0x84, 0x01, 0x1F, 0x00, 0x00, 0x00, 0xA8, 0x00],
    [0xAA, 0x09, 0x04, 0x85, 0x01, 0x00, 0x00, 0x74, 0x42, 0x40, 0x01],
    [0xAA, 0x09, 0x04, 0x86, 0x01, 0x40, 0x1F, 0x00, 0x00, 0xEA, 0x00],
    [0xAA, 0x09, 0x04, 0x87, 0x01, 0xC8, 0x00, 0x00, 0x00, 0x54, 0x01],
    [0xAA, 0x09, 0x04, 0x88, 0x01, 0xD0, 0x07, 0x00, 0x00, 0x64, 0x01],
    [0xAA, 0x09, 0x04, 0x89, 0x01, 0x28, 0x00, 0x00, 0x00, 0xB6, 0x00],
    [0xAA, 0x09, 0x04, 0x8A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x00],
    [0xAA, 0x09, 0x04, 0x8B, 0x01, 0x00, 0x00, 0x00, 0x00, 0x90, 0x00],
    [0xAA, 0x09, 0x04, 0x8C, 0x01, 0x64, 0x00, 0x00, 0x00, 0xF5, 0x00],
    [0xAA, 0x09, 0x04, 0x8D, 0x01, 0x03, 0x00, 0x00, 0x00, 0x95, 0x00],
    [0xAA, 0x09, 0x04, 0x8E, 0x01, 0x32, 0x00, 0x00, 0x00, 0xC5, 0x00],
    [0xAA, 0x09, 0x04, 0x8F, 0x01, 0x40, 0x0D, 0x03, 0x00, 0xE4, 0x00],
    [0xAA, 0x09, 0x04, 0x90, 0x01, 0x20, 0x4E, 0x00, 0x00, 0x03, 0x01],
    [0xAA, 0x09, 0x04, 0x91, 0x01, 0x10, 0x27, 0x00, 0x00, 0xCD, 0x00],
    [0xAA, 0x09, 0x04, 0x92, 0x01, 0x80, 0x01, 0x00, 0x00, 0x18, 0x01],
    [0xAA, 0x09, 0x04, 0x93, 0x01, 0x0A, 0x00, 0x00, 0x00, 0xA2, 0x00],
    [0xAA, 0x09, 0x04, 0x94, 0x01, 0x00, 0x00, 0x00, 0x00, 0x99, 0x00],
    [0xAA, 0x09, 0x04, 0x9E, 0x01, 0x40, 0x0D, 0x03, 0x00, 0xF3, 0x00],
    [0xAA, 0x09, 0x04, 0x9F, 0x01, 0x78, 0x00, 0x00, 0x00, 0x1C, 0x01],
    [0xAA, 0x09, 0x04, 0x95, 0x01, 0x01, 0x00, 0x00, 0x00, 0x9B, 0x00],
    [0xAA, 0x09, 0x04, 0x9B, 0x01, 0xC8, 0x00, 0xF0, 0x00, 0x58, 0x02],
    [0xAA, 0x09, 0x04, 0x9C, 0x01, 0x00, 0x00, 0x00, 0x00, 0xA1, 0x00],
    [0xAA, 0x09, 0x04, 0xA0, 0x01, 0x00, 0x00, 0x00, 0x00, 0xA5, 0x00]
    ]


#GPIO.add_event_detect(17,GPIO.FALLING,callback=count_edge,bouncetime=2)
while(1):
    P = GPIO.wait_for_edge(17,GPIO.FALLING,timeout=4000)
    if P is None:
        print("Timeout")
        break
    else:
        raw_data = i2c.read_i2c_block_data(Slave_address,0,15)
        data.append(raw_data)
        for i in range(len(param)):
            if(raw_data[2]==param[i][3]):
                i2c.write_i2c_block_data(Slave_address,0x55,param[i])
                break
            if(raw_data[2]==160):
                print("success")
#print(data)
"""
    raw_list = []
    raw_buffer = ""
    for i in range(5,1,-1):
        raw_buffer = raw_buffer + format(raw_data[i],'x').zfill(2)
    raw_hex = binascii.unhexlify(raw_buffer)
    raw_float = struct.unpack('!f',raw_hex)[0]
    raw_list.append(raw_float)
    raw_buffer = ""
    for i in range(9,5,-1):
        raw_buffer = raw_buffer + format(raw_data[i],'x').zfill(2)
    raw_hex = binascii.unhexlify(raw_buffer)
    raw_float = struct.unpack('!f',raw_hex)[0]
    raw_list.append(raw_float)
    raw_buffer = ""
    for i in range(13,9,-1):
        raw_buffer = raw_buffer + format(raw_data[i],'x').zfill(2)
    raw_hex = binascii.unhexlify(raw_buffer)
    raw_float = struct.unpack('!f',raw_hex)[0]
    raw_list.append(raw_float)
    raw_buffer = ""
    for i in range(17,13,-1):
        raw_buffer = raw_buffer + format(raw_data[i],'x').zfill(2)
    raw_hex = binascii.unhexlify(raw_buffer)
    raw_float = struct.unpack('!f',raw_hex)[0]
    raw_list.append(raw_float)
    data.append(raw_list)
    print(data)
"""
"""
    result = ""
    for i in range(7,3,-1):
        result = result + format(raw_data[i],'x').zfill(2)
    print(raw_data[2])
    print(result)
    h = binascii.unhexlify(result)
    print(struct.unpack('!f',h)[0])
    #print(struct.unpack('f',struct.pack("i",int(result,16))))    
        #print(data[x])

    if count>30:
        i2c.write_i2c_block_data(Slave_address,0x55,wave_stop)
        break
        """
"""

        #		if(count==1):
		#data.append(i2c.read_i2c_block_data(Slave_address,0x0D))
#			print(data)
#		else:
#			data = i2c.read_i2c_block_data(Slave_address,0x3E)
#		print(data)

#import matplotlib.pyplot as plt
#import numpy as np
"""
"""
x = []
for i in range(1,101):
	x.append(i)
y = []
for i in range(1,16):
	for j in range(1,14):
		if(data[i][2*j+5]>=128): #negative
			y.append(-1*((255^data[i][2*j+4])+1+(256*(data[i][2*j+5]^255))))
		else:
			y.append(data[i][2*j+4]+(256*data[i][2*j+5]))
for j in range(1,10):
	if(data[16][2*j+5]>=128):
		y.append(-1*((255^data[i][2*j+4])+1+(256*(data[16][2*j+5]^255))))
	else:
		y.append(data[16][2*j+4]+(256*data[16][2*j+5]))
#print("len(X)", len(x))
print("len(y)", len(y))
#print(y)
#plt.plot(x,y)
#plt.show()
"""
"""
#----------------------------------------------------------------------------



#	if(count!=0):
#		print(count)
#	if(GPIO.input(17)==0):
#		count = count+1
#		print(count)
"""