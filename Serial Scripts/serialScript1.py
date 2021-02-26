import serial
import time
import random
import os

os.system("clear")

ser1=serial.Serial("/dev/ttyUSB1", baudrate=57600,
 bytesize=serial.EIGHTBITS,
 parity=serial.PARITY_NONE,
 stopbits=serial.STOPBITS_ONE,
 timeout=5,
 xonxoff=False,
 rtscts=False,
 write_timeout=None,
 dsrdtr=False,
 inter_byte_timeout=None,
 exclusive=None)

print(hex(int(ser1.read().encode('hex'), 16)))

def sendVal():
	arrList=[]
	arrList.append(255)
	arrList.append(255)
	angle=random.randint(0, 1000)
	arrList.append(angle >> 8)
	arrList.append(angle & 0xFF)
	kd=random.randint(0, 255)
	arrList.append(kd)
	ki=random.randint(0, 255)
	arrList.append(ki)
	kp=random.randint(0, 255)
	arrList.append(kp)
	factor=random.randint(0, 255)
	arrList.append(factor)
	temp=arrList[2]+arrList[3]+arrList[4]+arrList[5]+arrList[6]+arrList[7]
	checker=temp%256
	arrList.append(checker)
	checNot=255-checker
	arrList.append(checNot)
	#print('angle->',angle,'arr->',arrList)		
	ser1.write(arrList)
	#print("Sended Vals=\n","Angle=",angle," kd=",kd," ki=",ki," kp=",kp," factor=",factor)

def readVal():
		arrList=[]
		type(arrList)
		in_bin1=ser1.read()
		in_hex=hex(int(in_bin1.encode('hex'), 16))
		
		if(hex(255)==in_hex):
			for x in range(9):
				in_bin1=ser1.read()
				in_hex=hex(int(in_bin1.encode('hex'), 16))
				arrList.append(in_hex)	
		
		if(len(arrList)>8):
			temp=int(arrList[1], 16)+int(arrList[2], 16)+int(arrList[3], 16)+int(arrList[4], 16)+int(arrList[5], 16)+int(arrList[6], 16)		
			checker=temp%256
			checNot=255-checker
			if (checker==int(arrList[7], 16) and checNot==int(arrList[8], 16)):
				angle=(int(arrList[1], 16) << 8) | int(arrList[2], 16)
				kd=arrList[3]
				ki=arrList[4]
				kp=arrList[5]
				factor=arrList[6]
				print("Read Vals=\n","Angle=",angle," kd=",kd," ki=",ki," kp=",kp," factor=",factor)
			else: 	
				angle=(int(arrList[1], 16) << 8) | int(arrList[2], 16)
				print("Read Vals= Uyusmadi","Angle=",angle," kd=",arrList[3]," ki=",arrList[4]," kp=",arrList[5]," factor=",arrList[6])
				print("Olmasi gereken=",checker,checNot,"Gelen=",int(arrList[7], 16),int(arrList[8], 16))


while 1:
	#command = b'\xFF\xFF\x01\x02\x00\x03\x02\x03\x0B\xF4'
	#command = b'\xFF\xFF\x01\x03\x01\x01\x02\x03\x0B\xF4'	
	#ser1.write(command)
	##time.sleep(0.0005)
	sendVal()	
	readVal()
	