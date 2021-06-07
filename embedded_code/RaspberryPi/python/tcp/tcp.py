#!/usr/bin/python

import RPi.GPIO as GPIO
import serial
import time

ser = serial.Serial('/dev/ttyS0',115200)
ser.flushInput()

power_key = 4
rec_buff = ''
ServerIP = '116.30.216.33'#Please replace your IP 
Port = '5001'#Please replace your port
Message = 'Waveshare'

def power_on(power_key):
	print('SIM7080X is starting:')
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(power_key,GPIO.OUT)
	time.sleep(0.1)
	GPIO.output(power_key,GPIO.HIGH)
	time.sleep(2)
	GPIO.output(power_key,GPIO.LOW)
	time.sleep(2)
	ser.flushInput()
	print('SIM7080X is ready')

def power_down(power_key):
	print('SIM7080X is loging off:')
	GPIO.output(power_key,GPIO.HIGH)
	time.sleep(1)
	GPIO.output(power_key,GPIO.LOW)
	time.sleep(2)
	print('Good bye')
	
def send_at(command,back,timeout):
	rec_buff = ''
	ser.write((command+'\r\n').encode())
	time.sleep(timeout)
	if ser.inWaiting():
		time.sleep(0.1 )
		rec_buff = ser.read(ser.inWaiting())
	if rec_buff != '':
		if back not in rec_buff.decode():
			print(command + ' ERROR')
			print(command + ' back:\t' + rec_buff.decode())
			return 0
		else:
			print(rec_buff.decode())
			return 1
	else:
		print(command + ' no responce')

try:
	power_on(power_key)
	send_at('AT+CSQ','OK',1)
	send_at('AT+CPSI?','OK',1)
	send_at('AT+CGREG?','+CGREG: 0,1',0.5)
	send_at('AT+CNACT=0,1','OK',1)
	send_at('AT+CACID=0', 'OK',5)
	send_at('AT+CAOPEN=0,\"TCP\",\"'+ServerIP+'\",'+Port,'+CAOPEN: 0,0', 5)
	send_at('AT+CASEND=0,9,10000', '>', 2)#If not sure the message number,write the command like this: AT+CIPSEND=0, (end with 1A(hex))
	ser.write(Message.encode())
	time.sleep(10);
	print('send message successfully!')
	send_at('AT+CACLOSE=0','OK',15)
	send_at('AT+CNACT=0,0', 'OK', 1)
	power_down(power_key)
except:
	if ser != None:
		ser.close()
		GPIO.cleanup()

if ser != None:
		ser.close()
		GPIO.cleanup()
