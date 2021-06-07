#!/usr/bin/python

import RPi.GPIO as GPIO
import serial
import time

ser = serial.Serial("/dev/ttyS0",9600)
ser.flushInput()

power_key = 4
command_input = ''
rec_buff = ''

def power_on(power_key):
	print('SIM7080X is starting:')
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(power_key,GPIO.OUT)
	time.sleep(0.1)
	GPIO.output(power_key,GPIO.HIGH)
	time.sleep(1)
	GPIO.output(power_key,GPIO.LOW)
	time.sleep(1)
	ser.flushInput()
	print('SIM7080X is ready')

def power_down(power_key):
	print('SIM7080X is loging off:')
	GPIO.output(power_key,GPIO.HIGH)
	time.sleep(2)
	GPIO.output(power_key,GPIO.LOW)
	time.sleep(2)
	print('Good bye')

try:
    power_on(power_key)
    while True:
        command_input = raw_input('Please input the AT command,press Ctrl+C to exit:')
        ser.write((command_input+  '\r\n' ).encode())
        time.sleep(0.1)
        if ser.inWaiting():
            time.sleep(0.01)
            rec_buff = ser.read(ser.inWaiting())
        if rec_buff != '':
            print(rec_buff.decode())
            rec_buff = ''
except :
    ser.close()
    power_down(power_key)
    GPIO.cleanup()
