#!/usr/bin/python

import RPi.GPIO as GPIO
import serial
import time
import xlrd

def power_on(power_key):
    print('SIM7070G is starting:')
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(power_key,GPIO.OUT)
    time.sleep(0.1)
    GPIO.output(power_key,GPIO.HIGH)
    time.sleep(2)
    GPIO.output(power_key,GPIO.LOW)
    time.sleep(2)
    ser.flushInput()
    ser.flushOutput()
    print('SIM7070G is ready')
    return 

def power_down(power_key):
	print('SIM7070G is loging off:')
	GPIO.output(power_key,GPIO.HIGH)
	time.sleep(2)
	GPIO.output(power_key,GPIO.LOW)
	time.sleep(2)
	print('Good bye')
	return 	

def send_at(command,back,timeout):
    rec_buff = ''
    ser.write((command+'\r\n').encode())
    
    if(timeout != 0):
        time.sleep(timeout)
        if ser.inWaiting():
            time.sleep(0.01)
            rec_buff = ser.read(ser.inWaiting())
    else: 
        waiting = True 
        while(waiting == True):
            time.sleep(0.1)
            if ser.inWaiting():
                time.sleep(0.2)
                rec_buff = ser.read(ser.inWaiting())
                waiting = False
        
    if rec_buff != '':
        if back not in rec_buff.decode():
            print(command + ' back:\t' + rec_buff.decode())
        else:
            print(rec_buff.decode())
        return rec_buff
    else:
        print(command + ' no responce')
        return "No response"
    return "Doesn't get here"

def send_message(message):
    if(message[0].isdigit()):
        print('Attempting to send message: '+ message)
        
        pub_AT_cmd = 'AT+SMPUB=\"testingGeec\",' + str(len(message)) + ',2,0'
        send_at(pub_AT_cmd,'OK',0)
        
        ser.write(message.encode())
        time.sleep(2)
        print('Sent message successfully!')
        
        return 	
    
ser = serial.Serial('/dev/ttyS0',9600)
ser.flushInput()

power_key = 4
rec_buff = ''

location = "/home/pi/Desktop/run1.xls"
book = xlrd.open_workbook(location) 
sheet = book.sheet_by_name('run1')
rowsAsStrings = [[sheet.cell_value(r,22)]for r in range(sheet.nrows)]

try:
    power_on(power_key)
    print('Wait for signal...')
    time.sleep(10)
    
    at = ''
    while('OK' not in at):
        print('Waiting to get an OK response from an AT command...')
        at = send_at('AT', 'OK', 1)
        time.sleep(1)
        
    # Signal Quality Report 
    send_at('AT+CSQ','OK',1)
    
    # Enquiring UE system information 
    send_at('AT+CPSI?','OK',1)
    
    # Network registration status
    send_at('AT+CGREG?','+CGREG: 0,1',0.5)
    
    send_at('AT+CNACT=0,1','OK',1)
    
    send_at('AT+CACID=0', 'OK',1)
    
    # Configuring MQTT connection
    send_at('AT+SMCONF=\"URL\",broker.emqx.io,1883','OK',1)
    send_at('AT+SMCONF=\"CLIENTID\",mqttx_546aaee8','OK',1) 
    send_at('AT+SMCONF=\"KEEPTIME\",60','OK',1)
    send_at('AT+SMCONF?','OK',1)
    send_at('AT+SMCONN','OK',5)
    
    for row  in rowsAsStrings: 
        send_message(str(row[0]))

    send_at('AT+SMDISC','OK',1)
    send_at('AT+CNACT=0,0', 'OK', 1)
    power_down(power_key)

except:
    if ser != None:
        ser.close()
        power_down(power_key)
        GPIO.cleanup()
        ser = None 

if ser != None:
		ser.close()
		GPIO.cleanup()