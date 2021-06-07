#   Script to udate the firebase realtime database without telemetry
#   GPS test
#   Aoife Prendergast 
#   5th of May 2021

import paho.mqtt.client as mqtt 
import time
import datetime

# Import database module.
from firebase_admin import db
import numpy as np 
import firebase_admin
from firebase_admin import credentials

import pandas as pd
import xlrd

#########################################################################
def on_message(client, userdata, message):
    try: 
        decodedMessage = str(message.payload.decode("utf-8"))
        sections = np.array(decodedMessage.split('#'))   
    
        print("\nmessage received ", decodedMessage)
        print("message topic=",message.topic)
        print("message qos=",message.qos)
        print("message retain flag=",message.retain)
        
        # Blank values are left blank 
        # Numerical values are not converted to floats 
        try:
            accelerometerData = np.array(sections[3].split(','))
        except: 
            accelerometerData = np.zeros(6,str)
        
        try:
            print(section[4])
            sq = np.array(sections[4].split(','))
        except: 
            sq = np.zeros(20,str)
        
        try:
            ueInfo = np.array(sections[5].split(','))
        except: 
            ueInfo = np.zeros(20,str)
        
        try:
            gpsInfo = np.array(sections[0].split(','))
        except: 
            gpsInfo = np.zeros(20,str)
        
        try:
            sensors = np.array(sections[2].split(','))
        except: 
            sensors = np.zeros(20,str)
        
        try:
            connectionStatus = sections[1][-1:]
        except: 
            connectionStatus = '' 
            
        now = time.time() - start_time
        print(now)
        
        try:
            if("GSM" in ueInfo[0]):
                print("inside if")
                testingRound.push().set({
                        'timeSinceStart': sensors[0],
                        'lapCount': sensors[1],
                        'lapTime': sensors[2],
                        'rpm': sensors[3], 
                        'roadSpeed': sensors[4],
                        'distanceTravelled': sensors[5],
                        'motorVoltage': sensors[6],
                        'motorCurrent': sensors[7],
                        'batteryVoltage': sensors[8],
                        'batteryCurrent': sensors[9],
                        'accelerator': sensors[10],
                        'tempOne': sensors[11],
                        'tempTwo': sensors[12],
                        'tempThree': sensors[13],
                        'ax': accelerometerData[0],
                        'ay': accelerometerData[1],
                        'az': accelerometerData[2],
                        'gx': accelerometerData[3], 
                        'gy': accelerometerData[4],
                        'gz': accelerometerData[5],
                        'runStatus': gpsInfo[0],
                        'fixStatus': gpsInfo[1],
                        'dateAndTime': gpsInfo[2],
                        'latitude': gpsInfo[3], 
                        'longitude': gpsInfo[4],
                        'mslAltitude': gpsInfo[5],
                        'speedOverGround': gpsInfo[6],
                        'courseOverGround': gpsInfo[7],
                        'fixMode': gpsInfo[8],
                        'reserved1': gpsInfo[9],
                        'HDOP': gpsInfo[10],
                        'PDOP': gpsInfo[11],
                        'VDOP': gpsInfo[12],
                        'reserved2': gpsInfo[13], 
                        'gnssInView': gpsInfo[14],
                        'reserevd3': gpsInfo[15],
                        'HPA': gpsInfo[16],
                        'VPA': gpsInfo[17],
                        'connectionStatus': connectionStatus,
                        'systemMode': ueInfo[0],
                        'opMode': ueInfo[1],
                        'MCC-MNC': ueInfo[2],
                        'LAC': ueInfo[3],
                        'CellID': ueInfo[4],
                        'ARFCN': ueInfo[5],
                        'RxLev': ueInfo[6],
                        'TrackLOAdjust': ueInfo[7],
                        'C1-C2': ueInfo[8],
                        'recByPCTime': now,
                        'rssi': sq[0], 
                        'sq': sq[1],
                })
            # if data was "published" while no connection 
            elif("NO SERVICE" in ueInfo[0]):
                testingRound.push().set({
                        'timeSinceStart': sensors[0],
                        'lapCount': sensors[1],
                        'lapTime': sensors[2],
                        'rpm': sensors[3], 
                        'roadSpeed': sensors[4],
                        'distanceTravelled': sensors[5],
                        'motorVoltage': sensors[6],
                        'motorCurrent': sensors[7],
                        'batteryVoltage': sensors[8],
                        'batteryCurrent': sensors[9],
                        'accelerator': sensors[10],
                        'tempOne': sensors[11],
                        'tempTwo': sensors[12],
                        'tempThree': sensors[13],
                        'ax': accelerometerData[0],
                        'ay': accelerometerData[1],
                        'az': accelerometerData[2],
                        'gx': accelerometerData[3], 
                        'gy': accelerometerData[4],
                        'gz': accelerometerData[5],
                        'runStatus': gpsInfo[0],
                        'fixStatus': gpsInfo[1],
                        'dateAndTime': gpsInfo[2],
                        'latitude': gpsInfo[3], 
                        'longitude': gpsInfo[4],
                        'mslAltitude': gpsInfo[5],
                        'speedOverGround': gpsInfo[6],
                        'courseOverGround': gpsInfo[7],
                        'fixMode': gpsInfo[8],
                        'reserved1': gpsInfo[9],
                        'HDOP': gpsInfo[10],
                        'PDOP': gpsInfo[11],
                        'VDOP': gpsInfo[12],
                        'reserved2': gpsInfo[13], 
                        'gnssInView': gpsInfo[14],
                        'reserevd3': gpsInfo[15],
                        'HPA': gpsInfo[16],
                        'VPA': gpsInfo[17],
                        'connectionStatus': connectionStatus,
                        'systemMode': ueInfo[0],
                        'opMode': ueInfo[1],
                        'recByPCTime': now,
                        'rssi': sq[0], 
                        'sq': sq[1],
                })
            elif("LTE" in ueInfo[0]):
                testingRound.push().set({
                        'timeSinceStart': sensors[0],
                        'lapCount': sensors[1],
                        'lapTime': sensors[2],
                        'rpm': sensors[3], 
                        'roadSpeed': sensors[4],
                        'distanceTravelled': sensors[5],
                        'motorVoltage': sensors[6],
                        'motorCurrent': sensors[7],
                        'batteryVoltage': sensors[8],
                        'batteryCurrent': sensors[9],
                        'accelerator': sensors[10],
                        'tempOne': sensors[11],
                        'tempTwo': sensors[12],
                        'tempThree': sensors[13],
                        'ax': accelerometerData[0],
                        'ay': accelerometerData[1],
                        'az': accelerometerData[2],
                        'gx': accelerometerData[3], 
                        'gy': accelerometerData[4],
                        'gz': accelerometerData[5],
                        'runStatus': gpsInfo[0],
                        'fixStatus': gpsInfo[1],
                        'dateAndTime': gpsInfo[2],
                        'latitude': gpsInfo[3], 
                        'longitude': gpsInfo[4],
                        'mslAltitude': gpsInfo[5],
                        'speedOverGround': gpsInfo[6],
                        'courseOverGround': gpsInfo[7],
                        'fixMode': gpsInfo[8],
                        'reserved1': gpsInfo[9],
                        'HDOP': gpsInfo[10],
                        'PDOP': gpsInfo[11],
                        'VDOP': gpsInfo[12],
                        'reserved2': gpsInfo[13], 
                        'gnssInView': gpsInfo[14],
                        'reserevd3': gpsInfo[15],
                        'HPA': gpsInfo[16],
                        'VPA': gpsInfo[17],
                        'connectionStatus': connectionStatus,
                        'systemMode': ueInfo[0],
                        'opMode': ueInfo[1],
                        'MCC-MNC': ueInfo[2],
                        'TAC': ueInfo[3],
                        'SCellID': ueInfo[4],
                        'PCellId': ueInfo[5],
                        'FreqBand': ueInfo[6],
                        'earfcn': ueInfo[7],
                        'dlbw': ueInfo[8],
                        'ulbw': ueInfo[9],
                        'RSRQ': ueInfo[10],
                        'RSRP': ueInfo[11],
                        'RSSI': ueInfo[12], 
                        'RSSNR': ueInfo[13],
                        'recByPCTime': now,
                        'rssi': sq[0], 
                        'sq': sq[1],
                })
        except: 
            print("Could not parse the message that was recieved")    
    except: 
        print("Recieved bad message")

#########################################################################
# Taking the user input 

# Track 
tracks = ["Pallas","Dangan","BenchDemo","Dyno","Other"]

for t in tracks: 
    print("\t" + str(tracks.index(t)) + " " + t)

trackNo = input("\nEnter track number: ") 
track = tracks[int(trackNo)]

if(trackNo == tracks.index("Other")): 
    track = input("\nWhat is the name of the new track: ")

# Date 
today = datetime.date.today()
print("\nToday's date: " + str(today) )

# Attempt Number of Name of Test
testName = input("\nName of this test: ") 

#########################################################################
# Firebase Realtime Database

cred = credentials.Certificate("C:/fyp_pc_work/app-geec-firebase-adminsdk-md6ee-ac784a797b.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://app-geec-default-rtdb.europe-west1.firebasedatabase.app/'
})

dbRef = track + '/' + str(today)

# Get a database reference to our blog.
ref = db.reference('saving-data/' + dbRef)

testingRound = ref.child(testName)

#########################################################################

# MQTT Interaction

broker_address="test.mosquitto.org"

#create new instance
print("\n\tCreating a new instance...")
client = mqtt.Client(client_id="mqtt_UNIQUE_1234", clean_session=True, userdata=None, transport="tcp")      
client.on_message=on_message #attach function to callback
print("\n\tConnecting to a broker...")
client.connect(broker_address)  #connect to broker

userInput = ""
while(userInput == ""): 
    userInput = input("\nPlease give me any indication to go! ") 
    
client.loop_start()

#publish
#client.publish("testingGeec","geecTxDataWindows")

#subribing to a topic
print("Subscribing to a topic...")
start_time = time.time()
client.subscribe("testingGeec")

#wait 3 hours
time.sleep(7200) 
client.loop_stop()