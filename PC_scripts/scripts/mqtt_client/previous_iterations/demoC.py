#   Script to udate the firebase realtime database without telemetry
#   GPS test
#   Aoife Prendergast 
#   14th of April 2021

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
    print()
    decodedMessage = str(message.payload.decode("utf-8"))
    # print("message received ", decodedMessage)
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)
    
    sections = np.array(decodedMessage.split('#'))
        
    accelerometerData = np.array(sections[3].split(','))
    accelerometerDataF = accelerometerData.astype(np.float) 
    
    ueInfo = np.array(sections[4].split(','))
    for index, s in enumerate(ueInfo): 
        if s == '': 
            ueInfo[index] = '-2021'
    
    gpsInfo = np.array(sections[0].split(','))
    for index, s in enumerate(gpsInfo): 
        if s == '': 
            gpsInfo[index] = '-2021'
    gpsInfoF = gpsInfo.astype(np.float) 
    
    sensors = np.array(sections[2].split(','))
    for index, s in enumerate(sensors): 
        if s == '': 
            sensors[index] = '-2021'
    sensorsF = sensors.astype(np.float) 
    
    connectionStatus = sections[1][-1:]
    
    if("GSM" in ueInfo[0]):
        testingRound.push().set({
                'timeSineStart': sensorsF[0],
                'lapCount': sensorsF[1],
                'lapTime': sensorsF[2],
                'rpm': sensorsF[3], 
                'roadSpeed': sensorsF[4],
                'distanceTravelled': sensorsF[5],
                'motorVoltage': sensorsF[6],
                'motorCurrent': sensorsF[7],
                'batteryVoltage': sensorsF[8],
                'batteryCurrent': sensorsF[9],
                'accelerator': sensorsF[10],
                'tempOne': sensorsF[11],
                'tempTwo': sensorsF[12],
                'tempThree': sensorsF[13],
                'ax': accelerometerDataF[0],
                'ay': accelerometerDataF[1],
                'az': accelerometerDataF[2],
                'gx': accelerometerDataF[3], 
                'gy': accelerometerDataF[4],
                'gz': accelerometerDataF[5],
                'runStatus': gpsInfoF[0],
                'fixStatus': gpsInfoF[1],
                'dateAndTime': gpsInfoF[2],
                'latitude': gpsInfoF[3], 
                'longitude': gpsInfoF[4],
                'mslAltitude': gpsInfoF[5],
                'speedOverGround': gpsInfoF[6],
                'courseOverGround': gpsInfoF[7],
                'fixMode': gpsInfoF[8],
                'reserved1': gpsInfoF[9],
                'HDOP': gpsInfoF[10],
                'PDOP': gpsInfoF[11],
                'VDOP': gpsInfoF[12],
                'reserved2': gpsInfoF[13], 
                'gnssInView': gpsInfoF[14],
                'reserevd3': gpsInfoF[15],
                'HPA': gpsInfoF[16],
                'VPA': gpsInfoF[17],
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
        })
    # if data was "published" while no connection 
    elif("NO SERVICE" in ueInfo[0]):
        testingRound.push().set({
                'timeSineStart': sensorsF[0],
                'lapCount': sensorsF[1],
                'lapTime': sensorsF[2],
                'rpm': sensorsF[3], 
                'roadSpeed': sensorsF[4],
                'distanceTravelled': sensorsF[5],
                'motorVoltage': sensorsF[6],
                'motorCurrent': sensorsF[7],
                'batteryVoltage': sensorsF[8],
                'batteryCurrent': sensorsF[9],
                'accelerator': sensorsF[10],
                'tempOne': sensorsF[11],
                'tempTwo': sensorsF[12],
                'tempThree': sensorsF[13],
                'ax': accelerometerDataF[0],
                'ay': accelerometerDataF[1],
                'az': accelerometerDataF[2],
                'gx': accelerometerDataF[3], 
                'gy': accelerometerDataF[4],
                'gz': accelerometerDataF[5],
                'runStatus': gpsInfoF[0],
                'fixStatus': gpsInfoF[1],
                'dateAndTime': gpsInfoF[2],
                'latitude': gpsInfoF[3], 
                'longitude': gpsInfoF[4],
                'mslAltitude': gpsInfoF[5],
                'speedOverGround': gpsInfoF[6],
                'courseOverGround': gpsInfoF[7],
                'fixMode': gpsInfoF[8],
                'reserved1': gpsInfoF[9],
                'HDOP': gpsInfoF[10],
                'PDOP': gpsInfoF[11],
                'VDOP': gpsInfoF[12],
                'reserved2': gpsInfoF[13], 
                'gnssInView': gpsInfoF[14],
                'reserevd3': gpsInfoF[15],
                'HPA': gpsInfoF[16],
                'VPA': gpsInfoF[17],
                'connectionStatus': connectionStatus,
                'systemMode': ueInfo[0],
                'opMode': ueInfo[1],
        })
    elif("LTE" in ueInfo[0]):
        testingRound.push().set({
                'timeSineStart': sensorsF[0],
                'lapCount': sensorsF[1],
                'lapTime': sensorsF[2],
                'rpm': sensorsF[3], 
                'roadSpeed': sensorsF[4],
                'distanceTravelled': sensorsF[5],
                'motorVoltage': sensorsF[6],
                'motorCurrent': sensorsF[7],
                'batteryVoltage': sensorsF[8],
                'batteryCurrent': sensorsF[9],
                'accelerator': sensorsF[10],
                'tempOne': sensorsF[11],
                'tempTwo': sensorsF[12],
                'tempThree': sensorsF[13],
                'ax': accelerometerDataF[0],
                'ay': accelerometerDataF[1],
                'az': accelerometerDataF[2],
                'gx': accelerometerDataF[3], 
                'gy': accelerometerDataF[4],
                'gz': accelerometerDataF[5],
                'runStatus': gpsInfoF[0],
                'fixStatus': gpsInfoF[1],
                'dateAndTime': gpsInfoF[2],
                'latitude': gpsInfoF[3], 
                'longitude': gpsInfoF[4],
                'mslAltitude': gpsInfoF[5],
                'speedOverGround': gpsInfoF[6],
                'courseOverGround': gpsInfoF[7],
                'fixMode': gpsInfoF[8],
                'reserved1': gpsInfoF[9],
                'HDOP': gpsInfoF[10],
                'PDOP': gpsInfoF[11],
                'VDOP': gpsInfoF[12],
                'reserved2': gpsInfoF[13], 
                'gnssInView': gpsInfoF[14],
                'reserevd3': gpsInfoF[15],
                'HPA': gpsInfoF[16],
                'VPA': gpsInfoF[17],
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
        })
    # Shouldnt get inside the else? 
    else:
        testingRound.push().set({
                'timeSineStart': sensorsF[0],
                'lapCount': sensorsF[1],
                'lapTime': sensorsF[2],
                'rpm': sensorsF[3], 
                'roadSpeed': sensorsF[4],
                'distanceTravelled': sensorsF[5],
                'motorVoltage': sensorsF[6],
                'motorCurrent': sensorsF[7],
                'batteryVoltage': sensorsF[8],
                'batteryCurrent': sensorsF[9],
                'accelerator': sensorsF[10],
                'tempOne': sensorsF[11],
                'tempTwo': sensorsF[12],
                'tempThree': sensorsF[13],
                'ax': accelerometerDataF[0],
                'ay': accelerometerDataF[1],
                'az': accelerometerDataF[2],
                'gx': accelerometerDataF[3], 
                'gy': accelerometerDataF[4],
                'gz': accelerometerDataF[5],
                'runStatus': gpsInfoF[0],
                'fixStatus': gpsInfoF[1],
                'dateAndTime': gpsInfoF[2],
                'latitude': gpsInfoF[3], 
                'longitude': gpsInfoF[4],
                'mslAltitude': gpsInfoF[5],
                'speedOverGround': gpsInfoF[6],
                'courseOverGround': gpsInfoF[7],
                'fixMode': gpsInfoF[8],
                'reserved1': gpsInfoF[9],
                'HDOP': gpsInfoF[10],
                'PDOP': gpsInfoF[11],
                'VDOP': gpsInfoF[12],
                'reserved2': gpsInfoF[13], 
                'gnssInView': gpsInfoF[14],
                'reserevd3': gpsInfoF[15],
                'HPA': gpsInfoF[16],
                'VPA': gpsInfoF[17],
                'connectionStatus': connectionStatus,
        })

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

cred = credentials.Certificate("C:/fyp/mqtt-trial-firebase-adminsdk-yrqgx-77140342e7.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://mqtt-trial-default-rtdb.firebaseio.com/'
})

dbRef = track + '/' + str(today)

# Get a database reference to our blog.
ref = db.reference('server/saving-data/' + dbRef)

testingRound = ref.child(testName)

#########################################################################

# MQTT Interaction

broker_address="broker.emqx.io"

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
client.subscribe("testingGeec")

#wait 3 hours
time.sleep(3600) 
client.loop_stop()