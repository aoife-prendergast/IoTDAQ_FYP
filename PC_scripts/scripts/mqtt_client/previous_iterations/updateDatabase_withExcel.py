#   Script to udate the firebase realtime database without telemetry
#   Aoife Prendergast 
#   16th of February 2021

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
def message_to_database(message):
    print(message)
    if(message[0].isdigit()):

        stringValues = np.array(message.split(','))

        # conerting to array of floats 
        # using np.astype 
        floatValues = stringValues.astype(np.float) 

        testingRound.push().set({
                'time': floatValues[0],
                'lapCount': floatValues[1],
                'lapTime': floatValues[2],
                'wRPM': floatValues[3],
                'speed': floatValues[4], 
                'distance': floatValues[5],
                'motorVoltage': floatValues[6],
                'motorCurrent': floatValues[7],
                'batteryVoltage': floatValues[8],
                'batteryCurrent': floatValues[9],
                'accelerator': floatValues[10],
    })

#########################################################################
# Taking the user input 

# Track 
tracks = ["Pallas","Dangan","BenchDemo","Dyno","Other"]

for t in tracks: 
    print( str(tracks.index(t)) + " " + t)

trackNo = input("Enter track number: ") 
track = tracks[int(trackNo)]

if(trackNo == tracks.index("Other")): 
    track = input("What is the name of the new track: ")

# Date 
today = datetime.date.today()
print(today)

# Attempt Number of Name of Test
testName = input("Name of this test: ") 

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

location = "C:/fyp_pc_work/txdata/run1.xls"
book = xlrd.open_workbook(location) 
sheet = book.sheet_by_name('run1')
rowsAsStrings = [[sheet.cell_value(r,22)]for r in range(sheet.nrows)]

for row  in rowsAsStrings: 
    message_to_database(str(row[0]))
    time.sleep(1)