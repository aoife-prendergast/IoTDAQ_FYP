import firebase_admin
from firebase_admin import credentials
# Import database module.
from firebase_admin import db
import numpy as np 

cred = credentials.Certificate("C:/fyp/mqtt-trial-firebase-adminsdk-yrqgx-77140342e7.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://mqtt-trial-default-rtdb.firebaseio.com/'
})

# Get a database reference to our blog.
ref = db.reference('server/saving-data/pallasTestingNov20')

testingRound = ref.child('run1')

recievedString = "1,0,0,-8.57,0,0,-0.66,0.3,30.47,0.39,515"

stringValues = np.array(recievedString.split(','))

# conerting to array of floats 
# using np.astype 
floatValues = stringValues.astype(np.float) 

testingRound.set({
    recievedString[0]: {
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
    },
})