import paho.mqtt.client as mqtt 
import time
import datetime

# Import database module.
from firebase_admin import db
import numpy as np 
import firebase_admin
from firebase_admin import credentials

#########################################################################
def on_message(client, userdata, message):
    print()
    decodedMessage = str(message.payload.decode("utf-8"))
    print("message received ", decodedMessage)
    print("message topic=",message.topic)
    print("message qos=",message.qos)
    print("message retain flag=",message.retain)

    stringValues = np.array(decodedMessage.split(','))

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
# MQTT Interaction

broker_address="broker.emqx.io"

#create new instance
print("Creating a new instance...")
client = mqtt.Client(client_id="mqtt_UNIQUE_1234", clean_session=True, userdata=None, transport="tcp")      
client.on_message=on_message #attach function to callback
print("Connecting to a broker...")
client.connect(broker_address)  #connect to broker
client.loop_start()

#publish
#client.publish("testingGeec","geecTxDataWindows")

#subribing to a topic
print("Subscribing to a topic...")
client.subscribe("testingGeec")

#wait
time.sleep(3600) 
client.loop_stop()
