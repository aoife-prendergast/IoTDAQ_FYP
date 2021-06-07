import numpy as np 
import matplotlib.pyplot as plt
import pandas as pd

import gmplot
from firebase_admin import db
import firebase_admin
from firebase_admin import credentials

#########################################################################
# Firebase Realtime Database

cred = credentials.Certificate("C:/fyp/mqtt-trial-firebase-adminsdk-yrqgx-77140342e7.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://mqtt-trial-default-rtdb.firebaseio.com/'
})

#########################################################################

ref = db.reference("/server/saving-data/Pallas/2021-03-13/aoifeDrives/")
aoifeDrivesGps = ref.get()

#print(all_data)
timeStamps = []
longitudes = [] 
latitudes = [] 
altitudes = []
speeds = [] 

for keyID in aoifeDrivesGps: 
    timeStamps.append( aoifeDrivesGps[keyID]['dateAndTime'] )
    longitudes.append( aoifeDrivesGps[keyID]['longitude'] )
    latitudes.append( aoifeDrivesGps[keyID]['latitude'] )
    altitudes.append( aoifeDrivesGps[keyID]['mslAltitude'] )
    speeds.append( aoifeDrivesGps[keyID]['speedOverGround'])

plt.figure(figsize = (14, 12))
plt.title('Loughrea Drive Speed Heatmap')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.scatter(longitudes, latitudes, c = speeds)
cbar = plt.colorbar()
cbar.set_label('Speed')

plt.savefig('./pictures/loughrea_drive_speed_heatmap.png')
plt.show()
