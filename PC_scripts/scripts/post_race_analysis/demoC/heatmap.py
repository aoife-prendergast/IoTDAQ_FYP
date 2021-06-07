import numpy as np 
import matplotlib.pyplot as plt
import pandas as pd

import gmplot
from firebase_admin import db
import firebase_admin
from firebase_admin import credentials

#########################################################################
# Firebase Realtime Database

cred = credentials.Certificate("C:/fyp_pc_work/app-geec-firebase-adminsdk-md6ee-277f1f1e83.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://app-geec-default-rtdb.europe-west1.firebasedatabase.app/'
})

#########################################################################

ref = db.reference("/saving-data/Other/2021-04-14/demoC/")
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
