import numpy as np 
import matplotlib.pyplot as plt
import pandas as pd

import gmplot
from firebase_admin import db
import firebase_admin
from firebase_admin import credentials

#########################################################################
# Firebase Realtime Database

cred = credentials.Certificate("C:/fyp_pc_work/app-geec-firebase-adminsdk-md6ee-ac784a797b.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://app-geec-default-rtdb.europe-west1.firebasedatabase.app/'
})

#########################################################################

ref = db.reference("/saving-data/Other/2021-05-05/cycleTest2/")
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

timeStampsF = []
longitudesF = [] 
latitudesF = [] 
altitudesF = []
speedsF = [] 

for i in timeStamps:
    if i == "":
        timeStamps.remove(i)
    else: 
        timeStampsF.append((float(i)))
        
for i in longitudes:
    if i == "":
        longitudes.remove(i)
    else: 
        longitudesF.append((float(i)))
        
for i in latitudes:
    if i == "":
        latitudes.remove(i)
    else: 
        latitudesF.append((float(i)))
        
for i in altitudes:
    if i == "":
        altitudes.remove(i)
    else: 
        altitudesF.append((float(i)))
        
for i in speeds:
    if i == "":
        speeds.remove(i)
    else: 
        speedsF.append((float(i)))

plt.figure(figsize = (14, 12))
plt.title('Cycle Speed Heatmap')
plt.xlabel('Longitude')
plt.ylabel('Latitude')
plt.scatter(longitudesF, latitudesF, c = speedsF)
cbar = plt.colorbar()
cbar.set_label('Speed')

plt.savefig("C:\\fyp_pc_work\\pictures\\cycle_heatmap.png" )
plt.show()
