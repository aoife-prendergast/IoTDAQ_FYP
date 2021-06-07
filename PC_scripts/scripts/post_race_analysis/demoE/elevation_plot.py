import gmplot
from firebase_admin import db
import firebase_admin
from firebase_admin import credentials

import numpy as np
import pandas as pd

#from mpl_toolkits.mplot3d.axes3d import *
import matplotlib.pyplot as plt
from matplotlib import cm

from scipy.interpolate import griddata

from mpl_toolkits import mplot3d

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

fig = plt.figure()
ax = plt.axes(projection='3d')

# Data for a three-dimensional line
ax.plot3D(longitudesF, latitudesF, altitudesF, 'gray')

# Data for three-dimensional scattered points
ax.scatter3D(longitudesF, latitudesF, altitudesF, c=altitudesF, cmap='rainbow')
ax.set_xlabel('Longitude')
ax.set_ylabel('Latitude')
ax.set_zlabel('Altitude')

plt.savefig('./pictures/CycleTest_3d_plot_line.png')
plt.show()