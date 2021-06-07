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
    
fig = plt.figure()
ax = plt.axes(projection='3d')

# Data for a three-dimensional line
ax.plot3D(longitudes, latitudes, altitudes, 'gray')

# Data for three-dimensional scattered points
ax.scatter3D(longitudes, latitudes, altitudes, c=altitudes, cmap='rainbow')
ax.set_xlabel('Longitude')
ax.set_ylabel('Latitude')
ax.set_zlabel('Altitude')

plt.savefig('./pictures/3d_plot_line.png')
plt.show()