# import gmplot package 
import gmplot
from firebase_admin import db
import firebase_admin
from firebase_admin import credentials

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

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

for keyID in aoifeDrivesGps: 
    timeStamps.append( aoifeDrivesGps[keyID]['dateAndTime'] )
    longitudes.append( aoifeDrivesGps[keyID]['longitude'] )
    latitudes.append( aoifeDrivesGps[keyID]['latitude'] )
    altitudes.append( aoifeDrivesGps[keyID]['mslAltitude'] )
  
# GoogleMapPlotter return Map object 
# Pass the center latitude and 
# center longitude )
gmap = gmplot.GoogleMapPlotter(53.211785, -8.559224, 13) 

# scatter method of map object  
# scatter points on the google map 
gmap.scatter( latitudes, longitudes, '#FF0000', size = 1, marker = False ) 
  
# Plot method Draw a line in 
# between given coordinates 
gmap.plot( latitudes, longitudes, 'cornflowerblue', edge_width = 6) 

gmap.apikey = "AIzaSyBPj3jPxuid_qvbctaN6qu_mETdw-VCsMI"
# Pass the absolute path 
gmap.draw( "C:\\fyp_pc_work\\pictures\\map_loughrea.html" )
