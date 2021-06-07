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

for keyID in aoifeDrivesGps: 
    timeStamps.append( aoifeDrivesGps[keyID]['dateAndTime'] )
    longitudes.append( aoifeDrivesGps[keyID]['longitude'] )
    latitudes.append( aoifeDrivesGps[keyID]['latitude'] )
    altitudes.append( aoifeDrivesGps[keyID]['mslAltitude'] )
    
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
        
  
# GoogleMapPlotter return Map object 
# Pass the center latitude and 
# center longitude )
gmap = gmplot.GoogleMapPlotter(53.211785, -8.559224, 13) 

# scatter method of map object  
# scatter points on the google map 
gmap.scatter( latitudesF, longitudesF, '#FF0000', size = 1, marker = False ) 
  
# Plot method Draw a line in 
# between given coordinates 
gmap.plot( latitudesF, longitudesF, 'cornflowerblue', edge_width = 6) 

gmap.apikey = "AIzaSyBPj3jPxuid_qvbctaN6qu_mETdw-VCsMI"
# Pass the absolute path 
gmap.draw( "C:\\fyp_pc_work\\pictures\\map_loughrea_cycle.html" )
