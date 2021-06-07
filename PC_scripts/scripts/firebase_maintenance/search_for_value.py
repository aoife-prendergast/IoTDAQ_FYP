#   Script to delete a large child node in realtime database
#   Aoife Prendergast 
#   30th of April 2021

from firebase import firebase

# Import database module.
from firebase_admin import db
import numpy as np 
import firebase_admin
from firebase_admin import credentials

#########################################################################
#########################################################################

# Firebase Realtime Database

cred = credentials.Certificate("C:/fyp_pc_work/mqtt-trial-firebase-adminsdk-yrqgx-77140342e7.json")
firebase_admin.initialize_app(cred, {
    'databaseURL': 'https://mqtt-trial-default-rtdb.firebaseio.com/'
})

# Get a database reference to our blog.
ref = db.reference('server/saving-data/Other/2021-05-01/deleteMe2')

print('Check if a unsire value is present in a recorded dataset for ')
snapshot = ref.order_by_child('dateAndTime').equal_to('20210501010449.000').get()

#chnage these keys with these values to be blank
for key, val in snapshot.items():
    print('The {0} node\'s dateAndTime is {1}'.format(key, val))
    
print('Check if a certain value is present in a recorded dataset for ')
snapshot2 = ref.order_by_child('dateAndTime').equal_to('20210501010744.000').get()

#chnage these keys with these values to be blank
for key, val in snapshot2.items():
    print('The {0} node\'s dateAndTime is {1}'.format(key, val))