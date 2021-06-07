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
ref = db.reference('server/saving-data/Other/2021-04-14/demoC')

print('Check if a certain value is present in a recorded dataset for ')
snapshot = ref.order_by_child('speedOverGround').equal_to(-2021).get()

#chnage these keys with these values to be blank
for key in snapshot:
    print(key)
    ref_dataset = ref.child(key)
    ref_dataset.update({
        'speedOverGround' : ''
    })

print('Checking again if there')
snapshot_eile = ref.order_by_child('speedOverGround').equal_to(-2021).get()

for key in snapshot_eile:
    print(key)