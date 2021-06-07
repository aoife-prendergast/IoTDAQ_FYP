#   Script to delete a large child node in realtime database
#   Aoife Prendergast 
#   18th of April 2021

from firebase import firebase

#########################################################################
#########################################################################

fb = firebase.FirebaseApplication('https://mqtt-trial-default-rtdb.firebaseio.com/')
result = fb.delete('server/saving-data/Dyno/2021-03-13/deleteLater', None)
print(result)