import pandas as pd
import xlrd

file_location = "C:/fyp/txdata/run1.xls"

book = xlrd.open_workbook(file_location)
sheet = book.sheet_by_name('run1')
rowsAsStrings = [[sheet.cell_value(r, 22) ] for r in range(sheet.nrows)]
print(len(rowsAsStrings))

print(rowsAsStrings[0])
print(rowsAsStrings[1])
print(rowsAsStrings[2])

for row in rowsAsStrings: 
    # measure the length of the string 
    length = len(row[0])
    #print(length)
    # send the mqtt message of appropraite length 
