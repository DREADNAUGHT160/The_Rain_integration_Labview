import math
import numpy as np

def transformPoints(alpha, x, y):
    R_z = np.array([[math.cos(alpha),-math.sin(alpha)],
                    [math.sin(alpha), math.cos(alpha)]])
    x, y = np.round(R_z.dot([x, y]),5)
    
    return x, y

def calculateCg(actor):
    cg_x = np.mean(np.array([min(actor.onlineXVehData), max(actor.onlineXVehData)]))
    cg_y = np.mean(np.array([min(actor.onlineYVehData), max(actor.onlineYVehData)]))
    
    return cg_x, cg_y

def searchForText(fileContent, text):
    rowData = []
    lineWithText = ''
    for row in fileContent.readlines():
        rowData.append(row)
    for item in rowData:
        try: 
            item.index(text)
            lineWithText = item
        except:
            pass

    return lineWithText

def appendDataOfFile(fileContent):
    rowData = []
    for row in fileContent.readlines():
        rowData.append(row)

    return rowData   