import numpy as np
import cv2
                #BLUE         # GREEN       # RED
colors = [(237, 177, 30), (84, 175, 26), (27, 13, 252)]
range = 10

espadas=0
arcos=0 
escudos=0
colorTypes = ([0, 0, 0], [0, 0, 0], [0, 0, 0])

def currentColor(BRG):
    color = -1
    index = 0
    for c in colors:
        if(BRG[0]>=c[0]-range and BRG[0]<=c[0]+range):
            if(BRG[1]>=c[1]-range and BRG[1]<=c[1]+range):
                if(BRG[2]>=c[2]-range and BRG[2]<=c[2]+range):
                    return index
        index+=1
    return color

def selectColor(index, type):
    colorTypes[type][index]+=1
                    

# Read Base Image
baseImage = cv2.imread('3er_Reto_Vision_Imagenes/Test1.png');
# Resize Base Image
baseImage = cv2.resize(baseImage, None, fx=.5, fy=.5, interpolation=cv2.INTER_AREA);
# baseImage = cv2.medianBlur(baseImage, 7)
# Filtrate and get contours of base image
grayBaseImage = cv2.cvtColor(baseImage, cv2.COLOR_BGR2GRAY)
ret1,threshBase = cv2.threshold(grayBaseImage, 250, 255, cv2.THRESH_BINARY)
# baseEdged = cv2.Canny(threshBase, 30, 200)
baseContours, hierarchy1 = cv2.findContours(threshBase.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
baseContours.pop(0)
    
# Read Test Image
testImage = cv2.imread('3er_Reto_Vision_Imagenes/Test4.png');
# Resize Test Image
testImage = cv2.resize(testImage, None, fx=.5, fy=.5, interpolation=cv2.INTER_AREA);
# testImage = cv2.medianBlur(testImage, 7)
# coloredImage = cv2.cvtColor(testImage, cv2.COLOR_BGR2HSV)
# Filtrate and get contours of base image
grayTestImage = cv2.cvtColor(testImage, cv2.COLOR_BGR2GRAY)
ret2,threshTest = cv2.threshold(grayTestImage, 250, 255, cv2.THRESH_BINARY)
# testEdged = cv2.Canny(threshTest, 30, 200)
testContours, hierarchy2 = cv2.findContours(threshTest.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
testContours.pop(0)
    
# target[0] -> espada
# target[1] -> arcos
# target[2] -> escudos
closest_contour = []
lowerMatch = []
for test in testContours:
    matchedOptions = []
    index = 0
    minValue = 1000
    for base in baseContours:
        # Iterate through each contour in the target image and 
        # use cv2.matchShapes to compare contour shapes
        match = cv2.matchShapes(base, test, 3, 0.0)
        # If the match value is less than 0.15 we
        matchedOptions.append(test)        
        if (match < .75):
            lowerMatch.append(match)            
            if(match < minValue):
                rightIndex = index
                minValue = match
        index += 1
    if len(lowerMatch) > 0:
        M = cv2.moments(test)
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00']) 
        color = currentColor(testImage[cy, cx]) 
        if rightIndex == 0:
            espadas+=1
        elif rightIndex == 1:
            arcos+=1
        elif rightIndex == 2:
            escudos+=1  
        selectColor(color, rightIndex)
        closest_contour.append(matchedOptions[rightIndex])  
     
for draws in closest_contour:
    cv2.drawContours(testImage, draws, -1, (0,0,0), 3)
    cv2.imshow('Output', testImage)
    cv2.waitKey(0)

print("Numero total de objetos: \t" + str(espadas+arcos+escudos) + "\n")
print("Numero de Escudos: \t" + str(escudos) + ":\t")
print("Azules:" + str(colorTypes[2][0]) + ", \tVerdes:" + str(colorTypes[2][1]) + ", \tRojos:" + str(colorTypes[2][2])+ "\n")
print("Numero de Arcos: \t" + str(arcos) + ":\t")
print("Azules:" + str(colorTypes[1][0]) + ", \tVerdes:" + str(colorTypes[1][1]) + ", \tRojos:" + str(colorTypes[1][2])+ "\n")
print("Numero de Espadas: \t" + str(espadas) + ":\t")
print("Azules:" + str(colorTypes[0][0]) + ", \tVerdes:" + str(colorTypes[0][1]) + ", \tRojos:" + str(colorTypes[0][2])+ "\n")
    
cv2.waitKey(0)    
cv2.destroyAllWindows()    