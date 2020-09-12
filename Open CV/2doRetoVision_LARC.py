import numpy as np
import cv2

minArea = 8000
filtratedGreenContours = []
filtratedBlueContours = []
filtratedRedContours = []
allFiltratedContours = []

lower_Green = np.array([34, 55, 55])
upper_Green = np.array([90, 255, 255])
lower_Blue = np.array([95, 65, 65])
upper_Blue = np.array([130, 255, 255])
lower_Red = np.array([130, 65, 65])
upper_Red = np.array([255, 255, 255])
kernel = np.ones((3,3), np.uint8)
kernel2 = np.ones((11,11), np.uint8)

# Read Base Image
baseImage = cv2.imread('2do_Reto_Vision_Imagenes/8.jpeg');
baseImage = cv2.resize(baseImage, None, fx=1, fy=.71, interpolation=cv2.INTER_AREA);
hsvBaseImage = cv2.cvtColor(baseImage, cv2.COLOR_BGR2HSV)
maskGreen = cv2.inRange(hsvBaseImage, lower_Green, upper_Green)
maskGreen = cv2.erode(maskGreen, kernel, iterations = 1)
maskBlue = cv2.inRange(hsvBaseImage, lower_Blue, upper_Blue)
maskBlue = cv2.erode(maskBlue, kernel, iterations = 1)
maskRed = cv2.inRange(hsvBaseImage, lower_Red, upper_Red)
maskRed = cv2.erode(maskRed, kernel, iterations = 1)
maskRed = cv2.dilate(maskRed, kernel2, iterations = 1)
wholeStack = cv2.bitwise_or(maskBlue, maskGreen)
wholeStack = cv2.bitwise_or(wholeStack, maskRed)
greenContours, hierarchy1 = cv2.findContours(maskGreen.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
blueContours, hierarchy1 = cv2.findContours(maskBlue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
redContours, hierarchy1 = cv2.findContours(maskRed.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

for test in greenContours:
    if(cv2.contourArea(test) > minArea):
        filtratedGreenContours.append(test)
for test in blueContours: 
    if(cv2.contourArea(test) > minArea):
        filtratedBlueContours.append(test)
for test in redContours:    
    if(cv2.contourArea(test) > minArea):
        filtratedRedContours.append(test)
        
def determineColorByIndex(index):
    green = len(filtratedGreenContours)
    blue = len(filtratedBlueContours)
    if index<=green:
        return "GREEN"
    elif index>green and index<=green+blue:
        return "BLUE"
    elif index>green+blue:
        return "RED"
def getIndexMin(tupla):
    mincY = baseImage.shape[0]
    for t in tupla:
        if t[2] < minCY:
            minCY = t[2]
    return minCY

# for test in filtratedGreenContours:
#     cv2.drawContours(baseImage, test, -1, (0,255,0), 3)
#     cv2.imshow('Output', baseImage)
#     cv2.waitKey(0)
# #     print(cv2.contourArea(test))     
# for test in filtratedBlueContours:
#     cv2.drawContours(baseImage, test, -1, (255,0,0), 3)
#     cv2.imshow('Output', baseImage)
#     cv2.waitKey(0)
# #     print(cv2.contourArea(test))     
# for test in filtratedRedContours:
#     cv2.drawContours(baseImage, test, -1, (0,0,255), 3)
#     cv2.imshow('Output', baseImage)
#     cv2.waitKey(0)
# #     print(cv2.contourArea(test))    

allFiltratedContours += filtratedGreenContours
allFiltratedContours += filtratedBlueContours
allFiltratedContours += filtratedRedContours
topCY1 = baseImage.shape[0]
topCY2 = baseImage.shape[0]
index = 0
for test1 in allFiltratedContours:
    M = cv2.moments(test1)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00']) 
    if cy < topCY1:
        topCY1 = cy
        topCX1 = cx        
        indexTopStack1 = index
        top1 = test1
    index+=1
index = 0
for test2 in allFiltratedContours:
    M = cv2.moments(test2)
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00']) 
    if cy<topCY2 and (cy != topCY1):
        topCY2 = cy
        topCX2 = cx 
        indexTopStack2 = index
        top2 = test2
    index+=1
    
# print(len(allFiltratedContours))
# print(indexTopStack1)
# print(indexTopStack2)

colorTopStack1 = determineColorByIndex(indexTopStack1)
colorTopStack2 = determineColorByIndex(indexTopStack2)

cv2.imshow('Output', baseImage)
cv2.waitKey(0)

if(topCX1 < topCX2):
    print("LEFT: \t" + colorTopStack1 + "\t|||\tRIGHT: \t" + colorTopStack2 + "\n")
else:
    print("LEFT: \t" + colorTopStack2 + "\t|||\tRIGHT: \t" + colorTopStack1 + "\n")
    
cv2.waitKey(0)    
cv2.destroyAllWindows()    