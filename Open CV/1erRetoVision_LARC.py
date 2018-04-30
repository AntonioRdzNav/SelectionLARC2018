import numpy as np
import cv2

lower_Green = np.array([45, 70, 70])
upper_Green = np.array([75, 255, 255])
lower_Blue = np.array([100, 70, 70])
upper_Blue = np.array([115, 255, 255])
kernel = np.ones((15,15), np.uint8)

def findSide(cx, cy):
    if(cx < testImage.shape[1]*(3.5/10)): #left side
        return "LEFT"
    elif(cx>testImage.shape[1]*(3.5/10) and cx<testImage.shape[1]*(6.5/10)):
        return "MIDDLE"
    elif(cx>testImage.shape[1]*(6.5/10)):
        return "RIGHT"

# Read Base Image
baseImage = cv2.imread('1er_Reto_Vision_Imagenes/1.jpeg');
baseImage = cv2.resize(baseImage, None, fx=1, fy=.71, interpolation=cv2.INTER_AREA);
hsvBaseImage = cv2.cvtColor(baseImage, cv2.COLOR_BGR2HSV)
maskGreen = cv2.inRange(hsvBaseImage, lower_Green, upper_Green)
maskGreen = cv2.erode(maskGreen, kernel, iterations = 1)
maskBlue = cv2.inRange(hsvBaseImage, lower_Blue, upper_Blue)
maskBlue = cv2.erode(maskBlue, kernel, iterations = 1)
greenContours, hierarchy1 = cv2.findContours(maskGreen.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
blueContours, hierarchy1 = cv2.findContours(maskBlue.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

greenTarget = greenContours[0];
blueTarget = blueContours[0];

# cv2.imshow('Output', hsvBaseImage)
# cv2.waitKey(0)
# cv2.imshow('Output', maskGreen)
# cv2.waitKey(0)
# cv2.imshow('Output', maskBlue)
# cv2.waitKey(0)
# cv2.drawContours(baseImage, targetGreen, -1, (0,255,0), 3)
# cv2.imshow('Output', baseImage)
# cv2.waitKey(0)
# cv2.drawContours(baseImage, targetBlue, -1, (255,0,0), 3)
# cv2.imshow('Output', baseImage)
# cv2.waitKey(0)
    
    
testImage = cv2.imread('1er_Reto_Vision_Imagenes/5.jpeg');
testImage = cv2.resize(testImage, None, fx=1, fy=.71, interpolation=cv2.INTER_AREA);
hsvTestImage = cv2.cvtColor(testImage, cv2.COLOR_BGR2HSV)
maskGreenTest = cv2.inRange(hsvTestImage, lower_Green, upper_Green)
maskGreenTest = cv2.erode(maskGreenTest, kernel, iterations = 1)
maskBlueTest = cv2.inRange(hsvTestImage, lower_Blue, upper_Blue)
maskBlueTest = cv2.erode(maskBlueTest, kernel, iterations = 1)
greenTestContours, hierarchy1 = cv2.findContours(maskGreenTest.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
blueTestContours, hierarchy1 = cv2.findContours(maskBlueTest.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)


isGreenBoat = False
isBlueBoat = False
for test in greenTestContours:
    matchGreen = cv2.matchShapes(greenTarget, test, 3, 0.0)
    if (matchGreen < 2):  
        greenBoat = test
        M = cv2.moments(test)
        cxGreen = int(M['m10'] / M['m00'])
        cyGreen = int(M['m01'] / M['m00'])         
        isGreenBoat = True
    
for test in blueTestContours:
    matchBlue = cv2.matchShapes(blueTarget, test, 3, 0.0)  
    if (matchBlue < 2):  
        blueBoat = test
        M = cv2.moments(test)
        cxBlue = int(M['m10'] / M['m00'])
        cyBlue = int(M['m01'] / M['m00'])           
        isBlueBoat = True

if isGreenBoat:
    cv2.drawContours(testImage, greenBoat, -1, (0,255,0), 3)
    cv2.imshow('Output', testImage)
    cv2.waitKey(0)
    print("Verde: \t" + str(isGreenBoat) + "\t|||\t" + findSide(cxGreen, cyGreen) + "\n")
if isBlueBoat:
    cv2.drawContours(testImage, blueBoat, -1, (255,0,0), 3)
    cv2.imshow('Output', testImage)
    cv2.waitKey(0)
    print("Blue: \t" + str(isBlueBoat) + "\t|||\t" + findSide(cxBlue, cyBlue) + "\n")
    
cv2.waitKey(0)    
cv2.destroyAllWindows()    