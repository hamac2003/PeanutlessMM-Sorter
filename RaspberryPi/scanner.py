# Author: Harrison McIntyre
# Last Updated: 11.04.2020
# Contact: hamac2003@gmail.com



### References / Credits

# Below are links to some of the example code and/or libraries that I integrated into my project.

"""
[Raspberry Pi - Arduino Serial Communication](https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/)

[Raspberry Pi Camera Capture](https://www.pyimagesearch.com/2015/03/30/accessing-the-raspberry-pi-camera-with-opencv-and-python/)

[Opencv Pixel Counting](https://stackoverflow.com/questions/45836214/opencv-python-count-pixels)

[First Non-Zero Function](https://stackoverflow.com/questions/47269390/numpy-how-to-find-first-non-zero-value-in-every-column-of-a-numpy-array)

[Convex Hull / STL Creation](https://stackoverflow.com/questions/61480649/triangulate-2d-shape-to-get-stl)

[Convex Hull Volume Calculation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.ConvexHull.html)

[HX711 - Raspberry Pi Interface](https://github.com/tatobari/hx711py)

[Python Radians-Degrees Conversion](https://stackoverflow.com/questions/9875964/how-can-i-convert-radians-to-degrees-with-python)

[Contour Normalization](https://medium.com/analytics-vidhya/tutorial-how-to-scale-and-rotate-contours-in-opencv-using-python-f48be59c35a2)
"""


import time
import cv2
import numpy as np
import math
import serial
import time
import sys
from picamera.array import PiRGBArray
from picamera import PiCamera
from scipy.spatial import ConvexHull
from scipy.spatial import Delaunay
from stl import mesh
from hx711py.hx711 import HX711

# If density is greater than this threshold, it's probably solid
densityThreshold = 1.38

# Known weight in milligrams
knownWeight = 5670

# Number of measurements to take when weighing
numOfAverages = 5
referenceUnit = 1

# Color detection pixel cutoff val
cutoffVal = 6000

# M&M Count
countFile = "mm_count.txt"

# Scale calibration factor
calibrationFile = "calibrationFactor.txt"

# Scan image directory
calibrationImagesDir = "calibrationImages"

# 3d model directory
stl_dir = "stls"

# CSV Delimeter
delimeter = ","

# CSV Data file
dataFile = "log.csv"

# Region of interest to crop to
ROI = [500,300,780,555]

# The angle of the laser relative to the camera
laserAngle = 64

# How many slices to scan
numSlices = 50

# Pixel calibration factor (15.118 pixels per millimeter)
ppmm = 96/6.35

# Scale Setup
hx = HX711(5, 6)
hx.set_reading_format("MSB", "MSB")
hx.set_reference_unit(referenceUnit)
hx.reset()

# Weight conversion factor (read from file)
conversionFactor = 5670/34900
f = open(calibrationFile, "r")
conversionFactor = float(f.read())
f.close()

# Initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (1280, 720)
camera.framerate = 10
camera.exposure_mode = 'verylong'
camera.saturation = 100
rawCapture = PiRGBArray(camera, size=(1280, 720))

# Allow the camera to warmup
time.sleep(0.1)

# Serial communication with arduino
stepper = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
stepper.flush()


def sendSerial(val):
    data = str(val) + "\n"

    stepper.write(data.encode('utf-8'))
    time.sleep(0.1)

def first_nonzero(arr, axis, invalid_val=-1):
    mask = arr!=0
    return np.where(mask.any(axis=axis), mask.argmax(axis=axis), invalid_val)    

# Color detection hsv values
possibleColors = {"red/orange/brown":[np.array([1,0,1]),np.array([255,100,255])],
                "yellow":[np.array([0,10,10]),np.array([200,255,255])],
                "green":[np.array([5,150,0]),np.array([255,255,255])],
                "blue":[np.array([200,50,0]),np.array([255,255,255])],}

# M&M color calibration ranges
calibrations = {"red/orange/brown":[10,255],
                "yellow":[100,255],
                "green":[100,255],
                "blue":[100,255],
                "brown":[10,255]}

time.sleep(3)  

# Main loop
while True:
    action = input("Awaiting Input (S for Scan, C for Calibrate, Q for Quit): ")
    
    if action.upper() == "Q":
        sys.exit()
    elif action.upper() == "C":
        # Scale Calibration Routine
        
        hx.power_up()
        sendSerial("C")
        tare = 0
        for i in range(numOfAverages):
            tare += hx.read_long()
        tare /= numOfAverages
        
        input("Tare complete, place known weight now")
        
        temp = 0
        for i in range(numOfAverages):
            temp += hx.read_long() - tare
        temp /= numOfAverages
        
        newConversion = knownWeight/temp
        print(f"Raw Scale Units: {temp}")
        switch = input(f"New Calibration: {newConversion}\n Old Calibration{conversionFactor}\n Change to new calibration factor? (y/n)")
        sendSerial("C_D")
        hx.power_down()
        if switch.upper() == "Y":
            conversionFactor = newConversion
            f = open(calibrationFile, "w")
            f.write(str(conversionFactor))
            f.close()
        else:
            pass
        continue
        
    else:
        pass

    # Intialize Variables
    scanComplete = False
    pauseCount = 10
    color = None
    lowerRange = 100
    upperRange = 255
    slices = []
    colorScores = {}
    laserRange = []
    csv = ""
    f = open(countFile, "r")
    mm_count = int(f.read())
    f.close()
    
    sendSerial("L_W")

    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        
        # Flush out delayed frames
        if pauseCount > 0:
            pauseCount -= 1
            rawCapture.truncate(0)
            continue
        # Detect what color M&M is being scanned
        if color is None:
            image = frame.array
            cv2.imwrite(calibrationImagesDir + "/" + str(mm_count) + ".PNG", image)
            imageSmaller = image[ROI[1]:ROI[3],ROI[0]:ROI[2]]
            cv2.imshow("Calibration", imageSmaller)
            for col, limits in possibleColors.items():
                tempImage = cv2.inRange(imageSmaller,limits[0],limits[1])
                colorScores[col] = cv2.countNonZero(tempImage)

            sendSerial("L_B")
            sortedScores = sorted(colorScores.values())
            highestScore = sortedScores[-1]
            if highestScore < cutoffVal:
                color = "brown"
            else:
                color = list(colorScores.keys())[list(colorScores.values()).index(highestScore)]
            
            laserRange = calibrations[color]
            print(f"The M&M is the color {color}. And it has a calibration of {laserRange}")
            
        else:
            
            # Scanning Routine
            
            image = frame.array

            imageSmaller = image[ROI[1]:ROI[3],ROI[0]:ROI[2]]

            gray = cv2.cvtColor(imageSmaller, cv2.COLOR_BGR2GRAY)

            mask = cv2.inRange(gray, laserRange[0], laserRange[1])

            res = cv2.bitwise_and(imageSmaller,imageSmaller,mask=mask)
            cv2.imshow("mask_1", mask)

            kernel = np.ones((5,5),np.uint8)
            mask = cv2.erode(mask,kernel,iterations = 1)
            cv2.imshow("mask", mask)
            
            
            h,w = np.shape(mask)
            finalShape = np.zeros((h,w))
            points = []
            xyz_coords = []
            line = first_nonzero(mask, axis=1, invalid_val=-1)
            
            for i, val in enumerate(line):
                if val == -1:
                    finalShape[i] = 0
                    continue
                finalShape[i][val] = mask[i][val]
                points.append([[val,i]])
            
            points = np.asarray(points)
            cv2.drawContours(imageSmaller, points, -1, (0, 0, 255), 2)


            newPoints = []
            
            cnt_norm = points - [int(w/2), 0]

            for i, point in enumerate(cnt_norm):
                x = point[0][0]/math.cos(math.radians(laserAngle))
                x = x/ppmm
                y = point[0][1]/ppmm

                z = 0

                xyz_coords.append([x,y,z])
            slices.append(xyz_coords)
            print(len(slices))
            if len(slices) == numSlices:
                scanComplete = True

            cv2.imshow("Mask", mask)
            cv2.imshow("Regular", imageSmaller)
            
            if scanComplete:
                finalData = []
                a = 0
                x = []
                y = []
                z = []
                for i,Slice in enumerate(slices):
                    a += 360/numSlices
                    rads = math.radians(a)
                    print(rads)
                    tempSlice = []
                    for point in Slice:
                        z_2 = 1*math.cos(rads) - point[0]*math.sin(rads)
                        x_2 = 1*math.sin(rads) + point[0]*math.cos(rads)
                        y_2 = point[1]
                        newPoint = [x_2,y_2,z_2]
                        finalData.append(newPoint)
                        x.append(x_2)
                        y.append(y_2)
                        z.append(z_2)

                slices = []
                
                rawCapture.truncate(0)
                break
            for i in range(int(200/numSlices)):
                sendSerial("I")

        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            sys.exit()

    mm_count+=1
    
    # Write the current M&M count to file
    f = open(countFile,"w")
    f.write(str(mm_count))
    f.close()
    
    csv += str(mm_count) + delimeter

    finalData = np.asarray(finalData)

    # Make a convex hull of our set of 3d points
    faces = ConvexHull(finalData)

    print("Volume: " + str(faces.volume))

    csv += str(faces.volume) + delimeter

    ms = mesh.Mesh(np.zeros(len(faces.simplices), dtype=mesh.Mesh.dtype))

    # STL generation
    for i, f in enumerate(faces.simplices):
        for j in range(3):
            ms.vectors[i][j] = finalData[f[j],:]
    
    # Save STL to file
    ms.save(stl_dir + "/" + str(mm_count) + ".stl")
    
    # Tare the scale
    hx.power_up()
    tare = 0
    for i in range(numOfAverages):
        tare += hx.read_long()
    tare /= numOfAverages
    time.sleep(1)

    sendSerial("S")

    time.sleep(3)

    # Weigh the candy
    milligrams = 0
    for i in range(numOfAverages):
        milligrams += (hx.read_long()-tare) * conversionFactor
    milligrams /= numOfAverages
    print(f"Weight (mg): {milligrams}")
    hx.power_down()
    density = milligrams/faces.volume
    print(f"Density (mg/mm^3): {density}")
    
    csv += str(milligrams) + delimeter
    csv += str(density) + delimeter
    csv += color + delimeter
    
    # Sorting Routine
    
    if density >= densityThreshold:
        sendSerial("N")
        csv += "S" + delimeter
    else:
        sendSerial("P")
        csv += "P" + delimeter
    time.sleep(4)

    peanut = None

    groundTruth = input("Has peanut? (Y/N): ")

    if groundTruth.upper() == "Y":
        peanut = True
    elif groundTruth.upper() == "N":
        peanut = False

    csv += str(peanut)
    
    # Write data to CSV
    f = open(dataFile, "a")
    
    f.write(csv + "\n")

    f.close()