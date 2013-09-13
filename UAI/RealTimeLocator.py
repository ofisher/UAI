#!/usr/bin/env python
'''
Created on 15/07/2013

@author: oliver
'''
import subprocess
import thread
from time import sleep
import signal
import sys
import argparse
from numpy import *
from Utils import *
from ErrorInterpolator import *
class DataPointAcquirer:
    def __init__(self):
        
        self.tx = 0
        self.ty = 0
        self.kinect_dist =0;
        self.hasRobotCoordsBeenReceived = False
        self.hasMarkerBeenDetected = False
        self.isQuitting = False
        self.p = None
        self.readRobotLocationThread = thread.start_new_thread(self.readRobotLocation, ())
        self.readMarkerLocationThread = thread.start_new_thread(self.readMarkerLocation, ())
        self.timeLastMarkerSeen = 0
        
    def readMarkerLocation(self):
        
        cmd = ['../kinectMarkerDetector/Debug/kinectMarkerDetector']
        p = subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        while not self.isQuitting:
            line = p.stdout.readline().rstrip()
            #print line
            tokens = line.rstrip().split(",")
            if len(tokens) < 7:
                #print "len(tokens)", len(tokens)
                #print line
                continue
            if tokens[0] != 'marker':
                print "tokens[0]" , tokens[0] 
                continue
            #print "valid input"
            self.hasMarkerBeenDetected = True
            self.marker_id = int(tokens[1])
            self.kinect_dist = int(tokens[2])
            self.pixel_x = float(tokens[3])
            self.pixel_y = float(tokens[4])
            

            self.tx = float(tokens[5])
            self.ty = float(tokens[6])
            self.tz = float(tokens[7])
            self.timeLastMarkerSeen = get_millis_now()
            #print self.timeLastMarkerSeen
        print "NOW TO KILL THREAD"
        p.terminate() 
    def readRobotLocation(self):
        print "launching UAI"
        cmd = ['python','UAI.py', '-o', '-c']
        p = subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT, stdin=subprocess.PIPE)
        self.p = p
        
        print "p set", p
        while not self.isQuitting:
            line = p.stdout.readline().rstrip()
            #print line
            tokens = line.split(",")
            #print len(tokens)
            if len(tokens) >= 13:
                if tokens[0] != 'coords':
                    continue
                coords = []
                joints = []
                for i in xrange(6):
                    coords.append(float(tokens[i + 1]))
                    
                    #print tokens[i + 7]
                    
                    joints.append(float(tokens[i + 7]))
                #print "coords", coords
                #print "joints", joints
                self.coords = coords
                self.joints = joints
                self.hasRobotCoordsBeenReceived = True
        p.terminate() 
        print "DONE"
    def sendToUAI(self,str):
        #print str >> self.p.stdin
        
        if self.p == None:
            print "Error UAI not running"
            return
        self.p.stdin.write(str + "\n")
        self.p.stdin.flush()
def getQuitter(d):
    def quitter(signal, frame):
        print "good bye"
        d.isQuitting = True
        killProcesses("python UAI.py")
        killProcesses("kinectMarkerDetector/Debug/kinectMarkerDetector")
        sleep(0.3)
        sys.exit()
    return quitter
def getMarkerCoords(d):
    if d.hasMarkerBeenDetected:
        to_write = ""
        to_write += str(d.marker_id) + "," + str(d.kinect_dist) + "," + str(d.pixel_x,) + "," + str(d.pixel_y)
        to_write += "," + str(d.pixel_x) + "," + str(d.pixel_y) + "," + str(d.tz) + ","
        return to_write
    else:
        return "MARKER NOT YET DETECTED"
def getRobotCoords(d):
    if d.hasRobotCoordsBeenReceived:
        to_write = ""
        for i in xrange(6):
            to_write += str(d.coords[i]) + ","
        for i in xrange(5):
            to_write += str(d.joints[i]) + ","
        to_write += str(d.joints[5]) + "\n"
        return to_write
    else:
        return "NO COORDS RECEIVED FROM ROBOT"

    

def printLocation(d,T,f,c):
    #load transformation matrix
    if not d.hasMarkerBeenDetected:
        print "market not yet detected"
        return
    if get_millis_now() - d.timeLastMarkerSeen > 200:
        print "marker not seen recently"
        return
    #get marker coordinates
    marker_pos_camera_space = [d.pixel_x, d.pixel_y, d.kinect_dist]
    #convert to robot coordinates
    marker_pos_robot_space = toCart(Transform(marker_pos_camera_space,T))
    robot_pos = d.coords[:3]
    
    marker_pos_robot_space_corrected = c.correctCoords(d.pixel_x,d.pixel_y,marker_pos_robot_space[0],marker_pos_robot_space[1],marker_pos_robot_space[2])
    
    print "Error: ",
    for truth, predicted ,corrected in zip(robot_pos,marker_pos_robot_space, marker_pos_robot_space_corrected):
        print corrected - truth, "amount better =", abs(predicted - truth) - abs(corrected - truth)
        f.write(str(corrected - truth) + ",")
    
    f.write(str(d.pixel_x) + "," + str(d.pixel_y) + "\n")
    print d.pixel_x, d.pixel_y
    
    
    
def continiouslyPrintLocation(d,c):
    T = loadTransformationMatrix("T_outliers_removed.txt")
    T = array(T)
    print "T = ", T
    while True:
        printLocation(d,T,c)
        sleep(1)
        

    
    
def moveForHeatMap(d,c):
    T = loadTransformationMatrix("T_outliers_removed.txt")
    T = array(T)
    
    num_rows = 10
    num_cols = 10
    time_per_move = 700
    time_to_stop = 300
    long_move_time = 5000
    depth = 550
    x_start = 70
    x_end = 570
    y_start = 70
    y_end = 410
    
    while d.p == None:
        sleep(0.1)
        print "waiting for UAI"
    with open("error points", 'wb') as f:
        for col in xrange(num_cols):
            for row in xrange(num_rows):
                
                time_for_this_move = None
                if row == 0:
                    time_for_this_move = long_move_time
                else:
                    time_for_this_move = time_per_move
                
                x = x_start + col * (x_end - x_start)/num_cols
                y = y_start + row * (y_end - y_start)/num_rows
                
                
                
                robot_target = toCart(Transform([x,y,depth],T))
                
                #print "moving to", robot_target
                command = "move," + str(time_for_this_move) + "," + str(robot_target[0]) + "," + str(robot_target[1]) + "," + str(robot_target[2]) + ",0.618754523955, 2.92335555786,-0.304642731012"
                print command
                d.sendToUAI(command)
                sleep(time_for_this_move/1000.0 + time_to_stop/1000.0)
                printLocation(d,T,f,c)
            
killProcesses("python UAI.py")
killProcesses("kinectMarkerDetector/Debug/kinectMarkerDetector")
sleep(0.5)
parser = argparse.ArgumentParser()
parser.add_argument('-a','--auto', help = "Get data automatically for heat map", action="store_true")
parser.add_argument('-c','--correct', help = "Correct using error map previously generated", action="store_true")
args = parser.parse_args()
d = DataPointAcquirer()

signal.signal(signal.SIGINT, getQuitter(d))


c = CoordinateCorrector(args.correct)
if args.auto:
    moveForHeatMap(d,c)
else:
    continiouslyPrintLocation(d,c)
getQuitter(d)(None,None)
