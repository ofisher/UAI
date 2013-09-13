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
import TransformationMatrixFinder
from Utils import get_millis_now
class DataPointAcquirer:
    def __init__(self):
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
        self.p.stdin.write(str + "\n")
        self.p.stdin.flush()
def getQuitter(d):
    def quitter(signal, frame):
        print "good bye"
        d.isQuitting = True
        sleep(0.3)
        sys.exit()
    return quitter
def getMarkerCoords(d):
    if d.hasMarkerBeenDetected:
        to_write = ""
        to_write += str(d.marker_id) + "," + str(d.kinect_dist) + "," + str(d.pixel_x,) + "," + str(d.pixel_y)
        to_write += "," + str(d.tx) + "," + str(d.ty) + "," + str(d.tz) + ","
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
        to_write += str(d.joints[5])
        return to_write
    else:
        return "NO COORDS RECEIVED FROM ROBOT"



def doManualConfig(f,d):
    marker_coords = ""
    robot_coords = ""
    f2 = open("output2.txt",'wb')
    while not d.isQuitting:
        val = raw_input("Enter to see current value, s to save: ")
        print "val received", val
        if val == "m":
            marker_coords = getMarkerCoords(d)
            print "marker_coords", marker_coords
        if val == "r":
            robot_coords = getRobotCoords(d)
            print "robot_coords", robot_coords
            
        if val =="b":
            marker_coords = getMarkerCoords(d)
            print "marker_coords", marker_coords
            robot_coords = getRobotCoords(d)
            print "robot_coords", robot_coords

            
        if val == "s":
            print "writing...", marker_coords + robot_coords
            f.write(marker_coords + robot_coords)
        if val == "a":
            print "trying an auto save"
            saveIfMarkerSeen(f2,d)
        if val == 'c':
            print "doing calculations manual saves"
            f.close()    
            TransformationMatrixFinder.run("output.txt")
            print "doing calculations auto saves"
            f2.close()    
            TransformationMatrixFinder.run("output2.txt")
            

def saveIfMarkerSeen(f,d):
    max_time_since_last_marker_update = 300
    if get_millis_now() - d.timeLastMarkerSeen > max_time_since_last_marker_update:
        print "marker has not been seen for", get_millis_now() - d.timeLastMarkerSeen
        
        return #marker has not been seen recently, don't do anything
    if d.kinect_dist < 0:
        print "no valid depth data (too close?)"
        return
    marker_coords = getMarkerCoords(d)
    robot_coords = getRobotCoords(d)
    print "marker_coords", marker_coords
    print "robot_coords", robot_coords
    f.writeline(marker_coords + robot_coords)
    
    
def interpolate(a,b,fraction_a):
    result = []
    for a_val, b_val in zip(a,b):
        result.append(fraction_a * a_val + (1 - fraction_a) * b_val)
    return result


def sendArmToPoint(p,d, ms = 5000):
    str_to_send = 'move,' + str(ms)
    for val in p:
        str_to_send += ',' + str(val)
    d.sendToUAI(str_to_send)
    sleep((ms/1000) + 2.5)
def doAutoConfig(f,d):
    print "doing auto config"
    while d.p is None:
        print "waiting for UAI process to spawn"
        sleep(1)
    d.sendToUAI("home,5000")
    sleep(5)


    points = [(-0.069962073126, -0.634611496613, 0.529683058227, 0.618754523955, 2.92335555786,-0.304642731012),
        (0.0332192549893, -0.250149241943, 0.404117986194,0.618754523955, 2.92335555786,-0.304642731012),
        (0.360676276275, -0.560893526358, 0.525384432707, 0.618754523955, 2.92335555786,-0.304642731012),
        (0.206564467056, -0.691813860126, 0.426688383806, 0.618754523955, 2.92335555786,-0.304642731012),
        (0.133259201853, -0.480879118384, 0.364228358003, 0.618754523955, 2.92335555786,-0.304642731012),
        (0.128337198084, -0.373165692493, 0.640037211585, 0.618754523955, 2.92335555786,-0.304642731012),
        (0.392017449111, -0.303887322325, 0.555465452073, 0.618754523955, 2.92335555786,-0.304642731012),
        (0.200448894242, -0.274100676343, 0.714583263429, 0.618754523955, 2.92335555786,-0.304642731012)]    
    
    sendArmToPoint(points[0],d)
    previous_point = points[0]
    saveIfMarkerSeen(f,d)
    for point in points[1:]:
        fraction_new_point = 0.0
        num_steps = 1#the number of samples it takes per point specified
        for step in xrange(1,num_steps + 1):
            fraction_new_point = float(step)/num_steps
            interpolated_point = interpolate(point,previous_point,fraction_new_point)
            sendArmToPoint(interpolated_point,d, 3500)
            saveIfMarkerSeen(f,d)
        previous_point = point
    f.close()    
    TransformationMatrixFinder.run("output.txt")
    
    #d.sendToUAI('home,5000:move,20000,0.053,0.53,-0.023,0.22,1.25,2.02')
    #sleep(20)
parser = argparse.ArgumentParser()
parser.add_argument('-a','--auto', help = "Do automatic config", action="store_true")



args = parser.parse_args()

d = DataPointAcquirer()

signal.signal(signal.SIGINT, getQuitter(d))





with open("output.txt","wb") as f:
    
    if args.auto:
        doAutoConfig(f,d)
    else:
        doManualConfig(f,d)
getQuitter(d)(None,None)
