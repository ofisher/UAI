#!/usr/bin/env python
'''
Created on 26/07/2013

@author: oliver
'''

from numpy import *
import sys

import subprocess
import thread
from time import sleep
import argparse
import ConfigParser
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

def Transform(p,T):
    if len(p) == 3:
        p = toHomo(p)
    return dot(T,p)

def toHomo(p):
    return [p[0], p[1], p[2], 1]
def toCart(p):
    if len(p) == 4:
        return array([p[0]/p[3], p[1]/p[3], p[2]/p[3]])
    else:
        return array(p)


def alternateMarkerAndHome(d):
    while True:
        d.sendToUAI('home,20000')
        print 'home,20000'
        sys.stdout.flush()
        sleep(25)
        
        #wait until we have actually detected a marker
        while not d.hasMarkerBeenDetected:
            sleep(1)
        
        marker_pos_camera_space = [d.pixel_x, d.pixel_y, d.kinect_dist]
        marker_pos_robot_space = toCart(Transform(marker_pos_camera_space,T))
        toprint = 'move,20000,' + str(marker_pos_robot_space[0]) + ","
        toprint  += str(marker_pos_robot_space[1]) + ","
        toprint  += str(marker_pos_robot_space[2]) + ","
        toprint  += '1.84,2.5,0'
        print toprint
        d.sendToUAI(toprint)
        sleep(25)

def getDistance(a,b):
    sqaure = 0
    for a_val, b_val in zip(a,b):
        sqaure += (a_val - b_val)**2
    return math.sqrt(sqaure)


def trackRealTime(d):
    
    while not d.hasMarkerBeenDetected:
        print "waiting to detect marker"
        sleep(1)
    
    
    max_speed = 0.05 # meters per second
    min_time = 1 #min amount of time to allow for a movement
    while True:
        
        
        while get_millis_now() - d.timeLastMarkerSeen > 500:
            print "Marker not detected"
            sleep(1)
            
        
        marker_pos_camera_space = [d.pixel_x, d.pixel_y, d.kinect_dist]
        marker_pos_robot_space = toCart(Transform(marker_pos_camera_space,T))
        robot_pos = d.coords[:3]
        distance = getDistance(marker_pos_robot_space,robot_pos)
        print distance
        ms = int(distance * 1000/max_speed)
        if ms < min_time * 1000:
            ms = min_time * 1000
        print "ms", ms
        
        toprint = 'move,'+ str(ms)  +',' + str(marker_pos_robot_space[0]) + ","
        toprint  += str(marker_pos_robot_space[1]) + ","
        toprint  += str(marker_pos_robot_space[2]) + ","
        toprint  += '1.84,2.5,0'
        print toprint
        d.sendToUAI(toprint)
        sleep(float(ms)/1000)

T = []
with open("T_outliers_removed.txt") as f:
    for line in f.readlines():
        row = []
        for val in line.split(" "):
            row.append(float(val))
        T.append(row)
T = array(T)
#print T

d = DataPointAcquirer()
sleep(1)

parser = argparse.ArgumentParser()
parser.add_argument('-t','--track', help = "Attempt to track the marker in realtime", action="store_true")
args = parser.parse_args()
if args.track:
    trackRealTime(d)
else:
    alternateMarkerAndHome(d)



