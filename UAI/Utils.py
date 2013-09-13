'''
Created on 19/06/2013

@author: oliver
'''
import datetime
import subprocess
import signal
import os
from numpy import *
def get_millis_now():
    delta = datetime.datetime.now() - datetime.datetime.utcfromtimestamp(0)
    return delta.total_seconds() * 1000
    
def loadTransformationMatrix(filename):
    T = []
    with open(filename) as f:
        for line in f.readlines():
            row = []
            for val in line.split(" "):
                row.append(float(val))
            T.append(row)
    return T
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
#this kills processes with to_kill in the name
#this simply uses ps -aux and then checks each line to see if it contains to kill
#if it does it extracts the PID and kills that PID
#so if you give it an empty string, or something that will match every line
#you may end up killing every process on your PC (until it kills itself anyway)
def killProcesses(to_kill):
    cmd = ["ps", "-aux"]
    p = subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    for line in iter(p.stdout.readline, b''):
        if to_kill in line:
            tokens = line.split(" ")
            for token in tokens[1:]:
                if token != "":
                    print "killing " + to_kill + " PID " + token
                    os.kill(int(token),signal.SIGINT)
                    break
    