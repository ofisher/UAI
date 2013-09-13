'''
Created on 26/06/2013

@author: oliver
'''
import sys
import time
print 'home,5000:move,20000,1.5,-2.2,-1.65,-1.2,-0.7,-0.6'
sys.stdout.flush()
time.sleep(7)
print 'home,5000:move,13000,1.6,-2.2,-1.65,-1.2,-0.7,-0.6'
sys.stdout.flush()
time.sleep(100)
#tell the UAI to exit, otherwise it will run forever
print 'exit'