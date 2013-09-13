'''
Created on 19/06/2013

@author: oliver
'''
import sys
import time
print 'home,5000:move,20000,0.053,0.53,-0.023,0.22,1.25,2.02'
sys.stdout.flush()
time.sleep(7)
print 'move,13000,0.073,0.58,-0.020,0.22,1.25,2.02'
sys.stdout.flush()
time.sleep(100)
#tell the UAI to exit, otherwise it will run forever
print 'exit'