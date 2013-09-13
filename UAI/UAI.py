'''
Created on 19/06/2013

@author: oliver
'''
#!/usr/bin/env python
import sys
from collections import namedtuple
import thread
import subprocess
import time
from Utils import *
from UR_TCP_Connector import *
import argparse
import ConfigParser
max_error_to_remaining_distance_ratio = 0.4 #the lower this number, the sooner the robot will adjust for changed targets

class UAI:
    def __init__(self, input_pipe, output_pipe, robot_ip, use_joint_space, output_coordinates = False):
        self.use_joint_space = use_joint_space
        self.commands = []
        self.connector = UR_TCP_Connector(robot_ip)
        self.connector_thread = thread.start_new_thread(self.connector.monitor_state,(self.newDataFromRobot, output_coordinates, output_pipe))
        self.current_target = None
        self.current_target_time = 0
        self.ep = 1e-6 #what distance is considered so small that it is the same as 0 (in meters)
        thread.start_new_thread(self.readInput,(input_pipe,))
        
    def moveHome(self, time = 0):
        print 'moving arm home'
        
        self.connector.moveHome(time)
    def readInput(self, pipe):
        Command = namedtuple('Command', ['type', 'start_time', 'end_time', 'values'])
        while True:
            new_commands = []
            line = pipe.readline().rstrip()
            if line == '':#ignore blank lines
                continue
            if line == 'exit':
                sys.exit()
            print 'input read', line
            command_strings = line.split(':')
            now_time = get_millis_now()
            last_command_time = now_time#used as the start time for self.commands
            for command_string in command_strings:
                command_sections = command_string.split(',')
                command_type = command_sections[0]
                start_time = last_command_time
                end_time = int(command_sections[1]) + now_time
                last_command_time = end_time
                if len(command_sections) > 2:
                    command_values = [float(x) for x in command_sections[2:]]
                else:
                    command_values = []
                command = Command(command_type,start_time, end_time,command_values)
                new_commands.append(command)
            self.commands = new_commands
    
    def getCurrentCommand(self):
        while len(self.commands) > 0:
            if self.commands[0].end_time < get_millis_now():
                print 'command', self.commands[0].type, ' expired'
                self.commands.pop(0)#current command is expired
            elif get_millis_now() > self.commands[0].start_time:
                return self.commands[0]
            else:
                return None#command not ready to start
        return None #no self.commands
    
    def executeCommand(self, command):
        if command == None:
            self.current_target = None
            return
        
        
        if command.type == 'move':
            if self.use_joint_space:
                self.connector.moveJoint(command.values, command.end_time)
            else:
                self.connector.moveCart(command.values, command.end_time )
            self.current_target = command.values
            self.current_target_time = command.end_time
        elif command.type == 'home':
            self.moveHome((command.end_time - get_millis_now())/1000)
            self.current_target = 'home'
            self.current_target_time = command.end_time
    
        
    
    
    def differenceBetweenTargets(self, target_a,target_b):
        difference = 0.0
        for a, b in zip(target_a[:3], target_b[:3]):
            difference += abs(a - b)
        #weight rotation difference less than movement difference 
        for a, b in zip(target_a[3:], target_b[3:]):
            difference += 0.1 * abs(a - b)
        return difference
    
    def newDataFromRobot(self, data):
        if self.current_target == None:
            #print 'no current target, executing command'
            self.executeCommand(self.getCurrentCommand())
        elif self.current_target == 'home':
            current_command = self.getCurrentCommand()
            if current_command is not None:
                if current_command.type != 'home' or get_millis_now() > self.current_target_time:
                    self.executeCommand(current_command)
        else:
            current_command = self.getCurrentCommand()
            if current_command is not None:
                if current_command.type == 'move':
                    if self.use_joint_space:
                        distance_to_target = self.differenceBetweenTargets(data.joint_positions, self.current_target)
                    else:
                        distance_to_target = self.differenceBetweenTargets(data.coordinates, self.current_target)
                    target_error = self.differenceBetweenTargets(current_command.values, self.current_target)
                    #print distance_to_target, target_error
                    if target_error > self.ep:
                        if distance_to_target == 0.0 or target_error/distance_to_target > max_error_to_remaining_distance_ratio:
                            print 'updating movement'
                            self.executeCommand(current_command)
                elif current_command.type == 'home':
                    self.executeCommand(current_command)
                    
if __name__=='__main__':
    
    config = ConfigParser.RawConfigParser()
    config.read('config.cfg')
    
    parser = argparse.ArgumentParser()
    parser.add_argument('-i','--ip', help = "IP address of Robot, default is " + config.get("U5","ip"))
    parser.add_argument('-c','--cart', help = "Use Cartesian coordinates, (default)", action="store_true")
    parser.add_argument('-j','--joint', help = "Use joint space", action="store_true")
    parser.add_argument('-p','--planner', help = "Program to run to output coordinates, if not given coordinates are accepted via standard in")
    parser.add_argument('-o','--coordinates', help = "Output the current coordinates continuously to standard out", action="store_true")
    
    args = parser.parse_args()
    
    if args.cart and args.joint:
        sys.stderr.write("Error: Cannot run in Cartesian coordinates and joint space at the same time\n")
        sys.stderr.write("Exiting...\n")
        sys.stderr.flush()
        sys.exit()
    
    if args.planner:
        cmd = args.planner.split()
        p = subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        input_pipe = p.stdout
        output_pipe = p.stdin
    else:
        output_pipe = None
        input_pipe = sys.stdin
    if args.ip:
        robot_ip = args.ip
    else:
        
        
        config.get("U5","ip")

        robot_ip = config.get("U5","ip")
    
    print "using_ip_address" , robot_ip
    
    uai = UAI(input_pipe, output_pipe, robot_ip, args.joint, args.coordinates)
    #for i in xrange(30):
    while True:
        time.sleep(1)