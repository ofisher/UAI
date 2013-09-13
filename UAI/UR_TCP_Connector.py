'''
Created on 20/06/2013

@author: oliver
'''

port = 30002
import math
import socket
import struct
from collections import namedtuple
from CheckLegal import isLegalPos
import sys

class UR_TCP_Connector:
    
    def __init__(self, robot_ip):
        self.connect(robot_ip)
        self.sendScript()
        self.ur_server = URServer()
        thread.start_new_thread(self.ur_server.startServer,())
        
    
    def connect(self, robot_ip):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((robot_ip, port))
    def sendPopUp(self, title = 'Test title', message = 'Hello'):
        command = 'popup(''+message+'', ''+title+'', False, False)\n'
        #command = 'popup('Test Message', 'Popup', False, False)\n'
        print command
        self.sock.send(command)
        self.sock.close()
    def moveCart(self,points, time = 0 ):
        command = 'movej(get_inverse_kin(p['
        
        for i, p in enumerate(points):
            command += str(p)
            if i + 1 < len(points):
                command += ', '
            
        command += '])'
        
        if time > 0:
            command += ', t = '+ str(time)
        
        command += ')\n'
        print command
        
        self.ur_server.set_target(points, time)
        
        #self.sock.send(command)
    def moveJoint(self,points, time = 0, blend_radius = 0):
        command = 'movej(['
        for i, p in enumerate(points):
            command += str(p)
            if i + 1 < len(points):
                command += ', '
        command += ']'
        if blend_radius > 0:
            command +=', r = ' + str(blend_radius)
        if time > 0:
            command +=', t = ' + str(time)
        
        command +=')\n'
        
        print command
        #self.sock.send(command)
    def moveHome(self, time = 0):
        j = [0,-math.pi/2, 0, -math.pi/2,0,0]
        self.moveJoint(j, time = time)
        
    def sendScript(self, file_name = 'URScript.py'):
        script = ''
        for line in open(file_name).readlines():
            script += line
        script += '\n' #must always end in a new line or it doesn't process the final line
        print script
        self.sock.send(script)
    
    def readLengthOfPacket(self,data):
        length_string = data[:4][::-1]
        length = struct.unpack('<L',length_string)
        return length[0]
    
    def printPacket(self, data, max_length = 1000):
        for c in data:
            print ord(c)
    def stop(self):
        self.sock.send("stop(1)\n")
        
    def monitor_state(self, cb, output_coordinates, output_pipe = sys.stdout):
        #the format of the data received from the robot can be found here 
        #http://wiki03.lynero.net/Technical/DataStreamFromURController
        #http://wiki03.lynero.net/Technical/PrimaryAndSecondaryClientInterface
        ROBOT_MODE_DATA = 0
        JOINT_DATA = 1
        TOOL_DATA = 2
        CARTESIAN_INFO = 4
        data = ''
        
        
        #test_data = [0, 0, 1, 209, 16, 0, 0, 0, 29, 0, 0, 0, 0, 0, 0, 5, 7, 246, 1, 1, 1, 0, 0, 0, 0, 0, 63, 240, 0, 0, 0, 0, 0, 0, 0, 0, 0, 251, 1, 64, 1, 78, 244, 77, 189, 249, 149, 64, 1, 78, 247, 89, 95, 104, 85, 0, 0, 0, 0, 0, 0, 0, 0, 188, 220, 97, 3, 66, 62, 0, 0, 66, 0, 102, 103, 66, 99, 153, 154, 253, 191, 246, 74, 170, 216, 242, 29, 102, 191, 246, 74, 178, 44, 92, 72, 137, 0, 0, 0, 0, 0, 0, 0, 0, 192, 1, 106, 78, 66, 63, 153, 154, 66, 4, 204, 205, 66, 104, 0, 0, 253, 63, 253, 49, 202, 91, 202, 8, 64, 63, 253, 49, 210, 233, 51, 16, 35, 0, 0, 0, 0, 0, 0, 0, 0, 191, 155, 135, 34, 66, 62, 0, 0, 65, 249, 153, 154, 66, 100, 0, 0, 253, 191, 220, 115, 204, 104, 205, 239, 254, 191, 220, 118, 68, 109, 49, 34, 158, 0, 0, 0, 0, 0, 0, 0, 0, 190, 60, 245, 109, 66, 63, 153, 154, 66, 25, 51, 51, 66, 116, 204, 205, 253, 63, 242, 146, 224, 105, 231, 66, 209, 63, 242, 146, 65, 3, 193, 196, 156, 0, 0, 0, 0, 0, 0, 0, 0, 190, 115, 157, 190, 66, 62, 0, 0, 66, 21, 51, 51, 66, 119, 153, 154, 253, 191, 231, 207, 8, 215, 85, 22, 88, 191, 231, 206, 77, 130, 151, 190, 17, 191, 146, 242, 158, 148, 114, 240, 57, 188, 224, 224, 96, 66, 68, 102, 103, 66, 37, 153, 154, 66, 127, 153, 154, 253, 0, 0, 0, 53, 4, 63, 217, 153, 52, 224, 36, 238, 93, 191, 217, 153, 169, 67, 241, 211, 23, 63, 207, 255, 137, 8, 22, 253, 198, 63, 240, 0, 170, 111, 207, 54, 176, 63, 243, 51, 88, 137, 58, 151, 217, 63, 201, 148, 119, 151, 70, 237, 237, 0, 0, 0, 29, 5, 64, 143, 64, 0, 0, 0, 0, 0, 64, 143, 64, 0, 0, 0, 0, 0, 64, 143, 64, 0, 0, 0, 0, 0, 0, 0, 0, 61, 3, 0, 63, 0, 0, 0, 0, 63, 123, 129, 184, 27, 129, 184, 28, 63, 112, 225, 14, 16, 225, 14, 17, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 66, 87, 51, 51, 66, 66, 0, 0, 62, 35, 215, 11, 61, 241, 169, 253, 0, 0, 0, 37, 2, 0, 0, 63, 141, 83, 47, 180, 171, 196, 232, 63, 137, 46, 99, 102, 69, 149, 155, 66, 55, 51, 51, 0, 59, 68, 155, 166, 66, 92, 0, 0, 253, 0]
   
        #for val in test_data:
            #data += chr(val)
        
        Result = namedtuple('Result', ['coordinates','joint_positions'])
        while True:
            data += self.sock.recv(2048)
            
            
            
            while len(data) > 4 and self.readLengthOfPacket(data) <= len(data):
                length = self.readLengthOfPacket(data)
                #print 'processing a packet', length, 'long'
                packet = data[5:length]#chop of the first 5 bytes which is the length we just read in, plus a type (all the same currently)
                data = data[length:]
                
                
                coordinates = []
                joint_positions = []
                while len(packet) > 3:
                    
                    sub_packet_length = self.readLengthOfPacket(packet) 
                    
                    message_type = ord(packet[4])
                    #print 'sub packet is ', sub_packet_length, 'long, and is type', message_type
                    sub_packet = packet[5:sub_packet_length]
                    #self.printPacket(sub_packet)
                    packet = packet[sub_packet_length:]
                    
                    if message_type == CARTESIAN_INFO:
                        
                        for i in xrange(6):
                            val = struct.unpack('!d',sub_packet[i*8:(i+1)*8])[0]
                            coordinates.append(val)
                    
                    if message_type == JOINT_DATA:
                        for joint in xrange(6):
                            joint_position_string = sub_packet[41 * joint:(41 * joint) + 8]
                            val = struct.unpack('!d',joint_position_string)[0]
                            joint_positions.append(val)
                        #print joint_positions
                
                if not isLegalPos(coordinates, joint_positions):
                    self.stop()
                    self.moveHome(60);
                    print "Out of bounds, going home and exiting"
                    sys.exit()
                
                
                if output_coordinates:
                    print "coords,",
                    for i in xrange(6):
                        print str(coordinates[i]) + ",",
                    for i in xrange(6):
                        print str(joint_positions[i]) + ",",
                    print 
                    sys.stdout.flush()
                    
                    
                if cb is not None:
                    
                    result = Result(coordinates, joint_positions)
                    cb(result)
                #print 'packet processed'
        
        
        
class URServer:
    def __init__(self):
        self.target = [-0.4,-0.3,0.575,-1.34,1.2,1.5]
        self.target_time = 0
        
        print "URServer"
    
    def set_target(self,target, time):
        self.target = list(target)
        self.target_time = time
    
    def format_coordinate(self,p, time):
        to_send = "("
        for val in p:
            to_send += str(val) + ", "
            
        time_remaining_seconds = (time - get_millis_now())/1000
        
        if time_remaining_seconds > 0:
        
            to_send += str(time_remaining_seconds) +  ", 1"
        else:
            to_send +=  "0.0, 0"
            
        to_send += ")"
        print to_send
        return to_send
    
    
    
    def process_received_line(self,line, clientsock):
        #print "line received", line
        if line == "get_pos":
            #print "sending response"
            clientsock.send(self.format_coordinate(self.target, self.target_time) + "\n")
    def handler(self,clientsock,addr):
        
        receive_buffer = ""
        
        while 1:
            data = clientsock.recv(1024)
            #print 'data:' + data
            receive_buffer += data
            
            while "\n" in receive_buffer:
                line = receive_buffer.split("\n")[0]
                receive_buffer = receive_buffer[len(line) + 1:]#chop off the line we just processed from the buffer
                self.process_received_line(line,clientsock)
            
            if not data: 
                break
        clientsock.close()
    def startServer(self):
        ADDR = (HOST, port_to_connect_to_script)
        serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serversock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serversock.bind(ADDR)
        serversock.listen(5)
        
        while True:
            print 'waiting for connection...'
            clientsock, addr = serversock.accept()
            print '...connected from:', addr
            thread.start_new_thread(self.handler, (clientsock, addr))


if __name__=='__main__':
    connector = UR_TCP_Connector()
    connector.connect()
    p = [0.25,0.64,0.31,0.1,0.1,-1]
    #connector.moveCart(p)
    
    j = [-1.9135521611253155, -0.06867831450242003, -0.940805363889819, -2.6783308950861193, 1.3423438030896497, 0.4017711370431652]
    
    #blend_radius = .5
    
    #connector.moveJoint(j, blend_radius = blend_radius)
    #time.sleep(0.5)
    #connector.moveJoint(j, blend_radius = 0)
    #connector.sendPopUp()
    #time.sleep(0.5)
    #connector.moveHome()
    connector.sendScript()
    #connector.monitor_state()



