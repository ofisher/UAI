'''
Created on 15/07/2013

@author: oliver
'''


def isLegalPos(coordinates, joint_positions):
    '''Returns false if the arm has gone out of bounds 
    coordinates, list of coordinates of the tool tip of the robot, x, y z, rx, ry and rz
    joint_positions, list of joint positions, starting at the bottom joint
    '''
    return True
    
    if joint_positions[1] > 0:#sholder greater than 0 degrees (sholder section crashing into frame
        return False
    if coordinates[0] > 0.7:#x pos greater than 700mm (floor)
        return False
    if coordinates[0] > 0.13 and coordinates[1] > 0.2: #xpos > 130mm and y coord > 200mm, this is for the generator
        return False
    if coordinates[2] < 0: #z < 0mm - back wall
        return False
    if coordinates[1] > 0.5: #y> 500mm - side wall
        return False
    return True