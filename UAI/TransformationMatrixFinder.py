'''
Created on May 19, 2013
This program generates a transformation matrix using a hill climbing least squares approch"
It tests the accuracy with and without outlier removal using cross validation on 5 data sets"
Then it does the same when random errors are added to the data sets"

@author: Oliver Fisher
'''





from numpy import *
from random import choice, uniform
import sys

errors = set()


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

#uses the first 4 points
def generateTransformationMatrix(points):
    robot_poses = []
    camera_poses = []
    for i in xrange(4):
        robot_poses.append(toHomo(points[i][0]))#robot positions
        camera_poses.append(toHomo(points[i][1]))#camera positions
    robot_matrix = transpose(array(robot_poses))
    #print robot_matrix
    camera_matrix = transpose(array(camera_poses))
    #print camera_matrix
    camera_matrix_inverse = linalg.inv(camera_matrix)
    #print camera_matrix_inverse
    
    T = dot(robot_matrix,camera_matrix_inverse)
    #print T
    return T

def stringToList(str):
    to_return = []
    for pos in str.split(","):
        to_return.append(float(pos))
    return to_return
        
def getErrorInRobotDistance(position, T, inAbsolute = True):
    robot_pos, cam_pos = position
    Tinv = linalg.inv(T)
    
    
    rob_pos, cam_pos = position
    calc_cam_pos = Transform(rob_pos,Tinv)

    calc_cam_pos_with_x_error = list(calc_cam_pos)
    calc_cam_pos_with_x_error[0] = cam_pos[0]
    calc_robot_pos_with_x_error = Transform(calc_cam_pos_with_x_error,T)
    
    calc_cam_pos_with_y_error = list(calc_cam_pos)
    calc_cam_pos_with_y_error[1] = cam_pos[1]
    calc_robot_pos_with_y_error = Transform(calc_cam_pos_with_y_error,T)
    
    calc_cam_pos_with_z_error = list(calc_cam_pos)
    calc_cam_pos_with_z_error[2] = cam_pos[2]
    calc_robot_pos_with_z_error = Transform(calc_cam_pos_with_z_error,T)
    
    #print calc_robot_pos_with_x_error
    
    x_error = linalg.norm(toCart(calc_robot_pos_with_x_error)-toCart(rob_pos))
    y_error = linalg.norm(toCart(calc_robot_pos_with_y_error)-toCart(rob_pos))
    z_error = linalg.norm(toCart(calc_robot_pos_with_z_error)-toCart(rob_pos))
    
    if not inAbsolute:
    
        if calc_cam_pos[0] < cam_pos[0]:
            x_error *= -1
            
        if calc_cam_pos[1] < cam_pos[1]:
            y_error *= -1
            
        if calc_cam_pos[1] < cam_pos[1]:
            z_error *= -1
    
    return x_error, y_error, z_error
    

def loadPositions(filename):
    
    all_locations = [] #contains [robot_pos,cam_pos]
    #for robot_pos, cam_pos in zip(open("robot pos.txt").readlines(),open(filename)):
    #    if "No marker detected" not in cam_pos:
    #        all_locations.append([stringToList(robot_pos), stringToList(cam_pos)])
    
    KINECT_DEPTH_INDEX = 1
    SCREEN_X_INDEX = 2
    SCREEN_Y_INDEX = 3
    AR_X_INDEX = 4
    AR_Y_INDEX = 5
    AR_Z_INDEX = 6
    ROBOT_X_INDEX = 7
    ROBOT_Y_INDEX = 8
    ROBOT_Z_INDEX = 9
    
    with open(filename) as f:
        for line in f.readlines():
            vals = line.split(',')
            robx = float(vals[ROBOT_X_INDEX])
            roby = float(vals[ROBOT_Y_INDEX])
            robz = float(vals[ROBOT_Z_INDEX])
            
            arx = float(vals[SCREEN_X_INDEX])
            ary = float(vals[SCREEN_Y_INDEX])
            arz = float(vals[KINECT_DEPTH_INDEX])
            all_locations.append([[robx,roby,robz],[arx,ary,arz]])
            
    
    return all_locations

discount_z = 1

def getError(position, Tinv ):
    rob_pos, cam_pos = position
    calc_cam = toCart(Transform(rob_pos,Tinv))
    dif = array(calc_cam) - array(cam_pos)
    return sqrt(dif[0]**2 + dif[1]**2 + (dif[2]/discount_z)**2)

def calculateCost(positions, T):
    
    Tinv = linalg.inv(T)
    cost = 0
    for position in positions:
        #calc_cam = toCart(Transform(rob_pos,Tinv))
        #dif = array(calc_cam) - array(cam_pos)
        #cost += abs(dif[0])**2 + abs(dif[1])**2 + abs(dif[2])**2/discount_z
        cost += getError(position,Tinv)
    return cost

ep = 1e-10
def getSlope(T, row, col, positions):
    cost_at_T = calculateCost(positions,T)
    T[row][col] += ep
    cost_above_T = calculateCost(positions,T)
    return (cost_above_T - cost_at_T) 

def getSlopeVector(T, vector, vector_scale, positions):
    cost_at_T = calculateCost(positions,T)
    T = array(T)
    T = T + vector * vector_scale
    cost_above_T = calculateCost(positions,T)
    return (cost_above_T - cost_at_T) 



def hillClimbVector(T,vector, positions):
    
    start_cost = calculateCost(positions,T)
    inital_T = array(T)
    
    new_T = array(T)
    
    upper = 100.0
    lower = 0
    
    while upper - lower > ep:
        
        middle = (upper + lower)/2
        
        new_T = T + vector * middle
        #print calculateCost(positions, new_T)
        slope = getSlopeVector(new_T,vector,middle, positions)
        #print "slope", slope
        if slope < 0:
            lower = middle
        else:
            upper = middle
    T = new_T
    
    found_cost = calculateCost(positions,T)
    if start_cost < found_cost:
        T = inital_T
        #print "hill climb VECTOR failed to finder better value"
    return T
            
    
def hillClimb(T, row, col, positions, search_width = 1000.0):
    #print T[row][col]
    
    start_cost = calculateCost(positions,T)
    
    initial = T[row][col]
    
    upper = T[row][col] + search_width
    lower = T[row][col] - search_width
    
    while upper - lower > ep:

        middle = (upper + lower)/2
        T[row][col] = middle
        if getSlope(T,row,col,positions) > 0:
            upper = middle
        else:
            lower = middle
            
    
    found_cost = calculateCost(positions,T)
    if start_cost < found_cost:
        T[row][col] = initial #we were better how we were
        #print "hill climb failed to finder better value"
        
    
    return T

def hillClimbAll(T, positions, max_iterations = 20, stop_when_change_less_than_relative = 0.1):
    
    search_width_for_hill_climb = amax(T)#start by looking this far +- when doing hill climb (binary search on it)
    cost_before = calculateCost(positions,T)
    
    for _ in xrange(max_iterations):
    
        
        old_T = array(T)
        for row in xrange(4):
            for col in xrange(4):
                T = hillClimb(T, row, col, positions, search_width_for_hill_climb)
        difference = T - old_T
        
        #don't bother looking more than a factor of 10 out from the previous biggest change
        search_width_for_hill_climb = amax(difference) * 10
        
        T = hillClimbVector(T, difference, positions)
        cost_after = calculateCost(positions,T)
        if (cost_before - cost_after)/cost_before < stop_when_change_less_than_relative:
            
            #print "stopping due to convergence"
            return T
        #print cost_after
        cost_before = cost_after
        #print calculateCost(positions,T)
    return T
    
    
    
def calculateErrorInEachCoordinate(positions, T):
    x_error = 0.0
    y_error = 0.0
    z_error = 0.0
    Tinv = linalg.inv(T)
    for position in positions:
    
        rob_pos, cam_pos = position
        calc_cam_pos = Transform(rob_pos,Tinv)
        #print cam_pos
        #print calc_cam_pos
        
        x_error += abs(cam_pos[0] - calc_cam_pos[0])
        y_error += abs(cam_pos[1] - calc_cam_pos[1])
        z_error += abs(cam_pos[2] - calc_cam_pos[2])
    return x_error, y_error, z_error
def findT(positions):
    T = generateTransformationMatrix(positions)#use 4 points a a starting point
    T = hillClimbAll(T,positions)
    return T


def findTImproveAndValidate(positions):
    T = generateTransformationMatrix(positions)#use 4 points a a starting point
    #print "cross validating with all" + str(crossValidate(positions))
    hillClimbAll(T,positions)
    
    
    T_all_data = T
    
    #this removes outliers
    ok_positions, bad_positions = removeWorstPoints(positions, T)
    
    error_before_remove = crossValidate(ok_positions,bad_positions)
    
    print "Cross validated average error (m) before outliers removed(x, y, z)", error_before_remove 

    #error_after_remove = crossValidate(ok_positions)
    
    #T_no_outliers = generateTransformationMatrix(ok_positions)
    #hillClimbAll(T_no_outliers,ok_positions)
    
    
    
    #hillClimbAll(T,ok_positions)
    #print "Cross validated average error (m) after outliers removed(x, y ,z)", error_after_remove
    
    
    return error_before_remove, error_before_remove, T_all_data, T_all_data


def crossValidate(ok_positions, bad_positions = [], naive_solution = False):
    error = 0
    error_from_bad_positions = 0
    
    robot_coord_error_x = 0.0
    robot_coord_error_y = 0.0
    robot_coord_error_z = 0.0
    
    all_positions = []
    
    for position in ok_positions:
        all_positions.append((True,position))
    for position in bad_positions:
        all_positions.append((False,position))
    
    for index_testing in xrange(len(all_positions)):
        
        #build positions to generate model with
        training_data = []
        for index, position_pair in enumerate(all_positions):
            if index != index_testing:
                _, position = position_pair
                training_data.append(position)
        
        
        #training_data.sort()
        #print training_data
        is_good, testing_data = all_positions[index_testing]
        
        
        if naive_solution:
            random_positions_to_generate_T_from = []
            for _ in xrange(4):
                index_to_use = choice(xrange(len(training_data)))
                random_positions_to_generate_T_from.append(training_data.pop(index_to_use))
            
            T = generateTransformationMatrix(random_positions_to_generate_T_from)#use 4 points a a starting point
        else:
            T = findT(training_data)
        
        this_error = getError(testing_data,linalg.inv(T))
        #print "this_error", this_error
        if is_good:
            
            x,y,z = getErrorInRobotDistance(testing_data, T, inAbsolute = False)
            robot_coord_error_x += abs(x)
            robot_coord_error_y += abs(y)
            robot_coord_error_z += abs(z)
            
            if naive_solution:
                errors.add((x,y,z))
            
            #print str(x) + "," + str(y) +  "," + str(z)
            
            error += this_error
        else:
            error_from_bad_positions += this_error
        
    #print "error_from_bad_positions", error_from_bad_positions
    
    robot_coord_error_x = robot_coord_error_x/len(ok_positions)
    robot_coord_error_y = robot_coord_error_y/len(ok_positions)
    robot_coord_error_z = robot_coord_error_z/len(ok_positions)
    
    #print "x, y and z error = ", robot_coord_error_x, robot_coord_error_y, robot_coord_error_z
    
    robot_error_dist = sqrt(robot_coord_error_x**2 + robot_coord_error_y**2 + robot_coord_error_z**2)
    
    #print "error distance in robot coords = ", robot_error_dist

    
    
    return robot_coord_error_x, robot_coord_error_y, robot_coord_error_z


def removeWorstPoints(positions, T):
    positions_and_error = []
    
    #print T
    Tinv = linalg.inv(T)
    total_error = 0
    for position in positions:
        error = getError(position, Tinv)
        positions_and_error.append([error,position])
        total_error += error
    #positions_and_error.sort(key = lambda x : x[0])
    
    ok_positions = []
    bad_positions = []
    average_error = total_error/len(positions)
    
    
    for error, pos in positions_and_error:
        #print error, pos
        if error > average_error * 2:
            #print "Removing outlier.", pos, "with", error/ average_error, "times more error than average"
            bad_positions.append(pos)
        else:
            ok_positions.append(pos)
    
    return ok_positions, bad_positions




def getAverageErrorInRobotDistance(positions, T):
    x_total,y_total,z_total = [0,0,0]
    for position in positions:
        x,y,z = getErrorInRobotDistance(position, T)
        x_total += x
        y_total += y
        z_total += z
    return x_total/len(positions), y_total/len(positions), z_total/len(positions)

def printValuesVaryingZDiscount(positions):
    global discount_z
    log_discount = -1
    discount = 10**log_discount
    T = findT(positions)
    ok_positions, bad_positions = removeWorstPoints(positions,T)
    while discount < 1000000:
        
        discount = 10**log_discount
        discount_z = discount
        T = findT(ok_positions)
        print discount_z, getAverageErrorInRobotDistance(ok_positions, T)
        log_discount += 0.25
    



def simulateGlitchInPositions(positions):
    
    index = choice(xrange(len(positions)))
    
    to_add = list(positions[index])
    
    to_add[1][0] += uniform(-1,1)
    to_add[1][1] += uniform(-1,1)
    to_add[1][2] += uniform(-1,1)
    positions.append(to_add)
    return positions

def saveMatrix(T, filename):
    with open(filename,'wb') as f:
        for row in T:
            for val in row:
                print >> f, val,
            print >> f
            
def run(filename = None):  
    if filename is not None:
        datafiles = [filename]
    else:
        datafiles = ["output.txt"]
    for filename in datafiles:
            
        print "processing", filename
        positions = loadPositions(filename)
        _,_, T_all, T_outliers_removed = findTImproveAndValidate(positions)
        #print T_all
        #print T_outliers_removed
        
        saveMatrix(T_all,"T_all.txt")
        saveMatrix(T_outliers_removed,"T_outliers_removed.txt")
        
    print "ALL DONE"

if __name__=="__main__":
    if len(sys.argv) > 1:
        run(sys.argv[1])
    else:
        run()

    