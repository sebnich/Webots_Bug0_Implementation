from controller import Robot, DistanceSensor, Motor, GPS, PositionSensor
import math
import numpy as np

MAX_SPEED = 5.24
MAX_SENSOR_NUMBER = 16
TIME_STEP = 64
Min_dist = 0.3

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# label sensors
sensors = []
sensorNames = [
    'so0', 'so1', 'so2', 'so3', 'so4', 'so5', 'so6', 'so7',
    'so8', 'so9', 'so10', 'so11', 'so12', 'so13', 'so14', 'so15' 
]

# enable each distance sensor and add timestep
for i in range(MAX_SENSOR_NUMBER):
    sensors.append(robot.getDistanceSensor(sensorNames[i]))
    sensors[i].enable(timestep)

# get left and right wheel motors
leftMotor = robot.getMotor('left wheel')
rightMotor = robot.getMotor('right wheel')

# set left and right wheel positions
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# get robot gps and enable it with a timestep
gps_sensor = robot.getGPS('gps')
gps_sensor.enable(timestep)

# initialize varaibles
t = 0 # total number of iterations done in simulation
goalX = 0  # goal coordinate variables
goalY = 0
goalVector = np.zeros(3) # Array to hold vector pointing from robot to goal
robotVector = np.ones(3) # Array to hold the robots orientation vector
# set at [1,1,1] so robot doesn't start off going backwards
angle = 0 # Initialize angle between goalVector and robotVector

go_backwards = False # Boolean used to check if the robot should go backwards
stopFlag = False # Boolean used to check if the robot should stop
timeBack = 0 # time used to count how long robot should travel backwards if it gets stuck

while robot.step(timestep) != -1:
    if t == 0:
        # During the first time step find the goal point position
        # and initialize the robots previous position
        prevPos = [0,0,0]
        currPos = gps_sensor.getValues()
        start = [currPos[0],currPos[1]]
        goalX = currPos[0] + 5
        goalY = currPos[2] - 5
    else:
        # At each time step find the robots orientation vector by subtracting
        # the robot's previous orientation from it's current orientation. 
        prevPos = currPos
        currPos = gps_sensor.getValues()
        robotVector = [currPos[0] - prevPos[0], currPos[2] - prevPos[2],0]
        #  Find the vector pointing from the robot to the goal by subtracting
        # the robot's current position from the goal coordinates.
        goalVector = [goalX - currPos[0], goalY-currPos[2],0]
        # The code below finds the angle between these two vectors
        num = np.dot(robotVector,goalVector)
        normA = np.linalg.norm(robotVector)
        normB = np.linalg.norm(goalVector)
        dem = normA*normB      
        if (1-(num/dem)) < .0000001:
            n_m = 1
        elif (1+(num/dem)) < .000001:
            n_m = -1
        else:
            n_m = num/dem 
        theta = math.acos(num/dem)
        q = np.cross(robotVector,goalVector)
        # This is the angle the robot needs to correct to in order to head straight towards the goal,
        # we can use this angle as an error value.
        angle = theta*(np.sign(q[2]))
    
    # Find distance values for each sensor
    sensor_values = []
    distance_values = []
    for i in range(MAX_SENSOR_NUMBER):
        sensor_values.append(sensors[i].getValue())
        distance_values.append(5.0 * (1.0 - (sensor_values[i] / 1024)))#MAX_SENSOR_VALUE=1024
    
    # Check if the robot isn't moving
    go_back = abs(robotVector[0]) < .00005 and abs(robotVector[1]) < .00005
    # Check sensors located at the front of the robot       
    front_obstacle = distance_values[3] < Min_dist or distance_values[4] < Min_dist or distance_values[5] < Min_dist or distance_values[6] < Min_dist or distance_values[1] < Min_dist or distance_values[2] < Min_dist
    # Check sensors on each side of the robot, this is essentially the obstacle avoidance comparator
    traverse = distance_values[15] < Min_dist or distance_values[0] < Min_dist or distance_values[7] < Min_dist or distance_values[8] < Min_dist or distance_values[1] < Min_dist or distance_values[6] < Min_dist
    # Checks if the angle is within a certain range, if not correct orientation
    headToGoal = abs(angle - .17) > .1
    # Checks if the robot is at the goal
    atGoal = ((currPos[0] - goalX)**2 + (currPos[2] - goalY)**2)**(1/2) < .01
    print(((currPos[0] - goalX)**2 + (currPos[2] - goalY)**2)**(1/2))
        
    # Set robots speed
    left_speed = 0.5 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED
    
    if stopFlag:
        left_speed = 0
        right_speed = 0
    
    elif go_backwards:
        # Go backwards for 30 timesteps
       if timeBack < 30:
           left_speed = -0.5 * MAX_SPEED
           right_speed = -0.5 * MAX_SPEED    
           timeBack += 1      
       else:
           # After 30 timesteps stop and reset go_backwards boolean and timeback variable
           go_backwards = False
           timeBack = 0
           
    # Set go_backwards boolean to true if the robot isn't moving
    elif go_back:
       go_backwards = True
    
    # Check front obstacle
    elif front_obstacle:
       # Turn right
       left_speed += 0.5 * MAX_SPEED
       right_speed -= 0.5 * MAX_SPEED
    
    # Checks both sides of the robot for an obstacle and uses error control to correct it's velocity
    elif traverse:
        # Turn right due to obstacle on the left
        if distance_values[0] < Min_dist:
            e = distance_values[0] - Min_dist
        elif distance_values[15] < Min_dist:
            e = distance_values[15] - Min_dist
        elif distance_values[1] < Min_dist:
            e = distance_values[1] - Min_dist
            
        # Turn left due to an obstacle on the right
        elif distance_values[7] < Min_dist:
            e = distance_values[7] - Min_dist
        elif distance_values[6] < Min_dist:
            e = distance_values[6] - Min_dist
        else:
            e = distance_values[8] - Min_dist
        
        # Proportional and integral control    
        k_p = .3
        k_i = k_p/2
        u_b = -k_i * e * (t/TIME_STEP)
        u = (-k_p * e) + u_b
        left_speed += u * MAX_SPEED
        right_speed -= u * MAX_SPEED
      
    elif headToGoal:
        # Correct orientation with proportional and integral control
        e = angle
        k_p = .05
        k_i = k_p/2
        u_b = -k_i * e * (t/TIME_STEP)
        u = (-k_p * e) + u_b
        left_speed -= u * MAX_SPEED
        right_speed += u * MAX_SPEED
        
    if atGoal:
        stopFlag = True
   
    # Correct any veclocities that are out of range
    if left_speed > MAX_SPEED:
        left_speed = MAX_SPEED
    if left_speed < -MAX_SPEED:
        left_speed = -MAX_SPEED
    if right_speed > MAX_SPEED:
        right_speed = MAX_SPEED
    if right_speed < -MAX_SPEED:
        right_speed = -MAX_SPEED
        
    if left_speed > 0 and right_speed > 0:
        reallyStuck = 0
    
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)
    
    t+=1 # Update time step                 
    pass

# Enter here exit cleanup code.
