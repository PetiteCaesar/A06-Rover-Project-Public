"""RoverController controller."""

from controller import Robot
import math

robot = Robot()

# timestep = int(robot.getBasicTimeStep())
timestep = 64
startTime = robot.getTime()


MAX_SPEED = 2.28
WHEEL_RADIUS = 0.0205#0.025
DISTANCE_BETWEEN_WHEELS = 0.052 + 0.0035#0.09#0.0527375578

linearVelocity = WHEEL_RADIUS * MAX_SPEED

def getTimeForDistance(distance):
    return distance / linearVelocity


leftMotor = robot.getDevice("left wheel motor")#motor 1
# leftMotor2 = robot.getDevice("left2")
rightMotor = robot.getDevice("right wheel motor")#motor 2
# rightMotor2 = robot.getDevice("right 2")

# usSensor = robot.getDevice("distance")
# usSensor.enable(timestep * 2)
# servo = robot.getDevice("servo")

leftMotor.setPosition(float("inf"))
# leftMotor2.setPosition(float("inf"))
rightMotor.setPosition(float("inf"))
# rightMotor2.setPosition(float("inf"))



def left(vel):
    leftMotor.setVelocity(vel)
    # leftMotor2.setVelocity(vel)

def right(vel):
    rightMotor.setVelocity(vel)
    # rightMotor2.setVelocity(vel)

# Global variables for state
state = "MOVE_FORWARD"  # Other states: WAIT_BEFORE_TURN, TURNING_LEFT, WAIT_AFTER_TURN
stateStartTime = 0

r = (2 * linearVelocity) / DISTANCE_BETWEEN_WHEELS
# turnDuration = (math.pi / 2) / r #+ 0.23538

turnDuration = getTimeForDistance(math.pi * DISTANCE_BETWEEN_WHEELS / 4) # 90 degrees turn


print("linearVelocity: ", linearVelocity)
print("r: ", r)
print("turnDuration: ", turnDuration)
def startWaitBeforeTurn():
    global state, stateStartTime
    left(0)
    right(0)
    stateStartTime = robot.getTime()
    state = "WAIT_BEFORE_TURN"

def startTurnLeft():
    global state, stateStartTime
    
    
    left(-MAX_SPEED)
    right(MAX_SPEED)
    stateStartTime = robot.getTime()
    state = "TURNING_LEFT"

def startWaitAfterTurn():
    global state, stateStartTime
    left(0)
    right(0)
    stateStartTime = robot.getTime()
    state = "WAIT_AFTER_TURN"

def resumeForward():
    global state
    left(MAX_SPEED)
    right(MAX_SPEED)
    state = "MOVE_FORWARD"

# Main control loop
while robot.step(timestep) != -1:
    currentTime = robot.getTime()

    if state == "MOVE_FORWARD":
        left(MAX_SPEED)
        right(MAX_SPEED)
        # if usSensor.getValue() < 40:
        #     startWaitBeforeTurn()
        if currentTime > startTime + getTimeForDistance(0.25):
            startWaitBeforeTurn()
            

    elif state == "WAIT_BEFORE_TURN":
        if currentTime - stateStartTime >= 2.0:
            startTurnLeft()

    elif state == "TURNING_LEFT":
        if currentTime - stateStartTime >= turnDuration:
            startWaitAfterTurn()


    elif state == "WAIT_AFTER_TURN":
        if currentTime - stateStartTime >= 2.0:
            resumeForward()
            startTime = currentTime
    
    # left(MAX_SPEED)
    # right(MAX_SPEED)

    # # Handle servo (unrelated)
    # if currentTime > startTime + getTimeForDistance(0.25):
    #     servo.setPosition(0)
    #     # startTime = currentTime
    #     left(0)
    #     right(0)
