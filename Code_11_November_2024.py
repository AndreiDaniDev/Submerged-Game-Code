import runloop, hub, motor, motor_pair, color, math, time, color_sensor
from hub import motion_sensor as gs

# ---> The class that has all acceleration methods <---
class SpeedMethods(object):
    # ---> https://www.desmos.com/calculator/etn4nkl1fz <---
    # ---> https://easings.net <---
    @staticmethod
    def liniar(x: float) -> float:
        return x
    @staticmethod
    def easeOutInLiniar(x: float) -> float:
        return 1 - abs(2 * x - 1)
    @staticmethod
    def easeOutInQuad(x: float) -> float:
        return 1 - math.pow(abs(2 * x - 1), 2)
    @staticmethod
    def easeInQuad(x: float) -> float:
        return math.pow(x, 2)
    @staticmethod
    def easeOutQuad(x: float) -> float:
        return math.pow(1 - x, 2)

# ---> The class for the lower part of the robot that is responsible for moving <---
class DriveBase(object):

    def __init__(self, leftMotorDB: int, rightMotorDB: int, leftMotorSYS: int, rightMotorSYS: int, oneUnit: int, scale: int, brakeMethod) -> None:

        '''
        ---> Parameters Initialization Class <---> DB - driveBase, SYS - systems <---
        leftMotorDB: the port of the left motor from the driveBase
        rightMotorDB: the port of the right motor from the driveBase

        leftMotorSYS: the port of the left motor from the systems mounting system
        rightMotorSYS: the port of the right motor from the systems mounting system

        oneUnit: the circumference written as a fraction (oneUnit / scale = circumference)
        scale: the circumference written as a fraction (oneUnit / scale = circumference)
        brakeMethod: how we want the robot to break for all functions
        '''

        # ---> Defining the constant values that won't change in any run <---
        motor_pair.pair(motor_pair.PAIR_1, leftMotorDB, rightMotorDB)
        self.oneUnit: int = abs(oneUnit); self.scaleUnit: int = scale
        self.leftMotorDB = leftMotorDB; self.rightMotorDB = rightMotorDB
        self.leftMotorSYS = leftMotorSYS; self.rightMotorSYS = rightMotorSYS
        self.pair: int = motor_pair.PAIR_1; self.brake = brakeMethod
        self.pairMaxSpeed: int = 1100; self.pairMinSpeed: int = 100
        self.usualMaxSpeed: int = 1000; self.usualMinSpeed: int = 200
        self.maxAngle = 3350; self.maxOneTurnAngle = 1650

        # ---> Defining the (global) values that will be used in the functions <---
        self.speed: int = 0; self.startSpeed: int = 0; self.endSpeed: int = 0
        self.diffSpeed: int = 0; self.leftSpeed: int = 0; self.rightSpeed: int = 0

        self.approximationScale: int = 0; self.currentApproximationValue: float = 0
        self.previousApproximationValue: float = 0; self.outputValue: int = 0

        self.currentReachDistance: int = 0; self.previousReachDistance: int = 0
        self.reachDistance: int = 0; self.reachAngle: int = 0; self.targetedAngle: int = 0

        self.position: int = 0; self.previousPosition: int = 0

        self.iterator: int = 0; self.previousFunction: int = 0

        self.kp: int = 0; self.ki: int = 0; self.kd: int = 0
        self.dt: int = 0; self.kLeft: int = 0; self.kRight: int = 0

        self.angle: int = 0; self.error: int = 0; self.previousError: int = 0
        self.proportional: int = 0; self.integral: int = 0
        self.derivative: int = 0; self.controllerOutput: int = 0
        self.leftColorError: int = 0; self.rightColorError: int = 0

        motor.reset_relative_position(self.leftMotorDB, 0)
        motor.reset_relative_position(self.rightMotorDB, 0)
        gs.reset_yaw(0); self.wasStuck: bool = False
        return None

    def initRun(self) -> None:
        # ---> This needs to be called at the start of every program / run <---
        # ---> In order for all variables to be reset <---

        self.speed = 0; self.startSpeed = 0; self.endSpeed = 0
        self.diffSpeed = 0; self.leftSpeed = 0; self.rightSpeed = 0

        self.approximationScale = 0; self.currentApproximationValue = 0
        self.previousApproximationValue = 0; self.outputValue = 0

        self.currentReachDistance = 0; self.previousReachDistance = 0
        self.reachDistance = 0; self.reachAngle = 0

        self.position = 0; self.previousPosition = 0

        self.iterator = 0; self.previousFunction = 0

        self.kp = 0; self.ki = 0; self.kd = 0
        self.dt = 0; self.kLeft = 0; self.kRight = 0

        self.angle = 0; self.error = 0; self.previousError = 0
        self.proportional = 0; self.integral = 0
        self.derivative = 0; self.controllerOutput = 0

        self.leftColorError = 0; self.rightColorError = 0

        motor.reset_relative_position(self.leftMotorDB, 0)
        motor.reset_relative_position(self.rightMotorDB, 0)
        gs.reset_yaw(0); self.wasStuck: bool = False

        return None

    def limit(self, value: int | float, valueMax: int | float, valueMin: int | float) -> int | float:
        # ---> Make a value be in range of [valueMin, valueMax] and return it <---
        return min(max(value, valueMin), valueMax)

    def limitAbs(self, value: int | float, valueMax: int | float, valueMin: int | float) -> int | float:
        # ---> Make a value be in range of [valueMin, valueMax] based on its sign and return it <---
        return (min(max(value, valueMin), valueMax) * (value / abs(value)) if (value != 0) else 0)

    def transformDistanceMM(self, distanceMM: int) -> int:
        return (abs(distanceMM) * self.oneUnit) // self.scaleUnit

    def getTimeDistance(self) -> float: # Return how far the robot has travelled based on the given distance
        return (self.position - self.previousReachDistance) / self.currentReachDistance

    def getTimeAngle(self) -> float: # return how far the robot has rotated based on the given angle
        return (self.angle) / (self.reachAngle)

    def getApproximation(self, value: float, approximationScale: int) -> float:
        # Return the approximated value based on a simple formula and the given scale
        return math.floor(value * approximationScale) / approximationScale

    def getSpeed(self, method, addition: int, divizor: int) -> int:
        if(self.currentApproximationValue != self.previousApproximationValue):
            self.outputValue = self.diffSpeed * method((self.currentApproximationValue + addition) / divizor)
            self.previousApproximationValue = self.currentApproximationValue
        return self.outputValue

    def PID_Controller(self, error: int, integralLimit: int, sign: int, scaleConstants: int) -> None:

        '''
        ---> Parameters for PID_Controller (Proportional - Integral - Derivative Controller) <---
        error: The error that was read by the sensor (in our case the gyroscopic sensor)
        integralLimit: A value that acts like a limit so that the integral won't overshoot from regular boundaries
        sign: A value that signifies the sign of the controller's output
        scaleConstants: kp, kd and ki are written as (x / scaleConstants, where x is either kp, kd or ki (integers))
        '''

        self.error = (error)
        self.proportional = (self.error)
        self.integral += (self.error * self.dt)
        self.integral = int(self.limit(self.integral, integralLimit, -integralLimit))
        self.derivative = (self.error - self.previousError)
        self.previousError = (self.error)

        self.controllerOutput = sign * (
            self.proportional * self.kp +
            self.integral * self.ki +
            self.derivative * self.kd // self.dt
        ) // scaleConstants

        return None

    async def gyroForwards(self, distance: int, startSpeed: int, endSpeed: int, *, kp: int = 50, ki: int = 20, kd: int = 75, dt: int = 1, constantsScale: int = 100, integralLimit: int = 250, stallDetectionIterator: int = 2000, stallDetectionThreshold: int = 5, accelerationScale: int = 100, accelerationMethod = SpeedMethods.easeOutInQuad, stop: bool = True) -> None:

        '''
        ---> Parameters Gyro Forwards / Backwards <---
        distance: How much we want the robot to travel (in milimeters)
        startSpeed: The speed that the robot will start
        endSpeed: The speed that the robot will reach

        kp, ki, kd: Positive coefficients that will be used in the Proportional - Integral - Derivative Controller (to calculate the correction that will be applied)
        dt: The time between the readings of two consecutive errors (acts like a delay -> slower response time, but sometimes this can help)
        constantScale: All of our constants are written as fractions (a / b = c) so that the PID Controller will have a faster response time.
        integralLimit: A value that acts like a limit so that the integral won't overshoot from regular boundaries
        stallDetectionIterator: This represents how frequently the controller checks if the robot is stuck or not (be aware that it's affected by dt)
        stallDetectionThreshold: If the difference between the current position and the last position is smaller that this value it means that the robot is stuck
        accelerationScale: It reprezents how many speed changes we want to have while travelling the given distance
        accelerationMethod: The graph (function) that will influence what the speed is based on the travelled distance until that moment
        stop: If we want to stop the robot after the funcion this will be true, otherwise false
        '''

        if(self.wasStuck == True and self.previousFunction == 1):
            print("Canceled function"); return None

        self.kp = abs(kp); self.ki = abs(ki); self.kd = abs(kd); self.dt = abs(dt)

        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle); self.reachAngle = 0
        self.startSpeed = abs(startSpeed); self.endSpeed = abs(endSpeed)
        self.currentReachDistance = self.transformDistanceMM(distance)
        self.approximationScale = abs(accelerationScale)
        self.diffSpeed = self.endSpeed - self.startSpeed
        self.outputValue = 0; self.iterator = 1
        self.previousApproximationValue = 0
        self.previousReachDistance = 0
        self.wasStuck = False

        # Make startSpeed & endSpeed be in the range of [self.pairMinSpeed, self.pairMaxSpeed]
        if(self.startSpeed > self.pairMaxSpeed): self.startSpeed = self.usualMaxSpeed
        if(self.startSpeed < self.pairMinSpeed): self.startSpeed = self.usualMinSpeed
        if(self.endSpeed > self.pairMaxSpeed): self.endSpeed = self.usualMaxSpeed
        if(self.endSpeed < self.pairMinSpeed): self.endSpeed = self.usualMinSpeed

        # Precalculating the necessary values
        if(self.previousFunction == 1):
            self.previousReachDistance = self.reachDistance
            self.reachDistance += self.currentReachDistance
        else:
            motor.reset_relative_position(self.leftMotorDB, 0)
            motor.reset_relative_position(self.rightMotorDB, 0)
            self.reachDistance = self.currentReachDistance
            self.previousPosition = 0; self.position = 0

            self.proportional = 0; self.integral = 0
            self.derivative = 0; self.error = 0
            self.previousError = 0; self.controllerOutput = 0

        await runloop.sleep_ms(25)

        while(self.position < self.reachDistance):
            self.angle = gs.tilt_angles()[0] # We only read once and store the value
            self.PID_Controller(self.reachAngle - self.angle, integralLimit, 1, constantsScale)
            # self.controllerOutput - The correction that needs to be applied

            # ---> Calculating the necessary speeds to fix the error and to accelerate <---
            self.currentApproximationValue = self.getApproximation(self.getTimeDistance(), self.approximationScale)
            self.speed = int(self.startSpeed + self.getSpeed(accelerationMethod, 0, 1))
            self.leftSpeed = int(self.limit(int(self.speed - self.controllerOutput), self.pairMaxSpeed, self.pairMinSpeed))
            self.rightSpeed = int(self.limit(int(self.speed + self.controllerOutput), self.pairMaxSpeed, self.pairMinSpeed))

            # ---> Object Stall Detection <---
            self.position = (abs(motor.relative_position(self.leftMotorDB)) + abs(motor.relative_position(self.rightMotorDB)))
            if(self.iterator >= stallDetectionIterator): # (self.iterator % stallDetectionIterator == 0)
                if(self.position - self.previousPosition < stallDetectionThreshold):
                    print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                self.previousPosition = self.position; self.iterator = 0

            motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
            await runloop.sleep_ms(dt); self.iterator += 1

        if(stop or self.wasStuck):
            motor_pair.stop(self.pair, stop = self.brake)
        self.previousFunction = 1

        return None

    async def gyroBackwards(self, distance: int, startSpeed: int, endSpeed: int, *, kp: int = 50, ki: int = 20, kd: int = 75, dt: int = 1, constantsScale: int = 100, integralLimit: int = 250, stallDetectionIterator: int = 2000, stallDetectionThreshold: int = 5, accelerationScale: int = 100, accelerationMethod = SpeedMethods.easeOutInQuad, stop: bool = True) -> None:

        '''
        ---> Parameters Gyro Forwards & Backwards <---
        distance: How much we want the robot to travel (in milimeters)
        startSpeed: The speed that the robot will start
        endSpeed: The speed that the robot will reach

        kp, ki, kd: Positive coefficients that will be used in the Proportional - Integral - Derivative Controller (to calculate the correction that will be applied)
        dt: The time between the readings of two consecutive errors (acts like a delay -> slower response time, but sometimes this can help)
        constantScale: All of our constants are written as fractions (a / b = c) so that the PID Controller will have a faster response time.
        integralLimit: A value that acts like a limit so that the integral won't overshoot from regular boundaries
        stallDetectionIterator: This represents how frequently the controller checks if the robot is stuck or not (be aware that it's affected by dt)
        stallDetectionThreshold: If the difference between the current position and the last position is smaller that this value it means that the robot is stuck
        accelerationScale: It reprezents how many speed changes we want to have while travelling the given distance
        accelerationMethod: The graph (function) that will influence what the speed is based on the travelled distance until that moment
        stop: If we want to stop the robot after the funcion this will be true, otherwise false
        '''

        if(self.wasStuck == True and self.previousFunction == 2):
            print("Canceled function"); return None

        self.kp = abs(kp); self.ki = abs(ki); self.kd = abs(kd); self.dt = abs(dt)

        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle); self.reachAngle = 0
        self.startSpeed = abs(startSpeed); self.endSpeed = abs(endSpeed)
        self.currentReachDistance = self.transformDistanceMM(distance)
        self.approximationScale = abs(accelerationScale)
        self.diffSpeed = self.endSpeed - self.startSpeed
        self.outputValue = 0; self.iterator = 1
        self.previousApproximationValue = 0
        self.previousReachDistance = 0
        self.wasStuck = False

        # Make startSpeed & endSpeed be in the range of [self.pairMinSpeed, self.pairMaxSpeed]
        if(self.startSpeed > self.pairMaxSpeed): self.startSpeed = self.usualMaxSpeed
        if(self.startSpeed < self.pairMinSpeed): self.startSpeed = self.usualMinSpeed
        if(self.endSpeed > self.pairMaxSpeed): self.endSpeed = self.usualMaxSpeed
        if(self.endSpeed < self.pairMinSpeed): self.endSpeed = self.usualMinSpeed

        # Precalculating the necessary values
        if(self.previousFunction == 2):
            self.previousReachDistance = self.reachDistance
            self.reachDistance += self.currentReachDistance
        else:
            motor.reset_relative_position(self.leftMotorDB, 0)
            motor.reset_relative_position(self.rightMotorDB, 0)
            self.reachDistance = self.currentReachDistance
            self.previousPosition = 0; self.position = 0

            self.proportional = 0; self.integral = 0
            self.derivative = 0; self.error = 0
            self.previousError = 0; self.controllerOutput = 0

        while(self.position < self.reachDistance):
            self.angle = gs.tilt_angles()[0] # We only read once and store the value
            self.PID_Controller(self.reachAngle - self.angle, integralLimit, -1, constantsScale)
            # self.controllerOutput - The correction that needs to be applied

            # ---> Calculating the necessary speeds to fix the error and to accelerate <---
            self.currentApproximationValue = self.getApproximation(self.getTimeDistance(), self.approximationScale)
            self.speed = int(self.startSpeed + self.getSpeed(accelerationMethod, 0, 1))
            self.leftSpeed = -int(self.limit(int(self.speed - self.controllerOutput), self.pairMaxSpeed, self.pairMinSpeed))
            self.rightSpeed = -int(self.limit(int(self.speed + self.controllerOutput), self.pairMaxSpeed, self.pairMinSpeed))

            # ---> Object Stall Detection <---
            self.position = (abs(motor.relative_position(self.leftMotorDB)) + abs(motor.relative_position(self.rightMotorDB)))
            if(self.iterator >= stallDetectionIterator): # (self.iterator % stallDetectionIterator == 0)
                if(self.position - self.previousPosition < stallDetectionThreshold):
                    print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                self.previousPosition = self.position; self.iterator = 0

            motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
            await runloop.sleep_ms(dt); self.iterator += 1

        if(stop or self.wasStuck):
            motor_pair.stop(self.pair, stop = self.brake)
        self.previousFunction = 2

        return None

    async def turnLeft(self, angle: int, startSpeed: int, endSpeed: int, *, kLeft: int = -1, kRight: int = 1, error: int = 50, stallDetectionIterator: int = 2000, stallDetectionThreshold: int = 5, accelerationScale: int = 100, accelerationMethod = SpeedMethods.easeOutInQuad, addition: int = 0, divizor: int = 1, stop: bool = True):

        '''
        ---> Parameters Turn Left & Right <---
        angle: how much we want the robot to turn (in decidegrees)
        startSpeed: The speed that the robot will start
        endSpeed: The speed that the robot will reach

        kLeft, kRight: multiplyers for the applied speeds (if 1 the wheel will move forwards, if -1 backwards and if 0 it won't move)
        possible pairs for [kLeft, kRight]:
            ( 0, 1) -> The Right wheel will move forwards (Right wheel turn)
            (-1, 0) -> The Left wheel will move backwards (Left wheel turn)
            (-1, 1) -> Both wheels will move in opposite directions (Pivot turn)
        error: the accepted marge of error so that the robot will be as close to the given angle (50 if it's a pivot turn / 75 if not)

        stallDetectionIterator: This represents how frequently the controller checks if the robot is stuck or not (be aware that it's affected by dt)
        stallDetectionThreshold: If the difference between the current position and the last position is smaller that this value it means that the robot is stuck
        accelerationScale: It reprezents how many speed changes we want to have while travelling the given distance
        accelerationMethod: The graph (function) that will influence what the speed is based on the travelled distance until that moment
        stop: If we want to stop the robot after the funcion this will be true, otherwise false

        addition, divizor: these values are used for breaking down this function into three seperate functions
        '''

        if(self.wasStuck and kLeft == self.kLeft and kRight == self.kRight):
            print("Canceled function"); return None

        if(((kLeft > 0) and (kRight > 0)) or ((kLeft < 0) and (kRight < 0)) or (kLeft == kRight)):
            print("Error - kLeft and kRight are of the same sign"); return None

        angle = min(abs(angle), self.maxAngle)

        if((kLeft > 0 and kRight == 0) or (kLeft > 0 and kRight < 0) or (kLeft == 0 and kRight < 0)):
            return None; # This option was removed, but we added the option to have larger turns

        if(angle > self.maxOneTurnAngle):
            await self.turnLeft(angle // 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, error = error, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale // 3, accelerationMethod = accelerationMethod, addition = 0, divizor = 3, stop = False)
            await self.turnLeft(angle // 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, error = error, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale // 3, accelerationMethod = accelerationMethod, addition = 1, divizor = 3, stop = False)
            await self.turnLeft(angle // 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, error = error, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale // 3, accelerationMethod = accelerationMethod, addition = 2, divizor = 3, stop = stop)
            return None

        # ---> Set the current angle to the last error from the previous function <---
        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle)
        self.reachAngle = angle; self.targetedAngle = (angle - error)

        # ---> Precalculating some values and resetting the position <---
        self.kLeft = kLeft; self.kRight = kRight
        self.startSpeed = abs(startSpeed)
        self.endSpeed = abs(endSpeed)
        self.diffSpeed = self.endSpeed - self.startSpeed

        self.approximationScale = accelerationScale
        self.currentApproximationValue = 0
        self.previousApproximationValue = 0
        self.outputValue = 0

        self.position = 0; self.previousPosition = 0
        self.iterator = 1; self.wasStuck = False

        motor.reset_relative_position(self.leftMotorDB, 0)
        motor.reset_relative_position(self.rightMotorDB, 0)

        await runloop.sleep_ms(25)

        self.angle = gs.tilt_angles()[0]

        # This type of controller is based on a function (the accelerating method)
        # In turns we don't use PID Controllers, because for the error to be fixed
        # it will need a lot more time. -> We use acceleration so that the turns
        # won't take a lot of time, and an error variable that makes the robot
        # stop when reaching (angle - error), so that it will lose its inertia

        while(self.angle < self.targetedAngle):
            # ---> Reading the angle from the gyro sensor <---
            self.angle = gs.tilt_angles()[0]

            # ---> Speed Calculations <---
            self.currentApproximationValue = self.getApproximation(self.getTimeAngle(), self.approximationScale)
            self.speed = int(self.startSpeed + self.getSpeed(accelerationMethod, addition, divizor))

            # ---> Multiplying the speeds with the constants <---
            self.leftSpeed = int(self.speed * self.kLeft)
            self.rightSpeed = int(self.speed * self.kRight)

            # ---> Object Stall Detection <---
            self.position = (abs(motor.relative_position(self.leftMotorDB)) + abs(motor.relative_position(self.rightMotorDB)))
            if(self.iterator >= stallDetectionIterator): # (self.iterator % stallDetectionIterator == 0)
                if(self.position - self.previousPosition < stallDetectionThreshold):
                    print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                self.previousPosition = self.position; self.iterator = 0

            motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
            await runloop.sleep_ms(1); self.iterator += 1

        if(stop or self.wasStuck):
            motor_pair.stop(self.pair, stop = self.brake)
        self.previousFunction = 3

        return None

    async def turnRight(self, angle: int, startSpeed: int, endSpeed: int, *, kLeft: int = 1, kRight: int = -1, error: int = 50, stallDetectionIterator: int = 2000, stallDetectionThreshold: int = 5, accelerationScale: int = 100, accelerationMethod = SpeedMethods.easeOutInQuad, addition: int = 0, divizor: int = 1, stop: bool = True):

        '''
        ---> Parameters Turn Left & Right <---
        angle: how much we want the robot to turn (in decidegrees)
        startSpeed: The speed that the robot will start
        endSpeed: The speed that the robot will reach

        kLeft, kRight: multiplyers for the applied speeds (if 1 the wheel will move forwards, if -1 backwards and if 0 it won't move)
        possible pairs for [kLeft, kRight]:
            (1, 0) -> The Left wheel will move forwards (Left wheel turn)
            (0, -1) -> The Right wheel will move backwards (Right wheel turn)
            (1, -1) -> Both wheels will move in opposite directions (Pivot turn)
        error: the accepted marge of error so that the robot will be as close to the given angle (50 if it's a pivot turn / 75 if not)

        stallDetectionIterator: This represents how frequently the controller checks if the robot is stuck or not (be aware that it's affected by dt)
        stallDetectionThreshold: If the difference between the current position and the last position is smaller that this value it means that the robot is stuck
        accelerationScale: It reprezents how many speed changes we want to have while travelling the given distance
        accelerationMethod: The graph (function) that will influence what the speed is based on the travelled distance until that moment
        stop: If we want to stop the robot after the funcion this will be true, otherwise false

        addition, divizor: these values are used for breaking down this function into three seperate functions
        '''

        if(self.wasStuck and kLeft == self.kLeft and kRight == self.kRight):
            print("Canceled function"); return None

        if(((kLeft > 0) and (kRight > 0)) or ((kLeft < 0) and (kRight < 0)) or (kLeft == kRight)):
            print("Error - kLeft and kRight are of the same sign"); return None

        angle = min(abs(angle), self.maxAngle)

        if((kLeft < 0 and kRight == 0) or (kLeft < 0 and kRight > 0) or (kLeft == 0 and kRight > 0)):
            return None; # This option was removed, but we added the option to have larger turns

        if(angle > self.maxOneTurnAngle):
            await self.turnRight(angle // 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, error = error, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale // 3, accelerationMethod = accelerationMethod, addition = 0, divizor = 3, stop = False)
            await self.turnRight(angle // 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, error = error, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale // 3, accelerationMethod = accelerationMethod, addition = 1, divizor = 3, stop = False)
            await self.turnRight(angle // 3, startSpeed, endSpeed, kLeft = kLeft, kRight = kRight, error = error, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale // 3, accelerationMethod = accelerationMethod, addition = 2, divizor = 3, stop = stop)
            return None

        # ---> Set the current angle to the last error from the previous function <---
        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle)
        self.reachAngle = -(angle); self.targetedAngle = -(angle - error)

        # ---> Precalculating some values and resetting the position <---
        self.kLeft = kLeft; self.kRight = kRight
        self.startSpeed = abs(startSpeed)
        self.endSpeed = abs(endSpeed)
        self.diffSpeed = self.endSpeed - self.startSpeed

        self.approximationScale = accelerationScale
        self.currentApproximationValue = 0
        self.previousApproximationValue = 0
        self.outputValue = 0

        self.position = 0; self.previousPosition = 0
        self.iterator = 1; self.wasStuck = False

        motor.reset_relative_position(self.leftMotorDB, 0)
        motor.reset_relative_position(self.rightMotorDB, 0)

        await runloop.sleep_ms(25)

        self.angle = gs.tilt_angles()[0]

        # This type of controller is based on a function (the accelerating method)
        # In turns we don't use PID Controllers, because for the error to be fixed
        # it will need a lot more time. -> We use acceleration so that the turns
        # won't take a lot of time, and an error variable that makes the robot
        # stop when reaching (angle - error), so that it will lose its inertia

        while(self.angle > self.targetedAngle):
            # ---> Reading the angle from the gyro sensor <---
            self.angle = gs.tilt_angles()[0]

            # ---> Speed Calculations <---
            self.currentApproximationValue = self.getApproximation(self.getTimeAngle(), self.approximationScale)
            self.speed = int(self.startSpeed + self.getSpeed(accelerationMethod, addition, divizor))

            # ---> Multiplying the speeds with the constants <---
            self.leftSpeed = int(self.speed * self.kLeft)
            self.rightSpeed = int(self.speed * self.kRight)

            # ---> Object Stall Detection <---
            self.position = (abs(motor.relative_position(self.leftMotorDB)) + abs(motor.relative_position(self.rightMotorDB)))
            if(self.iterator >= stallDetectionIterator): # (self.iterator % stallDetectionIterator == 0)
                if(self.position - self.previousPosition < stallDetectionThreshold):
                    print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                self.previousPosition = self.position; self.iterator = 0

            motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
            await runloop.sleep_ms(1); self.iterator += 1

        if(stop or self.wasStuck):
            motor_pair.stop(self.pair, stop = self.brake)
        self.previousFunction = 4

        return None

    async def arcTurnForwards(self, radius: int, outerWheel: int, angle: int, startSpeed: int, endSpeed: int, *, error: int = 17, stallDetectionIterator: int = 2000, stallDetectionThreshold: int = 5,accelerationScale: int = 100,accelerationMethod = SpeedMethods.easeOutInQuad, addition: int = 0, divizor: int = 1, stop: bool = True) -> None:

        '''
        ---> Parameters for Arc Turn Forwards / Backwards <---
        radius: the distance between the center of the circle that the robot will follow and the robot itself (in milimiters)
        outerWheel: the port of the motor that will be on the outside (it will have greater speed)
        angle: how much we want the robot to turn (in decidegrees)
        startSpeed: The speed that the robot will start
        endSpeed: The speed that the robot will reach

        stallDetectionIterator: This represents how frequently the controller checks if the robot is stuck or not (be aware that it's affected by dt)
        stallDetectionThreshold: If the difference between the current position and the last position is smaller that this value it means that the robot is stuck
        accelerationScale: It reprezents how many speed changes we want to have while travelling the given distance
        accelerationMethod: The graph (function) that will influence what the speed is based on the travelled distance until that moment
        stop: If we want to stop the robot after the funcion this will be true, otherwise false

        addition, divizor: these values are used for breaking down this function into three seperate functions
        '''

        if(self.wasStuck and self.previousFunction == 5):
            print("Canceled Arc Turn Forwards"); return None

        angle = min(abs(angle), self.maxAngle)

        if(angle > self.maxOneTurnAngle): # ---> Divide the ArcTurn in three ArcTurns <---
            await self.arcTurnForwards(radius, outerWheel, angle // 3, startSpeed, endSpeed, error = error, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale // 3, accelerationMethod = accelerationMethod, addition = 0, divizor = 3, stop = False)
            await self.arcTurnForwards(radius, outerWheel, angle // 3, startSpeed, endSpeed, error = error, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale // 3, accelerationMethod = accelerationMethod, addition = 1, divizor = 3, stop = False)
            await self.arcTurnForwards(radius, outerWheel, angle // 3, startSpeed, endSpeed, error = error, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale // 3, accelerationMethod = accelerationMethod, addition = 2, divizor = 3, stop = stop)
            return None

        # ---> Precalculating some values and resetting the position <---
        self.startSpeed = abs(startSpeed); self.endSpeed = abs(endSpeed)
        self.diffSpeed = self.endSpeed - self.startSpeed

        self.approximationScale = accelerationScale
        self.currentApproximationValue = 0
        self.previousApproximationValue = 0
        self.outputValue = 0

        self.position = 0; self.previousPosition = 0
        self.iterator = 1; self.wasStuck = False

        # ---> Set the current angle to the last error from the previous function <---
        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle)
        motor.reset_relative_position(self.leftMotorDB, 0)
        motor.reset_relative_position(self.rightMotorDB, 0)

        if(outerWheel == self.leftMotorDB): # ---> self.turnLeft() <---
            # Calculate wheel to wheel ratio based on some formulas (https://www.desmos.com/calculator/xomkwmen35)
            # await driveBase.turnLeft(900, 200, 200, kLeft = 0, kRight = 1) # OuterWheel Left, Forwards
            self.kLeft = 1000; self.kRight = (1000 * (radius + 40) // (radius - 40))
            self.reachAngle = angle; self.targetedAngle = (angle - error)

            await runloop.sleep_ms(25)

            self.angle = gs.tilt_angles()[0]

            # ---> A modified version of the turnLeft controller <---
            while(self.angle < self.targetedAngle):
                # ---> Reading the angle from the gyro sensor <---
                self.angle = gs.tilt_angles()[0]

                # ---> Speed Calculations <---
                self.currentApproximationValue = self.getApproximation(self.getTimeAngle(), self.approximationScale)
                self.speed = int(self.startSpeed + self.getSpeed(accelerationMethod, addition, divizor))

                # ---> Multiplying the speeds with the constants <---
                self.leftSpeed = int(self.speed * self.kLeft // 1000)
                self.rightSpeed = int(self.speed * self.kRight // 1000)

                # ---> Object Stall Detection <---
                self.position = (abs(motor.relative_position(self.leftMotorDB)) + abs(motor.relative_position(self.rightMotorDB)))
                if(self.iterator >= stallDetectionIterator): # (self.iterator % stallDetectionIterator == 0)
                    if(self.position - self.previousPosition < stallDetectionThreshold):
                        print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                    self.previousPosition = self.position; self.iterator = 0

                motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
                await runloop.sleep_ms(1); self.iterator += 1

        elif(outerWheel == self.rightMotorDB): # ---> self.turnLeft() <---
            # Calculate wheel to wheel ratio based on some formulas (https://www.desmos.com/calculator/xomkwmen35)
            # await driveBase.turnRight(900, 200, 200, kLeft = 1, kRight = 0) # OuterWheel Right, Forwards
            self.kLeft = (1000 * (radius + 40) // (radius - 40)); self.kRight = 1000
            self.reachAngle = -(angle); self.targetedAngle = -(angle - error)

            await runloop.sleep_ms(25)

            self.angle = gs.tilt_angles()[0]

            # ---> A modified version of the turnRight controller <---
            while(self.angle > self.targetedAngle):
                # ---> Reading the angle from the gyro sensor <---
                self.angle = gs.tilt_angles()[0]

                # ---> Speed Calculations <---
                self.currentApproximationValue = self.getApproximation(self.getTimeAngle(), self.approximationScale)
                self.speed = int(self.startSpeed + self.getSpeed(accelerationMethod, addition, divizor))

                # ---> Multiplying the speeds with the constants <---
                self.leftSpeed = int(self.speed * self.kLeft)
                self.rightSpeed = int(self.speed * self.kRight)

                # ---> Object Stall Detection <---
                self.position = (abs(motor.relative_position(self.leftMotorDB)) + abs(motor.relative_position(self.rightMotorDB)))
                if(self.iterator >= stallDetectionIterator): # (self.iterator % stallDetectionIterator == 0)
                    if(self.position - self.previousPosition < stallDetectionThreshold):
                        print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                    self.previousPosition = self.position; self.iterator = 0

                motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
                await runloop.sleep_ms(1); self.iterator += 1

        if(stop or self.wasStuck):
            motor_pair.stop(self.pair, stop = self.brake)
        self.previousFunction = 5

        return None

    async def arcTurnBackwards(self, radius: int, outerWheel: int, angle: int, startSpeed: int, endSpeed: int, *, error: int = 17, stallDetectionIterator: int = 2000, stallDetectionThreshold: int = 5, accelerationScale: int = 100, accelerationMethod = SpeedMethods.easeOutInQuad, addition: int = 0, divizor: int = 1, stop: bool = True) -> None:

        '''
        ---> Parameters for Arc Turn Forwards / Backwards <---
        radius: the distance between the center of the circle that the robot will follow and the robot itself (in milimiters)
        outerWheel: the port of the motor that will be on the outside (it will have greater speed)
        angle: how much we want the robot to turn (in decidegrees)
        startSpeed: The speed that the robot will start
        endSpeed: The speed that the robot will reach

        stallDetectionIterator: This represents how frequently the controller checks if the robot is stuck or not (be aware that it's affected by dt)
        stallDetectionThreshold: If the difference between the current position and the last position is smaller that this value it means that the robot is stuck
        accelerationScale: It reprezents how many speed changes we want to have while travelling the given distance
        accelerationMethod: The graph (function) that will influence what the speed is based on the travelled distance until that moment
        stop: If we want to stop the robot after the funcion this will be true, otherwise false

        addition, divizor: these values are used for breaking down this function into three seperate functions
        '''

        if(self.wasStuck and self.previousFunction == 6):
            print("Canceled Arc Turn Forwards"); return None

        angle = min(abs(angle), self.maxAngle)

        if(angle > self.maxOneTurnAngle): # ---> Divide the ArcTurn in three ArcTurns <---
            await self.arcTurnBackwards(radius, outerWheel, angle // 3, startSpeed, endSpeed, error = error, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale // 3, accelerationMethod = accelerationMethod, addition = 0, divizor = 3, stop = False)
            await self.arcTurnBackwards(radius, outerWheel, angle // 3, startSpeed, endSpeed, error = error, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale // 3, accelerationMethod = accelerationMethod, addition = 1, divizor = 3, stop = False)
            await self.arcTurnBackwards(radius, outerWheel, angle // 3, startSpeed, endSpeed, error = error, stallDetectionIterator = stallDetectionIterator, stallDetectionThreshold = stallDetectionThreshold, accelerationScale = accelerationScale // 3, accelerationMethod = accelerationMethod, addition = 2, divizor = 3, stop = stop)
            return None

        # ---> Precalculating some values and resetting the position <---
        self.startSpeed = abs(startSpeed); self.endSpeed = abs(endSpeed)
        self.diffSpeed = self.endSpeed - self.startSpeed

        self.approximationScale = accelerationScale
        self.currentApproximationValue = 0
        self.previousApproximationValue = 0
        self.outputValue = 0

        self.position = 0; self.previousPosition = 0
        self.iterator = 1; self.wasStuck = False

        # ---> Set the current angle to the last error from the previous function <---
        gs.reset_yaw(gs.tilt_angles()[0] - self.reachAngle)
        motor.reset_relative_position(self.leftMotorDB, 0)
        motor.reset_relative_position(self.rightMotorDB, 0)
        self.angle = (gs.tilt_angles()[0])

        if(outerWheel == self.leftMotorDB): # ---> self.turnRight() <---
            # Calculate wheel to wheel ratio based on some formulas (https://www.desmos.com/calculator/etn4nkl1fz)
            # await driveBase.turnRight(900, 200, 200, kLeft = 0, kRight = -1) # OuterWheel Left, Backwards
            self.kLeft = -1000; self.kRight = -(1000 * (radius + 40) // (radius - 40))

            self.reachAngle = -(angle); self.targetedAngle = -(angle - error)

            await runloop.sleep_ms(25)

            self.angle = gs.tilt_angles()[0]

            # ---> A modified version of the turnRight controller <---
            while(self.angle > self.targetedAngle):
                # ---> Reading the angle from the gyro sensor <---
                self.angle = gs.tilt_angles()[0]

                # ---> Speed Calculations <---
                self.currentApproximationValue = self.getApproximation(self.getTimeAngle(), self.approximationScale)
                self.speed = int(self.startSpeed + self.getSpeed(accelerationMethod, addition, divizor))

                # ---> Multiplying the speeds with the constants <---
                self.leftSpeed = int(self.speed * self.kLeft)
                self.rightSpeed = int(self.speed * self.kRight)

                # ---> Object Stall Detection <---
                self.position = (abs(motor.relative_position(self.leftMotorDB)) + abs(motor.relative_position(self.rightMotorDB)))
                if(self.iterator >= stallDetectionIterator): # (self.iterator % stallDetectionIterator == 0)
                    if(self.position - self.previousPosition < stallDetectionThreshold):
                        print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                    self.previousPosition = self.position; self.iterator = 0

                motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
                await runloop.sleep_ms(1); self.iterator += 1

        elif(outerWheel == self.rightMotorDB): # ---> self.turnLeft() <---
            # Calculate wheel to wheel ratio based on some formulas (https://www.desmos.com/calculator/etn4nkl1fz)
            # await driveBase.turnLeft(900, 200, 200, kLeft = -1, kRight = 0) # OuterWheel Right, Backwards
            self.kLeft = -(1000 * (radius + 40) // (radius - 40)); self.kRight = -1000

            self.reachAngle = angle; self.targetedAngle = (angle - error)

            await runloop.sleep_ms(25)

            self.angle = gs.tilt_angles()[0]

            # ---> A modified version of the turnLeft controller <---
            while(self.angle < self.targetedAngle):
                # ---> Reading the angle from the gyro sensor <---
                self.angle = gs.tilt_angles()[0]

                # ---> Speed Calculations <---
                self.currentApproximationValue = self.getApproximation(self.getTimeAngle(), self.approximationScale)
                self.speed = int(self.startSpeed + self.getSpeed(accelerationMethod, addition, divizor))

                # ---> Multiplying the speeds with the constants <---
                self.leftSpeed = int(self.speed * self.kLeft // 1000)
                self.rightSpeed = int(self.speed * self.kRight // 1000)

                # ---> Object Stall Detection <---
                self.position = (abs(motor.relative_position(self.leftMotorDB)) + abs(motor.relative_position(self.rightMotorDB)))
                if(self.iterator >= stallDetectionIterator): # (self.iterator % stallDetectionIterator == 0)
                    if(self.position - self.previousPosition < stallDetectionThreshold):
                        print("Robot is stuck (Stop Function)"); self.wasStuck = True; break
                    self.previousPosition = self.position; self.iterator = 0

                motor_pair.move_tank(self.pair, self.leftSpeed, self.rightSpeed)
                await runloop.sleep_ms(1); self.iterator += 1

        if(stop or self.wasStuck):
            motor_pair.stop(self.pair, stop = self.brake)
        self.previousFunction = 6

        return None

driveBase = DriveBase(hub.port.B, hub.port.D, hub.port.A, hub.port.C, 2045, 1000, motor.SMART_BRAKE)

# ---> Approach 1 to implementing runs (Optimal) <---
class Programs(object):
    def __init__(self) -> None: pass

    async def Run1(self) -> None:
        driveBase.initRun()
        # ---> Code <---

        return None

    async def Run2(self) -> None:
        driveBase.initRun()
        # ---> Standard Test for turns on the shortest path <---

        await driveBase.gyroForwards(950, 400, 500, kp = 160, ki = 3, kd = 0, constantsScale = 1000)
        await runloop.sleep_ms(500)

        await driveBase.turnRight(875, 200, 500, kLeft = 1, kRight = 0)
        await runloop.sleep_ms(1000)

        await driveBase.gyroForwards(475, 300, 500, kp = 160, ki = 5, kd = 0, constantsScale = 1000)
        await runloop.sleep_ms(500)

        await driveBase.gyroBackwards(250, 200, 100, kp = 160, ki = 5, kd = 0, constantsScale = 1000)
        await runloop.sleep_ms(500)

        await driveBase.turnLeft(700, 200, 650, kLeft = 0, kRight = 1)

        return None

    async def Run3(self) -> None:
        driveBase.initRun()
        # ---> Code <---
        
        await driveBase.gyroForwards(1175, 400, 1000, kp = 175, ki = 2, kd = 0, constantsScale = 100)
        await runloop.sleep_ms(500)
        await driveBase.turnLeft(500, 100, 100, kLeft = -1, kRight = 1, error = 75)
        await runloop.sleep_ms(500)
        await driveBase.gyroForwards(320, 300, 600, kp = 175, ki = 2, kd = 0, constantsScale = 100)
        await motor.run_for_degrees(driveBase.leftMotorSYS, 200, 250)
        motor.run_for_degrees(driveBase.leftMotorSYS, 100, 100)
        await driveBase.gyroForwards(125, 500, 1000, accelerationMethod = SpeedMethods.easeInQuad)
        
        await runloop.sleep_ms(1000)

        await driveBase.gyroBackwards(50, 200, 200, kp = 175, ki = 2, kd = 0, constantsScale = 100)

        await runloop.sleep_ms(1000)

        motor.run_for_degrees(driveBase.leftMotorSYS, -175, 175)
        await driveBase.gyroBackwards(200, 200, 200, kp = 175, ki = 2, kd = 0, constantsScale = 100)
        await driveBase.turnRight(400, 100, 100, kLeft = 1, kRight = -1, error = 75)
        await driveBase.gyroBackwards(1600, 400, 1000, kp = 175, ki = 2, kd = 0, constantsScale = 100)

        return None

    async def Run4(self) -> None:
        driveBase.initRun()
        # ---> Code <---

        return None

    async def Run5(self) -> None:
        driveBase.initRun()
        # ---> Code <---

        return None

    async def Run6(self) -> None:
        driveBase.initRun()
        # ---> Code <---

        return None

    async def Run7(self) -> None:
        driveBase.initRun()
        # ---> Code <---

        return None

    async def Run8(self) -> None:
        driveBase.initRun()
        # ---> Code <---

        return None

    async def Run9(self) -> None:
        driveBase.initRun()
        # ---> Code <---

        return None

    async def Run10(self) -> None:
        driveBase.initRun()
        # ---> Code <---

        return None

# ---> The main program <---
async def main() -> None:
    programs = Programs()
    
    return None

runloop.run(main())
