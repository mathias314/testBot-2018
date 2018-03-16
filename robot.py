#!/usr/bin/env python3

#USES TALON SR INSTEAD OF SPARKS

'''
here is a motivational story about a llama who is a professional golfer.
hans was a sad llama as a child, and he always tended to dream big. One day, his dreams became reality.
it was a beautiful summer day, the sun was high in the sky, and there were no clouds.
he walked out of his abode that morning, and understood what had to be done.
he reentered his home, picked up his driver, downed a bottle of jack, and made his way triumphantly out of his home.
it was a short walk to Small Pines putt-putt course, and only took him about 15 minutes to gallop to.
he enjoyed the exercise. After all, he didn't often get out of the house.
as he stepped through the front door to purchase his ticket, he was given an odd stare by the man at the counter.
"you cant use a driver on a putt putt course, you hecking idiot," he said.
Hans stared the man down with his steely llama gaze and retorted: "..."
Hans didn't say anything because he's a fucking llama, and llama's can't talk.
'''

import math
import wpilib
import ctre

from wpilib import RobotDrive
from networktables import NetworkTables

# TEST BOT IP: 10.45.93.23


class MyRobot(wpilib.SampleRobot):

    # update every 0.005 seconds/5 milliseconds (200Hz)
    kUpdatePeriod = 0.005

    def robotInit(self):
        '''Robot initialization function
        define and initialize the drivetrain motors'''

        self.NickSpeed = 1 #1 is regular speed, 0 is not moving, choose a number
        self.NickControls = True # If true the robot will obey the left and right side controls. If false the robot will obey the forward/reverse and rotate controls.

        # initialize Talons and Sparks
        self.RRM = ctre.wpi_talonsrx.WPI_TalonSRX(1)
        self.FRM = ctre.wpi_talonsrx.WPI_TalonSRX(2)
        self.RLM = ctre.wpi_talonsrx.WPI_TalonSRX(3)
        self.FLM = ctre.wpi_talonsrx.WPI_TalonSRX(4)

        self.forkliftSpark1 = wpilib.Spark(6)
        self.forkliftSpark2 = wpilib.Spark(7)

        self.intakeSparkRight = wpilib.Talon(5)
        self.intakeSparkLeft = wpilib.Talon(4)

        self.climbingSpark = wpilib.Talon(1)
        self.climbingSpark2 = wpilib.Talon(2)

        self.stick_1 = wpilib.XboxController(0)  # initialize the first joystick on port 0
        self.stick_2 = wpilib.XboxController(1)

        self.switchHigh = wpilib.AnalogInput(0)  # Blue wire on 3-way switch
        self.switchLow = wpilib.AnalogInput(1)  # Yellow and brown wires on 3-way switch

        self.limitSwitch1 = wpilib.DigitalInput(0)

        self.tireRadius = 3  # in inches
        self.robotRadius = 12.375  # in inches, center of tire to center of tire on center tires

        # self.roboTime = wpilib.Timer()

        self.pneumatic = wpilib.DoubleSolenoid(4, 3)  # setting up pneumatics
        self.pneumaticHook = wpilib.DoubleSolenoid(7, 0)

        self.pneumatic.set(2)
        self.pneumaticHook.set(2)

        wpilib.CameraServer.launch()

        print(str(self.RRM.configSelectedFeedbackSensor(ctre._impl.ctre_roborio.FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 1000)))  # encoders
        print(str(self.FLM.configSelectedFeedbackSensor(ctre._impl.ctre_roborio.FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 1000)))

        self.FLM.setSensorPhase(True)
        self.RRM.setSensorPhase(True)

        self.RRM.setInverted(True)
        self.RLM.setInverted(True)

        # yes
        # configure Talon movement parameters (for constant velocity motion)
        self.FLM.configNominalOutputForward(0, 5)
        self.FLM.configNominalOutputReverse(0, 5)
        self.FLM.configPeakOutputForward(1, 5)
        self.FLM.configPeakOutputReverse(-1, 5)

        self.RRM.configNominalOutputForward(0, 5)
        self.RRM.configNominalOutputReverse(0, 5)
        self.RRM.configPeakOutputForward(1, 5)
        self.RRM.configPeakOutputReverse(-1, 5)

        # TODO: configure these values for actual bot (run tests, preferably in a place with no obstacles)
        self.FLM.config_kF(0, 0.48, 5)  # feed-forward throttle (basically initial speed-up)
        self.FLM.config_kP(0, 0.12, 5)  # proportional (how fast it corrects)
        self.FLM.config_kI(0, 0, 5)
        self.FLM.config_kD(0, 0, 5)

        self.RRM.config_kF(0, 0.47, 5)  # feed-forward throttle (basically initial speed-up)
        self.RRM.config_kP(0, 0.12, 5)  # proportional (how fast it corrects)
        self.RRM.config_kI(0, 0, 5)
        self.RRM.config_kD(0, 0, 5)

        # reset encoders
        self.RRM.setPulseWidthPosition(0, 0)
        self.FLM.setPulseWidthPosition(0, 0)

        # configure slave Talons
        self.RLM.follow(self.RRM)
        self.FRM.follow(self.FLM)

        # TODO: Use this
        #self.forkliftEncoder = wpilib.Encoder(0, 1)

        self.counter = 1

        # self.gyro = wpilib.ADXRS450_Gyro()

    def moveForwards(self, rots):
        # reset encoders
        self.RRM.setPulseWidthPosition(0, 0)
        self.FLM.setPulseWidthPosition(0, 0)

        while self.isAutonomous() and (self.RRM.getSelectedSensorPosition(0)[1] < rots * 4000 - 150 or self.FLM.getSelectedSensorPosition(0)[1] < rots * 4000 - 150) and self.isEnabled():
            self.RRM.set(mode=ctre._impl.ctre_roborio.ControlMode.Velocity, value=925)
            self.FLM.set(mode=ctre._impl.ctre_roborio.ControlMode.Velocity, value=925)

        self.RRM.set(ctre._impl.ctre_roborio.ControlMode.Velocity, 0)
        self.FLM.set(ctre._impl.ctre_roborio.ControlMode.Velocity, 0)
        wpilib.Timer.delay(1)

    def turnRightTheta(self, theta):  # theta is in degrees
        self.RRM.setPulseWidthPosition(0, 0)
        self.FLM.setPulseWidthPosition(0, 0)
        while self.isAutonomous() and (self.RRM.getSelectedSensorPosition(0)[1] > -16000 / 360 * theta or self.FLM.getSelectedSensorPosition(0)[1] < 16000 / 360 * theta):  # 16000 is 360 degrees
            self.RRM.set(mode=ctre._impl.ctre_roborio.ControlMode.Velocity, value=(-925))  # -925
            self.FLM.set(mode=ctre._impl.ctre_roborio.ControlMode.Velocity, value=(925))

        self.RRM.set(ctre._impl.ctre_roborio.ControlMode.Velocity, 0)
        self.FLM.set(ctre._impl.ctre_roborio.ControlMode.Velocity, 0)
        wpilib.Timer.delay(1)

    def turnLeftTheta(self, theta):  # theta is in degrees
        self.RRM.setPulseWidthPosition(0, 0)
        self.FLM.setPulseWidthPosition(0, 0)
        while self.isAutonomous() and (self.RRM.getSelectedSensorPosition(0)[1] < 16000 / 360 * theta or self.FLM.getSelectedSensorPosition(0)[1] > -16000 / 360 * theta):  # 16000 is 360 degrees
            self.RRM.set(mode=ctre._impl.ctre_roborio.ControlMode.Velocity, value=(925))
            self.FLM.set(mode=ctre._impl.ctre_roborio.ControlMode.Velocity, value=(-925))
        self.RRM.set(ctre._impl.ctre_roborio.ControlMode.Velocity, 0)
        self.FLM.set(ctre._impl.ctre_roborio.ControlMode.Velocity, 0)
        wpilib.Timer.delay(1)

    def turnRight(self): #this function mom gay
        self.RRM.setPulseWidthPosition(0, 0)
        self.FLM.setPulseWidthPosition(0, 0)
        while (self.RRM.getSelectedSensorPosition(0)[1] < -4000 or self.FLM.getSelectedSensorPosition(0)[1] > -4000) and self.isEnabled():
            self.RRM.set(mode=ctre._impl.ctre_roborio.ControlMode.Velocity, value=925)
            self.FLM.set(mode=ctre._impl.ctre_roborio.ControlMode.Velocity, value=-925)

        self.RRM.set(ctre._impl.ctre_roborio.ControlMode.Velocity, 0)
        self.FLM.set(ctre._impl.ctre_roborio.ControlMode.Velocity, 0)

    def turnLeft(self): # litte known fact is that ur mom gay
        self.RRM.setPulseWidthPosition(0, 0)
        self.FLM.setPulseWidthPosition(0, 0)
        while (self.RRM.getSelectedSensorPosition(0)[1] > -3900 or self.FLM.getSelectedSensorPosition(0)[1] < 3900) and self.isEnabled():
            self.RRM.set(mode=ctre._impl.ctre_roborio.ControlMode.Velocity, value=-1000)
            self.FLM.set(mode=ctre._impl.ctre_roborio.ControlMode.Velocity, value=1000)

        self.RRM.set(ctre._impl.ctre_roborio.ControlMode.Velocity, 0)
        self.FLM.set(ctre._impl.ctre_roborio.ControlMode.Velocity, 0)

    def moveToShootHeight(self, up):
        if up:
            self.forkliftSpark1.set(.75)
            self.forkliftSpark2.set(.75)
        else:
            self.forkliftSpark1.set(-.6)
            self.forkliftSpark2.set(-.6)
        while self.limitSwitch1.get():
            print(self.limitSwitch1.get())
        self.forkliftSpark1.set(.2)
        self.forkliftSpark2.set(.2)

    def autonomous(self):
        gameData = wpilib.DriverStation.getInstance().getGameSpecificMessage()
        while len(gameData) < 3 and (gameData[0] != 'R' or gameData[0] != 'L'):
            gameData = wpilib.DriverStation.getInstance().getGameSpecificMessage()
        highvalue = self.switchHigh.getValue()  # setting switch stuff up
        lowvalue = self.switchLow.getValue()

        startPosition = "Middle"

        #lowvalue = 3500
        # pull in
        # grab cube
        # forklift up

        #if highvalue < 500 and lowvalue < 500:  # Middle
        if startPosition == "Middle":
            if gameData[0] == "L":
                # go switch on left
                self.moveForwards(4.5 * 12 / (self.tireRadius * 2 * math.pi))  # 4.5 is feet, *12 converts it to inches
                self.turnLeftTheta(90)
                self.moveForwards(5 * 12 / (self.tireRadius * 2 * math.pi))
                self.turnRightTheta(90)
                self.moveToShootHeight(1)
                self.moveForwards(3.6 * 12 / (self.tireRadius * 2 * math.pi))
                self.pneumatic.set(1)
                # TODO: score
            else:  # score on right side
                # go switch
                self.moveForwards(4.5 * 12 / (self.tireRadius * 2 * math.pi))
                self.turnRightTheta(90)
                self.moveForwards(5 * 12 / (self.tireRadius * 2 * math.pi))
                self.turnLeftTheta(90)
                self.moveToShootHeight(1)
                self.moveForwards(4.3 * 12 / (self.tireRadius * 2 * math.pi))
                self.pneumatic.set(1)
                # TODO: score
        #elif highvalue > 3000:  # Right
        elif startPosition == "Right":
            if gameData[0] == "L":
                # go scale
                self.moveForwards(20 * 12 / (self.tireRadius * 2 * math.pi))
                self.turnLeftTheta(90)
            else:  # score on right
                # go switch
                self.moveForwards(14 * 12 / (self.tireRadius * 2 * math.pi))
                self.turnLeftTheta(90)
                self.moveToShootHeight(1)
                self.moveForwards(0.7)
                self.pneumatic.set(1)
                # TODO: score
        #elif lowvalue > 3000:  # Left
        elif startPosition == "Left":
            if gameData[0] == "R":
                # go scale
                self.moveForwards(20 * 12 / (self.tireRadius * 2 * math.pi))
                self.turnRightTheta(90)
            else:
                # go switch
                self.moveForwards(14 * 12 / (self.tireRadius * 2 * math.pi))
                self.turnRightTheta(90)
                self.moveToShootHeight(1)
                self.moveForwards(0.7)
                self.pneumatic.set(1)
                #this is where it might all go to shit
                #self.moveForwards(-.6)
                #self.turnLeftTheta(90)
                # TODO: score

    def turn(self, delta):  # This is a turn function that utilizes the gyro. It is not always extremely accurate
        angleStart = self.gyro.getAngle()
        delta = delta * 1.25
        while abs(self.gyro.getAngle() - (angleStart + delta)) > 0.5:
            print(self.gyro.getAngle())
            if (angleStart + delta) > self.gyro.getAngle():
                self.RRM.set(mode=ctre._impl.ctre_roborio.ControlMode.Velocity, value=800)
                self.FLM.set(mode=ctre._impl.ctre_roborio.ControlMode.Velocity, value=-800)
            else:
                self.RRM.set(mode=ctre._impl.ctre_roborio.ControlMode.Velocity, value=-800)
                self.FLM.set(mode=ctre._impl.ctre_roborio.ControlMode.Velocity, value=800)


    def operatorControl(self):
        # Runs the motor from a joystick.
        '''Set the control mode of all four drivetrain motors'''
        self.RRM.setPulseWidthPosition(0, 0)

        while self.isOperatorControl() and self.isEnabled():

            #print(self.limitSwitch1.get())

            if self.NickControls:# getRawAxis here outputs -1 to +1. Questionable
                one = 0.75 * self.stick_1.getRawAxis(1)  # Left stick forward/backward
                five = 0.75 * self.stick_1.getRawAxis(5)   # Right stick left/right
                # self.boostLever = self.stick_1.getRawAxis(2) this doesn't do anything, but it will maybe eventually

                # Calculate the speed each motor should be
                speedLeft = -1 * five
                speedRight = -1 * one

                if abs(speedLeft) < 0.15:
                    speedLeft = 0

                if abs(speedRight) < 0.15:
                    speedRight = 0

                self.FLM.set(speedRight * self.NickSpeed)
                self.RRM.set(speedLeft * self.NickSpeed)
            else: #Alternate Controls, aka Jack control
                one = -0.75*self.stick_1.getRawAxis(1) #if this returns positive then go forwards
                four = 0.75*self.stick_1.getRawAxis(4) #if this returns positive then turn right. Jeremy prefers axis 4;, Jack prefers axis 0.

                speedLeft = (one-four)
                speedRight = (one+four)

                if abs(speedLeft) < 0.2:
                    speedLeft = 0
                if abs(speedRight) < 0.2:
                    speedRight = 0

                self.FLM.set(speedRight * self.NickSpeed)
                self.RRM.set(speedLeft * self.NickSpeed)

            # these control the pneumatic arm
            forward = self.stick_2.getAButton()  # close
            reverse = self.stick_2.getBButton()  # open

            if forward:
                self.pneumatic.set(2)
            elif reverse:
                self.pneumatic.set(1)
            else:
                pass


            intakeIn = self.stick_2.getBumper(1)  # Pull a power cube in
            intakeOut = self.stick_2.getYButton()  # Shoot a power cube out slowly
            intakeOut2 = self.stick_2.getBumper(0) # Shoot a power cube out fast

            if intakeIn:
                #print("intake in")
                self.intakeSparkRight.set(-1)
                self.intakeSparkLeft.set(-1)
            elif intakeOut:
                #print("intake out slow")
                self.intakeSparkRight.set(0.4)
                self.intakeSparkLeft.set(0.4)
            elif intakeOut2:
                #print("intake out fast")
                self.intakeSparkRight.set(0.8)
                self.intakeSparkLeft.set(0.8)
            else:
                self.intakeSparkRight.set(0)
                self.intakeSparkLeft.set(0)

            '''rotateOneWay = self.stick_2.getBumper(1)  # right hand
            rotateAnotherWay = self.stick_2.getBumper(0)  # left hand

            if rotateOneWay:
                print("oneway")
                self.intakeSparkRight.set(1)
                self.intakeSparkLeft.set(-1)
            elif rotateAnotherWay:
                print("anotherway")
                self.intakeSparkRight.set(1)
                self.intakeSparkLeft.set(-1)
            else:
                pass'''

            climberUp = self.stick_2.getTriggerAxis(3)

            # NEVER EVER UNDER ANY CIRCUMSTANCES RUN THE CLIMBER IN THE -1 DIRECTION.
            # IF YOU DO, I WILL BE DEEPLY DISAPPOINTED IN YOU, BECAUSE THE ROBOT
            # WILL BREAK AND YOUR PARENTS WILL BE NOTIFIED THAT YOU BROKE THE ROBOT.
            #
            # -Jon

            if climberUp > 0.06:
                self.climbingSpark.set(climberUp * 1)
                self.climbingSpark2.set(climberUp * 1)
            else:
                self.climbingSpark.set(0)
                self.climbingSpark2.set(0)

            endGame = self.stick_1.getAButton()
            notEndGame = self.stick_1.getBButton()

            if endGame:
                print("endgame")
                self.pneumaticHook.set(1)
            elif notEndGame:
                self.pneumaticHook.set(2)
            else:
                pass


            forkliftUp = (self.stick_1.getTriggerAxis(1))  # right hand
            forkliftDown = (self.stick_1.getTriggerAxis(0))  # left hand

            if forkliftUp > 0.06:
                self.forkliftSpark1.set(1*forkliftUp)
                self.forkliftSpark2.set(1*forkliftUp)
            elif forkliftDown > 0.06:
                self.forkliftSpark1.set(-0.5*forkliftDown)
                self.forkliftSpark2.set(-0.5 *forkliftDown)
            else:
                self.forkliftSpark1.set(0.15)
                self.forkliftSpark2.set(0.15)

            #print(self.limitSwitch1.get())

            up = self.stick_1.getStartButton()

            if up:
                self.moveToShootHeight(1)

            highvalue = self.switchHigh.getValue()
            lowvalue = self.switchLow.getValue()

            if highvalue < 500 and lowvalue < 500:
                robotPosition = "Middle"   # Middle
            elif highvalue > 3000:
                robotPosition = "Right"  # Right
            elif lowvalue > 3000:
                robotPosition = "Left"  # Left


            '''turnRight = self.stick_1.getRawButton(5)
            turnLeft = self.stick_1.getRawButton(4)



            if turnRight:
                self.turnRightTheta(90)
            if turnLeft:
                self.turnLeftTheta(90)'''

            '''
            leftYstick = self.stick_1.getY()
            motorOutput = self.FLM.getMotorOutputPercent()[1]
            stringToPrint = "FLM: out:" + str(motorOutput)
            stringToPrint += " speed:" + str(self.FLM.getSelectedSensorVelocity(0)[1])
            motorOutput1 = self.RRM.getMotorOutputPercent()[1]
            stringToPrint1 = "RRM: out:" + str(motorOutput1)
            stringToPrint1 += " speed:" + str(self.RRM.getSelectedSensorVelocity(0)[1])

            if self.stick_1.getRawButton(1):
                targetVelocity_UnitsPer100ms = leftYstick * 4096 * 500.0 / 600
                self.FLM.set(ctre._impl.ctre_roborio.ControlMode.Velocity, targetVelocity_UnitsPer100ms)
                self.RRM.set(ctre._impl.ctre_roborio.ControlMode.Velocity, targetVelocity_UnitsPer100ms)
                stringToPrint += " err:" + str(self.FLM.getClosedLoopError(0)[1])
                stringToPrint1 += " err:" + str(self.RRM.getClosedLoopError(0)[1])
                stringToPrint += " trg:" + str(targetVelocity_UnitsPer100ms)
                stringToPrint1 += " trg:" + str(targetVelocity_UnitsPer100ms)
                self.loops += 1
                if self.loops >= 10:
                    self.loops = 0
                    # print("FLM: " + str(motorOutput*1023/(self.FLM.getSelectedSensorVelocity(0)[1]+0.0001)))
                    # print("RRM: " + str(motorOutput1*1023/(self.RRM.getSelectedSensorVelocity(0)[1]+0.0001)))
                    print(stringToPrint)
                    print(stringToPrint1)
            else:
                self.FLM.set(0)
                self.RRM.set(0)
            '''

            wpilib.Timer.delay(self.kUpdatePeriod)  # wait 5ms to the next update


if __name__ == "__main__":
    wpilib.run(MyRobot)
