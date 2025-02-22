#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

# In the C:\Users\Admin_4034\Desktop\2025-robotics directory, run
#
#    python -m robotpy deploy
#
# to deploy code to the robot.

# Figure out how to use encoder

from wpilib import SmartDashboard 
import wpilib
import wpilib.drive
import phoenix5
# from wpilib.cameraserver import CameraServer
from cscore import CameraServer, VideoMode

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.leftDrive = phoenix5.WPI_TalonSRX(5)
        self.rightDrive = phoenix5.WPI_TalonSRX(3)
        self.elevator = phoenix5.WPI_TalonSRX(7)
        camera = CameraServer.startAutomaticCapture()
        camera.setPixelFormat( VideoMode.PixelFormat.kMJPEG )
        camera.setResolution( 1280, 720 )
        camera.setFPS( 30 )

        self.robotDrive = wpilib.drive.DifferentialDrive(self.leftDrive, self.rightDrive)
        self.controller = wpilib.XboxController(0)
        
        self.timer = wpilib.Timer()
        self.timeSnapshot = 0

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward. Depending on how your robot's
        # gearbox is constructed, you might have to invert the left side instead.
        self.rightDrive.setInverted(True)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.restart()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        # SmartDashboard.putBoolean("A", self.controller.getAButton())

        if self.timer.get() < 20:
            # Drive forwards half speed, make sure to turn input squaring off
            self.robotDrive.arcadeDrive(1, 0, squareInputs=True)
        else:
            self.robotDrive.stopMotor()  # Stop robot

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        #SmartDashboard.putBoolean("A", self.controller.getAButton()) How to detect a button press

        moveFB = self.controller.getLeftY()
        turnLR = self.controller.getRightX() * -1 # Negative lets you turn with perspective
        lTrig = self.controller.getLeftTriggerAxis()
        yPress = self.controller.getYButton()
        xPress = self.controller.getXButton()
        aPress = self.controller.getAButton()
        bPress = self.controller.getBButtonPressed()
        
        SmartDashboard.putNumber("LY Axis", moveFB)
        SmartDashboard.putNumber("RX Axis", turnLR)
        SmartDashboard.putNumber("L Trigger", lTrig)
        #SmartDashboard.putNumber("R Trigger", rTrig)
        SmartDashboard.putBoolean("Y Button", yPress)
        SmartDashboard.putBoolean("X Button", xPress)

        if aPress == True and xPress == True:
            self.robotDrive.arcadeDrive(moveFB, turnLR, squareInputs=True)                   #fast reverse
            self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, lTrig * -1)
        elif aPress == True:                                                                 #fast normal
            self.robotDrive.arcadeDrive(moveFB, turnLR, squareInputs=True)
            self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, lTrig)
        elif xPress == True:                                                                 #slow reverse
            self.robotDrive.arcadeDrive(moveFB * 0.5, turnLR * 0.5, squareInputs=True)
            self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, lTrig * -0.25)
        else:                                                                                #slow normal
            self.robotDrive.arcadeDrive(moveFB * 0.5, turnLR * 0.5, squareInputs=True)
            self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, lTrig * 0.25)

    """ if xPress == True:
            self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, 0.25)
        else:
            self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, 0)
            """

    def testInit(self):
        """This function is called once each time the robot enters test mode."""

    def testPeriodic(self):
        """This function is called periodically during test mode."""


if __name__ == "__main__":
    wpilib.run(MyRobot)
