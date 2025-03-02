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

from enum import Enum

from wpilib import SmartDashboard
import wpilib
import wpilib.drive
import phoenix5
# from wpilib.cameraserver import CameraServer
from cscore import CameraServer, VideoMode

class ElvPresets(Enum):
    PROCESSOR = 0
    REEF_LOW = 1
    REEF_HIGH = 2
    BARGE = 3

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.manualElvActive = True

        self.leftDrive = phoenix5.WPI_TalonSRX(5)
        self.rightDrive = phoenix5.WPI_TalonSRX(3)
        self.elevator = phoenix5.WPI_TalonSRX(7)
        self.arm = phoenix5.WPI_TalonSRX(9)

        self.elevator_encoder = wpilib.Encoder(0, 1, False, wpilib.Encoder.EncodingType.k2X)
        self.elevator_encoder.reset()

        self.arm_encoder = wpilib.DutyCycleEncoder(2)

        self.elevatorSelectedPreset = ElvPresets.PROCESSOR
        self.elevatorActivePreset = ElvPresets.PROCESSOR
        self.elevatorPresets = {
            ElvPresets.PROCESSOR: 0,
            ElvPresets.REEF_LOW: 12,
            ElvPresets.REEF_HIGH: 24,
            ElvPresets.BARGE: 36
        }

        distanceAtTopFirstStageDefault = 10890.5
        distanceAtTopFirstStageInches = 31.625
        self.elevator_encoder.setDistancePerPulse(
            distanceAtTopFirstStageInches / distanceAtTopFirstStageDefault )

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


    def arcade_drive_train(self, move_speed, turn_speed, speed_toggle):
        if speed_toggle == True:                                                                #fast normal
            self.robotDrive.arcadeDrive(move_speed, turn_speed, squareInputs=True)
        else:                                                                                   #slow normal
            self.robotDrive.arcadeDrive(move_speed * 0.5, turn_speed * 0.5, squareInputs=True)

    def elevator_drive_train(self, elevator_speed, speed_toggle, reverse_toggle,
                             switch_preset_left, switch_preset_right, activate_preset, elevator_height):

        def man_elevator_drive_train():
            if speed_toggle == True and reverse_toggle == True:                                      #fast reverse
                self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, elevator_speed * -1)
            elif speed_toggle == True:                                                               #fast normal    
                self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, elevator_speed)
            elif reverse_toggle == True:                                                             #slow reverse 
                self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, elevator_speed * -0.25)
            else:                                                                                    #slow normal         
                self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, elevator_speed * 0.25)

        def preset_data_collector():
            if switch_preset_left:
                # Use `.value` to change from text to number.
                selectedPresetVal = self.elevatorSelectedPreset.value

                # Use modulo to cycle modes.
                selectedPresetVal = (selectedPresetVal - 1) % len(self.elevatorPresets)

                # Cast number back to `ElvPresets` type.
                self.elevatorSelectedPreset = ElvPresets(selectedPresetVal)

            elif switch_preset_right:
                selectedPresetVal = self.elevatorSelectedPreset.value
                selectedPresetVal = (selectedPresetVal + 1) % len(self.elevatorPresets)
                self.elevatorSelectedPreset = ElvPresets(selectedPresetVal)

            if activate_preset == True:
                self.elevatorActivePreset = self.elevatorSelectedPreset
                self.manualElvActive = False

        def auto_elevator_drive_train():
            if ( elevator_height > activePresetHeight - 0.125 ) and ( elevator_height < activePresetHeight + 0.125):
                self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, 0)
            elif elevator_height < activePresetHeight:
                self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, 0.2)
            elif elevator_height > activePresetHeight:
                self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, -0.2)

        preset_data_collector()

        activePresetHeight = self.elevatorPresets[ self.elevatorActivePreset ]

        # If we use our button set for elevator speed, then we ignore preset mode.
        if elevator_speed > 0:
            self.manualElvActive = True
            man_elevator_drive_train()

        # If we set our active preset, then we will start moving towards it as long
        # as we don't change elevator speed.
        elif not self.manualElvActive: 
            auto_elevator_drive_train()

        # If were not moving the elevator, and havent set an active preset, maintain
        # the previous height.
        else:
            self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, 0)

    def arm_drive_train(self, arm_speed, speed_toggle, reverse_toggle):
        if speed_toggle == True and reverse_toggle == True:                                      #fast reverse
            self.arm.set(phoenix5.TalonSRXControlMode.PercentOutput, arm_speed * -1)
        elif speed_toggle == True:                                                               #fast normal    
            self.arm.set(phoenix5.TalonSRXControlMode.PercentOutput, arm_speed)
        elif reverse_toggle == True:                                                             #slow reverse 
            self.arm.set(phoenix5.TalonSRXControlMode.PercentOutput, arm_speed * -0.25)
        else:                                                                                    #slow normal         
            self.arm.set(phoenix5.TalonSRXControlMode.PercentOutput, arm_speed * 0.25)


    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.restart()

    def autonomousPeriodic(self):
        
        """This function is called periodically during autonomous."""
        # SmartDashboard.putBoolean("A", self.controller.getAButton())

        if self.elevator_encoder.getDistance() < 5:
            self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, 0.125)
        else:
            self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, 0)

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        safetyMode = False # Toggle to True if you want the robot to not move.

        moveFB = self.controller.getLeftY()
        turnLR = self.controller.getRightX() * -1 # Negative lets you turn with perspective
        lTrig = self.controller.getLeftTriggerAxis()
        rTrig = self.controller.getRightTriggerAxis()
        yPressed = self.controller.getYButtonPressed()
        xPress = self.controller.getXButton()
        aPress = self.controller.getAButton()
        bPress = self.controller.getBButton()
        lBumpPressed = self.controller.getLeftBumperButtonPressed()
        rBumpPressed = self.controller.getRightBumperButtonPressed()

        elHeight = self.elevator_encoder.getDistance()
        armAngle = self.arm_encoder.get()

        if safetyMode == True:
            moveFB = 0
            turnLR = 0
            lTrig = 0

        # Uses the labeled height to index a height from a dictionary.

        self.elevator_drive_train(lTrig, aPress, xPress, lBumpPressed, rBumpPressed, yPressed, elHeight)
        self.arcade_drive_train(moveFB, turnLR, aPress)
        self.arm_drive_train(rTrig, aPress, xPress)

        if bPress == True:
            self.elevator_encoder.reset()


        SmartDashboard.putNumber("Drive Power (LY)", moveFB)
        SmartDashboard.putNumber("Turn Power (RX)", turnLR)
        SmartDashboard.putNumber("Elevator Power (LTrig)", lTrig)
        SmartDashboard.putNumber("Arm Power (RTrig)", rTrig)

        SmartDashboard.putBoolean("(Y)", yPressed)
        SmartDashboard.putBoolean("Reverse Toggle (X)", xPress)
        SmartDashboard.putBoolean("Speed Toggle (A)", aPress)

        SmartDashboard.putBoolean("Height Reset (B)", bPress)
        SmartDashboard.putNumber("Elevator Height", elHeight)
        SmartDashboard.putString("Selected Preset Mode", self.elevatorSelectedPreset.name)
        SmartDashboard.putString("Active Preset Mode", self.elevatorActivePreset.name)

        SmartDashboard.putNumber("Arm Angle", armAngle)

    def testInit(self):
        """This function is called once each time the robot enters test mode."""

    def testPeriodic(self):
        """This function is called periodically during test mode."""


if __name__ == "__main__":
    wpilib.run(MyRobot)
