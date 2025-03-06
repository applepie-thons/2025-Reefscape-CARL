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
from cscore import CameraServer, VideoMode

ELV_ENCODER_A_PWM_PORT = 0
ELV_ENCODER_B_PWM_PORT = 1
ARM_ENCODER_DIO_PORT = 2

class ElvControlMode(Enum):
    MANUAL = 0
    AUTO = 1

class ElvPresets(Enum):
    PROCESSOR = 0
    REEF_LOW = 1
    REEF_HIGH = 2
    BARGE = 3

class MyRobot(wpilib.TimedRobot):
    def hardwareInit(self):
        # ------ Game Controller ------ #
        self.controller = wpilib.XboxController(0)

        # ------ Drivetrain Control ------ #
        self.leftDrive = phoenix5.WPI_TalonSRX(5)
        self.rightDrive = phoenix5.WPI_TalonSRX(3)
        self.robotDrive = wpilib.drive.DifferentialDrive(self.leftDrive, self.rightDrive)

        # We need to invert one side of the drivetrain so that positive voltages
        # result in both sides moving forward.
        self.rightDrive.setInverted(True)

        # ------- Elevator Control ------- #
        self.elevator = phoenix5.WPI_TalonSRX(7)

        reverseElvEncoder = False
        self.elevatorMaxHeight = 62.4
        self.elevator_encoder = wpilib.Encoder(
            ELV_ENCODER_A_PWM_PORT,
            ELV_ENCODER_B_PWM_PORT,
            reverseElvEncoder,
            wpilib.Encoder.EncodingType.k2X)
        self.elevator_encoder.reset()

        distanceAtTopFirstStageDefault = 10890.5
        distanceAtTopFirstStageInches = 31.625
        self.elevator_encoder.setDistancePerPulse(
            distanceAtTopFirstStageInches / distanceAtTopFirstStageDefault )

        # --------- Arm Control ---------- #
        self.arm_encoder = wpilib.DutyCycleEncoder(ARM_ENCODER_DIO_PORT)
        self.arm = phoenix5.WPI_TalonSRX(9)

        # ------------ Camera ------------ #
        camera = CameraServer.startAutomaticCapture()
        camera.setPixelFormat(VideoMode.PixelFormat.kMJPEG)
        camera.setResolution(1280, 720)
        camera.setFPS(30)

    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        self.hardwareInit()

        self.timer = wpilib.Timer()

        self.elevatorControlMode = ElvControlMode.MANUAL
        self.elevatorSelectedPreset = ElvPresets.PROCESSOR
        self.elevatorActivePreset = ElvPresets.PROCESSOR
        
        self.elevatorPresets = {
            ElvPresets.PROCESSOR: 0,
            ElvPresets.REEF_LOW: 12,
            ElvPresets.REEF_HIGH: 24,
            ElvPresets.BARGE: 36
        }

    def arcade_drive_train(self, move_speed, turn_speed, speed_toggle, square_val):
        if speed_toggle:
            speedMultipler = 1
        else:
            speedMultipler = 0.5

        self.robotDrive.arcadeDrive(
            move_speed * speedMultipler, turn_speed * speedMultipler, squareInputs=square_val)

    def elevator_drive_train(self, elevator_speed, speed_toggle, reverse_toggle,
                             switch_preset_left, switch_preset_right,
                             activate_preset, elevator_height):

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

            if activate_preset:
                self.elevatorActivePreset = self.elevatorSelectedPreset
                self.elevatorControlMode = ElvControlMode.AUTO

        def man_elevator_drive_train():
            if speed_toggle and reverse_toggle:
                speedMultiplier = -1
            elif speed_toggle:
                speedMultiplier = 1
            elif reverse_toggle:
                speedMultiplier = -0.25
            else:
                speedMultiplier = 0.25

            self.elevator.set(
                phoenix5.TalonSRXControlMode.PercentOutput, elevator_speed * speedMultiplier)

        def auto_elevator_drive_train():
            activePresetHeight = self.elevatorPresets[ self.elevatorActivePreset ]
            toleranceInches = 1/8
            if (elevator_height > activePresetHeight - toleranceInches) and \
                   (elevator_height < activePresetHeight + toleranceInches):
                speed = 0
            elif elevator_height < activePresetHeight:
                speed = 0.2
            elif elevator_height > activePresetHeight:
                speed = -0.2

            self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, speed )

        preset_data_collector()

        if elevator_speed > 0:
            # When using our analog input for elevator speed, go to manual mode.
            self.elevatorControlMode = ElvControlMode.MANUAL
            man_elevator_drive_train()

        elif self.elevatorControlMode == ElvControlMode.AUTO:
            # If we set our active preset, then we will start moving towards it as long
            # as we don't change elevator speed.
            auto_elevator_drive_train()

        else:
            # If were not manually moving the elevator and haven't set an active
            # preset, maintain the existing height.
            self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, 0)

    def arm_drive_train(self, arm_speed, speed_toggle, reverse_toggle):
        if speed_toggle and reverse_toggle:
            speedMultiplier = -1
        elif speed_toggle:
            speedMultiplier = 1
        elif reverse_toggle == True:
            speedMultiplier = -0.25
        else:
            speedMultiplier = 0.25

        self.arm.set(phoenix5.TalonSRXControlMode.PercentOutput,
                     arm_speed * speedMultiplier)

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""
        self.timer.restart()

    def autonomousPeriodic(self):
        """This function is called periodically during autonomous."""
        time = self.timer.get()
        if time < 8 and self.elevator_encoder.getDistance() < self.elevatorMaxHeight:
            self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, 0.125)
        elif time < 11 and self.elevator_encoder.getDistance() > 2:
            self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, -0.125)
        else:
            self.elevator.set(phoenix5.TalonSRXControlMode.PercentOutput, 0)


        if time < 2:
            self.arcade_drive_train(0.1, 0, True, False)
        elif time < 4:
            self.arcade_drive_train(0, 0.1, True, False)
        else:
            self.arcade_drive_train(0, 0, False, False)
        
        if time < 12:
            self.arm_drive_train(0.125, True, True)
        else:
            self.arm_drive_train(0, True, True)

    def teleopInit(self):
        """This function is called once each time the robot enters teleoperated mode."""

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
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

        #Multiplied by 360 to convert from 0 - 1 to 0 - 360
        armAngle = (self.arm_encoder.get() * 360)

        # Toggle to True if you want the robot to not move.
        safetyMode = False
        if safetyMode:
            moveFB = 0
            turnLR = 0
            lTrig = 0

        self.elevator_drive_train(lTrig, aPress, xPress, lBumpPressed, rBumpPressed,
                                  yPressed, elHeight)
        self.arcade_drive_train(moveFB, turnLR, aPress, True)
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
        SmartDashboard.putString("Elevator Control Mode", self.elevatorControlMode.name)
        SmartDashboard.putString("Selected Preset Mode", self.elevatorSelectedPreset.name)
        SmartDashboard.putString("Active Preset Mode", self.elevatorActivePreset.name)

        SmartDashboard.putNumber("Arm Angle", armAngle)

    def testInit(self):
        """This function is called once each time the robot enters test mode."""

    def testPeriodic(self):
        """This function is called periodically during test mode."""


if __name__ == "__main__":
    wpilib.run(MyRobot)
