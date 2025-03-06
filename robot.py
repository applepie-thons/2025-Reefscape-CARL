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

        # TODO: The servo motor controlling the ratchet will be initialized here,
        # probably like so.
        #
        #     self.armServo = wpilib.Servo( {the PWM port # the servo's attached to} )
        #
        # I assume we engage and disengage it by calling `self.armServo.set(0)` and 
        # `self.armServo.set(1)` (where `set()` takes a decimal value from 0 to 1
        # representing full-left and full-right rotation). I might be wrong, play with
        # numbers here.
        # https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/servos.html
        # https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib/Servo.html
        # https://www.andymark.com/products/ratchet-sport?Option=Ratchet%20with%20Servo%20Actuator&quantity=1
        #
        # Some ideas for how to integrate it into the code, and to program the arm:
        #     - When the arm motor is not powered, engage the servo (and vice versa,
        #       when the arm motor is powered, disengage the servo)
        #
        #     - If the arm moves X degrees from the last controller input,
        #       automatically power it until it's back to its old position.
        #
        #     - Add a limit to how far back the arm can go (when at the limit, engage
        #       the socket, stop powering the arm motor, and ignore controller input).
        #         - This limit should be the angle we fire the ball into the barge.
        #
        #     - Add a limit to how far forward the arm can go (when at the limit,
        #       move the arm up so that it doesn't keep sagging, and ignore all
        #       controller inputs)
        #         - Make this limit a little past how low we need to go to pick up
        #           from the ground? I don't think there's a reason for us to go
        #           any lower than that.

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
        
        # TODO: Figure out the actual heights we need for this. For the reef, if
        # we want the claw to sit right above the algae when the claw is brought
        # down to 90 degree angle, these values should be (based on the CAD models
        # online, and *measured from the ground*)
        #
        #     ElvPresets.REEF_LOW: 50 inches
        #     ElvPresets.REEF_HIGH: 63 inches
        #
        # The barge height should be our max height. The processor preset height is
        # difficult to determine without a physical model. The bottom edge of the
        # processor is 7 inches from the ground. The top edge is 27 inches from the
        # ground.
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
        # TODO: The times, distances, and motor speeds hard coded here are mostly
        # arbitrary. If we can't change the hardcoded values in time to reliably get
        # our robot to do anything meaningful in autonomous, comment most of these
        # lines and just make it move forward enough to cross the point line.

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

        # TODO: Note that this height is *not* the height from the ground. It is the
        # height relative to the elevator's starting position. This is important for
        # getting the elevator to a preset height.
        #
        # We need to either add the distance (in inches) from the bottom of the
        # starting elevator height to the ground, or subtract that same value from
        # all values in self.elevatorPresets.
        elHeight = self.elevator_encoder.getDistance()

        # Multiplied by 360 to convert from 0 - 1 to 0 - 360
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
