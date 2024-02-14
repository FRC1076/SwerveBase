import math
import time
import sys
import os
from datetime import datetime

import wpilib
import wpilib.drive
import wpimath.controller
from wpimath.geometry import Pose2d
from wpilib import interfaces
import rev
import ctre
from navx import AHRS
from networktables import NetworkTables

from robotconfig import robotconfig, MODULE_NAMES
from controller import Controller
from swervedrive import SwerveDrive
from swervemodule import SwerveModule
from swervemodule import ModuleConfig

from swervedrive import BalanceConfig
from swervedrive import TargetConfig
from swervedrive import BearingConfig
from swervedrive import VisionDriveConfig

from swervometer import FieldConfig
from swervometer import RobotPropertyConfig
from swervometer import Swervometer

from vision import Vision
from logger import Logger
from dashboard import Dashboard

from autonomous import Autonomous

# Drive Types
ARCADE = 1
TANK = 2
SWERVE = 3

# Test Mode
TEST_MODE = False

DASH_PREFIX = MODULE_NAMES.ROBOT

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        self.drivetrain = None
        self.swervometer = None
        self.driver = None
        self.operator = None
        self.auton = None
        self.vision = None

        # Even if no drivetrain, defaults to drive phase

        self.config = robotconfig

        self.dashboard = Dashboard.getDashboard(testMode=TEST_MODE)

        dir = ''
        if TEST_MODE:
            dir = os.getcwd() # this doesn't work on mac, will write to python dir. Fix later.
        
        self.logger = self.initLogger(dir)
        self.log('Robot init; TEST_MODE =', TEST_MODE)

        for key, config in self.config.items():
            if key == 'CONTROLLERS':
                controllers = self.initControllers(config)
                self.driver = controllers[0]
                self.operator = controllers[1]
            if key == 'AUTON':
                self.auton = self.initAuton(config)
            if key == 'VISION':
                self.vision = self.initVision(config)
            if key == 'SWERVOMETER':
                self.swervometer = self.initSwervometer(config)
            if key == 'DRIVETRAIN':
                self.drivetrain = self.initDrivetrain(config)

        if self.drivetrain:
            self.drivetrain.resetGyro()
            self.drivetrain.printGyro()
        
        self.swervometer.startTimer()
        self.swervometer.initPoseEstimator(self.drivetrain.getModules())


    def disabledExit(self):
        self.log("no longer disabled")
        if self.drivetrain:
            self.drivetrain.reset()


    def initLogger(self, dir):
        return Logger.getLogger(dir)
    
    def initControllers(self, config):
        ctrls = {}
        self.log(config)
        for ctrlConfig in config.values():
            self.log(ctrlConfig)
            controller_id = ctrlConfig['ID']
            ctrl = wpilib.XboxController(controller_id)
            dz = ctrlConfig['DEADZONE']
            lta = ctrlConfig['LEFT_TRIGGER_AXIS']
            rta = ctrlConfig['RIGHT_TRIGGER_AXIS']
            ctrls[controller_id] = Controller(ctrl, dz, lta, rta)
        return ctrls

    def initSwervometer(self, config):
        self.log("initSwervometer ran")
        
        if (config['TEAM_IS_RED']):
            self.team_is_red = True
            self.team_is_blu = False
            teamGyroAdjustment = 0 # Red Team faces 180 degrees at start.
            teamMoveAdjustment = 1 # Red Team start is oriented in the same direction as field.
        else:
            self.team_is_red = False
            self.team_is_blu = True
            teamGyroAdjustment = 180 # Blue Team faces 0 degrees at start.
            teamMoveAdjustment = -1 # Blue Team start is oriented 180 degrees from field.

        self.dashboard.putBoolean(DASH_PREFIX, 'Team is Red', self.team_is_red)

        self.log("FIELD_START_POSITION:", config['FIELD_START_POSITION'])

        if (config['FIELD_START_POSITION'] == 'A'):
            #self.dashboard.putString(DASH_PREFIX, 'Field Start Position', 'A')
            self.dashboard.putStringArray(DASH_PREFIX, 'Field Start Position', ['A', 'B', 'C'])
            self.fieldStartPosition = 'A'
            if self.team_is_red:
                starting_position_x = config['FIELD_RED_A_START_POSITION_X']
                starting_position_y = config['FIELD_RED_A_START_POSITION_Y']
                starting_angle = config['FIELD_RED_A_START_ANGLE']
            else: # self.team_is_blu
                starting_position_x = config['FIELD_BLU_A_START_POSITION_X']
                starting_position_y = config['FIELD_BLU_A_START_POSITION_Y']
                starting_angle = config['FIELD_BLU_A_START_ANGLE']
        elif (config['FIELD_START_POSITION'] == 'B'):
            self.dashboard.putString(DASH_PREFIX, 'Field Start Position', 'B')
            self.fieldStartPosition = 'B'
            if self.team_is_red:
                starting_position_x = config['FIELD_RED_B_START_POSITION_X']
                starting_position_y = config['FIELD_RED_B_START_POSITION_Y']
                starting_angle = config['FIELD_RED_B_START_ANGLE']
            else: # self.team_is_blu
                starting_position_x = config['FIELD_BLU_B_START_POSITION_X']
                starting_position_y = config['FIELD_BLU_B_START_POSITION_Y']
                starting_angle = config['FIELD_BLU_B_START_ANGLE']
        else: # config['FIELD_START_POSITION'] == 'C'
            self.dashboard.putString(DASH_PREFIX, 'Field Start Position', 'C')
            self.fieldStartPosition = 'C'
            if self.team_is_red:
                starting_position_x = config['FIELD_RED_C_START_POSITION_X']
                starting_position_y = config['FIELD_RED_C_START_POSITION_Y']
                starting_angle = config['FIELD_RED_C_START_ANGLE']
            else: # self.team_is_blu
                starting_position_x = config['FIELD_BLU_C_START_POSITION_X']
                starting_position_y = config['FIELD_BLU_C_START_POSITION_Y']
                starting_angle = config['FIELD_BLU_C_START_ANGLE']
        
        bumpers_attached = config['HAS_BUMPERS_ATTACHED']
        if bumpers_attached:
            actual_bumper_dimension_x = config['ROBOT_BUMPER_DIMENSION_X']
            actual_bumper_dimension_y = config['ROBOT_BUMPER_DIMENSION_Y']
        else:
            actual_bumper_dimension_x = 0.0
            actual_bumper_dimension_y = 0.0

        self.dashboard.putBoolean(DASH_PREFIX, 'Has Bumpers Attached', bumpers_attached)

        field_cfg = FieldConfig(sd_prefix='Field_Module',
                                origin_x=config['FIELD_ORIGIN_X'],
                                origin_y=config['FIELD_ORIGIN_Y'],
                                start_position_x= starting_position_x,
                                start_position_y= starting_position_y,
                                start_angle= starting_angle)
        
        robot_cfg = RobotPropertyConfig(sd_prefix='Robot_Property_Module',
                                is_red_team=self.team_is_red,
                                team_gyro_adjustment=teamGyroAdjustment,
                                team_move_adjustment=teamMoveAdjustment,
                                use_com_adjustment=config['USE_COM_ADJUSTMENT'],
                                frame_dimension_x=config['ROBOT_FRAME_DIMENSION_X'],
                                frame_dimension_y=config['ROBOT_FRAME_DIMENSION_Y'],
                                bumper_dimension_x=actual_bumper_dimension_x,
                                bumper_dimension_y=actual_bumper_dimension_y,
                                cof_offset_x=config['ROBOT_COF_OFFSET_X'],
                                cof_offset_y=config['ROBOT_COF_OFFSET_Y'],
                                com_offset_x=config['ROBOT_COM_OFFSET_X'],
                                com_offset_y=config['ROBOT_COM_OFFSET_Y'],
                                gyro_offset_x=config['ROBOT_GYRO_OFFSET_X'],
                                gyro_offset_y=config['ROBOT_GYRO_OFFSET_Y'],
                                camera_offset_x=config['ROBOT_CAMERA_OFFSET_X'],
                                camera_offset_y=config['ROBOT_CAMERA_OFFSET_Y'],
                                swerve_module_offset_x=config['ROBOT_SWERVE_MODULE_OFFSET_X'],
                                swerve_module_offset_y=config['ROBOT_SWERVE_MODULE_OFFSET_Y'])

        swervometer = Swervometer(field_cfg, robot_cfg)

        return swervometer
    
    def initVision(self, config):
        vision = Vision(NetworkTables.getTable('limelight'),
                        config['APRILTAGS'],
                        config['RETROREFLECTIVE'],
                        config['MIN_TARGET_ASPECT_RATIO_REFLECTIVE'],
                        config['MAX_TARGET_ASPECT_RATIO_REFLECTIVE'],
                        config['MIN_TARGET_ASPECT_RATIO_APRILTAG'],
                        config['MAX_TARGET_ASPECT_RATIO_APRILTAG'],
                        config['UPDATE_POSE'])
        vision.setToAprilTagPipeline()
        return vision


    
    def initDrivetrain(self, config):
        self.log("initDrivetrain ran")
        self.drive_type = config['DRIVETYPE']  # side effect!

        balance_cfg = BalanceConfig(sd_prefix='Balance_Module', balance_pitch_kP=config['BALANCE_PITCH_KP'], balance_pitch_kI=config['BALANCE_PITCH_KI'], balance_pitch_kD=config['BALANCE_PITCH_KD'], balance_yaw_kP=config['BALANCE_YAW_KP'], balance_yaw_kI=config['BALANCE_YAW_KI'], balance_yaw_kD=config['BALANCE_YAW_KD'])
        target_cfg = TargetConfig(sd_prefix='Target_Module', target_kP=config['TARGET_KP'], target_kI=config['TARGET_KI'], target_kD=config['TARGET_KD'])
        bearing_cfg = BearingConfig(sd_prefix='Bearing_Module', bearing_kP=config['BEARING_KP'], bearing_kI=config['BEARING_KI'], bearing_kD=config['BEARING_KD'])
        vision_cfg = VisionDriveConfig(sd_prefix='Vision_Module',
            x_visionDrive_kP=config['X_VISION_DRIVE_KP'], x_visionDrive_kI=config['X_VISION_DRIVE_KI'], x_visionDrive_kD=config['X_VISION_DRIVE_KD'],
            y_visionDrive_kP=config['Y_VISION_DRIVE_KP'], y_visionDrive_kI=config['Y_VISION_DRIVE_KI'], y_visionDrive_kD=config['Y_VISION_DRIVE_KD'],
            r_visionDrive_kP=config['R_VISION_DRIVE_KP'], r_visionDrive_kI=config['R_VISION_DRIVE_KI'], r_visionDrive_kD=config['R_VISION_DRIVE_KD'],
            target_offsetX_reflective=config['REFLECTIVE_TARGET_OFFSET_X'], target_target_size_reflective=config['REFLECTIVE_TARGET_TARGET_SIZE'],
            target_offsetX_april=config['APRIL_TARGET_OFFSET_X'], target_target_size_april=config['APRIL_TARGET_TARGET_SIZE'],
            max_target_offset_x=config['MAX_TARGET_OFFSET_X'], min_target_size=config['MIN_TARGET_SIZE'])
    
        flModule_cfg = ModuleConfig(sd_prefix='FrontLeft_Module', zero=125.4 + 90, inverted=True, allow_reverse=True, position_conversion=config['ROBOT_INCHES_PER_ROTATION'], heading_kP=config['HEADING_KP'], heading_kI=config['HEADING_KI'], heading_kD=config['HEADING_KD'])
        frModule_cfg = ModuleConfig(sd_prefix='FrontRight_Module', zero=293.6 + 90, inverted=True, allow_reverse=True, position_conversion=config['ROBOT_INCHES_PER_ROTATION'], heading_kP=config['HEADING_KP'], heading_kI=config['HEADING_KI'], heading_kD=config['HEADING_KD'])
        rlModule_cfg = ModuleConfig(sd_prefix='RearLeft_Module', zero=272.5 + 90, inverted=False, allow_reverse=True, position_conversion=config['ROBOT_INCHES_PER_ROTATION'], heading_kP=config['HEADING_KP'], heading_kI=config['HEADING_KI'], heading_kD=config['HEADING_KD'])
        rrModule_cfg = ModuleConfig(sd_prefix='RearRight_Module', zero=307.5 + 90, inverted=False, allow_reverse=True, position_conversion=config['ROBOT_INCHES_PER_ROTATION'], heading_kP=config['HEADING_KP'], heading_kI=config['HEADING_KI'], heading_kD=config['HEADING_KD'])
        
        motor_type = rev.CANSparkMaxLowLevel.MotorType.kBrushless

        # Drive motors
        flModule_driveMotor = rev.CANSparkMax(config['FRONTLEFT_DRIVEMOTOR'], motor_type)
        flModule_driveMotor_encoder = flModule_driveMotor.getEncoder()
        frModule_driveMotor = rev.CANSparkMax(config['FRONTRIGHT_DRIVEMOTOR'], motor_type)
        frModule_driveMotor_encoder = frModule_driveMotor.getEncoder()
        rlModule_driveMotor = rev.CANSparkMax(config['REARLEFT_DRIVEMOTOR'], motor_type)
        rlModule_driveMotor_encoder = rlModule_driveMotor.getEncoder()
        rrModule_driveMotor = rev.CANSparkMax(config['REARRIGHT_DRIVEMOTOR'], motor_type)
        rrModule_driveMotor_encoder = rrModule_driveMotor.getEncoder()

        # Rotate motors
        flModule_rotateMotor = rev.CANSparkMax(config['FRONTLEFT_ROTATEMOTOR'], motor_type)
        frModule_rotateMotor = rev.CANSparkMax(config['FRONTRIGHT_ROTATEMOTOR'], motor_type)
        rlModule_rotateMotor = rev.CANSparkMax(config['REARLEFT_ROTATEMOTOR'], motor_type)
        rrModule_rotateMotor = rev.CANSparkMax(config['REARRIGHT_ROTATEMOTOR'], motor_type)

        flModule_rotateMotor_encoder = ctre.CANCoder(config['FRONTLEFT_ENCODER'])
        frModule_rotateMotor_encoder = ctre.CANCoder(config['FRONTRIGHT_ENCODER'])
        rlModule_rotateMotor_encoder = ctre.CANCoder(config['REARLEFT_ENCODER'])
        rrModule_rotateMotor_encoder = ctre.CANCoder(config['REARRIGHT_ENCODER'])

        frontLeftModule = SwerveModule(flModule_driveMotor, flModule_driveMotor_encoder, flModule_rotateMotor, flModule_rotateMotor_encoder, flModule_cfg)
        frontRightModule = SwerveModule(frModule_driveMotor, frModule_driveMotor_encoder, frModule_rotateMotor, frModule_rotateMotor_encoder, frModule_cfg)
        rearLeftModule = SwerveModule(rlModule_driveMotor, rlModule_driveMotor_encoder, rlModule_rotateMotor, rlModule_rotateMotor_encoder, rlModule_cfg)
        rearRightModule = SwerveModule(rrModule_driveMotor, rrModule_driveMotor_encoder, rrModule_rotateMotor, rrModule_rotateMotor_encoder, rrModule_cfg)

        # Set Open and Closed Loop Ramp Rate for Teleop
        self.teleopOpenLoopRampRate = config['TELEOP_OPEN_LOOP_RAMP_RATE']
        self.teleopClosedLoopRampRate = config['TELEOP_CLOSED_LOOP_RAMP_RATE']
        self.autonSteerStraight = config['AUTON_STEER_STRAIGHT']
        self.teleopSteerStraight = config['TELEOP_STEER_STRAIGHT']

        #gyro = AHRS.create_spi()
        gyro = AHRS.create_spi(wpilib._wpilib.SPI.Port.kMXP, 500000, 50) # https://www.chiefdelphi.com/t/navx2-disconnecting-reconnecting-intermittently-not-browning-out/425487/36
        
        swerve = SwerveDrive(frontLeftModule, frontRightModule, rearLeftModule, rearRightModule, self.swervometer, self.vision, gyro, balance_cfg, target_cfg, bearing_cfg, vision_cfg, self.autonSteerStraight, self.teleopSteerStraight)

        return swerve

    def initAuton(self, config):
        self.autonOpenLoopRampRate = config['AUTON_OPEN_LOOP_RAMP_RATE']
        self.autonClosedLoopRampRate = config['AUTON_CLOSED_LOOP_RAMP_RATE']
        auton = Autonomous(config, self.team_is_red, self.fieldStartPosition, self.drivetrain)
        return auton
        

    def robotPeriodic(self):
        gyroAngle = self.drivetrain.getGyroAngle()
        modules = self.drivetrain.getModules()
        self.swervometer.updatePoseEstimator(gyroAngle, modules)
        if(self.vision.hasTargets()):
            self.swervometer.poseEstimator.addVisionMeasurement(Pose2d(self.vision.getPose()[0], self.vision.getPose()[1]), self.swervometer.getTimer() - self.vision.getTotalLatency() / 1000)
        return True

    def teleopInit(self):
        self.log("teleopInit ran")
        self.drivetrain.setRampRates(self.teleopOpenLoopRampRate, self.teleopClosedLoopRampRate)
        self.drivetrain.setInAuton(False)
        return True

    def teleopPeriodic(self):
        #print(self.vision.getPose()[0], self.vision.getPose()[1], self.vision.getPose()[2])
        self.drivetrain.visionPeriodic()
        if self.teleopDrivetrain():
            self.log("TeleoDrivetrain returned true. In a maneuver.")
            return
        else:
            self.log("TeleoDrivetrain returned False. Not in a maneuver.")

            return

    def teleopDrivetrain(self):
        if (not self.drivetrain):
            return False
        if (not self.driver):
            return False

        driver = self.driver.xboxController

        # Implement clutch on driving and rotating.
        translational_clutch = 1.0
        rotational_clutch = 1.0
        if (driver.getRightBumper()):
            translational_clutch = 0.4
            rotational_clutch = 0.4
        if (driver.getLeftBumper()): # This is deliberately an "if", not an "elif", to aid in driver transition.
            translational_clutch = 0.2
            rotational_clutch = 0.3 #0.2 was a little too slow for rotation, but perfect for translation

        # Reset the gyro in the direction bot is facing.
        # Note this is a bad idea in competition, since it's reset automatically in robotInit.
        if (driver.getLeftTriggerAxis() > 0.7 and driver.getRightTriggerAxis() > 0.7 and driver.getXButton()):
            self.drivetrain.resetGyro()
            #self.drivetrain.printGyro()

        # Determine if Wheel Lock is needed.
        if (driver.getLeftTriggerAxis() > 0.7 and not driver.getRightTriggerAxis() > 0.7):
            self.drivetrain.setWheelLock(True)
        else:
            self.drivetrain.setWheelLock(False)
        if(driver.getAButton()):
            self.drivetrain.alignWithApril(0, 75, 0)
            return False
        
        # Regular driving, not a maneuver
        if False:
             print("a")
        else:
            strafe = self.deadzoneCorrection(driver.getLeftX() * translational_clutch, self.driver.deadzone)
            fwd = self.deadzoneCorrection(driver.getLeftY() * translational_clutch, self.driver.deadzone)
            rcw = self.deadzoneCorrection(driver.getRightX() * rotational_clutch, self.driver.deadzone)
            
            fwd *= -1 # Because controller is backwards from you think
            
            # Bot starts facing controller
            controller_at_180_to_bot = -1
            fwd *= controller_at_180_to_bot
            strafe *= controller_at_180_to_bot

            # Need to adjust for Team:
            fwd *= self.swervometer.getTeamMoveAdjustment()
            strafe *= self.swervometer.getTeamMoveAdjustment()

            # No need to correct RCW, as clockwise is clockwise whether you are facing with or against bot.
            
            # If any joysticks are dictating movement.
            if fwd != 0 or strafe != 0 or rcw != 0:
                self.drivetrain.move(fwd, strafe, rcw, self.drivetrain.getBearing())
                
                self.log("TeleopDriveTrain: POV: ", driver.getPOV())
                if self.getPOVCorner(driver.getPOV()) == 'front_left':
                    self.drivetrain.execute('front_left')
                elif self.getPOVCorner(driver.getPOV()) == 'front_right':
                    self.drivetrain.execute('front_right')
                elif self.getPOVCorner(driver.getPOV()) == 'rear_left':
                    self.drivetrain.execute('rear_left')
                elif self.getPOVCorner(driver.getPOV()) == 'rear_right':
                    self.drivetrain.execute('rear_right')
                else:
                    self.drivetrain.execute('center')
            # If no joysticks are dictating movement, but we want to lock the wheels.
            elif self.drivetrain.getWheelLock():
                print("wheel locking")
                self.drivetrain.move(0, 0, 0, self.drivetrain.getBearing())
                self.drivetrain.execute('center')
            # Otherwise, make sure we are explicitly doing nothing, so bot does not drift.
            else:
                self.drivetrain.idle()
        return False

    def getPOVCorner(self, value):
        if (value >=0 and value < 90):
            return 'front_right'
        elif (value >=90 and value < 180):
            return 'rear_right'
        elif (value >=180 and value < 270):
            return 'rear_left'
        elif (value >= 270 and value < 360):
            return 'front_left'
        else:
            return 'center'
        
    def autonomousInit(self):
        if not self.auton:
            return
        if not self.drivetrain:
            return
        if not self.swervometer:
            return

        self.autonTimer = wpilib.Timer()
        self.autonTimer.start()

        self.drivetrain.setInAuton(True)

        self.drivetrain.resetGyro()
        if self.team_is_red:
            self.drivetrain.setBearing(180)
        else:
            self.drivetrain.setBearing(0)
        self.drivetrain.setRampRates(self.autonOpenLoopRampRate, self.autonClosedLoopRampRate)
        
    def autonomousPeriodic(self):
         self.auton.executeAuton()
         self.drivetrain.visionPeriodic()
         return False
   
    def deadzoneCorrection(self, val, deadzone):
        """
        Given the deadzone value x, the deadzone both eliminates all
        values between -x and x, and scales the remaining values from
        -1 to 1, to (-1 + x) to (1 - x)
        """
        if abs(val) < deadzone:
            return 0
        elif val < 0:
            x = (abs(val) - deadzone) / (1 - deadzone)
            return -x
        else:
            x = (val - deadzone) / (1 - deadzone)
            return x

    def log(self, *dataToLog):
        self.logger.log(MODULE_NAMES.ROBOT,dataToLog)

if __name__ == "__main__":
    if sys.argv[1] == 'sim':
        TEST_MODE = True
    wpilib.run(MyRobot)
