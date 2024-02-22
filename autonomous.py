import wpilib
from dashboard import Dashboard
from wpimath.trajectory import Trajectory, TrajectoryUtil
from pathplannerlib.path import PathPlannerPath
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d
from pathplannerlib.config import PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
import math
class Autonomous:

    def __init__(self, config, team_is_red, field_start_position, drivetrain, swervometer):
        taskListName = ""
        if team_is_red:
            taskListName += "TASK_RED"
        else:
            taskListName += "TASK_BLUE"
        if field_start_position == 'A':
            taskListName += "_A"
        elif field_start_position == 'B':
            taskListName += "_B"
        else:
            taskListName += "_C"
        self.taskList = config[taskListName]
        self.taskListCounter = 0
        self.autonTimer = wpilib.Timer()
        self.autonHasStarted = False
        self.drivetrain = drivetrain
        self.lastTime = -1
        self.dashboard = Dashboard.getDashboard()
        self.swervometer = swervometer

        self.holonomicController = PPHolonomicDriveController(PIDConstants(0, 0, 0), PIDConstants(0, 0, 0), 3, 0.5388, 0.2)

    def executeAuton(self):
        print(self.dashboard.getBoolean("ROBOT", "Team is Red", False))
        if not self.autonHasStarted:
            self.autonTimer.start()

        if self.taskListCounter >= len(self.taskList):
            self.drivetrain.set_fwd(0)
            self.drivetrain.set_strafe(0)
            self.drivetrain.set_rcw(0)
            self.drivetrain.execute('center')
            print("tasks arae done")
            return False
        
        self.autonTask = self.taskList[self.taskListCounter]

        if self.autonTask[0] == "MOVE":
            x = self.autonTask[1]
            y = self.autonTask[2]
            bearing = self.autonTask[3]
            if self.drivetrain.goToPose(x, y, bearing):
                self.drivetrain.set_fwd(0)
                self.drivetrain.set_strafe(0)
                self.drivetrain.set_rcw(0)
                self.drivetrain.execute('center')
                self.taskListCounter += 1 # Move on to next task.
            return True
        
        elif self.autonTask[0] == 'WHEEL_LOCK':           
            self.drivetrain.setWheelLock(True)
            self.drivetrain.move(0, 0, 0, self.drivetrain.getBearing())
            self.drivetrain.execute('center')
            #self.taskListCounter += 1 # Move on to next task.
        
        elif self.autonTask[0] == 'WAIT':
            if self.lastTime == -1:
                self.lastTime = self.autonTimer.get()
            if(self.autonTimer.get() - self.lastTime > self.autonTask[1]):
                self.lastTime = -1
                self.taskListCounter += 1
        
        elif self.autonTask[0] == 'UPDATE_POSE':
            #self.drivetrain.visionUpdatePose()
            self.taskListCounter += 1
        
        elif self.autonTask[0] == 'PATH':
            if self.lastTime == -1:
                self.lastTime = self.autonTimer.get()
                self.path = PathPlannerPath.fromPathFile(self.autonTask[1])
                self.pathTrajectory = self.path.getTrajectory(ChassisSpeeds(), Rotation2d())
            self.pathState = self.pathTrajectory.sample(self.autonTimer.get() - self.lastTime)
            """
            self.targetPose = self.pathState.getTargetHolonomicPose()
            print((self.targetPose.X() - 8.29) * 39.37, (self.targetPose.Y() - 4.11) * 39.37)
            print(self.swervometer.getCOF())
            self.xVelocity = self.pathState.velocityMps / 3 * math.cos(self.pathState.heading.radians())
            self.yVelocity = self.pathState.velocityMps / 3 * math.sin(self.pathState.heading.radians())
            self.drivetrain.move(self.xVelocity, -self.yVelocity, 0, self.swervometer.getTeamGyroAdjustment())
            self.drivetrain.execute('center')"""

            #calculate chassis speeds with feedback
            #GET COF IS NOT IN METERS AND WITH BLUE RIGHT HAND SIDE AS ORIGIN
            #while the units calculated are in meters, ours are in voltage (hopefully feedback control can account for this)
            self.chassisSpeeds = self.holonomicController.calculateRobotRelativeSpeeds(self.swervometer.getPathPlannerPose(), self.pathState)
            self.drivetrain.set_fwd(self.chassisSpeeds.vx)
            self.drivetrain.set_strafe(self.chassisSpeeds.vy)
            self.drivetrain.execute('center')

        return False
    
    def move(self):
        return