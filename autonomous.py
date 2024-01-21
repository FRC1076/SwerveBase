import wpilib
class Autonomous:

    def __init__(self, config, team_is_red, field_start_position, drivetrain):
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

    def executeAuton(self):
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

        return False
    
    def move(self):
        return