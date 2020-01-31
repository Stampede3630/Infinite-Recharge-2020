/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.PID.TurnPid;
import frc.robot.PID.StrafePid;
import frc.robot.PID.ForwardPid;



public class DriveTrain {

    RobotMap robotMap;
    DriveMode driveMode;
    public double turnSetpoint;
    public TurnPid turnPID;
    public StrafePid strafePID;
    public ForwardPid forwardPID;
    public boolean intake;

    public DriveTrain()
    {
        robotMap = RobotMap.getRobotMap();
        turnPID = new TurnPid();
        turnPID.turnPidSetup();

        strafePID = new StrafePid();
        strafePID.strafePidSetup();

        forwardPID = new ForwardPid();
        forwardPID.forwardPidSetup();
    }

    public void drive()
    {
        driveMode.driveRobot();
    }


    

}
