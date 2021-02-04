// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.DriveMap;

/** Add your docs here. */
public class ChallengeAuto {
    private Timer autoTime = new Timer();
    private int trajStep = 0;
    private double timeThreshold = 8;



    public void resetAutoTime() {
        autoTime.reset();
        autoTime.start();
    }

    public void trajectoryPeriodic()
    {
        switch(trajStep)
        {
            case 0:
            RobotMap.setDriveTalonsBrake();
		    RobotMap.resetEncoders();
		    TrajectoryContainer.getInstance().trajectoryFollowing.resetAll();
		    resetAutoTime();
            RobotMap.AutoBooleans.SHOOT_NOW = false;//true;
            RobotMap.AutoBooleans.TRAJECTORY_DONE = false;//false;
            RobotMap.AutoBooleans.INTAKE_NOW = false;
            Chooser.getInstance().autoChooser(Chooser.RobotState.INITIATION_LINE_SHOT);
            trajStep = 2;//++;
            break;
            
            case 1:
            Drivetrain.getInstance().drive(0, 0, TargetAlignDrive.getInstance().align()*RobotMap.DriveMap.MAX_SPEED, true);

            if(autoTime.get() > timeThreshold || BreakBeam.getInstance().getAutoNone())
            {
                RobotMap.AutoBooleans.SHOOT_NOW = false;
                RobotMap.AutoBooleans.INTAKE_NOW = true;
                trajStep++;
            }
            break;

            case 2:
            TrajectoryContainer.getInstance().trajectoryFollowing.auto();
            if(RobotMap.AutoBooleans.TRAJECTORY_DONE)
            {   
                resetAutoTime();
                RobotMap.AutoBooleans.INTAKE_NOW = false;
                Chooser.getInstance().autoChooser(Chooser.RobotState.SHORT_TRENCH);
                RobotMap.AutoBooleans.SHOOT_NOW = true;
                trajStep++;
            }
            break;

            case 3:
            Drivetrain.getInstance().drive(0, 0, TargetAlignDrive.getInstance().align()*RobotMap.DriveMap.MAX_SPEED, true);

            if(autoTime.get() > timeThreshold || BreakBeam.getInstance().getAutoNone())
            {
                RobotMap.AutoBooleans.SHOOT_NOW = false;
                RobotMap.AutoBooleans.INTAKE_NOW = true;
            }
            break; 
        }
    }

    public void slalomPeriodic()
    {
        switch(trajStep)
        {
            case 0:
            RobotMap.setDriveTalonsBrake();
		    RobotMap.resetEncoders();
		    TrajectoryContainer.getInstance().trajectoryFollowing.resetAll();
		    resetAutoTime();
            RobotMap.AutoBooleans.SHOOT_NOW = false;//true;
            RobotMap.AutoBooleans.TRAJECTORY_DONE = false;//false;
            RobotMap.AutoBooleans.INTAKE_NOW = false;
            Chooser.getInstance().autoChooser(Chooser.RobotState.INITIATION_LINE_SHOT);
            trajStep = 1;//++;
            break;

            case 1:

            break;


        }
    }

}
