/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.DriveMap;

/**
 * Add your docs here.
 */
public class BasicAuto {
    private Timer autoTime = new Timer();
    private double timeThreshold = 8;
    private double shooterWait = 0;
    private PIDController autoDistancePID = new PIDController(1, 0, 0);
    private double meterSetpoint = 1;
    private int step = 0;
    
  
    public void resetAutoTime()
    {
        autoTime.reset();
        autoTime.start();
    }
    public void periodicOld()
    {
        if(RobotMap.AutoBooleans.SHOOT_NOW && (autoTime.get() > timeThreshold))
        {
            RobotMap.AutoBooleans.SHOOT_NOW = false;
        }
        else if(RobotMap.AutoBooleans.SHOOT_NOW)
        {
            Drivetrain.getInstance().drive(0, 0, TargetAlignDrive.getInstance().align()*RobotMap.DriveMap.MAX_SPEED, true);
        } 
        else
        {
            //System.out.println("Trying to move");
            double xSpeed = autoDistancePID.calculate(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getX(), meterSetpoint);
            Drivetrain.getInstance().drive(xSpeed * RobotMap.DriveMap.MAX_SPEED, 0, 0, true);
        }
    }

    public void newPeriodic()
    {
        switch(step)
        {
            case 0:
            autoTime.reset();
            autoTime.start();
            RobotMap.AutoBooleans.SHOOT_NOW = true;
            step++;
            break;
            
            case 1:
            Drivetrain.getInstance().drive(0, 0, TargetAlignDrive.getInstance().align()*RobotMap.DriveMap.MAX_SPEED, true);
            
            if(RobotMap.AutoBooleans.SHOOT_NOW && (autoTime.get() > timeThreshold))
            {
                RobotMap.AutoBooleans.SHOOT_NOW = false;
                step++;
                

            }
            break;

            case 2:
            double xSpeed = autoDistancePID.calculate(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getX(), meterSetpoint);
            Drivetrain.getInstance().drive(xSpeed * RobotMap.DriveMap.MAX_SPEED, 0, 0, true);
            if(Math.abs(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getX() -  meterSetpoint) < 0.05)
            {
                
            }
            break;

        }
    }
}
