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
    private double timeThreshold = 10;
    private PIDController autoDistancePID = new PIDController(0.1, 0, 0);
    private double meterSetpoint = 1;
    
  
    public void resetAutoTime()
    {
        autoTime.reset();
        autoTime.start();
    }
    public void periodic()
    {
        if(RobotMap.AutoBooleans.SHOOT_NOW && autoTime.get() < timeThreshold)
        {
            RobotMap.AutoBooleans.SHOOT_NOW = false;
        }
        else
        {
            double xSpeed = autoDistancePID.calculate(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getX(), meterSetpoint);
            Drivetrain.getInstance().drive(xSpeed, 0, 0, true);
        }
    }

    public void postSmartDashboard()
    {
        SmartDashboard.putNumber("AutoDistance", 1);
        SmartDashboard.putNumber("Auto RPM", 3600);
    }

    public void setDistanceAndRPM()
    {
        meterSetpoint = SmartDashboard.getNumber("AutoDistance", 1);
        RobotMap.StateChooser.RPM = SmartDashboard.getNumber("Auto RPM", 3600);

    }
}
